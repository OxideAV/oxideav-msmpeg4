//! MS-MPEG4v3 picture-level decode.
//!
//! A picture is built out of macroblocks (MBs) laid out left-to-right,
//! top-to-bottom. Each MB is 16×16 pel of luma + 2× 8×8 chroma (4:2:0).
//! I-frames contain only intra MBs; P-frames intermix intra and
//! motion-compensated inter MBs.
//!
//! The current implementation parses the picture header and enforces
//! that frame dimensions are known (supplied by the container via
//! `CodecParameters`), then stops with a descriptive
//! [`Error::Unsupported`] when the first MB would be decoded — the VLC
//! tables needed to continue are staged in follow-up commits.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::header::{MsV3PictureHeader, PictureType};

/// Dimensions of a picture, derived from the container's
/// [`oxideav_core::CodecParameters`]. MS-MPEG4v3 does not carry
/// width/height in the bitstream itself (unlike MPEG-4 Part 2's VOL).
#[derive(Clone, Copy, Debug)]
pub struct PictureDims {
    pub width: u32,
    pub height: u32,
}

impl PictureDims {
    pub fn new(width: u32, height: u32) -> Result<Self> {
        if width == 0 || height == 0 {
            return Err(Error::invalid("msmpeg4v3: zero picture dimension"));
        }
        // H.263 / MS-MPEG4 require pel-count multiples of 16 for macroblock
        // alignment — or at least that's the expectation when we allocate
        // MB grids. The picture itself may be non-multiple-of-16 in theory
        // but practical files are always rounded up.
        if !(16..=4096).contains(&width) || !(16..=4096).contains(&height) {
            return Err(Error::invalid(format!(
                "msmpeg4v3: picture dimensions {}x{} out of supported range",
                width, height
            )));
        }
        Ok(Self { width, height })
    }

    /// Macroblock dimensions of the picture (16x16 per MB, rounding up).
    pub fn mb_dims(&self) -> (usize, usize) {
        (
            self.width.div_ceil(16) as usize,
            self.height.div_ceil(16) as usize,
        )
    }
}

/// A decoded picture in planar YUV420 (pel domain).
#[derive(Clone, Debug)]
pub struct Picture {
    pub width: u32,
    pub height: u32,
    pub y: Vec<u8>,
    pub cb: Vec<u8>,
    pub cr: Vec<u8>,
    pub y_stride: usize,
    pub c_stride: usize,
    pub picture_type: PictureType,
}

impl Picture {
    /// Allocate an all-zero picture of the given dimensions. The
    /// luminance plane is padded to a multiple of 16 (MB-aligned) for
    /// internal addressing; chroma is padded to a multiple of 8.
    pub fn alloc(dims: PictureDims, picture_type: PictureType) -> Self {
        let (mbw, mbh) = dims.mb_dims();
        let y_stride = mbw * 16;
        let c_stride = mbw * 8;
        Self {
            width: dims.width,
            height: dims.height,
            y: vec![0u8; y_stride * mbh * 16],
            cb: vec![0u8; c_stride * mbh * 8],
            cr: vec![0u8; c_stride * mbh * 8],
            y_stride,
            c_stride,
            picture_type,
        }
    }
}

/// Entry point: parse the picture header from `br` and, if the header
/// is valid, attempt to decode the frame.
///
/// Currently returns the parsed header inside a descriptive
/// [`Error::Unsupported`] on any I- or P-frame body; the goal of this
/// increment is to land the architectural surface so follow-up commits
/// can plug in VLC tables and MB decode without churning the API.
pub fn decode_picture(br: &mut BitReader<'_>, dims: PictureDims) -> Result<Picture> {
    let hdr = MsV3PictureHeader::parse(br)?;
    match hdr.picture_type {
        PictureType::I => {
            // I-frame MB decode pending VLC-table commits. The header is
            // already consumed; we keep `dims` validated so callers can
            // still detect obvious mismatches early.
            let _ = dims;
            Err(Error::unsupported(format!(
                "msmpeg4v3: I-frame MB decode not yet implemented (quant={}, \
                 slice_height={}, flag={})",
                hdr.quant, hdr.slice_height, hdr.flag as u8,
            )))
        }
        PictureType::P => Err(Error::unsupported(format!(
            "msmpeg4v3: P-frame MB decode not yet implemented (quant={}, \
             mv_table={}, rl_table={})",
            hdr.quant, hdr.mv_table_select, hdr.rl_table_select,
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dims_validate_range() {
        assert!(PictureDims::new(0, 480).is_err());
        assert!(PictureDims::new(640, 0).is_err());
        assert!(PictureDims::new(15, 480).is_err());
        assert!(PictureDims::new(640, 4097).is_err());
        let d = PictureDims::new(640, 480).unwrap();
        assert_eq!(d.mb_dims(), (40, 30));
        // Non-16 dims round up.
        let d = PictureDims::new(320, 241).unwrap();
        assert_eq!(d.mb_dims(), (20, 16));
    }

    #[test]
    fn picture_allocation_sizes() {
        let d = PictureDims::new(352, 288).unwrap();
        let p = Picture::alloc(d, PictureType::I);
        assert_eq!(p.y.len(), 352 * 288);
        assert_eq!(p.cb.len(), 176 * 144);
        assert_eq!(p.cr.len(), 176 * 144);
        assert_eq!(p.y_stride, 352);
        assert_eq!(p.c_stride, 176);
    }

    fn pack(fields: &[(u32, u32)]) -> Vec<u8> {
        // Duplicated (small) helper — keeps test modules independent.
        let mut out: Vec<u8> = Vec::new();
        let mut acc: u64 = 0;
        let mut bits: u32 = 0;
        for (v, w) in fields {
            let mask = if *w == 32 { u32::MAX } else { (1u32 << w) - 1 };
            acc = (acc << w) | ((*v & mask) as u64);
            bits += w;
            while bits >= 8 {
                let shift = bits - 8;
                out.push(((acc >> shift) & 0xff) as u8);
                acc &= (1u64 << shift) - 1;
                bits -= 8;
            }
        }
        if bits > 0 {
            let shift = 8 - bits;
            out.push(((acc << shift) & 0xff) as u8);
        }
        if out.is_empty() {
            out.push(0);
        }
        out
    }

    #[test]
    fn unsupported_carries_header_fields() {
        let bytes = pack(&[(0, 2), (7, 5), (0, 1), (0, 5), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        let d = PictureDims::new(352, 288).unwrap();
        let err = decode_picture(&mut br, d).unwrap_err();
        let msg = format!("{err}");
        assert!(msg.contains("I-frame"));
        assert!(msg.contains("quant=7"));
    }

    #[test]
    fn propagates_header_parse_errors() {
        // Zero-quant is invalid — propagates the header error as-is.
        let bytes = pack(&[(0, 2), (0, 5), (0, 1), (0, 5), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        let d = PictureDims::new(352, 288).unwrap();
        assert!(decode_picture(&mut br, d).is_err());
    }
}
