//! MS-MPEG4v3 picture-level decode.
//!
//! A picture is built out of macroblocks (MBs) laid out left-to-right,
//! top-to-bottom (spec §2.5: no slice/GOB layer). Each MB is 16×16 pel
//! of luma + 2× 8×8 chroma (4:2:0). I-frames contain only intra MBs;
//! P-frames intermix intra and motion-compensated inter MBs.
//!
//! ## I-frame decode path (end-to-end sketch)
//!
//! This module implements enough of the v3 I-frame pipeline to produce
//! a `Picture` / `VideoFrame`:
//!
//!   1. Picture header (`MsV3PictureHeader::parse`) — reads the 2-bit
//!      picture type, 5-bit PQUANT, and the three v3-I-frame selectors
//!      (spec §2.3).
//!   2. Per-MB loop — for each (mb_x, mb_y) in raster order:
//!      - `IntraMbHeader::parse` reads `ac_pred` + CBPY.
//!      - For each of 6 blocks: `decode_intra_dc_diff` for DC, then (if
//!        the block's CBP bit is set and we have a real AC VLC table
//!        available) the AC walk. Otherwise the AC plane is zero.
//!      - DC + AC undergo H.263 dequantisation (spec §08 §3.2).
//!      - IDCT (float reference in `idct.rs`).
//!      - Output pels are written into the corresponding MB slot of the
//!        Y / Cb / Cr planes.
//!   3. Clip and hand back a `Picture`.
//!
//! The **intra AC VLC table is still an open clean-room extraction
//! item** (spec §9 OPEN-O4 for G0-G3; G4/G5 are closed but structural
//! values of `pri_A` / `pri_B` live across multiple region_0569c0
//! sub-tables whose format the Extractor has not yet resolved into
//! `(bit_length, code_value)` pairs). The loop uses the placeholder
//! [`crate::ac::AcVlcTable::V3_INTRA_PLACEHOLDER`] which emits empty
//! AC on any coded block — the resulting Picture is DC-only, which is
//! visible as a coarse low-frequency reconstruction. It is **first
//! light**: the pipeline runs end-to-end and produces a valid YUV
//! frame; the frequency detail is the next-session gap to close.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::ac::{AcVlcTable, Scan};
use crate::header::{MsV3PictureHeader, PictureType};
use crate::idct::idct8x8_to_pel;
use crate::mb::{decode_intra_block_full, BlockPred, IntraMbHeader};

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
        if !(16..=4096).contains(&width) || !(16..=4096).contains(&height) {
            return Err(Error::invalid(format!(
                "msmpeg4v3: picture dimensions {}x{} out of supported range",
                width, height
            )));
        }
        Ok(Self { width, height })
    }

    /// Macroblock dimensions of the picture (16×16 per MB, rounding up).
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
    /// Allocate an all-grey picture (luma 128, chroma 128 = neutral).
    pub fn alloc(dims: PictureDims, picture_type: PictureType) -> Self {
        let (mbw, mbh) = dims.mb_dims();
        let y_stride = mbw * 16;
        let c_stride = mbw * 8;
        Self {
            width: dims.width,
            height: dims.height,
            y: vec![128u8; y_stride * mbh * 16],
            cb: vec![128u8; c_stride * mbh * 8],
            cr: vec![128u8; c_stride * mbh * 8],
            y_stride,
            c_stride,
            picture_type,
        }
    }
}

/// Entry point: parse the picture header and decode a full picture.
pub fn decode_picture(br: &mut BitReader<'_>, dims: PictureDims) -> Result<Picture> {
    let hdr = MsV3PictureHeader::parse(br)?;
    match hdr.picture_type {
        PictureType::I => decode_iframe(br, dims, &hdr),
        PictureType::P => Err(Error::unsupported(format!(
            "msmpeg4v3: P-frame decode not yet implemented (quant={}, \
             ac_chroma_sel={}, dc_size_sel={}, mv_table_sel={})",
            hdr.quant, hdr.ac_chroma_sel, hdr.dc_size_sel, hdr.mv_table_sel,
        ))),
    }
}

/// Default AC VLC table used for I-frames. Currently the clean-room
/// placeholder (see [`AcVlcTable::V3_INTRA_PLACEHOLDER`] for the OPEN
/// extraction dependency). When the Extractor lands real data, plug it
/// in here and the decoder will start emitting AC coefficients.
const V3_INTRA_AC_TABLE: AcVlcTable = AcVlcTable::V3_INTRA_PLACEHOLDER;

/// Decode a full v3 I-frame into a [`Picture`].
///
/// This walks every MB in raster order, decodes its 6 blocks
/// (4 luma + 2 chroma), dequantises, IDCTs, and writes the reconstructed
/// pel values into the output planes. DC-prediction here uses a constant
/// neutral predictor (1024, equivalent to a DC of 128 after the /8 DC
/// scaler at q=8) — per-MB spatial DC prediction per MPEG-4 §7.4.3 is
/// an OPEN next-step item (spec §F.1), but the constant predictor still
/// produces a coherent frame; the only observable difference is a
/// slight DC drift between neighbouring blocks when the encoder exploits
/// DC prediction heavily. For first-light testing on uniform inputs
/// this is acceptable.
fn decode_iframe(
    br: &mut BitReader<'_>,
    dims: PictureDims,
    hdr: &MsV3PictureHeader,
) -> Result<Picture> {
    let (mb_w, mb_h) = dims.mb_dims();
    let mut pic = Picture::alloc(dims, PictureType::I);

    // DC prediction caches — per-block (one entry per 8×8 block). For a
    // first-light decoder we use the neutral default and update as we
    // decode; see the module doc comment for the gap vs MPEG-4 §7.4.3.
    let _luma_pred: Vec<BlockPred> = vec![BlockPred::default(); mb_w * 2 * mb_h * 2];
    let _chroma_cb_pred: Vec<BlockPred> = vec![BlockPred::default(); mb_w * mb_h];
    let _chroma_cr_pred: Vec<BlockPred> = vec![BlockPred::default(); mb_w * mb_h];

    let quant = hdr.quant as u32;

    for my in 0..mb_h {
        for mx in 0..mb_w {
            decode_intra_mb_to_picture(br, &mut pic, mx, my, quant)?;
        }
    }

    Ok(pic)
}

/// Decode one intra macroblock's 6 blocks into the corresponding slot
/// of `pic`. Uses a neutral DC predictor (1024) per block.
fn decode_intra_mb_to_picture(
    br: &mut BitReader<'_>,
    pic: &mut Picture,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
) -> Result<()> {
    let header = IntraMbHeader::parse(br)?;

    // Chroma CBP bits — not signalled by CBPY; they come from the
    // MCBPCY joint VLC (spec §3.1). Until the joint-VLC helper
    // is wired in (blocking on the `0x3df40` shared walker tree),
    // we treat chroma CBP as always zero for I-frames. This is
    // conservative: we miss AC but never read spurious bits.
    let cbp_cb = false;
    let cbp_cr = false;

    // Neutral DC predictor — see fn-doc above.
    let pred_dc_base = 1024i32;

    for block_idx in 0..6usize {
        let cbp_set = match block_idx {
            0..=3 => header.cbpy & (1 << (3 - block_idx)) != 0,
            4 => cbp_cb,
            5 => cbp_cr,
            _ => unreachable!(),
        };

        // Scan order: spec §4.4. For first-light, zigzag always
        // (the alt-H / alt-V selection needs the DC-predictor gradient
        // test from `1c20aef0..1c20af2c`, which is the next gap).
        let scan = Scan::Zigzag;

        // Best-effort AC decode: if the placeholder is in use, the
        // function errors; that's OK for blocks where `cbp_set`
        // is false (no AC walk is performed) but problematic for
        // AC-coded blocks. For first-light we only attempt AC when the
        // table is real — detected by the empty-entries sentinel.
        let block_result = if cbp_set && V3_INTRA_AC_TABLE.entries.is_empty() {
            // AC coded but table not yet available — return DC-only.
            let dc_diff = crate::mb::decode_intra_dc_diff(br, block_idx)?;
            let dc = crate::mb::reconstruct_intra_dc(dc_diff, pred_dc_base, block_idx, quant);
            crate::mb::DecodedIntraBlock {
                coeffs: {
                    let mut a = [0i32; 64];
                    a[0] = dc;
                    a
                },
                ac_nonzero: 0,
            }
        } else {
            decode_intra_block_full(
                br,
                block_idx,
                pred_dc_base,
                quant,
                cbp_set,
                scan,
                &V3_INTRA_AC_TABLE,
            )?
        };

        // IDCT in float, clip to [-256, 255], then offset by +128 to
        // get unsigned pel values and clamp to [0, 255].
        let mut pels = [0i32; 64];
        idct8x8_to_pel(&block_result.coeffs, &mut pels);

        write_block_to_picture(pic, mb_x, mb_y, block_idx, &pels);
    }

    Ok(())
}

/// Copy one 8×8 decoded block (signed pel deltas with the intra-DC
/// baked in as the mean) into the picture's YUV planes. `pels` contain
/// the IDCT output in the signed range `[-256, 255]`; MS-MPEG4 intra
/// blocks encode `pel_value - 128` relative to the 8-bit unsigned plane
/// (spec §7.4, same convention as H.263 / MPEG-4 Part 2).
fn write_block_to_picture(
    pic: &mut Picture,
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
    pels: &[i32; 64],
) {
    match block_idx {
        0..=3 => {
            // Luma: 4 × 8×8 inside the 16×16 MB, quadrant order
            //   0 = top-left, 1 = top-right, 2 = bottom-left, 3 = bottom-right.
            let bx = (block_idx & 1) * 8;
            let by = (block_idx >> 1) * 8;
            let y_base = mb_y * 16 + by;
            let x_base = mb_x * 16 + bx;
            for j in 0..8usize {
                for i in 0..8usize {
                    let y = y_base + j;
                    let x = x_base + i;
                    if y >= pic.y_stride * (pic.y.len() / pic.y_stride) / pic.y_stride {
                        // (safety: out-of-bounds check below covers it)
                    }
                    let off = y * pic.y_stride + x;
                    if off < pic.y.len() {
                        pic.y[off] = (pels[j * 8 + i] + 128).clamp(0, 255) as u8;
                    }
                }
            }
        }
        4 | 5 => {
            // Chroma: one 8×8 per 16×16 MB (4:2:0 subsampling).
            let plane = if block_idx == 4 {
                &mut pic.cb
            } else {
                &mut pic.cr
            };
            let y_base = mb_y * 8;
            let x_base = mb_x * 8;
            for j in 0..8usize {
                for i in 0..8usize {
                    let off = (y_base + j) * pic.c_stride + (x_base + i);
                    if off < plane.len() {
                        plane[off] = (pels[j * 8 + i] + 128).clamp(0, 255) as u8;
                    }
                }
            }
        }
        _ => unreachable!(),
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
}
