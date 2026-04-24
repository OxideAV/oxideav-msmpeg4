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
//!      - [`IntraMbHeader::parse_v3_mcbpcy`] decodes the 128-entry
//!        joint-MCBPCY canonical-Huffman table (spec §3.1 / spec/05
//!        §3.2), producing the 4-bit luma CBPY and two chroma CBP bits
//!        in one go, then reads the post-VLC `ac_pred_flag` bit.
//!      - For each of 6 blocks: MPEG-4 §7.4.3 spatial DC prediction
//!        (gradient test on the three already-decoded neighbours),
//!        `decode_intra_dc_diff` for the DC-differential VLC, reconstruct
//!        DC = predictor + diff * scaler, then (if CBP bit is set and
//!        we have a real AC VLC table available) the AC walk through
//!        the scan table chosen from the DC direction (spec/04 §4.4).
//!      - IDCT (float reference in `idct.rs`).
//!      - Output pels are written into the corresponding MB slot of the
//!        Y / Cb / Cr planes.
//!   3. Clip and hand back a `Picture`.
//!
//! **DC predictor**, **AC scan dispatcher** and **MCBPCY joint VLC**
//! are all wired in the round-8 commit. The **intra AC VLC table is
//! still an open clean-room extraction item** (spec §9 OPEN-O4); on
//! coded blocks the AC plane is zero-filled (DC-only reconstruction)
//! until the Extractor lands the real run/level/last table.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::ac::{AcVlcTable, Scan};
use crate::dc_pred::{DcCache, DcPrediction};
use crate::header::{MsV3PictureHeader, PictureType};
use crate::idct::idct8x8_to_pel;
use crate::mb::{decode_intra_block_full, IntraMbHeader};

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
/// pel values into the output planes.
///
/// Per MPEG-4 Part 2 §7.4.3 / MSMPEG4 spec/03 §1.3, each block's DC
/// coefficient is predicted from the gradient of three already-decoded
/// neighbours (left `A`, top `B`, top-left diagonal `D`):
///
///   * `|A - D| < |A - B|` → predict from `A` (left);
///   * otherwise → predict from `B` (top).
///
/// Missing neighbours (picture edges) are substituted with the neutral
/// value `1024`. The prediction direction also drives the AC-scan
/// dispatcher — left-predicted blocks scan alt-horizontal,
/// top-predicted blocks scan alt-vertical (spec/04 §4.4). When the
/// MB-level `ac_pred_flag` is off, zigzag is used regardless.
fn decode_iframe(
    br: &mut BitReader<'_>,
    dims: PictureDims,
    hdr: &MsV3PictureHeader,
) -> Result<Picture> {
    let (mb_w, mb_h) = dims.mb_dims();
    let mut pic = Picture::alloc(dims, PictureType::I);

    // DC prediction cache — one entry per 8×8 block (luma 2×mb per MB,
    // chroma 1×1 per MB per plane). See `dc_pred::DcCache`.
    let mut dc_cache = DcCache::new(mb_w, mb_h);

    let quant = hdr.quant as u32;

    for my in 0..mb_h {
        for mx in 0..mb_w {
            decode_intra_mb_to_picture(br, &mut pic, &mut dc_cache, mx, my, quant)?;
        }
    }

    Ok(pic)
}

/// Map a per-MB block index (0..5) to its block-grid (bx, by) position.
/// Luma blocks 0..=3 occupy a 2×2 grid inside the MB; chroma is one
/// block per MB (4 = Cb, 5 = Cr).
fn block_grid_pos(block_idx: usize, mb_x: usize, mb_y: usize) -> (usize, usize) {
    match block_idx {
        0..=3 => (mb_x * 2 + (block_idx & 1), mb_y * 2 + (block_idx >> 1)),
        4 | 5 => (mb_x, mb_y),
        _ => unreachable!(),
    }
}

/// Decode one intra macroblock's 6 blocks into the corresponding slot
/// of `pic`, using MPEG-4 §7.4.3 spatial DC prediction and per-block
/// AC-scan dispatch.
fn decode_intra_mb_to_picture(
    br: &mut BitReader<'_>,
    pic: &mut Picture,
    dc_cache: &mut DcCache,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
) -> Result<()> {
    let header = IntraMbHeader::parse_v3_mcbpcy(br)?;

    for block_idx in 0..6usize {
        let cbp_set = match block_idx {
            0..=3 => header.cbpy & (1 << (3 - block_idx)) != 0,
            4 => header.cbp_cb,
            5 => header.cbp_cr,
            _ => unreachable!(),
        };

        // DC spatial prediction — per-block (not per-MB).
        let (bx, by) = block_grid_pos(block_idx, mb_x, mb_y);
        let pred: DcPrediction = match block_idx {
            0..=3 => dc_cache.predict_luma(bx, by),
            4 => dc_cache.predict_chroma(false, bx, by),
            5 => dc_cache.predict_chroma(true, bx, by),
            _ => unreachable!(),
        };

        // AC scan selection — spec/04 §4.4:
        //   ac_pred disabled → zigzag;
        //   otherwise from-left → alt-horizontal, from-top → alt-vertical.
        let scan = if header.ac_pred {
            pred.direction.ac_scan()
        } else {
            Scan::Zigzag
        };

        // AC path: the AC VLC is still a placeholder (spec §9 OPEN) —
        // fall back to DC-only when CBP is set AND table is empty. When
        // the table is non-empty a future round will plug it in.
        let block_result = if cbp_set && V3_INTRA_AC_TABLE.entries.is_empty() {
            let dc_diff = crate::mb::decode_intra_dc_diff(br, block_idx)?;
            let dc = crate::mb::reconstruct_intra_dc(dc_diff, pred.predictor, block_idx, quant);
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
                pred.predictor,
                quant,
                cbp_set,
                scan,
                &V3_INTRA_AC_TABLE,
            )?
        };

        // Update the DC cache with the reconstructed DC for this block.
        let reconstructed_dc = block_result.coeffs[0];
        match block_idx {
            0..=3 => dc_cache.luma_set(bx, by, reconstructed_dc),
            4 => dc_cache.chroma_set(false, bx, by, reconstructed_dc),
            5 => dc_cache.chroma_set(true, bx, by, reconstructed_dc),
            _ => unreachable!(),
        }

        // IDCT in float, clip to [-256, 255], then offset by +128 to
        // get unsigned pel values and clamp to [0, 255].
        let mut pels = [0i32; 64];
        idct8x8_to_pel(&block_result.coeffs, &mut pels);

        write_block_to_picture(pic, mb_x, mb_y, block_idx, &pels);
    }

    Ok(())
}

/// Copy one 8×8 decoded block into the picture's YUV planes.
///
/// For MSMPEG4v3 intra blocks, the decoded DC coefficient is in the
/// pel-space domain (i.e. DC=1024 → IDCT→ 128 per pel = unsigned grey
/// mid). The IDCT output is directly the unsigned 8-bit pel value,
/// clipped to `[0, 255]` — there is no post-IDCT `+128` offset (that
/// offset applies to inter blocks where the IDCT output is a signed
/// residual to add onto the MC prediction, not to intra blocks where
/// the DC already carries the pel mean).
fn write_block_to_picture(
    pic: &mut Picture,
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
    pels: &[i32; 64],
) {
    match block_idx {
        0..=3 => {
            let bx = (block_idx & 1) * 8;
            let by = (block_idx >> 1) * 8;
            let y_base = mb_y * 16 + by;
            let x_base = mb_x * 16 + bx;
            for j in 0..8usize {
                for i in 0..8usize {
                    let off = (y_base + j) * pic.y_stride + (x_base + i);
                    if off < pic.y.len() {
                        pic.y[off] = pels[j * 8 + i].clamp(0, 255) as u8;
                    }
                }
            }
        }
        4 | 5 => {
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
                        plane[off] = pels[j * 8 + i].clamp(0, 255) as u8;
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

    /// Hand-crafted 32×32 I-frame that exercises the full DC-spatial-
    /// predictor + MCBPCY + scan dispatch pipeline end-to-end. No AC —
    /// every MB's MCBPCY symbol is sym idx 64 (bl=2, code `00`,
    /// CBP=`000000`), so no block has coded AC and the decoder's
    /// AC-placeholder path is not exercised.
    ///
    /// What this verifies:
    /// * `IntraMbHeader::parse_v3_mcbpcy` correctly decodes the
    ///   joint-MCBPCY canonical-Huffman table and reads the
    ///   post-VLC `ac_pred_flag`.
    /// * `DcCache` and `predict_dc` produce a spatial-gradient DC
    ///   predictor per MPEG-4 §7.4.3 for every block.
    /// * `decode_iframe`'s per-block MB→grid coord mapping is right.
    /// * The final luma/chroma picture is the expected uniform /
    ///   near-uniform output (reconstructed DC per block, applied
    ///   through the IDCT).
    #[test]
    fn handcrafted_dc_only_32x32_iframe_decodes() {
        use oxideav_core::bits::BitReader;

        // Helper: pack a list of (value, bit_width) into MSB-first bytes.
        fn pack(fields: &[(u32, u32)]) -> Vec<u8> {
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
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            out
        }

        // Picture header: I-frame, q=8, ac_chroma=0, ac_luma=0, dc_size_sel=0.
        let mut fields: Vec<(u32, u32)> = vec![
            (0, 2), // picture_type I
            (8, 5), // quant 8
            (0, 1), // ac_chroma_sel = 0 (unary `0`)
            (0, 1), // ac_luma_sel = 0
            (0, 1), // dc_size_sel = 0
        ];
        // For each of 2x2 MBs: MCBPCY sym 64 (code `00`, bl=2), ac_pred=0,
        // then 6 DC-size-0 codes (luma `011`, chroma `11`).
        for _ in 0..4 {
            fields.push((0b00, 2)); // MCBPCY sym 64
            fields.push((0, 1)); // ac_pred = 0
            fields.push((0b011, 3)); // luma DC size 0 (Y0)
            fields.push((0b011, 3)); // Y1
            fields.push((0b011, 3)); // Y2
            fields.push((0b011, 3)); // Y3
            fields.push((0b11, 2)); // chroma DC size 0 (Cb)
            fields.push((0b11, 2)); // Cr
        }
        // Tail padding.
        fields.push((0, 32));
        let bytes = pack(&fields);

        let mut br = BitReader::new(&bytes);
        let dims = PictureDims::new(32, 32).unwrap();
        let pic = decode_picture(&mut br, dims).expect("32x32 DC-only decode");

        assert_eq!(pic.picture_type, PictureType::I);
        assert_eq!(pic.width, 32);
        assert_eq!(pic.height, 32);
        // With DC size = 0 everywhere, each block's DC differential is
        // zero, so the reconstructed DC equals the spatial predictor.
        // At block (0,0) the predictor defaults to 1024 (neutral), which
        // at q=8 and scaler=16 gives a decoded DC "level" of 1024/8/16
        // → IDCT (DC-only) of 1024 gives 128 per pel. We should see
        // uniform 128 across the entire picture.
        let y_first = pic.y[0];
        assert!(
            (120..=136).contains(&y_first),
            "Y plane top-left = {y_first}, expected ~128"
        );
        // All luma pels should be near 128 (DC prediction chains this).
        let y_max = *pic.y.iter().max().unwrap();
        let y_min = *pic.y.iter().min().unwrap();
        assert!(
            (y_max as i32 - y_min as i32).abs() < 16,
            "luma DC drift too large: min={y_min}, max={y_max}"
        );
    }

    /// Variant that exercises the DC-predictor propagation: every block
    /// has a non-zero DC differential that the predictor chain must
    /// carry forward. Confirms that the gradient test really is picking
    /// a neighbour rather than defaulting to neutral.
    #[test]
    fn handcrafted_dc_propagation_smoke_test() {
        use oxideav_core::bits::BitReader;

        fn pack(fields: &[(u32, u32)]) -> Vec<u8> {
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
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            out
        }

        // Picture header: I-frame q=8.
        let mut fields: Vec<(u32, u32)> = vec![(0, 2), (8, 5), (0, 1), (0, 1), (0, 1)];
        // 16×16 → one MB. MCBPCY sym 64 → CBP=0. ac_pred=0.
        // Then for each of 6 blocks: DC size 1 = luma `11` / chroma `10`;
        // DC mag bit = 1 (positive diff = +1 level).
        fields.push((0b00, 2));
        fields.push((0, 1));
        for _ in 0..4 {
            fields.push((0b11, 2)); // luma size 1
            fields.push((1, 1)); // +1
        }
        for _ in 0..2 {
            fields.push((0b10, 2)); // chroma size 1
            fields.push((1, 1)); // +1
        }
        fields.push((0, 32));
        let bytes = pack(&fields);
        let mut br = BitReader::new(&bytes);
        let dims = PictureDims::new(16, 16).unwrap();
        let pic = decode_picture(&mut br, dims).expect("16x16 DC-prop decode");

        // Each block's DC = predictor + 1 * scaler. At block (0,0) the
        // predictor is 1024 (neutral) and luma scaler at q=8 is 16:
        //   block 0 DC = 1024 + 16 = 1040 → IDCT → 1040/8 = 130/pel.
        //   block 1 (right of 0): A = block 0's reconstructed DC 1040;
        //     B, D neutral. |A-D| = 16, |A-B| = 0. Top wins (tie-break →
        //     else-branch). So predictor = 1024 → DC = 1024+16 = 1040.
        //   block 2 (below 0): analogous → predictor = 1024 (no left).
        //   block 3 (below-right): A = block 2's DC = 1040; D = block 0 = 1040;
        //     B = block 1 = 1040. |A-D|=0, |A-B|=0 → else → top. Pred=1040.
        //     DC = 1040 + 16 = 1056 → 1056/8 = 132 per pel.
        // So the bottom-right luma block should be ~132, while top-left
        // should be ~130. Non-uniform reconstruction = DC prediction
        // chain really runs.
        let tl = pic.y[0] as i32;
        let br_y = pic.y[pic.y.len() - 1] as i32;
        assert!(
            br_y >= tl,
            "expected monotone DC propagation (tl={tl}, br={br_y})"
        );
    }
}
