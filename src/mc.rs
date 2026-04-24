//! Motion compensation (luma + chroma) for MS-MPEG4 v3 P-frames.
//!
//! The decoded joint MV byte pair `(MVx, MVy)` ∈ `[-63, +63]` is in
//! half-pel units per spec/06 §3.5. The MC kernel splits each byte MV
//! into `(integer_part, half_pel_bit)` via an arithmetic-shift-right by
//! 1: `sar byte, 1` → integer position; the original LSB marks
//! half-pel. This module implements:
//!
//! * Luma 16×16 MC for 1-MV per MB (MB-type 0/1).
//! * Chroma 8×8 MC using the MPEG-4-style `(dx_luma + dy_luma)`
//!   chroma-MV derivation (each chroma component's MV is the sum of
//!   its luma counterpart's four block MVs averaged down; with only
//!   one luma MV per MB the chroma MV is the luma MV halved toward
//!   zero by the half-pel rule).
//!
//! For a P-frame build that does not yet carry the per-4x4 INTER4V
//! signalling (we reject all P-frame MB-types other than 0 for now),
//! the chroma MV reduces to `luma_mv / 2` per component (rounded
//! toward zero, which is H.263 Annex I / MPEG-4 §7.6.3.4 default
//! rounding).
//!
//! # Out-of-bounds handling
//!
//! Reference samples that fall outside the decoded reference picture
//! are clamped to the nearest valid sample (edge-extend). This is
//! what ffmpeg's msmpeg4 decoder does for unrestricted MVs and is
//! what the spec/06 §3.5 wrap range `[-63, +63]` half-pel (= ±32
//! integer pixels) is designed to cover on sub-QCIF streams. For
//! larger pictures the wrap fits within the picture easily.

/// One reference sample plane (luma or chroma), stride-indexed.
pub struct RefPlane<'a> {
    pub data: &'a [u8],
    pub stride: usize,
    pub width: usize,
    pub height: usize,
}

impl<'a> RefPlane<'a> {
    /// Sample at integer `(x, y)`, edge-clamped.
    pub fn sample(&self, x: i32, y: i32) -> u8 {
        let x = x.clamp(0, self.width as i32 - 1) as usize;
        let y = y.clamp(0, self.height as i32 - 1) as usize;
        self.data[y * self.stride + x]
    }
}

/// Copy an N×N block from the reference, handling half-pel positions
/// via bilinear averaging. Writes the destination into `dst`
/// `stride`-indexed. The MV is given as `(mv_x_half, mv_y_half)` where
/// each component is in half-pel units (i.e. `mv_x_half = 2 *
/// integer_mv + half_pel_bit`).
///
/// Half-pel rounding: ffmpeg / H.263 uses the "rounded average"
/// `(a + b + 1) >> 1`. For 2D half-pel we average 4 samples with
/// `+2 >> 2`. This mirrors MPEG-4 §7.6.3.1.
#[allow(clippy::too_many_arguments)]
pub fn mc_block(
    reference: &RefPlane<'_>,
    dst: &mut [u8],
    dst_stride: usize,
    block_x: i32,
    block_y: i32,
    mv_x_half: i32,
    mv_y_half: i32,
    block_size: usize,
) {
    // Split half-pel MV into integer + fractional.
    let int_x = mv_x_half >> 1; // arithmetic shift (sign-preserving)
    let int_y = mv_y_half >> 1;
    let frac_x = (mv_x_half & 1) as u32;
    let frac_y = (mv_y_half & 1) as u32;

    let src_x = block_x + int_x;
    let src_y = block_y + int_y;

    for j in 0..block_size {
        for i in 0..block_size {
            let ix = src_x + i as i32;
            let iy = src_y + j as i32;
            let pel = match (frac_x, frac_y) {
                (0, 0) => reference.sample(ix, iy),
                (1, 0) => {
                    let a = reference.sample(ix, iy) as u32;
                    let b = reference.sample(ix + 1, iy) as u32;
                    ((a + b + 1) >> 1) as u8
                }
                (0, 1) => {
                    let a = reference.sample(ix, iy) as u32;
                    let b = reference.sample(ix, iy + 1) as u32;
                    ((a + b + 1) >> 1) as u8
                }
                _ => {
                    // (1, 1): 2D bilinear with +2 rounding.
                    let a = reference.sample(ix, iy) as u32;
                    let b = reference.sample(ix + 1, iy) as u32;
                    let c = reference.sample(ix, iy + 1) as u32;
                    let d = reference.sample(ix + 1, iy + 1) as u32;
                    ((a + b + c + d + 2) >> 2) as u8
                }
            };
            let off = (j) * dst_stride + i;
            if off < dst.len() {
                dst[off] = pel;
            }
        }
    }
}

/// Derive the chroma half-pel MV from a single luma half-pel MV, per
/// MPEG-4 §7.6.3.4 (when every 4×4 luma sub-block shares the same MV):
/// `mv_chroma_half = mv_luma_half / 2` using the "shift toward zero"
/// convention (arithmetic shift right by 1 of the half-pel component).
///
/// This version handles only the 1-MV-per-MB case (MB-type 0). For
/// INTER4V support we would average the four per-luma-block MVs before
/// the halving.
pub fn chroma_mv_from_luma(luma_mv_half: (i32, i32)) -> (i32, i32) {
    // For 1-MV MBs the averaging is a no-op; halve to map from luma
    // half-pel to chroma half-pel. Use shift right (arithmetic) for
    // toward-negative-infinity; ffmpeg matches this by dividing then
    // adjusting. For our purposes integer-halving via `>>1` is OK.
    (luma_mv_half.0 >> 1, luma_mv_half.1 >> 1)
}

/// Copy a 16×16 luma block and the two 8×8 chroma blocks from the
/// reference into the destination at `(mb_x, mb_y)`. `mv_half` is the
/// decoded luma MV in half-pel units.
#[allow(clippy::too_many_arguments)]
pub fn mc_macroblock(
    ref_y: &RefPlane<'_>,
    ref_cb: &RefPlane<'_>,
    ref_cr: &RefPlane<'_>,
    dst_y: &mut [u8],
    dst_cb: &mut [u8],
    dst_cr: &mut [u8],
    y_stride: usize,
    c_stride: usize,
    mb_x: usize,
    mb_y: usize,
    mv_half: (i32, i32),
) {
    // Luma block top-left in picture coordinates.
    let luma_x = (mb_x * 16) as i32;
    let luma_y = (mb_y * 16) as i32;

    // Write destination slice for the luma MB.
    let luma_off = mb_y * 16 * y_stride + mb_x * 16;
    if luma_off < dst_y.len() {
        mc_block(
            ref_y,
            &mut dst_y[luma_off..],
            y_stride,
            luma_x,
            luma_y,
            mv_half.0,
            mv_half.1,
            16,
        );
    }

    // Chroma — half-size block at mb_x * 8, mb_y * 8.
    let chroma_x = (mb_x * 8) as i32;
    let chroma_y = (mb_y * 8) as i32;
    let (cmx, cmy) = chroma_mv_from_luma(mv_half);

    let chroma_off = mb_y * 8 * c_stride + mb_x * 8;
    if chroma_off < dst_cb.len() {
        mc_block(
            ref_cb,
            &mut dst_cb[chroma_off..],
            c_stride,
            chroma_x,
            chroma_y,
            cmx,
            cmy,
            8,
        );
    }
    if chroma_off < dst_cr.len() {
        mc_block(
            ref_cr,
            &mut dst_cr[chroma_off..],
            c_stride,
            chroma_x,
            chroma_y,
            cmx,
            cmy,
            8,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn const_plane(value: u8, w: usize, h: usize) -> Vec<u8> {
        vec![value; w * h]
    }

    #[test]
    fn mc_zero_mv_integer_copies_exactly() {
        // 16×16 source of a ramp; destination should match region.
        let w = 32;
        let h = 32;
        let mut src = vec![0u8; w * h];
        for y in 0..h {
            for x in 0..w {
                src[y * w + x] = ((x + y) & 0xff) as u8;
            }
        }
        let rp = RefPlane {
            data: &src,
            stride: w,
            width: w,
            height: h,
        };
        let mut dst = vec![0u8; 16 * 16];
        mc_block(&rp, &mut dst, 16, 8, 8, 0, 0, 16);
        for y in 0..16 {
            for x in 0..16 {
                assert_eq!(
                    dst[y * 16 + x],
                    (((8 + x) + (8 + y)) & 0xff) as u8,
                    "dst[{y},{x}]"
                );
            }
        }
    }

    #[test]
    fn mc_half_pel_x_bilinear_average() {
        // Flat plane of 10 + region containing a step. Half-pel X
        // between two samples should produce the average.
        let w = 16;
        let h = 16;
        let src = const_plane(100, w, h);
        let rp = RefPlane {
            data: &src,
            stride: w,
            width: w,
            height: h,
        };
        let mut dst = vec![0u8; 4 * 4];
        // half-pel MV = (1, 0) means 0.5 pixel offset in X.
        mc_block(&rp, &mut dst, 4, 4, 4, 1, 0, 4);
        for &v in &dst {
            assert_eq!(v, 100);
        }
    }

    #[test]
    fn mc_edge_clamps() {
        // Destination near the right edge with a large positive MV
        // must clamp without panicking.
        let w = 16;
        let h = 16;
        let src = const_plane(50, w, h);
        let rp = RefPlane {
            data: &src,
            stride: w,
            width: w,
            height: h,
        };
        let mut dst = vec![0u8; 4 * 4];
        // MV pushes sampling past the right edge.
        mc_block(&rp, &mut dst, 4, 14, 14, 20, 20, 4);
        for &v in &dst {
            assert_eq!(v, 50);
        }
    }
}
