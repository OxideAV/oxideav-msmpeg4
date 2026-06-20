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
//! are clamped to the nearest valid sample (edge-extend) — the
//! unrestricted-MV behaviour the spec/06 §3.5 wrap range `[-63, +63]`
//! half-pel (= ±32 integer pixels) is designed to cover on sub-QCIF
//! streams. For larger pictures the wrap fits within the picture
//! easily.

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
/// Half-pel rounding: the H.263 / MPEG-4 §7.6.3.1 "rounded average"
/// `(a + b + 1) >> 1`. For 2D half-pel we average 4 samples with
/// `+2 >> 2`.
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
/// This version handles only the 1-MV-per-MB case (MB-type 0 / 1). The
/// 4-MV-per-MB (INTER4V, MB-type 2) chroma MV is derived by
/// [`chroma_mv_from_four_luma`].
pub fn chroma_mv_from_luma(luma_mv_half: (i32, i32)) -> (i32, i32) {
    // For 1-MV MBs the averaging is a no-op; halve to map from luma
    // half-pel to chroma half-pel. Arithmetic shift right rounds toward
    // negative infinity; integer-halving via `>>1` is the §7.6.3.4
    // default for the single-MV reduction.
    (luma_mv_half.0 >> 1, luma_mv_half.1 >> 1)
}

/// Apply the §7.6.3.4 "modification towards the nearest half sample
/// position" to one eighth-sample-resolution chrominance MV component,
/// returning the half-sample-resolution component.
///
/// The full half-pel-domain derivation is: sum the four luminance
/// half-pel MV components (`s` = sum of four values, each `2*int +
/// half_bit`), giving an eighth-sample-resolution number once divided by
/// `2*K = 8`. Rather than dividing first (and losing the fractional
/// eighths), we keep `s` and read the eighth-position lookup directly
/// from `s`: the integer part is `s / 8` (toward −∞) and the residual
/// `s mod 8` selects the half-sample offset per ISO/IEC 14496-2:2004(E)
/// Table 7-12 (eighth pixel position → resulting position, in halves):
///
/// | eighth `r` | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
/// | result/2   | 0 | 0 | 1 | 1 | 1 | 1 | 1 | 2 |
///
/// So the resulting half-pel component = `2*(s div 8) + table[s mod 8]`.
///
/// # Source
///
/// `docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`
/// §7.6.3.4 ("the sum of the K luminance vectors … dividing this sum by
/// 2*K … modified towards the nearest half sample position") + Table
/// 7-12. spec/06 §3 establishes that MS-MPEG-4 v1/v2/v3 inherit the
/// §7.6.x motion-vector layout; the four-MV decode loop itself is traced
/// in `docs/video/msmpeg4/spec/16-mv-vlc-dc-mcbpc-extraction.md` §3.1.
fn eighth_to_half_component(sum_half: i32) -> i32 {
    // Table 7-12 (eighth → half), indexed by the non-negative residue.
    const TABLE: [i32; 8] = [0, 0, 1, 1, 1, 1, 1, 2];
    let div = sum_half.div_euclid(8);
    let rem = sum_half.rem_euclid(8) as usize;
    2 * div + TABLE[rem]
}

/// Derive the chroma half-pel MV from the four per-8x8-block luminance
/// half-pel MVs of an INTER4V (MB-type 2) macroblock, per MPEG-4
/// §7.6.3.4: sum the four luminance MVs, divide by `2*K = 8`, and modify
/// toward the nearest half-sample position (Table 7-12). See
/// [`eighth_to_half_component`] for the exact arithmetic and source.
pub fn chroma_mv_from_four_luma(block_mvs_half: [(i32, i32); 4]) -> (i32, i32) {
    let sum_x: i32 = block_mvs_half.iter().map(|&(x, _)| x).sum();
    let sum_y: i32 = block_mvs_half.iter().map(|&(_, y)| y).sum();
    (
        eighth_to_half_component(sum_x),
        eighth_to_half_component(sum_y),
    )
}

/// Copy the four 8x8 luma blocks of an MB (each with its own half-pel MV
/// in Figure 6-8 raster order) and the two 8x8 chroma blocks (with the
/// §7.6.3.4-derived shared chroma MV) from the reference into the
/// destination at `(mb_x, mb_y)`. This is the INTER4V (MB-type 2)
/// counterpart of [`mc_macroblock`].
///
/// `block_mvs_half[i]` is the half-pel luma MV for Figure 6-8 block
/// `i+1` (block 1 = top-left 8x8, block 2 = top-right, block 3 =
/// bottom-left, block 4 = bottom-right).
///
/// # Source
///
/// `docs/video/msmpeg4/spec/16-mv-vlc-dc-mcbpc-extraction.md` §3.1 (the
/// per-component MV decoder is looped 4× for MB-type 2, one MV per
/// luminance block) + `docs/video/mpeg4-visual/`
/// `ISO_IEC_14496-2-2004-3rd-edition.txt` §7.6.3.4 (per-block luminance
/// MC + the chroma MV sum/2K derivation).
#[allow(clippy::too_many_arguments)]
pub fn mc_macroblock_4mv(
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
    block_mvs_half: [(i32, i32); 4],
) {
    // Four 8x8 luma blocks in Figure 6-8 raster order.
    for (i, &(mvx, mvy)) in block_mvs_half.iter().enumerate() {
        let bx = (i & 1) * 8;
        let by = (i >> 1) * 8;
        let luma_x = (mb_x * 16 + bx) as i32;
        let luma_y = (mb_y * 16 + by) as i32;
        let luma_off = (mb_y * 16 + by) * y_stride + (mb_x * 16 + bx);
        if luma_off < dst_y.len() {
            mc_block(
                ref_y,
                &mut dst_y[luma_off..],
                y_stride,
                luma_x,
                luma_y,
                mvx,
                mvy,
                8,
            );
        }
    }

    // Shared 8x8 chroma blocks at the §7.6.3.4-derived MV.
    let chroma_x = (mb_x * 8) as i32;
    let chroma_y = (mb_y * 8) as i32;
    let (cmx, cmy) = chroma_mv_from_four_luma(block_mvs_half);
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
    fn eighth_to_half_table_7_12_zero_sum() {
        // Four (0,0) MVs → sum 0 → chroma MV 0 in both components.
        assert_eq!(chroma_mv_from_four_luma([(0, 0); 4]), (0, 0));
    }

    #[test]
    fn eighth_to_half_four_equal_mvs_reduce_to_single() {
        // Four identical luma MVs (each +2 half-pel) → sum 8 → div 8 = 1
        // full half-pel step, residue 0 → result 2 (= +1 full pel). This
        // matches the 1-MV `chroma_mv_from_luma((+2,0)) = (+1, 0)` after
        // the half-pel → chroma half-pel halving identity: four +2's
        // average to +2, halved to +1 chroma full pel = 2 chroma
        // half-pel.
        assert_eq!(chroma_mv_from_four_luma([(2, 2); 4]), (2, 2));
    }

    #[test]
    fn eighth_to_half_table_7_12_residues() {
        // Hold y at 0; sweep the x sum across one residue cycle to pin
        // Table 7-12 (eighth → half): residues {0,1}→0, {2..6}→1, 7→2.
        let f = |s: i32| chroma_mv_from_four_luma([(s, 0), (0, 0), (0, 0), (0, 0)]).0;
        assert_eq!(f(0), 0);
        assert_eq!(f(1), 0);
        assert_eq!(f(2), 1);
        assert_eq!(f(6), 1);
        assert_eq!(f(7), 2);
        // div carries: sum 8 → 2 (one full half-pel + residue 0), sum 9
        // → 2, sum 10 → 3 (= 2*1 + 1).
        assert_eq!(f(8), 2);
        assert_eq!(f(9), 2);
        assert_eq!(f(10), 3);
    }

    #[test]
    fn eighth_to_half_negative_sums_round_toward_neg_inf() {
        // Euclidean division keeps the residue non-negative so the
        // Table 7-12 lookup is well-defined for negative sums too.
        let f = |s: i32| chroma_mv_from_four_luma([(s, 0), (0, 0), (0, 0), (0, 0)]).0;
        // -8 → div -1, residue 0 → -2.
        assert_eq!(f(-8), -2);
        // -1 → div -1, residue 7 → -2 + 2 = 0.
        assert_eq!(f(-1), 0);
        // -2 → div -1, residue 6 → -2 + 1 = -1.
        assert_eq!(f(-2), -1);
    }

    #[test]
    fn mc_macroblock_4mv_zero_mvs_copies_each_block() {
        // 16x16 ramp reference; four (0,0) block MVs ⇒ exact copy of
        // the luma MB plus both chroma blocks.
        let w = 16usize;
        let h = 16usize;
        let mut y = vec![0u8; w * h];
        for j in 0..h {
            for i in 0..w {
                y[j * w + i] = ((i * 3 + j * 5) & 0xff) as u8;
            }
        }
        let cw = 8usize;
        let ch = 8usize;
        let mut cb = vec![0u8; cw * ch];
        let mut cr = vec![0u8; cw * ch];
        for j in 0..ch {
            for i in 0..cw {
                cb[j * cw + i] = (40 + i + j) as u8;
                cr[j * cw + i] = (200 - i - j) as u8;
            }
        }
        let ref_y = RefPlane {
            data: &y,
            stride: w,
            width: w,
            height: h,
        };
        let ref_cb = RefPlane {
            data: &cb,
            stride: cw,
            width: cw,
            height: ch,
        };
        let ref_cr = RefPlane {
            data: &cr,
            stride: cw,
            width: cw,
            height: ch,
        };
        let mut dy = vec![0u8; w * h];
        let mut dcb = vec![0u8; cw * ch];
        let mut dcr = vec![0u8; cw * ch];
        mc_macroblock_4mv(
            &ref_y,
            &ref_cb,
            &ref_cr,
            &mut dy,
            &mut dcb,
            &mut dcr,
            w,
            cw,
            0,
            0,
            [(0, 0); 4],
        );
        assert_eq!(dy, y, "four zero-MV luma blocks must copy the MB");
        assert_eq!(dcb, cb);
        assert_eq!(dcr, cr);
    }

    #[test]
    fn mc_macroblock_4mv_distinct_block_mvs() {
        // Give block 1 (top-left) an MVDx of +2 half-pel (= +1 full
        // pel); the other three blocks MV 0. Only the top-left 8x8 of
        // the destination should sample one column to the right.
        let w = 16usize;
        let h = 16usize;
        let mut y = vec![0u8; w * h];
        for j in 0..h {
            for i in 0..w {
                y[j * w + i] = (i + j * 16) as u8;
            }
        }
        let cw = 8usize;
        let ch = 8usize;
        let cb = vec![100u8; cw * ch];
        let cr = vec![150u8; cw * ch];
        let ref_y = RefPlane {
            data: &y,
            stride: w,
            width: w,
            height: h,
        };
        let ref_cb = RefPlane {
            data: &cb,
            stride: cw,
            width: cw,
            height: ch,
        };
        let ref_cr = RefPlane {
            data: &cr,
            stride: cw,
            width: cw,
            height: ch,
        };
        let mut dy = vec![0u8; w * h];
        let mut dcb = vec![0u8; cw * ch];
        let mut dcr = vec![0u8; cw * ch];
        mc_macroblock_4mv(
            &ref_y,
            &ref_cb,
            &ref_cr,
            &mut dy,
            &mut dcb,
            &mut dcr,
            w,
            cw,
            0,
            0,
            [(2, 0), (0, 0), (0, 0), (0, 0)],
        );
        for j in 0..8usize {
            for i in 0..8usize {
                let src_col = (i + 1).min(15);
                assert_eq!(dy[j * w + i], y[j * w + src_col], "block-1 ({j},{i})");
            }
        }
        // Block 2 (top-right) used MV 0 → exact copy.
        for j in 0..8usize {
            for i in 8..16usize {
                assert_eq!(dy[j * w + i], y[j * w + i], "block-2 ({j},{i})");
            }
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
