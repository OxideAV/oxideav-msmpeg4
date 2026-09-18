//! 8×8 inverse DCT — the decoder's exact integer kernel (spec/19 §1)
//! plus a float forward/inverse pair for the encoder's transform stage.
//!
//! ## The decode-side kernel
//!
//! `docs/video/msmpeg4/spec/19-idct-reconstruction-and-intra-prediction.md`
//! §1 transcribes the vendor decoder's IDCT: a Chen–Wang separable
//! integer butterfly (rows first, then columns; constants
//! `W1 = 2841, W2 = 2676, W3 = 2408, W5 = 1609, W6 = 1108, W7 = 565`
//! appearing as the sums / differences `2276, 3406, 799, 4017, 3784,
//! 1568` and `181 ≈ 128·√2`) in two bindings:
//!
//! * **scalar** (§1.3, `0x1c22abf2` / `0x1c22c0c1`): exact arithmetic —
//!   [`idct8x8_scalar`];
//! * **MMX** (§1.4, `0x1c22a039` / `0x1c22afcd`): the same butterfly in
//!   16-bit lanes — [`idct8x8_int`]. Inputs truncate to int16, the
//!   pair sums feeding each `pmaddwd` wrap to int16, the ×181 stage
//!   sees `sext23` of its operand, and the row-pass outputs saturate to
//!   int16 before the column pass.
//!
//! The two agree exactly while every intermediate stays inside int16 /
//! 23 bits (all 432 vendor-encoded blocks the docs traced), and differ
//! only on escape-coded extremes (`tables/idct-mmx-saturation-probe.csv`
//! is the discriminating hardware-grade test vector). spec/19 §1.4
//! recommends the MMX model as the normative MP42 / MP43 kernel — it is
//! what every MMX-capable machine runs and what every reference plane
//! was produced with — so [`idct8x8_to_pel`] routes through
//! [`idct8x8_int`]. The constants are pinned against
//! `tables/idct-mmx-constants.csv` / `tables/idct-scalar-arith-constants.csv`
//! (staged copies of the docs extractions) by the unit tests.
//!
//! Rounding summary (spec/19 §1.3): row pass `+128` at the DC term and
//! `(… + 128) >> 8` inside the √2 stage, outputs `>> 8`; column pass
//! `+4` before every `>> 3` of the odd part, `+8192` at the DC term,
//! the same √2 stage, outputs `>> 14`. Every shift is arithmetic
//! (floor) — ties never round away from zero.
//!
//! ## The encoder-side pair
//!
//! [`fdct8x8`] / [`idct8x8`] are the float orthonormal DCT-II pair
//!
//!   x[n] = 0.5 * Σ_k C_k * X[k] * cos((2n+1) k π / 16)
//!
//! with `C_0 = 1/√2`, `C_{>0} = 1`. The encoder only needs the forward
//! transform (its reconstruction goes through the production decoder,
//! so encoder recon == decoder recon by construction); the float
//! inverse is kept for the transform-pair unit tests.

use std::f32::consts::PI;
use std::sync::OnceLock;

// Butterfly constants of spec/19 §1.3 (`tables/idct-scalar-arith-constants.csv`
// `constant_roles`, `tables/idct-mmx-constants.csv` word pairs).
const W1_MINUS_W7: i64 = 2276;
const W1_PLUS_W7: i64 = 3406;
const W3_MINUS_W5: i64 = 799;
const W3_PLUS_W5: i64 = 4017;
const W2_PLUS_W6: i64 = 3784;
const W2_MINUS_W6: i64 = 1568;
const W3: i64 = 2408;
const W6: i64 = 1108;
const W7: i64 = 565;
const SQRT2_128: i64 = 181;

/// Low 16 bits of `v`, sign-extended (an int16 lane that wraps).
#[inline]
fn wrap16(v: i32) -> i32 {
    v as i16 as i32
}

/// Saturate `v` to int16 (`packssdw`).
#[inline]
fn sat16(v: i32) -> i32 {
    v.clamp(i16::MIN as i32, i16::MAX as i32)
}

/// Bits 0..22 of `v` with the sign taken from bit 22 (spec/19 §1.4
/// item 3: the operand the MMX ×181 stage actually sees).
#[inline]
fn sext23(v: i32) -> i32 {
    (v << 9) >> 9
}

/// One 1-D Chen–Wang butterfly on eight inputs.
///
/// `row_pass` selects the row-pass rounding (`+128` DC bias, `>> 8`
/// outputs, no odd-part pre-shift) versus the column-pass rounding
/// (`+8192` DC bias, `+4 >> 3` on the odd part, `>> 14` outputs).
/// `lanes16` enables the MMX lane model (§1.4): the pair sums wrap to
/// int16 and the √2 stage sees `sext23`. The arithmetic is carried in
/// `i64` so the scalar form is the exact (unbounded) §1.3 arithmetic
/// and the lane form — whose intermediates the int16 inputs bound to
/// well under 2³¹ — is exact too.
#[inline]
fn butterfly(c: [i64; 8], row_pass: bool, lanes16: bool) -> [i64; 8] {
    let pair = |a: i64, b: i64| {
        if lanes16 {
            wrap16((a + b) as i32) as i64
        } else {
            a + b
        }
    };
    let root = |v: i64| if lanes16 { sext23(v as i32) as i64 } else { v };
    let (dc_shift, dc_bias, odd_bias, odd_shift, out_shift) = if row_pass {
        (11, 128, 0, 0, 8)
    } else {
        (8, 8192, 4, 3, 14)
    };
    let x0 = (c[0] << dc_shift) + dc_bias;
    let x1 = c[4] << dc_shift;
    let t8 = W7 * pair(c[1], c[7]) + odd_bias;
    let x4 = (t8 + W1_MINUS_W7 * c[1]) >> odd_shift;
    let x5 = (t8 - W1_PLUS_W7 * c[7]) >> odd_shift;
    let t8 = W3 * pair(c[3], c[5]) + odd_bias;
    let x6 = (t8 - W3_MINUS_W5 * c[5]) >> odd_shift;
    let x7 = (t8 - W3_PLUS_W5 * c[3]) >> odd_shift;
    let x8 = x0 + x1;
    let x0 = x0 - x1;
    let t1 = W6 * pair(c[2], c[6]) + odd_bias;
    let x2 = (t1 - W2_PLUS_W6 * c[6]) >> odd_shift;
    let x3 = (t1 + W2_MINUS_W6 * c[2]) >> odd_shift;
    let x1 = x4 + x6;
    let x4 = x4 - x6;
    let x6 = x5 + x7;
    let x5 = x5 - x7;
    let x7 = x8 + x3;
    let x8 = x8 - x3;
    let x3 = x0 + x2;
    let x0 = x0 - x2;
    let x2 = (SQRT2_128 * root(x4 + x5) + 128) >> 8;
    let x4 = (SQRT2_128 * root(x4 - x5) + 128) >> 8;
    [
        (x7 + x1) >> out_shift,
        (x3 + x2) >> out_shift,
        (x0 + x4) >> out_shift,
        (x8 + x6) >> out_shift,
        (x8 - x6) >> out_shift,
        (x0 - x4) >> out_shift,
        (x3 - x2) >> out_shift,
        (x7 - x1) >> out_shift,
    ]
}

/// Separable two-pass driver shared by both bindings.
fn idct8x8_two_pass(coeffs: &[i32; 64], out: &mut [i32; 64], lanes16: bool) {
    let mut tmp = [0i64; 64];
    for r in 0..8 {
        let mut c = [0i64; 8];
        for (k, v) in c.iter_mut().enumerate() {
            let x = coeffs[r * 8 + k];
            *v = if lanes16 { wrap16(x) } else { x } as i64;
        }
        let o = butterfly(c, true, lanes16);
        for (k, v) in o.iter().enumerate() {
            tmp[r * 8 + k] = if lanes16 { sat16(*v as i32) as i64 } else { *v };
        }
    }
    for k in 0..8 {
        let mut t = [0i64; 8];
        for (r, v) in t.iter_mut().enumerate() {
            *v = tmp[r * 8 + k];
        }
        let o = butterfly(t, false, lanes16);
        for (r, v) in o.iter().enumerate() {
            out[r * 8 + k] = (*v).clamp(i32::MIN as i64, i32::MAX as i64) as i32;
        }
    }
}

/// The scalar Chen–Wang IDCT of spec/19 §1.3 (`0x1c22abf2` put /
/// `0x1c22c0c1` residual): the exact (unbounded) arithmetic on the
/// natural-order coefficient matrix. Output is the unclipped reconstruction (`put`
/// clips to 0..255 afterwards; `residual` adds it to the prediction).
///
/// Equal to [`idct8x8_int`] whenever every intermediate fits int16 /
/// 23 bits; kept as the reference the lane model is checked against.
pub fn idct8x8_scalar(coeffs: &[i32; 64], out: &mut [i32; 64]) {
    idct8x8_two_pass(coeffs, out, false);
}

/// The MMX IDCT of spec/19 §1.4 (`0x1c22a039` put / `0x1c22afcd`
/// residual) — the normative MP42 / MP43 kernel: the §1.3 butterfly in
/// 16-bit lanes (inputs truncated to int16, `pmaddwd` pair sums wrapped
/// to int16, `sext23` into the ×181 stage, row outputs saturated to
/// int16). Output is the unclipped reconstruction, as for
/// [`idct8x8_scalar`].
///
/// Reproduces `tables/idct-mmx-saturation-probe.csv` sample-exactly
/// (the escape-coded PQUANT-31 blocks on which the scalar form is 28 /
/// 21 samples off) and the 432 traced vendor blocks of spec/19 §1.3.
pub fn idct8x8_int(coeffs: &[i32; 64], out: &mut [i32; 64]) {
    idct8x8_two_pass(coeffs, out, true);
}

fn cos_table() -> &'static [[f32; 8]; 8] {
    static T: OnceLock<[[f32; 8]; 8]> = OnceLock::new();
    T.get_or_init(|| {
        let mut t = [[0.0f32; 8]; 8];
        for (k, row) in t.iter_mut().enumerate() {
            let c_k = if k == 0 {
                (1.0_f32 / 2.0_f32).sqrt()
            } else {
                1.0
            };
            for (n, cell) in row.iter_mut().enumerate() {
                *cell = 0.5 * c_k * ((2 * n + 1) as f32 * k as f32 * PI / 16.0).cos();
            }
        }
        t
    })
}

/// Inverse DCT of an 8×8 natural-order block, in-place.
///
/// Rows then columns — one of the standard separable IDCT orderings.
/// Output is the natural-domain signed pel prediction/residual.
pub fn idct8x8(block: &mut [f32; 64]) {
    let t = cos_table();
    let mut tmp = [0.0f32; 64];
    for y in 0..8 {
        for n in 0..8 {
            let mut s = 0.0f32;
            for k in 0..8 {
                s += t[k][n] * block[y * 8 + k];
            }
            tmp[y * 8 + n] = s;
        }
    }
    for x in 0..8 {
        for m in 0..8 {
            let mut s = 0.0f32;
            for k in 0..8 {
                s += t[k][m] * tmp[k * 8 + x];
            }
            block[m * 8 + x] = s;
        }
    }
}

/// Forward DCT of an 8×8 natural-order block, in-place.
///
/// The exact mathematical inverse of [`idct8x8`]: with the orthogonal
/// basis `t[k][n] = 0.5 · C_k · cos((2n+1) k π / 16)` the forward pair
/// is
///
///   X[k] = Σ_n t[k][n] · x[n]
///
/// applied first row-wise then column-wise (same separable ordering as
/// the inverse). `fdct8x8` followed by [`idct8x8`] reproduces the
/// input up to float rounding — the encoder's transform stage.
pub fn fdct8x8(block: &mut [f32; 64]) {
    let t = cos_table();
    let mut tmp = [0.0f32; 64];
    for y in 0..8 {
        for k in 0..8 {
            let mut s = 0.0f32;
            for n in 0..8 {
                s += t[k][n] * block[y * 8 + n];
            }
            tmp[y * 8 + k] = s;
        }
    }
    for x in 0..8 {
        for k in 0..8 {
            let mut s = 0.0f32;
            for m in 0..8 {
                s += t[k][m] * tmp[m * 8 + x];
            }
            block[k * 8 + x] = s;
        }
    }
}

/// Forward DCT of an `i32` pel/residual block into float DCT-domain
/// coefficients. Encoder-side entry point pairing with
/// [`idct8x8_to_pel`]: feed intra blocks the unsigned pel values
/// (0..=255 — the intra DC carries the pel mean directly, no −128
/// offset, matching the decode-side convention documented on
/// `picture::write_block_to_picture`) and inter blocks the signed
/// MC residual.
pub fn fdct8x8_from_pels(pels: &[i32; 64], out: &mut [f32; 64]) {
    for i in 0..64 {
        out[i] = pels[i] as f32;
    }
    fdct8x8(out);
}

/// Apply the decoder's exact IDCT ([`idct8x8_int`], spec/19 §1.4) to
/// an `i32` coefficient block (post-dequantisation, natural order) and
/// return the unclipped reconstruction: the pel values of an intra
/// `put` block (the caller clips to 0..255 — spec/19 §1.3 `clip255`)
/// or the signed residual of an inter block (added to the
/// motion-compensated prediction with a saturating 0..255 add,
/// spec/19 §1.5).
pub fn idct8x8_to_pel(coeffs: &[i32; 64], out: &mut [i32; 64]) {
    idct8x8_int(coeffs, out);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Parse `tables/idct-mmx-saturation-probe.csv` (staged copy of the
    /// docs extraction): per block, the 64 natural-order coefficients
    /// `c<r>_<k>` and the 64 output samples `p<r>_<k>`.
    fn saturation_probe() -> Vec<([i32; 64], [u8; 64])> {
        let csv = include_str!("../tables/idct-mmx-saturation-probe.csv");
        let mut lines = csv.lines().filter(|l| !l.trim().is_empty());
        let header: Vec<&str> = lines.next().unwrap().split(',').collect();
        let mut out = Vec::new();
        for line in lines {
            let cells: Vec<&str> = line.split(',').collect();
            assert_eq!(cells.len(), header.len());
            let mut c = [0i32; 64];
            let mut p = [0u8; 64];
            for (name, cell) in header.iter().zip(&cells) {
                if let Some(rest) = name.strip_prefix('c') {
                    let (r, k) = rest.split_once('_').unwrap();
                    c[r.parse::<usize>().unwrap() * 8 + k.parse::<usize>().unwrap()] =
                        cell.parse().unwrap();
                } else if let Some(rest) = name.strip_prefix('p') {
                    let (r, k) = rest.split_once('_').unwrap();
                    p[r.parse::<usize>().unwrap() * 8 + k.parse::<usize>().unwrap()] =
                        cell.parse().unwrap();
                }
            }
            out.push((c, p));
        }
        assert_eq!(out.len(), 2, "the probe carries two intra-put blocks");
        out
    }

    fn clip255(v: i32) -> u8 {
        v.clamp(0, 255) as u8
    }

    #[test]
    fn mmx_model_reproduces_the_hardware_saturation_probe() {
        // spec/19 §1.4: the int16 / 23-bit lane model reproduces both
        // escape-coded PQUANT-31 blocks 64/64 (hardware-grade capture,
        // tables/README-10.md §1).
        for (blk, (c, p)) in saturation_probe().iter().enumerate() {
            let mut out = [0i32; 64];
            idct8x8_int(c, &mut out);
            let got: Vec<u8> = out.iter().map(|&v| clip255(v)).collect();
            assert_eq!(&got[..], &p[..], "probe block {blk} (MMX model)");
        }
    }

    #[test]
    fn scalar_form_misses_the_saturation_probe_as_documented() {
        // tables/idct-mmx-saturation-probe.meta: the int32 scalar
        // arithmetic misses 28 and 21 of 64 samples on the two blocks —
        // the vector discriminates the two bindings, so a regression
        // that silently routes through the scalar form is caught here.
        let misses: Vec<usize> = saturation_probe()
            .iter()
            .map(|(c, p)| {
                let mut out = [0i32; 64];
                idct8x8_scalar(c, &mut out);
                out.iter()
                    .zip(p.iter())
                    .filter(|(&v, &want)| clip255(v) != want)
                    .count()
            })
            .collect();
        assert_eq!(misses, vec![28, 21]);
    }

    #[test]
    fn constants_match_the_staged_extractions() {
        // Every butterfly immediate of spec/19 §1.3 must appear in the
        // scalar-body extraction, and every MMX `pmaddwd` word pair must
        // be made of the same ten constants (idct-mmx-constants.meta:
        // `word_pairs_equal_scalar_immediates: yes`).
        let scalar = include_str!("../tables/idct-scalar-arith-constants.csv");
        let imm: std::collections::HashSet<i64> = scalar
            .lines()
            .skip(1)
            .filter_map(|l| l.split(',').nth(5).and_then(|v| v.parse().ok()))
            .collect();
        for w in [
            W7,
            W1_MINUS_W7,
            W1_PLUS_W7,
            W3,
            W3_MINUS_W5,
            W3_PLUS_W5,
            W6,
            W2_PLUS_W6,
            W2_MINUS_W6,
            SQRT2_128,
            128,
            4,
            32,
        ] {
            assert!(
                imm.contains(&w),
                "immediate {w} missing from the scalar extraction"
            );
        }
        // Shifts: 11 / 8 (row), 3 / 14 (column).
        for sh in [11, 8, 3, 14] {
            assert!(
                imm.contains(&sh),
                "shift {sh} missing from the scalar extraction"
            );
        }
        let mmx = include_str!("../tables/idct-mmx-constants.csv");
        let allowed: std::collections::HashSet<i64> = [
            W7,
            W1_MINUS_W7,
            -W1_PLUS_W7,
            W3,
            -W3_MINUS_W5,
            -W3_PLUS_W5,
            W6,
            -W2_PLUS_W6,
            W2_MINUS_W6,
            SQRT2_128,
            SQRT2_128 * 128,
            128,
            0,
        ]
        .into_iter()
        .collect();
        let mut rows = 0;
        for l in mmx.lines().skip(1) {
            let cells: Vec<&str> = l.split(',').collect();
            for w in &cells[2..6] {
                let w: i64 = w.parse().unwrap();
                assert!(allowed.contains(&w), "MMX word {w} is not a §1.3 constant");
            }
            rows += 1;
        }
        assert_eq!(rows, 16);
    }

    #[test]
    fn dc_only_block_follows_the_spec18_flat_formula() {
        // spec/18 §7: a DC-only intra block reconstructs flat at
        // `clip(((dc · 8) · 256 + 8192) >> 14)`, i.e. the row pass
        // yields `dc << 3` and the column pass `(t0 << 8 + 8192) >> 14`.
        for dc in [-3000, -1025, -7, 0, 1, 636, 1012, 1024, 2047, 4095] {
            let mut c = [0i32; 64];
            c[0] = dc;
            let mut out = [0i32; 64];
            idct8x8_int(&c, &mut out);
            let want = ((dc * 8) * 256 + 8192) >> 14;
            assert!(out.iter().all(|&v| v == want), "dc {dc}: got {out:?}");
            let mut sc = [0i32; 64];
            idct8x8_scalar(&c, &mut sc);
            assert_eq!(sc, out);
        }
        // Beyond `dc · 8 > 32767` the lane model saturates the row pass
        // (spec/19 §1.4 item 4) and the flat formula no longer applies;
        // every column still sees the same saturated vector.
        let mut c = [0i32; 64];
        c[0] = 5000;
        let mut out = [0i32; 64];
        idct8x8_int(&c, &mut out);
        for r in 0..8 {
            assert!(out[r * 8..r * 8 + 8].iter().all(|&v| v == out[r * 8]));
        }
        // DC 636 (PQUANT 6, 636 / 8 = 79.5): the exact kernel lands on
        // 80 — `(636 · 8 · 256 + 8192) >> 14 = 1310720 >> 14` is an
        // integer, there is no tie to break. (The float kernel this
        // replaced rounded that .5 downward to track the black-box
        // reference decode of the fixture harness, which is not the
        // vendor arithmetic — spec/19 §1 is.)
        let mut c = [0i32; 64];
        c[0] = 636;
        let mut out = [0i32; 64];
        idct8x8_to_pel(&c, &mut out);
        assert!(out.iter().all(|&v| v == 80));
    }

    #[test]
    fn lane_model_equals_scalar_form_inside_int16_range() {
        // spec/19 §1.4: within int16 / 23-bit range the two forms are
        // identical (all 432 traced vendor blocks). The 23-bit bound
        // sits on the odd-part sums (`x4 + x5` ≈ (W1 + 2·W7)·|c1| + …),
        // and the column pass sees row sums of up to eight of them, so
        // dense blocks stay under |coeff| ≤ 64 here (a sparse pass at
        // |coeff| ≤ 300 follows); the saturation probe covers the
        // out-of-range side.
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        for trial in 0..2000 {
            let mut c = [0i32; 64];
            let density = (trial % 8) + 1;
            for v in c.iter_mut() {
                let r = next();
                if (r % 8) < density as u64 {
                    *v = ((r >> 8) % 129) as i32 - 64;
                }
            }
            let mut a = [0i32; 64];
            let mut b = [0i32; 64];
            idct8x8_int(&c, &mut a);
            idct8x8_scalar(&c, &mut b);
            assert_eq!(a, b, "trial {trial}: {c:?}");
        }
        for trial in 0..2000 {
            let mut c = [0i32; 64];
            for _ in 0..3 {
                let r = next();
                c[(r % 64) as usize] = ((r >> 8) % 601) as i32 - 300;
            }
            let mut a = [0i32; 64];
            let mut b = [0i32; 64];
            idct8x8_int(&c, &mut a);
            idct8x8_scalar(&c, &mut b);
            assert_eq!(a, b, "sparse trial {trial}: {c:?}");
        }
    }

    #[test]
    fn lane_model_never_overflows_on_arbitrary_i32_input() {
        // Debug-build overflow guard: the int16 truncation of the
        // inputs bounds every intermediate well inside i32 whatever the
        // caller feeds in (the DC predictor chain is unbounded on
        // hostile streams).
        for &fill in &[i32::MIN, i32::MAX, -32768, 32767, 0x7fff_8000, -0x8000_0000] {
            let c = [fill; 64];
            let mut out = [0i32; 64];
            idct8x8_int(&c, &mut out);
            let _ = out;
        }
    }

    #[test]
    fn dc_only_block_is_uniform() {
        // 8 * 128 in DCT domain -> 128 per pel after IDCT normalisation.
        let mut b = [0.0f32; 64];
        b[0] = 8.0 * 128.0;
        idct8x8(&mut b);
        for &v in &b {
            assert!((v - 128.0).abs() < 1.0, "got {v}, want ~128");
        }
    }

    #[test]
    fn zero_in_zero_out() {
        let mut b = [0.0f32; 64];
        idct8x8(&mut b);
        assert!(b.iter().all(|&v| v.abs() < 1e-5));
        let c = [0i32; 64];
        let mut out = [1i32; 64];
        idct8x8_int(&c, &mut out);
        assert!(out.iter().all(|&v| v == 0));
    }

    #[test]
    fn fdct_uniform_block_is_dc_only() {
        // Uniform 128 per pel -> DC = 8 * 128, all AC ~0 (inverse of
        // `dc_only_block_is_uniform`).
        let mut b = [128.0f32; 64];
        fdct8x8(&mut b);
        assert!((b[0] - 8.0 * 128.0).abs() < 0.01, "DC got {}", b[0]);
        for &v in &b[1..] {
            assert!(v.abs() < 0.01, "AC leak {v}");
        }
    }

    #[test]
    fn fdct_then_idct_round_trips() {
        // Pseudo-random-ish pel content; fdct -> idct must reproduce it
        // to well under half a pel (float basis is orthogonal).
        let mut pels = [0i32; 64];
        for (i, p) in pels.iter_mut().enumerate() {
            *p = ((i * 37 + 11) % 256) as i32;
        }
        let mut f = [0.0f32; 64];
        fdct8x8_from_pels(&pels, &mut f);
        idct8x8(&mut f);
        for i in 0..64 {
            assert!(
                (f[i] - pels[i] as f32).abs() < 0.01,
                "pos {i}: got {} want {}",
                f[i],
                pels[i]
            );
        }
    }

    #[test]
    fn fdct_single_basis_function_round_trips_through_int_pipeline() {
        // A signed residual block survives fdct -> (round to i32) ->
        // idct8x8_to_pel with only small rounding error.
        let mut pels = [0i32; 64];
        for j in 0..8 {
            for i in 0..8 {
                pels[j * 8 + i] = if (i + j) % 2 == 0 { 20 } else { -20 };
            }
        }
        let mut f = [0.0f32; 64];
        fdct8x8_from_pels(&pels, &mut f);
        let mut coeffs = [0i32; 64];
        for i in 0..64 {
            coeffs[i] = f[i].round() as i32;
        }
        let mut out = [0i32; 64];
        idct8x8_to_pel(&coeffs, &mut out);
        for i in 0..64 {
            assert!(
                (out[i] - pels[i]).abs() <= 1,
                "pos {i}: got {} want {}",
                out[i],
                pels[i]
            );
        }
    }
}
