//! 8×8 inverse DCT (float reference).
//!
//! The DCT transform itself is defined identically across MPEG-1, 2,
//! H.263, MPEG-4 Part 2 and MS-MPEG4 — the mathematical formula
//!
//!   x[n] = 0.5 * Σ_k C_k * X[k] * cos((2n+1) k π / 16)
//!
//! with `C_0 = 1/√2`, `C_{>0} = 1`, applied first row-wise then
//! column-wise. This is a pure mathematical primitive — no codec
//! specifics — so there is no attribution burden.

use std::f32::consts::PI;
use std::sync::OnceLock;

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

/// Apply IDCT to an `i32` coefficient block, returning clipped signed
/// pel values in `[-256, 255]`. Input must be in DCT domain
/// (post-dequantisation).
pub fn idct8x8_to_pel(coeffs: &[i32; 64], out: &mut [i32; 64]) {
    let mut f = [0.0f32; 64];
    for i in 0..64 {
        f[i] = coeffs[i] as f32;
    }
    idct8x8(&mut f);
    for i in 0..64 {
        out[i] = (f[i].round() as i32).clamp(-256, 255);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn idct_to_pel_clips() {
        let mut c = [0i32; 64];
        c[0] = 10000; // absurd DC
        let mut out = [0i32; 64];
        idct8x8_to_pel(&c, &mut out);
        for &v in &out {
            assert!((-256..=255).contains(&v));
        }
    }
}
