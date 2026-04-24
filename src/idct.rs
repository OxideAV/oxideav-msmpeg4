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
