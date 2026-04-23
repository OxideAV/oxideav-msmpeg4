//! Inverse quantisation — H.263-style as used by MS-MPEG4v3.
//!
//! For the H.263-style path (the only path MS-MPEG4v3 uses), the
//! inverse quantisation is:
//!
//!   if level == 0:        coeff = 0
//!   else if quant is odd: coeff = quant * (2*|level| + 1) * sign(level)
//!   else:                 coeff = (quant * (2*|level| + 1) - 1) * sign(level)
//!
//! The intra DC coefficient uses a different scaling — see
//! [`dc_scaler`] — and is handled outside this dequantisation step.
//!
//! Final coefficients are clipped to the signed 12-bit range
//! [-2048, 2047] per the spec's 12-bit DCT-domain saturation.

use oxideav_core::{Error, Result};

/// H.263-style dequantisation for an AC-only block (used by inter MBs,
/// and by intra blocks after the DC is handled separately).
///
/// `level_start` is `0` for inter (all positions are bitstream levels)
/// and `1` for intra (skip the DC, which is reconstructed by the DC
/// predictor). `coeffs` is modified in place.
pub fn dequantise_h263(coeffs: &mut [i32; 64], quant: u32, level_start: usize) -> Result<()> {
    if !(1..=31).contains(&quant) {
        return Err(Error::invalid(format!(
            "msmpeg4v3: quant {quant} out of range 1..=31"
        )));
    }
    let q = quant as i32;
    let odd = (q & 1) != 0;
    for c in &mut coeffs[level_start..] {
        let lv = *c;
        if lv == 0 {
            continue;
        }
        let abs = lv.unsigned_abs() as i32;
        let mag = q * (2 * abs + 1);
        let out = if odd { mag } else { mag - 1 };
        *c = if lv < 0 { -out } else { out }.clamp(-2048, 2047);
    }
    Ok(())
}

/// Luma DC scaler by quantiser — MPEG-4 Part 2 Table 7-2, which
/// MS-MPEG4v3 reuses unchanged.
pub const Y_DC_SCALE_TABLE: [u8; 32] = [
    0, 8, 8, 8, 8, 10, 12, 14, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
    34, 36, 38, 40, 42, 44, 46,
];

/// Chroma DC scaler by quantiser — MPEG-4 Part 2 Table 7-3.
pub const C_DC_SCALE_TABLE: [u8; 32] = [
    0, 8, 8, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18,
    19, 20, 21, 22, 23, 24, 25,
];

/// Intra DC scaler — `block_idx` 0..=3 selects luma, 4..=5 selects
/// chroma. Panics if `quant > 31`.
pub fn dc_scaler(block_idx: usize, quant: u32) -> u32 {
    let q = quant as usize;
    if block_idx < 4 {
        Y_DC_SCALE_TABLE[q] as u32
    } else {
        C_DC_SCALE_TABLE[q] as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zeros_stay_zero() {
        let mut c = [0i32; 64];
        dequantise_h263(&mut c, 7, 0).unwrap();
        assert!(c.iter().all(|&v| v == 0));
    }

    #[test]
    fn positive_odd_quant() {
        let mut c = [0i32; 64];
        c[1] = 3;
        // q=5 (odd): (5*(2*3+1)) = 35.
        dequantise_h263(&mut c, 5, 1).unwrap();
        assert_eq!(c[1], 35);
    }

    #[test]
    fn negative_even_quant() {
        let mut c = [0i32; 64];
        c[5] = -2;
        // q=4 (even): 4*(2*2+1) - 1 = 19, negated -> -19.
        dequantise_h263(&mut c, 4, 0).unwrap();
        assert_eq!(c[5], -19);
    }

    #[test]
    fn intra_skips_dc() {
        let mut c = [0i32; 64];
        c[0] = 99; // DC, should remain untouched.
        c[1] = 1;
        dequantise_h263(&mut c, 7, 1).unwrap();
        assert_eq!(c[0], 99);
        assert_ne!(c[1], 1);
    }

    #[test]
    fn saturates_to_12_bits() {
        let mut c = [0i32; 64];
        c[1] = 200; // huge level
        dequantise_h263(&mut c, 31, 1).unwrap();
        assert_eq!(c[1], 2047); // saturated
        let mut c = [0i32; 64];
        c[1] = -200;
        dequantise_h263(&mut c, 31, 1).unwrap();
        assert_eq!(c[1], -2048);
    }

    #[test]
    fn quant_out_of_range_errors() {
        let mut c = [0i32; 64];
        assert!(dequantise_h263(&mut c, 0, 0).is_err());
        assert!(dequantise_h263(&mut c, 32, 0).is_err());
    }

    #[test]
    fn dc_scaler_luma_table_values() {
        assert_eq!(dc_scaler(0, 1), 8);
        assert_eq!(dc_scaler(0, 4), 8);
        assert_eq!(dc_scaler(0, 5), 10);
        assert_eq!(dc_scaler(0, 8), 16);
        assert_eq!(dc_scaler(0, 9), 17);
        assert_eq!(dc_scaler(0, 31), 46);
    }

    #[test]
    fn dc_scaler_chroma_table_values() {
        assert_eq!(dc_scaler(4, 1), 8);
        assert_eq!(dc_scaler(5, 4), 8);
        assert_eq!(dc_scaler(4, 5), 9);
        assert_eq!(dc_scaler(5, 31), 25);
    }

    #[test]
    fn dc_scaler_selects_by_block_idx() {
        for q in 1..=31u32 {
            assert_eq!(dc_scaler(0, q), dc_scaler(3, q));
            assert_eq!(dc_scaler(4, q), dc_scaler(5, q));
        }
    }
}
