//! Inverse quantisation — H.263-style as used by MS-MPEG4 v1/v2/v3.
//!
//! Per `docs/video/msmpeg4/spec/08-descriptor-constants.md` §3.2 /
//! `docs/video/msmpeg4/spec/07-remaining-opens.md` §4, all three
//! integer inner kernels (v1 inter `0x1c215d2c`, v2/v3 inter `0x1c215e6f`
//! / `0x1c21611b`, and v1/v2/v3 intra `0x1c216d97`) use the **same**
//! per-MB scalar pair `[esi+0x13c]` (mag) / `[esi+0x140]` (bias),
//! computed from PQUANT exactly as H.263 §6.2.2.1 Eq. 12:
//!
//!   parity = PQUANT & 1
//!   mag    = 2 * PQUANT
//!   bias   = PQUANT - parity
//!
//! The dequantised coefficient is:
//!
//!   coeff = level * mag + bias   if level > 0
//!   coeff = level * mag - bias   if level < 0
//!   coeff = 0                    if level == 0
//!
//! This is equivalent to the older "sum of half-step" form:
//!
//!   coeff = sign(level) * (quant * (2|level| + 1) - (1 - parity))
//!
//! because `2 * QP * |l| + (QP - parity) = QP * (2|l| + 1) - parity` and
//! parity=0 for odd QP in the old form (a sign-convention flip). The
//! spec/08 form is what the disassembly actually does, so that's what
//! we implement.
//!
//! Final coefficients are clipped to the signed 12-bit range
//! [-2048, 2047] per the 12-bit DCT-domain saturation (MPEG-4 Part 2
//! §7.4.4 / H.263 §6.2.2.1 post-dequant clip).
//!
//! The intra DC coefficient uses a different scaling — see
//! [`dc_scaler`] — and is handled outside this dequantisation step.

use oxideav_core::{Error, Result};

/// H.263-style dequantisation for an AC-only block (used by inter MBs,
/// and by intra blocks after the DC is handled separately).
///
/// `level_start` is `0` for inter (all positions are bitstream levels)
/// and `1` for intra (skip the DC, which is reconstructed by the DC
/// predictor). `coeffs` is modified in place. Each coefficient is a
/// signed "level" as read from the bitstream (`pri_A[idx]` with its
/// sign bit applied) — this function scales it into the final DCT
/// domain.
pub fn dequantise_h263(coeffs: &mut [i32; 64], quant: u32, level_start: usize) -> Result<()> {
    if !(1..=31).contains(&quant) {
        return Err(Error::invalid(format!(
            "msmpeg4v3: quant {quant} out of range 1..=31"
        )));
    }
    let q = quant as i32;
    let parity = q & 1;
    let mag = 2 * q;
    let bias = q - parity;
    for c in &mut coeffs[level_start..] {
        let lv = *c;
        if lv == 0 {
            continue;
        }
        // spec/08 §3.2 / spec/07 §4.5: sign(level) * (|level| * mag + bias).
        let scaled = lv.unsigned_abs() as i32 * mag + bias;
        *c = if lv < 0 { -scaled } else { scaled }.clamp(-2048, 2047);
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
        // spec/08: mag = 2*q = 10, parity = 1, bias = q - parity = 4.
        //   coeff = 3 * 10 + 4 = 34.
        dequantise_h263(&mut c, 5, 1).unwrap();
        assert_eq!(c[1], 34);
    }

    #[test]
    fn negative_even_quant() {
        let mut c = [0i32; 64];
        c[5] = -2;
        // q=4 (even): mag = 8, parity = 0, bias = 4.
        //   |coeff| = 2 * 8 + 4 = 20; sign of level => -20.
        dequantise_h263(&mut c, 4, 0).unwrap();
        assert_eq!(c[5], -20);
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
