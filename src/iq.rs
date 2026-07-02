//! Inverse quantisation — H.263-style as used by MS-MPEG4 v1/v2/v3.
//!
//! Per `docs/video/msmpeg4/spec/08-descriptor-constants.md` §5 (the
//! sandbox-verified runtime materialisation), all three integer inner
//! kernels (v1 inter `0x1c215d2c`, v2/v3 inter `0x1c215e6f` /
//! `0x1c21611b`, and v1/v2/v3 intra `0x1c216d97`) use the **same**
//! per-frame scalar pair `[ctx+0x13c]` (mag) / `[ctx+0x140]` (bias),
//! materialised once per frame from PQUANT at `0x1c2125b2..0x1c2125d0`:
//!
//!   even_flag = 1 if PQUANT is even else 0     ; [ctx+0x138]
//!   mag       = 2 * PQUANT                      ; [ctx+0x13c]
//!   bias      = PQUANT - even_flag              ; [ctx+0x140]
//!
//! i.e. `bias = PQUANT - 1` for **even** PQUANT and `bias = PQUANT` for
//! **odd** PQUANT. The dequantised coefficient is:
//!
//!   coeff = level * mag + bias   if level > 0
//!   coeff = level * mag - bias   if level < 0
//!   coeff = 0                    if level == 0
//!
//! This is the H.263 §6.2.2.1 Eq. 12 quantiser-parity rule:
//! `|REC| = QUANT·(2·|LEVEL| + 1) - (QUANT even ? 1 : 0)`, which expands
//! to `mag·|LEVEL| + (PQUANT - even_flag)`.
//!
//! ## Reconciliation of spec/07 §4.2 vs spec/08 §5
//!
//! `spec/07` §4.2 quoted the per-MB store at `0x1c2132c6` and annotated
//! the parity idiom `neg edx; sbb edx,edx; inc edx` (operating on
//! `edx = PQUANT % 2`) as "`edx = 1 if odd else 0`", concluding
//! `bias = PQUANT` for even PQUANT. That **prose annotation is wrong
//! about its own quoted assembly**: with `edx ∈ {0,1}`, `neg edx` sets
//! CF iff the operand was non-zero, `sbb edx,edx` then yields `-CF`, and
//! `inc edx` yields `1 - CF` = **1 if even (CF=0), 0 if odd (CF=1)**.
//! That matches `spec/08` §5 (sandbox audit `audit/06`, hand-patched
//! PQUANT trials watched at `[ctx+0x138]`), which states the flag is
//! "1 if PQUANT even else 0". `spec/08` §5 is the authoritative,
//! runtime-verified form and is what we implement here.
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
    // spec/08 §5: even_flag = 1 if PQUANT is even, else 0. The runtime
    // computes this via the `neg/sbb/inc` idiom at `0x1c2132c6` (see the
    // module-level reconciliation note); `1 - (q & 1)` is the same value.
    let even_flag = 1 - (q & 1);
    let mag = 2 * q;
    let bias = q - even_flag;
    for c in &mut coeffs[level_start..] {
        let lv = *c;
        if lv == 0 {
            continue;
        }
        // spec/08 §5: sign(level) * (|level| * mag + bias).
        let scaled = lv.unsigned_abs() as i32 * mag + bias;
        *c = if lv < 0 { -scaled } else { scaled }.clamp(-2048, 2047);
    }
    Ok(())
}

/// Encoder-side H.263 quantisation — the inverse of
/// [`dequantise_h263`].
///
/// Maps one float DCT-domain coefficient to the bitstream "level" the
/// AC walk encodes. The reconstruction rule is spec/08 §5
/// (`sign · (|level| · 2q + q − even_flag)`), i.e. the decoder's
/// reconstruction points sit at `2q·L + bias` with
/// `bias = q − even_flag`, so the encoder inverts that affine map:
/// `|level| = round((|coeff| − bias) / (2q))` clamped at 0. For any
/// non-zero chosen level the reconstruction lands within `q` of the
/// target; coefficients that quantise to level 0 sit in the implicit
/// dead zone below the first reconstruction point (`2q + bias`), where
/// the error can reach `~2q` — the standard H.263-style dead-zone
/// behaviour. Which level an encoder emits is NOT a conformance
/// surface (any level decodes); this choice just minimises the
/// per-coefficient reconstruction error outside the dead zone.
///
/// Returns a level clamped to `[-127, 127]` so the verbatim ESC tier
/// (8-bit signed level field, spec/04 §1.3 step 10 / §2.3 tier 3) can
/// always represent it.
pub fn quantise_h263(coeff: f32, quant: u32) -> i32 {
    let q = quant as f32;
    let even_flag = 1.0 - (quant % 2) as f32;
    let bias = q - even_flag;
    let level = ((coeff.abs() - bias) / (2.0 * q)).round().max(0.0) as i32;
    let level = level.min(127);
    if coeff < 0.0 {
        -level
    } else {
        level
    }
}

/// Quantise a whole 8×8 float DCT block into bitstream levels,
/// starting at `level_start` (0 for inter — DC is a coded coefficient
/// — and 1 for intra where the DC goes through the DC-scaler path
/// instead). Positions before `level_start` are left untouched.
///
/// Returns the number of non-zero levels produced at or after
/// `level_start`.
pub fn quantise_block_h263(
    coeffs: &[f32; 64],
    quant: u32,
    level_start: usize,
    out: &mut [i32; 64],
) -> u32 {
    let mut nz = 0u32;
    for i in level_start..64 {
        let lv = quantise_h263(coeffs[i], quant);
        out[i] = lv;
        if lv != 0 {
            nz += 1;
        }
    }
    nz
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
        // spec/08 §5: q=5 odd → even_flag = 0, mag = 2*q = 10,
        //   bias = q - even_flag = 5.  coeff = 3 * 10 + 5 = 35.
        dequantise_h263(&mut c, 5, 1).unwrap();
        assert_eq!(c[1], 35);
    }

    #[test]
    fn negative_even_quant() {
        let mut c = [0i32; 64];
        c[5] = -2;
        // spec/08 §5: q=4 even → even_flag = 1, mag = 8, bias = q - 1 = 3.
        //   |coeff| = 2 * 8 + 3 = 19; sign of level => -19.
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
    fn parity_bias_direction_matches_spec_08_section_5() {
        // spec/08 §5 (sandbox-verified): bias = PQUANT - even_flag where
        // even_flag = 1 iff PQUANT is even. So bias = PQUANT - 1 for EVEN
        // PQUANT and bias = PQUANT for ODD PQUANT. This is the opposite of
        // the pre-fix direction (which subtracted 1 for odd). Pin both
        // arms with |level| = 1 so coeff = mag + bias = 2*PQUANT + bias.
        for q in 1..=31u32 {
            let mut c = [0i32; 64];
            c[1] = 1;
            dequantise_h263(&mut c, q, 1).unwrap();
            let qi = q as i32;
            let even_flag = if q % 2 == 0 { 1 } else { 0 };
            let expected = (2 * qi + (qi - even_flag)).clamp(-2048, 2047);
            assert_eq!(
                c[1], expected,
                "q={q}: bias direction must be PQUANT - even_flag"
            );
        }
        // Concrete spot checks of the corrected direction.
        let mut even = [0i32; 64];
        even[1] = 1;
        dequantise_h263(&mut even, 4, 1).unwrap();
        assert_eq!(even[1], 2 * 4 + (4 - 1), "q=4 even → bias=3, coeff=11");
        let mut odd = [0i32; 64];
        odd[1] = 1;
        dequantise_h263(&mut odd, 5, 1).unwrap();
        assert_eq!(odd[1], 2 * 5 + 5, "q=5 odd → bias=5, coeff=15");
    }

    #[test]
    fn quant_out_of_range_errors() {
        let mut c = [0i32; 64];
        assert!(dequantise_h263(&mut c, 0, 0).is_err());
        assert!(dequantise_h263(&mut c, 32, 0).is_err());
    }

    #[test]
    fn quantise_then_dequantise_stays_within_q() {
        // For every quant and a sweep of DCT-domain magnitudes, the
        // encoder-side level must reconstruct to within q of the target
        // (the nearest-reconstruction property), except where the
        // ±127-level clamp saturates.
        for q in 1..=31u32 {
            for mag in [0i32, 1, 5, 17, 60, 200, 900, 2000] {
                for sign in [1i32, -1] {
                    let coeff = (mag * sign) as f32;
                    let lv = quantise_h263(coeff, q);
                    if lv.unsigned_abs() == 127 {
                        continue; // saturated — no distance guarantee
                    }
                    let mut c = [0i32; 64];
                    c[1] = lv;
                    dequantise_h263(&mut c, q, 1).unwrap();
                    let recon = if lv == 0 { 0 } else { c[1] };
                    // Non-zero levels reconstruct within q of the
                    // target; level 0 sits in the dead zone below the
                    // first reconstruction point (error < 2q).
                    let bound = if lv == 0 { 2 * q } else { q + 1 };
                    assert!(
                        (recon - mag * sign).unsigned_abs() <= bound,
                        "q={q} coeff={coeff}: level={lv} recon={recon}"
                    );
                }
            }
        }
    }

    #[test]
    fn quantise_zero_is_zero_and_sign_is_preserved() {
        assert_eq!(quantise_h263(0.0, 8), 0);
        assert!(quantise_h263(100.0, 4) > 0);
        assert!(quantise_h263(-100.0, 4) < 0);
        // Clamps to the verbatim-ESC-representable ±127.
        assert_eq!(quantise_h263(10_000.0, 1), 127);
        assert_eq!(quantise_h263(-10_000.0, 1), -127);
    }

    #[test]
    fn quantise_block_respects_level_start() {
        let mut coeffs = [0.0f32; 64];
        coeffs[0] = 500.0;
        coeffs[1] = 100.0;
        coeffs[10] = -60.0;
        let mut out = [0i32; 64];
        out[0] = 77; // must survive level_start = 1
        let nz = quantise_block_h263(&coeffs, 4, 1, &mut out);
        assert_eq!(out[0], 77);
        assert_eq!(nz, 2);
        assert!(out[1] > 0 && out[10] < 0);
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
