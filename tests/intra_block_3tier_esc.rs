//! End-to-end integration tests for the v1/v2/v3 intra block decoder
//! kernel `0x1c216d97` 3-tier ESC body, exercised through the public
//! [`decode_intra_block_full_v3`] boundary.
//!
//! The 3-tier ESC body itself is heavily unit-tested at the per-token
//! [`decode_token`] / [`decode_escape_body`] level inside `src/ac.rs`
//! (rounds 7 and 27). These integration tests cross-check that the
//! tiered escape behaves correctly when reached **through** the
//! production decode entry point — i.e. with the real intra-DC
//! direct-value VLC consuming the leading DC differential bits, the
//! G5 primary canonical-Huffman VLC consuming the AC walk, and the
//! H.263 dequantisation step running on the assembled coefficient
//! array.
//!
//! These tests exist to catch regressions that only manifest at the
//! `(DC VLC) + (AC walk) + (3-tier ESC body) + (dequant)` integration
//! boundary — for example, a bit-reader desync that the unit tests
//! cannot see because they call `decode_token` directly on a synthetic
//! ESC byte stream rather than chaining through `decode_intra_dc_diff_v3`
//! first.
//!
//! # Spec citations (clean-room, all docs/ only)
//!
//! - `docs/video/msmpeg4/spec/04-decoder-kernels.md` §2.3 (3-tier ESC
//!   body), §1.6 (intra scan dispatch), §2.6 (intra vs inter kernel
//!   differences table).
//! - `docs/video/msmpeg4/spec/07-remaining-opens.md` §5 (intra-DC
//!   direct-value 120-entry VLC, ESC sentinel = 119).
//! - `docs/video/msmpeg4/spec/11-walker-format-resolved.md` §3-§5
//!   (G5 packed-Huffman record layout, `(code, bit_length)` u32-LE
//!   pairs).
//! - `docs/video/msmpeg4/audit/01-report.md` §4.1 (G5 LMAX / RMAX
//!   audit cross-checked against MPEG-4 Part 2 Table 11-15
//!   ESCL(a) Intra TCOEF).

use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::ac::{AcVlcTable, Scan, Symbol};
use oxideav_msmpeg4::iq::{dc_scaler, dequantise_h263};
use oxideav_msmpeg4::mb::decode_intra_block_full_v3;
use oxideav_msmpeg4::scan::ZIGZAG;
use oxideav_msmpeg4::tables_data::{INTRA_DC_CHROMA_SEL0_RAW, INTRA_DC_LUMA_SEL0_RAW};

/// Bit-pack helper: append each `(value, width)` MSB-first, then pad
/// the tail with 8 zero bytes so the bit-reader peek does not starve.
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
        let shift = 8 - bits;
        out.push(((acc << shift) & 0xff) as u8);
    }
    out.extend_from_slice(&[0u8; 8]);
    out
}

/// Locate the (bit_length, code_value) for a given DC differential
/// magnitude in the luma sel=0 intra-DC VLC.  Returns `(bl, code)`.
fn dc_luma_sel0_code(magnitude: usize) -> (u32, u32) {
    INTRA_DC_LUMA_SEL0_RAW[magnitude]
}

/// Companion of [`dc_luma_sel0_code`] for the chroma sel=0 intra-DC
/// VLC — the v3 intra DC kernel `1c216cf8` routes block_idx ≥ 4
/// through this table per `spec/07-remaining-opens.md` §5.4 + spec/03
/// §3.2.
fn dc_chroma_sel0_code(magnitude: usize) -> (u32, u32) {
    INTRA_DC_CHROMA_SEL0_RAW[magnitude]
}

/// Locate the ESC entry in the G5 primary VLC. The G5 table has 102
/// alphabet entries + 1 ESC at index 102 (per spec/99 §5).
fn g5_esc_entry() -> (u32, u8) {
    let table = AcVlcTable::v3_intra_g5();
    let entry = table
        .entries
        .iter()
        .find(|e| matches!(e.value, Symbol::Escape))
        .expect("G5 ESC entry");
    (entry.code, entry.bits)
}

/// Find the G5 primary-VLC entry whose payload matches `(last, run,
/// |level|)`. Useful for synthesising bitstreams that re-fire the
/// primary VLC inside an ESC body (tier 1 / tier 2 walks). `level` is
/// the unsigned magnitude as stored in [`Symbol::RunLevel`] — the
/// sign bit is read separately by the decoder.
fn g5_entry_for(last: bool, run: u8, level: u16) -> (u32, u8) {
    let table = AcVlcTable::v3_intra_g5();
    let entry = table
        .entries
        .iter()
        .find(|e| {
            matches!(
                e.value,
                Symbol::RunLevel {
                    last: l,
                    run: r,
                    level: lv,
                } if l == last && r == run && lv == level
            )
        })
        .unwrap_or_else(|| {
            panic!("G5 entry not found for (last={last}, run={run}, level={level})")
        });
    (entry.code, entry.bits)
}

/// Find any sub-class-B (`last=true`) entry — used as a block
/// terminator at the end of every synthetic bitstream. Returns
/// `(code, bits, run, level_mag)` so callers can compute the expected
/// coefficient position. `level_mag` is the unsigned magnitude; the
/// signed level depends on the sign bit the caller emits after the
/// codeword.
fn g5_shortest_terminator() -> (u32, u8, u8, u16) {
    let table = AcVlcTable::v3_intra_g5();
    let entry = table
        .entries
        .iter()
        .filter(|e| matches!(e.value, Symbol::RunLevel { last: true, .. }))
        .min_by_key(|e| e.bits)
        .expect("G5 has at least one sub-class-B entry");
    if let Symbol::RunLevel { run, level, .. } = entry.value {
        (entry.code, entry.bits, run, level)
    } else {
        unreachable!()
    }
}

/// Dequantise a single AC coefficient at the given DCT-domain level
/// the way `dequantise_h263` would (so test expectations stay in
/// lock-step with the production dequant path without depending on
/// any one quantiser).
fn expected_dequantised_ac(level: i32, quant: u32) -> i32 {
    let mut coeffs = [0i32; 64];
    coeffs[1] = level;
    dequantise_h263(&mut coeffs, quant, 1).unwrap();
    coeffs[1]
}

// =====================================================================
// Test 1 — DC differential decode + immediate sub-class-B terminator.
// =====================================================================

/// The simplest end-to-end intra block: a single non-zero DC
/// differential, then a sub-class-B terminator. Verifies the DC VLC
/// composes correctly with the AC walk and that no ESC body is reached.
#[test]
fn intra_block_dc_diff_and_terminator() {
    let pred_dc = 1024i32;
    let dc_magnitude: usize = 3;
    let (dc_bl, dc_code) = dc_luma_sel0_code(dc_magnitude);
    // Round 420 standard sign convention: sign bit set ⇒ negative. We
    // want positive so sign = 0.
    let (term_code, term_bits, term_run, term_level) = g5_shortest_terminator();
    let bytes = pack(&[
        (dc_code, dc_bl),
        (0, 1), // DC sign bit = 0 ⇒ positive
        (term_code, term_bits as u32),
        (0, 1), // AC sign bit = 0 ⇒ POSITIVE per ac.rs decode_token (bit 0 = +, bit 1 = -)
    ]);
    let mut br = BitReader::new(&bytes);
    let quant: u32 = 5;
    let block = decode_intra_block_full_v3(
        &mut br,
        /*block_idx=*/ 0,
        pred_dc,
        quant,
        /*cbp_set=*/ true,
        Scan::Zigzag,
        &AcVlcTable::v3_intra_g5(),
        /*dc_size_sel=*/ 0,
    )
    .expect("intra block with DC + terminator decodes");
    let expected_dc = pred_dc + (dc_magnitude as i32) * dc_scaler(0, quant) as i32;
    assert_eq!(block.coeffs[0], expected_dc, "DC reconstructed");

    // Terminator: scan position advances from start_pos=1 by `run`
    // (intra walker). The terminator's level magnitude is dequantised
    // through `dequantise_h263` with the AC sign bit applied. Per
    // ac.rs `decode_token`, AC sign bit 0 = positive, bit 1 = negative.
    // We emit sign bit = 0 so the signed level is +term_level.
    let signed_level = term_level as i32;
    let pos = 1usize + term_run as usize;
    let expected_ac = expected_dequantised_ac(signed_level, quant);
    assert_eq!(
        block.coeffs[ZIGZAG[pos]], expected_ac,
        "terminator AC coefficient at zigzag[{pos}]"
    );
    assert_eq!(
        block.ac_nonzero, 1,
        "exactly one AC coefficient decoded (the terminator)"
    );
}

// =====================================================================
// Test 2 — DC differential is zero (no sign bit consumed).
// =====================================================================

/// Per spec/07 §5.2: `idx == 0` ⇒ DC differential = 0 with **no sign
/// bit consumed**. Verify that the AC walk lines up bit-for-bit when
/// the DC bit position differs from the magnitude-with-sign-bit case.
#[test]
fn intra_block_dc_zero_no_sign_bit() {
    let pred_dc = 1024i32;
    let (dc_bl, dc_code) = dc_luma_sel0_code(0);
    let (term_code, term_bits, term_run, _term_level) = g5_shortest_terminator();
    let bytes = pack(&[
        (dc_code, dc_bl),
        // NO sign bit here — `idx == 0` skips the sign read.
        (term_code, term_bits as u32),
        (1, 1), // AC sign bit for terminator (positive)
    ]);
    let mut br = BitReader::new(&bytes);
    let quant: u32 = 8;
    let block = decode_intra_block_full_v3(
        &mut br,
        0,
        pred_dc,
        quant,
        true,
        Scan::Zigzag,
        &AcVlcTable::v3_intra_g5(),
        0,
    )
    .expect("intra block with DC=0 decodes");
    assert_eq!(
        block.coeffs[0], pred_dc,
        "DC unchanged when differential = 0"
    );
    let pos = 1usize + term_run as usize;
    assert!(
        block.coeffs[ZIGZAG[pos]] != 0,
        "terminator AC at zigzag[{pos}] should be non-zero"
    );
}

// =====================================================================
// Test 3 — Tier 1 ESC body (level extension) reached after DC decode.
// =====================================================================

/// Round 7 wired the tier-1 escape body: after the primary VLC fires
/// ESC, the decoder re-fires the primary VLC; if it returns a normal
/// `(last, run, level_base)`, the emitted level is
/// `sign · (level_base + LMAX[last][run])`. Test this through the
/// public boundary with the leading DC VLC also consumed.
#[test]
fn intra_block_tier_1_esc_level_extension() {
    let pred_dc = 1024i32;
    let dc_magnitude: usize = 2;
    let (dc_bl, dc_code) = dc_luma_sel0_code(dc_magnitude);
    let (esc_code, esc_bits) = g5_esc_entry();
    // Tier-1 inner symbol = idx 0 (run=0, level=1, last=false). G5
    // LMAX[0][0] = 27 per audit/01 §4.1, so the actual level lands at
    // 1 + 27 = 28.
    let (inner_code, inner_bits) = g5_entry_for(false, 0, 1);
    // After tier-1 we need a sub-class-B terminator to end the block.
    let (term_code, term_bits, term_run, _) = g5_shortest_terminator();

    let bytes = pack(&[
        (dc_code, dc_bl),
        (0, 1), // DC sign (0 ⇒ positive, round 420)
        (esc_code, esc_bits as u32),
        (1, 1), // selector: level-extension tier (round 420)
        (inner_code, inner_bits as u32),
        (0, 1), // tier-1 sign bit (sign=0 ⇒ positive)
        (term_code, term_bits as u32),
        (1, 1), // terminator sign
    ]);
    let mut br = BitReader::new(&bytes);
    let quant: u32 = 5;
    let block = decode_intra_block_full_v3(
        &mut br,
        0,
        pred_dc,
        quant,
        true,
        Scan::Zigzag,
        &AcVlcTable::v3_intra_g5(),
        0,
    )
    .expect("intra block with tier-1 ESC decodes");

    // Tier-1 token writes at scan pos 1 + run(0) = 1, level = +28.
    let expected_tier1 = expected_dequantised_ac(28, quant);
    assert_eq!(
        block.coeffs[ZIGZAG[1]], expected_tier1,
        "tier-1 level-extension coefficient at zigzag[1]"
    );

    // Terminator at scan pos 1 + 1 + term_run.
    let term_pos = 2usize + term_run as usize;
    assert!(
        block.coeffs[ZIGZAG[term_pos]] != 0,
        "terminator at zigzag[{term_pos}]"
    );
    assert_eq!(block.ac_nonzero, 2, "tier-1 + terminator");
}

// =====================================================================
// Test 4 — Tier 2 ESC body (run extension) reached after DC decode.
// =====================================================================

/// Run-extension arm (spec/17 §3 ladder): ESC → selector-1 `0` →
/// selector-2 `1` → primary VLC → sign. The emitted run is `run_base + RMAX[last][|level|] + 1`. Use
/// inner = idx 0 (run=0, level=1, last=false): RMAX[0][1] = 14 per
/// audit/01 §4.1, so the emitted run is 0 + 14 + 1 = 15.
#[test]
fn intra_block_tier_2_esc_run_extension() {
    let pred_dc = 1024i32;
    let (dc_bl, dc_code) = dc_luma_sel0_code(0); // zero-DC fast path
    let (esc_code, esc_bits) = g5_esc_entry();
    let (inner_code, inner_bits) = g5_entry_for(false, 0, 1);
    let (term_code, term_bits, term_run, _term_level) = g5_shortest_terminator();

    let bytes = pack(&[
        (dc_code, dc_bl),
        // (no DC sign — diff is 0)
        (esc_code, esc_bits as u32),     // ESC marker
        (0, 1),                          // selector 1 = 0
        (1, 1),                          // selector 2 = 1 → run-extension arm
        (inner_code, inner_bits as u32), // re-VLC payload for tier 2
        (0, 1),                          // tier-2 sign bit (positive)
        (term_code, term_bits as u32),
        (1, 1),
    ]);
    let mut br = BitReader::new(&bytes);
    let quant: u32 = 5;
    let block = decode_intra_block_full_v3(
        &mut br,
        0,
        pred_dc,
        quant,
        true,
        Scan::Zigzag,
        &AcVlcTable::v3_intra_g5(),
        0,
    )
    .expect("intra block with tier-2 ESC decodes");

    // Tier-2 token: run = 15, level = 1. Position after start_pos=1 is
    // 1 + 15 = 16.
    let expected_pos = 16;
    let expected_level = expected_dequantised_ac(1, quant);
    assert_eq!(
        block.coeffs[ZIGZAG[expected_pos]], expected_level,
        "tier-2 run-extension coefficient at zigzag[{expected_pos}]"
    );
    // Terminator at scan pos expected_pos + 1 + term_run.
    let term_pos = expected_pos + 1 + term_run as usize;
    assert!(
        block.coeffs[ZIGZAG[term_pos]] != 0,
        "terminator at zigzag[{term_pos}]"
    );
    assert_eq!(
        block.ac_nonzero, 2,
        "tier-2 + terminator = 2 non-zero coefficients"
    );
}

// =====================================================================
// Test 5 — Tier 3 ESC body (verbatim FLC triple) reached after DC.
// =====================================================================

/// Verbatim arm (spec/17 §3 ladder): ESC → selector-1 `0` →
/// selector-2 `0` → 1+6+8-bit verbatim FLC triple. The terminator is
/// encoded inline (last=1 set inside the FLC), so no separate
/// terminator entry is needed.
#[test]
fn intra_block_tier_3_esc_verbatim() {
    let pred_dc = 1024i32;
    let (dc_bl, dc_code) = dc_luma_sel0_code(0);
    let (esc_code, esc_bits) = g5_esc_entry();

    let bytes = pack(&[
        (dc_code, dc_bl),
        (esc_code, esc_bits as u32),
        (0, 1), // selector 1 = 0
        (0, 1), // selector 2 = 0 → verbatim FLC
        // Verbatim triple: last=1, run=7, level=-5 (0xfb as signed 8-bit).
        (1, 1),
        (7, 6),
        (0xfb, 8),
    ]);
    let mut br = BitReader::new(&bytes);
    let quant: u32 = 5;
    let block = decode_intra_block_full_v3(
        &mut br,
        0,
        pred_dc,
        quant,
        true,
        Scan::Zigzag,
        &AcVlcTable::v3_intra_g5(),
        0,
    )
    .expect("intra block with tier-3 ESC decodes");

    // The tier-3 token lands at scan position 1 + 7 = 8 with the
    // signed level −5 dequantised.
    let expected_pos = 8;
    let expected_level = expected_dequantised_ac(-5, quant);
    assert_eq!(
        block.coeffs[ZIGZAG[expected_pos]], expected_level,
        "tier-3 verbatim coefficient at zigzag[{expected_pos}]"
    );
    assert_eq!(block.ac_nonzero, 1);
}

// =====================================================================
// Test 6 — CBP-not-set short-circuits the AC walk.
// =====================================================================

/// When the CBP bit for a block is unset, `decode_intra_block_full_v3`
/// must skip the AC walk entirely and leave AC coefficients zero — the
/// 3-tier ESC body should never even be reached. Verifies the
/// cbp-gating boundary so a future regression in the gating doesn't
/// silently start consuming AC bits.
#[test]
fn intra_block_cbp_zero_skips_ac_walk() {
    let pred_dc = 1024i32;
    let dc_magnitude: usize = 7;
    let (dc_bl, dc_code) = dc_luma_sel0_code(dc_magnitude);
    // Deliberately include AC-looking bytes after the DC bits to prove
    // they're not consumed.
    let bytes = pack(&[
        (dc_code, dc_bl),
        (0, 1), // DC sign (0 ⇒ positive, round 420)
        (0xff, 8),
        (0xff, 8),
    ]);
    let mut br = BitReader::new(&bytes);
    let dc_bits_consumed_expected = dc_bl + 1;
    let quant: u32 = 5;
    let block = decode_intra_block_full_v3(
        &mut br,
        0,
        pred_dc,
        quant,
        /*cbp_set=*/ false,
        Scan::Zigzag,
        &AcVlcTable::v3_intra_g5(),
        0,
    )
    .expect("intra block with cbp=0 decodes");

    assert_eq!(
        br.bit_position() as u32,
        dc_bits_consumed_expected,
        "cbp=0 must stop at end of DC bits — AC bits left untouched"
    );
    let expected_dc = pred_dc + (dc_magnitude as i32) * dc_scaler(0, quant) as i32;
    assert_eq!(block.coeffs[0], expected_dc, "DC reconstructed");
    assert!(
        block.coeffs[1..].iter().all(|&c| c == 0),
        "AC plane must be zero when cbp=0"
    );
    assert_eq!(block.ac_nonzero, 0);
}

// =====================================================================
// Test 7 — Chroma block routes through the chroma DC scaler.
// =====================================================================

/// Block idx 4 (Cb) / 5 (Cr) must use the chroma DC scaler table
/// (`C_DC_SCALE_TABLE`) at the same quant value where luma's table
/// differs — at quant=8 luma scales by 16, chroma by 10. This test
/// pins that routing through the public boundary.
#[test]
fn intra_block_chroma_uses_chroma_dc_scaler() {
    let pred_dc = 1024i32;
    let dc_magnitude: usize = 2;
    // Chroma block_idx ≥ 4 routes the intra-DC decode through the
    // chroma DC VLC (per `src/mb.rs::dc_table`), so we encode the
    // leading DC bits with the chroma table here.
    let (dc_bl, dc_code) = dc_chroma_sel0_code(dc_magnitude);
    let (term_code, term_bits, _term_run, _term_level) = g5_shortest_terminator();
    let bytes = pack(&[
        (dc_code, dc_bl),
        (0, 1), // DC sign (0 ⇒ positive, round 420)
        (term_code, term_bits as u32),
        (1, 1),
    ]);
    let mut br = BitReader::new(&bytes);
    let quant: u32 = 8;
    let block = decode_intra_block_full_v3(
        &mut br,
        /*block_idx=*/ 4, // Cb
        pred_dc,
        quant,
        true,
        Scan::Zigzag,
        // Chroma in v3 actually uses G4, but G5 here is fine for the
        // DC-scaler-routing test; only the DC scaler depends on
        // block_idx.
        &AcVlcTable::v3_intra_g5(),
        0,
    )
    .expect("chroma intra block decodes");
    let expected_dc = pred_dc + (dc_magnitude as i32) * dc_scaler(4, quant) as i32;
    assert_eq!(
        block.coeffs[0], expected_dc,
        "chroma DC scaler used for block_idx=4"
    );
    assert_ne!(
        dc_scaler(4, quant),
        dc_scaler(0, quant),
        "test premise: luma vs chroma scalers differ at quant=8"
    );
}

// =====================================================================
// Test 8 — DC ESC tier in the direct-value 120-entry VLC.
// =====================================================================

/// Per spec/07 §5.2 the intra-DC VLC fires its ESC sentinel at
/// `idx == 119`, after which the decoder reads 8 raw bits then a sign
/// bit. Verify the full intra-block decode handles the DC-ESC tier
/// correctly when followed by a normal AC walk.
#[test]
fn intra_block_dc_esc_tier_decodes() {
    let pred_dc = 1024i32;
    // idx 119 = ESC sentinel.
    let (esc_bl, esc_code) = dc_luma_sel0_code(119);
    let (term_code, term_bits, term_run, _) = g5_shortest_terminator();
    // ESC raw = 0x80, sign = 0 ⇒ positive 128 (round 420 standard
    // convention: sign bit set ⇒ negative).
    let bytes = pack(&[
        (esc_code, esc_bl),
        (0x80, 8),
        (0, 1), // sign bit clear ⇒ positive
        (term_code, term_bits as u32),
        (1, 1),
    ]);
    let mut br = BitReader::new(&bytes);
    let quant: u32 = 5;
    let block = decode_intra_block_full_v3(
        &mut br,
        0,
        pred_dc,
        quant,
        true,
        Scan::Zigzag,
        &AcVlcTable::v3_intra_g5(),
        0,
    )
    .expect("intra block with DC-ESC decodes");
    let expected_dc = pred_dc + 128i32 * dc_scaler(0, quant) as i32;
    assert_eq!(block.coeffs[0], expected_dc, "DC = pred + 128 * scaler");
    let pos = 1usize + term_run as usize;
    assert!(
        block.coeffs[ZIGZAG[pos]] != 0,
        "terminator at zigzag[{pos}]"
    );
}
