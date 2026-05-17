//! Integration tests for the G0..G3 extended-alphabet DCT AC TCOEF
//! pipeline wired in round 7 (2026-05-14).
//!
//! Per `docs/video/msmpeg4/spec/09-g0-g3-enumeration.md` the four
//! extended G-descriptors carry alphabets strictly larger than the
//! G4/G5 baseline (G0=168, G1=185, G2=148, G3=132 entries) and their
//! `(idx → (run, level, last))` enumeration is fully extracted into
//! `crates/oxideav-msmpeg4/tables/region_*_g{0..3}_enum.csv`. The
//! per-symbol canonical-Huffman bit-length array (the primary VLC
//! source) is still spec-OPEN per
//! `docs/video/msmpeg4/spec/99-current-understanding.md` §10 — the
//! candidate packed sources at file `0x57a30 / 0x57f80 / 0x58558 /
//! 0x58a08` are flagged `verdict: suspect`.
//!
//! These tests exercise:
//!
//! 1. **LMAX / RMAX wiring**: [`AcVlcTable::v3_intra_g{0..3}`] now
//!    carries `Some(lmax)` / `Some(rmax)` derived from the enumeration
//!    data. The per-(last, run) caps must match `spec/09` §8's
//!    consolidated table.
//! 2. **Synthetic-VLC round-trip**: [`AcVlcTable::v3_intra_g{0..3}_synthetic`]
//!    builds a fixed-length canonical-Huffman over the alphabet so the
//!    post-VLC pipeline can be exercised end-to-end before the
//!    bit-length array lands.
//! 3. **3-tier ESC body**: with `lmax` / `rmax` populated for G0..G3,
//!    the tier-1 (level extension) and tier-2 (run extension) paths
//!    chain correctly through [`decode_token`].

use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::ac::{decode_token, AcVlcTable, Symbol};
use oxideav_msmpeg4::g_descriptor::GSymbol;
use oxideav_msmpeg4::g_enum::GExtended;

/// Bit-pack helper: each `(value, width)` is appended MSB-first.
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

/// Compute the synthetic-VLC bit-width that
/// [`AcVlcTable::v3_intra_g0_synthetic`] (and siblings) uses:
/// `ceil(log2(count_A + 1))`.
fn synthetic_bl(count_a: usize) -> u32 {
    let mut bl: u32 = 0;
    while (1usize << bl) < count_a + 1 {
        bl += 1;
    }
    bl
}

/// Assert that `row[lo..=hi]` is uniformly `val`. `tag` is a short
/// human-readable identifier for the panic message ("G0 sub-A" etc).
fn assert_run_range_uniform(row: &[u8; 64], lo: usize, hi: usize, val: u8, tag: &str) {
    for (r, &v) in row.iter().enumerate().take(hi + 1).skip(lo) {
        assert_eq!(v, val, "{tag} r{r}");
    }
}

// =====================================================================
// §1 — LMAX / RMAX consistency vs spec/09 §8.
// =====================================================================

#[test]
fn g0_g1_g2_g3_carry_lmax_and_rmax_post_round_7() {
    // Round 7 promoted these from PLACEHOLDER to LMAX/RMAX-bearing
    // placeholders. The `entries` slice stays empty until the bit-length
    // extraction lands, but the ESC walk now has its extension offsets.
    for table in [
        AcVlcTable::v3_intra_g0(),
        AcVlcTable::v3_intra_g1(),
        AcVlcTable::v3_intra_g2(),
        AcVlcTable::v3_intra_g3(),
    ] {
        assert!(
            table.entries.is_empty(),
            "primary VLC entries still spec-OPEN (round 7 only wires LMAX/RMAX)"
        );
        assert!(table.lmax.is_some(), "lmax must be wired post round 7");
        assert!(table.rmax.is_some(), "rmax must be wired post round 7");
    }
}

#[test]
fn g0_lmax_matches_spec_09_section_8() {
    // spec/09 §8 last=0 (sub-A) LMAX vs run for G0:
    // r0=23, r1=11, r2=8, r3=7, r4=5, r5=5, r6=4, r7=4,
    // r8..11=3, r12..16=2, r17..26=1.
    let t = AcVlcTable::v3_intra_g0();
    let lmax = t.lmax.expect("g0 lmax");
    let last0 = &lmax[0];
    assert_eq!(last0[0], 23, "G0 sub-A r0");
    assert_eq!(last0[1], 11, "G0 sub-A r1");
    assert_eq!(last0[2], 8, "G0 sub-A r2");
    assert_eq!(last0[3], 7, "G0 sub-A r3");
    assert_eq!(last0[4], 5, "G0 sub-A r4");
    assert_eq!(last0[5], 5, "G0 sub-A r5");
    assert_eq!(last0[6], 4, "G0 sub-A r6");
    assert_eq!(last0[7], 4, "G0 sub-A r7");
    assert_run_range_uniform(last0, 8, 11, 3, "G0 sub-A");
    assert_run_range_uniform(last0, 12, 16, 2, "G0 sub-A");
    assert_run_range_uniform(last0, 17, 26, 1, "G0 sub-A");
    // G0 sub-A caps at r=26 — no entries at r >= 27.
    assert_eq!(last0[27], 0, "G0 sub-A r27 must be empty");

    // spec/09 §8 last=1 (sub-B) LMAX vs run for G0:
    // r0=9, r1=5, r2-3=4, r4-6=3, r7-14=2, r15-36=1.
    let last1 = &lmax[1];
    assert_eq!(last1[0], 9, "G0 sub-B r0");
    assert_eq!(last1[1], 5, "G0 sub-B r1");
    assert_run_range_uniform(last1, 2, 3, 4, "G0 sub-B");
    assert_run_range_uniform(last1, 4, 6, 3, "G0 sub-B");
    assert_run_range_uniform(last1, 7, 14, 2, "G0 sub-B");
    assert_run_range_uniform(last1, 15, 36, 1, "G0 sub-B");
    assert_eq!(last1[37], 0, "G0 sub-B r37 must be empty");
}

#[test]
fn g1_lmax_matches_spec_09_section_8() {
    // spec/09 §8 last=0 for G1: r0=19, r1=15, r2=12, r3=11, r4=6,
    // r5=5, r6..9=4, r10..15=3, r16..17=2, r18..30=1.
    let t = AcVlcTable::v3_intra_g1();
    let lmax = t.lmax.expect("g1 lmax");
    let last0 = &lmax[0];
    assert_eq!(last0[0], 19, "G1 sub-A r0");
    assert_eq!(last0[1], 15, "G1 sub-A r1");
    assert_eq!(last0[2], 12, "G1 sub-A r2");
    assert_eq!(last0[3], 11, "G1 sub-A r3");
    assert_eq!(last0[4], 6, "G1 sub-A r4");
    assert_eq!(last0[5], 5, "G1 sub-A r5");
    assert_run_range_uniform(last0, 6, 9, 4, "G1 sub-A");
    assert_run_range_uniform(last0, 10, 15, 3, "G1 sub-A");
    assert_run_range_uniform(last0, 16, 17, 2, "G1 sub-A");
    assert_run_range_uniform(last0, 18, 30, 1, "G1 sub-A");
    assert_eq!(last0[31], 0, "G1 sub-A r31 must be empty");

    // sub-B: r0=6, r1=5, r2-3=4, r4=3, r5..15=2, r16..37=1.
    let last1 = &lmax[1];
    assert_eq!(last1[0], 6, "G1 sub-B r0");
    assert_eq!(last1[1], 5, "G1 sub-B r1");
    assert_run_range_uniform(last1, 2, 3, 4, "G1 sub-B");
    assert_eq!(last1[4], 3, "G1 sub-B r4");
    assert_run_range_uniform(last1, 5, 15, 2, "G1 sub-B");
    assert_run_range_uniform(last1, 16, 37, 1, "G1 sub-B");
}

#[test]
fn g2_lmax_matches_spec_09_section_8() {
    // spec/09 §8 G2 sub-A: r0=14, r1=9, r2=5, r3-5=4, r6-12=3, r13-15=2,
    // r16-29=1.
    let t = AcVlcTable::v3_intra_g2();
    let lmax = t.lmax.expect("g2 lmax");
    let last0 = &lmax[0];
    assert_eq!(last0[0], 14, "G2 sub-A r0");
    assert_eq!(last0[1], 9, "G2 sub-A r1");
    assert_eq!(last0[2], 5, "G2 sub-A r2");
    assert_run_range_uniform(last0, 3, 5, 4, "G2 sub-A");
    assert_run_range_uniform(last0, 6, 12, 3, "G2 sub-A");
    assert_run_range_uniform(last0, 13, 15, 2, "G2 sub-A");
    assert_run_range_uniform(last0, 16, 29, 1, "G2 sub-A");

    // sub-B: r0=5, r1=4, r2-3=3, r4-15=2, r16-43=1.
    let last1 = &lmax[1];
    assert_eq!(last1[0], 5, "G2 sub-B r0");
    assert_eq!(last1[1], 4, "G2 sub-B r1");
    assert_run_range_uniform(last1, 2, 3, 3, "G2 sub-B");
    assert_run_range_uniform(last1, 4, 15, 2, "G2 sub-B");
    assert_run_range_uniform(last1, 16, 43, 1, "G2 sub-B");
    // spec/09 §5 / §8 highlight: G2 sub-B has the longest level-1 tail
    // of any G-descriptor (r ≤ 43). Confirm boundary.
    assert_eq!(last1[44], 0, "G2 sub-B r44 must be empty");
}

#[test]
fn g3_lmax_matches_spec_09_section_8() {
    // spec/09 §8 G3 sub-A: r0=16, r1=11, r2=8, r3=7, r4=5, r5-6=4,
    // r7-13=3, r14-15=2, r16-20=1.
    let t = AcVlcTable::v3_intra_g3();
    let lmax = t.lmax.expect("g3 lmax");
    let last0 = &lmax[0];
    assert_eq!(last0[0], 16, "G3 sub-A r0");
    assert_eq!(last0[1], 11, "G3 sub-A r1");
    assert_eq!(last0[2], 8, "G3 sub-A r2");
    assert_eq!(last0[3], 7, "G3 sub-A r3");
    assert_eq!(last0[4], 5, "G3 sub-A r4");
    assert_run_range_uniform(last0, 5, 6, 4, "G3 sub-A");
    assert_run_range_uniform(last0, 7, 13, 3, "G3 sub-A");
    assert_run_range_uniform(last0, 14, 15, 2, "G3 sub-A");
    assert_run_range_uniform(last0, 16, 20, 1, "G3 sub-A");
    // spec/09 §6 calls out G3 sub-A caps at r=20 — the smallest of the
    // four extended descriptors.
    assert_eq!(last0[21], 0, "G3 sub-A r21 must be empty");

    // sub-B: r0-1=4, r2-3=3, r4-13=2, r14-26=1.
    let last1 = &lmax[1];
    assert_run_range_uniform(last1, 0, 1, 4, "G3 sub-B");
    assert_run_range_uniform(last1, 2, 3, 3, "G3 sub-B");
    assert_run_range_uniform(last1, 4, 13, 2, "G3 sub-B");
    assert_run_range_uniform(last1, 14, 26, 1, "G3 sub-B");
    assert_eq!(last1[27], 0, "G3 sub-B r27 must be empty");
}

#[test]
fn g0_g1_g2_g3_rmax_consistent_with_alphabet() {
    // RMAX[last][|level|] must be > 0 for every observed (last, level)
    // pair in the alphabet, and the max-run claim per spec/09 §8 must be
    // reflected at level=1 (the longest tail).
    for (g, exp_max_run_sub_b_l1) in [
        (GExtended::G0, 36u8),
        (GExtended::G1, 37u8),
        (GExtended::G2, 43u8),
        (GExtended::G3, 26u8),
    ] {
        let table = match g {
            GExtended::G0 => AcVlcTable::v3_intra_g0(),
            GExtended::G1 => AcVlcTable::v3_intra_g1(),
            GExtended::G2 => AcVlcTable::v3_intra_g2(),
            GExtended::G3 => AcVlcTable::v3_intra_g3(),
        };
        let rmax = table.rmax.expect("rmax");
        assert_eq!(
            rmax[1][1], exp_max_run_sub_b_l1,
            "{g:?} RMAX[last=1][level=1] should be the sub-B level-1 tail max"
        );
        // RMAX[0][1] should also be > 0 (every G has at least one r1 sub-A).
        assert!(rmax[0][1] > 0, "{g:?} RMAX[last=0][level=1] > 0");
    }
}

// =====================================================================
// §2 — Synthetic-VLC round-trip per G alphabet.
// =====================================================================

fn round_trip_synthetic(g: GExtended) {
    let count_a = g.count_a();
    let bl = synthetic_bl(count_a);
    let table = match g {
        GExtended::G0 => AcVlcTable::v3_intra_g0_synthetic(),
        GExtended::G1 => AcVlcTable::v3_intra_g1_synthetic(),
        GExtended::G2 => AcVlcTable::v3_intra_g2_synthetic(),
        GExtended::G3 => AcVlcTable::v3_intra_g3_synthetic(),
    };
    assert_eq!(
        table.entries.len(),
        count_a + 1,
        "{g:?} synthetic entries = count_A + 1 (ESC)"
    );

    // Decode every non-ESC entry; check (last, run, level) reproduces the
    // enumeration.
    for idx in 0..count_a {
        let bytes = pack(&[(idx as u32, bl), (0, 1)]); // VLC bits + sign=0 (positive)
        let mut br = BitReader::new(&bytes);
        let tok = decode_token(&mut br, &table).unwrap_or_else(|e| {
            panic!("{g:?} idx {idx}: decode_token failed: {e}");
        });
        let GSymbol::Token(expected) = g.decode(idx).unwrap() else {
            unreachable!("non-ESC idx in range")
        };
        assert_eq!(tok.last, expected.last, "{g:?} idx {idx} last");
        assert_eq!(tok.run, expected.run, "{g:?} idx {idx} run");
        assert_eq!(
            tok.level, expected.level_mag as i16,
            "{g:?} idx {idx} level"
        );
    }
}

#[test]
fn g0_synthetic_round_trip_every_symbol() {
    round_trip_synthetic(GExtended::G0);
}

#[test]
fn g1_synthetic_round_trip_every_symbol() {
    round_trip_synthetic(GExtended::G1);
}

#[test]
fn g2_synthetic_round_trip_every_symbol() {
    round_trip_synthetic(GExtended::G2);
}

#[test]
fn g3_synthetic_round_trip_every_symbol() {
    round_trip_synthetic(GExtended::G3);
}

// =====================================================================
// §3 — ESC tier walking with G0..G3 LMAX/RMAX.
// =====================================================================

#[test]
fn g0_synthetic_esc_tier1_level_extension() {
    // ESC at idx == count_A. Tier 1 re-fires the primary VLC; the next
    // symbol's (last, run, level_base) is promoted to
    // level_actual = level_base + LMAX[last][run].
    let g = GExtended::G0;
    let count_a = g.count_a();
    let bl = synthetic_bl(count_a);
    let table = AcVlcTable::v3_intra_g0_synthetic();

    // Pick a (last=0, run=0, level=1) base — that's idx 0 in the
    // enumeration. After tier-1 promotion the level becomes
    // 1 + LMAX[0][0] = 1 + 23 = 24.
    let bytes = pack(&[
        (count_a as u32, bl), // ESC trigger
        (0u32, bl),           // base symbol (idx 0: last=0, run=0, level=1)
        (0, 1),               // sign = positive
    ]);
    let mut br = BitReader::new(&bytes);
    let tok = decode_token(&mut br, &table).expect("tier-1 ESC decode");
    assert!(!tok.last);
    assert_eq!(tok.run, 0);
    assert_eq!(tok.level, 24, "1 + LMAX[0][0] = 1 + 23 = 24");
}

#[test]
fn g1_synthetic_esc_tier2_run_extension() {
    // Tier 1 ESC then tier 1 ESC again ⇒ tier 2: the next decoded
    // (last, run_base, |level|) becomes (last, run_base + RMAX[last][level] + 1, level).
    let g = GExtended::G1;
    let count_a = g.count_a();
    let bl = synthetic_bl(count_a);
    let table = AcVlcTable::v3_intra_g1_synthetic();

    // Pick base idx 0 = (last=0, run=0, level=1). After tier-1 ESC,
    // tier-2 promotion gives run_actual = 0 + RMAX[0][1] + 1.
    let rmax_0_1 = table.rmax.unwrap()[0][1];
    let expected_run = rmax_0_1 + 1;
    let bytes = pack(&[
        (count_a as u32, bl), // first ESC (tier 1)
        (count_a as u32, bl), // second ESC (tier 2)
        (0u32, bl),           // base symbol (idx 0)
        (0, 1),               // sign = positive
    ]);
    let mut br = BitReader::new(&bytes);
    let tok = decode_token(&mut br, &table).expect("tier-2 ESC decode");
    assert!(!tok.last);
    assert_eq!(tok.run, expected_run, "tier-2 run = run_base + RMAX + 1");
    assert_eq!(tok.level, 1);
}

#[test]
fn g2_synthetic_esc_tier3_verbatim_flc() {
    // Triple ESC ⇒ tier 3: verbatim 1+6+8 FLC triple (per spec/04 §2.3 +
    // MPEG-4 Part 2 §7.4.1.3 fallback).
    let g = GExtended::G2;
    let count_a = g.count_a();
    let bl = synthetic_bl(count_a);
    let table = AcVlcTable::v3_intra_g2_synthetic();
    let bytes = pack(&[
        (count_a as u32, bl), // ESC tier 1
        (count_a as u32, bl), // ESC tier 2
        (count_a as u32, bl), // ESC tier 3
        (1, 1),               // last = 1
        (5, 6),               // run = 5
        // signed 8-bit level = -10. read_i32 sign-extends, but the bit
        // pattern for -10 in 8-bit two's-complement is 0xf6.
        (0xf6, 8),
    ]);
    let mut br = BitReader::new(&bytes);
    let tok = decode_token(&mut br, &table).expect("tier-3 verbatim decode");
    assert!(tok.last);
    assert_eq!(tok.run, 5);
    assert_eq!(tok.level, -10);
}

#[test]
fn g3_synthetic_picks_largest_idx_correctly() {
    // The synthetic VLC width is sized to fit count_A + 1, so the largest
    // non-ESC idx (count_A - 1) should still decode. Exercises the upper
    // boundary of the alphabet — useful regression for the canonical
    // partition (idx > count_B ⇒ sub-B).
    let g = GExtended::G3;
    let count_a = g.count_a();
    let count_b = g.count_b();
    let bl = synthetic_bl(count_a);
    let table = AcVlcTable::v3_intra_g3_synthetic();
    let last_idx = count_a - 1;
    assert!(last_idx > count_b, "last idx must land in sub-B");
    let bytes = pack(&[(last_idx as u32, bl), (1, 1)]); // sign = negative
    let mut br = BitReader::new(&bytes);
    let tok = decode_token(&mut br, &table).expect("largest idx decode");
    let GSymbol::Token(expected) = g.decode(last_idx).unwrap() else {
        unreachable!()
    };
    assert!(expected.last, "expected sub-B");
    assert!(tok.last);
    assert_eq!(tok.run, expected.run);
    assert_eq!(
        tok.level,
        -(expected.level_mag as i16),
        "negative sign applied"
    );
}

// =====================================================================
// §4 — Alphabet shape cross-checks (spec/15 §7 / spec/09 §1).
// =====================================================================

#[test]
fn alphabet_sizes_match_spec_15_section_7() {
    // spec/15 §7 G-base table (count_A / count_B) — values derived
    // directly from the binary's mb_mv_struct_init at 0x1c210643.
    assert_eq!(GExtended::G0.count_a(), 168);
    assert_eq!(GExtended::G0.count_b(), 98);
    assert_eq!(GExtended::G1.count_a(), 185);
    assert_eq!(GExtended::G1.count_b(), 118);
    assert_eq!(GExtended::G2.count_a(), 148);
    assert_eq!(GExtended::G2.count_b(), 80);
    assert_eq!(GExtended::G3.count_a(), 132);
    assert_eq!(GExtended::G3.count_b(), 84);
}

#[test]
fn synthetic_table_esc_lives_at_count_a() {
    // The synthetic canonical-Huffman fits count_A + 1 codes; the last
    // is ESC. Round 7 invariant.
    for g in [GExtended::G0, GExtended::G1, GExtended::G2, GExtended::G3] {
        let table = match g {
            GExtended::G0 => AcVlcTable::v3_intra_g0_synthetic(),
            GExtended::G1 => AcVlcTable::v3_intra_g1_synthetic(),
            GExtended::G2 => AcVlcTable::v3_intra_g2_synthetic(),
            GExtended::G3 => AcVlcTable::v3_intra_g3_synthetic(),
        };
        // The ESC entry is at code == count_A.
        let escs: Vec<_> = table
            .entries
            .iter()
            .filter(|e| matches!(e.value, Symbol::Escape))
            .collect();
        assert_eq!(escs.len(), 1, "{g:?} exactly one ESC entry");
        assert_eq!(
            escs[0].code,
            g.count_a() as u32,
            "{g:?} ESC code equals count_A"
        );
    }
}
