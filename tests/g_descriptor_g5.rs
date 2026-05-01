//! Integration tests for the round-19 G5 descriptor wiring (now full).
//!
//! Round 18 wired G5's pri_A but left pri_B in a 408-byte gap between
//! `region_0569c0`'s end at file `0x57898` and `region_057a30`'s start.
//! Round 19 extracts that gap into `tables/region_057898.hex` (102 ×
//! u32-LE records) and `build.rs` slices the low byte of each record
//! into `G5_PRI_B`. These tests exercise the public `g_descriptor` API
//! end-to-end against the audit/01 §4.1 enumeration of G5 (102 entries
//! plus ESC, MSMPEG4 intra-luma DCT TCOEF). They mirror the round-18
//! G4 integration tests in `g_descriptor_g4.rs`.

use oxideav_msmpeg4::g_descriptor::{g5_decode, g5_iter, GSymbol};
use oxideav_msmpeg4::tables_data::{G5_COUNT_A, G5_COUNT_B, G5_PRI_A, G5_PRI_B};

#[test]
fn g5_full_alphabet_dense_round_trip() {
    // Walk every idx in [0, 102]. Every non-ESC entry must produce
    // a (last, run, level) triple consistent with the partition test.
    let mut sub_a_count = 0;
    let mut sub_b_count = 0;
    let mut esc_count = 0;
    for (idx, sym) in g5_iter() {
        match sym {
            GSymbol::Esc => {
                assert_eq!(idx, G5_COUNT_A);
                esc_count += 1;
            }
            GSymbol::Token(t) => {
                if t.last {
                    assert!(idx > G5_COUNT_B, "idx {idx} marked last but in sub-A");
                    sub_b_count += 1;
                } else {
                    assert!(idx <= G5_COUNT_B, "idx {idx} marked !last but in sub-B");
                    sub_a_count += 1;
                }
                // level_mag must be in [1, 27] (G5 LMAX(0)=27).
                assert!(
                    (1..=27).contains(&t.level_mag),
                    "idx {idx} level_mag {} out of [1, 27]",
                    t.level_mag,
                );
                // run must be in [0, 20] (G5 max run = 20 on sub-B last).
                assert!(t.run <= 20, "idx {idx} run {} > 20", t.run);
            }
        }
    }
    assert_eq!(sub_a_count, 67);
    assert_eq!(sub_b_count, 35);
    assert_eq!(esc_count, 1);
}

#[test]
fn g5_pri_a_pri_b_match_decoder_output() {
    // The decoder reads `pri_A[idx]` directly as level_mag and
    // `pri_B[idx]` as run for non-ESC indices. This invariant must
    // hold for every idx in [0, count_A) — the round-19 wiring test.
    for idx in 0..G5_COUNT_A {
        let GSymbol::Token(t) = g5_decode(idx).unwrap() else {
            panic!("idx {idx} returned ESC unexpectedly");
        };
        assert_eq!(t.level_mag, G5_PRI_A[idx], "level mismatch at idx {idx}");
        assert_eq!(t.run, G5_PRI_B[idx], "run mismatch at idx {idx}");
    }
}

#[test]
fn g5_audit_01_sub_b_table_row_by_row() {
    // audit/01 §4.1 sub-B (idx 67..101) — 35 entries — has the explicit
    // per-row layout:
    //   67..74: (run=0, level=1..8)
    //   75..77: (run=1, level=1..3)
    //   78..79: (run=2, level=1..2)
    //   80..81: (run=3, level=1..2)
    //   82..83: (run=4, level=1..2)
    //   84..85: (run=5, level=1..2)
    //   86..87: (run=6, level=1..2)
    //   88..101: (run=7..20, level=1)
    let mut expected: Vec<(u8, u8)> = (1u8..=8).map(|l| (0u8, l)).collect();
    expected.extend((1u8..=3).map(|l| (1u8, l)));
    for r in 2u8..=6 {
        expected.push((r, 1));
        expected.push((r, 2));
    }
    for r in 7u8..=20 {
        expected.push((r, 1));
    }
    assert_eq!(expected.len(), 35, "should be 35 sub-B entries");
    for (i, &(want_run, want_level)) in expected.iter().enumerate() {
        let idx = G5_COUNT_B + 1 + i;
        let GSymbol::Token(t) = g5_decode(idx).unwrap() else {
            panic!("idx {idx} should be a sub-B token");
        };
        assert!(t.last, "sub-B always last=1");
        assert_eq!(t.run, want_run, "sub-B idx {idx}: run mismatch");
        assert_eq!(t.level_mag, want_level, "sub-B idx {idx}: level mismatch");
    }
}

#[test]
fn g5_audit_01_sub_a_first_29_rows() {
    // audit/01 §4.1 sub-A first 29 entries (run 0..2):
    //   0..26: (run=0, level=1..27)
    //   27..36: (run=1, level=1..10)
    // That's 27 + 10 = 37 entries — we exercise the first 29 (a tight
    // sample that crosses two run boundaries).
    let mut expected: Vec<(u8, u8)> = (1u8..=27).map(|l| (0u8, l)).collect();
    expected.extend((1u8..=10).map(|l| (1u8, l))); // run=1
    let take = 29.min(expected.len());
    for (idx, &(want_run, want_level)) in expected[..take].iter().enumerate() {
        let GSymbol::Token(t) = g5_decode(idx).unwrap() else {
            panic!("idx {idx} should be a sub-A token");
        };
        assert!(!t.last, "sub-A always last=0");
        assert_eq!(t.run, want_run, "sub-A idx {idx}: run mismatch");
        assert_eq!(t.level_mag, want_level, "sub-A idx {idx}: level mismatch");
    }
}

#[test]
fn g5_kraft_partition_count_matches_count_a() {
    // Total non-ESC entries should equal count_A = 102.
    let non_esc = g5_iter()
        .filter(|(_, s)| matches!(s, GSymbol::Token(_)))
        .count();
    assert_eq!(non_esc, G5_COUNT_A);
}

#[test]
fn g5_pri_a_no_zero_or_sentinel_byte() {
    // Same invariant as G4: no zero-level entries, no 0xff sentinel.
    for &b in G5_PRI_A {
        assert_ne!(b, 0, "G5 pri_A contains zero byte");
        assert_ne!(b, 0xff, "G5 pri_A contains 0xff sentinel");
    }
}

#[test]
fn g5_pri_b_low_byte_only() {
    // Round-19 invariant: every G5 pri_B byte fits within 0..=20 (the
    // overall max run for the G5 alphabet). The build script enforces
    // the upper-24-bit-zero invariant on the raw u32 records before
    // emitting the constant; this test re-checks the emitted bytes.
    for (i, &b) in G5_PRI_B.iter().enumerate() {
        assert!(
            b <= 20,
            "G5 pri_B[{i}] = {b:#x} exceeds the audit/01 §4.1 max run of 20"
        );
    }
}
