//! Integration tests for the round-18 G4 descriptor wiring.
//!
//! These tests exercise the public `g_descriptor` API end-to-end against
//! the audit/01 §3 enumeration of G4 (102 entries + ESC, MSMPEG4 inter
//! DCT TCOEF). They also act as documentation of the (idx → triple)
//! semantics for a downstream Implementer who hooks the canonical-Huffman
//! bit-length table into `AcVlcTable` once the walker tree at file
//! offset `0x3df40` (spec/99 §5.3) is resolved.

use oxideav_msmpeg4::g_descriptor::{g4_decode, g4_iter, GSymbol, GToken};
use oxideav_msmpeg4::tables_data::{G4_COUNT_A, G4_COUNT_B, G4_PRI_A, G4_PRI_B};

#[test]
fn g4_full_alphabet_dense_round_trip() {
    // Walk every idx in [0, 102]. Every non-ESC entry must produce
    // a (last, run, level) triple consistent with the partition test.
    let mut sub_a_count = 0;
    let mut sub_b_count = 0;
    let mut esc_count = 0;
    for (idx, sym) in g4_iter() {
        match sym {
            GSymbol::Esc => {
                assert_eq!(idx, G4_COUNT_A);
                esc_count += 1;
            }
            GSymbol::Token(t) => {
                if t.last {
                    assert!(idx > G4_COUNT_B, "idx {idx} marked last but in sub-A");
                    sub_b_count += 1;
                } else {
                    assert!(idx <= G4_COUNT_B, "idx {idx} marked !last but in sub-B");
                    sub_a_count += 1;
                }
                // level_mag must be in [1, 12] (G4 LMAX(0)=12).
                assert!(
                    (1..=12).contains(&t.level_mag),
                    "idx {idx} level_mag {} out of [1, 12]",
                    t.level_mag,
                );
                // run must be in [0, 40] (G4 max run = 40 on sub-B last).
                assert!(t.run <= 40, "idx {idx} run {} > 40", t.run);
            }
        }
    }
    assert_eq!(sub_a_count, 58);
    assert_eq!(sub_b_count, 44);
    assert_eq!(esc_count, 1);
}

#[test]
fn g4_pri_a_pri_b_match_decoder_output() {
    // The decoder reads `pri_A[idx]` directly as level_mag and
    // `pri_B[idx]` as run for non-ESC indices. This invariant must
    // hold for every idx in [0, count_A).
    for idx in 0..G4_COUNT_A {
        let GSymbol::Token(t) = g4_decode(idx).unwrap() else {
            panic!("idx {idx} returned ESC unexpectedly");
        };
        assert_eq!(t.level_mag, G4_PRI_A[idx], "level mismatch at idx {idx}");
        assert_eq!(t.run, G4_PRI_B[idx], "run mismatch at idx {idx}");
    }
}

#[test]
fn g4_audit_01_sub_b_table_row_by_row() {
    // audit/01 §2.2 sub-B (idx 58..101) has the explicit per-row layout:
    //   58: (run=0, level=1)
    //   59: (run=0, level=2)
    //   60: (run=0, level=3)
    //   61: (run=1, level=1)
    //   62: (run=1, level=2)
    //   63..101: (run=2..40, level=1)
    let expected: Vec<(u8, u8)> = vec![(0, 1), (0, 2), (0, 3), (1, 1), (1, 2)]
        .into_iter()
        .chain((2u8..=40u8).map(|r| (r, 1u8)))
        .collect();
    assert_eq!(expected.len(), 44, "should be 44 sub-B entries");
    for (i, &(want_run, want_level)) in expected.iter().enumerate() {
        let idx = G4_COUNT_B + 1 + i;
        let GSymbol::Token(t) = g4_decode(idx).unwrap() else {
            panic!("idx {idx} should be a sub-B token");
        };
        assert!(t.last, "sub-B always last=1");
        assert_eq!(t.run, want_run, "sub-B idx {idx}: run mismatch");
        assert_eq!(t.level_mag, want_level, "sub-B idx {idx}: level mismatch");
    }
}

#[test]
fn g4_audit_01_sub_a_first_27_rows() {
    // audit/01 §2.2 sub-A first 27 entries (run 0..2):
    //   0..11: (run=0, level=1..12)
    //   12..17: (run=1, level=1..6)
    //   18..21: (run=2, level=1..4)
    //   22..24: (run=3, level=1..3)
    //   25..27: (run=4, level=1..3)
    let expected: Vec<(u8, u8)> = (1..=12u8)
        .map(|l| (0u8, l))
        .chain((1..=6u8).map(|l| (1u8, l)))
        .chain((1..=4u8).map(|l| (2u8, l)))
        .chain((1..=3u8).map(|l| (3u8, l)))
        .chain((1..=3u8).map(|l| (4u8, l)))
        .collect();
    assert_eq!(expected.len(), 28, "12+6+4+3+3 = 28 entries");
    for (idx, &(want_run, want_level)) in expected.iter().enumerate() {
        let GSymbol::Token(t) = g4_decode(idx).unwrap() else {
            panic!("idx {idx} should be a sub-A token");
        };
        assert!(!t.last, "sub-A always last=0");
        assert_eq!(t.run, want_run, "sub-A idx {idx}: run mismatch");
        assert_eq!(t.level_mag, want_level, "sub-A idx {idx}: level mismatch");
    }
}

#[test]
fn g4_kraft_partition_count_matches_count_a() {
    // Total non-ESC entries should equal count_A = 102.
    let non_esc = g4_iter()
        .filter(|(_, s)| matches!(s, GSymbol::Token(_)))
        .count();
    assert_eq!(non_esc, G4_COUNT_A);
}

#[test]
fn g4_pri_a_no_zero_or_sentinel_byte() {
    // audit/01 §2.3: G4 has no pri_A == 0 (no zero-level entries) and
    // no 0xff sentinel — the saturation-fast-path concern raised in
    // earlier specs does not apply.
    for &b in G4_PRI_A {
        assert_ne!(b, 0, "pri_A contains zero byte");
        assert_ne!(b, 0xff, "pri_A contains 0xff sentinel");
    }
}

#[test]
fn g4_token_struct_layout_is_compact() {
    // Sanity: the public token struct doesn't accidentally bloat with
    // future field additions. A 3-byte payload aligned to 1 = 3 bytes;
    // with the bool padding the natural Rust layout sits at 4 bytes.
    use std::mem::size_of;
    assert!(
        size_of::<GToken>() <= 4,
        "GToken grew past 4 bytes — bookkeeping change?"
    );
}
