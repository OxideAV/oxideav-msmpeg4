//! G-descriptor (run, level, last) decoder for the MS-MPEG4 DCT AC VLC.
//!
//! Per `docs/video/msmpeg4/spec/99-current-understanding.md` §4.1 / §5,
//! the six DCT AC TCOEF tables (G0..G5) all share the same descriptor
//! shape: a primary canonical-Huffman VLC yields an `idx` in
//! `[0, count_A]`, where `count_A == idx` is the ESC sentinel and the
//! remaining indices map to a `(last, run, |level|)` triple via two
//! parallel arrays:
//!
//!   * `pri_A[idx]` — unsigned byte = `|level|` (the `01 02 03 ...`
//!     canonical level prefix per `spec/99 §5.1`),
//!   * `pri_B[idx]` — u32 with the `run` count in the low byte (upper
//!     bytes are zero for G4/G5 per audit/01 §2.4 and §4.4; for G0..G3
//!     they may carry extended-level/alternate-run bits — OPEN).
//!
//! `last` is derived from the partition test (`spec/04 §1.3 step 3`):
//!
//!   * `idx ≤ count_B` → sub-class A: `last = 0`,
//!   * `count_B < idx < count_A` → sub-class B: `last = 1`,
//!   * `idx == count_A` → ESC (caller handles separately).
//!
//! Round 18 wired G4 (chroma + all-inter, default for v1/v2 streams)
//! and G5 (intra-luma) `pri_A` plus G4's `pri_B`; G5's `pri_B` lived in
//! a 408-byte gap between `region_0569c0`'s end at file offset `0x57898`
//! and the next extracted region at `0x57a30`. **Round 19 fills that
//! gap**: `tables/region_057898.hex` carries the 102 × u32-LE records
//! and `build.rs` slices their low bytes into `G5_PRI_B`. G5's full
//! alphabet (sub-class A + sub-class B + ESC) now decodes through a
//! single byte-array lookup just like G4's.
//!
//! **This module exposes the decode logic ONLY**, not the bitstream
//! VLC. The canonical-Huffman bit-length array (the prefix-code shape)
//! lives inside the shared 68 KB walker tree at file offset `0x3df40`
//! (spec/99 §5.3) and has not yet been resolved into a per-G-descriptor
//! bit-length array — wiring G4/G5 into a runnable [`crate::ac::AcVlcTable`]
//! requires that resolution. The functions here let an Implementer
//! verify the (idx → triple) post-VLC mapping in isolation, and prepare
//! the G4/G5 hooks for a future round once the walker is resolved.

use crate::tables_data::{
    G4_COUNT_A, G4_COUNT_B, G4_PRI_A, G4_PRI_B, G5_COUNT_A, G5_COUNT_B, G5_PRI_A, G5_PRI_B,
};

/// Decoded (last, run, level_mag) post-VLC token for a G-descriptor
/// alphabet. `level_mag` is unsigned — the kernel applies the sign bit
/// read separately after the VLC match (per spec/99 §4.2 step 3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GToken {
    pub last: bool,
    pub run: u8,
    /// Unsigned level magnitude `|level|`. Caller applies sign bit.
    pub level_mag: u8,
}

/// ESC sentinel — emitted when `idx == count_A`. The kernel then reads
/// one of the 3-tier escape bodies (level-extension / run-extension /
/// fixed-length triple) per spec/99 §4.3.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GSymbol {
    Token(GToken),
    Esc,
}

/// Resolve a primary-VLC index through the G4 descriptor.
///
/// Returns `None` if `idx > count_A` (out of range — the canonical-Huffman
/// builder should never produce this). `idx == count_A` returns
/// `GSymbol::Esc`; otherwise returns the `(last, run, level_mag)` triple.
///
/// Per spec/99 §4.2 step 3: `idx ≤ count_B` ⇒ sub-class A (last=0);
/// `count_B < idx < count_A` ⇒ sub-class B (last=1).
pub fn g4_decode(idx: usize) -> Option<GSymbol> {
    if idx > G4_COUNT_A {
        return None;
    }
    if idx == G4_COUNT_A {
        return Some(GSymbol::Esc);
    }
    let last = idx > G4_COUNT_B;
    Some(GSymbol::Token(GToken {
        last,
        run: G4_PRI_B[idx],
        level_mag: G4_PRI_A[idx],
    }))
}

/// Resolve a primary-VLC index through the G5 descriptor.
///
/// Round 19 wires the full G5 alphabet (sub-class A + sub-class B +
/// ESC). Returns `None` if `idx > count_A`; otherwise returns the
/// `(last, run, level_mag)` triple drawn straight from the byte arrays
/// `G5_PRI_A` / `G5_PRI_B` (the latter is the low byte of each u32-LE
/// record at file `0x57898..0x57a30`).
pub fn g5_decode(idx: usize) -> Option<GSymbol> {
    if idx > G5_COUNT_A {
        return None;
    }
    if idx == G5_COUNT_A {
        return Some(GSymbol::Esc);
    }
    let last = idx > G5_COUNT_B;
    Some(GSymbol::Token(GToken {
        last,
        run: G5_PRI_B[idx],
        level_mag: G5_PRI_A[idx],
    }))
}

/// Inspect G4's full alphabet — produces 102 + 1 (ESC) = 103 entries for
/// validation tests. Useful as a corpus when downstream wiring lands.
pub fn g4_iter() -> impl Iterator<Item = (usize, GSymbol)> {
    (0..=G4_COUNT_A).map(|idx| (idx, g4_decode(idx).unwrap()))
}

/// Inspect G5's full alphabet — produces 102 + 1 (ESC) = 103 entries
/// for validation tests. Round 19 promoted `g5_iter_partial` to a full
/// iterator now that G5 pri_B is wired; the legacy name is kept as an
/// alias for source compatibility.
pub fn g5_iter() -> impl Iterator<Item = (usize, GSymbol)> {
    (0..=G5_COUNT_A).map(|idx| (idx, g5_decode(idx).unwrap()))
}

/// Backwards-compatible alias for [`g5_iter`]. Now that G5 pri_B is
/// wired (round 19) the partial / full distinction is academic, but
/// keeping the alias avoids breaking external test corpora.
pub fn g5_iter_partial() -> impl Iterator<Item = (usize, GSymbol)> {
    g5_iter()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn g4_idx_zero_is_run0_level1_sub_a() {
        assert_eq!(
            g4_decode(0),
            Some(GSymbol::Token(GToken {
                last: false,
                run: 0,
                level_mag: 1,
            })),
        );
    }

    #[test]
    fn g4_idx_eleven_is_run0_level12_sub_a() {
        // audit/01 §2.2 row 11: pri_A=12, run=0, level=12.
        assert_eq!(
            g4_decode(11),
            Some(GSymbol::Token(GToken {
                last: false,
                run: 0,
                level_mag: 12,
            })),
        );
    }

    #[test]
    fn g4_idx_count_b_is_run26_level1_sub_a() {
        // audit/01 §2.2: idx 57 = sub-A boundary, (run=26, level=1).
        assert_eq!(
            g4_decode(57),
            Some(GSymbol::Token(GToken {
                last: false,
                run: 26,
                level_mag: 1,
            })),
        );
    }

    #[test]
    fn g4_idx_count_b_plus_one_is_run0_level1_sub_b() {
        // audit/01 §2.2: idx 58 = sub-B start, (run=0, level=1, last=1).
        assert_eq!(
            g4_decode(58),
            Some(GSymbol::Token(GToken {
                last: true,
                run: 0,
                level_mag: 1,
            })),
        );
    }

    #[test]
    fn g4_idx_count_a_is_esc() {
        assert_eq!(g4_decode(G4_COUNT_A), Some(GSymbol::Esc));
    }

    #[test]
    fn g4_idx_out_of_range_is_none() {
        assert_eq!(g4_decode(G4_COUNT_A + 1), None);
        assert_eq!(g4_decode(usize::MAX), None);
    }

    #[test]
    fn g4_sub_a_partition_strict() {
        // Every idx in [0, 57] is sub-A (last=false); every idx in [58,
        // 101] is sub-B (last=true).
        for idx in 0..=G4_COUNT_B {
            let GSymbol::Token(t) = g4_decode(idx).unwrap() else {
                panic!()
            };
            assert!(!t.last, "idx {idx} should be sub-A");
        }
        for idx in (G4_COUNT_B + 1)..G4_COUNT_A {
            let GSymbol::Token(t) = g4_decode(idx).unwrap() else {
                panic!()
            };
            assert!(t.last, "idx {idx} should be sub-B");
        }
    }

    #[test]
    fn g4_max_run_at_idx_101_is_40() {
        // audit/01 §2.2: idx 101 = sub-B last, (run=40, level=1).
        assert_eq!(
            g4_decode(101),
            Some(GSymbol::Token(GToken {
                last: true,
                run: 40,
                level_mag: 1,
            })),
        );
    }

    #[test]
    fn g4_alphabet_size_is_102_plus_esc() {
        let total = g4_iter().count();
        assert_eq!(total, 103, "102 alphabet + 1 ESC");
        let escs = g4_iter().filter(|(_, s)| matches!(s, GSymbol::Esc)).count();
        assert_eq!(escs, 1);
    }

    #[test]
    fn g4_sub_a_per_run_lmax_matches_audit() {
        // audit/01 §3.3 Table 11-19 LMAX rows for inter (sub-A):
        // run 0 LMAX=12, run 1 LMAX=6, run 2 LMAX=4, run 3..6 LMAX=3,
        // run 7..10 LMAX=2, run 11..26 LMAX=1.
        let mut max_per_run: std::collections::BTreeMap<u8, u8> = std::collections::BTreeMap::new();
        for idx in 0..=G4_COUNT_B {
            let GSymbol::Token(t) = g4_decode(idx).unwrap() else {
                unreachable!()
            };
            let e = max_per_run.entry(t.run).or_insert(0);
            *e = (*e).max(t.level_mag);
        }
        assert_eq!(max_per_run[&0], 12);
        assert_eq!(max_per_run[&1], 6);
        assert_eq!(max_per_run[&2], 4);
        for r in 3..=6u8 {
            assert_eq!(max_per_run[&r], 3, "run {r}");
        }
        for r in 7..=10u8 {
            assert_eq!(max_per_run[&r], 2, "run {r}");
        }
        for r in 11..=26u8 {
            assert_eq!(max_per_run[&r], 1, "run {r}");
        }
    }

    #[test]
    fn g4_sub_b_per_run_lmax_matches_audit() {
        // audit/01 §3.3 sub-B (last=1): run 0 LMAX=3, run 1 LMAX=2,
        // run 2..40 LMAX=1.
        let mut max_per_run: std::collections::BTreeMap<u8, u8> = std::collections::BTreeMap::new();
        for idx in (G4_COUNT_B + 1)..G4_COUNT_A {
            let GSymbol::Token(t) = g4_decode(idx).unwrap() else {
                unreachable!()
            };
            let e = max_per_run.entry(t.run).or_insert(0);
            *e = (*e).max(t.level_mag);
        }
        assert_eq!(max_per_run[&0], 3);
        assert_eq!(max_per_run[&1], 2);
        for r in 2..=40u8 {
            assert_eq!(max_per_run[&r], 1, "sub-B run {r}");
        }
    }

    #[test]
    fn g5_idx_zero_is_run0_level1() {
        assert_eq!(
            g5_decode(0),
            Some(GSymbol::Token(GToken {
                last: false,
                run: 0,
                level_mag: 1,
            })),
        );
    }

    #[test]
    fn g5_idx_twenty_six_is_run0_level27() {
        // audit/01 §4.1 sub-A row r0 covers levels 1..27, so idx 26 is
        // (run=0, level=27).
        assert_eq!(
            g5_decode(26),
            Some(GSymbol::Token(GToken {
                last: false,
                run: 0,
                level_mag: 27,
            })),
        );
    }

    #[test]
    fn g5_idx_twenty_seven_is_run1_level1() {
        // First sub-A row after run=0 is run=1 (idx 27..36, levels 1..10).
        assert_eq!(
            g5_decode(27),
            Some(GSymbol::Token(GToken {
                last: false,
                run: 1,
                level_mag: 1,
            })),
        );
    }

    #[test]
    fn g5_sub_b_first_entry_is_run0_level1_last() {
        // Round 19: G5 pri_B is wired. audit/01 §4.1 sub-B row 0:
        // (run=0, level=1, last=1) at idx 67.
        assert_eq!(
            g5_decode(G5_COUNT_B + 1),
            Some(GSymbol::Token(GToken {
                last: true,
                run: 0,
                level_mag: 1,
            })),
        );
    }

    #[test]
    fn g5_sub_b_last_entry_is_run20_level1_last() {
        // audit/01 §4.1 sub-B last row: idx 101 = (run=20, level=1, last=1).
        assert_eq!(
            g5_decode(G5_COUNT_A - 1),
            Some(GSymbol::Token(GToken {
                last: true,
                run: 20,
                level_mag: 1,
            })),
        );
    }

    #[test]
    fn g5_esc_works() {
        assert_eq!(g5_decode(G5_COUNT_A), Some(GSymbol::Esc));
    }

    #[test]
    fn g5_idx_out_of_range_is_none() {
        assert_eq!(g5_decode(G5_COUNT_A + 1), None);
        assert_eq!(g5_decode(usize::MAX), None);
    }

    #[test]
    fn g5_sub_a_count_matches_audit() {
        // audit/01 §4.1: 67 sub-A entries.
        let n = g5_iter()
            .filter(|(_, s)| matches!(s, GSymbol::Token(t) if !t.last))
            .count();
        assert_eq!(n, 67);
    }

    #[test]
    fn g5_sub_b_count_matches_audit() {
        // audit/01 §4.1: 35 sub-B entries (idx 67..101).
        let n = g5_iter()
            .filter(|(_, s)| matches!(s, GSymbol::Token(t) if t.last))
            .count();
        assert_eq!(n, 35);
    }

    #[test]
    fn g5_alphabet_size_is_102_plus_esc() {
        let total = g5_iter().count();
        assert_eq!(total, 103, "102 alphabet + 1 ESC");
        let escs = g5_iter().filter(|(_, s)| matches!(s, GSymbol::Esc)).count();
        assert_eq!(escs, 1);
    }

    #[test]
    fn g5_sub_a_partition_strict() {
        // Every idx in [0, 66] is sub-A (last=false); every idx in
        // [67, 101] is sub-B (last=true).
        for idx in 0..=G5_COUNT_B {
            let GSymbol::Token(t) = g5_decode(idx).unwrap() else {
                panic!()
            };
            assert!(!t.last, "idx {idx} should be sub-A");
        }
        for idx in (G5_COUNT_B + 1)..G5_COUNT_A {
            let GSymbol::Token(t) = g5_decode(idx).unwrap() else {
                panic!()
            };
            assert!(t.last, "idx {idx} should be sub-B");
        }
    }

    #[test]
    fn g5_sub_b_per_run_lmax_matches_audit() {
        // audit/01 §4.1 sub-B (last=1):
        //   run 0 LMAX=8, run 1 LMAX=3, run 2..6 LMAX=2, run 7..20 LMAX=1.
        let mut max_per_run: std::collections::BTreeMap<u8, u8> = std::collections::BTreeMap::new();
        for idx in (G5_COUNT_B + 1)..G5_COUNT_A {
            let GSymbol::Token(t) = g5_decode(idx).unwrap() else {
                unreachable!()
            };
            let e = max_per_run.entry(t.run).or_insert(0);
            *e = (*e).max(t.level_mag);
        }
        assert_eq!(max_per_run[&0], 8);
        assert_eq!(max_per_run[&1], 3);
        for r in 2..=6u8 {
            assert_eq!(max_per_run[&r], 2, "sub-B run {r}");
        }
        for r in 7..=20u8 {
            assert_eq!(max_per_run[&r], 1, "sub-B run {r}");
        }
    }

    #[test]
    fn g5_sub_b_per_run_count_matches_audit() {
        // audit/01 §4.1 sub-B count-per-run:
        //   run 0: 8, run 1: 3, run 2..6: 2 each, run 7..20: 1 each.
        let mut count_per_run: std::collections::BTreeMap<u8, usize> =
            std::collections::BTreeMap::new();
        for idx in (G5_COUNT_B + 1)..G5_COUNT_A {
            let GSymbol::Token(t) = g5_decode(idx).unwrap() else {
                unreachable!()
            };
            *count_per_run.entry(t.run).or_insert(0) += 1;
        }
        assert_eq!(count_per_run[&0], 8);
        assert_eq!(count_per_run[&1], 3);
        for r in 2..=6u8 {
            assert_eq!(count_per_run[&r], 2, "sub-B run {r} count");
        }
        for r in 7..=20u8 {
            assert_eq!(count_per_run[&r], 1, "sub-B run {r} count");
        }
        // 8 + 3 + 5*2 + 14 = 35.
        let total: usize = count_per_run.values().sum();
        assert_eq!(total, 35);
    }

    #[test]
    fn g5_sub_a_per_run_lmax_matches_audit() {
        // audit/01 §4.1: r0:27, r1:10, r2:5, r3:4, r4..7:3, r8..9:2,
        // r10..14:1.
        let mut max_per_run: std::collections::BTreeMap<u8, u8> = std::collections::BTreeMap::new();
        for idx in 0..=G5_COUNT_B {
            let GSymbol::Token(t) = g5_decode(idx).unwrap() else {
                unreachable!()
            };
            let e = max_per_run.entry(t.run).or_insert(0);
            *e = (*e).max(t.level_mag);
        }
        assert_eq!(max_per_run[&0], 27);
        assert_eq!(max_per_run[&1], 10);
        assert_eq!(max_per_run[&2], 5);
        assert_eq!(max_per_run[&3], 4);
        for r in 4..=7u8 {
            assert_eq!(max_per_run[&r], 3, "run {r}");
        }
        for r in 8..=9u8 {
            assert_eq!(max_per_run[&r], 2, "run {r}");
        }
        for r in 10..=14u8 {
            assert_eq!(max_per_run[&r], 1, "run {r}");
        }
    }
}
