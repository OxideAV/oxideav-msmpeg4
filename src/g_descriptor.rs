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
//! Round 18 wires G4 (chroma + all-inter, default for v1/v2 streams)
//! and G5 (intra-luma) data — both `pri_A` arrays plus G4's `pri_B`.
//! G5's `pri_B` lives in a 408-byte gap between `region_0569c0`'s end
//! at file offset `0x57898` and the next extracted region at `0x57a30`;
//! it is not yet captured in `tables/`. See `tables/region_0569c0.meta`
//! for the gap analysis.
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
    G4_COUNT_A, G4_COUNT_B, G4_PRI_A, G4_PRI_B, G5_COUNT_A, G5_COUNT_B, G5_PRI_A,
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
/// **Partial wiring** — `pri_B` for G5 is not yet extracted (spec/99 §10.3
/// gap), so this function returns `None` for any sub-class B index
/// (`count_B < idx < count_A`). Sub-class A and ESC paths work today.
/// See `tables/region_0569c0.meta` for the gap analysis.
pub fn g5_decode(idx: usize) -> Option<GSymbol> {
    if idx > G5_COUNT_A {
        return None;
    }
    if idx == G5_COUNT_A {
        return Some(GSymbol::Esc);
    }
    let last = idx > G5_COUNT_B;
    if last {
        // sub-class B requires G5 pri_B which is not yet wired.
        return None;
    }
    // For sub-class A entries we only need pri_A; pri_B is implicitly
    // derivable from the canonical-level-prefix structure (run advances
    // when the level magnitude resets to 1 from a higher value), but we
    // refuse to derive it here — callers should treat G5 sub-A access
    // as informational until the gap is filled.
    Some(GSymbol::Token(GToken {
        last,
        run: g5_pri_b_sub_a_derived(idx)?,
        level_mag: G5_PRI_A[idx],
    }))
}

/// Reconstruct G5 pri_B sub-class A entries from the canonical-level
/// prefix structure. Per audit/01 §4.1, sub-A's (run, level) pairs are
/// laid out as:
///
///   run=0 levels 1..27   (idx 0..26)
///   run=1 levels 1..10   (idx 27..36)
///   run=2 levels 1..5    (idx 37..41)
///   run=3 levels 1..4    (idx 42..45)
///   run=4..7 levels 1..3 (idx 46..57)
///   run=8..9 levels 1..2 (idx 58..61)
///   run=10..14 level 1   (idx 62..66)
///
/// This is the LMAX(intra) profile from MPEG-4 Part 2 Table 11-18 (per
/// audit/01 §4.2), and it's reproduced here from the audit's per-row
/// dump — NOT the binary's pri_B bytes. The function returns `None` if
/// `idx >= count_B + 1` (sub-A boundary); the caller should fall back
/// to the not-yet-wired pri_B for sub-class B.
fn g5_pri_b_sub_a_derived(idx: usize) -> Option<u8> {
    // Cumulative count per (run, max_level) row:
    const SUBA_BREAKS: &[(u8, u8)] = &[
        // (run, count_at_this_run)
        (0, 27),
        (1, 10),
        (2, 5),
        (3, 4),
        (4, 3),
        (5, 3),
        (6, 3),
        (7, 3),
        (8, 2),
        (9, 2),
        (10, 1),
        (11, 1),
        (12, 1),
        (13, 1),
        (14, 1),
    ];
    let mut cum: usize = 0;
    for &(run, n) in SUBA_BREAKS {
        let next = cum + n as usize;
        if idx < next {
            return Some(run);
        }
        cum = next;
    }
    None
}

/// Inspect G4's full alphabet — produces 102 + 1 (ESC) = 103 entries for
/// validation tests. Useful as a corpus when downstream wiring lands.
pub fn g4_iter() -> impl Iterator<Item = (usize, GSymbol)> {
    (0..=G4_COUNT_A).map(|idx| (idx, g4_decode(idx).unwrap()))
}

/// Inspect G5's sub-class A + ESC slice (sub-class B not yet wired).
pub fn g5_iter_partial() -> impl Iterator<Item = (usize, GSymbol)> {
    (0..=G5_COUNT_B)
        .chain(std::iter::once(G5_COUNT_A))
        .map(|idx| (idx, g5_decode(idx).unwrap()))
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
    fn g5_sub_b_returns_none_until_table_fully_wired() {
        // Round 18 has G5 pri_B in a gap — sub-B access is not yet
        // available. Caller-visible semantics: explicit None.
        for idx in (G5_COUNT_B + 1)..G5_COUNT_A {
            assert_eq!(g5_decode(idx), None, "G5 sub-B idx {idx} should be None");
        }
    }

    #[test]
    fn g5_esc_works_despite_subb_gap() {
        assert_eq!(g5_decode(G5_COUNT_A), Some(GSymbol::Esc));
    }

    #[test]
    fn g5_sub_a_count_matches_audit() {
        // audit/01 §4.1: 67 sub-A entries.
        let n = g5_iter_partial()
            .filter(|(_, s)| matches!(s, GSymbol::Token(t) if !t.last))
            .count();
        assert_eq!(n, 67);
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
