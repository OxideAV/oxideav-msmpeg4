//! G0..G3 extended-descriptor (run, level, last) enumeration.
//!
//! Round-29 cleanroom output: per-symbol enumeration narrative for the
//! four DCT VLC source families that MS-MPEG4 uses on top of the
//! H.263-baseline G4/G5 pair. See:
//!
//! * `docs/video/msmpeg4/spec/09-g0-g3-enumeration.md` — narrative.
//! * `docs/video/msmpeg4/tables/region_<addr>_g<N>_enum.csv` — numeric
//!   transcription (one CSV per G-table, mirrored under
//!   `crates/oxideav-msmpeg4/tables/`).
//!
//! Per `spec/13` §2 the `last` flag is **not** stored anywhere in the
//! pri_A/pri_B byte arrays — it is derived from the sub-class partition
//! at decode time (`idx <= count_B` ⇒ sub-A / `last = 0`;
//! `count_B < idx < count_A` ⇒ sub-B / `last = 1`; `idx == count_A`
//! is the ESC sentinel).
//!
//! These tables expose the same `(idx → (run, level, last))` mapping
//! that [`crate::g_descriptor::g4_decode`] / [`g5_decode`] expose for
//! the H.263-baseline pair, but for the extended G0..G3 alphabets:
//!
//! | G  | count_A | count_B | live slot | selected when |
//! | -- | ------- | ------- | --------- | ------------- |
//! | G0 | 168 | 98  | `[esi+0xab0]` (chroma + all-inter) | `[esi+0xad0] = 1` |
//! | G1 | 185 | 118 | `[esi+0xab4]` (intra-luma)         | `[esi+0xad4] = 1` |
//! | G2 | 148 | 80  | `[esi+0xab0]`                       | `[esi+0xad0] = 0` |
//! | G3 | 132 | 84  | `[esi+0xab4]`                       | `[esi+0xad4] = 0` |
//!
//! (G4 / G5 are the v1/v2-default class-2 fallback per spec/09 §1; they
//! are wired through [`crate::g_descriptor`].)
//!
//! The runtime selector dispatch — i.e. which (selector_value,
//! frame_version) tuple resolves to which descriptor — is the
//! still-OPEN `desc+0x1c / +0x20` runtime population question called
//! out in `audit/04` §2.5 / `spec/13` §8 item 1. This module deliberately
//! exposes the four tables as parallel constants and leaves the
//! (selector → table) lookup to the caller.

use crate::g_descriptor::{GSymbol, GToken};
use crate::tables_data::{
    G0_COUNT_A, G0_COUNT_B, G0_ENUM, G1_COUNT_A, G1_COUNT_B, G1_ENUM, G2_COUNT_A, G2_COUNT_B,
    G2_ENUM, G3_COUNT_A, G3_COUNT_B, G3_ENUM,
};

/// Identify one of the four extended G-descriptors.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GExtended {
    /// G0 — chroma + all-inter (class 1). count_A=168, count_B=98.
    G0,
    /// G1 — intra-luma (class 1). count_A=185, count_B=118.
    G1,
    /// G2 — chroma + all-inter (class 0). count_A=148, count_B=80.
    G2,
    /// G3 — intra-luma (class 0). count_A=132, count_B=84.
    G3,
}

impl GExtended {
    /// Total alphabet size (== ESC sentinel index).
    pub const fn count_a(self) -> usize {
        match self {
            Self::G0 => G0_COUNT_A,
            Self::G1 => G1_COUNT_A,
            Self::G2 => G2_COUNT_A,
            Self::G3 => G3_COUNT_A,
        }
    }

    /// Sub-class partition boundary. `idx <= count_B` is sub-A
    /// (`last=0`, continues block). `count_B < idx < count_A` is sub-B
    /// (`last=1`, terminates block).
    pub const fn count_b(self) -> usize {
        match self {
            Self::G0 => G0_COUNT_B,
            Self::G1 => G1_COUNT_B,
            Self::G2 => G2_COUNT_B,
            Self::G3 => G3_COUNT_B,
        }
    }

    /// Look up the `(run, level_mag, last)` triple for `idx`. Returns
    /// `None` if `idx > count_A`. Returns `Some(GSymbol::Esc)` when
    /// `idx == count_A`.
    pub fn decode(self, idx: usize) -> Option<GSymbol> {
        let count_a = self.count_a();
        if idx > count_a {
            return None;
        }
        if idx == count_a {
            return Some(GSymbol::Esc);
        }
        // Per spec/13 §2 the partition test is `sym > count_B` ⇒ sub-B.
        let last = idx > self.count_b();
        let (run, level, last_csv) = self.entry(idx);
        // Sanity: the CSV's stored `last` flag must agree with the
        // partition test (build.rs already asserts this; this is a
        // belt-and-braces check at runtime to catch drift).
        debug_assert_eq!(
            last,
            last_csv != 0,
            "{:?} idx {} partition mismatch (count_B={})",
            self,
            idx,
            self.count_b()
        );
        Some(GSymbol::Token(GToken {
            last,
            run,
            level_mag: level,
        }))
    }

    /// Raw `(run, level_mag, last)` triple from the CSV-derived static
    /// table. Bypasses the `idx == count_A` ESC check; use [`decode`]
    /// for the ESC-aware variant.
    fn entry(self, idx: usize) -> (u8, u8, u8) {
        match self {
            Self::G0 => G0_ENUM[idx],
            Self::G1 => G1_ENUM[idx],
            Self::G2 => G2_ENUM[idx],
            Self::G3 => G3_ENUM[idx],
        }
    }

    /// Iterate every symbol in this G-table's alphabet (idx 0..count_A
    /// inclusive — the trailing entry is `GSymbol::Esc`).
    pub fn iter(self) -> impl Iterator<Item = (usize, GSymbol)> {
        let count_a = self.count_a();
        (0..=count_a).map(move |idx| (idx, self.decode(idx).unwrap()))
    }
}

/// Decode an idx through G0. Convenience for the common case.
pub fn g0_decode(idx: usize) -> Option<GSymbol> {
    GExtended::G0.decode(idx)
}

/// Decode an idx through G1.
pub fn g1_decode(idx: usize) -> Option<GSymbol> {
    GExtended::G1.decode(idx)
}

/// Decode an idx through G2.
pub fn g2_decode(idx: usize) -> Option<GSymbol> {
    GExtended::G2.decode(idx)
}

/// Decode an idx through G3.
pub fn g3_decode(idx: usize) -> Option<GSymbol> {
    GExtended::G3.decode(idx)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn count_a_count_b_match_spec() {
        // Per spec/09 §1 + spec/13 §7 G-base table.
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
    fn idx_zero_is_run0_level1_sub_a_for_all_g_tables() {
        // Every extended descriptor starts with the canonical
        // (run=0, level=1, last=0) lowest-run lowest-level entry per
        // spec/09 §3-§6 (sub-A row 0).
        for g in [GExtended::G0, GExtended::G1, GExtended::G2, GExtended::G3] {
            let GSymbol::Token(t) = g.decode(0).unwrap() else {
                panic!("{:?} idx 0 should be a token", g)
            };
            assert!(!t.last, "{:?} idx 0 sub-A", g);
            assert_eq!(t.run, 0, "{:?} idx 0 run", g);
            assert_eq!(t.level_mag, 1, "{:?} idx 0 level", g);
        }
    }

    #[test]
    fn idx_count_a_is_esc() {
        for g in [GExtended::G0, GExtended::G1, GExtended::G2, GExtended::G3] {
            assert_eq!(g.decode(g.count_a()), Some(GSymbol::Esc), "{:?} ESC", g);
        }
    }

    #[test]
    fn idx_out_of_range_is_none() {
        for g in [GExtended::G0, GExtended::G1, GExtended::G2, GExtended::G3] {
            assert_eq!(g.decode(g.count_a() + 1), None);
            assert_eq!(g.decode(usize::MAX), None);
        }
    }

    #[test]
    fn partition_strict() {
        // Every idx <= count_B is sub-A (last=false); every idx > count_B
        // (and < count_A) is sub-B (last=true).
        for g in [GExtended::G0, GExtended::G1, GExtended::G2, GExtended::G3] {
            for idx in 0..=g.count_b() {
                let GSymbol::Token(t) = g.decode(idx).unwrap() else {
                    panic!("{:?} idx {} should be a token", g, idx)
                };
                assert!(!t.last, "{:?} idx {} should be sub-A", g, idx);
            }
            for idx in (g.count_b() + 1)..g.count_a() {
                let GSymbol::Token(t) = g.decode(idx).unwrap() else {
                    panic!("{:?} idx {} should be a token", g, idx)
                };
                assert!(t.last, "{:?} idx {} should be sub-B", g, idx);
            }
        }
    }

    #[test]
    fn g0_sub_a_lmax_matches_spec_09() {
        // spec/09 §3 sub-class A (last = 0): r0 LMAX=23, r1=11, r2=8,
        // r3=7, r4..5=5, r6..7=4, r8..11=3, r12..16=2, r17..26=1.
        let mut max_per_run: std::collections::BTreeMap<u8, u8> =
            std::collections::BTreeMap::new();
        for idx in 0..=GExtended::G0.count_b() {
            let GSymbol::Token(t) = GExtended::G0.decode(idx).unwrap() else {
                unreachable!()
            };
            let e = max_per_run.entry(t.run).or_insert(0);
            *e = (*e).max(t.level_mag);
        }
        assert_eq!(max_per_run[&0], 23);
        assert_eq!(max_per_run[&1], 11);
        assert_eq!(max_per_run[&2], 8);
        assert_eq!(max_per_run[&3], 7);
        assert_eq!(max_per_run[&4], 5);
        assert_eq!(max_per_run[&5], 5);
        for r in 6..=7u8 {
            assert_eq!(max_per_run[&r], 4, "G0 sub-A run {r}");
        }
        for r in 8..=11u8 {
            assert_eq!(max_per_run[&r], 3, "G0 sub-A run {r}");
        }
        for r in 12..=16u8 {
            assert_eq!(max_per_run[&r], 2, "G0 sub-A run {r}");
        }
        for r in 17..=26u8 {
            assert_eq!(max_per_run[&r], 1, "G0 sub-A run {r}");
        }
    }

    #[test]
    fn g1_sub_a_lmax_matches_spec_09() {
        // spec/09 §4 sub-class A: r0=19, r1=15, r2=12, r3=11, r4=6,
        // r5=5, r6..9=4, r10..15=3, r16..17=2, r18..30=1.
        let mut max_per_run: std::collections::BTreeMap<u8, u8> =
            std::collections::BTreeMap::new();
        for idx in 0..=GExtended::G1.count_b() {
            let GSymbol::Token(t) = GExtended::G1.decode(idx).unwrap() else {
                unreachable!()
            };
            let e = max_per_run.entry(t.run).or_insert(0);
            *e = (*e).max(t.level_mag);
        }
        assert_eq!(max_per_run[&0], 19);
        assert_eq!(max_per_run[&1], 15);
        assert_eq!(max_per_run[&2], 12);
        assert_eq!(max_per_run[&3], 11);
        assert_eq!(max_per_run[&4], 6);
        assert_eq!(max_per_run[&5], 5);
        for r in 6..=9u8 {
            assert_eq!(max_per_run[&r], 4, "G1 sub-A run {r}");
        }
        for r in 10..=15u8 {
            assert_eq!(max_per_run[&r], 3, "G1 sub-A run {r}");
        }
        for r in 16..=17u8 {
            assert_eq!(max_per_run[&r], 2, "G1 sub-A run {r}");
        }
        for r in 18..=30u8 {
            assert_eq!(max_per_run[&r], 1, "G1 sub-A run {r}");
        }
    }

    #[test]
    fn g2_sub_b_max_run_is_43() {
        // spec/09 §5: G2 sub-B level-1 tail extends to r=43 (the
        // longest of any G-descriptor).
        let mut max_run = 0u8;
        for idx in (GExtended::G2.count_b() + 1)..GExtended::G2.count_a() {
            let GSymbol::Token(t) = GExtended::G2.decode(idx).unwrap() else {
                unreachable!()
            };
            if t.run > max_run {
                max_run = t.run;
            }
        }
        assert_eq!(max_run, 43);
    }

    #[test]
    fn g3_sub_a_max_run_is_20() {
        // spec/09 §6: G3 sub-A caps at r=20 (the smallest of the four).
        let mut max_run = 0u8;
        for idx in 0..=GExtended::G3.count_b() {
            let GSymbol::Token(t) = GExtended::G3.decode(idx).unwrap() else {
                unreachable!()
            };
            if t.run > max_run {
                max_run = t.run;
            }
        }
        assert_eq!(max_run, 20);
    }

    #[test]
    fn iter_yields_count_a_plus_esc_entries() {
        for g in [GExtended::G0, GExtended::G1, GExtended::G2, GExtended::G3] {
            let total = g.iter().count();
            assert_eq!(total, g.count_a() + 1, "{:?} iter total", g);
            let escs = g.iter().filter(|(_, s)| matches!(s, GSymbol::Esc)).count();
            assert_eq!(escs, 1, "{:?} ESC count", g);
        }
    }

    #[test]
    fn pri_b_upper_byte_refuted_max_run_fits_in_low_byte() {
        // spec/09 §7: every pri_B[idx] across G0..G3 fits in its low
        // byte (no upper bits ever set). The CSV-derived enumeration
        // stores run as u8, so any upper-byte data would have been
        // lost during build. The largest single value observed is 43
        // (G2 sub-B level-1 tail per spec/09 §5; spec/09 §7's quoted
        // "max 37" is a per-G1 statement, not a global cap). All
        // values comfortably fit in a u8 — pri_B IS pure-run.
        let mut absolute_max = 0u8;
        for g in [GExtended::G0, GExtended::G1, GExtended::G2, GExtended::G3] {
            for idx in 0..g.count_a() {
                let (run, _, _) = g.entry(idx);
                if run > absolute_max {
                    absolute_max = run;
                }
            }
        }
        assert_eq!(
            absolute_max, 43,
            "max run across G0..G3 enumerations should be 43 (G2 sub-B tail per spec/09 §5)"
        );
        assert!(
            absolute_max < 64,
            "max run > 63 would overflow scan_pos (kernel returns -100 per spec/13 §3)"
        );
    }
}
