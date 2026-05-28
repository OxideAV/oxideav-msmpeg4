//! Unified dispatch surface over the six MS-MPEG4 DCT-AC G-descriptors.
//!
//! Per `docs/video/msmpeg4/spec/15-count-ab-per-g-family.md` §1 / §2 /
//! §3 and `spec/14` §3, the MS-MPEG4 v1/v2/v3 decoder maintains six
//! parallel G-descriptors (G0..G5), each a 36-byte (`0x24`) record laid
//! out contiguously at state-struct offsets `[this+0x9d8 .. 0xab0)`.
//! Every descriptor exposes the same field layout (per spec/14 §1):
//!
//! | Descriptor offset | Field | Type |
//! | ----------------- | ----- | ---- |
//! | `+0x00` | per-slot decoder-object pointer | `u32` |
//! | `+0x04` | `count_A` (alphabet size; ESC marker index) | `u32` |
//! | `+0x08` | `count_B` (sub-A / sub-B partition boundary) | `u32` |
//! | `+0x0c` | sub-A level-extension array base pointer | `u32` |
//! | `+0x10` | sub-B level-extension array base pointer | `u32` |
//! | `+0x14` | sub-A run-extension array base pointer | `u32` |
//! | `+0x18` | sub-B run-extension array base pointer | `u32` |
//! | `+0x1c` | `pri_A` base pointer (level-magnitude byte array) | `u32` |
//! | `+0x20` | `pri_B` base pointer (run-value u32 array) | `u32` |
//!
//! Today the crate exposes two parallel post-VLC `(idx → (last, run,
//! |level|))` decoders: [`crate::g_descriptor::g4_decode`] /
//! [`g5_decode`] for the H.263-baseline G4 / G5 pair, and
//! [`crate::g_enum::GExtended`] for the four MS-MPEG4-specific G0..G3
//! extended descriptors. The two surfaces evolved separately as the
//! extraction work landed (round 18/19 for G4/G5, round 29 for G0..G3),
//! and they do not share a common Rust type today — a caller that wants
//! to iterate "every G-descriptor" has to special-case the G4/G5 split.
//!
//! This module unifies the six descriptors behind a single [`GFamily`]
//! enum so downstream code (notably the picture-header → AcVlcTable
//! dispatcher in [`crate::picture`]) can name a descriptor uniformly. It
//! adds no new tables and no new decode logic — every accessor delegates
//! to the existing `g_descriptor` / `g_enum` plumbing.
//!
//! ## What this is NOT
//!
//! [`GFamily`] does not expose the primary canonical-Huffman VLC for any
//! G-table — that lives in [`crate::ac::AcVlcTable`] and is gated on
//! per-G packed-Huffman extraction (today G4/G5 are wired; G0..G3 fall
//! back to DC-only reconstruction). Use this enum for the descriptor's
//! **structural** facts (counts, partition, role); use
//! [`crate::ac::AcVlcTable`] for the bitstream VLC.

use crate::g_descriptor::{g4_decode, g4_iter, g5_decode, g5_iter, GSymbol};
use crate::g_enum::GExtended;
use crate::tables_data::{G_COUNTS_SPEC15, G_SUBCLASS_SIZES_SPEC15};

/// The six MS-MPEG4 DCT-AC G-descriptors. Per spec/15 §3 and spec/14 §3
/// these are the only DCT-AC families the decoder ever reaches; every
/// other selector value is rejected at the picture-header parser.
///
/// The enum is `repr(u8)` with values matching the per-G index into
/// [`crate::tables_data::G_COUNTS_SPEC15`] /
/// [`crate::tables_data::G_SUBCLASS_SIZES_SPEC15`] so the discriminant
/// itself can index those arrays without a translation step.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum GFamily {
    /// G0 — chroma + all-inter, **class 1** (extended). Selected when
    /// `ac_chroma_sel == 1` per spec/14 §3.1. `count_A=168, count_B=98`.
    G0 = 0,
    /// G1 — intra-luma, **class 1** (extended). Selected when
    /// `ac_luma_sel == 1` per spec/14 §3.1. `count_A=185, count_B=118`.
    G1 = 1,
    /// G2 — chroma + all-inter, **class 0** (extended). Selected when
    /// `ac_chroma_sel == 0` per spec/14 §3.1. `count_A=148, count_B=80`.
    G2 = 2,
    /// G3 — intra-luma, **class 0** (extended). Selected when
    /// `ac_luma_sel == 0` per spec/14 §3.1. `count_A=132, count_B=84`.
    G3 = 3,
    /// G4 — chroma + all-inter, MPEG-4 Part 2 Inter LMAX baseline.
    /// Selected when `ac_chroma_sel == 2`, AND unconditionally for v1 /
    /// v2 chroma per spec/14 §3.1 v1/v2 fallthrough. `count_A=102,
    /// count_B=57`.
    G4 = 4,
    /// G5 — intra-luma, MPEG-4 Part 2 Intra LMAX baseline. Selected when
    /// `ac_luma_sel == 2`, AND unconditionally for v1 / v2 luma per
    /// spec/14 §3.1 v1/v2 fallthrough. `count_A=102, count_B=66`.
    G5 = 5,
}

/// Which **role** a G-descriptor fills in the per-frame DCT-AC dispatch
/// (spec/14 §3.1).
///
/// Each role is filled by exactly three of the six descriptors (one per
/// selector value 0/1/2). The picture-header selector bits
/// `ac_chroma_sel` ∈ {0,1,2} and `ac_luma_sel` ∈ {0,1,2} route per-frame
/// to one descriptor of each role; the v1/v2 fallthrough always picks
/// G4 (chroma) and G5 (luma).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GRole {
    /// Chroma blocks (Cb / Cr) + every all-inter block. Slot
    /// `[esi+0xab0]`, picture-header `ac_chroma_sel` ∈ {0,1,2} → {G2, G0,
    /// G4}.
    ChromaAndInter,
    /// Intra-luma blocks only (luma blocks of an intra MB, or intra-in-P
    /// luma blocks). Slot `[esi+0xab4]`, picture-header `ac_luma_sel` ∈
    /// {0,1,2} → {G3, G1, G5}.
    IntraLuma,
}

impl GFamily {
    /// All six G-families, in stable enumeration order (G0..G5).
    pub const ALL: [GFamily; 6] = [
        GFamily::G0,
        GFamily::G1,
        GFamily::G2,
        GFamily::G3,
        GFamily::G4,
        GFamily::G5,
    ];

    /// Index into [`crate::tables_data::G_COUNTS_SPEC15`] /
    /// [`crate::tables_data::G_SUBCLASS_SIZES_SPEC15`]. Equivalent to
    /// the `#[repr(u8)]` discriminant cast to `usize`.
    pub const fn index(self) -> usize {
        self as u8 as usize
    }

    /// Alphabet size (also the ESC sentinel index). Per spec/15 §3:
    /// G0=168, G1=185, G2=148, G3=132, G4=102, G5=102.
    pub const fn count_a(self) -> usize {
        G_COUNTS_SPEC15[self.index()].0 as usize
    }

    /// Sub-class A / sub-class B partition boundary. Per spec/15 §3:
    /// G0=98, G1=118, G2=80, G3=84, G4=57, G5=66. The kernel's
    /// partition test (per spec/13 §2 `1c216e2a..1c216e2f`) is `idx >
    /// count_B` ⇒ sub-class B.
    pub const fn count_b(self) -> usize {
        G_COUNTS_SPEC15[self.index()].1 as usize
    }

    /// Number of sub-class A symbols (= `count_B + 1`). Per spec/03 §4.4
    /// / spec/15 §5.2.
    pub const fn subclass_a_size(self) -> usize {
        G_SUBCLASS_SIZES_SPEC15[self.index()].0 as usize
    }

    /// Number of sub-class B symbols (= `count_A - count_B - 1`). Per
    /// spec/03 §4.4 / spec/15 §5.2.
    pub const fn subclass_b_size(self) -> usize {
        G_SUBCLASS_SIZES_SPEC15[self.index()].1 as usize
    }

    /// Per spec/14 §1 / spec/15 §2.1: the descriptor's base offset
    /// within the MS-MPEG4 state struct (`this+...`), where the
    /// 36-byte record `(+0x00..+0x24)` for this G-family lives.
    ///
    /// G0=`0x9d8`, G1=`0x9fc`, G2=`0xa20`, G3=`0xa44`, G4=`0xa68`,
    /// G5=`0xa8c`. The successive `0x24` stride is fixed and verified
    /// by spec/15 §2.1's literal-immediate disassembly evidence.
    pub const fn descriptor_base_offset(self) -> u32 {
        // Six descriptors × 0x24 bytes each, starting at 0x9d8.
        0x9d8 + (self.index() as u32) * 0x24
    }

    /// Per spec/14 §3.1: the role this descriptor fills in the per-frame
    /// dispatch. Chroma+all-inter for G0/G2/G4; intra-luma for G1/G3/G5.
    pub const fn role(self) -> GRole {
        match self {
            GFamily::G0 | GFamily::G2 | GFamily::G4 => GRole::ChromaAndInter,
            GFamily::G1 | GFamily::G3 | GFamily::G5 => GRole::IntraLuma,
        }
    }

    /// Resolve a picture-header **chroma** selector value ∈ {0,1,2} to
    /// the matching G-family. Per spec/14 §3.1: `0 → G2, 1 → G0, 2 →
    /// G4`. Any other selector value returns `None` (the picture-header
    /// parser already clamps to {0,1,2} via a unary 1-or-2-bit read, so
    /// this is a defence-in-depth check rather than a recoverable
    /// error path).
    pub const fn for_chroma_selector(sel: u8) -> Option<GFamily> {
        match sel {
            0 => Some(GFamily::G2),
            1 => Some(GFamily::G0),
            2 => Some(GFamily::G4),
            _ => None,
        }
    }

    /// Resolve a picture-header **luma** selector value ∈ {0,1,2} to
    /// the matching G-family. Per spec/14 §3.1: `0 → G3, 1 → G1, 2 →
    /// G5`. Any other selector value returns `None`.
    pub const fn for_luma_selector(sel: u8) -> Option<GFamily> {
        match sel {
            0 => Some(GFamily::G3),
            1 => Some(GFamily::G1),
            2 => Some(GFamily::G5),
            _ => None,
        }
    }

    /// Resolve a primary-VLC index through this G-family's post-VLC
    /// `(idx → (last, run, |level|))` mapping. Dispatches to
    /// [`crate::g_descriptor::g4_decode`] / [`g5_decode`] for G4 / G5
    /// or to [`crate::g_enum::GExtended::decode`] for G0..G3.
    ///
    /// `idx == count_A` returns `Some(GSymbol::Esc)`; `idx > count_A`
    /// returns `None`.
    pub fn decode(self, idx: usize) -> Option<GSymbol> {
        match self {
            GFamily::G0 => GExtended::G0.decode(idx),
            GFamily::G1 => GExtended::G1.decode(idx),
            GFamily::G2 => GExtended::G2.decode(idx),
            GFamily::G3 => GExtended::G3.decode(idx),
            GFamily::G4 => g4_decode(idx),
            GFamily::G5 => g5_decode(idx),
        }
    }

    /// Iterate every symbol in this G-family's alphabet (`idx` 0 through
    /// `count_A` inclusive; the trailing entry is `GSymbol::Esc`). Total
    /// length is `count_A + 1`.
    pub fn iter(self) -> Box<dyn Iterator<Item = (usize, GSymbol)>> {
        match self {
            GFamily::G0 => Box::new(GExtended::G0.iter()),
            GFamily::G1 => Box::new(GExtended::G1.iter()),
            GFamily::G2 => Box::new(GExtended::G2.iter()),
            GFamily::G3 => Box::new(GExtended::G3.iter()),
            GFamily::G4 => Box::new(g4_iter()),
            GFamily::G5 => Box::new(g5_iter()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_contains_six_distinct_families() {
        let mut seen = std::collections::HashSet::new();
        for g in GFamily::ALL {
            assert!(seen.insert(g), "{g:?} appears twice in ALL");
        }
        assert_eq!(seen.len(), 6);
    }

    #[test]
    fn index_matches_repr_discriminant() {
        // Cross-check that the manual repr(u8) discriminant values line
        // up with the position in ALL. If a future round reorders the
        // enum, this catches the silent G_COUNTS_SPEC15 index drift.
        for (slot, g) in GFamily::ALL.iter().enumerate() {
            assert_eq!(g.index(), slot, "{g:?} index");
        }
    }

    #[test]
    fn count_a_matches_spec_15_table() {
        // Per spec/15 §3: G0=168, G1=185, G2=148, G3=132, G4=102, G5=102.
        assert_eq!(GFamily::G0.count_a(), 168);
        assert_eq!(GFamily::G1.count_a(), 185);
        assert_eq!(GFamily::G2.count_a(), 148);
        assert_eq!(GFamily::G3.count_a(), 132);
        assert_eq!(GFamily::G4.count_a(), 102);
        assert_eq!(GFamily::G5.count_a(), 102);
    }

    #[test]
    fn count_b_matches_spec_15_table() {
        // Per spec/15 §3: G0=98, G1=118, G2=80, G3=84, G4=57, G5=66.
        assert_eq!(GFamily::G0.count_b(), 98);
        assert_eq!(GFamily::G1.count_b(), 118);
        assert_eq!(GFamily::G2.count_b(), 80);
        assert_eq!(GFamily::G3.count_b(), 84);
        assert_eq!(GFamily::G4.count_b(), 57);
        assert_eq!(GFamily::G5.count_b(), 66);
    }

    #[test]
    fn subclass_partition_sizes_match_spec_15_table() {
        // Per spec/15 §5.2: sub_A = count_B + 1, sub_B = count_A - count_B - 1.
        // Numeric values: G0 (99,69) G1 (119,66) G2 (81,67) G3 (85,47)
        //                 G4 (58,44) G5 (67,35).
        let expected = [
            (99usize, 69usize),
            (119, 66),
            (81, 67),
            (85, 47),
            (58, 44),
            (67, 35),
        ];
        for (g, (sa, sb)) in GFamily::ALL.iter().zip(expected.iter()) {
            assert_eq!(g.subclass_a_size(), *sa, "{g:?} sub_A");
            assert_eq!(g.subclass_b_size(), *sb, "{g:?} sub_B");
        }
    }

    #[test]
    fn subclass_arithmetic_self_consistent() {
        // sub_A + sub_B + 1 (ESC) == count_A + 1 (the iterator length).
        // Per spec/13 §2: sub-A ∪ sub-B ∪ {ESC} = full alphabet.
        for g in GFamily::ALL {
            assert_eq!(
                g.subclass_a_size() + g.subclass_b_size() + 1,
                g.count_a() + 1,
                "{g:?} sub-A + sub-B + ESC vs count_A + ESC"
            );
            assert_eq!(g.subclass_a_size(), g.count_b() + 1, "{g:?} sub_A def");
            assert_eq!(
                g.subclass_b_size(),
                g.count_a() - g.count_b() - 1,
                "{g:?} sub_B def"
            );
        }
    }

    #[test]
    fn descriptor_base_offsets_match_spec_15_constructor() {
        // Per spec/15 §2.1's literal-immediate disassembly:
        //   G0 +0x9d8, G1 +0x9fc, G2 +0xa20, G3 +0xa44, G4 +0xa68, G5 +0xa8c.
        // Stride is 0x24 (36-byte records, six of them = 0xd8 total span
        // ending at +0xab0 per spec/14 §1).
        let expected = [0x9d8u32, 0x9fc, 0xa20, 0xa44, 0xa68, 0xa8c];
        for (g, &off) in GFamily::ALL.iter().zip(expected.iter()) {
            assert_eq!(g.descriptor_base_offset(), off, "{g:?} base offset");
        }
        // Successive offsets differ by exactly 0x24 (the descriptor
        // record size per spec/14 §1).
        for pair in GFamily::ALL.windows(2) {
            let lo = pair[0].descriptor_base_offset();
            let hi = pair[1].descriptor_base_offset();
            assert_eq!(hi - lo, 0x24, "{:?}..{:?} stride", pair[0], pair[1]);
        }
        // Cluster ends at +0xab0 (the v1/v2 fallthrough boundary per
        // spec/14 §1).
        let g5 = GFamily::G5.descriptor_base_offset();
        assert_eq!(g5 + 0x24, 0xab0, "G5 end == 0xab0");
    }

    #[test]
    fn roles_partition_descriptors_three_each() {
        // Per spec/14 §3.1: G0/G2/G4 fill the chroma+all-inter role;
        // G1/G3/G5 fill the intra-luma role.
        let chroma: Vec<_> = GFamily::ALL
            .iter()
            .filter(|g| g.role() == GRole::ChromaAndInter)
            .collect();
        let luma: Vec<_> = GFamily::ALL
            .iter()
            .filter(|g| g.role() == GRole::IntraLuma)
            .collect();
        assert_eq!(chroma.len(), 3, "chroma+inter role count");
        assert_eq!(luma.len(), 3, "intra-luma role count");
        assert!(chroma.contains(&&GFamily::G0));
        assert!(chroma.contains(&&GFamily::G2));
        assert!(chroma.contains(&&GFamily::G4));
        assert!(luma.contains(&&GFamily::G1));
        assert!(luma.contains(&&GFamily::G3));
        assert!(luma.contains(&&GFamily::G5));
    }

    #[test]
    fn chroma_selector_dispatch_matches_spec_14() {
        // Per spec/14 §3.1: 0 → G2, 1 → G0, 2 → G4.
        assert_eq!(GFamily::for_chroma_selector(0), Some(GFamily::G2));
        assert_eq!(GFamily::for_chroma_selector(1), Some(GFamily::G0));
        assert_eq!(GFamily::for_chroma_selector(2), Some(GFamily::G4));
        // Out-of-range returns None (defence in depth — the picture
        // parser already clamps via unary 1-or-2-bit read).
        for sel in 3u8..=255 {
            assert_eq!(GFamily::for_chroma_selector(sel), None, "sel={sel}");
        }
    }

    #[test]
    fn luma_selector_dispatch_matches_spec_14() {
        // Per spec/14 §3.1: 0 → G3, 1 → G1, 2 → G5.
        assert_eq!(GFamily::for_luma_selector(0), Some(GFamily::G3));
        assert_eq!(GFamily::for_luma_selector(1), Some(GFamily::G1));
        assert_eq!(GFamily::for_luma_selector(2), Some(GFamily::G5));
        for sel in 3u8..=255 {
            assert_eq!(GFamily::for_luma_selector(sel), None, "sel={sel}");
        }
    }

    #[test]
    fn v1_v2_fallthrough_descriptors_are_g4_chroma_g5_luma() {
        // spec/14 §3.1 v1/v2 fallthrough at `0x1c212917` writes
        // [esi+0xab0] = G4 (chroma) and [esi+0xab4] = G5 (luma).
        // Confirm the role assignments line up so a v1/v2 dispatcher
        // can write `for_chroma_selector(2)` / `for_luma_selector(2)`
        // without a version-specific branch.
        assert_eq!(GFamily::for_chroma_selector(2), Some(GFamily::G4));
        assert_eq!(GFamily::for_luma_selector(2), Some(GFamily::G5));
        assert_eq!(GFamily::G4.role(), GRole::ChromaAndInter);
        assert_eq!(GFamily::G5.role(), GRole::IntraLuma);
    }

    #[test]
    fn every_selector_x_role_combination_is_covered() {
        // Cross-check that the chroma selector picks a chroma+inter
        // descriptor and the luma selector picks an intra-luma
        // descriptor, for every valid selector value. A future round
        // that accidentally swapped the dispatch arms would fail here.
        for sel in 0u8..=2 {
            let c = GFamily::for_chroma_selector(sel).expect("chroma sel valid");
            assert_eq!(c.role(), GRole::ChromaAndInter, "chroma sel={sel} -> {c:?}");
            let l = GFamily::for_luma_selector(sel).expect("luma sel valid");
            assert_eq!(l.role(), GRole::IntraLuma, "luma sel={sel} -> {l:?}");
        }
    }

    #[test]
    fn decode_dispatches_to_underlying_g_decoder() {
        // GFamily::decode must agree with the per-G decoders for every
        // idx, including the ESC sentinel and out-of-range cases.
        for g in GFamily::ALL {
            // Spot-check idx 0 (every G starts with (run=0, level=1, last=0)
            // per spec/09 §3-§6 and spec/99 §5.1).
            let zero = g.decode(0).expect("idx 0 decodes");
            assert_eq!(
                zero,
                GSymbol::Token(crate::g_descriptor::GToken {
                    last: false,
                    run: 0,
                    level_mag: 1,
                }),
                "{g:?} idx 0"
            );
            // idx == count_A is ESC for all G.
            assert_eq!(g.decode(g.count_a()), Some(GSymbol::Esc), "{g:?} ESC");
            // idx > count_A is None.
            assert_eq!(g.decode(g.count_a() + 1), None, "{g:?} OOR");
        }
    }

    #[test]
    fn iter_yields_count_a_plus_one_entries_with_single_esc_at_end() {
        for g in GFamily::ALL {
            let entries: Vec<_> = g.iter().collect();
            assert_eq!(
                entries.len(),
                g.count_a() + 1,
                "{g:?} iter length = count_A + ESC"
            );
            // ESC is the trailing entry; nowhere else.
            let esc_positions: Vec<_> = entries
                .iter()
                .enumerate()
                .filter(|(_, (_, s))| matches!(s, GSymbol::Esc))
                .map(|(i, _)| i)
                .collect();
            assert_eq!(esc_positions, vec![g.count_a()], "{g:?} ESC at end only");
            // Every non-ESC idx must agree with `decode(idx)`.
            for (idx, sym) in entries.iter().take(g.count_a()) {
                assert_eq!(g.decode(*idx), Some(*sym), "{g:?} iter idx {idx}");
            }
        }
    }

    #[test]
    fn partition_invariant_holds_for_all_g_families() {
        // The fundamental sub-class invariant per spec/13 §2: every
        // non-ESC token in [0, count_B] has last=false; every non-ESC
        // token in (count_B, count_A) has last=true. This is the
        // termination contract the inner kernel relies on (spec/13 §2
        // tail loop: sub-B token = clean exit).
        for g in GFamily::ALL {
            for idx in 0..g.count_a() {
                let GSymbol::Token(t) = g.decode(idx).expect("non-ESC decode") else {
                    panic!("{g:?} idx {idx} should be a token, not ESC")
                };
                let expected_last = idx > g.count_b();
                assert_eq!(
                    t.last,
                    expected_last,
                    "{g:?} idx {idx}: last mismatch (count_B={})",
                    g.count_b()
                );
            }
        }
    }
}
