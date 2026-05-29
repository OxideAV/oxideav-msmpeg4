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

/// Which sub-class partition an alphabet index falls in, per spec/13 §2.
///
/// The MS-MPEG4 inner kernel routes every non-ESC symbol to one of two
/// sub-classes based purely on its `idx` relative to `count_B` (per
/// spec/13 §2 disassembly at `1c216e2a..1c216e2f`):
///
/// | Range | Size | Class | `last` flag | Kernel behaviour |
/// | ----- | ---- | ----- | ----------- | ---------------- |
/// | `[0, count_B]` | `count_B + 1` | **sub-A** | 0 | continue scan |
/// | `(count_B, count_A)` | `count_A − count_B − 1` | **sub-B** | 1 | terminate |
///
/// This enum names the two classes. ESC (`idx == count_A`) is **not** a
/// sub-class — it is a separate sentinel that triggers the 3-tier ESC
/// body path; [`GFamily::subclass_of`] returns `None` for it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GSubclass {
    /// Sub-class **A** — non-terminating tokens (`last=0`). Span
    /// `[0, count_B]`, size `count_B + 1`. Per spec/13 §2 the inner
    /// kernel continues scanning after a sub-A token.
    A,
    /// Sub-class **B** — terminating tokens (`last=1`). Span
    /// `(count_B, count_A)`, size `count_A − count_B − 1`. Per spec/13
    /// §2 a sub-B token is the clean-exit signal for the block loop.
    B,
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

    /// **Inverse** of [`for_chroma_selector`]: the picture-header
    /// `ac_chroma_sel` value that dispatches to this G-family, or
    /// `None` if this descriptor never fills the chroma+all-inter role.
    /// Per spec/14 §3.1: `G2 → 0, G0 → 1, G4 → 2`; the three intra-luma
    /// descriptors (G1/G3/G5) return `None`.
    ///
    /// [`for_chroma_selector`]: GFamily::for_chroma_selector
    pub const fn chroma_selector(self) -> Option<u8> {
        match self {
            GFamily::G2 => Some(0),
            GFamily::G0 => Some(1),
            GFamily::G4 => Some(2),
            // G1/G3/G5 fill the intra-luma role and have no chroma
            // selector value per spec/14 §3.1.
            GFamily::G1 | GFamily::G3 | GFamily::G5 => None,
        }
    }

    /// **Inverse** of [`for_luma_selector`]: the picture-header
    /// `ac_luma_sel` value that dispatches to this G-family, or `None`
    /// if this descriptor never fills the intra-luma role. Per spec/14
    /// §3.1: `G3 → 0, G1 → 1, G5 → 2`; the three chroma+all-inter
    /// descriptors (G0/G2/G4) return `None`.
    ///
    /// [`for_luma_selector`]: GFamily::for_luma_selector
    pub const fn luma_selector(self) -> Option<u8> {
        match self {
            GFamily::G3 => Some(0),
            GFamily::G1 => Some(1),
            GFamily::G5 => Some(2),
            // G0/G2/G4 fill the chroma+all-inter role and have no luma
            // selector value per spec/14 §3.1.
            GFamily::G0 | GFamily::G2 | GFamily::G4 => None,
        }
    }

    /// Classify an alphabet index into its [`GSubclass`] per spec/13 §2.
    ///
    /// Returns:
    /// * `Some(GSubclass::A)` for `idx ∈ [0, count_B]` — non-terminating
    ///   token (the inner kernel continues scanning).
    /// * `Some(GSubclass::B)` for `idx ∈ (count_B, count_A)` —
    ///   terminating token (clean exit from the block loop).
    /// * `None` for `idx == count_A` (the ESC sentinel) and any `idx >
    ///   count_A` (out of range). ESC is not a sub-class; it triggers
    ///   the 3-tier ESC body path instead of the sub-A/sub-B split.
    ///
    /// This is the table-free, structural form of the
    /// [`partition_invariant_holds_for_all_g_families`] test that
    /// drives the kernel-side `last` flag in [`crate::g_descriptor`] /
    /// [`crate::g_enum`].
    pub const fn subclass_of(self, idx: usize) -> Option<GSubclass> {
        let count_a = self.count_a();
        let count_b = self.count_b();
        if idx <= count_b {
            Some(GSubclass::A)
        } else if idx < count_a {
            Some(GSubclass::B)
        } else {
            // idx == count_A is ESC; idx > count_A is OOR. Neither
            // falls in a sub-class partition.
            None
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

    #[test]
    fn subclass_of_classifies_every_alphabet_index_per_spec_13() {
        // Per spec/13 §2 the partition is:
        //   [0, count_B]            -> sub-A
        //   (count_B, count_A)      -> sub-B
        //   idx == count_A          -> ESC sentinel (None)
        //   idx >  count_A          -> out of range (None)
        for g in GFamily::ALL {
            for idx in 0..g.count_a() {
                let expected = if idx <= g.count_b() {
                    Some(GSubclass::A)
                } else {
                    Some(GSubclass::B)
                };
                assert_eq!(
                    g.subclass_of(idx),
                    expected,
                    "{g:?} idx={idx} (count_B={}, count_A={})",
                    g.count_b(),
                    g.count_a()
                );
            }
            // ESC sentinel maps to None.
            assert_eq!(
                g.subclass_of(g.count_a()),
                None,
                "{g:?} ESC idx not a subclass"
            );
            // Out-of-range maps to None.
            assert_eq!(
                g.subclass_of(g.count_a() + 1),
                None,
                "{g:?} OOR not a subclass"
            );
            assert_eq!(
                g.subclass_of(g.count_a() + 100),
                None,
                "{g:?} far-OOR not a subclass"
            );
        }
    }

    #[test]
    fn subclass_partition_sizes_match_subclass_of_counts() {
        // Cross-check that the partition produced by subclass_of
        // matches the spec/15 §5.2 sub_A / sub_B sizes returned by
        // subclass_a_size / subclass_b_size.
        for g in GFamily::ALL {
            let mut a = 0usize;
            let mut b = 0usize;
            for idx in 0..g.count_a() {
                match g.subclass_of(idx) {
                    Some(GSubclass::A) => a += 1,
                    Some(GSubclass::B) => b += 1,
                    None => panic!("{g:?} idx {idx} should classify as sub-A or sub-B"),
                }
            }
            assert_eq!(a, g.subclass_a_size(), "{g:?} sub-A count");
            assert_eq!(b, g.subclass_b_size(), "{g:?} sub-B count");
        }
    }

    #[test]
    fn subclass_of_agrees_with_decode_last_flag() {
        // Every non-ESC token's `last` flag must match the sub-class
        // returned by subclass_of (sub-A -> last=false, sub-B ->
        // last=true). This is the structural form of the kernel's
        // spec/13 §2 partition test.
        for g in GFamily::ALL {
            for idx in 0..g.count_a() {
                let GSymbol::Token(t) = g.decode(idx).expect("non-ESC decode") else {
                    panic!("{g:?} idx {idx} should be a token")
                };
                let cls = g.subclass_of(idx).expect("in-range idx classifies");
                let expected_last = matches!(cls, GSubclass::B);
                assert_eq!(
                    t.last, expected_last,
                    "{g:?} idx {idx}: subclass {cls:?} disagrees with decode().last"
                );
            }
        }
    }

    #[test]
    fn chroma_selector_inverts_for_chroma_selector() {
        // Inverse of for_chroma_selector for the three chroma+inter
        // descriptors per spec/14 §3.1.
        assert_eq!(GFamily::G2.chroma_selector(), Some(0));
        assert_eq!(GFamily::G0.chroma_selector(), Some(1));
        assert_eq!(GFamily::G4.chroma_selector(), Some(2));
        // Intra-luma descriptors have no chroma selector value.
        assert_eq!(GFamily::G1.chroma_selector(), None);
        assert_eq!(GFamily::G3.chroma_selector(), None);
        assert_eq!(GFamily::G5.chroma_selector(), None);
        // Round-trip: for each chroma+inter descriptor, the inverse must
        // route back through for_chroma_selector to the same descriptor.
        for g in [GFamily::G0, GFamily::G2, GFamily::G4] {
            let sel = g.chroma_selector().expect("chroma+inter has selector");
            assert_eq!(
                GFamily::for_chroma_selector(sel),
                Some(g),
                "{g:?} chroma round-trip"
            );
        }
    }

    #[test]
    fn luma_selector_inverts_for_luma_selector() {
        // Inverse of for_luma_selector for the three intra-luma
        // descriptors per spec/14 §3.1.
        assert_eq!(GFamily::G3.luma_selector(), Some(0));
        assert_eq!(GFamily::G1.luma_selector(), Some(1));
        assert_eq!(GFamily::G5.luma_selector(), Some(2));
        // Chroma+inter descriptors have no luma selector value.
        assert_eq!(GFamily::G0.luma_selector(), None);
        assert_eq!(GFamily::G2.luma_selector(), None);
        assert_eq!(GFamily::G4.luma_selector(), None);
        // Round-trip: for each intra-luma descriptor, the inverse must
        // route back through for_luma_selector to the same descriptor.
        for g in [GFamily::G1, GFamily::G3, GFamily::G5] {
            let sel = g.luma_selector().expect("intra-luma has selector");
            assert_eq!(
                GFamily::for_luma_selector(sel),
                Some(g),
                "{g:?} luma round-trip"
            );
        }
    }

    #[test]
    fn selector_inverses_are_role_exclusive() {
        // A G-family fills exactly one role per spec/14 §3.1, so for
        // any given descriptor exactly one of chroma_selector() /
        // luma_selector() returns Some. The two inverses together form
        // a total surjection from GFamily onto its role's selector value.
        for g in GFamily::ALL {
            let c = g.chroma_selector();
            let l = g.luma_selector();
            assert!(
                c.is_some() ^ l.is_some(),
                "{g:?}: exactly one of chroma_selector / luma_selector must be Some (got {c:?}/{l:?})"
            );
            // The Some-side selector must agree with the role.
            match g.role() {
                GRole::ChromaAndInter => {
                    assert!(c.is_some(), "{g:?} ChromaAndInter role -> chroma selector");
                    assert!(l.is_none(), "{g:?} ChromaAndInter role -> no luma selector");
                }
                GRole::IntraLuma => {
                    assert!(l.is_some(), "{g:?} IntraLuma role -> luma selector");
                    assert!(c.is_none(), "{g:?} IntraLuma role -> no chroma selector");
                }
            }
        }
    }
}
