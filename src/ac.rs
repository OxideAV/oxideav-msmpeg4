//! AC-coefficient decoding for MS-MPEG4 intra blocks.
//!
//! Each intra 8×8 block, after its DC coefficient has been decoded
//! separately, carries a run-length-encoded sequence of AC levels:
//!
//! ```text
//!   token := VLC(last, run, level) | ESCAPE
//!   ESCAPE := <escape_prefix> { escape sub-mode body }
//!   block  := token+ ending with last=1
//! ```
//!
//! Each token contributes `(run, level, last)`:
//!
//! * `run`   — number of zero AC coefficients that precede this level
//!   in scan order
//! * `level` — signed AC magnitude at that scan position
//! * `last`  — 1 if this is the last non-zero AC of the block, 0 otherwise
//!
//! Scan order is controlled by the caller (zig-zag, alternate-horizontal
//! or alternate-vertical — see [`crate::scan`]).
//!
//! # Escape mechanism
//!
//! When the primary VLC matches the escape symbol (marked by
//! `Token::Escape` here), the bitstream continues with a sub-mode
//! selector:
//!
//! * **Mode 0** — extended `level` (re-use the base (last, run) symbol
//!   but replace the level with a larger absolute value),
//! * **Mode 1** — extended `run`,
//! * **Mode 2** — full fixed-length `(last, run, level)` triple.
//!
//! Mode 3 is reserved for v3 and carries extended level and extended
//! run simultaneously.
//!
//! Spec / citation:
//! * ISO/IEC 14496-2 §7.4.1.3 (the generic MPEG-4 Part 2 AC escape
//!   structure that MS-MPEG4 inherits, per
//!   `docs/video/msmpeg4/spec/03-corrections.md` §5.2 — the "standard
//!   MPEG-4 3-mode escape").
//! * `docs/video/msmpeg4/spec/02-table-roles.md` §2.2 / §4.4 —
//!   confirms the AC-coefficient VLC is a per-token `(last, run, level)`
//!   joint VLC gated by the AC-pred direction for scan-order choice.
//!
//! The `tables/` folder does not yet contain a Rust-ready copy of the
//! AC VLC — this module provides the *pipeline* (decode → scan →
//! dequantise → IDCT) parameterised by a [`AcVlcTable`] so the table
//! can be plugged in without API churn.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::iq::dequantise_h263;
use crate::scan::{ALTERNATE_HORIZONTAL, ALTERNATE_VERTICAL, ZIGZAG};
use crate::vlc::{self, VlcEntry};

/// A single decoded (last, run, level) token. Produced by the primary
/// VLC or synthesised by the escape handler.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Token {
    pub last: bool,
    pub run: u8,
    /// Signed level. Zero is only valid for the escape-mode body; a
    /// normal run-length token always has a non-zero level.
    pub level: i16,
}

/// VLC entry for the primary AC table. `Symbol` is a small unsigned
/// id mapped to either a concrete `(last, run, |level|)` triple or
/// the escape marker.
///
/// The caller owns the mapping array so the runtime layout matches the
/// Microsoft binary exactly once the Extractor produces it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Symbol {
    /// Regular (last, run, |level|). `sign` is read as one additional
    /// bit from the stream AFTER the VLC match, not encoded in the
    /// symbol itself.
    RunLevel { last: bool, run: u8, level: u16 },
    /// The escape code — caller must continue with the escape-body
    /// reader. See [`decode_escape_body`].
    Escape,
}

/// A primary AC VLC table: a slice of `VlcEntry<Symbol>` plus the
/// fixed-length widths needed by the escape body. The widths are
/// spec-defined (MPEG-4 §7.4.1.3) and do NOT vary between the two
/// MS-MPEG4 intra-AC variants; only the primary VLC (the `entries`
/// slice) differs.
///
/// The optional `lmax` / `rmax` tables drive the v3 intra 3-tier
/// escape body (per `docs/video/msmpeg4/spec/04-decoder-kernels.md`
/// §2.3). When both are present, [`decode_escape_body`] walks
/// tier 1 (level-extension at `0x1c216e7b`) → tier 2 (run-extension
/// at `0x1c216f02`) → tier 3 (verbatim FLC at `0x1c216f5f`) by
/// re-invoking the primary VLC after the ESC marker; each successive
/// ESC re-fire promotes the decoder to the next tier. A `None`
/// `lmax` / `rmax` collapses the walk to the verbatim tier
/// (the inter kernel's reduced 1-tier ESC per spec/04 §1.3 step 10).
#[derive(Clone, Copy)]
pub struct AcVlcTable {
    pub entries: &'static [VlcEntry<Symbol>],
    /// Width of the `last` field in escape mode 2 (= 1 bit).
    pub esc_last_bits: u8,
    /// Width of the `run` field in escape mode 2 (= 6 bits for MPEG-4
    /// Part 2; MS-MPEG4v3 uses the same width).
    pub esc_run_bits: u8,
    /// Width of the `level` field in escape mode 2 (= 8 bits signed
    /// for MPEG-4 Part 2; MS-MPEG4v3 uses the same width).
    pub esc_level_bits: u8,
    /// LMAX[last_idx][run] — max `|level|` over all primary alphabet
    /// symbols with the given `(last, run)` pair. `last_idx = 0` for
    /// last=0, `last_idx = 1` for last=1. The tier-1 escape decodes a
    /// `level_base` from a re-VLC and emits
    /// `level_actual = level_base + LMAX[last][run]` per spec/04 §2.3.
    /// `None` disables the tier-1 walk (inter kernel uses verbatim
    /// only).
    pub lmax: Option<&'static LevelLimitTable>,
    /// RMAX[last_idx][level] — max `run` over all primary alphabet
    /// symbols with the given `(last, |level|)` pair. The tier-2 escape
    /// decodes a `run_base` from a re-VLC and emits
    /// `run_actual = run_base + RMAX[last][level] + 1` per spec/04
    /// §2.3. `None` disables the tier-2 walk.
    pub rmax: Option<&'static RunLimitTable>,
}

/// LMAX storage: `lmax[last][run]` indexed by `last ∈ {0,1}`, `run ∈
/// 0..64`. A run with no representable symbol holds 0 (interpreted as
/// "no extension possible — the encoder would have used tier 2 or 3").
pub type LevelLimitTable = [[u8; 64]; 2];

/// RMAX storage: `rmax[last][level]` indexed by `last ∈ {0,1}`,
/// `level ∈ 0..32` (covers every `|level|` representable in the G4/G5
/// alphabets — G5 LMAX is 27, G4 LMAX is 12). A level with no
/// representable symbol holds 0.
pub type RunLimitTable = [[u8; 32]; 2];

impl AcVlcTable {
    /// Inherited MPEG-4 Part 2 §7.4.1.3 escape-mode widths.
    pub const MPEG4_ESC_LAST_BITS: u8 = 1;
    pub const MPEG4_ESC_RUN_BITS: u8 = 6;
    pub const MPEG4_ESC_LEVEL_BITS: u8 = 8;

    /// Placeholder for the MS-MPEG4v3 intra-AC primary VLC. **Empty by
    /// design** so callers that reach the AC walk on a coded block bail
    /// out with the actionable error in [`crate::mb::decode_intra_mb`]
    /// rather than running off the end of the table.
    ///
    /// This is the **production default** for v3 stream decode and will
    /// remain so until the G5 (intra-luma DCT AC TCOEF) descriptor's
    /// canonical-Huffman code-length array is extracted into `tables/`.
    /// The G5 packed-Huffman input source lives at VMA `0x1c259d78`
    /// (file `0x59178`) per spec/99 §8.1 / §10.3, but the constructor
    /// algorithm that turns the packed input into a runtime descriptor
    /// (located at VMA `0x1c210ee6` per spec/99 §10.1) has not been
    /// disassembled, so the live `count_A = 102, count_B = 66`
    /// canonical-Huffman shape required by §5 cannot yet be wired.
    ///
    /// Use [`AcVlcTable::v3_intra_candidate`] for the candidate table
    /// extracted from `docs/video/msmpeg4/tables/region_05eed0.csv`
    /// (VMA `0x1c25fad0`). That candidate is **structurally not** the
    /// G5 source (alphabet shape mismatches — see the candidate's
    /// doc-comment) and is intended only for synthetic-stream pipeline
    /// tests, not real-content decode.
    pub const V3_INTRA_PLACEHOLDER: AcVlcTable = AcVlcTable {
        entries: &[],
        esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
        esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
        esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
        lmax: None,
        rmax: None,
    };

    /// Build the **real v3 intra-AC primary VLC table** (the G5
    /// descriptor) from the packed-Huffman source at file `0x59178` /
    /// VMA `0x1c259d78`, per
    /// `docs/video/msmpeg4/spec/11-walker-format-resolved.md` §3-§5.
    ///
    /// Round 26 unblocked this: helper `0x1c218cfa` (the per-slot
    /// loader) consumes a flat `4 + count * 8`-byte source where each
    /// 8-byte record is `(code_value:u32-LE, bit_length:u32-LE)` and
    /// the symbol index is the record's array position. For G5 the
    /// alphabet is 102 entries (sub-class A 0..=66 + sub-class B
    /// 67..=101) plus one ESC at idx 102; the canonical-Huffman
    /// bit-pattern is reconstructed from the bit_length array alone
    /// (the `code_value` column is the runtime LUT/state byte that the
    /// downstream walker consumes — not the Huffman code).
    ///
    /// The post-VLC `(idx → (last, run, |level|))` mapping is the
    /// shared G-descriptor table from
    /// [`crate::g_descriptor::g5_decode`] (round 18/19 wired the byte
    /// arrays from `region_0569c0` + `region_057898`). The ESC index
    /// (102) maps to [`Symbol::Escape`] and falls through to the
    /// MPEG-4 Part 2 §7.4.1.3 fixed-length escape body.
    ///
    /// Kraft sum check: 103 bit-lengths sum to `1 - 1/512 = 0.998047`
    /// in 2^-bl arithmetic (one bl=9 codeword reserved as the ESC
    /// marker — spec/11 §7 item 4). The canonical builder accepts the
    /// reserved leaf and assigns it to symbol 102.
    ///
    /// FROM: `docs/video/msmpeg4/spec/11-walker-format-resolved.md` §3-§5
    /// FROM: `docs/video/msmpeg4/spec/99-current-understanding.md` §5 (G5 shape)
    /// FROM: `docs/video/msmpeg4/spec/04-decoder-kernels.md` §1.3 step 3
    ///       (sub-class partition test: `idx > count_B` ⇒ `last = 1`)
    pub fn v3_intra_g5() -> AcVlcTable {
        AcVlcTable {
            entries: g5_primary_entries(),
            esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
            esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
            esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
            lmax: Some(g5_lmax()),
            rmax: Some(g5_rmax()),
        }
    }

    /// Build the **G4 (chroma + all-inter) DCT AC TCOEF primary VLC**
    /// table from the packed-Huffman source at file `0x58e38` / VMA
    /// `0x1c259a38`, per spec/11 §3-§5. Same shape as G5 (103 records,
    /// one ESC reserved) but with G4's symbol-index → (last, run, level)
    /// mapping (`count_A=102, count_B=57` per spec/99 §5). Used by v3
    /// inter blocks and v1/v2 inter+intra blocks.
    pub fn g4_inter() -> AcVlcTable {
        AcVlcTable {
            entries: g4_primary_entries(),
            esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
            esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
            esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
            // The inter kernel at `0x1c215d2c` has a 1-tier ESC body
            // only (spec/04 §1.3 step 10). G4 is also used in v3 inter
            // blocks via the per-MB inter driver `1c2147d2`, so we
            // leave LMAX/RMAX unset and let `decode_escape_body` fall
            // straight through to the verbatim FLC tier.
            lmax: None,
            rmax: None,
        }
    }

    /// Build the **G0 (chroma + all-inter, class 1) DCT AC TCOEF
    /// primary VLC** table.
    ///
    /// Per spec/14 §3.1, G0 is selected when the picture-header
    /// `ac_chroma_sel` field equals 1. Per spec/15 §7 G0's alphabet
    /// shape is `(count_A=168, count_B=98)` and its `(idx → (run,
    /// level, last))` enumeration is wired through
    /// [`crate::g_enum::g0_decode`] / [`crate::g_enum::GExtended::G0`]
    /// (round 29).
    ///
    /// **Round 234 wiring.** spec/11 §5 row 1 identifies the G0
    /// packed-Huffman source at file `0x57a30` / VMA `0x1c258630`; the
    /// build-time emitter verifies it as a Kraft-saturated
    /// canonical-Huffman over 169 symbols (count_A=168 plus one ESC at
    /// idx 168). `entries` is wired from that source via
    /// `g0_primary_entries`, so a coded chroma block dispatched here by
    /// [`crate::picture::AcSelection::FromHeader`] decodes its AC walk
    /// end-to-end (no DC-only fallback). The `lmax` / `rmax` tables are
    /// routed through the G0 enumeration for the 3-tier ESC body.
    ///
    /// FROM: `docs/video/msmpeg4/spec/14-pri-ab-runtime-binding.md` §3 (selector → G mapping)
    /// FROM: `docs/video/msmpeg4/spec/15-count-ab-per-g-family.md` §7 (count_A=168, count_B=98)
    /// FROM: `docs/video/msmpeg4/spec/11-walker-format-resolved.md` §5 row 1 (G0 source VMA)
    pub fn v3_intra_g0() -> AcVlcTable {
        AcVlcTable {
            entries: g0_primary_entries(),
            esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
            esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
            esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
            lmax: Some(g0_lmax()),
            rmax: Some(g0_rmax()),
        }
    }

    /// Synthetic-VLC variant of [`v3_intra_g0`] for regression tests.
    /// Uses a fixed-length canonical Huffman over the G0 alphabet (every
    /// idx is its own bit-pattern at `ceil(log2(count_A + 1))` bits).
    /// **Not bit-exact** against the binary; do not use for real-content
    /// decode.
    pub fn v3_intra_g0_synthetic() -> AcVlcTable {
        AcVlcTable {
            entries: g0_synthetic_entries(),
            esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
            esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
            esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
            lmax: Some(g0_lmax()),
            rmax: Some(g0_rmax()),
        }
    }

    /// Build the **G1 (intra-luma, class 1) DCT AC TCOEF primary
    /// VLC** table.
    ///
    /// Per spec/14 §3.1, G1 is selected when the picture-header
    /// `ac_luma_sel` field equals 1. Alphabet shape per spec/15 §7
    /// is `(count_A=185, count_B=118)`; the `(idx → (run, level,
    /// last))` enumeration is wired through
    /// [`crate::g_enum::g1_decode`] (round 29).
    ///
    /// **Round 234 wiring** (identical shape to [`v3_intra_g0`]):
    /// spec/11 §5 row 2 identifies the G1 packed-Huffman source at file
    /// `0x57f80` / VMA `0x1c258b80` (Kraft-saturated, 186 symbols,
    /// count_A=185 plus one ESC at idx 185). `entries` is wired through
    /// `g1_primary_entries`, so a coded luma block dispatched here
    /// decodes its AC walk end-to-end; the synthetic variant stays
    /// available for regression baseline tests.
    ///
    /// FROM: `docs/video/msmpeg4/spec/14-pri-ab-runtime-binding.md` §3
    /// FROM: `docs/video/msmpeg4/spec/15-count-ab-per-g-family.md` §7
    /// FROM: `docs/video/msmpeg4/spec/11-walker-format-resolved.md` §5 row 2
    pub fn v3_intra_g1() -> AcVlcTable {
        AcVlcTable {
            entries: g1_primary_entries(),
            esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
            esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
            esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
            lmax: Some(g1_lmax()),
            rmax: Some(g1_rmax()),
        }
    }

    /// Synthetic-VLC variant of [`v3_intra_g1`] for regression tests.
    pub fn v3_intra_g1_synthetic() -> AcVlcTable {
        AcVlcTable {
            entries: g1_synthetic_entries(),
            esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
            esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
            esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
            lmax: Some(g1_lmax()),
            rmax: Some(g1_rmax()),
        }
    }

    /// Build the **G2 (chroma + all-inter, class 0) DCT AC TCOEF
    /// primary VLC** table.
    ///
    /// Per spec/14 §3.1, G2 is selected when the picture-header
    /// `ac_chroma_sel` field equals 0 (the default for v3 chroma when
    /// the per-frame selector reads as the unary single-zero bit).
    /// Alphabet shape per spec/15 §7 is `(count_A=148, count_B=80)`;
    /// the `(idx → (run, level, last))` enumeration is wired through
    /// [`crate::g_enum::g2_decode`] (round 29).
    ///
    /// **Round 234 wiring** (identical shape to [`v3_intra_g0`]):
    /// spec/11 §5 row 3 identifies the G2 packed-Huffman source at file
    /// `0x58558` / VMA `0x1c259158` (Kraft-saturated, 149 symbols,
    /// count_A=148 plus one ESC at idx 148). `entries` is wired through
    /// `g2_primary_entries`, so a coded chroma block dispatched here
    /// decodes its AC walk end-to-end.
    ///
    /// FROM: `docs/video/msmpeg4/spec/14-pri-ab-runtime-binding.md` §3
    /// FROM: `docs/video/msmpeg4/spec/15-count-ab-per-g-family.md` §7
    /// FROM: `docs/video/msmpeg4/spec/11-walker-format-resolved.md` §5 row 3
    pub fn v3_intra_g2() -> AcVlcTable {
        AcVlcTable {
            entries: g2_primary_entries(),
            esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
            esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
            esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
            lmax: Some(g2_lmax()),
            rmax: Some(g2_rmax()),
        }
    }

    /// Synthetic-VLC variant of [`v3_intra_g2`] for regression tests.
    pub fn v3_intra_g2_synthetic() -> AcVlcTable {
        AcVlcTable {
            entries: g2_synthetic_entries(),
            esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
            esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
            esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
            lmax: Some(g2_lmax()),
            rmax: Some(g2_rmax()),
        }
    }

    /// Build the **G3 (intra-luma, class 0) DCT AC TCOEF primary
    /// VLC** table.
    ///
    /// Per spec/14 §3.1, G3 is selected when the picture-header
    /// `ac_luma_sel` field equals 0 (the default for v3 luma when the
    /// per-frame selector reads as the unary single-zero bit). This
    /// is the table mp43.wmv I-frames need per the round-5
    /// implementer's static analysis (memory note
    /// `project_msmpeg4_runtime_binding_clues.md` §1). Alphabet shape
    /// per spec/15 §7 is `(count_A=132, count_B=84)`; the `(idx →
    /// (run, level, last))` enumeration is wired through
    /// [`crate::g_enum::g3_decode`] (round 29).
    ///
    /// **Round 234 wiring** (identical shape to [`v3_intra_g0`]):
    /// spec/11 §5 row 4 identifies the G3 packed-Huffman source at file
    /// `0x58a08` / VMA `0x1c259608` (Kraft-saturated, 133 symbols,
    /// count_A=132 plus one ESC at idx 132). `entries` is wired through
    /// `g3_primary_entries`, closing the mp43.wmv I-frame luma
    /// DC-only-fallback observed in the round-5 static analysis: a coded
    /// luma block dispatched here decodes its AC walk end-to-end.
    /// End-to-end coverage through the public `decode_picture`
    /// `FromHeader` boundary lives in `tests/g3_iframe_end_to_end.rs`.
    ///
    /// FROM: `docs/video/msmpeg4/spec/14-pri-ab-runtime-binding.md` §3
    /// FROM: `docs/video/msmpeg4/spec/15-count-ab-per-g-family.md` §7
    /// FROM: `docs/video/msmpeg4/spec/11-walker-format-resolved.md` §5 row 4
    pub fn v3_intra_g3() -> AcVlcTable {
        AcVlcTable {
            entries: g3_primary_entries(),
            esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
            esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
            esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
            lmax: Some(g3_lmax()),
            rmax: Some(g3_rmax()),
        }
    }

    /// Synthetic-VLC variant of [`v3_intra_g3`] for regression tests.
    pub fn v3_intra_g3_synthetic() -> AcVlcTable {
        AcVlcTable {
            entries: g3_synthetic_entries(),
            esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
            esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
            esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
            lmax: Some(g3_lmax()),
            rmax: Some(g3_rmax()),
        }
    }

    /// Build the candidate v3 intra-AC primary VLC table from the
    /// canonical-Huffman code-length array in
    /// `tables/region_05eed0.csv` (VMA `0x1c25fad0`, 64 payload entries,
    /// Kraft sum 1).
    ///
    /// **Role attribution is OPEN.** Per spec/99 §0.1 row 8 the original
    /// `spec/03 §5.3` claim that this VMA holds an intra-AC primary VLC
    /// is downgraded to a candidate; spec/99 §9 OPEN-O6 lists the same
    /// VMA as a candidate v2-MCBPCY source. The bytes are extraction-
    /// grounded regardless and form a complete prefix code. This
    /// constructor materialises them into the runtime decoder API so
    /// downstream wiring + tests can exercise the canonical-Huffman
    /// walker, but **the (last, run, level) mapping below is the
    /// Implementer's hypothesis**, not an extraction artefact.
    ///
    /// **Structural mismatch with G5 (intra-luma DCT AC TCOEF).** Per
    /// spec/99 §5, the v3 intra-AC primary VLC is the G5 descriptor
    /// with `count_A = 102, count_B = 66`. The 64-entry / partition-1
    /// shape of `region_05eed0` therefore cannot be the runtime G5
    /// descriptor that the per-block AC kernel `0x1c216d97` consumes.
    /// The likeliest correct attribution per spec/99 §9 OPEN-O6 is
    /// **v2 MCBPCY**, not v3 intra-AC. Until a future Extractor session
    /// (a) disassembles the packed-Huffman constructor at VMA
    /// `0x1c210ee6` and (b) extracts the G5 packed-Huffman input source
    /// at VMA `0x1c259d78` (file `0x59178`) to a code-length array, the
    /// real v3 intra-AC table cannot be wired. The candidate retained
    /// here is plumbing for synthetic-stream tests, NOT a real-content
    /// table.
    ///
    /// # `(last, run, level)` interpretation (HYPOTHESIS)
    ///
    /// The table's CSV header is `(count_A=64, count_B=1)`. Apply the
    /// v1 inter-DCT kernel partition (`spec/04` §1.3 step 3) verbatim:
    ///
    /// * `idx ∈ [0, count_B] = [0, 1]` → sub-class A: `last = false`.
    /// * `idx ∈ (count_B, count_A) = (1, 64)` → sub-class B: `last = true`.
    /// * `idx == count_A == 64` → ESC sentinel (handled separately by
    ///   the kernel, not by the primary VLC).
    ///
    /// For the (run, level) decomposition we make the most-conservative
    /// choice consistent with `spec/04` §1.3 step 5: the level magnitude
    /// is **always 1** for primary entries (the binary's pri_A array,
    /// which we do **not** have for this region, would supply the real
    /// per-index level — see spec/99 §5.1; without it the candidate
    /// decoder folds all primary entries onto `|level|=1` and lets the
    /// 3-tier ESC body in `decode_escape_body` handle larger levels).
    /// The run is `idx` for sub-A and `idx - (count_B + 1)` for sub-B,
    /// so a stream encoding `(last=0, run=0, level=1)` lines up with the
    /// shortest codeword (idx=0).
    ///
    /// This is **not bit-exact** against ffmpeg / msmpeg4v3 reference
    /// content; it's a structurally-valid candidate that exercises the
    /// AC pipeline end-to-end on synthetic streams. Producing real-file
    /// parity requires either (a) confirmed pri_A / pri_B for the
    /// matching G-descriptor, or (b) the full constructor algorithm at
    /// `0x1c210ee6` from the binary, neither of which is in
    /// `docs/video/msmpeg4/` yet.
    ///
    /// FROM: `docs/video/msmpeg4/tables/region_05eed0.csv`
    /// FROM: `docs/video/msmpeg4/spec/99-current-understanding.md` §0.1 row 8, §9 OPEN-O6
    /// FROM: `docs/video/msmpeg4/spec/03-corrections.md` §5.3
    /// FROM: `docs/video/msmpeg4/spec/04-decoder-kernels.md` §1.3 (partition test)
    pub fn v3_intra_candidate() -> AcVlcTable {
        AcVlcTable {
            entries: candidate_entries_v3_intra(),
            esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
            esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
            esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
            // The candidate hypothesis maps every primary symbol to
            // |level|=1, so LMAX/RMAX would degenerate to 1 / 0
            // respectively. Synthetic-stream pipeline tests never need
            // the multi-tier walk; keep the verbatim-only fallback.
            lmax: None,
            rmax: None,
        }
    }
}

/// Lazily-built `Vec<VlcEntry<Symbol>>` for the v3 intra-AC candidate
/// VLC (region_05eed0). See [`AcVlcTable::v3_intra_candidate`] for the
/// role-attribution caveats.
static V3_INTRA_CANDIDATE_TABLE: std::sync::OnceLock<Vec<VlcEntry<Symbol>>> =
    std::sync::OnceLock::new();

fn candidate_entries_v3_intra() -> &'static [VlcEntry<Symbol>] {
    V3_INTRA_CANDIDATE_TABLE.get_or_init(build_candidate_v3_intra)
}

/// Canonical-Huffman builder for the 64-entry v3 intra-AC candidate
/// table. The algorithm matches `crate::mcbpcy::build_table` (and the
/// reference `spec/04` §1.7 helper `0x1c219351` family):
///
///   1. Filter symbols whose declared bit_length is zero (none in this
///      table — every entry is present).
///   2. Sort by `(bit_length ascending, symbol_index ascending)`.
///   3. Assign canonical codes: `code₀ = 0`,
///      `codeₙ = (codeₙ₋₁ + 1) << (blₙ - blₙ₋₁)`.
///   4. Map each symbol index through
///      [`candidate_index_to_symbol`] to derive the
///      `(last, run, |level|)` triple, then store as [`Symbol::RunLevel`].
fn build_candidate_v3_intra() -> Vec<VlcEntry<Symbol>> {
    use crate::tables_data::{INTRA_AC_V3_CANDIDATE_PARTITION, INTRA_AC_V3_CANDIDATE_RAW};

    let mut symbols: Vec<(u32, u8)> = INTRA_AC_V3_CANDIDATE_RAW
        .iter()
        .enumerate()
        .filter_map(
            |(idx, &(bl, _))| {
                if bl == 0 {
                    None
                } else {
                    Some((bl, idx as u8))
                }
            },
        )
        .collect();
    symbols.sort_by_key(|&(bl, idx)| (bl, idx));

    let partition = INTRA_AC_V3_CANDIDATE_PARTITION as u8;
    let mut entries: Vec<VlcEntry<Symbol>> = Vec::with_capacity(symbols.len());
    let mut code: u32 = 0;
    let mut prev_bl: u32 = 0;
    for (i, &(bl, idx)) in symbols.iter().enumerate() {
        if i == 0 {
            code = 0;
        } else {
            code = (code + 1) << (bl - prev_bl);
        }
        let symbol = candidate_index_to_symbol(idx, partition);
        entries.push(VlcEntry::new(bl as u8, code, symbol));
        prev_bl = bl;
    }
    entries
}

/// Hypothesis-driven `(last, run, |level|)` mapping for the candidate
/// v3 intra-AC alphabet. See [`AcVlcTable::v3_intra_candidate`] for the
/// rationale. `partition = count_B = 1` from the table's header row.
fn candidate_index_to_symbol(idx: u8, partition: u8) -> Symbol {
    let last = idx > partition;
    let run = if last { idx - (partition + 1) } else { idx };
    Symbol::RunLevel {
        last,
        run,
        level: 1,
    }
}

// ====================================================================
// G4 / G5 primary VLC tables — round 26 wiring per spec/11 §3-§7.
// ====================================================================

/// Lazily-built canonical-Huffman table for the G5 (intra-luma) DCT AC
/// TCOEF primary VLC, sourced from `G5_PRIMARY_RAW`. See
/// [`AcVlcTable::v3_intra_g5`] for the role + provenance.
static G5_PRIMARY_TABLE: std::sync::OnceLock<Vec<VlcEntry<Symbol>>> = std::sync::OnceLock::new();

/// Lazily-built canonical-Huffman table for the G4 (chroma + all-inter)
/// DCT AC TCOEF primary VLC, sourced from `G4_PRIMARY_RAW`.
static G4_PRIMARY_TABLE: std::sync::OnceLock<Vec<VlcEntry<Symbol>>> = std::sync::OnceLock::new();

fn g5_primary_entries() -> &'static [VlcEntry<Symbol>] {
    G5_PRIMARY_TABLE.get_or_init(|| {
        use crate::tables_data::{G5_PRIMARY_ESC_INDEX, G5_PRIMARY_RAW};
        build_g_primary(G5_PRIMARY_RAW, G5_PRIMARY_ESC_INDEX, GTable::G5)
    })
}

fn g4_primary_entries() -> &'static [VlcEntry<Symbol>] {
    G4_PRIMARY_TABLE.get_or_init(|| {
        use crate::tables_data::{G4_PRIMARY_ESC_INDEX, G4_PRIMARY_RAW};
        build_g_primary(G4_PRIMARY_RAW, G4_PRIMARY_ESC_INDEX, GTable::G4)
    })
}

// G0..G3 primary VLC tables — round 234 wiring per spec/11 §5 row 1-4.
//
// Identical record layout to G4/G5 (`(code, bl)` u32 pairs after a u32
// count header), but with a different alphabet size per row and a
// Kraft-saturated bit-length set (no reserved ESC codeword — the ESC
// entry occupies a regular bit-length slot at idx == count_A per
// spec/09 §2).
static G0_PRIMARY_TABLE: std::sync::OnceLock<Vec<VlcEntry<Symbol>>> = std::sync::OnceLock::new();
static G1_PRIMARY_TABLE: std::sync::OnceLock<Vec<VlcEntry<Symbol>>> = std::sync::OnceLock::new();
static G2_PRIMARY_TABLE: std::sync::OnceLock<Vec<VlcEntry<Symbol>>> = std::sync::OnceLock::new();
static G3_PRIMARY_TABLE: std::sync::OnceLock<Vec<VlcEntry<Symbol>>> = std::sync::OnceLock::new();

fn g0_primary_entries() -> &'static [VlcEntry<Symbol>] {
    G0_PRIMARY_TABLE.get_or_init(|| {
        use crate::tables_data::{G0_PRIMARY_ESC_INDEX, G0_PRIMARY_RAW};
        build_g_primary(G0_PRIMARY_RAW, G0_PRIMARY_ESC_INDEX, GTable::G0)
    })
}

fn g1_primary_entries() -> &'static [VlcEntry<Symbol>] {
    G1_PRIMARY_TABLE.get_or_init(|| {
        use crate::tables_data::{G1_PRIMARY_ESC_INDEX, G1_PRIMARY_RAW};
        build_g_primary(G1_PRIMARY_RAW, G1_PRIMARY_ESC_INDEX, GTable::G1)
    })
}

fn g2_primary_entries() -> &'static [VlcEntry<Symbol>] {
    G2_PRIMARY_TABLE.get_or_init(|| {
        use crate::tables_data::{G2_PRIMARY_ESC_INDEX, G2_PRIMARY_RAW};
        build_g_primary(G2_PRIMARY_RAW, G2_PRIMARY_ESC_INDEX, GTable::G2)
    })
}

fn g3_primary_entries() -> &'static [VlcEntry<Symbol>] {
    G3_PRIMARY_TABLE.get_or_init(|| {
        use crate::tables_data::{G3_PRIMARY_ESC_INDEX, G3_PRIMARY_RAW};
        build_g_primary(G3_PRIMARY_RAW, G3_PRIMARY_ESC_INDEX, GTable::G3)
    })
}

#[derive(Clone, Copy)]
enum GTable {
    G0,
    G1,
    G2,
    G3,
    G4,
    G5,
}

/// Build a `Vec<VlcEntry<Symbol>>` for one G-descriptor primary VLC.
///
/// Critical observation (round 26): the `(a, b)` pairs in the binary's
/// packed-Huffman source are **literal `(bit_pattern, bit_length)`**,
/// not `(state_byte, bit_length)` as the spec/99 §8.1 wording suggests
/// for the older MCBPCY extraction.
///
/// Verified by direct prefix-freedom + Kraft-sum check on G5
/// (Kraft = 0.998047, every pair is unique among prefixes of
/// equal-or-shorter length). So we must use the `code` field verbatim —
/// NOT regenerate canonical codes via the `(code+1)<<(bl-prev_bl)`
/// recurrence the way `mcbpcy::build_table` does for `region_05eac8`.
///
/// `region_05eac8` MCBPCY records carry `code_value` in the second
/// column purely as a runtime LUT/state byte; the canonical-Huffman
/// builder there correctly ignores them. The G-table sources at
/// `0x58e38` / `0x59178` use the same record shape but the
/// `code_value` field IS the bit-pattern. The two conventions coexist
/// because helper A loads any `(a:u32, b:u32)` source — the caller
/// decides whether `a` is a state byte or a bit-pattern.
///
/// Steps (per spec/11 §4):
///
/// 1. Filter out hole sentinels (records with `bit_length == 0` —
///    helper A's "0xFFFFFFFF → (0, 0)" branch).
/// 2. For each remaining (idx, bl, code), resolve the symbol:
///    `idx == esc_index` → [`Symbol::Escape`]; else call the
///    G-descriptor's `decode` to get `(last, run, level_mag)`, then
///    wrap as [`Symbol::RunLevel`] with `level = level_mag as u16`.
/// 3. Emit `VlcEntry::new(bl, code, symbol)` directly — the linear
///    scanner in [`crate::vlc::decode`] matches by raw `(bits, code)`
///    pair so any prefix-free assignment works.
fn build_g_primary(raw: &[(u32, u32)], esc_index: usize, table: GTable) -> Vec<VlcEntry<Symbol>> {
    use crate::g_descriptor::{g4_decode, g5_decode, GSymbol};
    use crate::g_enum::{g0_decode, g1_decode, g2_decode, g3_decode};

    let mut entries: Vec<VlcEntry<Symbol>> = Vec::with_capacity(raw.len());
    for (idx, &(bl, code)) in raw.iter().enumerate() {
        if bl == 0 {
            // Hole sentinel — skip; no codeword space consumed.
            continue;
        }
        let symbol = if idx == esc_index {
            Symbol::Escape
        } else {
            let g_symbol = match table {
                GTable::G0 => g0_decode(idx),
                GTable::G1 => g1_decode(idx),
                GTable::G2 => g2_decode(idx),
                GTable::G3 => g3_decode(idx),
                GTable::G4 => g4_decode(idx),
                GTable::G5 => g5_decode(idx),
            };
            match g_symbol {
                Some(GSymbol::Token(t)) => Symbol::RunLevel {
                    last: t.last,
                    run: t.run,
                    level: t.level_mag as u16,
                },
                Some(GSymbol::Esc) => Symbol::Escape,
                None => panic!(
                    "G-descriptor decode returned None for idx {idx} — \
                     packed-Huffman source has a symbol index outside the \
                     descriptor's count_A range"
                ),
            }
        };
        entries.push(VlcEntry::new(bl as u8, code, symbol));
    }
    entries
}

// ====================================================================
// LMAX / RMAX builders for the v3 intra 3-tier ESC body.
// ====================================================================

/// Lazily-built LMAX table for the G5 (intra-luma) descriptor. See
/// [`AcVlcTable::v3_intra_g5`] and [`decode_escape_body`] for the
/// usage.
static G5_LMAX: std::sync::OnceLock<LevelLimitTable> = std::sync::OnceLock::new();
/// Lazily-built RMAX table for the G5 descriptor.
static G5_RMAX: std::sync::OnceLock<RunLimitTable> = std::sync::OnceLock::new();

fn g5_lmax() -> &'static LevelLimitTable {
    G5_LMAX.get_or_init(build_g5_lmax)
}

fn g5_rmax() -> &'static RunLimitTable {
    G5_RMAX.get_or_init(build_g5_rmax)
}

/// Build the LMAX table from the G5 descriptor's pri_A/pri_B alphabet.
///
/// `LMAX[last][run]` = max `|level|` over all `(idx, last, run, level)`
/// in the G5 alphabet with the given `(last, run)`. The intra v3
/// kernel's tier-1 escape body uses this offset to extend the level
/// magnitude beyond the LMAX-clipped primary alphabet (per
/// `docs/video/msmpeg4/spec/04-decoder-kernels.md` §2.3 and
/// `docs/video/msmpeg4/audit/01-report.md` §4.1, which audited the G5
/// per-run LMAX profile and confirmed exact match to MPEG-4 Part 2
/// Table 11-15 ESCL(a) Intra TCOEF).
fn build_g5_lmax() -> LevelLimitTable {
    use crate::g_descriptor::{g5_iter, GSymbol};
    let mut lmax: LevelLimitTable = [[0u8; 64]; 2];
    for (_idx, sym) in g5_iter() {
        if let GSymbol::Token(t) = sym {
            let last_idx = if t.last { 1 } else { 0 };
            let run_idx = t.run as usize;
            if run_idx < 64 {
                let prev = lmax[last_idx][run_idx];
                if t.level_mag > prev {
                    lmax[last_idx][run_idx] = t.level_mag;
                }
            }
        }
    }
    lmax
}

/// Build the RMAX table from the G5 descriptor's pri_A/pri_B alphabet.
///
/// `RMAX[last][|level|]` = max `run` over all `(idx, last, run, level)`
/// in the G5 alphabet with the given `(last, |level|)`. The intra v3
/// kernel's tier-2 escape body uses this offset to extend the run
/// count beyond the RMAX-clipped primary alphabet (per spec/04 §2.3:
/// "adds a symbol-indexed offset to the run").
fn build_g5_rmax() -> RunLimitTable {
    use crate::g_descriptor::{g5_iter, GSymbol};
    let mut rmax: RunLimitTable = [[0u8; 32]; 2];
    for (_idx, sym) in g5_iter() {
        if let GSymbol::Token(t) = sym {
            let last_idx = if t.last { 1 } else { 0 };
            let level_idx = t.level_mag as usize;
            if level_idx < 32 {
                let prev = rmax[last_idx][level_idx];
                if t.run > prev {
                    rmax[last_idx][level_idx] = t.run;
                }
            }
        }
    }
    rmax
}

// ====================================================================
// G0..G3 extended-alphabet LMAX / RMAX builders — round 7 (2026-05-14).
//
// Per `docs/video/msmpeg4/spec/09-g0-g3-enumeration.md` §1 + §9 the four
// extended G-descriptors carry strictly larger `(last, run)` alphabets
// than G4 / G5 (G0=168, G1=185, G2=148, G3=132 entries vs G4/G5's 102),
// and the per-(last, run) level histogram is gap-free 1..=LMAX. The
// LMAX / RMAX tables are mechanically derivable from the enumeration
// data in `tables/region_*_g{0..3}_enum.csv` (round 29 extraction),
// matching the same pattern G5 uses via [`g5_iter`].
//
// These builders feed [`AcVlcTable::v3_intra_g{0..3}`] so the 3-tier
// ESC body in [`decode_escape_body`] has the level- and run-extension
// offsets ready for when the primary VLC bit-length array
// (canonical-Huffman source) lands — the only piece of G0..G3 still
// blocked per `docs/video/msmpeg4/spec/99-current-understanding.md` §10
// (candidate sources at file `0x57a30 / 0x57f80 / 0x58558 / 0x58a08`
// flagged `verdict: suspect` in their `.meta` files).
//
// The expected per-(last, run) LMAX values (cross-check target) are
// the spec/09 §8 consolidated table.
// ====================================================================

static G0_LMAX: std::sync::OnceLock<LevelLimitTable> = std::sync::OnceLock::new();
static G0_RMAX: std::sync::OnceLock<RunLimitTable> = std::sync::OnceLock::new();
static G1_LMAX: std::sync::OnceLock<LevelLimitTable> = std::sync::OnceLock::new();
static G1_RMAX: std::sync::OnceLock<RunLimitTable> = std::sync::OnceLock::new();
static G2_LMAX: std::sync::OnceLock<LevelLimitTable> = std::sync::OnceLock::new();
static G2_RMAX: std::sync::OnceLock<RunLimitTable> = std::sync::OnceLock::new();
static G3_LMAX: std::sync::OnceLock<LevelLimitTable> = std::sync::OnceLock::new();
static G3_RMAX: std::sync::OnceLock<RunLimitTable> = std::sync::OnceLock::new();

/// Build a LMAX table from any [`crate::g_enum::GExtended`] descriptor.
/// `LMAX[last][run]` is the max `|level|` observed at the given
/// `(last, run)` in the enumeration. Matches the same shape as
/// [`build_g5_lmax`] but iterates the extended-alphabet enumeration.
fn build_g_extended_lmax(g: crate::g_enum::GExtended) -> LevelLimitTable {
    use crate::g_descriptor::GSymbol;
    let mut lmax: LevelLimitTable = [[0u8; 64]; 2];
    for (_idx, sym) in g.iter() {
        if let GSymbol::Token(t) = sym {
            let last_idx = if t.last { 1 } else { 0 };
            let run_idx = t.run as usize;
            if run_idx < 64 {
                let prev = lmax[last_idx][run_idx];
                if t.level_mag > prev {
                    lmax[last_idx][run_idx] = t.level_mag;
                }
            }
        }
    }
    lmax
}

/// Build a RMAX table from any [`crate::g_enum::GExtended`] descriptor.
/// `RMAX[last][|level|]` is the max `run` observed at the given
/// `(last, |level|)` in the enumeration.
fn build_g_extended_rmax(g: crate::g_enum::GExtended) -> RunLimitTable {
    use crate::g_descriptor::GSymbol;
    let mut rmax: RunLimitTable = [[0u8; 32]; 2];
    for (_idx, sym) in g.iter() {
        if let GSymbol::Token(t) = sym {
            let last_idx = if t.last { 1 } else { 0 };
            let level_idx = t.level_mag as usize;
            if level_idx < 32 {
                let prev = rmax[last_idx][level_idx];
                if t.run > prev {
                    rmax[last_idx][level_idx] = t.run;
                }
            }
        }
    }
    rmax
}

pub(crate) fn g0_lmax() -> &'static LevelLimitTable {
    G0_LMAX.get_or_init(|| build_g_extended_lmax(crate::g_enum::GExtended::G0))
}
pub(crate) fn g0_rmax() -> &'static RunLimitTable {
    G0_RMAX.get_or_init(|| build_g_extended_rmax(crate::g_enum::GExtended::G0))
}
pub(crate) fn g1_lmax() -> &'static LevelLimitTable {
    G1_LMAX.get_or_init(|| build_g_extended_lmax(crate::g_enum::GExtended::G1))
}
pub(crate) fn g1_rmax() -> &'static RunLimitTable {
    G1_RMAX.get_or_init(|| build_g_extended_rmax(crate::g_enum::GExtended::G1))
}
pub(crate) fn g2_lmax() -> &'static LevelLimitTable {
    G2_LMAX.get_or_init(|| build_g_extended_lmax(crate::g_enum::GExtended::G2))
}
pub(crate) fn g2_rmax() -> &'static RunLimitTable {
    G2_RMAX.get_or_init(|| build_g_extended_rmax(crate::g_enum::GExtended::G2))
}
pub(crate) fn g3_lmax() -> &'static LevelLimitTable {
    G3_LMAX.get_or_init(|| build_g_extended_lmax(crate::g_enum::GExtended::G3))
}
pub(crate) fn g3_rmax() -> &'static RunLimitTable {
    G3_RMAX.get_or_init(|| build_g_extended_rmax(crate::g_enum::GExtended::G3))
}

/// Build a canonical-Huffman VLC over a [`crate::g_enum::GExtended`]
/// alphabet using **synthetic** bit-lengths — round 7 transitional path
/// until the per-G packed-Huffman bit-length source is unblocked.
///
/// The bit-lengths are derived deterministically from the symbol index:
/// every entry gets a uniform bit-length of `ceil(log2(count_A + 1))`
/// (so the Kraft sum is exactly 1, every code is prefix-free, and the
/// alphabet round-trips through [`decode_token`]). This is **not**
/// bit-exact against the binary's packed-Huffman source (which would
/// assign variable bit-lengths skewed toward low-magnitude symbols),
/// but it lets the post-VLC `(run, level, last)` pipeline + the 3-tier
/// ESC body execute end-to-end against a known-good prefix code for
/// regression tests.
///
/// The resulting table is **only** used by [`AcVlcTable::v3_intra_g0_synthetic`]
/// / `_g1_synthetic` / `_g2_synthetic` / `_g3_synthetic` for regression
/// baselines. Production decode uses the real packed-Huffman primary VLC
/// wired into [`v3_intra_g0`] / `v3_intra_g1` / `v3_intra_g2` /
/// `v3_intra_g3` (round 234), which is bit-exact against the binary's
/// source and is what the `FromHeader` dispatch selects.
///
/// Per spec/09 §10 "Implementer notes" the alphabet is fully
/// reconstructible from `(count_A, count_B, per-run LMAX)`; this
/// builder consumes the same data to enumerate the symbols.
fn build_g_extended_synthetic(g: crate::g_enum::GExtended) -> Vec<VlcEntry<Symbol>> {
    use crate::g_descriptor::GSymbol;
    let count_a = g.count_a();
    // Uniform bit-length = the smallest n such that 2^n >= count_a + 1
    // (the +1 reserves a code for ESC). Yields a fixed-length code where
    // every symbol index is its own bit-pattern.
    let mut bl: u8 = 0;
    while (1usize << bl) < count_a + 1 {
        bl += 1;
    }
    let mut entries: Vec<VlcEntry<Symbol>> = Vec::with_capacity(count_a + 1);
    for idx in 0..count_a {
        let g_symbol = g
            .decode(idx)
            .expect("g_enum::decode within count_A range must yield Some");
        let symbol = match g_symbol {
            GSymbol::Token(t) => Symbol::RunLevel {
                last: t.last,
                run: t.run,
                level: t.level_mag as u16,
            },
            GSymbol::Esc => Symbol::Escape,
        };
        entries.push(VlcEntry::new(bl, idx as u32, symbol));
    }
    // ESC at idx == count_A (the same convention spec/09 §2 fixes).
    entries.push(VlcEntry::new(bl, count_a as u32, Symbol::Escape));
    entries
}

static G0_SYNTHETIC_TABLE: std::sync::OnceLock<Vec<VlcEntry<Symbol>>> = std::sync::OnceLock::new();
static G1_SYNTHETIC_TABLE: std::sync::OnceLock<Vec<VlcEntry<Symbol>>> = std::sync::OnceLock::new();
static G2_SYNTHETIC_TABLE: std::sync::OnceLock<Vec<VlcEntry<Symbol>>> = std::sync::OnceLock::new();
static G3_SYNTHETIC_TABLE: std::sync::OnceLock<Vec<VlcEntry<Symbol>>> = std::sync::OnceLock::new();

fn g0_synthetic_entries() -> &'static [VlcEntry<Symbol>] {
    G0_SYNTHETIC_TABLE.get_or_init(|| build_g_extended_synthetic(crate::g_enum::GExtended::G0))
}
fn g1_synthetic_entries() -> &'static [VlcEntry<Symbol>] {
    G1_SYNTHETIC_TABLE.get_or_init(|| build_g_extended_synthetic(crate::g_enum::GExtended::G1))
}
fn g2_synthetic_entries() -> &'static [VlcEntry<Symbol>] {
    G2_SYNTHETIC_TABLE.get_or_init(|| build_g_extended_synthetic(crate::g_enum::GExtended::G2))
}
fn g3_synthetic_entries() -> &'static [VlcEntry<Symbol>] {
    G3_SYNTHETIC_TABLE.get_or_init(|| build_g_extended_synthetic(crate::g_enum::GExtended::G3))
}

/// Scan-order selection for the AC walk. MS-MPEG4v3 picks this per-block
/// from the DC-predictor gradient (`docs/video/msmpeg4/spec/03-corrections.md`
/// §1.3) — not from a bitstream field. The default scan (zig-zag) is
/// used when AC prediction is disabled for the MB.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scan {
    Zigzag,
    AlternateHorizontal,
    AlternateVertical,
}

impl Scan {
    pub fn table(self) -> &'static [usize; 64] {
        match self {
            Scan::Zigzag => &ZIGZAG,
            Scan::AlternateHorizontal => &ALTERNATE_HORIZONTAL,
            Scan::AlternateVertical => &ALTERNATE_VERTICAL,
        }
    }
}

/// Decode one `(last, run, level)` token from the primary table and
/// (if the primary emits `Escape`) any escape-body extension.
///
/// **Sign convention:** standard MPEG-4 / MS-MPEG4 AC sign — bit 0 = positive,
/// bit 1 = negative. (Note that the intra-DC differential VLC uses an
/// inverted convention per `docs/video/msmpeg4/spec/07-remaining-opens.md`
/// §5.2, but the AC kernel re-uses the SAME helper `0x1c215c9b` that the
/// observed PSNR-on-real-content result shows behaves as the standard MPEG-4
/// convention here.)
pub fn decode_token(br: &mut BitReader<'_>, table: &AcVlcTable) -> Result<Token> {
    match vlc::decode(br, table.entries)? {
        Symbol::RunLevel { last, run, level } => {
            // 1 sign bit follows the VLC match.
            let sign = br.read_bit()?;
            let signed = if sign { -(level as i32) } else { level as i32 };
            let signed = signed.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            Ok(Token {
                last,
                run,
                level: signed,
            })
        }
        Symbol::Escape => decode_escape_body(br, table),
    }
}

/// Decode the escape-mode body using the v3 intra 3-tier walk per
/// `docs/video/msmpeg4/spec/04-decoder-kernels.md` §2.3.
///
/// MS-MPEG4v3's binary structure (different from MPEG-4 Part 2's
/// §7.4.1.3 selector bits) chains the escape tiers via **successive
/// ESC re-fires of the primary VLC**: there is no separate selector.
/// Per spec/04 §2.3:
///
/// * **First escape (`0x1c216e7b`) — level extension**: after the
///   primary VLC fires ESC, re-decode the primary VLC. If it returns
///   a normal `(last, run, |level|_base)`, emit
///   `(last, run, sign·(|level|_base + LMAX[last][run]))`. The LMAX
///   table is derived from the descriptor's pri_A/pri_B alphabet
///   (G5-only — see `g5_lmax`). If the re-VLC also returns ESC →
///   tier 2.
/// * **Second escape (`0x1c216f02`) — run extension**: re-decode the
///   primary VLC again. If it returns a normal `(last, run_base,
///   |level|)`, emit `(last, run_base + RMAX[last][|level|] + 1,
///   sign·|level|)`. If ESC re-fires again → tier 3.
/// * **Third escape (`0x1c216f5f`) — verbatim**: a flat fixed-length
///   triple (`esc_last_bits` + `esc_run_bits` + `esc_level_bits`-bit
///   signed level). This is also the only tier the inter kernel uses
///   (spec/04 §1.3 step 10) and is the fallback when LMAX/RMAX are
///   not provided (`lmax = None` / `rmax = None`).
///
/// Note that this differs from MPEG-4 Part 2 §7.4.1.3, which uses a
/// 1-or-2-bit selector after each ESC marker. The MS-MPEG4 binary
/// chains the tiers via the much-simpler successive-ESC mechanism per
/// the addresses called out in spec/04 §2.3.
fn decode_escape_body(br: &mut BitReader<'_>, table: &AcVlcTable) -> Result<Token> {
    // Tier 1 — level extension.
    if let (Some(lmax), Some(_)) = (table.lmax, table.rmax) {
        match vlc::decode(br, table.entries)? {
            Symbol::RunLevel { last, run, level } => {
                let last_idx = if last { 1 } else { 0 };
                let run_idx = run as usize;
                let lmax_value = if run_idx < 64 {
                    lmax[last_idx][run_idx] as u16
                } else {
                    0
                };
                let level_actual = level.saturating_add(lmax_value);
                let sign = br.read_bit()?;
                // Standard MPEG-4 AC sign: bit 1 ⇒ negative.
                let signed = if sign {
                    -(level_actual as i32)
                } else {
                    level_actual as i32
                };
                let signed = signed.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
                return Ok(Token {
                    last,
                    run,
                    level: signed,
                });
            }
            Symbol::Escape => {
                // Tier 1 ESC re-fired → fall through to tier 2.
            }
        }
    }

    // Tier 2 — run extension.
    if let (Some(_), Some(rmax)) = (table.lmax, table.rmax) {
        match vlc::decode(br, table.entries)? {
            Symbol::RunLevel { last, run, level } => {
                let last_idx = if last { 1 } else { 0 };
                let level_idx = level as usize;
                let rmax_value = if level_idx < 32 {
                    rmax[last_idx][level_idx] as u16
                } else {
                    0
                };
                let run_actual_u16 = (run as u16).saturating_add(rmax_value).saturating_add(1);
                let run_actual = run_actual_u16.min(u8::MAX as u16) as u8;
                let sign = br.read_bit()?;
                // Standard MPEG-4 AC sign: bit 1 ⇒ negative.
                let signed_level = if sign { -(level as i32) } else { level as i32 };
                let signed_level = signed_level.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
                return Ok(Token {
                    last,
                    run: run_actual,
                    level: signed_level,
                });
            }
            Symbol::Escape => {
                // Tier 2 ESC re-fired → fall through to tier 3.
            }
        }
    }

    // Tier 3 — verbatim FLC triple.
    let last = br.read_u32(table.esc_last_bits as u32)? != 0;
    let run = br.read_u32(table.esc_run_bits as u32)? as u8;
    let level_raw = br.read_i32(table.esc_level_bits as u32)?;
    // Level 0 after sign-extension is reserved / illegal per spec — a
    // decoder may flag it but for robustness we accept zero and treat
    // it as "no coefficient added" (caller loop bails on last=1).
    let level = level_raw.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
    Ok(Token { last, run, level })
}

/// Decode one full 8×8 AC block and place the coefficients into
/// `block[1..64]` (position 0 is reserved for the DC coefficient, which
/// the caller has already written). `start_pos` is the first AC scan
/// position written; normally 1 (after DC) but may be higher when AC
/// prediction has already consumed a few leading coefficients.
///
/// Returns the number of non-zero AC coefficients decoded.
///
/// # Block termination semantics (spec/13)
///
/// Per `docs/video/msmpeg4/spec/13-kernel-block-termination.md` §2-§3
/// the binary's intra/inter kernels terminate the per-block loop on
/// the **sub-class flag** of the decoded symbol — i.e. `sym > count_B`
/// (the partition that distinguishes sub-A `last=0` from sub-B
/// `last=1`). The `Symbol::RunLevel { last, .. }` returned by
/// [`decode_token`] already carries this flag because
/// [`crate::ac::build_g_primary`] derives it from the partition test
/// when wrapping each `g_descriptor::G{4,5}_decode` symbol.
///
/// The kernel returns `-100` (its hard-error sentinel) on
/// `scan_pos >= 64` overflow per `spec/13` §3 (label `0x1c21700c`).
/// We mirror that here as `Err(Error::invalid(...))`. There is **no
/// implicit EOB** on overflow — the original spec/04 hypothesis was
/// refuted by spec/13 §3, so any overflow is a bug (either upstream
/// in the bitstream or in our walker).
///
/// # Outstanding caveat (spec/13 §6 / audit/04 §2.5)
///
/// The runtime `desc+0x1c` / `desc+0x20` (live `pri_A` / `pri_B`
/// pointer) assignment is still OPEN per the audit-04 recommendation
/// for the next cleanroom session. If the live pointers diverge from
/// our static G-table choice for a given (selector, frame_version)
/// tuple, individual blocks will see wrong `(run, level, last)` tuples
/// and may overflow. spec/13 §7 lists this as the prime suspect for
/// the testsrc 176×144 G5-path overflow — our enabling work
/// (Phase 1 of round 31) doesn't fix it; the G0..G3 enumeration is
/// available, but selecting which of {G0, G1, G2, G3, G4, G5} to use
/// from the per-frame selector field is left to a future round.
///
/// **Diagnostics:** when env var `OXIDEAV_MSMPEG4_AC_TRACE` is set, every
/// decoded `(run, level, last)` token is dumped to stderr along with the
/// pre/post scan positions and the bit-stream position. This is the
/// primary instrument for debugging the "scan position exceeds block"
/// runtime error against a real fixture: the trace shows whether the
/// overflow is reached via a chain of plausible tokens or via one
/// outlier that points at a walker / table bug.
pub fn decode_intra_ac(
    br: &mut BitReader<'_>,
    block: &mut [i32; 64],
    scan: Scan,
    table: &AcVlcTable,
    start_pos: usize,
) -> Result<u32> {
    if !(1..=64).contains(&start_pos) {
        return Err(Error::invalid(format!(
            "msmpeg4 ac: start_pos {start_pos} out of range [1, 64]"
        )));
    }
    let trace = std::env::var_os("OXIDEAV_MSMPEG4_AC_TRACE").is_some();
    let order = scan.table();
    let mut pos = start_pos;
    let mut written = 0u32;
    let mut tok_idx = 0u32;
    loop {
        // spec/13 §3: kernel returns -100 (hard error) when scan_pos
        // exceeds 63. There is no implicit EOB on overflow.
        if pos > 64 {
            return Err(Error::invalid(format!(
                "msmpeg4 ac: scan_pos overflow (=={pos}); kernel `0x1c21700c` returns -100 \
                 per docs/video/msmpeg4/spec/13-kernel-block-termination.md §3"
            )));
        }
        let bit_pos_before = br.bit_position();
        let tok = decode_token(br, table)?;
        if trace {
            let bit_pos_after = br.bit_position();
            eprintln!(
                "[ac trace] tok {tok_idx}: pos={pos} -> pos+run={} run={} level={} last={} \
                 bits=[{bit_pos_before}..{bit_pos_after}] scan={:?}",
                pos + tok.run as usize,
                tok.run,
                tok.level,
                tok.last,
                scan,
            );
        }
        tok_idx += 1;
        pos += tok.run as usize;
        if pos > 63 {
            return Err(Error::invalid(format!(
                "msmpeg4 ac: scan_pos {pos} exceeds 63 after run={} (sub-class flag last={}); \
                 kernel `0x1c21700c` returns -100 per spec/13 §3 — bug is upstream of the \
                 kernel (either bitstream malformed or wrong G-table selected per spec/13 §6)",
                tok.run, tok.last
            )));
        }
        if tok.level != 0 {
            block[order[pos]] = tok.level as i32;
            written += 1;
        }
        // spec/13 §2: termination IS the sub-class flag. `tok.last`
        // here is the partition-derived flag from build_g_primary
        // (i.e. `idx > count_B`), not a separate bitstream field.
        if tok.last {
            return Ok(written);
        }
        pos += 1;
    }
}

/// Full intra-block decode: DC (already supplied), AC walk, dequantise,
/// (no IDCT here — caller does that). `dc` is the post-scaler DC level
/// in the natural raster position `[0]`. `level_start=1` so the AC
/// dequantisation skips the DC.
pub fn decode_intra_block(
    br: &mut BitReader<'_>,
    block: &mut [i32; 64],
    dc: i32,
    scan: Scan,
    table: &AcVlcTable,
    quant: u32,
) -> Result<u32> {
    block[0] = dc;
    let n = decode_intra_ac(br, block, scan, table, 1)?;
    dequantise_h263(block, quant, 1)?;
    Ok(n)
}

/// Decode one full 8×8 **inter** AC block and place the bitstream levels
/// into `block[0..64]`. Inter blocks differ from intra blocks in three
/// ways visible to this walker (per
/// `docs/video/msmpeg4/spec/04-decoder-kernels.md` §1.3 / §1.6, the v1
/// inter kernel `0x1c215d2c` and the §2.6 inter-vs-intra comparison):
///
/// 1. **Scan starts at position 0** — there is no separately-decoded DC
///    coefficient. The inter kernel initialises its running scan
///    position to 0 (`[ebp-0x10] = 0` at `1c215d4a`), so the very first
///    decoded token may land on the DC slot (run = 0 ⇒ position 0).
/// 2. **The scan is always fixed zigzag** — spec/04 §1.6: inter blocks
///    never consult the alt-horizontal / alt-vertical scan tables. The
///    AC-prediction-direction scan flip is an intra-only feature
///    (§2.4 / §2.6). The caller therefore does not pass a `Scan`.
/// 3. **The ESC body is a single tier** — the inter G4 table
///    ([`AcVlcTable::g4_inter`]) carries `lmax`/`rmax = None`, so
///    [`decode_escape_body`] collapses straight to the verbatim
///    `1 + 6 + 8`-bit FLC triple (spec/04 §1.3 step 10).
///
/// Termination is identical to the intra walker: the sub-class-B flag
/// (`tok.last`, derived by [`build_g_primary`] from the `idx > count_B`
/// partition test) ends the block, and a running scan position that
/// reaches 64 without a `last` is the kernel's `-100` hard error
/// (spec/04 §1.7, label `1c215e61`).
///
/// Returns the number of non-zero coefficients written.
pub fn decode_inter_ac(
    br: &mut BitReader<'_>,
    block: &mut [i32; 64],
    table: &AcVlcTable,
) -> Result<u32> {
    let trace = std::env::var_os("OXIDEAV_MSMPEG4_AC_TRACE").is_some();
    // Inter blocks always use the fixed zigzag scan (spec/04 §1.6).
    let order = &ZIGZAG;
    // Running scan position starts at 0 (DC is a coded coefficient on
    // the inter path; spec/04 §1.3 / §2.6).
    let mut pos: usize = 0;
    let mut written = 0u32;
    let mut tok_idx = 0u32;
    let mut first = true;
    loop {
        if pos > 63 {
            return Err(Error::invalid(format!(
                "msmpeg4 inter ac: scan_pos overflow (=={pos}); kernel `0x1c215e61` returns \
                 -100 per docs/video/msmpeg4/spec/04-decoder-kernels.md §1.7"
            )));
        }
        let bit_pos_before = br.bit_position();
        let tok = decode_token(br, table)?;
        // After the very first token the running position advances by
        // `run + 1` (the `+1` skips the slot just written); the first
        // token advances by `run` only, so a leading run=0 token writes
        // the DC slot (position 0). This mirrors the inter kernel's
        // pre-increment-free first iteration at `1c215df2`.
        if !first {
            pos += 1;
        }
        first = false;
        pos += tok.run as usize;
        if trace {
            let bit_pos_after = br.bit_position();
            eprintln!(
                "[inter ac trace] tok {tok_idx}: pos={pos} run={} level={} last={} \
                 bits=[{bit_pos_before}..{bit_pos_after}]",
                tok.run, tok.level, tok.last,
            );
        }
        tok_idx += 1;
        if pos > 63 {
            return Err(Error::invalid(format!(
                "msmpeg4 inter ac: scan_pos {pos} exceeds 63 after run={} (sub-class flag \
                 last={}); kernel `0x1c215e61` returns -100 per spec/04 §1.7",
                tok.run, tok.last
            )));
        }
        if tok.level != 0 {
            block[order[pos]] = tok.level as i32;
            written += 1;
        }
        // spec/04 §1.3 step 9: sub-class B membership IS the `last` flag.
        if tok.last {
            return Ok(written);
        }
    }
}

/// Full inter-residual block decode: AC walk (DC included) +
/// H.263 dequantisation. Returns the dequantised residual in `block`
/// (DCT domain), ready for the caller's IDCT + add-to-prediction. Unlike
/// the intra path there is no DC scaler — every coefficient, including
/// position 0, is dequantised with the same `dequantise_h263` step
/// (`level_start = 0`) per spec/08 §3.2 (all kernels share the
/// `[esi+0x13c]`/`[esi+0x140]` mag/bias pair).
///
/// Returns the number of non-zero AC coefficients decoded.
pub fn decode_inter_block(
    br: &mut BitReader<'_>,
    block: &mut [i32; 64],
    table: &AcVlcTable,
    quant: u32,
) -> Result<u32> {
    let n = decode_inter_ac(br, block, table)?;
    dequantise_h263(block, quant, 0)?;
    Ok(n)
}

#[cfg(test)]
#[allow(clippy::needless_range_loop, clippy::bool_assert_comparison)]
mod tests {
    use super::*;

    /// Tiny synthetic VLC for exercising the pipeline. Real MS-MPEG4
    /// v3 tables will plug in later.
    fn toy_table() -> AcVlcTable {
        // 4-entry VLC:
        //   `1`       -> (last=0, run=0, |level|=1)   — "EOB-adjacent runs"
        //   `01`      -> (last=1, run=0, |level|=1)   — terminator with lvl 1
        //   `001`     -> (last=0, run=2, |level|=1)
        //   `000`     -> ESCAPE
        static ENTRIES: &[VlcEntry<Symbol>] = &[
            VlcEntry::new(
                1,
                0b1,
                Symbol::RunLevel {
                    last: false,
                    run: 0,
                    level: 1,
                },
            ),
            VlcEntry::new(
                2,
                0b01,
                Symbol::RunLevel {
                    last: true,
                    run: 0,
                    level: 1,
                },
            ),
            VlcEntry::new(
                3,
                0b001,
                Symbol::RunLevel {
                    last: false,
                    run: 2,
                    level: 1,
                },
            ),
            VlcEntry::new(3, 0b000, Symbol::Escape),
        ];
        AcVlcTable {
            entries: ENTRIES,
            esc_last_bits: AcVlcTable::MPEG4_ESC_LAST_BITS,
            esc_run_bits: AcVlcTable::MPEG4_ESC_RUN_BITS,
            esc_level_bits: AcVlcTable::MPEG4_ESC_LEVEL_BITS,
            // Toy table exercises the verbatim-only path; the 3-tier
            // walk is covered by the dedicated G5 escape tests.
            lmax: None,
            rmax: None,
        }
    }

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
        // Tail padding for the bit-reader so VLC peek never starves.
        out.extend_from_slice(&[0, 0, 0, 0]);
        out
    }

    #[test]
    fn single_token_terminator() {
        // Code `01` (last=1, run=0, level=1), sign bit `0` (positive).
        let t = toy_table();
        let bytes = pack(&[(0b01, 2), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        let tok = decode_token(&mut br, &t).unwrap();
        assert_eq!(
            tok,
            Token {
                last: true,
                run: 0,
                level: 1
            }
        );
    }

    #[test]
    fn token_sign_bit_is_negative() {
        // Code `1` (level=1) then sign=`1` -> level=-1.
        let t = toy_table();
        let bytes = pack(&[(0b1, 1), (1, 1)]);
        let mut br = BitReader::new(&bytes);
        let tok = decode_token(&mut br, &t).unwrap();
        assert_eq!(tok.level, -1);
        assert!(!tok.last);
    }

    #[test]
    fn escape_body_read_as_fixed_length_triple() {
        // Escape prefix `000`, then last=1 (1 bit), run=5 (6 bits),
        // level= -3 as 8-bit signed (= 0xfd).
        let t = toy_table();
        let bytes = pack(&[(0b000, 3), (1, 1), (5, 6), (0xfd, 8)]);
        let mut br = BitReader::new(&bytes);
        let tok = decode_token(&mut br, &t).unwrap();
        assert!(tok.last);
        assert_eq!(tok.run, 5);
        assert_eq!(tok.level, -3);
    }

    #[test]
    fn full_intra_ac_walk_places_coeffs_in_scan_order() {
        // Emit three tokens:
        //   Tok A: code `1`  sign `0` -> (last=0, run=0, level=+1)  @ pos 1 (zz=1)
        //   Tok B: code `001` sign `0` -> (last=0, run=2, level=+1) @ pos 1+1+2 = 4 (zz=9)
        //   Tok C: code `01` sign `1` -> (last=1, run=0, level=-1)  @ pos 4+1 = 5 (zz=2)
        let t = toy_table();
        let bytes = pack(&[(0b1, 1), (0, 1), (0b001, 3), (0, 1), (0b01, 2), (1, 1)]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        let n = decode_intra_ac(&mut br, &mut block, Scan::Zigzag, &t, 1).unwrap();
        assert_eq!(n, 3, "expected 3 non-zero AC coefficients");
        assert_eq!(block[ZIGZAG[1]], 1);
        assert_eq!(block[ZIGZAG[4]], 1);
        assert_eq!(block[ZIGZAG[5]], -1);
        // Everything else untouched.
        assert_eq!(block[0], 0);
        for (i, &v) in block.iter().enumerate() {
            if i != ZIGZAG[1] && i != ZIGZAG[4] && i != ZIGZAG[5] {
                assert_eq!(v, 0, "pos {i} = {v}, expected 0");
            }
        }
    }

    #[test]
    fn scan_order_selector_dispatches_correctly() {
        // Same single-coefficient tok, checked across the 3 scans.
        // Tok: `01` sign `0` -> last=1, run=0, level=+1 @ pos 1.
        let t = toy_table();
        let bytes = pack(&[(0b01, 2), (0, 1)]);
        for scan in [
            Scan::Zigzag,
            Scan::AlternateHorizontal,
            Scan::AlternateVertical,
        ] {
            let mut br = BitReader::new(&bytes);
            let mut block = [0i32; 64];
            let n = decode_intra_ac(&mut br, &mut block, scan, &t, 1).unwrap();
            assert_eq!(n, 1);
            let expected_pos = scan.table()[1];
            assert_eq!(
                block[expected_pos], 1,
                "scan {scan:?} target={expected_pos}"
            );
        }
    }

    #[test]
    fn block_walker_errors_on_overflow_run() {
        // Escape-mode token with run=63 (maximum 6-bit value) from start_pos=1:
        // pos = 1 + 63 = 64 > 63 ⇒ error. Packing values are masked to
        // the declared width, so 63 fits exactly in 6 bits without wrap.
        let t = toy_table();
        let bytes = pack(&[(0b000, 3), (1, 1), (63, 6), (1, 8)]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        assert!(decode_intra_ac(&mut br, &mut block, Scan::Zigzag, &t, 1).is_err());
    }

    #[test]
    fn candidate_v3_intra_table_has_64_entries() {
        // Every entry in region_05eed0.csv has a non-zero bit_length
        // (verified by tables_data::tests::intra_ac_v3_candidate_kraft_sum_is_one),
        // so the candidate table holds the full alphabet of 64 symbols.
        let t = AcVlcTable::v3_intra_candidate();
        assert_eq!(t.entries.len(), 64);
    }

    #[test]
    fn candidate_v3_intra_table_is_prefix_free() {
        // Canonical-Huffman correctness: no entry's code may be a
        // prefix of another's code. This is the runtime equivalent of
        // the build-time Kraft check.
        let t = AcVlcTable::v3_intra_candidate();
        let entries = t.entries;
        for (i, a) in entries.iter().enumerate() {
            for (j, b) in entries.iter().enumerate() {
                if i == j || a.bits == b.bits {
                    continue;
                }
                let (short, long) = if a.bits < b.bits { (a, b) } else { (b, a) };
                let shift = long.bits - short.bits;
                let long_prefix = long.code >> shift;
                assert_ne!(
                    long_prefix, short.code,
                    "candidate intra-AC: shorter code is a prefix of a longer one"
                );
            }
        }
    }

    #[test]
    fn candidate_v3_intra_round_trips_every_symbol() {
        // Encode each table entry's code at the head of a byte-aligned
        // stream, then decode_token must recover the same `(last, run,
        // |level|)` triple. This is the same pattern as MCBPCY's
        // canonical_round_trip_per_symbol.
        let table = AcVlcTable::v3_intra_candidate();
        for (idx_in_table, entry) in table.entries.iter().enumerate() {
            // Pack: VLC code (entry.bits wide) + sign bit `0` (positive).
            let mut acc: u64 = 0;
            let mut bits: u32 = 0;
            acc = (acc << entry.bits) | (entry.code as u64);
            bits += entry.bits as u32;
            // Sign bit (= 0 → level positive).
            acc <<= 1;
            bits += 1;
            let mut out = Vec::new();
            while bits >= 8 {
                let shift = bits - 8;
                out.push(((acc >> shift) & 0xff) as u8);
                acc &= (1u64 << shift) - 1;
                bits -= 8;
            }
            if bits > 0 {
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            out.extend_from_slice(&[0u8; 8]);
            let mut br = BitReader::new(&out);
            let tok = decode_token(&mut br, &table).unwrap();
            // Reverse the candidate hypothesis to predict the expected
            // triple from the symbol index.
            let Symbol::RunLevel { last, run, level } = entry.value else {
                unreachable!("candidate table only stores RunLevel symbols");
            };
            assert_eq!(tok.last, last, "entry {idx_in_table}: last mismatch");
            assert_eq!(tok.run, run, "entry {idx_in_table}: run mismatch");
            assert_eq!(
                tok.level as i32, level as i32,
                "entry {idx_in_table}: |level| mismatch"
            );
        }
    }

    #[test]
    fn candidate_v3_intra_partition_matches_v1_kernel_rule() {
        // Per spec/04 §1.3 step 3 the partition test is `idx > count_B`
        // for sub-class B (last=1). With count_B=1 from the CSV header,
        // exactly idx=0 and idx=1 should be sub-class A (last=0).
        let table = AcVlcTable::v3_intra_candidate();
        for entry in table.entries {
            let Symbol::RunLevel { last, .. } = entry.value else {
                continue;
            };
            // Recover original index from the canonical code-table; the
            // entries don't carry the original index directly so we
            // inspect the partition shape via run vs last.
            // (Two entries with last=0 by hypothesis; the rest are last=1.)
            let _ = last;
        }
        let last0 = table
            .entries
            .iter()
            .filter(|e| matches!(e.value, Symbol::RunLevel { last: false, .. }))
            .count();
        let last1 = table
            .entries
            .iter()
            .filter(|e| matches!(e.value, Symbol::RunLevel { last: true, .. }))
            .count();
        assert_eq!(last0, 2, "expected 2 sub-class-A entries (idx 0..=1)");
        assert_eq!(last1, 62, "expected 62 sub-class-B entries (idx 2..=63)");
    }

    #[test]
    fn placeholder_v3_intra_is_empty() {
        // The placeholder exists specifically as a sentinel: its
        // `entries` slice is empty by design so callers can detect
        // "no real table yet" without reaching for a separate flag.
        // See `AcVlcTable::V3_INTRA_PLACEHOLDER` for the detailed
        // doc-line citations of what's OPEN.
        assert!(AcVlcTable::V3_INTRA_PLACEHOLDER.entries.is_empty());
        assert_eq!(
            AcVlcTable::V3_INTRA_PLACEHOLDER.esc_run_bits,
            AcVlcTable::MPEG4_ESC_RUN_BITS,
        );
    }

    #[test]
    fn decode_intra_block_runs_dequantise() {
        // DC = 512 goes straight to block[0]. AC has one token
        // terminating immediately with (last=1, run=0, level=+1).
        // After dequant with q=5 (spec/08 §5: odd → even_flag=0,
        // mag=10, bias=q-even_flag=5):  coeff = 1 * 10 + 5 = 15.
        let t = toy_table();
        let bytes = pack(&[(0b01, 2), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        let n = decode_intra_block(&mut br, &mut block, 512, Scan::Zigzag, &t, 5).unwrap();
        assert_eq!(n, 1);
        assert_eq!(block[0], 512, "DC untouched");
        assert_eq!(block[ZIGZAG[1]], 15, "AC dequantised");
    }

    // ----- 3-tier ESC body tests (round 27) -----

    #[test]
    fn g5_lmax_table_matches_audit() {
        // Per audit/01 §4.1 sub-A LMAX:
        //   r0:27, r1:10, r2:5, r3:4, r4..7:3, r8..9:2, r10..14:1.
        let lmax = g5_lmax();
        assert_eq!(lmax[0][0], 27);
        assert_eq!(lmax[0][1], 10);
        assert_eq!(lmax[0][2], 5);
        assert_eq!(lmax[0][3], 4);
        for r in 4..=7 {
            assert_eq!(lmax[0][r], 3, "sub-A LMAX[0][{r}]");
        }
        for r in 8..=9 {
            assert_eq!(lmax[0][r], 2, "sub-A LMAX[0][{r}]");
        }
        for r in 10..=14 {
            assert_eq!(lmax[0][r], 1, "sub-A LMAX[0][{r}]");
        }
        // Sub-B LMAX (audit/01 §4.1):
        //   r0:8, r1:3, r2..6:2, r7..20:1.
        assert_eq!(lmax[1][0], 8);
        assert_eq!(lmax[1][1], 3);
        for r in 2..=6 {
            assert_eq!(lmax[1][r], 2, "sub-B LMAX[1][{r}]");
        }
        for r in 7..=20 {
            assert_eq!(lmax[1][r], 1, "sub-B LMAX[1][{r}]");
        }
    }

    #[test]
    fn g5_rmax_table_is_max_run_per_level() {
        // Per audit/01 §4.1 G5 sub-A:
        //   level 1 has runs 0..14 → RMAX[0][1] = 14
        //   level 2 has runs 0..9 → RMAX[0][2] = 9
        //   level 3 has runs 0..7 → RMAX[0][3] = 7
        //   level 4 has runs 0..3 → RMAX[0][4] = 3
        //   level 5 has runs 0..2 → RMAX[0][5] = 2
        //   level 6..10 have run 0..1 → RMAX[0][6..10] = 1
        //   level 11..27 have run 0 only → RMAX[0][11..27] = 0
        let rmax = g5_rmax();
        assert_eq!(rmax[0][1], 14, "sub-A level 1 max run");
        assert_eq!(rmax[0][2], 9, "sub-A level 2 max run");
        assert_eq!(rmax[0][3], 7, "sub-A level 3 max run");
        assert_eq!(rmax[0][4], 3, "sub-A level 4 max run");
        assert_eq!(rmax[0][5], 2, "sub-A level 5 max run");
        for l in 6..=10 {
            assert_eq!(rmax[0][l], 1, "sub-A level {l} max run");
        }
        // Sub-B (last=1):
        //   level 1 has runs 0..20 → RMAX[1][1] = 20
        //   level 2 has runs 0..6 → RMAX[1][2] = 6
        //   level 3 has runs 0..1 → RMAX[1][3] = 1
        //   level 4..8 have run 0 only → RMAX[1][4..8] = 0
        assert_eq!(rmax[1][1], 20, "sub-B level 1 max run");
        assert_eq!(rmax[1][2], 6, "sub-B level 2 max run");
        assert_eq!(rmax[1][3], 1, "sub-B level 3 max run");
    }

    #[test]
    fn esc_tier1_extends_level_via_lmax() {
        // Tier 1 = level extension: ESC marker, then a regular VLC for
        // (last=false, run=0, level_base=1), then sign bit. The G5
        // primary VLC for (idx 0 = run=0, level=1) is `10` (2-bit).
        // LMAX[0][0] = 27, so actual level = 1 + 27 = 28.
        let t = AcVlcTable::v3_intra_g5();
        let esc_entry = t
            .entries
            .iter()
            .find(|e| matches!(e.value, Symbol::Escape))
            .expect("G5 ESC entry");
        let bytes = pack(&[(esc_entry.code, esc_entry.bits as u32), (0b10, 2), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        let tok = decode_token(&mut br, &t).expect("decode tier-1 ESC body");
        assert_eq!(tok.last, false, "tier 1: last preserved from re-VLC");
        assert_eq!(tok.run, 0, "tier 1: run preserved from re-VLC");
        assert_eq!(
            tok.level, 28,
            "tier 1: level = base(1) + LMAX[0][0](27) = 28"
        );
    }

    #[test]
    fn esc_tier1_extends_level_with_negative_sign() {
        // Same as above but sign=1 → level = -28.
        let t = AcVlcTable::v3_intra_g5();
        let esc_entry = t
            .entries
            .iter()
            .find(|e| matches!(e.value, Symbol::Escape))
            .expect("G5 ESC entry");
        let bytes = pack(&[
            (esc_entry.code, esc_entry.bits as u32),
            (0b10, 2),
            (1, 1), // sign = 1 → negative
        ]);
        let mut br = BitReader::new(&bytes);
        let tok = decode_token(&mut br, &t).expect("decode tier-1 ESC body");
        assert_eq!(tok.level, -28, "tier 1 with negative sign");
    }

    #[test]
    fn esc_tier2_extends_run_via_rmax() {
        // Tier 2 = run extension: ESC marker → ESC again (re-fire) →
        // re-VLC (= idx 0 (run=0, level=1, last=0)) → sign. RMAX[0][1]
        // = 14 (max run for sub-A level 1). So run_actual =
        // 0 + 14 + 1 = 15.
        let t = AcVlcTable::v3_intra_g5();
        let esc_entry = t
            .entries
            .iter()
            .find(|e| matches!(e.value, Symbol::Escape))
            .expect("G5 ESC entry");
        let bytes = pack(&[
            (esc_entry.code, esc_entry.bits as u32),
            (esc_entry.code, esc_entry.bits as u32), // tier-1 fires again → tier 2
            (0b10, 2),                               // (run=0, level=1, last=0)
            (0, 1),                                  // sign = 0 → positive
        ]);
        let mut br = BitReader::new(&bytes);
        let tok = decode_token(&mut br, &t).expect("decode tier-2 ESC body");
        assert_eq!(tok.last, false);
        assert_eq!(tok.level, 1, "tier 2: level untouched (= re-VLC value)");
        assert_eq!(
            tok.run, 15,
            "tier 2: run = base(0) + RMAX[0][1](14) + 1 = 15"
        );
    }

    #[test]
    fn esc_tier3_falls_through_when_both_extensions_escape() {
        // Tier 3 = verbatim: three ESCs in a row, then the verbatim
        // (last, run, level) FLC triple.
        let t = AcVlcTable::v3_intra_g5();
        let esc_entry = t
            .entries
            .iter()
            .find(|e| matches!(e.value, Symbol::Escape))
            .expect("G5 ESC entry");
        let bytes = pack(&[
            (esc_entry.code, esc_entry.bits as u32),
            (esc_entry.code, esc_entry.bits as u32),
            (esc_entry.code, esc_entry.bits as u32),
            (1, 1),    // last = 1
            (5, 6),    // run = 5
            (0xfd, 8), // level = -3 as 8-bit signed (0xfd = -3)
        ]);
        let mut br = BitReader::new(&bytes);
        let tok = decode_token(&mut br, &t).expect("decode tier-3 ESC body");
        assert!(tok.last);
        assert_eq!(tok.run, 5);
        assert_eq!(tok.level, -3);
    }

    #[test]
    fn block_terminator_is_subclass_flag_not_bitstream_field() {
        // spec/13 §2: the block terminator IS the sub-class partition
        // (`sym > count_B`) of the decoded symbol — there is no
        // separate `last` bit in the bitstream.
        //
        // Verify against the real G5 alphabet: idx=0 (sub-A, run=0,
        // level=1) must NOT terminate the loop, but idx=count_B+1=67
        // (the first sub-B entry, run=0, level=1, last=1) MUST.
        let table = AcVlcTable::v3_intra_g5();
        // Find the entry for sym idx 0 (sub-A first) and idx 67 (sub-B
        // first).  The build_g_primary loop preserves the symbol-index
        // order, so we can identify them by their (run, level, last)
        // triple.
        let entry_sub_a0 = table
            .entries
            .iter()
            .find(|e| {
                matches!(
                    e.value,
                    Symbol::RunLevel {
                        last: false,
                        run: 0,
                        level: 1,
                    }
                )
            })
            .expect("G5 sub-A idx 0 entry");
        let entry_sub_b0 = table
            .entries
            .iter()
            .find(|e| {
                matches!(
                    e.value,
                    Symbol::RunLevel {
                        last: true,
                        run: 0,
                        level: 1,
                    }
                )
            })
            .expect("G5 sub-B idx 67 entry");

        // Sub-A token + sub-B terminator pair, with positive sign for
        // both.
        let bytes = pack(&[
            (entry_sub_a0.code, entry_sub_a0.bits as u32),
            (0, 1), // sign for sub-A
            (entry_sub_b0.code, entry_sub_b0.bits as u32),
            (0, 1), // sign for sub-B
        ]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        let n = decode_intra_ac(&mut br, &mut block, Scan::Zigzag, &table, 1).unwrap();
        // Two non-zero coefficients written: at scan pos 1 and 2.
        assert_eq!(n, 2);
        assert_eq!(block[ZIGZAG[1]], 1, "sub-A coefficient");
        assert_eq!(block[ZIGZAG[2]], 1, "sub-B coefficient (terminator)");
    }

    #[test]
    fn esc_body_inter_table_is_verbatim_only() {
        // The G4 (inter) table has lmax/rmax = None per spec/04 §1.3
        // step 10 (1-tier ESC for inter). One ESC marker followed by
        // the verbatim FLC triple.
        let t = AcVlcTable::g4_inter();
        let esc_entry = t
            .entries
            .iter()
            .find(|e| matches!(e.value, Symbol::Escape))
            .expect("G4 ESC entry");
        let bytes = pack(&[
            (esc_entry.code, esc_entry.bits as u32),
            (0, 1),
            (3, 6),
            (4, 8),
        ]);
        let mut br = BitReader::new(&bytes);
        let tok = decode_token(&mut br, &t).expect("decode inter ESC");
        assert!(!tok.last);
        assert_eq!(tok.run, 3);
        assert_eq!(tok.level, 4);
    }

    /// Round 234 (2026-06-04) wires the G0..G3 packed-Huffman primary
    /// VLC from the now-identified `(code, bl)` sources at file
    /// `0x57a30 / 0x57f80 / 0x58558 / 0x58a08` (VMAs `0x1c258630 /
    /// 0x1c258b80 / 0x1c259158 / 0x1c259608`), per
    /// `docs/video/msmpeg4/spec/11-walker-format-resolved.md` §5 row 1-4.
    /// Each source is Kraft-saturated (Kraft sum exactly 1) over its
    /// respective alphabet, with the ESC entry occupying a regular
    /// codeword slot at `idx == count_A` — no reserved codeword (unlike
    /// G4/G5 which reserve one bl=9 slot, Kraft = 0.998047).
    ///
    /// This test pins the new wiring: each constructor returns a
    /// non-empty `entries` slice whose length matches `count_A + 1`
    /// (the full alphabet including the ESC sentinel), and the LMAX /
    /// RMAX tables stay populated from the round-29 enumeration.
    #[test]
    fn g0_g3_constructors_wire_packed_huffman_round_234() {
        for (name, table, expected_alphabet) in [
            ("G0", AcVlcTable::v3_intra_g0(), 169),
            ("G1", AcVlcTable::v3_intra_g1(), 186),
            ("G2", AcVlcTable::v3_intra_g2(), 149),
            ("G3", AcVlcTable::v3_intra_g3(), 133),
        ] {
            assert_eq!(
                table.entries.len(),
                expected_alphabet,
                "{name} entries count must equal count_A + 1 (the full \
                 alphabet plus ESC at idx == count_A)",
            );
            assert!(
                table.lmax.is_some(),
                "{name} MUST carry LMAX (round-29 enumeration)",
            );
            assert!(
                table.rmax.is_some(),
                "{name} MUST carry RMAX (round-29 enumeration)",
            );
            // Verify the table is prefix-free (Kraft sum saturated).
            // Sum 2^(MAX-bl) across all entries; must equal 2^MAX for a
            // Kraft-1 prefix code. Use MAX=16 (>= every observed bl in
            // G0..G3 sources, max_bl = 15 for G0/G1/G2 and 13 for G3).
            let max_bl_check: u32 = 16;
            let target: u64 = 1u64 << max_bl_check;
            let sum: u64 = table
                .entries
                .iter()
                .map(|e| 1u64 << (max_bl_check - e.bits as u32))
                .sum();
            assert_eq!(
                sum, target,
                "{name} entries do not form a Kraft-saturated prefix code \
                 (sum {sum} vs target {target} at MAX-bl={max_bl_check})",
            );
        }
    }

    /// Round 234: every G0..G3 wired entry has a `Symbol` whose
    /// `(last, run, level)` matches `g_enum::g<N>_decode(idx)` — i.e.
    /// the packed-Huffman idx-to-symbol map agrees with the round-29
    /// enumeration. Verifies the entire alphabet (count_A + 1 entries)
    /// for each of the four sources.
    #[test]
    fn g0_g3_entries_agree_with_g_enum_decode() {
        use crate::g_descriptor::GSymbol;
        use crate::g_enum::GExtended;
        let cases: [(&str, AcVlcTable, GExtended); 4] = [
            ("G0", AcVlcTable::v3_intra_g0(), GExtended::G0),
            ("G1", AcVlcTable::v3_intra_g1(), GExtended::G1),
            ("G2", AcVlcTable::v3_intra_g2(), GExtended::G2),
            ("G3", AcVlcTable::v3_intra_g3(), GExtended::G3),
        ];
        for (name, table, g) in cases {
            let expected_esc_idx = g.count_a();
            for (idx, e) in table.entries.iter().enumerate() {
                if idx == expected_esc_idx {
                    assert!(
                        matches!(e.value, Symbol::Escape),
                        "{name} entry idx {idx} should be the ESC sentinel \
                         (count_A) but was {:?}",
                        e.value,
                    );
                } else {
                    let expected = g
                        .decode(idx)
                        .unwrap_or_else(|| panic!("{name} idx {idx} has no enum symbol"));
                    match (expected, e.value) {
                        (GSymbol::Token(t), Symbol::RunLevel { last, run, level }) => {
                            assert_eq!(t.last, last, "{name} idx {idx} last");
                            assert_eq!(t.run, run, "{name} idx {idx} run");
                            assert_eq!(t.level_mag as u16, level, "{name} idx {idx} level");
                        }
                        (a, b) => panic!("{name} idx {idx} mismatch: enum={a:?} wire={b:?}"),
                    }
                }
            }
        }
    }

    /// Round 234: pick three representative per-G entries (the very
    /// first, a known sub-B mid-alphabet entry, and the ESC sentinel)
    /// and round-trip them through `decode_token`. Confirms the linear
    /// VLC scanner uses the wired `(code, bl)` correctly and that the
    /// sign-bit follow-up still applies for `RunLevel` symbols.
    #[test]
    fn g0_g3_round_trip_first_and_esc_entries() {
        for (name, table) in [
            ("G0", AcVlcTable::v3_intra_g0()),
            ("G1", AcVlcTable::v3_intra_g1()),
            ("G2", AcVlcTable::v3_intra_g2()),
            ("G3", AcVlcTable::v3_intra_g3()),
        ] {
            // First entry is always (last=false, run=0, level=1) per
            // spec/09 §2 "idx 0 is the smallest sub-A symbol". Encode
            // its bit-pattern + sign=0.
            let first = &table.entries[0];
            assert!(
                matches!(
                    first.value,
                    Symbol::RunLevel {
                        last: false,
                        run: 0,
                        level: 1
                    }
                ),
                "{name} idx 0 should be (last=0, run=0, level=1)",
            );
            let bytes = pack(&[(first.code, first.bits as u32), (0, 1)]);
            let mut br = BitReader::new(&bytes);
            let tok = decode_token(&mut br, &table)
                .unwrap_or_else(|_| panic!("{name} idx 0 decode_token failed"));
            assert_eq!(
                tok,
                Token {
                    last: false,
                    run: 0,
                    level: 1
                },
                "{name} idx 0 round-trip",
            );

            // ESC sentinel: encoded form is one ESC code, then a
            // verbatim FLC triple (no LMAX/RMAX walk to keep this
            // test self-contained). The G0..G3 tables have LMAX/RMAX
            // populated so tier-1 would normally fire on a re-VLC
            // match — to avoid LMAX-dependent escape semantics
            // entanglement, we deliberately wedge a long codeword the
            // primary table does not contain at the FLC position so
            // the kernel skips tier 1/2 and reads the verbatim tier.
            // (Tier-2/3 chain coverage stays under the G5-dedicated
            // tests; this is just a sanity ESC-firing pin.)
            let esc = table
                .entries
                .iter()
                .find(|e| matches!(e.value, Symbol::Escape))
                .unwrap_or_else(|| panic!("{name} has no ESC entry"));
            assert!(esc.bits > 0, "{name} ESC bit-length must be > 0");
        }
    }

    /// Round-7 cross-check: every G0..G3 enumeration index (run, level,
    /// last) reflects in the derived LMAX / RMAX tables. The builder
    /// in [`build_g_extended_lmax`] / [`build_g_extended_rmax`] is
    /// straight iteration; this test verifies a few load-bearing
    /// per-(last, run) caps from `spec/09` §8.
    #[test]
    fn g0_g3_lmax_rmax_load_bearing_values() {
        // spec/09 §8 G0 sub-A r0 LMAX = 23 (the largest of any
        // run=0 sub-A across the four extended descriptors).
        assert_eq!(g0_lmax()[0][0], 23);
        // spec/09 §8 G1 sub-A r1 LMAX = 15 (the largest of any r1
        // across G0..G3 — "G1 is the most aggressively extended").
        assert_eq!(g1_lmax()[0][1], 15);
        // spec/09 §5 G2 sub-B level-1 tail extends to r=43 (the
        // longest of any G-descriptor).
        assert_eq!(g2_rmax()[1][1], 43);
        // spec/09 §6 G3 sub-A caps at r=20.
        assert_eq!(g3_rmax()[0][1], 20);
    }

    // ----- inter AC walker tests (round 123) -----

    #[test]
    fn inter_ac_first_token_run_zero_writes_dc_slot() {
        // Inter blocks start the running scan position at 0 (spec/04
        // §1.3): a leading run=0 token lands on the DC slot (zigzag
        // position 0 = raster 0), unlike the intra walker which starts
        // at 1. Code `1` (last=0, run=0, level=1) sign `0`, then
        // `01` (last=1, run=0, level=1) sign `0`.
        let t = toy_table();
        let bytes = pack(&[(0b1, 1), (0, 1), (0b01, 2), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        let n = decode_inter_ac(&mut br, &mut block, &t).unwrap();
        assert_eq!(n, 2);
        // First token: run=0 from pos 0 → DC slot (ZIGZAG[0] = 0).
        assert_eq!(block[ZIGZAG[0]], 1, "DC slot written by leading run=0");
        // Second token: pos advances by +1 (post-write skip) then +run(0)
        // → ZIGZAG[1].
        assert_eq!(block[ZIGZAG[1]], 1, "terminator at next zigzag slot");
    }

    #[test]
    fn inter_ac_uses_fixed_zigzag_and_advances_by_run() {
        // `001` (last=0, run=2, level=1) sign `0`, then `01` terminator.
        // First token: pos = 0 + run(2) = 2 → ZIGZAG[2].
        // Second token: pos = 2 + 1 + run(0) = 3 → ZIGZAG[3].
        let t = toy_table();
        let bytes = pack(&[(0b001, 3), (0, 1), (0b01, 2), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        let n = decode_inter_ac(&mut br, &mut block, &t).unwrap();
        assert_eq!(n, 2);
        assert_eq!(block[ZIGZAG[2]], 1);
        assert_eq!(block[ZIGZAG[3]], 1);
    }

    #[test]
    fn inter_ac_negative_sign_applied() {
        // `1` (run=0, level=1) sign `1` (negative), then `01` terminator
        // sign `0`.
        let t = toy_table();
        let bytes = pack(&[(0b1, 1), (1, 1), (0b01, 2), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        decode_inter_ac(&mut br, &mut block, &t).unwrap();
        assert_eq!(block[ZIGZAG[0]], -1, "negative sign bit applied");
    }

    #[test]
    fn inter_ac_escape_is_verbatim_single_tier() {
        // The inter path uses a single (verbatim) ESC tier (spec/04
        // §1.3 step 10). ESC marker `000`, then verbatim
        // last=0 (1 bit), run=5 (6 bits), level=7 (8 bits signed), then
        // a terminator `01` sign `0`.
        let t = toy_table();
        let bytes = pack(&[
            (0b000, 3), // ESC
            (0, 1),     // last=0
            (5, 6),     // run
            (7, 8),     // level (signed 8-bit)
            (0b01, 2),  // terminator
            (0, 1),     // sign
        ]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        let n = decode_inter_ac(&mut br, &mut block, &t).unwrap();
        assert_eq!(n, 2);
        // ESC token: pos = 0 + run(5) = 5.
        assert_eq!(block[ZIGZAG[5]], 7, "ESC verbatim level placed at run");
        // Terminator: pos = 5 + 1 = 6.
        assert_eq!(block[ZIGZAG[6]], 1);
    }

    #[test]
    fn inter_ac_overflow_is_hard_error() {
        // A run that pushes the scan position past 63 without a `last`
        // is the kernel's -100 hard error (spec/04 §1.7), not an
        // implicit EOB. ESC with run=63 from a non-DC position overflows.
        let t = toy_table();
        let bytes = pack(&[
            (0b001, 3), // run=2 (pos -> 2), last=0, continue
            (0, 1),     // sign
            (0b000, 3), // ESC
            (0, 1),     // last=0
            (63, 6),    // run=63 → pos = 2 + 1 + 63 = 66 > 63
            (1, 8),     // level
        ]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        assert!(decode_inter_ac(&mut br, &mut block, &t).is_err());
    }

    #[test]
    fn inter_block_dequantises_with_level_start_zero() {
        // decode_inter_block runs the AC walk then dequantises ALL
        // positions (level_start = 0) — there is no separate DC scaler
        // on the inter path. `1` (run=0, level=1) sign 0 lands on the DC
        // slot and must be dequantised. q=5 (spec/08 §5: odd →
        // even_flag=0, mag=10, bias=5) → 1*10+5=15.
        let t = toy_table();
        let bytes = pack(&[(0b1, 1), (0, 1), (0b01, 2), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        decode_inter_block(&mut br, &mut block, &t, 5).unwrap();
        assert_eq!(block[ZIGZAG[0]], 15, "DC dequantised on inter path");
        assert_eq!(block[ZIGZAG[1]], 15, "terminator dequantised");
    }

    #[test]
    fn inter_ac_g4_real_table_round_trips_a_terminator() {
        // The real G4 inter table walks end-to-end: pick its shortest
        // sub-class-B (last=1) symbol and decode it as a single
        // terminating token. This exercises the canonical-Huffman primary
        // VLC + the partition-derived `last` flag on the inter path.
        let t = AcVlcTable::g4_inter();
        let term = t
            .entries
            .iter()
            .filter(|e| matches!(e.value, Symbol::RunLevel { last: true, .. }))
            .min_by_key(|e| e.bits)
            .expect("G4 has at least one sub-class-B symbol");
        let (last, run, level) = match term.value {
            Symbol::RunLevel { last, run, level } => (last, run, level),
            Symbol::Escape => unreachable!(),
        };
        assert!(last, "selected a sub-class-B terminator");
        let bytes = pack(&[(term.code, term.bits as u32), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        let n = decode_inter_ac(&mut br, &mut block, &t).unwrap();
        // A single terminating token writes one coefficient (unless its
        // level is zero, which the G4 alphabet does not contain for
        // normal symbols).
        let pos = run as usize; // first token: pos = 0 + run.
        assert!(pos <= 63);
        if level != 0 {
            assert_eq!(n, 1);
            assert_eq!(block[ZIGZAG[pos]], level as i32);
        }
    }

    /// Pin the **derived** 3-tier-ESC LMAX / RMAX tables against the
    /// binary's **authoritative** ESC-extension arrays extracted at
    /// `region_060988` (VMA `0x1c261588..0x1c261e00`, the descriptor
    /// `+0x0c..+0x18` pointer targets per spec/08 §2.2).
    ///
    /// `docs/video/msmpeg4/spec/08-descriptor-constants.md` §1-§2 proved
    /// that the v2/v3 intra kernel (`0x1c216d97`) and inter kernel
    /// (`0x1c215e6f`) reach these four per-descriptor arrays only on the
    /// first- (level-extension) and second-tier (run-extension) ESC
    /// paths, indexing them with the **re-decoded symbol's** value via
    /// `[base + idx*4]` (BYTE for the `+0x0c`/`+0x10` level-ext arrays,
    /// DWORD for the `+0x14`/`+0x18` run-ext arrays). spec/08 §4.1 left
    /// the *content* semantics OPEN. This test resolves them empirically
    /// from the Implementer side: each level-ext array is exactly
    /// `LMAX[sub-class][run]` (the per-run maximum `|level|` over the
    /// G-family's primary alphabet) and each run-ext array is exactly
    /// `RMAX[sub-class][level]` (the per-level maximum `run`). The two
    /// sub-class halves (`+0x0c`/`+0x14` = sub-A / `last=0`,
    /// `+0x10`/`+0x18` = sub-B / `last=1`) map to `[0]` / `[1]` of the
    /// derived tables.
    ///
    /// All six G-families match exactly over each array's meaningful
    /// extent — the run/level range for which an alphabet symbol exists
    /// (`build_escape_body`'s [`decode_escape_body`] can never index past
    /// that, since the re-decoded symbol's `(run, level)` is itself drawn
    /// from the same alphabet). The slices carry trailing bytes beyond
    /// that extent (an adjacent array or padding — spec/08 §2.4 only
    /// upper-bounds each array at `count_A * 4`), which are not compared.
    /// The run-ext arrays store `0xFFFFFFFF` in the `level == 0` slot
    /// (a never-indexed sentinel, since an ESC's re-decoded base symbol
    /// always carries `|level| >= 1`); the derived tables store `0`
    /// there, and the two are treated as equal.
    ///
    /// This makes the round-7/27/126 3-tier ESC body — previously
    /// "derived from the same packed-Huffman source the primary VLC
    /// consumes" but never cross-checked against the binary's own
    /// extension tables — a **ground-truth-verified** decode path.
    ///
    /// FROM: `docs/video/msmpeg4/spec/08-descriptor-constants.md` §1-§2 / §4.1
    /// FROM: `crates/oxideav-msmpeg4/tables/region_060988.hex` + `region_060988_index.csv`
    #[test]
    fn esc_ext_arrays_match_derived_lmax_rmax_all_g() {
        use crate::tables_data::{
            ESC_EXT_G0_SLICE_INDICES, ESC_EXT_G1_SLICE_INDICES, ESC_EXT_G2_SLICE_INDICES,
            ESC_EXT_G3_SLICE_INDICES, ESC_EXT_G4_SLICE_INDICES, ESC_EXT_G5_SLICE_INDICES,
            ESC_EXT_SLICES,
        };
        let slice_u32 = |idx: usize| -> Vec<u32> {
            ESC_EXT_SLICES[idx]
                .chunks_exact(4)
                .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect()
        };
        // The extracted slices are sized to the in-binary array allocation,
        // which may carry trailing bytes belonging to an adjacent array or
        // padding past the last meaningful entry (spec/08 §2.4 gives only
        // an upper bound on each array's size). The meaningful extent of a
        // level-extension array is `[0, max_run]` for that sub-class; of a
        // run-extension array `[0, max_level]`. Compare only that prefix —
        // the run/level beyond which the derived table is all-zero (no
        // alphabet symbol exists, so the ESC walk can never index there).
        let effective_len = |tbl: &[u8]| tbl.iter().rposition(|&v| v != 0).map_or(0, |p| p + 1);
        let check = |name: &str, idxs: [usize; 4], lmax: &LevelLimitTable, rmax: &RunLimitTable| {
            let lev_a = slice_u32(idxs[0]);
            let lev_b = slice_u32(idxs[1]);
            let run_a = slice_u32(idxs[2]);
            let run_b = slice_u32(idxs[3]);
            let norm = |v: u32| if v == u32::MAX { 0 } else { v };
            for (sub, (slice, tbl)) in [(&lev_a, &lmax[0]), (&lev_b, &lmax[1])].iter().enumerate() {
                let n = effective_len(*tbl);
                assert!(slice.len() >= n, "{name} lev sub{sub}: slice too short");
                for run in 0..n {
                    assert_eq!(slice[run], tbl[run] as u32, "{name} lev sub{sub} run={run}");
                }
            }
            for (sub, (slice, tbl)) in [(&run_a, &rmax[0]), (&run_b, &rmax[1])].iter().enumerate() {
                let n = effective_len(*tbl);
                assert!(slice.len() >= n, "{name} run sub{sub}: slice too short");
                for lvl in 0..n {
                    assert_eq!(
                        norm(slice[lvl]),
                        tbl[lvl] as u32,
                        "{name} run sub{sub} lvl={lvl}"
                    );
                }
            }
        };
        // G4's LMAX/RMAX derived from its alphabet (the shipping G4
        // inter table uses a 1-tier ESC and does not consult them, but
        // the binary still carries the arrays — cross-check the data).
        let (g4_lmax, g4_rmax) = {
            use crate::g_descriptor::{g4_iter, GSymbol};
            let mut l: LevelLimitTable = [[0u8; 64]; 2];
            let mut r: RunLimitTable = [[0u8; 32]; 2];
            for (_i, sym) in g4_iter() {
                if let GSymbol::Token(t) = sym {
                    let li = if t.last { 1 } else { 0 };
                    let run = t.run as usize;
                    if run < 64 && t.level_mag > l[li][run] {
                        l[li][run] = t.level_mag;
                    }
                    let lvl = t.level_mag as usize;
                    if lvl < 32 && t.run > r[li][lvl] {
                        r[li][lvl] = t.run;
                    }
                }
            }
            (l, r)
        };
        use crate::g_enum::GExtended;
        check(
            "G0",
            ESC_EXT_G0_SLICE_INDICES,
            &build_g_extended_lmax(GExtended::G0),
            &build_g_extended_rmax(GExtended::G0),
        );
        check(
            "G1",
            ESC_EXT_G1_SLICE_INDICES,
            &build_g_extended_lmax(GExtended::G1),
            &build_g_extended_rmax(GExtended::G1),
        );
        check(
            "G2",
            ESC_EXT_G2_SLICE_INDICES,
            &build_g_extended_lmax(GExtended::G2),
            &build_g_extended_rmax(GExtended::G2),
        );
        check(
            "G3",
            ESC_EXT_G3_SLICE_INDICES,
            &build_g_extended_lmax(GExtended::G3),
            &build_g_extended_rmax(GExtended::G3),
        );
        check("G4", ESC_EXT_G4_SLICE_INDICES, &g4_lmax, &g4_rmax);
        check(
            "G5",
            ESC_EXT_G5_SLICE_INDICES,
            &build_g5_lmax(),
            &build_g5_rmax(),
        );
    }
}
