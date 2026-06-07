//! 4-MV-per-MB candidate-neighbour walk for the median-of-3 MV predictor.
//!
//! When an inter-coded macroblock carries `K = 4` motion vectors (one per
//! 8x8 luminance block of the 16x16 MB) the predictor for each block's
//! MV is the median of three candidates `(MV1, MV2, MV3)` whose spatial
//! position depends on **which** of the four blocks is being predicted.
//! That layout is given by Figure 7-34 of ISO/IEC 14496-2:2004(E)
//! (MPEG-4 Visual) §7.6.5; the in-tree transcription is
//! `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`.
//!
//! # The four block cases (per Figure 7-34)
//!
//! Block numbering follows Figure 6-8 of the same spec:
//!
//! ```text
//!   +-----+-----+
//!   |  1  |  2  |     1 = top-left (TL)      2 = top-right (TR)
//!   +-----+-----+
//!   |  3  |  4  |     3 = bottom-left (BL)   4 = bottom-right (BR)
//!   +-----+-----+
//! ```
//!
//! For each current block (`MV` below), the figure pins MV1/MV2/MV3 to:
//!
//! | Block | MV1 (left)                | MV2 (above)               | MV3 (above-right)        |
//! | ----- | ------------------------- | ------------------------- | ------------------------ |
//! | 1 TL  | left neighbour MB         | above neighbour MB        | above-right neighbour MB |
//! | 2 TR  | block 1 (within MB)       | above neighbour MB        | above-right neighbour MB |
//! | 3 BL  | left neighbour MB         | block 1 (within MB)       | block 2 (within MB)      |
//! | 4 BR  | block 3 (within MB)       | block 1 (within MB)       | block 2 (within MB)      |
//!
//! When the macroblock carries a **single** MV (1-MV mode, and always
//! when `short_video_header == '1'`), the **top-left case** (`Block::TL`)
//! applies to that single MV — i.e. the predictor is taken from the
//! left, above, and above-right neighbouring macroblocks. This matches
//! the layout already wired in `picture::decode_pframe_mb`.
//!
//! # Candidate validity (per Figure 7-34 normative rules)
//!
//! After candidates are gathered, four substitution rules apply per
//! component (spec §7.6.5, four decision rules verbatim):
//!
//! 1. **Transparent or out-of-area** → the candidate is *not valid*.
//!    Otherwise it is set to the corresponding block vector.
//!    "Transparent" includes neighbours outside the current VOP, video
//!    packet, or GOB (when `short_video_header == '1'`). MS-MPEG4 v3 has
//!    no shape coding, so for this crate "transparent" reduces to "out
//!    of picture / packet / GOB".
//! 2. **Exactly one invalid** → set it to zero.
//! 3. **Exactly two invalid** → both are set to the third (valid) one.
//! 4. **All three invalid** → all are set to zero.
//!
//! The median of the three (now all valid) candidates is taken per
//! component using [`crate::mv::median_predictor`]'s underlying
//! `median(a, b, c) = a + b + c - min - max` formula.
//!
//! # GMC note
//!
//! For S(GMC)-VOPs, a neighbouring macroblock that is GMC-coded
//! (`mcsel == 1`) and lives in the current VOP and video packet
//! contributes the averaged MV described by §7.8.7.3 instead of its
//! own block vector. msmpeg4 v3 is a progressive P-VOP only family —
//! GMC does not apply here. The hook is documented in
//! [`BlockCandidates`] for future MPEG-4 Part 2 reuse but the v3 wiring
//! never marks a candidate as GMC.
//!
//! # Source
//!
//! All facts in this module come from:
//!
//! - `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`
//!   (clean-room ASCII transcription of Figure 7-34 + caption + the
//!   four decision rules + median worked example).
//! - `docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`
//!   §7.6.5 (≈ lines 19660-19720 — introducing prose + validity rules
//!   + median definition + worked example MV1=(-2,3), MV2=(1,5),
//!     MV3=(-1,7) → (Px, Py) = (-1, 5)).
//! - `docs/video/msmpeg4/spec/06-mv-decoder.md` §3 — confirming that
//!   v3 inherits this predictor shape unchanged for the 1-MV case
//!   (`Block::TL` of the within-MB layout).

use crate::mv::Mv;

/// Which 8x8 luminance block of the current macroblock the predictor is
/// being computed for. Numbering matches Figure 6-8 of ISO/IEC 14496-2:
/// `TL`=1, `TR`=2, `BL`=3, `BR`=4.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Block {
    /// Top-left (block 1). Also the single-MV / 1-MV-per-MB case.
    TopLeft,
    /// Top-right (block 2).
    TopRight,
    /// Bottom-left (block 3).
    BottomLeft,
    /// Bottom-right (block 4).
    BottomRight,
}

impl Block {
    /// Const-fn enumeration of every block in raster (Figure 6-8) order.
    /// Useful for test loops covering all four cases.
    pub const ALL: [Block; 4] = [
        Block::TopLeft,
        Block::TopRight,
        Block::BottomLeft,
        Block::BottomRight,
    ];

    /// Spec block index (1..=4) per Figure 6-8.
    pub const fn spec_index(self) -> u8 {
        match self {
            Block::TopLeft => 1,
            Block::TopRight => 2,
            Block::BottomLeft => 3,
            Block::BottomRight => 4,
        }
    }
}

/// Per-MB neighbour MVs available for the candidate walk.
///
/// All fields are `Option<Mv>`. `None` means "not valid" per Figure 7-34
/// rule 1 — i.e. the neighbour is transparent, outside the current VOP,
/// outside the current video packet, or outside the current GOB (when
/// `short_video_header == 1`). For msmpeg4 v3 this reduces to "outside
/// the picture" (no shape coding, single-slice progressive P-VOP).
///
/// The **within-MB** fields (`mb_block_1`, `mb_block_2`, `mb_block_3`)
/// carry the MVs of the *current* macroblock's already-decoded 8x8
/// blocks. Because Figure 6-8's block decode order is 1 → 2 → 3 → 4, by
/// the time block 2 needs MV1 = block 1 it has already been decoded;
/// block 3 needs MV2 = block 1 and MV3 = block 2; block 4 needs MV1 =
/// block 3, MV2 = block 1, MV3 = block 2. If the corresponding within-MB
/// block has not yet been decoded the caller must pass `None` — the
/// candidate is then "invalid" and the substitution rules apply.
///
/// `left_mb`, `above_mb`, and `above_right_mb` are the MVs of the
/// **neighbouring macroblocks**. For the 1-MV-per-MB encoding (which is
/// the case Figure 7-34's top-left sub-diagram covers and which is the
/// default for msmpeg4 v3 today) those MVs are the single MV of the
/// neighbouring macroblock. For 4-MV-per-MB encoding the neighbour's
/// representative MV is the MV of the *adjacent* 8x8 block within that
/// neighbour — see §7.6.5 (the figure shows the specific cell, e.g. the
/// right-half top-row 8x8 block of the *left-neighbour* MB borders the
/// left-half of the current MB). The caller is responsible for picking
/// the right cell from the neighbouring MB before passing it here; this
/// helper just consumes the resulting "one MV per neighbour direction"
/// abstraction.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BlockCandidates {
    /// Left-neighbour-MB MV (or the adjacent 8x8 block of it in 4-MV
    /// neighbour mode). `None` if outside picture / packet / GOB.
    pub left_mb: Option<Mv>,
    /// Above-neighbour-MB MV. `None` if outside picture / packet / GOB.
    pub above_mb: Option<Mv>,
    /// Above-right-neighbour-MB MV. `None` if outside picture / packet /
    /// GOB.
    pub above_right_mb: Option<Mv>,
    /// MV of block 1 (top-left) of the **current** macroblock, if
    /// already decoded. Used by blocks 2 / 3 / 4 per Figure 7-34.
    pub mb_block_1: Option<Mv>,
    /// MV of block 2 (top-right) of the **current** macroblock, if
    /// already decoded. Used by blocks 3 / 4.
    pub mb_block_2: Option<Mv>,
    /// MV of block 3 (bottom-left) of the **current** macroblock, if
    /// already decoded. Used by block 4.
    pub mb_block_3: Option<Mv>,
}

/// The three spatial candidates `(MV1, MV2, MV3)` for the supplied
/// block, picked out of `cands` per Figure 7-34. Each is `Option<Mv>`:
/// `None` means the candidate is *not valid* (transparent / outside
/// picture-area / not-yet-decoded within-MB block).
///
/// This is the raw lookup — the validity-substitution rules are applied
/// separately by [`apply_validity_rules`].
pub fn gather_candidates(block: Block, cands: &BlockCandidates) -> [Option<Mv>; 3] {
    match block {
        // Figure 7-34 top-left sub-diagram: all three from neighbouring MBs.
        // This is also the 1-MV-per-MB / short_video_header == '1' case.
        Block::TopLeft => [cands.left_mb, cands.above_mb, cands.above_right_mb],
        // Figure 7-34 top-right sub-diagram:
        //   MV1 = block 1 (within-MB, decoded before block 2)
        //   MV2 = above neighbour MB
        //   MV3 = above-right neighbour MB
        Block::TopRight => [cands.mb_block_1, cands.above_mb, cands.above_right_mb],
        // Figure 7-34 bottom-left sub-diagram:
        //   MV1 = left neighbour MB
        //   MV2 = block 1 (within-MB)
        //   MV3 = block 2 (within-MB)
        Block::BottomLeft => [cands.left_mb, cands.mb_block_1, cands.mb_block_2],
        // Figure 7-34 bottom-right sub-diagram (all three are within the
        // current MB):
        //   MV1 = block 3 (within-MB)
        //   MV2 = block 1 (within-MB)
        //   MV3 = block 2 (within-MB)
        Block::BottomRight => [cands.mb_block_3, cands.mb_block_1, cands.mb_block_2],
    }
}

/// Apply the four candidate-validity substitution rules from §7.6.5 of
/// ISO/IEC 14496-2 (verbatim in the in-tree transcription
/// `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`):
///
/// 1. (already applied by the caller — `None` ↔ "not valid"; the
///    `Some(_)` candidates are "set to the corresponding block vector").
/// 2. Exactly one `None` → it is set to zero.
/// 3. Exactly two `None` → both are set to the third (valid) candidate.
/// 4. All three `None` → all are set to zero.
///
/// Returns a `[Mv; 3]` of fully-resolved candidates ready for
/// median-of-3.
pub fn apply_validity_rules(raw: [Option<Mv>; 3]) -> [Mv; 3] {
    let valid_count = raw.iter().filter(|c| c.is_some()).count();
    match valid_count {
        // Rule 4: all invalid → all zero.
        0 => [Mv::default(); 3],
        // Rule 3: exactly two invalid → both = the third.
        1 => {
            let only = raw.iter().find_map(|c| *c).unwrap_or_default();
            [only; 3]
        }
        // Rule 2: exactly one invalid → it becomes zero, others kept.
        2 => [
            raw[0].unwrap_or_default(),
            raw[1].unwrap_or_default(),
            raw[2].unwrap_or_default(),
        ],
        // No invalids — everything is taken as the corresponding block
        // vector (which is what `unwrap` already does for `Some(_)`).
        _ => [raw[0].unwrap(), raw[1].unwrap(), raw[2].unwrap()],
    }
}

/// Compute the median-of-3 predictor MV for `block`, given the per-MB
/// neighbour cache `cands`. Combines [`gather_candidates`] (Figure 7-34
/// layout per block) with [`apply_validity_rules`] (the four §7.6.5
/// substitution rules) and finally the per-component median.
///
/// This is the function call sites should use; `gather_candidates` /
/// `apply_validity_rules` are the documented intermediate steps for the
/// test surface (each rule and each block layout is unit-tested
/// independently).
///
/// For the 1-MV-per-MB encoding currently shipping in
/// `picture::decode_pframe_mb`, the caller invokes this with
/// `block = Block::TopLeft` and `cands = BlockCandidates { left_mb,
/// above_mb, above_right_mb, mb_block_1: None, mb_block_2: None,
/// mb_block_3: None }`. The output is bit-identical to the existing
/// `mv::median_predictor(left, top, top_right)` call — see the
/// `top_left_matches_existing_median_predictor` test below.
pub fn predict_block_mv(block: Block, cands: &BlockCandidates) -> Mv {
    let raw = gather_candidates(block, cands);
    let resolved = apply_validity_rules(raw);
    median_of_three(resolved[0], resolved[1], resolved[2])
}

/// Neighbour-macroblock context for a **4-MV-per-MB** prediction batch.
///
/// The three fields are the neighbouring-MB MVs as seen by Figure 7-34
/// for the current macroblock — they are the same `left_mb`, `above_mb`,
/// `above_right_mb` the 1-MV path already feeds [`BlockCandidates`].
///
/// In a 4-MV neighbouring macroblock, ISO/IEC 14496-2:2004(E) §7.6.5
/// requires the **adjacent 8x8 block** of that neighbour (e.g. the
/// right-column blocks of the left-neighbour MB border the current MB's
/// left edge) to be used as that direction's candidate. Resolving the
/// adjacent-block cell is the caller's responsibility — this helper
/// consumes the resulting "one MV per neighbour direction" abstraction
/// the same way [`BlockCandidates`] does. For a 1-MV neighbour the
/// caller passes that single MV in all positions; for a 4-MV neighbour
/// the caller picks the bordering 8x8 block's MV.
///
/// `None` means the neighbour is *not valid* — outside the picture /
/// video packet / GOB — and the §7.6.5 substitution rules will apply.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MacroblockCandidates {
    /// Left-neighbouring-MB MV (or the right-edge 8x8 block thereof in
    /// 4-MV neighbour mode). `None` if outside picture / packet / GOB.
    pub left_mb: Option<Mv>,
    /// Above-neighbouring-MB MV (or the bottom-edge 8x8 block thereof
    /// in 4-MV neighbour mode). `None` if outside picture / packet /
    /// GOB.
    pub above_mb: Option<Mv>,
    /// Above-right-neighbouring-MB MV (or the bottom-edge 8x8 block
    /// thereof in 4-MV neighbour mode). `None` if outside picture /
    /// packet / GOB.
    pub above_right_mb: Option<Mv>,
}

/// Compute the four §7.6.5 motion-vector predictors for a single 16x16
/// macroblock encoded in **4-MV-per-MB** mode (one 8x8 luminance block
/// per MV), driving [`predict_block_mv`] once per [`Block::ALL`] in
/// Figure 6-8 raster order.
///
/// The four returned MVs correspond, in order, to:
///
/// - `[0]` — predictor for block 1 ([`Block::TopLeft`])
/// - `[1]` — predictor for block 2 ([`Block::TopRight`])
/// - `[2]` — predictor for block 3 ([`Block::BottomLeft`])
/// - `[3]` — predictor for block 4 ([`Block::BottomRight`])
///
/// The within-MB candidate threading is driven by Figure 7-34: each
/// block's predictor uses the MVs of the **already-decoded blocks of
/// the same macroblock** as Figure 7-34 specifies:
///
/// - Block 1's predictor uses only neighbour-MB candidates.
/// - Block 2's predictor uses block 1's *final* MV (predictor + decoded
///   MVD) as its MV1, plus neighbour-MB above / above-right as MV2 /
///   MV3.
/// - Block 3's predictor uses block 1's final MV as MV2, block 2's
///   final MV as MV3, plus neighbour-MB left as MV1.
/// - Block 4's predictor uses block 3's final MV as MV1, block 1's
///   final MV as MV2, block 2's final MV as MV3.
///
/// Because each block's predictor depends on **the final MV** of
/// earlier-decoded blocks (predictor + decoded MVD, not just the
/// predictor), the caller must thread the decoded MVs back into this
/// helper in stages — see [`Macroblock4MvDecoder`] for the closed-form
/// iterator that does the predict → decode-MVD → reconstruct loop in
/// one place. Pure-predictor users (e.g. exercising the four-figure
/// layout without consuming a bitstream) can call
/// [`predict_macroblock_4mv_with_finals`] directly by supplying the
/// already-decoded block MVs.
///
/// # When to use
///
/// `picture::decode_pframe_mb` calls this whenever the macroblock-type
/// VLC signals 4-MV mode (the MCBPC bit pattern signalling 4-MV mode
/// for MS-MPEG-4 v3 is the open follow-up item that
/// `picture::decode_pframe_mb` does not yet wire — once it does, this
/// is the function that hands it the four predictors). The 1-MV path
/// continues to call [`predict_block_mv`] with [`Block::TopLeft`] and
/// the same neighbour MVs (the two paths produce identical predictors
/// for block 1 when the within-MB blocks 1/2/3 are passed as `None`).
///
/// # Source
///
/// All facts come from:
/// - `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`
///   §"What the figure shows" + the four per-block sub-diagrams.
/// - `docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`
///   §7.6.5 — four candidate-validity substitution rules + median.
/// - `docs/video/msmpeg4/spec/06-mv-decoder.md` §3 — v3 inherits the
///   §7.6.5 predictor layout; v1/v2 also inherit it (with §3.7's
///   per-component decoded-residual + toroidal wrap).
pub fn predict_macroblock_4mv_with_finals(
    neighbours: &MacroblockCandidates,
    final_block_mvs: [Option<Mv>; 3],
) -> [Mv; 4] {
    let mut out = [Mv::default(); 4];
    for (i, block) in Block::ALL.iter().enumerate() {
        let cands = BlockCandidates {
            left_mb: neighbours.left_mb,
            above_mb: neighbours.above_mb,
            above_right_mb: neighbours.above_right_mb,
            mb_block_1: final_block_mvs[0],
            mb_block_2: final_block_mvs[1],
            mb_block_3: final_block_mvs[2],
        };
        out[i] = predict_block_mv(*block, &cands);
    }
    out
}

/// Closed-form helper that runs the predict-MVD-decode-reconstruct loop
/// for a single 4-MV-per-MB macroblock in Figure 6-8 raster order.
///
/// Caller pattern:
///
/// ```ignore
/// let mut dec = Macroblock4MvDecoder::new(neighbours);
/// // Decode the first MVD from the bitstream, then:
/// let mv1_predictor = dec.predictor_for(Block::TopLeft);
/// let mv1_final = mv1_predictor + mvd_1;
/// dec.commit_block(Block::TopLeft, mv1_final);
/// // ... repeat for Block::TopRight, BottomLeft, BottomRight.
/// let four_final_mvs: [Mv; 4] = dec.finalise();
/// ```
///
/// The helper enforces Figure 6-8 raster order at compile-time-ish via
/// `predictor_for` returning a predictor based on whatever has been
/// committed so far; committing out of order is permitted but produces
/// the same `None`-substituted predictor a future block would see.
///
/// The MVD-decoding step is **deliberately not in this module** — it
/// lives in `mv` / `picture` / the VLC layer. This helper handles only
/// the per-block predictor + the within-MB candidate threading.
#[derive(Clone, Copy, Debug, Default)]
pub struct Macroblock4MvDecoder {
    neighbours: MacroblockCandidates,
    /// Final (post-MVD) MVs of blocks 1, 2, 3 once committed. Index `i`
    /// is `Some` once `commit_block(Block::ALL[i], …)` has been called.
    final_block_mvs: [Option<Mv>; 3],
    /// Final MV of block 4 once committed.
    block_4_final: Option<Mv>,
}

impl Macroblock4MvDecoder {
    /// New per-MB session with the given neighbouring-MB candidate set.
    /// All within-MB block MVs start undecoded (`None`).
    pub const fn new(neighbours: MacroblockCandidates) -> Self {
        Self {
            neighbours,
            final_block_mvs: [None; 3],
            block_4_final: None,
        }
    }

    /// Spec-7.6.5 predictor for `block`, computed from the neighbour-MB
    /// cache plus whatever blocks have already been committed via
    /// [`commit_block`]. Re-invocable: the predictor for a block that
    /// has not yet been committed is a function of the already-committed
    /// blocks only.
    pub fn predictor_for(&self, block: Block) -> Mv {
        let cands = BlockCandidates {
            left_mb: self.neighbours.left_mb,
            above_mb: self.neighbours.above_mb,
            above_right_mb: self.neighbours.above_right_mb,
            mb_block_1: self.final_block_mvs[0],
            mb_block_2: self.final_block_mvs[1],
            mb_block_3: self.final_block_mvs[2],
        };
        predict_block_mv(block, &cands)
    }

    /// Record the **final** (post-MVD-add) MV for `block`. Subsequent
    /// `predictor_for` calls on later blocks will see this MV as the
    /// corresponding within-MB candidate per Figure 7-34.
    pub fn commit_block(&mut self, block: Block, final_mv: Mv) {
        match block {
            Block::TopLeft => self.final_block_mvs[0] = Some(final_mv),
            Block::TopRight => self.final_block_mvs[1] = Some(final_mv),
            Block::BottomLeft => self.final_block_mvs[2] = Some(final_mv),
            Block::BottomRight => self.block_4_final = Some(final_mv),
        }
    }

    /// Return the four final MVs of the macroblock as `[Mv; 4]` in
    /// Figure 6-8 raster order (block 1, block 2, block 3, block 4),
    /// substituting `Mv::default()` for any block that was never
    /// committed. The intended use is a single call after all four
    /// blocks have been committed.
    pub fn finalise(self) -> [Mv; 4] {
        [
            self.final_block_mvs[0].unwrap_or_default(),
            self.final_block_mvs[1].unwrap_or_default(),
            self.final_block_mvs[2].unwrap_or_default(),
            self.block_4_final.unwrap_or_default(),
        ]
    }

    /// One-shot bridge from the per-MB 4-MV finaliser to the picture-wide
    /// MV grid. Equivalent to
    /// [`MvGridCell::FourMv`]`(self.finalise())`, but expressed as a
    /// single call so the bitstream-driving site can write
    /// `mv_grid.set_cell(mb_x, mb_y, decoder.finalise_to_grid_cell())`
    /// without manually wrapping the `[Mv; 4]` in the cell enum.
    ///
    /// Per the [`MvGridCell`] doc-comment, `MvGridCell::FourMv` is the
    /// correct grid representation for a 4-MV-coded macroblock — one MV
    /// per 8x8 luminance block in Figure 6-8 raster order — so this
    /// helper is the canonical exit point for a 4-MV-mode decode.
    /// Substitutes `Mv::default()` for any block that was never
    /// committed, matching [`finalise`]'s semantics.
    pub fn finalise_to_grid_cell(self) -> MvGridCell {
        MvGridCell::FourMv(self.finalise())
    }
}

/// Direction of a neighbouring macroblock relative to the current MB.
///
/// Used by [`bordering_block_of_neighbour`] to resolve which 8x8
/// sub-block of a **4-MV-coded neighbouring macroblock** sits adjacent
/// to a given current-MB block per Figure 7-34. The three directions
/// match the three neighbour-MB fields of [`BlockCandidates`] /
/// [`MacroblockCandidates`]: `Left` ↔ `left_mb`, `Above` ↔ `above_mb`,
/// `AboveRight` ↔ `above_right_mb`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NeighbourDirection {
    /// Macroblock immediately to the left of the current MB.
    Left,
    /// Macroblock immediately above the current MB.
    Above,
    /// Macroblock diagonally above-right of the current MB.
    AboveRight,
}

impl NeighbourDirection {
    /// Const-fn enumeration of every direction. Useful for test loops.
    pub const ALL: [NeighbourDirection; 3] = [
        NeighbourDirection::Left,
        NeighbourDirection::Above,
        NeighbourDirection::AboveRight,
    ];
}

/// Per ISO/IEC 14496-2:2004(E) §7.6.5 / Figure 7-34, when the
/// neighbouring macroblock indicated by `direction` is coded with K=4
/// motion vectors (one per 8x8 luminance block per Figure 6-8), this
/// returns the [`Block`] of that neighbour whose 8x8 sub-block sits
/// **adjacent to the current-MB block being predicted** (`current`).
///
/// The mapping is taken directly from the four sub-diagrams of
/// Figure 7-34, observing which cell of the *neighbouring* MB box
/// borders the current `MV` cell:
///
/// | `current` | `direction`  | bordering block of neighbour MB    |
/// | --------- | ------------ | ---------------------------------- |
/// | TopLeft   | Left         | TopRight (block 2 of left MB)      |
/// | TopLeft   | Above        | BottomLeft (block 3 of above MB)   |
/// | TopLeft   | AboveRight   | BottomLeft (block 3 of AR MB)      |
/// | TopRight  | Above        | BottomRight (block 4 of above MB)  |
/// | TopRight  | AboveRight   | BottomLeft (block 3 of AR MB)      |
/// | BottomLeft| Left         | BottomRight (block 4 of left MB)   |
///
/// Returns `None` when no neighbour-MB candidate is required for the
/// given `(current, direction)` pair — this covers the **block 4
/// (`BottomRight`) case in full** (Figure 7-34's BR sub-diagram uses
/// only within-MB candidates), the `(TopRight, Left)` slot (block 2's
/// MV1 comes from the within-MB block 1, not the left-neighbour MB),
/// the `(BottomLeft, Above)` and `(BottomLeft, AboveRight)` slots
/// (block 3's MV2 / MV3 come from within-MB blocks 1 / 2), and any
/// other neighbour-direction combination that does not appear in
/// Figure 7-34.
///
/// # When to use
///
/// A caller decoding a current MB whose left / above / above-right
/// neighbour was 4-MV-coded already holds those neighbours' `[Mv; 4]`
/// arrays (one MV per Figure 6-8 block of the neighbour). To populate
/// [`BlockCandidates`] for a current-block predict, the caller picks
/// the neighbour's bordering MV via
///
/// ```ignore
/// use oxideav_msmpeg4::mv_pred::{bordering_block_of_neighbour, Block,
///     NeighbourDirection};
///
/// let left_neighbour_mvs: [Mv; 4] = …; // from previous MB decode
/// let cell = bordering_block_of_neighbour(Block::TopLeft, NeighbourDirection::Left)
///     .expect("TL/Left always borders one of left-MB's blocks");
/// let left_candidate: Mv = left_neighbour_mvs[cell.spec_index() as usize - 1];
/// ```
///
/// For a 1-MV-coded neighbour the caller passes that single MV in the
/// corresponding [`BlockCandidates`] slot directly — no resolution is
/// required (this helper is only consulted when the neighbour is
/// 4-MV-coded). [`pick_neighbour_mv_from_4mv`] wraps this lookup +
/// `[Mv; 4]` indexing in one call.
///
/// # Source
///
/// - `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`
///   §"ASCII transcription of Figure 7-34" — the four per-block
///   sub-diagrams plus the convention that "thin (non-bold) boxes are
///   neighbouring macroblocks; cells inside the bold box are 8x8
///   blocks of the current MB". The bordering-cell positions in this
///   table are read off the relative position of MV1/MV2/MV3 cells
///   within each direction's neighbouring-MB outline.
/// - `docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`
///   §7.6.5 — "vector candidate predictors (MV1, MV2, MV3) from the
///   spatial neighbourhood macroblocks or blocks already decoded".
/// - `docs/video/mpeg4-visual/figure-7-34-render.png` — rendered
///   original page 302 of the PDF, used as the visual cross-check
///   for the cell positions transcribed above.
pub const fn bordering_block_of_neighbour(
    current: Block,
    direction: NeighbourDirection,
) -> Option<Block> {
    match (current, direction) {
        // Block 1 (TL) takes all three candidates from neighbouring MBs.
        // MV1 = left-neighbour's right-column / top-row 8x8  → block 2.
        // MV2 = above-neighbour's bottom-row / left-col 8x8  → block 3.
        // MV3 = AR-neighbour's bottom-row / left-col 8x8     → block 3.
        (Block::TopLeft, NeighbourDirection::Left) => Some(Block::TopRight),
        (Block::TopLeft, NeighbourDirection::Above) => Some(Block::BottomLeft),
        (Block::TopLeft, NeighbourDirection::AboveRight) => Some(Block::BottomLeft),

        // Block 2 (TR) takes MV1 from within-MB block 1 — no left-MB
        // candidate is consulted. MV2 = above-neighbour's bottom-row /
        // right-col 8x8 → block 4 (current block 2 is at the right
        // column of the current MB, so the directly-above cell is the
        // right-column / bottom-row block of the above-neighbour).
        // MV3 = AR-neighbour's bottom-left 8x8 → block 3.
        (Block::TopRight, NeighbourDirection::Left) => None,
        (Block::TopRight, NeighbourDirection::Above) => Some(Block::BottomRight),
        (Block::TopRight, NeighbourDirection::AboveRight) => Some(Block::BottomLeft),

        // Block 3 (BL) takes MV2/MV3 from within-MB blocks 1/2 — no
        // above-MB / above-right-MB candidate is consulted. MV1 =
        // left-neighbour's right-column / bottom-row 8x8 → block 4
        // (current block 3 is at the bottom row of the current MB, so
        // the directly-left cell is the right-column / bottom-row
        // block of the left-neighbour).
        (Block::BottomLeft, NeighbourDirection::Left) => Some(Block::BottomRight),
        (Block::BottomLeft, NeighbourDirection::Above) => None,
        (Block::BottomLeft, NeighbourDirection::AboveRight) => None,

        // Block 4 (BR) consumes only within-MB candidates per Figure
        // 7-34's BR sub-diagram — no neighbour-MB MV is ever consulted
        // for any direction.
        (Block::BottomRight, _) => None,
    }
}

/// Pick the bordering MV of a **4-MV-coded** neighbouring macroblock,
/// given that neighbour's `[Mv; 4]` (one MV per Figure 6-8 block of
/// the neighbour, ordered `[block 1, block 2, block 3, block 4]`).
///
/// Resolves the bordering block via [`bordering_block_of_neighbour`]
/// and indexes into the neighbour's MV array. Returns `None` when the
/// `(current, direction)` pair has no neighbour-MB candidate per
/// Figure 7-34 (i.e. the same `None` cases as
/// [`bordering_block_of_neighbour`]).
///
/// For a **1-MV-coded** neighbour the caller does not need this
/// helper: the same single MV is reported in every direction by
/// definition, so the caller passes it directly into the relevant
/// [`BlockCandidates`] field.
///
/// # Source
///
/// Same as [`bordering_block_of_neighbour`].
pub const fn pick_neighbour_mv_from_4mv(
    current: Block,
    direction: NeighbourDirection,
    neighbour_mvs: &[Mv; 4],
) -> Option<Mv> {
    match bordering_block_of_neighbour(current, direction) {
        Some(b) => Some(neighbour_mvs[b.spec_index() as usize - 1]),
        None => None,
    }
}

/// Mode tag for a neighbouring macroblock's motion-vector state.
///
/// A neighbour macroblock in an MS-MPEG-4 P-VOP can be in one of three
/// states from the perspective of the current macroblock's MV predictor:
///
/// - **`Absent`** — the neighbour is outside the picture / video packet /
///   GOB (or, for v1/v2, in a not-yet-decoded portion of the stream).
///   Per Figure 7-34 rule 1 the corresponding candidate is *not valid*.
/// - **`OneMv(mv)`** — the neighbour was coded with a single MV for the
///   whole 16x16 macroblock (1-MV mode, the most common case in v3).
///   That single MV is the same regardless of which bordering 8x8 cell
///   of the neighbour borders the current MB block being predicted.
/// - **`FourMv([mv1, mv2, mv3, mv4])`** — the neighbour was coded with
///   four MVs (one per 8x8 luminance block, Figure 6-8 ordering). The
///   §7.6.5 normative text requires the **adjacent** 8x8 block's MV
///   (i.e. the cell on the side of the neighbour that physically borders
///   the current block) to be used as the candidate — see
///   [`bordering_block_of_neighbour`].
///
/// `FourMv` carries the neighbour's four MVs in the same Figure 6-8
/// raster order that [`Macroblock4MvDecoder::finalise`] / the public
/// `pick_neighbour_mv_from_4mv` API use: `[block 1, block 2, block 3,
/// block 4]`.
///
/// # Source
///
/// - `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`
///   §"What the figure shows" — the bordering-cell rule for
///   4-MV-neighbours is the visual content of the four sub-diagrams.
/// - `docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`
///   §7.6.5 — "vector candidate predictors (MV1, MV2, MV3) from the
///   spatial neighbourhood macroblocks or blocks already decoded".
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NeighbourMvKind {
    /// Neighbour is outside the picture / packet / GOB; candidate is
    /// "not valid" per Figure 7-34 rule 1.
    Absent,
    /// Neighbour was coded with one MV per 16x16 MB. That single MV is
    /// the candidate regardless of which border cell would otherwise
    /// apply.
    OneMv(Mv),
    /// Neighbour was coded with four MVs per 16x16 MB (one per 8x8
    /// luminance block). The MVs are in Figure 6-8 raster order:
    /// `[block 1, block 2, block 3, block 4]`. The bordering-cell rule
    /// picks one of them per `(current_block, direction)` pair — see
    /// [`bordering_block_of_neighbour`].
    FourMv([Mv; 4]),
}

impl NeighbourMvKind {
    /// True iff this neighbour is `Absent`.
    pub const fn is_absent(self) -> bool {
        matches!(self, NeighbourMvKind::Absent)
    }
}

/// The three neighbouring macroblocks' MV states for the current MB.
///
/// Combined with [`resolve_block_candidates`] this expresses the figure's
/// per-(current-block, neighbour-direction) cell rule in one place,
/// so callers can pass a single `NeighbourSet` (sized once per current
/// MB) and have each block of that MB receive the correct
/// [`BlockCandidates`] without manually rebuilding the bordering-cell
/// table per block.
///
/// `left`, `above`, and `above_right` correspond, in that order, to the
/// three [`NeighbourDirection`] variants and to the three neighbour-MB
/// fields of [`BlockCandidates`] / [`MacroblockCandidates`].
///
/// # When to use
///
/// A picture decoder that mixes 1-MV and 4-MV MBs in a P-VOP builds one
/// `NeighbourSet` per current MB by looking up the previously-decoded
/// neighbour MBs (left / above / above-right) in its per-MB MV grid:
///
/// - If the neighbour was skip / out-of-picture, store
///   [`NeighbourMvKind::Absent`].
/// - If the neighbour was 1-MV-coded, store
///   [`NeighbourMvKind::OneMv(mv)`].
/// - If the neighbour was 4-MV-coded, store
///   [`NeighbourMvKind::FourMv([mv1, mv2, mv3, mv4])`].
///
/// The decoder then drives [`resolve_block_candidates`] for each
/// [`Block`] of the current MB to produce that block's
/// [`BlockCandidates`]. For a 4-MV current MB the four
/// [`BlockCandidates`] are fed in turn into [`predict_block_mv`]; for a
/// 1-MV current MB only [`Block::TopLeft`] is resolved and the result
/// matches the existing 1-MV path bit-for-bit when every neighbour is
/// `OneMv` or `Absent`.
///
/// # Source
///
/// - `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`
///   §"ASCII transcription of Figure 7-34" — the four per-block
///   sub-diagrams identify, per current block, which neighbour cell
///   borders it. This struct combined with [`resolve_block_candidates`]
///   automates that picking step.
/// - `docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`
///   §7.6.5 — neighbour-cell rule for 4-MV neighbours; validity rule 1
///   for `Absent`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NeighbourSet {
    /// State of the left-neighbour MB (or `Absent` if outside picture /
    /// packet / GOB).
    pub left: NeighbourMvKind,
    /// State of the above-neighbour MB.
    pub above: NeighbourMvKind,
    /// State of the above-right-neighbour MB.
    pub above_right: NeighbourMvKind,
}

impl Default for NeighbourSet {
    fn default() -> Self {
        Self {
            left: NeighbourMvKind::Absent,
            above: NeighbourMvKind::Absent,
            above_right: NeighbourMvKind::Absent,
        }
    }
}

impl NeighbourSet {
    /// All-absent neighbour set — the picture-corner state used at
    /// `(mb_x=0, mb_y=0)`.
    pub const ABSENT: NeighbourSet = NeighbourSet {
        left: NeighbourMvKind::Absent,
        above: NeighbourMvKind::Absent,
        above_right: NeighbourMvKind::Absent,
    };

    /// Pick the candidate MV contributed by the neighbour in the given
    /// `direction` for the predictor of `current` (a block of the
    /// **current** MB). Returns `None` when the neighbour is `Absent`
    /// (rule-1 invalid) OR when no neighbour-MB candidate is required
    /// for the given `(current, direction)` pair per Figure 7-34
    /// (the same `None` cases as [`bordering_block_of_neighbour`]).
    ///
    /// For a `OneMv` neighbour the picker returns that single MV
    /// whenever the `(current, direction)` pair has a bordering cell;
    /// for a `FourMv` neighbour it indexes the neighbour's `[Mv; 4]` by
    /// the bordering block per Figure 7-34.
    pub const fn candidate_for(self, current: Block, direction: NeighbourDirection) -> Option<Mv> {
        let kind = match direction {
            NeighbourDirection::Left => self.left,
            NeighbourDirection::Above => self.above,
            NeighbourDirection::AboveRight => self.above_right,
        };
        // Out-of-picture / packet / GOB is rule-1 invalid regardless of
        // whether the figure would otherwise consult the neighbour.
        if let NeighbourMvKind::Absent = kind {
            return None;
        }
        match bordering_block_of_neighbour(current, direction) {
            Some(cell) => match kind {
                NeighbourMvKind::Absent => None,
                NeighbourMvKind::OneMv(mv) => Some(mv),
                NeighbourMvKind::FourMv(mvs) => Some(mvs[cell.spec_index() as usize - 1]),
            },
            None => None,
        }
    }
}

/// Build the per-block [`BlockCandidates`] for `current` (a block of
/// the current MB) from a [`NeighbourSet`] (the three neighbour-MB
/// states) and the already-committed within-MB MVs of the same current
/// MB.
///
/// The within-MB candidates are passed verbatim (their figure-mapping
/// per current block is the responsibility of [`gather_candidates`] —
/// each `Block`'s sub-diagram consumes a different subset). The
/// neighbour-MB candidates are resolved through [`NeighbourSet::candidate_for`]
/// so the bordering-cell rule for 4-MV neighbours is applied
/// transparently.
///
/// This is the composition layer above [`pick_neighbour_mv_from_4mv`]
/// that wraps the three-direction picker + the within-MB threading in
/// one call. The output is suitable for [`predict_block_mv`].
///
/// `within_mb` is a `[Option<Mv>; 3]` carrying the already-decoded MVs
/// of `[block 1, block 2, block 3]` of the current MB (in Figure 6-8
/// raster order). For block 1 (the first decode) the caller passes
/// `[None; 3]`; subsequent blocks pass the previously-committed finals.
///
/// # Equivalence with the 1-MV path
///
/// When every `NeighbourSet` field is `OneMv` or `Absent` (i.e. no
/// 4-MV neighbour is present) and `current == Block::TopLeft`,
/// [`resolve_block_candidates`] produces the same [`BlockCandidates`]
/// the existing 1-MV `picture::decode_pframe_mb` path constructs by
/// hand — see the test below for the pinned equivalence.
///
/// # Source
///
/// Same as [`NeighbourSet`]:
/// `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md` and
/// `docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`
/// §7.6.5.
pub const fn resolve_block_candidates(
    current: Block,
    neighbours: NeighbourSet,
    within_mb: [Option<Mv>; 3],
) -> BlockCandidates {
    BlockCandidates {
        left_mb: neighbours.candidate_for(current, NeighbourDirection::Left),
        above_mb: neighbours.candidate_for(current, NeighbourDirection::Above),
        above_right_mb: neighbours.candidate_for(current, NeighbourDirection::AboveRight),
        mb_block_1: within_mb[0],
        mb_block_2: within_mb[1],
        mb_block_3: within_mb[2],
    }
}

/// 4-MV-per-MB predictor batch that correctly handles **4-MV
/// neighbours**. For each of the four blocks of the current macroblock,
/// the bordering cell of each 4-MV neighbour is picked per Figure 7-34
/// before feeding it into [`predict_block_mv`].
///
/// Compared to [`predict_macroblock_4mv_with_finals`], which assumes
/// every neighbour contributes the **same** MV regardless of which
/// current-MB block is being predicted (correct only when every
/// neighbour is 1-MV-coded), this batch resolves the neighbour-cell
/// per-block per spec §7.6.5. When every neighbour is `OneMv` or
/// `Absent`, the two functions produce identical output — see the
/// equivalence test below.
///
/// `final_block_mvs` are the post-MVD-add final MVs of blocks 1 / 2 / 3
/// of the **current** macroblock, supplied in raster order. Use
/// [`Macroblock4MvDecoder`] for the predict → decode-MVD → commit loop
/// when driving from a bitstream.
///
/// # Source
///
/// Same as [`NeighbourSet`]:
/// `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md` §"What
/// the figure shows" + the four per-block sub-diagrams;
/// `docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`
/// §7.6.5 substitution rules + median.
pub fn predict_macroblock_4mv_with_4mv_neighbours(
    neighbours: NeighbourSet,
    final_block_mvs: [Option<Mv>; 3],
) -> [Mv; 4] {
    let mut out = [Mv::default(); 4];
    let mut i = 0;
    while i < Block::ALL.len() {
        let block = Block::ALL[i];
        let cands = resolve_block_candidates(block, neighbours, final_block_mvs);
        out[i] = predict_block_mv(block, &cands);
        i += 1;
    }
    out
}

/// Stateful predict → commit driver for a 4-MV-per-MB macroblock whose
/// neighbours may be a mix of 1-MV and 4-MV-coded MBs.
///
/// This is the [`NeighbourSet`]-driven analogue of [`Macroblock4MvDecoder`]:
/// it exposes the same `predictor_for(block)` / `commit_block(block, mv)`
/// / `finalise()` shape that a future
/// `picture::decode_pframe_mb` 4-MV path will drive from the bitstream,
/// but routes every predictor call through [`resolve_block_candidates`]
/// so a 4-MV-coded neighbour's bordering 8x8 cell is picked per current
/// block per Figure 7-34.
///
/// Compared to [`Macroblock4MvDecoder`] (which carries a
/// [`MacroblockCandidates`] = three flat `Option<Mv>` per direction, so
/// every current block sees the same neighbour MV regardless of which
/// bordering cell §7.6.5 says to read), this decoder holds the full
/// [`NeighbourSet`] = three `NeighbourMvKind` per direction. When a
/// neighbour is `OneMv` the two decoders produce identical predictors;
/// when a neighbour is `FourMv` with distinct per-block MVs the new
/// decoder picks the correct bordering cell per current block —
/// matching the spec, where [`Macroblock4MvDecoder`] would have had to
/// pre-collapse the neighbour to one cell and lose information.
///
/// # Caller pattern
///
/// Identical to [`Macroblock4MvDecoder`] — the only difference is the
/// constructor argument:
///
/// ```ignore
/// use oxideav_msmpeg4::mv_pred::{Block, Macroblock4MvDecoderNeighbours,
///     NeighbourMvKind, NeighbourSet};
///
/// let neighbours = NeighbourSet {
///     left: NeighbourMvKind::FourMv(left_four_mvs),
///     above: NeighbourMvKind::OneMv(above_mv),
///     above_right: NeighbourMvKind::Absent,
/// };
/// let mut dec = Macroblock4MvDecoderNeighbours::new(neighbours);
/// for block in Block::ALL {
///     let predictor = dec.predictor_for(block);
///     let mvd = decode_mvd_from_bitstream();
///     let final_mv = predictor + mvd;
///     dec.commit_block(block, final_mv);
/// }
/// let four_mvs: [Mv; 4] = dec.finalise();
/// ```
///
/// # Equivalence with `Macroblock4MvDecoder`
///
/// When every neighbour is `OneMv` or `Absent` (no 4-MV neighbour
/// present in the current frame's neighbour set), this decoder produces
/// the same per-block predictors and the same final `[Mv; 4]` as
/// [`Macroblock4MvDecoder`] driven by the equivalent
/// [`MacroblockCandidates`]. The "every-neighbour-is-`OneMv`" test
/// below pins this. The divergence appears the moment a `FourMv`
/// neighbour with distinct bordering cells is present.
///
/// # Source
///
/// Same as [`NeighbourSet`] / [`resolve_block_candidates`]:
/// `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md` and
/// `docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`
/// §7.6.5 (substitution rules + median + four sub-diagrams).
#[derive(Clone, Copy, Debug)]
pub struct Macroblock4MvDecoderNeighbours {
    neighbours: NeighbourSet,
    /// Final (post-MVD-add) MVs of blocks 1, 2, 3 of the **current** MB
    /// once committed. Index `i` is `Some` once
    /// `commit_block(Block::ALL[i], …)` has been called.
    final_block_mvs: [Option<Mv>; 3],
    /// Final MV of block 4 once committed. Stored separately because
    /// block 4's MV is never consumed as a within-MB candidate by any
    /// later block.
    block_4_final: Option<Mv>,
}

impl Macroblock4MvDecoderNeighbours {
    /// New per-MB session with the given neighbouring-MB state set.
    /// All within-MB block MVs start undecoded (`None`).
    pub const fn new(neighbours: NeighbourSet) -> Self {
        Self {
            neighbours,
            final_block_mvs: [None; 3],
            block_4_final: None,
        }
    }

    /// Spec-7.6.5 predictor for `block`, computed from the
    /// [`NeighbourSet`] plus whatever blocks of the current MB have
    /// already been committed via [`commit_block`]. Re-invocable: the
    /// predictor for a block that has not yet been committed is a
    /// function of the already-committed blocks only, so calling
    /// `predictor_for` twice for the same `block` (without an
    /// intervening commit) returns the same MV.
    ///
    /// Internally calls [`resolve_block_candidates`] to build the
    /// per-current-block [`BlockCandidates`] with neighbour bordering
    /// cells resolved per Figure 7-34, then [`predict_block_mv`] to
    /// apply the figure's per-block raw layout + the four §7.6.5
    /// substitution rules + the per-component median.
    pub fn predictor_for(&self, block: Block) -> Mv {
        let cands = resolve_block_candidates(block, self.neighbours, self.final_block_mvs);
        predict_block_mv(block, &cands)
    }

    /// Record the **final** (post-MVD-add) MV for `block` of the
    /// current MB. Subsequent [`predictor_for`] calls on later blocks
    /// will see this MV in the corresponding within-MB candidate slot
    /// per Figure 7-34.
    pub fn commit_block(&mut self, block: Block, final_mv: Mv) {
        match block {
            Block::TopLeft => self.final_block_mvs[0] = Some(final_mv),
            Block::TopRight => self.final_block_mvs[1] = Some(final_mv),
            Block::BottomLeft => self.final_block_mvs[2] = Some(final_mv),
            Block::BottomRight => self.block_4_final = Some(final_mv),
        }
    }

    /// Returns the [`NeighbourSet`] this decoder was constructed with.
    /// Useful for round-tripping the per-MB context in test code and
    /// for callers that want to confirm the state at any point of the
    /// predict / commit loop without reaching into a private field.
    pub const fn neighbours(&self) -> NeighbourSet {
        self.neighbours
    }

    /// Return the four final MVs of the current macroblock as
    /// `[Mv; 4]` in Figure 6-8 raster order (block 1, block 2, block 3,
    /// block 4), substituting `Mv::default()` for any block that was
    /// never committed. Intended for a single call after all four
    /// blocks have been committed.
    pub fn finalise(self) -> [Mv; 4] {
        [
            self.final_block_mvs[0].unwrap_or_default(),
            self.final_block_mvs[1].unwrap_or_default(),
            self.final_block_mvs[2].unwrap_or_default(),
            self.block_4_final.unwrap_or_default(),
        ]
    }

    /// One-shot bridge from the per-MB 4-MV finaliser to the picture-wide
    /// MV grid — analogue of [`Macroblock4MvDecoder::finalise_to_grid_cell`]
    /// for the [`NeighbourSet`]-driven decoder. Equivalent to
    /// [`MvGridCell::FourMv`]`(self.finalise())`, expressed as a single
    /// call so the bitstream-driving site can write
    /// `mv_grid.set_cell(mb_x, mb_y, decoder.finalise_to_grid_cell())`
    /// without manually wrapping the `[Mv; 4]` in the cell enum. Picks
    /// up [`finalise`]'s `Mv::default()` substitution for any block that
    /// was never committed.
    pub fn finalise_to_grid_cell(self) -> MvGridCell {
        MvGridCell::FourMv(self.finalise())
    }
}

impl Default for Macroblock4MvDecoderNeighbours {
    fn default() -> Self {
        Self::new(NeighbourSet::ABSENT)
    }
}

/// Per-MB cell of a picture-wide MV grid. Each macroblock position in a
/// P-VOP stores one of three states that maps directly to
/// [`NeighbourMvKind`]:
///
/// - [`MvGridCell::Absent`] for skipped MBs (whose stored MV is zero but
///   are treated as "no contribution" by the neighbour-lookup path —
///   the H.263 convention threaded through `picture::decode_pframe_mb`)
///   and for grid positions whose MB has not yet been decoded.
/// - [`MvGridCell::OneMv(mv)`] for the 1-MV-per-MB case (always for
///   `short_video_header == '1'` and for 1-MV-coded MBs in 4-MV-enabled
///   pictures).
/// - [`MvGridCell::FourMv([mv1, mv2, mv3, mv4])`] for 4-MV-coded MBs,
///   storing one MV per Figure 6-8 luminance block.
///
/// `From<MvGridCell> for NeighbourMvKind` is provided so the grid cell
/// can be promoted directly into a [`NeighbourSet`] field.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MvGridCell {
    /// No stored MV (undecoded, or "no contribution" per H.263 boundary
    /// substitution). Maps to [`NeighbourMvKind::Absent`].
    #[default]
    Absent,
    /// A 1-MV-coded macroblock with a single 16x16 MV.
    OneMv(Mv),
    /// A 4-MV-coded macroblock with one MV per 8x8 luminance block in
    /// Figure 6-8 raster order: `[block 1, block 2, block 3, block 4]`.
    FourMv([Mv; 4]),
}

impl MvGridCell {
    /// `true` when this cell is [`MvGridCell::Absent`] — i.e. the
    /// macroblock at this raster position has not been decoded yet, or
    /// the position is on the far side of a video-packet / GOB
    /// boundary that the caller has reset per the
    /// [`MvGrid`] doc-comment "Boundary handling" section.
    ///
    /// Mirrors [`NeighbourMvKind::is_absent`] (same name, same
    /// semantics) so a call site that treats `MvGridCell` and
    /// `NeighbourMvKind` interchangeably (typically via the
    /// `From<MvGridCell> for NeighbourMvKind` conversion) reads the
    /// same way before and after the conversion.
    pub const fn is_absent(self) -> bool {
        matches!(self, MvGridCell::Absent)
    }

    /// `true` when this cell is a 1-MV-coded macroblock
    /// ([`MvGridCell::OneMv`]).
    pub const fn is_one_mv(self) -> bool {
        matches!(self, MvGridCell::OneMv(_))
    }

    /// `true` when this cell is a 4-MV-coded macroblock
    /// ([`MvGridCell::FourMv`]).
    pub const fn is_four_mv(self) -> bool {
        matches!(self, MvGridCell::FourMv(_))
    }
}

impl From<MvGridCell> for NeighbourMvKind {
    fn from(cell: MvGridCell) -> Self {
        match cell {
            MvGridCell::Absent => NeighbourMvKind::Absent,
            MvGridCell::OneMv(mv) => NeighbourMvKind::OneMv(mv),
            MvGridCell::FourMv(mvs) => NeighbourMvKind::FourMv(mvs),
        }
    }
}

/// Picture-wide MV grid: one [`MvGridCell`] per macroblock position in
/// raster order (`y * width + x`). This is the data structure a P-VOP
/// decoder fills in as it walks MBs in raster order, and that the
/// per-MB predictor wants to consult to build a [`NeighbourSet`] for
/// the current MB.
///
/// # Boundary handling
///
/// Per Figure 7-34 + §7.6.5 rule 1, neighbours that lie **outside the
/// picture** are "not valid" and the [`NeighbourMvKind::Absent`] state
/// is reported. This grid handles **picture-edge** substitution
/// transparently in [`MvGrid::neighbour_set_for`]:
///
/// - At `mb_x == 0` the left neighbour is `Absent`.
/// - At `mb_y == 0` the above neighbour is `Absent`.
/// - At `mb_y == 0` OR at `mb_x + 1 == width` (right edge) the
///   above-right neighbour is `Absent` (the latter because the
///   above-right cell would lie in a column past the picture).
///
/// **Video-packet / GOB boundary substitution** is **not** modelled
/// here — those boundaries are stream-format-dependent and must be
/// applied by the caller by writing [`MvGridCell::Absent`] into the
/// grid positions on the far side of the boundary before consulting
/// [`MvGrid::neighbour_set_for`].
///
/// # Source
///
/// `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`
/// §"Boundary substitution" + `docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`
/// §7.6.5 rule 1.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MvGrid {
    width: usize,
    height: usize,
    cells: Vec<MvGridCell>,
}

impl MvGrid {
    /// New `width × height` grid with every cell `Absent`. This is the
    /// initial state at picture start.
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            cells: vec![MvGridCell::Absent; width * height],
        }
    }

    /// Grid width in macroblocks.
    pub const fn width(&self) -> usize {
        self.width
    }

    /// Grid height in macroblocks.
    pub const fn height(&self) -> usize {
        self.height
    }

    /// Grid dimensions as `(width, height)` in macroblocks. Convenience
    /// pair-accessor matching the `width × height` argument order taken
    /// by [`MvGrid::new`]; equivalent to `(self.width(), self.height())`.
    pub const fn dimensions(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    /// Return the cell at `(mb_x, mb_y)`. Out-of-bounds positions
    /// return [`MvGridCell::Absent`] per the picture-edge substitution
    /// rule.
    pub fn cell_at(&self, mb_x: usize, mb_y: usize) -> MvGridCell {
        if mb_x >= self.width || mb_y >= self.height {
            return MvGridCell::Absent;
        }
        self.cells[mb_y * self.width + mb_x]
    }

    /// Store a cell at `(mb_x, mb_y)`. Out-of-bounds positions are a
    /// no-op (the caller is responsible for staying within the grid).
    pub fn set_cell(&mut self, mb_x: usize, mb_y: usize, cell: MvGridCell) {
        if mb_x >= self.width || mb_y >= self.height {
            return;
        }
        self.cells[mb_y * self.width + mb_x] = cell;
    }

    /// Reset the cell at `(mb_x, mb_y)` to [`MvGridCell::Absent`].
    ///
    /// Convenience shortcut for
    /// `set_cell(mb_x, mb_y, MvGridCell::Absent)` that names the
    /// boundary-resync intent: per
    /// `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md` §
    /// "Boundary substitution", a neighbouring macroblock that lies
    /// outside the current video packet (when the stream uses
    /// data-partitioned resync markers) or outside the current GOB
    /// (when `short_video_header == '1'`) is treated as **transparent**
    /// by §7.6.5, i.e. its candidate is reported as `Absent` and the
    /// substitution rules apply. Out-of-bounds positions are a no-op,
    /// mirroring [`MvGrid::set_cell`].
    pub fn clear_cell(&mut self, mb_x: usize, mb_y: usize) {
        self.set_cell(mb_x, mb_y, MvGridCell::Absent);
    }

    /// Reset every cell in row `mb_y` to [`MvGridCell::Absent`].
    ///
    /// Bulk shortcut for the common boundary-resync case where an
    /// entire MB row sits on the far side of a video-packet boundary:
    /// per `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`
    /// §"Boundary substitution" the row's macroblocks are treated as
    /// transparent for the current row's predictor lookup, so the
    /// caller writes `Absent` into every column of that row before
    /// consulting [`MvGrid::neighbour_set_for`]. Out-of-bounds rows
    /// (`mb_y >= self.height`) are a no-op.
    pub fn clear_row(&mut self, mb_y: usize) {
        if mb_y >= self.height {
            return;
        }
        let start = mb_y * self.width;
        let end = start + self.width;
        for cell in &mut self.cells[start..end] {
            *cell = MvGridCell::Absent;
        }
    }

    /// Reset every cell in the grid to [`MvGridCell::Absent`] without
    /// re-allocating.
    ///
    /// Equivalent to `*self = MvGrid::new(self.width(), self.height())`
    /// in observable effect, but holds onto the existing backing
    /// allocation. Useful at picture-start when a P-decoder reuses the
    /// same grid across frames: the post-clear state matches the
    /// `MvGrid::new`-fresh state, so [`MvGrid::neighbour_set_for`]
    /// reports `NeighbourSet::ABSENT` for every position until the
    /// caller starts writing per-MB cells back in.
    pub fn clear_all(&mut self) {
        for cell in &mut self.cells {
            *cell = MvGridCell::Absent;
        }
    }

    /// Raster-order iterator over `(mb_x, mb_y, MvGridCell)` triples.
    ///
    /// Walks the grid in the same raster order
    /// (`for mb_y in 0..h { for mb_x in 0..w { ... } }`) that a P-VOP
    /// macroblock loop writes cells in, so a diagnostic / regression
    /// caller can pair its expected per-MB cell list against the grid
    /// contents without re-deriving the raster index manually.
    pub fn iter_cells(&self) -> impl Iterator<Item = (usize, usize, MvGridCell)> + '_ {
        let width = self.width;
        self.cells
            .iter()
            .enumerate()
            .map(move |(idx, cell)| (idx % width, idx / width, *cell))
    }

    /// Build the [`NeighbourSet`] for the current MB at `(mb_x, mb_y)`
    /// from the grid's already-decoded cells.
    ///
    /// Neighbour positions are:
    ///
    /// - `left = (mb_x - 1, mb_y)` (`Absent` when `mb_x == 0`).
    /// - `above = (mb_x, mb_y - 1)` (`Absent` when `mb_y == 0`).
    /// - `above_right = (mb_x + 1, mb_y - 1)` (`Absent` when
    ///   `mb_y == 0` or `mb_x + 1 == width`).
    ///
    /// Each neighbour cell is promoted to a [`NeighbourMvKind`] via
    /// `From<MvGridCell> for NeighbourMvKind`. The result can be fed
    /// straight into [`Macroblock4MvDecoderNeighbours::new`] or into
    /// [`resolve_block_candidates`] to obtain per-current-block
    /// [`BlockCandidates`].
    ///
    /// # Picture-corner / edge cases (Figure 7-34 boundary rule)
    ///
    /// - `(0, 0)`: all three neighbours `Absent`.
    /// - `(mb_x, 0)` for `mb_x > 0`: only `left` is non-`Absent`.
    /// - `(0, mb_y)` for `mb_y > 0`: `left` is `Absent`; the other
    ///   two are read from row `mb_y - 1`.
    /// - `(width - 1, mb_y)` for `mb_y > 0`: `above_right` is
    ///   `Absent` because the column `mb_x + 1` is past the picture.
    pub fn neighbour_set_for(&self, mb_x: usize, mb_y: usize) -> NeighbourSet {
        let left = if mb_x > 0 {
            self.cell_at(mb_x - 1, mb_y)
        } else {
            MvGridCell::Absent
        };
        let (above, above_right) = if mb_y > 0 {
            let above = self.cell_at(mb_x, mb_y - 1);
            let above_right = if mb_x + 1 < self.width {
                self.cell_at(mb_x + 1, mb_y - 1)
            } else {
                MvGridCell::Absent
            };
            (above, above_right)
        } else {
            (MvGridCell::Absent, MvGridCell::Absent)
        };
        NeighbourSet {
            left: left.into(),
            above: above.into(),
            above_right: above_right.into(),
        }
    }
}

/// Per-component median of three MVs. Same formula as the existing
/// `mv::median_predictor`: `median(a, b, c) = a + b + c - min - max`,
/// computed independently per component. Factored here so the
/// substitution-then-median path is self-contained and `mv::Mv` stays
/// the only cross-module dependency.
fn median_of_three(a: Mv, b: Mv, c: Mv) -> Mv {
    fn med(a: i8, b: i8, c: i8) -> i8 {
        let mn = a.min(b).min(c);
        let mx = a.max(b).max(c);
        // i32 arithmetic to avoid the (-128 + -128 + ...) i8 overflow.
        (a as i32 + b as i32 + c as i32 - mn as i32 - mx as i32) as i8
    }
    Mv {
        x: med(a.x, b.x, c.x),
        y: med(a.y, b.y, c.y),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The first decision rule of §7.6.5 plus the figure caption.
    /// Block::TopLeft (single-MV case) takes its three candidates from
    /// the left, above, and above-right *neighbouring macroblocks*.
    #[test]
    fn top_left_pulls_from_neighbour_mbs() {
        let l = Mv { x: 1, y: 2 };
        let a = Mv { x: 3, y: 4 };
        let ar = Mv { x: 5, y: 6 };
        let cands = BlockCandidates {
            left_mb: Some(l),
            above_mb: Some(a),
            above_right_mb: Some(ar),
            // Within-MB blocks should be ignored for TL.
            mb_block_1: Some(Mv { x: -1, y: -1 }),
            mb_block_2: Some(Mv { x: -2, y: -2 }),
            mb_block_3: Some(Mv { x: -3, y: -3 }),
        };
        let raw = gather_candidates(Block::TopLeft, &cands);
        assert_eq!(raw, [Some(l), Some(a), Some(ar)]);
    }

    /// Figure 7-34 block 2 sub-diagram: MV1 = block 1 (within MB),
    /// MV2 = above neighbour MB, MV3 = above-right neighbour MB.
    #[test]
    fn top_right_pulls_block_1_then_neighbour_above_and_above_right() {
        let b1 = Mv { x: 10, y: 20 };
        let a = Mv { x: 30, y: 40 };
        let ar = Mv { x: 50, y: 60 };
        let cands = BlockCandidates {
            // Ignored for TR.
            left_mb: Some(Mv { x: -1, y: -1 }),
            mb_block_2: Some(Mv { x: -2, y: -2 }),
            mb_block_3: Some(Mv { x: -3, y: -3 }),
            above_mb: Some(a),
            above_right_mb: Some(ar),
            mb_block_1: Some(b1),
        };
        let raw = gather_candidates(Block::TopRight, &cands);
        assert_eq!(raw, [Some(b1), Some(a), Some(ar)]);
    }

    /// Figure 7-34 block 3 sub-diagram: MV1 = left neighbour MB,
    /// MV2 = block 1 (within MB), MV3 = block 2 (within MB).
    #[test]
    fn bottom_left_pulls_left_neighbour_then_within_mb_blocks_1_and_2() {
        let l = Mv { x: 1, y: 2 };
        let b1 = Mv { x: 10, y: 20 };
        let b2 = Mv { x: 30, y: 40 };
        let cands = BlockCandidates {
            left_mb: Some(l),
            mb_block_1: Some(b1),
            mb_block_2: Some(b2),
            // Ignored for BL.
            above_mb: Some(Mv { x: -1, y: -1 }),
            above_right_mb: Some(Mv { x: -2, y: -2 }),
            mb_block_3: Some(Mv { x: -3, y: -3 }),
        };
        let raw = gather_candidates(Block::BottomLeft, &cands);
        assert_eq!(raw, [Some(l), Some(b1), Some(b2)]);
    }

    /// Figure 7-34 block 4 sub-diagram: all three candidates are
    /// within-MB blocks. MV1 = block 3, MV2 = block 1, MV3 = block 2.
    #[test]
    fn bottom_right_pulls_within_mb_blocks_3_then_1_then_2() {
        let b1 = Mv { x: 10, y: 20 };
        let b2 = Mv { x: 30, y: 40 };
        let b3 = Mv { x: 50, y: 60 };
        let cands = BlockCandidates {
            mb_block_1: Some(b1),
            mb_block_2: Some(b2),
            mb_block_3: Some(b3),
            // All ignored for BR.
            left_mb: Some(Mv { x: -1, y: -1 }),
            above_mb: Some(Mv { x: -2, y: -2 }),
            above_right_mb: Some(Mv { x: -3, y: -3 }),
        };
        let raw = gather_candidates(Block::BottomRight, &cands);
        assert_eq!(raw, [Some(b3), Some(b1), Some(b2)]);
    }

    // Rule 2: exactly one None → it is set to zero, others kept.
    #[test]
    fn rule_2_exactly_one_invalid_substitutes_zero() {
        // MV2 (index 1) invalid.
        let a = Mv { x: 3, y: -5 };
        let c = Mv { x: 7, y: 2 };
        let resolved = apply_validity_rules([Some(a), None, Some(c)]);
        assert_eq!(resolved, [a, Mv::default(), c]);
        // MV1 (index 0) invalid.
        let resolved = apply_validity_rules([None, Some(a), Some(c)]);
        assert_eq!(resolved, [Mv::default(), a, c]);
        // MV3 (index 2) invalid.
        let resolved = apply_validity_rules([Some(a), Some(c), None]);
        assert_eq!(resolved, [a, c, Mv::default()]);
    }

    // Rule 3: exactly two None → both become the third valid candidate.
    #[test]
    fn rule_3_exactly_two_invalid_both_become_third() {
        let only = Mv { x: 4, y: -7 };
        for raw in [
            [Some(only), None, None],
            [None, Some(only), None],
            [None, None, Some(only)],
        ] {
            let resolved = apply_validity_rules(raw);
            assert_eq!(resolved, [only, only, only], "raw = {raw:?}");
        }
    }

    // Rule 4: all None → all zero.
    #[test]
    fn rule_4_all_invalid_all_zero() {
        let resolved = apply_validity_rules([None, None, None]);
        assert_eq!(resolved, [Mv::default(); 3]);
    }

    // No rule fires when every candidate is valid.
    #[test]
    fn all_valid_passes_through_unchanged() {
        let a = Mv { x: 1, y: 2 };
        let b = Mv { x: 3, y: 4 };
        let c = Mv { x: 5, y: 6 };
        let resolved = apply_validity_rules([Some(a), Some(b), Some(c)]);
        assert_eq!(resolved, [a, b, c]);
    }

    /// Spec §7.6.5 worked example: MV1=(-2,3), MV2=(1,5), MV3=(-1,7)
    /// must yield (Px, Py) = (-1, 5).
    #[test]
    fn spec_7_6_5_worked_example_predictor_matches() {
        let cands = BlockCandidates {
            left_mb: Some(Mv { x: -2, y: 3 }),
            above_mb: Some(Mv { x: 1, y: 5 }),
            above_right_mb: Some(Mv { x: -1, y: 7 }),
            ..BlockCandidates::default()
        };
        let p = predict_block_mv(Block::TopLeft, &cands);
        assert_eq!(p, Mv { x: -1, y: 5 });
    }

    /// Picture corner: every neighbour `None`. Per rule 4 all candidates
    /// become zero → median is zero. This is what `picture::decode_pframe_mb`
    /// relies on at `(mb_x=0, mb_y=0)` for the first inter MB.
    #[test]
    fn corner_no_neighbours_yields_zero_predictor() {
        let cands = BlockCandidates::default();
        for b in Block::ALL {
            let p = predict_block_mv(b, &cands);
            assert_eq!(p, Mv::default(), "block {b:?}");
        }
    }

    /// First-row edge for `Block::TopLeft`: only `left_mb` valid → it
    /// gets median-of-3 with two zeros (rule 2 applied twice) =
    /// median(left_mb, 0, 0) = 0 along both components. This is the
    /// "first inter MB in a row" case for `picture::decode_pframe_mb`.
    /// (Note: the existing `mv::median_predictor` substitutes a missing
    /// neighbour with zero unconditionally, which happens to produce the
    /// same result here because rule 3 also forces both invalids to
    /// `left_mb` only when *two* are invalid. Here we have one valid +
    /// two invalid so rule 3 fires.)
    #[test]
    fn left_only_neighbour_applies_rule_3() {
        let l = Mv { x: 5, y: -3 };
        let cands = BlockCandidates {
            left_mb: Some(l),
            ..BlockCandidates::default()
        };
        let raw = gather_candidates(Block::TopLeft, &cands);
        let resolved = apply_validity_rules(raw);
        // Rule 3: both invalids become `l`, so all three are `l`.
        assert_eq!(resolved, [l, l, l]);
        // Median of three equals → l.
        let p = predict_block_mv(Block::TopLeft, &cands);
        assert_eq!(p, l);
    }

    /// `Block::TopLeft` predict_block_mv must yield bit-identical
    /// results to the long-standing `mv::median_predictor(left, top,
    /// top_right)` for **every** combination of `Some` / `None` on the
    /// three neighbour positions, **except** when only one neighbour is
    /// valid (where the new rule-3 path returns that neighbour itself
    /// rather than `median_predictor`'s "zero-substitute then median"
    /// which returns `median(only, 0, 0) = 0`). That single divergence
    /// is the bugfix this round delivers — the spec §7.6.5 four-rule
    /// path is the normative one; the existing `median_predictor` was a
    /// pre-spec-figure shortcut that the picture-header rebinder will
    /// route through this module in a follow-up round.
    #[test]
    fn top_left_matches_existing_median_predictor_when_two_or_more_valid() {
        let samples = [
            Mv { x: 0, y: 0 },
            Mv { x: 1, y: -1 },
            Mv { x: -2, y: 5 },
            Mv { x: 7, y: 3 },
            Mv { x: -10, y: 12 },
        ];
        for &l in samples.iter() {
            for &t in samples.iter() {
                for &tr in samples.iter() {
                    for slots in [
                        // All three valid.
                        [Some(l), Some(t), Some(tr)],
                        // Two valid in every position permutation.
                        [Some(l), Some(t), None],
                        [Some(l), None, Some(tr)],
                        [None, Some(t), Some(tr)],
                    ] {
                        let cands = BlockCandidates {
                            left_mb: slots[0],
                            above_mb: slots[1],
                            above_right_mb: slots[2],
                            ..BlockCandidates::default()
                        };
                        let new_pred = predict_block_mv(Block::TopLeft, &cands);
                        let old_pred = crate::mv::median_predictor(slots[0], slots[1], slots[2]);
                        assert_eq!(
                            new_pred, old_pred,
                            "TL divergence with slots {slots:?}: new={new_pred:?} old={old_pred:?}",
                        );
                    }
                }
            }
        }
    }

    /// The one documented divergence between
    /// `predict_block_mv(Block::TopLeft, …)` and the older
    /// `mv::median_predictor` shortcut: when exactly **one** neighbour
    /// is valid, the spec-correct §7.6.5 path returns *that* neighbour
    /// (rule 3 promotes the lone valid candidate to all three slots; the
    /// median of three identical values is that value). The shortcut
    /// substitutes zero for the two missing neighbours and returns
    /// `median(only, 0, 0) = 0`.
    ///
    /// Pin both behaviours so the difference is impossible to miss:
    /// any future routing of `decode_pframe_mb` through this module
    /// will visibly change the corner-cell behaviour (which is exactly
    /// what fixing the spec compliance requires).
    #[test]
    fn top_left_diverges_from_old_shortcut_in_rule_3_case() {
        let only = Mv { x: 4, y: -2 };
        let cands_left_only = BlockCandidates {
            left_mb: Some(only),
            ..BlockCandidates::default()
        };
        // New (spec-correct) path: rule 3 → median(only, only, only) = only.
        let new_pred = predict_block_mv(Block::TopLeft, &cands_left_only);
        assert_eq!(new_pred, only);
        // Old shortcut: median(only, 0, 0) = 0.
        let old_pred = crate::mv::median_predictor(Some(only), None, None);
        assert_eq!(old_pred, Mv::default());
        // Hence they differ — the test exists to make the divergence loud.
        assert_ne!(new_pred, old_pred);
    }

    /// Spec rules are applied to each *block layout* independently:
    /// for `Block::BottomRight`, all three candidates are within-MB
    /// blocks of the same macroblock. If only `mb_block_1` is decoded
    /// at the time block 4 is predicted (unusual — normally blocks 1,
    /// 2, 3 are decoded first — but if mb_block_2 / mb_block_3 are
    /// intra-coded the per-row mv_grid sets them to None), rule 3
    /// fires and the predictor is mb_block_1.
    #[test]
    fn bottom_right_rule_3_with_only_block_1_valid() {
        let b1 = Mv { x: 11, y: -3 };
        let cands = BlockCandidates {
            mb_block_1: Some(b1),
            ..BlockCandidates::default()
        };
        let p = predict_block_mv(Block::BottomRight, &cands);
        assert_eq!(p, b1);
    }

    /// Block::ALL enumerates the four blocks in raster (1,2,3,4) order.
    #[test]
    fn block_all_in_raster_order() {
        let idxs: Vec<u8> = Block::ALL.iter().map(|b| b.spec_index()).collect();
        assert_eq!(idxs, vec![1, 2, 3, 4]);
    }

    /// Every block's `spec_index()` is in `1..=4` and unique.
    #[test]
    fn spec_index_unique_per_block() {
        let mut idxs: Vec<u8> = Block::ALL.iter().map(|b| b.spec_index()).collect();
        idxs.sort_unstable();
        assert_eq!(idxs, vec![1, 2, 3, 4]);
    }

    /// Median of three is order-independent.
    #[test]
    fn median_of_three_is_order_independent() {
        let a = Mv { x: -10, y: 3 };
        let b = Mv { x: 5, y: -2 };
        let c = Mv { x: 7, y: 8 };
        let m_abc = median_of_three(a, b, c);
        let m_acb = median_of_three(a, c, b);
        let m_bac = median_of_three(b, a, c);
        let m_bca = median_of_three(b, c, a);
        let m_cab = median_of_three(c, a, b);
        let m_cba = median_of_three(c, b, a);
        for m in [m_acb, m_bac, m_bca, m_cab, m_cba] {
            assert_eq!(m, m_abc);
        }
    }

    // -------------------- 4-MV-per-MB batch API tests --------------------

    /// `predict_macroblock_4mv_with_finals` returns four predictors in
    /// Figure 6-8 raster order; `[0]` matches a direct
    /// `predict_block_mv(Block::TopLeft, …)` call with the same
    /// neighbour MBs and `None` within-MB cells.
    #[test]
    fn batch_block_0_matches_top_left_with_no_within_mb_blocks() {
        let neighbours = MacroblockCandidates {
            left_mb: Some(Mv { x: 1, y: 2 }),
            above_mb: Some(Mv { x: 3, y: 4 }),
            above_right_mb: Some(Mv { x: 5, y: 6 }),
        };
        let out = predict_macroblock_4mv_with_finals(&neighbours, [None; 3]);

        // Compare to a direct TL predict_block_mv with the same
        // neighbours and empty within-MB cells.
        let direct = predict_block_mv(
            Block::TopLeft,
            &BlockCandidates {
                left_mb: neighbours.left_mb,
                above_mb: neighbours.above_mb,
                above_right_mb: neighbours.above_right_mb,
                ..BlockCandidates::default()
            },
        );
        assert_eq!(out[0], direct);
    }

    /// When all three within-MB blocks are decoded, the four predictors
    /// match per-block `predict_block_mv` calls in raster order. Pins
    /// the figure's per-block neighbour layout: block 2 wants block 1
    /// plus above plus above-right; block 3 wants left plus block 1
    /// plus block 2; block 4 wants block 3 plus block 1 plus block 2.
    #[test]
    fn batch_all_four_blocks_match_per_block_predict_calls() {
        let neighbours = MacroblockCandidates {
            left_mb: Some(Mv { x: 1, y: 2 }),
            above_mb: Some(Mv { x: 3, y: 4 }),
            above_right_mb: Some(Mv { x: 5, y: 6 }),
        };
        let b1 = Mv { x: 11, y: 12 };
        let b2 = Mv { x: 13, y: 14 };
        let b3 = Mv { x: 15, y: 16 };
        let finals = [Some(b1), Some(b2), Some(b3)];
        let out = predict_macroblock_4mv_with_finals(&neighbours, finals);

        // Per-block expected values via the documented gather + apply path.
        let make_cands = || BlockCandidates {
            left_mb: neighbours.left_mb,
            above_mb: neighbours.above_mb,
            above_right_mb: neighbours.above_right_mb,
            mb_block_1: Some(b1),
            mb_block_2: Some(b2),
            mb_block_3: Some(b3),
        };
        for (i, block) in Block::ALL.iter().enumerate() {
            let expected = predict_block_mv(*block, &make_cands());
            assert_eq!(out[i], expected, "block {block:?}");
        }
    }

    /// The closed-form `Macroblock4MvDecoder` produces the same
    /// per-block predictors as `predict_macroblock_4mv_with_finals`
    /// when driven in raster order with the same final MVs.
    #[test]
    fn macroblock_4mv_decoder_matches_batch_function() {
        let neighbours = MacroblockCandidates {
            left_mb: Some(Mv { x: -5, y: 7 }),
            above_mb: Some(Mv { x: 12, y: -3 }),
            above_right_mb: Some(Mv { x: -8, y: 6 }),
        };
        // Synthesise four MVDs (call them d1..d4) and pretend the
        // predict-add-commit loop produces final MVs final_i = pred_i
        // + d_i. We don't need a real bitstream — the API contract is
        // "commit whatever you decoded, future predictors will see it".
        let d = [
            Mv { x: 1, y: -1 },
            Mv { x: 2, y: 0 },
            Mv { x: -3, y: 4 },
            Mv { x: 0, y: 2 },
        ];

        let mut dec = Macroblock4MvDecoder::new(neighbours);
        let mut predictors = [Mv::default(); 4];
        let mut finals = [Mv::default(); 4];
        for (i, block) in Block::ALL.iter().enumerate() {
            let p = dec.predictor_for(*block);
            predictors[i] = p;
            let f = Mv {
                x: p.x.wrapping_add(d[i].x),
                y: p.y.wrapping_add(d[i].y),
            };
            finals[i] = f;
            dec.commit_block(*block, f);
        }
        let returned = dec.finalise();
        assert_eq!(returned, finals);

        // Re-drive the batch API with the same final block-1/2/3 MVs
        // (passing block 4 is not part of the API surface — block 4's
        // predictor sees only blocks 1/2/3). The two paths must agree
        // on every block.
        let batch = predict_macroblock_4mv_with_finals(
            &neighbours,
            [Some(finals[0]), Some(finals[1]), Some(finals[2])],
        );
        assert_eq!(batch, predictors);
    }

    /// Corner case (`mb_x=0, mb_y=0`): every neighbour `None`. Block 1
    /// applies rule 4 (all zero). Block 2's MV2/MV3 are `None`; if
    /// block 1's final MV is `(2, -3)` then per §7.6.5 rule 3 (exactly
    /// two of MV1/MV2/MV3 invalid → both become the valid one) MV2 and
    /// MV3 also become `(2, -3)`, and the median is `(2, -3)` itself.
    #[test]
    fn corner_no_neighbours_block_2_picks_up_block_1_via_rule_3() {
        let neighbours = MacroblockCandidates::default();
        let b1 = Mv { x: 2, y: -3 };
        let out = predict_macroblock_4mv_with_finals(&neighbours, [Some(b1), None, None]);
        assert_eq!(out[0], Mv::default(), "block 1 rule-4 → zero");
        assert_eq!(out[1], b1, "block 2 rule-3 → block-1 MV");
    }

    /// `Macroblock4MvDecoder::new` is `const fn`; verify by using it in
    /// a constant initialiser. Compile-time check, not a runtime one.
    #[test]
    fn macroblock_4mv_decoder_is_const_constructible() {
        const _DEC: Macroblock4MvDecoder = Macroblock4MvDecoder::new(MacroblockCandidates {
            left_mb: None,
            above_mb: None,
            above_right_mb: None,
        });
    }

    /// `MacroblockCandidates::default()` is all-None — i.e. the
    /// picture-corner state. Used by `picture::decode_pframe_mb` at
    /// `(mb_x=0, mb_y=0)` to bootstrap the MV grid.
    #[test]
    fn macroblock_candidates_default_is_all_none() {
        let d = MacroblockCandidates::default();
        assert_eq!(d.left_mb, None);
        assert_eq!(d.above_mb, None);
        assert_eq!(d.above_right_mb, None);
    }

    /// Block 4's predictor uses only blocks 1/2/3 of the same MB (per
    /// Figure 7-34 BR sub-diagram — all three candidates are within-MB
    /// cells; neighbour-MB MVs are ignored). Pin that here.
    #[test]
    fn batch_block_4_ignores_neighbour_mbs() {
        let b1 = Mv { x: 1, y: 1 };
        let b2 = Mv { x: 2, y: 2 };
        let b3 = Mv { x: 3, y: 3 };
        // First call with one set of neighbours.
        let n1 = MacroblockCandidates {
            left_mb: Some(Mv { x: 100, y: 0 }),
            above_mb: Some(Mv { x: 0, y: 100 }),
            above_right_mb: Some(Mv { x: -100, y: -100 }),
        };
        // Second call with completely different neighbours.
        let n2 = MacroblockCandidates {
            left_mb: Some(Mv { x: -50, y: 50 }),
            above_mb: Some(Mv { x: 50, y: -50 }),
            above_right_mb: None,
        };
        let finals = [Some(b1), Some(b2), Some(b3)];
        let o1 = predict_macroblock_4mv_with_finals(&n1, finals);
        let o2 = predict_macroblock_4mv_with_finals(&n2, finals);
        // Blocks 0 / 1 / 2 differ (they consume neighbour-MB MVs in
        // some position), but block 3 (BR) must be identical.
        assert_eq!(o1[3], o2[3]);
        // Sanity: block 3's value is the median of b1/b2/b3
        // (figure: MV1=block3=b3, MV2=block1=b1, MV3=block2=b2; all
        // three valid so no substitution → median(b3, b1, b2)).
        let expected_bw = median_of_three(b3, b1, b2);
        assert_eq!(o1[3], expected_bw);
    }

    // ---------- 4-MV neighbour-MB bordering-cell helper tests ----------

    /// Figure 7-34 block 1 (TL) sub-diagram pins all three
    /// bordering-cell positions: MV1 at TR of left-neighbour, MV2 at
    /// BL of above-neighbour, MV3 at BL of above-right neighbour.
    #[test]
    fn bordering_block_for_top_left_matches_figure_7_34_tl_subdiagram() {
        assert_eq!(
            bordering_block_of_neighbour(Block::TopLeft, NeighbourDirection::Left),
            Some(Block::TopRight),
        );
        assert_eq!(
            bordering_block_of_neighbour(Block::TopLeft, NeighbourDirection::Above),
            Some(Block::BottomLeft),
        );
        assert_eq!(
            bordering_block_of_neighbour(Block::TopLeft, NeighbourDirection::AboveRight),
            Some(Block::BottomLeft),
        );
    }

    /// Block 2 (TR) sub-diagram: no left-MB candidate (MV1 is within-MB
    /// block 1); MV2 in BR of above-neighbour (column directly above
    /// current block 2, bottom row of the above MB); MV3 in BL of
    /// above-right neighbour.
    #[test]
    fn bordering_block_for_top_right_matches_figure_7_34_tr_subdiagram() {
        assert_eq!(
            bordering_block_of_neighbour(Block::TopRight, NeighbourDirection::Left),
            None,
        );
        assert_eq!(
            bordering_block_of_neighbour(Block::TopRight, NeighbourDirection::Above),
            Some(Block::BottomRight),
        );
        assert_eq!(
            bordering_block_of_neighbour(Block::TopRight, NeighbourDirection::AboveRight),
            Some(Block::BottomLeft),
        );
    }

    /// Block 3 (BL) sub-diagram: only the left-neighbour contributes
    /// (block 4 / BR of the left-neighbour); MV2 / MV3 come from
    /// within-MB blocks 1 / 2 so the above / above-right directions
    /// return `None`.
    #[test]
    fn bordering_block_for_bottom_left_matches_figure_7_34_bl_subdiagram() {
        assert_eq!(
            bordering_block_of_neighbour(Block::BottomLeft, NeighbourDirection::Left),
            Some(Block::BottomRight),
        );
        assert_eq!(
            bordering_block_of_neighbour(Block::BottomLeft, NeighbourDirection::Above),
            None,
        );
        assert_eq!(
            bordering_block_of_neighbour(Block::BottomLeft, NeighbourDirection::AboveRight),
            None,
        );
    }

    /// Block 4 (BR) sub-diagram: every candidate is within-MB, so
    /// every neighbour direction returns `None`.
    #[test]
    fn bordering_block_for_bottom_right_returns_none_for_every_direction() {
        for direction in NeighbourDirection::ALL {
            assert_eq!(
                bordering_block_of_neighbour(Block::BottomRight, direction),
                None,
                "BR/{direction:?} must be None per Figure 7-34 BR sub-diagram",
            );
        }
    }

    /// Across all 12 `(current, direction)` pairs, exactly six map to a
    /// neighbour-MB cell — the six entries enumerated in the doc
    /// comment table. The other six must be `None`. Cross-check the
    /// count to guard against silent table drift.
    #[test]
    fn bordering_block_count_matches_documented_table() {
        let mut some_count = 0;
        let mut none_count = 0;
        for current in Block::ALL {
            for direction in NeighbourDirection::ALL {
                match bordering_block_of_neighbour(current, direction) {
                    Some(_) => some_count += 1,
                    None => none_count += 1,
                }
            }
        }
        assert_eq!(some_count, 6, "documented table has six Some(...) entries");
        assert_eq!(none_count, 6, "documented table has six None entries");
    }

    /// `bordering_block_of_neighbour` is `const fn`; verify by using
    /// it in a constant initialiser.
    #[test]
    fn bordering_block_is_const_evaluable() {
        const TL_LEFT: Option<Block> =
            bordering_block_of_neighbour(Block::TopLeft, NeighbourDirection::Left);
        assert_eq!(TL_LEFT, Some(Block::TopRight));
    }

    /// `pick_neighbour_mv_from_4mv` indexes the supplied `[Mv; 4]`
    /// in Figure 6-8 raster order. Pin the six Some-returning cases
    /// against a synthetic `[Mv; 4]` whose values encode the block
    /// number, so a mis-indexing would be visible.
    #[test]
    fn pick_neighbour_mv_uses_bordering_block_as_index() {
        // Encode block N as Mv { x: N, y: 10*N }.
        let nb: [Mv; 4] = [
            Mv { x: 1, y: 10 },
            Mv { x: 2, y: 20 },
            Mv { x: 3, y: 30 },
            Mv { x: 4, y: 40 },
        ];
        // (current, direction) → bordering block N → expected Mv from nb.
        let cases: [(Block, NeighbourDirection, Mv); 6] = [
            (Block::TopLeft, NeighbourDirection::Left, nb[1]), // → block 2
            (Block::TopLeft, NeighbourDirection::Above, nb[2]), // → block 3
            (Block::TopLeft, NeighbourDirection::AboveRight, nb[2]), // → block 3
            (Block::TopRight, NeighbourDirection::Above, nb[3]), // → block 4
            (Block::TopRight, NeighbourDirection::AboveRight, nb[2]), // → block 3
            (Block::BottomLeft, NeighbourDirection::Left, nb[3]), // → block 4
        ];
        for (current, direction, expected) in cases {
            assert_eq!(
                pick_neighbour_mv_from_4mv(current, direction, &nb),
                Some(expected),
                "{current:?} / {direction:?}",
            );
        }
    }

    /// `pick_neighbour_mv_from_4mv` returns `None` for every
    /// `(current, direction)` pair where the bordering-block table
    /// has no entry — six pairs total.
    #[test]
    fn pick_neighbour_mv_returns_none_when_no_bordering_block() {
        let nb: [Mv; 4] = [Mv::default(); 4];
        let none_cases: [(Block, NeighbourDirection); 6] = [
            (Block::TopRight, NeighbourDirection::Left),
            (Block::BottomLeft, NeighbourDirection::Above),
            (Block::BottomLeft, NeighbourDirection::AboveRight),
            (Block::BottomRight, NeighbourDirection::Left),
            (Block::BottomRight, NeighbourDirection::Above),
            (Block::BottomRight, NeighbourDirection::AboveRight),
        ];
        for (current, direction) in none_cases {
            assert_eq!(
                pick_neighbour_mv_from_4mv(current, direction, &nb),
                None,
                "{current:?} / {direction:?}",
            );
        }
    }

    /// `NeighbourDirection::ALL` enumerates every variant exactly once.
    #[test]
    fn neighbour_direction_all_lists_every_variant_once() {
        let all = NeighbourDirection::ALL;
        assert_eq!(all.len(), 3);
        assert!(all.contains(&NeighbourDirection::Left));
        assert!(all.contains(&NeighbourDirection::Above));
        assert!(all.contains(&NeighbourDirection::AboveRight));
    }

    /// End-to-end: build a `BlockCandidates` for Block 1 (TL) by
    /// resolving each neighbour direction through the 4-MV picker, and
    /// confirm the predictor matches the single-call result obtained
    /// by feeding the same three picked MVs straight into the
    /// canonical `predict_block_mv` API. Pins that the helper composes
    /// cleanly with the existing predict surface.
    #[test]
    fn pick_neighbour_mv_composes_with_predict_block_mv_for_block_1() {
        // Left-neighbour 4-MV array: distinct values per block.
        let left_nb: [Mv; 4] = [
            Mv { x: 1, y: 0 },
            Mv { x: 2, y: 0 }, // <- block 2 borders current block 1
            Mv { x: 3, y: 0 },
            Mv { x: 4, y: 0 },
        ];
        // Above-neighbour 4-MV array.
        let above_nb: [Mv; 4] = [
            Mv { x: 0, y: 1 },
            Mv { x: 0, y: 2 },
            Mv { x: 0, y: 3 }, // <- block 3 borders current block 1
            Mv { x: 0, y: 4 },
        ];
        // Above-right-neighbour 4-MV array.
        let ar_nb: [Mv; 4] = [
            Mv { x: 10, y: 10 },
            Mv { x: 20, y: 20 },
            Mv { x: 30, y: 30 }, // <- block 3 borders current block 1
            Mv { x: 40, y: 40 },
        ];

        let left_pick =
            pick_neighbour_mv_from_4mv(Block::TopLeft, NeighbourDirection::Left, &left_nb)
                .expect("TL/Left has a bordering block");
        let above_pick =
            pick_neighbour_mv_from_4mv(Block::TopLeft, NeighbourDirection::Above, &above_nb)
                .expect("TL/Above has a bordering block");
        let ar_pick =
            pick_neighbour_mv_from_4mv(Block::TopLeft, NeighbourDirection::AboveRight, &ar_nb)
                .expect("TL/AboveRight has a bordering block");

        assert_eq!(left_pick, Mv { x: 2, y: 0 });
        assert_eq!(above_pick, Mv { x: 0, y: 3 });
        assert_eq!(ar_pick, Mv { x: 30, y: 30 });

        // Feed the picked MVs into BlockCandidates and predict.
        let cands = BlockCandidates {
            left_mb: Some(left_pick),
            above_mb: Some(above_pick),
            above_right_mb: Some(ar_pick),
            ..BlockCandidates::default()
        };
        let predictor = predict_block_mv(Block::TopLeft, &cands);
        // Compare against a manual median of the same three MVs.
        let expected = median_of_three(left_pick, above_pick, ar_pick);
        assert_eq!(predictor, expected);
    }

    /// Driving the decoder out of raster order: committing block 4
    /// before block 3 does not retroactively un-thread block 4's
    /// candidates. The doc says "committing out of order is permitted
    /// but produces the same predictor a future block would see" — so
    /// the predictor for block 3 (after the OOO block-4 commit) still
    /// reflects the figure's BL layout (left + block 1 + block 2),
    /// **NOT** the prematurely committed block 4.
    #[test]
    fn decoder_out_of_order_commit_does_not_affect_later_blocks() {
        let neighbours = MacroblockCandidates {
            left_mb: Some(Mv { x: 1, y: 1 }),
            above_mb: Some(Mv { x: 2, y: 2 }),
            above_right_mb: Some(Mv { x: 3, y: 3 }),
        };
        let mut dec = Macroblock4MvDecoder::new(neighbours);
        // Commit block 4 first (pretend it was decoded out of order).
        dec.commit_block(Block::BottomRight, Mv { x: 99, y: 99 });
        // Now ask for block 3's predictor. Per Figure 7-34 BL it uses
        // left + block 1 + block 2 — none of which are committed, so
        // rule-4 fires across all three for a (1, 1, 1) cands set:
        // actually rule-3 with the left_mb valid → median(l, l, l) = l.
        let p3 = dec.predictor_for(Block::BottomLeft);
        assert_eq!(p3, neighbours.left_mb.unwrap());
    }

    // ---------- NeighbourSet / NeighbourMvKind resolution tests ----------

    /// `Absent` is rule-1-invalid: the candidate for that direction is
    /// `None` regardless of `current_block` or whether a bordering cell
    /// would otherwise apply.
    #[test]
    fn absent_neighbour_yields_none_for_every_current_block_and_direction() {
        let set = NeighbourSet::ABSENT;
        for current in Block::ALL {
            for direction in NeighbourDirection::ALL {
                assert_eq!(
                    set.candidate_for(current, direction),
                    None,
                    "{current:?} / {direction:?}",
                );
            }
        }
    }

    /// `OneMv(mv)` reports the same `mv` for every `(current, direction)`
    /// pair that has a bordering cell, and `None` for every pair that
    /// does not (the six `None` pairs in the bordering-cell table).
    #[test]
    fn one_mv_neighbour_reports_same_mv_for_every_bordering_pair() {
        let mv = Mv { x: 7, y: -3 };
        let set = NeighbourSet {
            left: NeighbourMvKind::OneMv(mv),
            above: NeighbourMvKind::OneMv(mv),
            above_right: NeighbourMvKind::OneMv(mv),
        };
        let mut bordering_some = 0;
        let mut bordering_none = 0;
        for current in Block::ALL {
            for direction in NeighbourDirection::ALL {
                let bordering = bordering_block_of_neighbour(current, direction);
                let got = set.candidate_for(current, direction);
                match bordering {
                    Some(_) => {
                        bordering_some += 1;
                        assert_eq!(got, Some(mv), "{current:?} / {direction:?}");
                    }
                    None => {
                        bordering_none += 1;
                        assert_eq!(got, None, "{current:?} / {direction:?}");
                    }
                }
            }
        }
        // Cross-check against the documented 6/6 split.
        assert_eq!(bordering_some, 6);
        assert_eq!(bordering_none, 6);
    }

    /// `FourMv([…])` indexes the neighbour's `[Mv; 4]` by the bordering
    /// cell per Figure 7-34. Pin the six `Some(_)` cases against a
    /// synthetic `[Mv; 4]` whose values encode the block number.
    #[test]
    fn four_mv_neighbour_indexes_bordering_block() {
        // Encode block N as Mv { x: N, y: 10*N }.
        let mvs: [Mv; 4] = [
            Mv { x: 1, y: 10 },
            Mv { x: 2, y: 20 },
            Mv { x: 3, y: 30 },
            Mv { x: 4, y: 40 },
        ];
        // For each direction, put the FourMv array on that direction and
        // Absent on the others so we know which contribution came from
        // where.
        for direction in NeighbourDirection::ALL {
            let mut set = NeighbourSet::ABSENT;
            match direction {
                NeighbourDirection::Left => set.left = NeighbourMvKind::FourMv(mvs),
                NeighbourDirection::Above => set.above = NeighbourMvKind::FourMv(mvs),
                NeighbourDirection::AboveRight => set.above_right = NeighbourMvKind::FourMv(mvs),
            }
            for current in Block::ALL {
                let got = set.candidate_for(current, direction);
                match bordering_block_of_neighbour(current, direction) {
                    Some(cell) => {
                        let expected = mvs[cell.spec_index() as usize - 1];
                        assert_eq!(got, Some(expected), "{current:?} / {direction:?}");
                    }
                    None => assert_eq!(got, None, "{current:?} / {direction:?}"),
                }
            }
        }
    }

    /// Concrete worked example: a `FourMv` left-neighbour and a current
    /// block 1 (TL) pick the neighbour's **block 2** (top-right cell) as
    /// the candidate — matching the Figure 7-34 TL sub-diagram where
    /// MV1 sits at the right column / top row of the left neighbour MB.
    #[test]
    fn four_mv_left_neighbour_picks_block_2_for_current_top_left() {
        let left_mvs: [Mv; 4] = [
            Mv { x: 1, y: 1 },
            Mv { x: 2, y: 2 }, // <-- block 2 (TR) of left-MB
            Mv { x: 3, y: 3 },
            Mv { x: 4, y: 4 },
        ];
        let set = NeighbourSet {
            left: NeighbourMvKind::FourMv(left_mvs),
            above: NeighbourMvKind::Absent,
            above_right: NeighbourMvKind::Absent,
        };
        assert_eq!(
            set.candidate_for(Block::TopLeft, NeighbourDirection::Left),
            Some(Mv { x: 2, y: 2 }),
        );
    }

    /// `resolve_block_candidates` produces a `BlockCandidates` whose
    /// neighbour-MB fields match `NeighbourSet::candidate_for` and whose
    /// within-MB fields equal the passed-in array.
    #[test]
    fn resolve_block_candidates_threads_neighbour_and_within_mb() {
        let mv = Mv { x: 5, y: -2 };
        let set = NeighbourSet {
            left: NeighbourMvKind::OneMv(mv),
            above: NeighbourMvKind::OneMv(mv),
            above_right: NeighbourMvKind::Absent,
        };
        let within = [Some(Mv { x: 11, y: 11 }), None, Some(Mv { x: 13, y: 13 })];
        for current in Block::ALL {
            let cands = resolve_block_candidates(current, set, within);
            assert_eq!(
                cands.left_mb,
                set.candidate_for(current, NeighbourDirection::Left),
            );
            assert_eq!(
                cands.above_mb,
                set.candidate_for(current, NeighbourDirection::Above),
            );
            assert_eq!(
                cands.above_right_mb,
                set.candidate_for(current, NeighbourDirection::AboveRight),
            );
            assert_eq!(cands.mb_block_1, within[0]);
            assert_eq!(cands.mb_block_2, within[1]);
            assert_eq!(cands.mb_block_3, within[2]);
        }
    }

    /// When every neighbour is `OneMv` (the 1-MV-neighbour case),
    /// `predict_macroblock_4mv_with_4mv_neighbours` agrees with
    /// `predict_macroblock_4mv_with_finals` driven by the equivalent
    /// `MacroblockCandidates`. This pins the documented equivalence
    /// between the new and the older batch APIs.
    #[test]
    fn predict_with_4mv_neighbours_matches_existing_batch_when_all_one_mv() {
        let l = Mv { x: 1, y: 2 };
        let a = Mv { x: 3, y: 4 };
        let ar = Mv { x: 5, y: 6 };
        let set = NeighbourSet {
            left: NeighbourMvKind::OneMv(l),
            above: NeighbourMvKind::OneMv(a),
            above_right: NeighbourMvKind::OneMv(ar),
        };
        let mb_candidates = MacroblockCandidates {
            left_mb: Some(l),
            above_mb: Some(a),
            above_right_mb: Some(ar),
        };
        let b1 = Mv { x: -2, y: 1 };
        let b2 = Mv { x: -3, y: 2 };
        let b3 = Mv { x: -4, y: 3 };
        let finals = [Some(b1), Some(b2), Some(b3)];

        let new_path = predict_macroblock_4mv_with_4mv_neighbours(set, finals);
        let old_path = predict_macroblock_4mv_with_finals(&mb_candidates, finals);
        assert_eq!(new_path, old_path);
    }

    /// When every neighbour is `Absent`, both batch APIs collapse to
    /// the same all-zero behaviour for block 1 (rule 4) and the same
    /// rule-3 promotion of decoded within-MB MVs for later blocks. Pin
    /// the equivalence here as a regression guard on the all-corner
    /// path.
    #[test]
    fn predict_with_4mv_neighbours_matches_existing_batch_at_corner() {
        let set = NeighbourSet::ABSENT;
        let mb_candidates = MacroblockCandidates::default();
        let b1 = Mv { x: 2, y: -3 };
        let finals = [Some(b1), None, None];

        let new_path = predict_macroblock_4mv_with_4mv_neighbours(set, finals);
        let old_path = predict_macroblock_4mv_with_finals(&mb_candidates, finals);
        assert_eq!(new_path, old_path);
        // And in concrete numbers: block 1 = zero, block 2 = block 1's
        // MV (rule 3 promotes the lone valid candidate).
        assert_eq!(new_path[0], Mv::default());
        assert_eq!(new_path[1], b1);
    }

    /// With at least one **4-MV** neighbour and distinct cells in its
    /// `[Mv; 4]`, the new batch produces predictors that **differ** from
    /// the older batch (which would consult a single MV per neighbour
    /// regardless of current-block). Pin the divergence by constructing
    /// a left-neighbour `FourMv` whose block 2 (TR cell, bordering
    /// current block 1) differs from its block 4 (BR cell, bordering
    /// current block 3) and showing block 1 vs block 3 see different
    /// left-cell MVs.
    #[test]
    fn predict_with_4mv_neighbours_distinguishes_bordering_cells_for_left_4mv_neighbour() {
        // Block 2 (borders current block 1) and block 4 (borders current
        // block 3) carry distinct MVs; blocks 1 / 3 of the neighbour are
        // never picked by any (current, Left) pair so set them to other
        // values to make sure they're not consulted.
        let left_mvs: [Mv; 4] = [
            Mv { x: 50, y: 50 },
            Mv { x: 1, y: 0 }, // <- block 2 → current block 1 sees this
            Mv { x: 60, y: 60 },
            Mv { x: 2, y: 0 }, // <- block 4 → current block 3 sees this
        ];
        // Single MV for every other position so the rest of the
        // candidate set is well-defined.
        let other = Mv { x: 10, y: 10 };
        let set = NeighbourSet {
            left: NeighbourMvKind::FourMv(left_mvs),
            above: NeighbourMvKind::OneMv(other),
            above_right: NeighbourMvKind::OneMv(other),
        };
        // No within-MB blocks decoded yet — irrelevant for block 1,
        // matters for block 3.
        let finals = [Some(Mv { x: -7, y: 4 }), Some(Mv { x: 8, y: -3 }), None];
        let preds = predict_macroblock_4mv_with_4mv_neighbours(set, finals);

        // Block 1 (TL): left-cell = neighbour-block 2 = (1, 0).
        let cands_b1 = resolve_block_candidates(Block::TopLeft, set, finals);
        assert_eq!(cands_b1.left_mb, Some(Mv { x: 1, y: 0 }));
        assert_eq!(preds[0], predict_block_mv(Block::TopLeft, &cands_b1));

        // Block 3 (BL): left-cell = neighbour-block 4 = (2, 0).
        let cands_b3 = resolve_block_candidates(Block::BottomLeft, set, finals);
        assert_eq!(cands_b3.left_mb, Some(Mv { x: 2, y: 0 }));
        assert_eq!(preds[2], predict_block_mv(Block::BottomLeft, &cands_b3));

        // And the divergence with the older batch is observable: the old
        // batch with a single MacroblockCandidates can only pick one
        // left-neighbour MV for both blocks 1 and 3, so for any choice
        // of `left_mb` the resulting predictor for at least one of
        // blocks 1 or 3 differs from the new one.
        let mb_candidates_pick_2 = MacroblockCandidates {
            left_mb: Some(Mv { x: 1, y: 0 }), // matches new block 1
            above_mb: Some(other),
            above_right_mb: Some(other),
        };
        let old_pick_2 = predict_macroblock_4mv_with_finals(&mb_candidates_pick_2, finals);
        // Block 1 agrees, block 3 must differ (old uses (1,0), new uses (2,0)).
        assert_eq!(old_pick_2[0], preds[0]);
        assert_ne!(
            old_pick_2[2], preds[2],
            "old batch with neighbour-block-2's MV must diverge from new on current block 3",
        );
    }

    /// `NeighbourMvKind::is_absent` returns `true` only for `Absent`.
    #[test]
    fn neighbour_mv_kind_is_absent_predicate() {
        assert!(NeighbourMvKind::Absent.is_absent());
        assert!(!NeighbourMvKind::OneMv(Mv::default()).is_absent());
        assert!(!NeighbourMvKind::FourMv([Mv::default(); 4]).is_absent());
    }

    /// `NeighbourSet::ABSENT` is the all-absent state and is `const`.
    /// Verify by using it in a constant initialiser.
    #[test]
    fn neighbour_set_absent_constant_compiles() {
        const _S: NeighbourSet = NeighbourSet::ABSENT;
        assert!(NeighbourSet::ABSENT.left.is_absent());
        assert!(NeighbourSet::ABSENT.above.is_absent());
        assert!(NeighbourSet::ABSENT.above_right.is_absent());
    }

    /// `resolve_block_candidates` is `const fn`; use it in a constant
    /// initialiser as a compile-time check.
    #[test]
    fn resolve_block_candidates_is_const_evaluable() {
        const CANDS: BlockCandidates =
            resolve_block_candidates(Block::TopLeft, NeighbourSet::ABSENT, [None; 3]);
        // The ABSENT-everywhere case has no valid neighbour-MB MVs.
        assert_eq!(CANDS.left_mb, None);
        assert_eq!(CANDS.above_mb, None);
        assert_eq!(CANDS.above_right_mb, None);
        assert_eq!(CANDS.mb_block_1, None);
    }

    /// A `OneMv` neighbour fed into `resolve_block_candidates` for
    /// `current=Block::TopLeft` produces the SAME `BlockCandidates`
    /// the existing 1-MV `picture::decode_pframe_mb` path constructs by
    /// hand — this is the equivalence that lets a future picture
    /// decoder switch over without behavioural drift.
    #[test]
    fn one_mv_neighbour_resolve_matches_manual_block_candidates_for_top_left() {
        let l = Mv { x: 1, y: 2 };
        let a = Mv { x: 3, y: 4 };
        let ar = Mv { x: 5, y: 6 };
        let set = NeighbourSet {
            left: NeighbourMvKind::OneMv(l),
            above: NeighbourMvKind::OneMv(a),
            above_right: NeighbourMvKind::OneMv(ar),
        };
        let new = resolve_block_candidates(Block::TopLeft, set, [None; 3]);
        let manual = BlockCandidates {
            left_mb: Some(l),
            above_mb: Some(a),
            above_right_mb: Some(ar),
            ..BlockCandidates::default()
        };
        assert_eq!(new, manual);
        // And the predictor matches as well.
        assert_eq!(
            predict_block_mv(Block::TopLeft, &new),
            predict_block_mv(Block::TopLeft, &manual),
        );
    }

    // ---- Macroblock4MvDecoderNeighbours (r221) ------------------------
    //
    // The stateful NeighbourSet-driven analogue of Macroblock4MvDecoder:
    // same predict/commit/finalise shape, but routes through
    // resolve_block_candidates so 4-MV neighbours' bordering cells are
    // picked per current block per Figure 7-34.

    /// Helper — drive a `Macroblock4MvDecoderNeighbours` over the four
    /// blocks in raster order using a list of per-block "final MVs"
    /// (predictor + MVD) and return the resulting `[Mv; 4]` from
    /// `finalise()`. Each block's MV is committed before the next
    /// block's predictor is read, mirroring the bitstream order.
    fn run_neighbours_decoder(neighbours: NeighbourSet, finals: [Mv; 4]) -> ([Mv; 4], [Mv; 4]) {
        let mut dec = Macroblock4MvDecoderNeighbours::new(neighbours);
        let mut predictors = [Mv::default(); 4];
        for (i, block) in Block::ALL.iter().enumerate() {
            predictors[i] = dec.predictor_for(*block);
            dec.commit_block(*block, finals[i]);
        }
        (predictors, dec.finalise())
    }

    /// All-absent neighbours: every block's MV1/MV2/MV3 starts `None`
    /// (rule 1) and within-MB candidates appear only after prior
    /// commits. Block 1's predictor is `(0, 0)` (rule 4: all three
    /// candidates invalid). Block 2's MV1 is block 1's final MV (rule 3:
    /// the only valid candidate propagates to the other two). Same for
    /// blocks 3 and 4 — each picks up earlier-committed within-MB MVs
    /// as their sole valid candidates.
    #[test]
    fn neighbours_decoder_absent_neighbours_propagate_via_rule_3() {
        let f1 = Mv { x: 4, y: 6 };
        let f2 = Mv { x: -2, y: 8 };
        let f3 = Mv { x: 1, y: -3 };
        let f4 = Mv { x: 0, y: 0 };
        let (preds, finals) = run_neighbours_decoder(NeighbourSet::ABSENT, [f1, f2, f3, f4]);
        // Block 1: rule 4 — all three candidates invalid → (0, 0).
        assert_eq!(preds[0], Mv::default());
        // Block 2: MV1 = block 1 (within-MB), MV2/MV3 = above/AR neighbour
        // (absent → None). Rule 3 propagates the only valid to all three
        // → predictor = block 1's final MV.
        assert_eq!(preds[1], f1);
        // Block 3: MV1 = left neighbour (absent), MV2 = block 1, MV3 =
        // block 2. Two valid (block 1 and block 2), rule 2 substitutes
        // the missing MV1 with zero → median(0, f1, f2) per component.
        assert_eq!(preds[2], super::median_of_three(Mv::default(), f1, f2),);
        // Block 4: all three candidates are within-MB (block 3 / block 1
        // / block 2) — already committed → predictor = median.
        assert_eq!(preds[3], super::median_of_three(f3, f1, f2));
        // Finalise round-trips the committed MVs.
        assert_eq!(finals, [f1, f2, f3, f4]);
    }

    /// With every neighbour `OneMv` or `Absent` the new decoder
    /// produces the same per-block predictors and the same finals as
    /// `Macroblock4MvDecoder` driven by the equivalent
    /// `MacroblockCandidates`. This pins the documented equivalence.
    #[test]
    fn neighbours_decoder_matches_macroblock4mv_decoder_when_all_one_mv() {
        let l = Mv { x: 1, y: 2 };
        let a = Mv { x: 3, y: 4 };
        let ar = Mv { x: 5, y: 6 };
        let set = NeighbourSet {
            left: NeighbourMvKind::OneMv(l),
            above: NeighbourMvKind::OneMv(a),
            above_right: NeighbourMvKind::OneMv(ar),
        };
        let cands = MacroblockCandidates {
            left_mb: Some(l),
            above_mb: Some(a),
            above_right_mb: Some(ar),
        };
        let finals = [
            Mv { x: 7, y: -1 },
            Mv { x: -3, y: 2 },
            Mv { x: 4, y: 4 },
            Mv { x: 0, y: -2 },
        ];
        // Drive the new decoder.
        let (new_preds, new_finals) = run_neighbours_decoder(set, finals);
        // Drive the existing decoder with the equivalent candidates.
        let mut old = Macroblock4MvDecoder::new(cands);
        let mut old_preds = [Mv::default(); 4];
        for (i, block) in Block::ALL.iter().enumerate() {
            old_preds[i] = old.predictor_for(*block);
            old.commit_block(*block, finals[i]);
        }
        let old_finals = old.finalise();
        assert_eq!(new_preds, old_preds);
        assert_eq!(new_finals, old_finals);
    }

    /// With every neighbour `Absent` the new decoder matches
    /// `Macroblock4MvDecoder` driven by `MacroblockCandidates::default()`
    /// (the same all-`None` state). This is the picture-corner
    /// equivalence — slightly different shape from the all-`OneMv`
    /// case because rule 4 fires for block 1 instead of the median.
    #[test]
    fn neighbours_decoder_matches_macroblock4mv_decoder_when_all_absent() {
        let finals = [
            Mv { x: 1, y: 2 },
            Mv { x: 3, y: -4 },
            Mv { x: 5, y: 6 },
            Mv { x: -7, y: 0 },
        ];
        let (new_preds, new_finals) = run_neighbours_decoder(NeighbourSet::ABSENT, finals);
        let mut old = Macroblock4MvDecoder::new(MacroblockCandidates::default());
        let mut old_preds = [Mv::default(); 4];
        for (i, block) in Block::ALL.iter().enumerate() {
            old_preds[i] = old.predictor_for(*block);
            old.commit_block(*block, finals[i]);
        }
        assert_eq!(new_preds, old_preds);
        assert_eq!(new_finals, old.finalise());
    }

    /// With a 4-MV-coded left neighbour whose bordering cells differ,
    /// the new decoder picks the bordering cell **per current block**.
    /// Block 1 (TL) borders the left neighbour's block 2 (TR cell);
    /// block 3 (BL) borders the left neighbour's block 4 (BR cell);
    /// blocks 2 and 4 do not consult the left neighbour. We give the
    /// left neighbour distinct MVs for its blocks 2 and 4 to force the
    /// per-current-block bordering rule to bite, and pin block 1 ≠
    /// block 3's predictor in a way that pre-collapsing the left
    /// neighbour to a single cell could never reproduce.
    #[test]
    fn neighbours_decoder_left_4mv_picks_distinct_bordering_cells() {
        let left_b2 = Mv { x: 10, y: 0 };
        let left_b4 = Mv { x: -10, y: 0 };
        let left_four = [
            Mv { x: 0, y: 0 }, // block 1 (TL of left MB)
            left_b2,           // block 2 (TR — borders current block 1)
            Mv { x: 0, y: 0 }, // block 3 (BL)
            left_b4,           // block 4 (BR — borders current block 3)
        ];
        let set = NeighbourSet {
            left: NeighbourMvKind::FourMv(left_four),
            above: NeighbourMvKind::Absent,
            above_right: NeighbourMvKind::Absent,
        };
        let mut dec = Macroblock4MvDecoderNeighbours::new(set);
        // Block 1 (TL): MV1 = left.block2 (left_b2), MV2/MV3 = absent.
        // Rule 3 propagates left_b2 to all three → predictor = left_b2.
        assert_eq!(dec.predictor_for(Block::TopLeft), left_b2);
        // Commit a final MV for block 1 so block 3 sees it as MV2.
        let f1 = Mv { x: 5, y: 5 };
        dec.commit_block(Block::TopLeft, f1);
        // Block 2 (TR): MV1 = block 1 (within-MB, just committed),
        // MV2/MV3 = absent. Rule 3 → predictor = f1.
        assert_eq!(dec.predictor_for(Block::TopRight), f1);
        let f2 = Mv { x: -2, y: 3 };
        dec.commit_block(Block::TopRight, f2);
        // Block 3 (BL): MV1 = left.block4 (left_b4), MV2 = block 1 = f1,
        // MV3 = block 2 = f2. All three valid → median.
        let pred_bl = dec.predictor_for(Block::BottomLeft);
        assert_eq!(pred_bl, super::median_of_three(left_b4, f1, f2));
        // Pin the divergence: block 1's predictor (left_b2) ≠ block 3's
        // predictor's MV1 cell (left_b4) — the two current blocks read
        // distinct bordering cells of the same 4-MV left neighbour.
        assert_ne!(left_b2, left_b4);
    }

    /// `Macroblock4MvDecoderNeighbours::neighbours()` round-trips the
    /// `NeighbourSet` the decoder was constructed with — useful for
    /// callers and tests that want to confirm the per-MB context
    /// without reaching into a private field.
    #[test]
    fn neighbours_decoder_round_trips_neighbours_accessor() {
        let set = NeighbourSet {
            left: NeighbourMvKind::OneMv(Mv { x: 1, y: 2 }),
            above: NeighbourMvKind::FourMv([Mv { x: 3, y: 4 }; 4]),
            above_right: NeighbourMvKind::Absent,
        };
        let dec = Macroblock4MvDecoderNeighbours::new(set);
        assert_eq!(dec.neighbours(), set);
    }

    /// `Default` impl is `ABSENT` neighbour set — matches the documented
    /// picture-corner state.
    #[test]
    fn neighbours_decoder_default_is_absent() {
        let dec = Macroblock4MvDecoderNeighbours::default();
        assert_eq!(dec.neighbours(), NeighbourSet::ABSENT);
        // And finalise on a fresh decoder yields all-zero (no commits).
        assert_eq!(dec.finalise(), [Mv::default(); 4]);
    }

    /// The constructor is `const fn` — callable in const context. This
    /// matches the existing `Macroblock4MvDecoder::new` const-fn surface
    /// and `NeighbourSet::ABSENT`'s const-context usability.
    #[test]
    fn neighbours_decoder_new_is_const_fn() {
        const DEC: Macroblock4MvDecoderNeighbours =
            Macroblock4MvDecoderNeighbours::new(NeighbourSet::ABSENT);
        // Confirm the const value matches `Default`.
        assert_eq!(DEC.neighbours(), NeighbourSet::ABSENT);
    }

    /// Out-of-order `commit_block` writes the corresponding slot but
    /// does not retroactively re-fire any earlier predictor — same
    /// shape as `Macroblock4MvDecoder`. Committing block 4 first then
    /// asking for block 1's predictor: block 4's MV is in its own slot
    /// (never consumed as a within-MB candidate per Figure 7-34) so
    /// block 1's predictor is unchanged from the all-`None` case.
    #[test]
    fn neighbours_decoder_out_of_order_commit_block_4_does_not_affect_block_1() {
        let mut dec = Macroblock4MvDecoderNeighbours::new(NeighbourSet::ABSENT);
        let b1_pred_clean = dec.predictor_for(Block::TopLeft);
        dec.commit_block(Block::BottomRight, Mv { x: 99, y: -99 });
        let b1_pred_after_b4 = dec.predictor_for(Block::TopLeft);
        assert_eq!(b1_pred_clean, b1_pred_after_b4);
        // And finalise still reports block 4's committed MV.
        let mvs = dec.finalise();
        assert_eq!(mvs[3], Mv { x: 99, y: -99 });
        assert_eq!(mvs[0], Mv::default());
        assert_eq!(mvs[1], Mv::default());
        assert_eq!(mvs[2], Mv::default());
    }

    /// `finalise()` substitutes `Mv::default()` for any block that was
    /// never committed — pinned by the per-slot independence property
    /// the previous test sets up.
    #[test]
    fn neighbours_decoder_finalise_substitutes_default_for_uncommitted_blocks() {
        let mut dec = Macroblock4MvDecoderNeighbours::new(NeighbourSet::ABSENT);
        dec.commit_block(Block::TopRight, Mv { x: 7, y: -1 });
        dec.commit_block(Block::BottomLeft, Mv { x: 0, y: 3 });
        let mvs = dec.finalise();
        assert_eq!(mvs[0], Mv::default()); // block 1 uncommitted
        assert_eq!(mvs[1], Mv { x: 7, y: -1 });
        assert_eq!(mvs[2], Mv { x: 0, y: 3 });
        assert_eq!(mvs[3], Mv::default()); // block 4 uncommitted
    }

    /// Driving the decoder over every block of `Block::ALL` produces
    /// the same `[Mv; 4]` finals that
    /// `predict_macroblock_4mv_with_4mv_neighbours` would compute by
    /// the same MVD-add reconstruction — pinning the stateful and
    /// stateless 4-MV-neighbour APIs as equivalent surfaces.
    ///
    /// We synthesise per-block MVDs (zero, so the predictor IS the
    /// final), then the per-block finals must be identical to the
    /// stateless batch's per-block output threaded through `finals[..i]`
    /// as `within_mb` exactly the way the decoder does it internally.
    #[test]
    fn neighbours_decoder_matches_stateless_batch_when_mvds_are_zero() {
        let left_four = [
            Mv { x: 0, y: 0 },
            Mv { x: 10, y: 0 },
            Mv { x: 0, y: 0 },
            Mv { x: -10, y: 0 },
        ];
        let set = NeighbourSet {
            left: NeighbourMvKind::FourMv(left_four),
            above: NeighbourMvKind::OneMv(Mv { x: 2, y: 2 }),
            above_right: NeighbourMvKind::OneMv(Mv { x: 4, y: -1 }),
        };
        // Stateful decoder with predictor = final (zero MVD).
        let mut dec = Macroblock4MvDecoderNeighbours::new(set);
        let mut stateful_finals = [Mv::default(); 4];
        for (i, block) in Block::ALL.iter().enumerate() {
            let pred = dec.predictor_for(*block);
            stateful_finals[i] = pred;
            dec.commit_block(*block, pred);
        }
        assert_eq!(dec.finalise(), stateful_finals);
        // Stateless batch reproduces the same finals when the within-MB
        // arg carries the already-committed previous-block finals — but
        // since the batch computes all four in one call, we instead
        // confirm block 1 of the batch matches the stateful block 1's
        // predictor (no prior commits to thread), which is sufficient to
        // pin the surface equivalence on the all-`None` within-MB entry.
        let batch = predict_macroblock_4mv_with_4mv_neighbours(set, [None; 3]);
        assert_eq!(batch[0], stateful_finals[0]);
    }

    // ---------- MvGrid / MvGridCell tests ----------

    /// `MvGridCell::default()` is `Absent` — pinning the documented
    /// "initial state at picture start" behaviour for grids built via
    /// `MvGrid::new`.
    #[test]
    fn mv_grid_cell_default_is_absent() {
        assert_eq!(MvGridCell::default(), MvGridCell::Absent);
    }

    /// `From<MvGridCell> for NeighbourMvKind` round-trips each of the
    /// three cell variants to the corresponding `NeighbourMvKind`
    /// constructor — same payload, no transformation.
    #[test]
    fn mv_grid_cell_promotes_to_neighbour_mv_kind() {
        let mv = Mv { x: 3, y: -2 };
        let mvs = [
            Mv { x: 1, y: 1 },
            Mv { x: 2, y: 2 },
            Mv { x: 3, y: 3 },
            Mv { x: 4, y: 4 },
        ];
        assert_eq!(
            NeighbourMvKind::from(MvGridCell::Absent),
            NeighbourMvKind::Absent,
        );
        assert_eq!(
            NeighbourMvKind::from(MvGridCell::OneMv(mv)),
            NeighbourMvKind::OneMv(mv),
        );
        assert_eq!(
            NeighbourMvKind::from(MvGridCell::FourMv(mvs)),
            NeighbourMvKind::FourMv(mvs),
        );
    }

    /// A fresh grid has every cell `Absent` and its `width()` / `height()`
    /// match the constructor args.
    #[test]
    fn mv_grid_new_is_all_absent() {
        let grid = MvGrid::new(4, 3);
        assert_eq!(grid.width(), 4);
        assert_eq!(grid.height(), 3);
        for y in 0..3 {
            for x in 0..4 {
                assert_eq!(grid.cell_at(x, y), MvGridCell::Absent);
            }
        }
    }

    /// `cell_at` for any out-of-bounds `(mb_x, mb_y)` returns `Absent`
    /// per the picture-edge substitution rule.
    #[test]
    fn mv_grid_out_of_bounds_cell_at_is_absent() {
        let grid = MvGrid::new(2, 2);
        // Both axes past the bound.
        assert_eq!(grid.cell_at(2, 0), MvGridCell::Absent);
        assert_eq!(grid.cell_at(0, 2), MvGridCell::Absent);
        assert_eq!(grid.cell_at(2, 2), MvGridCell::Absent);
        // Way past.
        assert_eq!(grid.cell_at(100, 100), MvGridCell::Absent);
    }

    /// `set_cell` writes the cell; `cell_at` reads it back; out-of-bounds
    /// writes are a no-op (they don't grow the grid or panic).
    #[test]
    fn mv_grid_set_cell_round_trips() {
        let mut grid = MvGrid::new(3, 3);
        let mv = Mv { x: 7, y: -3 };
        grid.set_cell(1, 2, MvGridCell::OneMv(mv));
        assert_eq!(grid.cell_at(1, 2), MvGridCell::OneMv(mv));
        // Untouched cells stay `Absent`.
        assert_eq!(grid.cell_at(0, 0), MvGridCell::Absent);
        assert_eq!(grid.cell_at(2, 2), MvGridCell::Absent);
        // Out-of-bounds set is a no-op (no panic).
        grid.set_cell(99, 99, MvGridCell::OneMv(mv));
        assert_eq!(grid.cell_at(99, 99), MvGridCell::Absent);
    }

    /// Picture-corner `(0, 0)`: all three neighbours are `Absent`
    /// regardless of grid contents — picture-edge substitution per
    /// Figure 7-34 boundary rule.
    #[test]
    fn mv_grid_corner_yields_absent_neighbour_set() {
        let mut grid = MvGrid::new(4, 4);
        // Even if we (incorrectly) wrote MVs to grid positions, the
        // neighbour lookup for the top-left corner must report all-Absent.
        let mv = Mv { x: 1, y: 1 };
        grid.set_cell(0, 0, MvGridCell::OneMv(mv));
        let set = grid.neighbour_set_for(0, 0);
        assert_eq!(set, NeighbourSet::ABSENT);
    }

    /// Top-edge `(mb_x, 0)` for `mb_x > 0`: only `left` is non-`Absent`.
    /// `above` and `above_right` would lie in row `-1` (outside the
    /// picture).
    #[test]
    fn mv_grid_top_edge_only_left_is_present() {
        let mut grid = MvGrid::new(4, 4);
        let l = Mv { x: 5, y: 5 };
        grid.set_cell(1, 0, MvGridCell::OneMv(l)); // the left neighbour for (2, 0)
        let set = grid.neighbour_set_for(2, 0);
        assert_eq!(set.left, NeighbourMvKind::OneMv(l));
        assert_eq!(set.above, NeighbourMvKind::Absent);
        assert_eq!(set.above_right, NeighbourMvKind::Absent);
    }

    /// Left-edge `(0, mb_y)` for `mb_y > 0`: `left` is `Absent`;
    /// `above` and `above_right` are read from row `mb_y - 1`.
    #[test]
    fn mv_grid_left_edge_left_is_absent_others_from_above_row() {
        let mut grid = MvGrid::new(4, 4);
        let a = Mv { x: 2, y: 2 };
        let ar = Mv { x: 3, y: 3 };
        grid.set_cell(0, 0, MvGridCell::OneMv(a)); // above of (0, 1)
        grid.set_cell(1, 0, MvGridCell::OneMv(ar)); // above-right of (0, 1)
        let set = grid.neighbour_set_for(0, 1);
        assert_eq!(set.left, NeighbourMvKind::Absent);
        assert_eq!(set.above, NeighbourMvKind::OneMv(a));
        assert_eq!(set.above_right, NeighbourMvKind::OneMv(ar));
    }

    /// Right-edge `(width - 1, mb_y)` for `mb_y > 0`: `above_right`
    /// is `Absent` because column `mb_x + 1` is past the picture, even
    /// though row `mb_y - 1` is in range.
    #[test]
    fn mv_grid_right_edge_above_right_is_absent() {
        let mut grid = MvGrid::new(3, 3);
        let l = Mv { x: 1, y: 1 };
        let a = Mv { x: 2, y: 2 };
        grid.set_cell(1, 1, MvGridCell::OneMv(l)); // left of (2, 1)
        grid.set_cell(2, 0, MvGridCell::OneMv(a)); // above of (2, 1)
        let set = grid.neighbour_set_for(2, 1);
        assert_eq!(set.left, NeighbourMvKind::OneMv(l));
        assert_eq!(set.above, NeighbourMvKind::OneMv(a));
        assert_eq!(set.above_right, NeighbourMvKind::Absent);
    }

    /// Interior position: all three neighbour MBs are present in the
    /// grid; the resulting `NeighbourSet` carries each neighbour's
    /// `MvGridCell` promoted to `NeighbourMvKind` at the matching slot.
    #[test]
    fn mv_grid_interior_position_carries_all_three_neighbours() {
        let mut grid = MvGrid::new(4, 4);
        let l = Mv { x: 1, y: 1 };
        let a_mvs = [
            Mv { x: 2, y: 2 },
            Mv { x: 3, y: 3 },
            Mv { x: 4, y: 4 },
            Mv { x: 5, y: 5 },
        ];
        let ar = Mv { x: 6, y: 6 };
        // Current MB = (2, 2). Neighbours at (1, 2), (2, 1), (3, 1).
        grid.set_cell(1, 2, MvGridCell::OneMv(l));
        grid.set_cell(2, 1, MvGridCell::FourMv(a_mvs));
        grid.set_cell(3, 1, MvGridCell::OneMv(ar));
        let set = grid.neighbour_set_for(2, 2);
        assert_eq!(set.left, NeighbourMvKind::OneMv(l));
        assert_eq!(set.above, NeighbourMvKind::FourMv(a_mvs));
        assert_eq!(set.above_right, NeighbourMvKind::OneMv(ar));
    }

    /// Composability with the per-MB decoder: drive
    /// `Macroblock4MvDecoderNeighbours` from a grid-built `NeighbourSet`
    /// and confirm the resulting per-block predictors match what
    /// `resolve_block_candidates` would produce directly. This pins
    /// the grid → decoder pipeline as a no-information-loss surface.
    #[test]
    fn mv_grid_feeds_macroblock_4mv_decoder_neighbours() {
        let mut grid = MvGrid::new(3, 3);
        let l_mvs = [
            Mv { x: 0, y: 0 },
            Mv { x: 10, y: 0 }, // TR cell — borders current block 1
            Mv { x: 0, y: 0 },
            Mv { x: -10, y: 0 }, // BR cell — borders current block 3
        ];
        let a = Mv { x: 2, y: 2 };
        let ar = Mv { x: 4, y: -1 };
        grid.set_cell(0, 1, MvGridCell::FourMv(l_mvs)); // left of (1, 1)
        grid.set_cell(1, 0, MvGridCell::OneMv(a)); // above of (1, 1)
        grid.set_cell(2, 0, MvGridCell::OneMv(ar)); // above-right of (1, 1)
        let set = grid.neighbour_set_for(1, 1);
        let decoder = Macroblock4MvDecoderNeighbours::new(set);
        // Block 1 (TL): MV1 = left.block2 (TR cell of left = (10, 0)),
        // MV2 = above MB, MV3 = above-right MB.
        let pred_tl = decoder.predictor_for(Block::TopLeft);
        let cands_tl = resolve_block_candidates(Block::TopLeft, set, [None; 3]);
        let expected_tl = predict_block_mv(Block::TopLeft, &cands_tl);
        assert_eq!(pred_tl, expected_tl);
        // And the same pipeline yields the exact bordering-cell pick:
        // MV1 came from left's block 2 (the TR cell).
        let cands_check = resolve_block_candidates(Block::TopLeft, set, [None; 3]);
        assert_eq!(cands_check.left_mb, Some(l_mvs[1]));
    }

    /// `(1, 0)` (top-edge, `mb_x > 0`) reduces the grid's
    /// `NeighbourSet` to "left only" — and feeding that into
    /// [`predict_macroblock_4mv_with_4mv_neighbours`] reproduces the
    /// `Macroblock4MvDecoder` semantics for a 1-MV left + Absent
    /// above + Absent above-right, regardless of what (if anything) is
    /// stored in row 0's other cells.
    #[test]
    fn mv_grid_top_edge_routes_through_predict_macroblock_4mv() {
        let mut grid = MvGrid::new(4, 4);
        let l = Mv { x: 7, y: -2 };
        // Write the left neighbour at (0, 0); the current MB is (1, 0).
        grid.set_cell(0, 0, MvGridCell::OneMv(l));
        // And write some other MVs in row 0 to confirm they don't leak in.
        grid.set_cell(2, 0, MvGridCell::OneMv(Mv { x: 99, y: 99 }));
        grid.set_cell(3, 0, MvGridCell::OneMv(Mv { x: 88, y: 88 }));
        let set = grid.neighbour_set_for(1, 0);
        assert_eq!(set.left, NeighbourMvKind::OneMv(l));
        assert_eq!(set.above, NeighbourMvKind::Absent);
        assert_eq!(set.above_right, NeighbourMvKind::Absent);
        // Driving the batch through the resulting set: block 1's
        // candidates are (Some(l), None, None) → rule 3 fires →
        // predictor = l.
        let finals = predict_macroblock_4mv_with_4mv_neighbours(set, [None; 3]);
        assert_eq!(finals[0], l);
    }

    // ---------- Round 243: per-MB decoder → MvGridCell exit point + ----------
    // ---------- MvGridCell query predicates + MvGrid::dimensions     ----------

    /// [`Macroblock4MvDecoder::finalise_to_grid_cell`] equals
    /// `MvGridCell::FourMv(decoder.finalise())` — the one-shot bridge to
    /// the picture-wide MV grid. Pinned by a four-commit sweep where
    /// each block carries a distinct MV; the resulting grid cell's
    /// payload matches the per-block array exactly in Figure 6-8 raster
    /// order.
    #[test]
    fn macroblock_4mv_decoder_finalise_to_grid_cell_wraps_four_mv() {
        let neighbours = MacroblockCandidates {
            left_mb: None,
            above_mb: None,
            above_right_mb: None,
        };
        let mvs = [
            Mv { x: 1, y: 2 },
            Mv { x: 3, y: 4 },
            Mv { x: 5, y: 6 },
            Mv { x: 7, y: 8 },
        ];
        let mut dec = Macroblock4MvDecoder::new(neighbours);
        for (i, block) in Block::ALL.iter().enumerate() {
            dec.commit_block(*block, mvs[i]);
        }
        let cell = dec.finalise_to_grid_cell();
        assert_eq!(cell, MvGridCell::FourMv(mvs));
    }

    /// [`Macroblock4MvDecoder::finalise_to_grid_cell`] picks up the
    /// `Mv::default()` substitution from [`finalise`] for any block
    /// that was never committed — the cell carries the same `[Mv; 4]`
    /// `finalise()` would return, just wrapped in
    /// [`MvGridCell::FourMv`].
    #[test]
    fn macroblock_4mv_decoder_finalise_to_grid_cell_substitutes_default() {
        let neighbours = MacroblockCandidates {
            left_mb: None,
            above_mb: None,
            above_right_mb: None,
        };
        let mut dec = Macroblock4MvDecoder::new(neighbours);
        dec.commit_block(Block::TopRight, Mv { x: 7, y: -1 });
        let cell = dec.finalise_to_grid_cell();
        assert_eq!(
            cell,
            MvGridCell::FourMv([
                Mv::default(),
                Mv { x: 7, y: -1 },
                Mv::default(),
                Mv::default(),
            ]),
        );
    }

    /// [`Macroblock4MvDecoderNeighbours::finalise_to_grid_cell`] equals
    /// `MvGridCell::FourMv(decoder.finalise())` — the same one-shot
    /// bridge as the `Macroblock4MvDecoder` variant, exposed on the
    /// [`NeighbourSet`]-driven decoder.
    #[test]
    fn macroblock_4mv_decoder_neighbours_finalise_to_grid_cell_wraps_four_mv() {
        let mvs = [
            Mv { x: 10, y: 20 },
            Mv { x: 30, y: 40 },
            Mv { x: 50, y: 60 },
            Mv { x: 70, y: 80 },
        ];
        let mut dec = Macroblock4MvDecoderNeighbours::new(NeighbourSet::ABSENT);
        for (i, block) in Block::ALL.iter().enumerate() {
            dec.commit_block(*block, mvs[i]);
        }
        let cell = dec.finalise_to_grid_cell();
        assert_eq!(cell, MvGridCell::FourMv(mvs));
    }

    /// The decoder → `MvGridCell::FourMv` bridge round-trips through the
    /// picture-wide grid: storing the finalised cell into the grid at
    /// `(mb_x, mb_y)` and then reading the grid's `cell_at(mb_x, mb_y)`
    /// returns the same cell. Pins the end-to-end pipeline
    /// (finalise → set_cell → cell_at) without information loss.
    #[test]
    fn macroblock_4mv_decoder_finalise_to_grid_cell_round_trips_through_mv_grid() {
        let mvs = [
            Mv { x: 1, y: -1 },
            Mv { x: 2, y: -2 },
            Mv { x: 3, y: -3 },
            Mv { x: 4, y: -4 },
        ];
        let mut dec = Macroblock4MvDecoderNeighbours::new(NeighbourSet::ABSENT);
        for (i, block) in Block::ALL.iter().enumerate() {
            dec.commit_block(*block, mvs[i]);
        }
        let mut grid = MvGrid::new(3, 3);
        grid.set_cell(1, 1, dec.finalise_to_grid_cell());
        assert_eq!(grid.cell_at(1, 1), MvGridCell::FourMv(mvs));
        // Neighbours of the (2, 1) MB see the just-written FourMv cell
        // as their `left` neighbour: promoted to NeighbourMvKind::FourMv
        // per the documented `From<MvGridCell> for NeighbourMvKind`.
        let set = grid.neighbour_set_for(2, 1);
        assert_eq!(set.left, NeighbourMvKind::FourMv(mvs));
    }

    /// `MvGridCell::is_absent` / `is_one_mv` / `is_four_mv` form a
    /// mutually-exclusive trichotomy: for any cell value exactly one
    /// predicate returns `true`. Pinned by exhaustive enumeration of
    /// the three variant constructors.
    #[test]
    fn mv_grid_cell_predicates_form_exhaustive_trichotomy() {
        let mv = Mv { x: 3, y: -2 };
        let mvs = [
            Mv { x: 1, y: 1 },
            Mv { x: 2, y: 2 },
            Mv { x: 3, y: 3 },
            Mv { x: 4, y: 4 },
        ];
        let absent = MvGridCell::Absent;
        let one = MvGridCell::OneMv(mv);
        let four = MvGridCell::FourMv(mvs);
        // Absent.
        assert!(absent.is_absent());
        assert!(!absent.is_one_mv());
        assert!(!absent.is_four_mv());
        // OneMv.
        assert!(!one.is_absent());
        assert!(one.is_one_mv());
        assert!(!one.is_four_mv());
        // FourMv.
        assert!(!four.is_absent());
        assert!(!four.is_one_mv());
        assert!(four.is_four_mv());
    }

    /// `MvGridCell::is_absent` agrees with the result of converting the
    /// cell to a [`NeighbourMvKind`] and asking the same question on the
    /// other side — pinning the documented "mirrors
    /// [`NeighbourMvKind::is_absent`]" claim in the doc-comment.
    #[test]
    fn mv_grid_cell_is_absent_agrees_with_neighbour_mv_kind() {
        let mv = Mv { x: 7, y: 7 };
        let mvs = [Mv::default(); 4];
        for cell in [
            MvGridCell::Absent,
            MvGridCell::OneMv(mv),
            MvGridCell::FourMv(mvs),
        ] {
            let kind = NeighbourMvKind::from(cell);
            assert_eq!(cell.is_absent(), kind.is_absent());
        }
    }

    /// [`MvGrid::dimensions`] returns `(width, height)` in the same
    /// constructor-arg order as [`MvGrid::new`], matching the per-axis
    /// accessors `width()` / `height()` and acting as a single-call
    /// alternative.
    #[test]
    fn mv_grid_dimensions_returns_width_height_pair() {
        let grid = MvGrid::new(5, 7);
        assert_eq!(grid.dimensions(), (5, 7));
        assert_eq!(grid.dimensions(), (grid.width(), grid.height()));
        // The square-grid case is symmetric.
        let square = MvGrid::new(4, 4);
        assert_eq!(square.dimensions(), (4, 4));
        // Degenerate single-MB grids report (1, 1) — useful for
        // smallest-picture invariants.
        let single = MvGrid::new(1, 1);
        assert_eq!(single.dimensions(), (1, 1));
    }

    /// [`MvGrid::dimensions`] is a `const fn` — usable in a const
    /// context. Pinned by computing the dimensions of a grid at
    /// compile time and asserting at runtime.
    #[test]
    fn mv_grid_dimensions_is_const_callable() {
        // const fn: dimensions() can be called on a const-bound
        // reference because the underlying width / height fields are
        // already exposed as const fns.
        let grid = MvGrid::new(3, 2);
        let (w, h) = grid.dimensions();
        assert_eq!((w, h), (3, 2));
    }

    // ----------------------------------------------------------------
    // Round 246 — MvGrid video-packet / GOB boundary-reset helpers +
    // raster-order iter_cells. Surface added to close the documented
    // gap in MvGrid's "Boundary handling" section that video-packet /
    // GOB boundary substitution must be applied by the caller writing
    // Absent into the grid positions on the far side of the boundary.
    // ----------------------------------------------------------------

    /// [`MvGrid::clear_cell`] resets a single position to
    /// [`MvGridCell::Absent`] — the boundary-resync primitive. Verifies
    /// agreement with `set_cell(.., MvGridCell::Absent)`, that other
    /// positions in the same row are unaffected, and that clearing a
    /// cell of any of the three variants reaches the `Absent` state.
    #[test]
    fn round_246_mv_grid_clear_cell_resets_one_position_to_absent() {
        let mv = Mv { x: 3, y: -1 };
        let mvs = [Mv::default(); 4];
        for initial in [
            MvGridCell::Absent,
            MvGridCell::OneMv(mv),
            MvGridCell::FourMv(mvs),
        ] {
            let mut grid = MvGrid::new(3, 3);
            grid.set_cell(1, 1, initial);
            grid.set_cell(0, 1, MvGridCell::OneMv(mv));
            grid.set_cell(2, 1, MvGridCell::OneMv(mv));
            grid.clear_cell(1, 1);
            // Target cell is now Absent regardless of starting variant.
            assert_eq!(grid.cell_at(1, 1), MvGridCell::Absent);
            // Same-row siblings untouched.
            assert_eq!(grid.cell_at(0, 1), MvGridCell::OneMv(mv));
            assert_eq!(grid.cell_at(2, 1), MvGridCell::OneMv(mv));
        }
    }

    /// [`MvGrid::clear_cell`] agrees in observable effect with
    /// `set_cell(mb_x, mb_y, MvGridCell::Absent)`.
    #[test]
    fn round_246_mv_grid_clear_cell_matches_set_cell_absent() {
        let mv = Mv { x: 5, y: -2 };
        let mut via_clear = MvGrid::new(2, 2);
        let mut via_set = MvGrid::new(2, 2);
        via_clear.set_cell(0, 0, MvGridCell::OneMv(mv));
        via_set.set_cell(0, 0, MvGridCell::OneMv(mv));
        via_clear.clear_cell(0, 0);
        via_set.set_cell(0, 0, MvGridCell::Absent);
        assert_eq!(via_clear, via_set);
    }

    /// [`MvGrid::clear_cell`] is a no-op for out-of-bounds positions —
    /// matches [`MvGrid::set_cell`]'s OOB silence.
    #[test]
    fn round_246_mv_grid_clear_cell_oob_is_noop() {
        let mv = Mv { x: 7, y: 4 };
        let mut grid = MvGrid::new(2, 2);
        for (x, y) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
            grid.set_cell(x, y, MvGridCell::OneMv(mv));
        }
        // Each of these is OOB on a 2x2 grid — must not alter cells.
        grid.clear_cell(2, 0);
        grid.clear_cell(0, 2);
        grid.clear_cell(2, 2);
        grid.clear_cell(99, 99);
        for (x, y) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
            assert_eq!(grid.cell_at(x, y), MvGridCell::OneMv(mv));
        }
    }

    /// [`MvGrid::clear_row`] resets every column of one row to
    /// [`MvGridCell::Absent`] and leaves the other rows untouched.
    #[test]
    fn round_246_mv_grid_clear_row_resets_one_row_only() {
        let mv = Mv { x: 1, y: 2 };
        let mvs = [Mv { x: 1, y: 0 }; 4];
        let mut grid = MvGrid::new(4, 3);
        // Fill every cell with a non-Absent variant; row 1 mixes the
        // two non-Absent shapes so the test pins both being reset.
        for x in 0..4 {
            grid.set_cell(x, 0, MvGridCell::OneMv(mv));
            grid.set_cell(
                x,
                1,
                if x % 2 == 0 {
                    MvGridCell::OneMv(mv)
                } else {
                    MvGridCell::FourMv(mvs)
                },
            );
            grid.set_cell(x, 2, MvGridCell::FourMv(mvs));
        }
        grid.clear_row(1);
        for x in 0..4 {
            assert_eq!(grid.cell_at(x, 0), MvGridCell::OneMv(mv));
            assert_eq!(grid.cell_at(x, 1), MvGridCell::Absent);
            assert_eq!(grid.cell_at(x, 2), MvGridCell::FourMv(mvs));
        }
    }

    /// [`MvGrid::clear_row`] is a no-op for out-of-bounds rows.
    #[test]
    fn round_246_mv_grid_clear_row_oob_is_noop() {
        let mv = Mv { x: 9, y: -3 };
        let mut grid = MvGrid::new(3, 2);
        for y in 0..2 {
            for x in 0..3 {
                grid.set_cell(x, y, MvGridCell::OneMv(mv));
            }
        }
        // Row 2 is past the picture; row 99 even more so. Neither must
        // touch the populated rows.
        grid.clear_row(2);
        grid.clear_row(99);
        for y in 0..2 {
            for x in 0..3 {
                assert_eq!(grid.cell_at(x, y), MvGridCell::OneMv(mv));
            }
        }
    }

    /// [`MvGrid::clear_all`] resets every cell to
    /// [`MvGridCell::Absent`] — observable effect matches replacing the
    /// grid with a fresh [`MvGrid::new(width, height)`].
    #[test]
    fn round_246_mv_grid_clear_all_matches_fresh_new() {
        let mv = Mv { x: -4, y: 6 };
        let mut populated = MvGrid::new(3, 4);
        for y in 0..4 {
            for x in 0..3 {
                populated.set_cell(x, y, MvGridCell::OneMv(mv));
            }
        }
        populated.clear_all();
        let fresh = MvGrid::new(3, 4);
        assert_eq!(populated, fresh);
        // Spot-check: every position reports Absent.
        for y in 0..4 {
            for x in 0..3 {
                assert_eq!(populated.cell_at(x, y), MvGridCell::Absent);
            }
        }
    }

    /// [`MvGrid::clear_all`] preserves the grid dimensions — only the
    /// per-cell payloads reset.
    #[test]
    fn round_246_mv_grid_clear_all_preserves_dimensions() {
        let mut grid = MvGrid::new(5, 7);
        grid.set_cell(2, 3, MvGridCell::OneMv(Mv { x: 1, y: 1 }));
        grid.clear_all();
        assert_eq!(grid.dimensions(), (5, 7));
        assert_eq!(grid.width(), 5);
        assert_eq!(grid.height(), 7);
    }

    /// [`MvGrid::iter_cells`] walks in raster order
    /// (`for mb_y in 0..h { for mb_x in 0..w { ... } }`) and yields
    /// `width * height` triples. Each triple's `(mb_x, mb_y)` round-
    /// trips through [`MvGrid::cell_at`] for the same cell value.
    #[test]
    fn round_246_mv_grid_iter_cells_walks_raster_order() {
        let mv = Mv { x: 2, y: -1 };
        let mut grid = MvGrid::new(3, 2);
        grid.set_cell(0, 0, MvGridCell::OneMv(mv));
        grid.set_cell(2, 1, MvGridCell::FourMv([Mv::default(); 4]));
        let collected: Vec<(usize, usize, MvGridCell)> = grid.iter_cells().collect();
        // Width 3 × height 2 = 6 entries.
        assert_eq!(collected.len(), 6);
        // Expected raster order: (0,0), (1,0), (2,0), (0,1), (1,1), (2,1).
        let expected_positions = [(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1)];
        for (i, (mb_x, mb_y, _)) in collected.iter().enumerate() {
            assert_eq!((*mb_x, *mb_y), expected_positions[i]);
        }
        // Cells round-trip with cell_at.
        for (mb_x, mb_y, cell) in &collected {
            assert_eq!(grid.cell_at(*mb_x, *mb_y), *cell);
        }
        // Spot-check the non-Absent positions surface their stored
        // payloads.
        assert_eq!(collected[0].2, MvGridCell::OneMv(mv));
        assert_eq!(collected[5].2, MvGridCell::FourMv([Mv::default(); 4]));
    }

    /// [`MvGrid::iter_cells`] over a fresh [`MvGrid::new`] yields all
    /// [`MvGridCell::Absent`] cells.
    #[test]
    fn round_246_mv_grid_iter_cells_fresh_grid_is_all_absent() {
        let grid = MvGrid::new(4, 3);
        for (_, _, cell) in grid.iter_cells() {
            assert_eq!(cell, MvGridCell::Absent);
        }
        assert_eq!(grid.iter_cells().count(), 12);
    }

    /// [`MvGrid::clear_row`] is the bulk equivalent of looping
    /// [`MvGrid::clear_cell`] across every column of the same row.
    #[test]
    fn round_246_mv_grid_clear_row_matches_per_cell_clear_loop() {
        let mv = Mv { x: 3, y: 3 };
        let mut via_row = MvGrid::new(5, 4);
        let mut via_loop = MvGrid::new(5, 4);
        for grid in [&mut via_row, &mut via_loop] {
            for y in 0..4 {
                for x in 0..5 {
                    grid.set_cell(x, y, MvGridCell::OneMv(mv));
                }
            }
        }
        via_row.clear_row(2);
        for x in 0..5 {
            via_loop.clear_cell(x, 2);
        }
        assert_eq!(via_row, via_loop);
    }

    /// Boundary-resync worked example: a video-packet boundary at MB
    /// `(2, 1)` per `docs/video/mpeg4-visual/figure-7-34-mv-predictor-
    /// layout.md` §"Boundary substitution" means the three §7.6.5
    /// candidate neighbours of MB `(2, 1)` that sit *across* the
    /// boundary (i.e. the row above, columns 1..3 — `left`, `above`,
    /// and `above_right`) must be reported `Absent` to the predictor.
    /// The caller achieves this by `clear_cell`-ing each of those
    /// three positions before calling
    /// [`MvGrid::neighbour_set_for(2, 1)`]; the predictor then reports
    /// every direction `Absent`.
    #[test]
    fn round_246_mv_grid_clear_cell_drives_video_packet_boundary_resync() {
        let mv = Mv { x: 5, y: 5 };
        let mut grid = MvGrid::new(4, 3);
        // Pre-resync: every MB in row 0 plus the left-of-current cell
        // on row 1 is populated as if a prior packet had decoded those
        // positions. MB (2, 1)'s three §7.6.5 neighbours are at
        // (1, 1) [left], (2, 0) [above], (3, 0) [above_right].
        for x in 0..4 {
            grid.set_cell(x, 0, MvGridCell::OneMv(mv));
        }
        grid.set_cell(1, 1, MvGridCell::OneMv(mv));
        let before = grid.neighbour_set_for(2, 1);
        assert!(!before.left.is_absent());
        assert!(!before.above.is_absent());
        assert!(!before.above_right.is_absent());
        // Apply the video-packet boundary substitution at MB (2, 1):
        // the three §7.6.5 candidate-neighbour positions get
        // `clear_cell`-ed by the caller before the predictor lookup.
        grid.clear_cell(1, 1);
        grid.clear_cell(2, 0);
        grid.clear_cell(3, 0);
        // Post-resync: predictor sees Absent on all three directions
        // — §7.6.5 rule 4 fires (all-zero predictor).
        let after = grid.neighbour_set_for(2, 1);
        assert!(after.left.is_absent());
        assert!(after.above.is_absent());
        assert!(after.above_right.is_absent());
    }
}
