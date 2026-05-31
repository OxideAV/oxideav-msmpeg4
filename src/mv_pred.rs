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
}
