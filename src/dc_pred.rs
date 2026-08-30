//! MPEG-4 §7.4.3 DC spatial predictor for MS-MPEG4 v3 intra blocks.
//!
//! MS-MPEG4v3 inherits MPEG-4 Part 2's §7.4.3 DC gradient rule verbatim
//! (spec/03 §1.3, spec/04 §4.4). For each intra 8×8 block the decoder
//! compares gradients between three already-decoded neighbours:
//!
//! ```text
//!   +---+---+
//!   | D | B |
//!   +---+---+
//!   | A | X |    X = current block
//!   +---+---+
//! ```
//!
//! where:
//!   * `A` = left neighbour (same row)
//!   * `B` = top neighbour (same column)
//!   * `D` = top-left diagonal neighbour
//!
//! Rule:
//!   * if `|A - D| < |A - B|` → predict from `A` (left) → horizontal-
//!     prediction direction;
//!   * else → predict from `B` (top) → vertical-prediction direction.
//!
//! Missing neighbours (picture boundary, or not yet decoded in raster
//! scan order) are substituted with the neutral value `1024`. This is
//! MPEG-4 Part 2 §7.4.3's handling and matches MSMPEG4 v3's disassembly
//! (spec/03 §1.3 cites the gradient comparison at
//! `1c20aef0..1c20af2c`; boundary replacement is the same as the MV
//! predictor's zero-substitution pattern documented in spec/99 §3.2.3).
//!
//! The prediction direction feeds both:
//!   1. The DC reconstruction: `DC_reconstructed = predictor + diff * scaler`.
//!   2. The AC-scan dispatcher (spec/04 §4.4): left-predicted blocks use
//!      the alternate-horizontal scan; top-predicted blocks use the
//!      alternate-vertical scan; when MB-level AC prediction is off,
//!      the scan is always zigzag.

use crate::ac::Scan;

/// Neutral DC value used for missing neighbours. Corresponds to the
/// mid-grey reconstruction at q=8 where `dc_scaler(q=8) = 16` and
/// `1024 / 16 = 64 → decoded pel ≈ 128` (the unsigned-8 mid).
pub const NEUTRAL_DC: i32 = 1024;

/// Which neighbour won the gradient comparison.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PredDir {
    /// Predict from the left (A) neighbour.
    FromLeft,
    /// Predict from the top (B) neighbour.
    FromTop,
}

impl PredDir {
    /// Map the DC prediction direction to the AC-scan selection per
    /// `docs/video/msmpeg4/spec/03-corrections.md` §1.1 / §1.2:
    ///   * top-predicted  (vertical predictor wins)  → **alternate-horizontal**
    ///     scan (binary VMA `0x1c261140`)
    ///   * left-predicted (horizontal predictor wins) → **alternate-vertical**
    ///     scan (binary VMA `0x1c261240`)
    ///
    /// This matches the dispatch helper at `1c20de2e`:
    ///   * `[mb+0x2c] = 1` (vertical pred wins, predict from TOP) → alt-horz
    ///   * `[mb+0x2c] = 0` (horizontal pred wins, predict from LEFT) → alt-vert
    ///
    /// (This is the standard MPEG-4 Part 2 §7.4.5.4 mapping: when AC
    /// coefficients in the first ROW are predicted from top, the energy
    /// concentrates along the row, so the alternate-horizontal scan —
    /// which traverses rows first — packs non-zero coefficients toward
    /// the start of the scan. Symmetrically for the column case.)
    ///
    /// When AC prediction is disabled at the MB level, the caller must
    /// use [`Scan::Zigzag`] instead — this function never returns zigzag.
    pub fn ac_scan(self) -> Scan {
        match self {
            PredDir::FromLeft => Scan::AlternateVertical,
            PredDir::FromTop => Scan::AlternateHorizontal,
        }
    }
}

/// Predicted DC + direction for one block, given three neighbour DC
/// values (any of which may be `None` at picture boundaries).
///
/// Missing neighbours are replaced with [`NEUTRAL_DC`]. The decision
/// rule is MPEG-4 §7.4.3's gradient test.
#[derive(Clone, Copy, Debug)]
pub struct DcPrediction {
    pub predictor: i32,
    pub direction: PredDir,
}

pub fn predict_dc(a_left: Option<i32>, b_top: Option<i32>, d_tl: Option<i32>) -> DcPrediction {
    let a = a_left.unwrap_or(NEUTRAL_DC);
    let b = b_top.unwrap_or(NEUTRAL_DC);
    let d = d_tl.unwrap_or(NEUTRAL_DC);
    // MPEG-4 §7.4.3 gradient rule (and `docs/video/msmpeg4/spec/03-corrections.md`
    // §1.3): compare `|A - D|` (horizontal gradient along the top-left
    // pair) with `|D - B|` (vertical gradient along the top pair).
    //
    //   |A - D| <= |D - B| → the horizontal gradient is smaller-or-equal
    //                        → predict from TOP (vertical predictor
    //                        wins). Ties predict from TOP.
    //   else               → predict from LEFT.
    //
    // Round 420: the previous reading had the two branches swapped
    // (and ties going the other way). Both were refuted empirically
    // against the staged real-content fixtures: with the swapped rule
    // the very first I-frame row of `div4.avi` mis-predicts block
    // (1, 0) of MB (0, 0) (+32 differential lands on the wrong
    // predictor), and the tie case first bites at MB (2, 0) block 2
    // (|A−D| == |D−B| == 10 must resolve to TOP to reconstruct the
    // reference pixel means). The corrected rule reconstructs every
    // luma/chroma DC of the first eight MB columns of both DIV3 AVI
    // fixtures exactly (see `tests/microsoft_fixtures.rs` ground
    // truth and CHANGELOG r420).
    if (a - d).abs() <= (d - b).abs() {
        DcPrediction {
            predictor: b,
            direction: PredDir::FromTop,
        }
    } else {
        DcPrediction {
            predictor: a,
            direction: PredDir::FromLeft,
        }
    }
}

/// Per-block DC cache indexed in block-grid coordinates.
///
/// The grid has one entry per 8×8 block in the picture:
///   * luma plane: `(2 * mb_w) × (2 * mb_h)` blocks,
///   * chroma planes: `mb_w × mb_h` blocks each.
///
/// `None` at a position means "no decoded DC here yet" (i.e. outside
/// the picture, or not yet visited in raster order) — callers should
/// treat it as the neutral-DC substitution.
pub struct DcCache {
    pub luma_w: usize,
    pub luma_h: usize,
    pub chroma_w: usize,
    pub chroma_h: usize,
    luma: Vec<Option<i32>>,
    cb: Vec<Option<i32>>,
    cr: Vec<Option<i32>>,
    luma_ac: Vec<AcEdges>,
    cb_ac: Vec<AcEdges>,
    cr_ac: Vec<AcEdges>,
}

/// The AC-prediction edges of one decoded intra block — its first
/// row (`row[k]` = quantised level at raster position `k`, `k` in
/// 1..8) and first column (`col[k]` = level at raster position
/// `8k`). Index 0 of both is unused. The values are the block's
/// **reconstructed quantised levels** (after its own AC prediction
/// was applied), which is what the next block predicts from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AcEdges {
    pub row: [i32; 8],
    pub col: [i32; 8],
}

impl AcEdges {
    /// Extract the edges from a block of quantised levels in raster
    /// order.
    pub fn from_levels(levels: &[i32; 64]) -> Self {
        let mut e = AcEdges::default();
        for k in 1..8 {
            e.row[k] = levels[k];
            e.col[k] = levels[8 * k];
        }
        e
    }

    /// The seven predicted AC levels for a block that predicts from
    /// this one in direction `dir`: the first row when predicting
    /// from the top neighbour, the first column when predicting from
    /// the left neighbour. Index 0 is zero.
    pub fn predicted(&self, dir: PredDir) -> [i32; 8] {
        match dir {
            PredDir::FromTop => self.row,
            PredDir::FromLeft => self.col,
        }
    }
}

impl DcCache {
    pub fn new(mb_w: usize, mb_h: usize) -> Self {
        let luma_w = mb_w * 2;
        let luma_h = mb_h * 2;
        let chroma_w = mb_w;
        let chroma_h = mb_h;
        Self {
            luma_w,
            luma_h,
            chroma_w,
            chroma_h,
            luma: vec![None; luma_w * luma_h],
            cb: vec![None; chroma_w * chroma_h],
            cr: vec![None; chroma_w * chroma_h],
            luma_ac: vec![AcEdges::default(); luma_w * luma_h],
            cb_ac: vec![AcEdges::default(); chroma_w * chroma_h],
            cr_ac: vec![AcEdges::default(); chroma_w * chroma_h],
        }
    }

    /// Record the AC edges of a decoded luma block.
    pub fn luma_ac_set(&mut self, x: usize, y: usize, edges: AcEdges) {
        if x < self.luma_w && y < self.luma_h {
            self.luma_ac[y * self.luma_w + x] = edges;
        }
    }

    /// Record the AC edges of a decoded chroma block.
    pub fn chroma_ac_set(&mut self, plane_is_cr: bool, x: usize, y: usize, edges: AcEdges) {
        if x < self.chroma_w && y < self.chroma_h {
            let plane = if plane_is_cr {
                &mut self.cr_ac
            } else {
                &mut self.cb_ac
            };
            plane[y * self.chroma_w + x] = edges;
        }
    }

    /// The AC prediction for luma block `(bx, by)` in direction
    /// `dir`: the neighbour's first row (from top) or first column
    /// (from left). All zero when the neighbour is outside the
    /// picture or holds no intra DC (an inter MB in a P-frame).
    pub fn ac_predict_luma(&self, bx: usize, by: usize, dir: PredDir) -> [i32; 8] {
        let (nx, ny) = match dir {
            PredDir::FromTop if by > 0 => (bx, by - 1),
            PredDir::FromLeft if bx > 0 => (bx - 1, by),
            _ => return [0; 8],
        };
        if self.luma_get(nx, ny).is_none() {
            return [0; 8];
        }
        self.luma_ac[ny * self.luma_w + nx].predicted(dir)
    }

    /// Chroma analogue of [`DcCache::ac_predict_luma`].
    pub fn ac_predict_chroma(
        &self,
        plane_is_cr: bool,
        bx: usize,
        by: usize,
        dir: PredDir,
    ) -> [i32; 8] {
        let (nx, ny) = match dir {
            PredDir::FromTop if by > 0 => (bx, by - 1),
            PredDir::FromLeft if bx > 0 => (bx - 1, by),
            _ => return [0; 8],
        };
        if self.chroma_get(plane_is_cr, nx, ny).is_none() {
            return [0; 8];
        }
        let plane = if plane_is_cr {
            &self.cr_ac
        } else {
            &self.cb_ac
        };
        plane[ny * self.chroma_w + nx].predicted(dir)
    }

    fn luma_get(&self, x: usize, y: usize) -> Option<i32> {
        if x >= self.luma_w || y >= self.luma_h {
            return None;
        }
        self.luma[y * self.luma_w + x]
    }

    fn chroma_get(&self, plane_is_cr: bool, x: usize, y: usize) -> Option<i32> {
        if x >= self.chroma_w || y >= self.chroma_h {
            return None;
        }
        let plane = if plane_is_cr { &self.cr } else { &self.cb };
        plane[y * self.chroma_w + x]
    }

    pub fn luma_set(&mut self, x: usize, y: usize, dc: i32) {
        if x < self.luma_w && y < self.luma_h {
            self.luma[y * self.luma_w + x] = Some(dc);
        }
    }

    pub fn chroma_set(&mut self, plane_is_cr: bool, x: usize, y: usize, dc: i32) {
        if x < self.chroma_w && y < self.chroma_h {
            let plane = if plane_is_cr {
                &mut self.cr
            } else {
                &mut self.cb
            };
            plane[y * self.chroma_w + x] = Some(dc);
        }
    }

    /// Predict DC for the luma block at `(bx, by)` in block-grid
    /// coordinates (so the top-left luma block of MB (0,0) is (0,0),
    /// etc.). Safely handles picture boundaries.
    pub fn predict_luma(&self, bx: usize, by: usize) -> DcPrediction {
        let a = if bx > 0 {
            self.luma_get(bx - 1, by)
        } else {
            None
        };
        let b = if by > 0 {
            self.luma_get(bx, by - 1)
        } else {
            None
        };
        let d = if bx > 0 && by > 0 {
            self.luma_get(bx - 1, by - 1)
        } else {
            None
        };
        predict_dc(a, b, d)
    }

    /// Predict DC for a chroma block at `(bx, by)` (one block per MB
    /// in 4:2:0; `plane_is_cr = false` for Cb, `true` for Cr).
    pub fn predict_chroma(&self, plane_is_cr: bool, bx: usize, by: usize) -> DcPrediction {
        let a = if bx > 0 {
            self.chroma_get(plane_is_cr, bx - 1, by)
        } else {
            None
        };
        let b = if by > 0 {
            self.chroma_get(plane_is_cr, bx, by - 1)
        } else {
            None
        };
        let d = if bx > 0 && by > 0 {
            self.chroma_get(plane_is_cr, bx - 1, by - 1)
        } else {
            None
        };
        predict_dc(a, b, d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_neutral_predicts_from_top() {
        // Tie: |A-D| == |D-B| == 0 → else-branch → top.
        let p = predict_dc(None, None, None);
        assert_eq!(p.predictor, NEUTRAL_DC);
        assert_eq!(p.direction, PredDir::FromTop);
    }

    #[test]
    fn small_horizontal_gradient_picks_top() {
        // D=100, A=200, B=500 → |A-D|=100 <= |D-B|=400 → TOP wins
        // (round 420 corrected rule; the old reading picked LEFT here
        // and was refuted on the real-content fixtures).
        // Per spec/03 §1.1, predict-from-TOP (vertical pred wins) ⇒
        // alt-horizontal scan (binary VMA 0x1c261140).
        let p = predict_dc(Some(200), Some(500), Some(100));
        assert_eq!(p.predictor, 500);
        assert_eq!(p.direction, PredDir::FromTop);
        assert_eq!(p.direction.ac_scan(), Scan::AlternateHorizontal);
    }

    #[test]
    fn large_horizontal_gradient_picks_left() {
        // D=100, A=500, B=200 → |A-D|=400 > |D-B|=100 → LEFT wins.
        // Per spec/03 §1.1, predict-from-LEFT (horizontal pred wins) ⇒
        // alt-vertical scan (binary VMA 0x1c261240).
        let p = predict_dc(Some(500), Some(200), Some(100));
        assert_eq!(p.predictor, 500);
        assert_eq!(p.direction, PredDir::FromLeft);
        assert_eq!(p.direction.ac_scan(), Scan::AlternateVertical);
    }

    #[test]
    fn tie_gradient_picks_top() {
        // A=764, B=744, D=754 → |A-D| == |D-B| == 10 → TOP (the tie
        // case pinned by MB (2, 0) block 2 of the div4.avi I-frame).
        let p = predict_dc(Some(764), Some(744), Some(754));
        assert_eq!(p.predictor, 744);
        assert_eq!(p.direction, PredDir::FromTop);
    }

    #[test]
    fn dc_cache_luma_roundtrip() {
        let mut c = DcCache::new(2, 2); // 4x4 luma block grid
        c.luma_set(0, 0, 1024);
        c.luma_set(1, 0, 2048);
        c.luma_set(0, 1, 512);
        // predict block (1, 1): A = (0,1) = 512, B = (1,0) = 2048, D = (0,0) = 1024.
        let p = c.predict_luma(1, 1);
        // |A-D| = |512 - 1024| = 512 <= |D-B| = |1024 - 2048| = 1024 → TOP.
        assert_eq!(p.predictor, 2048);
        assert_eq!(p.direction, PredDir::FromTop);
    }

    #[test]
    fn dc_cache_chroma_isolated_per_plane() {
        let mut c = DcCache::new(2, 2);
        c.chroma_set(false, 0, 0, 800); // Cb
        c.chroma_set(true, 0, 0, 200); // Cr
        let pcb = c.predict_chroma(false, 1, 1);
        let pcr = c.predict_chroma(true, 1, 1);
        // Only (0,0) is set; (0,1), (1,0), (1,1) are None → neutrals.
        // D=800, A=neutral(1024), B=neutral(1024). |A-D|=224, |D-B|=224 → tie → top.
        assert_eq!(pcb.direction, PredDir::FromTop);
        // For Cr: D=200, neutrals otherwise. |1024-200|=824, |200-1024|=824 → tie → top.
        assert_eq!(pcr.direction, PredDir::FromTop);
    }

    #[test]
    fn dc_cache_boundary_skipped_neighbours_are_neutral() {
        let c = DcCache::new(2, 2);
        // Block (0, 0) has no neighbours at all → all neutral → from-top.
        let p = c.predict_luma(0, 0);
        assert_eq!(p.predictor, NEUTRAL_DC);
        assert_eq!(p.direction, PredDir::FromTop);
    }
}
