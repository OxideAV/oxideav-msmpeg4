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
    /// spec/04 §4.4 when AC prediction is enabled for the MB:
    ///   * left-predicted  → alternate-horizontal (MPEG-4 §7.4.1.2 "row scan")
    ///   * top-predicted   → alternate-vertical   (MPEG-4 §7.4.1.2 "column scan")
    ///
    /// When AC prediction is disabled at the MB level, the caller must
    /// use [`Scan::Zigzag`] instead — this function never returns zigzag.
    pub fn ac_scan(self) -> Scan {
        match self {
            PredDir::FromLeft => Scan::AlternateHorizontal,
            PredDir::FromTop => Scan::AlternateVertical,
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
    // MPEG-4 §7.4.3 gradient rule.
    if (a - d).abs() < (a - b).abs() {
        DcPrediction {
            predictor: a,
            direction: PredDir::FromLeft,
        }
    } else {
        DcPrediction {
            predictor: b,
            direction: PredDir::FromTop,
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
        }
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
        // Tie: |A-D| == |A-B| == 0 → else-branch → top.
        let p = predict_dc(None, None, None);
        assert_eq!(p.predictor, NEUTRAL_DC);
        assert_eq!(p.direction, PredDir::FromTop);
    }

    #[test]
    fn strong_horizontal_gradient_picks_left() {
        // D=100, A=200, B=500 → |A-D|=100, |A-B|=300 → left.
        let p = predict_dc(Some(200), Some(500), Some(100));
        assert_eq!(p.predictor, 200);
        assert_eq!(p.direction, PredDir::FromLeft);
        assert_eq!(p.direction.ac_scan(), Scan::AlternateHorizontal);
    }

    #[test]
    fn strong_vertical_gradient_picks_top() {
        // D=100, A=500, B=200 → |A-D|=400, |A-B|=300 → top.
        let p = predict_dc(Some(500), Some(200), Some(100));
        assert_eq!(p.predictor, 200);
        assert_eq!(p.direction, PredDir::FromTop);
        assert_eq!(p.direction.ac_scan(), Scan::AlternateVertical);
    }

    #[test]
    fn dc_cache_luma_roundtrip() {
        let mut c = DcCache::new(2, 2); // 4x4 luma block grid
        c.luma_set(0, 0, 1024);
        c.luma_set(1, 0, 2048);
        c.luma_set(0, 1, 512);
        // predict block (1, 1): A = (0,1) = 512, B = (1,0) = 2048, D = (0,0) = 1024.
        let p = c.predict_luma(1, 1);
        // |A-D| = |512 - 1024| = 512, |A-B| = |512 - 2048| = 1536. left wins.
        assert_eq!(p.predictor, 512);
        assert_eq!(p.direction, PredDir::FromLeft);
    }

    #[test]
    fn dc_cache_chroma_isolated_per_plane() {
        let mut c = DcCache::new(2, 2);
        c.chroma_set(false, 0, 0, 800); // Cb
        c.chroma_set(true, 0, 0, 200); // Cr
        let pcb = c.predict_chroma(false, 1, 1);
        let pcr = c.predict_chroma(true, 1, 1);
        // Only (0,0) is set; (0,1), (1,0), (1,1) are None → neutrals.
        // D=800, A=neutral(1024), B=neutral(1024). |A-D|=224, |A-B|=0 → top.
        assert_eq!(pcb.direction, PredDir::FromTop);
        // For Cr: D=200, neutrals otherwise. |1024-200|=824, |1024-1024|=0 → top.
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
