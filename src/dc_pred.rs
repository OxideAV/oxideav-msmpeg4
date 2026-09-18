//! Intra DC / AC prediction context for MS-MPEG4 intra blocks
//! (spec/19 §2, spec/18 §7).
//!
//! Every 8×8 intra block owns a record holding its reconstructed DC
//! **level** (the quantised value `dc_level = dc(chosen) + dc_diff`,
//! spec/19 §2.3) and the first row / first column of its reconstructed
//! AC levels (§2.4). For each new block the decoder picks a predictor
//! record among the left (`L`), top (`T`) and top-left (`TL`)
//! neighbours:
//!
//! ```text
//!   +----+---+
//!   | TL | T |
//!   +----+---+
//!   | L  | X |    X = current block
//!   +----+---+
//! ```
//!
//! and decides the direction with the gradient rule of §2.3
//! (`0x1c215acc..0x1c215ae5`):
//!
//!   * `|dc(TL) − dc(T)| ≥ |dc(TL) − dc(L)|` → predict from `T` (top);
//!     ties go to TOP (v ≤ 3; v4 uses a strict `>`);
//!   * otherwise → predict from `L` (left).
//!
//! A neighbour that is **unavailable** — outside the picture, above a
//! slice boundary (§3), or, in a P-frame, an inter or skipped
//! macroblock (spec/18 §7) — is replaced by the frame's **default
//! record**: AC = 0 and `DC = floor(1024 / dc_scaler + 0.5)` (§2.2,
//! [`default_dc_level`]), computed per plane class (luma / chroma) from
//! the picture's PQUANT. The comparison and the prediction both happen
//! in the level domain; the coefficient is `dc_level · dc_scaler`.
//!
//! The direction also selects the alternate scan and which strip of
//! the chosen record feeds the AC predictor when the macroblock's
//! `ac_pred` flag is set (§2.4, [`PredDir::ac_scan`]).

use crate::ac::Scan;

/// The default record's DC level for a plane whose DC scaler is
/// `dc_scaler` (spec/19 §2.2): `floor(1024 / dc_scaler + 0.5)`,
/// i.e. round-half-up of `1024 / dc_scaler` — the vendor computes it
/// as `fild dc_scaler; fdivr 1024.0; fsubr −0.5; fistp` under a
/// truncating control word. Measured (round 28, hardware-grade):
/// scaler 16 → 64, 10 → 102, 8 → 128, 46 → 22, and the discriminating
/// 21 → 49, 13 → 79, 18 → 57, 11 → 93, 9 → 114 (plain `floor` would
/// give 48 / 78 / 56 / 93 / 113).
pub fn default_dc_level(dc_scaler: u32) -> i32 {
    let s = dc_scaler.max(1) as i32;
    (2048 + s) / (2 * s)
}

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

/// Predicted DC **level** + direction for one block, given the three
/// neighbour records' DC levels (`None` = unavailable).
#[derive(Clone, Copy, Debug)]
pub struct DcPrediction {
    /// The chosen neighbour's DC level (or the default record's when
    /// that side is unavailable) — added to the decoded differential.
    pub predictor: i32,
    pub direction: PredDir,
}

/// spec/19 §2.3 direction rule in the level domain: unavailable sides
/// take `default` (the plane's default record, [`default_dc_level`]);
/// `|dc(TL) − dc(T)| ≥ |dc(TL) − dc(L)|` → TOP, else LEFT.
pub fn predict_dc(
    a_left: Option<i32>,
    b_top: Option<i32>,
    d_tl: Option<i32>,
    default: i32,
) -> DcPrediction {
    let a = a_left.unwrap_or(default);
    let b = b_top.unwrap_or(default);
    let d = d_tl.unwrap_or(default);
    // `0x1c215acc..0x1c215ae5` (spec/19 §2.3): |TL − T| ≥ |TL − L| →
    // TOP, ties to TOP for v ≤ 3 (measured on a genuine `5 ≥ 5`).
    // Round 420 had pinned the same rule (then in the coefficient
    // domain) on the DIV3 fixtures' first I-frame rows; round 459
    // moves it to the level domain so the default record compares on
    // equal footing with real neighbours (a level-domain tie against
    // the default is not a coefficient-domain tie when `1024` is not
    // a multiple of the scaler).
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
    /// Default-record DC level for luma blocks (spec/19 §2.2).
    pub luma_default: i32,
    /// Default-record DC level for chroma blocks.
    pub chroma_default: i32,
    /// `Some(PQUANT)` for a v3 context (DC scaler = MPEG-4 Table 7-2
    /// of PQUANT), `None` for v1/v2 (constant scaler 8).
    v3_quant: Option<u32>,
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
    /// An empty context whose default records hold `luma_default` /
    /// `chroma_default` (DC levels, [`default_dc_level`]).
    pub fn new(mb_w: usize, mb_h: usize, luma_default: i32, chroma_default: i32) -> Self {
        let luma_w = mb_w * 2;
        let luma_h = mb_h * 2;
        let chroma_w = mb_w;
        let chroma_h = mb_h;
        Self {
            luma_w,
            luma_h,
            chroma_w,
            chroma_h,
            luma_default,
            chroma_default,
            v3_quant: None,
            luma: vec![None; luma_w * luma_h],
            cb: vec![None; chroma_w * chroma_h],
            cr: vec![None; chroma_w * chroma_h],
            luma_ac: vec![AcEdges::default(); luma_w * luma_h],
            cb_ac: vec![AcEdges::default(); chroma_w * chroma_h],
            cr_ac: vec![AcEdges::default(); chroma_w * chroma_h],
        }
    }

    /// Context for a v3 picture at `quant`: the default records follow
    /// the MPEG-4 Table 7-2 luma / chroma DC scalers of PQUANT
    /// (spec/19 §2.2, `0x1c212a2b..0x1c212a7b`).
    pub fn for_v3_quant(mb_w: usize, mb_h: usize, quant: u32) -> Self {
        let mut c = Self::new(
            mb_w,
            mb_h,
            default_dc_level(crate::iq::dc_scaler(0, quant)),
            default_dc_level(crate::iq::dc_scaler(4, quant)),
        );
        c.v3_quant = Some(quant);
        c
    }

    /// The DC scaler this context's picture applies to `block_idx`
    /// (0..=3 luma, 4..=5 chroma): the v3 PQUANT table, or the v1/v2
    /// constant 8.
    pub fn dc_scaler(&self, block_idx: usize) -> u32 {
        match self.v3_quant {
            Some(q) => crate::iq::dc_scaler(block_idx, q),
            None => crate::iq::DC_SCALER_V1V2,
        }
    }

    /// Context for a v1 / v2 picture: the DC scaler is the constant 8
    /// (spec/99 §10.2 slot `0x128 / 0x12c`), so both default records
    /// hold 128.
    pub fn for_v1v2(mb_w: usize, mb_h: usize) -> Self {
        let d = default_dc_level(crate::iq::DC_SCALER_V1V2);
        Self::new(mb_w, mb_h, d, d)
    }

    /// Forget every record (a slice boundary, spec/19 §3), keeping the
    /// default records.
    pub fn reset(&mut self) {
        let v3_quant = self.v3_quant;
        *self = Self::new(
            self.luma_w / 2,
            self.luma_h / 2,
            self.luma_default,
            self.chroma_default,
        );
        self.v3_quant = v3_quant;
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

    /// Record a decoded luma block's DC **level**.
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

    /// Predict the DC level for the luma block at `(bx, by)` in
    /// block-grid coordinates (so the top-left luma block of MB (0,0)
    /// is (0,0), etc.). Safely handles picture boundaries.
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
        predict_dc(a, b, d, self.luma_default)
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
        predict_dc(a, b, d, self.chroma_default)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_dc_level_is_round_half_up_of_1024_over_scaler() {
        // spec/19 §2.2 measured values (round 28, hardware-grade).
        for (scaler, want) in [
            (16, 64),
            (10, 102),
            (8, 128),
            (46, 22),
            (21, 49),
            (13, 79),
            (18, 57),
            (11, 93),
            (9, 114),
            (12, 85),
        ] {
            assert_eq!(default_dc_level(scaler), want, "scaler {scaler}");
            // Cross-check against the literal round-half-up.
            assert_eq!(
                default_dc_level(scaler),
                (1024.0 / scaler as f64 + 0.5).floor() as i32
            );
        }
    }

    #[test]
    fn all_neutral_predicts_from_top() {
        // Tie: |A-D| == |D-B| == 0 → top.
        let p = predict_dc(None, None, None, 64);
        assert_eq!(p.predictor, 64);
        assert_eq!(p.direction, PredDir::FromTop);
    }

    #[test]
    fn small_horizontal_gradient_picks_top() {
        // D=100, A=200, B=500 → |A-D|=100 <= |D-B|=400 → TOP wins.
        // Per spec/03 §1.1, predict-from-TOP (vertical pred wins) ⇒
        // alt-horizontal scan (binary VMA 0x1c261140).
        let p = predict_dc(Some(200), Some(500), Some(100), 64);
        assert_eq!(p.predictor, 500);
        assert_eq!(p.direction, PredDir::FromTop);
        assert_eq!(p.direction.ac_scan(), Scan::AlternateHorizontal);
    }

    #[test]
    fn large_horizontal_gradient_picks_left() {
        // D=100, A=500, B=200 → |A-D|=400 > |D-B|=100 → LEFT wins.
        // Per spec/03 §1.1, predict-from-LEFT (horizontal pred wins) ⇒
        // alt-vertical scan (binary VMA 0x1c261240).
        let p = predict_dc(Some(500), Some(200), Some(100), 64);
        assert_eq!(p.predictor, 500);
        assert_eq!(p.direction, PredDir::FromLeft);
        assert_eq!(p.direction.ac_scan(), Scan::AlternateVertical);
    }

    #[test]
    fn tie_gradient_picks_top() {
        // spec/19 §2.3: a genuine tie (`5 ≥ 5`) goes to TOP.
        let p = predict_dc(Some(764), Some(744), Some(754), 64);
        assert_eq!(p.predictor, 744);
        assert_eq!(p.direction, PredDir::FromTop);
    }

    #[test]
    fn only_left_available_uses_left_unless_it_equals_the_default() {
        // spec/19 §2.3: "left available, top and top-left unavailable:
        // 0 ≥ |D − dc(L)| holds only if dc(L) == D, so the left block
        // is used unless its DC equals the default."
        let p = predict_dc(Some(70), None, None, 64);
        assert_eq!(p.direction, PredDir::FromLeft);
        assert_eq!(p.predictor, 70);
        let p = predict_dc(Some(64), None, None, 64);
        assert_eq!(p.direction, PredDir::FromTop);
        assert_eq!(p.predictor, 64);
    }

    #[test]
    fn dc_cache_luma_roundtrip() {
        let mut c = DcCache::new(2, 2, 64, 102); // 4x4 luma block grid
        c.luma_set(0, 0, 60);
        c.luma_set(1, 0, 120);
        c.luma_set(0, 1, 30);
        // predict block (1, 1): A = (0,1) = 30, B = (1,0) = 120, D = (0,0) = 60.
        let p = c.predict_luma(1, 1);
        // |A-D| = 30 <= |D-B| = 60 → TOP.
        assert_eq!(p.predictor, 120);
        assert_eq!(p.direction, PredDir::FromTop);
    }

    #[test]
    fn dc_cache_chroma_isolated_per_plane() {
        let mut c = DcCache::new(2, 2, 64, 102);
        c.chroma_set(false, 0, 0, 80); // Cb
        c.chroma_set(true, 0, 0, 20); // Cr
        let pcb = c.predict_chroma(false, 1, 1);
        let pcr = c.predict_chroma(true, 1, 1);
        // Only (0,0) is set; (0,1), (1,0), (1,1) are None → defaults.
        // D=80, A=B=102. |A-D|=22, |D-B|=22 → tie → top.
        assert_eq!(pcb.direction, PredDir::FromTop);
        assert_eq!(pcb.predictor, 102);
        // For Cr: D=20, defaults otherwise → tie → top.
        assert_eq!(pcr.direction, PredDir::FromTop);
    }

    #[test]
    fn dc_cache_boundary_skipped_neighbours_are_default() {
        let c = DcCache::for_v3_quant(2, 2, 8);
        assert_eq!((c.luma_default, c.chroma_default), (64, 102));
        // Block (0, 0) has no neighbours at all → all default → from-top.
        let p = c.predict_luma(0, 0);
        assert_eq!(p.predictor, 64);
        assert_eq!(p.direction, PredDir::FromTop);
        let p = c.predict_chroma(true, 0, 0);
        assert_eq!(p.predictor, 102);
        let v12 = DcCache::for_v1v2(1, 1);
        assert_eq!((v12.luma_default, v12.chroma_default), (128, 128));
    }

    #[test]
    fn reset_forgets_records_but_keeps_defaults() {
        let mut c = DcCache::for_v3_quant(2, 2, 13);
        assert_eq!((c.luma_default, c.chroma_default), (49, 79));
        c.luma_set(0, 0, 5);
        c.luma_ac_set(0, 0, AcEdges::from_levels(&[7; 64]));
        c.reset();
        assert_eq!(c.predict_luma(1, 0).predictor, 49);
        assert_eq!(c.ac_predict_luma(1, 0, PredDir::FromLeft), [0; 8]);
        assert_eq!((c.luma_default, c.chroma_default), (49, 79));
        assert_eq!((c.dc_scaler(0), c.dc_scaler(4)), (21, 13));
        assert_eq!(DcCache::for_v1v2(1, 1).dc_scaler(0), 8);
    }
}
