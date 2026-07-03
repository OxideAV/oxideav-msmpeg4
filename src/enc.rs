//! MS-MPEG4 v3 picture-level encoder.
//!
//! Produces bitstreams this crate's own [`crate::picture::decode_picture`]
//! reconstructs — every syntax element goes through the encode-side
//! inverses of the decode surfaces (`header::MsV3PictureHeader::write`,
//! `mcbpcy::encode_mcbpcy`, `mb::encode_intra_dc_diff_v3`,
//! `ac::encode_intra_ac` / `encode_inter_ac`, `mv::encode_mv_with_table`),
//! so encode → decode round-trips by construction over the same
//! extracted tables (spec/11 §5, spec/16 §1, spec/99 §3.1).
//!
//! The encoder mirrors the decoder's prediction state exactly:
//!
//! * the intra DC spatial predictor threads the same reconstructed
//!   (integer) DC values through a [`crate::dc_pred::DcCache`], so the
//!   per-block predictor and differential agree with what the decoder
//!   derives (MPEG-4 §7.4.3 gradient rule);
//! * the P-frame §7.6.5 median MV predictor is computed over the same
//!   [`crate::mv_pred::MvGrid`] the decoder maintains, and motion
//!   vectors are constrained to the spec/06 §3.5 toroidal window
//!   around that predictor (`mv::mv_component_reachable`);
//! * AC prediction is not used (`ac_pred = 0` on every MB), so every
//!   coded block walks the fixed zigzag scan.
//!
//! Table choices are fixed per frame: intra luma = G5
//! (`ac_luma_sel = 2`), chroma = G4 (`ac_chroma_sel = 2`), default
//! intra-DC pair (`dc_size_sel = 0`), default joint-MV VLC
//! (`mv_table_sel = 0`). Inter residuals always use G4 (the alphabet
//! is not frame-selectable on the inter path, spec/04 §1.3).

use oxideav_core::bits::BitWriter;
use oxideav_core::{Error, Result};

use crate::ac::{encode_inter_ac, encode_intra_ac, AcVlcTable, Scan};
use crate::dc_pred::DcCache;
use crate::header::{MsV3PictureHeader, PictureType};
use crate::idct::fdct8x8_from_pels;
use crate::iq::{dc_scaler, quantise_block_h263};
use crate::mb::{encode_intra_dc_diff_v1v2, encode_intra_dc_diff_v3};
use crate::mc::{chroma_mv_from_luma, mc_block, RefPlane};
use crate::mcbpcy::{compose_cbp, encode_mcbpcy, encode_mcbpcy_v1, encode_mcbpcy_v2, V2FrameType};
use crate::mv::{
    encode_mv_v1v2, encode_mv_with_table, mv_component_reachable, mv_v1v2_component_reachable, Mv,
    MvTable,
};
use crate::mv_pred::{predict_block_mv, resolve_block_candidates, Block, MvGrid, MvGridCell};
use crate::picture::{MsV1V2Version, Picture, PictureDims};

/// Per-frame encoder settings.
#[derive(Clone, Copy, Debug)]
pub struct EncoderConfig {
    /// Frame-wide quantiser, 1..=31 (5-bit PQUANT).
    pub quant: u8,
    /// Half-pel motion-search range per component (0 = zero-MV only).
    /// The search window at each MB is `[-range, +range]` half-pel
    /// steps around the zero MV, additionally clipped to the spec/06
    /// §3.5 toroidal window around the §7.6.5 predictor.
    pub mv_search_range: u8,
}

impl Default for EncoderConfig {
    fn default() -> Self {
        Self {
            quant: 4,
            mv_search_range: 8,
        }
    }
}

/// The fixed per-frame table selectors this encoder emits (see the
/// module doc): G5 intra luma, G4 chroma, default DC pair, default MV
/// VLC.
const AC_LUMA_SEL: u8 = 2; // G5
const AC_CHROMA_SEL: u8 = 2; // G4
const DC_SIZE_SEL: u8 = 0;

/// One quantised 8×8 block of an intra MB, ready for serialisation.
struct IntraBlockPlan {
    dc_diff: i32,
    /// Bitstream AC levels in natural order (position 0 unused).
    levels: [i32; 64],
    coded: bool,
}

/// Encode one picture as a v3 **I-frame**. `input` must carry
/// MB-aligned planes for `dims` (the layout [`Picture::alloc`]
/// produces). Returns the frame's bitstream, decodable by
/// [`crate::picture::decode_picture`] with the same `dims`.
pub fn encode_iframe_v3(
    input: &Picture,
    dims: PictureDims,
    config: &EncoderConfig,
) -> Result<Vec<u8>> {
    validate_input(input, dims, config)?;
    let (mb_w, mb_h) = dims.mb_dims();
    let quant = config.quant as u32;

    let mut bw = BitWriter::new();
    let hdr = MsV3PictureHeader {
        picture_type: PictureType::I,
        quant: config.quant,
        ac_chroma_sel: AC_CHROMA_SEL,
        ac_luma_sel: AC_LUMA_SEL,
        dc_size_sel: DC_SIZE_SEL,
        mv_table_sel: 0,
    };
    hdr.write(&mut bw)?;

    let luma_ac = AcVlcTable::v3_intra_g5();
    let chroma_ac = AcVlcTable::g4_inter();
    let mut dc_cache = DcCache::new(mb_w, mb_h);

    for my in 0..mb_h {
        for mx in 0..mb_w {
            encode_intra_mb(
                &mut bw,
                input,
                &mut dc_cache,
                mx,
                my,
                quant,
                &luma_ac,
                &chroma_ac,
            )?;
        }
    }
    Ok(bw.finish())
}

/// Transform + quantise all 6 blocks of one intra MB, threading the DC
/// predictor exactly as the decoder will (§7.4.3 gradient over
/// reconstructed integer DCs). Version-independent: v1/v2/v3 share the
/// spatial DC predictor and the DC-scaler quantisation (spec/99 §4.4,
/// spec/16 §2 — only the DC-differential *codeword* scheme differs).
/// Returns the 6 block plans plus the (cbpy, cbp_cb, cbp_cr) split.
fn analyse_intra_mb(
    input: &Picture,
    dc_cache: &mut DcCache,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
) -> ([IntraBlockPlan; 6], u8, bool, bool) {
    let mut plans: [IntraBlockPlan; 6] = std::array::from_fn(|_| IntraBlockPlan {
        dc_diff: 0,
        levels: [0i32; 64],
        coded: false,
    });
    for (block_idx, plan) in plans.iter_mut().enumerate() {
        let pels = extract_block(input, mb_x, mb_y, block_idx);
        let mut f = [0.0f32; 64];
        fdct8x8_from_pels(&pels, &mut f);

        let (bx, by) = block_grid_pos(block_idx, mb_x, mb_y);
        let pred = match block_idx {
            0..=3 => dc_cache.predict_luma(bx, by),
            4 => dc_cache.predict_chroma(false, bx, by),
            5 => dc_cache.predict_chroma(true, bx, by),
            _ => unreachable!(),
        };

        let scaler = dc_scaler(block_idx, quant) as i32;
        let dc_diff =
            (((f[0] - pred.predictor as f32) / scaler as f32).round() as i32).clamp(-255, 255);
        let dc_recon = pred.predictor + dc_diff * scaler;
        match block_idx {
            0..=3 => dc_cache.luma_set(bx, by, dc_recon),
            4 => dc_cache.chroma_set(false, bx, by, dc_recon),
            5 => dc_cache.chroma_set(true, bx, by, dc_recon),
            _ => unreachable!(),
        }

        plan.dc_diff = dc_diff;
        let nz = quantise_block_h263(&f, quant, 1, &mut plan.levels);
        plan.coded = nz > 0;
    }
    let cbpy = (plans[0].coded as u8) << 3
        | (plans[1].coded as u8) << 2
        | (plans[2].coded as u8) << 1
        | (plans[3].coded as u8);
    let cbp_cb = plans[4].coded;
    let cbp_cr = plans[5].coded;
    (plans, cbpy, cbp_cb, cbp_cr)
}

/// Serialise the 6 planned blocks of an intra MB: per-block DC
/// differential through the version's codeword scheme, then the fixed
/// zigzag AC walk (ac_pred = 0 on every MB this encoder emits) for
/// each coded block.
fn write_intra_blocks(
    bw: &mut BitWriter,
    plans: &[IntraBlockPlan; 6],
    dc_scheme: DcScheme,
    luma_ac: &AcVlcTable,
    chroma_ac: &AcVlcTable,
) -> Result<()> {
    for (block_idx, plan) in plans.iter().enumerate() {
        match dc_scheme {
            DcScheme::V3 => encode_intra_dc_diff_v3(bw, block_idx, DC_SIZE_SEL, plan.dc_diff)?,
            DcScheme::V1V2 => encode_intra_dc_diff_v1v2(bw, block_idx, plan.dc_diff)?,
        }
        if plan.coded {
            let table = if block_idx <= 3 { luma_ac } else { chroma_ac };
            encode_intra_ac(bw, &plan.levels, Scan::Zigzag, table, 1)?;
        }
    }
    Ok(())
}

/// Which intra-DC differential codeword scheme a frame uses: the v3
/// 120-entry direct-value VLC (spec/07 §5.4) or the v1/v2 H.263
/// size+value scheme (spec/16 §2).
#[derive(Clone, Copy)]
enum DcScheme {
    V3,
    V1V2,
}

/// Analyse + serialise one v3 intra MB (I-frame path; this encoder
/// never emits intra-in-P MBs).
#[allow(clippy::too_many_arguments)]
fn encode_intra_mb(
    bw: &mut BitWriter,
    input: &Picture,
    dc_cache: &mut DcCache,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    luma_ac: &AcVlcTable,
    chroma_ac: &AcVlcTable,
) -> Result<()> {
    let (plans, cbpy, cbp_cb, cbp_cr) = analyse_intra_mb(input, dc_cache, mb_x, mb_y, quant);
    // Joint MCBPCY (I-type half: idx = cbp), then the ac_pred bit
    // (always 0: fixed zigzag).
    let cbp = compose_cbp(cbpy, cbp_cb, cbp_cr);
    encode_mcbpcy(bw, cbp)?;
    bw.write_bit(false); // ac_pred = 0
    write_intra_blocks(bw, &plans, DcScheme::V3, luma_ac, chroma_ac)
}

// ====================================================================
// P-frame encoding
// ====================================================================

/// Encode one picture as a v3 **P-frame** against `reference` — which
/// must be the *decoder-side reconstruction* of the previous frame
/// (what [`crate::picture::decode_picture`] produced for it), not the
/// original source, so encoder and decoder motion-compensate from
/// identical pels.
///
/// Per MB the encoder mirrors the decoder's §7.6.5 median predictor
/// over a shared [`MvGrid`], motion-searches a half-pel window
/// (`config.mv_search_range`, luma SAD, candidates clipped to the
/// spec/06 §3.5 toroidal window around the predictor), transforms and
/// quantises the 6-block MC residual, and emits either the 1-bit skip
/// (MV (0,0), no coded block — the decoder's copy-from-reference
/// branch) or the coded-MB syntax: joint MCBPCY (P-type half,
/// `idx = 64 + cbp`), the always-consumed ac_pred bit, the joint-MV
/// VLC, and a G4 inter AC walk per coded block (spec/04 §1.3).
pub fn encode_pframe_v3(
    input: &Picture,
    reference: &Picture,
    dims: PictureDims,
    config: &EncoderConfig,
) -> Result<Vec<u8>> {
    validate_input(input, dims, config)?;
    if reference.width != dims.width || reference.height != dims.height {
        return Err(Error::invalid(format!(
            "msmpeg4v3 encode: reference dimensions {}x{} differ from current {}x{}",
            reference.width, reference.height, dims.width, dims.height,
        )));
    }
    let (mb_w, mb_h) = dims.mb_dims();
    let quant = config.quant as u32;

    let mut bw = BitWriter::new();
    let hdr = MsV3PictureHeader {
        picture_type: PictureType::P,
        quant: config.quant,
        ac_chroma_sel: AC_CHROMA_SEL,
        ac_luma_sel: 0, // not carried on the P-frame wire (spec/99 §2.3)
        dc_size_sel: DC_SIZE_SEL,
        mv_table_sel: 0,
    };
    hdr.write(&mut bw)?;

    let inter_ac = AcVlcTable::g4_inter();
    let mut mv_grid = MvGrid::new(mb_w, mb_h);

    for my in 0..mb_h {
        for mx in 0..mb_w {
            encode_inter_mb(
                &mut bw,
                input,
                reference,
                &mut mv_grid,
                mx,
                my,
                quant,
                config,
                &inter_ac,
            )?;
        }
    }
    Ok(bw.finish())
}

/// One MB's motion-compensated prediction (luma 16×16 + chroma 8×8×2),
/// produced by the same [`mc_block`] kernel + §7.6.3.4 chroma-MV
/// derivation the decoder's `mc_macroblock` applies.
struct MbPrediction {
    luma: [u8; 256],
    cb: [u8; 64],
    cr: [u8; 64],
}

fn ref_planes(reference: &Picture) -> (RefPlane<'_>, RefPlane<'_>, RefPlane<'_>) {
    (
        RefPlane {
            data: &reference.y,
            stride: reference.y_stride,
            width: reference.width as usize,
            height: reference.height as usize,
        },
        RefPlane {
            data: &reference.cb,
            stride: reference.c_stride,
            width: (reference.width as usize).div_ceil(2),
            height: (reference.height as usize).div_ceil(2),
        },
        RefPlane {
            data: &reference.cr,
            stride: reference.c_stride,
            width: (reference.width as usize).div_ceil(2),
            height: (reference.height as usize).div_ceil(2),
        },
    )
}

/// Motion-compensate one MB's luma from the reference at `mv_half`.
fn predict_mb_luma(
    reference: &Picture,
    mb_x: usize,
    mb_y: usize,
    mv_half: (i32, i32),
) -> [u8; 256] {
    let (ref_y, _, _) = ref_planes(reference);
    let mut luma = [0u8; 256];
    mc_block(
        &ref_y,
        &mut luma,
        16,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        mv_half.0,
        mv_half.1,
        16,
    );
    luma
}

/// Full luma + chroma MC prediction for one MB, mirroring
/// `mc::mc_macroblock`'s per-plane kernel calls.
fn predict_mb(reference: &Picture, mb_x: usize, mb_y: usize, mv_half: (i32, i32)) -> MbPrediction {
    let (ref_y, ref_cb, ref_cr) = ref_planes(reference);
    let mut pred = MbPrediction {
        luma: [0u8; 256],
        cb: [0u8; 64],
        cr: [0u8; 64],
    };
    mc_block(
        &ref_y,
        &mut pred.luma,
        16,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        mv_half.0,
        mv_half.1,
        16,
    );
    let (cmx, cmy) = chroma_mv_from_luma(mv_half);
    let cx = (mb_x * 8) as i32;
    let cy = (mb_y * 8) as i32;
    mc_block(&ref_cb, &mut pred.cb, 8, cx, cy, cmx, cmy, 8);
    mc_block(&ref_cr, &mut pred.cr, 8, cx, cy, cmx, cmy, 8);
    pred
}

/// The decoder's 1-MV §7.6.5 predictor over the shared grid (mirror of
/// the private `picture::one_mv_predictor` — same public resolver +
/// predictor calls, Figure 7-34 top-left sub-diagram).
fn mv_predictor(grid: &MvGrid, mb_x: usize, mb_y: usize) -> Mv {
    let nset = grid.neighbour_set_for(mb_x, mb_y);
    let cands = resolve_block_candidates(Block::TopLeft, nset, [None, None, None]);
    predict_block_mv(Block::TopLeft, &cands)
}

/// Luma SAD between the input MB and the reference MC'd at `mv`.
fn mb_sad(input: &Picture, reference: &Picture, mb_x: usize, mb_y: usize, mv: (i32, i32)) -> u32 {
    let pred = predict_mb_luma(reference, mb_x, mb_y, mv);
    let mut sad = 0u32;
    for j in 0..16 {
        let row = (mb_y * 16 + j) * input.y_stride + mb_x * 16;
        for i in 0..16 {
            sad += (input.y[row + i] as i32 - pred[j * 16 + i] as i32).unsigned_abs();
        }
    }
    sad
}

/// Half-pel motion search: evaluates `(0, 0)`, the predictor, and the
/// full `[-range, +range]²` half-pel window, keeping the smallest luma
/// SAD among candidates that are (a) inside the `[-63, 63]` component
/// range and (b) toroidally reachable from `predictor` (spec/06 §3.5).
/// `(0, 0)` is always reachable-checked first so an all-static MB can
/// take the skip path.
fn motion_search(
    input: &Picture,
    reference: &Picture,
    mb_x: usize,
    mb_y: usize,
    predictor: Mv,
    range: u8,
    component_reachable: fn(i8, i8) -> bool,
) -> Mv {
    let reachable =
        |mv: Mv| component_reachable(mv.x, predictor.x) && component_reachable(mv.y, predictor.y);
    let mut best = Mv { x: 0, y: 0 };
    let mut have_best = false;
    let mut best_sad = u32::MAX;
    let mut consider = |mv: Mv, input: &Picture| {
        let sad = mb_sad(input, reference, mb_x, mb_y, (mv.x as i32, mv.y as i32));
        if sad < best_sad {
            best_sad = sad;
            best = mv;
            have_best = true;
        }
    };
    if reachable(Mv { x: 0, y: 0 }) {
        consider(Mv { x: 0, y: 0 }, input);
    }
    if predictor != (Mv { x: 0, y: 0 }) && reachable(predictor) {
        consider(predictor, input);
    }
    let r = range as i32;
    for dy in -r..=r {
        for dx in -r..=r {
            if dx == 0 && dy == 0 {
                continue;
            }
            if !(-63..=63).contains(&dx) || !(-63..=63).contains(&dy) {
                continue;
            }
            let cand = Mv {
                x: dx as i8,
                y: dy as i8,
            };
            if reachable(cand) {
                consider(cand, input);
            }
        }
    }
    if !have_best {
        // Pathological predictor window excluding even (0, 0): fall
        // back to the predictor itself (residual 0 — always
        // encodable).
        return predictor;
    }
    best
}

/// Analyse + serialise one P-frame MB.
#[allow(clippy::too_many_arguments)]
fn encode_inter_mb(
    bw: &mut BitWriter,
    input: &Picture,
    reference: &Picture,
    mv_grid: &mut MvGrid,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    config: &EncoderConfig,
    inter_ac: &AcVlcTable,
) -> Result<()> {
    let predictor = mv_predictor(mv_grid, mb_x, mb_y);
    let mv = motion_search(
        input,
        reference,
        mb_x,
        mb_y,
        predictor,
        config.mv_search_range,
        mv_component_reachable,
    );

    // Residual analysis over all 6 blocks at the chosen MV.
    let (levels, coded) = analyse_inter_residual(input, reference, mb_x, mb_y, mv, quant);
    let any_coded = coded.iter().any(|&c| c);
    if mv == (Mv { x: 0, y: 0 }) && !any_coded {
        // Skip MB: 1-bit flag, decoder copies the reference at (0, 0)
        // and stores a zero-MV grid cell.
        bw.write_bit(true);
        mv_grid.set_cell(mb_x, mb_y, MvGridCell::OneMv(Mv::default()));
        return Ok(());
    }

    bw.write_bit(false); // not skipped
    let cbpy =
        (coded[0] as u8) << 3 | (coded[1] as u8) << 2 | (coded[2] as u8) << 1 | (coded[3] as u8);
    let cbp = compose_cbp(cbpy, coded[4], coded[5]);
    // P-type (inter) half of the joint alphabet: idx = 64 + cbp.
    encode_mcbpcy(bw, 64 + cbp)?;
    // The decoder consumes the post-VLC ac_pred bit on every coded MB
    // (meaningful only for intra-in-P, which this encoder never emits).
    bw.write_bit(false);
    encode_mv_with_table(bw, predictor, MvTable::Default, mv)?;
    mv_grid.set_cell(mb_x, mb_y, MvGridCell::OneMv(mv));

    for block_idx in 0..6usize {
        if coded[block_idx] {
            encode_inter_ac(bw, &levels[block_idx], inter_ac)?;
        }
    }
    Ok(())
}

/// Transform + quantise the 6-block MC residual of one inter MB at the
/// chosen MV (`level_start = 0` — the inter DC is a coded coefficient,
/// spec/04 §1.3 / §2.6). Shared by the v3 and v1/v2 P-frame encoders
/// (the inter kernel shape is version-independent per spec/99 §6).
fn analyse_inter_residual(
    input: &Picture,
    reference: &Picture,
    mb_x: usize,
    mb_y: usize,
    mv: Mv,
    quant: u32,
) -> ([[i32; 64]; 6], [bool; 6]) {
    let pred = predict_mb(reference, mb_x, mb_y, (mv.x as i32, mv.y as i32));
    let mut levels = [[0i32; 64]; 6];
    let mut coded = [false; 6];
    for block_idx in 0..6usize {
        let cur = extract_block(input, mb_x, mb_y, block_idx);
        let mut residual = [0i32; 64];
        for (i, r) in residual.iter_mut().enumerate() {
            let p = match block_idx {
                0..=3 => {
                    let bx = (block_idx & 1) * 8 + (i & 7);
                    let by = (block_idx >> 1) * 8 + (i >> 3);
                    pred.luma[by * 16 + bx]
                }
                4 => pred.cb[i],
                5 => pred.cr[i],
                _ => unreachable!(),
            };
            *r = cur[i] - p as i32;
        }
        let mut f = [0.0f32; 64];
        fdct8x8_from_pels(&residual, &mut f);
        let nz = quantise_block_h263(&f, quant, 0, &mut levels[block_idx]);
        coded[block_idx] = nz > 0;
    }
    (levels, coded)
}

// ====================================================================
// v1 / v2 picture encoding
// ====================================================================

/// Encode one picture as a MS-MPEG4 **v1 or v2 I-frame**, decodable by
/// [`crate::picture::decode_picture_v1v2`]. Every MB is intra: the MB
/// header is the version's separate MCBPC + CBPY pair (v1 MB-type 3
/// INTRA at `mcbpc = 12 + cbpc` with the always-present leading COD
/// bit, spec/07 §1.5; v2 intra quotient at `mcbpc = 4 + cbpc` with the
/// post-MCBPC ac_pred bit, spec/07 §2.4), then the shared intra block
/// pipeline with the v1/v2 size+value DC scheme (spec/16 §2) and the
/// G5-luma / G4-chroma AC walks (the fixed v1/v2 fallthrough binding,
/// spec/14 §3.2 — no per-frame selectors exist on the v1/v2 wire).
pub fn encode_iframe_v1v2(
    input: &Picture,
    dims: PictureDims,
    config: &EncoderConfig,
    version: MsV1V2Version,
) -> Result<Vec<u8>> {
    validate_input(input, dims, config)?;
    let (mb_w, mb_h) = dims.mb_dims();
    let quant = config.quant as u32;

    let mut bw = BitWriter::new();
    let hdr = crate::header::MsV1V2PictureHeader {
        picture_type: PictureType::I,
        quant: config.quant,
        v1_umv_flag: false,
    };
    match version {
        MsV1V2Version::V1 => hdr.write_v1(&mut bw)?,
        MsV1V2Version::V2 => hdr.write_v2(&mut bw)?,
    }

    let luma_ac = AcVlcTable::v3_intra_g5();
    let chroma_ac = AcVlcTable::g4_inter();
    let mut dc_cache = DcCache::new(mb_w, mb_h);

    for my in 0..mb_h {
        for mx in 0..mb_w {
            let (plans, cbpy, cbp_cb, cbp_cr) =
                analyse_intra_mb(input, &mut dc_cache, mx, my, quant);
            let cbpc = ((cbp_cb as u8) << 1) | (cbp_cr as u8);
            match version {
                MsV1V2Version::V1 => {
                    // MB-type 3 (INTRA): mcbpc = 3 << 2 | cbpc. v1 has
                    // no ac_pred bit anywhere (spec/07 §1.4).
                    encode_mcbpcy_v1(&mut bw, false, 12 + cbpc, cbpy)?;
                }
                MsV1V2Version::V2 => {
                    // Intra quotient (mcbpc / 4 == 1): mcbpc = 4 + cbpc;
                    // ac_pred = 0 → fixed zigzag.
                    encode_mcbpcy_v2(&mut bw, V2FrameType::I, false, 4 + cbpc, false, cbpy)?;
                }
            }
            write_intra_blocks(&mut bw, &plans, DcScheme::V1V2, &luma_ac, &chroma_ac)?;
        }
    }
    Ok(bw.finish())
}

/// Encode one picture as a MS-MPEG4 **v1 or v2 P-frame** against the
/// decoder-side reconstruction of the previous frame, decodable by
/// [`crate::picture::decode_picture_v1v2`]. Same shape as the v3
/// P-frame encoder — shared §7.6.5 predictor grid, half-pel motion
/// search (clipped to the v1/v2 `[-32, +32]` per-component toroidal
/// window, spec/07 §3.2), skip on (0, 0)-MV/zero-CBP MBs — but with
/// the v1/v2 wire syntax: separate MCBPC (inter MB-type 0) + wrapped
/// CBPY, and the two per-component MV codewords instead of the v3
/// joint VLC. The residual is the shared G4 inter walk (spec/99 §6).
pub fn encode_pframe_v1v2(
    input: &Picture,
    reference: &Picture,
    dims: PictureDims,
    config: &EncoderConfig,
    version: MsV1V2Version,
) -> Result<Vec<u8>> {
    validate_input(input, dims, config)?;
    if reference.width != dims.width || reference.height != dims.height {
        return Err(Error::invalid(format!(
            "msmpeg4 v1/v2 encode: reference dimensions {}x{} differ from current {}x{}",
            reference.width, reference.height, dims.width, dims.height,
        )));
    }
    let (mb_w, mb_h) = dims.mb_dims();
    let quant = config.quant as u32;

    let mut bw = BitWriter::new();
    let hdr = crate::header::MsV1V2PictureHeader {
        picture_type: PictureType::P,
        quant: config.quant,
        // The v1 UMV flag is consumed as framing but not branched on by
        // the v<4 MV decoder body (spec/07 §3.4); write 0.
        v1_umv_flag: false,
    };
    match version {
        MsV1V2Version::V1 => hdr.write_v1(&mut bw)?,
        MsV1V2Version::V2 => hdr.write_v2(&mut bw)?,
    }

    let inter_ac = AcVlcTable::g4_inter();
    let mut mv_grid = MvGrid::new(mb_w, mb_h);

    for my in 0..mb_h {
        for mx in 0..mb_w {
            let predictor = mv_predictor(&mv_grid, mx, my);
            let mv = motion_search(
                input,
                reference,
                mx,
                my,
                predictor,
                config.mv_search_range,
                mv_v1v2_component_reachable,
            );
            let (levels, coded) = analyse_inter_residual(input, reference, mx, my, mv, quant);
            let any_coded = coded.iter().any(|&c| c);

            if mv == (Mv { x: 0, y: 0 }) && !any_coded {
                // Skip MB (1 bit): the decoder copies the reference at
                // (0, 0) and stores a zero-MV grid cell.
                match version {
                    MsV1V2Version::V1 => encode_mcbpcy_v1(&mut bw, true, 0, 0)?,
                    MsV1V2Version::V2 => {
                        encode_mcbpcy_v2(&mut bw, V2FrameType::P, true, 0, false, 0)?
                    }
                }
                mv_grid.set_cell(mx, my, MvGridCell::OneMv(Mv::default()));
                continue;
            }

            let cbpy = (coded[0] as u8) << 3
                | (coded[1] as u8) << 2
                | (coded[2] as u8) << 1
                | (coded[3] as u8);
            let cbpc = ((coded[4] as u8) << 1) | (coded[5] as u8);
            // Inter MB-type 0 for both versions: v1 mcbpc = 0 << 2 |
            // cbpc, v2 quotient-0 mcbpc = cbpc (the v2 remainder == 3
            // no-wrap sub-type is handled inside encode_mcbpcy_v2).
            match version {
                MsV1V2Version::V1 => encode_mcbpcy_v1(&mut bw, false, cbpc, cbpy)?,
                MsV1V2Version::V2 => {
                    encode_mcbpcy_v2(&mut bw, V2FrameType::P, false, cbpc, false, cbpy)?
                }
            }
            encode_mv_v1v2(&mut bw, predictor, mv)?;
            mv_grid.set_cell(mx, my, MvGridCell::OneMv(mv));

            for block_idx in 0..6usize {
                if coded[block_idx] {
                    encode_inter_ac(&mut bw, &levels[block_idx], &inter_ac)?;
                }
            }
        }
    }
    Ok(bw.finish())
}

/// Extract one 8×8 block (natural order) from the input picture.
/// `block_idx` follows the MB convention: 0..=3 luma raster, 4 = Cb,
/// 5 = Cr.
fn extract_block(pic: &Picture, mb_x: usize, mb_y: usize, block_idx: usize) -> [i32; 64] {
    let mut out = [0i32; 64];
    match block_idx {
        0..=3 => {
            let x0 = mb_x * 16 + (block_idx & 1) * 8;
            let y0 = mb_y * 16 + (block_idx >> 1) * 8;
            for j in 0..8 {
                for i in 0..8 {
                    out[j * 8 + i] = pic.y[(y0 + j) * pic.y_stride + x0 + i] as i32;
                }
            }
        }
        4 | 5 => {
            let plane = if block_idx == 4 { &pic.cb } else { &pic.cr };
            let x0 = mb_x * 8;
            let y0 = mb_y * 8;
            for j in 0..8 {
                for i in 0..8 {
                    out[j * 8 + i] = plane[(y0 + j) * pic.c_stride + x0 + i] as i32;
                }
            }
        }
        _ => unreachable!(),
    }
    out
}

/// Map a per-MB block index to its block-grid position (mirror of the
/// decoder-side helper in `picture.rs`).
fn block_grid_pos(block_idx: usize, mb_x: usize, mb_y: usize) -> (usize, usize) {
    match block_idx {
        0..=3 => (mb_x * 2 + (block_idx & 1), mb_y * 2 + (block_idx >> 1)),
        4 | 5 => (mb_x, mb_y),
        _ => unreachable!(),
    }
}

fn validate_input(input: &Picture, dims: PictureDims, config: &EncoderConfig) -> Result<()> {
    if !(1..=31).contains(&config.quant) {
        return Err(Error::invalid(format!(
            "msmpeg4v3 encode: quant {} out of range 1..=31",
            config.quant
        )));
    }
    let (mb_w, mb_h) = dims.mb_dims();
    if input.y_stride < mb_w * 16
        || input.c_stride < mb_w * 8
        || input.y.len() < input.y_stride * mb_h * 16
        || input.cb.len() < input.c_stride * mb_h * 8
        || input.cr.len() < input.c_stride * mb_h * 8
    {
        return Err(Error::invalid(
            "msmpeg4v3 encode: input planes smaller than the MB-aligned \
             layout Picture::alloc produces for these dimensions",
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::picture::decode_picture;
    use oxideav_core::bits::BitReader;

    fn fill_gradient(pic: &mut Picture) {
        let (w, h) = (pic.y_stride, pic.y.len() / pic.y_stride);
        for y in 0..h {
            for x in 0..w {
                pic.y[y * pic.y_stride + x] = (((x * 2 + y) / 2) % 200 + 20) as u8;
            }
        }
        let ch = pic.cb.len() / pic.c_stride;
        for y in 0..ch {
            for x in 0..pic.c_stride {
                pic.cb[y * pic.c_stride + x] = ((x + 100) % 220) as u8;
                pic.cr[y * pic.c_stride + x] = ((y + 140) % 220) as u8;
            }
        }
    }

    #[test]
    fn pframe_identical_content_is_all_skip_and_bit_exact() {
        let dims = PictureDims::new(48, 32).unwrap();
        let mut reference = Picture::alloc(dims, PictureType::I);
        fill_gradient(&mut reference);
        // Same content again: every MB must take the 1-bit skip path.
        let bytes =
            encode_pframe_v3(&reference, &reference, dims, &EncoderConfig::default()).unwrap();
        // Header (2+5+unary(1..2)+1+1 bits) + 6 MBs × 1 skip bit → 2 bytes.
        assert!(
            bytes.len() <= 3,
            "all-skip P-frame should be tiny, got {} bytes",
            bytes.len()
        );
        let mut br = BitReader::new(&bytes);
        let out = decode_picture(&mut br, dims, Some(&reference)).unwrap();
        assert_eq!(out.picture_type, PictureType::P);
        assert_eq!(out.y, reference.y, "skip MBs must copy the reference");
        assert_eq!(out.cb, reference.cb);
        assert_eq!(out.cr, reference.cr);
    }

    #[test]
    fn pframe_translated_content_is_recovered_by_motion_search() {
        let dims = PictureDims::new(48, 48).unwrap();
        let mut reference = Picture::alloc(dims, PictureType::I);
        fill_gradient(&mut reference);
        // Input = reference translated 2 pels right / 1 pel down
        // (MV (+4, +2) in half-pel units for every interior MB).
        let mut input = Picture::alloc(dims, PictureType::I);
        let h = input.y.len() / input.y_stride;
        for y in 0..h {
            for x in 0..input.y_stride {
                let sx = x.saturating_sub(2).min(input.y_stride - 1);
                let sy = y.saturating_sub(1).min(h - 1);
                input.y[y * input.y_stride + x] = reference.y[sy * reference.y_stride + sx];
            }
        }
        let ch = input.cb.len() / input.c_stride;
        for y in 0..ch {
            for x in 0..input.c_stride {
                let sx = x.saturating_sub(1).min(input.c_stride - 1);
                input.cb[y * input.c_stride + x] = reference.cb[y * reference.c_stride + sx];
                input.cr[y * input.c_stride + x] = reference.cr[y * reference.c_stride + sx];
            }
        }
        let config = EncoderConfig {
            quant: 2,
            mv_search_range: 6,
        };
        let bytes = encode_pframe_v3(&input, &reference, dims, &config).unwrap();
        let mut br = BitReader::new(&bytes);
        let out = decode_picture(&mut br, dims, Some(&reference)).unwrap();
        let mut sum = 0u64;
        for (a, b) in out.y.iter().zip(input.y.iter()) {
            sum += (*a as i64 - *b as i64).unsigned_abs();
        }
        let mae = sum as f64 / out.y.len() as f64;
        assert!(mae < 1.5, "translated P-frame luma MAE {mae} too large");
    }

    #[test]
    fn pframe_rejects_mismatched_reference_dims() {
        let dims = PictureDims::new(32, 32).unwrap();
        let other = PictureDims::new(48, 32).unwrap();
        let input = Picture::alloc(dims, PictureType::I);
        let reference = Picture::alloc(other, PictureType::I);
        assert!(encode_pframe_v3(&input, &reference, dims, &EncoderConfig::default()).is_err());
    }

    #[test]
    fn iframe_flat_grey_round_trips_exactly() {
        let dims = PictureDims::new(32, 32).unwrap();
        let input = Picture::alloc(dims, PictureType::I); // all-128 planes
        let bytes = encode_iframe_v3(&input, dims, &EncoderConfig::default()).expect("encode");
        let mut br = BitReader::new(&bytes);
        let out = decode_picture(&mut br, dims, None).expect("decode");
        assert_eq!(out.y, input.y, "flat grey luma must round-trip exactly");
        assert_eq!(out.cb, input.cb);
        assert_eq!(out.cr, input.cr);
    }

    #[test]
    fn iframe_gradient_reconstructs_close_at_fine_quant() {
        let dims = PictureDims::new(48, 32).unwrap();
        let mut input = Picture::alloc(dims, PictureType::I);
        fill_gradient(&mut input);
        let config = EncoderConfig {
            quant: 2,
            ..Default::default()
        };
        let bytes = encode_iframe_v3(&input, dims, &config).expect("encode");
        let mut br = BitReader::new(&bytes);
        let out = decode_picture(&mut br, dims, None).expect("decode");
        // Fine-quant lossy bound: every pel within a small tolerance,
        // and the mean absolute error well under one quant step.
        let mut sum_err = 0u64;
        for (a, b) in out.y.iter().zip(input.y.iter()) {
            let e = (*a as i32 - *b as i32).unsigned_abs();
            assert!(e <= 12, "luma pel error {e} too large at q=2");
            sum_err += e as u64;
        }
        let mae = sum_err as f64 / out.y.len() as f64;
        assert!(mae < 2.0, "luma MAE {mae} too large at q=2");
    }
}
