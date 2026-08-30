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
//! * the MB-level `ac_pred` flag is RD-decided: each intra MB's blocks
//!   are serialised both ways (fixed zigzag vs the per-block
//!   DC-gradient-derived alternate scans, spec/03 §1.1) and the
//!   alternate-scan variant is kept only when strictly cheaper. v1 has
//!   no `ac_pred` bit (spec/07 §1.4) and always walks zigzag.
//!
//! v3 I-frame table selectors (`ac_luma_sel` ∈ {G3, G1, G5},
//! `ac_chroma_sel` ∈ {G2, G0, G4}, `dc_size_sel` ∈ {0, 1}) are
//! RD-decided per frame by serialising the analysed frame under all
//! 18 combinations and keeping the smallest (see
//! [`encode_iframe_v3`]). P-frames transmit the fixed chroma = G4 /
//! default DC pair / default joint-MV VLC (`mv_table_sel = 0`); inter
//! residuals always use G4 (the alphabet is not frame-selectable on
//! the inter path, spec/04 §1.3), and intra-in-P luma binds G3 (the
//! parser's zero for the untransmitted P-wire `ac_luma_sel`).

use oxideav_core::bits::BitWriter;
use oxideav_core::{Error, Result};

use crate::ac::{encode_inter_ac, encode_intra_ac, AcVlcTable, Scan};
use crate::dc_pred::DcCache;
use crate::header::{MsV3PictureHeader, PictureType};
use crate::idct::fdct8x8_from_pels;
use crate::iq::{dc_scaler, quantise_block_h263};
use crate::mb::{encode_intra_dc_diff_v1v2, encode_intra_dc_diff_v3};
use crate::mc::{chroma_mv_from_four_luma, chroma_mv_from_luma, mc_block, RefPlane};
use crate::mcbpcy::{
    compose_cbp, encode_intra_cbpcy_sym, encode_mcbpcy, encode_mcbpcy_v1, encode_mcbpcy_v2,
    V2FrameType,
};
use crate::mv::{
    encode_mv_v1v2, encode_mv_with_table, mv_component_reachable, mv_v1v2_component_reachable, Mv,
    MvTable,
};
use crate::mv_pred::{
    predict_block_mv, resolve_block_candidates, Block, Macroblock4MvDecoderNeighbours, MvGrid,
    MvGridCell,
};
use crate::picture::{MsV1V2Version, Picture, PictureDims};
use crate::tables_data::MCBPCY_V3_PARTITION;

/// Value this encoder writes into the 5-bit I-frame header field
/// (`MsV3PictureHeader::iframe_ext`, spec/17 §1 field 3). 23 is the
/// value every Microsoft-produced 352x240 fixture and every spec/17
/// traced frame carries; its semantics are not established, so the
/// encoder emits the observed constant.
pub const IFRAME_EXT_DEFAULT: u8 = 23;

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

/// The fixed **P-frame** table selectors this encoder emits (I-frames
/// RD-select theirs per frame, see [`encode_iframe_v3`]): chroma G4 on
/// the P wire, default DC pair, default MV VLC.
const AC_CHROMA_SEL: u8 = 2; // G4
const DC_SIZE_SEL: u8 = 0;

/// One quantised 8×8 block of an intra MB, ready for serialisation.
struct IntraBlockPlan {
    dc_diff: i32,
    /// Reconstructed AC levels in natural order (position 0 unused).
    /// With AC prediction on, the *transmitted* levels are these minus
    /// the predicted first row / column ([`IntraBlockPlan::ac_pred_src`]).
    levels: [i32; 64],
    coded: bool,
    /// The scan this block walks when the MB's `ac_pred` flag is set:
    /// derived from the DC-predictor gradient direction exactly as the
    /// decoder derives it (spec/03 §1.1–§1.3), so encoder and decoder
    /// always agree on the per-block scan.
    alt_scan: Scan,
    /// The seven predicted AC levels for this block (index 0 unused):
    /// the causal neighbour's first row (top predictor) or column
    /// (left predictor), mirroring the decoder's
    /// [`crate::dc_pred::DcCache::ac_predict_luma`]. Subtracted from
    /// the block's first row / column when the MB transmits with
    /// `ac_pred = 1` (MPEG-4 §7.4.3.3; round 452).
    ac_pred_src: [i32; 8],
}

/// The positions the AC prediction touches for a block whose alternate
/// scan is `alt_scan`: the first row (top predictor → alt-horizontal)
/// or the first column (left predictor → alt-vertical).
fn ac_pred_positions(alt_scan: Scan) -> [usize; 7] {
    if alt_scan == Scan::AlternateHorizontal {
        [1, 2, 3, 4, 5, 6, 7]
    } else {
        [8, 16, 24, 32, 40, 48, 56]
    }
}

/// The levels actually transmitted for `plan` under the given MB
/// `ac_pred` polarity (with `apply_prediction` — the v3 path; v1/v2
/// transmit reconstruction-domain levels either way), plus the
/// resulting coded flag.
fn transmitted_levels(
    plan: &IntraBlockPlan,
    ac_pred: bool,
    apply_prediction: bool,
) -> ([i32; 64], bool) {
    let mut lv = plan.levels;
    if ac_pred && apply_prediction {
        for (k, &pos) in ac_pred_positions(plan.alt_scan).iter().enumerate() {
            lv[pos] -= plan.ac_pred_src[k + 1];
        }
    }
    let coded = lv.iter().skip(1).any(|&v| v != 0) || lv[0] != 0;
    (lv, coded)
}

/// Per-block coded flags of one MB under an `ac_pred` polarity.
fn coded_flags(plans: &[IntraBlockPlan; 6], ac_pred: bool, apply_prediction: bool) -> [bool; 6] {
    std::array::from_fn(|i| transmitted_levels(&plans[i], ac_pred, apply_prediction).1)
}

/// Encode one picture as a v3 **I-frame**. `input` must carry
/// MB-aligned planes for `dims` (the layout [`Picture::alloc`]
/// produces). Returns the frame's bitstream, decodable by
/// [`crate::picture::decode_picture`] with the same `dims`.
///
/// The per-frame table selectors are RD-decided: the frame is
/// analysed once (transform + quantise + DC-predictor threading —
/// none of which depend on the entropy tables), then serialised under
/// all 18 transmittable `(ac_luma_sel, ac_chroma_sel, dc_size_sel)`
/// combinations (3 × 3 × 2, spec/14 §3.1 + spec/99 §4.5) with the
/// per-MB ac_pred RD re-run against each table pair, and the smallest
/// bitstream wins — exact-rate by construction, since the probe *is*
/// the real serialiser.
pub fn encode_iframe_v3(
    input: &Picture,
    dims: PictureDims,
    config: &EncoderConfig,
) -> Result<Vec<u8>> {
    encode_iframe_v3_opts(input, dims, config, false)
}

/// Variant of [`encode_iframe_v3`] with a `first_of_sequence` flag.
/// The flag is **inert** since round 452: the 5-bit field it used to
/// gate is the per-I-frame header element `iframe_ext` (spec/17 §1
/// field 3), written on every I-frame by `MsV3PictureHeader::write`
/// with [`IFRAME_EXT_DEFAULT`]. Kept so existing callers compile.
pub fn encode_iframe_v3_opts(
    input: &Picture,
    dims: PictureDims,
    config: &EncoderConfig,
    first_of_sequence: bool,
) -> Result<Vec<u8>> {
    validate_input(input, dims, config)?;
    let (mb_w, mb_h) = dims.mb_dims();
    let quant = config.quant as u32;

    let mut dc_cache = DcCache::new(mb_w, mb_h);
    let mut mbs = Vec::with_capacity(mb_w * mb_h);
    for my in 0..mb_h {
        for mx in 0..mb_w {
            mbs.push(analyse_intra_mb(input, &mut dc_cache, mx, my, quant));
        }
    }
    let mut best: Option<Vec<u8>> = None;
    for luma_sel in 0..3u8 {
        for chroma_sel in 0..3u8 {
            for dc_size_sel in 0..2u8 {
                let bytes = assemble_iframe_v3(
                    &mbs,
                    mb_w,
                    config.quant,
                    luma_sel,
                    chroma_sel,
                    dc_size_sel,
                    first_of_sequence,
                )?;
                if std::env::var_os("OXIDEAV_MSMPEG4_ENC_TRACE").is_some() {
                    eprintln!(
                        "[enc combo] luma={luma_sel} chroma={chroma_sel} dc={dc_size_sel} len={}",
                        bytes.len()
                    );
                }
                if best.as_ref().map_or(true, |b| bytes.len() < b.len()) {
                    best = Some(bytes);
                }
            }
        }
    }
    Ok(best.expect("at least one selector combination was assembled"))
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
        alt_scan: Scan::Zigzag,
        ac_pred_src: [0i32; 8],
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

        // DC prediction happens in the quantised domain (the decoder's
        // `reconstruct_intra_dc` / `predicted_dc_level`), so quantise
        // the target DC first and difference the levels.
        let scaler = dc_scaler(block_idx, quant) as i32;
        let dc_level = (f[0] / scaler as f32).round() as i32;
        let pred_level = crate::mb::predicted_dc_level(pred.predictor, scaler);
        let dc_diff = (dc_level - pred_level).clamp(-255, 255);
        let dc_recon = crate::mb::reconstruct_intra_dc(dc_diff, pred.predictor, block_idx, quant);
        match block_idx {
            0..=3 => dc_cache.luma_set(bx, by, dc_recon),
            4 => dc_cache.chroma_set(false, bx, by, dc_recon),
            5 => dc_cache.chroma_set(true, bx, by, dc_recon),
            _ => unreachable!(),
        }

        plan.dc_diff = dc_diff;
        plan.alt_scan = pred.direction.ac_scan();
        let nz = quantise_block_h263(&f, quant, 1, &mut plan.levels);
        plan.coded = nz > 0;
        // AC-prediction bookkeeping, mirroring the decoder: this
        // block's reconstructed levels are the next blocks' prediction
        // source; its own prediction comes from the gradient-direction
        // neighbour.
        let src = match block_idx {
            0..=3 => dc_cache.ac_predict_luma(bx, by, pred.direction),
            4 => dc_cache.ac_predict_chroma(false, bx, by, pred.direction),
            5 => dc_cache.ac_predict_chroma(true, bx, by, pred.direction),
            _ => unreachable!(),
        };
        plan.ac_pred_src = src;
        let edges = crate::dc_pred::AcEdges::from_levels(&plan.levels);
        match block_idx {
            0..=3 => dc_cache.luma_ac_set(bx, by, edges),
            4 => dc_cache.chroma_ac_set(false, bx, by, edges),
            5 => dc_cache.chroma_ac_set(true, bx, by, edges),
            _ => unreachable!(),
        }
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
/// differential through the version's codeword scheme, then the AC
/// walk for each coded block — fixed zigzag when `ac_pred` is off, or
/// the per-block DC-gradient-derived alternate scan when it is on
/// (mirroring the decoder's spec/03 §1.1 scan dispatch).
fn write_intra_blocks(
    bw: &mut BitWriter,
    plans: &[IntraBlockPlan; 6],
    dc_scheme: DcScheme,
    luma_ac: &AcVlcTable,
    chroma_ac: &AcVlcTable,
    ac_pred: bool,
) -> Result<()> {
    write_intra_blocks_opts(bw, plans, dc_scheme, luma_ac, chroma_ac, ac_pred, false)
}

/// [`write_intra_blocks`] with the v3 coefficient-prediction switch:
/// when `apply_prediction` is set and `ac_pred` is on, each block
/// transmits its levels minus the predicted first row / column (the
/// decoder adds them back), and a block whose difference vanishes is
/// not coded at all. The caller's CBP bits must come from
/// [`coded_flags`] with the same arguments.
#[allow(clippy::too_many_arguments)]
fn write_intra_blocks_opts(
    bw: &mut BitWriter,
    plans: &[IntraBlockPlan; 6],
    dc_scheme: DcScheme,
    luma_ac: &AcVlcTable,
    chroma_ac: &AcVlcTable,
    ac_pred: bool,
    apply_prediction: bool,
) -> Result<()> {
    for (block_idx, plan) in plans.iter().enumerate() {
        match dc_scheme {
            DcScheme::V3(sel) => encode_intra_dc_diff_v3(bw, block_idx, sel, plan.dc_diff)?,
            DcScheme::V1V2 => encode_intra_dc_diff_v1v2(bw, block_idx, plan.dc_diff)?,
        }
        let (levels, coded) = transmitted_levels(plan, ac_pred, apply_prediction);
        if std::env::var_os("OXIDEAV_MSMPEG4_ENC_TRACE").is_some() {
            let nz: Vec<(usize, i32)> = levels
                .iter()
                .enumerate()
                .skip(1)
                .filter(|(_, &v)| v != 0)
                .map(|(i, &v)| (i, v))
                .collect();
            eprintln!("[enc blk] blk={block_idx} coded={coded} ac_pred={ac_pred} dc_diff={} bit={} nz={nz:?}", plan.dc_diff, bw.bit_position());
        }
        if coded {
            let table = if block_idx <= 3 { luma_ac } else { chroma_ac };
            let scan = if ac_pred { plan.alt_scan } else { Scan::Zigzag };
            encode_intra_ac(bw, &levels, scan, table, 1)?;
        }
    }
    Ok(())
}

/// Exact serialised bit count of [`write_intra_blocks`] for one MB
/// under the given `ac_pred` polarity (scratch-writer probe — the cost
/// function is the real serialiser, so the RD decision can never
/// disagree with the wire).
fn intra_blocks_bits(
    plans: &[IntraBlockPlan; 6],
    dc_scheme: DcScheme,
    luma_ac: &AcVlcTable,
    chroma_ac: &AcVlcTable,
    ac_pred: bool,
    apply_prediction: bool,
) -> Result<u64> {
    let mut bw = BitWriter::new();
    write_intra_blocks_opts(
        &mut bw,
        plans,
        dc_scheme,
        luma_ac,
        chroma_ac,
        ac_pred,
        apply_prediction,
    )?;
    Ok(bw.bit_position())
}

/// Rate decision for the MB-level `ac_pred` flag: serialise the MB's
/// blocks both ways and keep the alternate-scan variant only when it
/// is strictly cheaper. (The flag itself costs 1 bit in either
/// polarity, and the reconstruction is scan-independent, so pure rate
/// comparison IS the RD decision here.)
fn choose_ac_pred(
    plans: &[IntraBlockPlan; 6],
    dc_scheme: DcScheme,
    luma_ac: &AcVlcTable,
    chroma_ac: &AcVlcTable,
) -> Result<bool> {
    choose_ac_pred_opts(plans, dc_scheme, luma_ac, chroma_ac, false)
}

/// [`choose_ac_pred`] with the v3 coefficient-prediction switch.
fn choose_ac_pred_opts(
    plans: &[IntraBlockPlan; 6],
    dc_scheme: DcScheme,
    luma_ac: &AcVlcTable,
    chroma_ac: &AcVlcTable,
    apply_prediction: bool,
) -> Result<bool> {
    if !plans.iter().any(|p| p.coded) && !apply_prediction {
        return Ok(false);
    }
    let zig = intra_blocks_bits(
        plans,
        dc_scheme,
        luma_ac,
        chroma_ac,
        false,
        apply_prediction,
    )?;
    // The predicted-difference levels can exceed the verbatim escape
    // tier's 8-bit range; such an MB simply transmits without AC
    // prediction (the ac_pred=0 arm always encodes: its levels come
    // straight from the quantiser's ±127 clamp).
    let alt = match intra_blocks_bits(plans, dc_scheme, luma_ac, chroma_ac, true, apply_prediction)
    {
        Ok(bits) => bits,
        Err(_) => return Ok(false),
    };
    if std::env::var_os("OXIDEAV_MSMPEG4_ENC_TRACE").is_some() {
        eprintln!("[enc rd] zig={zig} alt={alt} apply={apply_prediction}");
    }
    Ok(alt < zig)
}

/// Which intra-DC differential codeword scheme a frame uses: the v3
/// 120-entry direct-value VLC (spec/07 §5.4) with its per-frame
/// `dc_size_sel` table-pair selector, or the v1/v2 H.263 size+value
/// scheme (spec/16 §2).
#[derive(Clone, Copy)]
enum DcScheme {
    V3(u8),
    V1V2,
}

/// The three v3 intra-luma AC alphabets the per-frame `ac_luma_sel`
/// selector can transmit (spec/14 §3.1: 0 → G3, 1 → G1, 2 → G5).
fn v3_luma_table_for_sel(sel: u8) -> AcVlcTable {
    match sel {
        0 => AcVlcTable::v3_intra_g3(),
        1 => AcVlcTable::v3_intra_g1(),
        _ => AcVlcTable::v3_intra_g5(),
    }
}

/// The three v3 intra-chroma AC alphabets `ac_chroma_sel` can
/// transmit (spec/14 §3.1: 0 → G2, 1 → G0, 2 → G4).
fn v3_chroma_table_for_sel(sel: u8) -> AcVlcTable {
    match sel {
        0 => AcVlcTable::v3_intra_g2(),
        1 => AcVlcTable::v3_intra_g0(),
        // The intra chroma path runs the intra kernel's escape
        // ladder, so G4 must carry LMAX/RMAX here (round 452; the
        // 1-tier `g4_inter` shape is the v1-only inter kernel's).
        _ => AcVlcTable::v3_intra_g4(),
    }
}

/// One analysed I-frame MB: the 6 block plans plus the CBP split.
type AnalysedMb = ([IntraBlockPlan; 6], u8, bool, bool);

/// Serialise a full v3 I-frame from pre-analysed MBs under one
/// concrete `(ac_luma_sel, ac_chroma_sel, dc_size_sel)` choice, with
/// the per-MB ac_pred RD run against those tables.
fn assemble_iframe_v3(
    mbs: &[AnalysedMb],
    mb_w: usize,
    quant: u8,
    luma_sel: u8,
    chroma_sel: u8,
    dc_size_sel: u8,
    first_of_sequence: bool,
) -> Result<Vec<u8>> {
    let mut bw = BitWriter::new();
    // `first_of_sequence` is retained for API stability: the 5-bit
    // I-frame field is a per-I-frame header element (spec/17 §1 field
    // 3, `MsV3PictureHeader::iframe_ext`), written by `hdr.write` on
    // every I-frame with the value the Microsoft fixtures carry.
    let _ = first_of_sequence;
    let hdr = MsV3PictureHeader {
        picture_type: PictureType::I,
        quant,
        iframe_ext: IFRAME_EXT_DEFAULT,
        mb_skip_enable: true,
        ac_chroma_sel: chroma_sel,
        ac_luma_sel: luma_sel,
        dc_size_sel,
        mv_table_sel: 0,
    };
    hdr.write(&mut bw)?;
    let luma_ac = v3_luma_table_for_sel(luma_sel);
    let chroma_ac = v3_chroma_table_for_sel(chroma_sel);
    let scheme = DcScheme::V3(dc_size_sel);
    // The coded flags depend on the per-MB ac_pred decision (a block
    // whose levels equal its AC prediction is not coded), so the
    // intra-CBPCY symbols are composed inline: XOR the luma coded bits
    // with the [`crate::picture::predict_cbp_bit`] spatial prediction
    // over the running actual-coded-bit grid; chroma raw (spec/17 §2).
    let mut luma_cbp = vec![false; (mb_w * 2) * (mbs.len() / mb_w) * 2];
    let gw = mb_w * 2;
    for (i, (plans, _cbpy, _cbp_cb, _cbp_cr)) in mbs.iter().enumerate() {
        let mx = i % mb_w;
        let my = i / mb_w;
        let ac_pred = choose_ac_pred_opts(plans, scheme, &luma_ac, &chroma_ac, true)?;
        let flags = coded_flags(plans, ac_pred, true);
        let mut sym = 0u8;
        for (blk, &flag) in flags.iter().take(4).enumerate() {
            let bx = mx * 2 + (blk & 1);
            let by = my * 2 + (blk >> 1);
            let l = bx > 0 && luma_cbp[by * gw + (bx - 1)];
            let t = by > 0 && luma_cbp[(by - 1) * gw + bx];
            let tl = bx > 0 && by > 0 && luma_cbp[(by - 1) * gw + (bx - 1)];
            let predicted = crate::picture::predict_cbp_bit(l, t, tl);
            luma_cbp[by * gw + bx] = flag;
            sym |= ((flag ^ predicted) as u8) << (5 - blk);
        }
        sym |= (flags[4] as u8) << 1;
        sym |= flags[5] as u8;
        if std::env::var_os("OXIDEAV_MSMPEG4_ENC_TRACE").is_some() {
            eprintln!(
                "[enc mb] mb=({mx},{my}) sym={sym:06b} ac_pred={ac_pred} bit={}",
                bw.bit_position()
            );
        }
        encode_intra_cbpcy_sym(&mut bw, sym)?;
        bw.write_bit(ac_pred);
        write_intra_blocks_opts(&mut bw, plans, scheme, &luma_ac, &chroma_ac, ac_pred, true)?;
    }
    Ok(bw.finish())
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
/// `idx = 64 + cbp`; the inter arm carries no ac_pred bit, spec/18
/// §3), the joint-MV VLC, and the selector-bound inter AC walk per
/// coded block (spec/99 §4.2).
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
    encode_pframe_v3_with_stats(input, reference, dims, config).map(|(bytes, _)| bytes)
}

/// Per-P-frame macroblock mode census, reported by the
/// `*_with_stats` P-frame encoders so callers (e.g. the GOP machine's
/// scene-cut policy) can see how much of the frame refused inter
/// coding.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PFrameStats {
    /// Total macroblocks in the frame.
    pub total_mbs: usize,
    /// 1-bit skip MBs (zero MV, no coded block).
    pub skip_mbs: usize,
    /// Motion-compensated inter MBs (1-MV or INTER4V).
    pub inter_mbs: usize,
    /// Intra-in-P MBs (the per-MB scene-change refuge).
    pub intra_mbs: usize,
}

/// How one P-frame MB was coded (internal to the per-MB encoders).
enum MbKind {
    Skip,
    Inter,
    Intra,
}

/// [`encode_pframe_v3`] plus the per-frame [`PFrameStats`] census.
pub fn encode_pframe_v3_with_stats(
    input: &Picture,
    reference: &Picture,
    dims: PictureDims,
    config: &EncoderConfig,
) -> Result<(Vec<u8>, PFrameStats)> {
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
        iframe_ext: 0,
        mb_skip_enable: true,
        quant: config.quant,
        ac_chroma_sel: AC_CHROMA_SEL,
        ac_luma_sel: 0, // not carried on the P-frame wire (spec/99 §2.3)
        dc_size_sel: DC_SIZE_SEL,
        mv_table_sel: 0,
    };
    hdr.write(&mut bw)?;

    // Inter blocks use the "chroma + all-inter" descriptor bound by
    // `ac_chroma_sel` with the single fixed-length escape (spec/99
    // §4.2); intra-in-P chroma blocks use the same primary through the
    // intra kernel's escape ladder (spec/17 §3).
    let inter_ac = AcVlcTable::inter_for_chroma_sel(AC_CHROMA_SEL);
    // Intra-in-P AC tables: `ac_luma_sel` is NOT carried on the v3
    // P-frame wire, so the decoder's dispatch sees the parser's zero
    // → G3 luma (spec/14 §3.1); chroma follows the transmitted
    // `ac_chroma_sel = 2` → G4. The encoder must serialise intra-in-P
    // blocks through the same pair.
    // `ac_luma_sel` persists from the most recent I-frame (spec/99
    // §2.3): use whatever selector the reference picture carries (the
    // I-frame RD's winner travels on the decoder-side reconstruction).
    let intra_luma_ac = v3_luma_table_for_sel(reference.ac_luma_sel);
    let intra_chroma_ac = AcVlcTable::v3_intra_g4();
    let mut mv_grid = MvGrid::new(mb_w, mb_h);
    // DC-prediction cache for intra-in-P MBs, mirroring the decoder's
    // per-P-frame cache: only intra MBs write cells; everything else
    // predicts against the neutral substitution.
    let mut dc_cache = DcCache::new(mb_w, mb_h);
    let mut stats = PFrameStats {
        total_mbs: mb_w * mb_h,
        ..Default::default()
    };

    for my in 0..mb_h {
        for mx in 0..mb_w {
            let kind = encode_pframe_mb_v3(
                &mut bw,
                input,
                reference,
                &mut mv_grid,
                &mut dc_cache,
                mx,
                my,
                quant,
                config,
                &inter_ac,
                &intra_luma_ac,
                &intra_chroma_ac,
            )?;
            match kind {
                MbKind::Skip => stats.skip_mbs += 1,
                MbKind::Inter => stats.inter_mbs += 1,
                MbKind::Intra => stats.intra_mbs += 1,
            }
        }
    }
    Ok((bw.finish(), stats))
}

/// Luma "intra activity" of one MB: sum of absolute deviations from
/// the MB mean — the classic cheap proxy for the residual energy an
/// intra coding of the MB would have to spend bits on. Compared
/// against the motion-compensated SAD (plus a margin biasing towards
/// inter, whose MV + CBP syntax is cheaper than a 6-block DC
/// differential set) to decide intra-in-P.
fn mb_intra_activity(input: &Picture, mb_x: usize, mb_y: usize) -> u32 {
    let mut sum = 0u32;
    for j in 0..16 {
        let row = (mb_y * 16 + j) * input.y_stride + mb_x * 16;
        for i in 0..16 {
            sum += input.y[row + i] as u32;
        }
    }
    let mean = (sum + 128) / 256;
    let mut act = 0u32;
    for j in 0..16 {
        let row = (mb_y * 16 + j) * input.y_stride + mb_x * 16;
        for i in 0..16 {
            act += (input.y[row + i] as i32 - mean as i32).unsigned_abs();
        }
    }
    act
}

/// Margin biasing the per-MB inter/intra decision towards inter: the
/// MC-side syntax (MV + CBP) is cheaper than a full 6-block intra DC
/// set, so intra must win by a clear distortion margin before it is
/// worth taking (H.263 TMN-lineage constant).
const INTRA_IN_P_MARGIN: u32 = 500;

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

/// Which MV entropy coder a motion search optimises its rate term
/// for: the v3 joint VLC (with the frame's table selector) or the
/// v1/v2 per-component codeword pair.
#[derive(Clone, Copy)]
enum MvCoder {
    V3(MvTable),
    V1V2,
}

/// Exact bit cost of coding `mv` against `predictor` under `coder`,
/// obtained by running the real encode-side serialiser into a scratch
/// writer (so the rate term of the motion-search cost function can
/// never drift from the wire format). `mv` must be reachable from
/// `predictor` under the version's toroidal window; unreachable
/// requests report an effectively-infinite cost.
fn mv_rate_bits(coder: MvCoder, predictor: Mv, mv: Mv) -> u32 {
    let mut bw = BitWriter::new();
    let res = match coder {
        MvCoder::V3(table) => encode_mv_with_table(&mut bw, predictor, table, mv),
        MvCoder::V1V2 => encode_mv_v1v2(&mut bw, predictor, mv),
    };
    match res {
        Ok(()) => bw.bit_position() as u32,
        Err(_) => u32::MAX / 2,
    }
}

/// Half-pel motion search with a rate-aware cost function:
///
/// > `cost(mv) = SAD(mv) + λ · bits(mv | predictor)`
///
/// where `bits` is the exact MV codeword length ([`mv_rate_bits`]) and
/// `λ = quant` (the classic SAD-domain Lagrange weight — one quantiser
/// step of distortion per residual bit). The candidate set is `(0, 0)`,
/// the predictor, and the union of two `[-range, +range]²` half-pel
/// windows centred on `(0, 0)` **and on the predictor** — the second
/// window lets MVs track motion beyond the nominal search range once
/// the §7.6.5 predictor chain has locked onto it (each MB extends the
/// reach of the next). Candidates outside `[-63, 63]` per component or
/// not toroidally reachable from `predictor` (spec/06 §3.5) are
/// excluded. `(0, 0)` is always evaluated first so an all-static MB
/// can take the skip path.
#[allow(clippy::too_many_arguments)]
fn motion_search(
    input: &Picture,
    reference: &Picture,
    mb_x: usize,
    mb_y: usize,
    predictor: Mv,
    range: u8,
    component_reachable: fn(i8, i8) -> bool,
    quant: u32,
    coder: MvCoder,
) -> Mv {
    let reachable =
        |mv: Mv| component_reachable(mv.x, predictor.x) && component_reachable(mv.y, predictor.y);
    let mut best = Mv { x: 0, y: 0 };
    let mut have_best = false;
    let mut best_cost = u64::MAX;
    // Visited-candidate mask over the [-63, 63]² component space so
    // the overlap of the two windows is evaluated once.
    let mut seen = vec![false; 127 * 127];
    let mut consider = |mv: Mv, input: &Picture| {
        let slot = (mv.y as i32 + 63) as usize * 127 + (mv.x as i32 + 63) as usize;
        if seen[slot] {
            return;
        }
        seen[slot] = true;
        if !reachable(mv) {
            return;
        }
        let sad = mb_sad(input, reference, mb_x, mb_y, (mv.x as i32, mv.y as i32));
        let cost = sad as u64 + quant as u64 * mv_rate_bits(coder, predictor, mv) as u64;
        if cost < best_cost {
            best_cost = cost;
            best = mv;
            have_best = true;
        }
    };
    consider(Mv { x: 0, y: 0 }, input);
    consider(predictor, input);
    let r = range as i32;
    for &(cx, cy) in &[(0i32, 0i32), (predictor.x as i32, predictor.y as i32)] {
        for dy in -r..=r {
            for dx in -r..=r {
                let (x, y) = (cx + dx, cy + dy);
                if !(-63..=63).contains(&x) || !(-63..=63).contains(&y) {
                    continue;
                }
                consider(
                    Mv {
                        x: x as i8,
                        y: y as i8,
                    },
                    input,
                );
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

/// Analyse + serialise one v3 P-frame MB: skip / inter / intra-in-P.
#[allow(clippy::too_many_arguments)]
fn encode_pframe_mb_v3(
    bw: &mut BitWriter,
    input: &Picture,
    reference: &Picture,
    mv_grid: &mut MvGrid,
    dc_cache: &mut DcCache,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    config: &EncoderConfig,
    inter_ac: &AcVlcTable,
    intra_luma_ac: &AcVlcTable,
    intra_chroma_ac: &AcVlcTable,
) -> Result<MbKind> {
    let predictor = mv_predictor(mv_grid, mb_x, mb_y);
    let mv = motion_search(
        input,
        reference,
        mb_x,
        mb_y,
        predictor,
        config.mv_search_range,
        mv_component_reachable,
        quant,
        MvCoder::V3(MvTable::Default),
    );

    // Scene-change refuge: when even the best MC prediction is worse
    // than what a from-scratch intra coding would face, code the MB
    // intra (joint MCBPCY I-type half, idx = cbp < 64). The decoder
    // leaves the MV-grid cell Absent for intra MBs, so the predictor
    // chain sees the same neighbourhood on both sides.
    let inter_sad = mb_sad(input, reference, mb_x, mb_y, (mv.x as i32, mv.y as i32));
    if mb_intra_activity(input, mb_x, mb_y) + INTRA_IN_P_MARGIN < inter_sad {
        let (plans, _cbpy, _cbp_cb, _cbp_cr) = analyse_intra_mb(input, dc_cache, mb_x, mb_y, quant);
        let ac_pred = choose_ac_pred_opts(
            &plans,
            DcScheme::V3(DC_SIZE_SEL),
            intra_luma_ac,
            intra_chroma_ac,
            true,
        )?;
        let flags = coded_flags(&plans, ac_pred, true);
        let cbpy = (flags[0] as u8) << 3
            | (flags[1] as u8) << 2
            | (flags[2] as u8) << 1
            | (flags[3] as u8);
        let cbp = compose_cbp(cbpy, flags[4], flags[5]);
        bw.write_bit(false); // not skipped
        encode_mcbpcy(bw, cbp)?;
        bw.write_bit(ac_pred);
        write_intra_blocks_opts(
            bw,
            &plans,
            DcScheme::V3(DC_SIZE_SEL),
            intra_luma_ac,
            intra_chroma_ac,
            ac_pred,
            true,
        )?;
        return Ok(MbKind::Intra);
    }

    // Residual analysis over all 6 blocks at the chosen MV.
    let (levels, coded) = analyse_inter_residual(input, reference, mb_x, mb_y, mv, quant);
    let any_coded = coded.iter().any(|&c| c);
    if mv == (Mv { x: 0, y: 0 }) && !any_coded {
        // Skip MB: 1-bit flag, decoder copies the reference at (0, 0)
        // and stores a zero-MV grid cell.
        bw.write_bit(true);
        mv_grid.set_cell(mb_x, mb_y, MvGridCell::OneMv(Mv::default()));
        return Ok(MbKind::Skip);
    }

    bw.write_bit(false); // not skipped
    let cbpy =
        (coded[0] as u8) << 3 | (coded[1] as u8) << 2 | (coded[2] as u8) << 1 | (coded[3] as u8);
    let cbp = compose_cbp(cbpy, coded[4], coded[5]);
    // P-type (inter) half of the joint alphabet: idx = 64 + cbp.
    encode_mcbpcy(bw, MCBPCY_V3_PARTITION as u8 + cbp)?;
    // spec/18 §3: the inter (bit-6-set) arm carries no ac_pred bit.
    encode_mv_with_table(bw, predictor, MvTable::Default, mv)?;
    mv_grid.set_cell(mb_x, mb_y, MvGridCell::OneMv(mv));

    for block_idx in 0..6usize {
        if coded[block_idx] {
            encode_inter_ac(bw, &levels[block_idx], inter_ac)?;
        }
    }
    Ok(MbKind::Inter)
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
    analyse_residual_with_pred(input, &pred, mb_x, mb_y, quant)
}

/// Core of [`analyse_inter_residual`], reusable with an arbitrary
/// (e.g. INTER4V) MC prediction.
fn analyse_residual_with_pred(
    input: &Picture,
    pred: &MbPrediction,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
) -> ([[i32; 64]; 6], [bool; 6]) {
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

/// Full luma + chroma MC prediction for one INTER4V MB: four per-block
/// half-pel luma MVs in Figure 6-8 raster order plus the §7.6.3.4
/// sum/2K + Table 7-12 chroma derivation — mirroring
/// `mc::mc_macroblock_4mv`'s kernel calls.
fn predict_mb_4mv(reference: &Picture, mb_x: usize, mb_y: usize, mvs: [Mv; 4]) -> MbPrediction {
    let (ref_y, ref_cb, ref_cr) = ref_planes(reference);
    let mut pred = MbPrediction {
        luma: [0u8; 256],
        cb: [0u8; 64],
        cr: [0u8; 64],
    };
    let mvs_half = [
        (mvs[0].x as i32, mvs[0].y as i32),
        (mvs[1].x as i32, mvs[1].y as i32),
        (mvs[2].x as i32, mvs[2].y as i32),
        (mvs[3].x as i32, mvs[3].y as i32),
    ];
    for (i, &(mvx, mvy)) in mvs_half.iter().enumerate() {
        let bx = (i & 1) * 8;
        let by = (i >> 1) * 8;
        let mut block = [0u8; 64];
        mc_block(
            &ref_y,
            &mut block,
            8,
            (mb_x * 16 + bx) as i32,
            (mb_y * 16 + by) as i32,
            mvx,
            mvy,
            8,
        );
        for j in 0..8 {
            for i2 in 0..8 {
                pred.luma[(by + j) * 16 + bx + i2] = block[j * 8 + i2];
            }
        }
    }
    let (cmx, cmy) = chroma_mv_from_four_luma(mvs_half);
    let cx = (mb_x * 8) as i32;
    let cy = (mb_y * 8) as i32;
    mc_block(&ref_cb, &mut pred.cb, 8, cx, cy, cmx, cmy, 8);
    mc_block(&ref_cr, &mut pred.cr, 8, cx, cy, cmx, cmy, 8);
    pred
}

/// Luma SAD of one Figure 6-8 8×8 block against the reference MC'd at
/// `mv`.
fn block_sad_8(
    input: &Picture,
    reference: &Picture,
    mb_x: usize,
    mb_y: usize,
    blk: usize,
    mv: Mv,
) -> u32 {
    let (ref_y, _, _) = ref_planes(reference);
    let bx = (blk & 1) * 8;
    let by = (blk >> 1) * 8;
    let mut pred = [0u8; 64];
    mc_block(
        &ref_y,
        &mut pred,
        8,
        (mb_x * 16 + bx) as i32,
        (mb_y * 16 + by) as i32,
        mv.x as i32,
        mv.y as i32,
        8,
    );
    let mut sad = 0u32;
    for j in 0..8 {
        let row = (mb_y * 16 + by + j) * input.y_stride + mb_x * 16 + bx;
        for i in 0..8 {
            sad += (input.y[row + i] as i32 - pred[j * 8 + i] as i32).unsigned_abs();
        }
    }
    sad
}

/// Greedy per-block INTER4V motion search for a v1 MB: for each Figure
/// 6-8 block in raster order, take the Figure-7-34 within-MB predictor
/// from the already-committed blocks (the exact interleaved
/// predict → decode → commit order the decoder replays), search the
/// union of the zero- and predictor-centred half-pel windows for the
/// smallest rate-aware cost (`SAD + quant · mv_bits`, mirroring
/// [`motion_search`]) among v1/v2-reachable candidates, and commit.
/// Returns the four final MVs and the total cost.
fn search_inter4v(
    input: &Picture,
    reference: &Picture,
    mv_grid: &MvGrid,
    mb_x: usize,
    mb_y: usize,
    range: u8,
    quant: u32,
) -> ([Mv; 4], u64) {
    let nset = mv_grid.neighbour_set_for(mb_x, mb_y);
    let mut dec = Macroblock4MvDecoderNeighbours::new(nset);
    let mut mvs = [Mv::default(); 4];
    let mut total_cost = 0u64;
    let r = range as i32;
    for (i, &block) in Block::ALL.iter().enumerate() {
        let predictor = dec.predictor_for(block);
        let mut best = predictor; // always reachable (residual 0)
        let mut best_cost = block_sad_8(input, reference, mb_x, mb_y, i, best) as u64
            + quant as u64 * mv_rate_bits(MvCoder::V1V2, predictor, best) as u64;
        for &(cx, cy) in &[(0i32, 0i32), (predictor.x as i32, predictor.y as i32)] {
            for dy in -r..=r {
                for dx in -r..=r {
                    let (x, y) = (cx + dx, cy + dy);
                    if !(-63..=63).contains(&x) || !(-63..=63).contains(&y) {
                        continue;
                    }
                    let cand = Mv {
                        x: x as i8,
                        y: y as i8,
                    };
                    if cand == best
                        || !mv_v1v2_component_reachable(cand.x, predictor.x)
                        || !mv_v1v2_component_reachable(cand.y, predictor.y)
                    {
                        continue;
                    }
                    let cost = block_sad_8(input, reference, mb_x, mb_y, i, cand) as u64
                        + quant as u64 * mv_rate_bits(MvCoder::V1V2, predictor, cand) as u64;
                    if cost < best_cost {
                        best_cost = cost;
                        best = cand;
                    }
                }
            }
        }
        dec.commit_block(block, best);
        mvs[i] = best;
        total_cost += best_cost;
    }
    (mvs, total_cost)
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
            let ac_pred = match version {
                // v1 has no ac_pred bit anywhere (spec/07 §1.4):
                // always the fixed zigzag.
                MsV1V2Version::V1 => false,
                MsV1V2Version::V2 => choose_ac_pred(&plans, DcScheme::V1V2, &luma_ac, &chroma_ac)?,
            };
            match version {
                MsV1V2Version::V1 => {
                    // MB-type 3 (INTRA): mcbpc = 3 << 2 | cbpc.
                    encode_mcbpcy_v1(&mut bw, false, 12 + cbpc, cbpy)?;
                }
                MsV1V2Version::V2 => {
                    // Intra quotient (mcbpc / 4 == 1): mcbpc = 4 + cbpc;
                    // ac_pred RD-decided per MB.
                    encode_mcbpcy_v2(&mut bw, V2FrameType::I, false, 4 + cbpc, ac_pred, cbpy)?;
                }
            }
            write_intra_blocks(
                &mut bw,
                &plans,
                DcScheme::V1V2,
                &luma_ac,
                &chroma_ac,
                ac_pred,
            )?;
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
    encode_pframe_v1v2_with_stats(input, reference, dims, config, version).map(|(bytes, _)| bytes)
}

/// [`encode_pframe_v1v2`] plus the per-frame [`PFrameStats`] census.
pub fn encode_pframe_v1v2_with_stats(
    input: &Picture,
    reference: &Picture,
    dims: PictureDims,
    config: &EncoderConfig,
    version: MsV1V2Version,
) -> Result<(Vec<u8>, PFrameStats)> {
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
    // Intra-in-P AC tables: v1/v2 bind the default G5 luma / G4 chroma
    // descriptors with no per-frame selector (spec/14 §3.2) — same
    // pair as the v1/v2 I-frame path.
    let intra_luma_ac = AcVlcTable::v3_intra_g5();
    let intra_chroma_ac = AcVlcTable::g4_inter();
    let mut mv_grid = MvGrid::new(mb_w, mb_h);
    let mut dc_cache = DcCache::new(mb_w, mb_h);
    let mut stats = PFrameStats {
        total_mbs: mb_w * mb_h,
        ..Default::default()
    };

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
                quant,
                MvCoder::V1V2,
            );

            // Scene-change refuge (same census rule as v3): when the
            // best MC prediction is worse than the MB's own intra
            // activity by a clear margin, code the MB intra — v1
            // MB-type 3 (mcbpc = 12 + cbpc, no ac_pred bit, spec/07
            // §1.4), v2 intra quotient (mcbpc = 4 + cbpc, post-MCBPC
            // ac_pred bit, spec/07 §2.4). Intra MBs leave the MV-grid
            // cell Absent, mirroring the decoder.
            let inter_sad = mb_sad(input, reference, mx, my, (mv.x as i32, mv.y as i32));
            if mb_intra_activity(input, mx, my) + INTRA_IN_P_MARGIN < inter_sad {
                let (plans, cbpy, cbp_cb, cbp_cr) =
                    analyse_intra_mb(input, &mut dc_cache, mx, my, quant);
                let cbpc = ((cbp_cb as u8) << 1) | (cbp_cr as u8);
                match version {
                    MsV1V2Version::V1 => {
                        encode_mcbpcy_v1(&mut bw, false, 12 + cbpc, cbpy)?;
                        write_intra_blocks(
                            &mut bw,
                            &plans,
                            DcScheme::V1V2,
                            &intra_luma_ac,
                            &intra_chroma_ac,
                            false,
                        )?;
                    }
                    MsV1V2Version::V2 => {
                        let ac_pred = choose_ac_pred(
                            &plans,
                            DcScheme::V1V2,
                            &intra_luma_ac,
                            &intra_chroma_ac,
                        )?;
                        encode_mcbpcy_v2(&mut bw, V2FrameType::P, false, 4 + cbpc, ac_pred, cbpy)?;
                        write_intra_blocks(
                            &mut bw,
                            &plans,
                            DcScheme::V1V2,
                            &intra_luma_ac,
                            &intra_chroma_ac,
                            ac_pred,
                        )?;
                    }
                }
                stats.intra_mbs += 1;
                continue;
            }

            // v1-only INTER4V (MB-type 2, spec/16 §3.1) mode decision:
            // rate-aware — the greedy per-block search's total cost
            // (`SAD + quant · mv_bits` over the four blocks) must beat
            // the 1-MV cost under the same λ before the 4-MV MB is
            // emitted. The v2 8-symbol MCBPC alphabet has no INTER4V
            // code (spec/16 §3.3), so v2 never takes this branch.
            let inter4v = if version == MsV1V2Version::V1 && config.mv_search_range > 0 {
                let cost_1mv = mb_sad(input, reference, mx, my, (mv.x as i32, mv.y as i32)) as u64
                    + quant as u64 * mv_rate_bits(MvCoder::V1V2, predictor, mv) as u64;
                let (mvs4, cost_4mv) = search_inter4v(
                    input,
                    reference,
                    &mv_grid,
                    mx,
                    my,
                    config.mv_search_range,
                    quant,
                );
                let uniform = mvs4.iter().all(|&m| m == mvs4[0]);
                if !uniform && cost_4mv < cost_1mv {
                    Some(mvs4)
                } else {
                    None
                }
            } else {
                None
            };

            if let Some(mvs4) = inter4v {
                let pred = predict_mb_4mv(reference, mx, my, mvs4);
                let (levels, coded) = analyse_residual_with_pred(input, &pred, mx, my, quant);
                let cbpy = (coded[0] as u8) << 3
                    | (coded[1] as u8) << 2
                    | (coded[2] as u8) << 1
                    | (coded[3] as u8);
                let cbpc = ((coded[4] as u8) << 1) | (coded[5] as u8);
                // MB-type 2 (INTER4V): mcbpc = 2 << 2 | cbpc.
                encode_mcbpcy_v1(&mut bw, false, 8 + cbpc, cbpy)?;
                // Serialise the four MVs replaying the same interleaved
                // Figure-7-34 predict → encode → commit order the
                // decoder walks.
                let nset = mv_grid.neighbour_set_for(mx, my);
                let mut dec = Macroblock4MvDecoderNeighbours::new(nset);
                for (i, &block) in Block::ALL.iter().enumerate() {
                    let block_pred = dec.predictor_for(block);
                    encode_mv_v1v2(&mut bw, block_pred, mvs4[i])?;
                    dec.commit_block(block, mvs4[i]);
                }
                mv_grid.set_cell(mx, my, dec.finalise_to_grid_cell());
                for block_idx in 0..6usize {
                    if coded[block_idx] {
                        encode_inter_ac(&mut bw, &levels[block_idx], &inter_ac)?;
                    }
                }
                stats.inter_mbs += 1;
                continue;
            }

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
                stats.skip_mbs += 1;
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
            stats.inter_mbs += 1;
        }
    }
    Ok((bw.finish(), stats))
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

    /// Deterministic high-frequency texture (motion must be matched
    /// exactly for the SAD to collapse — a plain gradient would let
    /// any MV win via a cheap DC residual).
    fn fill_texture(pic: &mut Picture) {
        let h = pic.y.len() / pic.y_stride;
        for y in 0..h {
            for x in 0..pic.y_stride {
                let a = (x * 7 + y * 13) % 251;
                let b = (x * x / 3 + y * y / 5) % 89;
                pic.y[y * pic.y_stride + x] = ((a + b) % 200 + 24) as u8;
            }
        }
        let ch = pic.cb.len() / pic.c_stride;
        for y in 0..ch {
            for x in 0..pic.c_stride {
                pic.cb[y * pic.c_stride + x] = ((x * 5 + y * 3) % 60 + 100) as u8;
                pic.cr[y * pic.c_stride + x] = ((x * 3 + y * 7) % 60 + 110) as u8;
            }
        }
    }

    #[test]
    fn pframe_predictor_chain_extends_search_beyond_window() {
        // Global pan of 5 pels right / 2 pels down = MV (+10, +4) in
        // half-pel units, searched with range 6: the zero-centred
        // window alone cannot reach it, but once one MB locks on, the
        // predictor-centred window lets every following MB track the
        // true motion (§7.6.5 predictor chain).
        let dims = PictureDims::new(64, 64).unwrap();
        let mut reference = Picture::alloc(dims, PictureType::I);
        fill_texture(&mut reference);
        let mut input = Picture::alloc(dims, PictureType::I);
        let h = input.y.len() / input.y_stride;
        for y in 0..h {
            for x in 0..input.y_stride {
                let sx = x.saturating_sub(5).min(input.y_stride - 1);
                let sy = y.saturating_sub(2).min(h - 1);
                input.y[y * input.y_stride + x] = reference.y[sy * reference.y_stride + sx];
            }
        }
        let ch = input.cb.len() / input.c_stride;
        for y in 0..ch {
            for x in 0..input.c_stride {
                let sx = x.saturating_sub(2).min(input.c_stride - 1);
                let sy = y.saturating_sub(1).min(ch - 1);
                input.cb[y * input.c_stride + x] = reference.cb[sy * reference.c_stride + sx];
                input.cr[y * input.c_stride + x] = reference.cr[sy * reference.c_stride + sx];
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
        assert!(
            mae < 2.5,
            "predictor-chained search should recover the out-of-window pan, MAE {mae}"
        );
    }

    /// A second scene, deliberately smooth (low intra activity) and
    /// uncorrelated with [`fill_texture`] (high MC SAD), so the
    /// per-MB intra-in-P decision clearly favours intra after a cut.
    fn fill_other_texture(pic: &mut Picture) {
        let h = pic.y.len() / pic.y_stride;
        for y in 0..h {
            for x in 0..pic.y_stride {
                pic.y[y * pic.y_stride + x] = ((x + 2 * y) / 4 % 96 + 130) as u8;
            }
        }
        let ch = pic.cb.len() / pic.c_stride;
        for y in 0..ch {
            for x in 0..pic.c_stride {
                pic.cb[y * pic.c_stride + x] = ((x + y) / 3 % 40 + 90) as u8;
                pic.cr[y * pic.c_stride + x] = ((x + y) / 4 % 40 + 130) as u8;
            }
        }
    }

    #[test]
    fn pframe_scene_change_takes_intra_in_p_and_decodes_v3() {
        let dims = PictureDims::new(64, 64).unwrap();
        let mut reference = Picture::alloc(dims, PictureType::I);
        fill_texture(&mut reference);
        // Scene change: the input shares nothing with the reference.
        let mut input = Picture::alloc(dims, PictureType::I);
        fill_other_texture(&mut input);
        let config = EncoderConfig {
            quant: 4,
            mv_search_range: 4,
        };
        let (bytes, stats) =
            encode_pframe_v3_with_stats(&input, &reference, dims, &config).unwrap();
        assert!(
            stats.intra_mbs > stats.total_mbs / 2,
            "scene change should code most MBs intra, got {stats:?}"
        );
        let mut br = BitReader::new(&bytes);
        let out = decode_picture(&mut br, dims, Some(&reference)).unwrap();
        let mae = out
            .y
            .iter()
            .zip(input.y.iter())
            .map(|(a, b)| (*a as i64 - *b as i64).unsigned_abs())
            .sum::<u64>() as f64
            / out.y.len() as f64;
        assert!(mae < 3.5, "v3 intra-in-P scene change MAE {mae} too large");
    }

    #[test]
    fn pframe_scene_change_takes_intra_in_p_and_decodes_v1_v2() {
        let dims = PictureDims::new(64, 64).unwrap();
        let mut reference = Picture::alloc(dims, PictureType::I);
        fill_texture(&mut reference);
        let mut input = Picture::alloc(dims, PictureType::I);
        fill_other_texture(&mut input);
        let config = EncoderConfig {
            quant: 4,
            mv_search_range: 4,
        };
        for version in [MsV1V2Version::V1, MsV1V2Version::V2] {
            let (bytes, stats) =
                encode_pframe_v1v2_with_stats(&input, &reference, dims, &config, version).unwrap();
            assert!(
                stats.intra_mbs > stats.total_mbs / 2,
                "{version:?}: scene change should code most MBs intra, got {stats:?}"
            );
            let mut br = BitReader::new(&bytes);
            let out = crate::picture::decode_picture_v1v2(&mut br, dims, version, Some(&reference))
                .unwrap();
            let mae = out
                .y
                .iter()
                .zip(input.y.iter())
                .map(|(a, b)| (*a as i64 - *b as i64).unsigned_abs())
                .sum::<u64>() as f64
                / out.y.len() as f64;
            assert!(
                mae < 3.5,
                "{version:?} intra-in-P scene change MAE {mae} too large"
            );
        }
    }

    #[test]
    fn pframe_partial_scene_change_mixes_intra_and_inter() {
        // Left half pans within the reference texture, right half cuts
        // to a new texture: the census must show both inter (or skip)
        // and intra MBs, and the whole frame must still decode.
        let dims = PictureDims::new(96, 48).unwrap();
        let mut reference = Picture::alloc(dims, PictureType::I);
        fill_texture(&mut reference);
        let mut other = Picture::alloc(dims, PictureType::I);
        fill_other_texture(&mut other);
        let mut input = reference.clone();
        let h = input.y.len() / input.y_stride;
        for y in 0..h {
            for x in 48..input.y_stride {
                input.y[y * input.y_stride + x] = other.y[y * other.y_stride + x];
            }
        }
        let ch = input.cb.len() / input.c_stride;
        for y in 0..ch {
            for x in 24..input.c_stride {
                input.cb[y * input.c_stride + x] = other.cb[y * other.c_stride + x];
                input.cr[y * input.c_stride + x] = other.cr[y * other.c_stride + x];
            }
        }
        let config = EncoderConfig {
            quant: 4,
            mv_search_range: 4,
        };
        let (bytes, stats) =
            encode_pframe_v3_with_stats(&input, &reference, dims, &config).unwrap();
        assert!(stats.intra_mbs > 0, "right half should go intra: {stats:?}");
        assert!(
            stats.skip_mbs + stats.inter_mbs > 0,
            "left half should stay inter/skip: {stats:?}"
        );
        let mut br = BitReader::new(&bytes);
        let out = decode_picture(&mut br, dims, Some(&reference)).unwrap();
        let mae = out
            .y
            .iter()
            .zip(input.y.iter())
            .map(|(a, b)| (*a as i64 - *b as i64).unsigned_abs())
            .sum::<u64>() as f64
            / out.y.len() as f64;
        assert!(mae < 3.5, "mixed P-frame MAE {mae} too large");
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
    fn ac_pred_rd_prefers_alt_scan_for_front_row_energy() {
        // Coefficients confined to the first DCT row (natural indices
        // 1..=5): the alternate-horizontal scan reaches them at scan
        // positions 1..=3 / 10..=11 while zigzag scatters them out to
        // position 14 — the RD probe must pick the alternate scan.
        let mut plans: [IntraBlockPlan; 6] = std::array::from_fn(|_| IntraBlockPlan {
            dc_diff: 0,
            levels: [0i32; 64],
            coded: false,
            alt_scan: Scan::AlternateHorizontal,
            ac_pred_src: [0i32; 8],
        });
        for (i, lv) in [(1usize, 6i32), (2, 4), (3, 3), (4, 2), (5, 1)] {
            plans[0].levels[i] = lv;
        }
        plans[0].coded = true;
        let luma = AcVlcTable::v3_intra_g5();
        let chroma = AcVlcTable::g4_inter();
        let zig = intra_blocks_bits(
            &plans,
            DcScheme::V3(DC_SIZE_SEL),
            &luma,
            &chroma,
            false,
            false,
        )
        .unwrap();
        let alt = intra_blocks_bits(
            &plans,
            DcScheme::V3(DC_SIZE_SEL),
            &luma,
            &chroma,
            true,
            false,
        )
        .unwrap();
        assert!(alt < zig, "alt scan should be cheaper: alt={alt} zig={zig}");
        assert!(choose_ac_pred(&plans, DcScheme::V3(DC_SIZE_SEL), &luma, &chroma).unwrap());
        // And an uncoded MB never pays for the probe.
        let empty: [IntraBlockPlan; 6] = std::array::from_fn(|_| IntraBlockPlan {
            dc_diff: 0,
            levels: [0i32; 64],
            coded: false,
            alt_scan: Scan::AlternateVertical,
            ac_pred_src: [0i32; 8],
        });
        assert!(!choose_ac_pred(&empty, DcScheme::V3(DC_SIZE_SEL), &luma, &chroma).unwrap());
    }

    /// Row-banded DC (forces the FromTop / alternate-horizontal
    /// direction) + strong intra-block horizontal frequency (fills the
    /// first DCT row): content engineered so the ac_pred RD fires on
    /// interior MBs.
    fn fill_row_bands_with_x_texture(pic: &mut Picture) {
        let h = pic.y.len() / pic.y_stride;
        for y in 0..h {
            for x in 0..pic.y_stride {
                let band = (y / 8) * 12;
                let tex = [0u8, 24, 48, 24][x % 4] as usize;
                pic.y[y * pic.y_stride + x] = (60 + band + tex).min(255) as u8;
            }
        }
        // Flat chroma keeps the test focused on the luma scan choice.
        pic.cb.fill(128);
        pic.cr.fill(128);
    }

    #[test]
    fn iframe_ac_pred_streams_decode_verified_v3_and_v2() {
        let dims = PictureDims::new(48, 48).unwrap();
        let mut input = Picture::alloc(dims, PictureType::I);
        fill_row_bands_with_x_texture(&mut input);
        let quant = 4u32;

        // The RD must actually fire on this content (replay the v3
        // analysis with coefficient prediction, as `assemble_iframe_v3`
        // runs it — round 452 made ac_pred a coefficient predictor, not
        // just a scan switch, so the rate balance moved).
        let luma = v3_luma_table_for_sel(0);
        let chroma = v3_chroma_table_for_sel(0);
        let (mb_w, mb_h) = dims.mb_dims();
        let mut dc_cache = DcCache::new(mb_w, mb_h);
        let mut fired = 0usize;
        for my in 0..mb_h {
            for mx in 0..mb_w {
                let (plans, _, _, _) = analyse_intra_mb(&input, &mut dc_cache, mx, my, quant);
                if choose_ac_pred_opts(&plans, DcScheme::V3(0), &luma, &chroma, true).unwrap() {
                    fired += 1;
                }
            }
        }
        assert!(fired > 0, "content should trigger ac_pred on some MB");

        let config = EncoderConfig {
            quant: quant as u8,
            ..Default::default()
        };
        // v3: encode + decode through the production path; a scan
        // mismatch between encoder and decoder would corrupt the
        // reconstruction far past the quantiser bound.
        let bytes = encode_iframe_v3(&input, dims, &config).unwrap();
        let mut br = BitReader::new(&bytes);
        let out = decode_picture(&mut br, dims, None).unwrap();
        let mae = out
            .y
            .iter()
            .zip(input.y.iter())
            .map(|(a, b)| (*a as i64 - *b as i64).unsigned_abs())
            .sum::<u64>() as f64
            / out.y.len() as f64;
        assert!(mae < 3.0, "v3 ac_pred I-frame MAE {mae} too large");

        // v2 carries the ac_pred bit after MCBPC (spec/07 §2.4).
        let bytes = encode_iframe_v1v2(&input, dims, &config, MsV1V2Version::V2).unwrap();
        let mut br = BitReader::new(&bytes);
        let out =
            crate::picture::decode_picture_v1v2(&mut br, dims, MsV1V2Version::V2, None).unwrap();
        let mae = out
            .y
            .iter()
            .zip(input.y.iter())
            .map(|(a, b)| (*a as i64 - *b as i64).unsigned_abs())
            .sum::<u64>() as f64
            / out.y.len() as f64;
        assert!(mae < 3.0, "v2 ac_pred I-frame MAE {mae} too large");
    }

    #[test]
    fn iframe_table_selector_rd_never_worse_and_decodes() {
        // Across several content families the 18-way selector RD must
        // (a) never lose to the historical fixed G5/G4/dc0 choice and
        // (b) still decode through the production path (the decoder
        // reads the selectors from the header it is handed).
        let dims = PictureDims::new(48, 48).unwrap();
        let fills: [fn(&mut Picture); 3] =
            [fill_gradient, fill_texture, fill_row_bands_with_x_texture];
        for (quant, fill) in
            [(2u8, 0usize), (4, 1), (8, 2), (4, 0), (8, 1)].map(|(q, f)| (q, fills[f]))
        {
            let mut input = Picture::alloc(dims, PictureType::I);
            fill(&mut input);
            let config = EncoderConfig {
                quant,
                ..Default::default()
            };
            let rd = encode_iframe_v3(&input, dims, &config).unwrap();

            // Re-run the analysis and force the historical selectors.
            let (mb_w, mb_h) = dims.mb_dims();
            let mut dc_cache = DcCache::new(mb_w, mb_h);
            let mut mbs = Vec::new();
            for my in 0..mb_h {
                for mx in 0..mb_w {
                    mbs.push(analyse_intra_mb(
                        &input,
                        &mut dc_cache,
                        mx,
                        my,
                        quant as u32,
                    ));
                }
            }
            let forced = assemble_iframe_v3(&mbs, mb_w, quant, 2, 2, 0, false).unwrap();
            assert!(
                rd.len() <= forced.len(),
                "selector RD lost to the fixed default: {} > {} (q={quant})",
                rd.len(),
                forced.len()
            );

            let mut br = BitReader::new(&rd);
            let out = decode_picture(&mut br, dims, None).unwrap();
            let mae = out
                .y
                .iter()
                .zip(input.y.iter())
                .map(|(a, b)| (*a as i64 - *b as i64).unsigned_abs())
                .sum::<u64>() as f64
                / out.y.len() as f64;
            assert!(mae < 2.0 * quant as f64, "q={quant} MAE {mae} too large");
        }
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
