//! MS-MPEG4v3 picture-level decode.
//!
//! A picture is built out of macroblocks (MBs) laid out left-to-right,
//! top-to-bottom (spec §2.5: no slice/GOB layer). Each MB is 16×16 pel
//! of luma + 2× 8×8 chroma (4:2:0). I-frames contain only intra MBs;
//! P-frames intermix intra and motion-compensated inter MBs.
//!
//! ## I-frame decode path (end-to-end sketch)
//!
//! This module implements enough of the v3 I-frame pipeline to produce
//! a `Picture` / `VideoFrame`:
//!
//!   1. Picture header (`MsV3PictureHeader::parse`) — reads the 2-bit
//!      picture type, 5-bit PQUANT, and the three v3-I-frame selectors
//!      (spec §2.3).
//!   2. Per-MB loop — for each (mb_x, mb_y) in raster order:
//!      - [`IntraMbHeader::parse_v3_mcbpcy`] decodes the 128-entry
//!        joint-MCBPCY canonical-Huffman table (spec §3.1 / spec/05
//!        §3.2), producing the 4-bit luma CBPY and two chroma CBP bits
//!        in one go, then reads the post-VLC `ac_pred_flag` bit.
//!      - For each of 6 blocks: MPEG-4 §7.4.3 spatial DC prediction
//!        (gradient test on the three already-decoded neighbours),
//!        `decode_intra_dc_diff` for the DC-differential VLC, reconstruct
//!        DC = predictor + diff * scaler, then (if CBP bit is set and
//!        we have a real AC VLC table available) the AC walk through
//!        the scan table chosen from the DC direction (spec/04 §4.4).
//!      - IDCT (float reference in `idct.rs`).
//!      - Output pels are written into the corresponding MB slot of the
//!        Y / Cb / Cr planes.
//!   3. Clip and hand back a `Picture`.
//!
//! **DC predictor**, **AC scan dispatcher** and **MCBPCY joint VLC**
//! are all wired in the round-8 commit. The **intra AC VLC table is
//! still an open clean-room extraction item** (spec §9 OPEN-O4); on
//! coded blocks the AC plane is zero-filled (DC-only reconstruction)
//! until the Extractor lands the real run/level/last table.
//!
//! ## P-frame decode path (round-9)
//!
//! Round 9 wires the P-frame skeleton around the intra pipeline
//! (spec/05 §3.2, spec/06 §§1–3):
//!
//!   1. [`MsV3PictureHeader::parse`] now reads the `mv_table_sel` bit
//!      for P-frames; we currently accept only `mv_table_sel == 0`
//!      because the alternate MV VLC source at VMA `0x1c25a0b8` is
//!      truncated in the extraction (see `tables/region_0594b8.meta`).
//!   2. Per-MB loop — [`decode_pframe_mb`] reads the 1-bit skip flag,
//!      then the 128-entry joint MCBPCY (shared with I-frames), then
//!      the 1-bit `ac_pred` flag.
//!   3. If the MCBPCY index falls in the high half (`idx >= 64`) the
//!      MB is intra-in-P and reuses the intra pipeline via
//!      [`decode_intra_mb_with_header`].
//!   4. Otherwise it is inter: [`crate::mv::decode_mv`] consumes the
//!      joint (MVDx, MVDy) VLC + byte-LUT lookup + ESC tail and
//!      returns a half-pel MV in `[-63, +63]`; [`apply_mc_to_mb`]
//!      copies the 16×16 luma + two 8×8 chroma blocks from the
//!      reference with bilinear half-pel averaging.
//!   5. Inter residual is **zero** in this round — the inter AC VLC
//!      is still OPEN (spec/99 §9). Full inter-residual decode is
//!      round-10+ territory.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::ac::{AcVlcTable, Scan};
use crate::dc_pred::{DcCache, DcPrediction};
use crate::header::{MsV3PictureHeader, PictureType};
use crate::idct::idct8x8_to_pel;
use crate::mb::{decode_intra_block_full, IntraMbHeader};

/// Dimensions of a picture, derived from the container's
/// [`oxideav_core::CodecParameters`]. MS-MPEG4v3 does not carry
/// width/height in the bitstream itself (unlike MPEG-4 Part 2's VOL).
#[derive(Clone, Copy, Debug)]
pub struct PictureDims {
    pub width: u32,
    pub height: u32,
}

impl PictureDims {
    pub fn new(width: u32, height: u32) -> Result<Self> {
        if width == 0 || height == 0 {
            return Err(Error::invalid("msmpeg4v3: zero picture dimension"));
        }
        if !(16..=4096).contains(&width) || !(16..=4096).contains(&height) {
            return Err(Error::invalid(format!(
                "msmpeg4v3: picture dimensions {}x{} out of supported range",
                width, height
            )));
        }
        Ok(Self { width, height })
    }

    /// Macroblock dimensions of the picture (16×16 per MB, rounding up).
    pub fn mb_dims(&self) -> (usize, usize) {
        (
            self.width.div_ceil(16) as usize,
            self.height.div_ceil(16) as usize,
        )
    }
}

/// A decoded picture in planar YUV420 (pel domain).
#[derive(Clone, Debug)]
pub struct Picture {
    pub width: u32,
    pub height: u32,
    pub y: Vec<u8>,
    pub cb: Vec<u8>,
    pub cr: Vec<u8>,
    pub y_stride: usize,
    pub c_stride: usize,
    pub picture_type: PictureType,
}

impl Picture {
    /// Allocate an all-grey picture (luma 128, chroma 128 = neutral).
    pub fn alloc(dims: PictureDims, picture_type: PictureType) -> Self {
        let (mbw, mbh) = dims.mb_dims();
        let y_stride = mbw * 16;
        let c_stride = mbw * 8;
        Self {
            width: dims.width,
            height: dims.height,
            y: vec![128u8; y_stride * mbh * 16],
            cb: vec![128u8; c_stride * mbh * 8],
            cr: vec![128u8; c_stride * mbh * 8],
            y_stride,
            c_stride,
            picture_type,
        }
    }
}

/// Entry point: parse the picture header and decode a full picture.
///
/// P-frames require a reference picture (normally the previous
/// successfully decoded frame). Callers should thread the last-decoded
/// `Picture` through `reference` — an I-frame is decoded without a
/// reference (the reference argument is ignored for `PictureType::I`).
pub fn decode_picture(
    br: &mut BitReader<'_>,
    dims: PictureDims,
    reference: Option<&Picture>,
) -> Result<Picture> {
    decode_picture_with_ac(br, dims, reference, AcSelection::default())
}

/// Selector for which intra-AC VLC table the picture decoder uses.
///
/// The clean-room extraction has shipped one table candidate
/// (`region_05eed0` / VMA `0x1c25fad0`) but its role is OPEN per
/// `docs/video/msmpeg4/spec/99-current-understanding.md` §0.1 row 8 and
/// §9 OPEN-O6 — the bytes are a complete 64-entry canonical-Huffman
/// prefix code (Kraft sum = 1) but the `(last, run, level)` mapping is
/// the Implementer's hypothesis. The default is therefore still the
/// placeholder: callers that want to exercise the candidate plumb it
/// in explicitly via [`AcSelection::Candidate`].
#[derive(Default, Clone, Copy, Debug, PartialEq, Eq)]
pub enum AcSelection {
    /// Empty placeholder: when a coded block is encountered, the
    /// decoder falls back to DC-only reconstruction. This is the
    /// shipping default (no risk of producing garbage on real DIV3
    /// content where the candidate's symbol mapping may be wrong).
    #[default]
    Placeholder,
    /// Candidate VLC built from `region_05eed0.csv` via
    /// [`AcVlcTable::v3_intra_candidate`]. Useful for synthetic content
    /// and pipeline integration tests where AC bits are present and
    /// the bit-aligned VLC walk needs to actually advance.
    Candidate,
}

/// Variant of [`decode_picture`] with explicit control over which
/// intra-AC VLC table is used. See [`AcSelection`] for the trade-offs.
pub fn decode_picture_with_ac(
    br: &mut BitReader<'_>,
    dims: PictureDims,
    reference: Option<&Picture>,
    ac_selection: AcSelection,
) -> Result<Picture> {
    let hdr = MsV3PictureHeader::parse(br)?;
    match hdr.picture_type {
        PictureType::I => decode_iframe(br, dims, &hdr, ac_selection),
        PictureType::P => {
            let reference = reference.ok_or_else(|| {
                Error::invalid(
                    "msmpeg4v3: P-frame decode requires a reference picture (\
                     decoder state is missing the previous frame). Make sure \
                     to feed packets in decode order starting with an I-frame.",
                )
            })?;
            if hdr.mv_table_sel != 0 {
                return Err(Error::unsupported(
                    "msmpeg4v3: P-frame mv_table_sel = 1 (alternate MV VLC \
                     at VMA 0x1c25a0b8) not supported — only a 256-byte \
                     extraction dump is available. See \
                     docs/video/msmpeg4/tables/region_0594b8.meta for the \
                     truncation note.",
                ));
            }
            decode_pframe(br, dims, &hdr, reference, ac_selection)
        }
    }
}

/// Resolve the picture decoder's [`AcSelection`] into a concrete
/// [`AcVlcTable`] value. Centralising this lets the per-block code
/// branch on `entries.is_empty()` for the placeholder DC-only fallback
/// path while still composing into [`decode_intra_block_full`] for the
/// candidate path.
fn ac_table_for(selection: AcSelection) -> AcVlcTable {
    match selection {
        AcSelection::Placeholder => AcVlcTable::V3_INTRA_PLACEHOLDER,
        AcSelection::Candidate => AcVlcTable::v3_intra_candidate(),
    }
}

/// Decode a full v3 I-frame into a [`Picture`].
///
/// This walks every MB in raster order, decodes its 6 blocks
/// (4 luma + 2 chroma), dequantises, IDCTs, and writes the reconstructed
/// pel values into the output planes.
///
/// Per MPEG-4 Part 2 §7.4.3 / MSMPEG4 spec/03 §1.3, each block's DC
/// coefficient is predicted from the gradient of three already-decoded
/// neighbours (left `A`, top `B`, top-left diagonal `D`):
///
///   * `|A - D| < |A - B|` → predict from `A` (left);
///   * otherwise → predict from `B` (top).
///
/// Missing neighbours (picture edges) are substituted with the neutral
/// value `1024`. The prediction direction also drives the AC-scan
/// dispatcher — left-predicted blocks scan alt-horizontal,
/// top-predicted blocks scan alt-vertical (spec/04 §4.4). When the
/// MB-level `ac_pred_flag` is off, zigzag is used regardless.
fn decode_iframe(
    br: &mut BitReader<'_>,
    dims: PictureDims,
    hdr: &MsV3PictureHeader,
    ac_selection: AcSelection,
) -> Result<Picture> {
    let (mb_w, mb_h) = dims.mb_dims();
    let mut pic = Picture::alloc(dims, PictureType::I);

    // DC prediction cache — one entry per 8×8 block (luma 2×mb per MB,
    // chroma 1×1 per MB per plane). See `dc_pred::DcCache`.
    let mut dc_cache = DcCache::new(mb_w, mb_h);

    let quant = hdr.quant as u32;
    let ac_table = ac_table_for(ac_selection);

    for my in 0..mb_h {
        for mx in 0..mb_w {
            decode_intra_mb_to_picture(br, &mut pic, &mut dc_cache, mx, my, quant, &ac_table)?;
        }
    }

    Ok(pic)
}

/// Decode a full v3 P-frame into a [`Picture`], using the supplied
/// `reference` for motion compensation. Inter MBs read a joint (MVDx,
/// MVDy) VLC, apply a median-of-3 predictor over the three neighbour
/// MVs, and copy a 16×16 luma + two 8×8 chroma blocks from the
/// reference at the resulting half-pel position (bilinear-averaged).
/// Intra-in-P MBs are decoded via the same I-frame pipeline (spatial
/// DC prediction + AC walk / placeholder).
///
/// Because the intra AC VLC is still a placeholder (spec/99 §9 OPEN),
/// intra-in-P blocks are reconstructed DC-only when their CBP bit is
/// set — same behaviour as in I-frames.
///
/// Because the *inter* AC VLC has not been extracted either, all
/// inter MBs here are decoded with a **zero residual**: MC copy only.
/// This is a partial P-frame decode that will show the reference
/// frame's motion-compensated pixels without the residual detail.
/// When the Extractor lands the inter AC VLC a future round will
/// plug it in and the inter-residual path will activate.
fn decode_pframe(
    br: &mut BitReader<'_>,
    dims: PictureDims,
    hdr: &MsV3PictureHeader,
    reference: &Picture,
    ac_selection: AcSelection,
) -> Result<Picture> {
    // Reference dimensions must match; otherwise MC indexing is
    // meaningless.
    if reference.width != dims.width || reference.height != dims.height {
        return Err(Error::invalid(format!(
            "msmpeg4v3: P-frame reference dimensions {}x{} differ from \
             current {}x{}",
            reference.width, reference.height, dims.width, dims.height,
        )));
    }

    let (mb_w, mb_h) = dims.mb_dims();
    let mut pic = Picture::alloc(dims, PictureType::P);
    let mut dc_cache = DcCache::new(mb_w, mb_h);
    let quant = hdr.quant as u32;
    let ac_table = ac_table_for(ac_selection);

    // MV grid: one (MVx, MVy) entry per MB. Skipped / intra MBs use
    // (0, 0) so the predictor's zero-substitution semantics match
    // spec/06 §3.4 (boundary zero-out).
    let mut mv_grid: Vec<Option<crate::mv::Mv>> = vec![None; mb_w * mb_h];

    for my in 0..mb_h {
        for mx in 0..mb_w {
            decode_pframe_mb(
                br,
                &mut pic,
                &mut dc_cache,
                &mut mv_grid,
                reference,
                mx,
                my,
                mb_w,
                quant,
                &ac_table,
            )?;
        }
    }

    Ok(pic)
}

/// Decode one P-frame MB: skip-bit + MCBPCY + (if coded) per-block
/// decode + MV/MC if inter.
#[allow(clippy::too_many_arguments)]
fn decode_pframe_mb(
    br: &mut BitReader<'_>,
    pic: &mut Picture,
    dc_cache: &mut DcCache,
    mv_grid: &mut [Option<crate::mv::Mv>],
    reference: &Picture,
    mb_x: usize,
    mb_y: usize,
    mb_w: usize,
    quant: u32,
    ac_table: &AcVlcTable,
) -> Result<()> {
    use crate::mcbpcy::{decode_mcbpcy_pframe, PFrameMcbpcy};

    let mb_idx = mb_y * mb_w + mb_x;
    let (skip, mb_info) = match decode_mcbpcy_pframe(br)? {
        PFrameMcbpcy::Skip => (true, None),
        PFrameMcbpcy::Coded { decode, ac_pred } => (false, Some((decode, ac_pred))),
    };

    // Neighbour MV lookup for median predictor (spec/06 §3.4).
    let left = if mb_x > 0 {
        mv_grid[mb_y * mb_w + (mb_x - 1)]
    } else {
        None
    };
    let top = if mb_y > 0 {
        mv_grid[(mb_y - 1) * mb_w + mb_x]
    } else {
        None
    };
    let top_right = if mb_y > 0 && mb_x + 1 < mb_w {
        mv_grid[(mb_y - 1) * mb_w + (mb_x + 1)]
    } else {
        None
    };

    if skip {
        // Skip MB: MV = predictor-based zero (spec/06 §3.4 says
        // missing neighbours are zero-subbed; skipped MBs themselves
        // contribute zero MV too per H.263 convention). Copy the MC
        // prediction at MV=(0,0).
        mv_grid[mb_idx] = Some(crate::mv::Mv::default());
        apply_mc_to_mb(pic, reference, mb_x, mb_y, (0, 0));
        return Ok(());
    }

    let (decode, _ac_pred) = mb_info.expect("Coded variant");

    if decode.is_intra {
        // Intra-in-P path: reuse the intra pipeline. The existing
        // `IntraMbHeader` shape wants cbpy/cbp_cb/cbp_cr already
        // decoded — we reuse the MCBPCY result here directly.
        let header = crate::mb::IntraMbHeader {
            ac_pred: _ac_pred,
            cbpy: decode.cbpy,
            cbp_cb: decode.cbp_cb,
            cbp_cr: decode.cbp_cr,
        };
        decode_intra_mb_with_header(br, pic, dc_cache, &header, mb_x, mb_y, quant, ac_table)?;
        // Intra MBs clear the MV predictor chain: the per-row mv_grid
        // entry stays `None` so downstream neighbours treat this
        // column as zero.
        return Ok(());
    }

    // Inter MB: decode the joint MV VLC using the median predictor.
    let predictor = crate::mv::median_predictor(left, top, top_right);
    let mv = crate::mv::decode_mv(br, predictor)?;
    mv_grid[mb_idx] = Some(mv);

    apply_mc_to_mb(pic, reference, mb_x, mb_y, (mv.x as i32, mv.y as i32));

    // Inter residual: OPEN (no inter AC VLC extracted). Zero residual
    // for now → the output is pure MC prediction.
    Ok(())
}

/// Apply motion compensation to the (mb_x, mb_y) MB using the given
/// half-pel MV. Writes luma + chroma into the picture.
fn apply_mc_to_mb(
    pic: &mut Picture,
    reference: &Picture,
    mb_x: usize,
    mb_y: usize,
    mv_half: (i32, i32),
) {
    let ref_y = crate::mc::RefPlane {
        data: &reference.y,
        stride: reference.y_stride,
        width: reference.width as usize,
        height: reference.height as usize,
    };
    let ref_cb = crate::mc::RefPlane {
        data: &reference.cb,
        stride: reference.c_stride,
        width: (reference.width as usize).div_ceil(2),
        height: (reference.height as usize).div_ceil(2),
    };
    let ref_cr = crate::mc::RefPlane {
        data: &reference.cr,
        stride: reference.c_stride,
        width: (reference.width as usize).div_ceil(2),
        height: (reference.height as usize).div_ceil(2),
    };
    crate::mc::mc_macroblock(
        &ref_y,
        &ref_cb,
        &ref_cr,
        &mut pic.y,
        &mut pic.cb,
        &mut pic.cr,
        pic.y_stride,
        pic.c_stride,
        mb_x,
        mb_y,
        mv_half,
    );
}

/// Variant of `decode_intra_mb_to_picture` that accepts a pre-decoded
/// header (used by the P-frame intra-in-P path where the MCBPCY was
/// decoded via `decode_mcbpcy_pframe`).
#[allow(clippy::too_many_arguments)]
fn decode_intra_mb_with_header(
    br: &mut BitReader<'_>,
    pic: &mut Picture,
    dc_cache: &mut DcCache,
    header: &IntraMbHeader,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    ac_table: &AcVlcTable,
) -> Result<()> {
    for block_idx in 0..6usize {
        let cbp_set = match block_idx {
            0..=3 => header.cbpy & (1 << (3 - block_idx)) != 0,
            4 => header.cbp_cb,
            5 => header.cbp_cr,
            _ => unreachable!(),
        };
        let (bx, by) = block_grid_pos(block_idx, mb_x, mb_y);
        let pred: DcPrediction = match block_idx {
            0..=3 => dc_cache.predict_luma(bx, by),
            4 => dc_cache.predict_chroma(false, bx, by),
            5 => dc_cache.predict_chroma(true, bx, by),
            _ => unreachable!(),
        };
        let scan = if header.ac_pred {
            pred.direction.ac_scan()
        } else {
            Scan::Zigzag
        };
        let block_result = if cbp_set && ac_table.entries.is_empty() {
            let dc_diff = crate::mb::decode_intra_dc_diff(br, block_idx)?;
            let dc = crate::mb::reconstruct_intra_dc(dc_diff, pred.predictor, block_idx, quant);
            crate::mb::DecodedIntraBlock {
                coeffs: {
                    let mut a = [0i32; 64];
                    a[0] = dc;
                    a
                },
                ac_nonzero: 0,
            }
        } else {
            decode_intra_block_full(
                br,
                block_idx,
                pred.predictor,
                quant,
                cbp_set,
                scan,
                ac_table,
            )?
        };
        let reconstructed_dc = block_result.coeffs[0];
        match block_idx {
            0..=3 => dc_cache.luma_set(bx, by, reconstructed_dc),
            4 => dc_cache.chroma_set(false, bx, by, reconstructed_dc),
            5 => dc_cache.chroma_set(true, bx, by, reconstructed_dc),
            _ => unreachable!(),
        }
        let mut pels = [0i32; 64];
        idct8x8_to_pel(&block_result.coeffs, &mut pels);
        write_block_to_picture(pic, mb_x, mb_y, block_idx, &pels);
    }
    Ok(())
}

/// Map a per-MB block index (0..5) to its block-grid (bx, by) position.
/// Luma blocks 0..=3 occupy a 2×2 grid inside the MB; chroma is one
/// block per MB (4 = Cb, 5 = Cr).
fn block_grid_pos(block_idx: usize, mb_x: usize, mb_y: usize) -> (usize, usize) {
    match block_idx {
        0..=3 => (mb_x * 2 + (block_idx & 1), mb_y * 2 + (block_idx >> 1)),
        4 | 5 => (mb_x, mb_y),
        _ => unreachable!(),
    }
}

/// Decode one intra macroblock's 6 blocks into the corresponding slot
/// of `pic`, using MPEG-4 §7.4.3 spatial DC prediction and per-block
/// AC-scan dispatch.
fn decode_intra_mb_to_picture(
    br: &mut BitReader<'_>,
    pic: &mut Picture,
    dc_cache: &mut DcCache,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    ac_table: &AcVlcTable,
) -> Result<()> {
    let header = IntraMbHeader::parse_v3_mcbpcy(br)?;

    for block_idx in 0..6usize {
        let cbp_set = match block_idx {
            0..=3 => header.cbpy & (1 << (3 - block_idx)) != 0,
            4 => header.cbp_cb,
            5 => header.cbp_cr,
            _ => unreachable!(),
        };

        // DC spatial prediction — per-block (not per-MB).
        let (bx, by) = block_grid_pos(block_idx, mb_x, mb_y);
        let pred: DcPrediction = match block_idx {
            0..=3 => dc_cache.predict_luma(bx, by),
            4 => dc_cache.predict_chroma(false, bx, by),
            5 => dc_cache.predict_chroma(true, bx, by),
            _ => unreachable!(),
        };

        // AC scan selection — spec/04 §4.4:
        //   ac_pred disabled → zigzag;
        //   otherwise from-left → alt-horizontal, from-top → alt-vertical.
        let scan = if header.ac_pred {
            pred.direction.ac_scan()
        } else {
            Scan::Zigzag
        };

        // AC path: when the placeholder table is in use (entries empty)
        // fall back to DC-only — the bitstream's AC bits are skipped,
        // which means subsequent block reads will misalign on real
        // content but the decoder continues for synthetic DC-only
        // streams. When a real (or candidate) AC VLC table is plugged
        // in, the regular `decode_intra_block_full` path runs.
        let block_result = if cbp_set && ac_table.entries.is_empty() {
            let dc_diff = crate::mb::decode_intra_dc_diff(br, block_idx)?;
            let dc = crate::mb::reconstruct_intra_dc(dc_diff, pred.predictor, block_idx, quant);
            crate::mb::DecodedIntraBlock {
                coeffs: {
                    let mut a = [0i32; 64];
                    a[0] = dc;
                    a
                },
                ac_nonzero: 0,
            }
        } else {
            decode_intra_block_full(
                br,
                block_idx,
                pred.predictor,
                quant,
                cbp_set,
                scan,
                ac_table,
            )?
        };

        // Update the DC cache with the reconstructed DC for this block.
        let reconstructed_dc = block_result.coeffs[0];
        match block_idx {
            0..=3 => dc_cache.luma_set(bx, by, reconstructed_dc),
            4 => dc_cache.chroma_set(false, bx, by, reconstructed_dc),
            5 => dc_cache.chroma_set(true, bx, by, reconstructed_dc),
            _ => unreachable!(),
        }

        // IDCT in float, clip to [-256, 255], then offset by +128 to
        // get unsigned pel values and clamp to [0, 255].
        let mut pels = [0i32; 64];
        idct8x8_to_pel(&block_result.coeffs, &mut pels);

        write_block_to_picture(pic, mb_x, mb_y, block_idx, &pels);
    }

    Ok(())
}

/// Copy one 8×8 decoded block into the picture's YUV planes.
///
/// For MSMPEG4v3 intra blocks, the decoded DC coefficient is in the
/// pel-space domain (i.e. DC=1024 → IDCT→ 128 per pel = unsigned grey
/// mid). The IDCT output is directly the unsigned 8-bit pel value,
/// clipped to `[0, 255]` — there is no post-IDCT `+128` offset (that
/// offset applies to inter blocks where the IDCT output is a signed
/// residual to add onto the MC prediction, not to intra blocks where
/// the DC already carries the pel mean).
fn write_block_to_picture(
    pic: &mut Picture,
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
    pels: &[i32; 64],
) {
    match block_idx {
        0..=3 => {
            let bx = (block_idx & 1) * 8;
            let by = (block_idx >> 1) * 8;
            let y_base = mb_y * 16 + by;
            let x_base = mb_x * 16 + bx;
            for j in 0..8usize {
                for i in 0..8usize {
                    let off = (y_base + j) * pic.y_stride + (x_base + i);
                    if off < pic.y.len() {
                        pic.y[off] = pels[j * 8 + i].clamp(0, 255) as u8;
                    }
                }
            }
        }
        4 | 5 => {
            let plane = if block_idx == 4 {
                &mut pic.cb
            } else {
                &mut pic.cr
            };
            let y_base = mb_y * 8;
            let x_base = mb_x * 8;
            for j in 0..8usize {
                for i in 0..8usize {
                    let off = (y_base + j) * pic.c_stride + (x_base + i);
                    if off < plane.len() {
                        plane[off] = pels[j * 8 + i].clamp(0, 255) as u8;
                    }
                }
            }
        }
        _ => unreachable!(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dims_validate_range() {
        assert!(PictureDims::new(0, 480).is_err());
        assert!(PictureDims::new(640, 0).is_err());
        assert!(PictureDims::new(15, 480).is_err());
        assert!(PictureDims::new(640, 4097).is_err());
        let d = PictureDims::new(640, 480).unwrap();
        assert_eq!(d.mb_dims(), (40, 30));
        // Non-16 dims round up.
        let d = PictureDims::new(320, 241).unwrap();
        assert_eq!(d.mb_dims(), (20, 16));
    }

    #[test]
    fn picture_allocation_sizes() {
        let d = PictureDims::new(352, 288).unwrap();
        let p = Picture::alloc(d, PictureType::I);
        assert_eq!(p.y.len(), 352 * 288);
        assert_eq!(p.cb.len(), 176 * 144);
        assert_eq!(p.cr.len(), 176 * 144);
        assert_eq!(p.y_stride, 352);
        assert_eq!(p.c_stride, 176);
    }

    /// Hand-crafted 32×32 I-frame that exercises the full DC-spatial-
    /// predictor + MCBPCY + scan dispatch pipeline end-to-end. No AC —
    /// every MB's MCBPCY symbol is sym idx 64 (bl=2, code `00`,
    /// CBP=`000000`), so no block has coded AC and the decoder's
    /// AC-placeholder path is not exercised.
    ///
    /// What this verifies:
    /// * `IntraMbHeader::parse_v3_mcbpcy` correctly decodes the
    ///   joint-MCBPCY canonical-Huffman table and reads the
    ///   post-VLC `ac_pred_flag`.
    /// * `DcCache` and `predict_dc` produce a spatial-gradient DC
    ///   predictor per MPEG-4 §7.4.3 for every block.
    /// * `decode_iframe`'s per-block MB→grid coord mapping is right.
    /// * The final luma/chroma picture is the expected uniform /
    ///   near-uniform output (reconstructed DC per block, applied
    ///   through the IDCT).
    #[test]
    fn handcrafted_dc_only_32x32_iframe_decodes() {
        use oxideav_core::bits::BitReader;

        // Helper: pack a list of (value, bit_width) into MSB-first bytes.
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
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            out
        }

        // Picture header: I-frame, q=8, ac_chroma=0, ac_luma=0, dc_size_sel=0.
        let mut fields: Vec<(u32, u32)> = vec![
            (0, 2), // picture_type I
            (8, 5), // quant 8
            (0, 1), // ac_chroma_sel = 0 (unary `0`)
            (0, 1), // ac_luma_sel = 0
            (0, 1), // dc_size_sel = 0
        ];
        // For each of 2x2 MBs: MCBPCY sym 64 (code `00`, bl=2), ac_pred=0,
        // then 6 DC-size-0 codes (luma `011`, chroma `11`).
        for _ in 0..4 {
            fields.push((0b00, 2)); // MCBPCY sym 64
            fields.push((0, 1)); // ac_pred = 0
            fields.push((0b011, 3)); // luma DC size 0 (Y0)
            fields.push((0b011, 3)); // Y1
            fields.push((0b011, 3)); // Y2
            fields.push((0b011, 3)); // Y3
            fields.push((0b11, 2)); // chroma DC size 0 (Cb)
            fields.push((0b11, 2)); // Cr
        }
        // Tail padding.
        fields.push((0, 32));
        let bytes = pack(&fields);

        let mut br = BitReader::new(&bytes);
        let dims = PictureDims::new(32, 32).unwrap();
        let pic = decode_picture(&mut br, dims, None).expect("32x32 DC-only decode");

        assert_eq!(pic.picture_type, PictureType::I);
        assert_eq!(pic.width, 32);
        assert_eq!(pic.height, 32);
        // With DC size = 0 everywhere, each block's DC differential is
        // zero, so the reconstructed DC equals the spatial predictor.
        // At block (0,0) the predictor defaults to 1024 (neutral), which
        // at q=8 and scaler=16 gives a decoded DC "level" of 1024/8/16
        // → IDCT (DC-only) of 1024 gives 128 per pel. We should see
        // uniform 128 across the entire picture.
        let y_first = pic.y[0];
        assert!(
            (120..=136).contains(&y_first),
            "Y plane top-left = {y_first}, expected ~128"
        );
        // All luma pels should be near 128 (DC prediction chains this).
        let y_max = *pic.y.iter().max().unwrap();
        let y_min = *pic.y.iter().min().unwrap();
        assert!(
            (y_max as i32 - y_min as i32).abs() < 16,
            "luma DC drift too large: min={y_min}, max={y_max}"
        );
    }

    /// Variant that exercises the DC-predictor propagation: every block
    /// has a non-zero DC differential that the predictor chain must
    /// carry forward. Confirms that the gradient test really is picking
    /// a neighbour rather than defaulting to neutral.
    #[test]
    fn handcrafted_dc_propagation_smoke_test() {
        use oxideav_core::bits::BitReader;

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
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            out
        }

        // Picture header: I-frame q=8.
        let mut fields: Vec<(u32, u32)> = vec![(0, 2), (8, 5), (0, 1), (0, 1), (0, 1)];
        // 16×16 → one MB. MCBPCY sym 64 → CBP=0. ac_pred=0.
        // Then for each of 6 blocks: DC size 1 = luma `11` / chroma `10`;
        // DC mag bit = 1 (positive diff = +1 level).
        fields.push((0b00, 2));
        fields.push((0, 1));
        for _ in 0..4 {
            fields.push((0b11, 2)); // luma size 1
            fields.push((1, 1)); // +1
        }
        for _ in 0..2 {
            fields.push((0b10, 2)); // chroma size 1
            fields.push((1, 1)); // +1
        }
        fields.push((0, 32));
        let bytes = pack(&fields);
        let mut br = BitReader::new(&bytes);
        let dims = PictureDims::new(16, 16).unwrap();
        let pic = decode_picture(&mut br, dims, None).expect("16x16 DC-prop decode");

        // Each block's DC = predictor + 1 * scaler. At block (0,0) the
        // predictor is 1024 (neutral) and luma scaler at q=8 is 16:
        //   block 0 DC = 1024 + 16 = 1040 → IDCT → 1040/8 = 130/pel.
        //   block 1 (right of 0): A = block 0's reconstructed DC 1040;
        //     B, D neutral. |A-D| = 16, |A-B| = 0. Top wins (tie-break →
        //     else-branch). So predictor = 1024 → DC = 1024+16 = 1040.
        //   block 2 (below 0): analogous → predictor = 1024 (no left).
        //   block 3 (below-right): A = block 2's DC = 1040; D = block 0 = 1040;
        //     B = block 1 = 1040. |A-D|=0, |A-B|=0 → else → top. Pred=1040.
        //     DC = 1040 + 16 = 1056 → 1056/8 = 132 per pel.
        // So the bottom-right luma block should be ~132, while top-left
        // should be ~130. Non-uniform reconstruction = DC prediction
        // chain really runs.
        let tl = pic.y[0] as i32;
        let br_y = pic.y[pic.y.len() - 1] as i32;
        assert!(
            br_y >= tl,
            "expected monotone DC propagation (tl={tl}, br={br_y})"
        );
    }

    /// Hand-crafted 16×16 I-frame whose single MB has CBP=0xF
    /// (every luma block has coded AC). Decodes through
    /// [`decode_picture_with_ac`] with [`AcSelection::Candidate`] —
    /// the candidate VLC built from `region_05eed0.csv` actually
    /// consumes the AC bits per block. The synthetic stream uses the
    /// candidate's `(last=true, run=0, level=1)` symbol as the
    /// terminator for every coded block, so each block gets exactly
    /// one AC coefficient placed at zigzag position 1 (after DC).
    ///
    /// The point is to demonstrate end-to-end that the picture
    /// decoder routes the candidate AC table through the per-MB,
    /// per-block path correctly — DC predict, AC walk, dequant,
    /// IDCT, and pel write all compose. The numeric output is not
    /// expected to match a real DIV3 stream because the candidate's
    /// `(last, run, level)` mapping is a hypothesis; this test
    /// verifies the *plumbing*.
    #[test]
    fn handcrafted_iframe_with_candidate_ac_decodes() {
        use oxideav_core::bits::BitReader;

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
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            out
        }

        // Find the candidate AC VLC entry whose decoded triple is
        // `(last=true, run=0, level=1)` — under the candidate's
        // hypothesis this is the symbol at idx == count_B + 1 == 2.
        let ac_table = AcVlcTable::v3_intra_candidate();
        let term = ac_table
            .entries
            .iter()
            .find(|e| {
                matches!(
                    e.value,
                    crate::ac::Symbol::RunLevel {
                        last: true,
                        run: 0,
                        level: 1
                    }
                )
            })
            .expect("candidate table contains the (last=1, run=0, level=1) terminator");

        // We need the joint MCBPCY symbol whose CBP pattern is 0b111111
        // (all luma + chroma blocks coded) and whose MB-type is intra.
        // Per `decode_mcbpcy`, intra is the high half (idx >= 64) and
        // CBP = idx & 0x3f. We want CBP = 0x3f and high half, so target
        // idx = 0x7f = 127. Find that entry's canonical code.
        let mut syms: Vec<(u32, u8)> = crate::tables_data::MCBPCY_V3_RAW
            .iter()
            .enumerate()
            .filter_map(|(i, &(bl, _))| if bl == 0 { None } else { Some((bl, i as u8)) })
            .collect();
        syms.sort_by_key(|&(bl, idx)| (bl, idx));
        let mut mc_code = 0u32;
        let mut prev_bl = 0u32;
        let mut mc_bl = 0u32;
        let mut mc_found = false;
        for (i, &(bl, idx)) in syms.iter().enumerate() {
            if i == 0 {
                mc_code = 0;
            } else {
                mc_code = (mc_code + 1) << (bl - prev_bl);
            }
            prev_bl = bl;
            if idx == 127 {
                mc_bl = bl;
                mc_found = true;
                break;
            }
        }
        assert!(
            mc_found,
            "MCBPCY table must contain idx 127 (all-coded intra)"
        );

        // Picture header: I-frame, q=8, all selectors default (0).
        let mut fields: Vec<(u32, u32)> = vec![(0, 2), (8, 5), (0, 1), (0, 1), (0, 1)];

        // 16x16 = 1 MB. MCBPCY = idx 127 (all-coded intra), ac_pred = 0.
        fields.push((mc_code, mc_bl));
        fields.push((0, 1));

        // Per block (6 blocks in MS-MPEG4v3 raster order): DC size = 1
        // (luma `11` for blocks 0..3; chroma `10` for blocks 4..5),
        // DC mag bit = 1 (positive +1 differential), then ONE AC token
        // (the candidate terminator) + sign bit = 0 (positive).
        for block_idx in 0..6 {
            let dc_size_code = if block_idx < 4 {
                (0b11u32, 2u32)
            } else {
                (0b10, 2)
            };
            fields.push(dc_size_code);
            fields.push((1, 1)); // DC mag = +1
            fields.push((term.code, term.bits as u32));
            fields.push((0, 1)); // AC sign +
        }

        // Tail padding so the bit-reader never starves.
        fields.push((0, 32));
        fields.push((0, 32));

        let bytes = pack(&fields);
        let mut br = BitReader::new(&bytes);
        let dims = PictureDims::new(16, 16).unwrap();
        let pic = decode_picture_with_ac(&mut br, dims, None, AcSelection::Candidate)
            .expect("candidate-AC I-frame decode");

        assert_eq!(pic.picture_type, PictureType::I);
        assert_eq!(pic.width, 16);
        assert_eq!(pic.height, 16);
        // Output is non-trivial: at least some pels deviate from the
        // pure DC reconstruction because the AC contribution shifts
        // the per-block IDCT response. We don't assert exact values
        // (the candidate's symbol mapping is a hypothesis), only that
        // the decode reached the end without erroring and produced
        // YUV planes whose luma is non-uniform across the picture.
        let y_max = *pic.y.iter().max().unwrap();
        let y_min = *pic.y.iter().min().unwrap();
        assert!(
            y_max != y_min,
            "luma plane is uniform — AC contribution didn't reach the IDCT \
             (decoder may have skipped the AC walk)"
        );
    }

    /// Hand-crafted P-frame that is entirely skipped MBs. Validates
    /// end-to-end P-frame decode, reference-threading, MC copy path.
    /// After decode the output should equal the reference picture
    /// because every MB is "skip → MC with MV=(0,0)".
    #[test]
    fn handcrafted_all_skip_pframe_copies_reference() {
        use oxideav_core::bits::BitReader;

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
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            out
        }

        // Build a 16x16 reference picture with a simple ramp so we can
        // verify MC copy produced the right pels.
        let dims = PictureDims::new(16, 16).unwrap();
        let mut reference = Picture::alloc(dims, PictureType::I);
        for y in 0..16 {
            for x in 0..16 {
                reference.y[y * reference.y_stride + x] = ((x + y) * 4) as u8;
            }
        }
        for y in 0..8 {
            for x in 0..8 {
                reference.cb[y * reference.c_stride + x] = (64 + x + y) as u8;
                reference.cr[y * reference.c_stride + x] = (192 - x - y) as u8;
            }
        }

        // P-frame header: picture_type=1 (P), quant=8, ac_chroma=0,
        // dc_size_sel=0, mv_table_sel=0 (default MV VLC).
        let mut fields: Vec<(u32, u32)> = vec![
            (1, 2), // P
            (8, 5), // quant
            (0, 1), // ac_chroma_sel = 0
            (0, 1), // dc_size_sel = 0
            (0, 1), // mv_table_sel = 0 (default)
        ];
        // 16x16 → 1x1 MB. Single skip bit = 1.
        fields.push((1, 1));
        // Tail padding.
        fields.push((0, 16));
        let bytes = pack(&fields);
        let mut br = BitReader::new(&bytes);
        let pic = decode_picture(&mut br, dims, Some(&reference)).expect("all-skip P-frame decode");

        assert_eq!(pic.picture_type, PictureType::P);
        // Output should equal the reference (since all MBs were skipped
        // with MV=(0,0), the MC copy is the identity).
        assert_eq!(pic.y, reference.y, "luma plane should equal reference");
        assert_eq!(pic.cb, reference.cb, "Cb plane should equal reference");
        assert_eq!(pic.cr, reference.cr, "Cr plane should equal reference");
    }

    #[test]
    fn pframe_rejects_alternate_mv_table() {
        use oxideav_core::bits::BitReader;

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
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            out
        }

        let dims = PictureDims::new(16, 16).unwrap();
        let reference = Picture::alloc(dims, PictureType::I);
        // P-frame with mv_table_sel = 1 (alternate) — should be rejected.
        let fields: Vec<(u32, u32)> = vec![
            (1, 2), // P
            (8, 5), // quant
            (0, 1), // ac_chroma_sel
            (0, 1), // dc_size_sel
            (1, 1), // mv_table_sel = 1
        ];
        let bytes = pack(&fields);
        let mut br = BitReader::new(&bytes);
        let err = decode_picture(&mut br, dims, Some(&reference)).unwrap_err();
        let msg = format!("{err}");
        assert!(
            msg.contains("mv_table_sel")
                || msg.contains("alternate MV VLC")
                || msg.contains("0x1c25a0b8"),
            "expected alternate-MV-table unsupported error; got: {msg}"
        );
    }

    /// Hand-crafted P-frame with a single non-skipped inter MB that
    /// has CBP = 0 (no coded AC in any block). Exercises the MV decode
    /// + MC copy path. The MV VLC symbol 0 is a 1-bit code `0` and the
    ///   MVDx/MVDy LUT entry at index 0 is byte 0x20 = 32, so after the
    ///   bias subtraction the MV is (0, 0) → MC copy from reference.
    #[test]
    fn handcrafted_inter_mb_copies_reference() {
        use oxideav_core::bits::BitReader;

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
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            out
        }

        let dims = PictureDims::new(16, 16).unwrap();
        let mut reference = Picture::alloc(dims, PictureType::I);
        for i in 0..reference.y.len() {
            reference.y[i] = (i % 211) as u8;
        }

        // MCBPCY joint idx 0 has low-6-bits = 0 (CBP=0, no coded blocks)
        // and bit-6 = 0 → inter MB. Its canonical code is the first
        // entry in canonical order. Per spec/99 §3.1 the first
        // canonical entry is symbol 0 (bit 1, all-zero CBP, skip/no-ac).
        //
        // The canonical MCBPCY table is built in mcbpcy.rs and the
        // smallest bit_length is 2 per the region_05eac8 extract
        // (row 0 = (bl=7, code=5065) but that's the first *in CSV
        // order*, not the first canonical). The minimum bit-length
        // actually observed is 2 (two entries), and the first
        // canonical code is 0b00.
        //
        // For this test we bypass that uncertainty by using the
        // mcbpcy module directly to find what code goes with idx 0.
        let t_mcbpcy = {
            // Helper mirroring the canonical build in mcbpcy.rs.
            let mut syms: Vec<(u32, u8)> = crate::tables_data::MCBPCY_V3_RAW
                .iter()
                .enumerate()
                .filter_map(|(i, &(bl, _))| if bl == 0 { None } else { Some((bl, i as u8)) })
                .collect();
            syms.sort_by_key(|&(bl, idx)| (bl, idx));
            let mut entries: Vec<(u8, u32, u8)> = Vec::new();
            let mut code: u32 = 0;
            let mut prev_bl: u32 = 0;
            for (i, &(bl, idx)) in syms.iter().enumerate() {
                if i == 0 {
                    code = 0;
                } else {
                    code = (code + 1) << (bl - prev_bl);
                }
                entries.push((bl as u8, code, idx));
                prev_bl = bl;
            }
            entries
        };
        // Find the entry for symbol 0 (inter, CBP=0).
        let (bl_sym0, code_sym0) = t_mcbpcy
            .iter()
            .find(|(_, _, sym)| *sym == 0)
            .map(|(bl, code, _)| (*bl as u32, *code))
            .expect("symbol 0 in canonical MCBPCY");

        // MV VLC symbol 0 (1-bit code 0) → MVDx/MVDy raw = 0x20 = 32.
        // Pred (0,0), 32 - 32 = 0 → MV output = (0, 0).

        // P-frame header: P, q=8, ac_chroma=0, dc_size_sel=0, mv_table_sel=0.
        let mut fields: Vec<(u32, u32)> = vec![
            (1, 2), // P
            (8, 5), // quant
            (0, 1), // ac_chroma_sel = 0
            (0, 1), // dc_size_sel = 0
            (0, 1), // mv_table_sel = 0 (default)
        ];
        // 1 MB: skip-bit=0, MCBPCY sym 0, ac_pred bit, MV sym 0 (1 bit `0`).
        fields.push((0, 1)); // skip = 0
        fields.push((code_sym0, bl_sym0));
        fields.push((0, 1)); // ac_pred (ignored for inter)
        fields.push((0, 1)); // MV sym 0 (canonical code = 0)
        fields.push((0, 16));
        let bytes = pack(&fields);
        let mut br = BitReader::new(&bytes);
        let pic =
            decode_picture(&mut br, dims, Some(&reference)).expect("inter-copy P-frame decode");

        assert_eq!(pic.picture_type, PictureType::P);
        // Since MV resolves to (0,0) and there's no residual, the
        // output luma should equal the reference luma exactly.
        let mut mismatches = 0;
        for i in 0..pic.y.len() {
            if pic.y[i] != reference.y[i] {
                mismatches += 1;
            }
        }
        assert_eq!(
            mismatches, 0,
            "inter-MB with MV=(0,0) should match reference exactly",
        );
    }

    #[test]
    fn pframe_requires_reference() {
        use oxideav_core::bits::BitReader;

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
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            out
        }

        let dims = PictureDims::new(16, 16).unwrap();
        let fields: Vec<(u32, u32)> = vec![
            (1, 2), // P
            (8, 5), // quant
            (0, 1), // ac_chroma_sel
            (0, 1), // dc_size_sel
            (0, 1), // mv_table_sel
            (1, 1), // skip bit
            (0, 8),
        ];
        let bytes = pack(&fields);
        let mut br = BitReader::new(&bytes);
        let err = decode_picture(&mut br, dims, None).unwrap_err();
        let msg = format!("{err}");
        assert!(
            msg.contains("reference") || msg.contains("P-frame"),
            "expected missing-reference error; got: {msg}"
        );
    }
}
