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
//!        [`crate::mb::decode_intra_dc_diff_v3`] for the v3 custom
//!        120-entry direct-value DC-differential VLC (per spec/07
//!        §5.4 + spec/11 §4 + spec/12 §3), with the picture-header
//!        `dc_size_sel` bit choosing between the two table pairs;
//!        reconstruct DC = predictor + diff * scaler, then (if CBP
//!        bit is set and we have a real AC VLC table available)
//!        the AC walk through the scan table chosen from the DC
//!        direction (spec/04 §4.4).
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
//!   1. [`MsV3PictureHeader::parse`] reads the `mv_table_sel` bit for
//!      P-frames; both `mv_table_sel == 0` (default) and `== 1`
//!      (alternate) MV VLC variants now decode end-to-end — the
//!      alternate source at VMA `0x1c25a0b8` was re-extracted at full
//!      size in Extractor 07 (spec/16 §1, `tables/region_0594b8_mvvlc.csv`).
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
use crate::header::{MsV1V2PictureHeader, MsV3PictureHeader, PictureType};
use crate::idct::idct8x8_to_pel;
use crate::mb::{decode_intra_block_full_v3, IntraMbHeader};

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
/// Round 26 (2026-05-01) wires the **G5 (intra-luma) primary VLC** built
/// from the packed-Huffman source at file `0x59178` per
/// `docs/video/msmpeg4/spec/11-walker-format-resolved.md`. G5 is now the
/// shipping default — the placeholder and 64-entry candidate are kept as
/// alternative selectors for pipeline tests and historical compatibility.
///
/// Round 32 (2026-05-03) adds [`AcSelection::FromHeader`] — the
/// dispatch path that actually consults the per-frame
/// `hdr.ac_chroma_sel` / `hdr.ac_luma_sel` selector bits from
/// [`MsV3PictureHeader`] and routes each (luma, chroma) block pair to
/// the correct G-family per spec/14 §3.1. This is the production
/// default for real-content decode; the G5 / Placeholder / Candidate
/// variants remain available for hand-crafted synthetic streams.
#[derive(Default, Clone, Copy, Debug, PartialEq, Eq)]
pub enum AcSelection {
    /// Per-frame dispatch from the picture-header selector bits.
    /// Resolves each block to one of {G0..G5} by mapping
    /// `hdr.ac_luma_sel` ∈ {0,1,2} → {G3, G1, G5} (luma) and
    /// `hdr.ac_chroma_sel` ∈ {0,1,2} → {G2, G0, G4} (chroma) per
    /// spec/14 §3.1. The dispatch routes through the named
    /// [`AcVlcTable::v3_intra_g0`] / [`v3_intra_g1`] /
    /// [`v3_intra_g2`] / [`v3_intra_g3`] / [`v3_intra_g5`] /
    /// [`g4_inter`] constructors. As of round 234 all six families
    /// carry their real packed-Huffman primary VLC — the four
    /// extended-alphabet sources at file offsets `0x57a30 / 0x57f80
    /// / 0x58558 / 0x58a08` (spec/11 §5 row 1-4) are wired through
    /// `v3_intra_g{0,1,2,3}()`, so every `ac_luma_sel` / `ac_chroma_sel`
    /// value decodes its coded AC blocks through the spec-correct
    /// G-family rather than the DC-only fallback. End-to-end coverage
    /// of the G3 (`ac_luma_sel == 0`) path through this dispatch lives
    /// in `tests/g3_iframe_end_to_end.rs`.
    #[default]
    FromHeader,
    /// Real G5 primary VLC built from the packed-Huffman source at file
    /// `0x59178` / VMA `0x1c259d78` (102 + 1 ESC entries) wired through
    /// [`AcVlcTable::v3_intra_g5`]. The post-VLC `(idx → (last, run,
    /// |level|))` mapping is the G5 descriptor from
    /// [`crate::g_descriptor::g5_decode`] (round 18/19). Hard-wired
    /// luma=G5, chroma=G4 path — useful for pre-round-32 regression
    /// comparison and tests that hand-craft a stream with `ac_luma_sel
    /// = 2`, `ac_chroma_sel = 2` selector bits.
    G5,
    /// Empty placeholder: when a coded block is encountered, the
    /// decoder falls back to DC-only reconstruction. Retained as an
    /// opt-in for diagnostic / regression-comparison runs.
    Placeholder,
    /// Candidate VLC built from `region_05eed0.csv` via
    /// [`AcVlcTable::v3_intra_candidate`]. Pre-round-26 plumbing for
    /// synthetic-stream pipeline tests; superseded by [`AcSelection::G5`]
    /// for real-content decode.
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
    // Diagnostic trace gated on `OXIDEAV_MSMPEG4_AC_TRACE`. Shows the
    // picture-header field values (so the implementer can tell when a
    // fixture is using the alternate intra-DC VLC `dc_size_sel=1` for
    // example, which our standard-MPEG-4-P2 DC-size tables cannot decode
    // bit-exactly — see task #113 and `crate::mb::decode_intra_dc_diff`).
    if std::env::var_os("OXIDEAV_MSMPEG4_AC_TRACE").is_some() {
        eprintln!(
            "[hdr trace] type={:?} q={} ac_chroma_sel={} ac_luma_sel={} dc_size_sel={} mv_table_sel={} bit_pos={}",
            hdr.picture_type, hdr.quant, hdr.ac_chroma_sel, hdr.ac_luma_sel,
            hdr.dc_size_sel, hdr.mv_table_sel, br.bit_position(),
        );
    }
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
            // Round 32 piece 3: per-frame `mv_table_sel` bit is now
            // threaded through `decode_pframe` → `decode_pframe_mb`
            // → `decode_mv_with_table`. The alternate variant (sel=1)
            // is still a placeholder (extraction blocker — see
            // [`crate::mv::MvTable::Alternate`] doc), but the
            // unsupported error now fires from the MV decode site
            // with full diagnostic instead of the early picture-
            // header reject. This makes the failure observable per
            // P-frame instead of per-packet, and lets all the
            // mv_table_sel=0 P-frames between two sel=1 ones still
            // decode normally.
            decode_pframe(br, dims, &hdr, reference, ac_selection)
        }
    }
}

/// Resolve the picture decoder's [`AcSelection`] into the **luma**
/// intra-AC table (blocks 0..=3 of an MB). Per
/// `docs/video/msmpeg4/spec/99-current-understanding.md` §5 / §5.2 and
/// spec/14 §3.1, luma uses slot `[esi+0xab4]`. The picture-header
/// `ac_luma_sel` ∈ {0, 1, 2} field selects between {G3, G1, G5}
/// respectively, dispatched through the named
/// [`AcVlcTable::v3_intra_g3`] / [`v3_intra_g1`] / [`v3_intra_g5`]
/// constructors. As of round 234 all three luma families carry their
/// real packed-Huffman primary VLC (G3 source at file `0x58a08`, G1 at
/// `0x57f80`, G5 at `0x59178` — spec/11 §5), so a coded luma block
/// decodes its AC through the spec-correct family for every
/// `ac_luma_sel` value rather than the DC-only fallback.
fn luma_ac_table_for(selection: AcSelection, hdr: &MsV3PictureHeader) -> AcVlcTable {
    match selection {
        AcSelection::FromHeader => match hdr.ac_luma_sel {
            // Spec/14 §3.1: 0 → G3, 1 → G1, 2 → G5. All three carry
            // their real packed-Huffman primary VLC (round 234), so the
            // AC walk runs end-to-end for each.
            0 => AcVlcTable::v3_intra_g3(),
            1 => AcVlcTable::v3_intra_g1(),
            2 => AcVlcTable::v3_intra_g5(),
            // Header parser already clamps to {0, 1, 2} via the unary-
            // capped-at-2 read; this branch is unreachable in practice
            // but guard against future header changes.
            _ => AcVlcTable::v3_intra_g5(),
        },
        AcSelection::G5 => AcVlcTable::v3_intra_g5(),
        AcSelection::Placeholder => AcVlcTable::V3_INTRA_PLACEHOLDER,
        AcSelection::Candidate => AcVlcTable::v3_intra_candidate(),
    }
}

/// Resolve the picture decoder's [`AcSelection`] into the **chroma**
/// intra-AC table (blocks 4 = Cb, 5 = Cr). Per spec/99 §5 / §5.2 and
/// spec/14 §3.1 the chroma path lives in slot `[esi+0xab0]`. The
/// picture-header `ac_chroma_sel` ∈ {0, 1, 2} field selects between
/// {G2, G0, G4} respectively, dispatched through the named
/// [`AcVlcTable::v3_intra_g2`] / [`v3_intra_g0`] / [`g4_inter`]
/// constructors. As of round 234 all three chroma families carry their
/// real packed-Huffman primary VLC (G2 source at file `0x58558`, G0 at
/// `0x57a30`, G4 at `0x58e38` — spec/11 §5), so a coded chroma block
/// decodes its AC through the spec-correct family for every
/// `ac_chroma_sel` value rather than the DC-only fallback.
fn chroma_ac_table_for(selection: AcSelection, hdr: &MsV3PictureHeader) -> AcVlcTable {
    match selection {
        AcSelection::FromHeader => match hdr.ac_chroma_sel {
            // Spec/14 §3.1: 0 → G2, 1 → G0, 2 → G4. All three carry
            // their real packed-Huffman primary VLC (round 234), so the
            // AC walk runs end-to-end for each.
            0 => AcVlcTable::v3_intra_g2(),
            1 => AcVlcTable::v3_intra_g0(),
            2 => AcVlcTable::g4_inter(),
            _ => AcVlcTable::g4_inter(),
        },
        AcSelection::G5 => AcVlcTable::g4_inter(),
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
    let luma_ac = luma_ac_table_for(ac_selection, hdr);
    let chroma_ac = chroma_ac_table_for(ac_selection, hdr);

    for my in 0..mb_h {
        for mx in 0..mb_w {
            decode_intra_mb_to_picture(
                br,
                &mut pic,
                &mut dc_cache,
                mx,
                my,
                quant,
                hdr.dc_size_sel,
                &luma_ac,
                &chroma_ac,
            )?;
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
/// The *inter* AC residual now decodes through the G4 inter VLC
/// ([`AcVlcTable::g4_inter`], whose packed-Huffman primary VLC is fully
/// extracted): each inter MB lays down its MC prediction and then adds
/// the per-block IDCT residual for every CBP-coded block (spec/04 §1 /
/// §2.6). Intra-in-P AC remains placeholder/DC-only where the intra
/// G-table VLC has not been extracted (spec/99 §9 OPEN).
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
    let luma_ac = luma_ac_table_for(ac_selection, hdr);
    let chroma_ac = chroma_ac_table_for(ac_selection, hdr);
    // Inter residual VLC: the G4 (chroma + all-inter) table is the
    // v1/v2/v3 inter-block AC alphabet (spec/04 §1, kernel
    // `0x1c215d2c`). It has a fully-extracted packed-Huffman primary
    // VLC, so inter residuals decode end-to-end. The same table covers
    // luma and chroma inter blocks — the inter kernel does not branch
    // on plane (spec/04 §2.6 "Magnitude/bias pairs" row: inter uses a
    // single mag/bias pair for all six blocks).
    let inter_ac = crate::ac::AcVlcTable::g4_inter();

    // MV grid: one [`crate::mv_pred::MvGridCell`] per MB. Round 240
    // (2026-06-06) replaces the previous parallel `Vec<Option<Mv>>`
    // book-keeping with the picture-wide grid surface introduced by
    // round 227. The grid's
    // [`crate::mv_pred::MvGrid::neighbour_set_for`] folds the three
    // §7.6.5-relevant neighbour positions (`left`, `above`,
    // `above_right`) into a [`crate::mv_pred::NeighbourSet`] —
    // out-of-bounds and corner cells are substituted with
    // [`crate::mv_pred::MvGridCell::Absent`] per
    // `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`'s
    // "Boundary substitution". Skip / inter MBs store
    // [`MvGridCell::OneMv`] (the 1-MV-per-MB v3 mode is the only
    // codepath currently exercised); intra-in-P MBs leave the cell
    // `Absent` so downstream neighbours treat that column as zero per
    // the existing semantics in `decode_pframe_mb`.
    let mut mv_grid = crate::mv_pred::MvGrid::new(mb_w, mb_h);

    // Spec/06 §2.1: per-frame `mv_table_sel` ∈ {0, 1} picks between
    // the default and alternate v3 MV VLC sources. Both are now fully
    // wired (the alternate VLC source landed in Extractor 07 / spec/16
    // §1; see [`crate::mv::MvTable::Alternate`] doc).
    let mv_table = if hdr.mv_table_sel == 0 {
        crate::mv::MvTable::Default
    } else {
        crate::mv::MvTable::Alternate
    };

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
                quant,
                hdr.dc_size_sel,
                &luma_ac,
                &chroma_ac,
                &inter_ac,
                mv_table,
            )?;
        }
    }

    Ok(pic)
}

/// Decode one P-frame MB: skip-bit + MCBPCY + (if coded) per-block
/// decode + MV/MC if inter.
///
/// Round 240 (2026-06-06): the parallel `Vec<Option<Mv>>` book-keeping
/// for the per-MB MV cache is replaced by a [`crate::mv_pred::MvGrid`].
/// Neighbour lookup goes through
/// [`crate::mv_pred::MvGrid::neighbour_set_for`], producing a
/// [`crate::mv_pred::NeighbourSet`] that folds the picture-edge
/// substitution rule (corner / top-row / left-edge / right-edge cells
/// are surfaced as [`crate::mv_pred::MvGridCell::Absent`]) into the
/// caller without any per-axis range arithmetic at this layer. The
/// 1-MV-per-MB v3 mode currently shipping consumes that
/// [`crate::mv_pred::NeighbourSet`] as a
/// [`crate::mv_pred::BlockCandidates`] for [`crate::mv_pred::Block::TopLeft`],
/// then writes [`crate::mv_pred::MvGridCell::OneMv`] back into the
/// grid via [`crate::mv_pred::MvGrid::set_cell`] — intra-in-P MBs
/// leave the cell `Absent` so downstream median predictors treat that
/// column as zero per the existing semantics.
#[allow(clippy::too_many_arguments)]
fn decode_pframe_mb(
    br: &mut BitReader<'_>,
    pic: &mut Picture,
    dc_cache: &mut DcCache,
    mv_grid: &mut crate::mv_pred::MvGrid,
    reference: &Picture,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    dc_size_sel: u8,
    luma_ac: &AcVlcTable,
    chroma_ac: &AcVlcTable,
    inter_ac: &AcVlcTable,
    mv_table: crate::mv::MvTable,
) -> Result<()> {
    use crate::mcbpcy::{decode_mcbpcy_pframe, PFrameMcbpcy};

    let (skip, mb_info) = match decode_mcbpcy_pframe(br)? {
        PFrameMcbpcy::Skip => (true, None),
        PFrameMcbpcy::Coded { decode, ac_pred } => (false, Some((decode, ac_pred))),
    };

    if skip {
        // Skip MB: MV = predictor-based zero (spec/06 §3.4 says
        // missing neighbours are zero-subbed; skipped MBs themselves
        // contribute zero MV too per H.263 convention). Copy the MC
        // prediction at MV=(0,0).
        mv_grid.set_cell(
            mb_x,
            mb_y,
            crate::mv_pred::MvGridCell::OneMv(crate::mv::Mv::default()),
        );
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
        decode_intra_mb_with_header(
            br,
            pic,
            dc_cache,
            &header,
            mb_x,
            mb_y,
            quant,
            dc_size_sel,
            luma_ac,
            chroma_ac,
        )?;
        // Intra MBs clear the MV predictor chain: the per-row mv_grid
        // entry stays `None` so downstream neighbours treat this
        // column as zero.
        return Ok(());
    }

    // Inter MB: decode the joint MV VLC using the median predictor.
    // The `mv_table` selector threads the picture-header
    // `mv_table_sel` bit through to the MV decoder; sel=1 routes to the
    // alternate VLC variant (now fully wired — Extractor 07 / spec/16
    // §1; see `crate::mv::MvTable::Alternate`).
    //
    // The 1-MV-per-MB case (which is the only mode currently shipping
    // for v3) is the Figure 7-34 top-left sub-diagram per
    // `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md` §
    // "When only one motion vector is present for the whole macroblock,
    // the top-left case in Figure 7-34 is applied." The §7.6.5
    // substitution rules then apply (`mv_pred::apply_validity_rules`).
    // Routing through [`mv_pred::predict_block_mv`] replaces the
    // pre-spec `mv::median_predictor` shortcut so the rule-3
    // ("exactly one valid neighbour → promote to all three slots")
    // corner is spec-correct: when only `left` (or only `top`, or only
    // `top_right`) is valid, the predictor is now that neighbour itself
    // instead of `median(neighbour, 0, 0) = 0`. This affects the first
    // inter MB after a row of intra MBs at row 0 (`left` is the sole
    // valid neighbour) and the analogous edge cases.
    let predictor = one_mv_predictor(mv_grid, mb_x, mb_y);
    let mv = crate::mv::decode_mv_with_table(br, predictor, mv_table)?;
    mv_grid.set_cell(mb_x, mb_y, crate::mv_pred::MvGridCell::OneMv(mv));

    // Lay down the motion-compensated prediction first; the residual
    // (decoded below) is added on top of it.
    apply_mc_to_mb(pic, reference, mb_x, mb_y, (mv.x as i32, mv.y as i32));

    decode_inter_residual_blocks(
        br,
        pic,
        mb_x,
        mb_y,
        quant,
        inter_ac,
        decode.cbpy,
        decode.cbp_cb,
        decode.cbp_cr,
    )
}

/// Compute the 1-MV-per-MB §7.6.5 predictor for `(mb_x, mb_y)` from the
/// picture-wide [`crate::mv_pred::MvGrid`].
///
/// Picture-edge-aware neighbour lookup (spec/06 §3.4 +
/// `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md` §
/// "Boundary substitution"): the grid surfaces missing neighbours as
/// [`crate::mv_pred::NeighbourMvKind::Absent`], which threads through
/// the [`crate::mv_pred::BlockCandidates`] fields as `None`; then
/// [`crate::mv_pred::predict_block_mv`] applies the four §7.6.5
/// candidate-validity substitution rules for the Figure 7-34 top-left
/// sub-diagram (the 1-MV-per-MB case). Shared by the v3 P-frame path
/// and the v1/v2 P-frame path — per spec/07 §3.5 the v1/v2 MV decoder
/// body calls the *same* median-of-3 predictor helper (`0x1c217c8c`)
/// as v3, with identical semantics.
fn one_mv_predictor(mv_grid: &crate::mv_pred::MvGrid, mb_x: usize, mb_y: usize) -> crate::mv::Mv {
    let nset = mv_grid.neighbour_set_for(mb_x, mb_y);
    // Build the §7.6.5 candidate set for the 1-MV-per-MB case (Figure
    // 7-34 top-left sub-diagram) through the spec-derived resolver
    // (round 214) rather than hand-picking the neighbour bytes. For a
    // 1-MV neighbour (`OneMv`) the resolver reports its single MV in
    // every bordering direction; for a 4-MV-coded neighbour
    // (`FourMv`) it indexes the neighbour's `[Mv; 4]` by the
    // physically-bordering 8x8 cell per
    // [`crate::mv_pred::bordering_block_of_neighbour`] (round 208,
    // `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`) —
    // i.e. the current MB's block 1 takes its left neighbour's block 2
    // (TR), its above neighbour's block 3 (BL), and its above-right
    // neighbour's block 3 (BL). Previously these three indices were
    // open-coded as `mvs[1]` / `mvs[2]` / `mvs[2]`, duplicating the
    // Figure 7-34 mapping that already lives in the resolver and
    // risking silent drift if that table is ever corrected. Routing
    // through [`crate::mv_pred::resolve_block_candidates`] keeps the
    // picture path provably consistent with the documented mapping.
    // There are no within-MB block dependencies in the 1-MV case, so
    // the within-MB slots are all `None`.
    let cands = crate::mv_pred::resolve_block_candidates(
        crate::mv_pred::Block::TopLeft,
        nset,
        [None, None, None],
    );
    crate::mv_pred::predict_block_mv(crate::mv_pred::Block::TopLeft, &cands)
}

/// Decode the inter AC residual for every CBP-coded block of one MB and
/// add it onto the MC prediction already laid down in `pic`.
///
/// Per spec/04 §1 the coded-block-pattern from the MB-header decode
/// selects which of the 6 blocks carry an AC walk. The four luma CBPY
/// bits are MSB-first (block 0 = bit 3); chroma blocks use the cbp_cb /
/// cbp_cr flags. Each coded block decodes the G4 inter AC VLC (spec/04
/// §1.3), dequantises (spec/08 §3.2, level_start = 0 — DC is a coded
/// coefficient on the inter path), IDCTs to a signed residual, and is
/// added onto the MC prediction. Shared by the v3 and v1/v2 P-frame
/// paths: per spec/99 §6 the v1 inter DCT kernel (`0x1c215d2c`) and the
/// v2/v3 inter kernels consume the same G4 alphabet with a hard-coded
/// zigzag, scan start 0, single-tier ESC, and the uniform H.263
/// Eq. 12 dequant pair (spec/08 §5).
#[allow(clippy::too_many_arguments)]
fn decode_inter_residual_blocks(
    br: &mut BitReader<'_>,
    pic: &mut Picture,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    inter_ac: &AcVlcTable,
    cbpy: u8,
    cbp_cb: bool,
    cbp_cr: bool,
) -> Result<()> {
    for block_idx in 0..6usize {
        let coded = match block_idx {
            0..=3 => cbpy & (1 << (3 - block_idx)) != 0,
            4 => cbp_cb,
            5 => cbp_cr,
            _ => unreachable!(),
        };
        if !coded {
            continue;
        }
        let mut coeffs = [0i32; 64];
        crate::ac::decode_inter_block(br, &mut coeffs, inter_ac, quant)?;
        let mut residual = [0i32; 64];
        idct8x8_to_pel(&coeffs, &mut residual);
        add_residual_to_picture(pic, mb_x, mb_y, block_idx, &residual);
    }
    Ok(())
}

// ==================== v1 / v2 picture decode ====================

/// Version selector for [`decode_picture_v1v2`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MsV1V2Version {
    /// MS-MPEG4 v1 (`MP41` / `MPG4`): 37-bit opaque picture-header
    /// preamble + P-frame UMV flag (spec/01 §1.4), separate 21-entry
    /// MCBPC VLC (spec/07 §1), no AC-prediction bit anywhere
    /// (spec/07 §1.4).
    V1,
    /// MS-MPEG4 v2 (`MP42`): no preamble, separate 8-entry MCBPC VLC
    /// (spec/07 §2), P-frame-gated skip bit, AC-prediction bit on
    /// intra-in-P MBs only (spec/07 §2.4).
    V2,
}

/// Decode one MS-MPEG4 v1 / v2 picture (I-frame or P-frame).
///
/// Wires the I-frame intra pipeline and the P-frame **skip + inter +
/// intra-in-P** macroblock pixel pipeline end-to-end:
///
///   1. Picture header via [`MsV1V2PictureHeader::parse_v1`] /
///      [`parse_v2`](MsV1V2PictureHeader::parse_v2) (spec/01 §1.4).
///      The v1 P-frame UMV flag is consumed as framing but not
///      branched on — per spec/07 §3.4 the v<4 MV decoder body does
///      not branch on it.
///   2. **I-frames** ([`decode_iframe_v1v2`]): every MB is intra. The
///      MB header is the v1/v2 MCBPCY (separate MCBPC + CBPY VLCs,
///      spec/07 §1-§2); each of the 6 blocks decodes through the shared
///      spatial DC predictor + the v1/v2 size-category DC differential
///      ([`crate::mb::decode_intra_dc_diff_v1v2`], spec/16 §2) + the
///      intra AC walk (luma G5, chroma G4 per spec/14 §3.2).
///   3. **P-frames**: per-MB loop (raster order, single slice per
///      spec/01 §3). Skipped MBs copy the reference at MV = (0, 0);
///      coded inter MBs decode the per-component MV pair
///      ([`crate::mv::decode_mv_v1v2`], shared 65-entry table at VMA
///      `0x1c24f930`, spec/07 §3) against the same §7.6.5 1-MV
///      predictor as v3 (same helper `0x1c217c8c` per spec/07 §3.5),
///      apply half-pel MC, then add the G4 inter AC residual for every
///      CBP-coded block — per spec/14 §3.1 the v1/v2 fallthrough at
///      `0x1c212917` pins the inter/chroma DCT descriptor to G4, and
///      per spec/99 §6 the inter kernels share the hard-zigzag /
///      scan-start-0 / single-tier-ESC shape across v1/v2/v3.
///      Intra-in-P MBs reuse the same intra path as I-frame MBs.
///
/// **v1/v2 intra DC rule (spec/16 §2, Extractor 07).** The intra-block
/// driver gates on version (`cmp [esi+8], 3`): for v < 3 it decodes the
/// DC differential through the classic H.263 size+value scheme
/// (`sub_15790`) using the binary's own luma/chroma size tables
/// (`region_054{2,3}c0`, VMAs `0x1c2542c0` / `0x1c2543c0`) — **not** the
/// v3 direct-value DC VLC and **not** the v3 `dc_size_sel` selector. The
/// previous gate cited an untraced construction-time default of the v3
/// `[esi+0x8bc]` selector; spec/16 §2 establishes that selector is never
/// consulted on the v1/v2 path, so the gate is dissolved. The spatial
/// DC-predictor gradient routine `0x1c20aef0` is shared with v3 (no
/// version gate, spec/99 §4.4), and the AC kernel `0x1c216d97` is the
/// common v1/v2/v3 intra kernel (spec/04 §2.6).
///
/// * **v1 inter MB sub-types** — wired. `spec/16` §3.1 +
///   `region_053140_mbtype.csv` pin the P-frame MB-type → MV-count map
///   {1, 1, 4, 0, 0}: type 0 (INTER) and type 1 (INTER+Q) are 1-MV
///   (the v1 MCBPCY body reads no quantiser-delta bit per spec/07
///   §1.4), and type 2 (INTER4V) loops the per-component MV decoder 4×
///   over the Figure 6-8 8x8 blocks. Types 3/4 are intra (handled by
///   the intra-in-P path).
pub fn decode_picture_v1v2(
    br: &mut BitReader<'_>,
    dims: PictureDims,
    version: MsV1V2Version,
    reference: Option<&Picture>,
) -> Result<Picture> {
    let codec = match version {
        MsV1V2Version::V1 => "msmpeg4v1",
        MsV1V2Version::V2 => "msmpeg4v2",
    };
    let hdr = match version {
        MsV1V2Version::V1 => MsV1V2PictureHeader::parse_v1(br)?,
        MsV1V2Version::V2 => MsV1V2PictureHeader::parse_v2(br)?,
    };
    match hdr.picture_type {
        PictureType::I => decode_iframe_v1v2(br, dims, &hdr, version),
        PictureType::P => {
            let reference = reference.ok_or_else(|| {
                Error::invalid(format!(
                    "{codec}: P-frame decode requires a reference picture \
                     (decoder state is missing the previous frame). Make \
                     sure to feed packets in decode order starting with an \
                     I-frame.",
                ))
            })?;
            decode_pframe_v1v2(br, dims, &hdr, version, codec, reference)
        }
    }
}

/// Decode a full v1/v2 P-frame into a [`Picture`], using the supplied
/// `reference` for motion compensation. See [`decode_picture_v1v2`]
/// for the pipeline shape and the documented gates.
fn decode_pframe_v1v2(
    br: &mut BitReader<'_>,
    dims: PictureDims,
    hdr: &MsV1V2PictureHeader,
    version: MsV1V2Version,
    codec: &'static str,
    reference: &Picture,
) -> Result<Picture> {
    if reference.width != dims.width || reference.height != dims.height {
        return Err(Error::invalid(format!(
            "{codec}: P-frame reference dimensions {}x{} differ from \
             current {}x{}",
            reference.width, reference.height, dims.width, dims.height,
        )));
    }

    let (mb_w, mb_h) = dims.mb_dims();
    let mut pic = Picture::alloc(dims, PictureType::P);
    let quant = hdr.quant as u32;
    // Per spec/14 §3.1 the v1/v2 fallthrough at `0x1c212917` binds the
    // inter/chroma DCT descriptor to G4 unconditionally — there is no
    // per-frame AC selector in the v1/v2 picture header (spec/01 §1.4:
    // the selector reads gate on `version == 3`).
    let inter_ac = crate::ac::AcVlcTable::g4_inter();
    // Intra-in-P AC tables (spec/14 §3.2): v1/v2 default luma DCT = G5,
    // chroma = G4. Same descriptors as the v1/v2 I-frame path.
    let intra_luma_ac = AcVlcTable::v3_intra_g5();
    let intra_chroma_ac = AcVlcTable::g4_inter();
    let mut mv_grid = crate::mv_pred::MvGrid::new(mb_w, mb_h);
    // DC prediction cache for the intra-in-P MBs (the spatial DC
    // predictor is shared with the I-frame path / v3 per spec/99 §4.4).
    let mut dc_cache = DcCache::new(mb_w, mb_h);

    for my in 0..mb_h {
        for mx in 0..mb_w {
            decode_pframe_mb_v1v2(
                br,
                &mut pic,
                &mut mv_grid,
                &mut dc_cache,
                reference,
                mx,
                my,
                quant,
                version,
                codec,
                &inter_ac,
                &intra_luma_ac,
                &intra_chroma_ac,
            )?;
        }
    }

    Ok(pic)
}

/// Decode one v1/v2 P-frame MB: skip bit, separate MCBPC / CBPY VLCs,
/// then (for a plain inter MB) per-component MV decode, half-pel MC,
/// and the G4 inter residual. See [`decode_picture_v1v2`] for the
/// documented gates on intra-in-P and the v1 non-zero sub-types.
#[allow(clippy::too_many_arguments)]
fn decode_pframe_mb_v1v2(
    br: &mut BitReader<'_>,
    pic: &mut Picture,
    mv_grid: &mut crate::mv_pred::MvGrid,
    dc_cache: &mut DcCache,
    reference: &Picture,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    version: MsV1V2Version,
    codec: &'static str,
    inter_ac: &AcVlcTable,
    intra_luma_ac: &AcVlcTable,
    intra_chroma_ac: &AcVlcTable,
) -> Result<()> {
    use crate::mcbpcy::{decode_mcbpcy_v1, decode_mcbpcy_v2, V2FrameType};

    let decode = match version {
        MsV1V2Version::V1 => decode_mcbpcy_v1(br)?,
        MsV1V2Version::V2 => decode_mcbpcy_v2(br, V2FrameType::P)?,
    };

    if decode.skip {
        // Skipped MB: copy the MC prediction at MV = (0, 0). The MV
        // grid records a zero MV so downstream neighbour predictors
        // see this column as a valid (0, 0) candidate — same
        // convention as the v3 skip path.
        mv_grid.set_cell(
            mb_x,
            mb_y,
            crate::mv_pred::MvGridCell::OneMv(crate::mv::Mv::default()),
        );
        apply_mc_to_mb(pic, reference, mb_x, mb_y, (0, 0));
        return Ok(());
    }

    if decode.is_intra {
        // Intra-in-P: decode the intra MB through the v1/v2 size-category
        // DC path (spec/16 §2) + the shared spatial DC predictor. The
        // previous `[esi+0x8bc]` gate was dissolved by spec/16 §2: v1/v2
        // do not use the v3 selector at all — they bind the dedicated
        // size-category DC tables (`region_054{2,3}c0`).
        let header = IntraMbHeader {
            ac_pred: decode.ac_pred,
            cbpy: decode.cbpy,
            cbp_cb: decode.cbp_cb,
            cbp_cr: decode.cbp_cr,
        };
        decode_intra_mb_v1v2_to_picture(
            br,
            pic,
            dc_cache,
            &header,
            mb_x,
            mb_y,
            quant,
            intra_luma_ac,
            intra_chroma_ac,
        )?;
        // Intra MBs clear the MV predictor chain: leave the mv_grid cell
        // `Absent` so downstream neighbours treat this column as zero
        // (same convention as the v3 intra-in-P path).
        return Ok(());
    }

    // Per `spec/16` §3.1 + `region_053140_mbtype.csv` the v1 P-frame
    // MB-type (= mcbpc >> 2) selects the motion mode with traced
    // motion-vector counts {1, 1, 4, 0, 0}:
    //   * MB-type 0 (INTER)    — 1 MV.
    //   * MB-type 1 (INTER+Q)  — 1 MV. spec/07 §1.4 confirms the v1
    //     MCBPCY body reads NO post-VLC bit (no `call 0x1c215c9b`
    //     after CBPY), i.e. there is no quantiser-delta read; the
    //     "+Q" is the H.263-Table-8 lineage name only, and the trace
    //     decodes it exactly like MB-type 0.
    //   * MB-type 2 (INTER4V)  — 4 MVs, one per Figure 6-8 8x8 block;
    //     spec/16 §3.1 says the per-component MV decoder loops 4×.
    // MB-types 3/4 are intra and are handled by the `is_intra` branch
    // above. The v2 8-symbol MCBPC alphabet only emits mb_type ∈
    // {0, 3} (quotient 0/1), so this dispatch is reached with
    // mb_type ∈ {0, 1, 2} on the inter path.
    //
    // The 1-MV vs 4-MV decision is driven by `decode.num_motion_vectors`
    // (sourced from `MB_TYPE_V1_INFO` per spec/16 §3.1's {1, 1, 4, 0, 0}
    // map) rather than re-deriving it from `mb_type`: that keeps the
    // dispatch grounded in the extracted table and the two stay in lock
    // step by construction.
    match decode.num_motion_vectors {
        1 => {
            // 1-MV inter: §7.6.5 median-of-3 predictor (same helper as
            // v3 per spec/07 §3.5), two separate component reads against
            // the shared 65-entry table (spec/07 §3.2), bias subtract +
            // toroidal wrap inside `decode_mv_v1v2`.
            let predictor = one_mv_predictor(mv_grid, mb_x, mb_y);
            let mv = crate::mv::decode_mv_v1v2(br, predictor)?;
            mv_grid.set_cell(mb_x, mb_y, crate::mv_pred::MvGridCell::OneMv(mv));
            apply_mc_to_mb(pic, reference, mb_x, mb_y, (mv.x as i32, mv.y as i32));
        }
        4 => {
            // INTER4V: decode one MVD per Figure 6-8 block in raster
            // order, threading each block's *final* MV back into the
            // Figure-7-34 within-MB predictor via the
            // `Macroblock4MvDecoderNeighbours` driver (handles a mix of
            // 1-MV / 4-MV / absent neighbours per spec §7.6.5).
            use crate::mv_pred::{Block, Macroblock4MvDecoderNeighbours};
            let nset = mv_grid.neighbour_set_for(mb_x, mb_y);
            let mut dec = Macroblock4MvDecoderNeighbours::new(nset);
            let mut block_mvs = [crate::mv::Mv::default(); 4];
            for (i, &block) in Block::ALL.iter().enumerate() {
                let predictor = dec.predictor_for(block);
                let mv = crate::mv::decode_mv_v1v2(br, predictor)?;
                dec.commit_block(block, mv);
                block_mvs[i] = mv;
            }
            mv_grid.set_cell(mb_x, mb_y, dec.finalise_to_grid_cell());
            let mvs_half = [
                (block_mvs[0].x as i32, block_mvs[0].y as i32),
                (block_mvs[1].x as i32, block_mvs[1].y as i32),
                (block_mvs[2].x as i32, block_mvs[2].y as i32),
                (block_mvs[3].x as i32, block_mvs[3].y as i32),
            ];
            apply_mc_4mv_to_mb(pic, reference, mb_x, mb_y, mvs_half);
        }
        other => {
            return Err(Error::invalid(format!(
                "{codec}: inter MB-type {} at ({mb_x}, {mb_y}) implies \
                 {other} motion vectors, outside the traced map \
                 {{1, 4}} (spec/16 §3.1); intra types 3/4 take the \
                 is_intra path.",
                decode.mb_type,
            )));
        }
    }

    decode_inter_residual_blocks(
        br,
        pic,
        mb_x,
        mb_y,
        quant,
        inter_ac,
        decode.cbpy,
        decode.cbp_cb,
        decode.cbp_cr,
    )
}

/// Decode a full MS-MPEG4 v1/v2 I-frame into a [`Picture`].
///
/// Per spec/16 §2 (Extractor 07) the v1/v2 intra path is the classic
/// H.263 size+value DC scheme plus the spatial DC predictor — both
/// shared with v3 except for the DC-differential decoder. Every MB in an
/// I-frame is intra (the MCBPC decode yields `is_intra == true`), so the
/// per-MB loop decodes the v1/v2 MCBPCY (separate MCBPC + CBPY VLCs,
/// spec/07 §1-§2), then the 6 blocks through the shared spatial-DC
/// predictor + [`crate::mb::decode_intra_block_full_v1v2`] +
/// [`crate::idct`]. Per spec/14 §3.2 the v1/v2 default luma DCT
/// descriptor is G5 and chroma/inter is G4; both carry their full
/// packed-Huffman primary VLC, so coded AC blocks decode end-to-end.
///
/// This closes the v1/v2 I-frame gate that previously surfaced as
/// `Unsupported`: the gate cited the untraced construction-time default
/// of the v3 `[esi+0x8bc]` selector, but spec/16 §2 establishes that
/// v1/v2 do not use that selector at all — they use the dedicated
/// size-category DC tables (`region_054{2,3}c0`), so no `[esi+0x8bc]`
/// default needs to be traced.
fn decode_iframe_v1v2(
    br: &mut BitReader<'_>,
    dims: PictureDims,
    hdr: &MsV1V2PictureHeader,
    version: MsV1V2Version,
) -> Result<Picture> {
    use crate::mcbpcy::{decode_mcbpcy_v1, decode_mcbpcy_v2, V2FrameType};

    let (mb_w, mb_h) = dims.mb_dims();
    let mut pic = Picture::alloc(dims, PictureType::I);
    let mut dc_cache = DcCache::new(mb_w, mb_h);
    let quant = hdr.quant as u32;
    // Spec/14 §3.2: v1/v2 default luma DCT descriptor = G5, chroma = G4.
    // Neither has a per-frame selector in v1/v2 (the AC-selector reads
    // gate on version == 3 per spec/01 §1.4).
    let luma_ac = AcVlcTable::v3_intra_g5();
    let chroma_ac = AcVlcTable::g4_inter();

    for my in 0..mb_h {
        for mx in 0..mb_w {
            // Decode the v1/v2 MB header. On an I-frame every MB is intra
            // (the MCBPC alphabet's intra MB-types); v1 still reads its
            // leading COD bit unconditionally (spec/07 §1.5 — it is 0 on
            // I-frames), v2 reads no skip bit in I-frame mode.
            let decode = match version {
                MsV1V2Version::V1 => decode_mcbpcy_v1(br)?,
                MsV1V2Version::V2 => decode_mcbpcy_v2(br, V2FrameType::I)?,
            };
            if !decode.is_intra {
                return Err(Error::invalid(format!(
                    "msmpeg4 v1/v2 I-frame: MB ({mx}, {my}) decoded a \
                     non-intra MB-type {} — every MB in an I-frame must be \
                     intra (spec/16 §3, MCBPC intra MB-types). The \
                     bitstream is corrupt or mis-framed.",
                    decode.mb_type,
                )));
            }
            let header = IntraMbHeader {
                ac_pred: decode.ac_pred,
                cbpy: decode.cbpy,
                cbp_cb: decode.cbp_cb,
                cbp_cr: decode.cbp_cr,
            };
            decode_intra_mb_v1v2_to_picture(
                br,
                &mut pic,
                &mut dc_cache,
                &header,
                mx,
                my,
                quant,
                &luma_ac,
                &chroma_ac,
            )?;
        }
    }

    Ok(pic)
}

/// Decode one v1/v2 intra macroblock's 6 blocks into `pic` using the
/// shared spatial DC predictor and the v1/v2 size-category DC decoder
/// ([`crate::mb::decode_intra_block_full_v1v2`], spec/16 §2). Shared by
/// the v1/v2 I-frame path and the intra-in-P path. Mirrors the v3
/// [`decode_intra_mb_with_header`] exactly except for the DC decoder.
#[allow(clippy::too_many_arguments)]
fn decode_intra_mb_v1v2_to_picture(
    br: &mut BitReader<'_>,
    pic: &mut Picture,
    dc_cache: &mut DcCache,
    header: &IntraMbHeader,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    luma_ac: &AcVlcTable,
    chroma_ac: &AcVlcTable,
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
        let ac_table = if block_idx <= 3 { luma_ac } else { chroma_ac };
        let block_result = if cbp_set && ac_table.entries.is_empty() {
            // No real AC table available: DC-only reconstruction (the AC
            // bits would misalign on real content, but this keeps
            // synthetic DC-only streams decodable, matching the v3 path).
            let dc_diff = crate::mb::decode_intra_dc_diff_v1v2(br, block_idx)?;
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
            crate::mb::decode_intra_block_full_v1v2(
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

/// Add a signed 8×8 IDCT residual onto the existing (motion-compensated)
/// pel values of the picture at the given block position, clamping the
/// sum to `[0, 255]`. Inter blocks reconstruct as `pred + residual`
/// (spec/04 §2.6: the inter IDCT output is a signed residual added to
/// the MC prediction, in contrast to the intra path where the IDCT
/// output is the final pel value).
fn add_residual_to_picture(
    pic: &mut Picture,
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
    residual: &[i32; 64],
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
                        let sum = pic.y[off] as i32 + residual[j * 8 + i];
                        pic.y[off] = sum.clamp(0, 255) as u8;
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
                        let sum = plane[off] as i32 + residual[j * 8 + i];
                        plane[off] = sum.clamp(0, 255) as u8;
                    }
                }
            }
        }
        _ => unreachable!(),
    }
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

/// Apply INTER4V (MB-type 2) motion compensation to the (mb_x, mb_y) MB:
/// each of the four 8x8 luma blocks uses its own half-pel MV in Figure
/// 6-8 raster order, and both chroma blocks use the §7.6.3.4-derived
/// shared MV. The four MVs are supplied as `(x, y)` half-pel pairs.
fn apply_mc_4mv_to_mb(
    pic: &mut Picture,
    reference: &Picture,
    mb_x: usize,
    mb_y: usize,
    block_mvs_half: [(i32, i32); 4],
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
    crate::mc::mc_macroblock_4mv(
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
        block_mvs_half,
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
    dc_size_sel: u8,
    luma_ac: &AcVlcTable,
    chroma_ac: &AcVlcTable,
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
        let ac_table = if block_idx <= 3 { luma_ac } else { chroma_ac };
        let block_result = if cbp_set && ac_table.entries.is_empty() {
            let dc_diff = crate::mb::decode_intra_dc_diff_v3(br, block_idx, dc_size_sel)?;
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
            decode_intra_block_full_v3(
                br,
                block_idx,
                pred.predictor,
                quant,
                cbp_set,
                scan,
                ac_table,
                dc_size_sel,
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
#[allow(clippy::too_many_arguments)]
fn decode_intra_mb_to_picture(
    br: &mut BitReader<'_>,
    pic: &mut Picture,
    dc_cache: &mut DcCache,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    dc_size_sel: u8,
    luma_ac: &AcVlcTable,
    chroma_ac: &AcVlcTable,
) -> Result<()> {
    let trace = std::env::var_os("OXIDEAV_MSMPEG4_AC_TRACE").is_some();
    let mb_start = br.bit_position();
    let header = IntraMbHeader::parse_v3_mcbpcy(br)?;
    if trace {
        eprintln!(
            "[mb trace] mb=({mb_x},{mb_y}) start_bit={mb_start} cbpy=0x{:x} cbp_cb={} cbp_cr={} ac_pred={} after_hdr={}",
            header.cbpy, header.cbp_cb, header.cbp_cr, header.ac_pred, br.bit_position(),
        );
    }

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

        // Per-block AC VLC routing: luma blocks (0..=3) use the
        // intra-luma table (G5 by default); chroma blocks (4, 5) use
        // the chroma + all-inter table (G4 by default), per spec/99
        // §5.2.
        let ac_table = if block_idx <= 3 { luma_ac } else { chroma_ac };

        // AC path: when the placeholder table is in use (entries empty)
        // fall back to DC-only — the bitstream's AC bits are skipped,
        // which means subsequent block reads will misalign on real
        // content but the decoder continues for synthetic DC-only
        // streams. When a real (or candidate) AC VLC table is plugged
        // in, the regular `decode_intra_block_full` path runs.
        let bit_before = br.bit_position();
        let block_result = if cbp_set && ac_table.entries.is_empty() {
            let dc_diff = crate::mb::decode_intra_dc_diff_v3(br, block_idx, dc_size_sel)?;
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
            decode_intra_block_full_v3(
                br,
                block_idx,
                pred.predictor,
                quant,
                cbp_set,
                scan,
                ac_table,
                dc_size_sel,
            )?
        };
        if trace {
            eprintln!(
                "[blk trace] mb=({mb_x},{mb_y}) blk={block_idx} cbp={cbp_set} scan={:?} bits=[{bit_before}..{}] dc={} nz={}",
                scan,
                br.bit_position(),
                block_result.coeffs[0],
                block_result.ac_nonzero,
            );
        }

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

    /// Round 32 piece 3: an all-skip P-frame with `mv_table_sel = 1`
    /// must NOT error at the picture-header dispatch (the previous
    /// behaviour) — every MB is skipped so the alternate-MV path is
    /// never reached and the picture decodes normally. This pins the
    /// "fail at first inter MB, not at picture entry" semantic that
    /// piece 3 wires.
    #[test]
    fn pframe_alternate_mv_table_with_all_skip_decodes() {
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
        // P-frame with mv_table_sel = 1, single MB skipped.
        let fields: Vec<(u32, u32)> = vec![
            (1, 2), // P
            (8, 5), // quant
            (0, 1), // ac_chroma_sel
            (0, 1), // dc_size_sel
            (1, 1), // mv_table_sel = 1 (alternate)
            (1, 1), // skip = 1 → no MV decode for this MB
            (0, 16),
        ];
        let bytes = pack(&fields);
        let mut br = BitReader::new(&bytes);
        // Should succeed: alternate-MV path is never reached.
        decode_picture(&mut br, dims, Some(&reference))
            .expect("all-skip P-frame with mv_table_sel=1 must decode (alt MV not exercised)");
    }

    /// Round 326: when `mv_table_sel = 1` AND the P-frame has at least
    /// one inter (non-skipped) MB, the alternate MV VLC now decodes
    /// end-to-end (Extractor 07 / spec/16 §1). A single inter MB whose
    /// alternate-MV symbol is index 0 (canonical 2-bit code `00`, alt
    /// byte LUT index 0 = `0x20`/`0x20` → MV = (0, 0) after the bias
    /// subtraction) with a zero predictor produces an identity MC copy,
    /// so the decoded picture equals the reference. This pins that the
    /// previously-`Unsupported` alternate path is wired and selects the
    /// alternate table rather than the default.
    #[test]
    fn pframe_alternate_mv_table_inter_mb_decodes() {
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

        // Reuse the inter-MB MCBPCY symbol-0 build from
        // `handcrafted_inter_mb_copies_reference` — the canonical
        // MCBPCY bit-length / code for sym 0 (CBP=0, inter).
        let t_mcbpcy = {
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
        let (bl_sym0, code_sym0) = t_mcbpcy
            .iter()
            .find(|(_, _, sym)| *sym == 0)
            .map(|(bl, code, _)| (*bl as u32, *code))
            .expect("symbol 0 in canonical MCBPCY");

        // Alternate MV VLC symbol 0 canonical code is the 2-bit `00`
        // (the shortest code; alt bit-lengths start at 2). Encoding two
        // copies of it gives MVDx symbol 0 and MVDy is part of the same
        // joint symbol — the joint table emits one symbol carrying both
        // components, so a single 2-bit `00` read suffices.
        let dims = PictureDims::new(16, 16).unwrap();
        let mut reference = Picture::alloc(dims, PictureType::I);
        // Give the reference a non-trivial pattern so an identity copy
        // is a meaningful equality check (an all-zero copy would pass
        // trivially against an all-zero reference).
        for (i, px) in reference.y.iter_mut().enumerate() {
            *px = (i % 251) as u8;
        }
        for (i, px) in reference.cb.iter_mut().enumerate() {
            *px = (i % 199) as u8;
        }
        for (i, px) in reference.cr.iter_mut().enumerate() {
            *px = (i % 197) as u8;
        }

        // P-frame: mv_table_sel = 1, single inter MB (no skip), alt MV
        // joint symbol 0 (code `00`).
        let fields: Vec<(u32, u32)> = vec![
            (1, 2),               // P
            (8, 5),               // quant
            (0, 1),               // ac_chroma_sel
            (0, 1),               // dc_size_sel
            (1, 1),               // mv_table_sel = 1 (alternate)
            (0, 1),               // skip = 0 (reaches MV decode)
            (code_sym0, bl_sym0), // MCBPCY sym 0 (inter, CBP=0)
            (0, 1),               // ac_pred (ignored for inter)
            (0b00, 2),            // alt MV joint symbol 0 → MV (0, 0)
            (0, 16),              // trailing padding
        ];
        let bytes = pack(&fields);
        let mut br = BitReader::new(&bytes);
        let pic = decode_picture(&mut br, dims, Some(&reference))
            .expect("inter P-frame MB with mv_table_sel=1 must decode (alt MV wired)");
        // MV = (0, 0) → identity MC copy → decoded picture == reference.
        assert_eq!(pic.y, reference.y, "alt-MV (0,0) luma must copy reference");
        assert_eq!(pic.cb, reference.cb, "alt-MV (0,0) Cb must copy reference");
        assert_eq!(pic.cr, reference.cr, "alt-MV (0,0) Cr must copy reference");
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

    /// Hand-crafted P-frame with a single non-skipped inter MB whose
    /// luma block 0 is CBP-coded and carries a G4 inter residual. With
    /// MV=(0,0) the MC prediction equals the reference; the decoded
    /// residual must then be *added* on top, so the output luma must
    /// differ from a pure MC copy precisely within block 0's 8×8 region
    /// and be identical everywhere else. Exercises the round-123 inter
    /// residual path end-to-end (G4 VLC walk + dequant + IDCT + add).
    #[test]
    fn handcrafted_inter_mb_applies_residual() {
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

        // Canonical MCBPCY table (mirrors mcbpcy.rs).
        let t_mcbpcy = {
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

        // We want an inter MB (bit 6 of idx clear) with luma block 0
        // coded: cbpy bit 3 set ⇒ cbpy = 0b1000 = 8 ⇒ pattern = 8 << 2
        // = 32 ⇒ idx = 32 (chroma + Y1..Y3 uncoded). Find its canonical
        // code.
        let target_idx: u8 = 32;
        let (bl_mb, code_mb) = t_mcbpcy
            .iter()
            .find(|(_, _, sym)| *sym == target_idx)
            .map(|(bl, code, _)| (*bl as u32, *code))
            .expect("MCBPCY symbol 32 (inter, Y0-coded) in canonical table");

        // G4 inter residual: pick the shortest sub-class-B (last=1)
        // terminator. The single token writes its level at zigzag
        // position `run`, then the block ends.
        let g4 = crate::ac::AcVlcTable::g4_inter();
        let term = g4
            .entries
            .iter()
            .filter_map(|e| match e.value {
                crate::ac::Symbol::RunLevel {
                    last: true,
                    run,
                    level,
                } => Some((e, run, level)),
                _ => None,
            })
            .filter(|(_, _, level)| *level != 0)
            .min_by_key(|(e, _, _)| e.bits)
            .expect("G4 sub-class-B terminator with non-zero level");
        let (term_entry, _term_run, _term_level) = term;

        let dims = PictureDims::new(16, 16).unwrap();
        let mut reference = Picture::alloc(dims, PictureType::I);
        // Flat mid-grey reference so the residual is the only source of
        // variation in the output.
        for p in reference.y.iter_mut() {
            *p = 128;
        }
        for p in reference.cb.iter_mut() {
            *p = 128;
        }
        for p in reference.cr.iter_mut() {
            *p = 128;
        }

        // P-frame header + one MB.
        let mut fields: Vec<(u32, u32)> = vec![
            (1, 2), // P
            (8, 5), // quant
            (0, 1), // ac_chroma_sel = 0
            (0, 1), // dc_size_sel = 0
            (0, 1), // mv_table_sel = 0
        ];
        fields.push((0, 1)); // skip = 0
        fields.push((code_mb, bl_mb)); // MCBPCY idx 32 (inter, Y0 coded)
        fields.push((0, 1)); // ac_pred (ignored for inter)
        fields.push((0, 1)); // MV sym 0 → MV (0,0)
                             // Luma block 0 residual: single sub-class-B terminator + sign 0.
        fields.push((term_entry.code, term_entry.bits as u32));
        fields.push((0, 1)); // sign = positive
        fields.push((0, 16)); // tail padding
        let bytes = pack(&fields);
        let mut br = BitReader::new(&bytes);
        let pic = decode_picture(&mut br, dims, Some(&reference))
            .expect("inter-residual P-frame must decode");

        assert_eq!(pic.picture_type, PictureType::P);

        // Block 0 occupies luma rows 0..8, cols 0..8. The residual must
        // have changed at least one pel there (it is added onto the
        // flat-128 prediction, then clamped).
        let mut block0_changed = false;
        for j in 0..8usize {
            for i in 0..8usize {
                if pic.y[j * pic.y_stride + i] != 128 {
                    block0_changed = true;
                }
            }
        }
        assert!(
            block0_changed,
            "inter residual must modify luma block 0 (was a flat MC copy)"
        );

        // Everything outside block 0 must remain the MC copy (128):
        // Y1..Y3 + chroma were not CBP-coded, so no residual there.
        for j in 0..16usize {
            for i in 0..16usize {
                if j < 8 && i < 8 {
                    continue; // block 0 — allowed to change
                }
                assert_eq!(
                    pic.y[j * pic.y_stride + i],
                    128,
                    "uncoded luma pel ({j},{i}) must stay the MC copy"
                );
            }
        }
        for &v in &pic.cb {
            assert_eq!(v, 128, "uncoded Cb must stay the MC copy");
        }
        for &v in &pic.cr {
            assert_eq!(v, 128, "uncoded Cr must stay the MC copy");
        }
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

    /// Round 240 (2026-06-06) regression pin: the MV-grid neighbour
    /// resolution that [`decode_pframe_mb`] now performs through
    /// [`crate::mv_pred::MvGrid::neighbour_set_for`] must produce the
    /// same `(left, top, top_right)` triple that the pre-round-240
    /// `Vec<Option<Mv>>` raster-index arithmetic produced for every MB
    /// position on a small grid populated with both `OneMv` and
    /// `Absent` cells.
    ///
    /// This exercises the
    /// `MvGridCell::OneMv → NeighbourMvKind::OneMv → BlockCandidates.left_mb`
    /// chain (and the analogous `Absent → None` chain for intra-in-P
    /// MBs and picture-edge substitution) at the same fan-out the
    /// pframe MB loop uses, so the wiring is anchored against the
    /// historical baseline without re-coding the pre-round-240 path.
    #[test]
    fn round_240_mv_grid_neighbour_lookup_matches_legacy_arithmetic() {
        use crate::mv::Mv;
        use crate::mv_pred::{MvGrid, MvGridCell, NeighbourMvKind};

        // 4x3 grid with a mix of OneMv (inter / skip MBs) and Absent
        // (intra-in-P MBs at column 1 throughout, plus the all-Absent
        // row 0 first cell to pin the picture-corner case).
        let mb_w: usize = 4;
        let mb_h: usize = 3;
        let mut legacy: Vec<Option<Mv>> = vec![None; mb_w * mb_h];
        let mut grid = MvGrid::new(mb_w, mb_h);
        for y in 0..mb_h {
            for x in 0..mb_w {
                // Column 1 is intra-in-P (Absent in both views).
                if x == 1 {
                    continue;
                }
                // Skip the picture-corner (0,0) — leave it Absent to
                // pin the corner substitution.
                if x == 0 && y == 0 {
                    continue;
                }
                let mv = Mv {
                    x: (x as i8) - 1,
                    y: (y as i8) + 2,
                };
                legacy[y * mb_w + x] = Some(mv);
                grid.set_cell(x, y, MvGridCell::OneMv(mv));
            }
        }

        // For every MB position in raster order, the new MvGrid route
        // must agree with the legacy `mv_grid[idx]` lookups on the
        // three Figure 7-34 neighbour positions.
        for my in 0..mb_h {
            for mx in 0..mb_w {
                let legacy_left = if mx > 0 {
                    legacy[my * mb_w + (mx - 1)]
                } else {
                    None
                };
                let legacy_top = if my > 0 {
                    legacy[(my - 1) * mb_w + mx]
                } else {
                    None
                };
                let legacy_top_right = if my > 0 && mx + 1 < mb_w {
                    legacy[(my - 1) * mb_w + (mx + 1)]
                } else {
                    None
                };

                let nset = grid.neighbour_set_for(mx, my);
                let grid_left = match nset.left {
                    NeighbourMvKind::Absent => None,
                    NeighbourMvKind::OneMv(mv) => Some(mv),
                    NeighbourMvKind::FourMv(mvs) => Some(mvs[1]),
                };
                let grid_top = match nset.above {
                    NeighbourMvKind::Absent => None,
                    NeighbourMvKind::OneMv(mv) => Some(mv),
                    NeighbourMvKind::FourMv(mvs) => Some(mvs[2]),
                };
                let grid_top_right = match nset.above_right {
                    NeighbourMvKind::Absent => None,
                    NeighbourMvKind::OneMv(mv) => Some(mv),
                    NeighbourMvKind::FourMv(mvs) => Some(mvs[2]),
                };

                assert_eq!(grid_left, legacy_left, "left mismatch at ({mx},{my})");
                assert_eq!(grid_top, legacy_top, "top mismatch at ({mx},{my})");
                assert_eq!(
                    grid_top_right, legacy_top_right,
                    "top_right mismatch at ({mx},{my})"
                );
            }
        }
    }

    /// Round 240 (2026-06-06): the per-MB MV grid pipeline records a
    /// skipped MB as [`crate::mv_pred::MvGridCell::OneMv(Mv::default())`]
    /// so its downstream neighbour position contributes a literal `(0, 0)`
    /// MV to the median predictor of the next MB — equivalent to the
    /// pre-round-240 `Some(Mv::default())` write into the parallel
    /// `Vec<Option<Mv>>` book-keeping.
    ///
    /// This pins the skip-MB cell semantics so a refactor that
    /// accidentally writes `Absent` (which would degrade the right
    /// neighbour to a `None` and trigger the §7.6.5 zero-substitution
    /// rule a second time, only changing behaviour at picture corners
    /// where the cumulative substitution already lands on zero anyway)
    /// is caught.
    #[test]
    fn round_240_skip_mb_cell_is_one_mv_default() {
        use crate::mv::Mv;
        use crate::mv_pred::{MvGrid, MvGridCell, NeighbourMvKind};

        let mut grid = MvGrid::new(3, 3);
        // Write a skip-MB cell at (0, 0) the same way `decode_pframe_mb`
        // does, then confirm cell + neighbour resolution.
        grid.set_cell(0, 0, MvGridCell::OneMv(Mv::default()));
        assert_eq!(grid.cell_at(0, 0), MvGridCell::OneMv(Mv::default()));

        // MB (1, 0) sees (0, 0) as its `left` neighbour — it must
        // surface as a literal `OneMv((0, 0))`, not `Absent`.
        let set = grid.neighbour_set_for(1, 0);
        assert_eq!(set.left, NeighbourMvKind::OneMv(Mv::default()));
        // `above` / `above_right` are out of bounds on row 0 — both
        // Absent regardless of the skip-MB write.
        assert_eq!(set.above, NeighbourMvKind::Absent);
        assert_eq!(set.above_right, NeighbourMvKind::Absent);
    }

    #[test]
    fn round_306_one_mv_predictor_unchanged_for_one_mv_neighbours() {
        // Routing `one_mv_predictor` through
        // `mv_pred::resolve_block_candidates` (replacing the open-coded
        // FourMv index match) must not perturb the all-`OneMv` /
        // `Absent` path that is the only one the shipping 1-MV-per-MB v3
        // / v1 / v2 picture decoder exercises today. Pin the predictor
        // output for a fully-populated interior MB against a direct
        // `predict_block_mv(Block::TopLeft, ..)` call built by hand from
        // the same three neighbour MVs.
        use crate::mv::Mv;
        use crate::mv_pred::{predict_block_mv, Block, BlockCandidates, MvGrid, MvGridCell};

        let mut grid = MvGrid::new(3, 3);
        let left = Mv { x: 4, y: -2 };
        let above = Mv { x: -1, y: 3 };
        let above_right = Mv { x: 6, y: 6 };
        // Lay neighbours of the interior MB (1, 1): left (0, 1),
        // above (1, 0), above-right (2, 0).
        grid.set_cell(0, 1, MvGridCell::OneMv(left));
        grid.set_cell(1, 0, MvGridCell::OneMv(above));
        grid.set_cell(2, 0, MvGridCell::OneMv(above_right));

        let got = one_mv_predictor(&grid, 1, 1);
        let want = predict_block_mv(
            Block::TopLeft,
            &BlockCandidates {
                left_mb: Some(left),
                above_mb: Some(above),
                above_right_mb: Some(above_right),
                mb_block_1: None,
                mb_block_2: None,
                mb_block_3: None,
            },
        );
        assert_eq!(got, want);
    }

    #[test]
    fn round_306_one_mv_predictor_picks_bordering_cell_of_four_mv_neighbour() {
        // When a neighbouring MB is 4-MV-coded, `one_mv_predictor` must
        // source the *physically-bordering* 8x8 cell per Figure 7-34
        // (round 208 `bordering_block_of_neighbour`), not an arbitrary
        // raster index. For the current MB's block 1 (Block::TopLeft):
        //   left neighbour     -> its block 2 (TR, raster index 1)
        //   above neighbour    -> its block 3 (BL, raster index 2)
        //   above-right neighbr -> its block 3 (BL, raster index 2)
        // This pins the post-r306 resolver path against the documented
        // mapping and against a by-hand `predict_block_mv` built from
        // exactly those bordering cells.
        use crate::mv::Mv;
        use crate::mv_pred::{predict_block_mv, Block, BlockCandidates, MvGrid, MvGridCell};

        let mk = |base: i8| {
            [
                Mv { x: base, y: base }, // block 1 (TL)
                Mv {
                    x: base + 1,
                    y: base + 1,
                }, // block 2 (TR)
                Mv {
                    x: base + 2,
                    y: base + 2,
                }, // block 3 (BL)
                Mv {
                    x: base + 3,
                    y: base + 3,
                }, // block 4 (BR)
            ]
        };
        let left4 = mk(10);
        let above4 = mk(20);
        let above_right4 = mk(30);

        let mut grid = MvGrid::new(3, 3);
        grid.set_cell(0, 1, MvGridCell::FourMv(left4));
        grid.set_cell(1, 0, MvGridCell::FourMv(above4));
        grid.set_cell(2, 0, MvGridCell::FourMv(above_right4));

        let got = one_mv_predictor(&grid, 1, 1);
        let want = predict_block_mv(
            Block::TopLeft,
            &BlockCandidates {
                left_mb: Some(left4[1]),               // left -> block 2 (TR)
                above_mb: Some(above4[2]),             // above -> block 3 (BL)
                above_right_mb: Some(above_right4[2]), // above-right -> block 3 (BL)
                mb_block_1: None,
                mb_block_2: None,
                mb_block_3: None,
            },
        );
        assert_eq!(got, want);
    }
}
