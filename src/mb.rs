//! Macroblock-level decode for MS-MPEG4v3.
//!
//! An MS-MPEG4v3 intra MB consists of:
//!
//! * 1 bit `ac_pred_flag` — is AC prediction active for this MB?
//! * VLC `CBPY` — coded-block-pattern for the 4 luma blocks. The two
//!   chroma blocks use a separate CBP field that lives in the MB-type
//!   escape chain (spec §3.1 MCBPCY) and is decoded elsewhere.
//! * Per-block (6 total — 4 luma + 2 chroma):
//!   - DC differential (DC-size VLC + that many bits of magnitude).
//!   - If this block's CBP bit is set: AC coefficients (run/level VLC
//!     — not yet implemented, see [`crate::tables`]).
//!
//! This module implements everything **up to** the AC walk, including
//! DC prediction and bookkeeping of prediction-cache state. The AC
//! walk currently returns [`Error::Unsupported`] pending the run/level
//! tables.

use std::sync::OnceLock;

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_core::{Error, Result};

use crate::ac::{decode_intra_ac, AcVlcTable, Scan};
use crate::iq::{dc_scaler, dequantise_h263};
use crate::tables::{CBPY_INTRA_TABLE, DC_SIZE_CHROMA_TABLE, DC_SIZE_LUMA_TABLE};
use crate::tables_data::{
    INTRA_DC_CHROMA_SEL0_ESC_INDEX, INTRA_DC_CHROMA_SEL0_RAW, INTRA_DC_CHROMA_SEL1_ESC_INDEX,
    INTRA_DC_CHROMA_SEL1_RAW, INTRA_DC_LUMA_SEL0_ESC_INDEX, INTRA_DC_LUMA_SEL0_RAW,
    INTRA_DC_LUMA_SEL1_ESC_INDEX, INTRA_DC_LUMA_SEL1_RAW,
};
use crate::vlc::{self, VlcEntry};

/// Per-block prediction-cache entry. Stores the reconstructed DC
/// coefficient (in pel/post-scaler domain) so the next MB's DC
/// predictor can read it.
#[derive(Clone, Copy, Debug, Default)]
pub struct BlockPred {
    pub dc: i32,
    /// `quant` in effect when this block was decoded (needed for AC
    /// prediction rescaling — same rule as in MPEG-4 Part 2).
    pub quant: u8,
    pub is_intra: bool,
}

/// Intra MB header — the state decoded before per-block decode begins.
///
/// Two shapes are supported:
///
/// * [`IntraMbHeader::parse`] — legacy H.263-style parse (`ac_pred` bit
///   followed by a CBPY VLC for the 4 luma blocks). The MSMPEG4v3 bit
///   layout does **not** match this; it is retained for v1/v2 paths and
///   for tests that exercise the old shape.
/// * [`IntraMbHeader::parse_v3_mcbpcy`] — MSMPEG4 v3 joint-MCBPCY parse
///   (spec §3.1 / spec/05 §3.2): decode the 128-entry joint-MCBPCY VLC
///   first, then read the post-VLC `ac_pred_flag` bit. Produces
///   `ac_pred`, 4-bit `cbpy`, and the two chroma CBP bits.
#[derive(Clone, Copy, Debug)]
pub struct IntraMbHeader {
    pub ac_pred: bool,
    /// 4-bit mask: bit `i` = 1 if luma block `i` has coded AC
    /// coefficients. Luma blocks are ordered raster (top-left,
    /// top-right, bottom-left, bottom-right).
    pub cbpy: u8,
    /// Cb chroma CBP bit (set only by the joint-MCBPCY parse).
    pub cbp_cb: bool,
    /// Cr chroma CBP bit (set only by the joint-MCBPCY parse).
    pub cbp_cr: bool,
}

impl IntraMbHeader {
    pub fn parse(br: &mut BitReader<'_>) -> Result<Self> {
        let ac_pred = br.read_bit()?;
        let cbpy = vlc::decode_named(br, CBPY_INTRA_TABLE, "intra cbpy")?;
        Ok(Self {
            ac_pred,
            cbpy,
            cbp_cb: false,
            cbp_cr: false,
        })
    }

    /// MSMPEG4 v3 intra MB header: decode the 128-entry joint-MCBPCY
    /// canonical-Huffman VLC, then read 1 bit for `ac_pred_flag`.
    ///
    /// The decoded joint index's low 6 bits give the 6-block CBP
    /// pattern (4 luma + 2 chroma), and the post-VLC bit is
    /// `ac_pred_flag` (spec/05 §3.2 step 5, "sign bit `0x1c215c9b` at
    /// `1c2178ca`").
    pub fn parse_v3_mcbpcy(br: &mut BitReader<'_>) -> Result<Self> {
        let dec = crate::mcbpcy::decode_mcbpcy(br)?;
        let ac_pred = br.read_bit()?;
        Ok(Self {
            ac_pred,
            cbpy: dec.cbpy,
            cbp_cb: dec.cbp_cb,
            cbp_cr: dec.cbp_cr,
        })
    }
}

/// Decode the intra DC differential at the current bit position for a
/// single 8×8 block. Returns the signed DC residual (in the post-scaler
/// / pel domain if multiplied by `dc_scaler(block_idx, quant)`; the
/// caller does that multiplication).
///
/// `block_idx` 0..=3 selects the luma table, 4..=5 selects chroma.
///
/// Uses the legacy MPEG-4 Part 2 §6.3.8 size-category VLC.
/// **NOT bit-exact for MS-MPEG4v3** — the v3 binary uses a custom
/// 120-entry direct-value VLC (see [`decode_intra_dc_diff_v3`]). Kept
/// for backwards compatibility with the existing v1/v2-style tests.
pub fn decode_intra_dc_diff(br: &mut BitReader<'_>, block_idx: usize) -> Result<i32> {
    let table = if block_idx < 4 {
        DC_SIZE_LUMA_TABLE
    } else {
        DC_SIZE_CHROMA_TABLE
    };
    let size = vlc::decode_named(br, table, "legacy intra-DC size")? as u32;
    if size == 0 {
        return Ok(0);
    }
    // `size` unsigned bits of DC value. The MSB is the sign — MPEG-4
    // Part 2 §6.3.8: value = raw if MSB set, else raw - (2^size - 1).
    let raw = br.read_u32(size)? as i32;
    let msb_set = raw & (1 << (size - 1)) != 0;
    let value = if msb_set {
        raw
    } else {
        raw - ((1 << size) - 1)
    };
    // MPEG-4 Part 2 §6.3.8 calls for a single-bit marker after the
    // magnitude when `size > 8` to prevent start-code emulation. MS-MPEG4v3
    // streams **do not** carry a start-code layer, so the binary
    // `1c216cf8` intra-DC kernel does not consume any extra bit. The
    // marker read is intentionally omitted here.
    Ok(value)
}

// ====================================================================
// MS-MPEG4 v1/v2 intra-DC size-category VLC (spec/16 §2 / Extractor 07).
// ====================================================================
//
// Per `docs/video/msmpeg4/spec/16-mv-vlc-dc-mcbpc-extraction.md` §2 the
// MS-MPEG4 v1 and v2 intra-block driver decodes the DC differential
// through the classic H.263 §5.4.1 / MPEG-4 Part 2 §7.4.3
// DC-size-then-value scheme (kernel `sub_15790`), NOT the v3
// direct-value VLC. The intra-block driver gates on version
// (`cmp [esi+8], 3`): v < 3 → this size+value path with the binary's
// own luma/chroma size tables (`region_0542c0` / `region_0543c0`, VMAs
// 0x1c2542c0 / 0x1c2543c0), distinct from the four v3 `dc_size_sel`
// tables consumed by `decode_intra_dc_diff_v3`.
//
// The size tables are loaded from the binary in `build.rs` as
// `DC_SIZE_LUMA_V1V2_RAW` / `DC_SIZE_CHROMA_V1V2_RAW`. They differ from
// the MPEG-4-Part-2 Annex-B `DC_SIZE_LUMA_TABLE` / `DC_SIZE_CHROMA_TABLE`
// in `tables.rs` (e.g. v1/v2 binary luma size 0 = `100`, whereas the
// MPEG-4-P2 Annex-B luma size 0 = `011`); the v1/v2 path therefore
// cannot reuse `decode_intra_dc_diff` and binds its own tables here.

static DC_SIZE_LUMA_V1V2_TABLE: OnceLock<Vec<VlcEntry<u8>>> = OnceLock::new();
static DC_SIZE_CHROMA_V1V2_TABLE: OnceLock<Vec<VlcEntry<u8>>> = OnceLock::new();

fn build_dc_size_v1v2(raw: &[(u8, u8, u32)]) -> Vec<VlcEntry<u8>> {
    raw.iter()
        .map(|&(sym, bl, code)| VlcEntry::new(bl, code, sym))
        .collect()
}

fn dc_size_v1v2_table(block_idx: usize) -> &'static [VlcEntry<u8>] {
    if block_idx < 4 {
        DC_SIZE_LUMA_V1V2_TABLE
            .get_or_init(|| build_dc_size_v1v2(crate::tables_data::DC_SIZE_LUMA_V1V2_RAW))
            .as_slice()
    } else {
        DC_SIZE_CHROMA_V1V2_TABLE
            .get_or_init(|| build_dc_size_v1v2(crate::tables_data::DC_SIZE_CHROMA_V1V2_RAW))
            .as_slice()
    }
}

/// Decode one v1/v2 intra-DC differential via the H.263 size-category
/// scheme (spec/16 §2, kernel `sub_15790`).
///
/// `block_idx` 0..=3 selects the luma size table, 4..=5 selects chroma.
///
/// Output is the signed DC *differential* (NOT yet added to the spatial
/// predictor and NOT yet multiplied by the DC scaler — the caller does
/// both via [`reconstruct_intra_dc`], exactly as on the v3 path).
///
/// The decode sequence (spec/16 §2.1):
///   1. Decode a size category `s` from the binary-extracted size VLC.
///   2. If `s == 0` → differential = 0 (no value bits consumed).
///   3. Else read `s` raw bits → `value`, then apply the standard
///      H.263 signed-DC fixup: if `value < 2^(s-1)` then
///      `value -= 2^s - 1`. (The binary realises this at `0x1580a`.)
///
/// There is **no** start-code-emulation marker bit for `s > 8` — the
/// MS-MPEG4 stream has no start-code layer (same reasoning as the v3 and
/// legacy paths in this module).
pub fn decode_intra_dc_diff_v1v2(br: &mut BitReader<'_>, block_idx: usize) -> Result<i32> {
    let table = dc_size_v1v2_table(block_idx);
    let size = vlc::decode_named(br, table, "v1/v2 intra-DC size")? as u32;
    if size == 0 {
        return Ok(0);
    }
    if size > 11 {
        // Size categories are 0..=8 in the binary tables; a decode above
        // that is a corrupt bitstream / table mismatch. Guard the shift.
        return Err(Error::invalid(format!(
            "msmpeg4 v1/v2 intra DC: size category {size} out of range (0..=8 \
             per spec/16 §2)"
        )));
    }
    let value = br.read_u32(size)? as i32;
    // H.263 §5.4.1 negative-value reconstruction (spec/16 §2.1 step 3):
    // values in the lower half of the 2^size range are negative.
    let half = 1i32 << (size - 1);
    let fixed = if value < half {
        value - ((1 << size) - 1)
    } else {
        value
    };
    Ok(fixed)
}

/// Encode one v1/v2 intra-DC differential — the bit-level inverse of
/// [`decode_intra_dc_diff_v1v2`] (H.263 §5.4.1 size+value scheme,
/// spec/16 §2): the size category is the bit-length of `|diff|`
/// (0 for a zero differential — no value bits), then `size` raw bits
/// carrying `diff` directly when positive or `diff + 2^size − 1` when
/// negative (the H.263 lower-half-negative fixup). The binary's size
/// tables cover categories 0..=8, bounding `|diff|` at 255 — the same
/// ceiling as the v3 ESC tier, and never exceeded by a DC differential
/// of any realisable 8-bit pel block at the minimum DC scaler of 8.
pub fn encode_intra_dc_diff_v1v2(bw: &mut BitWriter, block_idx: usize, diff: i32) -> Result<()> {
    if diff.unsigned_abs() > 255 {
        return Err(Error::invalid(format!(
            "msmpeg4 v1/v2 intra DC: |diff| {} exceeds size category 8 (spec/16 §2)",
            diff.unsigned_abs()
        )));
    }
    let table = dc_size_v1v2_table(block_idx);
    let size = 32 - diff.unsigned_abs().leading_zeros(); // 0 for diff == 0
    let entry = table
        .iter()
        .find(|e| e.value as u32 == size)
        .ok_or_else(|| {
            Error::invalid(format!(
                "msmpeg4 v1/v2 intra DC: size category {size} has no codeword"
            ))
        })?;
    bw.write_u32(entry.code, entry.bits as u32);
    if size == 0 {
        return Ok(());
    }
    let value = if diff > 0 {
        diff
    } else {
        diff + (1 << size) - 1
    };
    bw.write_u32(value as u32, size);
    Ok(())
}

// ====================================================================
// MS-MPEG4 v3 custom intra-DC differential VLC (round 28 / task #113).
// ====================================================================
//
// Per `docs/video/msmpeg4/spec/07-remaining-opens.md` §5.4 the v3 intra
// DC kernel `0x1c216cf8` does NOT use the MPEG-4 Part 2 size-category
// scheme: it reads a 120-entry canonical-Huffman VLC where the decoded
// `idx` is the differential magnitude directly (idx == 0 → diff=0 with
// no sign bit; idx ∈ [1, 118] → ±idx with a sign bit; idx == 119 → ESC
// sentinel which reads an 8-bit raw + sign bit).
//
// There are FOUR tables, picked by (luma vs chroma) × (`dc_size_sel`
// 0 vs 1). The picture-level `dc_size_sel` bit lives in the v3 picture
// header (`crate::header::MsV3PictureHeader::dc_size_sel`).

/// One canonical-Huffman entry for the intra-DC VLC: the decoded value
/// is the alphabet index (0..=119), where 0..=118 are direct DC
/// magnitudes and 119 is the ESC sentinel. Wrapped in `u16` for safety
/// even though `u8` would fit.
type DcVlcEntry = VlcEntry<u16>;

static INTRA_DC_LUMA_SEL0_TABLE: OnceLock<Vec<DcVlcEntry>> = OnceLock::new();
static INTRA_DC_CHROMA_SEL0_TABLE: OnceLock<Vec<DcVlcEntry>> = OnceLock::new();
static INTRA_DC_LUMA_SEL1_TABLE: OnceLock<Vec<DcVlcEntry>> = OnceLock::new();
static INTRA_DC_CHROMA_SEL1_TABLE: OnceLock<Vec<DcVlcEntry>> = OnceLock::new();

fn build_dc_table(raw: &[(u32, u32)]) -> Vec<DcVlcEntry> {
    // The packed source already carries `(bit_length, code_value)`
    // pairs for each symbol; the canonical bit-pattern is `code_value`
    // verbatim (Kraft sum = 1 across the 120 entries, verified at build
    // time). Hole sentinels (bl == 0) drop out of the entry list — they
    // never produce a matchable codeword for the linear-scan walker.
    raw.iter()
        .enumerate()
        .filter_map(|(idx, &(bl, code))| {
            if bl == 0 {
                None
            } else {
                Some(VlcEntry::new(bl as u8, code, idx as u16))
            }
        })
        .collect()
}

fn dc_table(block_idx: usize, dc_size_sel: u8) -> &'static [DcVlcEntry] {
    let is_luma = block_idx < 4;
    match (is_luma, dc_size_sel) {
        (true, 0) => INTRA_DC_LUMA_SEL0_TABLE
            .get_or_init(|| build_dc_table(INTRA_DC_LUMA_SEL0_RAW))
            .as_slice(),
        (false, 0) => INTRA_DC_CHROMA_SEL0_TABLE
            .get_or_init(|| build_dc_table(INTRA_DC_CHROMA_SEL0_RAW))
            .as_slice(),
        (true, _) => INTRA_DC_LUMA_SEL1_TABLE
            .get_or_init(|| build_dc_table(INTRA_DC_LUMA_SEL1_RAW))
            .as_slice(),
        (false, _) => INTRA_DC_CHROMA_SEL1_TABLE
            .get_or_init(|| build_dc_table(INTRA_DC_CHROMA_SEL1_RAW))
            .as_slice(),
    }
}

fn dc_esc_index(block_idx: usize, dc_size_sel: u8) -> usize {
    let is_luma = block_idx < 4;
    match (is_luma, dc_size_sel) {
        (true, 0) => INTRA_DC_LUMA_SEL0_ESC_INDEX,
        (false, 0) => INTRA_DC_CHROMA_SEL0_ESC_INDEX,
        (true, _) => INTRA_DC_LUMA_SEL1_ESC_INDEX,
        (false, _) => INTRA_DC_CHROMA_SEL1_ESC_INDEX,
    }
}

/// Decode the v3 intra-DC differential using the custom direct-value
/// VLC (per spec/07 §5.4).
///
/// Output is the signed DC differential MAGNITUDE (NOT yet multiplied
/// by the `dc_scaler` — the caller handles that). Range is `[-119, 119]`
/// for non-ESC paths and `[-255, 255]` for the ESC tier.
///
/// `dc_size_sel` is the picture-header bit (0 or 1) selecting between
/// the two pairs of tables (per spec/99 §4.5).
pub fn decode_intra_dc_diff_v3(
    br: &mut BitReader<'_>,
    block_idx: usize,
    dc_size_sel: u8,
) -> Result<i32> {
    let table = dc_table(block_idx, dc_size_sel);
    let esc = dc_esc_index(block_idx, dc_size_sel);
    // Label the decode with the concrete table: the direct-value DC
    // VLCs have bit-length-0 holes, so they are one of the two places
    // a real-content "no matching codeword" can legitimately originate
    // (the other being the AC primaries).
    let what = match (block_idx < 4, dc_size_sel) {
        (true, 0) => "v3 intra-DC luma sel0",
        (false, 0) => "v3 intra-DC chroma sel0",
        (true, _) => "v3 intra-DC luma sel1",
        (false, _) => "v3 intra-DC chroma sel1",
    };
    let idx = vlc::decode_named(br, table, what)? as usize;
    if idx == 0 {
        // idx == 0 ⇒ DC differential = 0, no sign bit consumed
        // (`1c216d2a: test al, al; je 0x1c216d47` per spec/07 §5.2).
        return Ok(0);
    }
    if idx == esc {
        // ESC tier: read an 8-bit raw unsigned magnitude then a sign
        // bit (`push 0x8; call 0x1c211e39` then `0x1c215c9b` per
        // spec/07 §5.2).
        //
        // Round 420 sign-convention correction: the earlier reading of
        // the trace (`if sign==0 neg eax` ⇒ "0 means negative") is
        // refuted by the staged real-content fixtures — the first
        // I-frame of div4.avi opens with a −58 luma DC differential
        // whose sign bit is `1`, and every DC of the first eight MB
        // columns of both DIV3 AVI fixtures reconstructs the reference
        // pixels exactly only under the standard convention:
        // **sign bit 1 ⇒ negative** (same as the AC walk).
        let raw = br.read_u32(8)? as i32;
        let sign = br.read_bit()?;
        return Ok(if sign { -raw } else { raw });
    }
    // Normal path: idx is the differential magnitude; sign bit follows
    // (1 ⇒ negative, same convention as the ESC tier).
    let mag = idx as i32;
    let sign = br.read_bit()?;
    Ok(if sign { -mag } else { mag })
}

/// Encode one v3 intra-DC differential — the bit-level inverse of
/// [`decode_intra_dc_diff_v3`] (kernel `0x1c216cf8`, spec/07 §5.4).
///
/// * `diff == 0` → the idx-0 codeword alone (no sign bit, per the
///   `test al, al; je` short-circuit at `1c216d2a`).
/// * `1 <= |diff| <= 118` with a present codeword (some magnitudes are
///   bit-length-0 holes in the extracted tables) → the magnitude's
///   codeword followed by the DC sign bit. Round 420: the sign
///   convention is the **standard** one (`1` ⇒ negative), the same as
///   the AC walk — the earlier "inverted" reading of the spec/07 §5.2
///   trace was refuted against the staged real-content fixtures (see
///   [`decode_intra_dc_diff_v3`]).
/// * otherwise → the ESC codeword (idx 119), an 8-bit raw magnitude,
///   then the same sign bit. Magnitudes above 255 are not
///   representable and error out (the intra DC differential of any
///   realisable 8-bit pel block stays well inside ±255 at the minimum
///   DC scaler of 8).
pub fn encode_intra_dc_diff_v3(
    bw: &mut BitWriter,
    block_idx: usize,
    dc_size_sel: u8,
    diff: i32,
) -> Result<()> {
    let table = dc_table(block_idx, dc_size_sel);
    let esc = dc_esc_index(block_idx, dc_size_sel);
    let write_idx = |bw: &mut BitWriter, idx: usize| -> Result<()> {
        let entry = table
            .iter()
            .find(|e| e.value as usize == idx)
            .ok_or_else(|| {
                Error::invalid(format!(
                    "msmpeg4v3 intra DC: magnitude {idx} has no codeword (bit-length hole)"
                ))
            })?;
        bw.write_u32(entry.code, entry.bits as u32);
        Ok(())
    };
    if diff == 0 {
        return write_idx(bw, 0);
    }
    let mag = diff.unsigned_abs() as usize;
    let negative = diff < 0;
    if mag < esc && write_idx(bw, mag).is_ok() {
        // Direct-magnitude codeword + sign bit (1 => negative).
        bw.write_bit(negative);
        return Ok(());
    }
    if mag > 255 {
        return Err(Error::invalid(format!(
            "msmpeg4v3 intra DC: |diff| {mag} exceeds the 8-bit ESC tier"
        )));
    }
    write_idx(bw, esc)?;
    bw.write_u32(mag as u32, 8);
    bw.write_bit(negative);
    Ok(())
}

/// Reconstruct the DC coefficient value for one intra block, combining
/// the bitstream differential with a predicted DC (provided by the
/// caller from the neighbour cache).
pub fn reconstruct_intra_dc(dc_diff: i32, pred_dc: i32, block_idx: usize, quant: u32) -> i32 {
    let scaler = dc_scaler(block_idx, quant) as i32;
    pred_dc + dc_diff * scaler
}

/// Fully-decoded 8×8 intra block in the coefficient domain (post
/// dequantisation, pre-IDCT).
#[derive(Clone, Copy, Debug)]
pub struct DecodedIntraBlock {
    pub coeffs: [i32; 64],
    /// Number of non-zero AC levels emitted (DC excluded).
    pub ac_nonzero: u32,
}

impl Default for DecodedIntraBlock {
    fn default() -> Self {
        Self {
            coeffs: [0i32; 64],
            ac_nonzero: 0,
        }
    }
}

/// Decode one full intra 8×8 block: DC differential (with the supplied
/// predictor), then an AC walk through `ac_table` if the block's CBP
/// bit is set, then H.263-style dequantisation of the AC levels.
///
/// `block_idx` follows the MS-MPEG4 intra convention: 0..=3 are the
/// four luma sub-blocks of the MB (raster order), 4 is Cb, 5 is Cr.
///
/// `cbp_set` is true when the CBP pattern for this block has its bit
/// set (meaning the bitstream has an AC run/level sequence for this
/// block). When false, the AC plane is all zero — only the DC is
/// written.
///
/// Distinct from [`crate::ac::decode_intra_block`], which is the
/// lower-level coefficient-array walker called *after* the DC has
/// already been written into the block array by the caller. This
/// version owns the DC decode as well.
pub fn decode_intra_block_full(
    br: &mut BitReader<'_>,
    block_idx: usize,
    pred_dc: i32,
    quant: u32,
    cbp_set: bool,
    scan: Scan,
    ac_table: &AcVlcTable,
) -> Result<DecodedIntraBlock> {
    // Legacy path: use the MPEG-4 Part 2 size-category DC VLC (matches
    // the existing v1/v2 tests and synthetic-stream callers). For v3
    // streams the picture decoder uses [`decode_intra_block_full_v3`]
    // directly with the picture-header `dc_size_sel` bit.
    let dc_diff = decode_intra_dc_diff(br, block_idx)?;
    let dc = reconstruct_intra_dc(dc_diff, pred_dc, block_idx, quant);

    let mut out = DecodedIntraBlock::default();
    out.coeffs[0] = dc;

    if cbp_set {
        out.ac_nonzero = decode_intra_ac(br, &mut out.coeffs, scan, ac_table, 1)?;
        dequantise_h263(&mut out.coeffs, quant, 1)?;
    }

    Ok(out)
}

/// Variant of [`decode_intra_block_full`] that takes the picture-level
/// `dc_size_sel` bit and routes the DC-differential decode through the
/// MS-MPEG4v3 custom 120-entry direct-value VLC (per spec/07 §5.4 +
/// spec/99 §4.5). Used by the v3 picture decoder; v1/v2 paths and tests
/// continue to call [`decode_intra_block_full`] which defaults to
/// `dc_size_sel = 0` and the legacy MPEG-4-P2-style decoder.
#[allow(clippy::too_many_arguments)]
pub fn decode_intra_block_full_v3(
    br: &mut BitReader<'_>,
    block_idx: usize,
    pred_dc: i32,
    quant: u32,
    cbp_set: bool,
    scan: Scan,
    ac_table: &AcVlcTable,
    dc_size_sel: u8,
) -> Result<DecodedIntraBlock> {
    let dc_diff = decode_intra_dc_diff_v3(br, block_idx, dc_size_sel)?;
    let dc = reconstruct_intra_dc(dc_diff, pred_dc, block_idx, quant);

    let mut out = DecodedIntraBlock::default();
    out.coeffs[0] = dc;

    if cbp_set {
        out.ac_nonzero = decode_intra_ac(br, &mut out.coeffs, scan, ac_table, 1)?;
        // H.263 dequant for AC (level_start=1 skips DC).
        dequantise_h263(&mut out.coeffs, quant, 1)?;
    }

    Ok(out)
}

/// Variant of [`decode_intra_block_full`] for the MS-MPEG4 **v1/v2**
/// intra path: the DC differential is decoded through the H.263
/// size-category VLC ([`decode_intra_dc_diff_v1v2`], spec/16 §2) using
/// the binary-extracted v1/v2 size tables, then the AC walk runs through
/// the supplied table (the shared G-family intra/inter VLCs).
///
/// This is the v1/v2 analogue of [`decode_intra_block_full_v3`]; the
/// only difference is the DC-differential decoder (v1/v2 size+value vs
/// v3 direct-value). The spatial-predictor reconstruction, AC walk, and
/// H.263 dequant are all shared with v3 per spec/99 §4.4 (the
/// DC-predictor gradient routine has no version gate) and spec/04 §2.6.
pub fn decode_intra_block_full_v1v2(
    br: &mut BitReader<'_>,
    block_idx: usize,
    pred_dc: i32,
    quant: u32,
    cbp_set: bool,
    scan: Scan,
    ac_table: &AcVlcTable,
) -> Result<DecodedIntraBlock> {
    let dc_diff = decode_intra_dc_diff_v1v2(br, block_idx)?;
    let dc = reconstruct_intra_dc(dc_diff, pred_dc, block_idx, quant);

    let mut out = DecodedIntraBlock::default();
    out.coeffs[0] = dc;

    if cbp_set {
        out.ac_nonzero = decode_intra_ac(br, &mut out.coeffs, scan, ac_table, 1)?;
        dequantise_h263(&mut out.coeffs, quant, 1)?;
    }

    Ok(out)
}

/// Decoded intra macroblock: header + 6 blocks (4 luma + 2 chroma).
#[derive(Clone, Debug)]
pub struct DecodedIntraMb {
    pub header: IntraMbHeader,
    pub blocks: [DecodedIntraBlock; 6],
}

/// Decode one full intra macroblock: parse the header, then all 6
/// blocks using the caller-supplied AC VLC table.
///
/// The chroma CBP bits (one per chroma block) are assumed to come from
/// outside the IntraMbHeader (they live in the MB-type escape chain,
/// see `mb.rs` module docs). Until that chain is wired up, `cbp_cb`
/// and `cbp_cr` must be passed explicitly — a conservative caller can
/// pass `true` for both to exercise the AC walk unconditionally.
///
/// **All 6 blocks are decoded in bitstream order:**
///   block 0..=3 = luma (raster top-left, top-right, bottom-left,
///   bottom-right), block 4 = Cb, block 5 = Cr. This matches the
///   MS-MPEG4v3 per-MB bitstream layout described in the module-level
///   doc comment.
pub fn decode_intra_mb(
    br: &mut BitReader<'_>,
    quant: u32,
    cbp_cb: bool,
    cbp_cr: bool,
    pred_dc: [i32; 6],
    scan: Scan,
    ac_table: &AcVlcTable,
) -> Result<DecodedIntraMb> {
    let header = IntraMbHeader::parse(br)?;

    // Early check: an empty AC table is the "placeholder" sentinel —
    // bail here with a precise, actionable error instead of letting
    // the first `vlc::decode` call fall over with a generic message.
    // See `AcVlcTable::V3_INTRA_PLACEHOLDER` in ac.rs for the full
    // doc-line citations of what's OPEN.
    if ac_table.entries.is_empty() {
        return Err(Error::unsupported(
            "msmpeg4v3 intra AC VLC: placeholder table in use — the G5 \
             (intra-luma DCT AC TCOEF) descriptor's canonical-Huffman \
             code-length array has not been extracted into a runtime \
             VlcEntry<Symbol> slice. Per docs/video/msmpeg4/spec/99 §5, \
             the G5 source lives at the packed-Huffman input region VMA \
             0x1c259d78 (file offset 0x59178) with runtime alphabet \
             count_A=102, count_B=66 — but the constructor algorithm at \
             VMA 0x1c210ee6 that turns the packed input into the runtime \
             descriptor (per spec/99 §10.1) has not been disassembled. \
             A standalone candidate canonical-Huffman block was extracted \
             into tables/region_05eed0.csv (VMA 0x1c25fad0, 64 entries) \
             but its alphabet shape mismatches G5 and per spec/99 §9 \
             OPEN-O6 it is more likely the v2-MCBPCY source. The OPEN \
             gating item is spec/99 §9 OPEN-O4 (G0..G3 entry-by-entry \
             enumeration also missing). To unblock real-content decode, \
             a future Extractor + Specifier session needs to (a) trace \
             the constructor at 0x1c210ee6, or (b) capture the runtime \
             descriptor bytes via Frida instrumentation of mpg4c32.dll \
             during a known-good decode.",
        ));
    }

    let mut blocks = [DecodedIntraBlock::default(); 6];
    for (i, block) in blocks.iter_mut().enumerate() {
        let cbp_set = match i {
            0..=3 => header.cbpy & (1 << (3 - i)) != 0,
            4 => cbp_cb,
            5 => cbp_cr,
            _ => unreachable!(),
        };
        *block = decode_intra_block_full(br, i, pred_dc[i], quant, cbp_set, scan, ac_table)?;
    }

    Ok(DecodedIntraMb { header, blocks })
}

#[cfg(test)]
mod tests {
    use super::*;

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
        if out.is_empty() {
            out.push(0);
        }
        out
    }

    #[test]
    fn parse_intra_mb_header_all_coded() {
        // ac_pred = 1; CBPY = 15 (shortest code `11`).
        let bytes = pack(&[(1, 1), (0b11, 2), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        let h = IntraMbHeader::parse(&mut br).unwrap();
        assert!(h.ac_pred);
        assert_eq!(h.cbpy, 15);
        assert!(!h.cbp_cb);
        assert!(!h.cbp_cr);
    }

    #[test]
    fn parse_intra_mb_header_no_ac() {
        // ac_pred = 0; CBPY = 0 (`0011`).
        let bytes = pack(&[(0, 1), (0b0011, 4), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        let h = IntraMbHeader::parse(&mut br).unwrap();
        assert!(!h.ac_pred);
        assert_eq!(h.cbpy, 0);
        assert!(!h.cbp_cb);
        assert!(!h.cbp_cr);
    }

    #[test]
    fn dc_diff_size_zero_returns_zero() {
        // Luma size 0 has code `011` (3 bits).
        let bytes = pack(&[(0b011, 3), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff(&mut br, 0).unwrap(), 0);
    }

    #[test]
    fn dc_diff_size_1_positive() {
        // Luma size 1 = code `11` (2 bits). Then 1 bit of magnitude: `1` = +1.
        let bytes = pack(&[(0b11, 2), (0b1, 1), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff(&mut br, 0).unwrap(), 1);
    }

    #[test]
    fn dc_diff_size_1_negative() {
        // Luma size 1, bit `0` -> raw=0 -> 0 - (2-1) = -1.
        let bytes = pack(&[(0b11, 2), (0b0, 1), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff(&mut br, 0).unwrap(), -1);
    }

    #[test]
    fn dc_diff_chroma_selects_chroma_table() {
        // Chroma size 1 = code `10` (2 bits).
        let bytes = pack(&[(0b10, 2), (0b1, 1), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff(&mut br, 4).unwrap(), 1);
    }

    #[test]
    fn reconstruct_intra_dc_applies_scaler() {
        // q=8 luma -> scaler 16. pred=1024, diff=1 -> 1024 + 16 = 1040.
        assert_eq!(reconstruct_intra_dc(1, 1024, 0, 8), 1040);
        // Chroma q=8 -> scaler 10. diff=-2 -> 1024 + (-2)*10 = 1004.
        assert_eq!(reconstruct_intra_dc(-2, 1024, 4, 8), 1004);
    }

    #[test]
    fn decode_intra_mb_placeholder_fails_with_actionable_error() {
        // Minimal header: ac_pred=0, CBPY=0 (no coded AC), then 6
        // DC-size-0 codewords (luma `011` x4, chroma `10` x2).
        let bytes = pack(&[
            (0, 1),      // ac_pred = 0
            (0b0011, 4), // CBPY = 0
            (0b011, 3),  // block 0 DC size 0
            (0b011, 3),  // block 1 DC size 0
            (0b011, 3),  // block 2 DC size 0
            (0b011, 3),  // block 3 DC size 0
            (0b11, 2),   // block 4 Cb DC size 0 (chroma code `11`)
            (0b11, 2),   // block 5 Cr DC size 0 (chroma code `11`)
            (0, 16),     // tail padding
        ]);
        let mut br = BitReader::new(&bytes);
        let pred = [1024i32; 6];
        // Placeholder table: empty entries array → sentinel path.
        let err = decode_intra_mb(
            &mut br,
            8,
            false,
            false,
            pred,
            crate::ac::Scan::Zigzag,
            &crate::ac::AcVlcTable::V3_INTRA_PLACEHOLDER,
        )
        .unwrap_err();
        let msg = format!("{err}");
        // The error must cite the actual G5 source VMA, the constructor
        // VMA whose disassembly is the gating action, and the spec/99
        // OPEN-O4 reference so whoever reads the failure can find the
        // gate item without grep-diving.
        assert!(msg.contains("0x1c259d78"), "msg = {msg}");
        assert!(msg.contains("0x1c210ee6"), "msg = {msg}");
        assert!(
            msg.contains("OPEN-O4") || msg.contains("OPEN-O6"),
            "msg = {msg}"
        );
    }

    #[test]
    fn decode_intra_mb_with_zero_cbp_walks_dc_only() {
        // When CBPY=0 and chroma CBP=false, no block has coded AC,
        // so the placeholder sentinel is *not* triggered by the CBP
        // branch — but the empty-table check fires at entry *before*
        // any block is decoded, so we can't distinguish "no AC
        // needed" from "AC table missing" until the Extractor lands
        // the real data. This test documents the current behaviour:
        // the sentinel check is unconditional on entry.
        //
        // Once a real AcVlcTable ships, flip this test to assert
        // a successful decode of 6 DC-only blocks.
        let bytes = pack(&[
            (0, 1),      // ac_pred = 0
            (0b0011, 4), // CBPY = 0
            (0b011, 3),  // luma DC size 0
            (0b011, 3),
            (0b011, 3),
            (0b011, 3),
            (0b11, 2), // chroma DC size 0 (code `11`)
            (0b11, 2),
            (0, 16),
        ]);
        let mut br = BitReader::new(&bytes);
        let pred = [1024i32; 6];
        assert!(decode_intra_mb(
            &mut br,
            8,
            false,
            false,
            pred,
            crate::ac::Scan::Zigzag,
            &crate::ac::AcVlcTable::V3_INTRA_PLACEHOLDER,
        )
        .is_err());
    }

    #[test]
    fn decode_intra_mb_with_toy_table_walks_6_blocks() {
        use crate::ac::{AcVlcTable, Scan, Symbol};
        use crate::vlc::VlcEntry;

        // A tiny AC table where code `1` = (last=1, run=0, |level|=1).
        // After each block we emit one token `1` + sign `0` (positive
        // level 1), terminating the AC walk for that block.
        static TOY_AC: &[VlcEntry<Symbol>] = &[VlcEntry::new(
            1,
            0b1,
            Symbol::RunLevel {
                last: true,
                run: 0,
                level: 1,
            },
        )];
        let ac_table = AcVlcTable {
            entries: TOY_AC,
            esc_last_bits: AcVlcTable::MPEG4_ESC_LAST_BITS,
            esc_run_bits: AcVlcTable::MPEG4_ESC_RUN_BITS,
            esc_level_bits: AcVlcTable::MPEG4_ESC_LEVEL_BITS,
            lmax: None,
            rmax: None,
        };

        // ac_pred=0, CBPY=15 (all 4 luma blocks coded; code `11`),
        // then for each of 6 blocks: DC size 0 + AC token `1` sign `0`.
        let mut fields: Vec<(u32, u32)> = vec![(0, 1), (0b11, 2)];
        // block 0..=3 (luma): DC size 0 = `011`, then AC token `1` + sign `0`.
        for _ in 0..4 {
            fields.push((0b011, 3)); // DC size 0
            fields.push((0b1, 1)); // AC: last=1 run=0 |level|=1
            fields.push((0, 1)); // sign=0
        }
        // block 4 (Cb), block 5 (Cr): chroma DC size 0 = code `11` (2 bits), then AC.
        for _ in 0..2 {
            fields.push((0b11, 2));
            fields.push((0b1, 1));
            fields.push((0, 1));
        }
        fields.push((0, 16)); // tail padding
        let bytes = pack(&fields);
        let mut br = BitReader::new(&bytes);
        let pred = [1024i32; 6];
        let decoded = decode_intra_mb(&mut br, 8, true, true, pred, Scan::Zigzag, &ac_table)
            .expect("6-block decode");

        assert_eq!(decoded.header.cbpy, 15);
        assert!(!decoded.header.ac_pred);
        // Each luma block carries one non-zero AC (level 1 post-decode).
        for (i, b) in decoded.blocks[..4].iter().enumerate() {
            assert_eq!(b.ac_nonzero, 1, "luma block {i} non-zero count");
            // DC = pred + 0 * scaler = 1024.
            assert_eq!(b.coeffs[0], 1024, "luma block {i} DC");
        }
        // Chroma also coded (we asked for cbp_cb=cbp_cr=true).
        for (i, b) in decoded.blocks[4..].iter().enumerate() {
            assert_eq!(b.ac_nonzero, 1, "chroma block {i} non-zero count");
        }
    }

    /// The first three luma sel=0 entries are `(bl=1, code=1)`,
    /// `(bl=2, code=1)`, `(bl=4, code=1)` for symbols 0, 1, 2 (from
    /// `region_05f0d8`). Symbol 0 ⇒ DC=0 (no sign read); symbol 1 with
    /// sign bit "1" ⇒ DC=−1; symbol 2 with sign bit "0" ⇒ DC=+2.
    /// This locks in both the (a=code, b=bl) parser convention from
    /// spec/11 §4 / §6 AND the round-420 standard sign convention
    /// (1 ⇒ negative — pinned on the real-content fixtures).
    #[test]
    fn decode_intra_dc_diff_v3_luma_sel0_first_symbols() {
        // Symbol 0: bl=1 code=1 → bit `1`, no sign bit follows.
        let bytes = pack(&[(0b1, 1), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v3(&mut br, 0, 0).unwrap(), 0);

        // Symbol 1: bl=2 code=1 → bits `01`, then sign=1 ⇒ -1.
        let bytes = pack(&[(0b01, 2), (0b1, 1), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v3(&mut br, 0, 0).unwrap(), -1);

        // Symbol 2: bl=4 code=1 → bits `0001`, then sign=0 ⇒ +2.
        let bytes = pack(&[(0b0001, 4), (0b0, 1), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v3(&mut br, 0, 0).unwrap(), 2);
    }

    /// Chroma sel=0 is `region_05f4a0` (round 420 table-pairing fix:
    /// the four DC regions pair as sel0 = {05f0d8 luma, 05f4a0 chroma}
    /// and sel1 = {05f868 luma, 05fc30 chroma}, per the spec/99 §4.5
    /// slot grouping and the staged `tables-ff/msmp4-dc-tables`
    /// companion — pinned empirically on the real-content fixtures,
    /// whose dc_size_sel=1 I-frames only reconstruct through the
    /// {05f868, 05fc30} pair). `region_05f4a0` has `(bl=2, code=0)`
    /// for symbol 0: bits `00` decode to DC=0 with no sign bit.
    #[test]
    fn decode_intra_dc_diff_v3_chroma_sel0_symbol_zero() {
        // Symbol 0: bl=2 code=0 → bits `00`. No sign bit.
        let bytes = pack(&[(0b00, 2), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v3(&mut br, 4, 0).unwrap(), 0);
    }

    /// Luma sel=1 is `region_05f868` (round 420 pairing): symbol 0 is
    /// `(bl=2, code=2)` → bits `10` ⇒ DC=0, no sign bit. Chroma sel=1
    /// is `region_05fc30`: symbol 0 is `(bl=2, code=0)` → bits `00`.
    #[test]
    fn decode_intra_dc_diff_v3_sel1_symbol_zero() {
        let bytes = pack(&[(0b10, 2), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v3(&mut br, 0, 1).unwrap(), 0);

        let bytes = pack(&[(0b00, 2), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v3(&mut br, 4, 1).unwrap(), 0);
    }

    // ==== v1/v2 intra DC size-category decode (spec/16 §2) ====
    //
    // Luma size codes (spec/16 §2 table): 0=`100`, 1=`00`, 2=`01`.
    // Chroma size codes: 0=`00`, 1=`01`, 2=`10`.

    #[test]
    fn dc_diff_v1v2_luma_size_zero_is_zero() {
        // Luma size 0 = code `100` (3 bits), no value bits.
        let bytes = pack(&[(0b100, 3), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v1v2(&mut br, 0).unwrap(), 0);
    }

    #[test]
    fn dc_diff_v1v2_luma_size_1_positive_and_negative() {
        // Luma size 1 = code `00` (2 bits). Value bit `1` → 1 >= 2^0=1 → +1.
        let bytes = pack(&[(0b00, 2), (0b1, 1), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v1v2(&mut br, 0).unwrap(), 1);

        // Value bit `0` → 0 < 1 → 0 - (2^1 - 1) = -1.
        let bytes = pack(&[(0b00, 2), (0b0, 1), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v1v2(&mut br, 0).unwrap(), -1);
    }

    #[test]
    fn dc_diff_v1v2_luma_size_2_signed_fixup() {
        // Luma size 2 = code `01`. Value `11` = 3 → 3 >= 2^1=2 → +3.
        let bytes = pack(&[(0b01, 2), (0b11, 2), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v1v2(&mut br, 0).unwrap(), 3);

        // Value `00` = 0 → 0 < 2 → 0 - (2^2 - 1) = -3.
        let bytes = pack(&[(0b01, 2), (0b00, 2), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v1v2(&mut br, 0).unwrap(), -3);

        // Value `01` = 1 → 1 < 2 → 1 - 3 = -2.
        let bytes = pack(&[(0b01, 2), (0b01, 2), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v1v2(&mut br, 0).unwrap(), -2);
    }

    #[test]
    fn dc_diff_v1v2_chroma_uses_chroma_table() {
        // Chroma size 0 = code `00` (2 bits) → DC 0, no value bits.
        // block_idx 4 selects the chroma table; if the luma table were
        // (wrongly) used, `00` is not a valid luma prefix → would error
        // or decode to a different size. Asserting 0 confirms the chroma
        // table is bound for chroma blocks.
        let bytes = pack(&[(0b00, 2), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v1v2(&mut br, 4).unwrap(), 0);

        // Chroma size 1 = code `01`, value bit `1` → +1.
        let bytes = pack(&[(0b01, 2), (0b1, 1), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v1v2(&mut br, 5).unwrap(), 1);
    }

    #[test]
    fn dc_diff_v1v2_tables_distinct_from_mpeg4_p2_annex_b() {
        // spec/16 §2: the v1/v2 binary luma size-0 code is `100`, whereas
        // the MPEG-4-P2 Annex-B luma size-0 code (DC_SIZE_LUMA_TABLE) is
        // `011`. Decoding the binary `100` through the v1/v2 path must
        // give size 0 (DC 0); the same bits through the legacy path would
        // NOT decode as size 0. This locks in that the two table sets are
        // genuinely different and the v1/v2 path binds the binary tables.
        let bytes = pack(&[(0b100, 3), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff_v1v2(&mut br, 0).unwrap(), 0);

        // The MPEG-4-P2 Annex-B luma table decodes `011` as size 0.
        let bytes = pack(&[(0b011, 3), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_intra_dc_diff(&mut br, 0).unwrap(), 0);
    }
}
