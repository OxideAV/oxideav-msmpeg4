//! v3 joint MCBPCY (MB-type + coded-block-pattern) VLC decode.
//!
//! MSMPEG4 v3 replaces H.263's separate MCBPC + CBPY with a single
//! 128-entry canonical-Huffman alphabet jointly coding MB-type and the
//! 6-bit per-block CBP pattern (4 luma + 2 chroma).
//!
//! * Source: `docs/video/msmpeg4/tables/region_05eac8.csv` — 128 records
//!   of `(bit_length, code_value)` consumed by the decoder at
//!   `0x1c21782f` through descriptor slot `[esi+0x8f8]`.
//! * Provenance: `docs/video/msmpeg4/spec/99-current-understanding.md`
//!   §3.1, §8.1 (the extractor CSV has a label-swap noted in the
//!   `.meta` — bytes are correct, role labelling is the artefact).
//! * Kraft sum over the 128 payload bit-lengths = 1 (spec/99 §3.1),
//!   confirming a complete canonical prefix code.
//! * Partition: `count_B = 64` splits the alphabet in two halves —
//!   indices 0..63 are I-type MBs, 64..127 are P-type (spec/99 §3.1
//!   "the 64 matches patent Table 1's I/P index split").
//!
//! The `code_value` column of the source CSV is **not** the
//! bit-pattern of the Huffman code (values exceed `2^bit_length - 1`
//! for most entries). It's a state / LUT byte consumed by the
//! downstream walker (spec/04 §1.3 pri_B semantics, §4.4). Its exact
//! role for the joint-MCBPCY table is not documented in the clean-room
//! spec and we don't need it to recover the joint index — canonical
//! Huffman codes are fully determined by the length array alone.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::tables::CBPY_INTRA_TABLE;
use crate::tables_data::{
    MB_TYPE_V1_INFO, MCBPCY_V3_PARTITION, MCBPCY_V3_RAW, MCBPC_V1_RAW, MCBPC_V2_RAW,
};
use crate::vlc::{self, VlcEntry};

/// Canonical-Huffman VLC table built from `MCBPCY_V3_RAW` bit lengths.
/// `value` = joint symbol index 0..127.
///
/// Build algorithm (canonical Huffman, MSB-first):
///   1. Enumerate symbols in `(bit_length, symbol_index)` ascending order.
///   2. Assign `code = 0` to the first symbol.
///   3. For each subsequent symbol: `code = (code + 1) << (bl_cur - bl_prev)`.
static MCBPCY_V3_TABLE: std::sync::OnceLock<Vec<VlcEntry<u8>>> = std::sync::OnceLock::new();

fn build_table() -> Vec<VlcEntry<u8>> {
    // Symbols whose bit_length is zero are "not present" — filter them out
    // so they never match a decoder peek.
    let mut symbols: Vec<(u32, u8)> = MCBPCY_V3_RAW
        .iter()
        .enumerate()
        .filter_map(
            |(idx, &(bl, _code))| {
                if bl == 0 {
                    None
                } else {
                    Some((bl, idx as u8))
                }
            },
        )
        .collect();

    // Canonical order: primary key = bit_length ascending, secondary = symbol index.
    symbols.sort_by_key(|&(bl, idx)| (bl, idx));

    let mut entries: Vec<VlcEntry<u8>> = Vec::with_capacity(symbols.len());
    let mut code: u32 = 0;
    let mut prev_bl: u32 = 0;
    for (i, &(bl, idx)) in symbols.iter().enumerate() {
        if i == 0 {
            code = 0;
        } else {
            code = (code + 1) << (bl - prev_bl);
        }
        entries.push(VlcEntry::new(bl as u8, code, idx));
        prev_bl = bl;
    }
    entries
}

fn table() -> &'static [VlcEntry<u8>] {
    MCBPCY_V3_TABLE.get_or_init(build_table)
}

/// Decoded MCBPCY packet: joint MB-type + 6-bit CBP pattern.
#[derive(Clone, Copy, Debug)]
pub struct McbpcyDecode {
    /// Raw joint symbol index 0..127.
    pub idx: u8,
    /// True if this MB is intra-coded. The 128-entry joint-MCBPCY
    /// alphabet is partitioned at 64: **indices 0..63 are I-type
    /// (intra) MBs, 64..127 are P-type (inter) MBs** — established by
    /// patent US 6,563,953 Table 1 (`audit/02` §1.4, read directly from
    /// the patent: "indices 0..63 are MB type = I; 64..127 are MB type
    /// = P") and corroborated by the table's own `(128, 64)`
    /// partition-boundary header (`audit/02` §4) baked into
    /// `MCBPCY_V3_PARTITION`. So `is_intra` is the **low** half
    /// (`idx < 64`).
    ///
    /// This also matches the binary's `test bl,0x40; je 0x1c2178bd`
    /// (spec/05 §3.2): `je` is taken when `test` yields zero, i.e. when
    /// bit 6 is **clear** (`idx < 64`), and the jump target `1c2178bd`
    /// is the branch that sets `MB-type = 3` (intra-in-P). The earlier
    /// reading of this slot as "high half = intra" inverted the `je`
    /// polarity (jump-on-zero) and pre-dated the `audit/02` patent
    /// extraction that resolved the spec/99 §3.1 ambiguity.
    pub is_intra: bool,
    /// 4-bit mask: bit `i` = 1 if luma block `i` has coded AC. Bits are
    /// stored with block 0 at MSB (bit 3), block 3 at LSB (bit 0).
    pub cbpy: u8,
    /// CBP bit for the Cb chroma block (1 = coded).
    pub cbp_cb: bool,
    /// CBP bit for the Cr chroma block (1 = coded).
    pub cbp_cr: bool,
}

impl McbpcyDecode {
    /// Number of motion vectors a v3 P-frame MB of this type implies:
    /// **0 for intra (I-type) MBs, 1 for inter (P-type) MBs**.
    ///
    /// Unlike the v1/v2 MCBPC alphabet — whose 21-symbol set carries a
    /// distinct INTER4V MB-type 2 with a 4-MV count (spec/16 §3.1,
    /// `V1V2McbpcyDecode::num_motion_vectors`) — the v3 128-entry joint
    /// MCBPCY alphabet (`region_05eac8`, patent US 6,563,953 Table 1,
    /// `audit/02` §4) partitions only into 64 I-type + 64 P-type entries
    /// (2 MB-types × 64 CBPCY patterns). It carries **no** INTER4V code,
    /// and no traced v3 bitstream signal (the per-MB driver `1c2131ff`
    /// → MCBPCY `1c21782f` → MV decoder `1c217f5a`, spec/05 §3 / spec/06
    /// §1) selects a 4-MV mode — the v3 MV decoder is invoked exactly
    /// once per inter MB. The v3 *MV decoder body itself* supports the
    /// 4-MV output layout (spec/06 §3.6 "first of four (or one)"), so
    /// only the **trigger** is missing.
    ///
    /// This is a documented hard docs gap (#1895): until the docs resolve
    /// where (or whether) v3 signals INTER4V, the v3 picture decoder is
    /// 1-MV-per-MB by construction, and this accessor is the single
    /// authoritative source of that count. A future round that resolves
    /// the trigger changes this method (and only this method) to return
    /// 4 for the 4-MV-coded MBs.
    #[must_use]
    pub const fn num_motion_vectors(&self) -> u8 {
        if self.is_intra {
            0
        } else {
            1
        }
    }
}

/// Decode one MCBPCY joint symbol from the bit stream using the v3
/// 128-entry canonical-Huffman table, and split the resulting index
/// into the (MB-type, 6-bit CBP) components.
///
/// Per `docs/video/msmpeg4/spec/05-ab0-resolution.md` §3.2 and patent
/// US 6,563,953 Table 1 (`audit/02` §1.4): the alphabet is partitioned
/// at 64 — indices 0..63 are I-type (intra) MBs, 64..127 are P-type
/// (inter). The binary's `test bl, 0x40; je 0x1c2178bd` jumps to the
/// intra (`MB-type = 3`) branch when bit 6 is **clear** (`idx < 64`),
/// so `is_intra = idx < MCBPCY_V3_PARTITION`.
///
/// The 6-bit CBP encoding: bit 5 = Y0, bit 4 = Y1, bit 3 = Y2,
/// bit 2 = Y3, bit 1 = Cb, bit 0 = Cr. (Bits assigned MSB-first in the
/// order the spec lists the storage offsets `[esi+0x14..+0x28]`.)
pub fn decode_mcbpcy(br: &mut BitReader<'_>) -> Result<McbpcyDecode> {
    let idx = vlc::decode(br, table())?;
    if idx as usize >= MCBPCY_V3_RAW.len() {
        return Err(Error::invalid(format!(
            "msmpeg4v3 mcbpcy: decoded index {idx} >= alphabet 128"
        )));
    }
    // Partition test: indices 0..63 (bit 6 clear) are I-type / intra;
    // 64..127 are P-type / inter (patent Table 1, audit/02 §1.4).
    let is_intra = (idx as usize) < MCBPCY_V3_PARTITION;
    let pattern = idx & 0x3f; // low 6 bits = CBP
    let cbpy = (pattern >> 2) & 0xf;
    let cbp_cb = (pattern & 0b10) != 0;
    let cbp_cr = (pattern & 0b01) != 0;
    Ok(McbpcyDecode {
        idx,
        is_intra,
        cbpy,
        cbp_cb,
        cbp_cr,
    })
}

/// P-frame MCBPCY parse: first read a 1-bit `skip` flag; if skipped,
/// the MB is copy-from-reference (no residual, zero MV). Otherwise
/// decode the same 128-entry joint VLC as the I-frame path.
///
/// Per spec/05 §3.2: on P-frames, `[edi+0x88] != 0` gates the 1-bit
/// skip read at `1c21786c..1c217877`. For I-frames (`[edi+0x88] == 0`)
/// the path skips directly to the joint-VLC decode, which is what
/// [`decode_mcbpcy`] implements. We split the P-frame variant here so
/// the caller can branch on the skip-flag before doing any per-MB
/// work.
pub enum PFrameMcbpcy {
    /// MB is skipped: duplicate the reference MB verbatim, using a
    /// zero MV. This is the "MB-coded=0" branch.
    Skip,
    /// Non-skipped MB: the decoded joint symbol + post-VLC ac_pred bit
    /// (the binary reads the bit at `1c2178c4..1c2178cf` regardless of
    /// bit-6 result; the ac_pred flag is consumed only for intra-in-P).
    Coded { decode: McbpcyDecode, ac_pred: bool },
}

/// Read the P-frame MB skip flag, then (if not skipped) decode the
/// joint MCBPCY + the post-VLC `ac_pred` bit.
pub fn decode_mcbpcy_pframe(br: &mut BitReader<'_>) -> Result<PFrameMcbpcy> {
    let skip = br.read_bit()?;
    if skip {
        return Ok(PFrameMcbpcy::Skip);
    }
    let decode = decode_mcbpcy(br)?;
    // ac_pred bit is read after the joint VLC; it is meaningful only
    // for intra-in-P MBs, but the decoder always consumes the bit
    // regardless so subsequent parsing stays aligned.
    let ac_pred = br.read_bit()?;
    Ok(PFrameMcbpcy::Coded { decode, ac_pred })
}

// =====================================================================
// v1 / v2 MCBPCY decoders (separate MCBPC + CBPY tables — H.263 lineage)
// =====================================================================
//
// Per spec/07 §1 (`0x1c2171c7`) and §2 (`0x1c21729c`) the v1 / v2
// MB-header decoders read TWO independent canonical-Huffman codes:
//
//   1. MCBPC (chroma CBP + MB-type, jointly coded): 21 entries (v1) or
//      8 entries (v2). v1 uses table at VMA 0x1c253d40 with max-bitlen 9;
//      v2 uses table at VMA 0x1c254140 with max-bitlen 7. Both go through
//      the canonical-Huffman walker `0x1c215811`.
//   2. CBPY (4-bit luma CBP, plus a one's-complement wrap): 16 entries
//      with max-bitlen 6 from the SHARED table at VMA 0x1c254240. Used
//      identically by both v1 and v2.
//
// The decoder body decomposes the joint MCBPC index `idx` into:
//   * MB-type = `idx >> 2`     (range 0..5; H.263 Annex B Table 8)
//   * CBPC    = `idx & 3`      (Cb in bit 1, Cr in bit 0)
//
// Then reads CBPY (with the 15-CBPY one's-complement wrap applied per
// spec/07 §1.2 except for the v2 intra-in-P sub-type per §2.5).
//
// v2 additionally reads a 1-bit `ac_pred` flag immediately after the
// CBPY decode, but only when the MCBPC index ≥ 4 (intra-in-P). This is
// a v2 innovation over v1; v1 has no AC prediction at all.
//
// References:
// * `docs/video/msmpeg4/spec/07-remaining-opens.md` §1 (v1) and §2 (v2)
// * `docs/video/msmpeg4/spec/99-current-understanding.md` §3.1
// * `docs/video/msmpeg4/tables/region_053140.hex` (LUT byte source)

/// Build a `Vec<VlcEntry<u8>>` from the `(symbol, bit_length, code)`
/// triples emitted by `build.rs`. The triples are already sorted by
/// symbol; the runtime just needs them in any order so the linear-scan
/// decoder can match.
fn build_vlc_from_triples(triples: &[(u8, u8, u32)]) -> Vec<VlcEntry<u8>> {
    triples
        .iter()
        .map(|&(sym, bl, code)| VlcEntry::new(bl, code, sym))
        .collect()
}

static MCBPC_V1_TABLE: std::sync::OnceLock<Vec<VlcEntry<u8>>> = std::sync::OnceLock::new();
static MCBPC_V2_TABLE: std::sync::OnceLock<Vec<VlcEntry<u8>>> = std::sync::OnceLock::new();

fn mcbpc_v1_table() -> &'static [VlcEntry<u8>] {
    MCBPC_V1_TABLE.get_or_init(|| build_vlc_from_triples(MCBPC_V1_RAW))
}

fn mcbpc_v2_table() -> &'static [VlcEntry<u8>] {
    MCBPC_V2_TABLE.get_or_init(|| build_vlc_from_triples(MCBPC_V2_RAW))
}

/// Decoded v1 / v2 MB-header packet. The output layout matches v3 (per
/// spec/07 §1.4) so downstream block-decode can consume it uniformly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct V1V2McbpcyDecode {
    /// Skip-MB flag (always read in P-frames; absent for v1 I-frames
    /// which are the inverse case but the wrapping `decode_v1_iframe`
    /// path doesn't read a skip bit either way).
    pub skip: bool,
    /// MB-type 0..5 per H.263 Annex B Table 8 / MPEG-4 Part 2 Table B-10.
    /// In MS-MPEG4 lineage: 0..3 are inter sub-types (with various
    /// quantiser / motion modes), 3 is intra (v3 calls this
    /// "intra-in-P"), 4 is intra-with-quant, 5 is reserved.
    /// Value `0` is also returned on a skipped MB.
    pub mb_type: u8,
    /// True if this is an intra MB. Per the authoritative v1 MB-type
    /// table (`MB_TYPE_V1_INFO`, spec/16 §3.2) both mb_type 3 (INTRA)
    /// and mb_type 4 (INTRA+Q) are intra; the previous `mb_type == 3`
    /// test mis-classified INTRA+Q as inter.
    pub is_intra: bool,
    /// Number of motion vectors the MB-type implies: 1 for INTER /
    /// INTER+Q (mb_type 0, 1), 4 for INTER4V (mb_type 2), 0 for the
    /// intra mb_types (3, 4) and skip MBs. Sourced from
    /// `MB_TYPE_V1_INFO` (spec/16 §3.1's {1, 1, 4, 0, 0} map) for v1;
    /// the v2 8-symbol alphabet emits only mb_type ∈ {0, 3} so its
    /// inter MBs are always 1-MV.
    pub num_motion_vectors: u8,
    /// 4-bit luma CBPY (post one's-complement wrap). Bit `i` (MSB-first
    /// at bit 3 = Y0) is 1 iff luma block `i` has coded AC.
    pub cbpy: u8,
    /// CBP bit for Cb chroma block.
    pub cbp_cb: bool,
    /// CBP bit for Cr chroma block.
    pub cbp_cr: bool,
    /// AC-prediction flag — v2 only, set when MB is intra-in-P.
    /// Always `false` for v1 (which lacks AC prediction).
    pub ac_pred: bool,
}

impl V1V2McbpcyDecode {
    /// Construct an all-zero "skip MB" decode result. Mirrors the
    /// `1c2171f4..1c217204` zero-fill epilogue of the v1 path and the
    /// equivalent in v2.
    fn skip_mb() -> Self {
        Self {
            skip: true,
            mb_type: 0,
            is_intra: false,
            num_motion_vectors: 0,
            cbpy: 0,
            cbp_cb: false,
            cbp_cr: false,
            ac_pred: false,
        }
    }
}

/// Decode CBPY using the existing intra-orientation table. The 4-bit
/// pattern returned by the VLC is the **inverted** form per H.263
/// §5.3.5; per spec/07 §1.2 the v1 decoder applies the
/// `15 - cbpy_raw` wrap unconditionally, and v2 applies the same wrap
/// EXCEPT when the MCBPC sub-type (`mcbpc % 4`) is exactly 3.
///
/// We always return the **post-wrap** value; callers that need the
/// raw VLC output can XOR back with 0xf.
fn decode_cbpy_with_wrap(br: &mut BitReader<'_>, apply_wrap: bool) -> Result<u8> {
    let raw = vlc::decode(br, CBPY_INTRA_TABLE)?;
    if raw > 15 {
        return Err(Error::invalid(format!(
            "msmpeg4 v1/v2 cbpy: decoded {raw} > 15 (table corruption?)"
        )));
    }
    let val = if apply_wrap { 15 - raw } else { raw };
    Ok(val)
}

/// Public entry point for the raw CBPY VLC decode (no `15 - raw` wrap).
///
/// Returns the literal 4-bit CBPY pattern read from the bitstream. The
/// runtime v1/v2 decoder bodies (`decode_mcbpcy_v1` / `decode_mcbpcy_v2`)
/// always apply the wrap per spec/07 §1.2 / §2.5; this raw form is
/// exposed for integration tests that need to cross-check the CBPY VLC
/// codes against an external table (e.g. the binary-derived
/// `CBPY_V1_V2_RAW` triples in `tables_data`).
pub fn decode_cbpy_no_wrap(br: &mut BitReader<'_>) -> Result<u8> {
    decode_cbpy_with_wrap(br, false)
}

/// Decode one v1 MCBPCY packet from the bitstream.
///
/// Sequence (matching spec/07 §1.1-§1.4, decoder body at
/// `0x1c2171c7..0x1c217287`):
///
///   1. Read 1-bit skip flag. If set, return [`V1V2McbpcyDecode::skip_mb`].
///   2. Decode MCBPC (≤9-bit canonical Huffman, table at VMA
///      `0x1c253d40`). Result must be in `[0, 20]`; otherwise error.
///   3. Decode CBPY (≤6-bit canonical Huffman, shared table at VMA
///      `0x1c254240`). Apply the `15 - cbpy_raw` one's-complement wrap.
///   4. Decompose: `mb_type = mcbpc >> 2`, CBPC bits = `mcbpc & 3`.
///
/// **No AC-prediction bit** — v1 lacks this MPEG-4 feature.
pub fn decode_mcbpcy_v1(br: &mut BitReader<'_>) -> Result<V1V2McbpcyDecode> {
    // Skip flag (the H.263 COD bit). v1 always reads this regardless of
    // I-frame vs P-frame (per spec/07 §1.5: "very first operation is a
    // 1-bit read").
    let skip = br.read_bit()?;
    if skip {
        return Ok(V1V2McbpcyDecode::skip_mb());
    }
    let mcbpc = vlc::decode(br, mcbpc_v1_table())?;
    if mcbpc > 20 {
        return Err(Error::invalid(format!(
            "msmpeg4 v1 mcbpc: decoded {mcbpc} > 20 (range check at 1c217224)"
        )));
    }
    // Symbol 20 is the STUFFING/ESC entry (`MB_TYPE_V1_INFO[20].0 ==
    // 0xff`). It carries no MB-type / CBP payload; the v1 decoder body
    // never produces a valid MB from it, so reject it explicitly rather
    // than feeding a bogus mb_type into the dispatcher.
    let (mb_type, is_intra, num_mv) = MB_TYPE_V1_INFO[mcbpc as usize];
    if mb_type == 0xff {
        return Err(Error::invalid(
            "msmpeg4 v1 mcbpc: decoded the STUFFING/ESC symbol (20); \
             it is not a valid macroblock type (spec/16 §3.2)"
                .to_string(),
        ));
    }
    let cbpy = decode_cbpy_with_wrap(br, true)?;
    let cbpc = mcbpc & 0b11;
    Ok(V1V2McbpcyDecode {
        skip: false,
        mb_type,
        is_intra,
        num_motion_vectors: num_mv,
        cbpy,
        cbp_cb: (cbpc & 0b10) != 0,
        cbp_cr: (cbpc & 0b01) != 0,
        ac_pred: false,
    })
}

/// Frame-type context for the v2 MCBPCY decoder.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum V2FrameType {
    /// I-frame: skip the leading skip-bit read (I-frames cannot have
    /// skipped MBs). Per spec/07 §2.1: `cmp [ebx+0x88], 0; je`.
    I,
    /// P-frame: read 1-bit skip flag first.
    P,
}

/// Decode one v2 MCBPCY packet. Per spec/07 §2 (`0x1c21729c`):
///
///   1. P-frame only: read skip bit; if set, return skip MB.
///   2. Decode MCBPC (≤7-bit canonical Huffman, table at VMA
///      `0x1c254140`). Range `[0, 7]`.
///   3. Compute `quotient = mcbpc / 4`, `remainder = mcbpc % 4`.
///      * `quotient == 0` → inter MB: CBPC = remainder, mb_type = 0,
///        decode CBPY with wrap (unless `remainder == 3`, in which case
///        skip the wrap per spec/07 §2.5).
///      * `quotient == 1` → intra-in-P: mb_type = 3, read 1-bit
///        AC-prediction flag, decode CBPY with wrap.
///   4. Decompose CBPC into Cb / Cr bits.
pub fn decode_mcbpcy_v2(
    br: &mut BitReader<'_>,
    frame_type: V2FrameType,
) -> Result<V1V2McbpcyDecode> {
    // P-frame: leading 1-bit skip read. I-frame: no skip bit.
    let skip = match frame_type {
        V2FrameType::P => br.read_bit()?,
        V2FrameType::I => false,
    };
    if skip {
        return Ok(V1V2McbpcyDecode::skip_mb());
    }
    let mcbpc = vlc::decode(br, mcbpc_v2_table())?;
    if mcbpc > 7 {
        return Err(Error::invalid(format!(
            "msmpeg4 v2 mcbpc: decoded {mcbpc} > 7 (range check at 1c217318)"
        )));
    }
    let quotient = mcbpc >> 2;
    let remainder = mcbpc & 0b11;

    let (mb_type, is_intra, ac_pred, apply_wrap) = match quotient {
        0 => {
            // Inter MB — wrap CBPY unless remainder == 3 (spec/07 §2.5).
            let wrap = remainder != 3;
            (0u8, false, false, wrap)
        }
        1 => {
            // Intra-in-P — read AC-prediction flag, then CBPY with wrap.
            let ac = br.read_bit()?;
            (3u8, true, ac, true)
        }
        _ => {
            return Err(Error::invalid(format!(
                "msmpeg4 v2 mcbpc: quotient {quotient} != {{0, 1}} from \
                 mcbpc={mcbpc} (out-of-range per spec/07 §2.4)"
            )));
        }
    };

    let cbpy = decode_cbpy_with_wrap(br, apply_wrap)?;
    // The v2 8-symbol P-frame MCBPC alphabet only emits mb_type ∈ {0, 3}
    // (quotient 0 = inter, quotient 1 = intra-in-P) — there is no INTER4V
    // code in the 8-symbol set (spec/16 §3.3). So inter MBs are always
    // 1-MV and intra MBs decode no MV.
    let num_motion_vectors = if is_intra { 0 } else { 1 };
    Ok(V1V2McbpcyDecode {
        skip: false,
        mb_type,
        is_intra,
        num_motion_vectors,
        cbpy,
        cbp_cb: (remainder & 0b10) != 0,
        cbp_cr: (remainder & 0b01) != 0,
        ac_pred,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_table_has_128_entries() {
        // All 128 symbols have non-zero bit_length per spec/99 §3.1
        // ("Kraft's sum is exactly 1 over the 128 payload bit-lengths").
        let t = table();
        assert_eq!(t.len(), 128);
    }

    #[test]
    fn canonical_codes_respect_kraft() {
        // If the canonical builder is correct, no two entries share a
        // prefix — a runtime decode can't be ambiguous. Verify by
        // checking that shorter codes are never prefixes of longer ones.
        let t = table();
        for (i, a) in t.iter().enumerate() {
            for (j, b) in t.iter().enumerate() {
                if i == j || a.bits == b.bits {
                    continue;
                }
                let (short, long) = if a.bits < b.bits { (a, b) } else { (b, a) };
                let shift = long.bits - short.bits;
                let long_prefix = long.code >> shift;
                assert_ne!(
                    long_prefix,
                    short.code,
                    "canonical Huffman: {:0width$b} is a prefix of {:0longw$b} (syms {}, {})",
                    short.code,
                    long.code,
                    short.value,
                    long.value,
                    width = short.bits as usize,
                    longw = long.bits as usize,
                );
            }
        }
    }

    #[test]
    fn canonical_round_trip_per_symbol() {
        // Every symbol should round-trip: re-pack its code, decode back.
        let t = table();
        for e in t {
            // Build a byte-aligned bit stream of the code followed by zero tail.
            let mut acc: u64 = 0;
            let mut bits: u32 = 0;
            acc = (acc << e.bits) | (e.code as u64);
            bits += e.bits as u32;
            let mut out = Vec::new();
            while bits >= 8 {
                let shift = bits - 8;
                out.push(((acc >> shift) & 0xff) as u8);
                acc &= (1u64 << shift) - 1;
                bits -= 8;
            }
            if bits > 0 {
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            // Tail padding so peek never starves.
            out.extend_from_slice(&[0u8; 8]);
            let mut br = BitReader::new(&out);
            let decoded = decode_mcbpcy(&mut br).unwrap();
            assert_eq!(
                decoded.idx,
                e.value,
                "symbol {} failed round-trip (code {:0w$b}, bits {})",
                e.value,
                e.code,
                e.bits,
                w = e.bits as usize,
            );
            // CBP must be within 6 bits of idx.
            assert_eq!(decoded.cbpy, ((e.value & 0x3f) >> 2) & 0xf);
        }
    }

    #[test]
    fn mcbpcy_cbp_bits_split_correctly() {
        // Manual construction: pretend the decoded idx is 0b00101101 (=45).
        // low 6 bits = 0b101101: Y0=1 Y1=0 Y2=1 Y3=1 Cb=0 Cr=1 → cbpy=0b1011, cb=false, cr=true.
        let idx = 45u8;
        let pattern = idx & 0x3f;
        let cbpy = (pattern >> 2) & 0xf;
        assert_eq!(cbpy, 0b1011);
        assert_eq!(pattern & 0b10, 0); // Cb = 0
        assert_ne!(pattern & 0b01, 0); // Cr = 1
    }

    #[test]
    fn v3_mcbpcy_partition_low_half_is_intra() {
        // Patent US 6,563,953 Table 1 (audit/02 §1.4): joint-MCBPCY
        // indices 0..63 are MB type = I (intra), 64..127 are MB type = P
        // (inter). Decode every symbol via the canonical round-trip and
        // assert `is_intra` is exactly the low-half membership. This pins
        // the partition polarity so a future regression cannot silently
        // re-invert it (the pre-correction code had `is_intra = idx >=
        // 64`, the inverse, mis-classifying every coded v3 P-frame MB).
        assert_eq!(MCBPCY_V3_PARTITION, 64);
        let t = table();
        // Track that both halves are actually exercised.
        let mut saw_intra = false;
        let mut saw_inter = false;
        for e in t {
            // Same byte-packing as `canonical_round_trip_per_symbol`.
            let mut acc: u64 = 0;
            let mut bits: u32 = 0;
            acc = (acc << e.bits) | (e.code as u64);
            bits += e.bits as u32;
            let mut out = Vec::new();
            while bits >= 8 {
                let shift = bits - 8;
                out.push(((acc >> shift) & 0xff) as u8);
                acc &= (1u64 << shift) - 1;
                bits -= 8;
            }
            if bits > 0 {
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            out.extend_from_slice(&[0u8; 8]);
            let mut br = BitReader::new(&out);
            let dec = decode_mcbpcy(&mut br).expect("symbol decodes");
            assert_eq!(
                dec.is_intra,
                (dec.idx as usize) < MCBPCY_V3_PARTITION,
                "idx {} is_intra polarity must match the patent low-half \
                 (0..63 = intra)",
                dec.idx,
            );
            if dec.is_intra {
                saw_intra = true;
            } else {
                saw_inter = true;
            }
        }
        assert!(saw_intra, "no intra (low-half) symbol exercised");
        assert!(saw_inter, "no inter (high-half) symbol exercised");
    }

    #[test]
    fn v3_mcbpcy_num_motion_vectors_is_one_for_every_inter_symbol() {
        // The v3 128-entry joint MCBPCY alphabet carries only an
        // I-type/P-type split (patent 6,563,953 Table 1, audit/02 §4) and
        // no INTER4V code; no traced v3 signal selects 4-MV (docs gap
        // #1895). So `McbpcyDecode::num_motion_vectors()` must be 0 for
        // every low-half (intra) symbol and exactly 1 for every high-half
        // (inter) symbol — never 4. This pins the v3 1-MV-per-MB invariant
        // so a future round cannot add a 4-MV path to v3 without first
        // resolving the trigger (which would change this accessor).
        let t = table();
        let mut saw_one = false;
        let mut saw_zero = false;
        for e in t {
            let mut acc: u64 = (e.code as u64) & ((1u64 << e.bits) - 1);
            let mut bits = e.bits as u32;
            let mut out = Vec::new();
            while bits >= 8 {
                let shift = bits - 8;
                out.push(((acc >> shift) & 0xff) as u8);
                acc &= (1u64 << shift) - 1;
                bits -= 8;
            }
            if bits > 0 {
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            out.extend_from_slice(&[0u8; 8]);
            let mut br = BitReader::new(&out);
            let dec = decode_mcbpcy(&mut br).expect("symbol decodes");
            let expected = if dec.is_intra { 0 } else { 1 };
            assert_eq!(
                dec.num_motion_vectors(),
                expected,
                "idx {} num_motion_vectors must be {expected} (v3 is \
                 1-MV-per-MB; docs gap #1895)",
                dec.idx,
            );
            assert_ne!(
                dec.num_motion_vectors(),
                4,
                "v3 must never imply 4 MVs (no INTER4V trigger; gap #1895)"
            );
            if dec.is_intra {
                saw_zero = true;
            } else {
                saw_one = true;
            }
        }
        assert!(saw_zero, "no intra (0-MV) symbol exercised");
        assert!(saw_one, "no inter (1-MV) symbol exercised");
    }

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
        // Tail padding so the bit-reader's max-bitlen peek never starves.
        out.extend_from_slice(&[0u8; 4]);
        out
    }

    /// Find the (sym, bl, code) triple for a given symbol in v1 MCBPC.
    fn v1_mcbpc_for(sym: u8) -> (u8, u32) {
        let &(_, bl, code) = MCBPC_V1_RAW
            .iter()
            .find(|&&(s, _, _)| s == sym)
            .expect("v1 MCBPC: symbol not in table");
        (bl, code)
    }

    fn v2_mcbpc_for(sym: u8) -> (u8, u32) {
        let &(_, bl, code) = MCBPC_V2_RAW
            .iter()
            .find(|&&(s, _, _)| s == sym)
            .expect("v2 MCBPC: symbol not in table");
        (bl, code)
    }

    /// CBPY canonical-Huffman code for a given 4-bit pattern (intra
    /// orientation). Uses the existing tables module.
    fn cbpy_for(intra_pattern: u8) -> (u8, u32) {
        let e = CBPY_INTRA_TABLE
            .iter()
            .find(|e| e.value == intra_pattern)
            .expect("CBPY: pattern not in table");
        (e.bits, e.code)
    }

    #[test]
    fn v1_mcbpcy_decodes_skip_mb() {
        // Skip bit = 1, no further reads.
        let bytes = pack(&[(1, 1)]);
        let mut br = BitReader::new(&bytes);
        let dec = decode_mcbpcy_v1(&mut br).unwrap();
        assert!(dec.skip);
        assert_eq!(dec.mb_type, 0);
        assert_eq!(dec.cbpy, 0);
        assert!(!dec.cbp_cb && !dec.cbp_cr);
        assert!(!dec.ac_pred);
    }

    #[test]
    fn v1_mcbpcy_decodes_inter_mb_with_cbpy() {
        // Skip = 0, MCBPC sym 0 (most common — bl=1 code=`1` per
        // extracted table), CBPY raw = 15 → wrap → 0; pattern 15 in
        // CBPY_INTRA is bl=2 code=`11`.
        // After: mb_type = 0 >> 2 = 0 (inter), CBPC bits = 0&3 = 0,
        // CBPY raw 15 → wrap → 0 (no luma blocks coded).
        let (mc_bl, mc_code) = v1_mcbpc_for(0);
        let (cb_bl, cb_code) = cbpy_for(15);
        let bytes = pack(&[(0, 1), (mc_code, mc_bl as u32), (cb_code, cb_bl as u32)]);
        let mut br = BitReader::new(&bytes);
        let dec = decode_mcbpcy_v1(&mut br).unwrap();
        assert!(!dec.skip);
        assert_eq!(dec.mb_type, 0);
        assert!(!dec.is_intra);
        assert_eq!(dec.cbpy, 0, "CBPY post-wrap should be 15 - 15 = 0");
        assert!(!dec.cbp_cb && !dec.cbp_cr);
    }

    #[test]
    fn v1_mcbpcy_intra_mb_type_3() {
        // Pick MCBPC sym 12 (decimal): mb_type = 12 >> 2 = 3 (intra).
        // CBPC = 12 & 3 = 0. CBPY raw 15 → wrap to 0.
        let (mc_bl, mc_code) = v1_mcbpc_for(12);
        let (cb_bl, cb_code) = cbpy_for(15);
        let bytes = pack(&[(0, 1), (mc_code, mc_bl as u32), (cb_code, cb_bl as u32)]);
        let mut br = BitReader::new(&bytes);
        let dec = decode_mcbpcy_v1(&mut br).unwrap();
        assert!(!dec.skip);
        assert_eq!(dec.mb_type, 3);
        assert!(dec.is_intra);
        assert_eq!(dec.cbpy, 0);
        assert!(!dec.ac_pred, "v1 has no AC prediction");
    }

    #[test]
    fn v2_mcbpcy_pframe_skip() {
        let bytes = pack(&[(1, 1)]);
        let mut br = BitReader::new(&bytes);
        let dec = decode_mcbpcy_v2(&mut br, V2FrameType::P).unwrap();
        assert!(dec.skip);
    }

    #[test]
    fn v2_mcbpcy_iframe_no_skip_bit_consumed() {
        // I-frame: no skip bit. Decode MCBPC sym 0 directly.
        let (mc_bl, mc_code) = v2_mcbpc_for(0);
        let (cb_bl, cb_code) = cbpy_for(15);
        let bytes = pack(&[(mc_code, mc_bl as u32), (cb_code, cb_bl as u32)]);
        let mut br = BitReader::new(&bytes);
        let dec = decode_mcbpcy_v2(&mut br, V2FrameType::I).unwrap();
        assert!(!dec.skip);
        assert_eq!(dec.mb_type, 0); // sym 0 → quotient=0 → mb_type=0
    }

    #[test]
    fn v2_mcbpcy_intra_in_p_reads_ac_pred() {
        // Pick MCBPC sym 4: quotient = 1 → intra-in-P, mb_type = 3.
        // After MCBPC the decoder reads 1-bit ac_pred BEFORE CBPY.
        let (mc_bl, mc_code) = v2_mcbpc_for(4);
        let (cb_bl, cb_code) = cbpy_for(15);
        // P-frame: skip=0, MCBPC sym 4, ac_pred=1, CBPY pattern 15.
        let bytes = pack(&[
            (0, 1),                  // skip = 0
            (mc_code, mc_bl as u32), // MCBPC
            (1, 1),                  // ac_pred = 1
            (cb_code, cb_bl as u32), // CBPY
        ]);
        let mut br = BitReader::new(&bytes);
        let dec = decode_mcbpcy_v2(&mut br, V2FrameType::P).unwrap();
        assert!(!dec.skip);
        assert_eq!(dec.mb_type, 3);
        assert!(dec.is_intra);
        assert!(dec.ac_pred, "intra-in-P MB must have ac_pred=true");
        assert_eq!(dec.cbpy, 0);
    }

    #[test]
    fn v2_mcbpcy_inter_sub3_skips_cbpy_wrap() {
        // MCBPC sym 3: quotient = 0 (inter), remainder = 3 → CBPY wrap is
        // SKIPPED (spec/07 §2.5). So the decoded CBPY equals the raw VLC
        // value, NOT 15 - raw.
        // Pick CBPY pattern 0 (raw VLC value): bl=4 code=`0011`.
        let (mc_bl, mc_code) = v2_mcbpc_for(3);
        let (cb_bl, cb_code) = cbpy_for(0);
        let bytes = pack(&[(0, 1), (mc_code, mc_bl as u32), (cb_code, cb_bl as u32)]);
        let mut br = BitReader::new(&bytes);
        let dec = decode_mcbpcy_v2(&mut br, V2FrameType::P).unwrap();
        assert!(!dec.skip);
        assert_eq!(dec.mb_type, 0);
        assert_eq!(
            dec.cbpy, 0,
            "remainder=3 path must skip the 15-wrap; raw=0 → cbpy=0"
        );
    }

    #[test]
    fn v1_mcbpc_round_trip_every_symbol() {
        // Every (sym, bl, code) in MCBPC_V1_RAW must round-trip through
        // the linear-scan VLC decoder and decompose exactly as the
        // authoritative `MB_TYPE_V1_INFO` table says (spec/16 §3).
        for &(sym, bl, code) in MCBPC_V1_RAW {
            let (cb_bl, cb_code) = cbpy_for(15);
            let bytes = pack(&[(0, 1), (code, bl as u32), (cb_code, cb_bl as u32)]);
            let mut br = BitReader::new(&bytes);
            let (mb_type, is_intra, num_mv) = MB_TYPE_V1_INFO[sym as usize];
            if mb_type == 0xff {
                // STUFFING/ESC (symbol 20) is not a valid MB; the decoder
                // must reject it.
                assert!(
                    decode_mcbpcy_v1(&mut br).is_err(),
                    "sym {sym}: STUFFING/ESC must be rejected"
                );
                continue;
            }
            let dec = decode_mcbpcy_v1(&mut br).unwrap();
            assert_eq!(dec.mb_type, sym >> 2, "sym {sym}: mb_type mismatch");
            assert_eq!(dec.mb_type, mb_type, "sym {sym}: table mb_type mismatch");
            assert_eq!(dec.is_intra, is_intra, "sym {sym}: is_intra mismatch");
            assert_eq!(
                dec.num_motion_vectors, num_mv,
                "sym {sym}: num_motion_vectors mismatch"
            );
        }
    }

    #[test]
    fn v1_mbtype_table_pins_inter4v_and_intra_plus_q() {
        // The newly-extracted `region_053140_mbtype.csv` (spec/16 §3)
        // is the value-level closure for the v1 inter sub-types. Pin the
        // two corrections it carries over the old `mb_type == 3` test:
        //   * MB-type 2 (INTER4V) implies 4 motion vectors;
        //   * MB-type 4 (INTRA+Q) is intra (the old test missed this).
        // Symbols 8..=11 are mb_type 2; 16..=19 are mb_type 4.
        for sym in 8u8..=11 {
            let (mb_type, is_intra, num_mv) = MB_TYPE_V1_INFO[sym as usize];
            assert_eq!(mb_type, 2, "sym {sym} should be INTER4V");
            assert!(!is_intra, "sym {sym} INTER4V is not intra");
            assert_eq!(num_mv, 4, "sym {sym} INTER4V is 4-MV");
        }
        for sym in 16u8..=19 {
            let (mb_type, is_intra, num_mv) = MB_TYPE_V1_INFO[sym as usize];
            assert_eq!(mb_type, 4, "sym {sym} should be INTRA+Q");
            assert!(is_intra, "sym {sym} INTRA+Q must be intra");
            assert_eq!(num_mv, 0, "sym {sym} intra decodes no MV");
        }
        // Symbol 20 is the STUFFING/ESC sentinel.
        assert_eq!(MB_TYPE_V1_INFO[20].0, 0xff, "symbol 20 = STUFFING/ESC");
    }

    #[test]
    fn v2_mcbpc_round_trip_every_symbol() {
        for &(sym, bl, code) in MCBPC_V2_RAW {
            // Build the right post-MCBPC sequence depending on quotient.
            let quotient = sym >> 2;
            let remainder = sym & 0b11;
            let (cb_bl, cb_code) = cbpy_for(15);
            let bytes = if quotient == 1 {
                // Intra-in-P: ac_pred bit then CBPY.
                pack(&[(0, 1), (code, bl as u32), (0, 1), (cb_code, cb_bl as u32)])
            } else if remainder == 3 {
                // Inter sub-3 path: no wrap; supply CBPY raw=0.
                let (cbr_bl, cbr_code) = cbpy_for(0);
                pack(&[(0, 1), (code, bl as u32), (cbr_code, cbr_bl as u32)])
            } else {
                pack(&[(0, 1), (code, bl as u32), (cb_code, cb_bl as u32)])
            };
            let mut br = BitReader::new(&bytes);
            let dec = decode_mcbpcy_v2(&mut br, V2FrameType::P).unwrap();
            assert_eq!(
                dec.mb_type,
                if quotient == 1 { 3 } else { 0 },
                "sym {sym}: mb_type mismatch"
            );
            assert_eq!(
                dec.cbp_cb,
                (remainder & 0b10) != 0,
                "sym {sym}: cb mismatch"
            );
            assert_eq!(
                dec.cbp_cr,
                (remainder & 0b01) != 0,
                "sym {sym}: cr mismatch"
            );
        }
    }
}
