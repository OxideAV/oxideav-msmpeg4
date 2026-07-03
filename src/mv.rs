//! MSMPEG4 v3 motion-vector decoder (joint (MVDx, MVDy) VLC + ESC
//! tail + median-of-3 predictor + toroidal clamp).
//!
//! # Decode algorithm (spec/06 §3.1)
//!
//! 1. Compute the per-component predictor `(predX, predY)` as the
//!    median of up to three neighbour MV bytes (left `A`, top `B`,
//!    top-right `C`). Missing neighbours (picture edge) are substituted
//!    with zero byte-for-byte. See [`median_predictor`].
//! 2. Read one joint VLC symbol `idx ∈ [0, 1099]` from
//!    [`MV_V3_RAW`](crate::tables_data::MV_V3_RAW):
//!    * If `idx == 1099` (ESC), read `get_bits(6)` for `MVDx_raw` and
//!      another `get_bits(6)` for `MVDy_raw`. Both are unsigned 6-bit
//!      values in `[0, 63]`.
//!    * Otherwise, `MVDx_raw = MVDX_V3_BYTES[idx]`, `MVDy_raw =
//!      MVDY_V3_BYTES[idx]`.
//! 3. For each component, compute
//!    `mv = MVD_raw + predictor - 32` (the `-32` cancels the `+32` bias
//!    baked into every LUT entry; for ESC raw values this just shifts
//!    the interpretation to signed `[-32, +31]`).
//! 4. Toroidal wrap into `[-63, +63]`: if `mv > 63`, subtract 64; if
//!    `mv < -63`, add 64. One pass suffices because the raw + predictor
//!    sum is bounded by `[-32..=95]` + `[-63..=63]` ≈ `[-95..=158]`,
//!    within one-wrap reach of both endpoints.
//! 5. Output `(MVx, MVy)` as signed bytes.
//!
//! # Clamp bounds
//!
//! The binary stores `(-63, +63)` in `[esi+0xb00..b04]` at DLL init
//! (spec/06 §3.5). These are the half-pel bounds; the integer part is
//! recovered by arithmetic-shift-right by 1 at MC time (spec/04 §3.1:
//! `sar eax, 1` → integer MV; LSB → half-pel fractional).
//!
//! # Alphabet variant selection
//!
//! MSMPEG4 v3 has two MV VLC tables selected by the per-P-frame
//! `mv_table_sel` bit (`[esi+0x834]`):
//!
//! | `mv_table_sel` | VLC source VMA   | MVDx LUT    | MVDy LUT    | Status |
//! | -------------- | ---------------- | ----------- | ----------- | ------ |
//! | 0 (default)    | `0x1c25cbc0`     | `0x1c25ee28`| `0x1c25f278`| **wired** ([`MvTable::Default`]) |
//! | 1 (alternate)  | `0x1c25a0b8`     | `0x1c25c320`| `0x1c25c770`| **wired** ([`MvTable::Alternate`]) |
//!
//! Both variants now decode end-to-end. The alternate VLC source at
//! `0x1c25a0b8` was re-extracted at its full 8804-byte size in
//! Extractor 07 (`docs/video/msmpeg4/spec/16-mv-vlc-dc-mcbpc-extraction.md`
//! §1, `tables/region_0594b8_mvvlc.csv` — 1100 entries, ESC at index
//! 1099, Kraft sum = 1.0, bit-lengths 2..15) — superseding the earlier
//! 256-byte truncation. Both variants decode against their extracted
//! `(bit_length, code)` wire patterns (spec/16 §1 / spec/12 §2), a
//! complete prefix code that is NOT a textbook-canonical assignment; the
//! two `(MVDx, MVDy)` byte LUTs at
//! `0x1c25c320` / `0x1c25c770` (1104 bytes each, bias-32 residuals) are
//! exposed as
//! [`MVDX_V3_ALT_BYTES`](crate::tables_data::MVDX_V3_ALT_BYTES) /
//! [`MVDY_V3_ALT_BYTES`](crate::tables_data::MVDY_V3_ALT_BYTES) and feed
//! the shared [`decode_mv_variant`] body. Streams that use the alternate
//! table include div3.avi frames 37/38/40 and div4.avi frames 1/16 per
//! `memory/project_msmpeg4_runtime_binding_clues.md` §2.1.

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_core::{Error, Result};

use crate::tables_data::{
    MVDX_V3_ALT_BYTES, MVDX_V3_BYTES, MVDY_V3_ALT_BYTES, MVDY_V3_BYTES, MV_V1_V2_BIAS,
    MV_V1_V2_RAW, MV_V3_ALT_RAW, MV_V3_ESC_INDEX, MV_V3_RAW,
};
use crate::vlc::{self, VlcEntry};

/// Per-frame MV VLC variant selector. The picture-header
/// `mv_table_sel` bit picks one of two MV VLC sources for v3 P-frames
/// (see module-level table). Both variants now have a fully-extracted
/// source and decode end-to-end (spec/16 §1 / Extractor 07).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MvTable {
    /// Default variant — VLC source at file `0x5bfc0` / VMA
    /// `0x1c25cbc0` with byte LUTs at `0x1c25ee28` / `0x1c25f278`.
    /// Wired through [`MV_V3_RAW`] / [`MVDX_V3_BYTES`] /
    /// [`MVDY_V3_BYTES`].
    #[default]
    Default,
    /// Alternate variant — VLC source at file `0x594b8` / VMA
    /// `0x1c25a0b8`, byte LUTs at `0x1c25c320` / `0x1c25c770`. Wired
    /// end-to-end: the full 8804-byte VLC source (1100 entries, ESC at
    /// 1099, Kraft = 1.0) was re-extracted in Extractor 07 (spec/16 §1,
    /// `tables/region_0594b8_mvvlc.csv`) and feeds the shared
    /// canonical-Huffman builder via
    /// [`MV_V3_ALT_RAW`](crate::tables_data::MV_V3_ALT_RAW), paired with
    /// [`MVDX_V3_ALT_BYTES`](crate::tables_data::MVDX_V3_ALT_BYTES) /
    /// [`MVDY_V3_ALT_BYTES`](crate::tables_data::MVDY_V3_ALT_BYTES).
    Alternate,
}

impl MvTable {
    /// Resolve a picture-header **`mv_table_sel`** bit ∈ {0, 1} to the
    /// matching [`MvTable`]. Per spec/06 §3.2 and
    /// `docs/video/msmpeg4/spec/01-bitstream-framing.md` §1.4, the
    /// v3-only per-frame slot `[esi+0x834]` is a 1-bit selector: `0`
    /// picks the default joint-MV VLC at VMA `0x1c25cbc0` (paired with
    /// the byte LUTs at `0x1c25ee28` / `0x1c25f278`), `1` picks the
    /// alternate variant at VMA `0x1c25a0b8` (paired with `0x1c25c320`
    /// / `0x1c25c770`).
    ///
    /// Returns `None` for any selector value outside `{0, 1}` — the
    /// picture-header parser ([`crate::header::MsV3PictureHeader::parse`])
    /// already clamps via a single bit read, so this is a
    /// defence-in-depth guard rather than a recoverable error path
    /// (mirrors the shape of [`crate::g_family::GFamily::for_chroma_selector`]
    /// / [`crate::g_family::GFamily::for_luma_selector`]).
    ///
    /// `mv_table_sel` is only meaningfully read on **v3 P-frames** per
    /// spec/01 §1.4 (`1c2120aa`); v1 / v2 paths never consume the bit
    /// and downstream code must substitute
    /// [`MvTable::Default`] (per the v1/v2 → v3-compat default
    /// `mv_table_sel = 0`, see
    /// [`crate::header::MsV1V2PictureHeader::V1_COMPAT_DEFAULTS`] /
    /// [`crate::header::MsV1V2PictureHeader::V2_COMPAT_DEFAULTS`]).
    /// I-frames also never read this bit (it is P-frame-scoped in
    /// `MsV3PictureHeader::parse` per spec/99 §2.3) and the picture
    /// header carries the zero default at `mv_table_sel: 0` so an
    /// I-frame caller that resolves through this helper picks up
    /// [`MvTable::Default`] without a version-specific branch.
    pub const fn from_sel(sel: u8) -> Option<Self> {
        match sel {
            0 => Some(MvTable::Default),
            1 => Some(MvTable::Alternate),
            _ => None,
        }
    }

    /// **Inverse** of [`MvTable::from_sel`]: the picture-header
    /// `mv_table_sel` bit value that dispatches to this variant. Per
    /// spec/06 §3.2: `Default → 0, Alternate → 1`.
    pub const fn to_sel(self) -> u8 {
        match self {
            MvTable::Default => 0,
            MvTable::Alternate => 1,
        }
    }
}

/// Lazy-built prefix-code table for the v3 MV VLC default variant.
/// 1100 symbols (indices 0..=1099); index 1099 is the ESC sentinel.
/// Built from `MV_V3_RAW`'s extracted `(bit_length, code)` pairs.
static MV_V3_TABLE: std::sync::OnceLock<Vec<VlcEntry<u16>>> = std::sync::OnceLock::new();

/// Lazy-built prefix-code table for the v3 MV VLC **alternate** variant
/// (`mv_table_sel == 1`). Same 1100-symbol shape (ESC at 1099), built
/// from `MV_V3_ALT_RAW`'s extracted `(bit_length, code)` pairs via the
/// shared builder (spec/16 §1 / Extractor 07, VMA 0x1c25a0b8).
static MV_V3_ALT_TABLE: std::sync::OnceLock<Vec<VlcEntry<u16>>> = std::sync::OnceLock::new();

/// VLC-table builder shared by both v3 MV VLC variants. The `raw` slice
/// carries `(bit_length, code)` pairs indexed by symbol, where `code` is
/// the **actual DLL wire bit-pattern** (MSB-first) extracted in
/// Extractor 07 (spec/16 §1). These codes form a complete prefix code
/// (Kraft = 1.0) but are NOT a textbook-canonical assignment: spec/12 §2
/// shows the per-slot walker builder is fed the literal `(code, bl)`
/// records (`emit_walker_entry(this, record.code, record.bl, sym, …)`),
/// and spec/12 §3 decodes them MSB-first. So the runtime matches against
/// the extracted `code` directly — the same convention the v1/v2
/// per-component MV table already uses ([`build_v1v2_table`]).
fn build_mv_table(raw: &[(u32, u32)]) -> Vec<VlcEntry<u16>> {
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

fn build_table() -> Vec<VlcEntry<u16>> {
    build_mv_table(MV_V3_RAW)
}

fn build_alt_table() -> Vec<VlcEntry<u16>> {
    build_mv_table(MV_V3_ALT_RAW)
}

fn table() -> &'static [VlcEntry<u16>] {
    MV_V3_TABLE.get_or_init(build_table)
}

fn alt_table() -> &'static [VlcEntry<u16>] {
    MV_V3_ALT_TABLE.get_or_init(build_alt_table)
}

/// Decoded motion-vector in half-pel units. Both components are in
/// the toroidal `[-63, +63]` range (signed 7-bit after wrap).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Mv {
    /// Half-pel X component, `[-63, +63]`.
    pub x: i8,
    /// Half-pel Y component, `[-63, +63]`.
    pub y: i8,
}

/// Median-of-3 predictor over the three neighbour bytes supplied per
/// component. `None` indicates a neighbour that is unavailable
/// (picture edge) — substituted with zero. Matches spec/06 §3.4.
///
/// The algorithm is the textbook three-way median: `median(a, b, c) =
/// a + b + c - min(a, b, c) - max(a, b, c)`. Each component is
/// computed independently.
pub fn median_predictor(left: Option<Mv>, top: Option<Mv>, top_right: Option<Mv>) -> Mv {
    let a = left.unwrap_or_default();
    let b = top.unwrap_or_default();
    let c = top_right.unwrap_or_default();

    fn med(a: i8, b: i8, c: i8) -> i8 {
        let mn = a.min(b).min(c);
        let mx = a.max(b).max(c);
        // `a + b + c - mn - mx` = median. Use i32 to avoid i8 overflow.
        (a as i32 + b as i32 + c as i32 - mn as i32 - mx as i32) as i8
    }

    Mv {
        x: med(a.x, b.x, c.x),
        y: med(a.y, b.y, c.y),
    }
}

/// Decode one joint VLC symbol, looking up the two component
/// residuals via the MVDx/MVDy byte LUTs (or reading raw FLC tails for
/// the ESC path), apply the predictor and the toroidal wrap.
///
/// Returns the final `(MVx, MVy)` byte-pair ready to store in the
/// MB-info row.
///
/// This variant uses the **default** v3 MV VLC ([`MvTable::Default`]).
/// For per-frame dispatch from the picture-header `mv_table_sel` bit
/// use [`decode_mv_with_table`].
pub fn decode_mv(br: &mut BitReader<'_>, predictor: Mv) -> Result<Mv> {
    decode_mv_with_table(br, predictor, MvTable::Default)
}

/// Variant of [`decode_mv`] with explicit MV-VLC selection. The v3
/// picture decoder threads the per-frame `mv_table_sel` bit through
/// here so streams with `mv_table_sel == 1` route to
/// [`MvTable::Alternate`]. Both variants are now wired end-to-end: the
/// alternate VLC source (VMA `0x1c25a0b8`, file `0x594b8`) was
/// re-extracted at its full 8804-byte size in Extractor 07 (spec/16 §1,
/// `tables/region_0594b8_mvvlc.csv` — 1100 entries, ESC at 1099,
/// Kraft = 1.0) and is decoded through the same canonical-Huffman
/// builder as the default, paired with the alternate `(MVDx, MVDy)`
/// byte LUTs at VMAs `0x1c25c320` / `0x1c25c770`.
pub fn decode_mv_with_table(
    br: &mut BitReader<'_>,
    predictor: Mv,
    mv_table: MvTable,
) -> Result<Mv> {
    match mv_table {
        MvTable::Default => decode_mv_variant(br, predictor, table(), MVDX_V3_BYTES, MVDY_V3_BYTES),
        MvTable::Alternate => decode_mv_variant(
            br,
            predictor,
            alt_table(),
            MVDX_V3_ALT_BYTES,
            MVDY_V3_ALT_BYTES,
        ),
    }
}

/// Internal: the v3 joint-MV decode body, parameterised by the VLC
/// table and the `(MVDx, MVDy)` byte LUTs so both `mv_table_sel`
/// variants share one implementation. The ESC path (index 1099) and the
/// bias/predictor/wrap arithmetic are identical across variants per
/// spec/06 §3.3 / §3.5 and spec/16 §1 (the alternate's alphabet shape
/// matches the default — same ESC index, same 6+6-bit FLC tail).
fn decode_mv_variant(
    br: &mut BitReader<'_>,
    predictor: Mv,
    vlc_table: &[VlcEntry<u16>],
    mvdx_lut: &[u8; 1104],
    mvdy_lut: &[u8; 1104],
) -> Result<Mv> {
    let idx = vlc::decode(br, vlc_table)? as usize;

    let (raw_x, raw_y) = if idx == MV_V3_ESC_INDEX {
        // ESC: two 6-bit FLC reads (spec/06 §3.3).
        let x = br.read_u32(6)? as u8;
        let y = br.read_u32(6)? as u8;
        (x, y)
    } else if idx >= MV_V3_ESC_INDEX {
        return Err(Error::invalid(format!(
            "msmpeg4v3 mv: decoded index {idx} out of alphabet range"
        )));
    } else {
        (mvdx_lut[idx], mvdy_lut[idx])
    };

    // spec/06 §3.5: subtract bias 32 to get signed residual, add
    // predictor, wrap into `[-63, +63]`.
    let mv_x = wrap_component(raw_x as i32 + predictor.x as i32 - 32);
    let mv_y = wrap_component(raw_y as i32 + predictor.y as i32 - 32);
    Ok(Mv { x: mv_x, y: mv_y })
}

/// Encoder-side inverse map for one v3 joint-MV VLC variant: for each
/// `(raw_x, raw_y)` byte pair in `0..64 × 0..64`, the joint symbol
/// index with the **shortest codeword** that decodes to that pair
/// through the paired byte LUTs, or `u16::MAX` when no non-ESC symbol
/// covers the pair (→ the encoder falls back to the ESC + 6+6-bit FLC
/// tail, which can express every pair).
struct MvEncodeLut {
    best_idx: Box<[u16; 4096]>,
}

impl MvEncodeLut {
    fn build(vlc_table: &[VlcEntry<u16>], mvdx_lut: &[u8; 1104], mvdy_lut: &[u8; 1104]) -> Self {
        let mut best_idx = vec![u16::MAX; 4096].into_boxed_slice();
        let mut best_bits = vec![u8::MAX; 4096].into_boxed_slice();
        for e in vlc_table {
            let idx = e.value as usize;
            if idx >= MV_V3_ESC_INDEX {
                continue; // ESC handled by the caller's fallback
            }
            let rx = mvdx_lut[idx] as usize;
            let ry = mvdy_lut[idx] as usize;
            if rx >= 64 || ry >= 64 {
                continue; // outside the 6-bit residual domain
            }
            let key = rx * 64 + ry;
            if e.bits < best_bits[key] {
                best_bits[key] = e.bits;
                best_idx[key] = e.value;
            }
        }
        let best_idx: Box<[u16; 4096]> = best_idx.try_into().expect("length 4096");
        Self { best_idx }
    }

    fn lookup(&self, raw_x: u8, raw_y: u8) -> Option<u16> {
        let v = self.best_idx[raw_x as usize * 64 + raw_y as usize];
        if v == u16::MAX {
            None
        } else {
            Some(v)
        }
    }
}

static MV_V3_ENCODE_LUT: std::sync::OnceLock<MvEncodeLut> = std::sync::OnceLock::new();
static MV_V3_ALT_ENCODE_LUT: std::sync::OnceLock<MvEncodeLut> = std::sync::OnceLock::new();

fn encode_lut(mv_table: MvTable) -> &'static MvEncodeLut {
    match mv_table {
        MvTable::Default => MV_V3_ENCODE_LUT
            .get_or_init(|| MvEncodeLut::build(table(), MVDX_V3_BYTES, MVDY_V3_BYTES)),
        MvTable::Alternate => MV_V3_ALT_ENCODE_LUT
            .get_or_init(|| MvEncodeLut::build(alt_table(), MVDX_V3_ALT_BYTES, MVDY_V3_ALT_BYTES)),
    }
}

/// Map one final MV component + predictor component to the biased
/// on-wire residual byte in `0..64` — the inverse of the
/// [`decode_mv_variant`] arithmetic (`mv = wrap(raw + pred − 32)`,
/// spec/06 §3.5). The residual is unique mod 64; whether the decoder's
/// single-pass wrap then reproduces `mv` (rather than `mv ± 64`) is
/// checked by [`mv_component_reachable`] — the toroidal coding makes a
/// component reachable from a predictor only within a contiguous
/// 64-wide window around it.
fn raw_component(mv: i8, pred: i8) -> u8 {
    (mv as i32 - pred as i32 + 32).rem_euclid(64) as u8
}

/// True when the decoder's `wrap(raw + pred − 32)` arithmetic
/// (spec/06 §3.5) can reconstruct the component `mv` from the
/// predictor component `pred` for **some** 6-bit residual — i.e. `mv`
/// lies in the 64-value toroidal window the residual can address from
/// `pred`. An encoder's motion search must only emit MVs whose both
/// components are reachable from the §7.6.5 predictor it shares with
/// the decoder.
pub fn mv_component_reachable(mv: i8, pred: i8) -> bool {
    let raw = raw_component(mv, pred);
    wrap_component(raw as i32 + pred as i32 - 32) == mv
}

/// Encode one v3 joint motion vector — the bit-level inverse of
/// [`decode_mv_with_table`]. `predictor` must be the same §7.6.5
/// median predictor the decoder will derive at this MB position, and
/// `mv` the final half-pel MV the decoder should reconstruct (both
/// components in `[-63, +63]`).
///
/// The encoder prefers the shortest non-ESC joint symbol whose
/// `(MVDx, MVDy)` byte-LUT pair matches the biased residual, falling
/// back to the ESC symbol (index 1099) + two 6-bit FLC components —
/// which can express any residual pair — when the joint alphabet has
/// no covering codeword (spec/06 §3.3).
pub fn encode_mv_with_table(
    bw: &mut BitWriter,
    predictor: Mv,
    mv_table: MvTable,
    mv: Mv,
) -> Result<()> {
    if !(-63..=63).contains(&mv.x) || !(-63..=63).contains(&mv.y) {
        return Err(Error::invalid(format!(
            "msmpeg4v3 mv: component ({}, {}) outside [-63, +63]",
            mv.x, mv.y
        )));
    }
    if !mv_component_reachable(mv.x, predictor.x) || !mv_component_reachable(mv.y, predictor.y) {
        return Err(Error::invalid(format!(
            "msmpeg4v3 mv: MV ({}, {}) not reachable from predictor ({}, {}) \
             through the 6-bit toroidal residual (spec/06 §3.5)",
            mv.x, mv.y, predictor.x, predictor.y
        )));
    }
    let raw_x = raw_component(mv.x, predictor.x);
    let raw_y = raw_component(mv.y, predictor.y);
    let vlc_table = match mv_table {
        MvTable::Default => table(),
        MvTable::Alternate => alt_table(),
    };
    let write_idx = |bw: &mut BitWriter, idx: u16| -> Result<()> {
        let entry = vlc_table.iter().find(|e| e.value == idx).ok_or_else(|| {
            Error::invalid(format!("msmpeg4v3 mv: joint symbol {idx} has no codeword"))
        })?;
        bw.write_u32(entry.code, entry.bits as u32);
        Ok(())
    };
    if let Some(idx) = encode_lut(mv_table).lookup(raw_x, raw_y) {
        return write_idx(bw, idx);
    }
    // ESC: joint symbol 1099 + 6-bit raw_x + 6-bit raw_y (spec/06 §3.3).
    write_idx(bw, MV_V3_ESC_INDEX as u16)?;
    bw.write_u32(raw_x as u32, 6);
    bw.write_u32(raw_y as u32, 6);
    Ok(())
}

/// Toroidal wrap per spec/06 §3.5: if `mv > 63`, subtract 64; if
/// `mv < -63`, add 64. One pass suffices.
fn wrap_component(mv: i32) -> i8 {
    let m = if mv > 63 {
        mv - 64
    } else if mv < -63 {
        mv + 64
    } else {
        mv
    };
    m as i8
}

// =====================================================================
// v1 / v2 per-component MV decoder (spec/06 §2.3 / spec/07 §3)
// =====================================================================
//
// Unlike v3's joint (MVDx, MVDy) coding, v1/v2 use **two separate**
// canonical-Huffman reads against the same 65-entry table at VMA
// 0x1c24f930 (round 12). The table is a flat 13-bit prefix LUT of 8192
// halfword entries; symbol values 0..=64 map after a `-32` bias to a
// signed MVD residual in [-32, +32]. The most-probable code (sym 32 =
// MVD 0) is a single bit ('1'); the next two (sym 31 / 33 = MVD ±1)
// are 3-bit codes ('010' / '011'); subsequent magnitudes have longer
// codes up to 13 bits at the alphabet endpoints.
//
// There is NO ESC path in v1/v2 — the alphabet is complete (Kraft sum
// = 1 - 4/2^13 with the 4 missing leaves reserved for the bit-reader-
// error sentinel; see build.rs::emit_mv_v1_v2 and the helper disassembly
// at 1c21587f).
//
// References:
// * `docs/video/msmpeg4/spec/06-mv-decoder.md` §2.3, §3.5, §4.5
// * `docs/video/msmpeg4/spec/07-remaining-opens.md` §3
// * `docs/video/msmpeg4/spec/99-current-understanding.md` §3.2.2
// * `crates/oxideav-msmpeg4/tables/region_04ed30_full.{hex,meta}`

/// Lazy-built canonical-Huffman table for the v1/v2 per-component MV
/// VLC. 65 symbols (raw indices 0..=64).
static MV_V1_V2_TABLE: std::sync::OnceLock<Vec<VlcEntry<u8>>> = std::sync::OnceLock::new();

fn build_v1v2_table() -> Vec<VlcEntry<u8>> {
    MV_V1_V2_RAW
        .iter()
        .map(|&(sym, bl, code)| VlcEntry::new(bl, code, sym))
        .collect()
}

fn v1v2_table() -> &'static [VlcEntry<u8>] {
    MV_V1_V2_TABLE.get_or_init(build_v1v2_table)
}

/// Decode one MV component from the v1/v2 per-component table at VMA
/// `0x1c24f930`. Returns the **raw VLC index** in `[0, 64]` — the caller
/// is responsible for the predictor add and toroidal wrap.
///
/// Per spec/07 §3.2: helper `0x1c215811` invoked with max-bitlen 13 against
/// the literal table at `0x1c24f930`. Result `eax = movzx al` lies in
/// `[0, 32]` (the spec's off-by-2× — actually 0..=64; the bias subtraction
/// `eax + ecx - 0x20` then yields a signed residual in `[-32, +32]`).
pub fn decode_mvd_v1v2_raw(br: &mut BitReader<'_>) -> Result<u8> {
    let raw = vlc::decode(br, v1v2_table())?;
    if raw > 64 {
        return Err(Error::invalid(format!(
            "msmpeg4 v1/v2 mv: decoded raw idx {raw} > 64 (alphabet 0..=64)"
        )));
    }
    Ok(raw)
}

/// Decode one v1/v2 motion vector — two separate component reads, each
/// against the shared table at VMA `0x1c24f930`, plus the predictor add
/// and toroidal wrap.
///
/// Per spec/07 §3.2 / spec/99 §3.2.2 the v1/v2 MV decoder body
/// (`0x1c217e56` v<4 branch) does:
///
/// ```text
///   raw_x = decode(MV_V1_V2_TABLE)        ; 1..=13 bits
///   raw_y = decode(MV_V1_V2_TABLE)        ; 1..=13 bits
///   mv_x  = (raw_x - 32) + predictor.x    ; bias subtract, predictor add
///   mv_y  = (raw_y - 32) + predictor.y
///   wrap each into [-63, +63]             ; ±64 toroidal step
/// ```
///
/// The predictor is the median-of-3 from `median_predictor` (same helper
/// `0x1c217c8c` as v3).
pub fn decode_mv_v1v2(br: &mut BitReader<'_>, predictor: Mv) -> Result<Mv> {
    let raw_x = decode_mvd_v1v2_raw(br)?;
    let raw_y = decode_mvd_v1v2_raw(br)?;
    let mv_x = wrap_component(raw_x as i32 - MV_V1_V2_BIAS + predictor.x as i32);
    let mv_y = wrap_component(raw_y as i32 - MV_V1_V2_BIAS + predictor.y as i32);
    Ok(Mv { x: mv_x, y: mv_y })
}

/// Find the v1/v2 on-wire raw index (`0..=64`) whose decode reproduces
/// the component `mv` from the predictor component `pred`, if one
/// exists: the residual `mv − pred` must fit `[-32, +32]` directly or
/// after a single ±64 toroidal correction, and the decoder's
/// single-pass wrap must land back on `mv` (spec/07 §3.2 — same
/// wrap-window constraint as the v3 joint decoder, but with the
/// 65-value `[-32, +32]` residual alphabet).
fn raw_v1v2_component(mv: i8, pred: i8) -> Option<u8> {
    let base = mv as i32 - pred as i32;
    for residual in [base, base - 64, base + 64] {
        if !(-32..=32).contains(&residual) {
            continue;
        }
        let raw = (residual + MV_V1_V2_BIAS) as u8;
        if wrap_component(raw as i32 - MV_V1_V2_BIAS + pred as i32) == mv {
            return Some(raw);
        }
    }
    None
}

/// True when the v1/v2 per-component coding can reconstruct `mv` from
/// the predictor component `pred` (see [`raw_v1v2_component`]).
pub fn mv_v1v2_component_reachable(mv: i8, pred: i8) -> bool {
    raw_v1v2_component(mv, pred).is_some()
}

/// Encode one v1/v2 motion vector — the bit-level inverse of
/// [`decode_mv_v1v2`]: two independent component codewords against the
/// shared 65-entry table (spec/07 §3.2), each carrying the biased
/// residual `mv − pred + 32` (with the single ±64 toroidal correction
/// where needed). Errors when a component is outside the reachable
/// window of its predictor — the motion search must pre-filter with
/// [`mv_v1v2_component_reachable`].
pub fn encode_mv_v1v2(bw: &mut BitWriter, predictor: Mv, mv: Mv) -> Result<()> {
    let raw_x = raw_v1v2_component(mv.x, predictor.x);
    let raw_y = raw_v1v2_component(mv.y, predictor.y);
    let (Some(raw_x), Some(raw_y)) = (raw_x, raw_y) else {
        return Err(Error::invalid(format!(
            "msmpeg4 v1/v2 mv: MV ({}, {}) not reachable from predictor ({}, {}) \
             through the [-32, +32] toroidal residual (spec/07 §3.2)",
            mv.x, mv.y, predictor.x, predictor.y
        )));
    };
    for raw in [raw_x, raw_y] {
        let entry = v1v2_table()
            .iter()
            .find(|e| e.value == raw)
            .ok_or_else(|| {
                Error::invalid(format!("msmpeg4 v1/v2 mv: raw index {raw} has no codeword"))
            })?;
        bw.write_u32(entry.code, entry.bits as u32);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mv_vlc_table_has_1100_entries() {
        let t = table();
        assert_eq!(t.len(), 1100);
    }

    #[test]
    fn mv_vlc_is_prefix_free() {
        // The extracted wire codes form a prefix code: no code is a
        // prefix of another. For every pair of entries with distinct
        // bit-lengths, the shorter is not a prefix of the longer.
        let t = table();
        for (i, a) in t.iter().enumerate() {
            for b in &t[i + 1..] {
                if a.bits == b.bits {
                    continue;
                }
                let (short, long) = if a.bits < b.bits { (a, b) } else { (b, a) };
                let shift = long.bits - short.bits;
                let long_prefix = long.code >> shift;
                assert_ne!(
                    long_prefix, short.code,
                    "prefix: sym {} (bl={}) is a prefix of sym {} (bl={})",
                    short.value, short.bits, long.value, long.bits,
                );
            }
        }
    }

    /// Replay a textbook-canonical code assignment over the bit-lengths
    /// (sort by `(bit_length, symbol_index)`, seed code 0, then
    /// `code = (code + 1) << Δbl`) and return it keyed by symbol.
    fn canonical_codes(raw: &[(u32, u32)]) -> std::collections::HashMap<u16, u32> {
        let mut syms: Vec<(u32, u16)> = raw
            .iter()
            .enumerate()
            .filter_map(|(i, &(bl, _))| if bl == 0 { None } else { Some((bl, i as u16)) })
            .collect();
        syms.sort_by_key(|&(bl, i)| (bl, i));
        let mut out = std::collections::HashMap::new();
        let mut code: u32 = 0;
        let mut prev_bl: u32 = 0;
        for (n, &(bl, i)) in syms.iter().enumerate() {
            if n == 0 {
                code = 0;
            } else {
                code = (code + 1) << (bl - prev_bl);
            }
            out.insert(i, code);
            prev_bl = bl;
        }
        out
    }

    /// Regression pin for spec/16 §1 / spec/12 §2: the runtime tables
    /// must carry the **actual extracted wire codes** from `MV_V3_RAW` /
    /// `MV_V3_ALT_RAW`, NOT a textbook-canonical reconstruction. The two
    /// disagree for nearly every symbol (the binary's per-slot walker
    /// builder is fed literal `(code, bl)` records), so a regression to
    /// canonical reconstruction would silently mis-decode almost every
    /// motion vector. We assert (a) every table code equals its raw code
    /// and (b) the two assignments genuinely differ for many symbols.
    #[test]
    fn mv_tables_use_extracted_codes_not_canonical() {
        for (raw, tbl) in [(MV_V3_RAW, table()), (MV_V3_ALT_RAW, alt_table())] {
            // (a) table codes are the extracted raw codes verbatim.
            for e in tbl {
                let (bl, code) = raw[e.value as usize];
                assert_eq!(e.bits as u32, bl, "sym {} bit-length", e.value);
                assert_eq!(e.code, code, "sym {} must use extracted wire code", e.value);
            }
            // (b) the extracted codes differ from canonical for most syms.
            let canon = canonical_codes(raw);
            let differ = tbl
                .iter()
                .filter(|e| canon.get(&e.value) != Some(&e.code))
                .count();
            assert!(
                differ > tbl.len() / 2,
                "extracted codes should differ from canonical for the \
                 majority of symbols (got {differ}/{}); a small count \
                 suggests a silent regression to canonical reconstruction",
                tbl.len()
            );
        }
    }

    #[test]
    fn median_predictor_picks_middle() {
        let a = Mv { x: 1, y: 0 };
        let b = Mv { x: 5, y: 0 };
        let c = Mv { x: 3, y: 0 };
        let p = median_predictor(Some(a), Some(b), Some(c));
        assert_eq!(p.x, 3);
        assert_eq!(p.y, 0);
    }

    #[test]
    fn median_predictor_missing_neighbours_zero() {
        // Only `left` available; the others are treated as zero.
        let a = Mv { x: 5, y: -2 };
        let p = median_predictor(Some(a), None, None);
        assert_eq!(p, Mv { x: 0, y: 0 }); // median(5, 0, 0) = 0
    }

    #[test]
    fn wrap_component_no_change_in_range() {
        for &v in &[-63i32, -32, 0, 31, 63] {
            assert_eq!(wrap_component(v), v as i8);
        }
    }

    #[test]
    fn wrap_component_torus() {
        // > +63 wraps down by 64.
        assert_eq!(wrap_component(64), 0);
        assert_eq!(wrap_component(95), 31);
        // < -63 wraps up by 64.
        assert_eq!(wrap_component(-64), 0);
        assert_eq!(wrap_component(-95), -31);
    }

    #[test]
    fn decode_mv_round_trip_single_symbol() {
        // The default table has exactly one 1-bit symbol (the most-
        // probable joint symbol). Its code is the actual DLL wire
        // pattern `1` (spec/16 §1, region_05bfc0_mvvlc) — NOT `0` as a
        // textbook-canonical assignment would produce. Encode that code
        // MSB-first + tail padding, decode, check MV output.
        let t = table();
        let one_bit: Vec<_> = t.iter().filter(|e| e.bits == 1).collect();
        assert_eq!(one_bit.len(), 1);
        let e = one_bit[0];
        assert_eq!(e.code, 1, "default 1-bit code is the wire pattern `1`");
        let sym = e.value as usize;
        let expected_raw_x = MVDX_V3_BYTES[sym] as i32;
        let expected_raw_y = MVDY_V3_BYTES[sym] as i32;
        // Predictor = (0, 0). Output = (raw - 32) after wrap.
        let exp_x = wrap_component(expected_raw_x - 32);
        let exp_y = wrap_component(expected_raw_y - 32);

        // Byte stream: 1 bit = '1' (MSB) followed by pad bits = 0x80.
        let data = [0x80u8, 0x00, 0x00];
        let mut br = BitReader::new(&data);
        let out = decode_mv(&mut br, Mv::default()).unwrap();
        assert_eq!(out.x, exp_x, "x mismatch: raw_x={expected_raw_x}");
        assert_eq!(out.y, exp_y, "y mismatch: raw_y={expected_raw_y}");
    }

    // =================================================================
    // v1/v2 MV decoder tests
    // =================================================================

    #[test]
    fn v1v2_table_has_65_entries() {
        let t = v1v2_table();
        assert_eq!(t.len(), 65);
        // Symbols are contiguous 0..=64 per build.rs invariant.
        let mut syms: Vec<u8> = t.iter().map(|e| e.value).collect();
        syms.sort_unstable();
        assert_eq!(syms, (0u8..=64).collect::<Vec<u8>>());
    }

    #[test]
    fn v1v2_table_kraft_sum_equals_complement_of_4_leaves() {
        // The full 13-bit prefix space has 8192 leaves. The 65 alphabet
        // codes plus 4 escape sentinels cover all 8192. Kraft over the
        // 65 code-lengths therefore sums to 8188/8192 = 1 - 4/2^13.
        let t = v1v2_table();
        let max_bl: u32 = 32;
        let target: u64 = (1u64 << max_bl) - (1u64 << (max_bl - 13)) * 4;
        let sum: u64 = t.iter().map(|e| 1u64 << (max_bl - e.bits as u32)).sum();
        assert_eq!(
            sum, target,
            "v1/v2 MV Kraft sum mismatch: 65 codes should leave exactly 4 \
             unused 13-bit leaves (the bit-reader-error escape slots)"
        );
    }

    #[test]
    fn v1v2_table_is_prefix_free() {
        let t = v1v2_table();
        for (i, a) in t.iter().enumerate() {
            for b in &t[i + 1..] {
                if a.bits == b.bits {
                    continue;
                }
                let (short, long) = if a.bits < b.bits { (a, b) } else { (b, a) };
                let shift = long.bits - short.bits;
                let long_prefix = long.code >> shift;
                assert_ne!(
                    long_prefix, short.code,
                    "v1/v2 MV: sym {} (bl={}) is a prefix of sym {} (bl={})",
                    short.value, short.bits, long.value, long.bits,
                );
            }
        }
    }

    #[test]
    fn v1v2_zero_mvd_is_one_bit() {
        // Per the LUT data: sym 32 (raw idx 32, MVD=0) has the single
        // 1-bit code 'b1. This is the most-probable code in the alphabet.
        let t = v1v2_table();
        let e = t.iter().find(|e| e.value == 32).expect("sym 32 present");
        assert_eq!(e.bits, 1, "MVD=0 (sym 32) must be 1-bit");
        assert_eq!(e.code, 0b1);
    }

    #[test]
    fn v1v2_pm1_mvd_is_three_bit() {
        // sym 31 (MVD=-1) and sym 33 (MVD=+1) are 3-bit codes 010/011.
        let t = v1v2_table();
        let s31 = t.iter().find(|e| e.value == 31).unwrap();
        let s33 = t.iter().find(|e| e.value == 33).unwrap();
        assert_eq!(s31.bits, 3);
        assert_eq!(s33.bits, 3);
        // 010 and 011 — the LUT layout puts 33 at the lower 3-bit code
        // (010) and 31 at 011 per the extracted slots (positive side
        // first within each bit-length tier).
        assert!(
            (s31.code == 0b011 && s33.code == 0b010) || (s31.code == 0b010 && s33.code == 0b011),
            "sym 31/33 codes must be 010 / 011 in some order, got {:03b}/{:03b}",
            s31.code,
            s33.code
        );
    }

    #[test]
    fn decode_mv_v1v2_zero_predictor_zero_mvd() {
        // Two 1-bit '1' reads in a row → raw_x = 32, raw_y = 32 →
        // (32 - 32) = 0 for both components.
        let data = [0b11000000u8, 0x00, 0x00];
        let mut br = BitReader::new(&data);
        let mv = decode_mv_v1v2(&mut br, Mv::default()).unwrap();
        assert_eq!(mv, Mv { x: 0, y: 0 });
    }

    #[test]
    fn decode_mv_v1v2_zero_predictor_pos1_pos1() {
        // sym 33 (MVD=+1) followed by sym 33 again. Look up the actual
        // canonical code from the table to assemble the bitstream.
        let t = v1v2_table();
        let e33 = t.iter().find(|e| e.value == 33).unwrap();
        // Build a stream of two e33.code ::3 followed by zero padding.
        let bits_needed = (e33.bits as u32) * 2;
        let mut acc: u64 = 0;
        let mut nbits: u32 = 0;
        acc = (acc << e33.bits) | e33.code as u64;
        nbits += e33.bits as u32;
        acc = (acc << e33.bits) | e33.code as u64;
        nbits += e33.bits as u32;
        // Pad to byte boundary.
        let pad = (8 - (nbits % 8)) % 8;
        acc <<= pad;
        nbits += pad;
        let mut bytes: Vec<u8> = Vec::new();
        while nbits > 0 {
            nbits -= 8;
            bytes.push(((acc >> nbits) & 0xff) as u8);
        }
        bytes.extend_from_slice(&[0u8; 4]);
        assert!(bytes.len() * 8 >= bits_needed as usize + 32);

        let mut br = BitReader::new(&bytes);
        let mv = decode_mv_v1v2(&mut br, Mv::default()).unwrap();
        assert_eq!(mv, Mv { x: 1, y: 1 });
    }

    #[test]
    fn decode_mv_v1v2_predictor_added() {
        // sym 33 (MVD=+1) then sym 33; predictor (3, -2). Result should be
        // (1 + 3, 1 - 2) = (4, -1).
        let t = v1v2_table();
        let e33 = t.iter().find(|e| e.value == 33).unwrap();
        let mut acc: u64 = 0;
        let mut nbits: u32 = 0;
        acc = (acc << e33.bits) | e33.code as u64;
        nbits += e33.bits as u32;
        acc = (acc << e33.bits) | e33.code as u64;
        nbits += e33.bits as u32;
        let pad = (8 - (nbits % 8)) % 8;
        acc <<= pad;
        nbits += pad;
        let mut bytes: Vec<u8> = Vec::new();
        while nbits > 0 {
            nbits -= 8;
            bytes.push(((acc >> nbits) & 0xff) as u8);
        }
        bytes.extend_from_slice(&[0u8; 4]);
        let mut br = BitReader::new(&bytes);
        let mv = decode_mv_v1v2(&mut br, Mv { x: 3, y: -2 }).unwrap();
        assert_eq!(mv, Mv { x: 4, y: -1 });
    }

    #[test]
    fn decode_mv_v1v2_wraps_torus() {
        // Need raw + predictor - 32 to overflow. Pick raw_x = 64 (MVD=+32),
        // predictor.x = +63 → 32 + 63 = 95 → wrap → 95 - 64 = 31.
        let t = v1v2_table();
        let e64 = t.iter().find(|e| e.value == 64).unwrap();
        let e32 = t.iter().find(|e| e.value == 32).unwrap();
        // Stream: e64.code (raw_x = 64), then e32.code (raw_y = 32 → MVD=0).
        let mut acc: u64 = 0;
        let mut nbits: u32 = 0;
        acc = (acc << e64.bits) | e64.code as u64;
        nbits += e64.bits as u32;
        acc = (acc << e32.bits) | e32.code as u64;
        nbits += e32.bits as u32;
        let pad = (8 - (nbits % 8)) % 8;
        acc <<= pad;
        nbits += pad;
        let mut bytes: Vec<u8> = Vec::new();
        while nbits > 0 {
            nbits -= 8;
            bytes.push(((acc >> nbits) & 0xff) as u8);
        }
        bytes.extend_from_slice(&[0u8; 4]);
        let mut br = BitReader::new(&bytes);
        let mv = decode_mv_v1v2(&mut br, Mv { x: 63, y: 0 }).unwrap();
        // (64 - 32) + 63 = 95 → wrap to 31. Y: 0 + 0 = 0.
        assert_eq!(mv, Mv { x: 31, y: 0 });
    }

    #[test]
    fn decode_mv_v1v2_round_trip_every_symbol() {
        // Round-trip every symbol 0..=64 via a single-component encode/decode.
        let t = v1v2_table();
        for &(sym, bl, code) in MV_V1_V2_RAW {
            // Pack code (bl bits) followed by tail padding.
            let mut acc: u64 = code as u64;
            let mut nbits: u32 = bl as u32;
            let pad = (8 - (nbits % 8)) % 8;
            acc <<= pad;
            nbits += pad;
            let mut bytes: Vec<u8> = Vec::new();
            while nbits > 0 {
                nbits -= 8;
                bytes.push(((acc >> nbits) & 0xff) as u8);
            }
            bytes.extend_from_slice(&[0u8; 4]);
            let mut br = BitReader::new(&bytes);
            let raw = decode_mvd_v1v2_raw(&mut br).unwrap();
            assert_eq!(raw, sym, "round-trip failed for sym {sym}");
            // Sanity: bit length consumed matches the table entry.
            // (Indirectly verified by raw == sym.)
            assert!(
                t.iter()
                    .any(|e| e.value == sym && e.bits == bl && e.code == code),
                "sym {sym} table lookup failed"
            );
        }
    }

    #[test]
    fn decode_mv_esc_path() {
        // ESC is index 1099; its canonical code is the longest shortest
        // path (one of the highest-bit-length entries, but since
        // canonical order is (bit_length asc, idx asc), ESC sits at
        // index 1099 — its code depends on the surrounding bit-length
        // array. Easiest end-to-end test: encode what we'll read (ESC
        // code + 6 bits X_raw + 6 bits Y_raw) and assert the result.
        let t = table();
        let esc_entry = t
            .iter()
            .find(|e| e.value as usize == MV_V3_ESC_INDEX)
            .expect("ESC symbol present");
        // X raw = 40, Y raw = 24 (both arbitrary in [0, 63]).
        let mut acc: u64 = 0;
        let mut bits: u32 = 0;
        acc = (acc << esc_entry.bits) | esc_entry.code as u64;
        bits += esc_entry.bits as u32;
        acc = (acc << 6) | 40;
        bits += 6;
        acc = (acc << 6) | 24;
        bits += 6;
        // Pad to byte boundary + a few extra bytes.
        let pad = (8 - (bits % 8)) % 8;
        acc <<= pad;
        bits += pad;
        let mut data: Vec<u8> = Vec::new();
        while bits > 0 {
            bits -= 8;
            data.push(((acc >> bits) & 0xff) as u8);
        }
        data.extend_from_slice(&[0u8; 4]);
        let mut br = BitReader::new(&data);
        let out = decode_mv(&mut br, Mv::default()).unwrap();
        // raw_x = 40, predictor 0 → 40 - 32 = 8, no wrap needed.
        assert_eq!(out.x, 8);
        // raw_y = 24, predictor 0 → 24 - 32 = -8, no wrap needed.
        assert_eq!(out.y, -8);
    }

    // =================================================================
    // Round 326: alternate-variant VLC source wired end-to-end
    // (spec/16 §1 / Extractor 07). The alternate MV VLC table decodes
    // through the same builder as the default, using its extracted wire
    // codes (spec/12 §2) rather than a canonical reconstruction.
    // =================================================================

    /// Full-alphabet round trip: for BOTH variants, stream every non-ESC
    /// symbol's actual extracted wire code (MSB-first) into the joint-MV
    /// decoder with a zero predictor and assert it recovers exactly that
    /// symbol's `(MVDx, MVDy)` byte-LUT residual (after the −32 bias and
    /// toroidal wrap). This exercises all 1099 payload codes through the
    /// real bit patterns end-to-end, locking the spec/16 §1 / spec/12 §2
    /// extracted-code path against any silent regression.
    #[test]
    fn decode_mv_v3_round_trip_every_payload_symbol() {
        use crate::tables_data::{
            MVDX_V3_ALT_BYTES, MVDX_V3_BYTES, MVDY_V3_ALT_BYTES, MVDY_V3_BYTES,
        };
        // (which table, raw (bl,code) pairs, MVDx LUT, MVDy LUT).
        type Variant = (
            MvTable,
            &'static [(u32, u32)],
            &'static [u8; 1104],
            &'static [u8; 1104],
        );
        let variants: [Variant; 2] = [
            (MvTable::Default, MV_V3_RAW, MVDX_V3_BYTES, MVDY_V3_BYTES),
            (
                MvTable::Alternate,
                MV_V3_ALT_RAW,
                MVDX_V3_ALT_BYTES,
                MVDY_V3_ALT_BYTES,
            ),
        ];
        for (which, raw, lut_x, lut_y) in variants {
            for idx in 0..MV_V3_ESC_INDEX {
                let (bl, code) = raw[idx];
                // Pack `code` in `bl` bits MSB-first, then pad to a byte
                // boundary plus slack for the BitReader.
                let mut acc: u64 = code as u64;
                let mut nbits = bl;
                let pad = (8 - (nbits % 8)) % 8;
                acc <<= pad;
                nbits += pad;
                let mut bytes: Vec<u8> = Vec::new();
                while nbits > 0 {
                    nbits -= 8;
                    bytes.push(((acc >> nbits) & 0xff) as u8);
                }
                bytes.extend_from_slice(&[0u8; 4]);

                let mut br = BitReader::new(&bytes);
                let out = decode_mv_with_table(&mut br, Mv::default(), which).unwrap();
                let exp_x = wrap_component(lut_x[idx] as i32 - 32);
                let exp_y = wrap_component(lut_y[idx] as i32 - 32);
                assert_eq!(
                    out,
                    Mv { x: exp_x, y: exp_y },
                    "{which:?} sym {idx} (bl={bl}, code={code}) round trip"
                );
                assert_eq!(
                    br.bit_position() as u32,
                    bl,
                    "{which:?} sym {idx} must consume exactly {bl} bits"
                );
            }
        }
    }

    /// The alternate MV VLC table builds to a complete 1100-entry prefix
    /// code with the ESC symbol (index 1099) present — the same alphabet
    /// shape as the default variant.
    #[test]
    fn alt_mv_vlc_table_has_1100_entries_with_esc() {
        let t = alt_table();
        assert_eq!(t.len(), 1100, "alt MV VLC must have 1100 symbols");
        assert!(
            t.iter().any(|e| e.value as usize == MV_V3_ESC_INDEX),
            "alt MV VLC must contain the ESC symbol (index 1099)"
        );
    }

    /// The alternate table's extracted wire codes are prefix-free.
    #[test]
    fn alt_mv_vlc_is_prefix_free() {
        let t = alt_table();
        for (i, a) in t.iter().enumerate() {
            for b in &t[i + 1..] {
                if a.bits == b.bits {
                    continue;
                }
                let (short, long) = if a.bits < b.bits { (a, b) } else { (b, a) };
                let shift = long.bits - short.bits;
                let long_prefix = long.code >> shift;
                assert_ne!(
                    long_prefix, short.code,
                    "alt MV: sym {} (bl={}) is a prefix of sym {} (bl={})",
                    short.value, short.bits, long.value, long.bits,
                );
            }
        }
    }

    /// `decode_mv_with_table(.., MvTable::Alternate)` no longer errors:
    /// the alternate joint symbol 0 (extracted 2-bit wire code `00`, alt
    /// byte LUT index 0 = `0x20` for both components) decodes to
    /// MV = (0, 0) with a zero predictor.
    #[test]
    fn decode_mv_alternate_symbol_zero_is_zero() {
        use crate::tables_data::{MVDX_V3_ALT_BYTES, MVDY_V3_ALT_BYTES};
        // Confirm the precondition the assertion below relies on.
        assert_eq!(MVDX_V3_ALT_BYTES[0], 32, "alt MVDx idx 0 must be bias 32");
        assert_eq!(MVDY_V3_ALT_BYTES[0], 32, "alt MVDy idx 0 must be bias 32");

        // Alt symbol 0's extracted wire code is `00` (2 bits, spec/16
        // §1, region_0594b8_mvvlc). Stream: `00` then padding.
        let data = [0x00u8, 0x00, 0x00];
        let mut br = BitReader::new(&data);
        let out = decode_mv_with_table(&mut br, Mv::default(), MvTable::Alternate).unwrap();
        assert_eq!(
            out,
            Mv { x: 0, y: 0 },
            "alt sym 0 + zero predictor → (0, 0)"
        );
        // Exactly 2 bits consumed for the joint symbol (no ESC tail).
        assert_eq!(br.bit_position(), 2, "alt sym 0 must consume only 2 bits");
    }

    /// The alternate and default tables are genuinely distinct. The
    /// default's most-probable symbol is the 1-bit wire code `1`; the
    /// alternate's is the 2-bit wire code `00` (spec/16 §1). Driving the
    /// `1` prefix (0x80) through the default consumes 1 bit; the `00`
    /// prefix (0x00) through the alternate consumes 2 bits. This pins
    /// that the dispatch actually selects the alternate table rather
    /// than silently falling back to the default.
    #[test]
    fn alt_and_default_tables_differ_at_dispatch() {
        let def_data = [0x80u8, 0x00, 0x00];
        let alt_data = [0x00u8, 0x00, 0x00];
        let mut br_def = BitReader::new(&def_data);
        let mut br_alt = BitReader::new(&alt_data);
        decode_mv_with_table(&mut br_def, Mv::default(), MvTable::Default).unwrap();
        decode_mv_with_table(&mut br_alt, Mv::default(), MvTable::Alternate).unwrap();
        assert_eq!(br_def.bit_position(), 1, "default 1-bit code `1`");
        assert_eq!(br_alt.bit_position(), 2, "alternate 2-bit code `00`");
    }

    // =================================================================
    // Round 251: alternate-variant byte LUTs landed (spec/06 §2.2)
    // =================================================================

    /// The alternate (MVDx, MVDy) byte LUTs are emitted by `build.rs`
    /// from `tables/region_05b720.hex` and `tables/region_05bb70.hex`
    /// (VMAs `0x1c25c320` / `0x1c25c770` per spec/06 §2.2). Pin the
    /// shape so a future regression can't quietly truncate or pad the
    /// emitted arrays.
    #[test]
    fn mv_alt_byte_luts_have_expected_shape() {
        use crate::tables_data::{MVDX_V3_ALT_BYTES, MVDY_V3_ALT_BYTES};
        // 1104 = 1099 alphabet entries + 5 alignment padding bytes per
        // spec/06 §2.2 (1104-byte spacing between adjacent LUT VMAs).
        assert_eq!(MVDX_V3_ALT_BYTES.len(), 1104, "alt MVDx LUT length");
        assert_eq!(MVDY_V3_ALT_BYTES.len(), 1104, "alt MVDy LUT length");
    }

    /// Each emitted byte is in the toroidal `[0, 63]` MV-residual range
    /// that the decoder's `raw - 32` bias-subtract assumes (spec/06
    /// §3.5). The byte values are unsigned 6-bit pre-biased residuals;
    /// stored byte 0..=63 = MVD `-32..=+31` after bias. A drift into
    /// the 64..=255 range would mean either the file got mis-parsed or
    /// the wrong file was wired.
    #[test]
    fn mv_alt_byte_luts_in_six_bit_range() {
        use crate::tables_data::{MVDX_V3_ALT_BYTES, MVDY_V3_ALT_BYTES};
        for (i, &b) in MVDX_V3_ALT_BYTES[..1099].iter().enumerate() {
            assert!(
                b < 64,
                "alt MVDx idx {i} = {b} not in [0, 63] (6-bit pre-biased range)"
            );
        }
        for (i, &b) in MVDY_V3_ALT_BYTES[..1099].iter().enumerate() {
            assert!(
                b < 64,
                "alt MVDy idx {i} = {b} not in [0, 63] (6-bit pre-biased range)"
            );
        }
    }

    /// The alt LUTs and the default LUTs are different alphabets per
    /// spec/06 §2.1 / §4.3 (patent 6,983,018 Table 1 / Table 2 — two
    /// distinct training corpora producing distinct byte values for the
    /// same alphabet shape). Pin that they don't accidentally point at
    /// the same source file or get loaded as identical arrays.
    #[test]
    fn mv_alt_byte_luts_differ_from_default() {
        use crate::tables_data::{
            MVDX_V3_ALT_BYTES, MVDX_V3_BYTES, MVDY_V3_ALT_BYTES, MVDY_V3_BYTES,
        };
        assert_ne!(
            &MVDX_V3_ALT_BYTES[..1099],
            &MVDX_V3_BYTES[..1099],
            "alt MVDx LUT must not match default — both training corpora are independent"
        );
        assert_ne!(
            &MVDY_V3_ALT_BYTES[..1099],
            &MVDY_V3_BYTES[..1099],
            "alt MVDy LUT must not match default — both training corpora are independent"
        );
    }

    /// Both alt LUTs' value distributions cluster around 32 (the +32
    /// bias baked into every LUT entry per spec/06 §3.5). A useful
    /// guard: the mean of all 1099 active entries should be in
    /// `[20, 44]` (i.e. within ±12 of the bias), which a corrupted /
    /// mis-stride'd or non-LUT byte stream would not satisfy. The
    /// default LUTs satisfy this trivially (means ~31.4 / 32.4 from a
    /// direct count over `tables/region_05e228.hex` /
    /// `region_05e678.hex`).
    #[test]
    fn mv_alt_byte_luts_cluster_around_bias() {
        use crate::tables_data::{MVDX_V3_ALT_BYTES, MVDY_V3_ALT_BYTES};
        let mean_x: f64 = MVDX_V3_ALT_BYTES[..1099]
            .iter()
            .map(|&b| b as f64)
            .sum::<f64>()
            / 1099.0;
        let mean_y: f64 = MVDY_V3_ALT_BYTES[..1099]
            .iter()
            .map(|&b| b as f64)
            .sum::<f64>()
            / 1099.0;
        assert!(
            (20.0..=44.0).contains(&mean_x),
            "alt MVDx mean {mean_x} should cluster around bias 32"
        );
        assert!(
            (20.0..=44.0).contains(&mean_y),
            "alt MVDy mean {mean_y} should cluster around bias 32"
        );
    }

    /// [`MvTable::Default`] decodes the default MV VLC bit-for-bit
    /// identically to [`decode_mv`] (which is just a thin wrapper).
    /// Sanity check that the new dispatch path doesn't drift.
    #[test]
    fn mv_table_default_matches_decode_mv() {
        // 0x80 = the default 1-bit wire code `1` (spec/16 §1).
        let data = [0x80u8, 0x00, 0x00];
        let mut br1 = BitReader::new(&data);
        let mut br2 = BitReader::new(&data);
        let a = decode_mv(&mut br1, Mv::default()).unwrap();
        let b = decode_mv_with_table(&mut br2, Mv::default(), MvTable::Default).unwrap();
        assert_eq!(a, b, "default-table dispatch must match plain decode_mv");
        assert_eq!(
            br1.bit_position(),
            br2.bit_position(),
            "bit consumption must match"
        );
    }

    // ---------------------------------------------------------------------
    // MvTable::from_sel / to_sel — typed dispatch of the picture-header
    // `mv_table_sel` bit per spec/06 §3.2 and spec/01 §1.4. The bit is
    // P-frame-scoped and v3-only; v1/v2 substitute the Default variant per
    // the V1/V2 compat-default contract (`MsV1V2PictureHeader::V*_COMPAT_DEFAULTS`).
    // ---------------------------------------------------------------------

    #[test]
    fn mv_table_from_sel_zero_is_default() {
        // Per spec/06 §3.2: `mv_table_sel = 0` selects the default joint-MV
        // VLC at VMA 0x1c25cbc0 (paired with the byte LUTs at 0x1c25ee28 /
        // 0x1c25f278).
        assert_eq!(MvTable::from_sel(0), Some(MvTable::Default));
    }

    #[test]
    fn mv_table_from_sel_one_is_alternate() {
        // Per spec/06 §3.2: `mv_table_sel = 1` selects the alternate
        // variant at VMA 0x1c25a0b8 (paired with 0x1c25c320 / 0x1c25c770).
        assert_eq!(MvTable::from_sel(1), Some(MvTable::Alternate));
    }

    #[test]
    fn mv_table_from_sel_out_of_range_is_none() {
        // The picture-header parser already clamps via a single bit read,
        // so values > 1 cannot reach this helper from a well-formed stream
        // — defence-in-depth check.
        for sel in 2u8..=255 {
            assert_eq!(MvTable::from_sel(sel), None, "sel={sel} must be None");
        }
    }

    #[test]
    fn mv_table_to_sel_inverts_from_sel() {
        // Round-trip: every variant's `to_sel` must route back through
        // `from_sel` to itself.
        for variant in [MvTable::Default, MvTable::Alternate] {
            let sel = variant.to_sel();
            assert_eq!(
                MvTable::from_sel(sel),
                Some(variant),
                "{variant:?} round-trip"
            );
        }
        // Same direction in the literal-value form per spec/06 §3.2.
        assert_eq!(MvTable::Default.to_sel(), 0);
        assert_eq!(MvTable::Alternate.to_sel(), 1);
    }

    #[test]
    fn mv_table_from_sel_default_value_matches_struct_default() {
        // The `#[derive(Default)]` on MvTable resolves to MvTable::Default;
        // ensure the from_sel(0) helper returns the same variant — so a
        // caller using either entry point sees identical behaviour.
        let from_zero = MvTable::from_sel(0).unwrap();
        let derived: MvTable = MvTable::default();
        assert_eq!(
            from_zero, derived,
            "from_sel(0) must match Default::default()"
        );
    }
}
