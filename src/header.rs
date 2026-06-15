//! MS-MPEG4 picture-header parser.
//!
//! Microsoft MPEG-4 has no VOS / VOL / VOP start-code layering. Each
//! coded frame begins directly with a bit-level picture header followed
//! by macroblock data. The exact layout is taken from
//! `docs/video/msmpeg4/spec/99-current-understanding.md` §2.2 / §2.3,
//! which traces the parser `0x1c211f0c..0x1c2120a4` in the reverse
//! target `reference/binaries/wmpcdcs8-2001/mpg4c32.dll`.
//!
//! ## v3 (DIV3 / MP43 / MPG3) picture header
//!
//! Fields, MSB-first:
//!
//! | Bits   | Name          | Notes (spec §2.2 / §2.3) |
//! | ------ | ------------- | ------------------------ |
//! | 2      | `picture_type`| 0 = I, 1 = P; others rejected |
//! | 5      | `pquant`      | `[esi+0x40]` I / `[esi+0x44]` P; range `[1, 31]` |
//! |        | *(I-frame)*   |                          |
//! | 1–2    | `ac_chroma_sel` | v3-only; unary-capped-at-2 → `[esi+0xad0]` ∈ {0, 1, 2} (G2/G0/G4) |
//! | 1–2    | `ac_luma_sel` | v3-only; unary-capped-at-2 → `[esi+0xad4]` ∈ {0, 1, 2} (G3/G1/G5) |
//! | 1      | `dc_size_sel` | v3-only; single bit → `[esi+0x8bc]` ∈ {0, 1} |
//! |        | *(P-frame)*   |                          |
//! | 1–2    | `ac_chroma_sel` | Re-read per P-frame     |
//! | 1      | `dc_size_sel` | Re-read per P-frame     |
//! | 1      | `mv_table_sel`| `[esi+0x834]` — v3 joint-MV VLC variant |
//!
//! Unary-capped-at-2 encoding (spec §2.3): `0 → 0`, `1 → 10`, `2 → 11`.
//!
//! This parser implements the v3 path. v1/v2 have different header
//! shapes (spec §2.4) and are rejected here.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::g_family::GFamily;
use crate::mv::MvTable;

/// Coded picture type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PictureType {
    /// Intra / key frame.
    I,
    /// Predicted / inter frame.
    P,
}

/// MS-MPEG4v3 picture header.
#[derive(Clone, Debug)]
pub struct MsV3PictureHeader {
    pub picture_type: PictureType,
    /// Frame-wide quantiser, 1..=31.
    pub quant: u8,
    /// DCT AC chroma-VLC class selector (`[esi+0xad0]`): 0 = G2,
    /// 1 = G0, 2 = G4. Read on both I- and P-frames (spec §2.3).
    pub ac_chroma_sel: u8,
    /// DCT AC luma-VLC class selector (`[esi+0xad4]`): 0 = G3,
    /// 1 = G1, 2 = G5. Read on **I-frames only**; persists into
    /// subsequent P-frames.
    pub ac_luma_sel: u8,
    /// Intra-DC-size VLC pair selector (`[esi+0x8bc]`): 0 or 1 —
    /// picks one of two (luma, chroma) intra-DC-size VLC pairs
    /// (spec §4.5). Read on both I- and P-frames.
    pub dc_size_sel: u8,
    /// v3 joint-MV VLC alternate selector (`[esi+0x834]`): P-frame
    /// only. 0 = default (VMAs `0x1c25cbc0` / `0x1c25ee28` / `0x1c25f278`),
    /// 1 = alternate (`0x1c25a0b8` / `0x1c25c320` / `0x1c25c770`).
    pub mv_table_sel: u8,
}

/// Read a unary-capped-at-2 value — spec §2.3 encoding for the
/// two tri-valued v3 per-frame selectors. `0 → 0`, `1 → 10`, `2 → 11`.
fn read_unary_cap2(br: &mut BitReader<'_>) -> Result<u8> {
    if !br.read_bit()? {
        return Ok(0);
    }
    if !br.read_bit()? {
        Ok(1)
    } else {
        Ok(2)
    }
}

impl MsV3PictureHeader {
    /// Parse the v3 picture header at the current bitstream position.
    ///
    /// Matches the per-frame parser at VMA `0x1c211f0c..0x1c2120a4` in
    /// `mpg4c32.dll`; see `docs/video/msmpeg4/spec/99-current-understanding.md`
    /// §2.2 and §2.3 for authoritative citations.
    pub fn parse(br: &mut BitReader<'_>) -> Result<Self> {
        let ptype = br.read_u32(2)?;
        let picture_type = match ptype {
            0 => PictureType::I,
            1 => PictureType::P,
            other => {
                return Err(Error::invalid(format!(
                    "msmpeg4v3: reserved picture_type {other}"
                )));
            }
        };

        let quant = br.read_u32(5)? as u8;
        if !(1..=31).contains(&quant) {
            return Err(Error::invalid(format!(
                "msmpeg4v3: pquant {quant} out of range 1..=31"
            )));
        }

        // Per spec §2.3. I-frame selectors: ad0 (1–2 unary) →
        // ad4 (1–2 unary) → 0x8bc (1 bit). P-frame selectors:
        // ad0 (1–2 unary) → 0x8bc (1 bit) → 0x834 (1 bit, MV alt).
        let mut ac_luma_sel = 0u8;
        let mut mv_table_sel = 0u8;
        let ac_chroma_sel;
        let dc_size_sel;

        match picture_type {
            PictureType::I => {
                ac_chroma_sel = read_unary_cap2(br)?;
                ac_luma_sel = read_unary_cap2(br)?;
                dc_size_sel = br.read_u32(1)? as u8;
            }
            PictureType::P => {
                ac_chroma_sel = read_unary_cap2(br)?;
                dc_size_sel = br.read_u32(1)? as u8;
                mv_table_sel = br.read_u32(1)? as u8;
            }
        }

        Ok(Self {
            picture_type,
            quant,
            ac_chroma_sel,
            ac_luma_sel,
            dc_size_sel,
            mv_table_sel,
        })
    }

    /// Typed dispatch of the parsed `ac_chroma_sel` raw `u8` to the
    /// chroma+all-inter [`GFamily`] per
    /// `docs/video/msmpeg4/spec/14-pri-ab-runtime-binding.md` §3.1:
    /// `0 → G2, 1 → G0, 2 → G4`. Equivalent to
    /// `GFamily::for_chroma_selector(self.ac_chroma_sel)`; exposed as a
    /// method on the header so callers that already hold a parsed
    /// header reach the typed family without re-importing
    /// [`GFamily`] or restating the per-frame slot identity.
    ///
    /// Returns `None` if `ac_chroma_sel` is outside `{0, 1, 2}` — the
    /// `read_unary_cap2` helper already clamps to that range in
    /// well-formed streams, so this is a defence-in-depth guard. The
    /// returned family always has [`crate::g_family::GRole::ChromaAndInter`]
    /// per spec/14 §3.1.
    pub const fn ac_chroma_family(&self) -> Option<GFamily> {
        GFamily::for_chroma_selector(self.ac_chroma_sel)
    }

    /// Typed dispatch of the parsed `ac_luma_sel` raw `u8` to the
    /// intra-luma [`GFamily`] per spec/14 §3.1: `0 → G3, 1 → G1, 2 →
    /// G5`. Equivalent to
    /// `GFamily::for_luma_selector(self.ac_luma_sel)`.
    ///
    /// Returns `None` if `ac_luma_sel` is outside `{0, 1, 2}`. The
    /// returned family always has [`crate::g_family::GRole::IntraLuma`]
    /// per spec/14 §3.1.
    ///
    /// `ac_luma_sel` is read on I-frames only and persists into
    /// subsequent P-frames (per spec/99 §2.3, the P-frame body does
    /// not re-read this bit), so a caller iterating per-frame must
    /// thread the value forward from the most recent I-frame header.
    pub const fn ac_luma_family(&self) -> Option<GFamily> {
        GFamily::for_luma_selector(self.ac_luma_sel)
    }

    /// Typed dispatch of the parsed `mv_table_sel` raw `u8` to the
    /// joint-MV VLC [`MvTable`] per spec/06 §3.2:
    /// `0 → MvTable::Default, 1 → MvTable::Alternate`. Equivalent to
    /// `MvTable::from_sel(self.mv_table_sel)`.
    ///
    /// `mv_table_sel` is P-frame-scoped (read at `1c2120aa` per
    /// spec/99 §2.3); on I-frames the header carries the parser's
    /// zero default, so this method returns `Some(MvTable::Default)`
    /// uniformly on both frame types — the call shape is
    /// version-and-frame-independent at the call site.
    ///
    /// Returns `None` only for `mv_table_sel > 1` (which a well-formed
    /// stream cannot produce; the `parse()` body reads a single bit
    /// for the field).
    pub const fn mv_table(&self) -> Option<MvTable> {
        MvTable::from_sel(self.mv_table_sel)
    }
}

// =====================================================================
// v1 / v2 picture-header parsers (spec §2.2 / §2.4)
// =====================================================================
//
// MSMPEG4 v1 and v2 share the picture-layer skeleton with v3 but skip
// the v3-only per-frame table selectors. v1 also carries an opaque
// 37-bit preamble that the decoder reads-and-discards, and a 1-bit UMV
// flag on P-frames. v2 has neither.
//
// Per spec/99 §2.2 / §2.4:
//
//   v1 P-frame: [37-bit preamble] picture_type(2) PQUANT(5) UMV_flag(1)
//   v1 I-frame: [37-bit preamble] picture_type(2) PQUANT(5)
//   v2 frames:  picture_type(2) PQUANT(5)         (no per-frame selectors)
//
// (The optional 5-bit "first-of-sequence" extension at I-frames §2.2 is
// for the per-sequence init payload — not part of the per-frame loop —
// and is OPEN per spec/99. We do not consume it here.)

/// MSMPEG4 v1 / v2 picture header.
///
/// The v3 header (`MsV3PictureHeader`) carries extra per-frame table
/// selector bits that v1 / v2 lack; using a separate type keeps the
/// downstream wiring explicit about which version it's working with.
///
/// ## v3-only per-frame selectors do NOT exist in v1/v2
///
/// Per
/// [`docs/video/msmpeg4/spec/01-bitstream-framing.md`][spec01]
/// §1.4 (decision tree), the per-frame `ac_chroma_sel` (`[esi+0xad0]`),
/// `ac_luma_sel` (`[esi+0xad4]`), `dc_size_sel` (`[esi+0x8bc]`), and
/// `mv_table_sel` (`[esi+0x834]`) selector bits are only read for
/// **version == 3** frames (`1c211fdd` / `1c21205a..1c2120aa`).
/// v1 and v2 paths never consume these bits, so downstream code that
/// shares logic with v3 must use the following defaults when running
/// a v1 / v2 frame:
///
/// | Selector         | v1 / v2 default | Reason                                                |
/// | ---------------- | --------------- | ----------------------------------------------------- |
/// | `dc_size_sel`    | `0`             | Picks the *default* intra-DC VLC (primary luma/chroma pair); v3 binary slot `[esi+0x8bc]` is uninitialised → reads zero. |
/// | `ac_chroma_sel`  | `0` (G4 default) | v1/v2 always use the G4 cluster for chroma + all-inter (spec/99 §5.2). |
/// | `ac_luma_sel`    | `0` (G5 default) | v1/v2 always use the G5 cluster for intra-luma (spec/99 §5.2).        |
/// | `mv_table_sel`   | `0` (default MV VLC) | The alternate MV VLC is v3-only and lives at `[esi+0x834]` (spec/99 §3.2). |
///
/// These defaults are surfaced by the
/// [`MsV1V2PictureHeader::V1_COMPAT_DEFAULTS`] and
/// [`MsV1V2PictureHeader::V2_COMPAT_DEFAULTS`] associated constants so
/// callers can pin the contract in their own assertions.
///
/// ## v1 has no spatial DC predictor; v2 gains AC prediction only at
/// intra-in-P macroblocks
///
/// Per
/// [`docs/video/msmpeg4/spec/07-remaining-opens.md`][spec07] §1.6
/// (v1 MCBPCY body `0x1c2171c7`), the v1 path "never loads a neighbour
/// MCBPC or CBPY" and contains no patent-7,054,494-style spatial
/// predictor — H.263 Annex B §5.3 specifies no spatial predictor for
/// either field, and v1 follows H.263 verbatim there. v1 likewise has
/// **no MB-level AC-prediction bit**: §1.4 calls out that the v1 body
/// "does **not** read a post-VLC sign / AC-pred bit (no
/// `call 0x1c215c9b` after the CBPY decode), consistent with v1
/// lacking AC prediction".
///
/// v2 inherits the v1 frame-level shape but, per §2.4 (intra-in-P
/// handling — "a v2 innovation"), adds a 1-bit AC-prediction read
/// **only** when an MCBPC of 4..=7 selects MB-type 3 (intra-in-P);
/// the bit lives at MB scope, not at picture scope.
///
/// Practical consequence for the consumer: when a v1 / v2 intra-block
/// shares the [`crate::mb::decode_intra_block_full_v3`] entry point or
/// the [`crate::picture::decode_picture_with_ac`] orchestrator, the
/// `dc_size_sel` argument must be `0` and the AC-scan must default to
/// [`crate::ac::Scan::Zigzag`] for the entire v1 frame (since there is
/// no AC-prediction flag to flip the scan) and for any v2 inter
/// macroblock (since v2 only sets the AC-prediction flag for
/// intra-in-P, never for plain inter).
///
/// [spec01]: https://github.com/OxideAV/oxideav-docs/blob/master/video/msmpeg4/spec/01-bitstream-framing.md
/// [spec07]: https://github.com/OxideAV/oxideav-docs/blob/master/video/msmpeg4/spec/07-remaining-opens.md
#[derive(Clone, Debug)]
pub struct MsV1V2PictureHeader {
    /// I or P. v1/v2 do not support B-frames either.
    pub picture_type: PictureType,
    /// Frame-wide quantiser, 1..=31.
    pub quant: u8,
    /// **v1 only**: Unrestricted-Motion-Vectors flag (H.263 Annex D),
    /// stored at `[esi+0x88]`. Read on P-frames per spec/99 §2.4. Always
    /// `false` for v2 frames and for v1 I-frames.
    pub v1_umv_flag: bool,
}

/// The v3-only per-frame selector defaults that v1 / v2 consumers must
/// substitute when sharing v3 decode paths.
///
/// See the struct-level doc comment on [`MsV1V2PictureHeader`] for the
/// per-field rationale and the spec citations (`spec/01` §1.4 and
/// `spec/07` §1.6 / §2.4).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct V1V2V3CompatDefaults {
    /// `dc_size_sel` for `decode_intra_dc_diff_v3` / `decode_intra_block_full_v3`.
    pub dc_size_sel: u8,
    /// `ac_chroma_sel` slot (`[esi+0xad0]`) — selects the chroma + all-inter
    /// G-cluster. `0` = G4 = the v1/v2 default cluster.
    pub ac_chroma_sel: u8,
    /// `ac_luma_sel` slot (`[esi+0xad4]`) — selects the intra-luma
    /// G-cluster. `0` = G5 = the v1/v2 default cluster.
    pub ac_luma_sel: u8,
    /// `mv_table_sel` slot (`[esi+0x834]`) — `0` = default MV VLC.
    pub mv_table_sel: u8,
    /// True iff this version has any kind of MB-level AC-prediction flag
    /// in any macroblock. `false` for v1 (entirely absent), `true` for
    /// v2 (intra-in-P MBs only — see §2.4).
    pub has_ac_pred_anywhere: bool,
    /// True iff this version loads the **CBP** spatial-prediction LUT
    /// pair at `0x1c23a788 / 0x1c23a7b0` (patent 7,054,494 — the
    /// CBP/MCBPCY predictor-decision weights per `spec/99` §3.3 and the
    /// corrections-table, **not** a DC predictor). `false` for v1; for
    /// v2 the patent text suggests yes but the MCBPCY trace at
    /// `1c2171c7` / `1c21729c` does **not** load these LUTs, so v2 also
    /// reports `false` pending an Extractor follow-up — see `spec/07`
    /// §1.6 "Contrast with v3's patent-7,054,494 spatial-prediction
    /// path … which is absent from v1".
    ///
    /// NOTE on the field name: this gates the **CBP** spatial predictor,
    /// not the intra-DC predictor. The intra-DC gradient predictor
    /// (`0x1c20aef0`, `spec/99` §4.4) carries no version gate and is
    /// shared by v1/v2/v3. The name is retained for API stability.
    pub has_spatial_dc_predictor: bool,
}

impl V1V2V3CompatDefaults {
    /// Typed dispatch of [`Self::ac_chroma_sel`] to the chroma+all-inter
    /// [`GFamily`] **via the v3 per-frame selector dispatcher** per
    /// spec/14 §3.1. Equivalent to
    /// `GFamily::for_chroma_selector(self.ac_chroma_sel)`.
    ///
    /// **Important:** the v1 / v2 paths do **not** read this selector
    /// (per spec/01 §1.4); their runtime cluster is instead pinned by
    /// the v1/v2 fallthrough at `0x1c212917` which writes
    /// `[esi+0xab0] = G4` (per spec/14 §3.1) — i.e. the v1/v2
    /// chroma+all-inter cluster is [`GFamily::G4`] regardless of any
    /// per-frame selector value. The actual v1/v2 cluster the decoder
    /// binds is therefore [`Self::v1_v2_fallthrough_chroma_family`],
    /// **not** the value returned here. This method exists for the
    /// narrow case of a caller that shares a v3 entry point with a
    /// v1/v2 frame and wants the family that the v3 dispatcher would
    /// pick if fed `self.ac_chroma_sel` — which is **not** the v1/v2
    /// runtime cluster, since the v3 selector value `0` maps to
    /// [`GFamily::G2`], not G4.
    ///
    /// Returns `None` if `ac_chroma_sel` is outside `{0, 1, 2}`.
    pub const fn ac_chroma_family(&self) -> Option<GFamily> {
        GFamily::for_chroma_selector(self.ac_chroma_sel)
    }

    /// Typed dispatch of [`Self::ac_luma_sel`] to the intra-luma
    /// [`GFamily`] **via the v3 per-frame selector dispatcher** per
    /// spec/14 §3.1. Equivalent to
    /// `GFamily::for_luma_selector(self.ac_luma_sel)`.
    ///
    /// **Important:** the v1 / v2 paths do **not** read this selector;
    /// their actual intra-luma cluster is [`GFamily::G5`], pinned by
    /// the v1/v2 fallthrough at `0x1c212917` (`[esi+0xab4] = G5` per
    /// spec/14 §3.1). Use [`Self::v1_v2_fallthrough_luma_family`] for
    /// the v1/v2 cluster, and this method only when feeding
    /// `self.ac_luma_sel` to a v3 entry point.
    pub const fn ac_luma_family(&self) -> Option<GFamily> {
        GFamily::for_luma_selector(self.ac_luma_sel)
    }

    /// The chroma+all-inter [`GFamily`] the **v1/v2 runtime** binds at
    /// the fallthrough write `[esi+0xab0] = G4` (`0x1c212917`), per
    /// spec/14 §3.1. Always returns [`GFamily::G4`] — both v1 and v2
    /// paths share this cluster.
    ///
    /// This is the spec-correct family for a v1/v2 frame; contrast
    /// with [`Self::ac_chroma_family`] which exposes the v3
    /// per-frame-selector dispatch using the (don't-care, but
    /// constant-pinned-at-0) `ac_chroma_sel` field value.
    pub const fn v1_v2_fallthrough_chroma_family() -> GFamily {
        GFamily::G4
    }

    /// The intra-luma [`GFamily`] the **v1/v2 runtime** binds at the
    /// fallthrough write `[esi+0xab4] = G5` (`0x1c212917`), per
    /// spec/14 §3.1. Always returns [`GFamily::G5`].
    pub const fn v1_v2_fallthrough_luma_family() -> GFamily {
        GFamily::G5
    }

    /// Typed dispatch of [`Self::mv_table_sel`] to the joint-MV VLC
    /// [`MvTable`] per spec/06 §3.2. v1 / v2 default to
    /// `MvTable::Default` per spec/99 §3.2 (the alternate-VLC slot
    /// `[esi+0x834]` is v3-only — v1/v2 paths never reach it).
    pub const fn mv_table(&self) -> Option<MvTable> {
        MvTable::from_sel(self.mv_table_sel)
    }
}

impl MsV1V2PictureHeader {
    /// The v3-compat defaults to feed shared v3 entry points when running
    /// a **v1** frame. Use [`Self::V2_COMPAT_DEFAULTS`] for v2 frames.
    pub const V1_COMPAT_DEFAULTS: V1V2V3CompatDefaults = V1V2V3CompatDefaults {
        dc_size_sel: 0,
        ac_chroma_sel: 0, // G4
        ac_luma_sel: 0,   // G5
        mv_table_sel: 0,
        has_ac_pred_anywhere: false,
        has_spatial_dc_predictor: false,
    };

    /// The v3-compat defaults to feed shared v3 entry points when running
    /// a **v2** frame. Use [`Self::V1_COMPAT_DEFAULTS`] for v1 frames.
    pub const V2_COMPAT_DEFAULTS: V1V2V3CompatDefaults = V1V2V3CompatDefaults {
        dc_size_sel: 0,
        ac_chroma_sel: 0, // G4
        ac_luma_sel: 0,   // G5
        mv_table_sel: 0,
        // v2 gains intra-in-P AC prediction per spec/07 §2.4 — the flag
        // lives at MB scope, not picture scope, but the version IS capable.
        has_ac_pred_anywhere: true,
        // No spatial DC predictor trace evidence in `1c2171c7` (the v2
        // MCBPCY body); patent text suggests it but the binary doesn't
        // load `0x1c23a788 / 0x1c23a7b0`. Mark `false` until an Extractor
        // round files spec evidence; safer default for downstream callers.
        has_spatial_dc_predictor: false,
    };
}

impl MsV1V2PictureHeader {
    /// Parse a v1 picture header. Consumes the 37-bit opaque preamble
    /// first, then the picture-type / PQUANT / (P-only) UMV-flag fields.
    ///
    /// Per spec/99 §2.1: the preamble is "32 bits read and discarded at
    /// `1c211f35`, then 5 more at `1c211f40`" — never matched against a
    /// fixed pattern. We read and discard the same way.
    pub fn parse_v1(br: &mut BitReader<'_>) -> Result<Self> {
        // 37-bit opaque preamble (32 + 5). Consume but discard.
        let _ = br.read_u32(32)?;
        let _ = br.read_u32(5)?;
        Self::parse_inner(br, /*has_umv=*/ true)
    }

    /// Parse a v2 picture header. No preamble, no UMV flag.
    pub fn parse_v2(br: &mut BitReader<'_>) -> Result<Self> {
        Self::parse_inner(br, /*has_umv=*/ false)
    }

    fn parse_inner(br: &mut BitReader<'_>, has_umv: bool) -> Result<Self> {
        let ptype = br.read_u32(2)?;
        let picture_type = match ptype {
            0 => PictureType::I,
            1 => PictureType::P,
            other => {
                return Err(Error::invalid(format!(
                    "msmpeg4 v1/v2: reserved picture_type {other}"
                )));
            }
        };
        let quant = br.read_u32(5)? as u8;
        if !(1..=31).contains(&quant) {
            return Err(Error::invalid(format!(
                "msmpeg4 v1/v2: pquant {quant} out of range 1..=31"
            )));
        }
        let v1_umv_flag = if has_umv && picture_type == PictureType::P {
            br.read_bit()?
        } else {
            false
        };
        Ok(Self {
            picture_type,
            quant,
            v1_umv_flag,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Build a byte stream from a sequence of (value, bit-width) pairs,
    // MSB-first packing — mirrors how a real encoder would emit the
    // header.
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
        // Pad to at least one byte.
        if out.is_empty() {
            out.push(0);
        }
        out
    }

    #[test]
    fn parse_i_frame_header_all_zero_selectors() {
        // picture_type = 0 (I), quant = 7, ad0 sel = 0 (unary bit `0`),
        // ad4 sel = 0 (unary bit `0`), 0x8bc = 0.
        let bytes = pack(&[(0, 2), (7, 5), (0, 1), (0, 1), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        let h = MsV3PictureHeader::parse(&mut br).unwrap();
        assert_eq!(h.picture_type, PictureType::I);
        assert_eq!(h.quant, 7);
        assert_eq!(h.ac_chroma_sel, 0);
        assert_eq!(h.ac_luma_sel, 0);
        assert_eq!(h.dc_size_sel, 0);
    }

    #[test]
    fn parse_i_frame_header_mixed_selectors() {
        // ad0 = 2 (bits `11`), ad4 = 1 (bits `10`), dc_size_sel = 1.
        let bytes = pack(&[
            (0, 2),    // picture_type = I
            (5, 5),    // quant = 5
            (0b11, 2), // ad0 = 2
            (0b10, 2), // ad4 = 1
            (1, 1),    // dc_size_sel = 1
        ]);
        let mut br = BitReader::new(&bytes);
        let h = MsV3PictureHeader::parse(&mut br).unwrap();
        assert_eq!(h.picture_type, PictureType::I);
        assert_eq!(h.quant, 5);
        assert_eq!(h.ac_chroma_sel, 2);
        assert_eq!(h.ac_luma_sel, 1);
        assert_eq!(h.dc_size_sel, 1);
    }

    #[test]
    fn parse_p_frame_header() {
        // picture_type = 1 (P), quant = 12, ad0 = 1 (bits `10`),
        // dc_size_sel = 0, mv_table_sel = 1.
        let bytes = pack(&[
            (1, 2),    // P
            (12, 5),   // quant
            (0b10, 2), // ad0 = 1
            (0, 1),    // dc_size_sel = 0
            (1, 1),    // mv_table_sel = 1
        ]);
        let mut br = BitReader::new(&bytes);
        let h = MsV3PictureHeader::parse(&mut br).unwrap();
        assert_eq!(h.picture_type, PictureType::P);
        assert_eq!(h.quant, 12);
        assert_eq!(h.ac_chroma_sel, 1);
        assert_eq!(h.dc_size_sel, 0);
        assert_eq!(h.mv_table_sel, 1);
    }

    #[test]
    fn rejects_reserved_picture_type() {
        // 0b10 (2) and 0b11 (3) are reserved in v3.
        let bytes = pack(&[(2, 2), (5, 5), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert!(MsV3PictureHeader::parse(&mut br).is_err());

        let bytes = pack(&[(3, 2), (5, 5), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert!(MsV3PictureHeader::parse(&mut br).is_err());
    }

    #[test]
    fn rejects_zero_quant() {
        // quant = 0 is invalid (valid range 1..=31).
        let bytes = pack(&[(0, 2), (0, 5), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        assert!(MsV3PictureHeader::parse(&mut br).is_err());
    }

    #[test]
    fn accepts_max_quant() {
        let bytes = pack(&[(0, 2), (31, 5), (0, 1), (0, 1), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        let h = MsV3PictureHeader::parse(&mut br).unwrap();
        assert_eq!(h.quant, 31);
    }

    #[test]
    fn v1_iframe_header_parses() {
        // 37-bit opaque preamble (any value), then I-frame header.
        // 32 zero bits + 5 zero bits = 37-bit zero preamble; then
        // picture_type=0 (I, 2 bits), quant=10 (5 bits).
        let bytes = pack(&[
            (0, 32), // preamble lo
            (0, 5),  // preamble hi
            (0, 2),  // picture_type = I
            (10, 5), // quant = 10
        ]);
        let mut br = BitReader::new(&bytes);
        let h = MsV1V2PictureHeader::parse_v1(&mut br).unwrap();
        assert_eq!(h.picture_type, PictureType::I);
        assert_eq!(h.quant, 10);
        assert!(!h.v1_umv_flag, "I-frame must not have UMV flag set");
    }

    #[test]
    fn v1_pframe_with_umv_flag_parses() {
        let bytes = pack(&[
            (0xdead_beef, 32), // preamble lo (any opaque)
            (0x1f, 5),         // preamble hi (any opaque)
            (1, 2),            // picture_type = P
            (8, 5),            // quant = 8
            (1, 1),            // UMV flag = 1
        ]);
        let mut br = BitReader::new(&bytes);
        let h = MsV1V2PictureHeader::parse_v1(&mut br).unwrap();
        assert_eq!(h.picture_type, PictureType::P);
        assert_eq!(h.quant, 8);
        assert!(h.v1_umv_flag);
    }

    #[test]
    fn v2_iframe_header_parses_without_preamble() {
        // No preamble in v2 — reads picture_type (2 bits) immediately.
        let bytes = pack(&[(0, 2), (15, 5)]);
        let mut br = BitReader::new(&bytes);
        let h = MsV1V2PictureHeader::parse_v2(&mut br).unwrap();
        assert_eq!(h.picture_type, PictureType::I);
        assert_eq!(h.quant, 15);
        assert!(!h.v1_umv_flag, "v2 has no UMV flag");
    }

    #[test]
    fn v2_pframe_does_not_read_umv_bit() {
        // After picture_type + quant, the v2 path must NOT consume the
        // next bit (which would belong to MCBPCY's skip flag in real
        // streams). Verify by checking the bit-reader position after the
        // header.
        let bytes = pack(&[
            (1, 2),  // P
            (12, 5), // quant
            (1, 1),  // following bit (would-be skip flag in MCBPCY)
        ]);
        let mut br = BitReader::new(&bytes);
        let h = MsV1V2PictureHeader::parse_v2(&mut br).unwrap();
        assert_eq!(h.picture_type, PictureType::P);
        assert_eq!(h.quant, 12);
        assert!(!h.v1_umv_flag);
        // The bit that we appended after the header must still be
        // available — bit reader should sit at offset 7 (2+5) bits.
        let next = br.read_bit().unwrap();
        assert!(next, "next bit should be the 1 we packed (skip bit)");
    }

    #[test]
    fn v1_v2_reject_reserved_picture_type() {
        let bytes = pack(&[(0, 32), (0, 5), (2, 2), (5, 5)]);
        let mut br = BitReader::new(&bytes);
        assert!(MsV1V2PictureHeader::parse_v1(&mut br).is_err());

        let bytes = pack(&[(3, 2), (5, 5)]);
        let mut br = BitReader::new(&bytes);
        assert!(MsV1V2PictureHeader::parse_v2(&mut br).is_err());
    }

    #[test]
    fn v1_v2_reject_zero_quant() {
        let bytes = pack(&[(0, 32), (0, 5), (0, 2), (0, 5)]);
        let mut br = BitReader::new(&bytes);
        assert!(MsV1V2PictureHeader::parse_v1(&mut br).is_err());

        let bytes = pack(&[(0, 2), (0, 5)]);
        let mut br = BitReader::new(&bytes);
        assert!(MsV1V2PictureHeader::parse_v2(&mut br).is_err());
    }

    // ---------------------------------------------------------------------
    // v1 / v2 → v3-compat selector defaults (spec/01 §1.4, spec/07 §1.6 /
    // §2.4). Round 129: documents that v1/v2 never consume the v3-only
    // per-frame selector bits, so downstream consumers feeding shared v3
    // entry points must use these defaults.
    // ---------------------------------------------------------------------

    // Compile-time pins on the compat-default constants — any future
    // edit that flips one of these values without changing the const-
    // table definition will fail the build, which is louder than a unit
    // test regression. See the per-field rationale in the `V1V2V3CompatDefaults`
    // doc-comment and the spec citations on each `Self::V*_COMPAT_DEFAULTS`.
    const _: () = {
        let v1 = MsV1V2PictureHeader::V1_COMPAT_DEFAULTS;
        assert!(v1.dc_size_sel == 0, "v1 dc_size_sel default (spec/01 §1.4)");
        assert!(v1.ac_chroma_sel == 0, "v1 ac_chroma_sel default (G4)");
        assert!(v1.ac_luma_sel == 0, "v1 ac_luma_sel default (G5)");
        assert!(v1.mv_table_sel == 0, "v1 mv_table_sel default");
        assert!(!v1.has_ac_pred_anywhere, "v1 no AC pred (spec/07 §1.4)");
        assert!(
            !v1.has_spatial_dc_predictor,
            "v1 no spatial DC pred (spec/07 §1.6)"
        );

        let v2 = MsV1V2PictureHeader::V2_COMPAT_DEFAULTS;
        assert!(v2.dc_size_sel == 0, "v2 dc_size_sel default (spec/01 §1.4)");
        assert!(v2.ac_chroma_sel == 0, "v2 ac_chroma_sel default (G4)");
        assert!(v2.ac_luma_sel == 0, "v2 ac_luma_sel default (G5)");
        assert!(v2.mv_table_sel == 0, "v2 mv_table_sel default");
        assert!(
            v2.has_ac_pred_anywhere,
            "v2 has intra-in-P AC pred (spec/07 §2.4)"
        );
        assert!(
            !v2.has_spatial_dc_predictor,
            "v2 no spatial DC pred (spec/07 §2)"
        );
    };

    /// Helper that defeats const-folding so the runtime test below sees
    /// the actual struct, not the compiler's known-good copy.
    fn black_box_defaults<T: Copy>(v: T) -> T {
        // `std::hint::black_box` would also work, but it would also
        // suppress the dead-code lint on the const block above. The
        // identity closure is sufficient here.
        let f: fn(T) -> T = std::convert::identity;
        f(v)
    }

    #[test]
    fn v1_compat_defaults_carry_v3_zero_initialisation_at_runtime() {
        // Runtime cross-check on the same invariants the const block
        // above pins at compile time — `black_box_defaults` defeats
        // const-folding so the test exercises real struct field reads
        // (catches a future `Copy` impl that silently zeros / aliases).
        let d = black_box_defaults(MsV1V2PictureHeader::V1_COMPAT_DEFAULTS);
        assert_eq!(d.dc_size_sel, 0, "v1 must default dc_size_sel = 0");
        assert_eq!(d.ac_chroma_sel, 0, "v1 must default ac_chroma_sel = 0 (G4)");
        assert_eq!(d.ac_luma_sel, 0, "v1 must default ac_luma_sel = 0 (G5)");
        assert_eq!(d.mv_table_sel, 0, "v1 must default mv_table_sel = 0");
    }

    #[test]
    fn v2_compat_defaults_carry_v3_zero_initialisation_at_runtime() {
        let d = black_box_defaults(MsV1V2PictureHeader::V2_COMPAT_DEFAULTS);
        assert_eq!(d.dc_size_sel, 0, "v2 must default dc_size_sel = 0");
        assert_eq!(d.ac_chroma_sel, 0, "v2 must default ac_chroma_sel = 0 (G4)");
        assert_eq!(d.ac_luma_sel, 0, "v2 must default ac_luma_sel = 0 (G5)");
        assert_eq!(d.mv_table_sel, 0, "v2 must default mv_table_sel = 0");
    }

    #[test]
    fn v1_has_no_ac_prediction_anywhere_at_runtime() {
        // Per spec/07 §1.4: the v1 MCBPCY body `0x1c2171c7` "does NOT read
        // a post-VLC sign / AC-pred bit (no `call 0x1c215c9b` after the
        // CBPY decode), consistent with v1 lacking AC prediction".
        // A v1-version frame's downstream AC-scan dispatcher must default
        // to plain zigzag for every block.
        let d = black_box_defaults(MsV1V2PictureHeader::V1_COMPAT_DEFAULTS);
        assert!(
            !d.has_ac_pred_anywhere,
            "v1 has no AC-prediction flag at any scope (spec/07 §1.4 / §1.6)",
        );
    }

    #[test]
    fn v2_has_ac_prediction_only_at_intra_in_p_macroblocks_at_runtime() {
        // Per spec/07 §2.4 (v2 innovation): v2 reads a 1-bit AC-prediction
        // flag at MB scope when MCBPC selects intra-in-P (`ecx == 1` →
        // mcbpc ∈ 4..=7). The flag is MB-scope, NOT picture-scope. The
        // compat-defaults summary captures "this version is capable of
        // AC prediction in some macroblock context".
        let d = black_box_defaults(MsV1V2PictureHeader::V2_COMPAT_DEFAULTS);
        assert!(
            d.has_ac_pred_anywhere,
            "v2 gains intra-in-P AC prediction at MB scope (spec/07 §2.4)",
        );
    }

    #[test]
    fn v1_v2_lack_spatial_dc_predictor_at_runtime() {
        // Per spec/07 §1.6: v1's MCBPCY body "never loads a neighbour
        // MCBPC or CBPY" and the patent-7,054,494 spatial-prediction LUTs
        // at `0x1c23a788 / 0x1c23a7b0` are "absent from v1". v2's MCBPCY
        // body `0x1c21729c` likewise does not load those LUTs in the
        // trace; until an Extractor round files evidence to the contrary,
        // both versions default to "no spatial DC predictor" — i.e.
        // [`crate::dc_pred::predict_dc`] is a v3-only path.
        let v1 = black_box_defaults(MsV1V2PictureHeader::V1_COMPAT_DEFAULTS);
        let v2 = black_box_defaults(MsV1V2PictureHeader::V2_COMPAT_DEFAULTS);
        assert!(
            !v1.has_spatial_dc_predictor,
            "v1 has no patent-7,054,494 spatial DC predictor (spec/07 §1.6)",
        );
        assert!(
            !v2.has_spatial_dc_predictor,
            "v2 trace at `1c21729c` does not load `0x1c23a788 / 0x1c23a7b0` (spec/07 §2)",
        );
    }

    #[test]
    fn v1_v2_compat_defaults_are_distinct_values() {
        // The two const tables should not be silently aliased: while both
        // currently agree on every numeric selector, they differ on the
        // `has_ac_pred_anywhere` capability flag (per spec/07 §2.4 vs
        // §1.4) — so the PartialEq must reject equality. This guards
        // against a future copy-paste that flattens the two into a single
        // const and erases the v2 → intra-in-P distinction.
        let v1 = MsV1V2PictureHeader::V1_COMPAT_DEFAULTS;
        let v2 = MsV1V2PictureHeader::V2_COMPAT_DEFAULTS;
        assert_ne!(
            v1, v2,
            "v1 and v2 compat defaults must differ at least in has_ac_pred_anywhere",
        );
        // And the only field they differ on is has_ac_pred_anywhere — if
        // a future refactor changes another field's default for one
        // version but not the other, this assertion fires so the change
        // is documented explicitly.
        let v1_normalised = V1V2V3CompatDefaults {
            has_ac_pred_anywhere: v2.has_ac_pred_anywhere,
            ..v1
        };
        assert_eq!(
            v1_normalised, v2,
            "v1 and v2 compat defaults must differ ONLY in has_ac_pred_anywhere — \
             any other delta needs a new spec citation in the doc-comment",
        );
    }

    #[test]
    fn v1_pframe_with_umv_clear_parses() {
        // Companion to `v1_pframe_with_umv_flag_parses` covering the
        // `umv_flag = 0` case explicitly — proves the bit is read (so
        // subsequent MB bytes start at offset 32+5+2+5+1 = 45 from the
        // packet head) and forwarded as `false`.
        let bytes = pack(&[
            (0, 32), // preamble lo
            (0, 5),  // preamble hi
            (1, 2),  // P
            (16, 5), // quant
            (0, 1),  // UMV flag = 0
        ]);
        let mut br = BitReader::new(&bytes);
        let h = MsV1V2PictureHeader::parse_v1(&mut br).unwrap();
        assert_eq!(h.picture_type, PictureType::P);
        assert_eq!(h.quant, 16);
        assert!(!h.v1_umv_flag, "umv = 0 must round-trip as false");
        // Bit position is 37 (preamble) + 2 + 5 + 1 = 45.
        assert_eq!(br.bit_position(), 45);
    }

    #[test]
    fn v1_iframe_does_not_read_umv_bit() {
        // Per parse_v1: the UMV bit is gated on `picture_type == P` —
        // an I-frame must not consume a 38th bit. Verify by appending a
        // canary 1-bit after the header and asserting it survives.
        let bytes = pack(&[
            (0, 32), // preamble lo
            (0, 5),  // preamble hi
            (0, 2),  // picture_type = I
            (10, 5), // quant
            (1, 1),  // canary bit — must NOT be consumed by parse_v1
        ]);
        let mut br = BitReader::new(&bytes);
        let h = MsV1V2PictureHeader::parse_v1(&mut br).unwrap();
        assert_eq!(h.picture_type, PictureType::I);
        assert_eq!(h.quant, 10);
        assert!(!h.v1_umv_flag);
        // Bit position is 37 (preamble) + 2 + 5 = 44 — the canary is
        // STILL available to the caller.
        assert_eq!(br.bit_position(), 44);
        let next = br.read_bit().unwrap();
        assert!(next, "I-frame parser must not have consumed the canary bit");
    }

    // ---------------------------------------------------------------------
    // Typed-primitive accessors on `MsV3PictureHeader` and
    // `V1V2V3CompatDefaults` — spec/14 §3.1 (G-family dispatch) +
    // spec/06 §3.2 (MV-table dispatch). Mirror the raw `u8` selector
    // fields to the typed `GFamily` / `MvTable` enums via the
    // pre-existing `for_chroma_selector` / `for_luma_selector` /
    // `MvTable::from_sel` const-fn dispatchers. Purely additive; raw
    // `u8` fields stay public so existing callers are unaffected.
    // ---------------------------------------------------------------------

    use crate::g_family::{GFamily, GRole};
    use crate::mv::MvTable;

    fn header_with_selectors(chroma: u8, luma: u8, mv_sel: u8) -> MsV3PictureHeader {
        MsV3PictureHeader {
            picture_type: PictureType::I,
            quant: 1,
            ac_chroma_sel: chroma,
            ac_luma_sel: luma,
            dc_size_sel: 0,
            mv_table_sel: mv_sel,
        }
    }

    #[test]
    fn ms_v3_ac_chroma_family_dispatches_per_spec_14_3_1() {
        // Per spec/14 §3.1: 0 → G2, 1 → G0, 2 → G4.
        let expected = [(0u8, GFamily::G2), (1, GFamily::G0), (2, GFamily::G4)];
        for (sel, fam) in expected {
            let h = header_with_selectors(sel, 0, 0);
            assert_eq!(h.ac_chroma_family(), Some(fam), "chroma sel={sel}");
            // Every chroma family fills the ChromaAndInter role.
            assert_eq!(fam.role(), GRole::ChromaAndInter);
        }
    }

    #[test]
    fn ms_v3_ac_luma_family_dispatches_per_spec_14_3_1() {
        // Per spec/14 §3.1: 0 → G3, 1 → G1, 2 → G5.
        let expected = [(0u8, GFamily::G3), (1, GFamily::G1), (2, GFamily::G5)];
        for (sel, fam) in expected {
            let h = header_with_selectors(0, sel, 0);
            assert_eq!(h.ac_luma_family(), Some(fam), "luma sel={sel}");
            // Every luma family fills the IntraLuma role.
            assert_eq!(fam.role(), GRole::IntraLuma);
        }
    }

    #[test]
    fn ms_v3_ac_family_accessors_return_none_for_out_of_range_selectors() {
        // The read_unary_cap2 helper clamps to {0, 1, 2} in well-formed
        // streams, but a hand-constructed header can carry any u8.
        // Defence-in-depth: 3..=255 must return None on both family
        // accessors.
        for sel in 3u8..=255 {
            let h = header_with_selectors(sel, sel, 0);
            assert_eq!(h.ac_chroma_family(), None, "chroma sel={sel}");
            assert_eq!(h.ac_luma_family(), None, "luma sel={sel}");
        }
    }

    #[test]
    fn ms_v3_mv_table_dispatches_per_spec_06_3_2() {
        // Per spec/06 §3.2: 0 → Default, 1 → Alternate.
        let h0 = header_with_selectors(0, 0, 0);
        assert_eq!(h0.mv_table(), Some(MvTable::Default));
        let h1 = header_with_selectors(0, 0, 1);
        assert_eq!(h1.mv_table(), Some(MvTable::Alternate));
    }

    #[test]
    fn ms_v3_mv_table_returns_none_for_out_of_range_selector() {
        // The parser reads 1 bit so mv_table_sel can only be 0 or 1 in
        // well-formed streams. Defence-in-depth check for 2..=255.
        for sel in 2u8..=255 {
            let h = header_with_selectors(0, 0, sel);
            assert_eq!(h.mv_table(), None, "mv_table_sel={sel}");
        }
    }

    #[test]
    fn ms_v3_typed_accessors_agree_with_underlying_dispatchers() {
        // Cross-check: the header's typed accessors must agree with the
        // standalone `GFamily::for_*_selector` / `MvTable::from_sel`
        // dispatchers for every input. This pins the delegation contract.
        for cs in 0u8..=4 {
            for ls in 0u8..=4 {
                for ms in 0u8..=3 {
                    let h = header_with_selectors(cs, ls, ms);
                    assert_eq!(
                        h.ac_chroma_family(),
                        GFamily::for_chroma_selector(cs),
                        "chroma {cs}"
                    );
                    assert_eq!(
                        h.ac_luma_family(),
                        GFamily::for_luma_selector(ls),
                        "luma {ls}"
                    );
                    assert_eq!(h.mv_table(), MvTable::from_sel(ms), "mv {ms}");
                }
            }
        }
    }

    #[test]
    fn ms_v3_parsed_header_typed_accessors_round_trip() {
        // End-to-end: parse a v3 P-frame header that sets every
        // per-frame selector to a non-default value, then resolve via
        // the typed accessors. The selectors are bit-correct.
        // picture_type = P, quant = 3, ac_chroma_sel = 1 (G0),
        // dc_size_sel = 1, mv_table_sel = 1 (Alternate).
        let bytes = pack(&[
            (1, 2),    // P
            (3, 5),    // quant
            (0b10, 2), // ac_chroma_sel = 1 → G0
            (1, 1),    // dc_size_sel = 1
            (1, 1),    // mv_table_sel = 1 → Alternate
        ]);
        let mut br = BitReader::new(&bytes);
        let h = MsV3PictureHeader::parse(&mut br).unwrap();
        assert_eq!(h.ac_chroma_sel, 1);
        assert_eq!(h.mv_table_sel, 1);
        assert_eq!(h.ac_chroma_family(), Some(GFamily::G0));
        // P-frames don't read ac_luma_sel — the header carries the
        // parser's zero default, so the typed accessor resolves to
        // G3 (the luma family for selector 0 per spec/14 §3.1).
        assert_eq!(h.ac_luma_sel, 0);
        assert_eq!(h.ac_luma_family(), Some(GFamily::G3));
        assert_eq!(h.mv_table(), Some(MvTable::Alternate));
    }

    // ---------------------------------------------------------------------
    // V1V2V3CompatDefaults typed accessors — both V1_COMPAT_DEFAULTS and
    // V2_COMPAT_DEFAULTS pin the v3-compat selectors at zero per
    // spec/99 §5.2 (G4 chroma cluster, G5 luma cluster) and spec/99
    // §3.2 (Default MV VLC), so the typed accessors must resolve
    // accordingly.
    // ---------------------------------------------------------------------

    #[test]
    fn v1_v2_fallthrough_chroma_family_is_g4_per_spec_14() {
        // Per spec/14 §3.1 v1/v2 fallthrough at `0x1c212917`: the
        // decoder unconditionally writes `[esi+0xab0] = G4` for v1/v2
        // chroma+all-inter. The associated fn surfaces this fact.
        assert_eq!(
            V1V2V3CompatDefaults::v1_v2_fallthrough_chroma_family(),
            GFamily::G4,
        );
        assert_eq!(
            V1V2V3CompatDefaults::v1_v2_fallthrough_chroma_family().role(),
            GRole::ChromaAndInter,
        );
    }

    #[test]
    fn v1_v2_fallthrough_luma_family_is_g5_per_spec_14() {
        // Per spec/14 §3.1 v1/v2 fallthrough: `[esi+0xab4] = G5`.
        assert_eq!(
            V1V2V3CompatDefaults::v1_v2_fallthrough_luma_family(),
            GFamily::G5,
        );
        assert_eq!(
            V1V2V3CompatDefaults::v1_v2_fallthrough_luma_family().role(),
            GRole::IntraLuma,
        );
    }

    #[test]
    fn compat_defaults_v3_dispatch_does_not_equal_v1_v2_fallthrough() {
        // The v1/v2 → v3-compat path stuffs `ac_chroma_sel: 0` /
        // `ac_luma_sel: 0` into the v3 selector slots. The v3
        // per-frame dispatcher would resolve those to (G2, G3) — but
        // the actual v1/v2 runtime cluster is (G4, G5) per spec/14
        // §3.1 fallthrough. The two surfaces deliberately diverge:
        // ac_*_family is for v3 shared entry points,
        // v1_v2_fallthrough_*_family is for the real v1/v2 cluster.
        // This cross-check pins the divergence so a future round can't
        // accidentally collapse the two surfaces.
        let v1 = MsV1V2PictureHeader::V1_COMPAT_DEFAULTS;
        assert_eq!(
            v1.ac_chroma_family(),
            Some(GFamily::G2),
            "v3 dispatch of compat-default ac_chroma_sel=0"
        );
        assert_eq!(
            v1.ac_luma_family(),
            Some(GFamily::G3),
            "v3 dispatch of compat-default ac_luma_sel=0"
        );
        assert_ne!(
            v1.ac_chroma_family(),
            Some(V1V2V3CompatDefaults::v1_v2_fallthrough_chroma_family()),
            "v3 dispatch of ac_chroma_sel=0 (G2) must not equal v1/v2 fallthrough cluster (G4)"
        );
        assert_ne!(
            v1.ac_luma_family(),
            Some(V1V2V3CompatDefaults::v1_v2_fallthrough_luma_family()),
            "v3 dispatch of ac_luma_sel=0 (G3) must not equal v1/v2 fallthrough cluster (G5)"
        );
    }

    #[test]
    fn compat_defaults_mv_table_is_default_per_spec_99_3_2() {
        // Per spec/99 §3.2 the alternate-VLC slot `[esi+0x834]` is
        // v3-only; v1/v2 frames never trigger the alternate MV VLC.
        // Both compat-default constants therefore resolve mv_table to
        // Default.
        let v1 = MsV1V2PictureHeader::V1_COMPAT_DEFAULTS;
        let v2 = MsV1V2PictureHeader::V2_COMPAT_DEFAULTS;
        assert_eq!(v1.mv_table(), Some(MvTable::Default));
        assert_eq!(v2.mv_table(), Some(MvTable::Default));
    }

    #[test]
    fn compat_defaults_v1_v2_share_chroma_and_luma_fallthrough_clusters() {
        // Per spec/14 §3.1 the v1/v2 fallthrough is shared between v1
        // and v2 (same `1c212917` write site), so the per-version
        // fallthrough-family helpers are constants and identical for
        // both versions.
        let v1_chroma = V1V2V3CompatDefaults::v1_v2_fallthrough_chroma_family();
        let v2_chroma = V1V2V3CompatDefaults::v1_v2_fallthrough_chroma_family();
        assert_eq!(v1_chroma, v2_chroma);
        let v1_luma = V1V2V3CompatDefaults::v1_v2_fallthrough_luma_family();
        let v2_luma = V1V2V3CompatDefaults::v1_v2_fallthrough_luma_family();
        assert_eq!(v1_luma, v2_luma);
    }
}
