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
}
