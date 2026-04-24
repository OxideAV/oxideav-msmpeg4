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
}
