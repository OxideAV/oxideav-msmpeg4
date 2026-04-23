//! MS-MPEG4 picture-header parser.
//!
//! Microsoft MPEG-4 has no VOS / VOL / VOP start-code layering. Each
//! coded frame begins directly with a bit-level picture header followed
//! by macroblock data. The header syntax is version-specific and was
//! reverse-engineered in the early 2000s (the original "OpenDivX" and
//! subsequently FFmpeg / libavcodec reference).
//!
//! ## v3 (DIV3 / MP43 / MPG3) picture header
//!
//! Fields, MSB-first:
//!
//! | Bits | Name                 | Meaning                                     |
//! | ---- | -------------------- | ------------------------------------------- |
//! | 2    | `picture_type_bits`  | `0b00` = I, `0b01` = P, others reserved     |
//! | 5    | `pquant`             | Frame quantiser scale, 1..=31               |
//! | 1    | `flag`               | "extra" / buggy flag (often 0)              |
//! |      | *(I-frame only)*     |                                             |
//! | 5    | `slice_height`       | Slice height in MBs (wraps frame on simple) |
//! |      | *(P-frame only)*     |                                             |
//! | 1    | `use_src_quant`      | If 1, use fixed quant table                 |
//! | 1    | `mv_table_select`    | 0 = table A, 1 = table B                    |
//! | ...  | `rl_table_select`    | Implicit from earlier bits                  |
//!
//! Note on simplifications: The exact on-wire layout has a few
//! version/sub-version dependent bits we treat as "skipped" for now in
//! the parser — enough to get a legal I-frame header decoded. When we
//! extend P-frame support more fields need honoring.

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
    /// Raw "extra" flag bit — some encoders set this to signal a
    /// non-standard mode. We propagate it so the MB layer can react.
    pub flag: bool,
    /// For I-frames: slice height in MBs. `0` means "whole frame is one
    /// slice" which is the common case for DivX 3 captures.
    pub slice_height: u8,
    /// For P-frames: which run/level table to use.
    pub rl_table_select: u8,
    /// For P-frames: which MV table to use.
    pub mv_table_select: u8,
    /// For P-frames: whether to honor `quant` for the frame or use the
    /// per-MB delta path only.
    pub use_src_quant: bool,
}

impl MsV3PictureHeader {
    /// Parse the picture header at the current bitstream position.
    ///
    /// The first packet in a stream always begins with the header; the
    /// caller should construct `br` over the entire packet payload.
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

        let flag = br.read_bit()?;

        let mut slice_height = 0u8;
        let mut rl_table_select = 0u8;
        let mut mv_table_select = 0u8;
        let mut use_src_quant = false;

        match picture_type {
            PictureType::I => {
                // 5 bits slice height (0 means whole frame).
                slice_height = br.read_u32(5)? as u8;
            }
            PictureType::P => {
                use_src_quant = br.read_bit()?;
                mv_table_select = br.read_u32(1)? as u8;
                rl_table_select = br.read_u32(1)? as u8;
            }
        }

        Ok(Self {
            picture_type,
            quant,
            flag,
            slice_height,
            rl_table_select,
            mv_table_select,
            use_src_quant,
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
    fn parse_i_frame_header() {
        // picture_type = 0 (I), quant = 7, flag = 0, slice_height = 0.
        let bytes = pack(&[(0, 2), (7, 5), (0, 1), (0, 5), (0, 3)]);
        let mut br = BitReader::new(&bytes);
        let h = MsV3PictureHeader::parse(&mut br).unwrap();
        assert_eq!(h.picture_type, PictureType::I);
        assert_eq!(h.quant, 7);
        assert!(!h.flag);
        assert_eq!(h.slice_height, 0);
    }

    #[test]
    fn parse_p_frame_header() {
        // picture_type = 1 (P), quant = 12, flag = 1,
        // use_src_quant = 0, mv_table = 1, rl_table = 0.
        let bytes = pack(&[(1, 2), (12, 5), (1, 1), (0, 1), (1, 1), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        let h = MsV3PictureHeader::parse(&mut br).unwrap();
        assert_eq!(h.picture_type, PictureType::P);
        assert_eq!(h.quant, 12);
        assert!(h.flag);
        assert!(!h.use_src_quant);
        assert_eq!(h.mv_table_select, 1);
        assert_eq!(h.rl_table_select, 0);
    }

    #[test]
    fn rejects_reserved_picture_type() {
        // 0b10 (2) and 0b11 (3) are reserved in v3.
        let bytes = pack(&[(2, 2), (5, 5), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        assert!(MsV3PictureHeader::parse(&mut br).is_err());

        let bytes = pack(&[(3, 2), (5, 5), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        assert!(MsV3PictureHeader::parse(&mut br).is_err());
    }

    #[test]
    fn rejects_zero_quant() {
        // quant = 0 is invalid (valid range 1..=31).
        let bytes = pack(&[(0, 2), (0, 5), (0, 1), (0, 5), (0, 3)]);
        let mut br = BitReader::new(&bytes);
        assert!(MsV3PictureHeader::parse(&mut br).is_err());
    }

    #[test]
    fn accepts_max_quant() {
        let bytes = pack(&[(0, 2), (31, 5), (1, 1), (3, 5), (0, 3)]);
        let mut br = BitReader::new(&bytes);
        let h = MsV3PictureHeader::parse(&mut br).unwrap();
        assert_eq!(h.quant, 31);
        assert!(h.flag);
        assert_eq!(h.slice_height, 3);
    }
}
