//! Macroblock-level decode for MS-MPEG4v3.
//!
//! An MS-MPEG4v3 intra MB consists of:
//! * 1 bit  `ac_pred_flag`   — is AC prediction active for this MB?
//! * VLC    `CBPY`           — coded-block-pattern for the 4 luma blocks (the
//!                              two chroma blocks use a separate CBP field
//!                              that lives in the MB-type escape chain and is
//!                              decoded elsewhere; stubbed here)
//! * per-block:               — one sub-block per the 6 (4 luma + 2 chroma)
//!   * DC differential (DC-size VLC + that many bits of magnitude, if any)
//!   * if this block's CBP bit is set: AC coefficients (run/level VLC — not
//!     yet implemented, see [`crate::tables`])
//!
//! This module implements everything **up to** the AC walk, including DC
//! prediction and bookkeeping of prediction-cache state. The AC walk
//! currently returns [`Error::Unsupported`] pending the run/level
//! tables.

use oxideav_core::bits::BitReader;
use oxideav_core::Result;

use crate::iq::dc_scaler;
use crate::tables::{CBPY_INTRA_TABLE, DC_SIZE_CHROMA_TABLE, DC_SIZE_LUMA_TABLE};
use crate::vlc;

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
#[derive(Clone, Copy, Debug)]
pub struct IntraMbHeader {
    pub ac_pred: bool,
    /// 4-bit mask: bit `i` = 1 if luma block `i` has coded AC
    /// coefficients. Luma blocks are ordered raster (top-left,
    /// top-right, bottom-left, bottom-right).
    pub cbpy: u8,
}

impl IntraMbHeader {
    pub fn parse(br: &mut BitReader<'_>) -> Result<Self> {
        let ac_pred = br.read_bit()?;
        let cbpy = vlc::decode(br, CBPY_INTRA_TABLE)?;
        Ok(Self { ac_pred, cbpy })
    }
}

/// Decode the intra DC differential at the current bit position for a
/// single 8×8 block. Returns the signed DC residual (in the post-scaler
/// / pel domain if multiplied by `dc_scaler(block_idx, quant)`; the
/// caller does that multiplication).
///
/// `block_idx` 0..=3 selects the luma table, 4..=5 selects chroma.
pub fn decode_intra_dc_diff(br: &mut BitReader<'_>, block_idx: usize) -> Result<i32> {
    let table = if block_idx < 4 {
        DC_SIZE_LUMA_TABLE
    } else {
        DC_SIZE_CHROMA_TABLE
    };
    let size = vlc::decode(br, table)? as u32;
    if size == 0 {
        return Ok(0);
    }
    // `size` unsigned bits of DC value. The MSB is the sign — MPEG-4
    // Part 2 §6.3.8: value = raw if MSB set, else raw - (2^size - 1).
    // MS-MPEG4v3 uses the same sign encoding.
    let raw = br.read_u32(size)? as i32;
    let msb_set = raw & (1 << (size - 1)) != 0;
    let value = if msb_set {
        raw
    } else {
        raw - ((1 << size) - 1)
    };
    // Marker bit for sizes >= 8 — MS-MPEG4v3 retains this from MPEG-4.
    if size > 8 {
        let _marker = br.read_u1()?;
    }
    Ok(value)
}

/// Reconstruct the DC coefficient value for one intra block, combining
/// the bitstream differential with a predicted DC (provided by the
/// caller from the neighbour cache).
pub fn reconstruct_intra_dc(dc_diff: i32, pred_dc: i32, block_idx: usize, quant: u32) -> i32 {
    let scaler = dc_scaler(block_idx, quant) as i32;
    pred_dc + dc_diff * scaler
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
    }

    #[test]
    fn parse_intra_mb_header_no_ac() {
        // ac_pred = 0; CBPY = 0 (`0011`).
        let bytes = pack(&[(0, 1), (0b0011, 4), (0, 8)]);
        let mut br = BitReader::new(&bytes);
        let h = IntraMbHeader::parse(&mut br).unwrap();
        assert!(!h.ac_pred);
        assert_eq!(h.cbpy, 0);
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
}
