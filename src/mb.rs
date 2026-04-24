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

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::ac::{decode_intra_ac, AcVlcTable, Scan};
use crate::iq::{dc_scaler, dequantise_h263};
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
    let dc_diff = decode_intra_dc_diff(br, block_idx)?;
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
            "msmpeg4v3 intra AC VLC: placeholder table in use — no concrete \
             (symbol, bit_length) pairs available yet. Candidate source VMAs \
             per docs/video/msmpeg4/spec/03-corrections.md §5.3: 0x1c25fad0 \
             (file offset 0x5eed0) and 0x1c25f6c8 (file offset 0x5eac8). \
             Both flagged OPEN in §6 and provenance/03-corrections.md line \
             101. No Extractor dump exists for either offset — see \
             tables/README.md. Extractor must produce a region dump and \
             translate it into a VlcEntry<Symbol> slice before this code \
             path can decode real bitstreams.",
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
        // The error must pin down *both* candidate VMAs and cite the
        // spec doc section so whoever reads the failure can find the
        // OPEN line without grep-diving.
        assert!(msg.contains("0x1c25fad0"), "msg = {msg}");
        assert!(msg.contains("0x1c25f6c8"), "msg = {msg}");
        assert!(msg.contains("§5.3"), "msg = {msg}");
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
}
