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

use crate::tables_data::{MCBPCY_V3_PARTITION, MCBPCY_V3_RAW};
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
    /// True if this MB is intra-coded (spec/05 §3.2 step 3: `test bl,
    /// 0x40`). Empirically, the partition test is "is bit 6 of the
    /// decoded idx set" — high-half of the 128-alphabet. spec/99 §3.1's
    /// wording on which half is I vs P is ambiguous; the binary uses
    /// `test bl,0x40` followed by "MB-type = 3 (intra-in-P)" on the
    /// set-branch, so high half = intra.
    pub is_intra: bool,
    /// 4-bit mask: bit `i` = 1 if luma block `i` has coded AC. Bits are
    /// stored with block 0 at MSB (bit 3), block 3 at LSB (bit 0).
    pub cbpy: u8,
    /// CBP bit for the Cb chroma block (1 = coded).
    pub cbp_cb: bool,
    /// CBP bit for the Cr chroma block (1 = coded).
    pub cbp_cr: bool,
}

/// Decode one MCBPCY joint symbol from the bit stream using the v3
/// 128-entry canonical-Huffman table, and split the resulting index
/// into the (MB-type, 6-bit CBP) components.
///
/// Per `docs/video/msmpeg4/spec/05-ab0-resolution.md` §3.2 step 3-4:
/// `test bl, 0x40` separates the "all-zero-block / no-skip" prefix
/// from the main payload; `je 0x1c2178bd` then sets `MB-type = 3`
/// (intra-in-P) on the SET path.
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
    // Partition test: bit 6 of idx == (idx >= MCBPCY_V3_PARTITION).
    let is_intra = (idx as usize) >= MCBPCY_V3_PARTITION;
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
}
