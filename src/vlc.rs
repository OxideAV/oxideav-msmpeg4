//! Linear-scan VLC decoder shared by the MS-MPEG4 tables.
//!
//! MS-MPEG4 VLC tables are modest in size (the largest, the intra/inter
//! run-level tables, have a few hundred entries). A linear scan per
//! codeword is fast enough for software decoding and keeps each entry
//! obvious to audit against the spec.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

/// One entry in a VLC table. `code` occupies the low `bits` bits MSB-first.
#[derive(Clone, Copy, Debug)]
pub struct VlcEntry<T: Copy> {
    pub bits: u8,
    pub code: u32,
    pub value: T,
}

impl<T: Copy> VlcEntry<T> {
    pub const fn new(bits: u8, code: u32, value: T) -> Self {
        Self { bits, code, value }
    }
}

/// Decode one symbol using linear scan over `table`.
///
/// The caller must ensure the table is non-empty and its `bits` field
/// is the true prefix length for each entry. On ambiguous matches
/// (shouldn't happen for a valid table) the first match wins.
pub fn decode<T: Copy>(br: &mut BitReader<'_>, table: &[VlcEntry<T>]) -> Result<T> {
    let max_bits = table.iter().map(|e| e.bits).max().unwrap_or(0) as u32;
    if max_bits == 0 {
        return Err(Error::invalid("msmpeg4 vlc: empty table"));
    }
    let remaining = br.bits_remaining() as u32;
    let peek_bits = max_bits.min(remaining);
    if peek_bits == 0 {
        return Err(Error::invalid("msmpeg4 vlc: no bits available"));
    }
    let peeked = br.peek_u32(peek_bits)?;
    let peeked_full = peeked << (max_bits - peek_bits);
    for e in table {
        if (e.bits as u32) > peek_bits {
            continue;
        }
        let shift = max_bits - e.bits as u32;
        let prefix = peeked_full >> shift;
        if prefix == e.code {
            br.consume(e.bits as u32)?;
            return Ok(e.value);
        }
    }
    Err(Error::invalid("msmpeg4 vlc: no matching codeword"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_entry_table() {
        let t = [VlcEntry::new(1, 0b1, 42u32)];
        let data = [0x80u8];
        let mut br = BitReader::new(&data);
        assert_eq!(decode(&mut br, &t).unwrap(), 42);
    }

    #[test]
    fn multi_entry_prefix_match() {
        // 1 -> 'A', 01 -> 'B', 001 -> 'C', 000 -> 'D'.
        let t = [
            VlcEntry::new(1, 0b1, 'A'),
            VlcEntry::new(2, 0b01, 'B'),
            VlcEntry::new(3, 0b001, 'C'),
            VlcEntry::new(3, 0b000, 'D'),
        ];
        // Encoded: A B C D -> bits "1 01 001 000" = 1010 0100 0... -> 0xA4 0x00.
        let data = [0xA4u8, 0x00];
        let mut br = BitReader::new(&data);
        assert_eq!(decode(&mut br, &t).unwrap(), 'A');
        assert_eq!(decode(&mut br, &t).unwrap(), 'B');
        assert_eq!(decode(&mut br, &t).unwrap(), 'C');
        assert_eq!(decode(&mut br, &t).unwrap(), 'D');
    }

    #[test]
    fn unknown_codeword_errors() {
        // Only 1 defined, everything else error.
        let t = [VlcEntry::new(1, 0b1, 0u32)];
        let data = [0x00u8];
        let mut br = BitReader::new(&data);
        assert!(decode(&mut br, &t).is_err());
    }
}
