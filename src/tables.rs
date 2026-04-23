//! VLC tables common to MS-MPEG4v3.
//!
//! The tables in this module are inherited unchanged from H.263 Annex E
//! (CBPY) and from the MPEG-4 Part 2 intra-DC-size tables. MS-MPEG4v3
//! uses modified run/level coefficient tables (staged in follow-up
//! work) but shares the block-pattern and DC-size VLCs verbatim.
//!
//! ## Attribution
//!
//! CBPY codes are specified in ITU-T H.263, §5.3.5 — a public
//! standards document. No implementation-side attribution is required
//! for reproducing the table values.
//!
//! The intra DC-size tables follow MPEG-4 Part 2 (ISO/IEC 14496-2)
//! §6.3.8 and its Annex B. Same deal: published standard, table values
//! not copyrightable.

use crate::vlc::VlcEntry;

/// CBPY — coded-block-pattern for luma, 4-bit bitmask of which of the
/// 4 luma 8×8 blocks are coded. Intra / inter use the same codewords
/// but the mapping of 4-bit patterns is *inverted* for inter (so a
/// frequent "all coded" intra value pairs with a frequent "none coded"
/// inter value, keeping the table short).
///
/// Decoded value is the raw 4-bit pattern in INTRA orientation. Callers
/// for inter MBs must XOR with `0b1111`.
///
/// H.263 Table 8 / MPEG-4 Part 2 Table B-6.
#[rustfmt::skip]
pub const CBPY_INTRA_TABLE: &[VlcEntry<u8>] = &[
    VlcEntry::new(4, 0b0011,   0),
    VlcEntry::new(5, 0b00101,  1),
    VlcEntry::new(5, 0b00100,  2),
    VlcEntry::new(4, 0b1001,   3),
    VlcEntry::new(5, 0b00011,  4),
    VlcEntry::new(4, 0b0111,   5),
    VlcEntry::new(6, 0b000010, 6),
    VlcEntry::new(4, 0b1011,   7),
    VlcEntry::new(5, 0b00010,  8),
    VlcEntry::new(6, 0b000011, 9),
    VlcEntry::new(4, 0b0101,  10),
    VlcEntry::new(4, 0b1010,  11),
    VlcEntry::new(4, 0b0100,  12),
    VlcEntry::new(4, 0b1000,  13),
    VlcEntry::new(4, 0b0110,  14),
    VlcEntry::new(2, 0b11,    15),
];

/// Intra-DC size VLC for luma. Spec Table B-13 (MPEG-4 Part 2 Annex B)
/// — the "intra DC additional" magnitude categorisation used by
/// MS-MPEG4v3 as well.
///
/// Decoded value is the bit-count of the DC-differential magnitude
/// (0..=12); the caller reads that many additional bits for the value.
#[rustfmt::skip]
pub const DC_SIZE_LUMA_TABLE: &[VlcEntry<u8>] = &[
    VlcEntry::new(3,  3,  0),
    VlcEntry::new(2,  3,  1),
    VlcEntry::new(2,  2,  2),
    VlcEntry::new(3,  2,  3),
    VlcEntry::new(3,  1,  4),
    VlcEntry::new(4,  1,  5),
    VlcEntry::new(5,  1,  6),
    VlcEntry::new(6,  1,  7),
    VlcEntry::new(7,  1,  8),
    VlcEntry::new(8,  1,  9),
    VlcEntry::new(9,  1, 10),
    VlcEntry::new(10, 1, 11),
    VlcEntry::new(11, 1, 12),
];

/// Intra-DC size VLC for chroma. Spec Table B-14.
#[rustfmt::skip]
pub const DC_SIZE_CHROMA_TABLE: &[VlcEntry<u8>] = &[
    VlcEntry::new(2,  3,  0),
    VlcEntry::new(2,  2,  1),
    VlcEntry::new(2,  1,  2),
    VlcEntry::new(3,  1,  3),
    VlcEntry::new(4,  1,  4),
    VlcEntry::new(5,  1,  5),
    VlcEntry::new(6,  1,  6),
    VlcEntry::new(7,  1,  7),
    VlcEntry::new(8,  1,  8),
    VlcEntry::new(9,  1,  9),
    VlcEntry::new(10, 1, 10),
    VlcEntry::new(11, 1, 11),
    VlcEntry::new(12, 1, 12),
];

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vlc;
    use oxideav_core::bits::BitReader;

    fn pack_bits(codes: &[(u32, u32)]) -> Vec<u8> {
        let mut out: Vec<u8> = Vec::new();
        let mut acc: u64 = 0;
        let mut bits: u32 = 0;
        for (v, w) in codes {
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
        out
    }

    #[test]
    fn cbpy_intra_all_codewords_decode() {
        for e in CBPY_INTRA_TABLE {
            let bytes = pack_bits(&[(e.code, e.bits as u32), (0, 32)]);
            let mut br = BitReader::new(&bytes);
            let v = vlc::decode(&mut br, CBPY_INTRA_TABLE).unwrap();
            assert_eq!(v, e.value, "CBPY codeword roundtrip failed");
        }
    }

    #[test]
    fn cbpy_intra_most_common() {
        // CBPY = 15 (0b1111 in INTRA; all 4 coded) has shortest code `11`.
        let bytes = pack_bits(&[(0b11, 2), (0, 32)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(vlc::decode(&mut br, CBPY_INTRA_TABLE).unwrap(), 15);
    }

    #[test]
    fn dc_size_luma_all_decode() {
        for e in DC_SIZE_LUMA_TABLE {
            let bytes = pack_bits(&[(e.code, e.bits as u32), (0, 32)]);
            let mut br = BitReader::new(&bytes);
            let v = vlc::decode(&mut br, DC_SIZE_LUMA_TABLE).unwrap();
            assert_eq!(v, e.value);
        }
    }

    #[test]
    fn dc_size_chroma_all_decode() {
        for e in DC_SIZE_CHROMA_TABLE {
            let bytes = pack_bits(&[(e.code, e.bits as u32), (0, 32)]);
            let mut br = BitReader::new(&bytes);
            let v = vlc::decode(&mut br, DC_SIZE_CHROMA_TABLE).unwrap();
            assert_eq!(v, e.value);
        }
    }

    #[test]
    fn dc_size_luma_prefixes_unique() {
        // Prefix uniqueness: no codeword is a prefix of another.
        for (i, a) in DC_SIZE_LUMA_TABLE.iter().enumerate() {
            for (j, b) in DC_SIZE_LUMA_TABLE.iter().enumerate() {
                if i == j || a.bits == b.bits {
                    continue;
                }
                let (short, long) = if a.bits < b.bits { (a, b) } else { (b, a) };
                let shift = long.bits - short.bits;
                let long_prefix = long.code >> shift;
                assert_ne!(
                    long_prefix, short.code,
                    "{:b} is a prefix of {:b}",
                    short.code, long.code
                );
            }
        }
    }
}
