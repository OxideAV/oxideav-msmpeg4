//! Integration tests for the G5 (intra-luma) DCT AC TCOEF primary VLC
//! wired in round 26 from the packed-Huffman source at file `0x59178` /
//! VMA `0x1c259d78` per
//! `docs/video/msmpeg4/spec/11-walker-format-resolved.md`.
//!
//! These tests exercise the canonical-Huffman walker against synthetic
//! bit streams using the codes drawn directly from the binary's source
//! table (not regenerated). They prove:
//!
//! * The shortest code (`10` = 2-bit) decodes to (run=0, level=1, last=0).
//! * Each non-ESC entry's `code` round-trips through the decoder.
//! * The ESC entry maps to `Symbol::Escape` and the escape body reads.
//! * The complete G5 table is prefix-free (the decoder cannot deadlock).

use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::ac::{decode_token, AcVlcTable, Symbol, Token};
use oxideav_msmpeg4::g_descriptor::{g5_decode, GSymbol};
use oxideav_msmpeg4::tables_data::{G5_PRIMARY_ESC_INDEX, G5_PRIMARY_RAW};

/// Bit-pack helper: each `(value, width)` is appended MSB-first.
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
    out.extend_from_slice(&[0u8; 8]);
    out
}

#[test]
fn g5_table_has_104_entries() {
    let table = AcVlcTable::v3_intra_g5();
    assert_eq!(
        table.entries.len(),
        104,
        "G5 has 102 alphabet + 1 ESC codeword at idx 102 + the reserved \
         9-bit `000000000` ESC-marker entry (spec/11 §7 item 4) = 104"
    );
}

#[test]
fn g5_shortest_code_is_two_bit_run0_level1() {
    // The 2-bit code `10` (= raw u32 `2`) lives at idx 0, which the G5
    // descriptor maps to (last=false, run=0, level=1) per audit/01 §4.1.
    let table = AcVlcTable::v3_intra_g5();
    let bytes = pack(&[(0b10, 2), (0, 1)]); // VLC + sign=0 (positive)
    let mut br = BitReader::new(&bytes);
    let tok = decode_token(&mut br, &table).expect("decode shortest VLC");
    assert_eq!(
        tok,
        Token {
            last: false,
            run: 0,
            level: 1,
        },
        "G5 shortest code (idx 0 = 2-bit `10`) must decode to (run=0, level=1, last=0)"
    );
}

#[test]
fn g5_three_bit_code_is_run0_level2() {
    // idx 1 in G5 source: code = 6 (`110`), bl = 3. Per audit/01 §4.1
    // sub-A row 0 covers level 1..27 at run 0; idx 1 is level 2.
    let table = AcVlcTable::v3_intra_g5();
    let bytes = pack(&[(0b110, 3), (0, 1)]);
    let mut br = BitReader::new(&bytes);
    let tok = decode_token(&mut br, &table).expect("decode bl=3 code");
    assert_eq!(tok.run, 0);
    assert_eq!(tok.level, 2);
    assert!(!tok.last);
}

#[test]
fn g5_every_non_esc_entry_round_trips() {
    // Every G5 non-ESC entry in the binary's source must round-trip
    // through decode_token: pack VLC + sign 0, decode, get back
    // (last, run, level_mag) consistent with g5_decode(idx).
    let table = AcVlcTable::v3_intra_g5();
    for (idx, &(bl, code)) in G5_PRIMARY_RAW.iter().enumerate() {
        if idx == G5_PRIMARY_ESC_INDEX {
            continue;
        }
        if bl == 0 {
            // Hole sentinel — never appears in stream.
            continue;
        }
        let bytes = pack(&[(code, bl), (0, 1)]); // sign=0 → positive level
        let mut br = BitReader::new(&bytes);
        let tok = decode_token(&mut br, &table)
            .unwrap_or_else(|e| panic!("idx {idx} (bl={bl}, code={code}): decode failed: {e}"));
        match g5_decode(idx) {
            Some(GSymbol::Token(t)) => {
                assert_eq!(tok.last, t.last, "idx {idx}: last mismatch");
                assert_eq!(tok.run, t.run, "idx {idx}: run mismatch");
                assert_eq!(
                    tok.level, t.level_mag as i16,
                    "idx {idx}: level mismatch (expected sign=+, mag={})",
                    t.level_mag
                );
            }
            other => panic!("idx {idx}: g5_decode unexpectedly returned {:?}", other),
        }
    }
}

#[test]
fn g5_esc_codeword_decodes_via_3_tier_escape_body() {
    // Round 27 wired the v3 intra 3-tier escape body per
    // `docs/video/msmpeg4/spec/04-decoder-kernels.md` §2.3. The G5
    // ESC sentinel (a single bl=9 codeword) selects tier 1; a second
    // ESC re-fire selects tier 2; a third selects the verbatim
    // fixed-length triple (1 bit last + 6 bits run + 8 bits signed
    // level). This test exercises the verbatim tier (the only path
    // the inter kernel uses, per spec/04 §1.3 step 10).
    let table = AcVlcTable::v3_intra_g5();
    let esc_entry = table
        .entries
        .iter()
        .find(|e| matches!(e.value, Symbol::Escape))
        .expect("G5 must have a Symbol::Escape entry");
    let bytes = pack(&[
        (esc_entry.code, esc_entry.bits as u32),
        (esc_entry.code, esc_entry.bits as u32),
        (esc_entry.code, esc_entry.bits as u32),
        (1, 1),    // last = 1
        (5, 6),    // run = 5
        (0xfd, 8), // level = -3 as 8-bit signed
    ]);
    let mut br = BitReader::new(&bytes);
    let tok = decode_token(&mut br, &table).expect("decode ESC verbatim tier");
    assert!(tok.last, "verbatim tier should set last=1");
    assert_eq!(tok.run, 5);
    assert_eq!(tok.level, -3);
}

#[test]
fn g5_table_is_prefix_free() {
    // No entry's code may be a prefix of another's. Without this, the
    // linear-scan decoder may match the wrong entry on bit-aligned input.
    let table = AcVlcTable::v3_intra_g5();
    for (i, a) in table.entries.iter().enumerate() {
        for (j, b) in table.entries.iter().enumerate() {
            if i == j || a.bits == b.bits {
                continue;
            }
            let (short, long) = if a.bits < b.bits { (a, b) } else { (b, a) };
            let shift = long.bits - short.bits;
            let long_prefix = long.code >> shift;
            assert_ne!(
                long_prefix, short.code,
                "G5: short code 0x{:x}/bl={} is a prefix of long code 0x{:x}/bl={}",
                short.code, short.bits, long.code, long.bits
            );
        }
    }
}

#[test]
fn g5_alphabet_partition_matches_audit() {
    // Per spec/99 §5: G5 has count_A=102, count_B=66.
    // Sub-class A (idx 0..=66, last=false): 67 entries.
    // Sub-class B (idx 67..=101, last=true): 35 entries.
    // ESC: idx 102's codeword + the reserved 9-bit marker = 2 entries.
    let table = AcVlcTable::v3_intra_g5();
    let last_false_count = table
        .entries
        .iter()
        .filter(|e| matches!(e.value, Symbol::RunLevel { last: false, .. }))
        .count();
    let last_true_count = table
        .entries
        .iter()
        .filter(|e| matches!(e.value, Symbol::RunLevel { last: true, .. }))
        .count();
    let esc_count = table
        .entries
        .iter()
        .filter(|e| matches!(e.value, Symbol::Escape))
        .count();
    assert_eq!(last_false_count, 67, "sub-A count");
    assert_eq!(last_true_count, 35, "sub-B count");
    // Two ESC entries: the idx-102 codeword (`0000011`, 7 bits; sym ==
    // count_A per spec/15) plus the reserved 9-bit `000000000` marker
    // covering the Kraft gap (spec/11 §7 item 4) that real MS-encoded
    // streams emit (tests/microsoft_fixtures.rs).
    assert_eq!(esc_count, 2, "idx-102 ESC + reserved 9-bit ESC marker");
    assert_eq!(
        last_false_count + last_true_count + esc_count,
        104,
        "total = 104"
    );
}
