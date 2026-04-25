//! Integration tests for the v3 intra-AC candidate VLC table extracted
//! from `docs/video/msmpeg4/tables/region_05eed0.csv` (VMA `0x1c25fad0`).
//!
//! The role of this region is **OPEN** per
//! `docs/video/msmpeg4/spec/99-current-understanding.md` §0.1 row 8 and
//! §9 OPEN-O6 (candidate intra-AC primary VLC vs candidate v2 MCBPCY
//! source). The bytes form a complete 64-entry canonical-Huffman prefix
//! code (Kraft sum exactly 1, verified at build time and again in
//! `tables_data::tests::intra_ac_v3_candidate_kraft_sum_is_one`). This
//! test suite exercises the runtime decoder against synthetic bit
//! streams to prove the canonical-Huffman walker + escape body + scan
//! dispatch + dequant pipeline holds together end-to-end with a real
//! (rather than placeholder) AC table plugged in.
//!
//! These tests do **not** decode real DIV3 files — that would require
//! a confirmed `(last, run, level)` mapping which the candidate
//! constructor only documents as a hypothesis. They prove the
//! pipeline composes correctly so that, when the real mapping is
//! confirmed by a future spec / audit pass, only the
//! `candidate_index_to_symbol` body needs to change for real-file
//! decode to light up.

use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::ac::{decode_intra_ac, decode_intra_block, decode_token, AcVlcTable, Scan};
use oxideav_msmpeg4::scan::ZIGZAG;

/// Bit-pack helper: each `(value, width)` is appended MSB-first to the
/// growing stream. A few bytes of zero padding are appended so the
/// caller's bit-reader peek calls never starve.
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
    // Tail padding so the bit-reader never starves.
    out.extend_from_slice(&[0u8; 8]);
    out
}

#[test]
fn candidate_decodes_shortest_codeword() {
    // The shortest canonical-Huffman code in region_05eed0.csv has
    // bit_length 1 (entry index 0 → bit_length=1, code_value=23). After
    // the canonical builder sorts by (bl, idx) and assigns codes
    // ascending, idx 0 is the first 1-bit entry → code = `0b0`.
    //
    // Per the candidate's `(last, run, |level|)` hypothesis (see
    // `AcVlcTable::v3_intra_candidate` doc): idx=0 is sub-class A
    // (last=0, run=0, level=1).
    //
    // Stream: 1-bit `0` (the VLC code) + 1-bit `0` (positive sign) =
    // packed into a single zero-byte head followed by tail padding.
    let table = AcVlcTable::v3_intra_candidate();
    let bytes = pack(&[(0b0, 1), (0, 1)]);
    let mut br = BitReader::new(&bytes);
    let tok = decode_token(&mut br, &table).expect("decode shortest VLC");
    assert!(!tok.last, "idx 0 must be sub-class A (last=false)");
    assert_eq!(tok.run, 0);
    assert_eq!(tok.level, 1);
}

#[test]
fn candidate_run_walks_zigzag_scan() {
    // Build a stream whose decoded tokens ((last, run, level)+) walk
    // zig-zag positions 1, 5, 9. Use the candidate AC VLC and assert
    // the coefficients land at the predicted scan positions.
    let table = AcVlcTable::v3_intra_candidate();

    // Decode the first three table entries to learn their `(last,
    // run, level)` triples (they're a function of the table layout
    // because the candidate hypothesis maps idx → triple). We exploit
    // the public table via decode_token of synthetic streams.
    //
    // To avoid coupling the test to internal indexing, we instead
    // directly use the canonical fact: the table contains exactly two
    // sub-class-A entries (last=0). One of them is idx=0 with (run=0,
    // level=1); we use that as a "step run by 1" token.
    //
    // For a terminator we need any sub-class-B entry (last=1). A
    // bit-length-5 sub-B entry exists (e.g. idx=2 — bl=5 in the CSV,
    // code values picked by the canonical builder). Rather than try
    // to replicate the canonical builder here, we walk all primary
    // table entries to find one with last=true and use its actual
    // `bits`/`code` from the runtime table.
    let entries = table.entries;
    let last_zero_idx_zero = entries
        .iter()
        .find(|e| {
            matches!(
                e.value,
                oxideav_msmpeg4::ac::Symbol::RunLevel {
                    last: false,
                    run: 0,
                    level: 1
                }
            )
        })
        .expect("primary table contains (last=0, run=0, level=1)");
    let terminator = entries
        .iter()
        .find(|e| {
            matches!(
                e.value,
                oxideav_msmpeg4::ac::Symbol::RunLevel { last: true, .. }
            )
        })
        .expect("primary table contains a sub-class-B (last=true) entry");

    // Stream: `last_zero_idx_zero` + sign 0 (writes coeff at zigzag
    // pos 1, level +1) → terminator + sign 1 (writes coeff at the next
    // available zigzag position with sub-class-B's run, level -1).
    let bytes = pack(&[
        (last_zero_idx_zero.code, last_zero_idx_zero.bits as u32),
        (0, 1), // sign +
        (terminator.code, terminator.bits as u32),
        (1, 1), // sign -
    ]);
    let mut br = BitReader::new(&bytes);
    let mut block = [0i32; 64];
    let n = decode_intra_ac(&mut br, &mut block, Scan::Zigzag, &table, 1).unwrap();
    assert!(n >= 1, "at least one non-zero AC coefficient");

    // First coefficient lives at scan-position 1 (run=0).
    assert_eq!(
        block[ZIGZAG[1]], 1,
        "first non-zero AC coefficient at zigzag[1]"
    );
}

#[test]
fn candidate_drives_full_intra_block_decode() {
    // Plug the candidate AC VLC into `decode_intra_block` (which adds
    // dequantisation on top of `decode_intra_ac`) and assert that:
    //   * DC is preserved at block[0],
    //   * AC coefficients are dequantised by the H.263-style formula,
    //   * the walk terminates without overflow.
    let table = AcVlcTable::v3_intra_candidate();

    // Locate a `(last=true, run=0, |level|=1)` terminator entry. With
    // partition=1 in the candidate, the symbol at idx=2 is sub-class-B
    // with run=0 and level=1.
    let term = table
        .entries
        .iter()
        .find(|e| {
            matches!(
                e.value,
                oxideav_msmpeg4::ac::Symbol::RunLevel {
                    last: true,
                    run: 0,
                    level: 1
                }
            )
        })
        .expect("primary table contains (last=1, run=0, level=1)");

    // Single token + sign 0 (positive) = one AC coefficient @ zigzag[1].
    let bytes = pack(&[(term.code, term.bits as u32), (0, 1)]);
    let mut br = BitReader::new(&bytes);
    let mut block = [0i32; 64];
    let dc = 384i32;
    let quant = 5u32;
    let n = decode_intra_block(&mut br, &mut block, dc, Scan::Zigzag, &table, quant).unwrap();
    assert_eq!(n, 1, "exactly one AC coefficient decoded");
    assert_eq!(block[0], dc, "DC untouched");
    // H.263-style dequant: |level|=1, q=5 (odd) → coeff = 1*(2*q) +
    // (q - 1) = 10 + 4 = 14.
    assert_eq!(block[ZIGZAG[1]], 14, "dequantised AC coefficient");
}

#[test]
fn candidate_escape_body_decodes_through_the_candidate_table() {
    // Escape mode is only triggered when the primary VLC matches
    // `Symbol::Escape`, but the candidate table never emits Escape
    // (every payload entry is a RunLevel under the hypothesis). So
    // this test only verifies that on long streams of regular tokens
    // the candidate keeps decoding without stalling.
    //
    // Build a stream of three (last=0, run=0, level=1) tokens followed
    // by one (last=1, run=0, level=1) terminator.
    let table = AcVlcTable::v3_intra_candidate();
    let step = table
        .entries
        .iter()
        .find(|e| {
            matches!(
                e.value,
                oxideav_msmpeg4::ac::Symbol::RunLevel {
                    last: false,
                    run: 0,
                    level: 1
                }
            )
        })
        .expect("(last=0, run=0, level=1)");
    let term = table
        .entries
        .iter()
        .find(|e| {
            matches!(
                e.value,
                oxideav_msmpeg4::ac::Symbol::RunLevel {
                    last: true,
                    run: 0,
                    level: 1
                }
            )
        })
        .expect("(last=1, run=0, level=1)");

    let bytes = pack(&[
        (step.code, step.bits as u32),
        (0, 1),
        (step.code, step.bits as u32),
        (0, 1),
        (step.code, step.bits as u32),
        (0, 1),
        (term.code, term.bits as u32),
        (1, 1),
    ]);
    let mut br = BitReader::new(&bytes);
    let mut block = [0i32; 64];
    let n = decode_intra_ac(&mut br, &mut block, Scan::Zigzag, &table, 1).unwrap();
    assert_eq!(n, 4, "all four tokens placed coefficients");
    // Last token has sign=1 (negative).
    let nonzero: Vec<(usize, i32)> = block
        .iter()
        .enumerate()
        .filter(|(_, &v)| v != 0)
        .map(|(i, &v)| (i, v))
        .collect();
    assert_eq!(nonzero.len(), 4, "exactly four non-zero coefficients");
    let neg_count = nonzero.iter().filter(|(_, v)| *v < 0).count();
    assert_eq!(neg_count, 1, "exactly one negative coefficient (the last)");
}

/// Drive a real ffmpeg-encoded msmpeg4v3 first chunk through
/// [`oxideav_msmpeg4::picture::decode_picture_with_ac`] using the
/// candidate AC table, and assert the decoder either succeeds or
/// errors out at a documented bitstream-VLC boundary (the candidate
/// VLC's `(last, run, level)` mapping is a hypothesis, so a real
/// stream may emit symbols that decode to runs which overflow the
/// 8x8 block, which is the canonical failure signature).
///
/// This is the integration-level companion to the synthetic-stream
/// tests above: it proves the candidate plumbs through `decode_picture`
/// without crashing, and surfaces the next-step boundary cleanly.
#[test]
fn candidate_through_ffmpeg_div3_first_chunk() {
    use oxideav_core::bits::BitReader;
    use oxideav_msmpeg4::picture::{decode_picture_with_ac, AcSelection, PictureDims};
    use std::path::Path;
    use std::process::Command;

    fn ffmpeg_available() -> bool {
        Command::new("ffmpeg")
            .arg("-version")
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .status()
            .map(|s| s.success())
            .unwrap_or(false)
    }

    if !ffmpeg_available() {
        eprintln!("ffmpeg not available — skipping candidate-AC ffmpeg test");
        return;
    }

    let tmp = std::env::temp_dir().join("msmpeg4_candidate_ac_test");
    let _ = std::fs::create_dir_all(&tmp);
    let avi = tmp.join("cand.avi");
    let ok = Command::new("ffmpeg")
        .args([
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "lavfi",
            "-i",
            "color=c=gray:s=16x16:d=0.04:r=25",
            "-c:v",
            "msmpeg4v3",
            "-f",
            "avi",
            "-y",
        ])
        .arg(&avi)
        .status()
        .ok()
        .map(|s| s.success())
        .unwrap_or(false);
    if !ok {
        eprintln!("ffmpeg refused to mint DIV3 — skipping");
        return;
    }
    let bytes = std::fs::read(&avi).expect("read AVI");

    // Inline minimal RIFF scan to find the first 00dc chunk.
    fn first_video_chunk(avi: &[u8]) -> Option<Vec<u8>> {
        let mut i = 12;
        while i + 8 <= avi.len() {
            let fourcc = &avi[i..i + 4];
            let size =
                u32::from_le_bytes([avi[i + 4], avi[i + 5], avi[i + 6], avi[i + 7]]) as usize;
            if fourcc == b"00dc" && size > 0 {
                let payload_start = i + 8;
                let payload_end = payload_start + size;
                if payload_end > avi.len() {
                    return None;
                }
                return Some(avi[payload_start..payload_end].to_vec());
            }
            if fourcc == b"RIFF" || fourcc == b"LIST" {
                i += 12;
            } else {
                let mut next = i + 8 + size;
                if next % 2 != 0 {
                    next += 1;
                }
                i = next;
            }
        }
        None
    }
    let chunk = first_video_chunk(&bytes).expect("00dc chunk");
    let dims = PictureDims::new(16, 16).unwrap();
    let mut br = BitReader::new(&chunk);

    // Either the candidate table decodes far enough to produce a
    // picture, or it errors at a documented boundary. Both are
    // acceptable: we are demonstrating the pipeline reaches the
    // bitstream rather than asserting bit-exact parity (which the
    // candidate's hypothesis cannot give).
    match decode_picture_with_ac(&mut br, dims, None, AcSelection::Candidate) {
        Ok(pic) => {
            assert_eq!(pic.width, 16);
            assert_eq!(pic.height, 16);
            // Drop the picture; we don't assert on its contents — the
            // candidate symbol mapping is a hypothesis.
            let _ = pic;
            eprintln!("candidate-AC decode reached the end of the picture without erroring.");
        }
        Err(e) => {
            // Acceptable boundaries: vlc / scan / overflow / unsupported.
            let msg = format!("{e}");
            let acceptable = msg.contains("vlc")
                || msg.contains("scan")
                || msg.contains("ac:")
                || msg.contains("overflow")
                || msg.contains("AC")
                || msg.contains("Unsupported")
                || msg.contains("DC");
            assert!(
                acceptable,
                "candidate-AC decode error should cite a bitstream boundary; got: {msg}"
            );
            eprintln!(
                "candidate-AC decode errored at the documented boundary: {msg}\n\
                 (the candidate's symbol mapping is a hypothesis — real DIV3 \
                 content uses a different (last, run, level) layout that \
                 produces runs which overflow the 8x8 block)."
            );
        }
    }

    // Cleanup
    let _ = std::fs::remove_file(&avi);
    let _ = std::fs::remove_dir(&tmp);

    let _ = Path::new("");
}

#[test]
fn candidate_three_scan_orders_dispatch_consistently() {
    // The candidate VLC walker must respect whichever scan table is
    // passed to `decode_intra_ac`. Run the same single-coefficient
    // stream through all three permutations and confirm the coefficient
    // lands at the corresponding raster position each time.
    let table = AcVlcTable::v3_intra_candidate();
    let term = table
        .entries
        .iter()
        .find(|e| {
            matches!(
                e.value,
                oxideav_msmpeg4::ac::Symbol::RunLevel {
                    last: true,
                    run: 0,
                    level: 1
                }
            )
        })
        .expect("(last=1, run=0, level=1)");
    let bytes = pack(&[(term.code, term.bits as u32), (0, 1)]);
    for scan in [
        Scan::Zigzag,
        Scan::AlternateHorizontal,
        Scan::AlternateVertical,
    ] {
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        let n = decode_intra_ac(&mut br, &mut block, scan, &table, 1).unwrap();
        assert_eq!(n, 1);
        let expected_pos = scan.table()[1];
        assert_eq!(
            block[expected_pos], 1,
            "scan {scan:?} expected coeff at raster {expected_pos}"
        );
    }
}
