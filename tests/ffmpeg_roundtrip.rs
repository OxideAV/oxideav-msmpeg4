//! Integration test that uses `ffmpeg` as a *black box* to mint a tiny
//! DIV3 AVI, extracts the first frame's encoded payload from the AVI
//! container (pure-Rust parsing of just enough of the RIFF/AVI
//! structure to locate the first `00dc` chunk), and drives the
//! picture-header + first macroblock path of our decoder.
//!
//! If `ffmpeg` is not present on the host, the test is skipped (passes
//! with a note in stderr). We never read ffmpeg's source.
//!
//! Why this test: it's the earliest end-to-end exercise of our parser
//! against bytes produced by a known-good encoder. Until the MS-MPEG4
//! AC VLC tables are extracted, the decoder will return
//! `Error::Unsupported` at the AC stage — that's the expected outcome
//! here, and the test asserts the progression (picture header OK, first
//! MB header OK, AC stage bails out with a recognisable error).

use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::Command;

use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::ac::{AcVlcTable, Scan};
use oxideav_msmpeg4::header::{MsV3PictureHeader, PictureType};
use oxideav_msmpeg4::mb::decode_intra_mb;

fn ffmpeg_available() -> bool {
    Command::new("ffmpeg")
        .arg("-version")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Write a 16x16 one-frame DIV3 AVI via ffmpeg. Returns the AVI path
/// or `None` if ffmpeg refused to run.
fn mint_div3_avi(dir: &Path) -> Option<PathBuf> {
    let out = dir.join("tiny.avi");
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
        .arg(&out)
        .status()
        .ok()
        .map(|s| s.success())
        .unwrap_or(false);
    if ok {
        Some(out)
    } else {
        None
    }
}

/// Minimalistic AVI/RIFF scan: look for `00dc` chunks (video payload)
/// inside `movi` list and return the first chunk's bytes.
fn first_video_chunk(avi: &[u8]) -> Option<Vec<u8>> {
    // Linear scan for `00dc` FOURCC — cheap and robust for a tiny file.
    let mut i = 12; // skip RIFF header
    while i + 8 <= avi.len() {
        let fourcc = &avi[i..i + 4];
        let size = u32::from_le_bytes([avi[i + 4], avi[i + 5], avi[i + 6], avi[i + 7]]) as usize;
        if fourcc == b"00dc" && size > 0 {
            let payload_start = i + 8;
            let payload_end = payload_start + size;
            if payload_end > avi.len() {
                return None;
            }
            return Some(avi[payload_start..payload_end].to_vec());
        }
        // RIFF chunks are 2-byte aligned; LIST/RIFF chunks contain
        // sub-chunks so we need to recurse into those. The cheap way
        // out: if this is a container, skip past its 4-byte list-type
        // and keep scanning; otherwise jump past the chunk.
        if fourcc == b"RIFF" || fourcc == b"LIST" {
            // 4 bytes of list type, then inner chunks. Move past the
            // fourcc + size + list-type = 12 bytes and resume scanning.
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

#[test]
fn ffmpeg_generated_div3_header_and_first_mb_parse() {
    if !ffmpeg_available() {
        eprintln!("ffmpeg not available — skipping integration test");
        return;
    }

    let tmp = tempdir();
    let Some(avi_path) = mint_div3_avi(&tmp) else {
        eprintln!("ffmpeg failed to produce DIV3 AVI — skipping");
        return;
    };
    let bytes = std::fs::read(&avi_path).expect("read AVI");
    let chunk = first_video_chunk(&bytes).expect("first 00dc chunk not found");
    assert!(
        !chunk.is_empty(),
        "first video chunk empty — AVI parse went wrong"
    );

    // Classification: the payload has no 0x000001 start code.
    let cls = oxideav_msmpeg4::classify(&chunk, Some(b"DIV3"));
    assert_eq!(
        cls,
        oxideav_msmpeg4::Classification::MsMpeg4V3,
        "chunk bytes {:02x?}",
        &chunk[..chunk.len().min(16)],
    );

    // Picture header: must parse, and the first frame is an I-frame.
    let mut br = BitReader::new(&chunk);
    let hdr = MsV3PictureHeader::parse(&mut br).expect("picture header parse");
    assert_eq!(
        hdr.picture_type,
        PictureType::I,
        "first frame must be an I-frame"
    );
    assert!(
        (1..=31).contains(&hdr.quant),
        "pquant {} out of range",
        hdr.quant
    );

    // Attempt to decode the first intra MB through `decode_intra_mb` using
    // the placeholder AC VLC table. Expected outcome: an `Unsupported`
    // error whose message names the two candidate source VMAs
    // (`0x1c25fad0` and `0x1c25f6c8` per
    // `docs/video/msmpeg4/spec/03-corrections.md` §5.3) — this is the
    // hand-off signal for the Extractor.
    //
    // This asserts the pipeline is wired end-to-end: classification →
    // picture header → MB header parse → the AC table is requested at
    // the right point. Once the Extractor lands real
    // `VlcEntry<Symbol>` data, flip the assertion to expect Ok(_).
    let pred = [1024i32; 6];
    let mb_result = decode_intra_mb(
        &mut br,
        hdr.quant as u32,
        true, // cbp_cb — worst-case (triggers AC path if table were real)
        true, // cbp_cr
        pred,
        Scan::Zigzag,
        &AcVlcTable::V3_INTRA_PLACEHOLDER,
    );
    let err = mb_result.expect_err(
        "first MB decode must fail (placeholder AC VLC) until Extractor lands real data",
    );
    let msg = format!("{err}");
    assert!(
        msg.contains("0x1c25fad0"),
        "error must cite candidate VMA 0x1c25fad0; got: {msg}",
    );
    assert!(
        msg.contains("0x1c25f6c8"),
        "error must cite candidate VMA 0x1c25f6c8; got: {msg}",
    );
    assert!(
        msg.contains("§5.3"),
        "error must cite spec section; got: {msg}",
    );
}

fn tempdir() -> PathBuf {
    let mut p = std::env::temp_dir();
    // Millisecond-resolution subdir so parallel test runs don't clobber.
    let nonce = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    p.push(format!("oxideav-msmpeg4-ffmpeg-{}", nonce));
    std::fs::create_dir_all(&p).expect("create tempdir");
    // Write a sentinel so we can tell which process minted this.
    let mut s = std::fs::File::create(p.join(".nonce")).unwrap();
    writeln!(s, "{}", nonce).unwrap();
    p
}
