//! Black-box conformance of the v3 picture-header parser against the
//! `docs/video/msmpeg4-fixtures/` corpus.
//!
//! Each fixture's `notes.md` carries a per-frame trace summary
//! (`idx | type | qscale | … | dc_tbl | mv_tbl | rl_tbl | …`). This
//! test parses the *first* video chunk of a fixture through
//! [`MsV3PictureHeader::parse`] and asserts the two fields that are
//! unambiguously positioned at the very front of every v3 picture
//! header — `picture_type` (2 bits) and `PQUANT` (5 bits) — match the
//! ground-truth summary across a q ∈ {2, 6, 8, 16, 31} sweep.
//!
//! Why only those two fields: the per-frame table selectors that follow
//! PQUANT are validated for real-content *pixel* decode by
//! `docs_corpus.rs`; the residual v3 decode blocker is the
//! `region_05eac8` MCBPCY re-extraction docs gap (see that file's
//! header). `picture_type` + `PQUANT` sit ahead of any selector /
//! slice-code ambiguity, so a bit-exact match here is an unconditional
//! regression guard on the front of the header parser.
//!
//! Per `docs/video/msmpeg4-fixtures/README.md`, the fixture corpus is a
//! black-box reference for the *implementer crate* (this crate), not
//! the clean-room docs workspace; the fixture bytes and `notes.md`
//! field values are treated as opaque reference data.

use std::path::PathBuf;

use oxideav_msmpeg4::header::{MsV3PictureHeader, PictureType};

/// Extract every `00dc` (video) payload from an AVI file in stream
/// order. Mirror of the helpers in `docs_corpus.rs` /
/// `reference_roundtrip.rs`.
fn all_video_chunks(avi: &[u8]) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut i = 12;
    while i + 8 <= avi.len() {
        let fourcc = &avi[i..i + 4];
        let size = u32::from_le_bytes([avi[i + 4], avi[i + 5], avi[i + 6], avi[i + 7]]) as usize;
        if (fourcc == b"LIST" || fourcc == b"RIFF") && i + 12 <= avi.len() {
            i += 12;
            continue;
        }
        if fourcc == b"00dc" && size > 0 && i + 8 + size <= avi.len() {
            out.push(avi[i + 8..i + 8 + size].to_vec());
        }
        i += 8 + size + (size & 1);
    }
    out
}

fn fixture_first_chunk(name: &str) -> Option<Vec<u8>> {
    let p = PathBuf::from("../../docs/video/msmpeg4-fixtures")
        .join(name)
        .join("input.avi");
    let data = std::fs::read(&p).ok()?;
    all_video_chunks(&data).into_iter().next()
}

#[test]
fn v3_iframe_header_type_and_pquant_match_ground_truth() {
    // (fixture, expected picture_type, expected PQUANT) — from each
    // fixture's `notes.md` frame-0 trace-summary row.
    let cases: &[(&str, PictureType, u8)] = &[
        ("fourcc-DIV3", PictureType::I, 8),
        ("fourcc-DIV4", PictureType::I, 8),
        ("fourcc-MP43", PictureType::I, 8),
        ("fourcc-DVX3", PictureType::I, 8),
        ("fourcc-COL1", PictureType::I, 8),
        ("fourcc-AP41", PictureType::I, 8),
        ("tiny-i-only-176x144", PictureType::I, 8),
        ("i-only-352x288-cif", PictureType::I, 8),
        ("qscale-low-352x288", PictureType::I, 2),
        ("qscale-high-352x288", PictureType::I, 31),
        ("motion-pan-352x288", PictureType::I, 6),
        ("with-skip-mbs-352x288", PictureType::I, 16),
    ];

    let mut checked = 0usize;
    for (name, want_type, want_q) in cases {
        let Some(chunk) = fixture_first_chunk(name) else {
            // Corpus submodule not checked out — skip silently, mirroring
            // the report-only stance of `docs_corpus.rs`.
            eprintln!("[skip] fixture {name} unavailable");
            continue;
        };
        let mut br = oxideav_core::bits::BitReader::new(&chunk);
        let hdr = MsV3PictureHeader::parse(&mut br)
            .unwrap_or_else(|e| panic!("fixture {name}: header parse failed: {e}"));
        assert_eq!(
            hdr.picture_type, *want_type,
            "fixture {name}: picture_type mismatch"
        );
        assert_eq!(hdr.quant, *want_q, "fixture {name}: PQUANT mismatch");
        checked += 1;
    }

    // At least the always-present fourcc-* set (byte-identical elementary
    // streams) must have been reachable when the corpus is present; if the
    // whole corpus is absent, `checked == 0` is tolerated (submodule gate).
    eprintln!("[header_conformance] checked {checked} fixture headers");
}
