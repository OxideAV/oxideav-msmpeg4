//! Integration tests against **real Microsoft-encoded** MS-MPEG4 v3
//! fixtures, hosted at `samples.oxideav.org/ffmpeg/testsuite/`.
//!
//! # Why this exists
//!
//! Per `project_msmpeg4_ffmpeg_fixture_bug.md`, the in-tree
//! `docs/video/msmpeg4-fixtures/` corpus is FFmpeg-encoded — and FFmpeg's
//! MS-MPEG-4 encoder produces non-standard streams that our decoder is
//! actively (and correctly per spec/13) refusing to consume. To
//! distinguish "is our decoder correct per Microsoft spec?" from "is
//! FFmpeg's encoder non-standard?" we need fixtures that originated from
//! a known Microsoft encoder.
//!
//! Three such fixtures are pinned here:
//!
//! - `div3.avi` — 174 KB, 352×240, 50 frames, msmpeg4v3 in AVI, FourCC
//!   `DIV3`. Stream-tag `div3` in `strh`, `DIV3` in BITMAPINFOHEADER.
//! - `div4.avi` — 110 KB, 352×240, 50 frames, msmpeg4v3 in AVI, FourCC
//!   `DIV3` in BITMAPINFOHEADER (filename / strh tag is `div4` only).
//! - `mp43.wmv` — 102 KB, 400×250, 50 frames, msmpeg4v3 in ASF — the
//!   *definitive* Microsoft fixture: file carries a `WMFSDKVersion
//!   7.00.00.1956` tag from 2001, predating FFmpeg's msmpeg4 encoder by
//!   years; "Shrek Trailer / PDI Dreamworks" content.
//!
//! # Tier
//!
//! Round 452 (spec/17 + spec/18 landed) promoted the Microsoft-encoded
//! fixtures to **asserted** minimums when the fixture data + `ffmpeg`
//! reference are available (see `Expectations`): mp43.wmv — the
//! definitive Microsoft WMFSDK 7 stream — must decode 49 of its 50
//! frames (the 50th packet is truncated at the container level) with
//! an aggregate luma exact-match well above 95%; the DIV3-tagged AVI
//! fixtures must decode their I-frames end-to-end at > 97% luma match.
//! Their P-frames still drift (first divergence at an intra-in-P MB;
//! see the crate README "What's still open"), so only frame-count
//! floors are asserted there. Offline runs still skip gracefully.
//!
//! # Workflow
//!
//! 1. On first run, fetch each fixture from `samples.oxideav.org` and
//!    cache under `target/test-fixtures/`. Subsequent runs are offline.
//! 2. SHA-256 verify; refuse a stale or truncated cache.
//! 3. For ASF (`mp43.wmv`), shell out to `ffmpeg -c:v copy -an -f avi`
//!    to remux to AVI so we can reuse the existing AVI parsing — no
//!    clean-room ASF demuxer in this workspace yet. If `ffmpeg` is
//!    absent, skip the WMV fixture but still run div3/div4.
//! 4. Decode each frame through the registry-driven `MsMpeg4Decoder`
//!    and diff against `ffmpeg -f rawvideo -pix_fmt yuv420p` ground
//!    truth.
//! 5. Report per-frame match-percentage + PSNR via `eprintln!`. Never
//!    fail the test on PSNR mismatch — see the Tier note above.
//!
//! # Network gating
//!
//! Set `OXIDEAV_NETWORK_TESTS=1` to allow first-time downloads. Without
//! the flag — and without a populated cache — the test prints a skip
//! message and returns success. This mirrors the
//! `oxideav-mod::rhmst_url_regression` pattern.
//!
//! # Workspace policy
//!
//! Per the workspace external-source policy, the fixtures are data
//! (MS-encoded bitstreams) and the `ffmpeg` CLI is used only as a
//! black-box container muxer + reference decoder.

use std::fs;
use std::io::Read;
use std::path::PathBuf;
use std::process::Command;

use oxideav_core::{
    time::TimeBase, CodecId, CodecParameters, CodecRegistry, Decoder, Error, Frame, Packet,
};

// ---------------------------------------------------------------------------
// Pinned fixtures
// ---------------------------------------------------------------------------

struct Fixture {
    /// File name on the CDN and on disk under `target/test-fixtures/`.
    name: &'static str,
    /// Source URL.
    url: &'static str,
    /// Pinned SHA-256 (lowercase hex). If the upstream blob changes we
    /// fail loudly so the regression is anchored to a specific binary.
    sha256: &'static str,
    /// Pinned content-length.
    bytes: u64,
    /// Decoded frame width.
    width: u32,
    /// Decoded frame height.
    height: u32,
    /// Number of frames in the stream.
    n_frames: usize,
    /// FourCC the AVI BITMAPINFOHEADER reports (after remux for ASF).
    /// Used to drive `classify()`.
    fourcc: [u8; 4],
    /// Container kind — drives the AVI vs ASF-remux dispatch.
    container: Container,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Container {
    /// Stream is already AVI; consume bytes directly.
    Avi,
    /// Stream is ASF; remux to AVI via the host `ffmpeg` first.
    AsfRemuxedAvi,
}

const FIXTURES: &[Fixture] = &[
    Fixture {
        name: "div3.avi",
        url: "https://samples.oxideav.org/ffmpeg/testsuite/div3.avi",
        sha256: "2ca8310ccceb468bdc531c557d6b3da16b94a6ab3bbb6e794a600273c05627ea",
        bytes: 174_080,
        width: 352,
        height: 240,
        n_frames: 50,
        fourcc: *b"DIV3",
        container: Container::Avi,
    },
    Fixture {
        name: "div4.avi",
        url: "https://samples.oxideav.org/ffmpeg/testsuite/div4.avi",
        sha256: "907d2896e2c5426c26eb5345f2bb402c2c83c6df047a5eaa352f2b61ffbd2c58",
        bytes: 110_592,
        width: 352,
        height: 240,
        n_frames: 50,
        // BITMAPINFOHEADER says DIV3 even though filename / strh is `div4`.
        fourcc: *b"DIV3",
        container: Container::Avi,
    },
    Fixture {
        name: "mp43.wmv",
        url: "https://samples.oxideav.org/ffmpeg/testsuite/mp43.wmv",
        sha256: "b155e3e9ae04a2ec801451c60fb4ec2b90517b1dd7568e53cb3bd437f769b64e",
        bytes: 102_400,
        width: 400,
        height: 250,
        n_frames: 50,
        fourcc: *b"MP43",
        container: Container::AsfRemuxedAvi,
    },
];

// ---------------------------------------------------------------------------
// Cache helper (mirror of the oxideav-mod URL-fixture pattern)
// ---------------------------------------------------------------------------

fn fixture_cache_dir() -> PathBuf {
    let target_dir = std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            let crate_dir = std::env::var("CARGO_MANIFEST_DIR")
                .map(PathBuf::from)
                .expect("CARGO_MANIFEST_DIR set during cargo test");
            crate_dir.join("..").join("..").join("target")
        });
    let dir = target_dir.join("test-fixtures").join("msmpeg4-microsoft");
    fs::create_dir_all(&dir).expect("create test-fixtures dir");
    dir
}

fn cache_path(name: &str) -> PathBuf {
    fixture_cache_dir().join(name)
}

fn network_tests_enabled() -> bool {
    matches!(
        std::env::var("OXIDEAV_NETWORK_TESTS").ok().as_deref(),
        Some("1") | Some("true") | Some("TRUE") | Some("yes") | Some("YES")
    )
}

fn fetch_fixture(fix: &Fixture) -> Option<Vec<u8>> {
    let path = cache_path(fix.name);
    if let Ok(bytes) = fs::read(&path) {
        if bytes.len() as u64 == fix.bytes && sha256_hex(&bytes) == fix.sha256 {
            eprintln!("[{}] using cached {}", fix.name, path.display());
            return Some(bytes);
        }
        eprintln!(
            "[{}] cached {} is stale (len {} / sha256 {}), re-downloading",
            fix.name,
            path.display(),
            bytes.len(),
            sha256_hex(&bytes)
        );
        let _ = fs::remove_file(&path);
    }
    if !network_tests_enabled() {
        eprintln!(
            "[{}] OXIDEAV_NETWORK_TESTS not set and no cached fixture at {} — skipping",
            fix.name,
            path.display()
        );
        return None;
    }
    eprintln!("[{}] downloading {}", fix.name, fix.url);
    let resp = match ureq::get(fix.url).call() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("[{}] download failed ({e}) — skipping", fix.name);
            return None;
        }
    };
    let mut buf = Vec::with_capacity(fix.bytes as usize);
    if let Err(e) = resp.into_body().into_reader().read_to_end(&mut buf) {
        eprintln!("[{}] body read failed ({e}) — skipping", fix.name);
        return None;
    }
    if buf.len() as u64 != fix.bytes {
        eprintln!(
            "[{}] downloaded size {} != expected {} — skipping",
            fix.name,
            buf.len(),
            fix.bytes
        );
        return None;
    }
    let got = sha256_hex(&buf);
    if got != fix.sha256 {
        panic!(
            "{}: sha256 mismatch:\n  expected {}\n  got      {}\n\
             Either the upstream blob changed (update sha256 in FIXTURES) \
             or the download was corrupted.",
            fix.name, fix.sha256, got,
        );
    }
    let tmp = path.with_extension("tmp");
    if let Err(e) = fs::write(&tmp, &buf).and_then(|_| fs::rename(&tmp, &path)) {
        eprintln!(
            "[{}] cache write to {} failed ({e})",
            fix.name,
            path.display()
        );
    } else {
        eprintln!("[{}] cached at {}", fix.name, path.display());
    }
    Some(buf)
}

// ---------------------------------------------------------------------------
// AVI parsing (copied from `tests/docs_corpus.rs` / `tests/reference_roundtrip.rs`
// to keep this test file self-contained; no new dep on `oxideav-avi`).
// ---------------------------------------------------------------------------

fn all_video_chunks(avi: &[u8]) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut i = 12;
    while i + 8 <= avi.len() {
        let fourcc = &avi[i..i + 4];
        let size = u32::from_le_bytes([avi[i + 4], avi[i + 5], avi[i + 6], avi[i + 7]]) as usize;
        if fourcc == b"00dc" && size > 0 {
            let payload_start = i + 8;
            let payload_end = payload_start + size;
            if payload_end <= avi.len() {
                out.push(avi[payload_start..payload_end].to_vec());
            }
            let mut next = payload_end;
            if next % 2 != 0 {
                next += 1;
            }
            i = next;
        } else if fourcc == b"RIFF" || fourcc == b"LIST" {
            i += 12;
        } else {
            let mut next = i + 8 + size;
            if next % 2 != 0 {
                next += 1;
            }
            i = next;
        }
    }
    out
}

// ---------------------------------------------------------------------------
// ffmpeg helpers (black-box only — no source consulted)
// ---------------------------------------------------------------------------

fn ffmpeg_available() -> bool {
    Command::new("ffmpeg")
        .arg("-version")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Remux the cached ASF/WMV bytes to an AVI alongside the cache, returning
/// the new path. Returns `None` if `ffmpeg` is unavailable or the remux
/// fails — the caller should then skip the WMV fixture.
fn remux_asf_to_avi(fix: &Fixture) -> Option<PathBuf> {
    if !ffmpeg_available() {
        eprintln!(
            "[{}] ffmpeg not available — cannot remux ASF to AVI; skipping",
            fix.name
        );
        return None;
    }
    let src = cache_path(fix.name);
    let out = cache_path(&format!("{}.remux.avi", fix.name));
    if let Ok(meta) = fs::metadata(&out) {
        if meta.len() > 0 {
            // Best-effort cache: if the remux already exists and is non-
            // empty, assume it's good. (A stale remux from an old SHA
            // is very unlikely because we re-fetch the source on hash
            // mismatch above.)
            return Some(out);
        }
    }
    let ok = Command::new("ffmpeg")
        .args(["-hide_banner", "-loglevel", "error", "-y", "-i"])
        .arg(&src)
        .args(["-c:v", "copy", "-an", "-f", "avi"])
        .arg(&out)
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    if ok {
        Some(out)
    } else {
        eprintln!("[{}] ffmpeg remux failed — skipping", fix.name);
        None
    }
}

/// Decode the cached source via ffmpeg into raw YUV420p ground truth and
/// return the bytes. Returns `None` if `ffmpeg` is unavailable or fails.
///
/// The reference YUV is cached at `<name>.yuv` next to the source. We
/// regenerate only when missing or when the size doesn't match the
/// expected `n_frames * 1.5 * width * height` — that's a cheap proxy
/// for "the source changed shape" without re-hashing.
fn ffmpeg_decode_to_yuv(fix: &Fixture) -> Option<Vec<u8>> {
    if !ffmpeg_available() {
        eprintln!(
            "[{}] ffmpeg not available — cannot produce reference YUV; skipping",
            fix.name
        );
        return None;
    }
    let src = cache_path(fix.name);
    let out = cache_path(&format!("{}.yuv", fix.name));
    let expected_yuv_bytes =
        (fix.n_frames as u64) * (fix.width as u64) * (fix.height as u64) * 3 / 2;
    let need = !fs::metadata(&out)
        .map(|m| m.len() == expected_yuv_bytes)
        .unwrap_or(false);
    if need {
        let ok = Command::new("ffmpeg")
            .args(["-hide_banner", "-loglevel", "error", "-y", "-i"])
            .arg(&src)
            .args(["-f", "rawvideo", "-pix_fmt", "yuv420p"])
            .arg(&out)
            .status()
            .map(|s| s.success())
            .unwrap_or(false);
        if !ok {
            eprintln!(
                "[{}] ffmpeg reference decode failed — skipping comparison",
                fix.name
            );
            return None;
        }
    }
    fs::read(&out).ok()
}

// ---------------------------------------------------------------------------
// Per-plane diff helpers (mirror of `docs_corpus.rs`)
// ---------------------------------------------------------------------------

#[derive(Default, Clone, Copy)]
struct PlaneDiff {
    total_pixels: usize,
    exact_pixels: usize,
    sum_sq_err: u64,
    max_abs_diff: u32,
}

impl PlaneDiff {
    fn merge(&mut self, other: &PlaneDiff) {
        self.total_pixels += other.total_pixels;
        self.exact_pixels += other.exact_pixels;
        self.sum_sq_err += other.sum_sq_err;
        self.max_abs_diff = self.max_abs_diff.max(other.max_abs_diff);
    }

    fn psnr_db(&self) -> f64 {
        if self.total_pixels == 0 || self.sum_sq_err == 0 {
            return f64::INFINITY;
        }
        let mse = self.sum_sq_err as f64 / self.total_pixels as f64;
        10.0 * (255.0_f64 * 255.0_f64 / mse).log10()
    }

    fn match_pct(&self) -> f64 {
        if self.total_pixels == 0 {
            0.0
        } else {
            self.exact_pixels as f64 / self.total_pixels as f64 * 100.0
        }
    }
}

fn diff_plane_strided(
    ours: &[u8],
    ours_stride: usize,
    reference: &[u8],
    width: usize,
    height: usize,
) -> PlaneDiff {
    let mut d = PlaneDiff {
        total_pixels: width * height,
        ..PlaneDiff::default()
    };
    for j in 0..height {
        let our_row = &ours[j * ours_stride..j * ours_stride + width];
        let ref_row = &reference[j * width..j * width + width];
        for i in 0..width {
            let a = our_row[i] as i32;
            let b = ref_row[i] as i32;
            let delta = (a - b).unsigned_abs();
            if delta == 0 {
                d.exact_pixels += 1;
            }
            if delta > d.max_abs_diff {
                d.max_abs_diff = delta;
            }
            d.sum_sq_err += (delta as u64) * (delta as u64);
        }
    }
    d
}

// ---------------------------------------------------------------------------
// Per-frame outcome
// ---------------------------------------------------------------------------
//
// Round 1 of the Microsoft-fixtures harness is **report-only across the
// board** — the cleanroom `pri_A`/`pri_B` runtime binding is still OPEN
// (rounds 31-32 both refuted candidate hypotheses), so neither I- nor P-
// frames can bit-exact match. Once the binding lands, individual
// fixtures can be promoted to bit-exact assertions. For now the per-
// fixture eprintln summary is the diagnostic that the next-cleanroom-
// round Implementer reads to triangulate.

struct FrameOutcome {
    diff: Option<(PlaneDiff, PlaneDiff, PlaneDiff)>,
    err: Option<String>,
}

/// Run the registry-driven decoder against the per-frame `00dc` chunks
/// and diff each visible frame against the `width × height × 1.5`
/// stride-packed YUV420p reference.
fn decode_and_compare(fix: &Fixture, avi_bytes: &[u8], yuv_ref: &[u8]) -> Vec<FrameOutcome> {
    let chunks = all_video_chunks(avi_bytes);
    if chunks.is_empty() {
        return vec![FrameOutcome {
            diff: None,
            err: Some("AVI has no 00dc chunks".to_string()),
        }];
    }

    let cls = oxideav_msmpeg4::classify(&chunks[0], Some(&fix.fourcc));
    let cid_str = match cls.codec_id() {
        Some(id) => id,
        None => {
            return vec![FrameOutcome {
                diff: None,
                err: Some("classify returned Unknown".to_string()),
            }];
        }
    };

    let mut reg = CodecRegistry::new();
    oxideav_msmpeg4::register_codecs(&mut reg);

    let mut params = CodecParameters::video(CodecId::new(cid_str));
    params.width = Some(fix.width);
    params.height = Some(fix.height);
    let mut dec: Box<dyn Decoder> = match reg.first_decoder(&params) {
        Ok(d) => d,
        Err(e) => {
            return vec![FrameOutcome {
                diff: None,
                err: Some(format!("make_decoder: {e:?}")),
            }];
        }
    };

    let w = fix.width as usize;
    let h = fix.height as usize;
    let cw = w / 2;
    let ch = h / 2;
    let y_size = w * h;
    let uv_size = cw * ch;
    let frame_size = y_size + 2 * uv_size;

    let expected_total = fix.n_frames * frame_size;
    if yuv_ref.len() != expected_total {
        eprintln!(
            "[{}] WARN reference YUV size {} != {} (frames {} * frame_size {})",
            fix.name,
            yuv_ref.len(),
            expected_total,
            fix.n_frames,
            frame_size,
        );
    }

    let mut outcomes: Vec<FrameOutcome> = Vec::with_capacity(fix.n_frames);
    let mut visible_idx = 0usize;

    for (pkt_idx, frame_bytes) in chunks.iter().enumerate() {
        if frame_bytes.is_empty() {
            // ffmpeg-emitted AVIs sometimes contain zero-length 00dc
            // chunks for "frame is identical to previous"; for our
            // ground-truth comparison treat as a skip.
            continue;
        }
        let mut pkt = Packet::new(0, TimeBase::new(1, 1000), frame_bytes.clone());
        pkt.pts = Some(pkt_idx as i64);
        let send_res = dec.send_packet(&pkt);
        if let Err(e) = send_res {
            outcomes.push(FrameOutcome {
                diff: None,
                err: Some(format!("packet {pkt_idx}: send_packet: {e}")),
            });
            continue;
        }
        loop {
            match dec.receive_frame() {
                Ok(Frame::Video(vf)) => {
                    if visible_idx >= fix.n_frames || frame_size * (visible_idx + 1) > yuv_ref.len()
                    {
                        outcomes.push(FrameOutcome {
                            diff: None,
                            err: Some(format!("visible {visible_idx}: out of reference range")),
                        });
                        visible_idx += 1;
                        continue;
                    }
                    let off = visible_idx * frame_size;
                    let ref_y = &yuv_ref[off..off + y_size];
                    let ref_u = &yuv_ref[off + y_size..off + y_size + uv_size];
                    let ref_v = &yuv_ref[off + y_size + uv_size..off + frame_size];
                    if vf.planes.len() < 3 {
                        outcomes.push(FrameOutcome {
                            diff: None,
                            err: Some(format!(
                                "visible {visible_idx}: decoder produced {} planes",
                                vf.planes.len()
                            )),
                        });
                        visible_idx += 1;
                        continue;
                    }
                    let dy =
                        diff_plane_strided(&vf.planes[0].data, vf.planes[0].stride, ref_y, w, h);
                    let du =
                        diff_plane_strided(&vf.planes[1].data, vf.planes[1].stride, ref_u, cw, ch);
                    let dv =
                        diff_plane_strided(&vf.planes[2].data, vf.planes[2].stride, ref_v, cw, ch);
                    outcomes.push(FrameOutcome {
                        diff: Some((dy, du, dv)),
                        err: None,
                    });
                    visible_idx += 1;
                }
                Ok(_) => continue,
                Err(Error::NeedMore) => break,
                Err(e) => {
                    outcomes.push(FrameOutcome {
                        diff: None,
                        err: Some(format!("visible {visible_idx}: receive_frame: {e}")),
                    });
                    break;
                }
            }
        }
    }
    outcomes
}

/// Asserted per-fixture minimums (only checked when the fixture and
/// the `ffmpeg` reference decode are actually available).
struct Expectations {
    /// Minimum number of frames that must decode without error.
    min_decoded: usize,
    /// Minimum aggregate luma exact-match percentage.
    min_y_match_pct: f64,
    /// Minimum luma exact-match percentage on frame 0 (the I-frame).
    min_frame0_y_pct: f64,
}

const EXPECTATIONS: &[Expectations] = &[
    // div3.avi: I-frames fully parse; P-frames drift (round 452).
    Expectations {
        min_decoded: 45,
        min_y_match_pct: 5.0,
        min_frame0_y_pct: 97.0,
    },
    // div4.avi: same tier as div3.
    Expectations {
        min_decoded: 45,
        min_y_match_pct: 5.0,
        min_frame0_y_pct: 97.0,
    },
    // mp43.wmv: full-clip decode; frame 49's packet is truncated in
    // the container, so 49 of 50.
    Expectations {
        min_decoded: 49,
        min_y_match_pct: 95.0,
        min_frame0_y_pct: 99.0,
    },
];

fn evaluate_outcomes(fix: &Fixture, expect: &Expectations, outcomes: &[FrameOutcome]) {
    let mut agg_y = PlaneDiff::default();
    let mut agg_u = PlaneDiff::default();
    let mut agg_v = PlaneDiff::default();
    let mut errors: Vec<String> = Vec::new();
    let mut decoded_frames = 0usize;
    let mut first_diverge: Option<usize> = None;

    for (i, o) in outcomes.iter().enumerate() {
        if let Some(err) = &o.err {
            eprintln!("  [{}] frame {i}: ERROR {err}", fix.name);
            errors.push(format!("frame {i}: {err}"));
            if first_diverge.is_none() {
                first_diverge = Some(i);
            }
        }
        if let Some((dy, du, dv)) = &o.diff {
            decoded_frames += 1;
            // Only print first 5 frames + every 10th to keep stderr terse.
            if i < 5 || i % 10 == 0 {
                eprintln!(
                    "  [{}] frame {i}: Y match {:.2}% / PSNR {:.2} dB (max diff {}); \
                     U {:.2}%/{:.2} dB; V {:.2}%/{:.2} dB",
                    fix.name,
                    dy.match_pct(),
                    dy.psnr_db(),
                    dy.max_abs_diff,
                    du.match_pct(),
                    du.psnr_db(),
                    dv.match_pct(),
                    dv.psnr_db(),
                );
            }
            if first_diverge.is_none() && dy.match_pct() < 99.0 {
                first_diverge = Some(i);
            }
            agg_y.merge(dy);
            agg_u.merge(du);
            agg_v.merge(dv);
        }
    }

    eprintln!(
        "[ReportOnly] {}: decoded {} of {} frames; aggregate Y PSNR {:.2} dB ({:.2}% match), \
         U PSNR {:.2} dB ({:.2}% match), V PSNR {:.2} dB ({:.2}% match), {} errors, \
         first_diverge={:?}",
        fix.name,
        decoded_frames,
        fix.n_frames,
        agg_y.psnr_db(),
        agg_y.match_pct(),
        agg_u.psnr_db(),
        agg_u.match_pct(),
        agg_v.psnr_db(),
        agg_v.match_pct(),
        errors.len(),
        first_diverge,
    );

    // Round 452 assertions: the Microsoft-arbitrated decoder must not
    // regress below the levels the spec/17 + spec/18 rebuild reached.
    assert!(
        decoded_frames >= expect.min_decoded,
        "{}: decoded {} frames, expected at least {}",
        fix.name,
        decoded_frames,
        expect.min_decoded,
    );
    assert!(
        agg_y.match_pct() >= expect.min_y_match_pct,
        "{}: aggregate Y match {:.2}% below floor {:.2}%",
        fix.name,
        agg_y.match_pct(),
        expect.min_y_match_pct,
    );
    if let Some(FrameOutcome {
        diff: Some((dy, _, _)),
        ..
    }) = outcomes.first()
    {
        assert!(
            dy.match_pct() >= expect.min_frame0_y_pct,
            "{}: I-frame 0 Y match {:.2}% below floor {:.2}%",
            fix.name,
            dy.match_pct(),
            expect.min_frame0_y_pct,
        );
    } else {
        panic!("{}: frame 0 (the I-frame) failed to decode", fix.name);
    }
}

/// Run one fixture end-to-end, fetching + remuxing as needed.
fn run_fixture(fix: &Fixture, expect: &Expectations) {
    eprintln!(
        "=== {} ({}x{}, {} frames) ===",
        fix.name, fix.width, fix.height, fix.n_frames
    );

    // 1. Fetch (or load from cache).
    let raw = match fetch_fixture(fix) {
        Some(b) => b,
        None => return,
    };

    // 2. Get an AVI handle on the bitstream.
    let avi_bytes: Vec<u8> = match fix.container {
        Container::Avi => raw,
        Container::AsfRemuxedAvi => {
            // Drop the original raw bytes — we need the remuxed AVI
            // for per-chunk frame access. ASF demux isn't available in
            // this workspace yet; the pragmatic black-box ffmpeg path
            // is the path of least resistance.
            drop(raw);
            let avi_path = match remux_asf_to_avi(fix) {
                Some(p) => p,
                None => return,
            };
            match fs::read(&avi_path) {
                Ok(b) => b,
                Err(e) => {
                    eprintln!("[{}] read remuxed AVI failed: {e} — skipping", fix.name);
                    return;
                }
            }
        }
    };

    // 3. Reference YUV via ffmpeg.
    let yuv_ref = match ffmpeg_decode_to_yuv(fix) {
        Some(b) => b,
        None => return,
    };

    // 4. Decode + diff.
    let outcomes = decode_and_compare(fix, &avi_bytes, &yuv_ref);
    evaluate_outcomes(fix, expect, &outcomes);
}

// ---------------------------------------------------------------------------
// Per-fixture tests (one #[test] each so the harness can parallelise +
// individual failures show up named)
// ---------------------------------------------------------------------------

#[test]
fn microsoft_fixture_div3_avi() {
    run_fixture(&FIXTURES[0], &EXPECTATIONS[0]);
}

#[test]
fn microsoft_fixture_div4_avi() {
    run_fixture(&FIXTURES[1], &EXPECTATIONS[1]);
}

#[test]
fn microsoft_fixture_mp43_wmv() {
    run_fixture(&FIXTURES[2], &EXPECTATIONS[2]);
}

// ---------------------------------------------------------------------------
// Inline SHA-256 (FIPS 180-4 byte-oriented reference; mirror of the
// implementation in `oxideav-mod/tests/rhmst_url_regression.rs`). Avoids a
// `sha2` dev-dependency.
// ---------------------------------------------------------------------------

fn sha256_hex(bytes: &[u8]) -> String {
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];
    let mut h: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    let bit_len = (bytes.len() as u64) * 8;
    let mut msg = Vec::with_capacity(bytes.len() + 72);
    msg.extend_from_slice(bytes);
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&bit_len.to_be_bytes());
    for chunk in msg.chunks(64) {
        let mut w = [0u32; 64];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([
                chunk[i * 4],
                chunk[i * 4 + 1],
                chunk[i * 4 + 2],
                chunk[i * 4 + 3],
            ]);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }
        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut hh] = h;
        for i in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ (!e & g);
            let t1 = hh
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let mj = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(mj);
            hh = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }
        h[0] = h[0].wrapping_add(a);
        h[1] = h[1].wrapping_add(b);
        h[2] = h[2].wrapping_add(c);
        h[3] = h[3].wrapping_add(d);
        h[4] = h[4].wrapping_add(e);
        h[5] = h[5].wrapping_add(f);
        h[6] = h[6].wrapping_add(g);
        h[7] = h[7].wrapping_add(hh);
    }
    let mut out = String::with_capacity(64);
    for word in h {
        for byte in word.to_be_bytes() {
            out.push_str(&format!("{:02x}", byte));
        }
    }
    out
}
