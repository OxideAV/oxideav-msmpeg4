//! Integration tests against the docs/video/msmpeg4-fixtures/ corpus.
//!
//! Each fixture under `../../docs/video/msmpeg4-fixtures/<name>/` carries
//! an `input.avi` (FFmpeg-encoded MSMPEG4 v3 stream) and an
//! `expected.yuv` (FFmpeg-decoded ground truth). This driver decodes
//! every fixture through the in-tree [`MsMpeg4Decoder`] (via the
//! registry) and reports per-fixture pixel-match + per-plane PSNR vs
//! the reference YUV.
//!
//! # Tier classification
//!
//! * [`Tier::BitExact`] — must round-trip exactly. Failure = CI red.
//! * [`Tier::ReportOnly`] — currently divergent; logged but not asserted.
//! * [`Tier::Ignored`] — disabled (e.g. selector for a still-OPEN
//!   table-binding question).
//!
//! Round 31 wires every fixture as `ReportOnly` because the runtime
//! `desc+0x1c / +0x20` (live `pri_A` / `pri_B` pointer) assignment is
//! still OPEN per `audit/04` §2.5 / `spec/13` §8 item 1 — without it
//! the v3 decoder cannot reliably select between {G0, G1, G2, G3, G4,
//! G5} for a given frame's chroma + intra-luma slot, so most fixtures
//! either error out or produce DC-only approximations far from the
//! reference YUV. The corpus is the input the next-cleanroom-round
//! Implementer will use to verify the fix once the binding question
//! lands.
//!
//! # Known caveat
//!
//! Per `project_msmpeg4_ffmpeg_fixture_bug.md` the FFmpeg encoder used
//! to produce `input.avi` may emit non-standard streams (testsrc-176×144
//! DIV3 fixtures specifically overflow scan_pos in our spec/13-correct
//! kernel). When we don't match the corpus, *it may be ours that's
//! correct per Microsoft spec*. Each per-fixture report distinguishes
//! "decoder error" from "decoded but doesn't match" so the human can
//! triage which side has the bug.
//!
//! # Workspace policy
//!
//! Per the docs/video/msmpeg4-fixtures/README.md "clean-room boundary"
//! warning, the *cleanroom docs workspace* (`docs/video/msmpeg4/`) must
//! NOT consume this corpus. We are the **implementer crate**, NOT the
//! cleanroom workspace, so consumption here is explicitly blessed. We
//! still don't read libavcodec source — the corpus is treated as a
//! black-box reference.

use std::fs;
use std::path::PathBuf;

use oxideav_core::{
    time::TimeBase, CodecId, CodecParameters, CodecRegistry, CodecResolver, CodecTag, Decoder,
    Frame, Packet, ProbeContext,
};

/// Locate `docs/video/msmpeg4-fixtures/<name>/`. Tests run with CWD set
/// to the crate root, so we walk two levels up to reach the workspace
/// root and then into `docs/`.
fn fixture_dir(name: &str) -> PathBuf {
    PathBuf::from("../../docs/video/msmpeg4-fixtures").join(name)
}

/// Extract every `00dc` (video) payload from an AVI file in stream
/// order. Mirror of the helper in `tests/ffmpeg_roundtrip.rs`.
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

/// Extract the FourCC from an AVI's `strh` chunk (the BITMAPINFOHEADER's
/// `biCompression` field at offset 0xC0 in the standard FFmpeg-emitted
/// AVI). Falls back to `*b"DIV3"` if not found — the README confirms
/// that all six fourcc-* fixtures share the same elementary bitstream.
fn fourcc_from_avi(avi: &[u8]) -> [u8; 4] {
    // FFmpeg's standard AVI layout puts the BITMAPINFOHEADER fourcc at
    // offset 0xC0 (= 192). A robust parser would walk the strf chunk;
    // for our deterministic FFmpeg-generated corpus a fixed offset
    // suffices.
    if avi.len() >= 0xC4 {
        let fc = [avi[0xC0], avi[0xC1], avi[0xC2], avi[0xC3]];
        // Sanity: must look like ASCII letters / digits.
        if fc.iter().all(|b| b.is_ascii_graphic()) {
            return fc;
        }
    }
    *b"DIV3"
}

/// Per-frame plane-by-plane diff result.
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
        // PSNR for 8-bit unsigned: 10*log10(255^2 / MSE).
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

fn diff_plane(ours: &[u8], reference: &[u8]) -> PlaneDiff {
    let mut d = PlaneDiff::default();
    let n = ours.len().min(reference.len());
    d.total_pixels = n;
    for i in 0..n {
        let a = ours[i] as i32;
        let b = reference[i] as i32;
        let delta = (a - b).unsigned_abs();
        if delta == 0 {
            d.exact_pixels += 1;
        }
        if delta > d.max_abs_diff {
            d.max_abs_diff = delta;
        }
        d.sum_sq_err += (delta as u64) * (delta as u64);
    }
    d
}

#[derive(Clone, Copy, Debug)]
enum Tier {
    /// Must decode bit-exactly. Test fails on any divergence.
    #[allow(dead_code)]
    BitExact,
    /// Decode is permitted to diverge; we log the deltas.
    ReportOnly,
}

struct CorpusCase {
    name: &'static str,
    width: usize,
    height: usize,
    n_frames: usize,
    tier: Tier,
}

/// Per-frame outcome:
struct FrameOutcome {
    /// Plane Y / U / V deltas.  None if decode errored before producing
    /// a frame (the err string is recorded separately).
    diff: Option<(PlaneDiff, PlaneDiff, PlaneDiff)>,
    err: Option<String>,
}

/// Decode one fixture through the registry-driven decoder; return per-
/// frame outcomes.
fn decode_fixture(case: &CorpusCase) -> Option<Vec<FrameOutcome>> {
    let dir = fixture_dir(case.name);
    let avi_path = dir.join("input.avi");
    let yuv_path = dir.join("expected.yuv");
    let avi = match fs::read(&avi_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("skip {}: missing {} ({e})", case.name, avi_path.display());
            return None;
        }
    };
    let yuv_ref = match fs::read(&yuv_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("skip {}: missing {} ({e})", case.name, yuv_path.display());
            return None;
        }
    };
    let chunks = all_video_chunks(&avi);
    if chunks.is_empty() {
        eprintln!("skip {}: AVI has no 00dc chunks", case.name);
        return None;
    }

    let fourcc = fourcc_from_avi(&avi);
    let cls = oxideav_msmpeg4::classify(&chunks[0], Some(&fourcc));
    let cid_str = match cls.codec_id() {
        Some(id) => id,
        None => {
            eprintln!("skip {}: classify returned Unknown", case.name);
            return None;
        }
    };

    // Build a registry, register our decoders, resolve via tag, and
    // construct an actual decoder instance.
    let mut reg = CodecRegistry::new();
    oxideav_msmpeg4::register_codecs(&mut reg);

    // Sanity-check that the registry resolves this fourcc to the same
    // codec id classify() picked (catches a registry/router mismatch).
    let tag = CodecTag::fourcc(&fourcc);
    let ctx = ProbeContext::new(&tag).packet(&chunks[0]);
    let resolved = CodecResolver::resolve_tag(&reg, &ctx);
    let resolved_id = resolved.as_ref().map(|c| c.as_str().to_string());
    if resolved_id.as_deref() != Some(cid_str) {
        eprintln!(
            "warn {}: registry resolved {:?} as {:?} but classify picked {cid_str}",
            case.name, fourcc, resolved
        );
    }

    let mut params = CodecParameters::video(CodecId::new(cid_str));
    params.width = Some(case.width as u32);
    params.height = Some(case.height as u32);
    let mut dec: Box<dyn Decoder> = match reg.make_decoder(&params) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("skip {}: make_decoder failed: {e:?}", case.name);
            return None;
        }
    };

    let y_size = case.width * case.height;
    let cw = case.width / 2;
    let ch = case.height / 2;
    let uv_size = cw * ch;
    let frame_size = y_size + 2 * uv_size;

    let expected_total = case.n_frames * frame_size;
    if yuv_ref.len() != expected_total {
        eprintln!(
            "warn {}: expected.yuv size {} != {} (frames {} * frame_size {})",
            case.name,
            yuv_ref.len(),
            expected_total,
            case.n_frames,
            frame_size
        );
    }

    let mut outcomes: Vec<FrameOutcome> = Vec::with_capacity(case.n_frames);
    let mut visible_idx = 0usize;

    for (pkt_idx, frame_bytes) in chunks.iter().enumerate() {
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
                    if visible_idx >= case.n_frames
                        || frame_size * (visible_idx + 1) > yuv_ref.len()
                    {
                        // More visible frames than reference — record but skip diff.
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
                    let dy = diff_plane(&vf.planes[0].data, ref_y);
                    let du = diff_plane(&vf.planes[1].data, ref_u);
                    let dv = diff_plane(&vf.planes[2].data, ref_v);
                    outcomes.push(FrameOutcome {
                        diff: Some((dy, du, dv)),
                        err: None,
                    });
                    visible_idx += 1;
                }
                Ok(_) => continue,
                Err(oxideav_core::Error::NeedMore) => break,
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

    Some(outcomes)
}

fn evaluate(case: &CorpusCase) {
    let outcomes = match decode_fixture(case) {
        Some(o) => o,
        None => return,
    };

    let mut agg_y = PlaneDiff::default();
    let mut agg_u = PlaneDiff::default();
    let mut agg_v = PlaneDiff::default();
    let mut errors: Vec<String> = Vec::new();
    let mut decoded_frames = 0usize;

    for (i, o) in outcomes.iter().enumerate() {
        if let Some(err) = &o.err {
            eprintln!("  frame {i}: ERROR {err}");
            errors.push(format!("frame {i}: {err}"));
        }
        if let Some((dy, du, dv)) = &o.diff {
            decoded_frames += 1;
            let psnr_y = dy.psnr_db();
            let psnr_u = du.psnr_db();
            let psnr_v = dv.psnr_db();
            eprintln!(
                "  frame {i}: Y match {:.2}% / PSNR {:.2} dB (max diff {}); \
                 U match {:.2}% / PSNR {:.2} dB; V match {:.2}% / PSNR {:.2} dB",
                dy.match_pct(),
                psnr_y,
                dy.max_abs_diff,
                du.match_pct(),
                psnr_u,
                dv.match_pct(),
                psnr_v,
            );
            agg_y.merge(dy);
            agg_u.merge(du);
            agg_v.merge(dv);
        }
    }

    eprintln!(
        "[{:?}] {}: decoded {} of {} frames; aggregate Y PSNR {:.2} dB ({:.2}% match), \
         U PSNR {:.2} dB ({:.2}% match), V PSNR {:.2} dB ({:.2}% match), {} errors",
        case.tier,
        case.name,
        decoded_frames,
        case.n_frames,
        agg_y.psnr_db(),
        agg_y.match_pct(),
        agg_u.psnr_db(),
        agg_u.match_pct(),
        agg_v.psnr_db(),
        agg_v.match_pct(),
        errors.len(),
    );

    match case.tier {
        Tier::BitExact => {
            assert!(
                errors.is_empty(),
                "{}: {} frame errors prevented bit-exact comparison: {:?}",
                case.name,
                errors.len(),
                errors
            );
            let total = agg_y.total_pixels + agg_u.total_pixels + agg_v.total_pixels;
            let exact = agg_y.exact_pixels + agg_u.exact_pixels + agg_v.exact_pixels;
            assert_eq!(
                exact, total,
                "{}: not bit-exact (Y max diff {}, U max diff {}, V max diff {})",
                case.name, agg_y.max_abs_diff, agg_u.max_abs_diff, agg_v.max_abs_diff
            );
        }
        Tier::ReportOnly => {
            // Don't fail. The eprintln! is the human-readable diagnostic.
        }
    }
}

// ---------------------------------------------------------------------------
// Per-fixture tests
// ---------------------------------------------------------------------------
//
// Round 31 ships these as ReportOnly. The expected per-fixture outcome
// is "decoder errors out at the first AC block" because the static
// G4/G5 selection wired today doesn't agree with the runtime selector
// path on most fixtures. spec/13 §6 lists the fix as future-cleanroom
// work; until then this matrix tracks the current decoder's behaviour
// as a baseline for the next round.
//
// The known FFmpeg-encoder caveat applies: per
// project_msmpeg4_ffmpeg_fixture_bug.md the testsrc 176×144 DIV3
// fixtures may also be non-standard, so even after fixing the
// selector binding, the smallest fixtures may need to be re-encoded
// with a real Microsoft encoder to be meaningful BitExact targets.

#[test]
fn corpus_tiny_i_only_176x144() {
    evaluate(&CorpusCase {
        name: "tiny-i-only-176x144",
        width: 176,
        height: 144,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_i_frame_then_p_frame_176x144() {
    evaluate(&CorpusCase {
        name: "i-frame-then-p-frame-176x144",
        width: 176,
        height: 144,
        n_frames: 2,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_i_only_352x288_cif() {
    evaluate(&CorpusCase {
        name: "i-only-352x288-cif",
        width: 352,
        height: 288,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_qscale_low_352x288() {
    evaluate(&CorpusCase {
        name: "qscale-low-352x288",
        width: 352,
        height: 288,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_qscale_high_352x288() {
    evaluate(&CorpusCase {
        name: "qscale-high-352x288",
        width: 352,
        height: 288,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_intra_pred_active_352x288() {
    evaluate(&CorpusCase {
        name: "intra-pred-active-352x288",
        width: 352,
        height: 288,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_gop_30_352x288() {
    evaluate(&CorpusCase {
        name: "gop-30-352x288",
        width: 352,
        height: 288,
        n_frames: 6,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_motion_pan_352x288() {
    evaluate(&CorpusCase {
        name: "motion-pan-352x288",
        width: 352,
        height: 288,
        n_frames: 4,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_with_skip_mbs_352x288() {
    evaluate(&CorpusCase {
        name: "with-skip-mbs-352x288",
        width: 352,
        height: 288,
        n_frames: 5,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_fourcc_div3() {
    evaluate(&CorpusCase {
        name: "fourcc-DIV3",
        width: 176,
        height: 144,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_fourcc_div4() {
    evaluate(&CorpusCase {
        name: "fourcc-DIV4",
        width: 176,
        height: 144,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_fourcc_dvx3() {
    evaluate(&CorpusCase {
        name: "fourcc-DVX3",
        width: 176,
        height: 144,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_fourcc_ap41() {
    evaluate(&CorpusCase {
        name: "fourcc-AP41",
        width: 176,
        height: 144,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_fourcc_mp43() {
    evaluate(&CorpusCase {
        name: "fourcc-MP43",
        width: 176,
        height: 144,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_fourcc_col1() {
    evaluate(&CorpusCase {
        name: "fourcc-COL1",
        width: 176,
        height: 144,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}
