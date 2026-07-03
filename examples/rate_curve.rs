//! Encoder quality measurement harness: PSNR / rate curves over a
//! deterministic synthetic sequence, for all three bitstream versions.
//!
//! Every packet the encoder emits is decoded back through the crate's
//! own production decode path (`picture::decode_picture` /
//! `decode_picture_v1v2` — the same chain the registered decoder
//! drives), so the printed PSNR is the *decoder-side* reconstruction
//! quality, not an encoder-internal estimate.
//!
//! Run with:
//!
//! ```text
//! cargo run --release --example rate_curve
//! ```

use oxideav_core::bits::BitReader;
use oxideav_core::{CodecId, CodecParameters, Frame, VideoFrame, VideoPlane};
use oxideav_msmpeg4::picture::PictureDims;
use oxideav_msmpeg4::picture::{decode_picture, decode_picture_v1v2, MsV1V2Version, Picture};

const W: usize = 176;
const H: usize = 144;
const FRAMES: usize = 30;
/// Frame index at which the synthetic sequence hard-cuts to a new
/// scene (texture family change, not just translation).
const SCENE_CUT: usize = 8;
const FPS: f64 = 25.0;

/// Deterministic smooth texture: sum of a diagonal gradient and two
/// coarse sinusoid-free "plasma" terms (integer arithmetic only, so
/// the sequence is bit-identical across platforms). `phase` selects
/// the texture family (scene identity), `(tx, ty)` is the sub-pel
/// translation in half-pel units.
fn luma_at(x: i64, y: i64, phase: i64) -> u8 {
    let a = (x * 3 + y * 2 + phase * 37) % 256;
    let b = ((x / 7 + phase) * (y / 5 + 2 * phase)) % 64;
    let c = ((x * x) / 91 + (y * y) / 53) % 48;
    (((a + b * 2 + c) % 200) + 24) as u8
}

fn make_frame(n: usize) -> VideoFrame {
    let (phase, base) = if n < SCENE_CUT {
        (1, n)
    } else {
        (9, n - SCENE_CUT)
    };
    // Translation of 1.5 pels right / 0.75 pel down per frame,
    // expressed in half-pel units then sampled at integer positions
    // (nearest): exercises the half-pel motion search without needing
    // a float resampler.
    let tx2 = (base as i64) * 3; // half-pel units
    let ty2 = (base as i64) * 3 / 2;
    let cw = W / 2;
    let chh = H / 2;
    let mut yp = vec![0u8; W * H];
    for j in 0..H {
        for i in 0..W {
            let sx = i as i64 * 2 - tx2;
            let sy = j as i64 * 2 - ty2;
            yp[j * W + i] = luma_at(sx.div_euclid(2), sy.div_euclid(2), phase);
        }
    }
    let mut cb = vec![0u8; cw * chh];
    let mut cr = vec![0u8; cw * chh];
    for j in 0..chh {
        for i in 0..cw {
            let sx = i as i64 - tx2 / 4;
            let sy = j as i64 - ty2 / 4;
            cb[j * cw + i] = 112 + (luma_at(sx, sy, phase + 3) % 32);
            cr[j * cw + i] = 120 + (luma_at(sy, sx, phase + 5) % 32);
        }
    }
    VideoFrame {
        pts: Some(n as i64),
        planes: vec![
            VideoPlane {
                stride: W,
                data: yp,
            },
            VideoPlane {
                stride: cw,
                data: cb,
            },
            VideoPlane {
                stride: cw,
                data: cr,
            },
        ],
    }
}

fn psnr_y(src: &VideoFrame, recon: &Picture) -> f64 {
    let mut sse = 0u64;
    for j in 0..H {
        for i in 0..W {
            let a = src.planes[0].data[j * W + i] as i64;
            let b = recon.y[j * recon.y_stride + i] as i64;
            sse += ((a - b) * (a - b)) as u64;
        }
    }
    if sse == 0 {
        return 99.0;
    }
    let mse = sse as f64 / (W * H) as f64;
    10.0 * (255.0f64 * 255.0 / mse).log10()
}

struct RunResult {
    total_bytes: usize,
    kbps: f64,
    mean_psnr: f64,
    min_psnr: f64,
    keyframes: usize,
}

fn run(version: &str, options: &[(&str, String)]) -> RunResult {
    let codec = match version {
        "v1" => "msmpeg4v1",
        "v2" => "msmpeg4v2",
        _ => "msmpeg4v3",
    };
    let mut params = CodecParameters::video(CodecId::new(codec));
    params.width = Some(W as u32);
    params.height = Some(H as u32);
    for (k, v) in options {
        params.options.insert(*k, v.clone());
    }
    let mut enc = match version {
        "v1" => oxideav_msmpeg4::encoder::make_encoder_v1(&params).unwrap(),
        "v2" => oxideav_msmpeg4::encoder::make_encoder_v2(&params).unwrap(),
        _ => oxideav_msmpeg4::encoder::make_encoder(&params).unwrap(),
    };
    let dims = PictureDims::new(W as u32, H as u32).unwrap();
    let v1v2 = match version {
        "v1" => Some(MsV1V2Version::V1),
        "v2" => Some(MsV1V2Version::V2),
        _ => None,
    };

    let mut total_bytes = 0usize;
    let mut psnr_sum = 0.0f64;
    let mut min_psnr = f64::INFINITY;
    let mut keyframes = 0usize;
    let mut last: Option<Picture> = None;
    for n in 0..FRAMES {
        let vf = make_frame(n);
        enc.send_frame(&Frame::Video(vf.clone())).unwrap();
        let pkt = enc.receive_packet().unwrap();
        total_bytes += pkt.data.len();
        if pkt.flags.keyframe {
            keyframes += 1;
        }
        let mut br = BitReader::new(&pkt.data);
        let recon = match v1v2 {
            None => decode_picture(&mut br, dims, last.as_ref()).unwrap(),
            Some(v) => decode_picture_v1v2(&mut br, dims, v, last.as_ref()).unwrap(),
        };
        let p = psnr_y(&vf, &recon);
        psnr_sum += p;
        min_psnr = min_psnr.min(p);
        last = Some(recon);
    }
    RunResult {
        total_bytes,
        kbps: total_bytes as f64 * 8.0 * FPS / FRAMES as f64 / 1000.0,
        mean_psnr: psnr_sum / FRAMES as f64,
        min_psnr,
        keyframes,
    }
}

fn main() {
    println!(
        "sequence: {W}x{H}, {FRAMES} frames, half-pel pan, hard scene cut at frame {SCENE_CUT}"
    );
    println!();
    println!("| ver | options | bytes | kbps@25 | mean Y-PSNR | min Y-PSNR | keyframes |");
    println!("|-----|---------|-------|---------|-------------|------------|-----------|");
    let mut matrix: Vec<(&str, Vec<(&str, String)>)> = Vec::new();
    for q in [2u32, 4, 8, 16, 31] {
        matrix.push(("v3", vec![("quant", q.to_string()), ("gop", "15".into())]));
    }
    for q in [4u32, 8] {
        matrix.push(("v1", vec![("quant", q.to_string()), ("gop", "15".into())]));
        matrix.push(("v2", vec![("quant", q.to_string()), ("gop", "15".into())]));
    }
    // Tight search window: the sequence pans at 3 half-pel/frame, so a
    // ±2 window only tracks it through the predictor-centred window
    // extension (the predictor chain).
    matrix.push((
        "v3",
        vec![
            ("quant", "8".into()),
            ("gop", "15".into()),
            ("mv_search_range", "2".into()),
        ],
    ));
    // Rate-controlled runs: the achieved kbps column should track the
    // requested bitrate.
    for br in [400_000u32, 150_000] {
        matrix.push((
            "v3",
            vec![("bitrate", br.to_string()), ("gop", "15".into())],
        ));
    }
    for (ver, opts) in &matrix {
        let r = run(ver, opts);
        let optstr = opts
            .iter()
            .map(|(k, v)| format!("{k}={v}"))
            .collect::<Vec<_>>()
            .join(" ");
        println!(
            "| {ver} | {optstr} | {} | {:.1} | {:.2} dB | {:.2} dB | {} |",
            r.total_bytes, r.kbps, r.mean_psnr, r.min_psnr, r.keyframes
        );
    }
}
