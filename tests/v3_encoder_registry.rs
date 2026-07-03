//! Registry-path end-to-end: create the v3 encoder through the
//! `oxideav_core::RuntimeContext` codec registry, feed it frames, and
//! decode the emitted packets through the **registered decoder**
//! (`send_packet` / `receive_frame`) — the full dual-API loop.

use oxideav_core::packet::PacketFlags;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase, VideoFrame, VideoPlane};

fn video_params(w: u32, h: u32) -> CodecParameters {
    let mut p = CodecParameters::video(CodecId::new("msmpeg4v3"));
    p.width = Some(w);
    p.height = Some(h);
    p
}

fn moving_frame(w: usize, h: usize, n: usize) -> VideoFrame {
    let cw = w.div_ceil(2);
    let chh = h.div_ceil(2);
    let mut y = vec![0u8; w * h];
    for j in 0..h {
        for i in 0..w {
            y[j * w + i] = (((i + n * 2) * 5 + j * 3) % 190 + 30) as u8;
        }
    }
    let mut cb = vec![0u8; cw * chh];
    let mut cr = vec![0u8; cw * chh];
    for j in 0..chh {
        for i in 0..cw {
            cb[j * cw + i] = ((i * 4 + n) % 180 + 40) as u8;
            cr[j * cw + i] = ((j * 4 + n) % 180 + 40) as u8;
        }
    }
    VideoFrame {
        pts: Some(n as i64),
        planes: vec![
            VideoPlane { stride: w, data: y },
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

#[test]
fn registry_exposes_v3_encoder_factory() {
    let mut ctx = oxideav_core::RuntimeContext::new();
    oxideav_msmpeg4::register(&mut ctx);
    assert!(ctx.codecs.has_encoder(&CodecId::new("msmpeg4v3")));
    assert!(ctx.codecs.has_encoder(&CodecId::new("div3")));
    // v1/v2 remain decode-only.
    assert!(!ctx.codecs.has_encoder(&CodecId::new("msmpeg4v1")));
    assert!(!ctx.codecs.has_encoder(&CodecId::new("msmpeg4v2")));
}

#[test]
fn registry_encoder_to_registered_decoder_loop() {
    let mut ctx = oxideav_core::RuntimeContext::new();
    oxideav_msmpeg4::register(&mut ctx);

    let mut params = video_params(64, 48);
    params.options.insert("quant", "2");
    params.options.insert("gop", "4");
    let mut enc = ctx.codecs.first_encoder(&params).expect("encoder factory");
    let mut dec = ctx.codecs.first_decoder(&params).expect("decoder factory");

    for n in 0..6usize {
        let vf = moving_frame(64, 48, n);
        enc.send_frame(&Frame::Video(vf.clone()))
            .expect("send_frame");
        let pkt = enc.receive_packet().expect("receive_packet");
        assert_eq!(
            pkt.flags.keyframe,
            n % 4 == 0,
            "frame {n}: gop=4 keyframe cadence"
        );

        // Re-wrap so the decoder sees exactly the on-wire payload.
        let mut in_pkt = Packet::new(0, TimeBase::MILLIS, pkt.data.clone());
        in_pkt.pts = pkt.pts;
        in_pkt.flags = PacketFlags {
            keyframe: pkt.flags.keyframe,
            ..PacketFlags::default()
        };
        dec.send_packet(&in_pkt).expect("send_packet");
        let frame = dec.receive_frame().expect("receive_frame");
        let Frame::Video(out) = frame else {
            panic!("expected a video frame");
        };
        assert_eq!(out.pts, Some(n as i64), "pts must round-trip");

        // Visible-region luma comparison against the source.
        let y = &out.planes[0];
        let mut sum = 0u64;
        for j in 0..48 {
            for i in 0..64 {
                let a = y.data[j * y.stride + i] as i64;
                let b = vf.planes[0].data[j * 64 + i] as i64;
                sum += (a - b).unsigned_abs();
            }
        }
        let mae = sum as f64 / (64.0 * 48.0);
        assert!(mae < 3.0, "frame {n}: registered-loop luma MAE {mae}");
    }
}
