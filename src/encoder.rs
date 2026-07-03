//! Registry-facing MS-MPEG4 v3 encoder: wraps the picture-level
//! [`crate::enc`] frame encoders in the [`oxideav_core::Encoder`]
//! frame-to-packet contract, and exposes the crate's historical direct
//! factory endpoint [`make_encoder`].
//!
//! The encoder is a classic I/P GOP machine: the first frame (and
//! every `gop`-th frame after it) is coded as a v3 I-frame, everything
//! else as a P-frame referencing the **decoder-side reconstruction**
//! of the previous frame — after emitting each packet the encoder
//! decodes its own bytes through [`crate::picture::decode_picture`],
//! so encoder and decoder prediction state can never drift.
//!
//! Recognised [`CodecOptions`] keys (all optional):
//!
//! | key               | meaning                          | default |
//! |-------------------|----------------------------------|---------|
//! | `quant`           | frame quantiser, 1..=31          | 4       |
//! | `gop`             | keyframe interval (1 = all-intra)| 12      |
//! | `mv_search_range` | half-pel search radius, 0..=63   | 8       |

use std::collections::VecDeque;

use oxideav_core::bits::BitReader;
use oxideav_core::packet::PacketFlags;
use oxideav_core::{
    CodecId, CodecParameters, Encoder, Error, Frame, Packet, Result, TimeBase, VideoFrame,
};

use crate::enc::{encode_iframe_v3, encode_pframe_v3, EncoderConfig};
use crate::header::PictureType;
use crate::picture::{decode_picture, Picture, PictureDims};

/// Direct factory endpoint (crate convention alongside the registry
/// path): build a boxed MS-MPEG4 v3 encoder from codec parameters.
/// `params.width` / `params.height` are required (MS-MPEG4 carries no
/// dimensions in the bitstream — the container transports them).
pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    MsMpeg4V3Encoder::boxed(params)
}

/// Parse a small positive integer option with a default and an
/// inclusive validity range.
fn int_option(params: &CodecParameters, key: &str, default: u32, lo: u32, hi: u32) -> Result<u32> {
    match params.options.get(key) {
        None => Ok(default),
        Some(raw) => {
            let v: u32 = raw.parse().map_err(|_| {
                Error::invalid(format!(
                    "msmpeg4v3 encoder: option {key}={raw} is not an integer"
                ))
            })?;
            if !(lo..=hi).contains(&v) {
                return Err(Error::invalid(format!(
                    "msmpeg4v3 encoder: option {key}={v} out of range {lo}..={hi}"
                )));
            }
            Ok(v)
        }
    }
}

struct MsMpeg4V3Encoder {
    codec_id: CodecId,
    output_params: CodecParameters,
    dims: PictureDims,
    config: EncoderConfig,
    /// Keyframe interval: 1 = every frame intra.
    gop: u32,
    /// Frames emitted since (and including) the last I-frame.
    frames_since_key: u32,
    /// Decoder-side reconstruction of the previously emitted frame.
    last_recon: Option<Picture>,
    queue: VecDeque<Packet>,
}

impl MsMpeg4V3Encoder {
    fn boxed(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
        let (w, h) = match (params.width, params.height) {
            (Some(w), Some(h)) if w > 0 && h > 0 => (w, h),
            _ => {
                return Err(Error::invalid(
                    "msmpeg4v3 encoder: CodecParameters must carry non-zero \
                     width/height (MS-MPEG4 streams do not transport dimensions; \
                     the container does)",
                ));
            }
        };
        let dims = PictureDims::new(w, h)?;
        let quant = int_option(params, "quant", 4, 1, 31)?;
        let gop = int_option(params, "gop", 12, 1, 600)?;
        let mv_search_range = int_option(params, "mv_search_range", 8, 0, 63)?;

        let mut output_params = params.clone();
        output_params.codec_id = CodecId::new(crate::CODEC_ID_V3);
        Ok(Box::new(Self {
            codec_id: CodecId::new(crate::CODEC_ID_V3),
            output_params,
            dims,
            config: EncoderConfig {
                quant: quant as u8,
                mv_search_range: mv_search_range as u8,
            },
            gop,
            frames_since_key: 0,
            last_recon: None,
            queue: VecDeque::new(),
        }))
    }

    /// Copy a 3-plane YUV420 [`VideoFrame`] into the MB-aligned
    /// [`Picture`] layout the picture encoder consumes, replicating the
    /// right/bottom edge pels into the padding region (the standard
    /// padding choice — it adds no artificial high-frequency energy at
    /// the picture boundary).
    fn frame_to_picture(&self, vf: &VideoFrame) -> Result<Picture> {
        if vf.planes.len() != 3 {
            return Err(Error::invalid(format!(
                "msmpeg4v3 encoder: expected 3 YUV420 planes, got {}",
                vf.planes.len()
            )));
        }
        let mut pic = Picture::alloc(self.dims, PictureType::I);
        let w = self.dims.width as usize;
        let h = self.dims.height as usize;
        let cw = w.div_ceil(2);
        let chh = h.div_ceil(2);
        copy_plane(
            &vf.planes[0].data,
            vf.planes[0].stride,
            w,
            h,
            &mut pic.y,
            pic.y_stride,
        )?;
        copy_plane(
            &vf.planes[1].data,
            vf.planes[1].stride,
            cw,
            chh,
            &mut pic.cb,
            pic.c_stride,
        )?;
        copy_plane(
            &vf.planes[2].data,
            vf.planes[2].stride,
            cw,
            chh,
            &mut pic.cr,
            pic.c_stride,
        )?;
        Ok(pic)
    }
}

/// Copy `w × h` pels from a source plane into the destination plane,
/// replicating the last source column/row across the MB-alignment
/// padding.
fn copy_plane(
    src: &[u8],
    src_stride: usize,
    w: usize,
    h: usize,
    dst: &mut [u8],
    dst_stride: usize,
) -> Result<()> {
    if w == 0 || h == 0 || src_stride < w || src.len() < (h - 1) * src_stride + w {
        return Err(Error::invalid(
            "msmpeg4v3 encoder: video plane smaller than the stream dimensions",
        ));
    }
    let dst_h = dst.len() / dst_stride;
    for y in 0..dst_h {
        let sy = y.min(h - 1);
        let src_row = &src[sy * src_stride..sy * src_stride + w];
        let dst_row = &mut dst[y * dst_stride..(y + 1) * dst_stride];
        dst_row[..w].copy_from_slice(src_row);
        let edge = src_row[w - 1];
        for d in dst_row[w..].iter_mut() {
            *d = edge;
        }
    }
    Ok(())
}

impl Encoder for MsMpeg4V3Encoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn output_params(&self) -> &CodecParameters {
        &self.output_params
    }

    fn send_frame(&mut self, frame: &Frame) -> Result<()> {
        let vf = match frame {
            Frame::Video(vf) => vf,
            _ => {
                return Err(Error::invalid("msmpeg4v3 encoder: expected a video frame"));
            }
        };
        let input = self.frame_to_picture(vf)?;

        let force_key = self.last_recon.is_none() || self.frames_since_key >= self.gop;
        let (bytes, is_key) = if force_key {
            (encode_iframe_v3(&input, self.dims, &self.config)?, true)
        } else {
            let reference = self.last_recon.as_ref().expect("checked above");
            (
                encode_pframe_v3(&input, reference, self.dims, &self.config)?,
                false,
            )
        };

        // Decode our own bytes so the next P-frame references exactly
        // what a decoder will hold (no encoder/decoder drift possible).
        let mut br = BitReader::new(&bytes);
        let recon = decode_picture(&mut br, self.dims, self.last_recon.as_ref())?;
        self.last_recon = Some(recon);
        self.frames_since_key = if is_key { 1 } else { self.frames_since_key + 1 };

        let time_base = self
            .output_params
            .frame_rate
            .map(|r| TimeBase::new(r.den, r.num))
            .unwrap_or(TimeBase::MILLIS);
        let mut pkt = Packet::new(0, time_base, bytes);
        pkt.pts = vf.pts;
        pkt.dts = vf.pts;
        pkt.flags = PacketFlags {
            keyframe: is_key,
            ..PacketFlags::default()
        };
        self.queue.push_back(pkt);
        Ok(())
    }

    fn receive_packet(&mut self) -> Result<Packet> {
        self.queue.pop_front().ok_or(Error::NeedMore)
    }

    fn flush(&mut self) -> Result<()> {
        // Intra/inter-only codec with no frame delay: nothing buffered
        // beyond the already-queued packets. Reset the GOP state so a
        // reused encoder restarts on a keyframe.
        self.last_recon = None;
        self.frames_since_key = 0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params(w: u32, h: u32) -> CodecParameters {
        let mut p = CodecParameters::video(CodecId::new("msmpeg4v3"));
        p.width = Some(w);
        p.height = Some(h);
        p
    }

    fn test_frame(w: usize, h: usize, shift: usize) -> VideoFrame {
        let cw = w.div_ceil(2);
        let chh = h.div_ceil(2);
        let mut y = vec![0u8; w * h];
        for j in 0..h {
            for i in 0..w {
                y[j * w + i] = (((i + shift) * 3 + j * 2) % 200 + 20) as u8;
            }
        }
        VideoFrame {
            pts: Some(shift as i64),
            planes: vec![
                oxideav_core::VideoPlane { stride: w, data: y },
                oxideav_core::VideoPlane {
                    stride: cw,
                    data: vec![120u8; cw * chh],
                },
                oxideav_core::VideoPlane {
                    stride: cw,
                    data: vec![136u8; cw * chh],
                },
            ],
        }
    }

    #[test]
    fn encoder_requires_dimensions() {
        let p = CodecParameters::video(CodecId::new("msmpeg4v3"));
        assert!(make_encoder(&p).is_err());
    }

    #[test]
    fn encoder_rejects_bad_options() {
        let mut p = params(32, 32);
        p.options.insert("quant", "0");
        assert!(make_encoder(&p).is_err());
        let mut p = params(32, 32);
        p.options.insert("quant", "wat");
        assert!(make_encoder(&p).is_err());
    }

    #[test]
    fn gop_machine_emits_i_then_p_then_i_at_interval() {
        let mut p = params(48, 32);
        p.options.insert("gop", "3");
        p.options.insert("quant", "3");
        let mut enc = make_encoder(&p).unwrap();
        let mut keyflags = Vec::new();
        for n in 0..7usize {
            enc.send_frame(&Frame::Video(test_frame(48, 32, n)))
                .unwrap();
            let pkt = enc.receive_packet().unwrap();
            assert_eq!(pkt.pts, Some(n as i64));
            keyflags.push(pkt.flags.keyframe);
        }
        assert_eq!(
            keyflags,
            vec![true, false, false, true, false, false, true],
            "gop=3 keyframe cadence"
        );
        assert!(matches!(enc.receive_packet(), Err(Error::NeedMore)));
    }

    #[test]
    fn encoded_stream_decodes_through_registered_decoder_path() {
        // Encoder packets → picture::decode_picture chain (the same
        // decode path the registered decoder drives) must reconstruct
        // each frame within the quantiser loss bound.
        let mut p = params(48, 48);
        p.options.insert("quant", "2");
        let mut enc = make_encoder(&p).unwrap();
        let dims = PictureDims::new(48, 48).unwrap();
        let mut last: Option<Picture> = None;
        for n in 0..4usize {
            let vf = test_frame(48, 48, n * 2);
            enc.send_frame(&Frame::Video(vf.clone())).unwrap();
            let pkt = enc.receive_packet().unwrap();
            let mut br = BitReader::new(&pkt.data);
            let out = decode_picture(&mut br, dims, last.as_ref()).unwrap();
            // Compare the visible region against the source frame.
            let mut sum = 0u64;
            for j in 0..48 {
                for i in 0..48 {
                    let a = out.y[j * out.y_stride + i] as i64;
                    let b = vf.planes[0].data[j * 48 + i] as i64;
                    sum += (a - b).unsigned_abs();
                }
            }
            let mae = sum as f64 / (48.0 * 48.0);
            assert!(mae < 3.0, "frame {n}: luma MAE {mae} too large");
            last = Some(out);
        }
    }

    #[test]
    fn flush_restarts_on_a_keyframe() {
        let p = params(32, 32);
        let mut enc = make_encoder(&p).unwrap();
        enc.send_frame(&Frame::Video(test_frame(32, 32, 0)))
            .unwrap();
        enc.send_frame(&Frame::Video(test_frame(32, 32, 1)))
            .unwrap();
        let first = enc.receive_packet().unwrap();
        let second = enc.receive_packet().unwrap();
        assert!(first.flags.keyframe && !second.flags.keyframe);
        enc.flush().unwrap();
        enc.send_frame(&Frame::Video(test_frame(32, 32, 2)))
            .unwrap();
        let after = enc.receive_packet().unwrap();
        assert!(after.flags.keyframe, "post-flush frame must be intra");
    }
}
