//! Microsoft MPEG-4 (v1 / v2 / v3) codec support — probe + decoder stubs.
//!
//! Microsoft MPEG-4 is a family of three pre-standard codecs Microsoft
//! shipped in 1999-2001 as part of Windows Media Tools. DivXNetworks
//! forked version 3 into the original "DivX ;-)" release. Despite the
//! name they are **not** the same bitstream as the later ISO/IEC
//! 14496-2 MPEG-4 Part 2 that XVID / Xvid / DivX 4+ and FMP4 implement.
//! The differences are substantial: different headers (no VOS / VOL),
//! different VLC tables for motion vectors + DC + AC coefficients, a
//! different slice layout, and different B-frame support.
//!
//! | Codec id   | Also known as                          | FourCCs                   |
//! | ---------- | -------------------------------------- | ------------------------- |
//! | msmpeg4v1  | MS MPEG-4 v1 / Windows Media Video 7 S | `MP41`, `MPG4`            |
//! | msmpeg4v2  | MS MPEG-4 v2                           | `MP42`                    |
//! | msmpeg4v3  | MS MPEG-4 v3 / DivX ;-) 3              | `MP43`, `MPG3`, `DIV3`,   |
//! |            |                                        | `DIV4`, `DIV5`, `DIV6`,   |
//! |            |                                        | `AP41`                    |
//!
//! # Why this crate exists
//!
//! The MS-MPEG4 and MPEG-4 Part 2 FourCC lists are routinely
//! mislabelled in the wild: DIV3 files that are really DX50 streams,
//! XVID files that are really MP43, and everything in between. Trusting
//! the FourCC alone produces garbage output or silent failures. The
//! [`classify`] function inspects the first few bytes of the actual
//! bitstream and reports which family it belongs to — a container
//! demuxer can then dispatch to the correct codec crate regardless of
//! what the file claims.
//!
//! # Status
//!
//! **v3 partial decoder.** [`classify`] is fully implemented and
//! tested. The v3 decoder now runs end-to-end for I-frames (with a
//! DC-only placeholder path where the intra-AC VLC is still the spec
//! §9 OPEN clean-room extraction item) and for P-frames (skip + inter
//! MC-only; inter-AC VLC also OPEN). P-frame MC uses the `0x1c25cbc0`
//! default joint MV VLC (1100 entries + ESC) with the `0x1c25ee28` /
//! `0x1c25f278` byte LUTs for MVDx / MVDy, all extracted into
//! `tables/` and wired through `build.rs`. The alternate MV VLC
//! (`0x1c25a0b8`) has only a 256-byte extraction dump and is rejected
//! with a documented `Unsupported` error.
//!
//! **v1 / v2 picture header + MCBPCY decoders.** Round 11 (2026-04-25)
//! wires:
//!
//! - [`header::MsV1V2PictureHeader::parse_v1`] / `::parse_v2` — picture
//!   header parsers per spec/99 §2.2 / §2.4 (including the v1 37-bit
//!   opaque preamble and 1-bit UMV flag on P-frames).
//! - [`mcbpcy::decode_mcbpcy_v1`] — v1 MB-header decoder per spec/07 §1
//!   (`0x1c2171c7`), driving the separated 21-entry MCBPC VLC at VMA
//!   `0x1c253d40` (max-bitlen 9) + the shared 16-entry CBPY VLC at VMA
//!   `0x1c254240` (max-bitlen 6, with the `15 - cbpy_raw` one's-
//!   complement wrap).
//! - [`mcbpcy::decode_mcbpcy_v2`] — v2 MB-header decoder per spec/07 §2
//!   (`0x1c21729c`), driving the 8-entry MCBPC VLC at VMA `0x1c254140`
//!   (max-bitlen 7) + the same shared CBPY, with v2's frame-type
//!   conditional skip-bit, intra-in-P AC-prediction bit, and
//!   conditional-wrap path for `mcbpc % 4 == 3`.
//!
//! Both MCBPC VLC tables are extracted directly from
//! `docs/video/msmpeg4/tables/region_053140.hex` (1280-byte combined
//! LUT region: bytes 0..1024 are the v1 9-bit LUT, bytes 1024..1280 are
//! the v2 7-bit LUT). The build script (`build.rs::emit_mcbpc_v1_v2`)
//! parses the 2-byte `(bit_length, symbol)` records and emits canonical-
//! Huffman `(symbol, bit_length, code)` triples; the runtime decoder
//! goes through the same linear-scan VLC path as v3's MCBPCY.
//!
//! **v1 / v2 per-component MV decoder.** Round 12 (2026-04-25) lands the
//! 65-entry MV VLC table at VMA `0x1c24f930` and the per-axis decoder
//! [`mv::decode_mv_v1v2`]. Per spec/06 §2.3 / spec/07 §3, the v1/v2
//! decoder body at `0x1c217e56` (v<4 branch) does two separate
//! canonical-Huffman reads against the same shared table, applies the
//! median-of-3 predictor (re-using v3's [`mv::median_predictor`]),
//! subtracts the bias 32, then toroidally wraps each component into
//! `[-63, +63]`. Three notable findings drove the round:
//!
//! - The docs region at `tables/region_04ed30.hex` only captures the
//!   first 4096 bytes (1/4 of the LUT). The helper `0x1c215811` is
//!   invoked with max-bitlen 13 → 8192 entries × 2 bytes = 16384 bytes;
//!   the FULL extraction lives in
//!   `crates/oxideav-msmpeg4/tables/region_04ed30_full.hex`, copied
//!   directly from the source DLL (verified SHA-256).
//! - The alphabet is **65 entries (0..=64)**, not 33 as documented in
//!   spec/06 §2.3 / spec/07 §3.3. The `eax + ecx - 0x20` bias
//!   arithmetic gives signed `[-32, +32]` only when the raw idx ranges
//!   `[0, 64]` (33 would only cover `[-32, 0]`). Symbol 32 = MVD 0 is
//!   the most-probable code at 1 bit.
//! - The 65 codes plus 4 escape sentinel slots (the bit-reader-error
//!   path at helper offset `1c21587f`) saturate the 13-bit prefix space
//!   (Kraft sum = 1 - 4/2^13 over the 65 codes).
//!
//! Per-MB pixel reconstruction for v1/v2 P-frames is still gated on the
//! per-version intra-AC and inter-AC VLC tables (spec/99 §9 OPEN-O4) —
//! the MV decoder produces the (MVx, MVy) byte pair that the MC
//! fetcher consumes, but the residual path needs the AC tables for a
//! complete pipeline. The v1/v2 `send_packet` therefore continues to
//! return the same documented `Unsupported` after parsing the picture
//! header.
//!
//! The full v1/v2 per-MB decode (intra-AC, inter-AC) still requires the
//! corresponding VLC tables which are spec/99 §9 OPEN — `send_packet`
//! parses the picture header successfully and then surfaces a
//! documented `Unsupported` citing the open spec item, so callers can
//! distinguish "header malformed" from "decoder gap".

#![deny(unsafe_code)]

use std::collections::VecDeque;

use oxideav_core::{
    CodecCapabilities, CodecId, CodecParameters, CodecTag, Error, Frame, Packet, ProbeContext,
    Result, VideoFrame, VideoPlane,
};
use oxideav_core::{CodecInfo, CodecRegistry, Decoder};

pub mod ac;
pub mod dc_pred;
pub mod header;
pub mod idct;
pub mod iq;
pub mod mb;
pub mod mc;
pub mod mcbpcy;
pub mod mv;
pub mod picture;
pub mod scan;
pub mod tables;
pub mod tables_data;
pub mod vlc;

pub const CODEC_ID_V1: &str = "msmpeg4v1";
pub const CODEC_ID_V2: &str = "msmpeg4v2";
pub const CODEC_ID_V3: &str = "msmpeg4v3";

// ==================== Classification ====================

/// What a given bitstream's first packet looks like.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Classification {
    /// Standard MPEG-4 Part 2 (ISO/IEC 14496-2). Dispatch to
    /// [`oxideav-mpeg4video`](https://crates.io/crates/oxideav-mpeg4video).
    Mpeg4Part2,
    /// Microsoft MPEG-4 v1. Dispatch to this crate's `msmpeg4v1`.
    MsMpeg4V1,
    /// Microsoft MPEG-4 v2. Dispatch to this crate's `msmpeg4v2`.
    MsMpeg4V2,
    /// Microsoft MPEG-4 v3 (DivX 3). Dispatch to this crate's `msmpeg4v3`.
    MsMpeg4V3,
    /// Buffer too small, empty, or doesn't match any known signature.
    Unknown,
}

impl Classification {
    pub fn is_ms_mpeg4(self) -> bool {
        matches!(self, Self::MsMpeg4V1 | Self::MsMpeg4V2 | Self::MsMpeg4V3)
    }

    pub fn is_mpeg4_part2(self) -> bool {
        matches!(self, Self::Mpeg4Part2)
    }

    /// The canonical codec id string for this classification, or
    /// `None` if the family is unknown.
    pub fn codec_id(self) -> Option<&'static str> {
        match self {
            Self::Mpeg4Part2 => Some("mpeg4video"),
            Self::MsMpeg4V1 => Some(CODEC_ID_V1),
            Self::MsMpeg4V2 => Some(CODEC_ID_V2),
            Self::MsMpeg4V3 => Some(CODEC_ID_V3),
            Self::Unknown => None,
        }
    }
}

/// Inspect a raw bitstream and classify it as MPEG-4 Part 2 or a
/// Microsoft MPEG-4 variant.
///
/// **The FourCC hint is advisory.** If `fourcc` is `Some(b"DIV3")` but
/// the bytes start with an ISO `0x000001B0` start code, the result is
/// [`Classification::Mpeg4Part2`] — the file was mislabelled and the
/// caller should dispatch to the standard decoder.
///
/// Heuristics (cheap, byte-level only):
///
/// 1. Any MPEG-4 Part 2 frame begins with one of the start-code
///    prefixes `0x000001B0`..`0x000001BF` / `0x00000120`..`0x0000012F`.
///    These cannot appear at the start of an MS-MPEG4 frame because MS
///    frames begin with a VLC-decoded picture header that almost never
///    has three leading zero bytes.
/// 2. If no start code is present, the FourCC hint disambiguates v1
///    vs v2 vs v3. Without the hint we default to v3 (by far the most
///    common variant).
pub fn classify(data: &[u8], fourcc: Option<&[u8; 4]>) -> Classification {
    if has_mpeg4_part2_start_code(data) {
        return Classification::Mpeg4Part2;
    }
    if data.is_empty() {
        return Classification::Unknown;
    }
    // No ISO start code — assume MS-MPEG4. Use the FourCC to pick the
    // variant; fall back to v3 (the most common) when the hint is
    // absent or unrecognised.
    match fourcc {
        Some(fc) => match &uppercase4(fc) {
            b"MP41" | b"MPG4" => Classification::MsMpeg4V1,
            b"MP42" => Classification::MsMpeg4V2,
            b"MP43" | b"MPG3" | b"DIV3" | b"DIV4" | b"DIV5" | b"DIV6" | b"AP41" => {
                Classification::MsMpeg4V3
            }
            _ => Classification::MsMpeg4V3,
        },
        None => Classification::MsMpeg4V3,
    }
}

/// True if `data` contains an MPEG-4 Part 2 start code prefix
/// (`0x000001` followed by a start-code-marker byte in the
/// visual-object / VOL / VOP / GOV / Visual-Object-Sequence range)
/// within the first 2 KB.
///
/// The spec requires that any MPEG-4 Part 2 elementary stream begins
/// with a `visual_object_sequence_start_code` (`0x000001B0`) or at
/// minimum with a `video_object_layer_start_code` (`0x00000120` -
/// `0x0000012F`). We scan the whole prefix rather than matching just
/// byte 0 so the check works even when a container prepends a short
/// bit of padding.
fn has_mpeg4_part2_start_code(data: &[u8]) -> bool {
    if data.len() < 4 {
        return false;
    }
    // Limit the scan so the worst case is bounded.
    let scan = &data[..data.len().min(2048)];
    for w in scan.windows(4) {
        if w[0] == 0x00 && w[1] == 0x00 && w[2] == 0x01 {
            let marker = w[3];
            // VOP / VOL / VOS / GOV / Visual-Object / user-data start codes.
            if matches!(marker, 0xB0..=0xBF) || (0x20..=0x2F).contains(&marker) {
                return true;
            }
        }
    }
    false
}

fn uppercase4(fc: &[u8; 4]) -> [u8; 4] {
    let mut out = [0u8; 4];
    for i in 0..4 {
        out[i] = fc[i].to_ascii_uppercase();
    }
    out
}

// ==================== Registration ====================

const ALIASES_V1: &[&str] = &[CODEC_ID_V1, "msmpeg4v1"];
const ALIASES_V2: &[&str] = &[CODEC_ID_V2, "msmpeg4v2"];
const ALIASES_V3: &[&str] = &[CODEC_ID_V3, "msmpeg4v3", "div3"];

/// Probe used in tag claims: returns a confidence in `0.0..=1.0` that
/// the context describes an MS-MPEG4 stream (not an ISO/IEC 14496-2
/// MPEG-4 Part 2 stream). Inspects `ctx.packet` (or `ctx.header` as a
/// fallback) via [`classify`]. When no bytes are available, returns a
/// moderate-confidence value because the FourCC has already routed the
/// probe here.
pub fn probe_is_msmpeg4(ctx: &ProbeContext) -> f32 {
    match ctx.packet.or(ctx.header) {
        Some(d) => {
            if classify(d, None).is_mpeg4_part2() {
                0.0
            } else {
                1.0
            }
        }
        None => 0.6,
    }
}

/// Register the msmpeg4v1 / v2 / v3 decoders with a [`CodecRegistry`]
/// and claim their container tags.
///
/// **Tag ownership:**
///
/// - `DIV3`, `DIV4`, `DIV5`, `DIV6`, `MP43`, `MPG3`, `AP41` — MS-MPEG4v3.
/// - `MP42` — MS-MPEG4v2.
/// - `MP41`, `MPG4` — MS-MPEG4v1.
///
/// Each claim carries the [`probe_is_msmpeg4`] probe so the registry
/// can re-route a packet that's actually MPEG-4 Part 2 (mislabelled
/// FourCC) to the mpeg4video decoder — `oxideav-mpeg4video` claims
/// the same FourCCs with its own mirror probe, and whichever probe
/// returns the higher confidence wins.
///
/// The decoders themselves currently return
/// `Error::Unsupported` on `send_packet`, with distinct diagnostic
/// messages for the two failure modes.
pub fn register(reg: &mut CodecRegistry) {
    fn make_caps(name: &'static str) -> CodecCapabilities {
        CodecCapabilities::video(name).with_intra_only(false)
    }

    // --- v1: factories for every alias, tags attached to the canonical one.
    for (idx, alias) in ALIASES_V1.iter().enumerate() {
        let mut info = CodecInfo::new(CodecId::new(*alias))
            .capabilities(make_caps("msmpeg4v1_sw"))
            .decoder(|params| MsMpeg4Decoder::boxed(params, MsVersion::V1));
        if idx == 0 {
            info = info
                .probe(probe_is_msmpeg4)
                .tags([CodecTag::fourcc(b"MP41"), CodecTag::fourcc(b"MPG4")]);
        }
        reg.register(info);
    }

    // --- v2.
    for (idx, alias) in ALIASES_V2.iter().enumerate() {
        let mut info = CodecInfo::new(CodecId::new(*alias))
            .capabilities(make_caps("msmpeg4v2_sw"))
            .decoder(|params| MsMpeg4Decoder::boxed(params, MsVersion::V2));
        if idx == 0 {
            info = info.probe(probe_is_msmpeg4).tag(CodecTag::fourcc(b"MP42"));
        }
        reg.register(info);
    }

    // --- v3 (DivX 3). Biggest FourCC pool; oxideav-mpeg4video claims the
    // same set with its mirror probe for the mislabel case.
    for (idx, alias) in ALIASES_V3.iter().enumerate() {
        let mut info = CodecInfo::new(CodecId::new(*alias))
            .capabilities(make_caps("msmpeg4v3_sw"))
            .decoder(|params| MsMpeg4Decoder::boxed(params, MsVersion::V3));
        if idx == 0 {
            info = info.probe(probe_is_msmpeg4).tags([
                CodecTag::fourcc(b"DIV3"),
                CodecTag::fourcc(b"DIV4"),
                CodecTag::fourcc(b"DIV5"),
                CodecTag::fourcc(b"DIV6"),
                CodecTag::fourcc(b"MP43"),
                CodecTag::fourcc(b"MPG3"),
                CodecTag::fourcc(b"AP41"),
            ]);
        }
        reg.register(info);
    }
}

// ==================== Picture → VideoFrame ====================

/// Wrap a decoded [`picture::Picture`] in an [`oxideav_core::VideoFrame`].
///
/// The picture's internal strides are MB-aligned multiples of 16 (luma)
/// and 8 (chroma); `VideoFrame` carries `(stride, data)` per plane
/// verbatim. `pts` is propagated from the source packet. Stream-level
/// properties (pixel format, width, height, time base) live on the
/// stream's [`CodecParameters`], not the frame.
fn picture_to_video_frame(pic: &picture::Picture, pts: Option<i64>) -> VideoFrame {
    VideoFrame {
        pts,
        planes: vec![
            VideoPlane {
                stride: pic.y_stride,
                data: pic.y.clone(),
            },
            VideoPlane {
                stride: pic.c_stride,
                data: pic.cb.clone(),
            },
            VideoPlane {
                stride: pic.c_stride,
                data: pic.cr.clone(),
            },
        ],
    }
}

// ==================== Decoder stub ====================

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MsVersion {
    V1,
    V2,
    V3,
}

impl MsVersion {
    fn codec_id(self) -> &'static str {
        match self {
            Self::V1 => CODEC_ID_V1,
            Self::V2 => CODEC_ID_V2,
            Self::V3 => CODEC_ID_V3,
        }
    }
}

struct MsMpeg4Decoder {
    codec_id: CodecId,
    version: MsVersion,
    output_queue: VecDeque<Frame>,
    // Cache the classification of the first packet so repeated
    // send_packet calls don't re-scan the same start-code window.
    classified: Option<Classification>,
    // Picture dimensions carried from the container's CodecParameters
    // (MS-MPEG4 does not encode dimensions in the bitstream). None if
    // the container supplied 0 or no dimensions — in that case the
    // decoder can still parse headers but not allocate pictures.
    dims: Option<picture::PictureDims>,
    // Last decoded picture, retained so P-frames can use it as an MC
    // reference. Cleared on `flush()`.
    last_picture: Option<picture::Picture>,
}

impl MsMpeg4Decoder {
    fn boxed(params: &CodecParameters, version: MsVersion) -> Result<Box<dyn Decoder>> {
        let dims = match (params.width, params.height) {
            (Some(w), Some(h)) if w > 0 && h > 0 => picture::PictureDims::new(w, h).ok(),
            _ => None,
        };
        Ok(Box::new(Self {
            codec_id: CodecId::new(version.codec_id()),
            version,
            output_queue: VecDeque::new(),
            classified: None,
            dims,
            last_picture: None,
        }))
    }
}

impl Decoder for MsMpeg4Decoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        if self.classified.is_none() {
            // Use the packet's bytes for classification; we don't know
            // the container's FourCC here, so rely on the start-code
            // check alone (which is what matters for mislabelling).
            self.classified = Some(classify(&packet.data, None));
        }
        match self.classified.unwrap() {
            Classification::Mpeg4Part2 => Err(Error::unsupported(format!(
                "{}: packet looks like standard MPEG-4 Part 2 (ISO/IEC 14496-2) — \
                 bitstream starts with a 0x000001 start code. The file's FourCC is \
                 probably mislabelled; dispatch to oxideav-mpeg4video instead.",
                self.version.codec_id(),
            ))),
            _ => {
                let dims = self.dims.ok_or_else(|| {
                    Error::invalid(format!(
                        "{}: CodecParameters missing width/height — MS-MPEG4 needs \
                         them from the container (AVI BITMAPINFOHEADER or equivalent).",
                        self.version.codec_id(),
                    ))
                })?;
                let mut br = oxideav_core::bits::BitReader::new(&packet.data);
                match self.version {
                    MsVersion::V3 => {
                        let pic =
                            picture::decode_picture(&mut br, dims, self.last_picture.as_ref())?;
                        let frame = picture_to_video_frame(&pic, packet.pts);
                        self.last_picture = Some(pic);
                        self.output_queue.push_back(Frame::Video(frame));
                        Ok(())
                    }
                    MsVersion::V1 | MsVersion::V2 => {
                        // v1/v2 picture-header parse + MCBPCY decoders
                        // are wired (spec/07 §1-§2). The downstream
                        // intra-AC and inter-AC VLCs are not yet
                        // extracted (spec/99 §9 OPEN). For now we parse
                        // the header (proves wiring is end-to-end) and
                        // surface a documented Unsupported with the
                        // exact spec citation so callers can tell apart
                        // "header malformed" from "decoder gap".
                        let hdr = match self.version {
                            MsVersion::V1 => header::MsV1V2PictureHeader::parse_v1(&mut br)?,
                            MsVersion::V2 => header::MsV1V2PictureHeader::parse_v2(&mut br)?,
                            _ => unreachable!(),
                        };
                        Err(Error::unsupported(format!(
                            "{}: parsed picture header (type={:?}, quant={}, umv={}) \
                             but per-MB decode requires the v1/v2 intra-AC and \
                             inter-AC VLC tables which are still OPEN per \
                             docs/video/msmpeg4/spec/99-current-understanding.md \
                             §9 OPEN-O4. The MCBPC + CBPY pipeline (spec/07 \
                             §1, §2) is wired and tested.",
                            self.version.codec_id(),
                            hdr.picture_type,
                            hdr.quant,
                            hdr.v1_umv_flag,
                        )))
                    }
                }
            }
        }
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        if let Some(f) = self.output_queue.pop_front() {
            return Ok(f);
        }
        Err(Error::NeedMore)
    }

    fn flush(&mut self) -> Result<()> {
        self.output_queue.clear();
        self.classified = None;
        self.last_picture = None;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vos_header() -> Vec<u8> {
        // Visual object sequence start code + a minimal payload.
        vec![0x00, 0x00, 0x01, 0xB0, 0x01, 0x00, 0x00, 0x01, 0xB5]
    }

    fn vol_header() -> Vec<u8> {
        // Video object layer start code (0x120).
        vec![0x00, 0x00, 0x01, 0x20, 0x08, 0x08, 0x40, 0x00]
    }

    fn vop_header() -> Vec<u8> {
        // VOP start code (0x1B6).
        vec![0x00, 0x00, 0x01, 0xB6, 0x40, 0x00, 0x00]
    }

    fn msmpeg4v3_picture_header() -> Vec<u8> {
        // MS-MPEG4v3 I-frame picture header: opens with `1` bit for
        // frame type followed by quant bits. The key property is that
        // there is NO 0x000001 start code anywhere in the first few
        // bytes.
        vec![0x85, 0x3F, 0xD4, 0x80, 0x00, 0xA2, 0x10, 0xFF]
    }

    #[test]
    fn mpeg4_part2_vos_classifies_as_iso() {
        assert_eq!(classify(&vos_header(), None), Classification::Mpeg4Part2,);
        assert_eq!(
            classify(&vos_header(), Some(b"XVID")),
            Classification::Mpeg4Part2,
        );
    }

    #[test]
    fn mpeg4_part2_vol_classifies_as_iso() {
        assert_eq!(classify(&vol_header(), None), Classification::Mpeg4Part2,);
    }

    #[test]
    fn mpeg4_part2_vop_classifies_as_iso() {
        assert_eq!(classify(&vop_header(), None), Classification::Mpeg4Part2,);
    }

    #[test]
    fn mislabelled_div3_fourcc_on_actual_part2_stream() {
        // File says DIV3 but bytes are MPEG-4 Part 2 (VOS start code).
        // classify() should prefer the bitstream evidence.
        assert_eq!(
            classify(&vos_header(), Some(b"DIV3")),
            Classification::Mpeg4Part2,
        );
    }

    #[test]
    fn msmpeg4v3_picture_no_start_code() {
        assert_eq!(
            classify(&msmpeg4v3_picture_header(), None),
            Classification::MsMpeg4V3,
        );
    }

    #[test]
    fn fourcc_disambiguates_ms_variants() {
        let bytes = msmpeg4v3_picture_header();
        assert_eq!(classify(&bytes, Some(b"MP41")), Classification::MsMpeg4V1);
        assert_eq!(classify(&bytes, Some(b"MP42")), Classification::MsMpeg4V2);
        assert_eq!(classify(&bytes, Some(b"MP43")), Classification::MsMpeg4V3);
        assert_eq!(classify(&bytes, Some(b"DIV3")), Classification::MsMpeg4V3);
        // Lower-case FourCC still works via uppercase4 normalisation.
        assert_eq!(classify(&bytes, Some(b"div3")), Classification::MsMpeg4V3);
    }

    #[test]
    fn empty_buffer_is_unknown() {
        assert_eq!(classify(&[], None), Classification::Unknown);
        // With a FourCC hint an empty buffer is still Unknown — we
        // won't commit to a variant without any bytes to look at.
        assert_eq!(classify(&[], Some(b"DIV3")), Classification::Unknown);
    }

    #[test]
    fn short_buffer_without_start_code_is_ms_v3() {
        // 3 bytes, no start code possible (need 4). Treat as MS.
        assert_eq!(
            classify(&[0x85, 0x3F, 0xD4], None),
            Classification::MsMpeg4V3
        );
    }

    #[test]
    fn start_code_scan_is_bounded() {
        // A 4 MB buffer of zeros followed by a start code should NOT
        // be classified as MPEG-4 Part 2 — we cap the scan window at
        // 2 KB so big buffers remain cheap.
        let mut big = vec![0u8; 4 * 1024 * 1024];
        big.extend_from_slice(&[0x00, 0x00, 0x01, 0xB0]);
        // The early bytes are all zero, which doesn't trigger the
        // start-code match; the real start code is beyond the scan
        // window, so we return the MS default.
        assert_eq!(classify(&big, None), Classification::MsMpeg4V3);
    }

    #[test]
    fn classification_codec_id_round_trip() {
        assert_eq!(Classification::Mpeg4Part2.codec_id(), Some("mpeg4video"));
        assert_eq!(Classification::MsMpeg4V1.codec_id(), Some(CODEC_ID_V1));
        assert_eq!(Classification::MsMpeg4V2.codec_id(), Some(CODEC_ID_V2));
        assert_eq!(Classification::MsMpeg4V3.codec_id(), Some(CODEC_ID_V3));
        assert_eq!(Classification::Unknown.codec_id(), None);
    }

    #[test]
    fn registered_tag_claims_route_correctly() {
        use oxideav_core::CodecResolver;
        let mut reg = CodecRegistry::new();
        register(&mut reg);

        let ms_bytes = msmpeg4v3_picture_header();
        let iso_bytes = vos_header();

        // DIV3 with MS bytes → msmpeg4v3.
        let div3 = CodecTag::fourcc(b"DIV3");
        let ctx = ProbeContext::new(&div3).packet(&ms_bytes);
        assert_eq!(
            CodecResolver::resolve_tag(&reg, &ctx).map(|c| c.as_str().to_string()),
            Some(CODEC_ID_V3.to_string()),
        );
        // DIV3 with MPEG-4 Part 2 bytes → probe rejects, no fallback in
        // this crate, so resolution returns None (the mpeg4video claim
        // would come from the other crate, not tested here).
        let ctx = ProbeContext::new(&div3).packet(&iso_bytes);
        assert!(CodecResolver::resolve_tag(&reg, &ctx).is_none());
        // MP42 with MS bytes → msmpeg4v2.
        let mp42 = CodecTag::fourcc(b"MP42");
        let ctx = ProbeContext::new(&mp42).packet(&ms_bytes);
        assert_eq!(
            CodecResolver::resolve_tag(&reg, &ctx).map(|c| c.as_str().to_string()),
            Some(CODEC_ID_V2.to_string()),
        );
        // MP41 with MS bytes → msmpeg4v1.
        let mp41 = CodecTag::fourcc(b"MP41");
        let ctx = ProbeContext::new(&mp41).packet(&ms_bytes);
        assert_eq!(
            CodecResolver::resolve_tag(&reg, &ctx).map(|c| c.as_str().to_string()),
            Some(CODEC_ID_V1.to_string()),
        );
    }

    #[test]
    fn is_ms_mpeg4_predicate() {
        assert!(Classification::MsMpeg4V1.is_ms_mpeg4());
        assert!(Classification::MsMpeg4V2.is_ms_mpeg4());
        assert!(Classification::MsMpeg4V3.is_ms_mpeg4());
        assert!(!Classification::Mpeg4Part2.is_ms_mpeg4());
        assert!(!Classification::Unknown.is_ms_mpeg4());
    }
}
