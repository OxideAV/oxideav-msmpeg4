//! End-to-end tests for the v1/v2 picture encoders: every produced
//! bitstream must decode through the production
//! `picture::decode_picture_v1v2` entry point, and the reconstruction
//! must track the input within the quantiser's loss bound.

use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::enc::{encode_iframe_v1v2, encode_pframe_v1v2, EncoderConfig};
use oxideav_msmpeg4::header::PictureType;
use oxideav_msmpeg4::picture::{decode_picture_v1v2, MsV1V2Version, Picture, PictureDims};

const VERSIONS: [MsV1V2Version; 2] = [MsV1V2Version::V1, MsV1V2Version::V2];

fn textured_input(dims: PictureDims) -> Picture {
    let mut pic = Picture::alloc(dims, PictureType::I);
    let h = pic.y.len() / pic.y_stride;
    for y in 0..h {
        for x in 0..pic.y_stride {
            let v = match ((x / 16) + (y / 16)) % 3 {
                0 => (x * 3 + y * 2) % 256,
                1 => {
                    if (x + y) % 2 == 0 {
                        112
                    } else {
                        168
                    }
                }
                _ => 128,
            };
            pic.y[y * pic.y_stride + x] = v as u8;
        }
    }
    let ch = pic.cb.len() / pic.c_stride;
    for y in 0..ch {
        for x in 0..pic.c_stride {
            pic.cb[y * pic.c_stride + x] = ((x * 5 + 30) % 200 + 28) as u8;
            pic.cr[y * pic.c_stride + x] = ((y * 7 + 90) % 200 + 28) as u8;
        }
    }
    pic
}

fn translate(pic: &Picture, dx: usize, dy: usize) -> Picture {
    let mut out = pic.clone();
    let h = pic.y.len() / pic.y_stride;
    for y in 0..h {
        for x in 0..pic.y_stride {
            let sx = x.saturating_sub(dx).min(pic.y_stride - 1);
            let sy = y.saturating_sub(dy).min(h - 1);
            out.y[y * pic.y_stride + x] = pic.y[sy * pic.y_stride + sx];
        }
    }
    let ch = pic.cb.len() / pic.c_stride;
    for y in 0..ch {
        for x in 0..pic.c_stride {
            let sx = x.saturating_sub(dx / 2).min(pic.c_stride - 1);
            let sy = y.saturating_sub(dy / 2).min(ch - 1);
            out.cb[y * pic.c_stride + x] = pic.cb[sy * pic.c_stride + sx];
            out.cr[y * pic.c_stride + x] = pic.cr[sy * pic.c_stride + sx];
        }
    }
    out
}

fn mae(a: &[u8], b: &[u8]) -> f64 {
    let sum: u64 = a
        .iter()
        .zip(b.iter())
        .map(|(x, y)| (*x as i64 - *y as i64).unsigned_abs())
        .sum();
    sum as f64 / a.len() as f64
}

#[test]
fn v1_v2_iframe_flat_grey_round_trips_exactly() {
    let dims = PictureDims::new(32, 32).unwrap();
    let input = Picture::alloc(dims, PictureType::I); // all-128 planes
    for version in VERSIONS {
        let bytes = encode_iframe_v1v2(&input, dims, &EncoderConfig::default(), version)
            .unwrap_or_else(|e| panic!("{version:?} encode: {e}"));
        let mut br = BitReader::new(&bytes);
        let out = decode_picture_v1v2(&mut br, dims, version, None)
            .unwrap_or_else(|e| panic!("{version:?} decode: {e}"));
        assert_eq!(out.y, input.y, "{version:?}: flat grey luma");
        assert_eq!(out.cb, input.cb, "{version:?}");
        assert_eq!(out.cr, input.cr, "{version:?}");
    }
}

#[test]
fn v1_v2_iframe_every_quant_produces_decodable_stream() {
    let dims = PictureDims::new(48, 48).unwrap();
    let input = textured_input(dims);
    for version in VERSIONS {
        for quant in 1..=31u8 {
            let config = EncoderConfig {
                quant,
                ..Default::default()
            };
            let bytes = encode_iframe_v1v2(&input, dims, &config, version)
                .unwrap_or_else(|e| panic!("{version:?} q={quant} encode: {e}"));
            let mut br = BitReader::new(&bytes);
            let out = decode_picture_v1v2(&mut br, dims, version, None)
                .unwrap_or_else(|e| panic!("{version:?} q={quant} decode: {e}"));
            let bound = 2.5 * quant as f64 + 3.0;
            let m = mae(&out.y, &input.y);
            assert!(m < bound, "{version:?} q={quant}: luma MAE {m} > {bound}");
        }
    }
}

#[test]
fn v1_v2_pframe_identical_content_is_all_skip() {
    let dims = PictureDims::new(48, 32).unwrap();
    let reference = textured_input(dims);
    for version in VERSIONS {
        let bytes = encode_pframe_v1v2(
            &reference,
            &reference,
            dims,
            &EncoderConfig::default(),
            version,
        )
        .unwrap();
        // v1 header is 44 bits + 6 skip bits; v2 header is 7 bits +
        // 6 skip bits — both within a handful of bytes.
        assert!(
            bytes.len() <= 8,
            "{version:?}: all-skip P-frame should be tiny, got {} bytes",
            bytes.len()
        );
        let mut br = BitReader::new(&bytes);
        let out = decode_picture_v1v2(&mut br, dims, version, Some(&reference)).unwrap();
        assert_eq!(out.picture_type, PictureType::P);
        assert_eq!(out.y, reference.y, "{version:?}: skip copy");
        assert_eq!(out.cb, reference.cb);
        assert_eq!(out.cr, reference.cr);
    }
}

#[test]
fn v1_v2_i_p_p_sequence_tracks_moving_content() {
    let dims = PictureDims::new(64, 48).unwrap();
    let config = EncoderConfig {
        quant: 3,
        mv_search_range: 8,
    };
    for version in VERSIONS {
        let frame0 = textured_input(dims);
        let frame1 = translate(&frame0, 2, 1);
        let frame2 = translate(&frame0, 4, 2);

        let i_bytes = encode_iframe_v1v2(&frame0, dims, &config, version).unwrap();
        let mut br = BitReader::new(&i_bytes);
        let recon0 = decode_picture_v1v2(&mut br, dims, version, None).unwrap();

        let p1 = encode_pframe_v1v2(&frame1, &recon0, dims, &config, version).unwrap();
        let mut br = BitReader::new(&p1);
        let recon1 = decode_picture_v1v2(&mut br, dims, version, Some(&recon0)).unwrap();
        let m1 = mae(&recon1.y, &frame1.y);
        assert!(m1 < 3.0, "{version:?} P1 luma MAE {m1}");

        let p2 = encode_pframe_v1v2(&frame2, &recon1, dims, &config, version).unwrap();
        let mut br = BitReader::new(&p2);
        let recon2 = decode_picture_v1v2(&mut br, dims, version, Some(&recon1)).unwrap();
        let m2 = mae(&recon2.y, &frame2.y);
        assert!(m2 < 3.0, "{version:?} P2 luma MAE {m2}");

        assert!(
            p1.len() < i_bytes.len() / 2,
            "{version:?}: P-frame ({}) should be much smaller than I ({})",
            p1.len(),
            i_bytes.len()
        );
    }
}

#[test]
fn v1_v2_pframe_residual_only_change_zero_mv() {
    let dims = PictureDims::new(48, 32).unwrap();
    for version in VERSIONS {
        let base = textured_input(dims);
        let cfg = EncoderConfig {
            quant: 2,
            mv_search_range: 0,
        };
        let i_bytes = encode_iframe_v1v2(&base, dims, &cfg, version).unwrap();
        let mut br = BitReader::new(&i_bytes);
        let recon0 = decode_picture_v1v2(&mut br, dims, version, None).unwrap();

        let mut changed = recon0.clone();
        for j in 0..16 {
            for i in 0..16 {
                let off = (16 + j) * changed.y_stride + 16 + i;
                changed.y[off] = changed.y[off].saturating_add(40);
            }
        }
        let bytes = encode_pframe_v1v2(&changed, &recon0, dims, &cfg, version).unwrap();
        let mut br = BitReader::new(&bytes);
        let out = decode_picture_v1v2(&mut br, dims, version, Some(&recon0)).unwrap();
        let m = mae(&out.y, &changed.y);
        assert!(m < 0.6, "{version:?}: residual-only P luma MAE {m}");
        // Untouched MBs skip: bit-exact there.
        for j in 0..16 {
            for i in 0..16 {
                let off = j * out.y_stride + i;
                assert_eq!(out.y[off], recon0.y[off], "{version:?}: skip MB pel");
            }
        }
    }
}
