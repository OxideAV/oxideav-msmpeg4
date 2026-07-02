//! End-to-end tests for the v3 picture encoder: every produced
//! bitstream must decode through the production
//! `picture::decode_picture` entry point (the same path real streams
//! take), and the reconstruction must track the input within the
//! quantiser's loss bound.

use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::enc::{encode_iframe_v3, EncoderConfig};
use oxideav_msmpeg4::header::PictureType;
use oxideav_msmpeg4::picture::{decode_picture, Picture, PictureDims};

fn textured_input(dims: PictureDims) -> Picture {
    let mut pic = Picture::alloc(dims, PictureType::I);
    let h = pic.y.len() / pic.y_stride;
    for y in 0..h {
        for x in 0..pic.y_stride {
            // Mixed low/high-frequency content: blocks of gradient,
            // checkerboard and flat regions.
            let v = match ((x / 16) + (y / 16)) % 3 {
                0 => (x * 3 + y * 2) % 256,
                // Checkerboard amplitude kept at ±28 so the highest-
                // frequency DCT coefficient (~8× the amplitude) stays
                // inside the format's representable level range at
                // q=1 — the verbatim ESC tier carries an 8-bit signed
                // level, capping |level| at 127 (spec/04 §1.3 step 10).
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

fn mae(a: &[u8], b: &[u8]) -> f64 {
    let sum: u64 = a
        .iter()
        .zip(b.iter())
        .map(|(x, y)| (*x as i64 - *y as i64).unsigned_abs())
        .sum();
    sum as f64 / a.len() as f64
}

#[test]
fn v3_iframe_every_quant_produces_decodable_stream() {
    let dims = PictureDims::new(48, 48).unwrap();
    let input = textured_input(dims);
    for quant in 1..=31u8 {
        let config = EncoderConfig {
            quant,
            ..Default::default()
        };
        let bytes = encode_iframe_v3(&input, dims, &config)
            .unwrap_or_else(|e| panic!("encode failed at q={quant}: {e}"));
        let mut br = BitReader::new(&bytes);
        let out = decode_picture(&mut br, dims, None)
            .unwrap_or_else(|e| panic!("decode failed at q={quant}: {e}"));
        assert_eq!(out.picture_type, PictureType::I);
        // Loss grows with quant but must stay bounded by a few quant
        // steps even at the coarse end.
        let bound = 2.5 * quant as f64 + 3.0;
        let m = mae(&out.y, &input.y);
        assert!(m < bound, "q={quant}: luma MAE {m} exceeds {bound}");
    }
}

#[test]
fn v3_iframe_quality_improves_with_finer_quant() {
    let dims = PictureDims::new(64, 48).unwrap();
    let input = textured_input(dims);
    let mut prev_mae = f64::MAX;
    for &quant in &[31u8, 12, 4, 1] {
        let config = EncoderConfig {
            quant,
            ..Default::default()
        };
        let bytes = encode_iframe_v3(&input, dims, &config).unwrap();
        let mut br = BitReader::new(&bytes);
        let out = decode_picture(&mut br, dims, None).unwrap();
        let m = mae(&out.y, &input.y);
        assert!(
            m <= prev_mae + 0.05,
            "quality must not degrade as quant decreases (q={quant}: {m} vs {prev_mae})"
        );
        prev_mae = m;
    }
    assert!(
        prev_mae < 1.0,
        "q=1 reconstruction should be near-lossless (MAE {prev_mae})"
    );
}

#[test]
fn v3_iframe_non_mb_aligned_dims_round_trip() {
    // 40x24 → 3x2 MBs with the picture padded per Picture::alloc; the
    // decoder path uses the same MB-aligned layout.
    let dims = PictureDims::new(40, 24).unwrap();
    let input = textured_input(dims);
    let bytes = encode_iframe_v3(&input, dims, &EncoderConfig::default()).unwrap();
    let mut br = BitReader::new(&bytes);
    let out = decode_picture(&mut br, dims, None).unwrap();
    assert_eq!(out.width, 40);
    assert_eq!(out.height, 24);
    assert!(mae(&out.y, &input.y) < 15.0);
}

#[test]
fn v3_iframe_finer_quant_costs_more_bits() {
    let dims = PictureDims::new(48, 48).unwrap();
    let input = textured_input(dims);
    let fine = encode_iframe_v3(
        &input,
        dims,
        &EncoderConfig {
            quant: 1,
            ..Default::default()
        },
    )
    .unwrap();
    let coarse = encode_iframe_v3(
        &input,
        dims,
        &EncoderConfig {
            quant: 31,
            ..Default::default()
        },
    )
    .unwrap();
    assert!(
        fine.len() > coarse.len(),
        "q=1 stream ({}) should be larger than q=31 ({})",
        fine.len(),
        coarse.len()
    );
}
