//! Encoder ↔ decoder consistency: arbitrary pel content is encoded as
//! a v3 I-frame and a following P-frame, and both must decode through
//! the production decoder (the encoder's own reconstruction path).
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::enc::{encode_iframe_v3, encode_pframe_v3, EncoderConfig};
use oxideav_msmpeg4::header::PictureType;
use oxideav_msmpeg4::picture::{decode_picture, Picture, PictureDims};

fn picture_from(data: &[u8], dims: PictureDims, kind: PictureType) -> Picture {
    let mut pic = Picture::alloc(dims, kind);
    let n = data.len().max(1);
    for (i, p) in pic.y.iter_mut().enumerate() {
        *p = data[i % n];
    }
    for (i, p) in pic.cb.iter_mut().enumerate() {
        *p = data[(i * 7 + 3) % n];
    }
    for (i, p) in pic.cr.iter_mut().enumerate() {
        *p = data[(i * 11 + 5) % n];
    }
    pic
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let w = 16 * (1 + (data[0] % 4) as u32);
    let h = 16 * (1 + (data[1] % 4) as u32);
    let dims = PictureDims::new(w, h).expect("dims in range");
    let config = EncoderConfig {
        quant: 1 + data[2] % 31,
        mv_search_range: data[3] % 4,
        ..Default::default()
    };
    let half = data.len() / 2;
    let input_i = picture_from(&data[4..half.max(5)], dims, PictureType::I);
    let bytes = encode_iframe_v3(&input_i, dims, &config).expect("I-frame encodes");
    let mut br = BitReader::new(&bytes);
    let recon_i = decode_picture(&mut br, dims, None).expect("own I-frame decodes");
    let input_p = picture_from(&data[half..], dims, PictureType::P);
    let bytes = encode_pframe_v3(&input_p, &recon_i, dims, &config).expect("P-frame encodes");
    let mut br = BitReader::new(&bytes);
    decode_picture(&mut br, dims, Some(&recon_i)).expect("own P-frame decodes");
});
