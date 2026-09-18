//! MS-MPEG4 v1 / v2 picture decoder over arbitrary bytes: an I-frame,
//! then up to three P-frames chained on it. Must never panic.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::picture::{decode_picture_v1v2, MsV1V2Version, Picture, PictureDims};

fuzz_target!(|data: &[u8]| {
    if data.len() < 4 {
        return;
    }
    let w = 16 * (1 + (data[0] % 8) as u32);
    let h = 16 * (1 + (data[1] % 8) as u32);
    let version = if data[2] & 1 == 0 {
        MsV1V2Version::V1
    } else {
        MsV1V2Version::V2
    };
    let dims = PictureDims::new(w, h).expect("dims in range");
    let mut br = BitReader::new(&data[3..]);
    let mut reference: Option<Picture> = None;
    for _ in 0..4 {
        match decode_picture_v1v2(&mut br, dims, version, reference.as_ref()) {
            Ok(pic) => reference = Some(pic),
            Err(_) => break,
        }
    }
});
