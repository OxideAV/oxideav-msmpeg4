//! MS-MPEG4 v3 picture decoder over arbitrary bytes: an I-frame, then
//! up to three P-frames chained on it. Must never panic (errors are
//! the expected outcome on hostile input).
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::picture::{decode_picture, Picture, PictureDims};

fuzz_target!(|data: &[u8]| {
    if data.len() < 3 {
        return;
    }
    let w = 16 * (1 + (data[0] % 8) as u32);
    let h = 16 * (1 + (data[1] % 8) as u32);
    let dims = PictureDims::new(w, h).expect("dims in range");
    let mut br = BitReader::new(&data[2..]);
    let mut reference: Option<Picture> = None;
    for _ in 0..4 {
        match decode_picture(&mut br, dims, reference.as_ref()) {
            Ok(pic) => reference = Some(pic),
            Err(_) => break,
        }
    }
});
