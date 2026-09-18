//! The spec/19 §1 integer IDCT kernels over arbitrary 32-bit
//! coefficient blocks: the lane model must stay panic-free (its int16
//! input truncation bounds every intermediate) and agree with the
//! exact scalar form on every in-range block.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_msmpeg4::idct::{idct8x8_int, idct8x8_scalar, idct8x8_to_pel};

fuzz_target!(|data: &[u8]| {
    if data.len() < 256 {
        return;
    }
    let mut c = [0i32; 64];
    for (i, v) in c.iter_mut().enumerate() {
        *v = i32::from_le_bytes([
            data[i * 4],
            data[i * 4 + 1],
            data[i * 4 + 2],
            data[i * 4 + 3],
        ]);
    }
    let mut a = [0i32; 64];
    let mut b = [0i32; 64];
    let mut p = [0i32; 64];
    idct8x8_int(&c, &mut a);
    idct8x8_scalar(&c, &mut b);
    idct8x8_to_pel(&c, &mut p);
    assert_eq!(a, p, "idct8x8_to_pel is the lane model");
    // In-range blocks (|coeff| <= 64, the bound the unit tests use)
    // must reproduce the scalar form exactly.
    if c.iter().all(|&v| (-64..=64).contains(&v)) {
        assert_eq!(a, b, "lane model diverged from the scalar form in range");
    }
});
