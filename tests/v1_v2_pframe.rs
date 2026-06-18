//! Integration tests for the v1/v2 P-frame pixel pipeline (round 285).
//!
//! Per the staged clean-room trace, a v1/v2 P-frame is a single slice
//! of raster-order MBs (spec/01 §3), each opening with the H.263-style
//! 1-bit skip flag and the separate MCBPC + CBPY VLCs (spec/07 §1-§2).
//! Plain inter MBs (v1 `mb_type == 0`, v2 `quotient == 0`) then read
//! two per-component MV codes against the shared 65-entry table at VMA
//! `0x1c24f930` (spec/07 §3) with the same §7.6.5 1-MV median
//! predictor as v3 (same helper `0x1c217c8c` per spec/07 §3.5), apply
//! half-pel MC, and add the G4 inter AC residual for every CBP-coded
//! block — the v1/v2 fallthrough at `0x1c212917` binds the inter
//! descriptor to G4 (spec/14 §3.1).
//!
//! Round 335 wires the v1 inter MB sub-types from the re-extracted
//! `spec/16` §3.1 + `region_053140_mbtype.csv`: the P-frame MB-type
//! (= mcbpc >> 2) selects the motion mode with MV-counts {1, 1, 4, 0,
//! 0}. MB-type 0 (INTER) and MB-type 1 (INTER+Q) are both 1-MV (the v1
//! MCBPCY body reads no quantiser-delta bit per spec/07 §1.4), and
//! MB-type 2 (INTER4V) loops the per-component MV decoder 4× over the
//! Figure 6-8 8x8 blocks, with the chroma MV derived per §7.6.3.4.
//!
//! Round 339 unblocks the v1/v2 **intra** path (I-frames + intra-in-P
//! MBs). spec/16 §2 (Extractor 07) established that v1/v2 decode the
//! intra DC differential through the classic H.263 size+value scheme
//! using the binary's own luma/chroma size tables (`region_054{2,3}c0`),
//! NOT the v3 `[esi+0x8bc]` `dc_size_sel` selector — so the previous
//! gate (which cited that selector's untraced default) is dissolved.
//! The spatial DC predictor and AC kernel are shared with v3; luma AC
//! binds G5, chroma G4 (spec/14 §3.2).

use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::header::PictureType;
use oxideav_msmpeg4::picture::{decode_picture_v1v2, MsV1V2Version, Picture, PictureDims};
use oxideav_msmpeg4::tables_data::{
    CBPY_V1_V2_RAW, DC_SIZE_CHROMA_V1V2_RAW, DC_SIZE_LUMA_V1V2_RAW, MCBPC_V1_RAW, MCBPC_V2_RAW,
    MV_V1_V2_RAW,
};

fn pack(fields: &[(u32, u32)]) -> Vec<u8> {
    let mut out: Vec<u8> = Vec::new();
    let mut acc: u64 = 0;
    let mut bits: u32 = 0;
    for (v, w) in fields {
        let mask = if *w == 32 { u32::MAX } else { (1u32 << w) - 1 };
        acc = (acc << w) | ((*v & mask) as u64);
        bits += w;
        while bits >= 8 {
            let shift = bits - 8;
            out.push(((acc >> shift) & 0xff) as u8);
            acc &= (1u64 << shift) - 1;
            bits -= 8;
        }
    }
    if bits > 0 {
        let shift = 8 - bits;
        out.push(((acc << shift) & 0xff) as u8);
    }
    out.extend_from_slice(&[0u8; 8]);
    out
}

/// Look up the `(code, bit_length)` pack-field for a symbol in one of
/// the `(sym, bl, code)` raw triple tables.
fn code_for(table: &[(u8, u8, u32)], sym: u8) -> (u32, u32) {
    let &(_, bl, code) = table
        .iter()
        .find(|&&(s, _, _)| s == sym)
        .unwrap_or_else(|| panic!("sym {sym} not in raw triple table"));
    (code, bl as u32)
}

/// v1 P-frame picture-header fields: 37-bit opaque preamble + 2-bit
/// type (P = 1) + 5-bit quant + 1-bit UMV (spec/01 §1.4).
fn v1_pframe_header(quant: u32) -> Vec<(u32, u32)> {
    vec![(0, 32), (0, 5), (1, 2), (quant, 5), (0, 1)]
}

/// v2 P-frame picture-header fields: 2-bit type + 5-bit quant.
fn v2_pframe_header(quant: u32) -> Vec<(u32, u32)> {
    vec![(1, 2), (quant, 5)]
}

/// A 16x16 reference picture with a per-pel gradient so MC shifts are
/// observable.
fn gradient_reference() -> (PictureDims, Picture) {
    let dims = PictureDims::new(16, 16).unwrap();
    let mut reference = Picture::alloc(dims, PictureType::I);
    for y in 0..16 {
        for x in 0..16 {
            reference.y[y * reference.y_stride + x] = ((x * 13 + y * 7) % 251) as u8;
        }
    }
    for y in 0..8 {
        for x in 0..8 {
            reference.cb[y * reference.c_stride + x] = (60 + x * 3 + y) as u8;
            reference.cr[y * reference.c_stride + x] = (200 - x - y * 2) as u8;
        }
    }
    (dims, reference)
}

#[test]
fn v1_pframe_all_skip_copies_reference() {
    // 32x32 → 4 MBs, all skipped (one `1` bit each).
    let dims = PictureDims::new(32, 32).unwrap();
    let mut reference = Picture::alloc(dims, PictureType::I);
    for (i, p) in reference.y.iter_mut().enumerate() {
        *p = (i % 247) as u8;
    }
    let mut fields = v1_pframe_header(8);
    for _ in 0..4 {
        fields.push((1, 1)); // skip
    }
    let bytes = pack(&fields);
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture_v1v2(&mut br, dims, MsV1V2Version::V1, Some(&reference))
        .expect("v1 all-skip P-frame decode");
    assert_eq!(pic.picture_type, PictureType::P);
    assert_eq!(pic.y, reference.y);
    assert_eq!(pic.cb, reference.cb);
    assert_eq!(pic.cr, reference.cr);
}

#[test]
fn v2_pframe_all_skip_copies_reference() {
    let (dims, reference) = gradient_reference();
    let mut fields = v2_pframe_header(8);
    fields.push((1, 1)); // single MB, skipped
    let bytes = pack(&fields);
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture_v1v2(&mut br, dims, MsV1V2Version::V2, Some(&reference))
        .expect("v2 all-skip P-frame decode");
    assert_eq!(pic.y, reference.y);
    assert_eq!(pic.cb, reference.cb);
    assert_eq!(pic.cr, reference.cr);
}

#[test]
fn v1_pframe_inter_mb_zero_mv_copies_reference() {
    let (dims, reference) = gradient_reference();
    // skip=0, MCBPC sym 0 (mb_type 0, CBPC 0), CBPY raw sym 15
    // (post-wrap pattern 15 - 15 = 0 → no coded luma), MV sym 32
    // twice (MVD 0 per component; sym 32 is the bias point).
    let (mcbpc_code, mcbpc_bl) = code_for(MCBPC_V1_RAW, 0);
    let (cbpy_code, cbpy_bl) = code_for(CBPY_V1_V2_RAW, 15);
    let (mv0_code, mv0_bl) = code_for(MV_V1_V2_RAW, 32);
    let mut fields = v1_pframe_header(8);
    fields.push((0, 1)); // skip = 0
    fields.push((mcbpc_code, mcbpc_bl));
    fields.push((cbpy_code, cbpy_bl));
    fields.push((mv0_code, mv0_bl)); // MVDx = 0
    fields.push((mv0_code, mv0_bl)); // MVDy = 0
    let bytes = pack(&fields);
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture_v1v2(&mut br, dims, MsV1V2Version::V1, Some(&reference))
        .expect("v1 inter MB with MV=(0,0) decode");
    assert_eq!(pic.picture_type, PictureType::P);
    assert_eq!(pic.y, reference.y, "MV=(0,0), CBP=0 must be a pure copy");
    assert_eq!(pic.cb, reference.cb);
    assert_eq!(pic.cr, reference.cr);
}

#[test]
fn v1_pframe_inter_mb_nonzero_mv_shifts_reference() {
    let (dims, reference) = gradient_reference();
    // MVDx = sym 34 - 32 = +2 half-pel = +1 full pel; MVDy = 0. The
    // first MB has no valid neighbours, so the §7.6.5 rule-4 predictor
    // is (0, 0) and the final MV is exactly the MVD.
    let (mcbpc_code, mcbpc_bl) = code_for(MCBPC_V1_RAW, 0);
    let (cbpy_code, cbpy_bl) = code_for(CBPY_V1_V2_RAW, 15);
    let (mvx_code, mvx_bl) = code_for(MV_V1_V2_RAW, 34);
    let (mv0_code, mv0_bl) = code_for(MV_V1_V2_RAW, 32);
    let mut fields = v1_pframe_header(8);
    fields.push((0, 1)); // skip = 0
    fields.push((mcbpc_code, mcbpc_bl));
    fields.push((cbpy_code, cbpy_bl));
    fields.push((mvx_code, mvx_bl)); // MVDx = +2 (half-pel)
    fields.push((mv0_code, mv0_bl)); // MVDy = 0
    let bytes = pack(&fields);
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture_v1v2(&mut br, dims, MsV1V2Version::V1, Some(&reference))
        .expect("v1 inter MB with MV=(+2,0) decode");
    // Each output luma pel samples the reference one column to the
    // right (edge-clamped on the last column).
    for row in 0..16usize {
        for col in 0..16usize {
            let src_col = (col + 1).min(15);
            assert_eq!(
                pic.y[row * pic.y_stride + col],
                reference.y[row * reference.y_stride + src_col],
                "luma ({row}, {col}) must sample reference column {src_col}",
            );
        }
    }
}

#[test]
fn v1_pframe_inter_mb_applies_g4_residual() {
    // Luma block 0 CBP-coded: post-wrap CBPY = 0b1000 = 8 → raw sym
    // 15 - 8 = 7. The residual is a single shortest sub-class-B
    // (last=1) G4 terminator + positive sign, on a flat-128 reference.
    let dims = PictureDims::new(16, 16).unwrap();
    let reference = Picture::alloc(dims, PictureType::I); // flat 128

    let g4 = oxideav_msmpeg4::ac::AcVlcTable::g4_inter();
    let term = g4
        .entries
        .iter()
        .filter(|e| {
            matches!(
                e.value,
                oxideav_msmpeg4::ac::Symbol::RunLevel {
                    last: true,
                    run: _,
                    level,
                } if level != 0
            )
        })
        .min_by_key(|e| e.bits)
        .expect("G4 sub-class-B terminator with non-zero level");

    let (mcbpc_code, mcbpc_bl) = code_for(MCBPC_V1_RAW, 0);
    let (cbpy_code, cbpy_bl) = code_for(CBPY_V1_V2_RAW, 7); // post-wrap 8
    let (mv0_code, mv0_bl) = code_for(MV_V1_V2_RAW, 32);
    let mut fields = v1_pframe_header(8);
    fields.push((0, 1)); // skip = 0
    fields.push((mcbpc_code, mcbpc_bl));
    fields.push((cbpy_code, cbpy_bl));
    fields.push((mv0_code, mv0_bl)); // MVDx = 0
    fields.push((mv0_code, mv0_bl)); // MVDy = 0
    fields.push((term.code, term.bits as u32)); // block 0 residual
    fields.push((0, 1)); // sign = positive
    let bytes = pack(&fields);
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture_v1v2(&mut br, dims, MsV1V2Version::V1, Some(&reference))
        .expect("v1 inter MB with G4 residual decode");

    let mut block0_changed = false;
    for j in 0..8usize {
        for i in 0..8usize {
            if pic.y[j * pic.y_stride + i] != 128 {
                block0_changed = true;
            }
        }
    }
    assert!(block0_changed, "G4 residual must modify luma block 0");
    // Everything outside block 0 stays the MC copy (128).
    for j in 0..16usize {
        for i in 0..16usize {
            if j < 8 && i < 8 {
                continue;
            }
            assert_eq!(
                pic.y[j * pic.y_stride + i],
                128,
                "luma ({j}, {i}) outside block 0 must stay the MC copy",
            );
        }
    }
    assert!(pic.cb.iter().all(|&p| p == 128));
    assert!(pic.cr.iter().all(|&p| p == 128));
}

#[test]
fn v2_pframe_inter_mb_zero_mv_copies_reference() {
    let (dims, reference) = gradient_reference();
    // v2: skip=0, MCBPC sym 0 (quotient 0 → inter, CBPC 0, wrap
    // applied since remainder != 3), CBPY raw 15 → post-wrap 0,
    // MV = (0, 0).
    let (mcbpc_code, mcbpc_bl) = code_for(MCBPC_V2_RAW, 0);
    let (cbpy_code, cbpy_bl) = code_for(CBPY_V1_V2_RAW, 15);
    let (mv0_code, mv0_bl) = code_for(MV_V1_V2_RAW, 32);
    let mut fields = v2_pframe_header(8);
    fields.push((0, 1)); // skip = 0
    fields.push((mcbpc_code, mcbpc_bl));
    fields.push((cbpy_code, cbpy_bl));
    fields.push((mv0_code, mv0_bl));
    fields.push((mv0_code, mv0_bl));
    let bytes = pack(&fields);
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture_v1v2(&mut br, dims, MsV1V2Version::V2, Some(&reference))
        .expect("v2 inter MB with MV=(0,0) decode");
    assert_eq!(pic.y, reference.y);
    assert_eq!(pic.cb, reference.cb);
    assert_eq!(pic.cr, reference.cr);
}

#[test]
fn v1_iframe_intra_dc_only_decodes_flat_grey() {
    // spec/16 §2 (Extractor 07): the v1/v2 I-frame intra path is the
    // H.263 size+value DC scheme with the binary's own luma/chroma size
    // tables — NOT the v3 `dc_size_sel` selector. A 16x16 I-frame with
    // one intra MB whose DC-size is 0 for every block reconstructs to a
    // flat grey MB (DC differential 0 + spatial predictor 1024 → DC =
    // 1024 → IDCT → 128 per pel).
    let dims = PictureDims::new(16, 16).unwrap();
    // v1 I-frame header: 37-bit preamble + type 0 (I) + quant 8.
    let mut fields: Vec<(u32, u32)> = vec![(0, 32), (0, 5), (0, 2), (8, 5)];
    // MB header: skip bit 0 (v1 reads the COD bit even on I-frames),
    // MCBPC sym 12 (mb_type 3 = INTRA, CBPC 0), CBPY raw 15 → post-wrap
    // 0 (no coded luma).
    let (mcbpc_code, mcbpc_bl) = code_for(MCBPC_V1_RAW, 12);
    let (cbpy_code, cbpy_bl) = code_for(CBPY_V1_V2_RAW, 15);
    fields.push((0, 1)); // skip / COD = 0
    fields.push((mcbpc_code, mcbpc_bl));
    fields.push((cbpy_code, cbpy_bl));
    // 6 DC-size-0 codewords (4 luma + 2 chroma), no value bits.
    let (luma0_code, luma0_bl) = code_for(DC_SIZE_LUMA_V1V2_RAW, 0);
    let (chroma0_code, chroma0_bl) = code_for(DC_SIZE_CHROMA_V1V2_RAW, 0);
    for _ in 0..4 {
        fields.push((luma0_code, luma0_bl));
    }
    for _ in 0..2 {
        fields.push((chroma0_code, chroma0_bl));
    }
    let bytes = pack(&fields);
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture_v1v2(&mut br, dims, MsV1V2Version::V1, None)
        .expect("v1 I-frame intra DC-only decode");
    assert_eq!(pic.picture_type, PictureType::I);
    // DC = predictor (1024) + 0 → 1024; intra IDCT yields flat 128.
    assert!(
        pic.y.iter().all(|&p| p == 128),
        "v1 I-frame DC-only luma must reconstruct flat grey 128"
    );
    assert!(pic.cb.iter().all(|&p| p == 128));
    assert!(pic.cr.iter().all(|&p| p == 128));
}

#[test]
fn v1_iframe_intra_nonzero_dc_shifts_luma() {
    // A non-zero DC differential must shift the reconstructed luma away
    // from flat grey. spec/16 §2.1: size category 1 + value bit `1` →
    // differential +1 (no negative fixup since 1 >= 2^(1-1) = 1). With
    // the q=8 luma DC scaler the reconstructed DC moves the whole 8x8
    // block off 128.
    let dims = PictureDims::new(16, 16).unwrap();
    let mut fields: Vec<(u32, u32)> = vec![(0, 32), (0, 5), (0, 2), (8, 5)];
    let (mcbpc_code, mcbpc_bl) = code_for(MCBPC_V1_RAW, 12); // INTRA
    let (cbpy_code, cbpy_bl) = code_for(CBPY_V1_V2_RAW, 15); // post-wrap 0
    fields.push((0, 1));
    fields.push((mcbpc_code, mcbpc_bl));
    fields.push((cbpy_code, cbpy_bl));
    // Luma block 0: size category 1, value bit 1 → diff +1.
    let (luma1_code, luma1_bl) = code_for(DC_SIZE_LUMA_V1V2_RAW, 1);
    fields.push((luma1_code, luma1_bl));
    fields.push((1, 1)); // 1 value bit = `1` → +1
                         // Luma blocks 1..=3 + 2 chroma: size 0.
    let (luma0_code, luma0_bl) = code_for(DC_SIZE_LUMA_V1V2_RAW, 0);
    let (chroma0_code, chroma0_bl) = code_for(DC_SIZE_CHROMA_V1V2_RAW, 0);
    for _ in 0..3 {
        fields.push((luma0_code, luma0_bl));
    }
    for _ in 0..2 {
        fields.push((chroma0_code, chroma0_bl));
    }
    let bytes = pack(&fields);
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture_v1v2(&mut br, dims, MsV1V2Version::V1, None)
        .expect("v1 I-frame intra non-zero DC decode");
    // Luma block 0 (top-left 8x8) must have shifted off flat grey.
    let mut block0_changed = false;
    for j in 0..8usize {
        for i in 0..8usize {
            if pic.y[j * pic.y_stride + i] != 128 {
                block0_changed = true;
            }
        }
    }
    assert!(
        block0_changed,
        "non-zero DC differential must shift luma block 0 off 128"
    );
}

#[test]
fn v2_pframe_intra_in_p_decodes() {
    // spec/16 §2 dissolves the old [esi+0x8bc] gate: the v1/v2 intra-in-P
    // path decodes through the same size-category DC scheme as I-frames.
    // v2 MCBPC sym 4 → quotient 1 → intra-in-P (mb_type 3). The decoder
    // reads ac_pred + CBPY, then the 6 intra blocks.
    let (dims, reference) = gradient_reference();
    let (mcbpc_code, mcbpc_bl) = code_for(MCBPC_V2_RAW, 4);
    let (cbpy_code, cbpy_bl) = code_for(CBPY_V1_V2_RAW, 15); // post-wrap 0
    let (luma0_code, luma0_bl) = code_for(DC_SIZE_LUMA_V1V2_RAW, 0);
    let (chroma0_code, chroma0_bl) = code_for(DC_SIZE_CHROMA_V1V2_RAW, 0);
    let mut fields = v2_pframe_header(8);
    fields.push((0, 1)); // skip = 0
    fields.push((mcbpc_code, mcbpc_bl));
    fields.push((0, 1)); // ac_pred = 0 (zigzag scan)
    fields.push((cbpy_code, cbpy_bl));
    for _ in 0..4 {
        fields.push((luma0_code, luma0_bl));
    }
    for _ in 0..2 {
        fields.push((chroma0_code, chroma0_bl));
    }
    let bytes = pack(&fields);
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture_v1v2(&mut br, dims, MsV1V2Version::V2, Some(&reference))
        .expect("v2 intra-in-P MB decode");
    assert_eq!(pic.picture_type, PictureType::P);
    // The single intra MB reconstructs DC-only flat grey (DC diff 0 +
    // predictor 1024 → 128), overwriting the gradient reference.
    assert!(
        pic.y.iter().all(|&p| p == 128),
        "v2 intra-in-P DC-only luma must reconstruct flat grey 128"
    );
}

#[test]
fn v1_pframe_inter_plus_q_subtype_one_mv_zero_copies_reference() {
    // spec/16 §3.1: MB-type 1 (INTER+Q) is a 1-MV inter MB decoded
    // exactly like MB-type 0 (the v1 MCBPCY body reads no quantiser
    // delta per spec/07 §1.4). MCBPC sym 4 → mb_type = 1, CBPC = 0.
    let (dims, reference) = gradient_reference();
    let (mcbpc_code, mcbpc_bl) = code_for(MCBPC_V1_RAW, 4);
    let (cbpy_code, cbpy_bl) = code_for(CBPY_V1_V2_RAW, 15); // post-wrap 0
    let (mv0_code, mv0_bl) = code_for(MV_V1_V2_RAW, 32); // MVD 0
    let mut fields = v1_pframe_header(8);
    fields.push((0, 1)); // skip = 0
    fields.push((mcbpc_code, mcbpc_bl));
    fields.push((cbpy_code, cbpy_bl));
    fields.push((mv0_code, mv0_bl)); // MVDx = 0
    fields.push((mv0_code, mv0_bl)); // MVDy = 0
    let bytes = pack(&fields);
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture_v1v2(&mut br, dims, MsV1V2Version::V1, Some(&reference))
        .expect("v1 INTER+Q (mb_type 1) MV=(0,0) decode");
    assert_eq!(pic.picture_type, PictureType::P);
    assert_eq!(pic.y, reference.y, "INTER+Q MV=(0,0), CBP=0 must be a copy");
    assert_eq!(pic.cb, reference.cb);
    assert_eq!(pic.cr, reference.cr);
}

#[test]
fn v1_pframe_inter4v_zero_mvs_copies_reference() {
    // spec/16 §3.1: MB-type 2 (INTER4V) loops the per-component MV
    // decoder 4×, one MV per Figure 6-8 8x8 block. MCBPC sym 8 →
    // mb_type = 2, CBPC = 0. Four (0,0) MVs ⇒ every block copies the
    // reference verbatim; CBPY post-wrap 0 ⇒ no residual.
    let (dims, reference) = gradient_reference();
    let (mcbpc_code, mcbpc_bl) = code_for(MCBPC_V1_RAW, 8);
    let (cbpy_code, cbpy_bl) = code_for(CBPY_V1_V2_RAW, 15); // post-wrap 0
    let (mv0_code, mv0_bl) = code_for(MV_V1_V2_RAW, 32); // MVD 0
    let mut fields = v1_pframe_header(8);
    fields.push((0, 1)); // skip = 0
    fields.push((mcbpc_code, mcbpc_bl));
    fields.push((cbpy_code, cbpy_bl));
    // 4 blocks × (MVDx, MVDy), all zero.
    for _ in 0..4 {
        fields.push((mv0_code, mv0_bl));
        fields.push((mv0_code, mv0_bl));
    }
    let bytes = pack(&fields);
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture_v1v2(&mut br, dims, MsV1V2Version::V1, Some(&reference))
        .expect("v1 INTER4V (mb_type 2) four-zero-MV decode");
    assert_eq!(pic.picture_type, PictureType::P);
    assert_eq!(pic.y, reference.y, "INTER4V four (0,0) MVs must be a copy");
    assert_eq!(pic.cb, reference.cb);
    assert_eq!(pic.cr, reference.cr);
}

#[test]
fn v1_pframe_inter4v_per_block_mvs_shift_independently() {
    // Each 8x8 luma block of an INTER4V MB uses its own MV. Give block
    // 1 (top-left) an MVDx of +2 half-pel (= +1 full pel). The first MB
    // has no neighbours, so block 1's §7.6.5 predictor is (0,0) and its
    // final MV is exactly the decoded MVD = (+2, 0). We assert the
    // top-left 8x8 sampled one column to the right (the within-MB
    // predictor threading for the later blocks is exercised by the
    // dedicated mv_pred unit tests, not re-asserted here).
    let (dims, reference) = gradient_reference();
    let (mcbpc_code, mcbpc_bl) = code_for(MCBPC_V1_RAW, 8); // mb_type 2
    let (cbpy_code, cbpy_bl) = code_for(CBPY_V1_V2_RAW, 15); // post-wrap 0
    let (mvx_code, mvx_bl) = code_for(MV_V1_V2_RAW, 34); // +2 half-pel
    let (mv0_code, mv0_bl) = code_for(MV_V1_V2_RAW, 32); // 0
    let mut fields = v1_pframe_header(8);
    fields.push((0, 1)); // skip = 0
    fields.push((mcbpc_code, mcbpc_bl));
    fields.push((cbpy_code, cbpy_bl));
    // Block 1: MVDx = +2, MVDy = 0.
    fields.push((mvx_code, mvx_bl));
    fields.push((mv0_code, mv0_bl));
    // Blocks 2..4: MVD 0 each.
    for _ in 0..3 {
        fields.push((mv0_code, mv0_bl));
        fields.push((mv0_code, mv0_bl));
    }
    let bytes = pack(&fields);
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture_v1v2(&mut br, dims, MsV1V2Version::V1, Some(&reference))
        .expect("v1 INTER4V per-block MV decode");
    // Top-left 8x8 block samples the reference one column to the right.
    for row in 0..8usize {
        for col in 0..8usize {
            let src_col = (col + 1).min(15);
            assert_eq!(
                pic.y[row * pic.y_stride + col],
                reference.y[row * reference.y_stride + src_col],
                "block-1 luma ({row}, {col}) must sample reference column {src_col}",
            );
        }
    }
}

#[test]
fn v1_pframe_without_reference_requires_reference() {
    let (dims, _) = gradient_reference();
    let bytes = pack(&v1_pframe_header(8));
    let mut br = BitReader::new(&bytes);
    let err = decode_picture_v1v2(&mut br, dims, MsV1V2Version::V1, None).unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("reference"),
        "P-frame without reference must name the missing reference; got: {msg}"
    );
}

/// End-to-end through the registered decoder: a v2 I-frame packet with
/// a single DC-only intra MB now decodes to a flat-grey frame (spec/16
/// §2 unblocks the v1/v2 intra path).
#[test]
fn send_packet_v2_iframe_decodes_intra() {
    use oxideav_core::time::TimeBase;
    use oxideav_core::{CodecId, CodecParameters, CodecRegistry, Packet};

    let mut reg = CodecRegistry::new();
    oxideav_msmpeg4::register_codecs(&mut reg);
    let mut params = CodecParameters::video(CodecId::new("msmpeg4v2"));
    params.width = Some(16);
    params.height = Some(16);
    let mut dec = reg.first_decoder(&params).expect("decoder creation");

    // v2 I-frame: type 0 + quant 8, then one intra MB (no skip bit on
    // I-frames): MCBPC sym 4 (quotient 1 → intra), ac_pred 0, CBPY raw
    // 15 → post-wrap 0, six DC-size-0 codewords.
    let (mcbpc_code, mcbpc_bl) = code_for(MCBPC_V2_RAW, 4);
    let (cbpy_code, cbpy_bl) = code_for(CBPY_V1_V2_RAW, 15);
    let (luma0_code, luma0_bl) = code_for(DC_SIZE_LUMA_V1V2_RAW, 0);
    let (chroma0_code, chroma0_bl) = code_for(DC_SIZE_CHROMA_V1V2_RAW, 0);
    let mut fields: Vec<(u32, u32)> = vec![(0, 2), (8, 5)];
    fields.push((mcbpc_code, mcbpc_bl));
    fields.push((0, 1)); // ac_pred = 0
    fields.push((cbpy_code, cbpy_bl));
    for _ in 0..4 {
        fields.push((luma0_code, luma0_bl));
    }
    for _ in 0..2 {
        fields.push((chroma0_code, chroma0_bl));
    }
    let bytes = pack(&fields);
    let pkt = Packet::new(0, TimeBase::new(1, 25), bytes)
        .with_pts(0)
        .with_keyframe(true);
    dec.send_packet(&pkt).expect("v2 I-frame send_packet");
    let frame = dec.receive_frame().expect("a decoded frame");
    let vf = match frame {
        oxideav_core::Frame::Video(vf) => vf,
        other => panic!("expected a video frame, got {other:?}"),
    };
    assert_eq!(vf.planes.len(), 3, "YUV420 → 3 planes");
    // DC-only intra reconstructs flat grey: the 16x16 luma plane (one MB)
    // must be all 128.
    for row in 0..16usize {
        for col in 0..16usize {
            assert_eq!(
                vf.planes[0].data[row * vf.planes[0].stride + col],
                128,
                "v2 I-frame DC-only luma ({row}, {col}) must be flat grey 128"
            );
        }
    }
}
