//! End-to-end picture-level decode of the **MS-MPEG4 v3 intra-in-P**
//! macroblock path, exercised through the public
//! [`oxideav_msmpeg4::picture::decode_picture`] boundary on a *P-frame*.
//!
//! # Why this exists
//!
//! Every prior v3 P-frame picture-level test in this crate decodes an
//! **inter** macroblock (joint-MCBPCY index `>= 64`, the P-type half of
//! the 128-entry alphabet per patent US 6,563,953 Table 1 / `audit/02`
//! §1.4): all-skip copies, zero-MV copies, non-zero-MV shifts,
//! alternate-MV-table selection, and CBP-coded inter residuals. None of
//! them drives the **intra-in-P** branch of
//! [`oxideav_msmpeg4::picture::decode_picture`]'s P-frame macroblock
//! driver — the `decode.is_intra` arm reached when the joint-MCBPCY index
//! lands in the **low half** (idx `< 64`, the I-type / intra partition,
//! `MB-type = 3` per `docs/video/msmpeg4/spec/05-ab0-resolution.md` §3.2,
//! disasm site `1c2178bd`). That arm re-uses the shared intra pixel
//! pipeline (DC differential + spatial DC predictor + AC scan dispatch +
//! IDCT) the same way an I-frame macroblock does, but it is entered
//! through the P-frame driver after a leading **skip bit** and a
//! post-VLC **ac_pred bit**.
//!
//! This file closes that coverage gap. It builds a 16×16 single-MB v3
//! P-frame whose only macroblock is intra-in-P (DC-only, no AC), decodes
//! it against a deliberately *non-grey* reference, and asserts the output
//! is **intra-reconstructed content independent of the reference** — i.e.
//! the picture was rebuilt from the bitstream's DC differentials, not
//! copied from the reference as an inter/skip MB would be. A companion
//! assertion proves the two ac_pred scan choices (zigzag vs the
//! AC-prediction-direction scan) are both reachable through the P-frame
//! intra-in-P path.
//!
//! # Spec citations (clean-room, docs/ only)
//!
//! - `docs/video/msmpeg4/spec/05-ab0-resolution.md` §3.2 (v3 joint-MCBPCY
//!   decoder `0x1c21782f`: skip bit, joint VLC, `test bl,0x40` partition
//!   test, `MB-type = 3` intra-in-P branch at `1c2178bd`, post-VLC
//!   sign/ac-pred bit).
//! - `docs/video/msmpeg4/spec/01-bitstream-framing.md` §1.4 (v3 P-frame
//!   picture header: `picture_type`, `pquant`, `ac_chroma_sel`,
//!   `dc_size_sel`, `mv_table_sel`).
//! - `docs/video/msmpeg4/spec/07-remaining-opens.md` §5 (intra-DC
//!   direct-value VLC; DC-size category 0 → differential 0, no value bits).
//! - `docs/video/msmpeg4/spec/14-pri-ab-runtime-binding.md` §3.1
//!   (`ac_luma_sel == 0 → G3`, `ac_chroma_sel == 0 → G2`).

use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::header::PictureType;
use oxideav_msmpeg4::mcbpcy::decode_mcbpcy;
use oxideav_msmpeg4::picture::{decode_picture, Picture, PictureDims};
use oxideav_msmpeg4::tables_data::{
    INTRA_DC_CHROMA_SEL0_RAW, INTRA_DC_LUMA_SEL0_RAW, MCBPCY_V3_PARTITION,
};

/// Bit-pack helper: append each `(value, width)` MSB-first, then pad the
/// tail with 8 zero bytes so a trailing bit-reader peek does not starve.
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
        out.push(((acc << (8 - bits)) & 0xff) as u8);
    }
    out.extend_from_slice(&[0u8; 8]);
    out
}

/// Find the canonical-Huffman `(code, bit_length)` for a target
/// joint-MCBPCY symbol index by using the production [`decode_mcbpcy`]
/// as a black-box oracle (no dependency on the private canonical table
/// builder). Scans candidate `(code, bit_length)` pairs in canonical
/// order; the first that round-trips back to `target` and consumes
/// exactly `bit_length` bits is the encoding.
fn mcbpcy_code_for(target: u8) -> (u32, u32) {
    for bl in 1u32..=13 {
        for code in 0u32..(1u32 << bl) {
            let bytes = pack(&[(code, bl)]);
            let mut br = BitReader::new(&bytes);
            if let Ok(dec) = decode_mcbpcy(&mut br) {
                if dec.idx == target && br.bit_position() == bl as u64 {
                    return (code, bl);
                }
            }
        }
    }
    panic!("no MCBPCY code found for symbol {target}");
}

/// Build a 16×16 single-macroblock v3 **P-frame** whose only MB is
/// intra-in-P, DC-only (every block's CBP bit clear → no AC walk), with
/// every DC differential = 0 (DC-size category 0). The macroblock is
/// reached through the P-frame driver: leading skip = 0, then the
/// joint-MCBPCY low-half (intra) symbol, then the ac_pred bit.
///
/// `ac_pred` selects the post-VLC AC-prediction bit value. For a DC-only
/// block the scan choice does not change the (empty) AC output, so both
/// settings reconstruct the same DC-only picture — the point of varying
/// it is to prove the bit is consumed on the intra-in-P path without
/// desynchronising the rest of the stream.
fn build_v3_pframe_intra_in_p(ac_pred: u32) -> Vec<u8> {
    // --- Picture header (spec/01 §1.4) ---
    let mut fields: Vec<(u32, u32)> = vec![
        (1, 2), // picture_type = P
        (8, 5), // pquant = 8
        (0, 1), // ac_chroma_sel = 0 (→ G2)
        (0, 1), // dc_size_sel = 0
        (0, 1), // mv_table_sel = 0 (default)
    ];

    // --- Macroblock layer (spec/05 §3.2) ---
    // P-frame: leading skip bit. skip = 0 reaches the joint-MCBPCY VLC.
    fields.push((0, 1));

    // Pick an intra (I-type) symbol in the LOW half (idx < 64) with
    // CBP = 0 → all six blocks DC-only. idx 0 has low-6 bits = 0 →
    // cbpy = 0, cbp_cb = 0, cbp_cr = 0.
    let mcbpcy_idx = 0u8;
    assert!(
        (mcbpcy_idx as usize) < MCBPCY_V3_PARTITION,
        "idx {mcbpcy_idx} must be in the intra/low half (< {MCBPCY_V3_PARTITION})"
    );
    let (mc_code, mc_bl) = mcbpcy_code_for(mcbpcy_idx);
    fields.push((mc_code, mc_bl));

    // Post-VLC ac_pred bit (spec/05 §3.2 site `1c2178c4..1c2178cf`).
    fields.push((ac_pred, 1));

    // --- Six DC-only blocks (CBP all clear) ---
    // Luma blocks 0..3: DC-size category 0 → differential 0, no value
    // bits, no sign bit (spec/07 §5.2 `test al,al; je zero_DC`).
    for _ in 0..4 {
        let (bl, code) = INTRA_DC_LUMA_SEL0_RAW[0];
        fields.push((code, bl));
    }
    // Chroma blocks 4,5: DC-size category 0.
    for _ in 0..2 {
        let (bl, code) = INTRA_DC_CHROMA_SEL0_RAW[0];
        fields.push((code, bl));
    }

    fields.push((0, 32)); // tail padding
    pack(&fields)
}

/// Build a non-grey reference picture so that "the output equals the
/// reference" (the inter/skip behaviour) is a meaningful negative —
/// distinct from "the output is intra-reconstructed grey/predicted
/// content" (the intra-in-P behaviour).
fn striped_reference(dims: PictureDims) -> Picture {
    let mut reference = Picture::alloc(dims, PictureType::I);
    for (i, px) in reference.y.iter_mut().enumerate() {
        *px = (i % 251) as u8;
    }
    for (i, px) in reference.cb.iter_mut().enumerate() {
        *px = (i % 199) as u8;
    }
    for (i, px) in reference.cr.iter_mut().enumerate() {
        *px = (i % 197) as u8;
    }
    reference
}

#[test]
fn v3_pframe_intra_in_p_mb_reconstructs_independent_of_reference() {
    let dims = PictureDims::new(16, 16).unwrap();
    let reference = striped_reference(dims);
    let bytes = build_v3_pframe_intra_in_p(0);

    let mut br = BitReader::new(&bytes);
    let pic = decode_picture(&mut br, dims, Some(&reference))
        .expect("v3 P-frame with a single intra-in-P MB must decode");

    assert_eq!(pic.picture_type, PictureType::P);

    // The intra-in-P MB rebuilds every block from the bitstream's DC
    // differentials (all 0 here) + the spatial DC predictor — it does NOT
    // copy the reference. With a deliberately striped reference, an
    // inter/skip MB would have reproduced that stripe pattern. An
    // intra-in-P MB produces a spatially-uniform DC-only block per plane
    // instead. Assert the decoded luma is NOT the reference stripe (the
    // load-bearing distinction between the intra-in-P branch and the
    // inter/skip branch).
    assert_ne!(
        pic.y, reference.y,
        "intra-in-P luma must be reconstructed (not a copy of the striped reference)",
    );

    // And the DC-only intra reconstruction is spatially uniform within
    // luma block 0 (top-left 8×8): every DC differential is 0, no AC, so
    // each block is a flat patch.
    let first = pic.y[0];
    for j in 0..8usize {
        for i in 0..8usize {
            assert_eq!(
                pic.y[j * pic.y_stride + i],
                first,
                "DC-only intra-in-P luma block 0 must be spatially uniform at ({i},{j})",
            );
        }
    }
}

#[test]
fn v3_pframe_intra_in_p_consumes_ac_pred_bit_both_polarities() {
    // The ac_pred bit is read on the intra-in-P path regardless of its
    // value (spec/05 §3.2). For a DC-only block the scan choice does not
    // alter the (empty) AC output, so both polarities must decode to the
    // SAME DC-only picture — proving the bit is consumed (the stream stays
    // aligned) and the intra-in-P branch handles either value.
    let dims = PictureDims::new(16, 16).unwrap();
    let reference = striped_reference(dims);

    let bytes0 = build_v3_pframe_intra_in_p(0);
    let mut br0 = BitReader::new(&bytes0);
    let pic0 = decode_picture(&mut br0, dims, Some(&reference))
        .expect("ac_pred = 0 intra-in-P P-frame decodes");

    let bytes1 = build_v3_pframe_intra_in_p(1);
    let mut br1 = BitReader::new(&bytes1);
    let pic1 = decode_picture(&mut br1, dims, Some(&reference))
        .expect("ac_pred = 1 intra-in-P P-frame decodes");

    assert_eq!(
        pic0.y, pic1.y,
        "DC-only intra-in-P luma must be identical under ac_pred 0 and 1 \
         (scan choice is a no-op with no AC coefficients)",
    );
    assert_eq!(
        pic0.cb, pic1.cb,
        "DC-only intra-in-P Cb must match across ac_pred"
    );
    assert_eq!(
        pic0.cr, pic1.cr,
        "DC-only intra-in-P Cr must match across ac_pred"
    );
}

#[test]
fn v3_pframe_intra_in_p_then_decodes_cleanly() {
    // Smoke: the whole picture decodes without consuming past the
    // bitstream and yields the documented P-frame type. (Guards against a
    // regression where the intra-in-P branch is unreachable and the
    // decoder errors or mis-routes the low-half symbol to the inter path,
    // which would try to read a MV VLC where the DC-size code sits.)
    let dims = PictureDims::new(16, 16).unwrap();
    let reference = striped_reference(dims);
    let bytes = build_v3_pframe_intra_in_p(0);
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture(&mut br, dims, Some(&reference)).expect("clean decode");
    assert_eq!(pic.width, 16);
    assert_eq!(pic.height, 16);
}

/// Look up the G3 primary-VLC `(code, bit_length)` for a regular
/// `(last, run, |level|)` token. Returns `None` if no such token exists
/// in the G3 alphabet (so the caller can pick an alternative).
fn g3_code_for(last: bool, run: u8, level: u16) -> Option<(u32, u32)> {
    use oxideav_msmpeg4::ac::{AcVlcTable, Symbol};
    let t = AcVlcTable::v3_intra_g3();
    for e in t.entries.iter() {
        if let Symbol::RunLevel {
            last: el,
            run: er,
            level: ev,
        } = e.value
        {
            if el == last && er == run && ev == level {
                return Some((e.code, e.bits as u32));
            }
        }
    }
    None
}

/// Build a 16×16 single-MB v3 P-frame whose only MB is intra-in-P with
/// **luma block 0 CBP-coded**, carrying a single real G3 AC token at the
/// requested `(run, level)` followed by a sub-B terminator. `ac_pred`
/// selects the post-VLC AC-prediction bit so the test can compare the
/// zigzag scan (ac_pred = 0) against the AC-prediction-direction scan
/// (ac_pred = 1).
///
/// The token sequence per block 0 is: DC-size-0 code (DC differential 0),
/// then a sub-A continuing token `(last=false, run, |level|)` + sign,
/// then a sub-B terminator `(last=true, 0, 1)` + sign. Blocks 1..5 are
/// uncoded (DC-size 0).
fn build_v3_pframe_intra_in_p_coded(ac_pred: u32, run: u8, level: u16) -> Vec<u8> {
    let mut fields: Vec<(u32, u32)> = vec![
        (1, 2), // P
        (8, 5), // pquant = 8
        (0, 1), // ac_chroma_sel = 0 (→ G2)
        (0, 1), // dc_size_sel = 0
        (0, 1), // mv_table_sel = 0
    ];

    fields.push((0, 1)); // skip = 0

    // Intra (low-half) symbol whose low-6 CBP pattern codes luma block 0
    // only: cbpy bit 3 set → cbpy = 0b1000 = 8 → pattern = 8 << 2 = 32.
    // 32 < 64, so it is still in the intra/low half.
    let mcbpcy_idx = 32u8;
    assert!((mcbpcy_idx as usize) < MCBPCY_V3_PARTITION);
    let (mc_code, mc_bl) = mcbpcy_code_for(mcbpcy_idx);
    fields.push((mc_code, mc_bl));
    fields.push((ac_pred, 1)); // post-VLC ac_pred bit

    // Block 0 (luma, CBP set): DC size 0 → differential 0.
    let (dc_bl, dc_code) = INTRA_DC_LUMA_SEL0_RAW[0];
    fields.push((dc_code, dc_bl));
    // sub-A continuing AC token at (run, level), then sign.
    let (ac0_code, ac0_bl) = g3_code_for(false, run, level)
        .unwrap_or_else(|| panic!("no G3 token for (last=false, run={run}, level={level})"));
    fields.push((ac0_code, ac0_bl));
    fields.push((0, 1)); // sign = positive
                         // sub-B terminator (last=true, 0, 1), then sign.
    let (ac1_code, ac1_bl) =
        g3_code_for(true, 0, 1).expect("G3 alphabet must carry a (last=true, 0, 1) terminator");
    fields.push((ac1_code, ac1_bl));
    fields.push((0, 1)); // sign = positive

    // Blocks 1..3 (luma uncoded): DC size 0 each.
    for _ in 0..3 {
        let (bl, code) = INTRA_DC_LUMA_SEL0_RAW[0];
        fields.push((code, bl));
    }
    // Blocks 4,5 (chroma uncoded): DC size 0 each.
    for _ in 0..2 {
        let (bl, code) = INTRA_DC_CHROMA_SEL0_RAW[0];
        fields.push((code, bl));
    }

    fields.push((0, 32)); // tail padding
    pack(&fields)
}

#[test]
fn v3_pframe_intra_in_p_coded_block_consults_ac_pred_scan() {
    use oxideav_msmpeg4::scan::{ALTERNATE_HORIZONTAL, ZIGZAG};

    // For the picture's first block (0,0) every DC neighbour is absent →
    // the §7.4.3 gradient test compares |A−D| = 0 vs |D−B| = 0, the
    // `<` is false, so the predictor is FROM-TOP → the ac_pred-on scan is
    // ALTERNATE_HORIZONTAL (dc_pred::PredDir::FromTop → Scan::AlternateHorizontal).
    //
    // Pick a (run, level) AC token whose single non-DC coefficient lands
    // at a scan position where ZIGZAG and ALTERNATE_HORIZONTAL diverge.
    // A run=1 token sits at scan position 2 (DC at 0, the run skips one
    // coefficient): ZIGZAG[2] = 8 (natural row 1, col 0) but
    // ALTERNATE_HORIZONTAL[2] = 2 (natural row 0, col 2). So the same
    // coefficient lands at a different natural 8×8 position under the two
    // scans — and after the IDCT the two reconstructions differ.
    assert_ne!(
        ZIGZAG[2], ALTERNATE_HORIZONTAL[2],
        "test premise: scan position 2 must differ between zigzag and \
         alternate-horizontal for this to distinguish the scans",
    );

    let dims = PictureDims::new(16, 16).unwrap();
    // No reference needed for an intra-in-P MB's reconstruction, but the
    // P-frame decode path requires one to satisfy the inter/skip branch
    // contract; supply a flat one.
    let reference = striped_reference(dims);

    let bytes0 = build_v3_pframe_intra_in_p_coded(0, 1, 1);
    let mut br0 = BitReader::new(&bytes0);
    let pic0 = decode_picture(&mut br0, dims, Some(&reference))
        .expect("ac_pred = 0 (zigzag) coded intra-in-P decodes");

    let bytes1 = build_v3_pframe_intra_in_p_coded(1, 1, 1);
    let mut br1 = BitReader::new(&bytes1);
    let pic1 = decode_picture(&mut br1, dims, Some(&reference))
        .expect("ac_pred = 1 (alternate scan) coded intra-in-P decodes");

    // Both reconstruct non-uniform luma block 0 (the AC token perturbs it
    // away from the DC-only flat patch).
    let block0_uniform = |p: &Picture| -> bool {
        let first = p.y[0];
        (0..8).all(|j| (0..8).all(|i| p.y[j * p.y_stride + i] == first))
    };
    assert!(
        !block0_uniform(&pic0),
        "zigzag coded intra-in-P block 0 must be non-uniform (AC token applied)",
    );
    assert!(
        !block0_uniform(&pic1),
        "alternate-scan coded intra-in-P block 0 must be non-uniform (AC token applied)",
    );

    // The load-bearing assertion: because the AC coefficient lands at a
    // different natural position under the two scans, the two
    // reconstructions of luma block 0 must DIFFER — proving the
    // intra-in-P path genuinely routes the ac_pred bit into the scan
    // selection (a regression that ignored ac_pred and always used zigzag
    // would make these identical).
    let block0_differs = (0..8)
        .any(|j| (0..8).any(|i| pic0.y[j * pic0.y_stride + i] != pic1.y[j * pic1.y_stride + i]));
    assert!(
        block0_differs,
        "ac_pred = 0 (zigzag) and ac_pred = 1 (alternate-horizontal) must \
         reconstruct DIFFERENT luma block 0 content — the scan choice is \
         consulted on the v3 intra-in-P path (spec/05 §3.2 ac_pred bit \
         → picture.rs scan dispatch)",
    );
}

/// Build a 16×16 single-MB v3 I-frame, DC-only (one intra MB, CBP=0,
/// every DC differential 0). I-frames carry no skip bit and read the
/// extra `ac_luma_sel` header field (spec/01 §1.4).
fn build_v3_iframe_dc_only() -> Vec<u8> {
    let mut fields: Vec<(u32, u32)> = vec![
        (0, 2), // picture_type = I
        (8, 5), // pquant = 8
        (0, 1), // ac_chroma_sel = 0 (→ G2)
        (0, 1), // ac_luma_sel = 0 (→ G3) — I-frame only
        (0, 1), // dc_size_sel = 0
    ];
    // Round 420: the registered decoder treats its very first packet
    // as first-of-sequence, so the I-frame header carries the 5-bit
    // per-sequence extension (spec/99 §2.2).
    fields.push((0, 5));
    // One intra MB: no skip bit (I-frame), the 64-entry intra-CBPCY
    // symbol 0 (wire code `1`; CBP=0 after XOR resolution), ac_pred,
    // six DC-only blocks.
    fields.push((0b1, 1));
    fields.push((0, 1)); // ac_pred = 0
    for _ in 0..4 {
        let (bl, code) = INTRA_DC_LUMA_SEL0_RAW[0];
        fields.push((code, bl));
    }
    for _ in 0..2 {
        let (bl, code) = INTRA_DC_CHROMA_SEL0_RAW[0];
        fields.push((code, bl));
    }
    fields.push((0, 32));
    pack(&fields)
}

#[test]
fn registered_decoder_v3_iframe_then_intra_in_p_pframe_sequence() {
    // Drive an I-frame followed by an intra-in-P P-frame through the
    // **registered Decoder trait** (send_packet / receive_frame), not the
    // lower-level decode_picture entry. This exercises the parts the
    // decode_picture tests skip: the classifier dispatch, the `last_picture`
    // reference threading across packets, and the picture→Frame::Video
    // plane conversion. Deterministic (no ffmpeg dependency).
    use oxideav_core::time::TimeBase;
    use oxideav_core::{CodecId, CodecParameters, CodecRegistry, Frame, Packet};

    let mut reg = CodecRegistry::new();
    oxideav_msmpeg4::register_codecs(&mut reg);
    let mut params = CodecParameters::video(CodecId::new("msmpeg4v3"));
    params.width = Some(16);
    params.height = Some(16);
    let mut dec = reg.first_decoder(&params).expect("v3 decoder creation");

    // I-frame packet → flat-grey DC-only frame; also becomes last_picture.
    let i_bytes = build_v3_iframe_dc_only();
    let i_pkt = Packet::new(0, TimeBase::new(1, 25), i_bytes)
        .with_pts(0)
        .with_keyframe(true);
    dec.send_packet(&i_pkt).expect("v3 I-frame send_packet");
    let i_frame = dec.receive_frame().expect("I-frame output");
    let i_vf = match i_frame {
        Frame::Video(vf) => vf,
        other => panic!("expected Frame::Video for I-frame, got {other:?}"),
    };
    assert_eq!(i_vf.planes.len(), 3, "YUV420 → 3 planes");
    // DC-only intra I-frame → flat grey 128.
    for row in 0..16usize {
        for col in 0..16usize {
            assert_eq!(
                i_vf.planes[0].data[row * i_vf.planes[0].stride + col],
                128,
                "v3 I-frame DC-only luma ({row}, {col}) must be flat grey 128",
            );
        }
    }

    // P-frame packet with a single intra-in-P MB. It is decoded against
    // the retained I-frame reference but, being intra-in-P, reconstructs
    // independently → flat grey 128 again (DC-only). The point is that the
    // registered decoder threaded the reference and the intra-in-P branch
    // ran through the full public path.
    let p_bytes = build_v3_pframe_intra_in_p(0);
    let p_pkt = Packet::new(0, TimeBase::new(1, 25), p_bytes).with_pts(1);
    dec.send_packet(&p_pkt)
        .expect("v3 intra-in-P P-frame send_packet (reference threaded from the I-frame)");
    let p_frame = dec.receive_frame().expect("P-frame output");
    let p_vf = match p_frame {
        Frame::Video(vf) => vf,
        other => panic!("expected Frame::Video for P-frame, got {other:?}"),
    };
    assert_eq!(p_vf.planes.len(), 3);
    for row in 0..16usize {
        for col in 0..16usize {
            assert_eq!(
                p_vf.planes[0].data[row * p_vf.planes[0].stride + col],
                128,
                "v3 intra-in-P P-frame DC-only luma ({row}, {col}) must be flat grey 128",
            );
        }
    }
}
