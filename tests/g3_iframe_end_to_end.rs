//! End-to-end I-frame decode through the **G3 intra-luma primary VLC**,
//! exercised through the public [`oxideav_msmpeg4::picture::decode_picture`]
//! boundary with the shipping default [`AcSelection::FromHeader`].
//!
//! # Why this exists
//!
//! Round 234 (2026-06-04) wired the G0..G3 packed-Huffman primary VLCs
//! (`AcVlcTable::v3_intra_g{0,1,2,3}` now return their real 169 / 186 /
//! 149 / 133-entry alphabets from the sources at file offsets
//! `0x57a30 / 0x57f80 / 0x58558 / 0x58a08`, per
//! `docs/video/msmpeg4/spec/11-walker-format-resolved.md` §5 row 1-4).
//! The picture decoder's [`AcSelection::FromHeader`] default already
//! routes `ac_luma_sel == 0` to `v3_intra_g3()` and `ac_chroma_sel == 0`
//! to `v3_intra_g2()` (per `docs/video/msmpeg4/spec/14-pri-ab-runtime-
//! binding.md` §3.1: luma `0 → G3`, chroma `0 → G2`). Until this round
//! the only G0..G3 coverage was at the per-table / per-symbol level
//! (`tests/g0_g3_extended.rs`); no test drove a real-table G3-coded
//! I-frame through the production `decode_picture` entry point.
//!
//! This file closes that gap. It builds a single-macroblock 16×16 v3
//! I-frame whose luma block 0 carries two real G3 AC tokens (one sub-A
//! continuing token + one sub-B terminator), decodes it through
//! `decode_picture` (default `FromHeader`), and asserts the coded
//! block reconstructs **non-DC-only** pel content — i.e. the G3
//! 133-entry canonical-Huffman walker actually consumed the AC bits.
//! A companion assertion proves the same picture decoded with the
//! `Placeholder` selection (empty table) takes the DC-only fallback
//! path, so the difference is attributable to G3 AC decode.
//!
//! # Spec citations (clean-room, docs/ only)
//!
//! - `docs/video/msmpeg4/spec/14-pri-ab-runtime-binding.md` §3.1
//!   (luma `ac_luma_sel == 0 → G3`, chroma `ac_chroma_sel == 0 → G2`).
//! - `docs/video/msmpeg4/spec/11-walker-format-resolved.md` §5 row 4
//!   (G3 packed-Huffman source at file `0x58a08`, 133 symbols).
//! - `docs/video/msmpeg4/spec/13-kernel-block-termination.md` §2 / §7
//!   (sub-A `sym ≤ count_B` continues; sub-B `count_B < sym < count_A`
//!   terminates; G3 `count_A = 132, count_B = 84`).
//! - `docs/video/msmpeg4/spec/01-bitstream-framing.md` §1.4 (v3 I-frame
//!   picture header: `picture_type`, `pquant`, `ac_chroma_sel`,
//!   `ac_luma_sel`, `dc_size_sel`).
//! - `docs/video/msmpeg4/spec/07-remaining-opens.md` §5 (intra-DC
//!   direct-value VLC, ESC sentinel 119).

use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::ac::{AcVlcTable, Symbol};
use oxideav_msmpeg4::picture::{decode_picture, decode_picture_with_ac, AcSelection, PictureDims};
use oxideav_msmpeg4::tables_data::{INTRA_DC_LUMA_SEL0_RAW, MCBPCY_V3_INTRA_RAW};

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

/// Look up the G3 primary-VLC `(code, bit_length)` for a regular
/// `(last, run, |level|)` token. Panics if no such token exists in the
/// G3 alphabet.
fn g3_code_for(last: bool, run: u8, level: u16) -> (u32, u32) {
    let t = AcVlcTable::v3_intra_g3();
    for e in t.entries.iter() {
        if let Symbol::RunLevel {
            last: el,
            run: er,
            level: ev,
        } = e.value
        {
            if el == last && er == run && ev == level {
                return (e.code, e.bits as u32);
            }
        }
    }
    panic!("no G3 token for (last={last}, run={run}, level={level})");
}

/// Build a 16×16 (single-macroblock) v3 I-frame whose luma block 0
/// carries two real G3 AC tokens. The other five blocks are uncoded
/// (DC-size 0, no AC), so the only coded AC in the whole picture flows
/// through the G3 luma table.
///
/// Returns the packed byte stream.
fn build_g3_coded_iframe() -> Vec<u8> {
    // --- Picture header (spec/01 §1.4) ---
    // I-frame, pquant=8, ac_chroma_sel=0 (G2), ac_luma_sel=0 (G3),
    // dc_size_sel=0.
    let mut fields: Vec<(u32, u32)> = vec![
        (0, 2), // picture_type = I
        (8, 5), // pquant = 8
        (0, 1), // ac_chroma_sel = 0 (unary single-zero → 0 → G2)
        (0, 1), // ac_luma_sel = 0 (unary single-zero → 0 → G3)
        (0, 1), // dc_size_sel = 0
    ];

    // --- Macroblock header (round 420): 64-entry intra-CBPCY symbol +
    // ac_pred bit. The symbol bits are in block-decode order (MSB
    // first: Y(0,0), Y(1,0), Y(0,1), Y(1,1), Cb, Cr) and the luma bits
    // are XOR-predicted — for the first MB of a picture every
    // predicted bit is 0, so the symbol equals the actual CBP. Luma
    // block 0 only ⇒ sym = 0b100000 = 32.
    let intra_cbpcy_sym = 32usize;
    let (mc_bl, mc_code) = MCBPCY_V3_INTRA_RAW[intra_cbpcy_sym];
    fields.push((mc_code, mc_bl));
    fields.push((0, 1)); // ac_pred = 0 → zigzag scan, no AC prediction

    // --- Block 0 (luma, CBP set): DC + two G3 AC tokens ---
    // DC differential = 0 (DC-size category 0): luma sel=0 code from the
    // 120-entry direct-value intra-DC VLC (spec/07 §5.4).
    let (dc_bl, dc_code) = INTRA_DC_LUMA_SEL0_RAW[0];
    fields.push((dc_code, dc_bl));
    // No sign bit is read when the DC differential magnitude is 0
    // (spec/07 §5.2: `test al,al; je zero_DC`).

    // First AC token: a sub-A *continuing* token (last=false). idx 0 of
    // every G-alphabet is (last=false, run=0, level=1) per spec/09 §2.
    let (ac0_code, ac0_bl) = g3_code_for(false, 0, 1);
    fields.push((ac0_code, ac0_bl));
    fields.push((1, 1)); // sign bit: 1 ⇒ negative per the AC sign convention

    // Second AC token: a sub-B *terminator* (last=true). Pick the
    // smallest such token present in the G3 alphabet.
    let (last_run, last_level) = pick_g3_terminator();
    let (ac1_code, ac1_bl) = g3_code_for(true, last_run, last_level);
    fields.push((ac1_code, ac1_bl));
    fields.push((0, 1)); // sign bit: 0 ⇒ positive

    // --- Blocks 1..5 (uncoded): DC-size 0 each, no AC ---
    // Luma blocks 1..3 (CBP clear) and chroma blocks 4,5 (CBP clear) all
    // read just a DC-size-0 code. Luma uses the sel=0 luma DC VLC, chroma
    // uses the sel=0 chroma DC VLC. Their CBP bits are clear, so the
    // decoder reads only the DC-size code (no AC walk) for each.
    use oxideav_msmpeg4::tables_data::INTRA_DC_CHROMA_SEL0_RAW;
    for _ in 0..3 {
        let (bl, code) = INTRA_DC_LUMA_SEL0_RAW[0];
        fields.push((code, bl)); // luma DC size 0, diff 0
    }
    for _ in 0..2 {
        let (bl, code) = INTRA_DC_CHROMA_SEL0_RAW[0];
        fields.push((code, bl)); // chroma DC size 0, diff 0
    }

    fields.push((0, 32)); // tail padding
    pack(&fields)
}

/// The smallest `(run, |level|)` of a sub-B terminator token present in
/// the real G3 alphabet. Per spec/13 §2 sub-B is `count_B < sym <
/// count_A` (G3: 84 < sym < 132), every entry of which has `last=true`.
fn pick_g3_terminator() -> (u8, u16) {
    let t = AcVlcTable::v3_intra_g3();
    let mut best: Option<(u8, u16)> = None;
    for e in t.entries.iter() {
        if let Symbol::RunLevel {
            last: true,
            run,
            level,
        } = e.value
        {
            let cand = (run, level);
            best = Some(match best {
                Some(cur) if cur <= cand => cur,
                _ => cand,
            });
        }
    }
    best.expect("G3 alphabet has at least one sub-B (last=true) token")
}

#[test]
fn g3_iframe_decodes_real_ac_through_from_header_default() {
    let bytes = build_g3_coded_iframe();
    let dims = PictureDims::new(16, 16).unwrap();

    // Production path: default AcSelection::FromHeader, ac_luma_sel = 0
    // resolves to the real G3 133-entry primary VLC.
    let mut br = BitReader::new(&bytes);
    let pic = decode_picture(&mut br, dims, None).expect("G3-coded 16x16 I-frame decodes");
    assert_eq!(pic.width, 16);
    assert_eq!(pic.height, 16);

    // Luma block 0 occupies the top-left 8×8 of the picture. With AC
    // coefficients decoded (two non-DC tokens at distinct scan
    // positions), the reconstructed block is NOT spatially uniform — a
    // DC-only reconstruction would be a flat 8×8 patch.
    let mut min = i32::MAX;
    let mut max = i32::MIN;
    for j in 0..8usize {
        for i in 0..8usize {
            let v = pic.y[j * pic.y_stride + i] as i32;
            min = min.min(v);
            max = max.max(v);
        }
    }
    assert!(
        max - min > 0,
        "G3 AC decode must produce a non-uniform block (min={min}, max={max}); \
         a flat block means the AC walk was skipped (DC-only fallback)"
    );
}

#[test]
fn placeholder_selection_takes_dc_only_fallback_for_same_stream() {
    // Decoding the SAME stream with the empty Placeholder table must take
    // the DC-only fallback (entries.is_empty() branch in
    // decode_intra_mb_to_picture) and produce a spatially-uniform block.
    // This pins that the non-uniformity in the FromHeader test is
    // attributable to the real G3 AC walk, not to some other source.
    let bytes = build_g3_coded_iframe();
    let dims = PictureDims::new(16, 16).unwrap();

    let mut br = BitReader::new(&bytes);
    // The DC-only fallback skips the AC bits, so subsequent block reads
    // misalign — but the FIRST coded block's DC reconstruction is well
    // defined and uniform. We only inspect block 0.
    let pic = decode_picture_with_ac(&mut br, dims, None, AcSelection::Placeholder)
        .expect("placeholder DC-only decode of block 0");

    let first = pic.y[0] as i32;
    let mut uniform = true;
    for j in 0..8usize {
        for i in 0..8usize {
            if pic.y[j * pic.y_stride + i] as i32 != first {
                uniform = false;
            }
        }
    }
    assert!(
        uniform,
        "DC-only fallback must reconstruct a spatially-uniform block 0"
    );
}

#[test]
fn from_header_luma_zero_resolves_to_real_g3_table() {
    // Structural guard: confirm the FromHeader dispatch for ac_luma_sel=0
    // routes to the real (non-empty) G3 primary VLC. If a future change
    // re-introduces a placeholder for G3, the end-to-end test above would
    // still pass via the DC-only path for an all-DC stream, so we pin the
    // table non-emptiness here directly.
    let g3 = AcVlcTable::v3_intra_g3();
    assert_eq!(
        g3.entries.len(),
        133,
        "G3 primary VLC must carry its full 132 + 1 ESC alphabet (spec/11 §5 row 4)"
    );
    assert!(
        g3.lmax.is_some() && g3.rmax.is_some(),
        "G3 must carry LMAX/RMAX for the 3-tier ESC body (spec/08 §4.1)"
    );
}
