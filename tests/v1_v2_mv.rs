//! Integration tests for the v1/v2 per-component motion-vector decoder
//! added in round 12.
//!
//! Per spec/06 §2.3 / spec/07 §3 the v1/v2 MV decoder uses TWO separate
//! canonical-Huffman reads against the shared 65-entry table at VMA
//! `0x1c24f930` — one for MVDx, one for MVDy. The full 16384-byte LUT
//! lives at `crates/oxideav-msmpeg4/tables/region_04ed30_full.hex` and
//! is parsed by `build.rs::emit_mv_v1_v2` into the runtime
//! `MV_V1_V2_RAW` triples.
//!
//! This file exercises:
//! 1. The end-to-end MCBPCY → median-predictor → per-axis MV decode
//!    chain, mirroring the binary's `0x1c217e56` v<4 control flow.
//! 2. Boundary behaviour at the alphabet endpoints (MVD = ±32) and the
//!    toroidal wrap to `[-63, +63]`.
//! 3. Symmetry of the alphabet around the bias point (sym 32 = MVD 0):
//!    sym 32+k and sym 32-k always have the same canonical bit length.
//!
//! NOTE: full per-MB pixel reconstruction is still gated on the v1/v2
//! intra-AC and inter-AC VLC tables (spec/99 §9 OPEN-O4). The MV
//! decoder built in this round produces the (MVx, MVy) byte pair the
//! MC fetcher consumes, but the residual path is still placeholder.

use oxideav_core::bits::BitReader;
use oxideav_msmpeg4::mcbpcy::{decode_mcbpcy_v1, decode_mcbpcy_v2, V2FrameType};
use oxideav_msmpeg4::mv::{decode_mv_v1v2, decode_mvd_v1v2_raw, median_predictor, Mv};
use oxideav_msmpeg4::tables_data::{MV_V1_V2_BIAS, MV_V1_V2_RAW};

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

/// Look up the (bit_length, code) of a single MV symbol.
fn mv_for(sym: u8) -> (u32, u32) {
    let &(_, bl, code) = MV_V1_V2_RAW
        .iter()
        .find(|&&(s, _, _)| s == sym)
        .unwrap_or_else(|| panic!("v1/v2 MV: sym {sym} not in alphabet"));
    (bl as u32, code)
}

#[test]
fn mv_alphabet_is_65_entries_contiguous() {
    assert_eq!(MV_V1_V2_RAW.len(), 65);
    let syms: Vec<u8> = MV_V1_V2_RAW.iter().map(|&(s, _, _)| s).collect();
    let expected: Vec<u8> = (0u8..=64).collect();
    assert_eq!(syms, expected);
    assert_eq!(MV_V1_V2_BIAS, 32);
}

#[test]
fn mv_alphabet_is_symmetric_around_bias() {
    // For every magnitude k in 1..=32, sym 32+k and sym 32-k must share
    // the same canonical bit-length. (The codes themselves differ by 1
    // bit per the LUT extraction; the property here is bit-length
    // symmetry, which is the H.263-lineage MVD VLC's defining shape.)
    for k in 1u8..=32 {
        let (bl_pos, _) = mv_for(32 + k);
        let (bl_neg, _) = mv_for(32 - k);
        assert_eq!(
            bl_pos, bl_neg,
            "alphabet asymmetric at magnitude {k}: bl(+k)={bl_pos} bl(-k)={bl_neg}",
        );
    }
}

#[test]
fn mvd_zero_decodes_to_one_bit() {
    // Sym 32 = MVD 0. Code = 'b1, length 1.
    let (bl, code) = mv_for(32);
    assert_eq!(bl, 1);
    assert_eq!(code, 0b1);
    let bytes = pack(&[(code, bl)]);
    let mut br = BitReader::new(&bytes);
    let raw = decode_mvd_v1v2_raw(&mut br).unwrap();
    assert_eq!(raw, 32);
}

#[test]
fn decode_mv_v1v2_pair_zero_predictor() {
    // Decode (sym=32, sym=32) → MVD=(0, 0) with predictor (0, 0) → MV=(0, 0).
    let (blx, codex) = mv_for(32);
    let (bly, codey) = mv_for(32);
    let bytes = pack(&[(codex, blx), (codey, bly)]);
    let mut br = BitReader::new(&bytes);
    let mv = decode_mv_v1v2(&mut br, Mv::default()).unwrap();
    assert_eq!(mv, Mv { x: 0, y: 0 });
}

#[test]
fn decode_mv_v1v2_max_negative_with_zero_predictor() {
    // Sym 0 = raw 0 → MVD = -32 after bias subtract.
    let (blx, codex) = mv_for(0);
    let (bly, codey) = mv_for(32);
    let bytes = pack(&[(codex, blx), (codey, bly)]);
    let mut br = BitReader::new(&bytes);
    let mv = decode_mv_v1v2(&mut br, Mv::default()).unwrap();
    assert_eq!(mv, Mv { x: -32, y: 0 });
}

#[test]
fn decode_mv_v1v2_max_positive_with_zero_predictor() {
    // Sym 64 = raw 64 → MVD = +32.
    let (blx, codex) = mv_for(64);
    let (bly, codey) = mv_for(32);
    let bytes = pack(&[(codex, blx), (codey, bly)]);
    let mut br = BitReader::new(&bytes);
    let mv = decode_mv_v1v2(&mut br, Mv::default()).unwrap();
    assert_eq!(mv, Mv { x: 32, y: 0 });
}

#[test]
fn decode_mv_v1v2_predictor_pulls_into_torus() {
    // MVD = +32 + predictor +32 = +64 → wraps to 0.
    let (blx, codex) = mv_for(64);
    let (bly, codey) = mv_for(32);
    let bytes = pack(&[(codex, blx), (codey, bly)]);
    let mut br = BitReader::new(&bytes);
    let mv = decode_mv_v1v2(&mut br, Mv { x: 32, y: 0 }).unwrap();
    // 32 + 32 = 64 → wrap → 0.
    assert_eq!(mv, Mv { x: 0, y: 0 });
}

#[test]
fn decode_mv_v1v2_negative_wrap() {
    // MVD = -32, predictor -32 → -64 → wraps to 0.
    let (blx, codex) = mv_for(0);
    let (bly, codey) = mv_for(32);
    let bytes = pack(&[(codex, blx), (codey, bly)]);
    let mut br = BitReader::new(&bytes);
    let mv = decode_mv_v1v2(&mut br, Mv { x: -32, y: 0 }).unwrap();
    assert_eq!(mv, Mv { x: 0, y: 0 });
}

#[test]
fn full_chain_mcbpcy_mv_v1_inter_mb() {
    // Full v1 P-frame inter-MB decode chain (without the AC residual,
    // which is still spec/99 §9 OPEN):
    //
    // 1. Decode MCBPCY: skip=0, MCBPC=0 (mb_type=0 inter), CBPY pattern
    //    (post-wrap) = 0 (no luma blocks coded).
    // 2. Decode MV: MVD=(+1, -2) with predictor (0, 0) → MV=(1, -2).
    use oxideav_msmpeg4::tables_data::MCBPC_V1_RAW;

    let &(_, mc_bl, mc_code) = MCBPC_V1_RAW.iter().find(|&&(s, _, _)| s == 0).unwrap();
    // CBPY pattern 15 raw → wrap → 0; CBPY 15 has bl=2 code=11 in CBPY_INTRA.
    use oxideav_msmpeg4::tables::CBPY_INTRA_TABLE;
    let cb = CBPY_INTRA_TABLE.iter().find(|e| e.value == 15).unwrap();

    // MV: sym 33 = MVD +1, sym 30 = MVD -2.
    let (mvx_bl, mvx_code) = mv_for(33);
    let (mvy_bl, mvy_code) = mv_for(30);

    let bytes = pack(&[
        (0, 1),                    // skip = 0
        (mc_code, mc_bl as u32),   // MCBPC sym 0
        (cb.code, cb.bits as u32), // CBPY raw 15 → wrap → 0
        (mvx_code, mvx_bl),        // MVDx sym 33 → +1
        (mvy_code, mvy_bl),        // MVDy sym 30 → -2
    ]);

    let mut br = BitReader::new(&bytes);
    let dec = decode_mcbpcy_v1(&mut br).unwrap();
    assert!(!dec.skip);
    assert_eq!(dec.mb_type, 0, "inter MB");
    assert!(!dec.is_intra);
    assert_eq!(dec.cbpy, 0, "no luma residual");

    let mv = decode_mv_v1v2(&mut br, Mv::default()).unwrap();
    assert_eq!(mv, Mv { x: 1, y: -2 });
}

#[test]
fn full_chain_mcbpcy_mv_v2_inter_mb_with_predictor() {
    // v2 P-frame inter-MB chain with non-zero predictor.
    use oxideav_msmpeg4::tables::CBPY_INTRA_TABLE;
    use oxideav_msmpeg4::tables_data::MCBPC_V2_RAW;

    // v2 MCBPC sym 0: quotient=0 (inter), remainder=0 (no chroma CBP).
    let &(_, mc_bl, mc_code) = MCBPC_V2_RAW.iter().find(|&&(s, _, _)| s == 0).unwrap();
    let cb = CBPY_INTRA_TABLE.iter().find(|e| e.value == 15).unwrap();

    // MVD = (+5, -3); predictor = (10, -8). Final MV = (15, -11).
    let (mvx_bl, mvx_code) = mv_for((32i32 + 5) as u8);
    let (mvy_bl, mvy_code) = mv_for((32i32 - 3) as u8);

    let bytes = pack(&[
        (0, 1),                    // skip = 0
        (mc_code, mc_bl as u32),   // MCBPC sym 0
        (cb.code, cb.bits as u32), // CBPY
        (mvx_code, mvx_bl),
        (mvy_code, mvy_bl),
    ]);

    let mut br = BitReader::new(&bytes);
    let dec = decode_mcbpcy_v2(&mut br, V2FrameType::P).unwrap();
    assert!(!dec.skip);

    let predictor = Mv { x: 10, y: -8 };
    let mv = decode_mv_v1v2(&mut br, predictor).unwrap();
    assert_eq!(mv, Mv { x: 15, y: -11 });
}

#[test]
fn median_predictor_chains_with_decode() {
    // Three neighbour MVs: left=(2, 1), top=(5, 3), top_right=(1, 4).
    // X median: median(2, 5, 1) = 2; Y median: median(1, 3, 4) = 3.
    let left = Some(Mv { x: 2, y: 1 });
    let top = Some(Mv { x: 5, y: 3 });
    let top_right = Some(Mv { x: 1, y: 4 });
    let pred = median_predictor(left, top, top_right);
    assert_eq!(pred, Mv { x: 2, y: 3 });

    // Now decode an MVD = (0, 0) — pred is the result.
    let (blx, codex) = mv_for(32);
    let (bly, codey) = mv_for(32);
    let bytes = pack(&[(codex, blx), (codey, bly)]);
    let mut br = BitReader::new(&bytes);
    let mv = decode_mv_v1v2(&mut br, pred).unwrap();
    assert_eq!(mv, pred);
}

#[test]
fn round_trip_every_mv_combination_with_zero_predictor() {
    // For every alphabet symbol pair (sym_x, sym_y), pack the canonical
    // codes and verify decode_mv_v1v2 returns the MVD-bias-adjusted MV.
    // 65 × 65 = 4225 cases — fast enough.
    let pred = Mv::default();
    for &(sx, blx, codex) in MV_V1_V2_RAW {
        let mvd_x = sx as i32 - MV_V1_V2_BIAS;
        for &(sy, bly, codey) in MV_V1_V2_RAW {
            let mvd_y = sy as i32 - MV_V1_V2_BIAS;
            let bytes = pack(&[(codex, blx as u32), (codey, bly as u32)]);
            let mut br = BitReader::new(&bytes);
            let mv = decode_mv_v1v2(&mut br, pred).unwrap();
            assert_eq!(
                mv.x as i32, mvd_x,
                "x mismatch at sym ({sx}, {sy}) — expected MVD={mvd_x}, got x={}",
                mv.x
            );
            assert_eq!(
                mv.y as i32, mvd_y,
                "y mismatch at sym ({sx}, {sy}) — expected MVD={mvd_y}, got y={}",
                mv.y
            );
        }
    }
}
