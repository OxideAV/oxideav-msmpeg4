//! Round-trip tests for the encoder-side entropy primitives — each
//! `encode_*` function is the bit-level inverse of its `decode_*`
//! counterpart, so `decode(encode(x)) == x` must hold over the whole
//! representable domain (or a dense sweep of it).

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_msmpeg4::ac::{
    decode_inter_ac, decode_intra_ac, decode_token, encode_inter_ac, encode_intra_ac, encode_token,
    AcVlcTable, Scan, Symbol, Token,
};
use oxideav_msmpeg4::header::{MsV3PictureHeader, PictureType};
use oxideav_msmpeg4::mb::{decode_intra_dc_diff_v3, encode_intra_dc_diff_v3};
use oxideav_msmpeg4::mcbpcy::{compose_cbp, decode_mcbpcy, encode_mcbpcy};
use oxideav_msmpeg4::mv::{
    decode_mv_with_table, encode_mv_with_table, mv_component_reachable, Mv, MvTable,
};

// ==================== picture header ====================

#[test]
fn v3_picture_header_write_parse_round_trips() {
    for &quant in &[1u8, 2, 7, 16, 31] {
        for ac_chroma_sel in 0..=2u8 {
            for dc_size_sel in 0..=1u8 {
                // I-frame shape: ac_luma_sel carried, mv_table_sel not.
                for ac_luma_sel in 0..=2u8 {
                    let hdr = MsV3PictureHeader {
                        picture_type: PictureType::I,
                        quant,
                        ac_chroma_sel,
                        ac_luma_sel,
                        dc_size_sel,
                        mv_table_sel: 0,
                    };
                    let mut bw = BitWriter::new();
                    hdr.write(&mut bw).unwrap();
                    let bytes = bw.finish();
                    let mut br = BitReader::new(&bytes);
                    let parsed = MsV3PictureHeader::parse(&mut br).unwrap();
                    assert_eq!(parsed.picture_type, PictureType::I);
                    assert_eq!(parsed.quant, quant);
                    assert_eq!(parsed.ac_chroma_sel, ac_chroma_sel);
                    assert_eq!(parsed.ac_luma_sel, ac_luma_sel);
                    assert_eq!(parsed.dc_size_sel, dc_size_sel);
                    assert_eq!(parsed.mv_table_sel, 0);
                }
                // P-frame shape: mv_table_sel carried, ac_luma_sel not.
                for mv_table_sel in 0..=1u8 {
                    let hdr = MsV3PictureHeader {
                        picture_type: PictureType::P,
                        quant,
                        ac_chroma_sel,
                        ac_luma_sel: 0,
                        dc_size_sel,
                        mv_table_sel,
                    };
                    let mut bw = BitWriter::new();
                    hdr.write(&mut bw).unwrap();
                    let bytes = bw.finish();
                    let mut br = BitReader::new(&bytes);
                    let parsed = MsV3PictureHeader::parse(&mut br).unwrap();
                    assert_eq!(parsed.picture_type, PictureType::P);
                    assert_eq!(parsed.quant, quant);
                    assert_eq!(parsed.ac_chroma_sel, ac_chroma_sel);
                    assert_eq!(parsed.dc_size_sel, dc_size_sel);
                    assert_eq!(parsed.mv_table_sel, mv_table_sel);
                }
            }
        }
    }
}

#[test]
fn v3_picture_header_write_rejects_invalid_fields() {
    let mut hdr = MsV3PictureHeader {
        picture_type: PictureType::I,
        quant: 0,
        ac_chroma_sel: 0,
        ac_luma_sel: 0,
        dc_size_sel: 0,
        mv_table_sel: 0,
    };
    let mut bw = BitWriter::new();
    assert!(hdr.write(&mut bw).is_err(), "quant 0 must be rejected");
    hdr.quant = 5;
    hdr.ac_chroma_sel = 3;
    let mut bw = BitWriter::new();
    assert!(hdr.write(&mut bw).is_err(), "selector 3 must be rejected");
}

// ==================== joint MCBPCY ====================

#[test]
fn v3_mcbpcy_encode_decode_round_trips_all_128_symbols() {
    for idx in 0..128u8 {
        let mut bw = BitWriter::new();
        encode_mcbpcy(&mut bw, idx).unwrap();
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let dec = decode_mcbpcy(&mut br).unwrap();
        assert_eq!(dec.idx, idx, "joint symbol {idx} did not round-trip");
        assert_eq!(dec.is_intra, idx < 64);
        // The CBP split must agree with compose_cbp on the low 6 bits.
        assert_eq!(
            compose_cbp(dec.cbpy, dec.cbp_cb, dec.cbp_cr),
            idx & 0x3f,
            "CBP recomposition mismatch at idx {idx}"
        );
    }
}

#[test]
fn v3_mcbpcy_encode_rejects_out_of_alphabet() {
    let mut bw = BitWriter::new();
    assert!(encode_mcbpcy(&mut bw, 128).is_err());
}

// ==================== v3 intra-DC differential ====================

#[test]
fn v3_intra_dc_diff_round_trips_full_range() {
    for &block_idx in &[0usize, 4] {
        for dc_size_sel in 0..=1u8 {
            for diff in -255i32..=255 {
                let mut bw = BitWriter::new();
                encode_intra_dc_diff_v3(&mut bw, block_idx, dc_size_sel, diff).unwrap();
                let bytes = bw.finish();
                let mut br = BitReader::new(&bytes);
                let back = decode_intra_dc_diff_v3(&mut br, block_idx, dc_size_sel).unwrap();
                assert_eq!(
                    back, diff,
                    "DC diff {diff} (blk {block_idx}, sel {dc_size_sel}) did not round-trip"
                );
            }
        }
    }
}

#[test]
fn v3_intra_dc_diff_rejects_over_esc_range() {
    let mut bw = BitWriter::new();
    assert!(encode_intra_dc_diff_v3(&mut bw, 0, 0, 256).is_err());
    let mut bw = BitWriter::new();
    assert!(encode_intra_dc_diff_v3(&mut bw, 0, 0, -256).is_err());
}

// ==================== v3 joint MV ====================

#[test]
fn v3_mv_round_trips_dense_window_both_tables() {
    for &table in &[MvTable::Default, MvTable::Alternate] {
        for &pred in &[Mv { x: 0, y: 0 }, Mv { x: 5, y: -7 }, Mv { x: -20, y: 13 }] {
            for mx in -16i32..=16 {
                for my in -16i32..=16 {
                    let mv = Mv {
                        x: (pred.x as i32 + mx) as i8,
                        y: (pred.y as i32 + my) as i8,
                    };
                    let mut bw = BitWriter::new();
                    encode_mv_with_table(&mut bw, pred, table, mv).unwrap();
                    let bytes = bw.finish();
                    let mut br = BitReader::new(&bytes);
                    let back = decode_mv_with_table(&mut br, pred, table).unwrap();
                    assert_eq!(back, mv, "MV {mv:?} pred {pred:?} table {table:?}");
                }
            }
        }
    }
}

#[test]
fn v3_mv_round_trips_every_residual_from_zero_predictor() {
    // The 6-bit toroidal residual addresses exactly the window
    // [pred-32, pred+31] per component; from pred = 0 that covers
    // [-32, +31] on each axis. Exhaustive over the whole window.
    let pred = Mv { x: 0, y: 0 };
    for &table in &[MvTable::Default, MvTable::Alternate] {
        for x in -32i32..=31 {
            for y in -32i32..=31 {
                let mv = Mv {
                    x: x as i8,
                    y: y as i8,
                };
                let mut bw = BitWriter::new();
                encode_mv_with_table(&mut bw, pred, table, mv).unwrap();
                let bytes = bw.finish();
                let mut br = BitReader::new(&bytes);
                let back = decode_mv_with_table(&mut br, pred, table).unwrap();
                assert_eq!(back, mv);
            }
        }
    }
}

#[test]
fn v3_mv_unreachable_component_is_rejected() {
    // From pred = +63 the residual inputs to the single-pass wrap span
    // [31, 94], so the reachable set is [31, 63] ∪ [0, 30] = [0, 63]:
    // every negative component is unreachable.
    assert!(!mv_component_reachable(-63, 63));
    assert!(!mv_component_reachable(-1, 63));
    assert!(mv_component_reachable(0, 63));
    assert!(mv_component_reachable(30, 63));
    assert!(mv_component_reachable(63, 63));
    let mut bw = BitWriter::new();
    let err = encode_mv_with_table(
        &mut bw,
        Mv { x: 63, y: 0 },
        MvTable::Default,
        Mv { x: -63, y: 0 },
    );
    assert!(err.is_err(), "unreachable MV must be rejected");
}

// ==================== AC tokens ====================

fn all_tables() -> Vec<(&'static str, AcVlcTable)> {
    vec![
        ("G0", AcVlcTable::v3_intra_g0()),
        ("G1", AcVlcTable::v3_intra_g1()),
        ("G2", AcVlcTable::v3_intra_g2()),
        ("G3", AcVlcTable::v3_intra_g3()),
        ("G5", AcVlcTable::v3_intra_g5()),
        ("G4", AcVlcTable::g4_inter()),
    ]
}

#[test]
fn ac_token_round_trips_every_primary_alphabet_entry_both_signs() {
    for (name, table) in all_tables() {
        for e in table.entries {
            let Symbol::RunLevel { last, run, level } = e.value else {
                continue;
            };
            for &sign in &[1i16, -1] {
                let tok = Token {
                    last,
                    run,
                    level: level as i16 * sign,
                };
                let mut bw = BitWriter::new();
                encode_token(&mut bw, &table, tok).unwrap();
                let bytes = bw.finish();
                let mut br = BitReader::new(&bytes);
                let back = decode_token(&mut br, &table).unwrap();
                assert_eq!(back, tok, "{name}: primary token {tok:?}");
            }
        }
    }
}

#[test]
fn ac_token_escape_tiers_round_trip_on_three_tier_table() {
    let table = AcVlcTable::v3_intra_g5();
    let (lmax, rmax) = (table.lmax.unwrap(), table.rmax.unwrap());

    // Tier 1: (last=0, run=0) with |level| = LMAX + 1.
    let l1 = lmax[0][0] as i16 + 1;
    // Tier 2: (last=0, level=1) with run = RMAX + 1.
    let r2 = rmax[0][1] + 1;
    // Tier 3: far outside every extension (run 63 never appears with
    // |level| 100 in any G-family alphabet).
    for tok in [
        Token {
            last: false,
            run: 0,
            level: l1,
        },
        Token {
            last: false,
            run: r2,
            level: -1,
        },
        Token {
            last: true,
            run: 63,
            level: 100,
        },
        Token {
            last: true,
            run: 63,
            level: -100,
        },
    ] {
        let mut bw = BitWriter::new();
        encode_token(&mut bw, &table, tok).unwrap();
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let back = decode_token(&mut br, &table).unwrap();
        assert_eq!(back, tok, "3-tier escape token {tok:?}");
    }
}

#[test]
fn ac_token_escape_round_trips_on_single_tier_inter_table() {
    let table = AcVlcTable::g4_inter();
    assert!(table.lmax.is_none() && table.rmax.is_none());
    for tok in [
        Token {
            last: false,
            run: 40,
            level: 90,
        },
        Token {
            last: true,
            run: 63,
            level: -127,
        },
    ] {
        let mut bw = BitWriter::new();
        encode_token(&mut bw, &table, tok).unwrap();
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let back = decode_token(&mut br, &table).unwrap();
        assert_eq!(back, tok, "1-tier escape token {tok:?}");
    }
}

#[test]
fn ac_token_rejects_invalid_shapes() {
    let table = AcVlcTable::v3_intra_g5();
    let mut bw = BitWriter::new();
    assert!(encode_token(
        &mut bw,
        &table,
        Token {
            last: false,
            run: 0,
            level: 0
        }
    )
    .is_err());
    let mut bw = BitWriter::new();
    assert!(encode_token(
        &mut bw,
        &table,
        Token {
            last: false,
            run: 64,
            level: 1
        }
    )
    .is_err());
}

// ==================== AC block walks ====================

#[test]
fn intra_ac_block_round_trips_across_scans_and_tables() {
    for (name, table) in all_tables() {
        for &scan in &[
            Scan::Zigzag,
            Scan::AlternateHorizontal,
            Scan::AlternateVertical,
        ] {
            // Deterministic sparse pattern with mixed runs/levels/signs.
            let mut levels = [0i32; 64];
            let order = scan.table();
            levels[order[1]] = 3;
            levels[order[2]] = -1;
            levels[order[7]] = 1;
            levels[order[20]] = -25;
            levels[order[63]] = 2;

            let mut bw = BitWriter::new();
            encode_intra_ac(&mut bw, &levels, scan, &table, 1).unwrap();
            let bytes = bw.finish();
            let mut br = BitReader::new(&bytes);
            let mut back = [0i32; 64];
            let n = decode_intra_ac(&mut br, &mut back, scan, &table, 1).unwrap();
            assert_eq!(n, 5, "{name}/{scan:?}: wrong non-zero count");
            assert_eq!(back, levels, "{name}/{scan:?}: levels mismatch");
        }
    }
}

#[test]
fn inter_ac_block_round_trips_including_dc_position() {
    let table = AcVlcTable::g4_inter();
    // DC (zigzag position 0) coded — exercises the first-token run
    // semantics of the inter kernel.
    let mut levels = [0i32; 64];
    levels[0] = -4; // zigzag pos 0 == natural 0
    let zz = Scan::Zigzag.table();
    levels[zz[3]] = 2;
    levels[zz[40]] = -1;

    let mut bw = BitWriter::new();
    encode_inter_ac(&mut bw, &levels, &table).unwrap();
    let bytes = bw.finish();
    let mut br = BitReader::new(&bytes);
    let mut back = [0i32; 64];
    let n = decode_inter_ac(&mut br, &mut back, &table).unwrap();
    assert_eq!(n, 3);
    assert_eq!(back, levels);

    // A block whose first coded coefficient is NOT the DC.
    let mut levels = [0i32; 64];
    levels[zz[5]] = 1;
    levels[zz[6]] = -1;
    let mut bw = BitWriter::new();
    encode_inter_ac(&mut bw, &levels, &table).unwrap();
    let bytes = bw.finish();
    let mut br = BitReader::new(&bytes);
    let mut back = [0i32; 64];
    decode_inter_ac(&mut br, &mut back, &table).unwrap();
    assert_eq!(back, levels);
}

#[test]
fn ac_block_walks_reject_all_zero_blocks() {
    let table = AcVlcTable::v3_intra_g5();
    let levels = [0i32; 64];
    let mut bw = BitWriter::new();
    assert!(encode_intra_ac(&mut bw, &levels, Scan::Zigzag, &table, 1).is_err());
    let inter = AcVlcTable::g4_inter();
    let mut bw = BitWriter::new();
    assert!(encode_inter_ac(&mut bw, &levels, &inter).is_err());
}
