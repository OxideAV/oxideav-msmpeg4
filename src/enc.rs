//! MS-MPEG4 v3 picture-level encoder.
//!
//! Produces bitstreams this crate's own [`crate::picture::decode_picture`]
//! reconstructs — every syntax element goes through the encode-side
//! inverses of the decode surfaces (`header::MsV3PictureHeader::write`,
//! `mcbpcy::encode_mcbpcy`, `mb::encode_intra_dc_diff_v3`,
//! `ac::encode_intra_ac` / `encode_inter_ac`, `mv::encode_mv_with_table`),
//! so encode → decode round-trips by construction over the same
//! extracted tables (spec/11 §5, spec/16 §1, spec/99 §3.1).
//!
//! The encoder mirrors the decoder's prediction state exactly:
//!
//! * the intra DC spatial predictor threads the same reconstructed
//!   (integer) DC values through a [`crate::dc_pred::DcCache`], so the
//!   per-block predictor and differential agree with what the decoder
//!   derives (MPEG-4 §7.4.3 gradient rule);
//! * the P-frame §7.6.5 median MV predictor is computed over the same
//!   [`crate::mv_pred::MvGrid`] the decoder maintains, and motion
//!   vectors are constrained to the spec/06 §3.5 toroidal window
//!   around that predictor (`mv::mv_component_reachable`);
//! * AC prediction is not used (`ac_pred = 0` on every MB), so every
//!   coded block walks the fixed zigzag scan.
//!
//! Table choices are fixed per frame: intra luma = G5
//! (`ac_luma_sel = 2`), chroma = G4 (`ac_chroma_sel = 2`), default
//! intra-DC pair (`dc_size_sel = 0`), default joint-MV VLC
//! (`mv_table_sel = 0`). Inter residuals always use G4 (the alphabet
//! is not frame-selectable on the inter path, spec/04 §1.3).

use oxideav_core::bits::BitWriter;
use oxideav_core::{Error, Result};

use crate::ac::{encode_intra_ac, AcVlcTable, Scan};
use crate::dc_pred::DcCache;
use crate::header::{MsV3PictureHeader, PictureType};
use crate::idct::fdct8x8_from_pels;
use crate::iq::{dc_scaler, quantise_block_h263};
use crate::mb::encode_intra_dc_diff_v3;
use crate::mcbpcy::{compose_cbp, encode_mcbpcy};
use crate::picture::{Picture, PictureDims};

/// Per-frame encoder settings.
#[derive(Clone, Copy, Debug)]
pub struct EncoderConfig {
    /// Frame-wide quantiser, 1..=31 (5-bit PQUANT).
    pub quant: u8,
    /// Half-pel motion-search range per component (0 = zero-MV only).
    /// The search window at each MB is `[-range, +range]` half-pel
    /// steps around the zero MV, additionally clipped to the spec/06
    /// §3.5 toroidal window around the §7.6.5 predictor.
    pub mv_search_range: u8,
}

impl Default for EncoderConfig {
    fn default() -> Self {
        Self {
            quant: 4,
            mv_search_range: 8,
        }
    }
}

/// The fixed per-frame table selectors this encoder emits (see the
/// module doc): G5 intra luma, G4 chroma, default DC pair, default MV
/// VLC.
const AC_LUMA_SEL: u8 = 2; // G5
const AC_CHROMA_SEL: u8 = 2; // G4
const DC_SIZE_SEL: u8 = 0;

/// One quantised 8×8 block of an intra MB, ready for serialisation.
struct IntraBlockPlan {
    dc_diff: i32,
    /// Bitstream AC levels in natural order (position 0 unused).
    levels: [i32; 64],
    coded: bool,
}

/// Encode one picture as a v3 **I-frame**. `input` must carry
/// MB-aligned planes for `dims` (the layout [`Picture::alloc`]
/// produces). Returns the frame's bitstream, decodable by
/// [`crate::picture::decode_picture`] with the same `dims`.
pub fn encode_iframe_v3(
    input: &Picture,
    dims: PictureDims,
    config: &EncoderConfig,
) -> Result<Vec<u8>> {
    validate_input(input, dims, config)?;
    let (mb_w, mb_h) = dims.mb_dims();
    let quant = config.quant as u32;

    let mut bw = BitWriter::new();
    let hdr = MsV3PictureHeader {
        picture_type: PictureType::I,
        quant: config.quant,
        ac_chroma_sel: AC_CHROMA_SEL,
        ac_luma_sel: AC_LUMA_SEL,
        dc_size_sel: DC_SIZE_SEL,
        mv_table_sel: 0,
    };
    hdr.write(&mut bw)?;

    let luma_ac = AcVlcTable::v3_intra_g5();
    let chroma_ac = AcVlcTable::g4_inter();
    let mut dc_cache = DcCache::new(mb_w, mb_h);

    for my in 0..mb_h {
        for mx in 0..mb_w {
            encode_intra_mb(
                &mut bw,
                input,
                &mut dc_cache,
                mx,
                my,
                quant,
                &luma_ac,
                &chroma_ac,
            )?;
        }
    }
    Ok(bw.finish())
}

/// Analyse + serialise one intra MB (shared by the I-frame path; the
/// P-frame encoder currently never emits intra-in-P MBs).
#[allow(clippy::too_many_arguments)]
fn encode_intra_mb(
    bw: &mut BitWriter,
    input: &Picture,
    dc_cache: &mut DcCache,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    luma_ac: &AcVlcTable,
    chroma_ac: &AcVlcTable,
) -> Result<()> {
    // Pass 1: transform + quantise all 6 blocks, threading the DC
    // predictor exactly as the decoder will (§7.4.3 gradient over
    // reconstructed DCs).
    let mut plans: Vec<IntraBlockPlan> = Vec::with_capacity(6);
    for block_idx in 0..6usize {
        let pels = extract_block(input, mb_x, mb_y, block_idx);
        let mut f = [0.0f32; 64];
        fdct8x8_from_pels(&pels, &mut f);

        let (bx, by) = block_grid_pos(block_idx, mb_x, mb_y);
        let pred = match block_idx {
            0..=3 => dc_cache.predict_luma(bx, by),
            4 => dc_cache.predict_chroma(false, bx, by),
            5 => dc_cache.predict_chroma(true, bx, by),
            _ => unreachable!(),
        };

        let scaler = dc_scaler(block_idx, quant) as i32;
        let dc_diff =
            (((f[0] - pred.predictor as f32) / scaler as f32).round() as i32).clamp(-255, 255);
        let dc_recon = pred.predictor + dc_diff * scaler;
        match block_idx {
            0..=3 => dc_cache.luma_set(bx, by, dc_recon),
            4 => dc_cache.chroma_set(false, bx, by, dc_recon),
            5 => dc_cache.chroma_set(true, bx, by, dc_recon),
            _ => unreachable!(),
        }

        let mut levels = [0i32; 64];
        let nz = quantise_block_h263(&f, quant, 1, &mut levels);
        plans.push(IntraBlockPlan {
            dc_diff,
            levels,
            coded: nz > 0,
        });
    }

    // Pass 2: serialise — joint MCBPCY (I-type half: idx = cbp),
    // ac_pred bit (always 0: fixed zigzag), then per-block DC + AC.
    let cbpy = (plans[0].coded as u8) << 3
        | (plans[1].coded as u8) << 2
        | (plans[2].coded as u8) << 1
        | (plans[3].coded as u8);
    let cbp = compose_cbp(cbpy, plans[4].coded, plans[5].coded);
    encode_mcbpcy(bw, cbp)?;
    bw.write_bit(false); // ac_pred = 0

    for (block_idx, plan) in plans.iter().enumerate() {
        encode_intra_dc_diff_v3(bw, block_idx, DC_SIZE_SEL, plan.dc_diff)?;
        if plan.coded {
            let table = if block_idx <= 3 { luma_ac } else { chroma_ac };
            encode_intra_ac(bw, &plan.levels, Scan::Zigzag, table, 1)?;
        }
    }
    Ok(())
}

/// Extract one 8×8 block (natural order) from the input picture.
/// `block_idx` follows the MB convention: 0..=3 luma raster, 4 = Cb,
/// 5 = Cr.
fn extract_block(pic: &Picture, mb_x: usize, mb_y: usize, block_idx: usize) -> [i32; 64] {
    let mut out = [0i32; 64];
    match block_idx {
        0..=3 => {
            let x0 = mb_x * 16 + (block_idx & 1) * 8;
            let y0 = mb_y * 16 + (block_idx >> 1) * 8;
            for j in 0..8 {
                for i in 0..8 {
                    out[j * 8 + i] = pic.y[(y0 + j) * pic.y_stride + x0 + i] as i32;
                }
            }
        }
        4 | 5 => {
            let plane = if block_idx == 4 { &pic.cb } else { &pic.cr };
            let x0 = mb_x * 8;
            let y0 = mb_y * 8;
            for j in 0..8 {
                for i in 0..8 {
                    out[j * 8 + i] = plane[(y0 + j) * pic.c_stride + x0 + i] as i32;
                }
            }
        }
        _ => unreachable!(),
    }
    out
}

/// Map a per-MB block index to its block-grid position (mirror of the
/// decoder-side helper in `picture.rs`).
fn block_grid_pos(block_idx: usize, mb_x: usize, mb_y: usize) -> (usize, usize) {
    match block_idx {
        0..=3 => (mb_x * 2 + (block_idx & 1), mb_y * 2 + (block_idx >> 1)),
        4 | 5 => (mb_x, mb_y),
        _ => unreachable!(),
    }
}

fn validate_input(input: &Picture, dims: PictureDims, config: &EncoderConfig) -> Result<()> {
    if !(1..=31).contains(&config.quant) {
        return Err(Error::invalid(format!(
            "msmpeg4v3 encode: quant {} out of range 1..=31",
            config.quant
        )));
    }
    let (mb_w, mb_h) = dims.mb_dims();
    if input.y_stride < mb_w * 16
        || input.c_stride < mb_w * 8
        || input.y.len() < input.y_stride * mb_h * 16
        || input.cb.len() < input.c_stride * mb_h * 8
        || input.cr.len() < input.c_stride * mb_h * 8
    {
        return Err(Error::invalid(
            "msmpeg4v3 encode: input planes smaller than the MB-aligned \
             layout Picture::alloc produces for these dimensions",
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::picture::decode_picture;
    use oxideav_core::bits::BitReader;

    fn fill_gradient(pic: &mut Picture) {
        let (w, h) = (pic.y_stride, pic.y.len() / pic.y_stride);
        for y in 0..h {
            for x in 0..w {
                pic.y[y * pic.y_stride + x] = (((x * 2 + y) / 2) % 200 + 20) as u8;
            }
        }
        let ch = pic.cb.len() / pic.c_stride;
        for y in 0..ch {
            for x in 0..pic.c_stride {
                pic.cb[y * pic.c_stride + x] = ((x + 100) % 220) as u8;
                pic.cr[y * pic.c_stride + x] = ((y + 140) % 220) as u8;
            }
        }
    }

    #[test]
    fn iframe_flat_grey_round_trips_exactly() {
        let dims = PictureDims::new(32, 32).unwrap();
        let input = Picture::alloc(dims, PictureType::I); // all-128 planes
        let bytes = encode_iframe_v3(&input, dims, &EncoderConfig::default()).expect("encode");
        let mut br = BitReader::new(&bytes);
        let out = decode_picture(&mut br, dims, None).expect("decode");
        assert_eq!(out.y, input.y, "flat grey luma must round-trip exactly");
        assert_eq!(out.cb, input.cb);
        assert_eq!(out.cr, input.cr);
    }

    #[test]
    fn iframe_gradient_reconstructs_close_at_fine_quant() {
        let dims = PictureDims::new(48, 32).unwrap();
        let mut input = Picture::alloc(dims, PictureType::I);
        fill_gradient(&mut input);
        let config = EncoderConfig {
            quant: 2,
            ..Default::default()
        };
        let bytes = encode_iframe_v3(&input, dims, &config).expect("encode");
        let mut br = BitReader::new(&bytes);
        let out = decode_picture(&mut br, dims, None).expect("decode");
        // Fine-quant lossy bound: every pel within a small tolerance,
        // and the mean absolute error well under one quant step.
        let mut sum_err = 0u64;
        for (a, b) in out.y.iter().zip(input.y.iter()) {
            let e = (*a as i32 - *b as i32).unsigned_abs();
            assert!(e <= 12, "luma pel error {e} too large at q=2");
            sum_err += e as u64;
        }
        let mae = sum_err as f64 / out.y.len() as f64;
        assert!(mae < 2.0, "luma MAE {mae} too large at q=2");
    }
}
