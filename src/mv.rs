//! MSMPEG4 v3 motion-vector decoder (joint (MVDx, MVDy) VLC + ESC
//! tail + median-of-3 predictor + toroidal clamp).
//!
//! # Decode algorithm (spec/06 §3.1)
//!
//! 1. Compute the per-component predictor `(predX, predY)` as the
//!    median of up to three neighbour MV bytes (left `A`, top `B`,
//!    top-right `C`). Missing neighbours (picture edge) are substituted
//!    with zero byte-for-byte. See [`median_predictor`].
//! 2. Read one joint VLC symbol `idx ∈ [0, 1099]` from
//!    [`MV_V3_RAW`](crate::tables_data::MV_V3_RAW):
//!    * If `idx == 1099` (ESC), read `get_bits(6)` for `MVDx_raw` and
//!      another `get_bits(6)` for `MVDy_raw`. Both are unsigned 6-bit
//!      values in `[0, 63]`.
//!    * Otherwise, `MVDx_raw = MVDX_V3_BYTES[idx]`, `MVDy_raw =
//!      MVDY_V3_BYTES[idx]`.
//! 3. For each component, compute
//!    `mv = MVD_raw + predictor - 32` (the `-32` cancels the `+32` bias
//!    baked into every LUT entry; for ESC raw values this just shifts
//!    the interpretation to signed `[-32, +31]`).
//! 4. Toroidal wrap into `[-63, +63]`: if `mv > 63`, subtract 64; if
//!    `mv < -63`, add 64. One pass suffices because the raw + predictor
//!    sum is bounded by `[-32..=95]` + `[-63..=63]` ≈ `[-95..=158]`,
//!    within one-wrap reach of both endpoints.
//! 5. Output `(MVx, MVy)` as signed bytes.
//!
//! # Clamp bounds
//!
//! The binary stores `(-63, +63)` in `[esi+0xb00..b04]` at DLL init
//! (spec/06 §3.5). These are the half-pel bounds; the integer part is
//! recovered by arithmetic-shift-right by 1 at MC time (spec/04 §3.1:
//! `sar eax, 1` → integer MV; LSB → half-pel fractional).
//!
//! # Alphabet variant selection
//!
//! MSMPEG4 v3 has two MV VLC tables selected by the per-P-frame
//! `mv_table_sel` bit (`[esi+0x834]`). This crate currently carries
//! **only the default variant (selector 0)**; the alternate (selector
//! 1, VMA `0x1c25a0b8`) has only a 256-byte extraction dump available
//! and is not yet usable. Streams with `mv_table_sel == 1` are rejected
//! with [`oxideav_core::Error::Unsupported`].

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::tables_data::{MVDX_V3_BYTES, MVDY_V3_BYTES, MV_V3_ESC_INDEX, MV_V3_RAW};
use crate::vlc::{self, VlcEntry};

/// Lazy-built canonical-Huffman table for the v3 MV VLC default variant.
/// 1100 symbols (indices 0..=1099); index 1099 is the ESC sentinel.
static MV_V3_TABLE: std::sync::OnceLock<Vec<VlcEntry<u16>>> = std::sync::OnceLock::new();

fn build_table() -> Vec<VlcEntry<u16>> {
    // Canonical-Huffman builder (same shape as mcbpcy.rs): sort by
    // (bit_length, symbol_index), assign code = 0 for first, then
    // `code = (code + 1) << (bl_cur - bl_prev)` for each subsequent.
    let mut symbols: Vec<(u32, u16)> = MV_V3_RAW
        .iter()
        .enumerate()
        .filter_map(|(idx, &(bl, _code))| {
            if bl == 0 {
                None
            } else {
                Some((bl, idx as u16))
            }
        })
        .collect();
    symbols.sort_by_key(|&(bl, idx)| (bl, idx));

    let mut entries: Vec<VlcEntry<u16>> = Vec::with_capacity(symbols.len());
    let mut code: u32 = 0;
    let mut prev_bl: u32 = 0;
    for (i, &(bl, idx)) in symbols.iter().enumerate() {
        if i == 0 {
            code = 0;
        } else {
            code = (code + 1) << (bl - prev_bl);
        }
        entries.push(VlcEntry::new(bl as u8, code, idx));
        prev_bl = bl;
    }
    entries
}

fn table() -> &'static [VlcEntry<u16>] {
    MV_V3_TABLE.get_or_init(build_table)
}

/// Decoded motion-vector in half-pel units. Both components are in
/// the toroidal `[-63, +63]` range (signed 7-bit after wrap).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Mv {
    /// Half-pel X component, `[-63, +63]`.
    pub x: i8,
    /// Half-pel Y component, `[-63, +63]`.
    pub y: i8,
}

/// Median-of-3 predictor over the three neighbour bytes supplied per
/// component. `None` indicates a neighbour that is unavailable
/// (picture edge) — substituted with zero. Matches spec/06 §3.4.
///
/// The algorithm is the textbook three-way median: `median(a, b, c) =
/// a + b + c - min(a, b, c) - max(a, b, c)`. Each component is
/// computed independently.
pub fn median_predictor(left: Option<Mv>, top: Option<Mv>, top_right: Option<Mv>) -> Mv {
    let a = left.unwrap_or_default();
    let b = top.unwrap_or_default();
    let c = top_right.unwrap_or_default();

    fn med(a: i8, b: i8, c: i8) -> i8 {
        let mn = a.min(b).min(c);
        let mx = a.max(b).max(c);
        // `a + b + c - mn - mx` = median. Use i32 to avoid i8 overflow.
        (a as i32 + b as i32 + c as i32 - mn as i32 - mx as i32) as i8
    }

    Mv {
        x: med(a.x, b.x, c.x),
        y: med(a.y, b.y, c.y),
    }
}

/// Decode one joint VLC symbol, looking up the two component
/// residuals via the MVDx/MVDy byte LUTs (or reading raw FLC tails for
/// the ESC path), apply the predictor and the toroidal wrap.
///
/// Returns the final `(MVx, MVy)` byte-pair ready to store in the
/// MB-info row.
pub fn decode_mv(br: &mut BitReader<'_>, predictor: Mv) -> Result<Mv> {
    let idx = vlc::decode(br, table())? as usize;

    let (raw_x, raw_y) = if idx == MV_V3_ESC_INDEX {
        // ESC: two 6-bit FLC reads (spec/06 §3.3).
        let x = br.read_u32(6)? as u8;
        let y = br.read_u32(6)? as u8;
        (x, y)
    } else if idx >= MV_V3_ESC_INDEX {
        return Err(Error::invalid(format!(
            "msmpeg4v3 mv: decoded index {idx} out of alphabet range"
        )));
    } else {
        (MVDX_V3_BYTES[idx], MVDY_V3_BYTES[idx])
    };

    // spec/06 §3.5: subtract bias 32 to get signed residual, add
    // predictor, wrap into `[-63, +63]`.
    let mv_x = wrap_component(raw_x as i32 + predictor.x as i32 - 32);
    let mv_y = wrap_component(raw_y as i32 + predictor.y as i32 - 32);
    Ok(Mv { x: mv_x, y: mv_y })
}

/// Toroidal wrap per spec/06 §3.5: if `mv > 63`, subtract 64; if
/// `mv < -63`, add 64. One pass suffices.
fn wrap_component(mv: i32) -> i8 {
    let m = if mv > 63 {
        mv - 64
    } else if mv < -63 {
        mv + 64
    } else {
        mv
    };
    m as i8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mv_vlc_table_has_1100_entries() {
        let t = table();
        assert_eq!(t.len(), 1100);
    }

    #[test]
    fn mv_vlc_is_prefix_free() {
        // Canonical-Huffman: no code is a prefix of another. Cross-
        // check by sampling: for every pair of entries with distinct
        // bit-lengths, the shorter is not a prefix of the longer.
        let t = table();
        for (i, a) in t.iter().enumerate() {
            for b in &t[i + 1..] {
                if a.bits == b.bits {
                    continue;
                }
                let (short, long) = if a.bits < b.bits { (a, b) } else { (b, a) };
                let shift = long.bits - short.bits;
                let long_prefix = long.code >> shift;
                assert_ne!(
                    long_prefix, short.code,
                    "prefix: sym {} (bl={}) is a prefix of sym {} (bl={})",
                    short.value, short.bits, long.value, long.bits,
                );
            }
        }
    }

    #[test]
    fn median_predictor_picks_middle() {
        let a = Mv { x: 1, y: 0 };
        let b = Mv { x: 5, y: 0 };
        let c = Mv { x: 3, y: 0 };
        let p = median_predictor(Some(a), Some(b), Some(c));
        assert_eq!(p.x, 3);
        assert_eq!(p.y, 0);
    }

    #[test]
    fn median_predictor_missing_neighbours_zero() {
        // Only `left` available; the others are treated as zero.
        let a = Mv { x: 5, y: -2 };
        let p = median_predictor(Some(a), None, None);
        assert_eq!(p, Mv { x: 0, y: 0 }); // median(5, 0, 0) = 0
    }

    #[test]
    fn wrap_component_no_change_in_range() {
        for &v in &[-63i32, -32, 0, 31, 63] {
            assert_eq!(wrap_component(v), v as i8);
        }
    }

    #[test]
    fn wrap_component_torus() {
        // > +63 wraps down by 64.
        assert_eq!(wrap_component(64), 0);
        assert_eq!(wrap_component(95), 31);
        // < -63 wraps up by 64.
        assert_eq!(wrap_component(-64), 0);
        assert_eq!(wrap_component(-95), -31);
    }

    #[test]
    fn decode_mv_round_trip_single_symbol() {
        // Pick the first 1-bit symbol in the canonical table. In
        // canonical Huffman the shortest code is `0` (the builder
        // seeds `code = 0` for the first symbol). Encode `0` + tail
        // padding, decode, check MV output.
        let t = table();
        let one_bit: Vec<_> = t.iter().filter(|e| e.bits == 1).collect();
        // Per spec/99: exactly one 1-bit code exists (symbol 0).
        assert_eq!(one_bit.len(), 1);
        let e = one_bit[0];
        assert_eq!(e.code, 0);
        let sym = e.value as usize;
        let expected_raw_x = MVDX_V3_BYTES[sym] as i32;
        let expected_raw_y = MVDY_V3_BYTES[sym] as i32;
        // Predictor = (0, 0). Output = (raw - 32) after wrap.
        let exp_x = wrap_component(expected_raw_x - 32);
        let exp_y = wrap_component(expected_raw_y - 32);

        // Byte stream: 1 bit = '0' followed by pad bits = 0x00.
        let data = [0x00u8, 0x00, 0x00];
        let mut br = BitReader::new(&data);
        let out = decode_mv(&mut br, Mv::default()).unwrap();
        assert_eq!(out.x, exp_x, "x mismatch: raw_x={expected_raw_x}");
        assert_eq!(out.y, exp_y, "y mismatch: raw_y={expected_raw_y}");
    }

    #[test]
    fn decode_mv_esc_path() {
        // ESC is index 1099; its canonical code is the longest shortest
        // path (one of the highest-bit-length entries, but since
        // canonical order is (bit_length asc, idx asc), ESC sits at
        // index 1099 — its code depends on the surrounding bit-length
        // array. Easiest end-to-end test: encode what we'll read (ESC
        // code + 6 bits X_raw + 6 bits Y_raw) and assert the result.
        let t = table();
        let esc_entry = t
            .iter()
            .find(|e| e.value as usize == MV_V3_ESC_INDEX)
            .expect("ESC symbol present");
        // X raw = 40, Y raw = 24 (both arbitrary in [0, 63]).
        let mut acc: u64 = 0;
        let mut bits: u32 = 0;
        acc = (acc << esc_entry.bits) | esc_entry.code as u64;
        bits += esc_entry.bits as u32;
        acc = (acc << 6) | 40;
        bits += 6;
        acc = (acc << 6) | 24;
        bits += 6;
        // Pad to byte boundary + a few extra bytes.
        let pad = (8 - (bits % 8)) % 8;
        acc <<= pad;
        bits += pad;
        let mut data: Vec<u8> = Vec::new();
        while bits > 0 {
            bits -= 8;
            data.push(((acc >> bits) & 0xff) as u8);
        }
        data.extend_from_slice(&[0u8; 4]);
        let mut br = BitReader::new(&data);
        let out = decode_mv(&mut br, Mv::default()).unwrap();
        // raw_x = 40, predictor 0 → 40 - 32 = 8, no wrap needed.
        assert_eq!(out.x, 8);
        // raw_y = 24, predictor 0 → 24 - 32 = -8, no wrap needed.
        assert_eq!(out.y, -8);
    }
}
