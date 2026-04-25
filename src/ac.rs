//! AC-coefficient decoding for MS-MPEG4 intra blocks.
//!
//! Each intra 8×8 block, after its DC coefficient has been decoded
//! separately, carries a run-length-encoded sequence of AC levels:
//!
//! ```text
//!   token := VLC(last, run, level) | ESCAPE
//!   ESCAPE := <escape_prefix> { escape sub-mode body }
//!   block  := token+ ending with last=1
//! ```
//!
//! Each token contributes `(run, level, last)`:
//!
//! * `run`   — number of zero AC coefficients that precede this level
//!   in scan order
//! * `level` — signed AC magnitude at that scan position
//! * `last`  — 1 if this is the last non-zero AC of the block, 0 otherwise
//!
//! Scan order is controlled by the caller (zig-zag, alternate-horizontal
//! or alternate-vertical — see [`crate::scan`]).
//!
//! # Escape mechanism
//!
//! When the primary VLC matches the escape symbol (marked by
//! `Token::Escape` here), the bitstream continues with a sub-mode
//! selector:
//!
//! * **Mode 0** — extended `level` (re-use the base (last, run) symbol
//!   but replace the level with a larger absolute value),
//! * **Mode 1** — extended `run`,
//! * **Mode 2** — full fixed-length `(last, run, level)` triple.
//!
//! Mode 3 is reserved for v3 and carries extended level and extended
//! run simultaneously.
//!
//! Spec / citation:
//! * ISO/IEC 14496-2 §7.4.1.3 (the generic MPEG-4 Part 2 AC escape
//!   structure that MS-MPEG4 inherits, per
//!   `docs/video/msmpeg4/spec/03-corrections.md` §5.2 — the "standard
//!   MPEG-4 3-mode escape").
//! * `docs/video/msmpeg4/spec/02-table-roles.md` §2.2 / §4.4 —
//!   confirms the AC-coefficient VLC is a per-token `(last, run, level)`
//!   joint VLC gated by the AC-pred direction for scan-order choice.
//!
//! The `tables/` folder does not yet contain a Rust-ready copy of the
//! AC VLC — this module provides the *pipeline* (decode → scan →
//! dequantise → IDCT) parameterised by a [`AcVlcTable`] so the table
//! can be plugged in without API churn.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::iq::dequantise_h263;
use crate::scan::{ALTERNATE_HORIZONTAL, ALTERNATE_VERTICAL, ZIGZAG};
use crate::vlc::{self, VlcEntry};

/// A single decoded (last, run, level) token. Produced by the primary
/// VLC or synthesised by the escape handler.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Token {
    pub last: bool,
    pub run: u8,
    /// Signed level. Zero is only valid for the escape-mode body; a
    /// normal run-length token always has a non-zero level.
    pub level: i16,
}

/// VLC entry for the primary AC table. `Symbol` is a small unsigned
/// id mapped to either a concrete `(last, run, |level|)` triple or
/// the escape marker.
///
/// The caller owns the mapping array so the runtime layout matches the
/// Microsoft binary exactly once the Extractor produces it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Symbol {
    /// Regular (last, run, |level|). `sign` is read as one additional
    /// bit from the stream AFTER the VLC match, not encoded in the
    /// symbol itself.
    RunLevel { last: bool, run: u8, level: u16 },
    /// The escape code — caller must continue with the escape-body
    /// reader. See [`decode_escape_body`].
    Escape,
}

/// A primary AC VLC table: a slice of `VlcEntry<Symbol>` plus the
/// fixed-length widths needed by the escape body. The widths are
/// spec-defined (MPEG-4 §7.4.1.3) and do NOT vary between the two
/// MS-MPEG4 intra-AC variants; only the primary VLC (the `entries`
/// slice) differs.
#[derive(Clone, Copy)]
pub struct AcVlcTable {
    pub entries: &'static [VlcEntry<Symbol>],
    /// Width of the `last` field in escape mode 2 (= 1 bit).
    pub esc_last_bits: u8,
    /// Width of the `run` field in escape mode 2 (= 6 bits for MPEG-4
    /// Part 2; MS-MPEG4v3 uses the same width).
    pub esc_run_bits: u8,
    /// Width of the `level` field in escape mode 2 (= 8 bits signed
    /// for MPEG-4 Part 2; MS-MPEG4v3 uses the same width).
    pub esc_level_bits: u8,
}

impl AcVlcTable {
    /// Inherited MPEG-4 Part 2 §7.4.1.3 escape-mode widths.
    pub const MPEG4_ESC_LAST_BITS: u8 = 1;
    pub const MPEG4_ESC_RUN_BITS: u8 = 6;
    pub const MPEG4_ESC_LEVEL_BITS: u8 = 8;

    /// Placeholder for the MS-MPEG4v3 intra-AC primary VLC. **Empty by
    /// design** so callers that reach the AC walk on a coded block bail
    /// out with the actionable error in [`crate::mb::decode_intra_mb`]
    /// rather than running off the end of the table.
    ///
    /// Use [`AcVlcTable::v3_intra_candidate`] for the candidate table
    /// extracted from `docs/video/msmpeg4/tables/region_05eed0.csv`
    /// (VMA `0x1c25fad0`). That table's role is OPEN per
    /// `docs/video/msmpeg4/spec/99-current-understanding.md` §0.1 row 8
    /// and §9 OPEN-O6 — confirmed-canonical-Huffman code-length data
    /// (Kraft sum exactly 1 over its 64 payload bit-lengths) but the
    /// alphabet's `(last, run, level)` mapping is not fixed by the
    /// extracted bytes alone. The candidate constructor documents one
    /// concrete interpretation; a future spec/audit pass may revise it.
    pub const V3_INTRA_PLACEHOLDER: AcVlcTable = AcVlcTable {
        entries: &[],
        esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
        esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
        esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
    };

    /// Build the candidate v3 intra-AC primary VLC table from the
    /// canonical-Huffman code-length array in
    /// `tables/region_05eed0.csv` (VMA `0x1c25fad0`, 64 payload entries,
    /// Kraft sum 1).
    ///
    /// **Role attribution is OPEN.** Per spec/99 §0.1 row 8 the original
    /// `spec/03 §5.3` claim that this VMA holds an intra-AC primary VLC
    /// is downgraded to a candidate; spec/99 §9 OPEN-O6 lists the same
    /// VMA as a candidate v2-MCBPCY source. The bytes are extraction-
    /// grounded regardless and form a complete prefix code. This
    /// constructor materialises them into the runtime decoder API so
    /// downstream wiring + tests can exercise the canonical-Huffman
    /// walker, but **the (last, run, level) mapping below is the
    /// Implementer's hypothesis**, not an extraction artefact.
    ///
    /// # `(last, run, level)` interpretation (HYPOTHESIS)
    ///
    /// The table's CSV header is `(count_A=64, count_B=1)`. Apply the
    /// v1 inter-DCT kernel partition (`spec/04` §1.3 step 3) verbatim:
    ///
    /// * `idx ∈ [0, count_B] = [0, 1]` → sub-class A: `last = false`.
    /// * `idx ∈ (count_B, count_A) = (1, 64)` → sub-class B: `last = true`.
    /// * `idx == count_A == 64` → ESC sentinel (handled separately by
    ///   the kernel, not by the primary VLC).
    ///
    /// For the (run, level) decomposition we make the most-conservative
    /// choice consistent with `spec/04` §1.3 step 5: the level magnitude
    /// is **always 1** for primary entries (the binary's pri_A array,
    /// which we do **not** have for this region, would supply the real
    /// per-index level — see spec/99 §5.1; without it the candidate
    /// decoder folds all primary entries onto `|level|=1` and lets the
    /// 3-tier ESC body in `decode_escape_body` handle larger levels).
    /// The run is `idx` for sub-A and `idx - (count_B + 1)` for sub-B,
    /// so a stream encoding `(last=0, run=0, level=1)` lines up with the
    /// shortest codeword (idx=0).
    ///
    /// This is **not bit-exact** against ffmpeg / msmpeg4v3 reference
    /// content; it's a structurally-valid candidate that exercises the
    /// AC pipeline end-to-end on synthetic streams. Producing real-file
    /// parity requires either (a) confirmed pri_A / pri_B for the
    /// matching G-descriptor, or (b) the full constructor algorithm at
    /// `0x1c210ee6` from the binary, neither of which is in
    /// `docs/video/msmpeg4/` yet.
    ///
    /// FROM: `docs/video/msmpeg4/tables/region_05eed0.csv`
    /// FROM: `docs/video/msmpeg4/spec/99-current-understanding.md` §0.1 row 8, §9 OPEN-O6
    /// FROM: `docs/video/msmpeg4/spec/03-corrections.md` §5.3
    /// FROM: `docs/video/msmpeg4/spec/04-decoder-kernels.md` §1.3 (partition test)
    pub fn v3_intra_candidate() -> AcVlcTable {
        AcVlcTable {
            entries: candidate_entries_v3_intra(),
            esc_last_bits: Self::MPEG4_ESC_LAST_BITS,
            esc_run_bits: Self::MPEG4_ESC_RUN_BITS,
            esc_level_bits: Self::MPEG4_ESC_LEVEL_BITS,
        }
    }
}

/// Lazily-built `Vec<VlcEntry<Symbol>>` for the v3 intra-AC candidate
/// VLC (region_05eed0). See [`AcVlcTable::v3_intra_candidate`] for the
/// role-attribution caveats.
static V3_INTRA_CANDIDATE_TABLE: std::sync::OnceLock<Vec<VlcEntry<Symbol>>> =
    std::sync::OnceLock::new();

fn candidate_entries_v3_intra() -> &'static [VlcEntry<Symbol>] {
    V3_INTRA_CANDIDATE_TABLE.get_or_init(build_candidate_v3_intra)
}

/// Canonical-Huffman builder for the 64-entry v3 intra-AC candidate
/// table. The algorithm matches `crate::mcbpcy::build_table` (and the
/// reference `spec/04` §1.7 helper `0x1c219351` family):
///
///   1. Filter symbols whose declared bit_length is zero (none in this
///      table — every entry is present).
///   2. Sort by `(bit_length ascending, symbol_index ascending)`.
///   3. Assign canonical codes: `code₀ = 0`,
///      `codeₙ = (codeₙ₋₁ + 1) << (blₙ - blₙ₋₁)`.
///   4. Map each symbol index through
///      [`candidate_index_to_symbol`] to derive the
///      `(last, run, |level|)` triple, then store as [`Symbol::RunLevel`].
fn build_candidate_v3_intra() -> Vec<VlcEntry<Symbol>> {
    use crate::tables_data::{INTRA_AC_V3_CANDIDATE_PARTITION, INTRA_AC_V3_CANDIDATE_RAW};

    let mut symbols: Vec<(u32, u8)> = INTRA_AC_V3_CANDIDATE_RAW
        .iter()
        .enumerate()
        .filter_map(
            |(idx, &(bl, _))| {
                if bl == 0 {
                    None
                } else {
                    Some((bl, idx as u8))
                }
            },
        )
        .collect();
    symbols.sort_by_key(|&(bl, idx)| (bl, idx));

    let partition = INTRA_AC_V3_CANDIDATE_PARTITION as u8;
    let mut entries: Vec<VlcEntry<Symbol>> = Vec::with_capacity(symbols.len());
    let mut code: u32 = 0;
    let mut prev_bl: u32 = 0;
    for (i, &(bl, idx)) in symbols.iter().enumerate() {
        if i == 0 {
            code = 0;
        } else {
            code = (code + 1) << (bl - prev_bl);
        }
        let symbol = candidate_index_to_symbol(idx, partition);
        entries.push(VlcEntry::new(bl as u8, code, symbol));
        prev_bl = bl;
    }
    entries
}

/// Hypothesis-driven `(last, run, |level|)` mapping for the candidate
/// v3 intra-AC alphabet. See [`AcVlcTable::v3_intra_candidate`] for the
/// rationale. `partition = count_B = 1` from the table's header row.
fn candidate_index_to_symbol(idx: u8, partition: u8) -> Symbol {
    let last = idx > partition;
    let run = if last { idx - (partition + 1) } else { idx };
    Symbol::RunLevel {
        last,
        run,
        level: 1,
    }
}

/// Scan-order selection for the AC walk. MS-MPEG4v3 picks this per-block
/// from the DC-predictor gradient (`docs/video/msmpeg4/spec/03-corrections.md`
/// §1.3) — not from a bitstream field. The default scan (zig-zag) is
/// used when AC prediction is disabled for the MB.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scan {
    Zigzag,
    AlternateHorizontal,
    AlternateVertical,
}

impl Scan {
    pub fn table(self) -> &'static [usize; 64] {
        match self {
            Scan::Zigzag => &ZIGZAG,
            Scan::AlternateHorizontal => &ALTERNATE_HORIZONTAL,
            Scan::AlternateVertical => &ALTERNATE_VERTICAL,
        }
    }
}

/// Decode one `(last, run, level)` token from the primary table and
/// (if the primary emits `Escape`) any escape-body extension.
pub fn decode_token(br: &mut BitReader<'_>, table: &AcVlcTable) -> Result<Token> {
    match vlc::decode(br, table.entries)? {
        Symbol::RunLevel { last, run, level } => {
            // 1 sign bit follows the VLC match.
            let sign = br.read_bit()?;
            let signed = if sign { -(level as i32) } else { level as i32 };
            let signed = signed.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            Ok(Token {
                last,
                run,
                level: signed,
            })
        }
        Symbol::Escape => decode_escape_body(br, table),
    }
}

/// Decode the escape-mode body. In the inherited MPEG-4 Part 2 mode
/// the escape is a plain fixed-length `(last, run, level_signed)`
/// triple. MS-MPEG4v3 adds a 2-bit mode selector for extended-level and
/// extended-run variants, but the table-plug-in direction we leave for
/// a subsequent commit — right now the pipeline supports the MPEG-4
/// fixed-length fallback (mode `11` in MS-MPEG4v3 lingo) which is the
/// conservative path used when the other modes have been exhausted.
fn decode_escape_body(br: &mut BitReader<'_>, table: &AcVlcTable) -> Result<Token> {
    let last = br.read_u32(table.esc_last_bits as u32)? != 0;
    let run = br.read_u32(table.esc_run_bits as u32)? as u8;
    let level_raw = br.read_i32(table.esc_level_bits as u32)?;
    // Level 0 after sign-extension is reserved / illegal per spec — a
    // decoder may flag it but for robustness we accept zero and treat
    // it as "no coefficient added" (caller loop bails on last=1).
    let level = level_raw.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
    Ok(Token { last, run, level })
}

/// Decode one full 8×8 AC block and place the coefficients into
/// `block[1..64]` (position 0 is reserved for the DC coefficient, which
/// the caller has already written). `start_pos` is the first AC scan
/// position written; normally 1 (after DC) but may be higher when AC
/// prediction has already consumed a few leading coefficients.
///
/// Returns the number of non-zero AC coefficients decoded.
pub fn decode_intra_ac(
    br: &mut BitReader<'_>,
    block: &mut [i32; 64],
    scan: Scan,
    table: &AcVlcTable,
    start_pos: usize,
) -> Result<u32> {
    if !(1..=64).contains(&start_pos) {
        return Err(Error::invalid(format!(
            "msmpeg4 ac: start_pos {start_pos} out of range [1, 64]"
        )));
    }
    let order = scan.table();
    let mut pos = start_pos;
    let mut written = 0u32;
    loop {
        if pos > 64 {
            return Err(Error::invalid("msmpeg4 ac: block overflow (>64 coeffs)"));
        }
        let tok = decode_token(br, table)?;
        pos += tok.run as usize;
        if pos > 63 {
            return Err(Error::invalid(format!(
                "msmpeg4 ac: scan position {pos} exceeds block (run={}, last={})",
                tok.run, tok.last
            )));
        }
        if tok.level != 0 {
            block[order[pos]] = tok.level as i32;
            written += 1;
        }
        if tok.last {
            return Ok(written);
        }
        pos += 1;
    }
}

/// Full intra-block decode: DC (already supplied), AC walk, dequantise,
/// (no IDCT here — caller does that). `dc` is the post-scaler DC level
/// in the natural raster position `[0]`. `level_start=1` so the AC
/// dequantisation skips the DC.
pub fn decode_intra_block(
    br: &mut BitReader<'_>,
    block: &mut [i32; 64],
    dc: i32,
    scan: Scan,
    table: &AcVlcTable,
    quant: u32,
) -> Result<u32> {
    block[0] = dc;
    let n = decode_intra_ac(br, block, scan, table, 1)?;
    dequantise_h263(block, quant, 1)?;
    Ok(n)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Tiny synthetic VLC for exercising the pipeline. Real MS-MPEG4
    /// v3 tables will plug in later.
    fn toy_table() -> AcVlcTable {
        // 4-entry VLC:
        //   `1`       -> (last=0, run=0, |level|=1)   — "EOB-adjacent runs"
        //   `01`      -> (last=1, run=0, |level|=1)   — terminator with lvl 1
        //   `001`     -> (last=0, run=2, |level|=1)
        //   `000`     -> ESCAPE
        static ENTRIES: &[VlcEntry<Symbol>] = &[
            VlcEntry::new(
                1,
                0b1,
                Symbol::RunLevel {
                    last: false,
                    run: 0,
                    level: 1,
                },
            ),
            VlcEntry::new(
                2,
                0b01,
                Symbol::RunLevel {
                    last: true,
                    run: 0,
                    level: 1,
                },
            ),
            VlcEntry::new(
                3,
                0b001,
                Symbol::RunLevel {
                    last: false,
                    run: 2,
                    level: 1,
                },
            ),
            VlcEntry::new(3, 0b000, Symbol::Escape),
        ];
        AcVlcTable {
            entries: ENTRIES,
            esc_last_bits: AcVlcTable::MPEG4_ESC_LAST_BITS,
            esc_run_bits: AcVlcTable::MPEG4_ESC_RUN_BITS,
            esc_level_bits: AcVlcTable::MPEG4_ESC_LEVEL_BITS,
        }
    }

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
        // Tail padding for the bit-reader so VLC peek never starves.
        out.extend_from_slice(&[0, 0, 0, 0]);
        out
    }

    #[test]
    fn single_token_terminator() {
        // Code `01` (last=1, run=0, level=1), sign bit `0` (positive).
        let t = toy_table();
        let bytes = pack(&[(0b01, 2), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        let tok = decode_token(&mut br, &t).unwrap();
        assert_eq!(
            tok,
            Token {
                last: true,
                run: 0,
                level: 1
            }
        );
    }

    #[test]
    fn token_sign_bit_is_negative() {
        // Code `1` (level=1) then sign=`1` -> level=-1.
        let t = toy_table();
        let bytes = pack(&[(0b1, 1), (1, 1)]);
        let mut br = BitReader::new(&bytes);
        let tok = decode_token(&mut br, &t).unwrap();
        assert_eq!(tok.level, -1);
        assert!(!tok.last);
    }

    #[test]
    fn escape_body_read_as_fixed_length_triple() {
        // Escape prefix `000`, then last=1 (1 bit), run=5 (6 bits),
        // level= -3 as 8-bit signed (= 0xfd).
        let t = toy_table();
        let bytes = pack(&[(0b000, 3), (1, 1), (5, 6), (0xfd, 8)]);
        let mut br = BitReader::new(&bytes);
        let tok = decode_token(&mut br, &t).unwrap();
        assert!(tok.last);
        assert_eq!(tok.run, 5);
        assert_eq!(tok.level, -3);
    }

    #[test]
    fn full_intra_ac_walk_places_coeffs_in_scan_order() {
        // Emit three tokens:
        //   Tok A: code `1`  sign `0` -> (last=0, run=0, level=+1)  @ pos 1 (zz=1)
        //   Tok B: code `001` sign `0` -> (last=0, run=2, level=+1) @ pos 1+1+2 = 4 (zz=9)
        //   Tok C: code `01` sign `1` -> (last=1, run=0, level=-1)  @ pos 4+1 = 5 (zz=2)
        let t = toy_table();
        let bytes = pack(&[(0b1, 1), (0, 1), (0b001, 3), (0, 1), (0b01, 2), (1, 1)]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        let n = decode_intra_ac(&mut br, &mut block, Scan::Zigzag, &t, 1).unwrap();
        assert_eq!(n, 3, "expected 3 non-zero AC coefficients");
        assert_eq!(block[ZIGZAG[1]], 1);
        assert_eq!(block[ZIGZAG[4]], 1);
        assert_eq!(block[ZIGZAG[5]], -1);
        // Everything else untouched.
        assert_eq!(block[0], 0);
        for (i, &v) in block.iter().enumerate() {
            if i != ZIGZAG[1] && i != ZIGZAG[4] && i != ZIGZAG[5] {
                assert_eq!(v, 0, "pos {i} = {v}, expected 0");
            }
        }
    }

    #[test]
    fn scan_order_selector_dispatches_correctly() {
        // Same single-coefficient tok, checked across the 3 scans.
        // Tok: `01` sign `0` -> last=1, run=0, level=+1 @ pos 1.
        let t = toy_table();
        let bytes = pack(&[(0b01, 2), (0, 1)]);
        for scan in [
            Scan::Zigzag,
            Scan::AlternateHorizontal,
            Scan::AlternateVertical,
        ] {
            let mut br = BitReader::new(&bytes);
            let mut block = [0i32; 64];
            let n = decode_intra_ac(&mut br, &mut block, scan, &t, 1).unwrap();
            assert_eq!(n, 1);
            let expected_pos = scan.table()[1];
            assert_eq!(
                block[expected_pos], 1,
                "scan {scan:?} target={expected_pos}"
            );
        }
    }

    #[test]
    fn block_walker_errors_on_overflow_run() {
        // Escape-mode token with run=63 (maximum 6-bit value) from start_pos=1:
        // pos = 1 + 63 = 64 > 63 ⇒ error. Packing values are masked to
        // the declared width, so 63 fits exactly in 6 bits without wrap.
        let t = toy_table();
        let bytes = pack(&[(0b000, 3), (1, 1), (63, 6), (1, 8)]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        assert!(decode_intra_ac(&mut br, &mut block, Scan::Zigzag, &t, 1).is_err());
    }

    #[test]
    fn candidate_v3_intra_table_has_64_entries() {
        // Every entry in region_05eed0.csv has a non-zero bit_length
        // (verified by tables_data::tests::intra_ac_v3_candidate_kraft_sum_is_one),
        // so the candidate table holds the full alphabet of 64 symbols.
        let t = AcVlcTable::v3_intra_candidate();
        assert_eq!(t.entries.len(), 64);
    }

    #[test]
    fn candidate_v3_intra_table_is_prefix_free() {
        // Canonical-Huffman correctness: no entry's code may be a
        // prefix of another's code. This is the runtime equivalent of
        // the build-time Kraft check.
        let t = AcVlcTable::v3_intra_candidate();
        let entries = t.entries;
        for (i, a) in entries.iter().enumerate() {
            for (j, b) in entries.iter().enumerate() {
                if i == j || a.bits == b.bits {
                    continue;
                }
                let (short, long) = if a.bits < b.bits { (a, b) } else { (b, a) };
                let shift = long.bits - short.bits;
                let long_prefix = long.code >> shift;
                assert_ne!(
                    long_prefix, short.code,
                    "candidate intra-AC: shorter code is a prefix of a longer one"
                );
            }
        }
    }

    #[test]
    fn candidate_v3_intra_round_trips_every_symbol() {
        // Encode each table entry's code at the head of a byte-aligned
        // stream, then decode_token must recover the same `(last, run,
        // |level|)` triple. This is the same pattern as MCBPCY's
        // canonical_round_trip_per_symbol.
        let table = AcVlcTable::v3_intra_candidate();
        for (idx_in_table, entry) in table.entries.iter().enumerate() {
            // Pack: VLC code (entry.bits wide) + sign bit `0` (positive).
            let mut acc: u64 = 0;
            let mut bits: u32 = 0;
            acc = (acc << entry.bits) | (entry.code as u64);
            bits += entry.bits as u32;
            // Sign bit (= 0 → level positive).
            acc <<= 1;
            bits += 1;
            let mut out = Vec::new();
            while bits >= 8 {
                let shift = bits - 8;
                out.push(((acc >> shift) & 0xff) as u8);
                acc &= (1u64 << shift) - 1;
                bits -= 8;
            }
            if bits > 0 {
                out.push(((acc << (8 - bits)) & 0xff) as u8);
            }
            out.extend_from_slice(&[0u8; 8]);
            let mut br = BitReader::new(&out);
            let tok = decode_token(&mut br, &table).unwrap();
            // Reverse the candidate hypothesis to predict the expected
            // triple from the symbol index.
            let Symbol::RunLevel { last, run, level } = entry.value else {
                unreachable!("candidate table only stores RunLevel symbols");
            };
            assert_eq!(tok.last, last, "entry {idx_in_table}: last mismatch");
            assert_eq!(tok.run, run, "entry {idx_in_table}: run mismatch");
            assert_eq!(
                tok.level as i32, level as i32,
                "entry {idx_in_table}: |level| mismatch"
            );
        }
    }

    #[test]
    fn candidate_v3_intra_partition_matches_v1_kernel_rule() {
        // Per spec/04 §1.3 step 3 the partition test is `idx > count_B`
        // for sub-class B (last=1). With count_B=1 from the CSV header,
        // exactly idx=0 and idx=1 should be sub-class A (last=0).
        let table = AcVlcTable::v3_intra_candidate();
        for entry in table.entries {
            let Symbol::RunLevel { last, .. } = entry.value else {
                continue;
            };
            // Recover original index from the canonical code-table; the
            // entries don't carry the original index directly so we
            // inspect the partition shape via run vs last.
            // (Two entries with last=0 by hypothesis; the rest are last=1.)
            let _ = last;
        }
        let last0 = table
            .entries
            .iter()
            .filter(|e| matches!(e.value, Symbol::RunLevel { last: false, .. }))
            .count();
        let last1 = table
            .entries
            .iter()
            .filter(|e| matches!(e.value, Symbol::RunLevel { last: true, .. }))
            .count();
        assert_eq!(last0, 2, "expected 2 sub-class-A entries (idx 0..=1)");
        assert_eq!(last1, 62, "expected 62 sub-class-B entries (idx 2..=63)");
    }

    #[test]
    fn placeholder_v3_intra_is_empty() {
        // The placeholder exists specifically as a sentinel: its
        // `entries` slice is empty by design so callers can detect
        // "no real table yet" without reaching for a separate flag.
        // See `AcVlcTable::V3_INTRA_PLACEHOLDER` for the detailed
        // doc-line citations of what's OPEN.
        assert!(AcVlcTable::V3_INTRA_PLACEHOLDER.entries.is_empty());
        assert_eq!(
            AcVlcTable::V3_INTRA_PLACEHOLDER.esc_run_bits,
            AcVlcTable::MPEG4_ESC_RUN_BITS,
        );
    }

    #[test]
    fn decode_intra_block_runs_dequantise() {
        // DC = 512 goes straight to block[0]. AC has one token
        // terminating immediately with (last=1, run=0, level=+1).
        // After dequant with q=5 (odd, parity=1, mag=10, bias=4):
        //   coeff = 1 * 10 + 4 = 14.
        let t = toy_table();
        let bytes = pack(&[(0b01, 2), (0, 1)]);
        let mut br = BitReader::new(&bytes);
        let mut block = [0i32; 64];
        let n = decode_intra_block(&mut br, &mut block, 512, Scan::Zigzag, &t, 5).unwrap();
        assert_eq!(n, 1);
        assert_eq!(block[0], 512, "DC untouched");
        assert_eq!(block[ZIGZAG[1]], 14, "AC dequantised");
    }
}
