//! Per-slot two/three-tier prefix-LUT walker for MS-MPEG4 packed-Huffman
//! sources (round 26/27 — spec/11 + spec/12).
//!
//! The Microsoft binary stores each per-slot canonical-Huffman alphabet
//! as a flat `4 + count * 8`-byte source (a `count: u32-LE` header
//! followed by `count` records of `(code:u32-LE, bl:u32-LE)`).  That
//! source is loaded by the per-slot constructor at `0x1c218cfa` and
//! walked at decode time by the helper at `0x1c219351`.  Per
//! `docs/video/msmpeg4/spec/12-walker-decoder-and-builder.md` §2-§5
//! the walker is a **two-or-three-tier prefix LUT** with a
//! **dynamically-allocated set of sub-tables per tier**:
//!
//! * Default per-tier peek widths `(10, 11, max_bl - 21)` — spec/12 §2
//!   trace at `1c218e8d..1c218ea4`.
//! * Tier 0 always holds exactly one `2^10 = 1024` entry table.  Each
//!   entry is either:
//!   * **Terminal**: `(sym, bl)` with `bl != 0` — consume `bl` bits from
//!     the bitstream and emit `sym`.
//!   * **Refill**: `bl == 0` and `sym` is the **index of the next
//!     sub-table** (within the walker's sub-table pool).  Consume the
//!     current tier's `peek_width` bits and descend.
//! * Tier 1 is a *family* of `2^11` entry tables — one per unique
//!   tier-0 prefix that needs refill (per spec/12 §6 step 3:
//!   *"allocate a new tier 1 if not seen before"*).  Each refill in
//!   tier 0 points to its own tier-1 sub-table.
//! * Tier 2 is similarly a family of `2^(max_bl - 21)` entry tables, one
//!   per unique tier-1 sub-table prefix that needs refill (only when
//!   `max_bl > 21`).
//!
//! This per-prefix sub-table model mirrors the binary's
//! `tier_descriptor` array at `[this+0x18]` (spec/12 §3 / §5): each
//! refill's `sym_or_subidx` is a *separate* descriptor index into a
//! dynamically-grown list of `(peek_width, entry_table_ptr)` records.
//! A single shared tier-1 table cannot work because two codewords with
//! different tier-0 prefixes but identical tier-1 sub-prefixes would
//! collide (e.g. G5 sym 21 `code=0x6/bl=11` and sym 22 `code=0x20/bl=11`
//! both have `sub_code=0` after dropping their respective top-10 bits).
//!
//! The bit ordering is **MSB-first big-endian**: per spec/12 §4.1 the
//! peek helper at `0x1c21923f` merges fresh bytes into the accumulator
//! via `shl reg, 8; or reg, byte`, so the first byte read becomes the
//! high byte.  A byte `0b1011_0000` peeked at width 4 returns `0xB`.
//!
//! For G4 / G5 (`max_bl = 12`) only tier 0 + a small set of tier-1
//! sub-tables are used.  For the 1100-entry MV slots (`max_bl ≈ 21`)
//! tier 2 is never allocated because `max_bl - 21 == 0` — only tiers
//! 0 and 1 fire.
//!
//! This module exposes:
//!
//! * [`Walker`] — the runtime decoder built from a `(code, bl)` array.
//! * [`Walker::build`] — the algorithmic mirror of helper B
//!   (`0x1c218dd8`).  Bit-exact equivalent in the sense of "decodes the
//!   same bitstream to the same symbol", though the internal entry
//!   layout is Rust-native rather than the binary's `(u32, u32)` records.
//! * [`Walker::decode`] — one symbol per call against an
//!   [`oxideav_core::bits::BitReader`] (which already implements MSB-
//!   first big-endian bit-packing per [`oxideav_core::bits`]).
//!
//! See [`crate::vlc::decode`] for the simpler O(n)-per-symbol linear
//! scanner; the walker is the speed-optimised replacement and gives
//! bit-identical results.
//!
//! Provenance: `docs/video/msmpeg4/spec/11-walker-format-resolved.md`,
//! `docs/video/msmpeg4/spec/12-walker-decoder-and-builder.md`. Source
//! binary `mpg4c32.dll` SHA-256
//! `aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099`.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

/// One entry in a sub-table's lookup vector.
///
/// `bl != 0` ⇒ terminal: consume `bl` bits **within this sub-table**
/// (the remaining bit-length below whatever prefix bits were consumed
/// by upstream refills) and emit `sym`.
/// `bl == 0` ⇒ refill: consume the sub-table's `peek_width` bits and
/// descend to the sub-table indexed by `sym`.
#[derive(Clone, Copy, Debug)]
struct Entry {
    /// Terminal: symbol index.  Refill: index of the next sub-table in
    /// [`Walker::sub_tables`].
    sym: u32,
    /// Terminal: per-sub-table bit length to consume.  For a code of
    /// total length `bl` placed at depth `d` after upstream refills
    /// consumed `consumed_above` bits, this field is `bl -
    /// consumed_above`.
    /// Refill: zero (sentinel).
    bl: u32,
}

/// One sub-table of the walker — its peek width (the number of fresh
/// bits to look at to address `entries`) and the entry vector.  This
/// mirrors one record in the binary's `tier_descriptor` array at
/// `[this+0x18]` (spec/12 §3): an `(peek_width:u32, entry_table_ptr:u32)`
/// pair, where the entry table holds `2^peek_width` 8-byte
/// `(sym, bl)` records.
#[derive(Debug)]
struct SubTable {
    peek_width: u32,
    entries: Vec<Entry>,
}

/// MS-MPEG4 per-slot packed-Huffman walker.
///
/// Construct via [`Walker::build`] from the source's `(code, bl)`
/// records and the alphabet's `max_bl`.  Decode with [`Walker::decode`]
/// against a [`BitReader`].
///
/// Internally the walker holds a flat pool of [`SubTable`]s.  Sub-table
/// 0 is always tier 0 (peek width 10).  Sub-tables for tier 1 and tier
/// 2 are appended on demand — one per unique upstream prefix that needs
/// refill, exactly as helper B builds them in the binary
/// (spec/12 §3 / §5: dynamic `tier_descriptor` array).
#[derive(Debug)]
pub struct Walker {
    sub_tables: Vec<SubTable>,
    /// Maximum bit-length over all records — kept for diagnostics.
    pub max_bl: u32,
    /// Alphabet size (== `records.len()`) — kept so callers can
    /// distinguish a returned ESC index from a valid symbol.
    pub count: usize,
}

impl Walker {
    /// Build a walker from `(code, bit_length)` records using the default
    /// tier widths `(10, 11, max_bl - 21)` per spec/12 §2.
    ///
    /// `code` is the canonical Huffman bit pattern packed into the low
    /// `bl` bits of a u32, MSB-first within those bits (e.g. a 4-bit
    /// code `0b1011` is stored as `code = 0b1011`).
    ///
    /// Records with `bl == 0` are treated as hole sentinels (helper A's
    /// `0xFFFFFFFF → (0, 0)` branch per spec/11 §3) and skipped during
    /// the placement loop.
    pub fn build(records: &[(u32, u32)]) -> Result<Self> {
        let max_bl = records.iter().map(|&(_, bl)| bl).max().unwrap_or(0);
        if max_bl == 0 {
            return Err(Error::invalid(
                "msmpeg4 walker: empty alphabet (no records with bl > 0)",
            ));
        }
        if max_bl > 32 {
            return Err(Error::invalid(format!(
                "msmpeg4 walker: max_bl {max_bl} > 32 — record format violation"
            )));
        }

        // Compute tier widths per spec/12 §2 default config.
        let tier_widths = compute_tier_widths(max_bl);

        // Seed sub-table 0 = tier 0.  Its `peek_width` is the tier-0
        // width (10).  Every entry starts as `(sym=0, bl=0)`, which
        // is the "uninitialised" sentinel — both fields zero.  A
        // genuine refill into sub-table 0 is impossible so this can't
        // be confused with a valid refill (refill `sym` is always
        // ≥ 1).  A genuine bl=0 terminal is also impossible (the
        // record would have been filtered as a hole).
        let mut sub_tables: Vec<SubTable> = Vec::with_capacity(1);
        sub_tables.push(SubTable {
            peek_width: tier_widths[0],
            entries: vec![Entry { sym: 0, bl: 0 }; 1usize << tier_widths[0]],
        });

        // Place each record.  Refills allocate fresh sub-tables on
        // demand and store the new sub-table's index in the parent's
        // refill `sym` field — mirroring spec/12 §6 step 3 "(or
        // allocate a new tier 1 if not seen before)".
        for (sym_idx, &(code, bl)) in records.iter().enumerate() {
            if bl == 0 {
                // Hole sentinel — never decoded, no codeword space.
                continue;
            }
            place_code(&mut sub_tables, code, bl, sym_idx as u32, &tier_widths)?;
        }

        Ok(Self {
            sub_tables,
            max_bl,
            count: records.len(),
        })
    }

    /// Decode one symbol from `br`.  Returns the symbol index in
    /// `0..count`.
    pub fn decode(&self, br: &mut BitReader<'_>) -> Result<u32> {
        // Always start at sub-table 0 (= tier 0).
        let mut sub_idx = 0usize;
        loop {
            let sub = self.sub_tables.get(sub_idx).ok_or_else(|| {
                Error::invalid(format!(
                    "msmpeg4 walker: refill into missing sub-table {sub_idx}"
                ))
            })?;
            let w = sub.peek_width;
            let remaining = br.bits_remaining() as u32;
            // Peek up to w bits.  When fewer than w bits remain, pad
            // with zeros on the right (the binary's peek helper does
            // the same via `eof_flag`); the canonical Huffman matcher
            // will still resolve the correct prefix because the placed
            // entries already cover every w-bit suffix of any valid
            // codeword.
            let peek_bits = w.min(remaining);
            let raw = if peek_bits == 0 {
                0u32
            } else {
                br.peek_u32(peek_bits)? << (w - peek_bits)
            };
            let entry = &sub.entries[raw as usize];
            if entry.bl != 0 {
                // Terminal: consume bl bits and return.
                br.consume(entry.bl)?;
                return Ok(entry.sym);
            }
            // Refill: consume the full peek_width and descend.
            if peek_bits < w {
                return Err(Error::invalid(
                    "msmpeg4 walker: refill needed but bitstream exhausted",
                ));
            }
            br.consume(w)?;
            sub_idx = entry.sym as usize;
        }
    }
}

/// Default tier widths for `max_bl`, per spec/12 §2.
fn compute_tier_widths(max_bl: u32) -> Vec<u32> {
    let mut widths = vec![10u32];
    if max_bl > 10 {
        widths.push(11);
    }
    if max_bl > 21 {
        widths.push(max_bl - 21);
    }
    widths
}

/// Place one codeword into the walker's sub-table tree.  Mirrors the
/// algorithm described in spec/12 §6 "to build the walker for a slot",
/// with sub-tables allocated on demand:
///
/// * If `bl <= tier0_width` (= 10): place a terminal at every tier-0
///   slot whose top `bl` bits equal `code`.  Stored `entry.bl = bl`
///   (the full code length — tier 0 hasn't consumed anything else).
/// * Else if `bl <= tier0_width + tier1_width` (= 21): if the matching
///   tier-0 slot is not yet a refill, allocate a fresh tier-1 sub-table
///   and store its index in the tier-0 entry's `sym` field.  Then
///   place the terminal in that tier-1 sub-table at the address given
///   by the next `t1` bits of the code, with `entry.bl = bl - tier0_width`.
/// * Else (only when `max_bl > 21`): cascade — if the matching tier-0
///   slot has no tier-1 sub-table yet, allocate one; if that tier-1
///   sub-table's matching slot has no tier-2 sub-table yet, allocate
///   one; place the terminal in the tier-2 sub-table.
fn place_code(
    sub_tables: &mut Vec<SubTable>,
    code: u32,
    bl: u32,
    sym: u32,
    tier_widths: &[u32],
) -> Result<()> {
    let t0 = tier_widths[0];
    if bl <= t0 {
        // Terminal in tier 0 at every slot whose top `bl` bits == code.
        let prefix = code << (t0 - bl);
        let span = 1u32 << (t0 - bl);
        for i in 0..span {
            let idx = (prefix | i) as usize;
            let existing = sub_tables[0].entries[idx];
            if existing.bl != 0 {
                return Err(Error::invalid(format!(
                    "msmpeg4 walker: tier-0 slot {idx} double-terminal (sym {sym}, code 0x{code:x}/{bl}b)"
                )));
            }
            // Refill marker has `bl == 0` and `sym != 0`.
            if existing.sym != 0 {
                return Err(Error::invalid(format!(
                    "msmpeg4 walker: tier-0 slot {idx} already a refill, can't place terminal sym {sym}"
                )));
            }
            sub_tables[0].entries[idx] = Entry { sym, bl };
        }
        return Ok(());
    }

    // Need tier 1 (always present when max_bl > 10).
    if tier_widths.len() < 2 {
        return Err(Error::invalid(format!(
            "msmpeg4 walker: bl {bl} exceeds tier-0 width {t0} but no tier 1 configured"
        )));
    }
    let t1 = tier_widths[1];
    let tier0_prefix = (code >> (bl - t0)) as usize;

    if bl <= t0 + t1 {
        // Locate (or allocate) the tier-1 sub-table for this tier-0
        // prefix.
        let tier1_idx = ensure_child_subtable(sub_tables, 0, tier0_prefix, t1, code, bl)?;

        // Place terminal in the tier-1 sub-table.  Address: next `t1`
        // bits of the code below the tier-0 prefix.  The terminal
        // consumes `bl - t0` bits at this tier (per spec/12 §3 — the
        // per-tier remaining length).
        let bits_under_t0 = bl - t0;
        let sub_code = code & ((1u32 << bits_under_t0) - 1);
        let slot_prefix = sub_code << (t1 - bits_under_t0);
        let span = 1u32 << (t1 - bits_under_t0);
        for i in 0..span {
            let idx = (slot_prefix | i) as usize;
            let existing = sub_tables[tier1_idx].entries[idx];
            if existing.bl != 0 {
                return Err(Error::invalid(format!(
                    "msmpeg4 walker: tier-1 slot {idx} double-terminal (sym {sym}, code 0x{code:x}/{bl}b)"
                )));
            }
            if existing.sym != 0 {
                return Err(Error::invalid(format!(
                    "msmpeg4 walker: tier-1 slot {idx} already a refill, can't place terminal sym {sym}"
                )));
            }
            sub_tables[tier1_idx].entries[idx] = Entry {
                sym,
                bl: bits_under_t0,
            };
        }
        return Ok(());
    }

    // Need tier 2.
    if tier_widths.len() < 3 {
        return Err(Error::invalid(format!(
            "msmpeg4 walker: bl {bl} exceeds tier-0+tier-1 width {} but no tier 2 configured",
            t0 + t1
        )));
    }
    let t2 = tier_widths[2];

    // Allocate tier-1 sub-table for this tier-0 prefix if needed.
    let tier1_idx = ensure_child_subtable(sub_tables, 0, tier0_prefix, t1, code, bl)?;

    // Tier-1 address is the next t1 bits of the code below the tier-0
    // prefix.
    let tier1_prefix = ((code >> (bl - t0 - t1)) & ((1u32 << t1) - 1)) as usize;

    // Allocate tier-2 sub-table for this tier-1 prefix if needed.
    let tier2_idx = ensure_child_subtable(sub_tables, tier1_idx, tier1_prefix, t2, code, bl)?;

    // Tier 2 terminal — per-tier remaining bit length is bl - t0 - t1.
    let bits_under_t01 = bl - t0 - t1;
    let sub_code = code & ((1u32 << bits_under_t01) - 1);
    let slot_prefix = sub_code << (t2 - bits_under_t01);
    let span = 1u32 << (t2 - bits_under_t01);
    for i in 0..span {
        let idx = (slot_prefix | i) as usize;
        let existing = sub_tables[tier2_idx].entries[idx];
        if existing.bl != 0 {
            return Err(Error::invalid(format!(
                "msmpeg4 walker: tier-2 slot {idx} double-terminal (sym {sym}, code 0x{code:x}/{bl}b)"
            )));
        }
        if existing.sym != 0 {
            return Err(Error::invalid(format!(
                "msmpeg4 walker: tier-2 slot {idx} already a refill, can't place terminal sym {sym}"
            )));
        }
        sub_tables[tier2_idx].entries[idx] = Entry {
            sym,
            bl: bits_under_t01,
        };
    }
    Ok(())
}

/// Look up the parent's slot at `parent_slot` and either return the
/// already-allocated child sub-table index (if the slot is already a
/// refill) or allocate a fresh child sub-table with `child_width` and
/// wire the parent slot to it.
///
/// Per spec/12 §6 step 3: *"or allocate a new tier 1 if not seen
/// before"*.  This is the dynamic-allocation step that matches the
/// binary's `tier_descriptor` growth at `[this+0x18]` (spec/12 §3 / §5).
fn ensure_child_subtable(
    sub_tables: &mut Vec<SubTable>,
    parent_idx: usize,
    parent_slot: usize,
    child_width: u32,
    code: u32,
    bl: u32,
) -> Result<usize> {
    let existing = sub_tables[parent_idx].entries[parent_slot];
    if existing.bl != 0 {
        // Slot is already a terminal — can't refill through it.
        return Err(Error::invalid(format!(
            "msmpeg4 walker: parent sub-table {parent_idx} slot {parent_slot} already terminal but bl {bl} code 0x{code:x} wants to refill"
        )));
    }
    if existing.sym != 0 {
        // Slot is already a refill into a previously-allocated child.
        return Ok(existing.sym as usize);
    }
    // Allocate a fresh child sub-table.
    let new_idx = sub_tables.len();
    sub_tables.push(SubTable {
        peek_width: child_width,
        entries: vec![Entry { sym: 0, bl: 0 }; 1usize << child_width],
    });
    sub_tables[parent_idx].entries[parent_slot] = Entry {
        sym: new_idx as u32,
        bl: 0,
    };
    Ok(new_idx)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pack a list of (value, width-in-bits) tuples MSB-first into a
    /// byte buffer with a few trailing zero bytes (tail padding for the
    /// bit-reader so it never starves on the last partial byte).
    fn pack(fields: &[(u32, u32)]) -> Vec<u8> {
        let mut out: Vec<u8> = Vec::new();
        let mut acc: u64 = 0;
        let mut bits: u32 = 0;
        for &(v, w) in fields {
            let mask = if w == 32 { u32::MAX } else { (1u32 << w) - 1 };
            acc = (acc << w) | ((v & mask) as u64);
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
        out.extend_from_slice(&[0u8; 4]);
        out
    }

    #[test]
    fn tier_widths_default_max_bl_12() {
        // G4/G5 case: max_bl = 12 → tiers (10, 11). Tier 2 not allocated.
        assert_eq!(compute_tier_widths(12), vec![10, 11]);
    }

    #[test]
    fn tier_widths_default_max_bl_8() {
        // Small alphabet — only tier 0 is needed.
        assert_eq!(compute_tier_widths(8), vec![10]);
    }

    #[test]
    fn tier_widths_default_max_bl_22() {
        // 1100-entry MV alphabet edge: 22 → tiers (10, 11, 1).
        assert_eq!(compute_tier_widths(22), vec![10, 11, 1]);
    }

    #[test]
    fn tier_widths_default_max_bl_21() {
        // 21 → only tiers (10, 11) needed; tier 2 width would be 0.
        assert_eq!(compute_tier_widths(21), vec![10, 11]);
    }

    #[test]
    fn build_and_decode_simple_4_entry() {
        // 4-symbol alphabet:
        //   sym 0: code `1`     bl 1
        //   sym 1: code `01`    bl 2
        //   sym 2: code `001`   bl 3
        //   sym 3: code `000`   bl 3
        let records = [(0b1u32, 1u32), (0b01, 2), (0b001, 3), (0b000, 3)];
        let walker = Walker::build(&records).expect("build");
        let bytes = pack(&[(0b1, 1), (0b01, 2), (0b001, 3), (0b000, 3)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(walker.decode(&mut br).unwrap(), 0);
        assert_eq!(walker.decode(&mut br).unwrap(), 1);
        assert_eq!(walker.decode(&mut br).unwrap(), 2);
        assert_eq!(walker.decode(&mut br).unwrap(), 3);
    }

    #[test]
    fn build_handles_hole_sentinels() {
        // Helper A's hole records carry (code=0, bl=0) — they consume
        // no codeword space and never decode.  Walker::build must skip
        // them without confusing a valid sym 0 of code `1`/bl 1.
        let records = [(0b1u32, 1u32), (0, 0), (0b0, 1)];
        let walker = Walker::build(&records).expect("build with hole");
        let bytes = pack(&[(0b1, 1), (0b0, 1)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(walker.decode(&mut br).unwrap(), 0);
        // sym 1 is the hole; sym 2 at code 0 / bl 1.
        assert_eq!(walker.decode(&mut br).unwrap(), 2);
    }

    #[test]
    fn build_rejects_empty_alphabet() {
        assert!(Walker::build(&[]).is_err());
        // All-hole records also invalid (max_bl = 0).
        assert!(Walker::build(&[(0u32, 0u32), (0, 0)]).is_err());
    }

    #[test]
    fn decode_long_code_uses_tier_1() {
        // Build an alphabet with one 12-bit code (forces tier-1 use).
        // Sym 0: 1-bit code `0` (most of the prefix space).
        // Sym 1: 12-bit code `1111_1111_1111`.
        let records = [(0b0u32, 1u32), (0b1111_1111_1111u32, 12u32)];
        let walker = Walker::build(&records).expect("build");
        // Verify both decode correctly.
        let bytes = pack(&[(0, 1), (0b1111_1111_1111, 12)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(walker.decode(&mut br).unwrap(), 0);
        assert_eq!(walker.decode(&mut br).unwrap(), 1);
    }

    /// The build.rs `*_PRIMARY_RAW` constants carry `(bit_length,
    /// code_value)` pairs (per the comment in build.rs:1200 + 1243).
    /// [`Walker::build`] expects `(code, bl)` per spec/11 §4.  This
    /// helper swaps the order so we can feed the existing
    /// G4/G5 tables directly.
    fn swap_to_code_bl(raw: &[(u32, u32)]) -> Vec<(u32, u32)> {
        raw.iter().map(|&(bl, code)| (code, bl)).collect()
    }

    #[test]
    fn distinct_tier0_prefixes_get_separate_tier1_subtables() {
        // Regression for the round-31 phase-2 bug: two bl=11 codewords
        // with different tier-0 prefixes but identical tier-1
        // sub-prefixes must NOT collide.  In the broken single-shared-
        // tier-1 model, codewords like G5 sym 21 (`code=0x6/bl=11`,
        // tier-0 prefix `0x3`) and sym 22 (`code=0x20/bl=11`, tier-0
        // prefix `0x10`) — both with `sub_code = 0` after dropping
        // their respective top 10 bits — would clash on tier-1 slot 0.
        // Per spec/12 §6 step 3 ("allocate a new tier 1 if not seen
        // before") each unique tier-0 prefix must own its own tier-1
        // sub-table.
        let records = [
            (0b00000000110u32, 11u32), // sym 0: tier-0 prefix 0x3, sub_code=0
            (0b10000000000u32, 11u32), // sym 1: tier-0 prefix 0x10, sub_code=0
        ];
        let walker = Walker::build(&records).expect("two-prefix walker");
        for (sym, &(code, bl)) in records.iter().enumerate() {
            let bytes = pack(&[(code, bl)]);
            let mut br = BitReader::new(&bytes);
            assert_eq!(walker.decode(&mut br).unwrap(), sym as u32);
        }
    }

    #[test]
    fn cross_check_against_real_g5_packed_source() {
        // Build the walker from the G5 PRIMARY canonical-Huffman VLC
        // and decode every symbol's bit pattern + verify it lands on
        // its own index.  This is the same property that the linear
        // scanner [`crate::vlc::decode`] guarantees; the walker MUST
        // produce identical outputs (spec/12 §6 "bit-exact equivalent").
        use crate::tables_data::G5_PRIMARY_RAW;
        let records = swap_to_code_bl(G5_PRIMARY_RAW);
        let walker = Walker::build(&records).expect("G5 walker");
        for (sym, &(code, bl)) in records.iter().enumerate() {
            if bl == 0 {
                continue;
            }
            let bytes = pack(&[(code, bl)]);
            let mut br = BitReader::new(&bytes);
            let decoded = walker.decode(&mut br).expect("decode");
            assert_eq!(
                decoded, sym as u32,
                "G5 sym {sym}: code 0x{code:x}/{bl}b decoded to {decoded}"
            );
        }
    }

    #[test]
    fn cross_check_against_real_g4_packed_source() {
        // Same property for G4 (chroma + all-inter alphabet).
        use crate::tables_data::G4_PRIMARY_RAW;
        let records = swap_to_code_bl(G4_PRIMARY_RAW);
        let walker = Walker::build(&records).expect("G4 walker");
        for (sym, &(code, bl)) in records.iter().enumerate() {
            if bl == 0 {
                continue;
            }
            let bytes = pack(&[(code, bl)]);
            let mut br = BitReader::new(&bytes);
            let decoded = walker.decode(&mut br).expect("decode");
            assert_eq!(
                decoded, sym as u32,
                "G4 sym {sym}: code 0x{code:x}/{bl}b decoded to {decoded}"
            );
        }
    }
}
