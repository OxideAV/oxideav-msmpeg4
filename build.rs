//! Build script — parses clean-room extracted MS-MPEG4 tables from
//! `docs/video/msmpeg4/tables/*.csv` and emits them as compile-time Rust
//! constants. **The Implementer must NOT retype these numbers manually**;
//! they come straight from the CSV produced by the Extractor session
//! from `reference/binaries/wmpcdcs8-2001/mpg4c32.dll` SHA-256
//! `aedb4cf3...b3c099`.
//!
//! Tables emitted (all under `$OUT_DIR/`, included via `src/tables_data.rs`):
//!
//! * `mcbpcy_v3.rs` — v3 joint-MCBPCY VLC source (`region_05eac8.csv`,
//!   spec/99 §3.1 / §8.1). 128 `(bit_length, code_value)` pairs.
//! * `mv_v3.rs` — v3 joint (X, Y) MV VLC source default variant
//!   (`region_05bfc0.csv`, spec/06 §2.1, VMA `0x1c25cbc0`). 1100
//!   `(bit_length, code_value)` pairs; index 1099 is ESC.
//! * `mv_lut_v3.rs` — MVDx/MVDy byte LUTs default variant (from
//!   `region_05e228.hex` / `region_05e678.hex`, VMAs `0x1c25ee28` /
//!   `0x1c25f278`). 1104 bytes each; only indices 0..1099 are read by
//!   the decoder.
//! * `intra_ac_v3.rs` — candidate v3 intra AC TCOEF run/level/last
//!   primary VLC source (`region_05eed0.csv`, VMA `0x1c25fad0`, file
//!   offset `0x5eed0`). 64 `(bit_length, code_value)` payload entries
//!   plus a `(count_A=64, count_B=1)` header row. Kraft sum over the
//!   64 bit-lengths is exactly 1, confirming the table is a complete
//!   canonical-Huffman prefix code. Per spec/99 §0.1 row 8 the role
//!   attribution of this region is **OPEN** (candidate v2 MCBPCY
//!   source vs intra-AC TCOEF candidate per spec/03 §5.3); the bytes
//!   are extracted reproducibly regardless and the Implementer wires
//!   them through the same canonical-Huffman builder used for MCBPCY,
//!   leaving the (last, run, level) symbol decoding behind a guarded
//!   constructor (`AcVlcTable::v3_intra_candidate`) so callers opt in
//!   explicitly.
//!
//! The CSV column naming is historically mis-labelled: the `symbol_dec`
//! column holds the `bit_length`, the `bit_length` column holds the
//! `code_value`. See `docs/video/msmpeg4/spec/99-current-understanding.md`
//! §8.1 for the full provenance chain. Canonical Huffman codes are
//! derived from the `bit_length` array alone, so the `code_value` column
//! is carried through but not consumed by the runtime decoder.

use std::env;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};

/// Authoritative per-G-family `(count_A, count_B)` values lifted from
/// the `mb_mv_struct_init` constructor body at VMA `0x1c210643` per
/// `docs/video/msmpeg4/spec/15-count-ab-per-g-family.md` §2.1 / §3.
///
/// Each tuple is `(count_A, count_B)` for descriptors G0 through G5 in
/// index order. `count_A` is the alphabet size (= the ESC sentinel
/// index); `count_B` is the sub-A / sub-B partition boundary
/// (`idx ≤ count_B` ⇒ sub-A / `last = 0`; `count_B < idx < count_A` ⇒
/// sub-B / `last = 1`; `idx == count_A` ⇒ ESC).
///
/// These six pairs are the single source of truth the build script
/// cross-checks every per-G consumer against:
///
/// * `emit_g_enum` for G0..G3 (per-G enum CSV row counts must equal
///   `count_A` and the implied sub-A length must equal `count_B + 1`).
/// * `emit_g_descriptor_cluster` for G4/G5 (the byte slices' lengths
///   are sized by `count_A`; the sub-A/B partition pivot index is
///   `count_B`).
///
/// Per spec/15 §7 ("All six G-families CONSISTENT") this table is
/// cross-checked against eight independent prior `spec/*.md` documents
/// (02, 03, 04, 05, 08, 09, 13, 14) and the per-G `tables/region_*g{0..3}_enum.{csv,meta}`
/// extractor outputs — every citation agrees. If the binary's
/// constructor immediates ever drift from these values, this constant
/// (and every consumer of it) needs to be re-derived from a fresh
/// disassembly per spec/15 §2; do NOT edit the numbers in-place
/// without re-running that workflow.
///
/// FROM: `docs/video/msmpeg4/spec/15-count-ab-per-g-family.md` §3
/// (table at lines 181-187) and §2.1 (constructor disassembly at
/// VMAs `1c210653..1c21086c`).
const G_COUNTS_SPEC15: [(u32, u32); 6] = [
    (168, 98),  // G0 — chroma + all-inter (class 1); constructor 1c210653 / 1c21065d
    (185, 118), // G1 — intra-luma (class 1); constructor 1c2106b1 / 1c2106bb
    (148, 80),  // G2 — chroma + all-inter (class 0); constructor 1c210710 / 1c21071a
    (132, 84),  // G3 — intra-luma (class 0); constructor 1c210778 / 1c210788
    (102, 57),  // G4 — MPEG-4 Part 2 Inter LMAX baseline; constructor 1c210810 / 1c21081a
    (102, 66),  // G5 — MPEG-4 Part 2 Intra LMAX baseline; constructor 1c210862 / 1c21086c
];

/// Sub-class partition arithmetic per spec/03 §4.4 / spec/15 §5.2:
/// `sub_A = count_B + 1`; `sub_B = count_A - count_B - 1`. Returns the
/// `(sub_A, sub_B)` pair for the supplied G-family index.
fn g_subclass_sizes(g: usize) -> (u32, u32) {
    let (count_a, count_b) = G_COUNTS_SPEC15[g];
    (count_b + 1, count_a - count_b - 1)
}

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let tables_dir = manifest_dir.join("tables");
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Emit the authoritative spec/15 §3 per-G (count_A, count_B) table
    // as a runtime constant so downstream tests and any future per-G
    // dispatch logic can re-cite it instead of hardcoding six pairs.
    // This is the single source of truth the rest of build.rs
    // cross-checks against (see `emit_g_enum` and
    // `emit_g_descriptor_cluster`).
    emit_g_counts_spec15(&out_dir.join("g_counts.rs"));

    // MCBPCY table — 05eac8.
    let mcbpcy_csv = tables_dir.join("region_05eac8.csv");
    println!("cargo:rerun-if-changed={}", mcbpcy_csv.display());
    emit_mcbpcy_v3(&mcbpcy_csv, &out_dir.join("mcbpcy_v3.rs"));

    // v3 joint-MV VLC default variant — 05bfc0.
    let mv_csv = tables_dir.join("region_05bfc0.csv");
    println!("cargo:rerun-if-changed={}", mv_csv.display());
    emit_mv_v3(&mv_csv, &out_dir.join("mv_v3.rs"));

    // MVDx / MVDy byte LUTs default variant — 05e228 + 05e678.
    let mvdx_hex = tables_dir.join("region_05e228.hex");
    let mvdy_hex = tables_dir.join("region_05e678.hex");
    println!("cargo:rerun-if-changed={}", mvdx_hex.display());
    println!("cargo:rerun-if-changed={}", mvdy_hex.display());
    emit_mv_byte_lut_v3(&mvdx_hex, &mvdy_hex, &out_dir.join("mv_lut_v3.rs"));

    // Intra AC candidate primary VLC — 05eed0 (VMA 0x1c25fad0).
    // 64-entry canonical Huffman; role attribution OPEN per spec/99 §0.1.
    let ac_csv = tables_dir.join("region_05eed0.csv");
    println!("cargo:rerun-if-changed={}", ac_csv.display());
    emit_intra_ac_v3(&ac_csv, &out_dir.join("intra_ac_v3.rs"));

    // v1/v2 MCBPC tables — 053140 (combined LUT region 0x53140..0x53640;
    // 1024 bytes are v1 MCBPC 9-bit LUT, then 256 bytes are v2 MCBPC
    // 7-bit LUT). Per spec/07 §1.3 / §2.3 v1 MCBPC at VMA 0x1c253d40
    // (max-bitlen 9, 21 entries) and v2 MCBPC at VMA 0x1c254140
    // (= 0x1c253d40 + 0x400, max-bitlen 7, 8 entries). Both are read
    // by helper 0x1c215811 which interprets the source as a packed
    // tiered-Huffman LUT: each 2-byte record is (bit_length, symbol).
    let mcbpc_hex = tables_dir.join("region_053140.hex");
    println!("cargo:rerun-if-changed={}", mcbpc_hex.display());
    emit_mcbpc_v1_v2(&mcbpc_hex, &out_dir.join("mcbpc_v1_v2.rs"));

    // v1/v2 shared CBPY table — 053640 (4096 bytes at file 0x53640 / VMA
    // 0x1c254240). Per spec/07 §1.3 / §2.3 the shared 16-entry CBPY VLC
    // (max-bitlen 6) is consumed by helper 0x1c215811 from both v1's
    // 0x1c2171c7 and v2's 0x1c21729c MB-header decoders. The first 64
    // entries (= 6-bit pre-expanded LUT, 128 bytes) hold the CBPY table
    // itself; the trailing 3968 bytes are sibling LUTs unrelated to CBPY
    // (different pre-expansion widths per the per-pair stride observed
    // in `tables/region_053640.hex`).
    //
    // Round 75 (2026-05-18): the binary's CBPY pre-expanded LUT is now
    // parsed in build.rs and cross-checked byte-for-byte against the
    // hand-derived `CBPY_INTRA_TABLE` (which comes from H.263 Table 8 /
    // MPEG-4 Part 2 Table B-6, public standards). This closes the v1/v2
    // CBPY provenance gap: the H.263-derived (bl, code, sym) tuples
    // match the binary's pre-expanded LUT exactly, and a build break
    // fires if the docs ever drift. The binary additionally fills the
    // 6-bit-window slots `000000` and `000001` with sentinel symbols
    // (0x10, 0x11) — these are not part of the 16-symbol CBPY alphabet
    // and mark "invalid bitstream" outcomes; the runtime decoder rejects
    // syms ≥ 16 via the existing range check in `decode_cbpy_with_wrap`.
    let cbpy_hex = tables_dir.join("region_053640.hex");
    println!("cargo:rerun-if-changed={}", cbpy_hex.display());
    emit_cbpy_v1_v2(&cbpy_hex, &out_dir.join("cbpy_v1_v2.rs"));

    // v1/v2 per-component MV VLC table — 04ed30_full (16384 bytes at file
    // offset 0x4ed30 / VMA 0x1c24f930). Per spec/06 §2.3 / spec/07 §3 the
    // helper 0x1c215811 is invoked with max-bitlen 13, so the LUT is
    // 2^13 = 8192 halfword entries × 2 bytes = 16384 bytes. The
    // docs/video/msmpeg4/tables/region_04ed30.hex extraction captured
    // only the first 4096 bytes (1/4 of the LUT — see the .meta on
    // tables/region_04ed30_full.meta). We use the FULL 16384-byte copy
    // here. The alphabet is 65 symbols (raw indices 0..64); after the
    // bias subtraction (raw - 0x20) the signed MVD residual lands in
    // [-32, +32].
    let mv_v1v2_hex = tables_dir.join("region_04ed30_full.hex");
    println!("cargo:rerun-if-changed={}", mv_v1v2_hex.display());
    emit_mv_v1_v2(&mv_v1v2_hex, &out_dir.join("mv_v1_v2.rs"));

    // G-descriptor pri_A / pri_B cluster — 0569c0 (3800 bytes at file
    // offset 0x569c0 / VMA 0x1c2575c0). Per spec/99 §10.3 this region
    // holds all six G-descriptor pri_A (byte-per-symbol |level|) and
    // pri_B (u32-per-symbol run) arrays. Round 18 wired G4 (chroma +
    // all-inter, default for v1/v2 streams; selected on v3 when
    // [esi+0xad0]=2) — count_A=102, count_B=57, sub-A 58 entries
    // (last=0), sub-B 44 entries (last=1) — and G5 pri_A
    // (count_A=102, count_B=66). G5 pri_B was missing because it lives
    // in a 408-byte gap immediately after the cluster region ends.
    //
    // Round 19 fills that gap: `tables/region_057898.hex` carries the
    // 408 bytes (102 × u32-LE) at file 0x57898..0x57a30 / VMA
    // 0x1c258498. emit_g_descriptor_cluster now consumes BOTH region
    // dumps and emits a complete G5 pri_B (run-count low byte per
    // symbol) alongside G4 / G5 pri_A and G4 pri_B.
    let g_cluster_hex = tables_dir.join("region_0569c0.hex");
    let g5_pri_b_hex = tables_dir.join("region_057898.hex");
    println!("cargo:rerun-if-changed={}", g_cluster_hex.display());
    println!("cargo:rerun-if-changed={}", g5_pri_b_hex.display());
    emit_g_descriptor_cluster(
        &g_cluster_hex,
        &g5_pri_b_hex,
        &out_dir.join("g_descriptors.rs"),
    );

    // G4 / G5 packed-Huffman PRIMARY VLC sources (round 26 — spec/11
    // §3 / §4). Each source is `4 + 103 * 8` bytes:
    //   * `count: u32-LE = 103` (alphabet size including ESC)
    //   * 103 records of `(a:u32-LE, b:u32-LE)` where
    //         a = code_value (state byte; not the canonical bit-pattern)
    //         b = bit_length (0..12 for G4/G5)
    //
    // The canonical Huffman bit-pattern is reconstructed from the
    // bit-length array alone (see `emit_packed_huffman_primary` below).
    // Kraft sum over 103 bit-lengths = 1 - 2/1024 (one bl=9 codeword
    // reserved for ESC); the ESC entry is at array index 102 with a
    // valid bit_length, and decoders treat the recovered idx == 102 as
    // ESC per spec/11 §7 item 4.
    //
    // FROM: docs/video/msmpeg4/spec/11-walker-format-resolved.md §3-§4
    // FROM: reference/binaries/wmpcdcs8-2001/mpg4c32.dll @ file 0x58e38
    //       / 0x59178; SHA-256 aedb4cf3...b3c099.
    let g4_packed_hex = tables_dir.join("region_058e38_full.hex");
    let g5_packed_hex = tables_dir.join("region_059178_full.hex");
    println!("cargo:rerun-if-changed={}", g4_packed_hex.display());
    println!("cargo:rerun-if-changed={}", g5_packed_hex.display());
    emit_packed_huffman_primary(
        &g4_packed_hex,
        &out_dir.join("g4_primary.rs"),
        "G4",
        0x58e38,
        0x1c258e38,
    );
    emit_packed_huffman_primary(
        &g5_packed_hex,
        &out_dir.join("g5_primary.rs"),
        "G5",
        0x59178,
        0x1c259d78,
    );

    // G0..G3 packed-Huffman PRIMARY VLC sources (round 234 — spec/11
    // §5 row 1-4). Each source has the same `(code, bl)` record layout
    // as G4/G5 but a different alphabet size (count = count_A + 1, with
    // the +1 being the ESC sentinel at idx == count_A per spec/09 §2):
    //
    //   G0  region_057a30 (VMA 0x1c258630)  count 169 (= 168 + 1 ESC)
    //   G1  region_057f80 (VMA 0x1c258b80)  count 186 (= 185 + 1 ESC)
    //   G2  region_058558 (VMA 0x1c259158)  count 149 (= 148 + 1 ESC)
    //   G3  region_058a08 (VMA 0x1c259608)  count 133 (= 132 + 1 ESC)
    //
    // Unlike G4/G5 (which reserve one bl=9 codeword for ESC, Kraft sum
    // 0.998047), all four G0..G3 sources saturate Kraft to exactly 1
    // (spec/11 §5 "G0..G3 Kraft 1.00000"); the ESC entry occupies a
    // regular bit-length slot.
    //
    // FROM: docs/video/msmpeg4/spec/11-walker-format-resolved.md §5
    // FROM: docs/video/msmpeg4/spec/03-corrections.md §3.1 (slot → G mapping)
    // FROM: docs/video/msmpeg4/spec/15-count-ab-per-g-family.md §3 (count_A per G)
    let g0_packed_hex = tables_dir.join("region_057a30_full.hex");
    let g1_packed_hex = tables_dir.join("region_057f80_full.hex");
    let g2_packed_hex = tables_dir.join("region_058558_full.hex");
    let g3_packed_hex = tables_dir.join("region_058a08_full.hex");
    println!("cargo:rerun-if-changed={}", g0_packed_hex.display());
    println!("cargo:rerun-if-changed={}", g1_packed_hex.display());
    println!("cargo:rerun-if-changed={}", g2_packed_hex.display());
    println!("cargo:rerun-if-changed={}", g3_packed_hex.display());
    emit_packed_huffman_g_extended(
        &g0_packed_hex,
        &out_dir.join("g0_primary.rs"),
        "G0",
        169,
        0x57a30,
        0x1c258630,
    );
    emit_packed_huffman_g_extended(
        &g1_packed_hex,
        &out_dir.join("g1_primary.rs"),
        "G1",
        186,
        0x57f80,
        0x1c258b80,
    );
    emit_packed_huffman_g_extended(
        &g2_packed_hex,
        &out_dir.join("g2_primary.rs"),
        "G2",
        149,
        0x58558,
        0x1c259158,
    );
    emit_packed_huffman_g_extended(
        &g3_packed_hex,
        &out_dir.join("g3_primary.rs"),
        "G3",
        133,
        0x58a08,
        0x1c259608,
    );

    // Intra-DC custom direct-value VLCs (round 28 — spec/07 §5.4 +
    // spec/11 §3 / §4). MS-MPEG4v3 does NOT use the MPEG-4 Part 2
    // §6.3.8 (size category VLC + magnitude bits) intra-DC scheme.
    // Instead, each intra-DC differential is read through a 120-entry
    // canonical-Huffman VLC where the decoded `idx` is the differential
    // magnitude (idx == 0 → DC diff = 0, idx ∈ [1, 0x76] → ±idx with a
    // separately-read sign bit, idx == 0x77 == 119 → ESC: read 8-bit raw
    // + sign bit) per spec/07 §5.4. There are FOUR such tables — one per
    // (luma, chroma) × (dc_size_sel = 0, 1) — selected per picture by
    // the `dc_size_sel` bit decoded in the picture header (spec/99 §4.5).
    //
    // Binary format: identical `4 + 120 * 8` packed-Huffman source
    // (spec/11 §3 / §4) BUT the `(a, b)` byte pair is the OPPOSITE
    // convention from G4/G5: here `a = bit_length, b = code_value`, NOT
    // `(code_value, bit_length)`. That's confirmed by the Kraft sum:
    // summing 2^-a over the 120 entries gives exactly 1 (a complete
    // prefix code at the bit-length given by `a`), whereas summing
    // 2^-b would give garbage. The candidate-VLC region_05eed0 uses
    // the same `(bl, code)` convention; G4/G5 use the opposite (because
    // the helper that walks the source treats them differently — the
    // (bl, code) form is the LUT-style reverse-mapping the bit-reader
    // helper `0x1c219351` consumes directly).
    //
    // FROM: docs/video/msmpeg4/spec/07-remaining-opens.md §5.4
    // FROM: docs/video/msmpeg4/spec/99-current-understanding.md §4.5
    // Per task #114 / spec/07 §5.4 the four 968-byte VMAs in
    // file-offset order are:
    //
    //   0x05f0d8 (VMA 0x1c25fcd8) → INTRA_DC_LUMA_SEL0
    //   0x05f4a0 (VMA 0x1c2600a0) → INTRA_DC_LUMA_SEL1
    //   0x05f868 (VMA 0x1c260468) → INTRA_DC_CHROMA_SEL0
    //   0x05fc30 (VMA 0x1c260830) → INTRA_DC_CHROMA_SEL1
    //
    // The pair-by-axis order is consistent with the binary's call-site
    // grouping where the luma path (VMA 0x1c214b0e) accesses the first
    // pair and the chroma paths (Cb / Cr) access the second pair.
    let dc_luma_sel0 = tables_dir.join("region_05f0d8.hex");
    let dc_luma_sel1 = tables_dir.join("region_05f4a0.hex");
    let dc_chroma_sel0 = tables_dir.join("region_05f868.hex");
    let dc_chroma_sel1 = tables_dir.join("region_05fc30.hex");
    println!("cargo:rerun-if-changed={}", dc_luma_sel0.display());
    println!("cargo:rerun-if-changed={}", dc_luma_sel1.display());
    println!("cargo:rerun-if-changed={}", dc_chroma_sel0.display());
    println!("cargo:rerun-if-changed={}", dc_chroma_sel1.display());
    emit_intra_dc_vlc(
        &dc_luma_sel0,
        &out_dir.join("intra_dc_luma_sel0.rs"),
        "INTRA_DC_LUMA_SEL0",
        0x5f0d8,
        0x1c25fcd8,
    );
    emit_intra_dc_vlc(
        &dc_luma_sel1,
        &out_dir.join("intra_dc_luma_sel1.rs"),
        "INTRA_DC_LUMA_SEL1",
        0x5f4a0,
        0x1c2600a0,
    );
    emit_intra_dc_vlc(
        &dc_chroma_sel0,
        &out_dir.join("intra_dc_chroma_sel0.rs"),
        "INTRA_DC_CHROMA_SEL0",
        0x5f868,
        0x1c260468,
    );
    emit_intra_dc_vlc(
        &dc_chroma_sel1,
        &out_dir.join("intra_dc_chroma_sel1.rs"),
        "INTRA_DC_CHROMA_SEL1",
        0x5fc30,
        0x1c260830,
    );

    // G0..G3 (run, level, last) enumeration CSVs (round 29 — spec/09).
    // Each CSV maps a primary-VLC symbol index to its (last, run, |level|)
    // triple plus the alphabet-size ESC sentinel at the final row.
    //
    // FROM: docs/video/msmpeg4/spec/09-g0-g3-enumeration.md
    // FROM: docs/video/msmpeg4/tables/region_<addr>_g<N>_enum.csv
    //       (extracted by extract-06.sh)
    let g0_enum = tables_dir.join("region_056c60_g0_enum.csv");
    let g1_enum = tables_dir.join("region_056ff0_g1_enum.csv");
    let g2_enum = tables_dir.join("region_057300_g2_enum.csv");
    let g3_enum = tables_dir.join("region_0575a8_g3_enum.csv");
    println!("cargo:rerun-if-changed={}", g0_enum.display());
    println!("cargo:rerun-if-changed={}", g1_enum.display());
    println!("cargo:rerun-if-changed={}", g2_enum.display());
    println!("cargo:rerun-if-changed={}", g3_enum.display());
    // Per-G enum extraction. The `(count_A, count_B)` arguments are
    // pulled from the spec/15 §3 authoritative table — DO NOT hardcode
    // them locally. The emitter additionally cross-checks the CSV row
    // count against `count_A` and the sub-A row count against
    // `count_B + 1` (see `emit_g_enum`).
    let (g0_ca, g0_cb) = G_COUNTS_SPEC15[0];
    let (g1_ca, g1_cb) = G_COUNTS_SPEC15[1];
    let (g2_ca, g2_cb) = G_COUNTS_SPEC15[2];
    let (g3_ca, g3_cb) = G_COUNTS_SPEC15[3];
    emit_g_enum(
        &g0_enum,
        &out_dir.join("g0_enum.rs"),
        "G0",
        g0_ca as usize,
        g0_cb as usize,
    );
    emit_g_enum(
        &g1_enum,
        &out_dir.join("g1_enum.rs"),
        "G1",
        g1_ca as usize,
        g1_cb as usize,
    );
    emit_g_enum(
        &g2_enum,
        &out_dir.join("g2_enum.rs"),
        "G2",
        g2_ca as usize,
        g2_cb as usize,
    );
    emit_g_enum(
        &g3_enum,
        &out_dir.join("g3_enum.rs"),
        "G3",
        g3_ca as usize,
        g3_cb as usize,
    );

    // ESC-extension table cluster — region_060988 (round 33; spec/08 §1
    // + §2 + §4). 24 contiguous slices at file 0x60988..0x61200 (VMA
    // 0x1c261588..0x1c261e00, 2168 bytes). Per spec/08 §2.2 each
    // G-descriptor (G0..G5) carries four pointers in slots
    // `desc+0x0c..+0x18` to byte / u32 extension arrays the v2/v3
    // inter kernel `1c215e6f` and v1/v2/v3 intra kernel `1c216d97` read
    // on first-tier (`+0x0c/+0x10`) and second-tier (`+0x14/+0x18`) ESC
    // paths. The slice boundaries come from
    // `region_060988_index.csv` (24 rows, one per slice); per-VMA
    // attribution to G0..G5 × {sub-A lev-ext, sub-B lev-ext, sub-A
    // run-ext, sub-B run-ext} comes from spec/08 §2.2 / spec/14 §2.1.
    //
    // **Semantic interpretation OPEN per spec/08 §4.1.** The 8-byte
    // `(symbol_u32_le, bit_length_u32_le)` record format the slices
    // mechanically parse as is consistent with the per-slot packed-
    // Huffman source format used elsewhere (G4/G5 primary VLC, intra-
    // DC), but the kernel's `[base + idx*4]` access pattern (BYTE PTR
    // for `+0x0c/+0x10`, DWORD PTR for `+0x14/+0x18`) does not
    // straightforwardly map onto an 8-byte-per-record secondary VLC.
    // This round (33) wires the **bytes and slice boundaries only**;
    // the (level-ext, run-ext) decoder semantics await a future
    // Specifier round on the inter / intra kernel ESC bodies.
    let esc_ext_hex = tables_dir.join("region_060988.hex");
    let esc_ext_idx = tables_dir.join("region_060988_index.csv");
    println!("cargo:rerun-if-changed={}", esc_ext_hex.display());
    println!("cargo:rerun-if-changed={}", esc_ext_idx.display());
    emit_esc_ext_cluster(
        &esc_ext_hex,
        &esc_ext_idx,
        &out_dir.join("esc_ext_cluster.rs"),
    );

    println!("cargo:rerun-if-changed=build.rs");
}

/// Parse `region_05eac8.csv` and emit a Rust file with the raw
/// `(bit_length, code_value)` pairs and the header fields.
fn emit_mcbpcy_v3(csv_path: &Path, out_path: &Path) {
    let text = fs::read_to_string(csv_path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", csv_path.display()));

    let mut records: Vec<(u32, u32)> = Vec::with_capacity(129);
    for (line_no, line) in text.lines().enumerate() {
        if line_no == 0 {
            // Skip CSV column header.
            continue;
        }
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let parts: Vec<&str> = line.split(',').collect();
        if parts.len() < 5 {
            panic!("malformed row at line {}: {line}", line_no + 1);
        }
        let bit_length: u32 = parts[3]
            .parse()
            .unwrap_or_else(|_| panic!("bad bit_length at line {}: {}", line_no + 1, parts[3]));
        let code_value: u32 = parts[4]
            .parse()
            .unwrap_or_else(|_| panic!("bad code_value at line {}: {}", line_no + 1, parts[4]));
        records.push((bit_length, code_value));
    }

    if records.len() != 129 {
        panic!(
            "expected 129 records in {} (1 header + 128 payload), got {}",
            csv_path.display(),
            records.len()
        );
    }

    let (alphabet_size, partition) = records[0];
    if alphabet_size != 128 {
        panic!(
            "unexpected alphabet size {alphabet_size} in {} (expected 128)",
            csv_path.display()
        );
    }
    if partition != 64 {
        panic!(
            "unexpected partition {partition} in {} (expected 64)",
            csv_path.display()
        );
    }

    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    writeln!(
        f,
        "// Auto-generated by build.rs from \
         docs/video/msmpeg4/tables/region_05eac8.csv. DO NOT EDIT.\n\
         // Source binary: mpg4c32.dll SHA-256 \
         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099\n\
         // Role: v3 joint-MCBPCY VLC source (per spec/99 §3.1 / §8.1).\n\
         \n\
         pub const MCBPCY_V3_ALPHABET_SIZE: usize = {};\n\
         pub const MCBPCY_V3_PARTITION: usize = {};\n\
         \n\
         /// 128 × (bit_length, code_value) canonical-Huffman entries for\n\
         /// the v3 joint-MCBPCY VLC. Index 0..63 are I-type MBs; 64..127\n\
         /// are P-type. See spec/99 §3.1 for consumer semantics.\n\
         pub const MCBPCY_V3_RAW: &[(u32, u32)] = &[",
        alphabet_size, partition,
    )
    .unwrap();
    for &(bl, code) in &records[1..] {
        writeln!(f, "    ({bl}, {code}),").unwrap();
    }
    writeln!(f, "];").unwrap();
}

/// Parse `region_05bfc0.csv` and emit the v3 joint-MV VLC table as raw
/// `(bit_length, code_value)` pairs. Record 0 is the alphabet header
/// `(1100, 1)`; records 1..=1100 are the 1100 canonical-Huffman payload
/// entries (index 1099 = ESC).
fn emit_mv_v3(csv_path: &Path, out_path: &Path) {
    let text = fs::read_to_string(csv_path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", csv_path.display()));

    let mut records: Vec<(u32, u32)> = Vec::with_capacity(1101);
    for (line_no, line) in text.lines().enumerate() {
        if line_no == 0 {
            continue; // CSV column header
        }
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let parts: Vec<&str> = line.split(',').collect();
        if parts.len() < 5 {
            panic!("malformed row at line {}: {line}", line_no + 1);
        }
        // Same column convention as MCBPCY: symbol_dec = bit_length,
        // bit_length = code_value.
        let bit_length: u32 = parts[3]
            .parse()
            .unwrap_or_else(|_| panic!("bad bit_length at line {}: {}", line_no + 1, parts[3]));
        let code_value: u32 = parts[4]
            .parse()
            .unwrap_or_else(|_| panic!("bad code_value at line {}: {}", line_no + 1, parts[4]));
        records.push((bit_length, code_value));
    }

    if records.len() != 1101 {
        panic!(
            "expected 1101 records in {} (1 header + 1100 payload), got {}",
            csv_path.display(),
            records.len()
        );
    }

    let (alphabet_size, _marker) = records[0];
    if alphabet_size != 1100 {
        panic!(
            "unexpected alphabet size {alphabet_size} in {} (expected 1100)",
            csv_path.display()
        );
    }

    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    writeln!(
        f,
        "// Auto-generated by build.rs from \
         docs/video/msmpeg4/tables/region_05bfc0.csv. DO NOT EDIT.\n\
         // Source binary: mpg4c32.dll SHA-256 \
         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099\n\
         // Role: v3 joint (X, Y) MV VLC source default variant (VMA\n\
         // 0x1c25cbc0, per spec/06 §2.1). Index 1099 is the ESC\n\
         // sentinel; ESC tail is 6 bits MVDx + 6 bits MVDy.\n\
         \n\
         pub const MV_V3_ALPHABET_SIZE: usize = {};\n\
         pub const MV_V3_ESC_INDEX: usize = 1099;\n\
         \n\
         /// 1100 × (bit_length, code_value) canonical-Huffman entries.\n\
         /// Index 1099 is ESC (any bit_length maps to ESC fallthrough);\n\
         /// indices 0..=1098 are non-ESC joint (MVDx, MVDy) codes.\n\
         pub const MV_V3_RAW: &[(u32, u32)] = &[",
        alphabet_size,
    )
    .unwrap();
    for &(bl, code) in &records[1..] {
        writeln!(f, "    ({bl}, {code}),").unwrap();
    }
    writeln!(f, "];").unwrap();
}

/// Parse `region_05e228.hex` and `region_05e678.hex` as xxd-format
/// files and emit their bytes as `pub static MVDX_V3_BYTES: [u8; 1104]`
/// and `pub static MVDY_V3_BYTES: [u8; 1104]`. Only indices 0..=1098
/// are read by the decoder (ESC takes index 1099).
fn emit_mv_byte_lut_v3(mvdx_path: &Path, mvdy_path: &Path, out_path: &Path) {
    let mvdx = parse_xxd(mvdx_path);
    let mvdy = parse_xxd(mvdy_path);
    if mvdx.len() != 1104 {
        panic!(
            "unexpected byte count {} in {} (expected 1104)",
            mvdx.len(),
            mvdx_path.display()
        );
    }
    if mvdy.len() != 1104 {
        panic!(
            "unexpected byte count {} in {} (expected 1104)",
            mvdy.len(),
            mvdy_path.display()
        );
    }

    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    writeln!(
        f,
        "// Auto-generated by build.rs from \
         docs/video/msmpeg4/tables/region_05e228.hex and\n\
         // docs/video/msmpeg4/tables/region_05e678.hex. DO NOT EDIT.\n\
         // Source binary: mpg4c32.dll SHA-256 \
         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099\n\
         // Role: v3 MVDx / MVDy byte LUTs default variant (VMAs\n\
         // 0x1c25ee28, 0x1c25f278 per spec/06 §2.2).\n\
         \n\
         pub static MVDX_V3_BYTES: &[u8; 1104] = &["
    )
    .unwrap();
    emit_byte_array(&mut f, &mvdx);
    writeln!(f, "];\n").unwrap();
    writeln!(f, "pub static MVDY_V3_BYTES: &[u8; 1104] = &[").unwrap();
    emit_byte_array(&mut f, &mvdy);
    writeln!(f, "];").unwrap();
}

/// Parse `region_05eed0.csv` and emit the candidate v3 intra AC TCOEF
/// primary VLC source. The CSV uses the same column convention as
/// MCBPCY: the `symbol_dec` column is the bit_length, the `bit_length`
/// column is the code_value (see spec/99 §8.1). Row 0 is the header
/// `(count_A=64, count_B=1)`; rows 1..=64 are the 64 payload entries.
///
/// Provenance: `docs/video/msmpeg4/tables/region_05eed0.csv` —
/// extracted from `mpg4c32.dll` (SHA-256
/// `aedb4cf3...b3c099`) at file offset `0x5eed0`, VMA `0x1c25fad0`.
/// Spec/99 §0.1 row 8 flags this VMA as a candidate intra-AC primary
/// VLC (per spec/03 §5.3) but also notes it could be the v2 MCBPCY
/// source (`spec/99` §9 OPEN-O6) — the role is unresolved. The bytes
/// are extraction-grounded regardless and Kraft sums to exactly 1
/// over the 64 payload bit-lengths (verified at build time below).
fn emit_intra_ac_v3(csv_path: &Path, out_path: &Path) {
    let text = fs::read_to_string(csv_path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", csv_path.display()));

    let mut records: Vec<(u32, u32)> = Vec::with_capacity(65);
    for (line_no, line) in text.lines().enumerate() {
        if line_no == 0 {
            continue; // CSV column header
        }
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let parts: Vec<&str> = line.split(',').collect();
        if parts.len() < 5 {
            panic!("malformed row at line {}: {line}", line_no + 1);
        }
        // Same column convention as MCBPCY: parts[3] = symbol_dec column =
        // bit_length value; parts[4] = bit_length column = code_value.
        let bit_length: u32 = parts[3]
            .parse()
            .unwrap_or_else(|_| panic!("bad bit_length at line {}: {}", line_no + 1, parts[3]));
        let code_value: u32 = parts[4]
            .parse()
            .unwrap_or_else(|_| panic!("bad code_value at line {}: {}", line_no + 1, parts[4]));
        records.push((bit_length, code_value));
    }

    if records.len() != 65 {
        panic!(
            "expected 65 records in {} (1 header + 64 payload), got {}",
            csv_path.display(),
            records.len()
        );
    }

    let (alphabet_size, partition) = records[0];
    if alphabet_size != 64 {
        panic!(
            "unexpected alphabet size {alphabet_size} in {} (expected 64)",
            csv_path.display()
        );
    }
    if partition != 1 {
        panic!(
            "unexpected partition {partition} in {} (expected 1)",
            csv_path.display()
        );
    }

    // Verify Kraft sum == 1 over the 64 payload bit-lengths (in 2^-bl
    // arithmetic). All bls must be in [1, 32] for a valid
    // canonical-Huffman code. Compute exactly using the LCM trick:
    // sum of 2^-bl == 1 iff sum of 2^(MAX-bl) == 2^MAX.
    let max_bl = 32u32;
    let target: u64 = 1u64 << max_bl;
    let mut sum: u64 = 0;
    for &(bl, _) in &records[1..] {
        if !(1..=32).contains(&bl) {
            panic!(
                "intra-AC candidate: bit_length {bl} out of range [1, 32] — \
                 cannot form a canonical-Huffman code"
            );
        }
        sum += 1u64 << (max_bl - bl);
    }
    if sum != target {
        panic!(
            "intra-AC candidate: Kraft sum != 1 (sum of 2^-bl gives {} / {} \
             in fixed-point); the table is not a complete prefix code",
            sum, target
        );
    }

    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    writeln!(
        f,
        "// Auto-generated by build.rs from \
         docs/video/msmpeg4/tables/region_05eed0.csv. DO NOT EDIT.\n\
         // Source binary: mpg4c32.dll SHA-256 \
         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099\n\
         // Role (CANDIDATE — OPEN per spec/99 §0.1 row 8 / §9 OPEN-O6):\n\
         //   v3 intra AC TCOEF run/level/last primary VLC source\n\
         //   (alternative: v2 joint-MCBPCY source). Kraft sum of the\n\
         //   64 payload bit-lengths is exactly 1 (verified at build time).\n\
         \n\
         pub const INTRA_AC_V3_CANDIDATE_ALPHABET: usize = {};\n\
         pub const INTRA_AC_V3_CANDIDATE_PARTITION: usize = {};\n\
         \n\
         /// 64 × (bit_length, code_value) canonical-Huffman entries for\n\
         /// the v3 intra AC TCOEF primary VLC candidate. The `code_value`\n\
         /// column is the runtime LUT/state byte (same convention as\n\
         /// MCBPCY — spec/99 §8.1) and is **not** the Huffman bit-pattern;\n\
         /// the bit-pattern is reconstructed by the canonical-Huffman\n\
         /// builder from the bit_length array alone.\n\
         pub const INTRA_AC_V3_CANDIDATE_RAW: &[(u32, u32)] = &[",
        alphabet_size, partition,
    )
    .unwrap();
    for &(bl, code) in &records[1..] {
        writeln!(f, "    ({bl}, {code}),").unwrap();
    }
    writeln!(f, "];").unwrap();
}

/// Parse the combined v1 + v2 MCBPC LUT region (1280 bytes at file offset
/// 0x53140 / VMA 0x1c253d40) and emit canonical-Huffman code-length tables
/// for both decoders.
///
/// LUT format (per spec/07 §1.2 + the helper `0x1c215811` shape): each
/// 2-byte record is `(bit_length, symbol)`. The full input range of
/// `max_bitlen` bits is pre-expanded so that index N (read MSB-first
/// from the bitstream) directly indexes into the LUT. For a code with
/// bit-length L and prefix P, all 2^(max_bitlen - L) consecutive entries
/// at indices `P << (max_bitlen - L) ..` carry the same `(L, sym)` pair.
///
/// Layout in the 1280-byte region:
///   * bytes [0..1024) — v1 MCBPC 9-bit LUT (512 entries × 2 bytes;
///     VMA 0x1c253d40, alphabet 0..20 + idx-0 sentinel for the all-zero
///     9-bit pattern).
///   * bytes [1024..1280) — v2 MCBPC 7-bit LUT (128 entries × 2 bytes;
///     VMA 0x1c254140 = 0x1c253d40 + 0x400, alphabet 0..7).
///
/// We extract the unique `(symbol, bit_length, canonical_code)` tuples by
/// walking the LUT and capturing the first occurrence of each symbol,
/// where `code = idx >> (max_bitlen - bit_length)`. The result is a
/// list of (sym, bl, code) triples that's identical to canonical-Huffman
/// codes assigned in (bit_length, symbol) ascending order.
fn emit_mcbpc_v1_v2(hex_path: &Path, out_path: &Path) {
    let bytes = parse_xxd(hex_path);
    if bytes.len() != 1280 {
        panic!(
            "expected 1280 bytes in {} (combined v1+v2 MCBPC LUT), got {}",
            hex_path.display(),
            bytes.len()
        );
    }

    // v1 MCBPC: first 1024 bytes = 9-bit LUT.
    let v1 = extract_lut_alphabet(&bytes[..1024], 9);
    // v2 MCBPC: next 256 bytes = 7-bit LUT.
    let v2 = extract_lut_alphabet(&bytes[1024..1280], 7);

    // Spec sanity per spec/07 §1.3 (v1: 21 entries, sym 0..20)
    // and §2.3 (v2: 8 entries, sym 0..7). The v1 LUT idx 0
    // (all-zero 9-bit pattern) carries (bl=0, sym=0xff) as a sentinel —
    // extract_lut_alphabet drops it because bl==0.
    if v1.len() != 21 {
        panic!(
            "v1 MCBPC: expected 21 unique symbols (per spec/07 §1.3), got {}",
            v1.len()
        );
    }
    if v2.len() != 8 {
        panic!(
            "v2 MCBPC: expected 8 unique symbols (per spec/07 §2.3), got {}",
            v2.len()
        );
    }

    // Kraft sum check (==1 for v2; <1 for v1 because idx 0 sentinel
    // reserves 1 leaf for the ESC sequence). v1 sum should be exactly
    // 1 - 2^-9 = 511/512.
    let kraft_v1: u64 = v1.iter().map(|&(_, bl, _)| 1u64 << (32 - bl)).sum();
    let kraft_v2: u64 = v2.iter().map(|&(_, bl, _)| 1u64 << (32 - bl)).sum();
    let target: u64 = 1u64 << 32;
    if kraft_v1 != target - (1u64 << (32 - 9)) {
        panic!(
            "v1 MCBPC Kraft sum {} != expected {} (= 1 - 2^-9 in fixed point)",
            kraft_v1,
            target - (1u64 << (32 - 9))
        );
    }
    if kraft_v2 != target {
        panic!("v2 MCBPC Kraft sum {} != 1.0 (target {})", kraft_v2, target);
    }

    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    writeln!(
        f,
        "// Auto-generated by build.rs from \
         docs/video/msmpeg4/tables/region_053140.hex (copied to\n\
         // crates/oxideav-msmpeg4/tables/). DO NOT EDIT.\n\
         // Source binary: mpg4c32.dll SHA-256 \
         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099\n\
         // Roles:\n\
         //   v1 MCBPC at VMA 0x1c253d40 (per spec/07 §1.3, helper\n\
         //   0x1c215811 with max-bitlen 9, alphabet 0..20).\n\
         //   v2 MCBPC at VMA 0x1c254140 (per spec/07 §2.3, helper\n\
         //   0x1c215811 with max-bitlen 7, alphabet 0..7).\n\
         \n\
         /// (symbol, bit_length, canonical_code) triples for the v1 MCBPC\n\
         /// VLC. 21 entries; the all-zero 9-bit input pattern is a\n\
         /// sentinel (no valid code) per the LUT's `(bl=0, sym=0xff)`\n\
         /// entry at idx 0, which represents the start-of-sequence\n\
         /// reserved code per H.263 §5.3.\n\
         pub const MCBPC_V1_RAW: &[(u8, u8, u32)] = &["
    )
    .unwrap();
    for &(sym, bl, code) in &v1 {
        writeln!(f, "    ({sym}, {bl}, 0x{code:x}),").unwrap();
    }
    writeln!(
        f,
        "];\n\n\
         /// (symbol, bit_length, canonical_code) triples for the v2 MCBPC\n\
         /// VLC. 8 entries (alphabet 0..7) + Kraft sum exactly 1.\n\
         pub const MCBPC_V2_RAW: &[(u8, u8, u32)] = &["
    )
    .unwrap();
    for &(sym, bl, code) in &v2 {
        writeln!(f, "    ({sym}, {bl}, 0x{code:x}),").unwrap();
    }
    writeln!(f, "];").unwrap();
}

/// Parse the shared v1/v2 CBPY pre-expanded LUT (region at file 0x53640 /
/// VMA 0x1c254240) and emit canonical-Huffman `(sym, bit_length, code)`
/// triples for the 16-entry CBPY alphabet.
///
/// LUT format: same `(bit_length, symbol)` halfword convention as
/// `emit_mcbpc_v1_v2`, with max-bitlen 6. The first 64 entries (128
/// bytes) hold the CBPY pre-expansion; subsequent sub-tables in the
/// 4096-byte region belong to different VLCs (different alphabets and
/// pre-expansion widths per the per-pair stride observed in the hex
/// dump) and are NOT consumed here. Per spec/07 §1.3 / §2.3, only the
/// first 6-bit window is the CBPY VLC.
///
/// Special-case decoding: the binary fills the two reserved 6-bit
/// prefixes `000000` and `000001` with sentinel symbols 0x10 and 0x11
/// respectively. Neither is in the 0..15 CBPY alphabet — they mark
/// "invalid bitstream" outcomes that the runtime decoder rejects via
/// `decode_cbpy_with_wrap`'s `raw > 15` range check. Per `audit/01`
/// §2.3's sentinel convention this is consistent with how the binary
/// marks reserved codewords.
///
/// **Cross-check guarantee:** the emitted `CBPY_V1_V2_RAW` table is
/// verified at build time against the hand-derived `CBPY_INTRA_TABLE`
/// (H.263 Table 8 / MPEG-4 Part 2 Table B-6, public standards). If the
/// binary's pre-expanded LUT ever drifts from the standard, the build
/// breaks with a precise error pointing at the divergent (sym, bl,
/// code) tuple.
fn emit_cbpy_v1_v2(hex_path: &Path, out_path: &Path) {
    let bytes = parse_xxd(hex_path);
    if bytes.len() != 4096 {
        panic!(
            "expected 4096 bytes in {} (shared CBPY + sibling LUTs at file 0x53640), got {}",
            hex_path.display(),
            bytes.len()
        );
    }

    // First 128 bytes = 6-bit pre-expanded CBPY LUT (64 entries × 2 bytes).
    let cbpy_lut = &bytes[..128];
    let raw = extract_lut_alphabet(cbpy_lut, 6);

    // Filter out the two reserved-sentinel symbols (0x10 and 0x11) — both
    // are bl=6 codes at the all-zero / all-zero-but-LSB 6-bit windows,
    // not part of the 16-entry CBPY alphabet. The runtime decoder
    // rejects raw values > 15 via the existing range check.
    let cbpy: Vec<(u8, u8, u32)> = raw.iter().filter(|&&(s, _, _)| s < 16).copied().collect();

    if cbpy.len() != 16 {
        panic!(
            "v1/v2 CBPY: expected 16 alphabet symbols (per H.263 Table 8 / MPEG-4 Part 2 \
             Table B-6, spec/07 §1.3), got {} after filtering reserved sentinels {{0x10, 0x11}}",
            cbpy.len()
        );
    }

    // Cross-check against the H.263 / MPEG-4 Part 2 Table B-6 values that
    // ship in `src/tables.rs::CBPY_INTRA_TABLE`. The table is small
    // enough to inline here as the cross-check oracle. If the binary
    // ever drifts from H.263 the build breaks with a precise tuple
    // mismatch error.
    let expected: &[(u8, u8, u32)] = &[
        (0, 4, 0b0011),
        (1, 5, 0b00101),
        (2, 5, 0b00100),
        (3, 4, 0b1001),
        (4, 5, 0b00011),
        (5, 4, 0b0111),
        (6, 6, 0b000010),
        (7, 4, 0b1011),
        (8, 5, 0b00010),
        (9, 6, 0b000011),
        (10, 4, 0b0101),
        (11, 4, 0b1010),
        (12, 4, 0b0100),
        (13, 4, 0b1000),
        (14, 4, 0b0110),
        (15, 2, 0b11),
    ];
    for (got, exp) in cbpy.iter().zip(expected.iter()) {
        if got != exp {
            panic!(
                "v1/v2 CBPY binary-vs-H.263 mismatch at sym {}: binary (bl={}, code=0b{:0width$b}) \
                 vs H.263 Table 8 (bl={}, code=0b{:0width$b})",
                exp.0,
                got.1,
                got.2,
                exp.1,
                exp.2,
                width = exp.1 as usize,
            );
        }
    }

    // Kraft sum check: 16 codes + 2 reserved sentinels = 18 leaves of
    // a 6-bit prefix space. Kraft sum over the 16 alphabet entries
    // (excluding sentinels) is 1 - 2 * 2^-6 = 1 - 1/32 = 31/32. The two
    // reserved sentinels each occupy one bl=6 leaf.
    let kraft: u64 = cbpy.iter().map(|&(_, bl, _)| 1u64 << (32 - bl)).sum();
    let target_kraft: u64 = (1u64 << 32) - 2 * (1u64 << (32 - 6));
    if kraft != target_kraft {
        panic!(
            "v1/v2 CBPY Kraft sum {kraft} != expected {target_kraft} (= 1 - 2/64 \
             in fixed-point, accounting for the 2 reserved bl=6 sentinels)"
        );
    }

    // Also capture the two sentinel symbols + their code positions for
    // the diagnostic test surface (so a regression in the binary's
    // sentinel positioning is visible).
    let sentinels: Vec<(u8, u8, u32)> = raw.iter().filter(|&&(s, _, _)| s >= 16).copied().collect();
    if sentinels.len() != 2 {
        panic!(
            "v1/v2 CBPY: expected exactly 2 reserved-sentinel symbols (per binary @ VMA \
             0x1c254240 LUT slots 0..1), got {} (syms: {:?})",
            sentinels.len(),
            sentinels.iter().map(|&(s, _, _)| s).collect::<Vec<_>>()
        );
    }

    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    writeln!(
        f,
        "// Auto-generated by build.rs from \
         docs/video/msmpeg4/tables/region_053640.hex (copied to\n\
         // crates/oxideav-msmpeg4/tables/). DO NOT EDIT.\n\
         // Source binary: mpg4c32.dll SHA-256 \
         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099\n\
         // Role: shared v1/v2 CBPY VLC at VMA 0x1c254240 (per spec/07 §1.3 / §2.3,\n\
         // helper 0x1c215811 with max-bitlen 6, 16-entry alphabet + 2 reserved\n\
         // sentinels). Cross-checked at build time against H.263 Table 8 /\n\
         // MPEG-4 Part 2 Table B-6 values in src/tables.rs::CBPY_INTRA_TABLE.\n\
         \n\
         /// (symbol, bit_length, canonical_code) triples for the shared v1/v2 CBPY\n\
         /// VLC, extracted from the binary's 6-bit pre-expanded LUT and verified\n\
         /// to match H.263 Table 8 / MPEG-4 Part 2 Table B-6 byte-for-byte. The\n\
         /// runtime decoder uses src/tables.rs::CBPY_INTRA_TABLE; this constant\n\
         /// exists so the build cross-checks the two against each other and so\n\
         /// downstream tests can assert binary-traceability of the CBPY VLC path.\n\
         pub const CBPY_V1_V2_RAW: &[(u8, u8, u32)] = &["
    )
    .unwrap();
    for &(sym, bl, code) in &cbpy {
        writeln!(f, "    ({sym}, {bl}, 0x{code:x}),").unwrap();
    }
    writeln!(
        f,
        "];\n\n\
         /// Reserved-sentinel `(sym, bl, code)` entries the binary's CBPY LUT\n\
         /// places at the all-zero (`000000`) and next-to-all-zero (`000001`)\n\
         /// 6-bit windows. Both syms are outside the 0..15 CBPY alphabet and\n\
         /// would be rejected by the runtime decoder's `raw > 15` range check.\n\
         /// Per spec/07 §1.3 these are 'invalid bitstream' markers.\n\
         pub const CBPY_V1_V2_SENTINELS: &[(u8, u8, u32)] = &["
    )
    .unwrap();
    for &(sym, bl, code) in &sentinels {
        writeln!(f, "    ({sym}, {bl}, 0x{code:x}),").unwrap();
    }
    writeln!(f, "];").unwrap();
}

/// Parse the v1/v2 per-component MV VLC LUT (16384 bytes at file offset
/// 0x4ed30 / VMA 0x1c24f930) and emit canonical-Huffman triples.
///
/// LUT format: same packed `(bit_length, symbol)` halfword convention as
/// the MCBPC tables (helper `0x1c215811`), but with max-bitlen 13 so the
/// LUT has 2^13 = 8192 halfword slots × 2 bytes = 16384 bytes. Per
/// spec/06 §2.3 / spec/07 §3 the alphabet is 65 raw symbols (indices
/// 0..=64), with index 32 representing zero-MVD (the bias point that
/// the binary subtracts at `1c217ead`: `lea eax, [eax + ecx*1 - 0x20]`).
/// After the bias subtract the signed MVD residual is in [-32, +32].
///
/// Slot 0..=3 of the LUT carry `(bl=0, sym=0xff)` sentinels — the
/// helper writes the bit-reader-error flag at `[ecx+0x10]=3` when those
/// slots are hit (see disassembly at `1c21587f`). They are not part of
/// the alphabet and are dropped by `extract_lut_alphabet` (bl==0 branch).
///
/// The 65 payload symbols form a complete prefix code over 13-bit
/// prefixes minus 4 sentinel slots. Kraft sum == 1 - 4/2^13 = 8188/8192
/// (verified at build time below). The four "missing" leaves are
/// reserved for the bit-reader-error escape, not for an extra alphabet
/// entry.
fn emit_mv_v1_v2(hex_path: &Path, out_path: &Path) {
    let bytes = parse_xxd(hex_path);
    if bytes.len() != 16384 {
        panic!(
            "expected 16384 bytes in {} (full v1/v2 MV LUT, 8192 entries × 2 bytes), got {}",
            hex_path.display(),
            bytes.len()
        );
    }

    let triples = extract_lut_alphabet(&bytes, 13);

    // Spec sanity per spec/06 §2.3 / spec/07 §3: 65 entries with
    // contiguous symbols 0..=64. (The previously-reported "33 entries"
    // figure in spec/06 §2.3 / spec/07 §3.3 derived from the bias
    // arithmetic `eax - 0x20` is off by 2× — the raw index actually
    // ranges 0..=64 so the bias yields signed [-32, +32], matching the
    // MV byte range.)
    if triples.len() != 65 {
        panic!(
            "v1/v2 MV: expected 65 unique symbols, got {}. The LUT spans the raw \
             alphabet 0..=64 (65 symbols) per spec/06 §3.5's [-32, +32] signed range.",
            triples.len()
        );
    }
    let symbols: Vec<u8> = triples.iter().map(|&(s, _, _)| s).collect();
    let expected: Vec<u8> = (0u8..=64).collect();
    if symbols != expected {
        panic!(
            "v1/v2 MV: symbols not contiguous 0..=64 — got first/last/len = \
             {:?}/{:?}/{}",
            symbols.first(),
            symbols.last(),
            symbols.len()
        );
    }
    for &(_, bl, _) in &triples {
        if !(1..=13).contains(&bl) {
            panic!("v1/v2 MV: bit_length {bl} out of [1, 13] (helper push 0xd)");
        }
    }

    // Kraft sum check: 1 - 4/2^13 (4 escape sentinel leaves at LUT slot 0..=3).
    let max_bl: u32 = 32;
    let target: u64 = (1u64 << max_bl) - (1u64 << (max_bl - 13)) * 4;
    let sum: u64 = triples
        .iter()
        .map(|&(_, bl, _)| 1u64 << (max_bl - bl as u32))
        .sum();
    if sum != target {
        panic!(
            "v1/v2 MV Kraft sum {} != expected {} (= 1 - 4/2^13 in fixed point — \
             4 sentinel slots reserved for the bit-reader-error path)",
            sum, target
        );
    }

    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    writeln!(
        f,
        "// Auto-generated by build.rs from \
         crates/oxideav-msmpeg4/tables/region_04ed30_full.hex.\n\
         // DO NOT EDIT.\n\
         // Source binary: mpg4c32.dll SHA-256 \
         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099\n\
         // Role: v1/v2 (and v4+) per-component MV VLC table at VMA 0x1c24f930.\n\
         //   Loaded as literal at 0x1c217e8d in 0x1c217e56's v<4 branch.\n\
         //   Helper 0x1c215811 invoked with max-bitlen 13.\n\
         //   Alphabet: 65 raw symbols (indices 0..=64). After bias\n\
         //   subtraction (raw - 0x20) the signed MVD residual is in\n\
         //   [-32, +32]. The bias point (raw idx 32) corresponds to MVD=0,\n\
         //   the most-probable code (1-bit code 'b1).\n\
         \n\
         pub const MV_V1_V2_ALPHABET_SIZE: usize = 65;\n\
         pub const MV_V1_V2_BIAS: i32 = 32;\n\
         \n\
         /// 65 × (sym, bit_length, canonical_code) triples for the v1/v2\n\
         /// per-component MV VLC. Symbols are contiguous 0..=64; sym 32 =\n\
         /// MVD 0; sym 0 = MVD -32; sym 64 = MVD +32. No ESC path —\n\
         /// alphabet is complete (Kraft sum = 1 - 4/2^13).\n\
         pub const MV_V1_V2_RAW: &[(u8, u8, u32)] = &["
    )
    .unwrap();
    for &(sym, bl, code) in &triples {
        writeln!(f, "    ({sym}, {bl}, 0x{code:x}),").unwrap();
    }
    writeln!(f, "];").unwrap();
}

/// Parse `region_0569c0.hex` (G-descriptor cluster) and `region_057898.hex`
/// (G5 pri_B gap fill) and emit byte slices for the G-descriptor pri_A
/// (`|level|`) and pri_B (`run`) tables. Round 18 wired G4 (full) and G5's
/// pri_A; round 19 fills G5's pri_B from `region_057898.hex` which captures
/// the 408 bytes that live immediately after the cluster region ends
/// (file 0x57898..0x57a30, VMA 0x1c258498).
///
/// Provenance: every byte is sliced verbatim from one of the two cluster
/// dumps; the Implementer never types numeric values. The slice offsets
/// are derived from the absolute VMAs in spec/99 §5 (G4 pri_A `0x1c258230`,
/// G4 pri_B `0x1c258298`, G5 pri_A `0x1c258430`, G5 pri_B `0x1c258498`)
/// minus the region's base VMA. `region_057898.hex` is its own region —
/// G5 pri_B starts exactly at its first byte, so no in-region slice
/// arithmetic is needed.
///
/// The shape (count_A, count_B) for each group is the spec/99 §5 row
/// values. Sub-class A entries (`idx <= count_B`) carry `last=0`;
/// sub-class B entries (`count_B < idx < count_A`) carry `last=1`. The
/// (run, level) decomposition is then `pri_A[idx]` = `|level|` and
/// `pri_B[idx] & 0xFF` = `run` (the upper bytes of pri_B are zero for
/// G4/G5 per audit/01 §2.4 and §4.4 — verified at build time below).
fn emit_g_descriptor_cluster(hex_path: &Path, g5_pri_b_path: &Path, out_path: &Path) {
    let bytes = parse_xxd(hex_path);
    if bytes.len() != 3800 {
        panic!(
            "expected 3800 bytes in {} (G-descriptor cluster region_0569c0), got {}",
            hex_path.display(),
            bytes.len()
        );
    }
    let g5_pri_b_bytes = parse_xxd(g5_pri_b_path);
    if g5_pri_b_bytes.len() != 408 {
        panic!(
            "expected 408 bytes in {} (G5 pri_B = 102 × u32-LE), got {}",
            g5_pri_b_path.display(),
            g5_pri_b_bytes.len()
        );
    }

    // G4 pri_A: file 0x57630..0x57696 (102 bytes). Region base file 0x569c0
    // ⇒ slice offset 0xc70..0xcd6.
    const G4_PRIA_OFF: usize = 0xc70;
    const G4_PRIB_OFF: usize = 0xcd8;
    const G5_PRIA_OFF: usize = 0xe70;
    // G4 / G5 (count_A, count_B) pulled from the spec/15 §3
    // authoritative table — do NOT inline literal values here.
    // Per spec/15 §2.1 the constructor literals at VMAs `1c210810` (G4)
    // and `1c210862` (G5) store `0x66 = 102` for both `count_A`s;
    // `1c21081a` stores `0x39 = 57` for G4 `count_B` and `1c21086c`
    // stores `0x42 = 66` for G5 `count_B`. The local constants below
    // are cross-checked against `G_COUNTS_SPEC15` at build time so a
    // future drift on either side surfaces immediately.
    let (g4_ca_u32, g4_cb_u32) = G_COUNTS_SPEC15[4];
    let (g5_ca_u32, g5_cb_u32) = G_COUNTS_SPEC15[5];
    const G4_COUNT_A: usize = 102;
    const G4_COUNT_B: usize = 57;
    const G5_COUNT_A: usize = 102;
    const G5_COUNT_B: usize = 66;
    if G4_COUNT_A as u32 != g4_ca_u32 || G4_COUNT_B as u32 != g4_cb_u32 {
        panic!(
            "build.rs G-descriptor cluster: G4 ({G4_COUNT_A}, {G4_COUNT_B}) mismatch vs \
             spec/15 §3 G_COUNTS_SPEC15[4] = ({g4_ca_u32}, {g4_cb_u32}) — re-derive \
             from the constructor disassembly per spec/15 §2"
        );
    }
    if G5_COUNT_A as u32 != g5_ca_u32 || G5_COUNT_B as u32 != g5_cb_u32 {
        panic!(
            "build.rs G-descriptor cluster: G5 ({G5_COUNT_A}, {G5_COUNT_B}) mismatch vs \
             spec/15 §3 G_COUNTS_SPEC15[5] = ({g5_ca_u32}, {g5_cb_u32}) — re-derive \
             from the constructor disassembly per spec/15 §2"
        );
    }

    let g4_pri_a = &bytes[G4_PRIA_OFF..G4_PRIA_OFF + G4_COUNT_A];
    let g4_pri_b_bytes = &bytes[G4_PRIB_OFF..G4_PRIB_OFF + G4_COUNT_A * 4];
    let g5_pri_a = &bytes[G5_PRIA_OFF..G5_PRIA_OFF + G5_COUNT_A];

    // Decode pri_B as little-endian u32, low byte = run count. Audit/01
    // §2.4 / §4.4 confirm the upper 24 bits are always zero for G4 / G5.
    let g4_pri_b = decode_pri_b_u32_le(g4_pri_b_bytes, "G4 pri_B");
    let g5_pri_b = decode_pri_b_u32_le(&g5_pri_b_bytes, "G5 pri_B");
    if g4_pri_b.len() != G4_COUNT_A {
        panic!(
            "G4 pri_B decoded {} entries, expected {}",
            g4_pri_b.len(),
            G4_COUNT_A
        );
    }
    if g5_pri_b.len() != G5_COUNT_A {
        panic!(
            "G5 pri_B decoded {} entries, expected {}",
            g5_pri_b.len(),
            G5_COUNT_A
        );
    }

    // Sanity: pri_A must start with the canonical level prefix
    // `01 02 03 .. <LMAX>` for run=0 (per audit/01 §2.2 row 0..11 for G4
    // and audit/01 §4.1 row 0..26 for G5). G4 LMAX(run=0) = 12 ⇒ first 12
    // bytes are 01..0c.
    for (i, &b) in g4_pri_a.iter().take(12).enumerate() {
        let want = (i + 1) as u8;
        if b != want {
            panic!(
                "G4 pri_A[{}] = {:#x}, expected {:#x} (canonical level prefix); \
                 region_0569c0 slice may be misaligned",
                i, b, want
            );
        }
    }
    for (i, &b) in g5_pri_a.iter().take(27).enumerate() {
        let want = (i + 1) as u8;
        if b != want {
            panic!(
                "G5 pri_A[{}] = {:#x}, expected {:#x} (canonical level prefix); \
                 region_0569c0 slice may be misaligned",
                i, b, want
            );
        }
    }
    // Sanity: pri_B[0..LMAX(run=0)] must be all zero (run=0 entries).
    for (i, &v) in g4_pri_b.iter().take(12).enumerate() {
        if v != 0 {
            panic!(
                "G4 pri_B[{}] = {:#x}, expected 0 (run=0 prefix); region slice misaligned",
                i, v
            );
        }
    }
    for (i, &v) in g5_pri_b.iter().take(27).enumerate() {
        if v != 0 {
            panic!(
                "G5 pri_B[{}] = {:#x}, expected 0 (run=0 prefix); region_057898 misaligned",
                i, v
            );
        }
    }
    // Cross-check the partition: pri_B values strictly increase across
    // sub-class A then restart at zero or stay flat across sub-class B.
    // Sub-A's last entry (idx = count_B = 57) has run = 26 per audit/01
    // §2.2; sub-B's first entry (idx = 58) restarts at run = 0.
    if g4_pri_b[G4_COUNT_B] != 26 {
        panic!(
            "G4 pri_B[count_B={}] = {}, expected 26 (sub-A last entry, run=26); \
             slice may be misaligned",
            G4_COUNT_B, g4_pri_b[G4_COUNT_B]
        );
    }
    if g4_pri_b[G4_COUNT_B + 1] != 0 {
        panic!(
            "G4 pri_B[count_B+1={}] = {}, expected 0 (sub-B restart); \
             slice may be misaligned",
            G4_COUNT_B + 1,
            g4_pri_b[G4_COUNT_B + 1]
        );
    }
    // G5 partition cross-check: per audit/01 §4.1 sub-A's last entry
    // (idx 66 = count_B) is (run=14, level=1); sub-B's first entry
    // (idx 67) restarts at run=0.
    if g5_pri_b[G5_COUNT_B] != 14 {
        panic!(
            "G5 pri_B[count_B={}] = {}, expected 14 (sub-A last entry, run=14 \
             per audit/01 §4.1); region_057898 may be misaligned",
            G5_COUNT_B, g5_pri_b[G5_COUNT_B]
        );
    }
    if g5_pri_b[G5_COUNT_B + 1] != 0 {
        panic!(
            "G5 pri_B[count_B+1={}] = {}, expected 0 (sub-B restart per \
             audit/01 §4.1); region_057898 may be misaligned",
            G5_COUNT_B + 1,
            g5_pri_b[G5_COUNT_B + 1]
        );
    }
    // G5 sub-B last entry: idx 101 = (run=20, level=1) per audit/01 §4.1.
    if g5_pri_b[G5_COUNT_A - 1] != 20 {
        panic!(
            "G5 pri_B[{}] = {}, expected 20 (sub-B last entry, run=20 per \
             audit/01 §4.1); region_057898 may be misaligned",
            G5_COUNT_A - 1,
            g5_pri_b[G5_COUNT_A - 1]
        );
    }
    // G5 sub-B per-row LMAX cross-check against audit/01 §4.1:
    //   r0:8 entries (idx 67..74), r1:3 (75..77), r2..6:2 each (78..87),
    //   r7..20: 1 each (88..101).
    let mut g5_subb_expected_runs: Vec<u32> = Vec::with_capacity(35);
    g5_subb_expected_runs.extend(std::iter::repeat(0u32).take(8));
    g5_subb_expected_runs.extend(std::iter::repeat(1u32).take(3));
    for r in 2u32..=6 {
        g5_subb_expected_runs.push(r);
        g5_subb_expected_runs.push(r);
    }
    g5_subb_expected_runs.extend(7u32..=20);
    assert_eq!(g5_subb_expected_runs.len(), 35, "sub-B row count");
    for (i, &expected_run) in g5_subb_expected_runs.iter().enumerate() {
        let idx = G5_COUNT_B + 1 + i;
        if g5_pri_b[idx] != expected_run {
            panic!(
                "G5 pri_B[{idx}] = {}, expected {} (audit/01 §4.1 sub-B row {i})",
                g5_pri_b[idx], expected_run
            );
        }
    }

    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    writeln!(
        f,
        "// Auto-generated by build.rs from \
         crates/oxideav-msmpeg4/tables/region_0569c0.hex\n\
         // and crates/oxideav-msmpeg4/tables/region_057898.hex.\n\
         // DO NOT EDIT.\n\
         // Source binary: mpg4c32.dll SHA-256 \
         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099\n\
         // Roles (per spec/99 §5 / audit/01 §3 / §4):\n\
         //   G4 = MSMPEG4 inter-block DCT AC TCOEF run/level/last alphabet,\n\
         //        shape-equivalent to MPEG-4 Part 2 Table 11-16 Inter under\n\
         //        ESCL(b) LMAX. count_A=102, count_B=57. Live slot\n\
         //        [esi+0xab0] (chroma + all-inter); selected when\n\
         //        [esi+0xad0]=2 (default for v1/v2; v3 selector).\n\
         //   G5 = MSMPEG4 intra-block DCT AC TCOEF run/level/last alphabet,\n\
         //        shape-equivalent to MPEG-4 Part 2 Table 11-15 Intra under\n\
         //        ESCL(a) LMAX. count_A=102, count_B=66. Live slot\n\
         //        [esi+0xab4] (intra-luma); selected when [esi+0xad4]=2.\n\
         //        Round 19 fills G5 pri_B from region_057898.hex (the\n\
         //        408 bytes immediately following the cluster region).\n\
         \n\
         pub const G4_COUNT_A: usize = {};\n\
         pub const G4_COUNT_B: usize = {};\n\
         pub const G5_COUNT_A: usize = {};\n\
         pub const G5_COUNT_B: usize = {};\n\
         \n\
         /// G4 pri_A — `|level|` byte per symbol, indexed [0, count_A-1].\n\
         /// Sliced from region_0569c0 file offset 0x57630..0x57696 (102 bytes).\n\
         pub static G4_PRI_A: &[u8; {}] = &[",
        G4_COUNT_A, G4_COUNT_B, G5_COUNT_A, G5_COUNT_B, G4_COUNT_A,
    )
    .unwrap();
    emit_byte_array(&mut f, g4_pri_a);
    writeln!(f, "];\n").unwrap();

    writeln!(
        f,
        "/// G4 pri_B — `run` count per symbol (low byte of u32-LE record;\n\
         /// upper 24 bits zero per audit/01 §2.4). Sliced from region_0569c0\n\
         /// file offset 0x57698..0x57830 (408 bytes = 102 × u32-LE).\n\
         pub static G4_PRI_B: &[u8; {}] = &[",
        G4_COUNT_A,
    )
    .unwrap();
    let g4_pri_b_low: Vec<u8> = g4_pri_b.iter().map(|&v| v as u8).collect();
    emit_byte_array(&mut f, &g4_pri_b_low);
    writeln!(f, "];\n").unwrap();

    writeln!(
        f,
        "/// G5 pri_A — `|level|` byte per symbol, indexed [0, count_A-1].\n\
         /// Sliced from region_0569c0 file offset 0x57830..0x57896 (102 bytes).\n\
         pub static G5_PRI_A: &[u8; {}] = &[",
        G5_COUNT_A,
    )
    .unwrap();
    emit_byte_array(&mut f, g5_pri_a);
    writeln!(f, "];\n").unwrap();

    writeln!(
        f,
        "/// G5 pri_B — `run` count per symbol (low byte of u32-LE record;\n\
         /// upper 24 bits zero per audit/01 §4.4). Loaded from\n\
         /// region_057898.hex (file 0x57898..0x57a30, 408 bytes = 102 ×\n\
         /// u32-LE) — the gap immediately after the G-descriptor cluster.\n\
         pub static G5_PRI_B: &[u8; {}] = &[",
        G5_COUNT_A,
    )
    .unwrap();
    let g5_pri_b_low: Vec<u8> = g5_pri_b.iter().map(|&v| v as u8).collect();
    emit_byte_array(&mut f, &g5_pri_b_low);
    writeln!(f, "];").unwrap();
}

/// Decode a sequence of u32-LE records and assert each fits in a u8 (low
/// byte is the run count; upper 24 bits must be zero per audit/01 §2.4 and
/// §4.4). Returns the run count per record as a `u32` for downstream
/// numeric checks (the emitter casts to `u8` when writing the constant).
fn decode_pri_b_u32_le(bytes: &[u8], label: &str) -> Vec<u32> {
    let mut out: Vec<u32> = Vec::with_capacity(bytes.len() / 4);
    for (i, chunk) in bytes.chunks_exact(4).enumerate() {
        let v = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        if v > 0xff {
            panic!(
                "{label}[{i}] u32 {:#x} has bits above bit 7 — contradicts \
                 audit/01 §2.4 / §4.4 (extraction error or wrong slice offset)",
                v
            );
        }
        out.push(v);
    }
    out
}

/// Walk a packed-tiered-Huffman LUT (each 2-byte record is `(bit_length,
/// symbol)`, `2^max_bitlen` records total) and return a list of unique
/// `(symbol, bit_length, canonical_code)` triples sorted by symbol value.
/// Entries with `bit_length == 0` are sentinel slots (no valid code at
/// the corresponding bit pattern) and are skipped.
fn extract_lut_alphabet(lut: &[u8], max_bitlen: u32) -> Vec<(u8, u8, u32)> {
    let n_entries = 1usize << max_bitlen;
    assert!(
        lut.len() >= n_entries * 2,
        "LUT too small: {} bytes < {} expected for max_bitlen {}",
        lut.len(),
        n_entries * 2,
        max_bitlen,
    );
    let mut seen: std::collections::BTreeMap<u8, (u8, u32)> = std::collections::BTreeMap::new();
    for idx in 0..n_entries {
        let bl = lut[idx * 2];
        let sym = lut[idx * 2 + 1];
        if bl == 0 || (bl as u32) > max_bitlen {
            continue;
        }
        let code = (idx as u32) >> (max_bitlen - bl as u32);
        match seen.get(&sym) {
            None => {
                seen.insert(sym, (bl, code));
            }
            Some(&(prev_bl, prev_code)) => {
                if prev_bl != bl || prev_code != code {
                    panic!(
                        "LUT inconsistency at idx {idx}: sym={sym} previously \
                         (bl={prev_bl}, code=0x{prev_code:x}) now (bl={bl}, code=0x{code:x})"
                    );
                }
            }
        }
    }
    let mut out: Vec<(u8, u8, u32)> = seen
        .into_iter()
        .map(|(sym, (bl, code))| (sym, bl, code))
        .collect();
    out.sort_by_key(|&(sym, _, _)| sym);
    out
}

fn emit_byte_array<W: Write>(out: &mut W, bytes: &[u8]) {
    for chunk in bytes.chunks(16) {
        write!(out, "   ").unwrap();
        for b in chunk {
            write!(out, " 0x{:02x},", b).unwrap();
        }
        writeln!(out).unwrap();
    }
}

/// Parse a packed-Huffman primary-VLC source file (round 26 — spec/11
/// §3 / §4) and emit a Rust constants file holding the per-symbol
/// `(bit_length, code_value)` array plus the alphabet count and ESC
/// index for downstream canonical-Huffman building.
///
/// Format (verbatim from spec/11 §4):
///
/// ```text
/// +0x00 : count : u32-LE                    ; alphabet size (incl. ESC)
/// +0x04 : records[0..count] : 8 B each      ; (a:u32-LE, b:u32-LE)
///           a == 0xFFFFFFFF → "hole" sentinel: stored as (0,0); 8 B still consumed
///           otherwise       → (a, b) where a = code_value (state byte)
///                                    and  b = bit_length
/// ```
///
/// Total source size in bytes = `4 + count * 8`. Both G4 (`region_058e38`)
/// and G5 (`region_059178`) have `count = 103` ⇒ 828 bytes. One bl=9
/// codeword in each is reserved for the ESC marker (Kraft sum 1 - 2/1024
/// = 0.998047), giving the alphabet shape `count_A = 102 entries +
/// 1 ESC sentinel at idx 102` quoted in spec/11 §5 / §7.
///
/// **Provenance:** verbatim slice from the binary; bytes were captured
/// into `tables/region_<off>_full.hex` via Python at session start (no
/// numeric typing). Build-time invariants enforce:
///   * file length == 828
///   * count == 103
///   * each bl is in [0, 16] (the binary's max-bitlen budget)
///   * Kraft sum over the 103 bls equals 1 - 2/1024 in 2^-bl arithmetic
fn emit_packed_huffman_primary(
    hex_path: &Path,
    out_path: &Path,
    label: &str,
    file_off: u32,
    vma: u32,
) {
    let bytes = parse_xxd(hex_path);
    if bytes.len() != 828 {
        panic!(
            "{label}: expected 828 bytes in {} (4-byte count + 103 * 8 records), got {}",
            hex_path.display(),
            bytes.len()
        );
    }
    let count = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    if count != 103 {
        panic!(
            "{label}: header count {count} != 103 (expected G-table alphabet shape per spec/11 §5)",
        );
    }
    let mut records: Vec<(u32, u32)> = Vec::with_capacity(count as usize);
    for i in 0..count as usize {
        let off = 4 + i * 8;
        let a = u32::from_le_bytes([bytes[off], bytes[off + 1], bytes[off + 2], bytes[off + 3]]);
        let b = u32::from_le_bytes([
            bytes[off + 4],
            bytes[off + 5],
            bytes[off + 6],
            bytes[off + 7],
        ]);
        // Per spec/11 §4: a == 0xFFFFFFFF is a hole sentinel; helper A
        // stores the record as (0, 0) but still advances 8 bytes. Mirror
        // that here so a hole symbol drops out of the canonical-Huffman
        // builder (bl == 0 ⇒ filtered).
        let (a, b) = if a == 0xFFFF_FFFF {
            (0u32, 0u32)
        } else {
            (a, b)
        };
        if b > 16 {
            panic!(
                "{label}: record {i} has bit_length {b} > 16 — exceeds the binary's max-bitlen \
                 budget; extraction misalignment suspected"
            );
        }
        records.push((b, a)); // store as (bit_length, code_value) for parity with INTRA_AC_V3_CANDIDATE_RAW
    }

    // Kraft sum check. Per spec/11 §5 both G4 and G5 saturate at
    // `1 - 1/512 = 0.998047` (= "2/1024" in the spec wording, where the
    // unit is 2^-10; equivalently one bl=9 codeword reserved for ESC).
    // We verify the exact value so a future extraction drift (different
    // binary version, wrong slice offsets) trips the build.
    let max_bl_for_kraft: u32 = 32; // arithmetic header; not the table's max bl
    let target: u64 = (1u64 << max_bl_for_kraft) - (1u64 << (max_bl_for_kraft - 9));
    let sum: u64 = records
        .iter()
        .filter(|&&(bl, _)| bl > 0)
        .map(|&(bl, _)| 1u64 << (max_bl_for_kraft - bl))
        .sum();
    if sum != target {
        panic!(
            "{label}: Kraft sum {sum} != expected {target} (= 1 - 2/1024 in 2^-bl fixed point) — \
             primary VLC source is not the round-26 G-table shape; spec/11 §5 lists \
             count=103 with one bl=9 ESC reservation"
        );
    }

    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    let label_lc = label.to_lowercase();
    writeln!(
        f,
        "// Auto-generated by build.rs from \
         crates/oxideav-msmpeg4/tables/region_{file_off:06x}_full.hex.\n\
         // DO NOT EDIT.\n\
         // Source binary: mpg4c32.dll SHA-256 \
         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099\n\
         // Region: file offset 0x{file_off:06x}, VMA 0x{vma:08x}, 828 bytes (4 + 103 * 8).\n\
         // Format (per docs/video/msmpeg4/spec/11-walker-format-resolved.md §4):\n\
         //   u32-LE count = 103, then 103 * (a:u32-LE = code_value, b:u32-LE = bit_length).\n\
         // Role: {label} primary VLC source for the DCT AC TCOEF (run, level, last) walk.\n\
         // ESC index 102 is reserved (Kraft sum = 1 - 2/1024 over the 103 bit-lengths,\n\
         // leaving exactly one bl=9 codeword for the ESC sentinel — spec/11 §5 / §7 item 4).\n\
         \n\
         pub const {label}_PRIMARY_ALPHABET: usize = 103;\n\
         pub const {label}_PRIMARY_ESC_INDEX: usize = 102;\n\
         \n\
         /// 103 * (bit_length, code_value) pairs for the {label} primary VLC.\n\
         /// Index in the array == symbol index passed to the post-VLC\n\
         /// `(idx -> (last, run, level))` map (spec/04 §1.3 step 3 / spec/99 §5).\n\
         /// Symbol 102 is ESC; the canonical-Huffman builder still emits it as\n\
         /// a regular leaf (the bit_length is non-zero) and the consumer maps\n\
         /// the decoded idx == 102 to the escape body via [`g_descriptor::g{label_lc_digit}_decode`].\n\
         pub const {label}_PRIMARY_RAW: &[(u32, u32)] = &[",
        label = label,
        label_lc_digit = label_lc.trim_start_matches('g'),
        file_off = file_off,
        vma = vma,
    )
    .unwrap();
    for &(bl, code) in &records {
        writeln!(f, "    ({bl}, {code}),").unwrap();
    }
    writeln!(f, "];").unwrap();
}

/// Parse a G0..G3 packed-Huffman primary-VLC source (round 234).
///
/// Identical record layout to [`emit_packed_huffman_primary`] (G4/G5)
/// per `docs/video/msmpeg4/spec/11-walker-format-resolved.md` §3 / §4:
///
/// ```text
/// +0x00 : count : u32-LE                    ; alphabet size (incl. ESC at idx = count-1)
/// +0x04 : records[0..count] : 8 B each      ; (a:u32-LE, b:u32-LE)
///           a == 0xFFFFFFFF → hole sentinel: stored as (0,0); 8 B still consumed
///           otherwise       → (a, b) = (code_value, bit_length)
/// ```
///
/// Differences from the G4/G5 emitter:
///
/// * `count` is a per-table caller-supplied invariant (169 / 186 / 149 / 133)
///   rather than a hard-coded 103 — the spec/11 §5 table pins each.
/// * The Kraft sum is verified to be **exactly 1** (saturated, no ESC
///   codeword reserved) — spec/11 §5 reports "Kraft = 1.00000" for the
///   four G-extended sources (vs G4/G5's 0.998047 = 1 - 2/1024).
/// * The source file may include trailing alignment padding (the
///   binary's per-slot region is aligned to 4 B; some sources have 4
///   trailing bytes past the last record). The emitter consumes exactly
///   `4 + count * 8` bytes and ignores any trailing padding.
///
/// Build-time invariants enforced:
///   * file length >= `4 + count * 8`
///   * header count == caller-supplied `expected_count`
///   * each bl in `[0, 16]` (the binary's max-bitlen budget; spec/11 §5
///     reports max_bl values 13..15 for the four sources)
///   * Kraft sum over the bls equals `1` exactly in 2^-bl arithmetic
fn emit_packed_huffman_g_extended(
    hex_path: &Path,
    out_path: &Path,
    label: &str,
    expected_count: u32,
    file_off: u32,
    vma: u32,
) {
    let bytes = parse_xxd(hex_path);
    let needed = 4 + (expected_count as usize) * 8;
    if bytes.len() < needed {
        panic!(
            "{label}: expected at least {needed} bytes in {} (4-byte count + {expected_count} * 8 records), got {}",
            hex_path.display(),
            bytes.len()
        );
    }
    let count = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    if count != expected_count {
        panic!(
            "{label}: header count {count} != {expected_count} (expected per spec/11 §5 row for {label})",
        );
    }
    let mut records: Vec<(u32, u32)> = Vec::with_capacity(count as usize);
    for i in 0..count as usize {
        let off = 4 + i * 8;
        let a = u32::from_le_bytes([bytes[off], bytes[off + 1], bytes[off + 2], bytes[off + 3]]);
        let b = u32::from_le_bytes([
            bytes[off + 4],
            bytes[off + 5],
            bytes[off + 6],
            bytes[off + 7],
        ]);
        let (a, b) = if a == 0xFFFF_FFFF {
            (0u32, 0u32)
        } else {
            (a, b)
        };
        if b > 16 {
            panic!(
                "{label}: record {i} has bit_length {b} > 16 — exceeds the binary's max-bitlen \
                 budget; extraction misalignment suspected"
            );
        }
        records.push((b, a)); // store as (bit_length, code_value)
    }

    // Kraft sum check. Per spec/11 §5 G0..G3 all saturate to exactly 1
    // (no ESC codeword reservation — the ESC entry occupies a regular
    // bit-length slot at idx = count - 1).
    let max_bl_for_kraft: u32 = 32;
    let target: u64 = 1u64 << max_bl_for_kraft;
    let sum: u64 = records
        .iter()
        .filter(|&&(bl, _)| bl > 0)
        .map(|&(bl, _)| 1u64 << (max_bl_for_kraft - bl))
        .sum();
    if sum != target {
        panic!(
            "{label}: Kraft sum {sum} != expected {target} (= 1.0 in 2^-bl fixed point) — \
             {label} packed-Huffman source is not a saturated prefix code; spec/11 §5 \
             reports Kraft=1.00000 for {label}"
        );
    }

    let esc_index = (count - 1) as usize;
    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    writeln!(
        f,
        "// Auto-generated by build.rs from \
         crates/oxideav-msmpeg4/tables/region_{file_off:06x}_full.hex.\n\
         // DO NOT EDIT.\n\
         // Source binary: mpg4c32.dll SHA-256 \
         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099\n\
         // Region: file offset 0x{file_off:06x}, VMA 0x{vma:08x}, {} bytes (4 + {count} * 8).\n\
         // Format (per docs/video/msmpeg4/spec/11-walker-format-resolved.md §4):\n\
         //   u32-LE count = {count}, then {count} * (a:u32-LE = code_value, b:u32-LE = bit_length).\n\
         // Role: {label} primary VLC source for the DCT AC TCOEF (run, level, last) walk.\n\
         // ESC index {esc_index} occupies a regular Kraft-saturated codeword slot (Kraft = 1).\n\
         \n\
         pub const {label}_PRIMARY_ALPHABET: usize = {count};\n\
         pub const {label}_PRIMARY_ESC_INDEX: usize = {esc_index};\n\
         \n\
         /// {count} * (bit_length, code_value) pairs for the {label} primary VLC.\n\
         /// Index in the array == symbol index passed to the post-VLC\n\
         /// `(idx -> (last, run, level))` map (spec/04 §1.3 step 3 / spec/09).\n\
         /// Symbol {esc_index} is ESC; the canonical-Huffman builder emits it as\n\
         /// a regular leaf and the consumer maps the decoded idx == {esc_index} to the\n\
         /// escape body via [`g_enum::g{label_lc_digit}_decode`].\n\
         pub const {label}_PRIMARY_RAW: &[(u32, u32)] = &[",
        4 + count * 8,
        count = count,
        esc_index = esc_index,
        label = label,
        label_lc_digit = label.to_lowercase().trim_start_matches('g'),
        file_off = file_off,
        vma = vma,
    )
    .unwrap();
    for &(bl, code) in &records {
        writeln!(f, "    ({bl}, {code}),").unwrap();
    }
    writeln!(f, "];").unwrap();
}

/// Parse an MS-MPEG4 intra-DC packed-Huffman source.
///
/// Per `docs/video/msmpeg4/spec/11-walker-format-resolved.md` §3-§4
/// the source layout is identical for ALL 14 packed-Huffman slots
/// (G4 / G5 / MCBPCY / MV / intra-DC):
///
/// ```text
/// +0x00 : count : u32                       ; alphabet size
/// +0x04 : records[0..count] : <8 B each>
///           (a:u32, b:u32) where
///             a == 0xFFFFFFFF → hole sentinel: stored as (0,0)
///             otherwise       → (a, b) = (canonical_code_value, bit_length)
/// ```
///
/// The decoded `idx` from a successful VLC walk is the symbol's
/// position in the records array (per spec/07 §5.4 for intra-DC):
/// idx 0..118 → DC differential magnitude (±idx with a separately
/// read sign bit); idx 119 → ESC sentinel that triggers an 8-bit
/// raw + sign read.
///
/// We emit a `&[(u32, u32)]` of `(bit_length, code_value)` records
/// for parity with the runtime conventions used by the rest of the
/// crate (G4/G5 emitter and the candidate region_05eed0 emitter both
/// store `(bl, code)`).
fn emit_intra_dc_vlc(hex_path: &Path, out_path: &Path, label: &str, file_off: u32, vma: u32) {
    let bytes = parse_xxd(hex_path);
    // 4-byte u32-LE count header + 120 * 8-byte records = 964 bytes
    // total. The xxd extract may include trailing padding from the
    // adjacent region (this is fine — we only consume `4 + 120 * 8`).
    let needed = 4 + 120 * 8;
    if bytes.len() < needed {
        panic!(
            "{label}: expected at least {needed} bytes in {} (4-byte count + 120 * 8 records), got {}",
            hex_path.display(),
            bytes.len()
        );
    }
    let header_count = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    if header_count != 120 {
        panic!(
            "{label}: header count {header_count} != 120 (expected MS-MPEG4 intra-DC alphabet shape per spec/07 §5.4)",
        );
    }
    let mut records: Vec<(u32, u32)> = Vec::with_capacity(header_count as usize);
    for i in 0..header_count as usize {
        // Payload starts at offset 4 (immediately after the count u32).
        let off = 4 + i * 8;
        let a = u32::from_le_bytes([bytes[off], bytes[off + 1], bytes[off + 2], bytes[off + 3]]);
        let b = u32::from_le_bytes([
            bytes[off + 4],
            bytes[off + 5],
            bytes[off + 6],
            bytes[off + 7],
        ]);
        // Per spec/11 §4: a == 0xFFFFFFFF marks a hole; helper A
        // writes (0, 0) but advances 8 source bytes either way.
        let (code, bl) = if a == 0xFFFF_FFFF {
            (0u32, 0u32)
        } else {
            // Canonical convention per spec/11 §4 / §6: a = code_value,
            // b = bit_length. Same convention as G4/G5/MCBPCY/MV.
            (a, b)
        };
        if bl > 26 {
            panic!(
                "{label}: record {i} has bit_length {bl} > 26 — exceeds the binary's max-bitlen \
                 budget for these tables; extraction misalignment suspected"
            );
        }
        records.push((bl, code));
    }

    // Kraft sum check — must equal exactly 1 in 2^-bl arithmetic.
    // Carry the arithmetic in 2^-32 fixed-point to keep margin.
    let max_bl_for_kraft: u32 = 32;
    let target: u64 = 1u64 << max_bl_for_kraft;
    let sum: u64 = records
        .iter()
        .filter(|&&(bl, _)| bl > 0)
        .map(|&(bl, _)| 1u64 << (max_bl_for_kraft - bl))
        .sum();
    if sum != target {
        panic!(
            "{label}: Kraft sum {sum} != expected {target} (= 1 in 2^-32 fixed point) — \
             intra-DC VLC source is not a complete prefix code under the spec/11 §4 \
             (a=code, b=bl) convention."
        );
    }

    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    writeln!(
        f,
        "// Auto-generated by build.rs from \
         crates/oxideav-msmpeg4/tables/region_{file_off:06x}.hex.\n\
         // DO NOT EDIT.\n\
         // Source binary: mpg4c32.dll SHA-256 \
         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099\n\
         // Region: file offset 0x{file_off:06x}, VMA 0x{vma:08x}, 964 bytes (4 + 120 * 8).\n\
         // Format (per docs/video/msmpeg4/spec/11-walker-format-resolved.md §4 +\n\
         // docs/video/msmpeg4/spec/07-remaining-opens.md §5.4):\n\
         //   u32-LE count = 120, then 120 * (a:u32-LE = code_value, b:u32-LE = bit_length).\n\
         // Role: {label} intra-DC differential VLC source.\n\
         // Symbol idx 0..118 = direct DC differential magnitude (±idx with separate sign bit).\n\
         // Symbol idx 119 = ESC sentinel (decoder reads 8-bit raw + sign).\n\
         // Kraft sum over the 120 bit-lengths = 1 exactly (complete prefix code).\n\
         \n\
         pub const {label}_ALPHABET: usize = 120;\n\
         pub const {label}_ESC_INDEX: usize = 119;\n\
         \n\
         /// 120 * (bit_length, code_value) pairs for the {label} intra-DC VLC.\n\
         /// Index in the array == decoded symbol = differential magnitude\n\
         /// (with sign bit read separately) for idx 0..118; idx == 119 is ESC.\n\
         pub const {label}_RAW: &[(u32, u32)] = &[",
    )
    .unwrap();
    for &(bl, code) in &records {
        writeln!(f, "    ({bl}, {code}),").unwrap();
    }
    writeln!(f, "];").unwrap();
}

/// Parse one of the G0..G3 enumeration CSVs and emit a static Rust
/// constant of `(run, level_mag, last)` triples plus the count_A /
/// count_B header constants.
///
/// CSV format (per docs/video/msmpeg4/tables/region_<addr>_g<N>_enum.csv,
/// produced by extract-06.sh):
///
///   `idx,sub_class,last,run,level,pri_A_byte,pri_B_u32_hex,...`
///
/// Rows 0..count_A-1 hold the alphabet (sub-A 0..count_B then sub-B
/// count_B+1..count_A-1). Row `count_A` is the ESC sentinel
/// (`idx,ESC,-,-,-,-,-,...`).
///
/// The emitted table has length `count_A` (no ESC entry — callers test
/// `idx == count_A` separately, matching `g_descriptor::g4_decode` /
/// `g5_decode` semantics).
///
/// Per spec/09 §3-§7 / §11 there are no holes: every (last, run) class
/// enumerates `1, 2, ..., LMAX[last][run]` with no gaps. This function
/// asserts that property at build time.
fn emit_g_enum(
    csv_path: &Path,
    out_path: &Path,
    label: &str,
    expected_count_a: usize,
    expected_count_b: usize,
) {
    let text = fs::read_to_string(csv_path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", csv_path.display()));

    // Each entry: (idx, sub_class_char, last_flag, run, level_mag).
    let mut entries: Vec<(usize, char, u8, u8, u8)> = Vec::with_capacity(expected_count_a + 1);
    for (line_no, line) in text.lines().enumerate() {
        if line_no == 0 {
            // CSV column header.
            continue;
        }
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let parts: Vec<&str> = line.split(',').collect();
        if parts.len() < 5 {
            panic!(
                "{label}: malformed row at line {} in {}: {line}",
                line_no + 1,
                csv_path.display()
            );
        }
        let idx: usize = parts[0]
            .parse()
            .unwrap_or_else(|_| panic!("{label}: bad idx at line {}: {}", line_no + 1, parts[0]));
        let sub_class = parts[1];
        if sub_class == "ESC" {
            entries.push((idx, 'E', 0, 0, 0));
            continue;
        }
        let class_char = match sub_class {
            "A" => 'A',
            "B" => 'B',
            other => panic!(
                "{label}: unknown sub_class '{other}' at line {}",
                line_no + 1
            ),
        };
        let last_flag: u8 = parts[2]
            .parse()
            .unwrap_or_else(|_| panic!("{label}: bad last at line {}: {}", line_no + 1, parts[2]));
        let run: u8 = parts[3]
            .parse()
            .unwrap_or_else(|_| panic!("{label}: bad run at line {}: {}", line_no + 1, parts[3]));
        let level: u8 = parts[4]
            .parse()
            .unwrap_or_else(|_| panic!("{label}: bad level at line {}: {}", line_no + 1, parts[4]));
        entries.push((idx, class_char, last_flag, run, level));
    }

    // Validate header — last row must be ESC at index count_A.
    let esc_row = entries
        .last()
        .unwrap_or_else(|| panic!("{label}: empty CSV {}", csv_path.display()));
    if esc_row.1 != 'E' {
        panic!(
            "{label}: last row is not ESC sentinel: {esc_row:?} ({})",
            csv_path.display()
        );
    }
    let count_a = esc_row.0;
    if count_a != expected_count_a {
        panic!("{label}: count_A from CSV ({count_a}) != expected ({expected_count_a})");
    }
    if entries.len() != expected_count_a + 1 {
        panic!(
            "{label}: expected {} rows (count_A + ESC), got {}",
            expected_count_a + 1,
            entries.len()
        );
    }

    // Validate sub-class partition + emit the (run, level, last) tuples
    // in symbol-index order.
    let mut payload: Vec<(u8, u8, u8)> = Vec::with_capacity(expected_count_a);
    for (i, e) in entries[..expected_count_a].iter().enumerate() {
        if e.0 != i {
            panic!(
                "{label}: row {i} has idx field {} (table not contiguous)",
                e.0
            );
        }
        let expected_class = if i <= expected_count_b { 'A' } else { 'B' };
        if e.1 != expected_class {
            panic!(
                "{label}: idx {i} has sub_class '{}' but expected '{expected_class}' \
                 (count_B={expected_count_b})",
                e.1
            );
        }
        let expected_last = if i <= expected_count_b { 0 } else { 1 };
        if e.2 != expected_last {
            panic!(
                "{label}: idx {i} has last={} but expected {expected_last}",
                e.2
            );
        }
        payload.push((e.3, e.4, e.2));
    }

    // Cross-check the gap-free property (spec/09 §9): for every observed
    // (last, run) class, levels enumerate 1..=LMAX with no holes.
    let mut max_level_per_class: std::collections::BTreeMap<(u8, u8), u8> =
        std::collections::BTreeMap::new();
    let mut levels_seen_per_class: std::collections::BTreeMap<(u8, u8), Vec<u8>> =
        std::collections::BTreeMap::new();
    for &(run, level, last) in &payload {
        let key = (last, run);
        let lev_list = levels_seen_per_class.entry(key).or_default();
        lev_list.push(level);
        let cur = max_level_per_class.entry(key).or_insert(0);
        if level > *cur {
            *cur = level;
        }
    }
    for (key, levels) in &levels_seen_per_class {
        let lmax = max_level_per_class[key];
        let mut sorted = levels.clone();
        sorted.sort_unstable();
        let expected: Vec<u8> = (1..=lmax).collect();
        if sorted != expected {
            panic!(
                "{label}: gap-free violation at (last={}, run={}) — observed {sorted:?}, \
                 expected {expected:?}. spec/09 §9 asserts levels enumerate 1..=LMAX.",
                key.0, key.1
            );
        }
    }

    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    writeln!(
        f,
        "// Auto-generated by build.rs from \
         crates/oxideav-msmpeg4/tables/{csv_name}.\n\
         // DO NOT EDIT.\n\
         // Source: docs/video/msmpeg4/tables/{csv_name} (extract-06.sh,\n\
         // round 29). Cleanroom narrative: docs/video/msmpeg4/spec/09-g0-g3-enumeration.md.\n\
         // Source binary: mpg4c32.dll SHA-256 \
         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099\n\
         // Format: each entry is (run:u8, level_mag:u8, last:u8) per\n\
         // primary-VLC symbol index. Index range: [0, {label}_COUNT_A).\n\
         // ESC sentinel lives at idx == {label}_COUNT_A and is NOT in the\n\
         // array (callers test the boundary explicitly per spec/04 §1.3 step 3).\n\
         // Sub-class partition (spec/13 §2): idx <= {label}_COUNT_B is sub-A\n\
         // (last=0, continues block); idx > {label}_COUNT_B is sub-B (last=1,\n\
         // terminates block).\n\
         \n\
         pub const {label}_COUNT_A: usize = {expected_count_a};\n\
         pub const {label}_COUNT_B: usize = {expected_count_b};\n\
         \n\
         /// (run, level_mag, last) for each primary-VLC symbol index.\n\
         pub const {label}_ENUM: &[(u8, u8, u8); {expected_count_a}] = &[",
        csv_name = csv_path.file_name().unwrap().to_string_lossy(),
    )
    .unwrap();
    for &(run, level, last) in &payload {
        writeln!(f, "    ({run}, {level}, {last}),").unwrap();
    }
    writeln!(f, "];").unwrap();
}

/// Parse an xxd hex-dump file (format: `<offset>: <byte-pairs> <ascii>`)
/// and return the decoded byte stream.
fn parse_xxd(path: &Path) -> Vec<u8> {
    let text = fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()));
    let mut out: Vec<u8> = Vec::new();
    for (line_no, line) in text.lines().enumerate() {
        let line = line.trim_end();
        if line.is_empty() {
            continue;
        }
        // Grab everything after the first ':' up to the (optional) ascii
        // gutter. xxd's gutter is two spaces then the ascii; the hex
        // section is before that.
        let after_colon = match line.split_once(':') {
            Some((_, rest)) => rest,
            None => panic!("line {} in {} has no colon", line_no + 1, path.display()),
        };
        // xxd separates the hex section from the ASCII gutter with two
        // or more consecutive spaces. Split on that double-space.
        let hex_part = match after_colon.find("  ") {
            Some(idx) => &after_colon[..idx],
            None => after_colon,
        };
        for tok in hex_part.split_whitespace() {
            // Each token is 1..=4 hex chars (xxd groups two bytes at a
            // time by default: `2020` = two bytes 0x20, 0x20). Parse as
            // a stream of 2-char hex pairs.
            if tok.len() % 2 != 0 {
                panic!(
                    "odd hex token '{tok}' at line {} in {}",
                    line_no + 1,
                    path.display()
                );
            }
            for i in (0..tok.len()).step_by(2) {
                let pair = &tok[i..i + 2];
                let b = u8::from_str_radix(pair, 16).unwrap_or_else(|_| {
                    panic!(
                        "bad hex pair '{pair}' at line {} in {}",
                        line_no + 1,
                        path.display()
                    )
                });
                out.push(b);
            }
        }
    }
    out
}

/// Parse `region_060988.hex` (the ESC-extension table cluster) and
/// `region_060988_index.csv` (the per-slice byte boundaries) and emit
/// per-G-descriptor byte slices for the 4-pointer descriptor block.
///
/// Per `docs/video/msmpeg4/spec/08-descriptor-constants.md` §2.2 +
/// §2.4 / `spec/14-pri-ab-runtime-binding.md` §2.1, the 24 cluster
/// slices map to (G, descriptor-offset) tuples as follows:
///
/// ```text
/// G0  +0x0c=0x1c261710 (slice 4)   +0x10=0x1c261780 (slice 5)
/// G0  +0x14=0x1c261818 (slice 6)   +0x18=0x1c261878 (slice 7)
/// G1  +0x0c=0x1c261588 (slice 0)   +0x10=0x1c261608 (slice 1)
/// G1  +0x14=0x1c2616a0 (slice 2)   +0x18=0x1c2616f0 (slice 3)
/// G2  +0x0c=0x1c2619c8 (slice 12)  +0x10=0x1c261a40 (slice 13)
/// G2  +0x14=0x1c261af0 (slice 14)  +0x18=0x1c261b30 (slice 15)
/// G3  +0x0c=0x1c2618a0 (slice 8)   +0x10=0x1c2618f8 (slice 9)
/// G3  +0x14=0x1c261968 (slice 10)  +0x18=0x1c2619b0 (slice 11)
/// G4  +0x0c=0x1c261c78 (slice 20)  +0x10=0x1c261ce8 (slice 21)
/// G4  +0x14=0x1c261d90 (slice 22)  +0x18=0x1c261dc8 (slice 23)
/// G5  +0x0c=0x1c261b48 (slice 16)  +0x10=0x1c261b88 (slice 17)
/// G5  +0x14=0x1c261be0 (slice 18)  +0x18=0x1c261c50 (slice 19)
/// ```
///
/// The function emits a 24-element `ESC_EXT_SLICES: [&[u8]; 24]` in
/// slice-index order plus six 4-tuple `ESC_EXT_G{n}_SLICE_INDICES:
/// [usize; 4]` constants giving the slice-array indices that match
/// each G-descriptor's `(+0x0c, +0x10, +0x14, +0x18)` pointer block.
/// Build-time invariants enforced:
///   * cluster size matches the meta (`length_bytes: 2168`)
///   * 24 slice rows in the CSV
///   * slice boundaries reconstruct the cluster byte-for-byte
///   * each slice length is a multiple of 8 (the record stride per
///     `region_060988.meta`)
///   * VMAs in `region_060988_index.csv` match the
///     spec/08 §2.2 / spec/14 §2.1 G-descriptor table above
fn emit_esc_ext_cluster(hex_path: &Path, csv_path: &Path, out_path: &Path) {
    let bytes = parse_xxd(hex_path);
    if bytes.len() != 2168 {
        panic!(
            "ESC-ext cluster: expected 2168 bytes in {} per spec/08 §2.4 (range 0x60988..0x61200), got {}",
            hex_path.display(),
            bytes.len()
        );
    }

    // Parse the slice index CSV. Columns:
    //   slice_index, relative_offset, absolute_file_offset_hex,
    //   vma_hex, next_slice_relative_offset, slice_length_bytes
    let csv_text = fs::read_to_string(csv_path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", csv_path.display()));
    let mut slices: Vec<(usize, u32, usize)> = Vec::new(); // (rel_off, vma, len)
    for (lineno, line) in csv_text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with("slice_index") {
            continue;
        }
        let cols: Vec<&str> = line.split(',').collect();
        if cols.len() < 6 {
            panic!(
                "{}:{}: expected 6 columns, got {}: {line:?}",
                csv_path.display(),
                lineno + 1,
                cols.len()
            );
        }
        let rel_off: usize = cols[1].parse().unwrap_or_else(|_| {
            panic!(
                "{}:{}: bad rel_off {:?}",
                csv_path.display(),
                lineno + 1,
                cols[1]
            )
        });
        let vma_hex = cols[3].trim_start_matches("0x");
        let vma = u32::from_str_radix(vma_hex, 16).unwrap_or_else(|_| {
            panic!(
                "{}:{}: bad vma {:?}",
                csv_path.display(),
                lineno + 1,
                cols[3]
            )
        });
        let len: usize = cols[5].parse().unwrap_or_else(|_| {
            panic!(
                "{}:{}: bad slice_length {:?}",
                csv_path.display(),
                lineno + 1,
                cols[5]
            )
        });
        slices.push((rel_off, vma, len));
    }
    if slices.len() != 24 {
        panic!(
            "ESC-ext cluster: expected 24 slices in {}, got {}",
            csv_path.display(),
            slices.len()
        );
    }

    // Verify boundaries reconstruct the cluster contiguously and each
    // slice length is a multiple of 8.
    let mut expected_rel = 0usize;
    for (i, &(rel_off, _vma, len)) in slices.iter().enumerate() {
        if rel_off != expected_rel {
            panic!(
                "ESC-ext slice {i}: rel_off {rel_off} != expected {expected_rel} \
                 (slices must be contiguous per region_060988.meta)"
            );
        }
        if len % 8 != 0 {
            panic!(
                "ESC-ext slice {i}: length {len} is not a multiple of 8 \
                 (records are 8 bytes per region_060988.meta)"
            );
        }
        if rel_off + len > bytes.len() {
            panic!(
                "ESC-ext slice {i}: rel_off+len = {} exceeds cluster size {}",
                rel_off + len,
                bytes.len()
            );
        }
        expected_rel += len;
    }
    if expected_rel != bytes.len() {
        panic!(
            "ESC-ext cluster: slice spans sum to {} but cluster has {} bytes \
             (region_060988_index.csv must cover the cluster exactly)",
            expected_rel,
            bytes.len()
        );
    }

    // Per-G slice-index attribution per spec/08 §2.2 (slice index =
    // (vma - 0x1c261588) / 8 conceptually, but we cross-check directly
    // against the CSV's vma column).
    let g_table: &[(&str, [u32; 4])] = &[
        ("G0", [0x1c261710, 0x1c261780, 0x1c261818, 0x1c261878]),
        ("G1", [0x1c261588, 0x1c261608, 0x1c2616a0, 0x1c2616f0]),
        ("G2", [0x1c2619c8, 0x1c261a40, 0x1c261af0, 0x1c261b30]),
        ("G3", [0x1c2618a0, 0x1c2618f8, 0x1c261968, 0x1c2619b0]),
        ("G4", [0x1c261c78, 0x1c261ce8, 0x1c261d90, 0x1c261dc8]),
        ("G5", [0x1c261b48, 0x1c261b88, 0x1c261be0, 0x1c261c50]),
    ];
    let mut g_slice_idx: Vec<(&str, [usize; 4])> = Vec::with_capacity(6);
    for &(name, vmas) in g_table {
        let mut idxs = [0usize; 4];
        for (k, &want_vma) in vmas.iter().enumerate() {
            let pos = slices
                .iter()
                .position(|&(_, vma, _)| vma == want_vma)
                .unwrap_or_else(|| {
                    panic!(
                        "ESC-ext cluster: descriptor {name} slot {k} VMA 0x{want_vma:08x} \
                         not found in region_060988_index.csv \
                         (spec/08 §2.2 / spec/14 §2.1 lookup failed)"
                    )
                });
            idxs[k] = pos;
        }
        g_slice_idx.push((name, idxs));
    }

    // Emit the file.
    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    writeln!(
        f,
        "// Auto-generated by build.rs from \
         crates/oxideav-msmpeg4/tables/region_060988.hex +\n\
         // crates/oxideav-msmpeg4/tables/region_060988_index.csv.\n\
         // DO NOT EDIT.\n\
         // Source binary: mpg4c32.dll SHA-256 \
         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099\n\
         // Region: file 0x060988..0x061200, VMA 0x1c261588..0x1c261e00, 2168 bytes.\n\
         // Role per docs/video/msmpeg4/spec/08-descriptor-constants.md §1-§2:\n\
         //   24 contiguous slices, one per (G-descriptor × {{+0x0c, +0x10,\n\
         //   +0x14, +0x18}}) tuple. Inter / intra kernels read these as base\n\
         //   pointers for the first- and second-tier ESC bodies (level- and\n\
         //   run-extension lookup arrays). Detailed slice-content semantics\n\
         //   remain spec-OPEN per spec/08 §4.1; this file wires the byte\n\
         //   slices and per-descriptor attribution only.\n\
         "
    )
    .unwrap();

    writeln!(
        f,
        "/// Number of ESC-extension cluster slices.\n\
         pub const ESC_EXT_SLICE_COUNT: usize = 24;\n\n\
         /// Total cluster size in bytes (per `region_060988.meta`).\n\
         pub const ESC_EXT_CLUSTER_BYTES: usize = 2168;\n"
    )
    .unwrap();

    // Emit each slice as a `&[u8]` constant + the master `ESC_EXT_SLICES`
    // array.
    for (i, &(rel_off, vma, len)) in slices.iter().enumerate() {
        writeln!(
            f,
            "/// Slice {i}: VMA 0x{vma:08x}, file 0x{:06x}, {len} bytes.",
            0x60988usize + rel_off,
        )
        .unwrap();
        writeln!(f, "pub const ESC_EXT_SLICE_{i}: &[u8] = &[").unwrap();
        emit_byte_array(&mut f, &bytes[rel_off..rel_off + len]);
        writeln!(f, "];\n").unwrap();
    }

    writeln!(f, "/// All 24 cluster slices in slice-index order.").unwrap();
    writeln!(
        f,
        "pub const ESC_EXT_SLICES: [&[u8]; ESC_EXT_SLICE_COUNT] = ["
    )
    .unwrap();
    for i in 0..slices.len() {
        writeln!(f, "    ESC_EXT_SLICE_{i},").unwrap();
    }
    writeln!(f, "];\n").unwrap();

    writeln!(
        f,
        "/// Per-slice (relative_offset, vma, length_bytes) metadata."
    )
    .unwrap();
    writeln!(
        f,
        "pub const ESC_EXT_SLICE_META: [(usize, u32, usize); ESC_EXT_SLICE_COUNT] = ["
    )
    .unwrap();
    for &(rel_off, vma, len) in &slices {
        writeln!(f, "    ({rel_off}, 0x{vma:08x}, {len}),").unwrap();
    }
    writeln!(f, "];\n").unwrap();

    // Per-descriptor slice indices.
    for &(name, idxs) in &g_slice_idx {
        writeln!(
            f,
            "/// Slice indices for {name}'s descriptor pointer block \
             (`+0x0c`, `+0x10`, `+0x14`, `+0x18`) per spec/08 §2.2."
        )
        .unwrap();
        writeln!(
            f,
            "pub const ESC_EXT_{name}_SLICE_INDICES: [usize; 4] = [{}, {}, {}, {}];\n",
            idxs[0], idxs[1], idxs[2], idxs[3],
        )
        .unwrap();
    }
}

/// Emit `g_counts.rs` carrying the spec/15 §3 authoritative per-G
/// `(count_A, count_B)` table as a runtime constant. The values are
/// lifted verbatim from `G_COUNTS_SPEC15` (see top of build.rs); each
/// pair was originally extracted from the binary's `mb_mv_struct_init`
/// constructor body at VMA `0x1c210643` per spec/15 §2.1.
///
/// Build-time self-check: the partition arithmetic
/// `sub_A = count_B + 1` / `sub_B = count_A - count_B - 1` must yield
/// the sizes spec/03 §4.4 documents. This catches the case where one
/// of the six (count_A, count_B) pairs is off-by-one without the
/// partition pivot in `emit_g_descriptor_cluster` / `emit_g_enum`
/// noticing.
///
/// FROM: `docs/video/msmpeg4/spec/15-count-ab-per-g-family.md` §3
/// (binary), §4.1 (enum CSVs), §5 (eight prior spec citations).
fn emit_g_counts_spec15(out_path: &Path) {
    // Spec/03 §4.4 / spec/15 §5.2 sub-A / sub-B sizes per G. These are
    // the reference values the partition arithmetic must reproduce
    // exactly. If a future round needs to add G6+ or revise the binary
    // constructor disassembly, update both `G_COUNTS_SPEC15` and this
    // reference table together (and re-run the build).
    let expected_sub_a_b: [(u32, u32); 6] = [
        (99, 69),  // G0
        (119, 66), // G1
        (81, 67),  // G2
        (85, 47),  // G3
        (58, 44),  // G4
        (67, 35),  // G5
    ];
    for (g, &(want_a, want_b)) in expected_sub_a_b.iter().enumerate() {
        let (got_a, got_b) = g_subclass_sizes(g);
        if got_a != want_a || got_b != want_b {
            panic!(
                "G{g} sub-class sizes ({got_a}, {got_b}) mismatch vs spec/03 §4.4 / \
                 spec/15 §5.2 ({want_a}, {want_b}) — partition arithmetic on \
                 G_COUNTS_SPEC15[{g}] is wrong; check whether (count_A, count_B) = \
                 {:?} is consistent with the constructor disassembly per spec/15 §2.1",
                G_COUNTS_SPEC15[g],
            );
        }
    }

    let mut f = fs::File::create(out_path)
        .unwrap_or_else(|e| panic!("failed to create {}: {e}", out_path.display()));
    writeln!(
        f,
        "// Auto-generated by build.rs from the in-source `G_COUNTS_SPEC15` table.\n\
         // DO NOT EDIT.\n\
         // Source: docs/video/msmpeg4/spec/15-count-ab-per-g-family.md §3 /\n\
         //         constructor disassembly `mb_mv_struct_init` at VMA 0x1c210643\n\
         //         in mpg4c32.dll (SHA-256\n\
         //         aedb4cf3d33c8554ab8acf04afe2d936eaa7c49107c5fefe163bca2e94b3c099).\n\
         \n\
         /// Per-G-family `(count_A, count_B)` for the six MS-MPEG4 DCT-AC\n\
         /// G-descriptors (G0..G5). Indexed by G-family number. `count_A`\n\
         /// is the alphabet size (and ESC sentinel index). `count_B` is\n\
         /// the sub-A / sub-B partition boundary: `idx <= count_B` is\n\
         /// sub-A (`last = 0`), `count_B < idx < count_A` is sub-B\n\
         /// (`last = 1`), `idx == count_A` is ESC.\n\
         ///\n\
         /// Per spec/15 §7 (\"All six G-families CONSISTENT\") these values\n\
         /// have been cross-checked against eight independent prior spec\n\
         /// documents (02, 03, 04, 05, 08, 09, 13, 14) and the\n\
         /// per-G enum CSV row counts; every citation agrees.\n\
         pub const G_COUNTS_SPEC15: [(u32, u32); 6] = ["
    )
    .unwrap();
    for (g, &(ca, cb)) in G_COUNTS_SPEC15.iter().enumerate() {
        writeln!(f, "    ({ca}, {cb}), // G{g}").unwrap();
    }
    writeln!(f, "];\n").unwrap();

    writeln!(
        f,
        "/// Per-G-family `(sub_A_size, sub_B_size)` derived from\n\
         /// `G_COUNTS_SPEC15` via the partition arithmetic in\n\
         /// spec/03 §4.4 / spec/15 §5.2: `sub_A = count_B + 1`,\n\
         /// `sub_B = count_A - count_B - 1`. Cross-checked at build\n\
         /// time against the explicit reference table emit_g_counts_spec15\n\
         /// carries (so a future drift in either source surfaces as a\n\
         /// build break, not a runtime mismatch).\n\
         pub const G_SUBCLASS_SIZES_SPEC15: [(u32, u32); 6] = ["
    )
    .unwrap();
    for (g, _) in G_COUNTS_SPEC15.iter().enumerate() {
        let (sa, sb) = g_subclass_sizes(g);
        writeln!(f, "    ({sa}, {sb}), // G{g}").unwrap();
    }
    writeln!(f, "];\n").unwrap();
}
