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

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let tables_dir = manifest_dir.join("tables");
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

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

fn emit_byte_array<W: Write>(out: &mut W, bytes: &[u8]) {
    for chunk in bytes.chunks(16) {
        write!(out, "   ").unwrap();
        for b in chunk {
            write!(out, " 0x{:02x},", b).unwrap();
        }
        writeln!(out).unwrap();
    }
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
