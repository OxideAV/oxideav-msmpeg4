# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Round 75 — v1/v2 shared CBPY VLC binary cross-check** (2026-05-18):
  the binary's 6-bit pre-expanded CBPY pre-expansion LUT at VMA
  `0x1c254240` (file offset `0x53640`, region `region_053640.hex`,
  128 bytes for the 64-entry 6-bit window) is now parsed at build time
  by `build.rs::emit_cbpy_v1_v2` and verified to match the H.263
  Table 8 / MPEG-4 Part 2 Table B-6 hand-derived `CBPY_INTRA_TABLE`
  byte-for-byte. The binary additionally fills the two unused 6-bit
  prefix slots (`000000` and `000001`) with reserved-sentinel symbols
  (0x10 and 0x11); these are emitted as `CBPY_V1_V2_SENTINELS` and
  flagged in tests, with the runtime decoder's existing `raw > 15`
  range check rejecting them as malformed bitstream per spec/07 §1.3.
  The build breaks with a precise tuple-mismatch error if the binary
  ever drifts from the public-standard CBPY codes; the same invariants
  are pinned as 7 new `tables_data::tests` (alphabet size, max
  bit-length, exact match vs `CBPY_INTRA_TABLE`, Kraft sum accounting
  for sentinels, sentinel placement, prefix-freedom, sym=15 short-code
  load-bearing value) plus 2 new `tests/v1_v2_mcbpcy.rs` integration
  tests (every-code round-trip via the new public
  `mcbpcy::decode_cbpy_no_wrap` entry point, reserved-sentinel
  rejection). Test count delta: 229 lib + 7 v1_v2_mcbpcy = +9 tests.
  This closes the v1/v2 CBPY VLC provenance gap: the H.263-derived
  table and the binary's pre-expanded LUT are now lock-stepped through
  a build-time invariant. The runtime decoder continues to consume
  `CBPY_INTRA_TABLE` (no behavioural change); the new
  `CBPY_V1_V2_RAW` / `CBPY_V1_V2_SENTINELS` constants serve as the
  binary-traceable cross-check oracle.

- **Round 7 — G0..G3 LMAX/RMAX + synthetic-VLC pipeline wired**
  (2026-05-17): the four extended-alphabet DCT AC TCOEF descriptors
  (G0/G1/G2/G3) now carry `Some(lmax)` / `Some(rmax)` extension tables
  derived from the round-29 enumeration data
  (`tables/region_*_g{0..3}_enum.csv`), per
  `docs/video/msmpeg4/spec/09-g0-g3-enumeration.md` §1 + §8. The 3-tier
  ESC body in `decode_escape_body` can now chain through tier-1
  (level extension) and tier-2 (run extension) for the G0..G3 paths
  when the primary canonical-Huffman bit-length array lands.
  Cross-checked at runtime against every per-(last, run) cap in
  spec/09 §8's consolidated summary table — G0 sub-A r0 LMAX=23,
  G1 sub-A r1 LMAX=15, G2 sub-B level-1 tail r=43, G3 sub-A caps at
  r=20. The `entries` slice on `AcVlcTable::v3_intra_g{0..3}()`
  remains empty (primary VLC bit-length extraction still spec-OPEN
  per `docs/video/msmpeg4/spec/99-current-understanding.md` §10 —
  candidate sources at `0x57a30 / 0x57f80 / 0x58558 / 0x58a08`
  flagged `verdict: suspect`).
- **`AcVlcTable::v3_intra_g{0..3}_synthetic`** — new test-only
  constructors that build a fixed-length canonical Huffman over the
  G0..G3 enumeration alphabet (every idx is its own bit-pattern at
  `ceil(log2(count_A + 1))` bits). NOT bit-exact against the binary
  but lets the post-VLC pipeline + 3-tier ESC body run end-to-end
  against a known-good prefix code for regression tests. The
  synthetic VLC bit-width is 8 bits for G0/G2/G3 (count_A 132–168)
  and 8 bits for G1 (count_A 185). ESC lives at `code == count_A`
  per spec/09 §2.
- **`tests/g0_g3_extended.rs`** — 16 new integration tests covering
  (1) LMAX/RMAX wiring assertions per spec/09 §8, (2) synthetic-VLC
  round-trip of every non-ESC symbol for all four G-alphabets
  (633 token round-trips total across G0..G3), (3) 3-tier ESC body
  walking (tier-1 level extension on G0, tier-2 run extension on G1,
  tier-3 verbatim FLC on G2, sub-A/sub-B boundary on G3), (4)
  alphabet shape cross-checks vs spec/15 §7.
- **Renamed test** `g0_g3_placeholder_constructors_return_empty_entries`
  → `g0_g3_placeholder_constructors_have_lmax_rmax_round_7` to reflect
  the round-7 invariant change. New companion test
  `g0_g3_lmax_rmax_load_bearing_values` pins four load-bearing
  values from spec/09 §8 against the derived tables.

### Notes

- **PSNR baseline unchanged** at 9.76 dB Y on `testsrc2_32x32_ffmpeg_parity`
  and 10.93 dB Y on the 176×144 real-fixture diagnostic. Round 7
  wires the LMAX/RMAX scaffolding and the synthetic-VLC test path;
  real-content decode of streams that select G0..G3 still bails to
  DC-only reconstruction until the primary canonical-Huffman
  bit-length array lands. The unblocking is then `entries: g{0..3}_primary_entries()`
  swapped into the existing `v3_intra_g{0..3}()` constructors — the
  LMAX/RMAX are already in place.
- **Alt-MV (mv_table_sel=1) remains unsupported**: the docs
  collaborator's round-34..37 docs landed G0..G3 enumeration but did
  NOT re-extract the truncated 256-byte `region_0594b8.hex` dump.
  Per spec/06 §2.1 the alt-MV VLC source at VMA `0x1c25a0b8` is the
  full ~8 KB 1099-entry table — still missing. Dispatch wiring
  through `decode_mv_with_table` (round 32) is already complete; the
  swap-in is local to `mv.rs::decode_mv_with_table` once the
  extraction lands.

## [0.0.6](https://github.com/OxideAV/oxideav-msmpeg4/compare/v0.0.5...v0.0.6) - 2026-05-07

### Other

- round 33: wire ESC extension table cluster (region_060988)
- drop dead `linkme` dep
- registry calls: rename make_decoder/make_encoder → first_decoder/first_encoder
- auto-register via oxideav_core::register! macro (linkme distributed slice)
- unify entry point on register(&mut RuntimeContext) ([#502](https://github.com/OxideAV/oxideav-msmpeg4/pull/502))

### Added

- **Round 33 — ESC extension table cluster wired** (2026-05-08):
  clean-room ingest of `region_060988.hex` (2168 bytes at file
  `0x60988..0x61200`, VMA `0x1c261588..0x1c261e00`) plus
  `region_060988_index.csv` (24 slice-boundary rows). Per
  `docs/video/msmpeg4/spec/08-descriptor-constants.md` §1-§2 +
  `spec/14-pri-ab-runtime-binding.md` §2.1, the cluster is the
  back-store for every G-descriptor's `+0x0c..+0x18` pointer block
  (sub-A / sub-B level- and run-extension arrays the v2/v3 inter
  kernel `0x1c215e6f` and v1/v2/v3 intra kernel `0x1c216d97` read on
  first- and second-tier ESC paths). The new `build.rs::emit_esc_ext_cluster`
  step parses both files and emits `tables_data::ESC_EXT_SLICES:
  [&[u8]; 24]`, per-slice metadata `ESC_EXT_SLICE_META`, and six
  per-G-descriptor index quadruples `ESC_EXT_G{0..5}_SLICE_INDICES`
  matching the four-pointer spec/08 §2.2 attribution. Build-time
  invariants verify cluster size (= 2168), slice contiguity, every
  slice length is a multiple of the 8-byte record stride, and each
  G-descriptor's four VMAs resolve to slice-array indices.
  **Slice-content semantics remain spec-OPEN per `spec/08` §4.1**
  — the kernel reads `[base + idx*4]` (BYTE for `+0x0c/+0x10`,
  DWORD for `+0x14/+0x18`) but the slices' on-disk record format is
  `(symbol_u32_le, bit_length_u32_le)` 8-byte pairs, and the
  relationship between the two is not yet pinned down by docs.
  Round 33 wires the bytes and per-descriptor attribution; the
  ESC-body decoder using these tables awaits a future Specifier
  round on the inter / intra kernel ESC bodies.
- **8 new `tables_data::tests`** for the ESC-ext cluster: cluster
  size, slice-lengths-sum-to-total, first/last slice VMAs from the
  CSV index, per-G-descriptor index attribution (spec/08 §2.2),
  per-G VMA round-trip, the 24 verbatim slice lengths in CSV order,
  the multiple-of-8 invariant, and a first-record byte spot-check
  for slice 0 (G1 sub-A level-extension).

## [0.0.5](https://github.com/OxideAV/oxideav-msmpeg4/compare/v0.0.4...v0.0.5) - 2026-05-03

### Other

- thread mv_table_sel into MV decode (round 32 piece 3 of 3)
- add G0..G3 named AC table constructors (round 32 piece 2 of 3)
- wire AcSelection::FromHeader (round 32 piece 1 of 3)

## [0.0.4](https://github.com/OxideAV/oxideav-msmpeg4/compare/v0.0.3...v0.0.4) - 2026-05-03

### Other

- rename G5-test locals to snake_case
- cargo fmt rustfmt 1.95 chain/closure wraps

### Fixed

- **Round 28 — DC predictor + AC-scan dispatch corrected per spec/03 §1**
  (2026-05-01, task #125): the gradient test in `dc_pred::predict_dc`
  compared `|A - D|` vs `|A - B|` (left-vs-top distance), but the
  MPEG-4 §7.4.3 / spec/03 §1.3 rule compares `|A - D|` (left-column
  gradient) vs `|D - B|` (top-row gradient). The smaller-gradient axis
  wins. Likewise, `PredDir::ac_scan` mapped `FromLeft → AlternateHorizontal`
  and `FromTop → AlternateVertical`, but the binary's dispatch helper
  at `1c20de2e` (per spec/03 §1.1) returns:
    * `[mb+0x2c] = 1` (vertical pred wins, predict from TOP) → alt-horizontal
      scan (VMA `0x1c261140`)
    * `[mb+0x2c] = 0` (horizontal pred wins, predict from LEFT) → alt-vertical
      scan (VMA `0x1c261240`)
  Both fixes lift testsrc 176×144 DC-only PSNR from 10.69 dB Y →
  10.93 dB Y; testsrc2 32×32 (G5 path) from 9.73 dB Y → 9.76 dB Y.
  The AC walker still trips on a non-terminating sub-A token chain in
  the failing 176×144 fixture (`scan position 70 exceeds block`); the
  encoder appears to emit valid sub-A `(run=14, level=1)` at scan pos
  56 — likely a deeper issue with table interpretation at the sub-A/
  sub-B boundary that needs further binary disassembly.

### Added

- **Round 27 — v3 intra 3-tier ESC body wired** (2026-05-01):
  unblocks the `decode_escape_body` path past the verbatim-only
  fallback per `docs/video/msmpeg4/spec/04-decoder-kernels.md`
  §2.3 (intra kernel `0x1c216d97` 3-tier escape: level-extension at
  `0x1c216e7b`, run-extension at `0x1c216f02`, verbatim at
  `0x1c216f5f`). The chained-ESC walker runs:
  tier 1 (re-decode primary VLC, apply `level_actual = base + LMAX[last][run]`)
  → tier 2 if ESC re-fires (re-decode primary VLC, apply
  `run_actual = base + RMAX[last][|level|] + 1`) → tier 3 if ESC
  re-fires again (verbatim 1+6+8 FLC triple). LMAX / RMAX are
  derived from the G5 descriptor's pri_A/pri_B alphabet at first
  use (lazy `OnceLock`); `AcVlcTable` carries them as
  `Option<&'static LevelLimitTable>` /
  `Option<&'static RunLimitTable>` so the inter G4 path skips the
  walk and goes straight to verbatim (matching spec/04 §1.3 step 10's
  reduced 1-tier inter ESC). `picture::decode_iframe` /
  `decode_pframe` now also route luma blocks through G5 and chroma
  blocks through G4 per spec/99 §5.2 (slot `[esi+0xab4]` for luma,
  `[esi+0xab0]` for chroma + all-inter), unblocking per-block VLC
  alphabet alignment for real-content decode.
  `tests/ffmpeg_roundtrip::testsrc2_32x32_ffmpeg_parity` now decodes
  end-to-end (where round-26 errored) at Y PSNR 6.4 dB Y vs the
  spec/11 §9 placeholder baseline of 5.30 dB. The 176×144 testsrc
  fixture still errors before reaching ESC due to a primary-VLC /
  per-MB scan-order issue out of scope for the ESC body work.
- **Round 26 — G5 / G4 primary canonical-Huffman VLC wired**
  (2026-05-01): unblocked by
  `docs/video/msmpeg4/spec/11-walker-format-resolved.md` (the
  Specifier session that disassembled the per-slot loader helper at
  `0x1c218cfa`). The G5 (intra-luma) primary VLC at file offset
  `0x59178` / VMA `0x1c259d78` and the G4 (chroma + all-inter)
  primary VLC at file `0x58e38` / VMA `0x1c259a38` are now extracted
  into `crates/oxideav-msmpeg4/tables/region_05917 8_full.hex` and
  `region_058e38_full.hex` (828 bytes each = `4 + 103 * 8`) and wired
  through `build.rs::emit_packed_huffman_primary` into
  `G5_PRIMARY_RAW` / `G4_PRIMARY_RAW`. Helper-A's `(a:u32, b:u32)`
  records are interpreted as `(canonical_bit_pattern, bit_length)`
  for the G-tables — verified by direct prefix-freedom + Kraft sum
  exactly `1 - 1/512 = 0.998047` on both tables (one bl=9 codeword
  reserved for ESC at idx 102). The new `AcVlcTable::v3_intra_g5`
  constructor builds a 103-entry `Symbol::RunLevel` /
  `Symbol::Escape` table that the existing `decode_intra_ac` walker
  drives end-to-end; `picture::AcSelection::G5` is now the shipping
  default for v3 intra blocks. `tests/g5_primary.rs` covers the new
  table with 7 dedicated tests (shortest code, 3-bit code,
  every-non-ESC round-trip, ESC body, prefix-freedom, alphabet
  partition, length sanity).
- **Round 19 — G5 pri_B gap fill (full G5 alphabet wired)**
  (2026-04-30): clean-room extraction of the 408-byte gap that
  immediately follows `region_0569c0` (file `0x57898..0x57a30`,
  VMA `0x1c258498`) lands in
  `crates/oxideav-msmpeg4/tables/region_057898.hex` (102 × u32-LE
  records). `build.rs::emit_g_descriptor_cluster` now consumes
  both `region_0569c0.hex` and `region_057898.hex` and emits a
  complete `G5_PRI_B: &[u8; 102]` (low byte of each u32-LE record
  per `audit/01 §4.4`). Build-time guards verify:
    - run-zero prefix `[0; 27]` for sub-class A's run=0 row
      (LMAX(intra, run=0) = 27 per `audit/01 §4.2`),
    - sub-A boundary at idx=count_B=66 carries run=14 (last sub-A
      entry per `audit/01 §4.1`),
    - sub-B restart at idx=67 carries run=0,
    - sub-B last entry at idx=101 carries run=20,
    - per-row LMAX cross-check across all 35 sub-B entries
      (r0×8, r1×3, r2..6×2, r7..20×1) against the audit's
      enumeration.
  `g_descriptor::g5_decode` now handles the full alphabet (102
  symbols + ESC) — sub-class B used to return `None` because pri_B
  wasn't wired. The legacy `g5_iter_partial` remains as an alias
  for `g5_iter` for source-compatibility. The hand-derived
  `g5_pri_b_sub_a_derived` helper is removed; runs come straight
  from the byte array now.
- **Test coverage delta**: +21 tests (current total 218).
    - `src/g_descriptor.rs::tests`: +7 tests including
      `g5_sub_b_first_entry_is_run0_level1_last`,
      `g5_sub_b_last_entry_is_run20_level1_last`,
      `g5_sub_b_per_run_lmax_matches_audit`,
      `g5_sub_b_per_run_count_matches_audit`,
      `g5_alphabet_size_is_102_plus_esc`,
      `g5_sub_a_partition_strict`,
      `g5_idx_out_of_range_is_none`.
    - `src/tables_data.rs::tests`: +7 tests including
      `g5_pri_b_run_zero_prefix`,
      `g5_partition_matches_audit_table`,
      `g5_pri_b_max_run_is_20`,
      `g5_pri_b_sub_a_run_progression`,
      `g5_pri_a_sub_b_canonical_level_prefix`,
      `g5_pri_b_sub_b_run_progression`,
      `g5_pri_b_no_sentinel`.
    - `tests/g_descriptor_g5.rs`: NEW 7-test integration suite
      mirroring `g_descriptor_g4.rs` (alphabet round-trip, audit
      §4.1 row-by-row sub-A and sub-B cross-checks, decoder/byte-array
      consistency, no-zero / no-sentinel pri_A invariant, low-byte
      pri_B invariant).
- **PSNR baseline unchanged** at 5.30 dB Y on `testsrc2 32×32`.
  The G5 pri_B wiring closes the data half of G5's descriptor;
  runnable AC decode still awaits the canonical-Huffman bit-length
  walker tree at file `0x3df40` (`spec/99 §5.3`) which maps
  bitstream bits to the alphabet idx.

- **Round 18 — G4 / G5 DCT-descriptor pri_A / pri_B wiring**
  (2026-04-26): clean-room extraction of `region_0569c0.hex`
  (file `0x569c0..0x57898`, 3800 bytes — the G-descriptor
  cluster per `spec/99 §10.3`) is copied verbatim into
  `crates/oxideav-msmpeg4/tables/region_0569c0.hex` and
  `build.rs::emit_g_descriptor_cluster` slices out four byte
  arrays:
    - `G4_PRI_A` (102 bytes, file `0x57630..0x57696`) — `|level|`
      per symbol for the inter DCT TCOEF alphabet.
    - `G4_PRI_B` (102 bytes, low byte of u32-LE record at
      file `0x57698..0x57830`) — `run` per symbol.
    - `G5_PRI_A` (102 bytes, file `0x57830..0x57896`) —
      `|level|` per symbol for intra-luma DCT TCOEF.
    - `G5_PRI_B` is **not yet captured**: it lives in a 408-byte
      gap between `region_0569c0`'s end (`0x57898`) and
      `region_057a30`'s start, not in any `tables/*` file.
  All slice offsets are derived from `spec/99 §5` VMAs minus the
  region base — no numeric values are typed by the Implementer.
  Build-time guards verify the canonical level prefix
  (`01 02 03 ...` per `spec/99 §5.1`), the run-zero prefix in
  pri_B (12 zero u32s for G4 sub-A start), and the partition
  cross-check against `audit/01 §2.2` (`pri_B[count_B] == 26` for
  G4's sub-A last run, `pri_B[count_B+1] == 0` for sub-B restart).
- **`g_descriptor` module** — public `(idx → (last, run, level_mag))`
  decoder for G4 (full alphabet + ESC) and G5 (sub-class A + ESC,
  sub-class B returns `None` until G5 pri_B is captured). Wraps
  `spec/04 §1.3 step 3`'s partition test and `spec/99 §4.2` step 2.
- **18 new `g_descriptor::tests`** + **7 new
  `tests/g_descriptor_g4.rs` integration tests** + **11 new
  `tables_data::tests` invariants**: total +36 tests vs r17, all
  passing. Coverage includes:
    - LMAX-per-run cross-check against `audit/01 §3.3` (G4 inter
      ESCL(b) profile) and `§4.2` (G5 intra ESCL(a) profile).
    - sub-A / sub-B partition strict-monotone check across all 102
      G4 indices.
    - byte-level round-trip: pri_A/pri_B values match decoder output
      for every idx in [0, count_A).
    - sentinel-byte guards (no `0` or `0xff` in pri_A per
      `audit/01 §2.3`).
- **Spec-OPEN documentation refreshed**: README status table now
  carries explicit rows for G4/G5 wiring state and the
  bit-length / G5-pri_B gaps. The "What's still spec-OPEN"
  section names the walker tree at file `0x3df40` (per
  `spec/99 §5.3`) as the remaining blocker for runnable AC decode.

  PSNR baseline on `testsrc2 32×32` is unchanged at 5.30 dB Y —
  this round wires the data structures and validates them against
  the audit's per-row enumeration; runnable bit-stream AC decode
  awaits the next round's bit-length resolution.

- **Intra-AC primary VLC candidate wired** (round 10): clean-room
  extraction of `region_05eed0.csv` (VMA `0x1c25fad0`, file offset
  `0x5eed0`) lands in the build pipeline. The 64-entry canonical-
  Huffman code-length array is verified at build time (Kraft sum
  = 1) and exposed via `AcVlcTable::v3_intra_candidate`. The
  `(last, run, level)` symbol mapping is the Implementer's
  hypothesis (partition test from `spec/04` §1.3 step 3,
  `|level|=1` baseline with ESC body for larger levels) — the
  underlying region's role is OPEN per `spec/99` §0.1 row 8 and
  §9 OPEN-O6. A future spec/audit pass may revise the mapping.
- **`AcSelection` enum + `decode_picture_with_ac`**: callers
  opt into the candidate AC table explicitly via the new
  `picture::decode_picture_with_ac(br, dims, ref, AcSelection::Candidate)`
  entry point. The `Decoder` trait still defaults to
  `AcSelection::Placeholder` (DC-only reconstruction on coded
  blocks) so existing consumers see no behaviour change.
- **Test coverage**: 8 new unit tests (kraft-sum, prefix-free,
  per-symbol round-trip, partition-rule sanity) plus 6 new
  integration tests (`tests/intra_ac_candidate.rs`) exercising
  the candidate VLC end-to-end against synthetic streams +
  ffmpeg-encoded DIV3 first-chunk smoke.
- **`build.rs`**: new `emit_intra_ac_v3` step parses
  `tables/region_05eed0.csv` and emits
  `INTRA_AC_V3_CANDIDATE_RAW` / `_ALPHABET` / `_PARTITION` into
  `OUT_DIR/intra_ac_v3.rs`. Build-time Kraft check enforces
  `sum(2^-bl) == 1` over the 64 payload bit-lengths and fails
  the build if the CSV ever drifts.

## [0.0.3](https://github.com/OxideAV/oxideav-msmpeg4/compare/v0.0.2...v0.0.3) - 2026-04-24

### Other

- v3 P-frame skeleton — MV VLC + MC copy + reference threading
- DC spatial predictor + AC-scan dispatcher + MCBPCY wire
- v3 I-frame first light — end-to-end Frame::Video output
- wire 6-block intra MB decode + annotate OPEN intra AC VLC
- add intra AC coefficient decoder + ffmpeg integration test
- wire first-MB decode path + update README status
- add intra macroblock header + DC differential decode
- add CBPY + DC-size VLC tables
- add scan tables, H.263-style dequant, and 8x8 IDCT
- add picture module + wire decoder to v3 parser
- add linear-scan VLC decoder infrastructure
- add v3 picture-header parser module

### Added

- **v3 P-frame decoder skeleton** (round 9): `decode_pframe` wires
  the full P-frame pipeline around the intra path. Per-MB skip-bit
  read + joint MCBPCY (shared with I-frames) + post-VLC `ac_pred`
  flag + (if intra) reuse the intra pipeline or (if inter) decode
  the joint (MVDx, MVDy) MV VLC and copy a 16×16 luma + two 8×8
  chroma blocks from the previous reference picture. The decoder
  now retains the last decoded picture (cleared on `flush()`) and
  threads it as the MC reference automatically.
- **MV VLC + MVDx/MVDy byte LUTs** (`mv.rs`): the default v3 joint
  (X, Y) MV VLC source at VMA `0x1c25cbc0` (1100 entries + ESC
  index 1099) and the `0x1c25ee28` / `0x1c25f278` byte LUTs are
  extracted into `tables/region_05bfc0.csv` +
  `region_05e228.hex` / `region_05e678.hex` and compiled via
  `build.rs` into `MV_V3_RAW` / `MVDX_V3_BYTES` / `MVDY_V3_BYTES`.
  Canonical Huffman builder (shared with MCBPCY), median-of-3
  neighbour predictor, ESC tail (6 bits MVDx + 6 bits MVDy), and
  toroidal `[-63, +63]` wrap all wired per spec/06 §§3.1–3.5.
- **Motion compensation** (`mc.rs`): integer + half-pel bilinear
  luma/chroma MC with edge-clamp OOB handling. `mc_macroblock`
  handles the 16×16 luma + 2× 8×8 chroma copy in one call, using
  the MPEG-4 §7.6.3.4 chroma-MV derivation `chroma = luma >> 1` for
  the 1-MV-per-MB case.
- **P-frame MCBPCY variant** (`decode_mcbpcy_pframe`): reads the
  1-bit skip prefix (spec/05 §3.2), then on non-skip decodes the
  joint VLC + post-VLC `ac_pred` bit. Returns `PFrameMcbpcy::Skip`
  on the skip branch so the caller can MC-copy from the reference
  directly.
- Alternate MV table (`mv_table_sel == 1`, VMA `0x1c25a0b8`) is
  rejected with a documented `Unsupported` error — the available
  extraction dump is truncated to 256 of the 8800 bytes needed for
  the 1100-entry alphabet. The default table covers the most
  common content.
- v3 I-frame decoder produces a `Frame::Video` end-to-end: picture
  header → MB loop → 6 blocks per MB → DC-path reconstruction +
  IDCT → YUV420P pel planes. Clean-room first-light milestone.
- `build.rs` + `tables_data` module: parse the vendored
  `region_05eac8.csv` dump (clean-room extraction of the v3
  joint-MCBPCY VLC source) into compile-time constants via
  `include!(concat!(env!("OUT_DIR"), …))`. Implementer logic only;
  no manual retyping of VLC numerics.
- `MsV3PictureHeader` now parses the three v3 per-frame selectors
  (`[esi+0xad0]`, `[esi+0xad4]`, `[esi+0x8bc]`) with the spec-§2.3
  unary-capped-at-2 encoding for the tri-valued fields.
- **DC spatial predictor** (`dc_pred.rs`): MPEG-4 §7.4.3 gradient
  test `|A - D| < |A - B|` → left vs top, with `DcCache`
  tracking per-block reconstructed DC values across the raster
  scan. Picture-edge neighbours substituted with the neutral value
  1024. `PredDir::ac_scan()` maps the chosen direction to the AC
  scan per spec/04 §4.4.
- **AC-scan dispatcher** wired: `decode_iframe` now selects
  `Scan::Zigzag` when `ac_pred_flag=0`, `Scan::AlternateHorizontal`
  when the DC predictor picked the left neighbour (row scan), and
  `Scan::AlternateVertical` when it picked the top neighbour
  (column scan).
- **MCBPCY joint-VLC wired** (`mcbpcy.rs`): the 128-entry
  canonical-Huffman table compiled from `region_05eac8.csv` in
  round 7 is now consumed by `IntraMbHeader::parse_v3_mcbpcy`.
  Decodes the joint MB-type + 6-bit CBP pattern (4 luma + 2
  chroma) per spec/05 §3.2 in one VLC call, then reads the
  post-VLC `ac_pred_flag` bit. `cbp_cb` / `cbp_cr` now come from
  the MCBPCY symbol rather than being hard-coded to false.
- Hand-crafted DC-only 32×32 and DC-propagation 16×16 unit tests
  in `picture::tests` exercise the end-to-end MCBPCY → DC predict
  → scan dispatch → IDCT pipeline without relying on ffmpeg.
- `testsrc2_32x32_ffmpeg_parity` integration test: mints a
  testsrc2 DIV3 AVI via ffmpeg, decodes with both our crate and
  ffmpeg, computes per-plane PSNR, and logs the delta. Currently
  Y PSNR ~5 dB (AC VLC still placeholder — coded AC bits
  misalign downstream reads). Target PSNR >25 dB awaits the
  Extractor landing real intra-AC run/level/last table.

### Changed

- Dequantisation follows `spec/08` / `spec/07` §4: the H.263
  §6.2.2.1 formula `coeff = level * (2*PQUANT) ± (PQUANT − parity)`
  uniformly for v1/v2/v3. Replaces the prior "odd/even half-step"
  encoding.
- `write_block_to_picture` no longer double-offsets intra pels —
  the intra DC already carries the pel mean, so the post-IDCT
  `+128` is removed. Fixes the 32×32 DC-only test's `Y=255`
  saturation regression.
- `IntraMbHeader` now includes `cbp_cb` / `cbp_cr` fields
  populated by the joint-MCBPCY parse; the legacy `parse` method
  still reads H.263 CBPY for v1/v2 paths and sets both chroma
  bits to false.

## [0.0.2](https://github.com/OxideAV/oxideav-msmpeg4/compare/v0.0.1...v0.0.2) - 2026-04-19

### Other

- drop Cargo.lock — this crate is a library
- migrate register() to CodecInfo builder
- bump oxideav-core + oxideav-codec deps to "0.1"
