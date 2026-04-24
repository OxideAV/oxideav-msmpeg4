# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
