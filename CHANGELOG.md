# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.9](https://github.com/OxideAV/oxideav-msmpeg4/compare/v0.0.8...v0.0.9) - 2026-07-03

### Other

- round 386 CHANGELOG + README - encoder quality drive documented
- v3 I-frame table-selector RD - 18-way exact-rate header choice
- bit-budget rate control - per-frame QP adaptation (all versions)
- scene-cut GOP policy - census-driven P-to-I upgrade
- fix doc_lazy_continuation lint on encode_intra_mb doc comment
- intra-in-P scene-change macroblocks + PFrameStats census (v1/v2/v3)
- RD-decided ac_pred - per-MB alternate-scan rate probe (v3 + v2)
- rate-aware predictor-centred motion search (all versions)
- rate_curve example - decoder-verified PSNR/rate measurement harness
- correct the round-383 wire-code caveat - CBPY is binary-extracted
- v1 INTER4V (4-MV) encoding - greedy per-block search through the Figure-7-34 driver
- round 383 CHANGELOG + README - full-family encoder documented
- register v1/v2 Encoder factories - full-family frame-to-packet encode
- v1/v2 I-frame + P-frame picture encoders through decode_picture_v1v2
- v1/v2 entropy-encode primitives (header / MCBPC+CBPY / size+value DC / per-component MV)
- registered v3 Encoder (GOP I/P machine) + encoder::make_encoder factory
- v3 P-frame encoder - skip / motion-searched inter MBs through decode_picture
- v3 I-frame picture encoder end-to-end through decode_picture
- v3 entropy-encode primitives - bit-level inverses of every v3 decode surface
- encoder-side forward DCT + H.263 quantiser (inverse of idct/dequant)
- correct round 374 CHANGELOG test counts (7 tests; v1_v2_pframe 18->21)
- deterministic registered-decoder v3 I-frame to intra-in-P P-frame sequence
- CHANGELOG + README for round 374 intra-in-P picture-level coverage
- pin v3 intra-in-P MB contributes Absent (not a MV) to the predictor grid
- pin the v1 P-frame intra-in-P MB path (no ac_pred)
- pin v2 intra-in-P ac_pred scan-flip on a CBP-coded block
- pin v3 intra-in-P ac_pred scan-flip on a CBP-coded block
- picture-level pin for the v3 intra-in-P P-frame MB path
- msmpeg4 README: round 366 rollup — INTER4V coverage + v3 gap #1895 firmed
- picture-level INTER4V -> 1-MV-neighbour MV propagation pin
- v3 McbpcyDecode::num_motion_vectors accessor pins 1-MV invariant
- deepen v1/v2 INTER4V picture coverage + correct stale mc.rs doc
- correct v3 joint-MCBPCY intra/inter partition polarity (round 362)
- CHANGELOG + README for round 362 alt-MV byte-LUT pin
- pin alternate joint-MV VLC byte-LUT selection end-to-end (round 362)
- pin v3 P-frame median-predictor propagation end-to-end (round 359)
- README — record round 356 MV wire-code correction + MCBPCY docs gap
- full-alphabet round-trip pin for v3 joint-MV extracted codes
- decode v3 joint-MV VLC against extracted wire codes, not canonical
- rename ffmpeg_roundtrip.rs -> reference_roundtrip.rs (neutral filename; black-box validator invocation unchanged)
- README — record v1 MB-type table grounding + precise v3 4-MV docs gap
- scrub external-decoder naming from src/ doc comments
- ground v1 P-frame MB-type decode in extracted region_053140_mbtype (spec/16 §3)
- cross-check v1/v2 DC-size tables against spec/16 §2 + unit-test the size+value decode
- unblock v1/v2 intra pixel pipeline (I-frame + intra-in-P) via spec/16 §2 size+value DC rule
- round 335: wire v1 P-frame inter sub-types — INTER+Q + INTER4V (4-MV)
- wire v3 alternate joint-MV VLC (mv_table_sel==1) end-to-end
- round 317: narrow v1/v2 I-frame docs gap to intra-DC-size selector [esi+0x8bc] default
- refresh to current status, drop per-round changelog cruft

### Added

- round 386: **encoder quality drive — rate-aware motion search,
  ac_pred / table-selector RD, intra-in-P scene-change MBs, scene-cut
  GOP policy, bit-budget rate control.** Six milestones on top of the
  round-383 encoder family, each decoder-verified in-loop and measured
  with the new `examples/rate_curve.rs` harness (176×144, 30 frames,
  half-pel pan + hard mid-GOP scene cut; every packet decoded back
  through the production `decode_picture` chain):
  1. `examples/rate_curve.rs` — PSNR/rate measurement harness (also
     the source of every number below).
  2. Rate-aware predictor-centred motion search: cost =
     `SAD + quant·mv_bits` with the rate term computed by the real MV
     serialisers into a scratch writer, over the union of the zero-
     and predictor-centred half-pel windows, so the §7.6.5 predictor
     chain extends the effective search reach MB by MB. v1 INTER4V
     greedy search + mode decision share the same λ. Headline: with a
     deliberately tight ±2 window the chain still tracks the
     3-half-pel/frame pan — 220125 → 72792 B (−67 %) at +1.12 dB.
  3. RD-decided `ac_pred` (v3 + v2): each intra MB is serialised both
     ways (fixed zigzag vs the per-block DC-gradient alternate scans,
     spec/03 §1.1–§1.3) and the alternate variant kept only when
     strictly cheaper; v1 carries no ac_pred bit (spec/07 §1.4).
  4. Intra-in-P scene-change MBs (v1/v2/v3) + `PFrameStats` census:
     TMN-lineage activity-vs-SAD decision per P-frame MB; v3 emits
     the joint-MCBPCY I-type half behind the skip bit (intra-in-P
     luma binds G3 — the parser zero of the untransmitted P-wire
     `ac_luma_sel` — chroma the transmitted G4), v1 MB-type 3, v2 the
     intra quotient with its post-MCBPC ac_pred bit; DC cache/MV-grid
     state mirrors the decoder exactly (intra cells stay Absent).
     Scene-cut frame: min Y-PSNR 35.95 → 46.08 dB (+10.1 dB) at q=2
     while total bytes *drop* 270566 → 264324.
  5. Scene-cut GOP policy (`scene_cut` option, % of intra MBs,
     default 60, 0 = off): a P-frame whose census crosses the
     threshold is discarded and re-encoded as a keyframe, restarting
     the GOP at the cut (v3 q=8: 69262 → 67316 B, −2.8 %).
  6. Bit-budget rate control (`bitrate` option, bits/s): per-frame
     budget `bitrate/fps` with the classic I:P = 4:1 GOP split
     normalised to the GOP average, a virtual buffer draining 1/8 per
     frame into the adjusted target, ≤3 re-encode trials per frame
     (step ⌈qp/4⌉, 0.6..1.3 dead-band) + ±1 drift. Requested 400 /
     150 kbit/s → achieved 422 / 158 on the 30-frame sequence
     (startup + scene cut included); 64×64 steady state lands within
     25 % of budget.
  7. v3 I-frame table-selector RD: the frame is analysed once and
     serialised under all 18 transmittable (`ac_luma_sel`,
     `ac_chroma_sel`, `dc_size_sel`) combinations (spec/14 §3.1 +
     spec/99 §4.5), smallest bitstream wins (q=31 I-frames −1.4 %).
  Whole-curve result (v3, gop=15, quant 2/4/8/16/31): 271800 → 264010,
  152382 → 147648, 72078 → 67143, 32755 → 30913, 17122 → 16045 bytes
  (−2.9 % … −6.9 %) at equal-or-better PSNR everywhere, with the
  scene-cut frame +10 dB at fine quant.

- round 383: **MS-MPEG-4 family encoder (v1 / v2 / v3) — I-frames +
  P-frames, decoder-verified end-to-end, registered.** The crate was
  decode-only; this round adds the complete encode direction over the
  same extracted tables, driven to round-trip against this crate's own
  production decoders (`picture::decode_picture` /
  `decode_picture_v1v2`). Eight commits:
  1. Encoder-side transform + quantiser: `idct::fdct8x8` /
     `fdct8x8_from_pels` (exact float inverse of the shipping
     `idct8x8`) and `iq::quantise_h263` / `quantise_block_h263`
     (inverse of the spec/08 §5 reconstruction rule
     `sign·(|level|·2q + q − even_flag)` with a level-0 dead zone,
     clamped to ±127 for the verbatim ESC tier).
  2. v3 entropy-encode primitives — bit-level inverses of every v3
     decode surface: `header::MsV3PictureHeader::write` (I/P selector
     cluster in frame-type order), `mcbpcy::encode_mcbpcy` (+
     `compose_cbp`), `mb::encode_intra_dc_diff_v3` (direct-value DC
     VLC incl. bit-length holes → ESC and the inverted DC sign
     convention, spec/07 §5.2), `mv::encode_mv_with_table`
     (shortest-codeword joint-symbol lookup via a per-variant
     4096-slot inverse LUT over the MVDx/MVDy byte LUTs, ESC + 6+6
     FLC fallback, and the spec/06 §3.5 toroidal-window reachability
     rule `mv_component_reachable`), and `ac::encode_token` /
     `encode_intra_ac` / `encode_inter_ac` (primary → tier-1 level
     extension → tier-2 run extension → verbatim FLC in decoder order,
     3×ESC on 3-tier intra tables vs 1×ESC on the 1-tier inter kernel;
     both block walkers' run/position semantics inverted incl. the
     inter first-token rule). 16-test round-trip suite
     (`tests/encode_primitives.rs`) sweeps every primary alphabet
     entry of all six G-families both signs, all 128 MCBPCY symbols,
     DC diffs −255..=255, and exhaustive/dense MV windows on both VLC
     variants.
  3. v3 I-frame picture encoder (`enc::encode_iframe_v3`): forward
     DCT, §7.4.3 spatial DC prediction over the same
     reconstructed-integer-DC `DcCache` the decoder derives, joint
     MCBPCY (I-type half), ac_pred pinned 0 (fixed zigzag), G5-luma /
     G4-chroma AC walks. Flat-grey round-trips pel-exact; all-31-quant
     decodability sweep with per-quant loss bounds
     (`tests/v3_encoder.rs`).
  4. v3 P-frame encoder (`enc::encode_pframe_v3`): per-MB §7.6.5
     median predictor over a shared `MvGrid`, half-pel SAD motion
     search clipped to the toroidal window, MC through the same
     `mc_block` kernel + §7.6.3.4 chroma derivation as the decoder,
     6-block residual quantisation (`level_start = 0`), 1-bit skip vs
     coded-MB syntax (MCBPCY idx = 64 + cbp, always-consumed ac_pred
     bit, joint-MV VLC, G4 inter walks). I-P-P sequences chain through
     the decoder's own reconstruction.
  5. Registered `oxideav_core::Encoder` (I/P GOP machine; options
     `quant` / `gop` / `mv_search_range`) + the direct factory
     `encoder::make_encoder`; emits pts/dts + keyframe-flagged packets
     and decodes its own bytes each frame so encoder/decoder
     prediction state cannot drift. Registry loop test drives
     `first_encoder` → registered-decoder `send_packet` /
     `receive_frame`.
  6. v1/v2 entropy-encode primitives: `MsV1V2PictureHeader::write_v1`
     (37-bit zeroed opaque preamble per spec/99 §2.1 + P-only UMV
     flag) / `write_v2`, `mcbpcy::encode_mcbpcy_v1` (COD bit +
     21-symbol MCBPC, STUFFING/ESC rejected) / `encode_mcbpcy_v2`
     (P-gated skip bit, intra-quotient ac_pred bit) /
     `encode_cbpy_v1v2` (the spec/07 §1.2 `15 − cbpy` wrap incl. the
     v2 `remainder == 3` no-wrap sub-type),
     `mb::encode_intra_dc_diff_v1v2` (H.263 §5.4.1 size+value scheme,
     spec/16 §2), and `mv::encode_mv_v1v2` (+
     `mv_v1v2_component_reachable`, the [−32, +32] per-component
     toroidal residual of spec/07 §3.2).
  7. v1/v2 picture encoders (`enc::encode_iframe_v1v2` /
     `encode_pframe_v1v2`) over a version-independent analysis core
     factored out of the v3 paths (`analyse_intra_mb`,
     `write_intra_blocks`, `analyse_inter_residual`, and a
     `motion_search` parameterised over the per-version reachability
     rule). v1 I-frame MBs emit MB-type 3 INTRA behind the
     always-present COD bit; v2 emits the intra quotient + ac_pred
     bit; P-frames share the skip / motion / G4-residual shape with
     the per-component MV pair (`tests/v1_v2_encoder.rs`, 5 tests ×
     both versions).
  8. Registered v1/v2 encoder factories (`encoder::make_encoder_v1` /
     `make_encoder_v2`) — all three family members are now
     encode+decode in the registry; the registered-loop test runs for
     every codec id.
  9. **v1 INTER4V (4-MV, MB-type 2) encoding** — the traced spec/16
     §3.1 four-MV path now has an encode-side exerciser: a greedy
     per-Figure-6-8-block motion search walks the same interleaved
     Figure-7-34 predict → code → commit order the decoder replays
     (`Macroblock4MvDecoderNeighbours`), the MC prediction mirrors
     `mc_macroblock_4mv` (per-block luma + §7.6.3.4 sum/2K chroma
     derivation), and the mode decision emits `mcbpc = 8 + cbpc` when
     the 4-MV SAD beats 1-MV by more than the extra-codeword margin
     (v2 never takes the branch — its 8-symbol alphabet has no INTER4V
     code, spec/16 §3.3). Pinned by a comparative test: on
     divergent-block-motion content at coarse quant the v1 encode is
     near-exact (MAE < 1) while the 1-MV-only v2 encode of identical
     input carries over 2× the error.
  Test suite 431 lib-only → 606 total (0 failed). The encoder targets
  THIS crate's decoder: for the one table whose wire codes are
  canonically reconstructed rather than binary-extracted (the v3
  128-entry joint MCBPCY — see the README docs-gap on
  `region_05eac8`), byte-exactness against the original binary
  remains unverified either way; every binary-extracted table
  (G0..G5 primaries, both v3 joint-MV VLCs + byte LUTs, the v1/v2
  MV/MCBPC/DC-size tables, the shared v1/v2 CBPY table from
  `region_053640` — itself build-cross-checked against the H.263
  Table 8 codes — and the four v3 intra-DC VLCs) is used verbatim in
  both directions.

### Tests

- round 374: **picture-level coverage of the intra-in-P macroblock path
  across all three versions.** Every prior P-frame picture-level test
  decoded an *inter* MB; the `decode.is_intra` branch of the v3 / v1 / v2
  P-frame drivers (the intra-in-P MB, reached after the skip bit when the
  joint-MCBPCY / MCBPC index lands in the intra partition) was shipping
  but untested end-to-end. Seven new tests close that:
  - `tests/v3_pframe_intra_in_p.rs` (4 tests): a v3 P-frame with a single
    intra-in-P MB (joint-MCBPCY idx < 64, the I-type/low half per patent
    US 6,563,953 Table 1 / `audit/02` §1.4) reconstructs intra content
    *independent of the reference* (DC-only → spatially uniform, not a
    copy of a striped reference); the post-VLC ac_pred bit (spec/05 §3.2)
    is consumed under both polarities; and on a **CBP-coded** block a real
    G3 `(run=1, level=1)` AC token lands at a different natural position
    under zigzag (ac_pred=0) vs alternate-horizontal (ac_pred=1), so the
    two reconstructions DIFFER — proving the ac_pred bit genuinely routes
    into the scan dispatch.
  - `tests/v1_v2_pframe.rs` (+3 tests): the v2 analogue of the
    CBP-coded ac_pred scan-flip (spec/07 §2.4, G5 luma per spec/14 §3.2);
    plus the **v1** P-frame intra-in-P path, which reads **no** ac_pred
    bit (spec/07 §1.4) — a DC-only pin (reconstructs flat grey 128, not a
    gradient-reference copy) and a CBP-coded zigzag-only pin (the G5 AC
    walk fires without a phantom ac_pred read desynchronising the stream).
  - `src/picture.rs` (+1 lib test): a 2-MB v3 P-frame with an intra-in-P
    MB(0,0) followed by an inter MB(1,0) — the intra-in-P MB must leave
    its `MvGrid` cell `Absent` (not leak a MV), so MB(1,0)'s §7.6.5
    predictor is the rule-4 all-zero predictor and its final MV equals its
    raw residual; reconstructed independently via `apply_mc_to_mb`.
  - `tests/v3_pframe_intra_in_p.rs` (+1 test): a **deterministic** (no
    ffmpeg) I-frame → intra-in-P P-frame sequence driven through the
    registered `Decoder` trait (`send_packet` / `receive_frame`) — covers
    the classifier dispatch, the `last_picture` reference threading across
    packets, and the `picture → Frame::Video` plane conversion that the
    `decode_picture` tests skip.
  Lib tests 424 → 425; integration tests +8
  (`v3_pframe_intra_in_p.rs` new with 5; `v1_v2_pframe.rs` 18 → 21).
  Purely additive; no runtime path changed.

### Added

- round 366: make the **v3 1-MV-per-MB invariant a first-class,
  gap-#1895-grounded property**. New `McbpcyDecode::num_motion_vectors()`
  const accessor returns **0 for intra (I-type) MBs and 1 for inter
  (P-type) MBs** — mirroring the v1/v2 `V1V2McbpcyDecode` field but with
  the v3-specific fact baked in: the 128-entry joint MCBPCY alphabet
  (`region_05eac8`, patent US 6,563,953 Table 1, `audit/02` §4)
  partitions only into 64 I-type + 64 P-type entries, carries no INTER4V
  code, and no traced v3 bitstream signal selects 4-MV (the per-MB driver
  `1c2131ff` → MCBPCY `1c21782f` → MV decoder `1c217f5a` invokes the MV
  decoder exactly once per inter MB; spec/05 §3 / spec/06 §1). The v3
  P-frame driver `picture::decode_pframe_mb` now takes its MV count from
  this accessor (with a `debug_assert` + hard-error guard against any
  count ≠ 1), keeping it structurally parallel to the v1/v2
  `decode_pframe_mb_v1v2` INTER4V dispatch and giving a future round a
  single switch to flip if the docs ever resolve the v3 4-MV trigger.
  A new lib test
  (`mcbpcy::tests::v3_mcbpcy_num_motion_vectors_is_one_for_every_inter_symbol`)
  decodes all 128 symbols and pins the accessor (0 for the low half, 1
  for the high half, never 4). Lib tests 423 → 424; no v3 runtime
  behaviour change (the count was already implicitly 1).

### Tests

- round 366: deepen the **v1/v2 INTER4V (4-MV) picture-level coverage**
  — the real, traced 4-MV path per `spec/16` §3.1 (MB-type 2). Two new
  integration tests in `tests/v1_v2_pframe.rs`:
  `v1_pframe_inter4v_uniform_mv_shifts_luma_and_derives_chroma` drives a
  uniform-residual INTER4V MB and pins (a) block 1's luma 8×8 shifting
  one column under its (+2, 0) half-pel final MV and (b) the §7.6.3.4
  chroma derivation actually firing — both chroma planes diverge from
  the reference, distinguishing the real `mc::chroma_mv_from_four_luma`
  (sum/2K + Table 7-12) from a silent (0, 0) fallback. The prior INTER4V
  tests used four (0, 0) MVs (trivial chroma derivation) or a single
  non-zero block, so the uniform-MV chroma derivation was previously
  unexercised through the full decoder.
  `v1_pframe_inter4v_applies_cbp_residual_on_block` pins that
  `decode_inter_residual_blocks` runs *after* the 4-MV MC on the INTER4V
  path (not only the 1-MV path): a CBP-coded luma block 0 carries the
  shortest G4 sub-class-B terminator and perturbs that 8×8 away from the
  flat-128 MC copy while the three uncoded blocks stay verbatim.
  Integration tests `v1_v2_pframe.rs` 15 → 17.
- round 366: add the first **picture-level INTER4V → 1-MV-neighbour MV
  propagation** pin (`v1_v2_pframe.rs::`
  `v1_pframe_inter4v_neighbour_propagates_block2_mv_to_next_mb`). A
  2-MB-wide v1 P-frame decodes MB(0,0) as INTER4V (block 1 at a +2
  half-pel MV) and MB(1,0) as a 1-MV inter MB with MVD (0,0); per Figure
  7-34 (`mv_pred::bordering_block_of_neighbour`, round 208/306) MB(1,0)'s
  1-MV predictor must source MB(0,0)'s **block 2** (top-right 8×8) cell,
  so MB(1,0) reconstructs as the reference shifted by exactly that MV.
  The test independently replays MB(0,0)'s within-MB Figure-7-34 driver
  (cross-checking block 1's MV against the production `decode_mv_v1v2`)
  and reconstructs MB(1,0) via the public `mc::mc_macroblock`, guarding
  the inequality against a dropped predictor (which would copy the
  reference unshifted). All prior INTER4V coverage was single-MB or
  synthetic-grid only, so the cross-MB bordering-cell propagation through
  the full decoder was previously unexercised. Integration tests 17 → 18.

### Other

- round 366: correct the stale `src/mc.rs` module doc that still claimed
  the MC kernel is "v3 P-frames" only and that the decoder "reject[s] all
  P-frame MB-types other than 0" / "does not yet carry the per-4x4
  INTER4V signalling". INTER4V (4-MV) MC has shipped for v1/v2 since
  round 335 (`mc_macroblock_4mv` lives in this module). The doc now
  enumerates the 1-MV vs INTER4V luma paths, the two chroma-MV
  derivations (`chroma_mv_from_luma` `>> 1` toward −∞ vs
  `chroma_mv_from_four_luma` sum/2K + Table 7-12), and records the
  `spec/16` §3.1 fact that 4-MV is the v1/v2 MCBPC MB-type 2 while the v3
  128-entry joint MCBPCY alphabet (`region_05eac8`, patent 6,563,953
  Table 1) carries only an I-type/P-type split and **no** INTER4V code,
  so the v3 picture decoder stays 1-MV-per-MB (docs gap #1895). Comment
  only; no behaviour change.

### Fixed

- round 362: **correct the v3 joint-MCBPCY intra/inter partition
  polarity.** The 128-entry joint-MCBPCY alphabet partitions at 64:
  indices 0..63 are I-type (intra) MBs and 64..127 are P-type (inter) —
  established by patent US 6,563,953 Table 1 (`audit/02` §1.4, "indices
  0..63 are MB type = I; 64..127 are MB type = P") and corroborated by
  the table's own `(128, 64)` partition-boundary header. The decoder had
  `is_intra = (idx >= 64)`, the **inverse**, which mis-classified every
  coded v3 P-frame macroblock (P-type/inter entries were routed to the
  intra-in-P pipeline and I-type/intra entries to the inter MV/MC path).
  The earlier reading mis-resolved the binary's `test bl,0x40; je
  0x1c2178bd` (spec/05 §3.2): `je` is jump-on-zero, so the
  `MB-type = 3` (intra-in-P) target is reached when bit 6 is **clear**
  (`idx < 64`), not set — and it pre-dated the `audit/02` patent
  extraction that resolved the spec/99 §3.1 ambiguity. The I-frame path
  is unaffected (it treats every MB as intra and consumes only the
  low-6-bit CBP). A new `mcbpcy::tests::v3_mcbpcy_partition_low_half_is_intra`
  unit test decodes all 128 symbols and pins `is_intra == (idx < 64)`,
  exercising both halves, so a future regression cannot silently
  re-invert it. The hand-crafted v3 P-frame inter tests were updated to
  craft inter MBs at idx 64..127 (e.g. idx 64 for CBP=0) instead of the
  previously-used low-half indices.

### Tests

- round 362: add a **discriminating** end-to-end pin for the alternate
  joint-MV VLC table
  (`picture::tests::pframe_alternate_mv_table_selects_alternate_byte_lut`).
  The pre-existing alternate-table P-frame test only used joint symbol 0
  (MV (0,0)), which is identical in the default and alternate byte LUTs,
  so it could not prove the `mv_table_sel=1` path actually consults the
  alternate table. The new test feeds the 9-bit wire pattern `010011111`,
  which decodes to alternate symbol 36 → alternate byte-LUT MV (+4, 0)
  half-pel (an integer +2-pixel shift), whereas the default table maps the
  same bits to (-2, -2). Asserting the decoded luma equals the reference
  translated by exactly (+2, 0) — and that the identical payload under
  `mv_table_sel=0` yields a different picture — pins that the selector
  routes through the alternate VLC + its paired byte LUTs (spec/16 §1).
  Provenance asserts on `MV_V3_ALT_RAW[36]` and the alt/default byte-LUT
  entries fail loudly if a future re-extraction shifts the alphabet.

### Other

- round 362: scrub stale comments that described the alternate
  `mv_table_sel=1` joint-MV path as "still a placeholder / unsupported /
  256-byte truncation" (`src/picture.rs`, `build.rs`). The full 8804-byte
  / 1100-entry alternate source landed in Extractor 07 (spec/16 §1) and
  decodes end-to-end; the byte LUTs at VMAs 0x1c25c320 / 0x1c25c770 pair
  with it. Also correct the `decode_pframe` doc that claimed intra-in-P AC
  is "placeholder/DC-only" — all six G-families carry their real
  packed-Huffman primary VLC (round 234) and intra-in-P routes through the
  same `FromHeader` AC dispatch as I-frames.

- round 359: add the first picture-level end-to-end pin for the v3
  P-frame **median-predictor propagation** across a multi-MB row
  (`picture::tests::pframe_median_predictor_propagates_neighbour_mv`).
  A 2-MB-wide P-frame decodes MB(0,0) at a non-zero integer-pixel joint-MV
  symbol (picked at runtime from the extracted `MVDX/MVDY_V3_BYTES` LUTs,
  spec/16 §1, so the test follows the tables if re-rolled) and MB(1,0) at
  joint-MV symbol 0 (residual 0); the §7.6.5 single-valid-neighbour
  rule-3 substitution must promote MB(0,0)'s MV into MB(1,0)'s predictor
  so MB(1,0) reconstructs at the propagated MV. The assertion compares
  MB(1,0)'s decoded luma against an independent `apply_mc_to_mb`
  reconstruction at the propagated MV and guards the inequality against a
  dropped predictor (which would copy the reference unshifted). All prior
  v3 P-frame inter tests used MV (0,0) or single MBs, so the non-trivial
  median predictor was previously unexercised at the picture level.

### Other

- round 352: scrub external-implementation naming from `src/` doc
  comments (`mc.rs`, `ac.rs`) — the half-pel rounding, unrestricted-MV
  edge-extend, and chroma-MV reduction are re-cited to their H.263 /
  MPEG-4 §7.6.3.x spec authority instead of naming a third-party
  decoder. Comments only; no behaviour change.
- round 352: ground the v1 P-frame MB-type decomposition in the
  re-extracted authoritative table `tables/region_053140_mbtype.csv`
  (Extractor 07 / `spec/16` §3). New `build.rs` emitter (`emit_mbtype_v1`)
  loads the 21-symbol map into `MB_TYPE_V1_INFO`
  `[(mb_type, is_intra, num_motion_vectors); 21]`, cross-checking the
  `spec/16` §3.1 decomposition (`mb_type == symbol >> 2`), the
  {1, 1, 4, 0, 0} MV-count map, and the intra classification at build
  time. `mcbpcy::decode_mcbpcy_v1` now classifies `is_intra` and exposes
  `num_motion_vectors` from this table instead of the implicit
  `mb_type == 3` test — fixing a correctness bug where MB-type 4
  (INTRA+Q, symbols 16..=19) was mis-classified as inter and v1 I-frame
  MBs carrying it were wrongly rejected. The v1 P-frame INTER4V dispatch
  in `picture::decode_pframe_mb_v1v2` is now driven by
  `num_motion_vectors` (1 → 1-MV, 4 → INTER4V) so the two stay in lock
  step with the extracted table. STUFFING/ESC (symbol 20) is rejected
  explicitly. New tests:
  `mcbpcy::v1_mbtype_table_pins_inter4v_and_intra_plus_q`,
  `v1_v2_pframe::v1_iframe_intra_plus_q_mb_type_4_decodes`; the existing
  `v1_mcbpc_round_trip_every_symbol` now cross-checks every symbol's
  `(mb_type, is_intra, num_motion_vectors)` against `MB_TYPE_V1_INFO`.
- round 339: unblock the v1/v2 **intra** pixel pipeline (I-frames +
  intra-in-P MBs), driven by the re-extracted `spec/16` §2 + the
  binary-extracted DC-size tables (`tables/region_0542c0_dcsize.csv`
  luma, `region_0543c0_dcsize.csv` chroma). spec/16 §2 established that
  the v1/v2 intra-block driver gates on version (`cmp [esi+8], 3`): for
  v < 3 the DC differential is decoded through the classic H.263 §5.4.1
  / MPEG-4 Part 2 §7.4.3 size+value scheme (`sub_15790`) using the
  binary's own luma/chroma size-category tables (VMAs `0x1c2542c0` /
  `0x1c2543c0`), NOT the v3 direct-value DC VLC and NOT the v3
  `[esi+0x8bc]` `dc_size_sel` selector. The previous gate cited that
  selector's untraced construction-time default; since v1/v2 never
  consult it, the gate is dissolved. New `build.rs` emitter
  (`emit_dc_size_v1v2`) loads the two CSVs into
  `DC_SIZE_LUMA_V1V2_RAW` / `DC_SIZE_CHROMA_V1V2_RAW` with a
  prefix-code cross-check; new `mb::decode_intra_dc_diff_v1v2` +
  `mb::decode_intra_block_full_v1v2` implement the size+value decode
  with the H.263 signed-DC fixup; `picture::decode_iframe_v1v2` +
  `decode_intra_mb_v1v2_to_picture` wire the I-frame and intra-in-P
  paths through the shared spatial DC predictor + intra AC walk (luma
  G5, chroma G4 per spec/14 §3.2). Replaces the three stale
  `Unsupported`-gate assertions with positive intra-decode tests
  (v1/v2 DC-only flat-grey I-frame, non-zero DC luma shift, v2
  intra-in-P, registered-decoder `send_packet` v2 I-frame). Real-content
  bit-exactness against an encoder oracle remains a pending Auditor item.
- round 335: wire the v1 P-frame inter MB sub-types from the re-extracted
  `spec/16` §3.1 + `tables/region_053140_mbtype.csv`. The P-frame MB-type
  (= mcbpc >> 2) now selects the motion mode with the traced MV-count map
  {1, 1, 4, 0, 0}: MB-type 0 (INTER) and MB-type 1 (INTER+Q) decode as
  1-MV inter MBs (the v1 MCBPCY body reads no quantiser-delta bit per
  spec/07 §1.4 — "+Q" is the H.263-Table-8 lineage name only), and
  MB-type 2 (INTER4V) loops the per-component MV decoder 4× over the
  Figure 6-8 8x8 blocks, threading each block's final MV through the
  Figure-7-34 `Macroblock4MvDecoderNeighbours` predictor, applying
  per-block half-pel luma MC and the §7.6.3.4 sum/2K chroma MV
  derivation (new `mc::mc_macroblock_4mv` / `chroma_mv_from_four_luma`,
  Table 7-12 eighth→half rounding). Previously every `mb_type != 0`
  inter MB returned `Unsupported`; INTER+Q and INTER4V MBs now decode
  end-to-end. Adds picture-level integration tests (INTER+Q 1-MV,
  INTER4V four-zero-MV copy, INTER4V per-block MV shift) plus `mc` unit
  tests for the chroma derivation and 4-MV macroblock MC.
- round 326: wire the v3 alternate joint-MV VLC (`mv_table_sel == 1`,
  VMA `0x1c25a0b8`) end-to-end. The full 8804-byte source (1100 entries,
  ESC at index 1099, Kraft = 1.0, bit-lengths 2..15) was re-extracted in
  Extractor 07 (spec/16 §1, `tables/region_0594b8_mvvlc.csv`),
  superseding the earlier 256-byte truncation. `build.rs` emits
  `MV_V3_ALT_RAW` (with build-time Kraft completeness + contiguous-index
  checks); `mv::decode_mv_with_table(.., MvTable::Alternate)` now decodes
  through the same canonical-Huffman builder and the already-wired
  alternate `(MVDx, MVDy)` byte LUTs instead of returning `Unsupported`.
  P-frames with `mv_table_sel == 1` (e.g. div3.avi frames 37/38/40,
  div4.avi frames 1/16) no longer reject at the inter-MB MV decode.
- round 317: narrow the v1/v2 I-frame & intra-in-P docs gap to the
  single untraced construction-time default of intra-DC-size selector
  `[esi+0x8bc]`; correct the prior diagnostic that misattributed the
  CBP spatial-prediction LUT pair `0x1c23a788/0x1c23a7b0` (patent
  7,054,494) as a DC-prediction blocker. The intra kernel, DC-predictor
  gradient routine, and all four intra-DC-size VLC tables are confirmed
  shared with v3 and already wired (spec/01 §1.4, spec/04 §2.6, spec/99
  §4.4/§4.5).

## [0.0.8](https://github.com/OxideAV/oxideav-msmpeg4/compare/v0.0.7...v0.0.8) - 2026-06-15

### Other

- round 309: G3 intra-luma I-frame end-to-end via decode_picture FromHeader
- round 306: route P-frame 1-MV predictor neighbour candidates through resolve_block_candidates
- validate 3-tier ESC LMAX/RMAX against binary ESC-extension arrays (spec/08 §4.1)
- per-G-family pri_A/pri_B runtime-binding accessors (spec/14 §3)
- round 285: v1/v2 P-frame pixel pipeline (skip + inter MBs) end-to-end
- correct H.263 quantiser-parity bias direction to spec/08 §5
- round 266: typed-primitive accessors on picture-header parser
- round 254: per-descriptor field-offset accessors on GFamily
- round 251: alternate-variant v3 MVDx / MVDy byte LUTs landed
- drop release-plz.toml — use release-plz defaults across the workspace
- round 246: MvGrid video-packet / GOB boundary-reset helpers + iter_cells
- round 243: per-MB 4-MV decoder -> MvGridCell one-shot bridge
- round 240: decode_pframe MV cache routed through MvGrid
- round 234: G0..G3 packed-Huffman primary VLC wired
- round 227: picture-wide MV grid -> NeighbourSet builder
- round 221: 4-MV stateful predict/commit driver with NeighbourSet-aware bordering
- round 214: 4-MV neighbour-state resolver (1-MV vs 4-MV per neighbour)
- round 208: 4-MV neighbour-MB bordering-cell picker (Figure 7-34)
- round 202: README — add 4-MV-per-MB predictor batch surface status row
- round 202: 4-MV-per-MB bitstream integration tests (Figure 7-34 + real joint-VLC)

### Changed

- **Round 306 — P-frame 1-MV predictor neighbour-MB candidate
  construction routed through the spec-derived resolver** (2026-06-15):
  `picture::one_mv_predictor` (shared by the v3 and v1/v2 P-frame
  paths) now builds its `mv_pred::BlockCandidates` via
  `mv_pred::resolve_block_candidates(Block::TopLeft, nset, [None; 3])`
  (round 214) instead of hand-picking 4-MV neighbour cells with
  open-coded `mvs[1]` / `mvs[2]` / `mvs[2]` raster indices. For a
  4-MV-coded neighbour the §7.6.5 predictor sources the
  physically-bordering 8x8 cell per Figure 7-34 (round 208
  `bordering_block_of_neighbour`,
  `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`): the
  current MB's block 1 takes its left neighbour's block 2 (TR), its
  above neighbour's block 3 (BL), and its above-right neighbour's
  block 3 (BL). The old indices agreed with that mapping but
  duplicated the Figure 7-34 table inline; routing through the single
  documented resolver removes the duplication and keeps the picture
  path provably consistent. Two new lib tests pin the all-`OneMv` /
  `Absent` path (unchanged byte-for-byte) and the FourMv bordering-cell
  selection. Lib tests 401 → 403 (+2). No runtime behavioural change on
  the shipping 1-MV path.

### Added

- **Round 309 — G3 intra-luma I-frame end-to-end through the
  `decode_picture` `FromHeader` boundary** (2026-06-15): closes the
  coverage gap between round 234 (which wired the G0..G3 packed-Huffman
  primary VLCs into `AcVlcTable::v3_intra_g{0,1,2,3}`) and the
  production decode entry point. Until this round the only G0..G3
  coverage was per-table / per-symbol (`tests/g0_g3_extended.rs`); no
  test drove a real-table G3-coded I-frame through the shipping public
  `picture::decode_picture` with the default
  `picture::AcSelection::FromHeader`. New `tests/g3_iframe_end_to_end.rs`
  (3 tests) builds a single-macroblock 16×16 v3 I-frame with
  `ac_luma_sel == 0` (→ G3 per
  `docs/video/msmpeg4/spec/14-pri-ab-runtime-binding.md` §3.1) whose
  luma block 0 carries two real G3 AC tokens — one sub-A continuing
  token (idx 0 = `(last=false, run=0, level=1)` per spec/09 §2) and one
  sub-B terminator (spec/13 §2: G3 `count_A=132, count_B=84`, sub-B is
  `last=true`) — decodes it through `decode_picture`, and asserts the
  coded block reconstructs **non-DC-only** pel content (the G3 133-entry
  canonical-Huffman walker actually consumed the AC bits). A companion
  test decodes the same stream with `AcSelection::Placeholder` (empty
  table) and asserts the DC-only fallback produces a spatially-uniform
  block, so the non-uniformity in the FromHeader case is attributable to
  the real G3 AC walk. A structural guard pins the G3 alphabet at
  133 entries with `lmax`/`rmax` present (spec/11 §5 row 4 / spec/08
  §4.1). The MCBPCY symbol is encoded by using the production
  `mcbpcy::decode_mcbpcy` as a black-box oracle (no dependency on the
  private canonical-table builder). Also corrects stale doc comments on
  `AcSelection::FromHeader`, `picture::{luma,chroma}_ac_table_for`, the
  four `AcVlcTable::v3_intra_g{0,1,2,3}` constructors, and the
  `build_g_extended_synthetic` helper, all of which still claimed
  G0..G3 "fall back to DC-only reconstruction" / "packed-Huffman sources
  not yet extracted / flagged suspect" — factually false since round
  234. Integration tests +3; no runtime path is rewired, no new tables.

- **Round 299 — 3-tier ESC LMAX/RMAX validated against the binary's
  authoritative ESC-extension arrays** (2026-06-14): a new lib test
  `ac::tests::esc_ext_arrays_match_derived_lmax_rmax_all_g` pins the
  **derived** level-extension (LMAX, indexed by run) and run-extension
  (RMAX, indexed by level) tables that drive the v3 intra 3-tier escape
  body (`decode_escape_body`, rounds 7/27/126) against the binary's
  **ground-truth** ESC-extension arrays extracted at `region_060988`
  (VMA `0x1c261588..0x1c261e00`). Per
  `docs/video/msmpeg4/spec/08-descriptor-constants.md` §1-§2 those four
  per-descriptor arrays (`desc+0x0c`/`+0x10` level-ext, `desc+0x14`/
  `+0x18` run-ext) are the only data the intra (`0x1c216d97`) and inter
  (`0x1c215e6f`) kernels read on the first- and second-tier ESC paths;
  spec/08 §4.1 left their *content* semantics OPEN. This round resolves
  them empirically from the Implementer side: for all six G-families the
  derived tables reproduce the extracted bytes **exactly** over each
  array's meaningful extent, with the sub-A half mapping to `[last=0]`
  and the sub-B half to `[last=1]`. The run-ext arrays carry a
  never-indexed `0xFFFFFFFF` sentinel in the `level == 0` slot (an ESC's
  re-decoded base symbol always has `|level| >= 1`); slices also carry
  trailing bytes past the last alphabet-representable run/level
  (spec/08 §2.4 only upper-bounds each array at `count_A * 4`), neither
  of which is compared. The previously "derived from the same
  packed-Huffman source the primary VLC consumes, but never
  cross-checked against the binary's own extension tables" 3-tier ESC
  body is therefore now a **ground-truth-verified** decode path. Lib
  tests 400 → 401 (+1). Purely additive; no runtime path is rewired and
  no new tables are introduced.

- **Round 292 — per-G-family `pri_A` / `pri_B` runtime-binding
  accessor surface** (2026-06-14): four additive const-fn accessors on
  `g_family::GFamily` mirroring the static binding table in
  `docs/video/msmpeg4/spec/14-pri-ab-runtime-binding.md` §3.
  `pri_a_vma()` / `pri_b_vma()` return the `.data` VMA the constructor
  `mb_mv_struct_init` (`0x1c210643`) writes into each descriptor's
  `+0x1c` (level-magnitude byte array) / `+0x20` (run-value u32 array)
  slots — one-shot, static, never re-bound (spec/14 §2.1 / §5.2 item 5):
  G0=`0x1c257860`/`0x1c2575c0`, G1=`0x1c257bf0`/`0x1c257908`,
  G2=`0x1c257f00`/`0x1c257cb0`, G3=`0x1c2581a8`/`0x1c257f98`,
  G4=`0x1c258230`/`0x1c258298`, G5=`0x1c258430`/`0x1c258498`.
  `pri_a_size_bytes()` returns `count_A` (1 byte per symbol) and
  `pri_b_size_bytes()` returns `count_A * 4` (a u32 run value per
  symbol) per spec/14 §1 / §3. This completes the schema-binding
  reference the round-254 `field_state_struct_offset(PriABase/PriBBase)`
  surface points at (slot offsets) by resolving each `+0x1c` / `+0x20`
  slot to its bound `.data` VMA and array size — giving a future Auditor
  a single canonical reference for the binary's per-family `pri_A` /
  `pri_B` base pointers (`GFamily::field_state_struct_offset(field)` =
  the descriptor slot; `GFamily::pri_{a,b}_vma()` = the VMA stored
  there). Four new lib tests pin: every VMA against the spec/14 §3 §2.1
  literal-immediate disassembly; pri_A/pri_B byte sizes against the §3
  table and their definitional equality with `count_a()` / `count_a()*4`;
  the spec/14 §2.3 cluster containment + pairwise non-overlap for all 12
  arrays in `[0x1c2575c0, 0x1c258630)` with the tail ending exactly at
  the first per-slot packed-Huffman source VMA (`0x1c258630`); and the
  const-fn property. Lib tests 396 → 400 (+4). Purely additive; no
  runtime path is rewired and no new tables are introduced.

- **Round 285 — v1/v2 P-frame pixel pipeline (skip + inter MBs)
  end-to-end** (2026-06-12): new public
  `picture::decode_picture_v1v2` (+ `picture::MsV1V2Version`)
  decodes MS-MPEG4 v1/v2 P-frames into a `Picture`: picture header
  (spec/01 §1.4, the v1 UMV flag consumed as framing per spec/07 §3.4),
  per-MB skip bit + separate MCBPC / CBPY VLCs (spec/07 §1-§2 via the
  existing `mcbpcy::decode_mcbpcy_v{1,2}`), the §7.6.5 1-MV predictor
  (same helper `0x1c217c8c` as v3 per spec/07 §3.5, threaded through
  the shared `MvGrid` surface), the per-component MV pair
  (`mv::decode_mv_v1v2`, spec/07 §3), half-pel MC, and the G4 inter AC
  residual for every CBP-coded block — per spec/14 §3.1 the v1/v2
  fallthrough at `0x1c212917` binds the inter/chroma DCT descriptor to
  G4, and per spec/99 §6 the inter kernels share the hard-zigzag /
  scan-start-0 / single-tier-ESC shape across v1/v2/v3.
  `send_packet` on `msmpeg4v1` / `msmpeg4v2` now produces
  `Frame::Video` for P-frames instead of the blanket round-12
  `Unsupported`. Still gated with a *documented* `Unsupported` carrying
  the precise citation: v1/v2 I-frames + intra-in-P MBs (spec/07 §1.6
  pins that v1/v2 do not load the v3 spatial-prediction LUT pair at
  `0x1c23a788 / 0x1c23a7b0`, but no staged chapter documents the
  replacement DC-prediction rule or the v1/v2 intra DC-size descriptor
  binding) and the v1 non-zero inter sub-types (spec/07 §1.4 asserts
  the H.263 Table-8 lineage only structurally). Internally the v3
  P-frame path's 1-MV predictor lookup and CBP-driven inter-residual
  loop were factored into shared helpers (`one_mv_predictor`,
  `decode_inter_residual_blocks`) consumed by both version families —
  no behavioural change on the v3 path. New integration suite
  `tests/v1_v2_pframe.rs` (11 tests): all-skip identity copy (v1 4-MB
  + v2 1-MB), zero-MV inter copy (v1 + v2), non-zero-MV reference
  shift, G4-residual application confined to the coded block, the
  three documented `Unsupported` gates (I-frame, intra-in-P, v1
  sub-type ≠ 0) with diagnostic-content asserts, the missing-reference
  error, and an end-to-end `send_packet` docs-gap surface check. The
  ffmpeg-backed `v1_v2_mcbpcy.rs` expectation was updated to the new
  stop-line (a real MP42 I-frame now reaches the DC-prediction gate).
  Integration tests 97 → 108 (+11); lib tests unchanged at 396.

### Fixed

- **Round 275 — Dequant parity-bias direction corrected to spec/08 §5**
  (2026-06-11): `iq::dequantise_h263` computed the H.263 §6.2.2.1 Eq. 12
  quantiser-parity offset as `bias = PQUANT - (PQUANT & 1)`, i.e. it
  subtracted 1 for **odd** PQUANT. The sandbox-verified runtime
  materialisation in `docs/video/msmpeg4/spec/08-descriptor-constants.md`
  §5 (backed by `audit/06` hand-patched-PQUANT trials watched at
  `[ctx+0x138]`) establishes the opposite: the per-frame even-parity flag
  is `1` iff PQUANT is **even**, and `bias = PQUANT - even_flag`, so
  `bias = PQUANT - 1` for even PQUANT and `bias = PQUANT` for odd PQUANT.
  `spec/07` §4.2's prose annotation "`edx = 1 if odd else 0`" was wrong
  about its own quoted `neg edx; sbb edx,edx; inc edx` idiom — that
  idiom over `edx = PQUANT % 2 ∈ {0,1}` yields `1 - CF` = 1 if even, 0 if
  odd, matching spec/08 §5. The fix flips the parity term to
  `even_flag = 1 - (q & 1)` so every non-zero AC/inter coefficient
  dequantises to `mag·|level| + (PQUANT - even_flag)` for even PQUANT and
  the correct half-step for odd — affecting bit-exact reconstruction of
  every even-PQUANT block (the majority of real streams). A new lib test
  `iq::tests::parity_bias_direction_matches_spec_08_section_5` sweeps
  PQUANT 1..=31 and pins both arms; four existing tests
  (`iq::tests::{positive_odd_quant,negative_even_quant}`,
  `ac::tests::{decode_intra_block_runs_dequantise,
  inter_block_dequantises_with_level_start_zero}`) and one integration
  expectation (`tests/intra_ac_candidate.rs`) were updated to the
  corrected values (q=5 odd |level|=1 → 15, not 14; q=4 even |level|=2
  → −19, not −20). Lib tests 395 → 396 (+1). The module docs now carry
  the spec/07-vs-spec/08 reconciliation inline.

### Added

- **Round 266 — Typed-primitive accessor surface on the picture-header
  parser** (2026-06-09): mirrors the raw `u8` per-frame selector fields
  on `header::MsV3PictureHeader` (and the compat-default constants on
  `header::V1V2V3CompatDefaults`) onto the existing typed
  `g_family::GFamily` / `mv::MvTable` enums via five new const-fn
  accessors. `mv::MvTable::from_sel(sel: u8) -> Option<Self>` resolves
  the picture-header `mv_table_sel` bit per
  `docs/video/msmpeg4/spec/06-mv-decoder.md` §3.2 (`0 → Default,
  1 → Alternate`); `mv::MvTable::to_sel(self) -> u8` is the inverse.
  `header::MsV3PictureHeader::ac_chroma_family` /
  `ac_luma_family` /  `mv_table` delegate to the standalone
  dispatchers — chroma/luma per spec/14 §3.1 (`chroma: 0→G2, 1→G0,
  2→G4`; `luma: 0→G3, 1→G1, 2→G5`), MV per spec/06 §3.2 — so a caller
  holding a parsed header reaches the typed family without
  re-importing `GFamily` / `MvTable` or restating the per-frame slot
  identity. Two new associated fns on `V1V2V3CompatDefaults` —
  `v1_v2_fallthrough_chroma_family` / `v1_v2_fallthrough_luma_family`
  — surface the spec/14 §3.1 v1/v2 fallthrough cluster the runtime
  binds at the `0x1c212917` write site (`[esi+0xab0] = G4` chroma,
  `[esi+0xab4] = G5` luma), independent of the (zero-pinned,
  don't-care) `ac_chroma_sel` / `ac_luma_sel` compat fields. The
  `ac_chroma_family` / `ac_luma_family` accessors on the same struct
  intentionally dispatch through the **v3** selector path
  (`for_chroma_selector(0) = G2`, `for_luma_selector(0) = G3`) for
  the niche case of a caller threading a v1/v2 frame through a shared
  v3 entry point; the two surfaces deliberately diverge and a test
  (`compat_defaults_v3_dispatch_does_not_equal_v1_v2_fallthrough`)
  pins the divergence so a future round can't collapse them by
  accident. Seventeen new lib tests across `src/mv.rs::tests` (5) and
  `src/header.rs::tests` (12) pin every dispatch arm, the OOR-`None`
  defence-in-depth path on every accessor, the inverse round-trip
  for both selectors, the spec/14 §3.1 v1/v2 fallthrough family
  identities, the spec/06 §3.2 `mv_table_sel` mapping and the
  end-to-end parse-then-resolve flow through a real bit-packed v3
  P-frame header with `ac_chroma_sel=1` + `mv_table_sel=1`. Lib
  tests 378 → 395 (+17); integration tests unchanged. Purely
  additive; no runtime path is rewired and no new tables are
  introduced.

- **Round 254 — Per-descriptor field-offset accessor surface on
  `g_family::GFamily`** (2026-06-08): mirrors the 36-byte G-descriptor
  record schema documented in `docs/video/msmpeg4/spec/15-count-ab-per-g-family.md`
  §1 onto the in-tree dispatch enum. The new
  `g_family::GDescriptorField` enum names all nine `u32` slots
  (`DecoderObj` at `+0x00`, `CountA` at `+0x04`, `CountB` at
  `+0x08`, `SubALevelExtPtr` at `+0x0c`, `SubBLevelExtPtr` at
  `+0x10`, `SubARunExtPtr` at `+0x14`, `SubBRunExtPtr` at `+0x18`,
  `PriABase` at `+0x1c`, `PriBBase` at `+0x20`) and exposes
  `offset_in_record()` / `size_in_bytes()`. Two new const-fn
  accessors on `GFamily` — `field_offset(field)` and
  `field_state_struct_offset(field)` — combine the family's
  descriptor base with the field's record-relative offset, yielding
  the absolute state-struct VMA the constructor's literal-immediate
  stores at `1c210643..1c2108d0` write into (per spec/15 §2.1). Two
  new module-level constants `DESCRIPTOR_RECORD_BYTES` (`0x24`) and
  `DESCRIPTOR_CLUSTER_END_OFFSET` (`0xab0`) document the per-record
  size and the G0..G5 cluster span per spec/14 §1. Eight new lib
  tests in `src/g_family.rs::tests`
  (`descriptor_field_offsets_match_spec_15_record_schema`,
  `descriptor_field_all_lists_every_field_in_record_order`,
  `descriptor_field_offset_delegates_to_field`,
  `descriptor_field_state_struct_offsets_match_spec_15_disassembly`,
  `descriptor_field_state_struct_offset_decomposes`,
  `descriptor_record_size_and_cluster_end_match_spec_14`,
  `descriptor_fields_stay_inside_record_and_cluster`,
  `count_storage_offsets_consistent_with_count_values`) pin the
  record-schema against spec/15 §1 / §2.1: in particular, the per-G
  `count_A` / `count_B` storage VMAs match the constructor literals
  cited for every family (G0=`+0x9dc,+0x9e0`, G1=`+0xa00,+0xa04`,
  G2=`+0xa24,+0xa28`, G3=`+0xa48,+0xa4c`, G4=`+0xa6c,+0xa70`,
  G5=`+0xa90,+0xa94`). Lib tests 370 → 378 (+8); integration tests
  unchanged. Purely additive; no runtime path is rewired and no new
  tables are introduced.

- **Round 251 — Alternate-variant v3 MVDx / MVDy byte LUTs**
  (2026-06-07): wires the spec/06 §2.2 alternate-variant byte LUTs
  for the v3 joint MV decoder (`mv_table_sel == 1` path) at VMAs
  `0x1c25c320` (MVDx) and `0x1c25c770` (MVDy), each 1104 bytes
  (1099 alphabet entries + 5 bytes of alignment padding, identical
  shape to the default-variant LUTs at `0x1c25ee28` / `0x1c25f278`).
  Pre-extracted hex files `tables/region_05b720.hex` and
  `tables/region_05bb70.hex` (already present in
  `docs/video/msmpeg4/tables/`) are copied into the crate's
  `tables/` directory and parsed at build time by a new emitter
  `emit_mv_byte_lut_v3_alt` in `build.rs` — same shape as the
  default-variant `emit_mv_byte_lut_v3` (xxd parser + 1104-byte
  length assert + two `pub static …: &[u8; 1104]` arrays). The
  emitted constants [`crate::tables_data::MVDX_V3_ALT_BYTES`] and
  [`crate::tables_data::MVDY_V3_ALT_BYTES`] are ready for the
  future alt-VLC-source-aware `mv::decode_mv_with_table` body.
  Four new lib tests in `src/mv.rs::tests`
  (`mv_alt_byte_luts_have_expected_shape`,
  `mv_alt_byte_luts_in_six_bit_range`,
  `mv_alt_byte_luts_differ_from_default`,
  `mv_alt_byte_luts_cluster_around_bias`) pin the array shape, the
  unsigned-6-bit pre-biased value range, the distinct-from-default
  invariant, and the bias-32 distribution. The existing
  `mv_table_alternate_is_unsupported_with_diagnostic` test was
  extended to require the diagnostic now surfaces
  `MVDX_V3_ALT_BYTES` / `MVDY_V3_ALT_BYTES` and the spec/11 §5
  full-source size `8804` — the only remaining alt-path blocker.
  Lib tests 366 → 370 (+4); integration tests unchanged. Purely
  additive; `MvTable::Alternate` still returns the
  `Error::Unsupported` diagnostic.
- **Round 246 — `MvGrid` video-packet / GOB boundary-reset helpers +
  raster-order `iter_cells`** (2026-06-07): four purely-additive
  helpers on [`crate::mv_pred::MvGrid`] that close the documented
  gap in the type's "Boundary handling" section that video-packet /
  GOB boundary substitution per
  `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md` §
  "Boundary substitution" must be applied by the caller writing
  [`crate::mv_pred::MvGridCell::Absent`] into grid positions on the
  far side of the boundary.
  (1) [`crate::mv_pred::MvGrid::clear_cell`] resets one
  `(mb_x, mb_y)` position to [`crate::mv_pred::MvGridCell::Absent`]
  — the boundary-resync primitive, naming the intent and agreeing
  in observable effect with `set_cell(mb_x, mb_y,
  MvGridCell::Absent)`. (2) [`crate::mv_pred::MvGrid::clear_row`]
  bulk-resets every column of one MB row to
  [`crate::mv_pred::MvGridCell::Absent`] for the common
  video-packet-at-row-start case where the caller pre-resets the
  row above the new packet so the §7.6.5 candidate-neighbour
  lookups treat it as transparent. (3)
  [`crate::mv_pred::MvGrid::clear_all`] resets every cell in the
  grid to [`crate::mv_pred::MvGridCell::Absent`] without
  re-allocating the backing storage — picture-start grid reuse
  without a fresh allocation. (4)
  [`crate::mv_pred::MvGrid::iter_cells`] is a raster-order iterator
  over `(mb_x, mb_y, MvGridCell)` triples (same order as the P-VOP
  MB loop), useful for diagnostic / regression callers that want
  to pair their expected per-MB cell list against the grid
  contents without re-deriving the raster index manually. Eleven
  new lib tests in `src/mv_pred.rs::tests` pin: `clear_cell` sets
  the target position to `Absent` regardless of the starting
  variant (`Absent` / `OneMv` / `FourMv`) and leaves same-row
  siblings untouched; `clear_cell` matches `set_cell(.., Absent)`
  and is a no-op on out-of-bounds positions; `clear_row` resets
  every column of one row only and is OOB-no-op; `clear_all`
  matches a fresh `MvGrid::new(width, height)` in observable
  effect and preserves dimensions; `iter_cells` walks the
  documented raster order (`(0,0), (1,0), (2,0), (0,1), …`),
  round-trips its triples through `cell_at`, and on a fresh grid
  yields all `Absent` cells; the bulk-vs-per-cell equivalence
  `clear_row(mb_y) == clear_cell(0..width, mb_y)`; and a worked
  boundary-resync example where MB `(2, 1)` initially sees three
  populated neighbours, the caller `clear_cell`-s the three
  §7.6.5 neighbour positions
  (`(1, 1)` / `(2, 0)` / `(3, 0)`), and
  [`MvGrid::neighbour_set_for(2, 1)`] then reports every direction
  `Absent` (rule-4 / all-zero predictor). Lib tests 355 → 366
  (+11); integration tests unchanged. Purely additive; the
  existing 1-MV `picture::decode_pframe_mb` path is unchanged.
- **Round 243 — per-MB 4-MV decoder → `MvGridCell` one-shot bridge +
  `MvGridCell` query predicates + `MvGrid::dimensions`** (2026-06-07):
  three purely-additive `mv_pred` surface extensions that complete the
  per-MB-decoder → picture-wide-grid handoff plumbed by round 240.
  (1) [`crate::mv_pred::Macroblock4MvDecoder::finalise_to_grid_cell`]
  and [`crate::mv_pred::Macroblock4MvDecoderNeighbours::finalise_to_grid_cell`]
  return [`crate::mv_pred::MvGridCell::FourMv`]`(self.finalise())` so
  the future 4-MV-mode site can write
  `mv_grid.set_cell(mb_x, mb_y, decoder.finalise_to_grid_cell())` in
  one call without manually wrapping the `[Mv; 4]` in the cell enum.
  Both helpers pick up `finalise`'s `Mv::default()` substitution for
  any block that was never committed, matching the documented
  semantics. (2) Three `const fn` query predicates on `MvGridCell`:
  [`crate::mv_pred::MvGridCell::is_absent`] /
  [`crate::mv_pred::MvGridCell::is_one_mv`] /
  [`crate::mv_pred::MvGridCell::is_four_mv`], mirroring
  [`crate::mv_pred::NeighbourMvKind::is_absent`] (same name, same
  semantics) so a call site that treats the two types interchangeably
  via the `From<MvGridCell> for NeighbourMvKind` conversion reads the
  same way before and after. (3)
  [`crate::mv_pred::MvGrid::dimensions`] — a `const fn` returning
  `(width, height)` as a pair, in the same constructor-arg order as
  [`crate::mv_pred::MvGrid::new`], matching the per-axis accessors
  `width()` / `height()` and acting as a single-call alternative.
  Eight new lib tests in `src/mv_pred.rs::tests` pin: the four-commit
  sweep through `Macroblock4MvDecoder::finalise_to_grid_cell`
  returning the correct `FourMv` payload in Figure 6-8 raster order;
  the `Mv::default()` substitution on uncommitted blocks; the same
  shape on the `Macroblock4MvDecoderNeighbours` variant; the
  end-to-end round-trip through `MvGrid::set_cell` + `cell_at` +
  `neighbour_set_for` (writing a 4-MV cell at one position then
  reading it as the `left` neighbour of the next-column MB); the
  mutually-exclusive trichotomy of the three predicates over the
  three cell variants; `is_absent` agreement with the
  `NeighbourMvKind::is_absent` result on the converted side;
  `dimensions()` matching `(width(), height())` across (5, 7) /
  (4, 4) / (1, 1) shapes; and the `const fn` callable shape of
  `dimensions`. Lib tests 347 → 355 (+8); integration tests
  unchanged. Purely additive; the existing 1-MV
  `picture::decode_pframe_mb` path is unchanged.
- **Round 240 — `decode_pframe` MV book-keeping consolidated onto
  `MvGrid`** (2026-06-06): replaces the parallel `Vec<Option<Mv>>`
  raster-indexed MV cache that `picture::decode_pframe` /
  `decode_pframe_mb` carried since the round-9 P-frame skeleton with
  the production-tested [`crate::mv_pred::MvGrid`] /
  [`crate::mv_pred::MvGridCell`] surface introduced in round 227.
  The pframe MB loop now allocates one `MvGrid::new(mb_w, mb_h)` and
  drives the three §7.6.5 neighbour positions through
  [`crate::mv_pred::MvGrid::neighbour_set_for`], which folds the
  picture-edge substitution rule
  (`docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md` §
  "Boundary substitution") into a single
  [`crate::mv_pred::NeighbourSet`] without any per-axis range
  arithmetic at the pframe layer. Skip / inter MBs write
  [`MvGridCell::OneMv`] back into the grid via
  [`MvGrid::set_cell`]; intra-in-P MBs leave the cell `Absent` so the
  next MB's median predictor treats that column as zero (identical
  to the pre-round-240 `None` semantics). The
  [`NeighbourMvKind::FourMv`] branch is wired through as
  `mvs[1]` / `mvs[2]` / `mvs[2]` (the §7.6.5 bordering-cell picks
  for left / above / above-right respectively) so when the 4-MV-per-MB
  bitstream signalling lands in a future round the predictor layer
  is already consuming the correct cells. Two new lib tests pin the
  refactor:
  `picture::tests::round_240_mv_grid_neighbour_lookup_matches_legacy_arithmetic`
  walks a 4x3 mixed `OneMv` / `Absent` grid through both the new
  `MvGrid::neighbour_set_for` route and the historical
  raster-indexed `Vec<Option<Mv>>` lookups and asserts the
  `(left, top, top_right)` triple matches at every MB position; and
  `picture::tests::round_240_skip_mb_cell_is_one_mv_default` pins
  that a skipped MB writes [`MvGridCell::OneMv(Mv::default())`] (not
  `Absent`) so its neighbour position contributes a literal `(0, 0)`
  MV to the next MB's median predictor — equivalent to the
  pre-round-240 `Some(Mv::default())` skip-write. Lib tests 345 →
  347 (+2); integration tests unchanged in count.
- **Round 234 — G0..G3 packed-Huffman primary VLC wired** (2026-06-04):
  closes the OPEN status the README has carried for the four extended
  G-descriptor (G0..G3) canonical-Huffman primary VLCs. Per
  `docs/video/msmpeg4/spec/11-walker-format-resolved.md` §5 row 1-4
  the four packed-Huffman sources live at file `0x57a30` (G0, VMA
  `0x1c258630`, count 169), `0x57f80` (G1, VMA `0x1c258b80`, count
  186), `0x58558` (G2, VMA `0x1c259158`, count 149), and `0x58a08`
  (G3, VMA `0x1c259608`, count 133), each in the same `(code, bl)`
  u32-pair format that spec/11 §4 established for G4 / G5 / MCBPCY /
  intra-DC. The `region_*_full.hex` slices are copied into
  `crates/oxideav-msmpeg4/tables/`; a new
  `emit_packed_huffman_g_extended` build.rs emitter parses each
  source (header u32-LE count, then `count × (code:u32-LE,
  bl:u32-LE)` records, with the `0xFFFFFFFF` hole-sentinel branch
  mirroring helper A from spec/11 §3) and enforces three build-time
  invariants per source: file length `>= 4 + count * 8`, header count
  matches the spec/15 §3 alphabet shape, and Kraft sum is exactly
  `2^32` (saturated — unlike G4 / G5 which reserve one bl=9
  codeword for ESC at Kraft `1 - 2/1024`, G0..G3 use a regular
  bit-length slot at `idx == count_A` per spec/09 §2). The four
  emitted `G{0,1,2,3}_PRIMARY_RAW` arrays flow through the same
  `build_g_primary` builder the G4 / G5 wiring uses; the `GTable`
  enum now has six variants and each non-ESC idx is resolved to its
  `(last, run, level)` triple via `g_enum::GExtended::decode`
  (round 29). `AcVlcTable::v3_intra_g{0,1,2,3}` now return non-empty
  entries (169 / 186 / 149 / 133 each); the `_synthetic` variants
  are kept for diagnostic regression baselines. Two new lib tests
  pin (a) per-source alphabet size and Kraft saturation
  (`g0_g3_constructors_wire_packed_huffman_round_234`) and (b)
  per-idx agreement between the wired entries and `GExtended::decode`
  for the full `count_A + 1` alphabet of every source
  (`g0_g3_entries_agree_with_g_enum_decode`); a third
  (`g0_g3_round_trip_first_and_esc_entries`) runs `decode_token` for
  idx 0 (always `(last=false, run=0, level=1)` per spec/09 §2) and
  confirms the ESC sentinel surfaces at the expected position. The
  integration test
  `g0_g1_g2_g3_carry_lmax_and_rmax_post_round_7` (in
  `tests/g0_g3_extended.rs`) was renamed to
  `g0_g1_g2_g3_carry_lmax_and_rmax_post_round_234` and updated to
  assert non-empty entries (`expected_entries = count_A + 1` per
  source). The mp43.wmv I-frame luma DC-only-fallback observed in
  the round-5 implementer's static analysis is now unblocked:
  `picture::AcSelection::FromHeader` plus `for_luma_selector(0)`
  dispatches v3 intra-luma blocks through G3's 133-entry
  canonical-Huffman walker instead of the empty placeholder the
  round-7..233 path returned. Lib tests 343 → 345 (+2);
  integration tests unchanged in count.
- **Round 227 — picture-wide MV grid → `NeighbourSet` builder
  (`mv_pred::MvGrid` + `mv_pred::MvGridCell`)** (2026-06-04): two
  additive public APIs in [`mv_pred`] that lift r214's per-MB
  [`NeighbourSet`] to a per-picture grid the P-VOP driver can
  consult by `(mb_x, mb_y)`. [`mv_pred::MvGridCell`] is a per-MB
  cell variant (`Absent` / `OneMv(Mv)` / `FourMv([Mv; 4])`) with
  `From<MvGridCell> for NeighbourMvKind` so a grid cell promotes
  directly into a [`NeighbourSet`] field. [`mv_pred::MvGrid`] is a
  `width × height` raster-ordered `Vec<MvGridCell>` with `new`
  (all-Absent initial state), `width` / `height` accessors,
  `cell_at(mb_x, mb_y)` / `set_cell(mb_x, mb_y, cell)` (out-of-bounds
  reads return `Absent` per the picture-edge substitution rule;
  out-of-bounds writes are a no-op), and `neighbour_set_for(mb_x,
  mb_y)` — the load-bearing method that builds a [`NeighbourSet`]
  from the grid's three neighbour positions per Figure 7-34: left
  = `(mb_x - 1, mb_y)`, above = `(mb_x, mb_y - 1)`, above-right
  = `(mb_x + 1, mb_y - 1)`, with picture-edge cells (`mb_x == 0` /
  `mb_y == 0` / `mb_x + 1 == width`) substituted to `Absent` per
  the §7.6.5 rule 1 "outside the current VOP → not valid"
  boundary handling. The resulting [`NeighbourSet`] feeds straight
  into [`mv_pred::Macroblock4MvDecoderNeighbours::new`] or
  [`mv_pred::resolve_block_candidates`] so a P-VOP decoder can drive
  the r214/r221 predictor pipeline without reimplementing the
  picture-edge lookup each per-MB. Twelve new lib tests in
  `src/mv_pred.rs::tests` pin: `MvGridCell::default()` is `Absent`;
  the three `From<MvGridCell>` round-trips into `NeighbourMvKind`;
  `MvGrid::new(w, h)` starts all-Absent; out-of-bounds `cell_at`
  returns `Absent`; `set_cell` / `cell_at` round-trip with
  out-of-bounds-write no-op; the picture-corner `(0, 0)` yielding
  `NeighbourSet::ABSENT` regardless of grid contents; the
  top-edge case `(mb_x, 0)` for `mb_x > 0` with only `left`
  non-`Absent`; the left-edge case `(0, mb_y)` for `mb_y > 0` with
  `above` and `above_right` from row `mb_y - 1`; the right-edge
  case `(width - 1, mb_y)` with `above_right` substituted to
  `Absent`; the interior case with all three neighbours present
  including a `FourMv` neighbour promoted intact; a grid →
  [`Macroblock4MvDecoderNeighbours`] composition check including
  the bordering-cell pick for a 4-MV left neighbour; and the
  top-edge case threading through
  [`predict_macroblock_4mv_with_4mv_neighbours`] (block 1's
  predictor = the left neighbour MV per rule 3). Test suite 331 →
  343 (+12) lib tests; integration + doc tests unchanged. Purely
  additive; the existing `picture::decode_pframe_mb` 1-MV path's
  hand-built `BlockCandidates` is unchanged. Source —
  `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`
  §"Boundary substitution" (verbatim wording of the rule-1
  "outside the current VOP / video packet / GOB → transparent"
  cases) plus `docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`
  §7.6.5 rule 1. No new tables, no new decode logic; the grid
  composes the r214 `NeighbourSet` API over the picture-edge
  substitution rule the spec already mandates.

- **Round 221 — 4-MV stateful predict/commit driver with
  `NeighbourSet`-aware bordering** (2026-06-03): adds
  [`mv_pred::Macroblock4MvDecoderNeighbours`], the
  [`mv_pred::NeighbourSet`]-driven analogue of r196's
  [`mv_pred::Macroblock4MvDecoder`]. Same shape (`new(neighbours)`,
  `predictor_for(block)`, `commit_block(block, final_mv)`,
  `neighbours()`, `finalise() -> [Mv; 4]`, `Default` =
  `NeighbourSet::ABSENT`, `new` is `const fn`) but the predictor
  calls route through [`mv_pred::resolve_block_candidates`] so a
  4-MV-coded neighbour's bordering 8x8 cell is picked **per
  current-MB block** per ISO/IEC 14496-2:2004(E) §7.6.5 /
  Figure 7-34 (`docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`).
  Compared with r196's [`Macroblock4MvDecoder`] (which collapses
  neighbours to one `Option<Mv>` per direction via
  [`mv_pred::MacroblockCandidates`] and is correct only when every
  neighbour is 1-MV-coded), the new decoder picks the right
  bordering MV per current block — e.g. when a 4-MV left
  neighbour has distinct MVs at its TR cell (block 2, borders
  current block 1) and its BR cell (block 4, borders current
  block 3), the new decoder reads each correctly, whereas the
  old surface could only pick one. Both shapes co-exist;
  1-MV-only neighbours can keep using the original decoder. Ten
  new lib tests in `src/mv_pred.rs::tests` pin: the absent-neighbour
  predictor chain (block 1 = (0, 0) by rule 4; blocks 2/3/4 pick
  up earlier-committed within-MB MVs by rules 2/3); equivalence
  with [`Macroblock4MvDecoder`] when every neighbour is `OneMv`
  (same per-block predictors, same finals); the same equivalence
  when every neighbour is `Absent`; the distinct-cell divergence
  with a 4-MV left neighbour (block 1 picks left's block 2 cell,
  block 3 picks left's block 4 cell); `neighbours()` accessor
  round-trip; `Default::default()` resolves to
  `NeighbourSet::ABSENT`; the const-fn property of `new`; the
  out-of-order block-4-then-block-1 commit independence (block 4's
  MV is never read as a within-MB candidate so committing it early
  doesn't affect block 1's predictor); `finalise`'s
  `Mv::default()` substitution for uncommitted blocks; and the
  surface equivalence with
  [`mv_pred::predict_macroblock_4mv_with_4mv_neighbours`] for the
  block-1 entry. Test suite 321 → 331 (+10) lib tests; integration
  + doc tests unchanged. Purely additive; the existing 1-MV
  `picture::decode_pframe_mb` call site is unchanged. Source —
  `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`
  §"What the figure shows" + four per-block sub-diagrams, plus
  `docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`
  §7.6.5 (the four substitution rules + median + worked example),
  same source set as r214's `resolve_block_candidates`. No new
  tables, no new decode logic; the additive surface is wholly the
  stateful wrapper around r214's primitive.

- **Round 214 — 4-MV neighbour-state resolver (1-MV vs 4-MV per
  neighbour)** (2026-06-03): three additive public APIs in
  [`mv_pred`] that close the gap between r208's per-pair
  bordering-cell picker and a real-stream 4-MV picture decoder.
  [`mv_pred::NeighbourMvKind`] tags each neighbouring MB's MV mode
  for the current frame: `Absent` (outside picture / video packet /
  GOB → §7.6.5 rule-1 invalid), `OneMv(Mv)` (1-MV-coded), or
  `FourMv([Mv; 4])` (4-MV-coded with one MV per Figure-6-8 block).
  [`mv_pred::NeighbourSet`] bundles the three directions (`left`,
  `above`, `above_right`) into a single per-current-MB context with
  a `NeighbourSet::ABSENT` const for the picture-corner case and a
  `NeighbourSet::candidate_for(current, direction) -> Option<Mv>`
  method that routes `OneMv` straight through and routes `FourMv`
  via [`mv_pred::bordering_block_of_neighbour`] to pick the
  bordering cell. [`mv_pred::resolve_block_candidates(current,
  neighbours, within_mb) -> BlockCandidates`] composes
  `NeighbourSet::candidate_for` over the three directions with the
  within-MB-block threading [`mv_pred::BlockCandidates`] expects, so
  callers can build a per-current-MB-block predictor input in one
  call regardless of how each neighbour was coded. The higher-level
  [`mv_pred::predict_macroblock_4mv_with_4mv_neighbours(neighbours,
  finals) -> [Mv; 4]`] drives the predict loop over
  [`mv_pred::Block::ALL`] internally; compared to r196's
  [`mv_pred::predict_macroblock_4mv_with_finals`] (which collapses
  neighbours to a single `MacroblockCandidates` MV per direction
  and is therefore correct only when every neighbour is 1-MV-coded),
  the new batch picks the bordering cell per current-MB block —
  pinning the divergence on the `current=BottomLeft` case where
  the left neighbour's block 4 (BR cell) is the candidate but
  `current=TopLeft` picks the left neighbour's block 2 (TR cell).
  When every neighbour is `OneMv` or `Absent` the two batch APIs
  produce identical output (one of the ten new lib tests pins this).
  Twelve new lib tests in `src/mv_pred.rs::tests` cover: the
  `Absent` rule-1 short-circuit on all 12 `(current, direction)`
  pairs; `OneMv`'s 6-Some / 6-None symmetry against the
  bordering-cell table; `FourMv`'s indexing on every direction; the
  concrete Figure 7-34 TL/Left worked example (left=FourMv →
  block 2 of left neighbour); `resolve_block_candidates` threading
  neighbour and within-MB fields; the documented equivalence with
  `predict_macroblock_4mv_with_finals` when all neighbours are
  `OneMv`; the corner-case equivalence when all neighbours are
  `Absent`; the per-block divergence with a distinct-cells `FourMv`
  left neighbour; the `NeighbourMvKind::is_absent` predicate; and
  the const-fn property of `NeighbourSet::ABSENT` plus
  `resolve_block_candidates`. Test suite 309 → 321 (+12) lib tests.
  Purely additive; the existing 1-MV
  `picture::decode_pframe_mb` call site is unchanged. The future
  picture-decoder rewrite that wires the MS-MPEG-4 v3 MCBPC
  1-MV-vs-4-MV bit will build its current-MB predictors by calling
  `resolve_block_candidates` or the batch wrapper once per MB,
  transparently handling any mix of 1-MV and 4-MV neighbours through
  the same surface. Source citations:
  `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`
  §"What the figure shows" + the four per-block sub-diagrams;
  `docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`
  §7.6.5 (substitution rules + bordering-cell-of-4MV-neighbour
  prose); `docs/video/msmpeg4/spec/06-mv-decoder.md` §3 (v3 inherits
  §7.6.5 unchanged).

- **Round 208 — 4-MV neighbour-MB bordering-cell picker** (2026-06-02):
  Two new public const-fn APIs in [`mv_pred`] that close the
  "the caller is responsible for picking the right cell from the
  neighbouring MB" doc comment in [`mv_pred::MacroblockCandidates`].
  [`mv_pred::bordering_block_of_neighbour(current, direction) ->
  Option<Block>`] resolves, per ISO/IEC 14496-2:2004(E) §7.6.5 /
  Figure 7-34, which 8x8 sub-block of a **4-MV-coded** neighbouring
  macroblock sits adjacent to the current-MB block being predicted.
  [`mv_pred::pick_neighbour_mv_from_4mv(current, direction, &[Mv; 4])
  -> Option<Mv>`] composes that lookup with an index into the
  neighbour's `[Mv; 4]` raster-order MV array. A new
  [`mv_pred::NeighbourDirection`] enum (variants `Left` / `Above` /
  `AboveRight`, plus `NeighbourDirection::ALL`) names the three
  neighbour-MB directions referenced by Figure 7-34. The
  `(current-block, direction)` → bordering-block table has exactly six
  Some entries (block 1 / TopLeft takes all three directions; block 2
  / TopRight takes above + above-right; block 3 / BottomLeft takes
  left only; block 4 / BottomRight takes nothing — its BR sub-diagram
  is all-within-MB) and six None entries (where the within-MB
  candidate slot of [`mv_pred::BlockCandidates`] is used instead).
  Bordering positions are derived from the four sub-diagrams of
  `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`,
  cross-checked against `docs/video/mpeg4-visual/figure-7-34-render.png`
  (PDF page 302): block 1's MV1 (left) sits in the right-column /
  top-row 8x8 of the left-neighbour MB = block 2 (TopRight) of the
  left-neighbour; block 1's MV2 (above) and MV3 (above-right) sit in
  the bottom-row / left-col 8x8 of those neighbours = block 3
  (BottomLeft); block 2's MV2 (above) sits in the bottom-row /
  right-col 8x8 = block 4 (BottomRight) of the above-neighbour;
  block 3's MV1 (left) sits in the right-col / bottom-row 8x8 of the
  left-neighbour = block 4 (BottomRight). Ten new lib tests in
  `src/mv_pred.rs::tests` pin every per-current-block sub-diagram
  (`bordering_block_for_{top_left,top_right,bottom_left,bottom_right}_…`),
  the documented six-Some / six-None split
  (`bordering_block_count_matches_documented_table`), the const-fn
  property (`bordering_block_is_const_evaluable`), the `[Mv; 4]`
  indexing behaviour for the six Some-returning pairs
  (`pick_neighbour_mv_uses_bordering_block_as_index`), the six
  None-returning pairs (`pick_neighbour_mv_returns_none_when_no_bordering_block`),
  the `NeighbourDirection::ALL` enumeration, and the
  predictor-composition for the block-1 case
  (`pick_neighbour_mv_composes_with_predict_block_mv_for_block_1`).
  Test suite 299 → 309 (+10) lib tests. Purely additive; no
  rewiring of `picture::decode_pframe_mb` or any existing call site;
  the helper is the API a future 4-MV-mode picture decoder will
  consult when populating [`mv_pred::BlockCandidates`] /
  [`mv_pred::MacroblockCandidates`] from 4-MV-coded neighbours.

- **Round 202 — 4-MV-per-MB bitstream integration tests** (2026-06-01):
  New `tests/macroblock_4mv_bitstream.rs` exercises
  [`mv_pred::Macroblock4MvDecoder`] end-to-end through the full
  predict → joint-VLC-decode → commit loop for a single 16x16
  macroblock, streaming real codes from the v3 default joint-MV VLC
  (`MV_V3_RAW`, source at VMA `0x1c25cbc0`). Four cases pin the
  decoder against a real bitstream rather than synthesised `Mv`
  values: the picture-corner where block 1 hits §7.6.5 rule 4 and
  blocks 2 / 3 / 4 chain through within-MB candidates via rules 2 /
  3; the all-neighbours case where every block exercises its
  distinct Figure 7-34 layout without firing a substitution rule;
  a "rigid-motion" zero-MVD case where four copies of the
  `(MVDx_raw, MVDy_raw) = (32, 32)` joint code against a non-zero
  `left_mb` predictor collapse all four blocks to the same MV; and
  a parallel-reader cross-check proving
  `Macroblock4MvDecoder::{predictor_for, commit_block}` is a
  faithful sequencer of manual
  [`mv_pred::predict_block_mv`] calls (every per-block predictor
  and decoded MV agree; bit-consumption matches). The shared `pack`
  helper + `canonical_code_for` builder are local to the integration
  file so the new tests do not depend on private rebuilds of the
  canonical-Huffman walker. Fully additive; the 1-MV
  `picture::decode_pframe_mb` path is unchanged. Integration test
  count grows by 4 (76 → 80 cross-file tests across the
  `tests/` harness).

- **Round 196 — 4-MV-per-MB batch predictor surface** (2026-06-01):
  Two new public APIs in [`mv_pred`] that thread the per-block
  within-MB candidate cells per Figure 7-34 of ISO/IEC
  14496-2:2004(E) across all four 8x8 luminance blocks in one
  call. [`mv_pred::predict_macroblock_4mv_with_finals`] takes a
  [`mv_pred::MacroblockCandidates`] (the three neighbour-MB MVs)
  plus the already-decoded block-1/2/3 MVs and returns `[Mv; 4]`,
  computed by calling [`mv_pred::predict_block_mv`] once per
  [`mv_pred::Block::ALL`]. [`mv_pred::Macroblock4MvDecoder`] is
  the closed-form helper for the predict-MVD-decode-reconstruct
  loop a future `picture::decode_pframe_mb` 4-MV path will drive:
  alternate `predictor_for(block)` (read the §7.6.5 spec
  predictor given whatever has been committed so far) and
  `commit_block(block, final_mv)` (record the post-MVD-add MV so
  later blocks see it as the corresponding within-MB candidate).
  Per `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`
  the four block sub-diagrams pin: block 1 takes left + above +
  above-right neighbour MBs; block 2 takes block 1 + above +
  above-right; block 3 takes left + block 1 + block 2; block 4
  takes block 3 + block 1 + block 2 (all within-MB). Eight new
  lib tests in `src/mv_pred.rs::tests` pin every structural fact
  (batch-block-0 equivalence with direct TopLeft call, all-four-
  blocks match per-block predicts, closed-form decoder agrees
  with batch fn, picture-corner rule-4-then-rule-3 chain on block
  1 → block 2, block 4 ignores neighbour MBs, decoder accepts
  out-of-order commits without re-threading). Test suite 291 →
  299 (+8) lib tests. Purely additive; the 1-MV call site in
  `picture::decode_pframe_mb` is unchanged — wiring the 4-MV path
  into the picture decoder waits on the MS-MPEG-4 v3 MCBPC bit
  pattern signalling 1-MV vs 4-MV mode (open spec gap; see
  README "Still lacks" tail).

## [0.0.7](https://github.com/OxideAV/oxideav-msmpeg4/compare/v0.0.6...v0.0.7) - 2026-05-30

### Other

- round 191: route P-frame 1-MV predictor through mv_pred (Figure 7-34 top-left)
- round 185: MV-predictor candidate-neighbour walk per MPEG-4 Visual Figure 7-34
- round 181: GFamily selector inverses + subclass_of partition classifier
- round 174: unified GFamily dispatch surface over G0..G5
- round 129: v1/v2 → v3-compat selector defaults pinned as public API
- round 126: 3-tier ESC body integration tests + stale README row fix
- round 123: inter (P-frame) AC residual decode wired
- round 81: spec/15 §3 per-G (count_A, count_B) source-of-truth pin
- round 75: v1/v2 shared CBPY VLC binary cross-check (region_053640)
- round 7: G0..G3 LMAX/RMAX wired + synthetic-VLC pipeline

### Changed

- **Round 191 — P-frame 1-MV predictor routed through `mv_pred`
  (Figure 7-34 top-left case)** (2026-05-30):
  [`picture::decode_pframe_mb`] now obtains its motion-vector
  predictor from [`mv_pred::predict_block_mv`] with
  [`mv_pred::Block::TopLeft`] (driven by a
  [`mv_pred::BlockCandidates`] populated from the per-row `left` /
  `top` / `top_right` neighbour MVs the existing call site already
  computes), rather than the historical pre-spec
  [`mv::median_predictor`] shortcut. Per `docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`
  §"When only one motion vector is present for the whole macroblock,
  the top-left case in Figure 7-34 is applied", and per ISO/IEC
  14496-2:2004(E) §7.6.5 prose ("In the case of only one motion
  vector present for the complete macroblock, the top-left case in
  Figure 7-34 is applied."), `Block::TopLeft` is the spec-mandated
  layout for the 1-MV-per-MB case which is the only mode the v3
  picture decoder currently supports.

  Behavioural delta (relative to the pre-r191 shortcut): the rule-3
  corner of §7.6.5 ("If two and only two candidate predictors are not
  valid, they are set to the third candidate predictor.") now fires
  whenever exactly **one** of the three neighbours is valid. The
  predictor becomes that lone valid neighbour itself rather than the
  median of `(neighbour, 0, 0)` which the shortcut returned as zero.
  In practice this affects two corner cases:

  - The first inter MB in a row whose row-0 neighbours are all
    intra-coded and whose left neighbour is the only valid one
    (`left` Some, `top` and `top_right` both `None`). Pre-r191
    predictor = `(0, 0)`; r191 predictor = `left.mv`.
  - Symmetric cases where `top` or `top_right` is the sole valid
    neighbour.

  All other neighbour configurations (zero valid, two valid, three
  valid) keep their pre-r191 behaviour bit-for-bit — the
  [`mv_pred::tests::top_left_matches_existing_median_predictor_when_two_or_more_valid`]
  test in r185 already pinned that equivalence over a 5×5×5×4 sample
  space. The lone divergence is the rule-3 "exactly one valid" case
  pinned by [`mv_pred::tests::top_left_diverges_from_old_shortcut_in_rule_3_case`].

  This routing is what r185's CHANGELOG entry queued as "for the next
  round once the 4-MV-per-MB MCBPC variant lands". Re-reading
  Figure 7-34's caption and §7.6.5's prose this round shows that the
  1-MV case is independent of the 4-MV decode path: the spec applies
  the top-left sub-diagram unconditionally to the 1-MV mode (and to
  every short-video-header stream), so wiring 1-MV through
  `mv_pred::predict_block_mv(Block::TopLeft, …)` does not need the
  4-MV MCBPC variant. When 4-MV lands in a future round the same
  call site grows a per-block loop over `Block::ALL`, each
  invocation reading the same `mv_pred::predict_block_mv` API.

  [`mv::median_predictor`] is retained (still consumed by the v1/v2
  MV test surface at `tests/v1_v2_mv.rs::median_predictor_chains_with_decode`
  and the in-module mv tests) and deprecated only in documentation;
  no API removal.

  No new tests this round — the behavioural delta is already pinned
  in `mv_pred::tests` (15 spec-7.6.5 + 2 divergence + boundary +
  shortcut-equivalence tests, all green). Test suite unchanged at
  291 lib + 113 integration tests.

### Added

- **Round 185 — MV-predictor candidate-neighbour walk (Figure 7-34)**
  (2026-05-29): new module [`mv_pred`] wires the per-block candidate
  layout for the 4-MV-per-MB median-of-3 motion-vector predictor as
  defined by Figure 7-34 of ISO/IEC 14496-2:2004(E) §7.6.5 (MPEG-4
  Visual). Pulls truth from the in-tree clean-room ASCII transcription
  [`docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md`] plus
  the §7.6.5 prose in
  [`docs/video/mpeg4-visual/ISO_IEC_14496-2-2004-3rd-edition.txt`].
  New public surface:
  - [`mv_pred::Block`] enum (`TopLeft` / `TopRight` / `BottomLeft` /
    `BottomRight`) with `Block::ALL` const-array iteration order
    matching Figure 6-8 raster (1, 2, 3, 4) and `Block::spec_index()`
    for the spec block number.
  - [`mv_pred::BlockCandidates`] — `Option<Mv>` cache covering the six
    cells Figure 7-34 references across the four sub-diagrams: the
    three neighbouring-MB MVs (`left_mb` / `above_mb` /
    `above_right_mb`) and the three already-decoded within-MB block
    MVs (`mb_block_1` / `mb_block_2` / `mb_block_3`). Default is all-
    `None` (picture corner / all neighbours out-of-area).
  - [`mv_pred::gather_candidates`] — pure block→`[Option<Mv>; 3]`
    lookup driven by the four sub-diagrams of Figure 7-34. Each block
    routes to its spec-mandated `(MV1, MV2, MV3)` cells:
    * Block 1 (TL): left/above/above-right neighbour MBs (same as the
      1-MV-per-MB case which Figure 7-34 explicitly designates as
      using the top-left sub-diagram).
    * Block 2 (TR): within-MB block 1, above neighbour MB, above-right
      neighbour MB.
    * Block 3 (BL): left neighbour MB, within-MB block 1, within-MB
      block 2.
    * Block 4 (BR): within-MB blocks 3, 1, 2 (all three from the
      current MB).
  - [`mv_pred::apply_validity_rules`] — implements the four candidate-
    validity substitution rules of §7.6.5 verbatim from the
    transcription:
    1. `None` means "not valid" (transparent / outside VOP / outside
       video packet / outside GOB).
    2. Exactly one `None` → set to zero.
    3. Exactly two `None` → both = the third valid candidate.
    4. All three `None` → all zero.
  - [`mv_pred::predict_block_mv`] — full pipeline: gather candidates
    per block, apply validity rules, then per-component median-of-3.
    The `Block::TopLeft` invocation with only-neighbour-MB candidates
    is the 1-MV-per-MB single-MV-of-the-macroblock case explicitly
    designated by §7.6.5 ("In the case of only one motion vector
    present for the complete macroblock, the top-left case in Figure
    7-34 is applied.").

  Spec compliance vs. the older [`mv::median_predictor`]: 17 unit
  tests pin the new behaviour, including a documented divergence in
  the "exactly one neighbour valid" rule-3 corner. The old shortcut
  zero-substitutes both missing neighbours then takes a median →
  `median(only, 0, 0) = 0`; the spec-correct §7.6.5 rule 3 promotes
  the lone valid candidate to all three slots → `median(only, only,
  only) = only`. The new module returns the spec-correct result. The
  shortcut still ships in [`mv::median_predictor`] and the picture
  decoder still consumes it; routing `picture::decode_pframe_mb`
  through [`mv_pred::predict_block_mv`] (which flips the corner-cell
  result) is queued for the next round once the 4-MV-per-MB MCBPC
  variant lands. The spec §7.6.5 worked example
  (`MV1=(-2,3), MV2=(1,5), MV3=(-1,7) → (Px,Py)=(-1,5)`) is pinned in
  [`mv_pred::tests::spec_7_6_5_worked_example_predictor_matches`].

  17 new tests in `src/mv_pred.rs::tests`: per-block gather
  (`top_left_pulls_from_neighbour_mbs`,
  `top_right_pulls_block_1_then_neighbour_above_and_above_right`,
  `bottom_left_pulls_left_neighbour_then_within_mb_blocks_1_and_2`,
  `bottom_right_pulls_within_mb_blocks_3_then_1_then_2`), each
  validity rule (`rule_2_exactly_one_invalid_substitutes_zero`,
  `rule_3_exactly_two_invalid_both_become_third`,
  `rule_4_all_invalid_all_zero`, `all_valid_passes_through_unchanged`),
  the §7.6.5 worked example, the picture-corner zero predictor across
  all four blocks, the rule-3 left-only short-row case, the
  `Block::TopLeft` ↔ `mv::median_predictor` equivalence wherever ≥ 2
  neighbours are valid, the documented rule-3 divergence, a
  `Block::BottomRight` rule-3 instance, `Block::ALL` raster order,
  `spec_index` uniqueness, and median order-independence.

- **Round 181 — `GFamily` selector inverses + `subclass_of` partition
  classifier** (2026-05-29): extends the round-174
  [`g_family::GFamily`] surface with three new const-fn accessors that
  close the structural API. (1) [`GFamily::subclass_of(idx)`] returns
  `Option<GSubclass>` classifying every alphabet index into sub-A
  (`[0, count_B]`, `last=0` per spec/13 §2 — non-terminating, kernel
  continues scanning) or sub-B (`(count_B, count_A)`, `last=1` per
  spec/13 §2 — clean exit), with `idx == count_A` (ESC sentinel) and
  any `idx > count_A` returning `None`. Per `docs/video/msmpeg4/spec/
  13-kernel-block-termination.md` §2 the kernel's partition test at
  `1c216e2a..1c216e2f` (`inc eax; mov [ebp-0x8], eax`) sets the
  sub-class flag purely from this `idx > count_B` predicate;
  `subclass_of` is the table-free structural form of that test. The
  new [`g_family::GSubclass`] enum (variants `A`/`B`) names the two
  partition classes. (2) [`GFamily::chroma_selector()`] is the inverse
  of [`for_chroma_selector`], returning the picture-header
  `ac_chroma_sel` value ∈ {0,1,2} that dispatches to this G-family per
  spec/14 §3.1 (`G2→0, G0→1, G4→2`) or `None` for the three intra-luma
  descriptors (G1/G3/G5). (3) [`GFamily::luma_selector()`] is the
  inverse of [`for_luma_selector`], returning the picture-header
  `ac_luma_sel` value per spec/14 §3.1 (`G3→0, G1→1, G5→2`) or `None`
  for the three chroma+all-inter descriptors (G0/G2/G4). Six new unit
  tests in `src/g_family.rs::tests`:
  `subclass_of_classifies_every_alphabet_index_per_spec_13` (every
  in-range idx classifies; ESC and OOR return `None`),
  `subclass_partition_sizes_match_subclass_of_counts` (iterating
  `subclass_of` over `0..count_A` reproduces the spec/15 §5.2
  subclass_a_size / subclass_b_size totals),
  `subclass_of_agrees_with_decode_last_flag` (structural partition
  matches every non-ESC token's `last` flag from the underlying
  `g_descriptor` / `g_enum` decode dispatch),
  `chroma_selector_inverts_for_chroma_selector` and
  `luma_selector_inverts_for_luma_selector` (both bijections
  round-trip with `for_chroma_selector` / `for_luma_selector`), and
  `selector_inverses_are_role_exclusive` (`chroma_selector().is_some()
  XOR luma_selector().is_some()` holds for every G-family, since each
  fills exactly one role per spec/14 §3.1). Test suite grows from
  **268 → 274** lib tests (+6). Purely additive API; no rewiring of
  existing dispatch sites, no new tables, no new decode logic — every
  accessor derives from the spec/13 §2 partition equation and the
  spec/14 §3.1 bijection that already underlay the round-174 surface.

- **Round 174 — unified `GFamily` dispatch surface over G0..G5** (2026-05-29):
  closes the long-standing asymmetry between the two parallel post-VLC
  `(idx → (last, run, |level|))` surfaces — [`g_descriptor::g4_decode`] /
  `g5_decode` (round 18/19) for the H.263-baseline pair, and
  [`g_enum::GExtended`] (round 29) for the four MS-MPEG4-specific G0..G3
  extended descriptors. Both surfaces evolved separately as the extraction
  work landed and did not share a common Rust type; a caller that wanted
  to iterate "every G-descriptor" had to special-case the G4/G5 split. The
  new [`g_family::GFamily`] enum (6 variants `G0..G5`, `#[repr(u8)]` with
  discriminants matching `G_COUNTS_SPEC15` / `G_SUBCLASS_SIZES_SPEC15`
  indices) unifies all six behind a single dispatch surface with `const fn`
  accessors for `count_a()` / `count_b()` / `subclass_a_size()` /
  `subclass_b_size()` (per spec/15 §3 / §5.2) and `descriptor_base_offset()`
  (per spec/15 §2.1's literal-immediate `mb_mv_struct_init` constructor
  evidence: G0=`0x9d8` through G5=`0x9d8 + 5*0x24 = 0xa8c`, cluster
  ending at `+0xab0`). A `role()` const fn classifies each as either
  `GRole::ChromaAndInter` (G0/G2/G4) or `GRole::IntraLuma` (G1/G3/G5) per
  spec/14 §3.1. Two const dispatch fns `for_chroma_selector(sel)` /
  `for_luma_selector(sel)` resolve picture-header selector values ∈ {0,1,2}
  to the matching G-family (`0 → G2, 1 → G0, 2 → G4` for chroma;
  `0 → G3, 1 → G1, 2 → G5` for luma per spec/14 §3.1). The `v1/v2`
  fallthrough (spec/14 §3.1) writes `[esi+0xab0] = G4` and
  `[esi+0xab4] = G5` unconditionally — captured as the `sel == 2` cases
  of the two const dispatch fns, so a v1/v2 dispatcher can write
  `for_chroma_selector(2)` / `for_luma_selector(2)` without a
  version-specific branch. `decode(idx)` and `iter()` instance methods
  delegate to the existing `g_descriptor` / `g_enum` plumbing — this
  module adds no new tables and no new decode logic. 15 new unit tests
  in `src/g_family.rs::tests` pin every structural fact (counts,
  partition sizes, base offsets / `0x24` stride, role distribution,
  selector dispatch arms, partition invariant `idx > count_b ⇔
  last=true` from spec/13 §2) across all six G-families. Test suite
  grows from **253 → 268** lib tests (+15). Purely additive API; no
  behavioural change, no rewiring of existing dispatch sites.

- **Round 129 — v1 / v2 → v3-compat selector defaults pinned as public API**
  (2026-05-25): closes the docs-gap noted in r126 about "v1/v2 picture-level
  decode parameters" by surfacing the spec/01 §1.4 + spec/07 §1.4 / §1.6 /
  §2.4 facts as two associated constants on
  [`header::MsV1V2PictureHeader`]: `V1_COMPAT_DEFAULTS` and
  `V2_COMPAT_DEFAULTS` (each a `V1V2V3CompatDefaults` carrying
  `dc_size_sel`, `ac_chroma_sel`, `ac_luma_sel`, `mv_table_sel`,
  `has_ac_pred_anywhere`, and `has_spatial_dc_predictor`). The v1 and v2
  paths never read the v3-only per-frame selector bits (the reads gate on
  `version == 3` at VMA `1c211fdd` / `1c21205a..1c2120aa` per spec/01 §1.4),
  so downstream code sharing a v3 decode entry point must use
  `dc_size_sel = 0` (primary intra-DC VLC pair) and the G4 + G5 default
  clusters for chroma + luma. v1 additionally lacks both the MB-level
  AC-prediction bit (spec/07 §1.4 — the v1 MCBPCY body `0x1c2171c7` "does
  NOT read a post-VLC sign / AC-pred bit (no `call 0x1c215c9b` after the
  CBPY decode)") and the patent-7,054,494 spatial DC predictor (§1.6 — the
  `0x1c23a788 / 0x1c23a7b0` LUTs are "absent from v1"); v2 adds AC
  prediction only at intra-in-P macroblocks (§2.4 — a "v2 innovation"
  gated on `mcbpc / 4 == 1`). A const-block at module scope pins every
  default at **compile time** (any silent drift fails the build) and a
  block of 8 new runtime `#[test]`s in `src/header.rs::tests` exercises
  the same invariants via a `black_box_defaults` helper that defeats
  const-folding so the lints see real field reads. The new tests are
  `v1_compat_defaults_carry_v3_zero_initialisation_at_runtime`,
  `v2_compat_defaults_carry_v3_zero_initialisation_at_runtime`,
  `v1_has_no_ac_prediction_anywhere_at_runtime`,
  `v2_has_ac_prediction_only_at_intra_in_p_macroblocks_at_runtime`,
  `v1_v2_lack_spatial_dc_predictor_at_runtime`,
  `v1_v2_compat_defaults_are_distinct_values` (rejects a future copy-paste
  that flattens the two consts), `v1_pframe_with_umv_clear_parses` (covers
  the previously-untested `umv = 0` round-trip), and
  `v1_iframe_does_not_read_umv_bit` (canary-bit assertion that the I-frame
  parser doesn't consume a 38th bit). Test suite 338 → 346 (+8). No
  runtime behavioural change; the consts are new **public** API that
  downstream consumers (oxideav-avi tag dispatch, oxideav-mkv codec
  resolver) can read to spell out the "v1/v2 share v3 decode paths but
  with these defaults" contract.

- **Round 126 — 3-tier ESC body integration tests + stale-row fix**
  (2026-05-25): adds `tests/intra_block_3tier_esc.rs` (8 tests) that
  exercise the v1/v2/v3 intra block decoder kernel `0x1c216d97`
  3-tier ESC body end-to-end through the public
  [`mb::decode_intra_block_full_v3`] boundary — i.e. the real
  intra-DC direct-value 120-entry VLC consumes the leading bits, the
  G5 primary canonical-Huffman VLC consumes the AC walk, the 3-tier
  ESC body (level extension via `LMAX[last][run]`, run extension via
  `RMAX[last][|level|] + 1`, verbatim `1 + 6 + 8`-bit FLC) handles
  each escape tier, and `dequantise_h263` runs on the assembled
  coefficient array. Cases covered: (1) DC differential + sub-class-B
  terminator, (2) zero-DC fast path (no sign bit consumed per spec/07
  §5.2), (3) tier-1 level extension, (4) tier-2 run extension,
  (5) tier-3 verbatim FLC triple, (6) CBP-zero short-circuit (verifies
  the AC walk is skipped and zero AC bits are consumed),
  (7) chroma DC-scaler routing (block_idx ≥ 4 must use
  `C_DC_SCALE_TABLE` per `iq::dc_scaler`), and (8) the DC ESC tier
  in the 120-entry intra-DC VLC (idx == 119 ⇒ 8-bit raw + sign per
  `spec/07-remaining-opens.md` §5.2). These integration tests catch
  regressions that only manifest at the
  `(DC VLC) + (AC walk) + (3-tier ESC body) + (dequant)` boundary —
  e.g. a bit-reader desync that the per-token `decode_token` unit
  tests in `src/ac.rs` cannot see because they call into a synthetic
  ESC byte stream rather than chaining through
  `decode_intra_dc_diff_v3` first. Test suite 330 → 338 (+8). No
  runtime behavioural change; no new spec tables consulted (spec/04
  §2.3 + spec/07 §5.2 + spec/11 §3-§5 + audit/01 §4.1 cite the
  already-staged docs that drove the round-7 / round-27 work).
  Also fixes a stale README status row that still claimed the
  MS-MPEG4v3 3-tier ESC body was "OPEN — MPEG-4 fallback only" —
  rounds 7 and 27 wired the full tier 1 / 2 / 3 walk in
  `ac::decode_escape_body`; the row now reflects the implemented
  state and links the new integration tests.

- **Round 123 — inter (P-frame) AC residual decode wired**
  (2026-05-25): the P-frame inter-MB path now decodes and applies the
  per-block AC residual instead of laying down a pure motion-compensated
  copy. New `ac::decode_inter_ac` / `ac::decode_inter_block` mirror the
  inter kernel `0x1c215d2c` per
  `docs/video/msmpeg4/spec/04-decoder-kernels.md` §1: the running scan
  position starts at **0** (DC is a coded coefficient on the inter path,
  not separately predicted), the scan is the **fixed zigzag** (§1.6 —
  inter blocks never consult the alt-horizontal / alt-vertical tables),
  the ESC body is a **single verbatim tier** (§1.3 step 10 — the G4
  inter table carries `lmax`/`rmax = None` so `decode_escape_body`
  collapses to the `1 + 6 + 8`-bit FLC triple), and termination is the
  sub-class-B `last` flag (§1.3 step 9). `decode_pframe_mb` now decodes
  the G4 inter VLC ([`AcVlcTable::g4_inter`], whose packed-Huffman
  primary VLC is fully extracted) for every CBP-coded block (luma CBPY
  bits MSB-first, chroma cbp_cb / cbp_cr), dequantises with
  `dequantise_h263(.., level_start = 0)` per spec/08 §3.2, IDCTs to a
  signed residual, and **adds** it onto the MC prediction (new
  `add_residual_to_picture`, clamped to `[0, 255]` per spec/04 §2.6
  "inter IDCT output is a signed residual added to the prediction").
  Eight new tests: seven `ac::tests` covering the inter walker
  (leading-run-0 writes the DC slot, fixed-zigzag run advance, negative
  sign, single-tier verbatim ESC, scan-overflow hard error, inter-block
  dequant with `level_start = 0`, and a real-G4-table terminator
  round-trip) plus one `picture::tests::handcrafted_inter_mb_applies_residual`
  integration test (a coded inter MB over a flat-grey reference: the
  residual modifies luma block 0 while every uncoded block stays the MC
  copy). Test suite 322 → 330 (+8). No new spec tables required —
  spec/04 §1 + the already-extracted G4 packed-Huffman source were
  sufficient.

- **Round 81 — spec/15 §3 per-G `(count_A, count_B)` source-of-truth pin**
  (2026-05-21): introduce a single authoritative `G_COUNTS_SPEC15:
  [(u32, u32); 6]` table in `build.rs` (top-of-file), emitted into
  `tables_data::G_COUNTS_SPEC15` plus a derived
  `G_SUBCLASS_SIZES_SPEC15` of `(sub_A, sub_B)` pairs. The six values
  `(168,98) (185,118) (148,80) (132,84) (102,57) (102,66)` are lifted
  verbatim from the `mb_mv_struct_init` constructor disassembly at
  VMA `0x1c210643` per `docs/video/msmpeg4/spec/15-count-ab-per-g-family.md`
  §2.1, and have been independently cross-checked across eight prior
  spec documents (02/03/04/05/08/09/13/14) per spec/15 §7
  ("All six G-families CONSISTENT"). The existing scattered literals
  in `build.rs` — four `emit_g_enum` call-sites for G0..G3 and the
  G4/G5 hardcoded constants inside `emit_g_descriptor_cluster` — are
  now pulled from `G_COUNTS_SPEC15` with build-time `panic!` cross-
  checks if they ever drift. The build-side `emit_g_counts_spec15`
  additionally verifies the partition arithmetic `sub_A = count_B+1`
  / `sub_B = count_A-count_B-1` reproduces the spec/03 §4.4 reference
  values exactly. Eight new `tables_data::tests` pin: alphabet length
  is six; each tuple matches its constructor disassembly; sub-class
  sizes match spec/03 §4.4; runtime partition arithmetic matches the
  build-time emit; G4/G5 cross-check against the legacy
  `G{4,5}_COUNT_A`/`_COUNT_B` constants; G0..G3 cross-check via the
  `g_enum::GExtended::count_a()` / `count_b()` dispatch surface;
  `sub_A + sub_B = count_A` (the ESC sentinel sits outside both
  sub-classes); and G4/G5 share `count_A = 102` but differ on
  `count_B`. Test count delta: 229 → 237 lib (+8). Total suite:
  314 → 322 tests passing. This collapses six scattered (count_A,
  count_B) literal-pair sites to a single source-of-truth with
  build-time and runtime cross-checks, so future per-G count drift
  surfaces immediately rather than silently diverging across the
  build script, the `g_descriptor` module, and the `g_enum` module.
  No runtime behavioural change.

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
