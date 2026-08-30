# oxideav-msmpeg4

[![CI](https://github.com/OxideAV/oxideav-msmpeg4/actions/workflows/ci.yml/badge.svg)](https://github.com/OxideAV/oxideav-msmpeg4/actions/workflows/ci.yml) [![crates.io](https://img.shields.io/crates/v/oxideav-msmpeg4.svg)](https://crates.io/crates/oxideav-msmpeg4) [![docs.rs](https://docs.rs/oxideav-msmpeg4/badge.svg)](https://docs.rs/oxideav-msmpeg4) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Pure-Rust decoder **and encoder** for the **Microsoft MPEG-4** family —
v1, v2, and v3
(a.k.a. DivX ;-) 3). These were Microsoft's pre-standard MPEG-4 codecs
shipped in Windows Media Tools (1999-2001) and forked by DivXNetworks
into the original "DivX" ripper codec. They are **not** the same
bitstream as standard MPEG-4 Part 2 (ISO/IEC 14496-2) — despite the
name, the headers, VLC tables, and slice structure all differ.

If you have a file whose FourCC is one of `DIV3`, `DIV4`, `DIV5`,
`DIV6`, `MP41`, `MP42`, `MP43`, `MPG3`, `DVX3`, `AP41`, or `COL1`, you
want this crate.
If you have `XVID`, `DIVX` (note the missing 3), `DX50`, `MP4V`, or
`FMP4` you want [`oxideav-mpeg4video`](https://github.com/OxideAV/oxideav-mpeg4video)
instead.

Because the two codec families are constantly mislabelled in the wild
(encoders stamped DIV3 on actual MPEG-4 Part 2 streams and vice versa),
this crate exposes [`classify`] — a bitstream sniffer that reports which
codec is actually present regardless of the container's FourCC. Use it
from a container implementation (AVI / MKV) to dispatch to the right
decoder when a packet arrives.

## Status

**In progress.** `classify` and the v3 intra/P-frame pixel pipeline are
production-ready; v1/v2 P-frames decode to pixels, including the INTER+Q
and INTER4V (4-MV) inter sub-types. As of round 339 the **v1/v2 I-frame
and intra-in-P intra pixel pipeline** decodes end-to-end too — spec/16 §2
established that v1/v2 use a dedicated H.263 size+value DC scheme
(`region_054{2,3}c0`), not the v3 `dc_size_sel` selector, dissolving the
previous gate. Real-content bit-exactness against an encoder oracle is a
pending Auditor item.

As of round 383 the crate also carries a **full-family encoder**
(v1 / v2 / v3, I-frames + motion-searched P-frames): every syntax
element is written through the bit-level inverse of its decode surface
over the same extracted tables, and every produced stream round-trips
through this crate's own production decoders (`picture::decode_picture`
/ `decode_picture_v1v2`). The registered `oxideav_core::Encoder` (an
I/P GOP machine with `quant` / `gop` / `mv_search_range` /
`scene_cut` / `bitrate` options) decodes its own bytes each frame so
encoder/decoder prediction state cannot drift; direct factories live
at `encoder::make_encoder` (+ `_v1` / `_v2`).

Round 386 drove encoder **quality**: rate-aware predictor-centred
motion search (`SAD + q·mv_bits` over the union of the zero- and
predictor-centred half-pel windows — a ±2 window still tracks a
3-half-pel/frame pan at −67 % bytes / +1.1 dB), RD-decided `ac_pred`
(v3 + v2) and the 18-way v3 I-frame table-selector RD (both exact-rate:
the probe is the real serialiser), per-MB intra-in-P scene-change
refuge on all three versions (+10 dB at a hard cut, q=2), a
census-driven scene-cut P→I GOP policy, and frame-level bit-budget
rate control (virtual buffer + bounded re-encode trials; requested
400/150 kbit/s → achieved 422/158 on the 30-frame
`examples/rate_curve.rs` sequence). Whole-curve: −2.9 %…−6.9 % bytes
at equal-or-better PSNR (v3, q ∈ {2,4,8,16,31}).

As of round 405 every VLC both directions consume is binary-extracted
wire codes — the last canonical reconstruction (the v3 128-entry joint
MCBPCY) was retired when the `region_05eac8_mcbpcy` re-extraction
landed (Kraft = 1.0; the old canonical assignment matched the real
codes for 0 of 128 symbols).

Round 452 applied the spec/17 + spec/18 docs stagings (intra MB layer +
escape ladder; inter MB header) and re-arbitrated every open semantic
on the pinned Microsoft fixtures: the TCOEF escape ladder's two
selector bits and signed fixed-length arm, the per-I-frame 5-bit header
field and the predictor slices it sizes, intra AC **coefficient**
prediction, the coded-bit luma-CBP spatial predictor, quantised-domain
DC prediction, half-down IDCT rounding, the P-frame `mb_skip_enable`
header bit, the intra-only `ac_pred` element of the joint MB header,
the selector-bound inter AC descriptor (with the same escape ladder on
v2/v3 inter blocks), and `ac_luma_sel` persistence into P-frames. The
definitive Microsoft stream (`mp43.wmv`, WMFSDK 7) now decodes
end-to-end: every I-frame parses 400/400 MBs, frames 44..48 are fully
pixel-exact, and the other P-frames hold ~84% pixel-exact MBs (the
residual is float-vs-integer IDCT rounding, not parse); the DIV3 AVI
fixtures' I-frames parse 330/330 at ~99% exact luma (see "What's still
open" for their P-frame drift).

| Piece                                          | Status     |
| ---------------------------------------------- | ---------- |
| Bitstream classifier (`classify`)              | complete   |
| V3 picture-header parser (I / P)               | complete   |
| Scan tables (zigzag + alternate H/V)           | complete   |
| IDCT (float reference)                         | complete   |
| Quantiser dequantisation + DC scalers          | complete   |
| CBPY + DC-size VLCs                             | complete   |
| Intra MB header + DC differential decode       | complete   |
| Joint MCBPCY VLC (v3 **P-frames**, 128-entry, extracted wire codes) | complete (round 405 wire codes, Kraft=1.0; round 420 pinned the table as P-frame-only per its staged `tables-ff` companion role) |
| Intra CBPCY VLC (v3 **I-frames**, 64-entry, XOR-predicted luma bits) | complete (round 420: re-aligned `region_05eed0` ≡ staged `msmp4-mb-i-table`, Kraft=1.0; patent 7,054,494 CBPCY-XOR rule pinned on both DIV3 fixtures) |
| DC spatial predictor + AC scan dispatcher      | complete   |
| Intra MB pipeline (DC pred + IDCT + store)     | complete   |
| G0..G5 canonical-Huffman primary AC VLC        | complete   |
| MS-MPEG4v3 intra/inter TCOEF escape ladder     | complete (spec/17 §3 selector-1/selector-2 dispatch; LMAX/RMAX for all 6 G-families; run-extension arm is an inference — unobserved on Microsoft streams) |
| Inter AC residual (`ac_chroma_sel`-bound G2/G0/G4 VLC → IDCT → add to MC) | complete (round 452: per-frame descriptor binding + intra-kernel escape ladder on v2/v3 inter blocks) |
| P-frame MV VLC + half-pel MC (default + alt)   | complete (decodes against extracted wire codes, spec/16 §1; alt-table byte-LUT selection picture-level-pinned, round 362) |
| P-frame 1-MV predictor (Figure 7-34)           | complete (picture-level median-propagation pin, round 359) |
| 4-MV-per-MB predictor surface + neighbour resolver | complete (per-block bordering-cell pick; INTER4V→1-MV-neighbour propagation picture-level-pinned, round 366) |
| V3 intra-luma I-frame end-to-end via `decode_picture` | complete |
| Intra-in-P MB (v1/v2/v3) picture-level pixel path | complete (round 374: v3 + v1/v2 intra-in-P pinned end-to-end through `decode_picture`; ac_pred scan-flip on a CBP-coded block, v1 zigzag-only, intra-in-P → `Absent` predictor cell) |
| V1 / V2 P-frame pixel pipeline (incl. INTER+Q + INTER4V) | complete (INTER4V luma + §7.6.3.4 chroma + per-MB-neighbour propagation picture-level-pinned, round 366) |
| V1 P-frame MB-type table (`MB_TYPE_V1_INFO`)   | complete (binary-extracted, spec/16 §3) |
| V1 / V2 intra pipeline (I-frame + intra-in-P)  | complete (size+value DC, spec/16 §2) |
| V1 / V2 intra DC-size category VLCs (luma/chroma) | complete (binary-extracted, spec/16 §2) |
| V1 / V2 shared CBPY VLC                         | complete   |
| Picture-wide MV grid (`MvGrid`)                | complete   |
| Per-G-family descriptor / runtime-binding accessors | complete |
| v3 encoder: I-frame + P-frame (skip / half-pel motion search / G4 residual) | complete (round 383; decoder-verified round-trip, all 31 quants) |
| v1 / v2 encoder: I-frame + P-frame              | complete (round 383; size+value DC, MCBPC/CBPY wrap, per-component MV) |
| Registered `Encoder` + `encoder::make_encoder{,_v1,_v2}` | complete (round 383; GOP machine, decode-own-bytes reference) |

## What's still open for real-content decode

- **V3 real-content decode (round 452 frontier)**: spec/17 + spec/18
  closed the round-420 asks — the escape-body selector routing
  (selector-1 `1` = level extension; `0`,`0` = the signed verbatim FLC
  arm), the raw chroma CBP bits, and the inter MB header's
  intra-only `ac_pred` element are all pinned by the docs and
  validated on the fixtures. The mid-frame drift is resolved: it was
  the compound of the mis-shaped escape ladder, missing intra AC
  coefficient prediction, the DC-gradient CBP predictor, pel-domain DC
  prediction, and (on MP43) the un-modelled predictor slices.
  Remaining opens, in Microsoft-fixture priority order:
  1. **Exact IDCT**: the crate's float IDCT with half-down rounding
     leaves scattered ±1-pel diffs against the reference decode
     (~60/400 MBs per busy MP43 frame; they accumulate slowly across a
     GOP but re-zero at each I-frame). **Docs ask**: transcribe the
     DLL's non-MMX integer IDCT (`1c20d426..1c20e4be` + constant
     tables `0x1c2610f0` / `0x1c261138` / `0x1c261360`) so the kernel
     can be integer-exact.
  2. **DIV3/DIV4 P-frame drift**: both third-party-encoded AVI
     fixtures decode structurally (skip-disable header bit `0` → no
     skip prefixes, alternate MV table, G0 inter) but drift from the
     first **intra-in-P** macroblock; the reference reconstructs
     those MBs with DC values our neutral-predictor intra-in-P path
     does not reach. No Microsoft-produced fixture exercises
     intra-in-P at all (mp43.wmv codes zero intra MBs in 49
     P-frames), so the Microsoft-ground-truth rule can't arbitrate.
     **Docs ask**: trace the v3 intra-in-P DC/AC prediction context —
     what the DLL uses as the DC predictor when the causal neighbours
     are inter MBs (neutral 1024, or a value derived from the
     reconstructed neighbour pels), and whether the CBP/AC prediction
     state survives inter MBs.
  3. **`iframe_ext` semantics**: the 5-bit per-I-frame field (23 on
     both 352x240 fixtures and every spec/17-traced 176x144 frame; 24
     on the 400x250 MP43 stream) is modelled as `slices = value − 22`
     with DC/AC prediction restarting per slice — it fits all three
     fixtures but is an inference from two observed values. **Docs
     ask**: trace the `1c21224b` consumer.
  4. **Selector-2 = 1 escape arm**: unobserved in 7472 traced escapes
     (spec/17 §3); decoded as the run-extension re-VLC per the
     kernel-layout inference. Any stream that actually exercises it
     would firm this up.
- **V3 4-MV-per-MB picture decode (hard docs gap #1895)**: the
  predictor / neighbour-resolver surface is complete and is exercised
  end-to-end on the v1 P-frame INTER4V path (`spec/16` §3.1, the real
  traced 4-MV path = MCBPC MB-type 2). Wiring 4-MV into the **v3**
  picture decoder is blocked: the v3 joint 128-entry MCBPCY alphabet
  (`region_05eac8`, `audit/02` §4, patent 6,563,953 Table 1) encodes
  only an intra/inter split (64 I-type + 64 P-type CBPCY patterns = 2
  MB-types × 64 CBPCY), so it carries **no** INTER4V code, and every
  traced part of the v3 per-MB driver (`1c2131ff` → MCBPCY `1c21782f` →
  MV decoder `1c217f5a`, spec/05 §3 / spec/06 §1) invokes the MV decoder
  exactly **once** per inter MB — there is no second VLC, no per-MB 4-MV
  flag, and no other signal between MCBPCY and the MV decode. The v3 MV
  decoder body itself supports the 4-MV output layout (`spec/06` §3.6
  "first of four (or one)") — only the **trigger** is missing. As of
  round 366 this 1-MV-per-MB invariant is a first-class property
  (`McbpcyDecode::num_motion_vectors()` returns 0/intra, 1/inter, never
  4) consulted by the v3 driver with a hard-error guard, so the v3 path
  stays 1-MV-per-MB until the docs resolve where (or whether) v3 signals
  INTER4V. **Docs ask:** trace the v3 P-frame MB layer for any signal
  selecting a 4-MV mode (a second MCBPC-extension VLC, an OBMC/advanced
  flag, or a per-MB bit) — or confirm authoritatively that MS-MPEG-4 v3
  has no INTER4V mode (in which case #1895 closes as "v3 is 1-MV by
  design").
- **V1 / V2 I-frames and intra-in-P MBs**: now decode to pixels (round
  339). spec/16 §2 (Extractor 07) established that the v1/v2 intra-block
  driver gates on version (`cmp [esi+8], 3`): for v < 3 it decodes the DC
  differential through the classic H.263 §5.4.1 / MPEG-4 Part 2 §7.4.3
  size+value scheme (`sub_15790`) using the binary's own luma/chroma
  size-category tables (`region_0542c0` / `region_0543c0`, VMAs
  `0x1c2542c0` / `0x1c2543c0`) — **not** the v3 direct-value DC VLC and
  **not** the v3 `[esi+0x8bc]` `dc_size_sel` selector. The previous gate
  cited that selector's untraced construction-time default; since v1/v2
  never consult it, the gate is dissolved. The spatial DC-predictor
  gradient `0x1c20aef0` and intra AC kernel `0x1c216d97` are shared with
  v3 (no version gate); v1/v2 default luma AC = G5, chroma = G4 (spec/14
  §3.2). Real-content bit-exactness against an encoder oracle (the AC
  walk + spatial-predictor reconstruction matching the binary) is a
  pending Auditor-round validation.
- **V1 inter sub-types**: wired and table-grounded. `spec/16` §3.1 +
  `region_053140_mbtype.csv` pin the P-frame MB-type → MV-count map
  {1, 1, 4, 0, 0}; round 352 loads that 21-symbol map into the
  build-time `MB_TYPE_V1_INFO` table (cross-checked for the `>> 2`
  decomposition, the MV-count map, and intra classification) and drives
  `decode_mcbpcy_v1`'s `is_intra` / `num_motion_vectors` plus the
  INTER4V dispatch from it. MB-type 0 (INTER) and 1 (INTER+Q) are 1-MV
  (the v1 MCBPCY body reads no quantiser-delta bit per spec/07 §1.4);
  MB-type 2 (INTER4V) loops the per-component MV decoder 4× over the
  Figure 6-8 8x8 blocks with the chroma MV derived per §7.6.3.4;
  MB-types 3 (INTRA) **and 4 (INTRA+Q)** are intra. The earlier
  `is_intra = mb_type == 3` shortcut mis-classified MB-type 4 as inter
  (rejecting v1 I-frame MBs that carried it); the table closes that.
  The STUFFING/ESC symbol (20) is rejected explicitly.

## License

MIT.
