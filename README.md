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
on the pinned Microsoft fixtures; round 459 applied spec/19 (the
decoder's IDCT, the intra prediction context and the slice law) and
closed the P-frame frontier of the third-party DIV3/DIV4 fixtures.

**Round 459 scorecard** (`tests/microsoft_fixtures.rs`, black-box
reference decode with its `int` IDCT selection — see the caveat below):

| Fixture | Frames | Frame 0 (I) Y / U / V exact | Aggregate Y / U / V | max \|Δ\| |
| --- | --- | --- | --- | --- |
| `mp43.wmv` (Microsoft, 400×250, 2 slices) | 49/50 (frame 49 truncated in the container) | 99.72 % / 99.88 % / 99.83 % | 96.7 % / 98.6 % / 98.6 % | 1 (I), ≤ 5 by frame 40 |
| `div3.avi` (third-party, 352×240) | 50/50 | 99.42 % / 99.56 % / 99.80 % | 96.9 % / 96.8 % / 99.0 % | 1 (I), 2 (every P) |
| `div4.avi` (third-party, 352×240) | 50/50 | 99.42 % / 99.84 % / 99.83 % | 97.3 % / 99.2 % / 99.5 % | 1 (I), 2 (every P) |

Before round 459: mp43 98.5 % / 99.6 % / 99.5 % with frame 0 at 99.71 %
and max |Δ| 17; div3 / div4 decoded 47/50 frames at 12 % / 16 % Y
(P-frames desynchronised at the first intra-in-P macroblock).

Every remaining difference is ±1 (I-frames) growing to ±2..5 through a
GOP: the black-box reference's `int` IDCT is not the vendor arithmetic
either (its default kernel rounds a DC-only 636 to 79 where spec/19 §1
gives 80, and scored 53 % against the exact kernel), and the ±1
residue it leaves on ~0.5 % of intra samples accumulates through
motion compensation. The crate implements spec/19 §1.4's MMX lane
model, pixel-exact on the hardware-grade saturation probe
(`tables/idct-mmx-saturation-probe.csv`, 128/128 samples).

| Piece                                          | Status     |
| ---------------------------------------------- | ---------- |
| Bitstream classifier (`classify`)              | complete   |
| V3 picture-header parser (I / P)               | complete   |
| Scan tables (zigzag + alternate H/V)           | complete   |
| IDCT (spec/19 §1 exact integer kernel: MMX int16/23-bit lane model, scalar reference form) | complete (round 459; hardware-grade saturation probe 128/128) |
| Quantiser dequantisation + DC scalers          | complete   |
| CBPY + DC-size VLCs                             | complete   |
| Intra MB header + DC differential decode       | complete   |
| Joint MCBPCY VLC (v3 **P-frames**, 128-entry, extracted wire codes) | complete (round 405 wire codes, Kraft=1.0; round 420 pinned the table as P-frame-only per its staged `tables-ff` companion role) |
| Intra CBPCY VLC (v3 **I-frames**, 64-entry, XOR-predicted luma bits) | complete (round 420: re-aligned `region_05eed0` ≡ staged `msmp4-mb-i-table`, Kraft=1.0; patent 7,054,494 CBPCY-XOR rule pinned on both DIV3 fixtures) |
| DC/AC prediction context (level domain, spec/19 §2 default record, slice restart) | complete (round 459) |
| Intra MB pipeline (DC pred + IDCT + store)     | complete   |
| G0..G5 canonical-Huffman primary AC VLC        | complete   |
| MS-MPEG4v3 intra/inter TCOEF escape ladder     | complete (spec/17 §3 selector-1/selector-2 dispatch; LMAX/RMAX for all 6 G-families; run-extension arm `run = run_lut[s] + RMAX` pinned on the spec/17 G3 probes and on two `mp43.wmv` I-frame blocks; the inter kernel's copy adds 1 — fixture-selected, see docs asks) |
| Inter AC residual (`ac_chroma_sel`-bound G2/G0/G4 VLC → IDCT → add to MC) | complete (round 452: per-frame descriptor binding + intra-kernel escape ladder on v2/v3 inter blocks) |
| P-frame MV VLC + half-pel MC (default + alt)   | complete (decodes against extracted wire codes, spec/16 §1; alt-table byte-LUT selection picture-level-pinned, round 362; round 459: H.263-style chroma-MV rounding + per-P-frame half-pel rounding alternation, both fixture-arbitrated — see docs asks) |
| P-frame 1-MV predictor (Figure 7-34)           | complete (picture-level median-propagation pin, round 359) |
| 4-MV-per-MB predictor surface + neighbour resolver | complete (per-block bordering-cell pick; INTER4V→1-MV-neighbour propagation picture-level-pinned, round 366) |
| V3 intra-luma I-frame end-to-end via `decode_picture` | complete |
| Intra-in-P MB (v1/v2/v3) picture-level pixel path | complete (round 459: DIV3/DIV4 intra-in-P MBs decode — luma AC table = luma class of the P-frame selector, default-record predictors for inter neighbours per spec/18 §7, zero-MV grid cell) |
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

- **V3 real-content decode (round 459 frontier)**: the three Microsoft
  fixtures decode end-to-end (scorecard above). What remains is
  docs-side — rules the fixtures arbitrated that no staged trace
  covers, plus one reference-side limitation:
  1. **Reference planes.** The harness reference is a black-box decode
     whose IDCT is not the vendor kernel; `spec/19` §4 lists
     hardware-grade vendor planes (`provenance/sandbox-04/outputs/`)
     that sit outside the Implementer wall. **Docs ask**: stage those
     three `.planes.yuv` files (+ their input streams) under
     `tables/` or a wall-legal fixtures directory so the crate can
     assert 100 % sample-exactness against the vendor decoder instead
     of a ±1 proxy.
  2. **Inter-kernel run-extension arm.** spec/17 §3 pins the intra
     kernel's `run = run_lut[s] + run_ext[last][level]`; the inter
     kernel (`0x1c215e6f`, its own copy of the ladder at
     `1c216021`/`1c216030`, spec/08 §1) needs `+ 1` on every one of
     the 46 run-extension escapes of the `mp43.wmv` P-frames. **Docs
     ask**: trace `0x1c215fdb..0x1c216040` — is there an `inc` /
     `+1` after the `run_ext` load, and is the `last` threshold the
     same `desc[+8] + 1`?
  3. **Intra-in-P luma table binding.** spec/99 §2.3 says `[esi+0xad4]`
     persists from the I-frame into P-frames; both DIV3/DIV4 fixtures
     decode their intra-in-P luma blocks only through the luma class
     of the **P-frame's** selector (`{G3, G1, G5}[[0xad0]]`, G1 here)
     and desynchronise under the persisted G5. **Docs ask**: trace
     the P-frame descriptor copy at `1c2138d1..1c2138f3` — which
     selector indexes the live intra-luma slot `[0xab4]` on
     P-frames?
  4. **Chroma MV rounding.** The crate uses the H.263 §6.1.1
     quarter-to-half rule (`|v| = 4k+1 → 2k+1`); the binary's
     byte-indexed chroma-MV LUT at `0x1c23a7d8` (spec/99 §3.3) is not
     staged. **Docs ask**: extract the LUT into `tables/`.
  5. **Half-pel rounding alternation.** spec/04 §3.1 / spec/99 §4.6
     read the MC kernels as a fixed `(a + b + 1) >> 1`; the DIV3/DIV4
     fixtures need the MPEG-4-style toggle (`+1` on the first P-frame
     after an I-frame, `+0` on the next, alternating; reset by every
     I-frame) or every half-pel macroblock of the second P-frame is
     off by one. **Docs ask**: trace the rounding constant of
     `0x1c22f01c` / `0x1c22d7db` (and the second MC vtable
     `ds:0x1c2ae500`, spec/05 §2.2) for per-frame state.
  6. **MV predictor with an intra neighbour.** spec/06 §3.4 loads
     every neighbour from the MV store; the crate now stores a zero
     MV for intra-in-P macroblocks (the §7.6.5 promotion of the
     remaining neighbour desynchronised both fixtures). **Docs ask**:
     confirm the MB loop writes `(0, 0)` to the MV store for intra
     macroblocks.
  7. **`iframe_ext` read frequency.** spec/19 §3 says the field is
     read once per decoder instance; every I-frame of the three
     fixtures carries it (the DIV3 clips' second I-frame at frame 36
     included) and the crate reads it on every I-frame. **Docs ask**:
     is the `[esi+0xb2c]` guard reset per keyframe by the ICM wrapper?
- **V1 / V2 header fields (spec/99 §2.4, §0.1 row 28)**: the crate's
  v1/v2 picture header does not yet read the first-I-frame 5-bit
  `iframe_ext` (v1: rows per slice = value; v2: the v3 law) nor the
  v2 P-frame skip-enable bit, and still reads a v1 P-frame UMV bit
  that row 28 refutes. Encoder and decoder are symmetric today and no
  v1/v2 fixture exists; scheduled as a follow-up.
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
