# oxideav-msmpeg4

Pure-Rust decoder **and encoder** for the **Microsoft MPEG-4** family —
v1, v2, and v3
(a.k.a. DivX ;-) 3). These were Microsoft's pre-standard MPEG-4 codecs
shipped in Windows Media Tools (1999-2001) and forked by DivXNetworks
into the original "DivX" ripper codec. They are **not** the same
bitstream as standard MPEG-4 Part 2 (ISO/IEC 14496-2) — despite the
name, the headers, VLC tables, and slice structure all differ.

If you have a file whose FourCC is one of `DIV3`, `DIV4`, `DIV5`,
`DIV6`, `MP41`, `MP42`, `MP43`, `MPG3`, or `AP41`, you want this crate.
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
I/P GOP machine with `quant` / `gop` / `mv_search_range` options)
decodes its own bytes each frame so encoder/decoder prediction state
cannot drift; direct factories live at `encoder::make_encoder`
(+ `_v1` / `_v2`). Caveat: for the two tables whose wire codes are
canonically *reconstructed* rather than binary-extracted (the v3
128-entry joint MCBPCY, see the docs gap below, and the shared v1/v2
CBPY table) the encoder matches this crate's decoder by construction,
and byte-exactness against the original binary remains exactly as
unverified as on the decode side.

| Piece                                          | Status     |
| ---------------------------------------------- | ---------- |
| Bitstream classifier (`classify`)              | complete   |
| V3 picture-header parser (I / P)               | complete   |
| Scan tables (zigzag + alternate H/V)           | complete   |
| IDCT (float reference)                         | complete   |
| Quantiser dequantisation + DC scalers          | complete   |
| CBPY + DC-size VLCs                             | complete   |
| Intra MB header + DC differential decode       | complete   |
| Joint MCBPCY VLC (v3, 128-entry canonical)     | complete (intra/inter partition corrected to patent polarity 0..63=intra, round 362) |
| DC spatial predictor + AC scan dispatcher      | complete   |
| Intra MB pipeline (DC pred + IDCT + store)     | complete   |
| G0..G5 canonical-Huffman primary AC VLC        | complete   |
| MS-MPEG4v3 intra 3-tier ESC body               | complete (LMAX/RMAX ground-truth-validated for all 6 G-families) |
| Inter AC residual (G4 VLC → IDCT → add to MC)  | complete   |
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

- **V3 joint-MCBPCY wire codes (docs gap)**: round 356 corrected the v3
  joint-MV VLC to decode against its actual extracted wire codes (spec/16
  §1 / spec/12 §2 — the per-slot walker builder is fed literal `(code, bl)`
  records, and the codes are NOT a textbook-canonical assignment). The
  128-entry joint-MCBPCY table (`region_05eac8`) is still built via
  canonical reconstruction, and its CSV `code` column is the *old*
  mis-decoded extraction (32 of 128 entries have `code ≥ 2^bit_length`,
  i.e. impossible as wire codes). To apply the same correctness fix to
  MCBPCY, the docs need an Extractor-07-style re-extraction of
  `region_05eac8` carrying the real MSB-first wire codes (a
  `region_05eac8_mcbpcy.csv` with the `symbol_index,code_dec,code_bin,
  bit_length` layout, Kraft 1.0). Until then MCBPCY stays canonical and
  whether that matches the binary is unverified.
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
