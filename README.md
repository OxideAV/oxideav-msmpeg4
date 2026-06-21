# oxideav-msmpeg4

Pure-Rust decoder for the **Microsoft MPEG-4** family — v1, v2, and v3
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

| Piece                                          | Status     |
| ---------------------------------------------- | ---------- |
| Bitstream classifier (`classify`)              | complete   |
| V3 picture-header parser (I / P)               | complete   |
| Scan tables (zigzag + alternate H/V)           | complete   |
| IDCT (float reference)                         | complete   |
| Quantiser dequantisation + DC scalers          | complete   |
| CBPY + DC-size VLCs                             | complete   |
| Intra MB header + DC differential decode       | complete   |
| Joint MCBPCY VLC (v3, 128-entry canonical)     | complete   |
| DC spatial predictor + AC scan dispatcher      | complete   |
| Intra MB pipeline (DC pred + IDCT + store)     | complete   |
| G0..G5 canonical-Huffman primary AC VLC        | complete   |
| MS-MPEG4v3 intra 3-tier ESC body               | complete (LMAX/RMAX ground-truth-validated for all 6 G-families) |
| Inter AC residual (G4 VLC → IDCT → add to MC)  | complete   |
| P-frame MV VLC + half-pel MC (default + alt)   | complete (decodes against extracted wire codes, spec/16 §1) |
| P-frame 1-MV predictor (Figure 7-34)           | complete   |
| 4-MV-per-MB predictor surface + neighbour resolver | wired (per-block bordering-cell pick) |
| V3 intra-luma I-frame end-to-end via `decode_picture` | complete |
| V1 / V2 P-frame pixel pipeline (incl. INTER+Q + INTER4V) | complete |
| V1 P-frame MB-type table (`MB_TYPE_V1_INFO`)   | complete (binary-extracted, spec/16 §3) |
| V1 / V2 intra pipeline (I-frame + intra-in-P)  | complete (size+value DC, spec/16 §2) |
| V1 / V2 intra DC-size category VLCs (luma/chroma) | complete (binary-extracted, spec/16 §2) |
| V1 / V2 shared CBPY VLC                         | complete   |
| Picture-wide MV grid (`MvGrid`)                | complete   |
| Per-G-family descriptor / runtime-binding accessors | complete |

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
- **V3 4-MV-per-MB picture decode**: the predictor / neighbour-resolver
  surface is complete and is now exercised end-to-end on the v1 P-frame
  INTER4V path (`spec/16` §3.1). Wiring it into the **v3** picture
  decoder is a docs gap: the v3 joint 128-entry MCBPCY alphabet
  (`region_05eac8`, `audit/02` §4) encodes only an intra/inter split
  (64 I-type + 64 P-type CBPCY patterns = 2 MB-types × 64 CBPCY), so it
  carries **no** INTER4V code, and the v3 per-MB driver's 1-MV vs 4-MV
  decision (`spec/06` §1.3 "for each MV needed by the decoded MB-type")
  is not traced to a concrete signal. The v3 MV decoder itself already
  supports the 4-MV output layout (`spec/06` §3.6 "first of four (or
  one)") — only the trigger is missing. Until the docs resolve where v3
  signals INTER4V, the v3 picture decoder stays 1-MV-per-MB.
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
