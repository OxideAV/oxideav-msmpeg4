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
production-ready; v1/v2 P-frames decode to pixels. v1/v2 I-frames and a
few inter sub-types remain gated on open spec items (see below).

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
| P-frame MV VLC + half-pel MC (default + alt)   | complete   |
| P-frame 1-MV predictor (Figure 7-34)           | complete   |
| 4-MV-per-MB predictor surface + neighbour resolver | wired (per-block bordering-cell pick) |
| V3 intra-luma I-frame end-to-end via `decode_picture` | complete |
| V1 / V2 P-frame pixel pipeline                 | complete   |
| V1 / V2 shared CBPY VLC                         | complete   |
| Picture-wide MV grid (`MvGrid`)                | complete   |
| Per-G-family descriptor / runtime-binding accessors | complete |

## What's still open for real-content decode

- **4-MV-per-MB picture decode**: the predictor / neighbour-resolver
  surface is complete, but wiring it into the picture decoder waits on
  the v3 MCBPC bit pattern that signals 1-MV vs 4-MV mode (an open spec
  gap).
- **V1 / V2 I-frames and intra-in-P MBs**: gated behind a documented
  `Unsupported` error, now narrowed to a **single** trace target. The
  intra block path is otherwise fully shared with v3 and already wired:
  the AC run/level/last kernel `0x1c216d97` is the common v1/v2/v3 intra
  kernel, the DC-predictor gradient routine `0x1c20aef0` carries no
  version gate, and the intra-DC-size VLC descriptor binding (all four
  luma/chroma × selector tables) is enumerated in the staged spec. The
  one remaining unknown is the **construction-time default of the
  intra-DC-size selector `[esi+0x8bc]`** — that bit is read from the
  bitstream only on `version == 3`, so on v1/v2 the slot keeps its
  constructor value, which no staged chapter traces. (The CBP
  spatial-prediction LUT pair `0x1c23a788 / 0x1c23a7b0` that v1 omits is
  the *CBP* predictor, not a DC-prediction LUT, so its absence does not
  block intra DC decode.)
- **V1 non-zero inter sub-types** (`mb_type ∈ {1, 2, 4, 5}`): the
  per-sub-type side reads are not yet traced.

## License

MIT.
