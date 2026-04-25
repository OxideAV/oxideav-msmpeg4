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
this crate exposes [`classify`] — a bitstream sniffer that tells you
which codec is actually present regardless of the container's FourCC.
Use it from a container implementation (AVI / MKV) to dispatch to the
right decoder when a packet arrives.

## Status

**In progress.** [`classify`] is production-ready.

| Piece                                          | Status                |
| ---------------------------------------------- | --------------------- |
| Bitstream classifier (`classify`)              | complete              |
| V3 picture-header parser (I / P)               | complete              |
| Scan tables (zigzag + alternate H/V)           | complete              |
| IDCT (float reference)                         | complete              |
| H.263-style dequantisation + DC scalers        | complete              |
| CBPY + DC-size VLCs                            | complete              |
| Intra MB header + DC differential decode       | complete              |
| Joint MCBPCY VLC (v3, 128-entry canonical)     | complete              |
| DC spatial predictor + AC scan dispatcher      | complete              |
| Intra MB pipeline (DC pred + IDCT + store)     | complete              |
| Intra AC run/level VLC plumbing                | wired (candidate)     |
| Intra AC `(last, run, level)` symbol mapping   | OPEN — hypothesis     |
| P-frame MV VLC + half-pel MC (default table)   | complete              |
| P-frame MV VLC alternate table                 | unsupported (truncated dump) |
| Inter AC run/level VLC                         | pending               |
| V1 / V2 bitstream                              | pending               |

The intra-AC primary VLC was wired this round from a clean-room
extraction of `region_05eed0.csv` (VMA `0x1c25fad0`, file offset
`0x5eed0`). The 64-entry canonical-Huffman code-length array is
verified at build time (Kraft sum = 1) and exposed via
[`AcVlcTable::v3_intra_candidate`]. The role attribution of the
underlying region is **OPEN** per
`docs/video/msmpeg4/spec/99-current-understanding.md` §0.1 row 8 and
§9 OPEN-O6 (candidate intra-AC TCOEF source per `spec/03` §5.3,
alternative v2 MCBPCY source). The candidate's `(last, run, level)`
mapping is the Implementer's hypothesis (see the doc-comment on
`v3_intra_candidate` for the exact partition rule); a future spec
audit may revise it.

`send_packet` on a v3 stream defaults to the AC placeholder
(DC-only reconstruction); callers that want to exercise the
candidate AC pipeline use [`picture::decode_picture_with_ac`] with
[`picture::AcSelection::Candidate`]. Synthetic streams + Kraft +
prefix-free + per-symbol round-trip + scan-order dispatch tests
all pass against the candidate (see `tests/intra_ac_candidate.rs`).

## License

MIT.
