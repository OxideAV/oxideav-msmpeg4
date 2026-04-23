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

| Piece                                          | Status   |
| ---------------------------------------------- | -------- |
| Bitstream classifier (`classify`)              | complete |
| V3 picture-header parser (I / P)               | complete |
| Scan tables (zigzag + alternate H/V)           | complete |
| IDCT (float reference)                         | complete |
| H.263-style dequantisation + DC scalers        | complete |
| CBPY + DC-size VLCs                            | complete |
| Intra MB header + DC differential decode       | complete |
| Intra AC run/level VLCs                        | pending  |
| Intra MB pipeline (DC/AC pred + IDCT + store)  | pending  |
| P-frame MV VLCs + motion compensation          | pending  |
| V1 / V2 bitstream                              | pending  |

Calling `send_packet` on a v3 stream currently parses the picture
header and the first MB's header + DC differential, then returns an
[`Error::Unsupported`] diagnostic pointing at the AC decode boundary.

## License

MIT.
