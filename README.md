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
| Inter AC run/level VLC                         | pending — spec OPEN   |
| V1 / V2 bitstream                              | header + MV + MCBPC done; AC OPEN |

### What's still spec-OPEN for real-content decode

Despite the spec-doc consolidation in `docs/video/msmpeg4/spec/99`,
the **real intra-AC and inter-AC VLC code-length tables** for v3
(per the G5 / G4 descriptors at VMAs `0x1c258430` / `0x1c258230`)
have not been extracted into `tables/` in a form an Implementer can
consume. What's extracted is:

* `region_05eed0.csv` — a 64-entry canonical-Huffman block at VMA
  `0x1c25fad0`. **Wired as the v3 intra-AC candidate** in
  [`AcVlcTable::v3_intra_candidate`], but its role is OPEN per
  `spec/99 §0.1 row 8` / `§9 OPEN-O6`. Critically, **the alphabet
  size mismatches G5**: G5 has `count_A = 102, count_B = 66`
  (spec/99 §5), but `region_05eed0` declares `count_A = 64,
  count_B = 1`. The candidate is therefore structurally **not** the
  v3 intra-AC primary VLC source — but it is a complete prefix code
  in its own right and is retained as Implementer plumbing for
  pipeline tests.
* No corresponding extraction exists for G4 (inter-AC); the G4 / G5
  Huffman code-length data lives in the packed-Huffman input source
  regions `0x1c259a38` / `0x1c259d78` (file `0x58e38` / `0x59178`),
  which were dumped as 256-byte over-dumps under the wrong role
  label ("scan order candidate") per spec/99 §8.3. The constructor
  algorithm at VMA `0x1c210ee6` that turns these packed inputs into
  the runtime-walker descriptor has not been disassembled — without
  that algorithm or a verified canonical-Huffman code-length list,
  the Implementer cannot wire the v3 inter-AC or the spec-correct
  v3 intra-AC.

Net effect: real `.avi` v3 fixtures decode the picture / MB /
MCBPCY / DC layers but produce DC-only intra reconstruction (PSNR
≈ 5–16 dB on `testsrc2 32×32`) and zero-residual inter MBs. The
decoder reports the gap with a `Result::Err` citing
`docs/video/msmpeg4/spec/99-current-understanding.md` §9 OPEN-O4
when the candidate AC plumbing is engaged on bitstream that diverges
from its hypothesis.

`send_packet` on a v3 stream defaults to the AC placeholder
(DC-only reconstruction); callers that want to exercise the
candidate AC pipeline use [`picture::decode_picture_with_ac`] with
[`picture::AcSelection::Candidate`]. Synthetic streams + Kraft +
prefix-free + per-symbol round-trip + scan-order dispatch tests
all pass against the candidate (see `tests/intra_ac_candidate.rs`).

## License

MIT.
