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
| G4 (inter) `pri_A` + `pri_B` byte arrays       | wired (round 18)      |
| G5 (intra-luma) `pri_A`                        | wired (round 18)      |
| G5 (intra-luma) `pri_B`                        | wired (round 19)      |
| G4 / G5 canonical-Huffman bit-lengths          | OPEN — walker tree at `0x3df40` |
| P-frame MV VLC + half-pel MC (default table)   | complete              |
| P-frame MV VLC alternate table                 | unsupported (truncated dump) |
| Inter AC run/level VLC                         | pending — spec OPEN   |
| V1 / V2 bitstream                              | header + MV + MCBPC done; AC OPEN |

### What's still spec-OPEN for real-content decode

Round 18 (2026-04-26) wired the **G4 / G5 `pri_A` / `pri_B` byte
arrays** from the cluster region `region_0569c0` (file
`0x569c0..0x57898`, 3800 bytes). **Round 19 (2026-04-30)** completes
the pri_B set by extracting the 408-byte gap that immediately follows
the cluster region into `tables/region_057898.hex` (102 × u32-LE at
file `0x57898..0x57a30`, VMA `0x1c258498`):

* **G4** (chroma + all-inter; default for v1/v2 streams; v3 selector
  `[esi+0xad0]=2`): `count_A=102, count_B=57`. `pri_A` (102 bytes
  at file `0x57630`) and `pri_B` (102 × u32-LE at file `0x57698`)
  are both extracted and wired into [`g_descriptor::g4_decode`],
  the post-VLC `(idx → (last, run, |level|))` symbol mapper.
  Per-row content cross-checked against `audit/01 §2.2` (sub-A
  58 entries, sub-B 44 entries; max run 40, max level at run=0
  is 12 — matches MPEG-4 Part 2 Table 11-19 ESCL(b) LMAX exactly).
* **G5** (intra-luma; v3 selector `[esi+0xad4]=2`):
  `count_A=102, count_B=66`. `pri_A` (102 bytes at file `0x57830`)
  was wired in r18; `pri_B` (102 × u32-LE at file `0x57898`) is now
  wired in r19 from the dedicated `region_057898.hex` extraction.
  [`g_descriptor::g5_decode`] now returns `Some(Token)` for the
  full alphabet (idx 0..=101) plus ESC at idx 102 — sub-class A
  (67 entries, last=0) and sub-class B (35 entries, last=1) both
  decode through a single byte-array lookup. Run / level layouts
  cross-checked against `audit/01 §4.1` to the entry: sub-B has
  r0×8, r1×3, r2..6×2, r7..20×1 with max run 20.

What's **still missing for runnable AC decode**:

* The **canonical-Huffman bit-length array** for G4 / G5 — the
  prefix-code shape that maps bitstream bits to alphabet `idx`. Per
  `spec/99 §5.3` it lives inside the shared 68 KB walker tree at
  file offset `0x3df40` / VMA `0x1c2780c8`, used by helper
  `0x1c219351` for every VLC in the binary. The walker is tiered
  (8-bit primary pre-expansion + up to 2 refill tiers, stride
  `0x8000`); resolving it into a per-G-descriptor bit-length array
  requires either (a) disassembling the constructor at
  `0x1c210ee6` (which builds the runtime descriptor from the packed
  source regions `0x1c259a38` / `0x1c259d78`) or (b) decoding the
  walker structure directly.
* **Round 25 (2026-05-01)** confirmed the byte layout: 8 544
  records of `(symbol:u32-LE, bit_length:u32-LE)` partitioned as
  4 096 (primary) + 4 096 (refill A) + 352 (refill B / tail), with
  `(sym=11, bl=4)` as the dominant primary sentinel and `(sym=0, bl=8)`
  as the dominant refill-A sentinel. **The bit-reader semantics that
  navigate the walker (window width, MSB/LSB, refill index
  derivation) cannot be derived from the data alone** — three
  obvious indexing hypotheses (12-bit MSB-first, 12-bit LSB-first,
  8-bit pre-expansion stacked 16×) were tested against the
  observed entry distribution and all refuted. See
  `docs/video/msmpeg4/spec/10-walker-investigation.md` for the
  detailed write-up. Next round needs binary disassembly of
  helper `0x1c219351` and/or constructor `0x1c210ee6`.

The previously-shipped `region_05eed0.csv` candidate — a 64-entry
canonical-Huffman block at VMA `0x1c25fad0` — remains wired as
[`AcVlcTable::v3_intra_candidate`] for the synthetic-stream
pipeline tests, but its role is OPEN per `spec/99 §0.1 row 8` /
`§9 OPEN-O6` and **the alphabet size mismatches G5** (64 vs 102),
so it is not the v3 intra-AC source.

Net effect: real `.avi` v3 fixtures still decode at PSNR ≈ 5.30 dB
Y on `testsrc2 32×32` (DC-only intra reconstruction, zero-residual
inter), unchanged from r17/r18. The G4 / G5 descriptor data is now
fully runtime-accessible and audit-verified end-to-end; only the
bit-length walker tree at `0x3df40` is between the current state
and runnable AC decode.

`send_packet` on a v3 stream defaults to the AC placeholder
(DC-only reconstruction); callers that want to exercise the
candidate AC pipeline use [`picture::decode_picture_with_ac`] with
[`picture::AcSelection::Candidate`]. Synthetic streams + Kraft +
prefix-free + per-symbol round-trip + scan-order dispatch tests
all pass against the candidate (see `tests/intra_ac_candidate.rs`).
G4 / G5 descriptor coverage lives in `tests/g_descriptor_g4.rs`,
`tests/g_descriptor_g5.rs`, and `src/g_descriptor.rs::tests`
(39 tests total, including the LMAX-per-run audit cross-check
for both inter and intra alphabets).

## License

MIT.
