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
| Intra AC run/level VLC plumbing                | wired (G5)            |
| Intra AC `(last, run, level)` symbol mapping   | wired (G5 desc + ESC) |
| G4 (inter) `pri_A` + `pri_B` byte arrays       | wired (round 18)      |
| G5 (intra-luma) `pri_A`                        | wired (round 18)      |
| G5 (intra-luma) `pri_B`                        | wired (round 19)      |
| G4 / G5 canonical-Huffman primary VLC          | wired (round 26)      |
| G0..G3 `(idx → (run, level, last))` enumeration | wired (round 29)     |
| G0..G3 LMAX / RMAX (ESC-extension offsets)     | wired (round 7, 2026-05-17) |
| G0..G3 canonical-Huffman primary VLC           | OPEN — extraction blocked (spec/99 §10) |
| MS-MPEG4v3 3-tier ESC body                     | OPEN — MPEG-4 fallback only |
| P-frame MV VLC + half-pel MC (default table)   | complete              |
| P-frame MV VLC alternate table                 | unsupported (truncated dump) |
| Inter AC run/level VLC (G4 wired but unused)   | pending — needs P-frame plumbing |
| V1 / V2 bitstream                              | header + MV + MCBPC done; AC OPEN |
| V1 / V2 shared CBPY VLC                        | binary cross-check vs H.263 (round 75) |
| G0..G5 (count_A, count_B) provenance pin       | spec/15 §3 binary-derived (round 81) |

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

**Round 26 (2026-05-01)** wires the **G4 / G5 primary canonical-Huffman
VLC** from the packed-Huffman sources at file offsets `0x58e38` and
`0x59178` (828 bytes each — `4 + 103 * 8`). Per
`docs/video/msmpeg4/spec/11-walker-format-resolved.md` §3-§5 the loader
helper `0x1c218cfa` consumes a flat `(code:u32-LE, bit_length:u32-LE)`
record stream — the `code` field IS the literal bit-pattern (verified by
direct prefix-freedom + Kraft sum 0.998047 over both tables). The
`code_value` column the spec/99 §8.1 wording attached to MCBPCY
(`region_05eac8`) was a runtime LUT/state byte for that table only; the
G-table sources use the same record shape but `code_value` is the actual
canonical bit-pattern. With this in place [`AcVlcTable::v3_intra_g5`]
returns a 103-entry [`Symbol::RunLevel`] / [`Symbol::Escape`] table that
the existing [`ac::decode_intra_ac`] walker drives end-to-end.

[`picture::AcSelection::G5`] is now the shipping default for v3 intra
blocks. The 68 KB walker tree at file `0x3df40` (the previous round-25
investigation target) turned out to be **not** the G-table primary VLC
input — per spec/11 §1, the walker is consumed by a separate helper
(`0x1c219351`) for performance acceleration; the per-slot loader
operates on the much smaller `4 + count * 8` packed sources directly,
and the `(code, bl)` arrays produced by it are sufficient for an
O(n)-per-symbol reference decoder.

What's **still missing for bit-exact real-content decode**:

* **MS-MPEG4v3 intra 3-tier ESC body** (spec/04 §2.3 /
  `0x1c216d97`). The current `decode_escape_body` implements only the
  MPEG-4 Part 2 fixed-length fallback (1 + 6 + 8 = 15 bits); the v3
  intra path uses (a) a level-extension VLC tier, (b) a run-extension
  VLC tier with a symbol-indexed run offset, then (c) the verbatim
  6/8 fallback. ESC is rare on low-bitrate intra content but causes
  bit-stream desync whenever it fires, which produces the
  `scan position ≥ 64` overflow currently seen on `testsrc2 32×32`
  past the first MB or two. Wiring this is the next round's work.
* **Inter AC** — G4 primary VLC is also wired (`AcVlcTable::g4_inter`)
  but the P-frame block-decode path doesn't yet consume it. Round
  26 leaves the G4 hookup as plumbing for the next inter round.

Net effect: `testsrc2 32×32` real DIV3 decode now reaches per-block
AC walks (errors at `scan position 64 exceeds block` past the first
intra block, indicating bitstream desync at the first ESC); the G5
canonical-Huffman walker itself is exercised end-to-end by 7 dedicated
synthetic-stream tests (`tests/g5_primary.rs`) covering shortest code,
3-bit code, every-non-ESC round-trip, ESC body, prefix-freedom, and
alphabet partition.

`send_packet` on a v3 stream now uses the G5 path by default; the
placeholder and 64-entry candidate are still selectable via
[`picture::decode_picture_with_ac`] with the matching
[`picture::AcSelection`] variant for diagnostic / regression runs.
Synthetic streams + Kraft + prefix-free + per-symbol round-trip +
scan-order dispatch tests all pass against G5
(`tests/g5_primary.rs`, 7 tests) and the candidate
(`tests/intra_ac_candidate.rs`, 6 tests).
G4 / G5 descriptor coverage lives in `tests/g_descriptor_g4.rs`,
`tests/g_descriptor_g5.rs`, and `src/g_descriptor.rs::tests`
(39 tests total, including the LMAX-per-run audit cross-check
for both inter and intra alphabets).

## License

MIT.
