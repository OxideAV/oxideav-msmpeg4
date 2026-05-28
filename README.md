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
| MS-MPEG4v3 3-tier ESC body                     | wired (rounds 7 + 27 + 126: G5 LMAX/RMAX + tier 1/2/3 walk; integration-tested through `decode_intra_block_full_v3`) |
| P-frame MV VLC + half-pel MC (default table)   | complete              |
| P-frame MV VLC alternate table                 | unsupported (truncated dump) |
| Inter AC residual (G4 VLC → IDCT → add to MC)  | complete (round 123) |
| V1 / V2 bitstream                              | header + MV + MCBPC done; AC OPEN; v3-compat defaults pinned (round 129) |
| V1 / V2 shared CBPY VLC                        | binary cross-check vs H.263 (round 75) |
| G0..G5 (count_A, count_B) provenance pin       | spec/15 §3 binary-derived (round 81) |
| Unified `GFamily` dispatch surface (G0..G5)    | wired (round 174)     |

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

* **MS-MPEG4v3 intra 3-tier ESC body** — wired end-to-end as of
  **rounds 7 + 27** (and integration-hardened in round 126). The
  `decode_escape_body` walker chains tier 1 (level extension via
  `LMAX[last][run]`) → tier 2 (run extension via
  `RMAX[last][|level|] + 1`) → tier 3 (verbatim `1 + 6 + 8`-bit FLC
  triple) per spec/04 §2.3, driven by the G5 LMAX / RMAX tables
  (`audit/01` §4.1) re-derived from the same packed-Huffman source
  the primary VLC consumes. The same builders feed the G0..G3
  placeholders (round 7), so when those packed-Huffman bit-length
  sources land the 3-tier walk is unblocked for them too without
  re-touching `decode_escape_body`. Eight integration tests at
  `tests/intra_block_3tier_esc.rs` exercise the full DC + AC + ESC
  chain through the public `decode_intra_block_full_v3` boundary
  (tier 1 / 2 / 3 / DC-ESC tier / CBP-zero short-circuit / chroma
  DC scaler routing).
* **Inter AC** — wired end-to-end as of **round 123**. The P-frame
  inter-MB path now decodes the G4 inter VLC (`AcVlcTable::g4_inter`)
  for every CBP-coded block, dequantises, IDCTs to a signed residual,
  and adds it onto the MC prediction (`ac::decode_inter_block` +
  `picture::add_residual_to_picture`, spec/04 §1 / §2.6). Inter blocks
  start the scan at position 0 (DC is coded), always use fixed zigzag,
  and use the single-tier verbatim ESC. What remains inter-side is the
  **alternate MV VLC** (`mv_table_sel == 1`, truncated dump) and
  **INTER4V** (per-4×4 MV) signalling.

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

Round-126 adds `tests/intra_block_3tier_esc.rs` (8 tests): the
3-tier ESC body is exercised end-to-end through the public
`decode_intra_block_full_v3` entry point (DC VLC → AC walk → ESC
tiers → dequant). Tests cover (1) DC differential + sub-class-B
terminator, (2) zero-DC fast path (no sign bit consumed),
(3) tier-1 level extension, (4) tier-2 run extension,
(5) tier-3 verbatim FLC triple, (6) CBP-zero short-circuit
(no AC bits consumed), (7) chroma DC-scaler routing
(block_idx ≥ 4 uses `C_DC_SCALE_TABLE`), and (8) the DC ESC tier
in the direct-value 120-entry intra-DC VLC.

Round-174 (2026-05-29) lands a **unified [`g_family::GFamily`] dispatch
surface** over all six MS-MPEG4 DCT-AC G-descriptors (G0..G5), closing
the asymmetry between the existing `g_descriptor::g{4,5}_decode` and
`g_enum::GExtended` surfaces that had evolved separately as the
extraction work landed. The new enum exposes `const fn` accessors for
`count_a()` / `count_b()` (per spec/15 §3), `subclass_a_size()` /
`subclass_b_size()` (per spec/03 §4.4 / spec/15 §5.2),
`descriptor_base_offset()` (per spec/15 §2.1 — G0=`0x9d8` through
G5=`0xa8c` at 0x24-byte stride, ending at `+0xab0`), and `role()`
classifying each as `ChromaAndInter` (G0/G2/G4) or `IntraLuma`
(G1/G3/G5) per spec/14 §3.1. Two const dispatch fns
`for_chroma_selector(sel)` / `for_luma_selector(sel)` resolve
picture-header selector values ∈ {0,1,2} to the matching G-family per
spec/14 §3.1 (`0 → G2, 1 → G0, 2 → G4` chroma; `0 → G3, 1 → G1,
2 → G5` luma), with `sel == 2` doubling as the v1/v2 fallthrough so a
v1/v2 dispatcher can write `for_chroma_selector(2)` /
`for_luma_selector(2)` without a version-specific branch. `decode(idx)`
and `iter()` instance methods delegate to the existing `g_descriptor` /
`g_enum` plumbing — no new tables, no new decode logic, purely
additive. 15 new tests pin every structural fact (counts, partition
sizes, base offsets, role distribution, selector dispatch arms, and the
`idx > count_b ⇔ last=true` partition invariant from spec/13 §2)
across all six families. Test suite 253 → 268 (+15) lib tests.

Round-129 pins the **v1 / v2 → v3-compat selector defaults** as
two associated constants on `MsV1V2PictureHeader`
(`V1_COMPAT_DEFAULTS` / `V2_COMPAT_DEFAULTS`). Per
`docs/video/msmpeg4/spec/01-bitstream-framing.md` §1.4 the v1 and
v2 paths never read the v3-only per-frame selectors
`ac_chroma_sel` / `ac_luma_sel` / `dc_size_sel` / `mv_table_sel`
(those reads gate on `version == 3` at `1c211fdd` and
`1c21205a..1c2120aa`), so downstream code sharing a v3 entry
point must use `dc_size_sel = 0` (primary intra-DC VLC pair) and
the G4 + G5 default clusters for chroma + luma respectively. Per
spec/07 §1.4 / §1.6 v1 also has **no MB-level AC-prediction bit**
(`1c2171c7` body does not `call 0x1c215c9b` after the CBPY
decode) and **no patent-7,054,494 spatial DC predictor**
(`0x1c23a788 / 0x1c23a7b0` LUTs are not loaded); v2 adds the
AC-prediction bit only at intra-in-P macroblocks (spec/07 §2.4).
Eight new lib tests in `src/header.rs::tests` pin those facts at
runtime (`v{1,2}_compat_defaults_carry_v3_zero_initialisation_at_runtime`,
`v1_has_no_ac_prediction_anywhere_at_runtime`,
`v2_has_ac_prediction_only_at_intra_in_p_macroblocks_at_runtime`,
`v1_v2_lack_spatial_dc_predictor_at_runtime`,
`v1_v2_compat_defaults_are_distinct_values`,
`v1_pframe_with_umv_clear_parses`,
`v1_iframe_does_not_read_umv_bit`) plus a const-block
compile-time assertion that mirrors the runtime invariants so any
silent drift in the constants fails the build. Test suite
338 → 346 (+8). No runtime behavioural change; the consts are
new public API that downstream consumers (oxideav-avi tag
dispatch, oxideav-mkv codec resolver) can read to spell out the
"v1/v2 share v3 decode paths but with these defaults" contract.

## License

MIT.
