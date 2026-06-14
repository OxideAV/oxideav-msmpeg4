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
| H.263-style dequantisation + DC scalers        | complete (parity-bias direction corrected to spec/08 §5, round 275) |
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
| G0..G3 canonical-Huffman primary VLC           | wired (round 234, spec/11 §5 row 1-4) |
| MS-MPEG4v3 3-tier ESC body                     | wired (rounds 7 + 27 + 126: G5 LMAX/RMAX + tier 1/2/3 walk; integration-tested through `decode_intra_block_full_v3`); LMAX/RMAX **ground-truth-validated** against the binary's ESC-extension arrays (`region_060988`) for all 6 G-families (round 299, spec/08 §1-§2/§4.1) |
| P-frame MV VLC + half-pel MC (default table)   | complete              |
| P-frame MV VLC alternate table                 | byte LUTs landed (round 251); VLC source still truncated |
| Inter AC residual (G4 VLC → IDCT → add to MC)  | complete (round 123) |
| V1 / V2 bitstream                              | header + MV + MCBPC done; **P-frame skip + inter pixel pipeline end-to-end (round 285)**; I-frame / intra-in-P gated on the spec/07 §1.6 DC-prediction docs gap |
| V1 / V2 shared CBPY VLC                        | binary cross-check vs H.263 (round 75) |
| G0..G5 (count_A, count_B) provenance pin       | spec/15 §3 binary-derived (round 81) |
| Unified `GFamily` dispatch surface (G0..G5)    | wired (round 174); selector inverses + `subclass_of` (round 181); per-field descriptor offsets (round 254) |
| P-frame 1-MV predictor (Figure 7-34 top-left)  | routed through `mv_pred::predict_block_mv` (round 191) |
| 4-MV-per-MB predictor batch surface             | `Macroblock4MvDecoder` + bitstream tests (rounds 196 / 202) |
| 4-MV neighbour-MB bordering-cell picker         | `bordering_block_of_neighbour` + `pick_neighbour_mv_from_4mv` (round 208) |
| 4-MV neighbour-state resolver (1-MV vs 4-MV)    | `NeighbourMvKind` + `NeighbourSet` + `resolve_block_candidates` (round 214) |
| 4-MV stateful predict / commit driver           | `Macroblock4MvDecoderNeighbours` (round 221) |
| Picture-wide MV grid → `NeighbourSet` builder    | `MvGrid` + `MvGridCell` (round 227)          |
| G0..G3 packed-Huffman primary VLC                | wired (round 234, all 4 sources Kraft=1)     |
| `decode_pframe` MV cache routed through `MvGrid` | wired (round 240, replaces parallel `Vec<Option<Mv>>`) |
| Per-MB 4-MV decoder → `MvGridCell` one-shot bridge | `finalise_to_grid_cell` on both 4-MV decoders (round 243) |
| `MvGrid` video-packet / GOB boundary-reset helpers + raster iter | `clear_cell` / `clear_row` / `clear_all` + `iter_cells` (round 246) |
| Per-G-family `pri_A` / `pri_B` runtime-binding accessors | `pri_a_vma` / `pri_b_vma` / `pri_{a,b}_size_bytes` on `GFamily` (round 292, spec/14 §3) |

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

Round-181 (2026-05-29) extends `GFamily` with three new const-fn
accessors that complete the structural API: `subclass_of(idx)` returns
`Option<GSubclass>` classifying every alphabet index into sub-A
(`[0, count_B]`, `last=0`) or sub-B (`(count_B, count_A)`, `last=1`)
per spec/13 §2's disassembly at `1c216e2a..1c216e2f`, with the ESC
sentinel and out-of-range indices returning `None`; `chroma_selector()`
and `luma_selector()` are the **inverses** of `for_chroma_selector` /
`for_luma_selector` per spec/14 §3.1, returning the picture-header
selector value ∈ {0,1,2} that dispatches to this G-family or `None`
for the other role (`G2→0, G0→1, G4→2` for chroma; `G3→0, G1→1, G5→2`
for luma). A new `GSubclass` enum (variants `A`/`B`) names the two
partition classes documented through spec/13 §2 and §4.4. Six new
tests pin the partition counts (subclass_of × count_a iteration
matches `subclass_a_size` / `subclass_b_size`), agreement between
`subclass_of` and `decode().last` for every non-ESC idx in every
G-family, round-trip closure of both selector inverses, and the
role-exclusive structural property `chroma_selector().is_some() XOR
luma_selector().is_some()` from spec/14 §3.1. Test suite 268 → 274
(+6) lib tests. Purely additive; no rewiring.

Round-191 (2026-05-30) routes the P-frame **1-MV predictor** in
[`picture::decode_pframe_mb`] through the round-185
[`mv_pred::predict_block_mv(Block::TopLeft, …)`] surface, closing
the queued integration item from r185's CHANGELOG. Per
`docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md` and
ISO/IEC 14496-2:2004(E) §7.6.5 the 1-MV-per-MB case (the only mode
the v3 picture decoder currently supports) is *explicitly* mapped to
Figure 7-34's top-left sub-diagram, with the §7.6.5 four candidate-
validity substitution rules applied to the three neighbouring-MB
candidate cells (`left_mb`, `above_mb`, `above_right_mb`). The
pre-r191 path consumed [`mv::median_predictor`], a pre-spec
shortcut that zero-substitutes missing neighbours and then takes a
median — producing the spec-correct answer when **two or three**
neighbours are valid but diverging on the rule-3 case ("exactly two
candidate predictors are not valid → both set to the third"). The
rule-3 case fires when exactly **one** of `left`/`top`/`top_right`
is valid: pre-r191 the predictor was `median(only, 0, 0) = 0`;
r191 it is the lone valid neighbour itself. The equivalence on the
≥ 2-valid path is pinned by
[`mv_pred::tests::top_left_matches_existing_median_predictor_when_two_or_more_valid`]
(5×5×5×4 sample sweep, all green); the divergence on the
exactly-1-valid path is pinned by
[`mv_pred::tests::top_left_diverges_from_old_shortcut_in_rule_3_case`].
[`mv::median_predictor`] is retained for the v1/v2 test surface;
no API removal. Future round (4-MV-per-MB MCBPC variant) grows
this call site into a per-block loop over [`mv_pred::Block::ALL`],
each invocation reading the same predict_block_mv API.

Round-202 (2026-06-01) adds an **end-to-end bitstream exercise of
the 4-MV-per-MB decoder**: a new integration file
`tests/macroblock_4mv_bitstream.rs` drives `Macroblock4MvDecoder`
through the full predict → joint-VLC-decode → commit loop for a single
16x16 macroblock, using real codes from the v3 default joint-MV VLC
(`MV_V3_RAW`, source at VMA `0x1c25cbc0`). Four tests pin: (1) the
picture-corner case where block 1 fires §7.6.5 rule 4 (zero
predictor) and blocks 2 / 3 / 4 pick up earlier-committed within-MB
MVs via rules 2 / 3 — proving the decoder threads candidates
correctly across a real bitstream and not just synthesised `Mv`
values; (2) the all-neighbours case where every block exercises its
distinct Figure 7-34 layout without firing a substitution rule; (3)
a "rigid-motion" case where four zero-MVD codes against a non-zero
predictor produce the same `left_mb` MV across all four blocks
(every rule fires in turn and they all collapse to `left`); and
(4) a parallel-reader cross-check that `Macroblock4MvDecoder`
produces the same per-step predictor and decoded-MV as manual
`predict_block_mv` calls fed the same bitstream — pinning the
helper as a faithful sequencer over the public API. Test suite
gains 4 integration tests (376 → 380); fully additive, no
behavioural change in the 1-MV `picture::decode_pframe_mb` path.

Round-196 (2026-06-01) adds the **4-MV-per-MB batch surface** that
the r191 CHANGELOG queued: two new public APIs in
[`mv_pred`] that thread the within-MB candidate cells per Figure 7-34
across all four luminance blocks in a single call.
[`mv_pred::predict_macroblock_4mv_with_finals`] takes a
[`mv_pred::MacroblockCandidates`] (the three neighbour-MB MVs) plus
the already-decoded block-1/2/3 MVs and returns a `[Mv; 4]` of
per-block predictors, computed by calling
[`mv_pred::predict_block_mv`] once per [`mv_pred::Block::ALL`].
[`mv_pred::Macroblock4MvDecoder`] is the closed-form helper for
the predict-MVD-decode-reconstruct loop a future
`picture::decode_pframe_mb` 4-MV path will drive — caller
alternates `predictor_for(block)` (read the §7.6.5 spec predictor
given whatever is committed so far) and `commit_block(block,
final_mv)` (record the post-MVD-add MV so later blocks see it as a
within-MB candidate). Eight new lib tests pin: block-0-of-batch
matches a direct `Block::TopLeft` call (regression guard); all
four blocks match per-block `predict_block_mv` invocations under
the figure's per-block neighbour layout; the closed-form decoder
agrees with the batch function; the corner case where rule-4 fires
for block 1 and rule-3 promotes block-1's MV to block 2's
predictor when no neighbour MBs are available; block 4's
predictor *ignores* the three neighbour-MB MVs (BR sub-diagram is
all-within-MB per Figure 7-34); and out-of-order `commit_block` is
allowed but doesn't retroactively affect later blocks'
candidates. Test suite 291 → 299 (+8) lib tests. Purely additive;
the existing 1-MV call site in `picture::decode_pframe_mb` is
unchanged — wiring the 4-MV path into the picture decoder waits on
the MS-MPEG-4 v3 MCBPC bit pattern that signals 1-MV vs 4-MV mode
(an open spec gap; see "Still lacks" tail below).

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

Round-208 (2026-06-02) adds the **4-MV-per-MB neighbour-MB
bordering-cell picker** — a purely additive const-fn surface that
closes the "the caller is responsible for picking the right cell from
the neighbouring MB" comment in
[`mv_pred::MacroblockCandidates`]. When a current macroblock is
predicted in 4-MV-per-MB mode and one of its neighbouring macroblocks
(left, above, above-right) was *also* coded with K=4 motion vectors,
ISO/IEC 14496-2:2004(E) §7.6.5 / Figure 7-34 pins which of the
neighbour's 8x8 blocks sits adjacent to the current-MB block being
predicted. Two new const-fn APIs in [`mv_pred`] resolve the lookup:
[`mv_pred::bordering_block_of_neighbour(current, direction) ->
Option<Block>`] returns the bordering [`mv_pred::Block`] cell of the
indicated neighbour, and [`mv_pred::pick_neighbour_mv_from_4mv(current,
direction, &[Mv; 4]) -> Option<Mv>`] composes that lookup with an
index into the neighbour's `[Mv; 4]` raster-order array. A new
[`mv_pred::NeighbourDirection`] enum (variants `Left` / `Above` /
`AboveRight`) names the three neighbour-MB directions used by Figure
7-34. The (current-block, direction) → bordering-block table has
exactly six entries (block 1 takes left/above/above-right; block 2
takes above + above-right; block 3 takes left; block 4 takes nothing —
the BR sub-diagram is all-within-MB); the other six pairs return
`None` and the caller must use the within-MB candidate slot instead.
Adjacency facts come from the four Figure 7-34 sub-diagrams in
`docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md` cross-
checked against the rendered `figure-7-34-render.png` (PDF page 302)
— e.g. block 1's MV1 (left) sits in the right column / top row of the
left-neighbour MB, i.e. that neighbour's block 2 (TR); block 3's MV1
(left) sits in the right column / bottom row, i.e. that neighbour's
block 4 (BR). Ten new lib tests in `src/mv_pred.rs::tests` pin every
entry of the six-Some-six-None table per current-block sub-diagram,
the `[Mv; 4]` indexing behaviour, the `None` cases, the
`NeighbourDirection::ALL` enumeration, the const-fn property, and the
composition with [`mv_pred::predict_block_mv`] for the block-1
predict path. Test suite 299 → 309 (+10) lib tests. Purely additive;
existing `picture::decode_pframe_mb` 1-MV path is unchanged. The
helper is the public API a future 4-MV-mode picture decoder will use
when threading 4-MV-coded neighbour MBs into the
[`mv_pred::BlockCandidates`] / [`mv_pred::MacroblockCandidates`]
slots; for 1-MV-coded neighbours no resolution is required (the
neighbour's single MV is reported in every direction by definition).

Round-214 (2026-06-03) closes the gap between r208's per-pair
bordering-cell picker and a real-stream 4-MV picture decoder by
adding a **neighbour-state resolver** that consumes the three
neighbour-MB MV states (each `Absent` / 1-MV / 4-MV) and produces a
spec-correct [`mv_pred::BlockCandidates`] per current-MB block in one
call. The new public surface is three additive pieces in
[`mv_pred`]: a [`NeighbourMvKind`] enum (`Absent` / `OneMv(Mv)` /
`FourMv([Mv; 4])`) tagging each neighbour-MB's MV mode for that frame,
a [`NeighbourSet`] struct (`left` / `above` / `above_right`) bundling
the three directions, and a const-fn [`resolve_block_candidates`]
that composes [`NeighbourSet::candidate_for`] (which routes through
r208's [`bordering_block_of_neighbour`] internally) with the
within-MB-block threading [`BlockCandidates`] expects. A higher-level
[`predict_macroblock_4mv_with_4mv_neighbours`] batch wraps the
predict-loop over [`Block::ALL`]; unlike r196's
[`predict_macroblock_4mv_with_finals`] (which treats every neighbour
as 1-MV-coded by collapsing it to a single `MacroblockCandidates` MV
per direction), the new batch resolves the bordering-cell **per
current-MB block** — so when a 4-MV-coded left neighbour has distinct
MVs at its block 2 (TR, borders current block 1) and its block 4 (BR,
borders current block 3), the new path picks the correct one for each
current block, whereas r196's path can only pick one of them. Twelve
new lib tests in `src/mv_pred.rs::tests` pin: the `Absent` rule-1
short-circuit across every `(current, direction)` pair (12 cases);
the `OneMv` "same MV for every bordering pair" symmetry with the
documented 6-Some / 6-None split; the `FourMv` indexing by Figure 6-8
spec index per direction; the concrete worked example
(`current=TopLeft`, `left=FourMv` → picks block-2/TR cell of the left
neighbour); `resolve_block_candidates`'s neighbour-vs-within-MB
threading; bit-identical equivalence with `predict_macroblock_4mv_with_finals`
when every neighbour is `OneMv` or `Absent`; the corner-case
equivalence when every neighbour is `Absent`; the per-block divergence
when a distinct-cells `FourMv` left neighbour is present; the
`NeighbourMvKind::is_absent` predicate; and the const-fn property of
both `NeighbourSet::ABSENT` and `resolve_block_candidates`. Test suite
309 → 321 (+12) lib tests.
Purely additive; the existing 1-MV `picture::decode_pframe_mb` call
site is unchanged. The future picture-decoder rewrite that wires the
MS-MPEG-4 v3 MCBPC 1-MV-vs-4-MV bit will build its current-MB
predictors by calling `resolve_block_candidates` or the batch wrapper
once per MB, transparently handling any mix of 1-MV and 4-MV
neighbours through the same surface.

Round-221 (2026-06-03) adds [`mv_pred::Macroblock4MvDecoderNeighbours`],
the [`NeighbourSet`]-driven analogue of r196's
[`mv_pred::Macroblock4MvDecoder`]. It exposes the same predict /
commit / finalise shape — a future `picture::decode_pframe_mb` 4-MV
path will drive it from the bitstream — but routes every
`predictor_for(block)` call through [`mv_pred::resolve_block_candidates`]
so the bordering 8x8 cell of a 4-MV-coded neighbour is picked **per
current-MB block** per Figure 7-34. Compared with r196's
[`Macroblock4MvDecoder`] (which holds a flat
[`mv_pred::MacroblockCandidates`] = one `Option<Mv>` per direction
and therefore reports the same neighbour MV for every current block),
the new decoder carries the full [`mv_pred::NeighbourSet`]: when a
4-MV-coded left neighbour has distinct MVs at its TR (block 2,
borders current block 1) and BR (block 4, borders current block 3)
cells, the new decoder picks the correct one for each current block,
whereas the old surface can only pick one of them. Both shapes
co-exist: 1-MV-only neighbours can keep using the older decoder for
back-compat. Ten new lib tests in `src/mv_pred.rs::tests` pin: the
absent-neighbour predictor chain (block 1 = (0, 0) per rule 4;
blocks 2/3/4 pick up earlier within-MB MVs per rules 2/3); the
documented equivalence with [`Macroblock4MvDecoder`] when every
neighbour is `OneMv` (same predictors, same finals); the same
equivalence when every neighbour is `Absent`; the distinct-cell
divergence with a 4-MV left neighbour; the `neighbours()` accessor
round-trip; `Default` resolving to `NeighbourSet::ABSENT`; the
const-fn property of `new`; the out-of-order block-4-then-block-1
commit independence; `finalise`'s `Mv::default()` substitution for
uncommitted blocks; and the surface equivalence with
[`mv_pred::predict_macroblock_4mv_with_4mv_neighbours`] for the
block-1 entry. Test suite 321 → 331 (+10) lib tests; integration +
doc tests unchanged. Purely additive; the existing 1-MV
`picture::decode_pframe_mb` call site is unchanged.

Round-234 (2026-06-04) closes the **G0..G3 canonical-Huffman primary
VLC** OPEN item that has gated real-content v3 intra-luma + chroma AC
decode since round 7. Per `docs/video/msmpeg4/spec/11-walker-format-
resolved.md` §5 row 1-4 the four packed-Huffman sources for the
extended-alphabet G-descriptors live at file `0x57a30 / 0x57f80 /
0x58558 / 0x58a08` (VMAs `0x1c258630 / 0x1c258b80 / 0x1c259158 /
0x1c259608`), each in the same `(code, bl)` u32-pair format that
spec/11 §4 established for G4 / G5 / MCBPCY / intra-DC. The
`region_*_full.hex` slices were copied into `crates/oxideav-msmpeg4/
tables/`; a new `emit_packed_huffman_g_extended` build.rs emitter
parses each source (header u32-LE count, then `count × (code:u32-LE,
bl:u32-LE)` records, with the `0xFFFFFFFF` hole-sentinel branch
mirroring helper A from spec/11 §3) and enforces three build-time
invariants per source: file length `>= 4 + count * 8`, header count
matches the spec/15 §3 alphabet shape (169 / 186 / 149 / 133), and
Kraft sum is exactly `2^32` (saturated — unlike G4 / G5 which reserve
one bl=9 codeword for ESC at Kraft `1 - 2/1024`, G0..G3 use a regular
bit-length slot at `idx == count_A` per spec/09 §2). The four
emitted `G{0,1,2,3}_PRIMARY_RAW` arrays flow through the same
`build_g_primary` builder the G4 / G5 wiring uses; the `GTable` enum
now has six variants and each non-ESC idx is resolved to its
`(last, run, level)` triple via `g_enum::GExtended::decode` (round
29). `AcVlcTable::v3_intra_g{0,1,2,3}` now return non-empty entries
(169 / 186 / 149 / 133 each) — the `_synthetic` variants are kept
for diagnostic regression baselines. Two new lib tests pin (a) the
alphabet size + Kraft saturation per source, and (b) per-idx
agreement between the wired entries and `g_enum::GExtended::decode`
for the full count_A + 1 alphabet of every source; one additional
round-trip test runs a `decode_token` for idx 0 (always
`(last=false, run=0, level=1)` per spec/09 §2) and confirms the ESC
sentinel surfaces at the expected position. Lib tests 343 → 345
(+2); the integration test `g0_g1_g2_g3_carry_lmax_and_rmax_post_round_7`
was renamed and updated to assert non-empty entries
(`expected_entries = count_A + 1` per source). The mp43.wmv I-frame
luma DC-only-fallback observed in the round-5 implementer's static
analysis is now unblocked: `picture::AcSelection::FromHeader` plus
`for_luma_selector(0)` dispatches v3 intra-luma blocks through G3's
133-entry canonical-Huffman walker (instead of the empty placeholder
the round-7..233 path returned).

Round-246 (2026-06-07) closes the documented gap in
[`mv_pred::MvGrid`]'s "Boundary handling" section that
video-packet / GOB boundary substitution per
`docs/video/mpeg4-visual/figure-7-34-mv-predictor-layout.md` §
"Boundary substitution" must be applied by the caller writing
[`MvGridCell::Absent`] into grid positions on the far side of the
boundary. Four purely-additive helpers on [`mv_pred::MvGrid`]:
(1) [`mv_pred::MvGrid::clear_cell`] resets one `(mb_x, mb_y)`
position to [`MvGridCell::Absent`] — the boundary-resync primitive,
naming the intent and agreeing in observable effect with
`set_cell(mb_x, mb_y, MvGridCell::Absent)`. (2)
[`mv_pred::MvGrid::clear_row`] bulk-resets an entire MB row to
[`MvGridCell::Absent`] for the common video-packet-at-row-start
case. (3) [`mv_pred::MvGrid::clear_all`] resets every cell in the
grid to [`MvGridCell::Absent`] without re-allocating the backing
storage — picture-start reuse without a fresh allocation. (4)
[`mv_pred::MvGrid::iter_cells`] is a raster-order iterator over
`(mb_x, mb_y, MvGridCell)` triples (same order as the P-VOP MB
loop), useful for diagnostic / regression callers that want to
pair their expected per-MB cell list against the grid contents
without re-deriving the raster index manually. Eleven new lib
tests in `src/mv_pred.rs::tests` pin: `clear_cell` sets the target
position to `Absent` regardless of the starting variant (`Absent`
/ `OneMv` / `FourMv`) and leaves same-row siblings untouched;
`clear_cell` matches `set_cell(.., Absent)` and is a no-op on
out-of-bounds positions; `clear_row` resets every column of one
row only and is OOB-no-op; `clear_all` matches a fresh
`MvGrid::new(width, height)` in observable effect and preserves
dimensions; `iter_cells` walks the documented raster order
(`(0,0), (1,0), (2,0), (0,1), …`), round-trips its triples through
`cell_at`, and on a fresh grid yields all `Absent` cells; the
bulk-vs-per-cell equivalence
`clear_row(mb_y) == clear_cell(0..width, mb_y)` for every column
on the cleared row; and a worked boundary-resync example where MB
`(2, 1)` initially sees three populated neighbours, the caller
`clear_cell`-s those three positions, and
[`MvGrid::neighbour_set_for(2, 1)`] then reports every direction
`Absent` (rule-4 / all-zero predictor). Lib tests 355 → 366 (+11);
integration tests unchanged. Purely additive; the existing 1-MV
`picture::decode_pframe_mb` path is unchanged.

Round-243 (2026-06-07) closes the per-MB-decoder → picture-wide-grid
handoff that round 240 plumbed at the `picture::decode_pframe_mb`
layer. Three purely-additive surface extensions in
[`mv_pred`]: (1) [`mv_pred::Macroblock4MvDecoder::finalise_to_grid_cell`]
and [`mv_pred::Macroblock4MvDecoderNeighbours::finalise_to_grid_cell`]
return [`mv_pred::MvGridCell::FourMv`]`(self.finalise())` so a future
4-MV-mode site can write
`mv_grid.set_cell(mb_x, mb_y, decoder.finalise_to_grid_cell())` in a
single call, without manually wrapping the `[Mv; 4]` in the cell enum
— both helpers pick up `finalise()`'s `Mv::default()` substitution for
any block that was never committed, matching the documented
semantics. (2) Three `const fn` query predicates on `MvGridCell`:
[`mv_pred::MvGridCell::is_absent`] / `is_one_mv` / `is_four_mv`,
mirroring [`mv_pred::NeighbourMvKind::is_absent`] so a call site that
treats the two types interchangeably via the
`From<MvGridCell> for NeighbourMvKind` conversion reads the same way
before and after. (3) [`mv_pred::MvGrid::dimensions`] returning
`(width, height)` as a pair, in the same constructor-arg order as
`MvGrid::new`, matching the per-axis accessors and acting as a
single-call alternative. Eight new lib tests in
`src/mv_pred.rs::tests` pin: the four-commit sweep on both decoders
producing the correct `FourMv` payload in Figure 6-8 raster order;
the `Mv::default()` substitution for uncommitted blocks; the
end-to-end round-trip through `MvGrid::set_cell` + `cell_at` +
`neighbour_set_for` (writing a 4-MV cell at one position then reading
it as the `left` neighbour of the next-column MB); the
mutually-exclusive trichotomy of the three predicates over the three
cell variants; `is_absent` agreement with the `NeighbourMvKind`
conversion side; and the `dimensions()` accessor matching
`(width(), height())` across multiple grid shapes plus its `const fn`
property. Lib tests 347 → 355 (+8); integration tests unchanged.
Purely additive; the existing 1-MV `picture::decode_pframe_mb` path
is unchanged.

Round-251 (2026-06-07) lands the **alternate-variant MVDx / MVDy byte
LUTs** for the v3 joint MV decoder (`mv_table_sel == 1` path). Per
`docs/video/msmpeg4/spec/06-mv-decoder.md` §2.2 the alternate VLC
source at VMA `0x1c25a0b8` pairs with two byte LUTs at VMAs
`0x1c25c320` (MVDx) and `0x1c25c770` (MVDy), each 1104 bytes
(1099 alphabet entries + 5 bytes of alignment padding, identical
shape to the default variant's `0x1c25ee28` / `0x1c25f278`). The two
pre-extracted hex files `tables/region_05b720.hex` and
`tables/region_05bb70.hex` (1104 bytes each, mean ≈ 32 matching the
spec/06 §3.5 bias-32 distribution) were already present in
`docs/video/msmpeg4/tables/` and are now copied into the crate's
`tables/` directory and wired through a new `build.rs` emitter
`emit_mv_byte_lut_v3_alt`. The emitter follows the exact same shape
as `emit_mv_byte_lut_v3` (xxd parser + 1104-byte length assert + two
`pub static …: &[u8; 1104]` arrays), producing
[`tables_data::MVDX_V3_ALT_BYTES`] and
[`tables_data::MVDY_V3_ALT_BYTES`]. Four new lib tests in
`src/mv.rs::tests` pin: (1) both arrays are exactly 1104 bytes;
(2) every active entry (indices 0..=1098) is in `[0, 63]` — the
unsigned-6-bit pre-biased MV-residual range that the decoder's
`raw - 32` bias-subtract assumes per spec/06 §3.5; (3) the alt LUTs
differ from the default LUTs over indices 0..=1098 (the two patent
6,983,018 training corpora produce distinct values for the same
alphabet shape); (4) the alt LUT means cluster around the +32 bias
(both in `[20, 44]`). The existing
`mv_table_alternate_is_unsupported_with_diagnostic` test was
extended to require the diagnostic now surfaces
`MVDX_V3_ALT_BYTES` / `MVDY_V3_ALT_BYTES` and the spec/11 §5
full-source size `8804` — so that when the VLC source at file
`0x594b8` is re-extracted at its real 8804-byte size
(currently a 256-byte truncation per
`docs/video/msmpeg4/tables/region_0594b8.meta`), the alternate-path
swap in `mv::decode_mv_with_table` is a single
canonical-Huffman-builder edit: the byte LUTs it would consume are
already wired. Lib tests 366 → 370 (+4); integration tests
unchanged. Purely additive; `MvTable::Alternate` still returns the
`Error::Unsupported` diagnostic — affected fixtures
(`div3.avi` frames 37/38/40, `div4.avi` frames 1/16 per
`memory/project_msmpeg4_runtime_binding_clues.md` §2.1) still
reject at picture-dispatch time, exactly as before.

Round-254 (2026-06-08) lands a **per-descriptor field-offset accessor
surface** on [`crate::g_family::GFamily`] mirroring the 36-byte
record schema documented in
`docs/video/msmpeg4/spec/15-count-ab-per-g-family.md` §1. The new
[`crate::g_family::GDescriptorField`] enum names every one of the
nine `u32` slots a G-descriptor holds (`DecoderObj`, `CountA`,
`CountB`, `SubALevelExtPtr`, `SubBLevelExtPtr`, `SubARunExtPtr`,
`SubBRunExtPtr`, `PriABase`, `PriBBase`), and exposes
`offset_in_record()` (`+0x00, 0x04, …, 0x20` per spec/15 §1) plus a
uniform `size_in_bytes()` (every slot is `u32`). Two new const-fn
accessors on `GFamily` thread these through the family's own base
offset: `field_offset(field)` returns the descriptor-relative offset
(family-invariant per spec/15 §1) and `field_state_struct_offset(field)`
returns the absolute state-struct offset (`descriptor_base_offset()
+ field.offset_in_record()`). Two new module-level constants —
[`crate::g_family::DESCRIPTOR_RECORD_BYTES`] (`0x24`) and
[`crate::g_family::DESCRIPTOR_CLUSTER_END_OFFSET`] (`0xab0`) —
document the per-record size and the `[+0x9d8, +0xab0)` G0..G5
cluster span per spec/14 §1. Eight new lib tests pin the field-offset
schema against spec/15 §1 (`descriptor_field_offsets_match_spec_15_record_schema`,
`descriptor_field_all_lists_every_field_in_record_order`,
`descriptor_field_offset_delegates_to_field`,
`descriptor_field_state_struct_offsets_match_spec_15_disassembly`
[exact `count_A` / `count_B` storage VMAs for all six families per
spec/15 §2.1's literal-immediate disassembly:
G0=`+0x9dc,+0x9e0`, G1=`+0xa00,+0xa04`, G2=`+0xa24,+0xa28`,
G3=`+0xa48,+0xa4c`, G4=`+0xa6c,+0xa70`, G5=`+0xa90,+0xa94`],
`descriptor_field_state_struct_offset_decomposes`,
`descriptor_record_size_and_cluster_end_match_spec_14`,
`descriptor_fields_stay_inside_record_and_cluster`,
`count_storage_offsets_consistent_with_count_values`). Lib tests
370 → 378 (+8); integration tests unchanged. Purely additive; no
runtime path is rewired and no new tables are introduced. The
schema-mirror surface is intended as a docs-level binding (the
decoder does not lay out a literal in-memory `GDescriptor` struct
today, it stores the runtime equivalents on Rust-side enum variants)
so that any future Auditor-style reconciliation between the binary's
constructor stores and the in-tree decoder's state has a single
canonical reference: `GFamily::field_state_struct_offset(field)`
must equal the VMA the constructor disassembly cites for that
`(family, field)` pair.

Round-266 (2026-06-09) lands a **typed-primitive accessor surface** on
the picture-header parser so callers that already hold a parsed
[`crate::header::MsV3PictureHeader`] reach the spec-correct
[`crate::g_family::GFamily`] / [`crate::mv::MvTable`] enums without
re-importing the standalone dispatchers or restating per-frame slot
identities. Five new const-fn accessors:
[`crate::mv::MvTable::from_sel`] resolves a `mv_table_sel` bit ∈ {0,1}
to `Some(MvTable::Default)` / `Some(MvTable::Alternate)` per spec/06
§3.2 (and `None` for out-of-range); [`crate::mv::MvTable::to_sel`] is
its inverse (`Default → 0, Alternate → 1`).
[`crate::header::MsV3PictureHeader::ac_chroma_family`] /
[`crate::header::MsV3PictureHeader::ac_luma_family`] delegate to
[`crate::g_family::GFamily::for_chroma_selector`] /
[`crate::g_family::GFamily::for_luma_selector`] (spec/14 §3.1: chroma
`0→G2, 1→G0, 2→G4`; luma `0→G3, 1→G1, 2→G5`).
[`crate::header::MsV3PictureHeader::mv_table`] delegates to
[`crate::mv::MvTable::from_sel`]. Two new associated fns on
[`crate::header::V1V2V3CompatDefaults`]
(`v1_v2_fallthrough_chroma_family` / `v1_v2_fallthrough_luma_family`)
surface the real v1/v2 runtime cluster bound at the
`0x1c212917` fallthrough write site per spec/14 §3.1
(`[esi+0xab0] = G4` chroma, `[esi+0xab4] = G5` luma) — independent of
the (zero-pinned, don't-care) `ac_chroma_sel` / `ac_luma_sel` compat
fields, which would otherwise dispatch through the v3 per-frame
selector path to G2 / G3 (i.e. **not** the v1/v2 cluster). The two
surfaces deliberately diverge, pinned by
`compat_defaults_v3_dispatch_does_not_equal_v1_v2_fallthrough` so a
future round can't accidentally collapse them. Seventeen new lib
tests across `src/mv.rs::tests` (5) and `src/header.rs::tests` (12)
pin every dispatch arm, the OOR-`None` defence-in-depth path on every
accessor, the inverse round-trip for both selectors, the spec/14 §3.1
v1/v2 fallthrough family identities, and the end-to-end
parse-then-resolve flow through a real bit-packed v3 P-frame header
with `ac_chroma_sel=1` + `mv_table_sel=1`. Lib tests 378 → 395 (+17);
integration tests unchanged. Purely additive; no runtime path is
rewired and no new tables are introduced.

Round-285 (2026-06-12) lands the **v1/v2 P-frame pixel pipeline** —
the first version-1/-2 path that produces actual pixels. The new
public [`picture::decode_picture_v1v2`] (+ [`picture::MsV1V2Version`])
chains the pieces earlier rounds wired in isolation: the v1/v2
picture-header parsers (spec/01 §1.4; the v1 P-frame UMV flag is
consumed as framing but never branched on, per spec/07 §3.4's "the
v<4 body does not branch on it"), the per-MB skip bit + separate
MCBPC / CBPY VLCs (spec/07 §1-§2, `decode_mcbpcy_v{1,2}` from round
11), the §7.6.5 1-MV predictor (spec/07 §3.5 pins the v1/v2 MV body
to the *same* median-of-3 helper `0x1c217c8c` as v3, so the path
routes through the same `MvGrid` + `predict_block_mv` surface the v3
P-loop uses), the per-component MV pair (`mv::decode_mv_v1v2`, round
12), half-pel MC, and the **G4 inter AC residual** for every
CBP-coded block — spec/14 §3.1's v1/v2 fallthrough at `0x1c212917`
binds the inter/chroma DCT descriptor to G4 unconditionally, and
spec/99 §6 pins the inter kernels to the shared hard-zigzag /
scan-start-0 / single-tier-ESC shape across v1/v2/v3. `send_packet`
on `msmpeg4v1` / `msmpeg4v2` now emits `Frame::Video` for P-frames.
Three paths stay behind *documented* `Unsupported` gates with the
precise citation in the error text: (1) **v1/v2 I-frames and
intra-in-P MBs** — spec/07 §1.6 pins that v1/v2 do **not** load the
v3 spatial-prediction LUT pair at `0x1c23a788 / 0x1c23a7b0`, but no
staged chapter documents the replacement DC-prediction rule (or the
v1/v2 intra DC-size descriptor binding) for the shared intra kernel
`0x1c216d97`, so intra pixels would be guesswork; (2) **v1 non-zero
inter sub-types** (`mb_type ∈ {1, 2, 4, 5}`) — spec/07 §1.4 asserts
only the `mcbpc >> 2` decomposition and the H.263 Table-8 *lineage*
("structural, value-level match not asserted"), leaving the
per-sub-type side reads untraced. The v3 P-loop's predictor lookup
and residual walk were factored into shared helpers
(`one_mv_predictor` / `decode_inter_residual_blocks`) with no v3
behavioural change. New `tests/v1_v2_pframe.rs` (11 tests) covers
all-skip identity, zero-MV copy, +1-pel MV shift, residual locality,
all three gates' diagnostics, the missing-reference error, and the
`send_packet` surface; the ffmpeg-backed MP42 expectation in
`tests/v1_v2_mcbpcy.rs` now pins the real-stream I-frame stop-line.
Integration tests 97 → 108 (+11).

Round-275 (2026-06-11) **corrects the H.263 quantiser-parity bias
direction** in [`iq::dequantise_h263`]. The kernel had computed the
Eq. 12 additive offset as `bias = PQUANT - (PQUANT & 1)` — subtracting 1
for **odd** PQUANT. The sandbox-verified runtime materialisation in
`docs/video/msmpeg4/spec/08-descriptor-constants.md` §5 (the
`audit/06` hand-patched-PQUANT trials watched at `[ctx+0x138]`) shows
the per-frame even-parity flag is `1` iff PQUANT is **even** and
`bias = PQUANT - even_flag`, i.e. `bias = PQUANT - 1` for even PQUANT
and `bias = PQUANT` for odd. `spec/07` §4.2's inline annotation
"`edx = 1 if odd else 0`" mis-described its own quoted
`neg edx; sbb edx,edx; inc edx` idiom: applied to `edx = PQUANT % 2 ∈
{0,1}` that idiom evaluates to `1 - CF` = 1 when even, 0 when odd, in
agreement with spec/08 §5. The one-line fix (`even_flag = 1 - (q & 1)`)
changes every non-zero coefficient's dequantised magnitude for **even**
PQUANT (the common case in real streams) — e.g. q=4, |level|=1 now
yields `2·4 + (4−1) = 11` instead of the previous `2·4 + 4 = 12`. A new
sweep test pins the corrected direction across PQUANT 1..=31 and four
existing value-expectation tests plus one integration assertion were
updated. Lib tests 395 → 396 (+1); the module docs now carry the
spec/07-vs-spec/08 reconciliation inline.

Round-292 (2026-06-14) lands a **per-G-family `pri_A` / `pri_B`
runtime-binding accessor surface** on [`g_family::GFamily`], completing
the schema-binding reference the round-254
`field_state_struct_offset(PriABase / PriBBase)` accessors point at
(they yield the descriptor `+0x1c` / `+0x20` slot offsets; this round
resolves each slot to the `.data` VMA it is bound to and the bound
array's byte size). Per
`docs/video/msmpeg4/spec/14-pri-ab-runtime-binding.md` §2.1 / §3 the
constructor `mb_mv_struct_init` (`0x1c210643`) writes 12 literal-
immediate stores binding every G-descriptor's `pri_A` (level-magnitude
byte array) and `pri_B` (run-value u32 array) base pointers — a
**static, one-shot** binding that is never re-bound for the lifetime of
the decoder (spec/14 §5.2 item 5). The four new const-fn accessors are
[`GFamily::pri_a_vma`] (G0=`0x1c257860` … G5=`0x1c258430`),
[`GFamily::pri_b_vma`] (G0=`0x1c2575c0` … G5=`0x1c258498`),
[`GFamily::pri_a_size_bytes`] (= `count_A`, 1 byte/symbol) and
[`GFamily::pri_b_size_bytes`] (= `count_A * 4`, a u32 run value/symbol).
Four new lib tests pin every VMA against the spec/14 §3 binding table,
the byte sizes against §3 and their definitional equality with
`count_a()`, the spec/14 §2.3 cluster-containment + pairwise-non-overlap
of all 12 arrays inside `[0x1c2575c0, 0x1c258630)` (the tail ending
exactly at the first per-slot packed-Huffman source VMA `0x1c258630`),
and the const-fn property. Test suite 396 → 400 (+4) lib tests; purely
additive, no runtime path rewired and no new tables introduced. The
surface is intended as the canonical Auditor reconciliation reference:
`GFamily::field_state_struct_offset(field)` names the descriptor slot,
`GFamily::pri_{a,b}_vma()` names the `.data` VMA stored there, and the
in-tree `(idx → |level|)` / `(idx → run)` mappings (via
[`g_descriptor`] / [`g_enum`]) are the materialised form of the byte /
u32 arrays at those VMAs.

Round-299 (2026-06-14) closes the spec/08 §4.1 "ESC-extension
slice-content semantics OPEN" item from the Implementer side. The
binary's intra (`0x1c216d97`) and inter (`0x1c215e6f`) kernels reach
four per-descriptor arrays (`desc+0x0c`/`+0x10` level-extension,
`desc+0x14`/`+0x18` run-extension) only on the first- and second-tier
ESC paths, indexing them with the re-decoded symbol's value
(`docs/video/msmpeg4/spec/08-descriptor-constants.md` §1-§2). Those
arrays were extracted at `region_060988` (VMA `0x1c261588..0x1c261e00`)
and wired byte-for-byte in round 33, but their *content* meaning was
left OPEN. A new lib test
`ac::tests::esc_ext_arrays_match_derived_lmax_rmax_all_g` proves
empirically that each level-extension array is exactly the per-run
`LMAX[sub-class]` table and each run-extension array is exactly the
per-level `RMAX[sub-class]` table that the round-7/27/126 3-tier ESC
body (`decode_escape_body`) already derives from the primary alphabet
— and that the two agree **bit-for-bit** for all six G-families over
each array's meaningful extent (sub-A → `[last=0]`, sub-B →
`[last=1]`). The run-extension arrays store a never-indexed
`0xFFFFFFFF` sentinel at the `level == 0` slot (an ESC's re-decoded
base symbol always carries `|level| >= 1`) and trailing bytes past the
last alphabet-representable run/level (spec/08 §2.4 only upper-bounds
each array at `count_A * 4`); neither is compared. Net effect: the
previously "derived but never cross-checked against the binary's own
extension tables" 3-tier ESC body is now a **ground-truth-verified**
decode path. Lib tests 400 → 401 (+1). Purely additive; no runtime
path is rewired and no new tables are introduced.

## License

MIT.
