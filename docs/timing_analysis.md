---
layout: default
---

# Timing analysis

**Rust module:** `src/analysis/timing.rs` (overlay: `src/app/timing_view.rs`)
**Reference:** ProtoPirate's Timing Tuner, `REFERENCES/ProtoPirate/scenes/protopirate_scene_timing_tuner.c`
and `scenes/plugins/protopirate_timing_tuner_plugin.c`

ProtoPirate's Timing Tuner compares "real fob signal timing against protocol definitions". It shows
the protocol's short/long/tolerance, the received average/min/max/count, the difference and jitter,
and a conclusion on whether the timing matches or needs adjusting. KAT runs the same analysis on a
capture's raw level/duration pairs. For every capture, and especially for undecoded ones, it adds
pulse clustering, a base-TE estimate and a line-coding guess as research help.

## Use

- Signal menu (Enter on a capture): **Timing analysis** (after "Export .sub (Flipper)"; also on
  receive-only devices).
- Command: `:timing` analyses the selected capture, `:timing 7` capture 7.

The report opens in a scrollable overlay (Up/Down, PgUp/PgDn, Home/End; Esc or Enter closes). Its title
gives the coding guess and the TE estimate, e.g. `Timing analysis: #02 Ford V0 (Manchester, TE ~251 us)`.
Captures without raw pairs (a `.fob` exported without them) cannot be analysed.

## Protocol comparison (decoded captures)

The definition is the decoder's `ProtocolDecoder::timing()` (`te_short`, `te_long`, `te_delta`).
Legacy and display names resolve the way the registry does (`PSA2` → PSA, `Kia V4` → Kia V3/V4,
`Keeloq (X)` → KeeLoq).

As in ProtoPirate's `calculate_timing_stats`:

- Durations of **both levels** count. The valid window is `te_short − 2·te_delta` … `te_long +
  2·te_delta`; durations outside it (gaps, noise) are ignored.
- Valid durations below `(te_short + te_long) / 2` are **short**, the rest **long**. Each class gets
  its average (integer), min, max and count.
- **Diff** = average − definition; **jitter** = max − min.
- Per class: **EXCELLENT** if |diff| ≤ 15 µs, **OK (±d)** if |diff| ≤ `te_delta`, else **HIGH by d** /
  **LOW by d**.
- Verdict: both EXCELLENT → "Timing matches fob! / No changes needed."; both OK → "Within tolerance. /
  Consider fine-tuning."; otherwise "NEEDS ADJUSTMENT:" with `Set te_short=<avg>` / `Set te_long=<avg>`
  for each class out of tolerance.
- Fewer than 10 valid durations: "Not enough samples".

KAT differences, each for a reason found on real captures:

| ProtoPirate | KAT | Why |
|---|---|---|
| Valid window floored at 100 µs | floored at `min(100, te_short / 2)` µs (identical for every `te_short` ≥ 200 µs) | the 100 µs floor drops every short pulse of Ford V1 (65 µs), Honda Static (63 µs), BMW (62 µs) and Mercedes V1 (50 µs), leaving nothing to analyse |
| The last 512 durations before the decode | the decoded frame: the last `2·min_count_bit + 32` pairs (at most 512) | a `.sub` file's first decoded segment can be thousands of noise pulses followed by one frame (`FORD/0000_Ford_ts_x4.sub`: 6168 pairs). With the frame window the Ford V0 comparison reads "Timing matches fob!" instead of noise averages |
| A class without samples prints "LOW by te" and `Set te_long=0` | "NO SAMPLES" and the verdict **INCOMPLETE** ("no long durations (≥ N us) in the analysed pairs") | e.g. Honda Static's `te_long` is its 700 µs sync, which the frame does not contain |

Protocols with several timing variants report against the one definition `timing()` returns. VAG's
is 500/1000 µs (Type 3/4), so a Type 1/2 capture (300/600 µs) has no "long" durations and comes out
INCOMPLETE; its research view still shows TE ≈ 300 µs.

## Research view (every capture)

**Analysed pairs.** A decoded capture uses the frame window above. An undecoded capture uses all of
its pairs if there are at most 512; otherwise it uses the most regular 512-pair window, searched in
steps of 128. The score is the share of durations explained by one TE times how well the HIGH and LOW
estimates agree. The first line of the report says which pairs were used.

**Clusters.** HIGH and LOW durations are clustered separately the way rtl_433's pulse analyzer does it:
a duration joins the nearest cluster within ±20 %, otherwise it starts a new one, and neighbouring
clusters that drift within 20 % of each other are fused. For each cluster the report shows mean,
count, min, max, jitter (max − min) and the multiple of the TE. A cluster holding at least 8 % of its
level's durations is a data width. The others (glitches, sync pulses, inter-frame gaps) are marked
"(minor)", and beyond eight per level they are summarised.

**Base TE.** For each level: the data-width cluster whose 1–4 multiples (within ±0.3) explain the most
durations (the larger unit on a tie), refined by the least-squares fit `TE = Σ n·mean·k / Σ n·k²`
over the clusters it explains. When the HIGH and LOW estimates agree (within ×1.6) the TE is their
mean. That cancels the constant widening an AM envelope detector adds to pulses and takes from gaps,
and the report states it ("HIGH widths stretched by ~12 us"). It also gives the share of durations
that are 1–4 × TE.

**Line coding guess.** Built from the two dominant widths per level (short S, long L) and from how each
pulse pairs with the next gap. Both orders are tried, HIGH→LOW and LOW→HIGH, because an inverted
capture pairs the halves of adjacent bits. Runs of 8+ identical pairs count as preamble and are skipped.

| Guess | Rule |
|---|---|
| PWM | two widths per level, and S+L / L+S pairs while L+L is rare (≤ 5 %) in one order; the bit period is the mean pair length |
| PWM, equal-length pairs | S+S and L+L pairs only (mixed ≤ 5 %): the pair length carries the bit (Kia V0 / Suzuki V0) |
| Manchester / bi-phase | both widths ~1:2 on both levels, and mixed and L+L pairs ≥ 10 % in both orders (differential Manchester looks the same at this level) |
| PWM with fixed gap | two pulse widths, one gap width |
| PPM / pulse-distance | one pulse width, two or more gap widths (GM style) |
| NRZ / PCM | three or more widths on a level, all multiples of the TE |
| single width | one pulse and one gap width (preamble or carrier bursts) |

Without a protocol reference the report closes with a suggested definition, `te_short ~TE` and
`te_long ~k·TE` (the smallest data multiple ≥ 2), as a starting point for a new decoder.

## Results on IMPORTS

`cargo test sweep_imports_timing -- --ignored --nocapture` analyses every IMPORTS decode the way
the app stores it (the decoded segment of each `.sub`):

| Protocol | Decodes | Matches | Within tol. | Incomplete | Needs adj. | Coding guess |
|---|---|---|---|---|---|---|
| Ford V0 | 141 | 135 | 6 | 0 | 0 | Manchester 141 |
| Ford V3 | 27 | 20 | 7 | 0 | 0 | Manchester 26, NRZ 1 |
| Fiat V0 | 11 | 0 | 11 | 0 | 0 | Manchester 11 |
| Fiat V1 | 7 | 0 | 7 | 0 | 0 | Manchester 7 |
| Fiat V2 | 29 | 29 | 0 | 0 | 0 | Manchester 29 |
| Kia V1 | 35 | 35 | 0 | 0 | 0 | Manchester 35 |
| Kia V3 / Kia V4 | 300 | 300 | 0 | 0 | 0 | PWM 300 |
| Kia V5 | 119 | 119 | 0 | 0 | 0 | Manchester 119 |
| Suzuki V0 | 15 | 15 | 0 | 0 | 0 | PWM 15 (equal-length pairs) |
| Subaru | 5 | 3 | 2 | 0 | 0 | PWM 5 |
| PSA | 4 | 0 | 4 | 0 | 0 | Manchester 4 |
| VAG | 198 | 33 | 68 | 97 | 0 | Manchester 194, other 2, single width 1, PPM 1 |
| Honda Static | 41 | 0 | 1 | 37 | 3 | Manchester 41 |

The coding guess agrees with KAT's own encoding label (`Capture::modulation()`) for 927 of the 932
decodes. The VAG INCOMPLETE rows are its Type 1/2 frames and the Honda Static rows its 700 µs sync
definition, both explained above. A regular test (`imports_captures_match_their_protocol_timing`)
checks Kia V1, Ford V0, Subaru and Kia V3 captures against their definitions.

## Example

`IMPORTS/FORD/2008_Focus_Lock.sub`, second decode (shortened):

```
Analysed pairs 124..283 of 283 (80 HIGH, 80 LOW): the decoded frame

PROTOCOL DEFINITION (Ford V0):
  Short: 250 us
  Long: 500 us
  Tolerance: +/-120 us

RECEIVED SIGNAL:
  (both levels; short < 375 us <= long; valid 100..740 us; 23 ignored)
  Short Avg: 241 us  Min: 100 us  Max: 281 us  Samples: 83
  Long  Avg: 501 us  Min: 395 us  Max: 557 us  Samples: 54

ANALYSIS:
  Short Diff: -9 us
  Long Diff: +1 us
  Short Jitter: 181 us
  Long Jitter: 162 us

CONCLUSION:
  Short: EXCELLENT
  Long: EXCELLENT
  Timing matches fob!
  No changes needed.

PULSE CLUSTERS (per level; jitter = max - min):
  HIGH     256 us  x41    min    227  max    281  jitter    54    1 TE
           515 us  x26    min    493  max    557  jitter    64    2 TE
          6445 us  x2     min   6439  max   6451  jitter    12   25 TE  (minor)
  LOW      242 us  x36    min    214  max    272  jitter    58    1 TE
           493 us  x27    min    472  max    516  jitter    44    2 TE
         15545 us  x3     min  13486  max  16966  jitter  3480  63.5 TE  (minor)

Base TE estimate: 251 us (HIGH 257 us, LOW 245 us)
  82% of durations are 1-4 x TE
Line coding guess: Manchester / bi-phase (HIGH and LOW both 1 TE and 2 TE, long-long pairs occur)
```

## Limits

- The analysis sees what the demodulator produced. An FSK fob recorded with an OOK preset, or a
  capture of noise, gives clusters but no meaningful TE.
- The line-coding guess is a heuristic for research, not a decode. Differential Manchester, bi-phase
  and Manchester are not told apart, and a capture with polarity flipped swaps "PPM" and "PWM with fixed gap".
- Everything is computed from the stored pairs. Live-capture timing is only as good as the receive
  path; nothing here has been checked against hardware, only against IMPORTS `.sub` files and synthetic
  pulse trains.
