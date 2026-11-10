---
layout: default
---

# Timing analysis

**Rust module:** `src/analysis/timing.rs`  
**Reference:** ProtoPirate's Timing Tuner, `REFERENCES/ProtoPirate/scenes/protopirate_scene_timing_tuner.c`
and `scenes/plugins/protopirate_timing_tuner_plugin.c`

ProtoPirate's Timing Tuner compares "real fob signal timing against protocol definitions": the
protocol's short and long widths and its tolerance against the average, minimum and maximum of what
was received, with a conclusion on whether the timing matches or needs adjusting. KAT runs the same
comparison on a decoded capture's pulses. It is the **Protocol timing** section of the
[Analyze](analyze.html) report.

The pulse widths, the TE estimate and the line-coding guess, which earlier versions of KAT showed
under this name as well, are the other sections of that report; they are described on the
[Analyze](analyze.html) page.

## Use

Press `a` on a decoded capture, choose **Analyze** in its Signal menu (Enter), or run `:analyze`
(`:analyze 7` for capture 7; `:timing` still works). The section is near the end of the report. For
`IMPORTS/FORD/2008_Focus_Lock.sub`:

```
PROTOCOL TIMING   Ford V0: 250/500 us, +/-120 us
         avg   min   max  count   diff
short    249   100   363     94     -1  EXCELLENT
long     508   399   691     51     +8  EXCELLENT
Timing matches fob! No changes needed.
Both levels count: short is under 375 us, long from there up; 100..740 us
is read, 15 left out.
```

The heading gives the definition: `te_short`/`te_long` and the tolerance `te_delta`. Each row is one
class of durations: the average, the shortest and the longest, how many there were, the average's
difference from the definition and the verdict. The last line says how the durations were sorted.

A decoded capture whose protocol KAT has no definition for says so, and an undecoded capture gets
the [Similar timing](analyze.html#similar-timing-undecoded-captures) section instead.

## The comparison

The definition is the decoder's `ProtocolDecoder::timing()` (`te_short`, `te_long`, `te_delta`).
Legacy and display names resolve the way the registry does (`PSA2` → PSA, `Kia V4` → Kia V3/V4,
`Keeloq (X)` → KeeLoq).

As in ProtoPirate's `calculate_timing_stats`:

- Durations of **both levels** count. The valid window is `te_short − 2·te_delta` … `te_long +
  2·te_delta`; durations outside it (gaps, noise) are left out.
- Valid durations below `(te_short + te_long) / 2` are **short**, the rest **long**. Each class gets
  its average (integer), minimum, maximum and count.
- **diff** = average − definition.
- Per class: **EXCELLENT** if |diff| ≤ 15 µs, **OK** if |diff| ≤ `te_delta`, else **HIGH by d** /
  **LOW by d**.
- Verdict: both EXCELLENT → "Timing matches fob! No changes needed."; both OK → "Within tolerance.
  Consider fine-tuning."; otherwise "NEEDS ADJUSTMENT:" with `Set te_short=<avg>` / `Set te_long=<avg>`
  for each class out of tolerance.
- Fewer than 10 valid durations: "Not enough samples".

KAT differs where real captures gave a reason:

| ProtoPirate | KAT | Why |
|---|---|---|
| Valid window floored at 100 µs | floored at `min(100, te_short / 2)` µs (identical for every `te_short` ≥ 200 µs) | the 100 µs floor drops every short pulse of Ford V1 (65 µs), Honda Static (63 µs), BMW (62 µs) and Mercedes V1 (50 µs), leaving nothing to analyse |
| The last 512 durations before the decode | the decoded frame: the last `2·min_count_bit + 32` pairs (at most 512), ending where the decoder completes the frame when the capture is decoded again | a `.sub` file's first decoded segment can be thousands of noise pulses followed by one frame (`FORD/0000_Ford_ts_x4.sub`: 6168 pairs), and a live burst goes on after the frame. With the frame window the Ford V0 comparison reads "Timing matches fob!" instead of noise averages |
| A class without samples prints "LOW by te" and `Set te_long=0` | "NO SAMPLES" and the verdict **INCOMPLETE** ("no long durations (>= N us) in the analysed pairs") | e.g. Honda Static's `te_long` is its 700 µs sync, which the frame does not contain |
| One definition per protocol | a protocol sent with more than one timing is compared with the one the capture fits | see below |

## Protocols with more than one timing

Two of KAT's protocols are sent with two timings, and `timing()` returns only one of them:

| Protocol | `timing()` | The other (`timing_variants()`) |
|---|---|---|
| VAG | 500/1000 µs ±80 (Type 3/4) | 300/600 µs ±100 (Type 1/2) |
| Fiat V1 | 250/500 µs ±100 (variant A) | 100/200 µs ±50 (variant B) |

Against the wrong one a capture has no "long" durations at all and comes out INCOMPLETE: half of the
VAG decodes in IMPORTS did. So the capture is compared with each definition and the report keeps
the one it fits: the better verdict, and between equal verdicts the averages nearer to the
definition. The heading shows the definition used, and a last line says there was a choice: `VAG is
sent with 2 timings: this is the one the capture fits.`

## Results on IMPORTS

`cargo test sweep_imports_timing -- --ignored --nocapture` analyses every IMPORTS decode the way
the app stores it (the decoded segment of each `.sub`), 1170 decodes in all:

| Protocol | Decodes | Matches | Within tol. | Needs adj. | Incomplete | Coding guess |
|---|---|---|---|---|---|---|
| Chevrolet V0 | 21 | 21 | 0 | 0 | 0 | Manchester 21 |
| Chrysler V0 | 20 | 0 | 14 | 6 | 0 | PWM 20 |
| Fiat V0 | 11 | 0 | 11 | 0 | 0 | Manchester 11 |
| Fiat V1 | 18 | 11 | 7 | 0 | 0 | Manchester 14, other 4 |
| Fiat V2 | 76 | 76 | 0 | 0 | 0 | Manchester 76 |
| Ford V0 | 141 | 135 | 6 | 0 | 0 | Manchester 141 |
| Ford V3 | 27 | 20 | 7 | 0 | 0 | Manchester 27 |
| Honda Static | 1 | 0 | 0 | 0 | 1 | Manchester 1 |
| Honda V5 | 40 | 40 | 0 | 0 | 0 | Manchester 40 |
| Kia V1 | 35 | 35 | 0 | 0 | 0 | Manchester 35 |
| Kia V3 / Kia V4 | 300 | 300 | 0 | 0 | 0 | PWM 300 |
| Kia V5 | 119 | 119 | 0 | 0 | 0 | Manchester 119 |
| Opel V1 | 5 | 5 | 0 | 0 | 0 | Manchester 5 |
| PSA | 15 | 0 | 15 | 0 | 0 | Manchester 15 |
| Renault V1 | 2 | 0 | 2 | 0 | 0 | other 2 |
| Subaru | 9 | 7 | 2 | 0 | 0 | PWM 9 |
| Suzuki V0 | 15 | 15 | 0 | 0 | 0 | PWM 15 (equal-length pairs) |
| Toyota | 53 | 0 | 53 | 0 | 0 | Manchester 53 |
| Toyota (FoxFW) | 16 | 0 | 16 | 0 | 0 | PWM 16 |
| VAG | 198 | 89 | 109 | 0 | 0 | Manchester 196, single width 1, PPM 1 |
| VAG Polo | 48 | 48 | 0 | 0 | 0 | Manchester 48 |

- **VAG**: 101 decodes fit the 500/1000 µs definition and 97 the 300/600 µs one. **Fiat V1**: 7 fit
  variant A and 11 variant B.
- **Chrysler V0**: a 1 is a 600 µs pulse and a 0 a 300 µs pulse, both below the short/long split, so
  the "short" average sits near 450 µs, at the edge of the ±150 µs tolerance. ProtoPirate's method
  has two classes; this protocol has four widths.
- **Honda Static**: its `te_long` is the 700 µs sync, which the frame does not contain.

The coding guess agrees with KAT's own encoding label (`Capture::modulation()`) for 1162 of the 1170
decodes, counting Toyota's differential Manchester as Manchester, which it is at the pulse level.
The other eight are Fiat V1 and Renault V1 frames of 100 µs pulses (six "other") and two VAG
decodes. A regular test (`imports_captures_match_their_protocol_timing`) checks Kia V1, Ford V0,
Subaru and Kia V3 captures against their definitions.

## Limits

- The comparison sees what the demodulator produced: a receiver that stretches every pulse moves the
  averages. The [Pulse widths](analyze.html#pulse-widths) section of the same report shows that as
  a skew.
- Two classes are all the method has. A protocol with more widths (a sync pulse as `te_long`, two
  pulse widths below the split) reads INCOMPLETE or NEEDS ADJUSTMENT without anything being wrong.
- Everything is computed from the stored pulses. The numbers above come from IMPORTS `.sub`
  recordings and synthetic pulse trains, not from KAT's own receivers.
