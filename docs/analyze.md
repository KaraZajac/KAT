---
layout: default
---

# Analyze

**Rust modules:** `src/analysis/signal.rs` and `src/analysis/signal/` (the report), `src/analysis/timing.rs`
(the measurements); overlay: `src/app/analyze_view.rs`, `src/ui/analyze_overlay.rs`  
**Reference:** ProtoPirate's Timing Tuner for the protocol comparison
(`REFERENCES/ProtoPirate/scenes/protopirate_scene_timing_tuner.c` and
`scenes/plugins/protopirate_timing_tuner_plugin.c`); rtl_433's pulse analyzer for the width clusters

Analyze takes the pulses of one capture and works out what the signal is: how it is keyed (OOK or
FSK), its pulse widths and base time element (TE), its line coding, its frames and the bits in them.
It works on every capture that has pulses, decoded or not, and is where to start with a signal no
decoder knows. For a decoded capture it also compares the received timing with the protocol's
definition, the way ProtoPirate's Timing Tuner does. Earlier versions of KAT called that part
"Timing analysis"; it is now one section of this report ([Timing analysis](timing_analysis.html)).

## Use

- `a` on a capture in the list.
- Signal menu (Enter on a capture): **Analyze**, after the exports. Unknown captures have it too, and
  so do receive-only radios.
- Command: `:analyze` analyses the selected capture, `:analyze 7` capture 7. `:timing` is the older
  name and still works.

The report opens in an overlay. Up/Down, PgUp/PgDn and Home/End scroll; Left/Right show the capture
before or after this one (captures without pulses are skipped); Esc, Enter or `q` close. The list
selects the capture the report shows, so closing it leaves the list there. The title holds the
verdict, e.g. `Analyze: #26 Unknown (OOK, PWM, TE ~479 us)`.

A capture without recorded pulses has nothing to analyse: a `.fob` exported without them, or a remote
made with `:new`. Its menu has no Analyze entry and `:analyze` says so.

## The report

| Section | What it holds |
|---|---|
| SUMMARY | the verdict in one line (keying, coding, TE), the frames and how they repeat, the capture's length |
| MODULATION | how the signal is keyed and how KAT knows; for a HackRF or RTL-SDR capture the measured frequency, tone distance, level and burst length |
| PULSE WIDTHS | each level's widths with count, range and multiple of the TE; then the TE, the skew, the symbol rate and the coding |
| ROWS | the capture cut at its pauses: preambles, rows of bits, long pulses; which rows repeat and how often |
| BITS | a trial decode of the rows of bits, in hex and binary |
| WAVEFORM | the frame drawn, one column per TE |
| PROTOCOL TIMING | decoded captures: the received widths against the protocol's definition |
| SIMILAR TIMING | undecoded captures: KAT's protocols with widths near these |
| RAW | the frame's durations in µs |

The report is laid out for the overlay's width (54 to 110 columns), so nothing is cut at 60×16.

### Summary

`Looks like` is the verdict: keying, line coding and TE, or what there is instead (`carrier without
on/off data`, `no regular pulse train`). `Frames` counts the rows of bits: `4 frames of 29 bits, all
identical, every 71.1 ms` for a fixed code sent four times, `all different` or `3 distinct` when the
bits change between rows, `2 rows of bits: 9 and 80 bits` when the rows have nothing in common.
`Length` is the capture from its first pulse to its last.

### Modulation

A file of pulses or a Flipper gives durations only, so how sure the keying is depends on where the
capture came from:

| Capture from | Keying |
|---|---|
| HackRF or RTL-SDR, live or from an IQ file | **measured.** KAT demodulates every burst twice, by its envelope (AM path) and by its frequency (FM path). Two distinct tones give `2-FSK: two tones +/-47.6 kHz around the centre (95.2 kHz apart)`; one carrier switched on and off gives `OOK / ASK: one carrier switched on and off (no second tone found)` |
| a `.sub` recording | **by the recording's preset**: `OOK by the recording's preset (AM650)`, or 2-FSK with the preset's deviation. That is what the recorder was set to, which need not be what the transmitter does |
| a Flipper as the radio, or a session saved by an older KAT | **by the receive path** the pulses came from (AM: OOK, FM: 2-FSK), or `unknown` when nothing says |

For a measured capture the section goes on with:

- `Centre` (FSK) or `Carrier` (OOK): where the signal sat, e.g. `433.933 MHz (+12.5 kHz from the tuned
  433.920 MHz)`. Remotes are often tens of kHz off their nominal frequency.
- `Level`: dB over the noise floor, `strong` from 20 dB, `fair` from 10 dB, else `weak`.
- `Burst`: from the first carrier to the last.

`Path` says what HIGH and LOW mean in the rest of the report: `AM (envelope): HIGH is carrier on, LOW
is carrier off`, or `FM (frequency): HIGH is the upper tone, LOW the lower one`.

Notes follow where they apply:

- An FSK burst seen through the AM path: its envelope is the carrier coming and going, not the
  data. The FM capture of the same burst holds the data, and is named (`capture #12`) when it is
  still in the list.
- The other way round: an FM capture whose burst also gave pulses on the AM path names that capture.
- **Carrier blocks** in an undecoded AM capture: pulses of 5 ms or more hold over 60 % of the carrier
  time. In a recording that is most likely an FSK transmitter received as OOK: record it again with
  an FSK preset (FM476, FM238), or with a HackRF or RTL-SDR. In a measured capture KAT found no
  second tone, so it is plain carrier, or FSK narrower than KAT looks for.
- `KAT finds no regular pulse train here`: noise, a signal too weak or too short, or a coding KAT
  does not know.

### Pulse widths

The first line says which durations were analysed: all of them, the decoded frame (a decoded
capture: its last `2·min_count_bit + 32` pairs, at most 512, ending where the decoder completed the
frame), or the most regular 512 (an undecoded capture longer than that; windows are tried in steps
of 128, and one with data in it wins over one that is all preamble).

HIGH and LOW durations are clustered separately the way rtl_433's pulse analyzer does it: a duration
joins the nearest cluster within ±20 %, and clusters that drift within 20 % of each other are fused.
Each width is listed with its count, its range, its multiple of the TE and a bar. A cluster holding
at least 8 % of its level's durations is a data width; the others (pauses, sync pulses, noise) are
summed up in one line.

- **TE.** For each level, the data width whose 1–4 multiples (within ±0.3) explain the most durations,
  refined by the least-squares fit `TE = Σ n·mean·k / Σ n·k²`. When HIGH and LOW agree (within ×1.6)
  the TE is their mean. The line gives both and the share of durations that are 1–4 × TE.
- **Skew.** A receiver often makes every HIGH longer and every LOW shorter by one constant (an AM
  envelope detector), or the other way round (an FSK slicer off the middle of the two tones). From
  10 µs on the report says so. When the skew is so large that neither level fits a TE of its own
  (235 and 575 µs HIGH with 470 and 815 µs LOW are a TE of 350 µs skewed by −115 µs), the TE and the
  skew are fitted together.
- **Symbol rate**: one symbol per TE, in baud.
- **Coding**: the guess below, with the bit rate it implies.

| Coding | Rule |
|---|---|
| PWM | two widths per level; short+long pairs, and long+long pairs are rare (≤ 5 %) in one pairing order. The bit period is the mean pair length |
| PWM, equal-length pairs | short+short and long+long pairs only (mixed ≤ 5 %): the pair length carries the bit (Kia V0, Suzuki V0) |
| Manchester / bi-phase | both widths about 1:2 on both levels, and either mixed and long+long pairs of 10 % or more in both orders, or the short pulses between two long ones come in twos nine times in ten. Differential Manchester looks the same at this level |
| NRZ / PCM | nine in ten durations are a whole number of TEs, a tenth of those three TEs or more; or three or more widths on a level, all multiples of the TE |
| PWM with fixed gap | two pulse widths, one gap width |
| PPM / pulse-distance | one pulse width, the gap width carries the bit |
| single width | one pulse and one gap width: a preamble, or carrier bursts |

Pulses are paired with the gap after them and with the gap before them, because an inverted capture
pairs the halves of neighbouring bits. Runs of eight or more identical pairs are preamble and do not
count.

### Rows

A pause ends a row. The pause is at least 2.5 times the widest data width and 6 TE (40 TE for NRZ,
whose runs of equal bits are long), between 600 µs and 80 ms; without a TE it is 5 ms. On the FM path
a long HIGH is a pause as well: a tone held, not data. Each row is one of:

| Content | Meaning |
|---|---|
| `29 bits` | the trial decode read it. `same as #2` when an earlier row has the same bits, `3 unlike #2` when it is as long but differs in 3 bits; `12 preamble + 64 bits` when it starts with a run of one bit value |
| `one width: 484/482 us` | pulses of one width per level: a preamble |
| `long pulse: 5373 us` | a lone pulse of 4 TE or more (at least 1 ms): carrier, a wake-up or a sync pulse |
| `not read as bits` | regular pulses the trial decode cannot read |

Up to twelve rows are listed: the rows of bits first, then preambles, then the rest, in the order
sent, so the row numbers can have gaps. Pieces that are none of these (noise, broken rows) are only
counted. When the rows come round again the report says how many times and how far apart: `Rows #1
to #2 come round 4 times, 71.1 ms from one to the next.`

### Bits

The rows of bits, one per distinct content (up to four), in hex and in binary with the first bit
sent on the left; the last hex digit is padded with zeros, and a row of more than 192 bits is shown
in hex only. One convention per coding, stated under the bits:

| Coding | Read as |
|---|---|
| PWM, PWM with fixed gap | a long pulse is 1, a short pulse is 0 |
| PWM, equal-length pairs | a long pulse and gap is 1, a short pair is 0 |
| PPM | a long gap is 1, a short gap is 0 |
| Manchester | HIGH then LOW is 1, LOW then HIGH is 0; a leading run of one value is left out as preamble |
| NRZ | HIGH is 1, LOW is 0, one bit per TE |

A protocol may use the opposite, or code the bits again (differential codes, whitening). These are
the bits as sent: enough to compare presses, find a counter that moves and a serial that does not.

### Waveform and raw durations

`WAVEFORM` draws the longest row of bits (the frame), one column per TE: `▀` is HIGH, `▁` is LOW.
`RAW` gives the same row's durations in µs, `+` for HIGH and `-` for LOW, as a `.sub` file holds
them. Both stop after a few lines. Without a row of bits they show the first row of several pulses,
or the capture's start.

### Protocol timing (decoded captures)

The received short and long widths against the decoder's definition, with the verdicts of
ProtoPirate's Timing Tuner: `EXCELLENT` within 15 µs, `OK` within the protocol's tolerance, else
`HIGH by` or `LOW by` so many µs, and a conclusion from "Timing matches fob!" to "NEEDS ADJUSTMENT".
A protocol sent with more than one timing (VAG, Fiat V1) is compared with the one the capture fits.
[Timing analysis](timing_analysis.html) has the rules and the results on real captures.

### Similar timing (undecoded captures)

When the capture is a regular pulse train, the report lists up to five of KAT's protocols whose
`te_short` and `te_long` are both within their own tolerance of the measured widths, nearest first,
with their coding and RF. It is a pointer, not a decode: many protocols share 500/1000 µs. The last
line is the measured pair as a starting point for a new decoder, `As a protocol te_short ~479 us,
te_long ~958 us`.

## Example

A made-up remote: PWM with a TE of 480 µs, a preamble of twelve short pulses, 28 bits and a stop
pulse, sent four times; written as a Flipper `.sub` recording with the AM650 preset and loaded with
`:load`. No decoder knows it. The overlay's title is `Analyze: #26 Unknown (OOK, PWM, TE ~479 us)`.

```
Signal #26  Unknown  433.92 MHz  2026-11-10 00:15:31 UTC-05:00
made_up_remote.sub

SUMMARY
Looks like   OOK, PWM, TE ~479 us
Frames       4 frames of 29 bits, all identical, every 71.1 ms
Length       269.8 ms with 168 pulses

MODULATION
Keying       OOK by the recording's preset (AM650)
A file of pulses holds no RF measurements: the keying is what the recorder
was set to, which need not be what the transmitter does.
Path         AM (envelope): HIGH is carrier on, LOW is carrier off

PULSE WIDTHS   all 336 durations
     width   count       min..max    x TE
HIGH (carrier on): 168 durations
    482 us    x104       440..519     1.0  ██████████
    955 us     x64      892..1014     2.0  ███████
LOW (carrier off): 168 durations
    480 us    x112       441..514     1.0  ██████████
    958 us     x48      899..1006     2.0  █████
     + 2 other widths (8 durations): pauses, sync or noise
TE           ~479 us (1 TE is 479 us HIGH, 479 us LOW); 98% of the durations
             are 1-4 x TE
Symbol rate  2087 baud (one symbol per TE)
Coding       PWM (pulse width carries the bit; short+long pairs, bit period
             ~1438 us); 695 bit/s

ROWS   a pause of 2874 us or more ends a row
  #        at  pulses      then  content
  1      0 us      13   3885 us  one width: 484/482 us
  2   15.8 ms      29   14.7 ms  29 bits
  3   71.2 ms      13   3949 us  one width: 481/487 us
  4   87.1 ms      29   14.2 ms  29 bits, same as #2
  5  142.3 ms      13   3913 us  one width: 489/482 us
  6  158.2 ms      29   14.4 ms  29 bits, same as #2
  7  213.3 ms      13   3751 us  one width: 477/480 us
  8  229.1 ms      29         -  29 bits, same as #2
Rows #1 to #2 come round 4 times, 71.1 ms from one to the next.

BITS   a trial decode, the first bit sent on the left
#2   29 bits
     hex  A5C3 F170
     bin  10100101 11000011 11110001 01110
Read as: a long pulse is 1, a short pulse is 0. A protocol may use the
opposite, or code the bits again.

WAVEFORM   row #2, one column is 479 us
▀▀▁▀▁▁▀▀▁▀▁▁▀▁▁▀▀▁▀▁▁▀▀▁▀▀▁▀▀▁▀▁▁▀▁▁▀▁▁▀▁▁▀▀▁▀▀▁▀▀▁▀▀▁▀▀▁▀▀▁▀▁▁▀▁▁▀▁▁▀▀▁▀▁▁
▀▀▁▀▀▁▀▀▁▀

SIMILAR TIMING   KAT's protocols near these widths; not a decode
BMW V2       500/1000 us  Manchester        AM
CAME TWEE    500/1000 us  Manchester        AM
Hormann HSM  500/1000 us  PWM               AM
Kia V2       500/1000 us  Manchester        AM/FM
Mazda V2     500/1000 us  Diff. Manchester  AM/FM
As a protocol te_short ~479 us, te_long ~958 us

RAW   row #2 in us: + is HIGH, - is LOW
+940 -504 +488 -994 +942 -469 +475 -956 +489 -967 +947 -466 +472 -995 +936
-483 +972 -458 +961 -498 +450 -950 +478 -936 +487 -958 +458 -983 +979 -493
+1001 -485 +963 -461 +977 -471 +946 -461 +932 -472 +498 -901 +459 -966 +500
-976 +905 -443 +485 -938 +927 -494 +991 -482 +967 -486 +502
```

What the report got right without knowing the protocol: the keying is the preset's (and it says a
recording cannot tell more), the TE is 479 µs for the 480 µs sent, the coding is PWM, each frame is a
preamble row and a row of bits, and the four frames are the same. The 29th bit is the stop pulse: a
short pulse before the pause reads as one more 0. Where a frame starts and ends is the protocol's
business, which is why the bits are called a trial decode.

## Limits

- The report sees what the receiver produced. A Flipper or a `.sub` recording gives one path only, so
  an FSK remote recorded with an OOK preset shows carrier blocks and no data, and a capture of noise
  gives widths but no TE.
- OOK and 2-FSK are the keyings KAT's receive paths tell apart. ASK with several levels, 4-FSK, PSK
  and spread-spectrum signals are not recognised as such.
- The line coding is a guess for research, not a decode. Manchester, differential Manchester and the
  other bi-phase codes look the same at the pulse level, and a capture with its polarity flipped
  swaps "PPM" and "PWM with fixed gap".
- An undecoded capture of at most 512 durations is analysed whole, so a short frame among a lot of
  noise can be out-voted by the noise.
- Rows end at pauses. A remote that sends its frame again without one (the next preamble straight
  after the last bit, as VAG's Type 1/2 does) gives one long row, and its bits are those of several
  frames run together.
- The trial decode reads each row by one convention. Bit order, inversion, whitening, checksums and
  rolling codes are left to the reader.
- Everything is computed from the stored pulses. The measurements of a live capture (centre, tones,
  level) are taken once, when it is received, and are saved with the session; a session saved by an
  older KAT has none.
