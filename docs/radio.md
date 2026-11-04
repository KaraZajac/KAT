---
layout: default
---

# Receive chain, IQ files and frequency hopping

**Rust modules:** `src/radio/pipeline.rs` (receive pipeline), `src/radio/dsp.rs` (filters, mixer,
DC removal), `src/radio/rx.rs` (live-receiver glue), `src/radio/hopper.rs`, `src/radio/iq_file.rs`,
`src/radio/recorder.rs`, `src/app/rx.rs` (commands); tests in `src/radio/sim.rs`.
**References:** Flipper-ARF `lib/subghz/subghz_setting.c` (hopper list), ProtoPirate
`helpers/protopirate_txrx.c` (`protopirate_hopper_update`).

Since KAT 1.5 the HackRF, the RTL-SDR and IQ files all go through one receive pipeline, so a
recording decodes exactly like the live signal it came from. The pipeline finds a fob anywhere in
the band it receives, not only at the frequency you tuned to: cheap SAW-resonator remotes are often
75–150 kHz off, and a Flipper Zero (OOK650 preset) receives ±325 kHz.

## Overview

```text
device / file ──► raw IQ (cs8 HackRF, cu8 RTL-SDR, cf32)
   ├──► :record (raw bytes, writer thread)
   └──► DC removal ─► [> 2.5 MS/s: mixer + FIR ÷D] ─► wide-band buffer
          └──► detector: FFT, ~31 kHz bins, noise floor per bin ─► bursts, each on its own carrier
                 └──► per burst: mixer (carrier → 0 Hz) ─► FIR ÷4 ─► channel FIR ÷2 ─► ~250 kS/s
                        └──► slicer, pass 1 (20 µs) ─► frames, element length, carrier offset / tones
                               └──► pass 2 per frame (≈ TE/5) ─► AM pairs / FM pairs ─► captures
```

Captures carry the **target** frequency (the one you tuned to or that the hopper was on), and
`received_rf` = AM or FM for the path that produced them. The burst's measured carrier offset from
the target is in the debug log and in `kat --decode --verbose`.

## The pipeline

**Offset tuning.** HackRF and RTL-SDR both have a spike at 0 Hz (DC offset, LO leakage, 1/f noise).
Live receivers tune the LO **fs/8 below the target** (250 kHz at 2 MS/s), so the target is clear of
the spike. The detector leaves the three bins around the LO out (an OOK fob within about ±15 kHz of
it, 250 kHz below the target, isn't heard; FSK tones ±47.6 kHz either side of it are) and the band
edges beyond ±0.45·fs (filter roll-off, aliases). At
2 MS/s a fob is found from **1.15 MHz below to 0.65 MHz above the target**, which includes the
±325 kHz of a Flipper's OOK650 window. The HackRF's baseband filter is set to 1.75 MHz
(±875 kHz around the LO), the edge of that band.

**Files** recorded at the fob frequency (no offset) are searched over their whole band, around the
circle: a fob at the Nyquist edge of a 250 kS/s rtl_433 recording is found too. KAT's own
recordings (`rx<target>Hz` in the name) are offset-tuned and handled like live RX. Input faster
than 2.5 MS/s is first mixed (target → 0 Hz) and decimated to ≤ 2.5 MS/s (searched band ±0.35 of
that rate, ±875 kHz at 10 MS/s input).

**DC removal.** The mean of each ~2 ms block is tracked and subtracted, but learned only while no
burst is open, so a fob sitting exactly at 0 Hz in a file recorded at its own frequency is not
removed with it.

**Detection.** Hann-windowed FFT frames (64 points at 2 MS/s: 31.25 kHz bins; 8 at 250 kS/s), 50 %
overlap, eight frames per batch through one vectorised FFT. The power of each bin is averaged over
256 µs blocks (16 frames) and compared with a **noise floor per bin**: the median of the first
blocks after start or retune, then tracked while no burst is open (a strong burst can move the
whole band: an RTL-SDR's AGC drops the gain while it is on); one floor set per hop frequency,
re-learned after a gain change. A bin more than **5.4 dB** over its floor is "carrier present"
(90 s of pure noise at three levels: not one bin-block got there). The bins don't need to be
flat: a steady spur, the DC residue or an interferer only raises its own bin's floor.

- **Signals.** Present bins up to ~100 kHz apart are one signal (both tones of ±47.6 kHz FSK),
  unless a separate power peak stands out (≥ 6 dB over the dip towards a stronger peak more than
  ~100 kHz away): a fob next to the skirt of a strong carrier. Signals further apart are separate
  bursts, so **two fobs on different frequencies at the same time** are two captures (synthetic
  test: 300 kHz apart; a fob 300 kHz from a carrier 15 dB stronger that is on all the time). A
  burst follows its carrier as it drifts (the Chrysler remotes in the rtl_433 tests drift 40 kHz
  during a press) and a transmitter that hops (the Chamberlain remote in the rtl_433 tests sends
  each press on two frequencies 500 kHz apart) gives a burst per frequency.
- **Burst end.** A burst closes after **80 ms without carrier in its bins** (one multi-frame button
  press stays one capture, noise between bursts can't keep it open). It needs ≥ 4 present blocks
  (1 ms). One that never drops back to its floor for 3 s is cut and those bins re-baselined
  (a steady interferer, a gain step).
- **Artifacts.** A strong fob shows up weakly elsewhere in the band: its I/Q-imbalance image
  (mirrored around the LO, HackRF), spurs and intermodulation of the receiver or the transmitter
  (the HCS362 and Chrysler recordings show lines 16–31 dB down). A burst ≥ 10 dB weaker than
  another is that one's artifact, and dropped, if it is keyed with it: ≥ 90 % of its present blocks
  are the other's, and the other is keyed over its span, or their block levels are correlated
  (≥ 0.5), or the other is on throughout and starts and ends with it (a steady FSK fob and its
  spur). A fob keyed under a carrier that is simply on all the time is none of these. A burst whose
  carrier ends up within 50 kHz of a stronger burst's at the same time (a weak line of the same
  transmitter, re-centred onto it by the slicer) is a duplicate and dropped too.
- **Carrier.** The centre of the burst's bins, weighted by how much their power changed while it
  was on: the OOK carrier, or (for FSK) between the two tones. A steady interferer next to it
  doesn't change and doesn't pull the estimate; a carrier that was already on when the floor was
  learned (a recording starting mid-transmission) shows as a drop and still counts. In a file
  recorded at the fob frequency, a bin learned > 10 dB above the median bin is taken for such a
  transmission rather than for background.

**Channel, per burst.** When a burst closes, its samples (kept in the wide-band buffer since just
before it opened) are mixed so its carrier is at 0 Hz and filtered: a cheap first stage to
~500 kS/s (pass ±120 kHz) and the channel stage to ~250 kS/s: **pass ±100 kHz, stop ±150 kHz,
60 dB**, so ±47.6 kHz FSK and carrier drift within the burst fit. The rate is chosen per input rate
(2 MS/s → 250 kS/s, 2.048 → 256, 2.4 → 240, 1 → 250, 0.92 → 230, 10 → 250 kS/s). The noise level
for the slicer is measured on the quiet channel samples just before and after the burst, but at
most that of the burst's own quietest stretches (OOK gaps): what was around the burst may not be
there while it is on. It is scaled by 0.92, the bias of the 1.5 squelch's floor tracker that the
slicer's thresholds were tuned with (0.3 dB of OOK sensitivity at the limit). If the slicer then finds the carrier (or the middle of the FSK tones) more
than 20 kHz from the centre, the burst is filtered and sliced again around it; `offset_hz` is the
carrier's offset from the target. At 250 kS/s one sample is 4 µs; edges are interpolated between
samples, so durations come out within a few µs at reasonable SNR (shortest TE in the registry:
Mercedes V1, 50 µs).

**Slicing (per burst).**

- *Pass 1:* AM: |z| averaged over 20 µs, threshold halfway between the noise level and the carrier
  level (iterated mean of the samples above the midpoint), 10 % hysteresis, edges at the
  interpolated midpoint crossing, levels shorter than 30 µs dropped. FM: the discriminator
  `arg Σ z[n]·z*[n−1]` over 20 µs (summing before `arg` suppresses noise clicks); two-means clustering
  of the carrier-on samples finds **both tones**, so any remaining carrier error doesn't matter
  (per-burst AFC); a burst is FSK only if the tones are ≥ 4 kHz apart and ≥ 3.5× their spread; the
  slicer cuts halfway between them with hysteresis of 10 % of the separation; outside the carrier the
  level is low.
- *Pass 2:* the burst is split into frames at lows of 6 ms or more. For each frame the element
  length TE is estimated from pass 1 (10 % time-weighted quantile of its level durations); if it is
  ≥ 100 µs the frame is sliced again with ~TE/5 averaging and TE/4 debounce. For OOK the frame's
  carrier is first mixed to 0 Hz (its offset measured from pass 1) and averaged **coherently**, which
  averages the noise away before the envelope is taken, close to a matched filter; for FSK the
  discriminator is averaged and the two tones are found again per frame. Short-TE frames keep
  pass 1, and a burst that also holds something else (another fob, interference) doesn't distort
  the TE estimate of the frame next to it.

**Output.** Level/duration pairs in µs, `true` = carrier on (AM) or upper tone (FM). Each capture
starts with a low for the idle time before it (10 ms … 1 s) and ends with the trailing gap (≥ 80 ms),
which decoders need to start and finish frames. An FSK burst is not also emitted as an AM "carrier
on/off" capture and an OOK burst not as FM noise: the AM path is kept for an FSK burst only if it has
real on/off keying (≥ 4 carrier pulses shorter than 5 ms). Bursts with fewer than 5 pairs are dropped.

**RSSI** (the bar on the right) is the peak carrier magnitude in any detector bin per buffer,
full scale = 1.

**Threads.** The HackRF's USB callback only records the buffer and queues it (with any retune
point) for a processing thread, like the RTL-SDR reader; slicing a long burst can take longer than
libhackrf's transfers can wait (4 × 65 ms). The queue holds ~2 s; if processing falls that far
behind, buffers are dropped (logged) rather than stalling USB.

## IQ files

Supported: `.cs8` (signed 8-bit I/Q, `hackrf_transfer`), `.cu8` (unsigned 8-bit, `rtl_sdr`,
`rtl_433 -w`), `.cf32` / `.cfile` / `.complex` (float32, GNU Radio, URH). Files are processed on a
background thread; progress shows in the status-bar title and the result in the status line. Captures
go through the normal decode / de-duplication / research-mode path, with or without a radio.

```sh
# Record (fob at 433.92 MHz)
hackrf_transfer -r fob.cs8 -f 433920000 -s 2000000 -l 24 -g 20   # HackRF, 2 MS/s
rtl_sdr -f 433920000 -s 2048000 -g 30 fob.cu8                     # RTL-SDR, 2.048 MS/s

# Open
kat --iq fob.cs8 --freq 433.92 --rate 2M
```

In KAT: `:iq <file> [freq MHz|Hz] [rate]` (paths may be absolute, relative to the working directory
or to the import/export directories; `:iq stop` cancels), or pick the file in the `:load` browser.
Frequency and rate default to what the file name says, else the current frequency and 2 MS/s.
Understood names: KAT's own (below), rtl_433 (`g001_433.92M_250k.cu8`), gqrx-style bare numbers
(`…_433920000_2000000_…`), and tokens like `433.92MHz`, `2Msps`, `2.4M`, `250k`.

**`:record [file]` / `:record stop`** writes the IQ exactly as received (HackRF `.cs8`, RTL-SDR `.cu8`)
to the export directory, on a writer thread (the receive path never waits for the disk; if the disk
falls 4 s behind, buffers are dropped and counted). The header shows `● REC <size>`. Recording starts
RX if needed; it stops with RX (or a retune) because a file has one centre frequency, and it can't be
combined with hopping. The file name carries the **true centre frequency** of the samples (the LO,
fs/8 below the target) so other tools, and `hackrf_transfer -t … -f <that frequency>`, see the right
frequency; `rx<target>Hz` tells `:iq` which channel to decode:

```text
kat_433670000Hz_2000000sps_rx433920000Hz_20261104-153000.cs8
```

## Frequency hopping

`:hop on` / `:hop off` / `:hop` (toggle), `:hop 433.92 434.42 868.35` (set the list and start),
`:hop dwell 150`, `:hop default`; or the **Hop** setting next to Freq (Tab). The header shows
`HOP 433.92 MHz (4/7)` with the frequency being received.

- Default list: Flipper-ARF's hopper list 315.00, 390.00, 430.50, 433.92, 434.25, 434.42,
  868.35 MHz; dwell 100 ms (ProtoPirate's tick). Config (`[radio]`): `hop_frequencies` (MHz, comma
  separated) and `hop_dwell_ms`.
- Behaviour follows ProtoPirate's `protopirate_hopper_update`, with the burst detector as the RSSI
  test: a quiet frequency is left after one dwell; while a burst is open anywhere in the band the
  hopper holds, then holds 10 dwells more (ProtoPirate's RSSI time-out); after a decode it pauses
  50 dwells (ProtoPirate's "key found" pause). `:freq` (or the Freq setting) switches hopping off.
- Retuning keeps the stream running. HackRF: `set_freq` while streaming; the receive callback works out
  from the buffer arrival times which sample the new frequency starts at and drops 2 ms before to
  10 ms after that point (HackRF buffers are 65 ms long, so the switch can't be seen directly).
  RTL-SDR: the reader thread retunes between 16 ms reads and tags every buffer with its frequency,
  dropping the first 6 ms after a retune. Each capture carries the frequency it was heard on.

## Measurements (synthetic IQ)

Synthetic IQ as in the v1.5 radio lab: frames from KAT's encoders (or real `IMPORTS` RAW timings),
OOK or 2FSK at a fob frequency offset, random carrier phase, continuous Gaussian noise before and after
the burst, int8 quantisation, A = 60 LSB, SNR quoted over the full 2 MHz as `10·log10(A²/2σ²)`, target
at +fs/8 as in live RX; 6 trials per cell. Reproduce with
`cargo test --release --offline sweep_ -- --ignored --nocapture --test-threads 1`.

### Fobs off frequency

Where 50 % of the presses decode, in dB over 2 MHz (lower is better), against the fob's offset from
the target; live tuning (target at +fs/8), A = 20 LSB (no int8 clipping), 8 trials per SNR step,
full-length frames (`sweep_offset_sensitivity`). "Before" is KAT 1.5 with the ±100 kHz channel;
"–" = no decode even at +4 dB:

| 50 % point, dB over 2 MHz | −325 | −300 | −200 | −100 | 0 | +100 | +200 | +300 | +325 kHz |
|---|---|---|---|---|---|---|---|---|---|
| Kia V1 (OOK), before | – | – | – | −6.4 | −8.5 | −6.4 | – | – | – |
| Kia V1 (OOK), now | −8.6 | −8.5 | −8.6 | −8.4 | −8.5 | −8.4 | −8.4 | −8.4 | −8.4 |
| Kia V5 (FSK ±47.6 kHz), before | – | – | – | −7.7* | −5.8 | – | – | – | – |
| Kia V5 (FSK ±47.6 kHz), now | −5.6 | −6.0 | −6.0 | −6.0 | −6.2 | −6.8 | −6.4 | −6.0 | −6.3 |
| Ford V0 (FSK ±20 kHz), before | – | – | – | – | −6.5 | – | – | – | – |
| Ford V0 (FSK ±20 kHz), now | −6.5 | −6.5 | −6.6 | −6.7 | −6.5 | −6.6 | −6.5 | −6.5 | −6.5 |

\* only the upper tone was in the channel and decoded as OOK.

At 0 Hz, 24 trials and 0.5 dB steps: OOK −8.6 dB (before −8.6), FSK ±47.6 kHz −6.3 (−6.2),
FSK ±20 kHz −6.7 (−6.7). In none of these is detection the limit: the bursts are still found 3–5 dB
below (FSK at −11 dB), it is the slicer's pass 1 / element-length estimate that gives up.
The blind spot at the LO (target − 250 kHz): OOK doesn't decode within about ±15 kHz of it and
needs ~4 dB more within ±25 kHz; ±47.6 kHz FSK decodes right through it.

### 2FSK

Decode rate vs fob frequency offset, SNR 6 dB (the 1.4 FM demodulator decoded none of these once
there was any noise, and none with an offset larger than the deviation):

| ±20 kHz deviation | −40 kHz | −20 | −10 | 0 | +10 | +20 | +30 | +40 kHz |
|---|---|---|---|---|---|---|---|---|
| Kia V5  | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |
| Kia V3  | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |
| Ford V0 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |

| ±47.6 kHz deviation | −40 kHz | −20 | −10 | 0 | +10 | +20 | +30 | +40 kHz |
|---|---|---|---|---|---|---|---|---|
| Kia V5  | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |
| Kia V3  | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |
| Ford V0 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |

Decode rate vs SNR (dB over 2 MHz), fob 20 kHz off (KAT 1.4 row: fob exactly on frequency):

| ±20 kHz deviation | 12 | 6 | 3 | 0 | −3 | −6 | −9 |
|---|---|---|---|---|---|---|---|
| Kia V5  | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 5/6 | 0/6 |
| Kia V3  | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 0/6 |
| Ford V0 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 0/6 |
| Kia V5, KAT 1.4 | 6/6 | 0/6 | 0/6 | 0/6 | 0/6 | 0/6 | 0/6 |

| ±47.6 kHz deviation | 12 | 6 | 3 | 0 | −3 | −6 | −9 |
|---|---|---|---|---|---|---|---|
| Kia V5  | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 4/6 | 0/6 |
| Kia V3  | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 0/6 |
| Ford V0 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 0/6 |
| Kia V5, KAT 1.4 | 6/6 | 6/6 | 0/6 | 0/6 | 0/6 | 0/6 | 0/6 |

### OOK

Decode rate vs SNR (dB over 2 MHz). The 16-tap row is the lab's moving average in front of the 1.4
demodulator (from the v1.5 brief, 10 kHz offset):

| case | 18.6 | 12.6 | 9 | 6.5 | 0.5 | −3 | −5.5 | −8 | −10 |
|---|---|---|---|---|---|---|---|---|---|
| Kia V1, 10 kHz offset | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 0/6 |
| Kia V1, 50 kHz offset | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 0/6 |
| Kia V1, 75 kHz offset | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 5/6 | 0/6 |
| Ford V0 (as OOK), 10 kHz | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 0/6 |
| Kia V1, 10 kHz, KAT 1.4 | 6/6 | 5/6 | 0/6 | 0/6 | 0/6 | 0/6 | 0/6 | 0/6 | 0/6 |
| Kia V1, 10 kHz, 1.4 + 16-tap (lab) | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 3/6 | – | – |

Below about −6 dB the int8 samples clip heavily (σ ≈ 1.6× the carrier amplitude), so the last
columns say as much about the test signal as about the receiver.

### Robustness

| test | result |
|---|---|
| static DC offset 0 / 10 / 30 LSB on I and Q, file (target at 0 Hz) and live (fs/8) tuning, Kia V1 OOK and Kia V5 FSK at 6 dB | 6/6 everywhere |
| pure noise, σ = 3 / 20 / 60 LSB, 30 s each at 2 MS/s | 0 bursts; not one bin-block over the threshold (normal suite: 5 s at 250 kS/s) |
| 2.048, 2.4 and 1 MS/s input (rtl_sdr rates) | decodes like 2 MS/s |
| FSK burst → no AM twin, OOK burst → no FM twin | checked in the normal suite |
| OOK and ±47.6 kHz FSK 150 and 300 kHz either side of the target, 6 dB | decode, offset within 5 kHz, deviation within 3 kHz (normal suite) |
| fob at the Nyquist edge of a 250 kS/s file (−121, +117, +124 kHz) | decodes (normal suite) |
| two fobs at once: OOK 150 kHz below the target, FSK 150 kHz above, overlapping, 10 dB | two captures, both decode (normal suite) |
| fob 300 kHz from a carrier 15 dB stronger that comes on before it and stays on | decodes (normal suite) |
| strong fob (30 dB) with a −30 dB I/Q image 540 kHz away | one capture (normal suite) |

### Real recordings (rtl_433 test files)

rtl_433's public test recordings (`rtl_433_tests`, real IQ from RTL-SDRs, `.cu8` at 250 kS/s to
2.048 MS/s, recorded at or near the remote's frequency) through `kat --decode`. Compared with KAT
1.5 before the wide-band receiver, every decode is the same, plus:

| recording | carrier offset | before | now |
|---|---|---|---|
| Microchip HCS200, BTN2 / BTN3 (6 files) | −116…−125 / +121 kHz (edge of the 250 kS/s band) | nothing | KeeLoq (Magic_3), serial D0921, counters AC4…ACC |
| Microchip HCS200, BTN1 (6 files) | −95…−112 kHz | KeeLoq, `g002` 1 frame | same, `g002` 4 frames |
| Chrysler car remote (7 files, 920 kS/s) | −74…−384 kHz | no burst beyond −126 kHz | every burst at its offset, 15–17 dB, clean 320 / 640 µs PWM (no KAT decoder for it) |
| Microchip HCS362 (868.3 MHz, 1 MS/s) | +280 kHz | 10 dB leakage of it | the bursts at +280.5 kHz, 38 dB |
| Chamberlain CWPIRC (915 MHz, 1 MS/s) | −283 and +217 kHz | noise-level bursts | each press on both of its frequencies, 25 dB |

Other things these recordings exercise: a transmitter keying its own carrier on and off around the
data (compustar), an RTL-SDR AGC that drops the gain during strong pulses (Chrysler), spurs and
lines of a strong transmitter 16–31 dB down (HCS362, Chrysler; dropped as artifacts or duplicates).
`rtl433_recordings_decode` (ignored; set `KAT_RTL433_TESTS` to the corpus's `tests` directory)
checks these decodes and bursts.

### Real pulse timings (IMPORTS)

Every `IMPORTS/*.sub` with decodes (38 files, 979 decodes in `testdata/imports_decodes.txt`):
each golden decode's RAW pulses are re-transmitted as a burst (OOK or 2FSK according to
`Capture::rf_modulation`, the recording preset for "both"), at 10 dB, 12 kHz fob offset, through the
pipeline and the registry. **972/979 decodes reproduced, 32/38 files identical** (971 / 31 with
the 1.5 channel). The 7 others:

- 3 × Kia V1 (`KIA V1 N1 RAW`, `KiaV1_5cl_5op`, `HYU V1 N1 RAW`): the recording ends on the frame's
  last low half-bit. Over the air that low runs into the silence after the transmission, and the
  Kia V1 decoder needs its length for the last bit; frames followed by another frame decode.
- 4 decodes (VAG `Golf4` × 3, two of them CRC-fail frames; Suzuki V0 × 1; plus two repeated
  Fiat V2 frames in the Chrysler file) depend on 50–130 µs spikes and drop-outs that the
  Flipper's receiver put into the RAW next to the frame. Re-transmitted they become real pulses,
  which pass 2 removes (shorter than TE/4), as a clean receiver would.

The re-transmitted pulses themselves come back within a few µs of the RAW values. The normal test
suite runs the first decode of 8 files (AM: Kia V1, Ford V0, PSA, VAG; FM: Kia V5, Kia V3, Honda
Static, Suzuki V0) at 1 MS/s and requires identical decodes.

### Throughput

One thread, release build, MS/s per CPU second (`sweep_throughput`, best of 5 on a machine shared
with other builds; "before" is KAT 1.5 with the streaming ±100 kHz channel):

| input | idle noise | burst-heavy signal (2.5 s of FSK bursts back to back) |
|---|---|---|
| 2 MS/s | 63 MS/s, 32× real time (before 62) | 20 MS/s, 10× (before 30) |
| 2.4 MS/s | | 23 MS/s, 10× (before 38) |
| 10 MS/s (files) | | 39 MS/s, 3.9× (before 99) |
| 250 kS/s (files) | 90 MS/s (before 22) | |

On idle noise the pipeline costs ~16 ns per 2 MS/s input sample, ~11 ns of it in the detector's
FFT; the channel filter (mixer and both FIR stages, ~12 ns per sample) now only runs over bursts,
on top of the detector, which is why burst-heavy input got slower. Finalising a burst (filtering and
slicing it) happens when it closes, on the processing thread: ~0.1 s for a 3 s burst. Live RX needs
a release build (a debug build is ~20× slower). `sweep_throughput` / `sweep_stage_costs` measure
yours.

**Not verified on hardware.** The synthetic measurements above and the rtl_433 recordings (real
RTL-SDR IQ) are all there is. Live HackRF/RTL-SDR operation with the wide-band detector (the LO / DC
bins, HackRF spurs and its I/Q image, an RTL-SDR's AGC live), the HackRF processing thread and
retune-boundary estimate, and real fobs' behaviour still need a check with the devices.
