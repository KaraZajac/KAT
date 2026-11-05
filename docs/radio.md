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
                               └──► pass 2 per frame (≈ TE/5; OOK: carrier frequency tracked)
                                      ─► AM pairs / FM pairs ─► captures
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

**Dead input.** A run of identical samples lasting 1 ms or more carries neither signal nor noise:
an RTL-SDR's zero bytes before its first real buffer (seven of rtl_433's recordings start with
58–260 ms of them), a stalled stream. A live receiver always has some noise, so this never happens
with real samples. Neither the DC estimate nor the detector's noise floor is learned from it (they
used to be: the floor came out ~100 dB too low, every bin of the band was "present" from then on and
the whole file was one burst at "106 dB SNR").

**Detection.** Hann-windowed FFT frames (64 points at 2 MS/s: 31.25 kHz bins; 8 at 250 kS/s), 50 %
overlap, eight frames per batch through one vectorised FFT. The power of each bin is averaged over
256 µs blocks (16 frames) and compared with a **noise floor per bin**: the median of the first
blocks after start or retune, then tracked while no burst is open (a strong burst can move the
whole band: an RTL-SDR's AGC drops the gain while it is on); one floor set per hop frequency,
re-learned after a gain change. A bin more than **5.4 dB** over its floor is "carrier present"
(90 s of pure noise at three levels: not one bin-block got there). The bins don't need to be
flat: a steady spur, the DC residue or an interferer only raises its own bin's floor. **Noise
step:** when every bin of the band stands above its floor by about the same factor (within 9 dB of
each other) for 8 blocks (2 ms), the floor has moved, not a signal come on (a signal is peaked,
even a strong one's skirt is ≥ 20 dB below its carrier): a receiver still settling after it
started (`Microchip/HCS361/04/unlock` has ~6 ms of near-silent samples after its zero bytes, the
floor learned there was ~30 dB low), a gain change, a broadband interferer. The floors are
re-learned within 2 ms instead of one burst holding the whole band open until the 3 s cut.

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
  (1 ms). One still on after 3 s is cut. If the cut part carries data (FSK tones or OOK keying, at
  least 10 dB SNR) it is a transmission: a Flipper replaying an FSK RAW file holds the carrier for
  the whole file, both levels being tones, and so does a held FSK fob. It goes on as a new burst
  from the cut part's last frame gap (a low of ≥ 6 ms, for FSK a held tone of ≥ 6 ms; at most
  500 ms back), so the frame across the cut is whole in the next part and decoded once. Frames
  back to back with no gap at all (a held Kia V1 fob sends a new code every ~380 ms, separated only
  by its preamble) go on from 500 ms back: the frame across the cut is whole, and one before it may
  be decoded twice (the app drops the repeat as a duplicate). Its noise
  level comes from the detector's floor, as nothing around it is quiet. Only a cut part without
  data (a steady interferer, a gain step) has its bins re-baselined; before, a continuous FSK
  carrier was re-baselined too and was then noise to the detector until it stopped.
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
  level is low. **Short symbols:** if that finds no FSK, or finds FSK with at least 10 % of its
  levels shorter than 60 µs (symbols the smoothing merged: Somfy io-homecontrol's 26 µs bits at
  38.4 kbit/s show both tones but come out as a few dozen 50–1000 µs levels), and the carrier is on
  for ≥ 1 ms at a stretch, the discriminator is sliced again without smoothing; if it holds two tones, its levels
  are clean multiples (80 %) of an element shorter than 60 µs (≥ 2 samples), and those short levels
  fill at least half of the carrier's time, on both tones (≥ 25 % each), that is the FM path,
  debounced at TE/4 (a CC1101-style GFSK packet: Elero's 13 µs bits at 75 kbps come out as
  13/26/39/52 µs, as rtl_433 `-Y minmax` measures them; io-homecontrol's as 26 µs multiples).
  Otherwise pass 1's slicing stands. FSK that pass 1 resolves never gets here, nor does noise (not
  two tones) or OOK: a pulled carrier's key-on blips are short but rare (the Chrysler remote's are
  21 µs every 4 ms).
- *Pass 2:* the burst is split into frames at lows of 6 ms or more. For each frame the element
  length TE is estimated from pass 1: the 10 % time-weighted quantile of its level durations, or
  that of its HIGHs or of its LOWs alone when that is under a third of it and ≥ 100 µs (short
  pulses between long lows carry under 10 % of a frame's time: a Chrysler V0 preamble of 320 µs
  pulses 3.7 ms apart would get TE 3.7 ms, and its TE/4 debounce would erase every pulse). If TE is
  ≥ 100 µs the frame is sliced again with ~TE/5 averaging and TE/4 debounce. For OOK the frame's
  carrier phase is removed and the samples averaged **coherently**, which averages the noise away
  before the envelope is taken, close to a matched filter; for FSK the discriminator is averaged and
  the two tones are found again per frame. Short-TE frames keep pass 1, and a burst that also holds
  something else (another fob, interference) doesn't distort the TE estimate of the frame next to
  it.
- *OOK carrier tracking (pass 2).* A coherent average over TE/5 loses the carrier wherever its
  frequency is more than ~1/(4·TE/5) off the one removed (±2.7 kHz for a 445 µs TE), and cheap
  remotes move far more: the Audiovox PRO-OE3B in rtl_433's recordings drifts −63 → −101 kHz within
  one 33 ms frame, and every key-on pulls it (−63 → −74 kHz within a 444 µs pulse, −67 → −83 kHz within
  a 900 µs one; it is back by the next pulse). With one frequency per frame, the frame's first pulses
  and the ends of its long pulses (where the carrier is furthest from the mean) shrank or vanished:
  920 µs pulses came out at 230–460 µs. Now the phase comes from the frame's mean frequency, or from
  a **track of the carrier's frequency** where that follows a movement the mean misses: a slow track
  (the discriminator summed over the carrier-on samples within ±8 coherent windows, across pulses:
  drift over the press), then a fast one (±1 window, within each pulse: the pull). A track replaces
  the mean (or the slow track) only if the movement it follows beyond its own noise is ≥ 2× that
  noise (rms; 4× in variance) and large enough to matter (≥ 0.12 cycles per coherent window, rms),
  and only if its noise is < 0.35 cycles per window. The noise is measured, not modelled: the
  track is computed again from two interleaved halves of the samples, and their difference
  (weighted by the carrier in each window) is noise only. A steady carrier keeps the mean (its
  output is bit-identical to before); at low SNR a track's noise would cost more than a small drift
  gains. Only keyed carriers are tracked: a frame whose carrier is on > 90 % of its span is weak
  FSK that pass 1 didn't recognise, whose two tones the mean's coherent average separates like an
  OOK envelope (one passes, the other doesn't: some of the Ford V0 ±20 kHz presses at −8 dB in
  the sensitivity sweep decode that way), and a track would follow the tones.

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

A recording never replaces a file: a name in use gets `_2`, `_3`, … before the extension. The
extension always names the device's format, because `:iq` and other tools read the samples by it: a
wrong IQ extension is replaced (`:record x.cu8` on a HackRF writes `x.cs8`) and any other one is kept
with the format's added (`:record fob_433.92` writes `fob_433.92.cs8`); a folder (`:record runs/`) gets
the default name. While a recording runs, transmitting is refused ("Recording in progress — :record stop
first"): TX stops RX, which would end the recording.

**When the receiver stops by itself** — the device cannot be opened (busy, unplugged), a setting is
refused, or it goes away while streaming (an unplugged HackRF stops streaming, which KAT checks every
100 ms; a failed RTL-SDR read ends RX, as in `rtl_sdr`) — the status line says "Receiver stopped:
<reason> (r restarts)", the header goes back to idle with the level at zero, a running recording ends
with it, and one `r` starts RX again. If no radio is found at startup, the warning names the reason for
each device (not found, no USB permission, busy: another program or the RTL-SDR's DVB-T driver has it),
and `kat.log` has the libraries' errors.

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
demodulator (as measured for v1.5, 10 kHz offset):

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

### Drifting carriers

OOK whose carrier moves like a cheap remote's (`sweep_drift`; model from the Audiovox PRO-OE3B
recordings, measured pulse by pulse: a drift of up to −30 kHz with a 30 ms time constant, −1 kHz/ms
at first, and a pull of up to −19 kHz with a 0.5 ms time constant at every key-on, gone by the next
pulse), 10 kHz fob offset, live tuning, 16 trials; 50 % points in dB over 2 MHz, "before" is
KAT 1.7:

| carrier | Kia V1 (TE 800 µs), before | Kia V1, now | Ford V0 as OOK (TE 250 µs), before | Ford V0, now |
|---|---|---|---|---|
| steady | −8.5 | −8.5 (identical) | < −9 (8/16 at −9) | < −9 (identical) |
| drift −30 kHz / 30 ms | −8.4 | −8.4 | < −9 (9/16) | < −9 (identical) |
| pulled −19 kHz / 0.5 ms | +4.5 | −5.3 | < −9 (9/16) | < −9 (identical) |
| both (the Audiovox) | > +12 (no decode at any SNR) | −5.3 | < −9 (8/16) | < −9 (identical) |

The longer the element, the longer the coherent window (TE/5) and the less carrier movement it
tolerates: Ford V0's 50 µs window took the pull before, Kia V1's 160 µs one didn't. A pulled carrier
still needs ~3 dB more than a steady one: the fast track has to be estimated from one pulse at a
time. With noise added to the Audiovox recordings (+3 / +4.8 / +7 / +9.5 dB of noise; two noise
seeds each) KAT 1.7 decoded at most 1 frame per file, now lock 3, 3, 3, 1–2 of 3; option 10 of 10
throughout; unlock 13, 11–13, 9–11, 7–9 of 13 (pass 1 alone, without pass 2: 3 / 10 / 13 up to
+4.8 dB, nothing at +9.5 dB).

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
| OOK fob drifting −30 kHz (30 ms time constant) and pulled −19 kHz at every key-on (the Audiovox PRO-OE3B), 3 dB, at +10 and −60 kHz | decodes (normal suite; KAT 1.7: Kia V1 on this carrier didn't decode at any SNR, see *Drifting carriers*) |
| continuous ±47.6 kHz FSK carrier for 10 s (Kia V5 frames every 100 ms, 250 kS/s, 10 dB) | 4 bursts, each continuing at a frame gap; 99–100 of the 100 frames decode, none twice (normal suite; `kat --decode` of the same IQ: 92). KAT 1.7: the floor was re-baselined to the carrier at 3 s and 6 s, one burst in the last second, 9 decodes |

### Real recordings (rtl_433 test files)

rtl_433's public test recordings (`rtl_433_tests`, real IQ from RTL-SDRs, `.cu8` at 250 kS/s to
2.048 MS/s, recorded at or near the remote's frequency) through `kat --decode`. Compared with KAT
1.5 before the wide-band receiver, every decode is the same, plus:

| recording | carrier offset | before | now |
|---|---|---|---|
| Microchip HCS200, BTN2 / BTN3 (6 files) | −116…−125 / +121 kHz (edge of the 250 kS/s band) | nothing | KeeLoq (Magic_3), serial D0921, counters AC4…ACC |
| Microchip HCS200, BTN1 (6 files) | −95…−112 kHz | KeeLoq, `g002` 1 frame | same, `g002` 4 frames |
| Chrysler car remote (7 files, 920 kS/s) | −74…−384 kHz | no burst beyond −126 kHz | every burst at its offset, 15–17 dB, the ~7 ms lead-in carrier, preamble and 320 / 640 µs PWM data that rtl_433 `-A` measures (121–123 pulses; Chrysler-CarRemote decodes them) |
| Microchip HCS362 (868.3 MHz, 1 MS/s) | +280 kHz | 10 dB leakage of it | the bursts at +280.5 kHz, 38 dB |
| Chamberlain CWPIRC (915 MHz, 1 MS/s) | −283 and +217 kHz | noise-level bursts | each press on both of its frequencies, 25 dB |

Since KAT 1.7 (carrier tracking in pass 2, dead input, noise steps, long transmissions, short FSK
symbols, pass 2's element length for short pulses far apart), compared over both rtl_433 corpora
(230 recordings), every decode is the same, plus:

| recording | what it is | KAT 1.7 | now |
|---|---|---|---|
| Audiovox PRO-OE3B lock / option / unlock (303.3 MHz, 250 kS/s) | carrier −50…−101 kHz, drifting up to 38 kHz within a frame and pulled at every key-on | 1 / 1 / 0 of rtl_433's 3 / 10 / 13 frames, plus a corrupt one (ID 0029) | 3 / 10 / 13, as rtl_433; pulses within 2 µs of the raw envelope |
| Chrysler car remote 04 (920 kS/s) | −384 kHz, drifting | 1 of the 2 packets | 2 |
| Chrysler car remote 01, 02, 04, 05 (920 kS/s) | a 7 ms lead-in carrier, then 25 pulses of ~310 µs 3.7 ms apart, then the PWM data | the preamble erased by pass 2: 96–98 of rtl_433's 123 pulses (the decodes don't need it) | 121–123 pulses, as rtl_433 |
| Microchip HCS361 02 / 03 / 04, GM ABO1502T `g001`, Nutek 01 / 03, PT2262 `gfile001` | start with 58–260 ms of zero bytes (and HCS361/04 with ~6 ms of a settling receiver) | one burst over the whole file at "106 dB SNR", or none (HCS361/04 unlock); GM `g001` and PT2262 `gfile001` (at its 250 kS/s) got their decodes from that burst's FM path | the real bursts, 14–18 dB (HCS361 and GM at rtl_433's times); GM: the same two decodes and PT2262: the same seven, now from the AM path like the other files of those remotes |
| Elero 01–03 (869.4 MHz, 2.048 MS/s) | CC1101 GFSK, 13 µs bits, ±26 kHz, +119 kHz | no burst | one FSK burst each, pulses 13 / 26 / 39 / 52 µs as rtl_433 `-Y minmax`; Elero decodes all three with rtl_433's ID, command and counter |

The only changed field: EV1527 `g026`, one of its two Princeton decodes has TE 357 instead of 356 µs
(both now 357; the remote's carrier is pulled ~2.6 kHz rms within its pulses, so pass 2 tracks it).

**Pulse widths against rtl_433.** KAT's AM pulses come out 5–47 µs shorter than `rtl_433 -A`'s
(gaps longer by as much), but they agree with the signal: the raw full-band envelope of the same
recordings, sliced at half the carrier level with interpolated edges, gives the same widths within
1–5 µs (HCS200 `g001`: KAT 373 / 756 µs, raw 375 / 757, rtl_433 420 / 800; Audiovox lock: 448 / 900,
raw 447 / 899, rtl_433 464 / 920; Compustar lock: 717 / 1089, raw 721 / 1092, rtl_433 740 / 1112;
KeeLoq `g012`: 415 / 877, raw 417 / 877, rtl_433 432 / 892). rtl_433 measures long: its envelope
filter (1st-order IIR, τ ≈ 25 µs) and ±12 % hysteresis around a threshold that follows the pulse
level, and it counts the sample that ends a pulse. Only on drifting carriers did KAT's own pulses
shrink (see *OOK carrier tracking*).

Other things these recordings exercise: a transmitter keying its own carrier on and off around the
data (compustar), an RTL-SDR AGC that drops the gain during strong pulses (Chrysler), spurs and
lines of a strong transmitter 16–31 dB down (HCS362, Chrysler; dropped as artifacts or duplicates).
`rtl433_recordings_decode` (ignored; set `KAT_RTL433_TESTS` to the corpus's `tests` directory;
files the corpus doesn't have are skipped) checks these decodes and bursts, the Audiovox frame
counts, HCS361/04 unlock's burst and Elero's 13 µs FM grid.

### Over the air: a Flipper Zero replaying `.sub` files, received live by a HackRF

A Flipper Zero (Momentum firmware) sent 67 `.sub` files from its USB command line
(`subghz tx_from_file`): 18 car captures from `IMPORTS` and 49 of Flipper-ARF's unit-test captures
(gate remotes), on 314–434 and 868 MHz. KAT received them live (HackRF, 2 MS/s; the Flipper and the
HackRF plugged into the same computer), and each file's live captures were compared with
`kat --decode` of the file's own pulses: 250 distinct frames (protocol, serial, button, counter). A
frame a file shares with one sent before it counts as received (KAT drops a repeat of a capture it
already has).

| KAT | frames received |
|---|---|
| 1.7 receive chain, one capture per burst | 138 of 250 |
| one capture per frame; carrier tracking, dead input, transmissions over 3 s, short FSK symbols | 233 |
| pass 2's element length for short pulses far apart; back-to-back frames over the 3 s cut | 239 |

What the runs turned up, each reproduced in simulation (`src/radio/sim.rs`) before it was fixed:

- A burst gave one capture, of its first frame, so presses in quick succession and replayed
  recordings lost every later frame (Somfy Telis 1 → 8 of 8, Princeton 1 → 4 of 4).
- A Flipper replaying an FSK RAW file holds its carrier for the whole file (25–37 s): at the 3 s
  cut the floor was re-baselined to the carrier and the rest was noise (Kia V3, V4 and V5, Suzuki:
  1–2 → all of their 15–20 frames).
- Frames back to back with no gap (`KIA V1 N1 RAW.sub`, a held fob sending a new code every
  ~380 ms) lost the frame across the 3 s cut.
- A Chrysler V0 preamble (a 17 ms carrier, then 320 µs pulses 3.7 ms apart) was erased by pass 2:
  3 of the 10 frames of `Chrysler PT Cruiser 2006 close.sub` never decoded.

Still missed (11 of 250):

- FAAC SLH (3 frames): the Flipper's TX timing, below.
- The last frame of `KIA V1 N1 RAW.sub`: its final low runs into silence, which the Kia V1 decoder,
  like ProtoPirate's, doesn't read as a bit (a real fob repeats its frame, so only the last copy of
  a press is lost).
- `Opel Vectra 2008.sub`: two of its Fiat V1 frames come out as Renault V1 (same serial and
  counter, both HITAG2), two are missed.
- One frame each of Fiat V0 (`Fiat Bravo 2010`), VAG (`Skoda_Fabia_2007`) and Princeton
  (`gate_tx_raw.sub`: received with two bits flipped, which Princeton, having no checksum, decodes
  as another serial).
- Decodes the files don't have: that Princeton frame, and one Renault V1 (ARF) frame during
  `Holden Barina 2016 AU.sub` (its checks are weak, as in the reference).

**FAAC SLH.** `faac_slh_raw.sub` replayed by the Flipper and recorded by KAT live (433.92 MHz)
doesn't decode: its LOWs between short pulses come out
at 297 / 330 / 363 µs, and 91 of its 644 are 363 µs, above FAAC SLH's limit (te_short 255 +
te_delta 100). That is what is on the air, not the receiver: the raw envelope of the recording
(0.5 µs resolution, no channel filter) has the same distribution as KAT's pulses (LOW short median
330 / KAT 328 µs, the same 91 over 355 µs). The Flipper's CC1101 samples its asynchronous data
input at 8× the preset's data rate (Ook650Async: 3.79 kBaud → 30.35 kHz), so every replayed edge
lands on a 32.95 µs tick: the file's 318 µs LOWs become 9, 10 or 11 ticks.

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

The carrier tracking, the dead-input test (a strided check per chunk; per sample only where two
samples 32 apart are equal) and the noise-step test leave these numbers as they were, measured back
to back with KAT 1.7 on the same machine (2 MS/s noise 61.8 → 64 MS/s, FSK bursts 20.0 → 20.1,
2.4 MS/s 22.9 → 23.0, 10 MS/s 38.3 → 38.2, 250 kS/s noise 88 → 90). OOK bursts cost more to slice:
both tracks and their noise are evaluated on every OOK frame, ~20 ms of CPU per second of burst
(the three Audiovox recordings, decoded 20 times: 1.45 → 2.03 s); all 230 rtl_433 recordings:
6.0 → 6.85 s.

**Not verified on hardware.** The synthetic measurements above and the rtl_433 recordings (real
RTL-SDR IQ) are all there is. Live HackRF/RTL-SDR operation with the wide-band detector (the LO / DC
bins, HackRF spurs and its I/Q image, an RTL-SDR's AGC live), the HackRF processing thread and
retune-boundary estimate, and real fobs' behaviour still need a check with the devices.
