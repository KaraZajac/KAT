---
layout: default
---

# Recording a remote

How the recordings in the [library](https://kat.karazajac.io/captures.html) are made, which Flipper preset holds
which signal, and how to tell on the spot whether a recording is worth keeping.

## Overview

A decoder starts with a recording. KAT 1.17 reads 427 of the library's 518 files. Eight of the rest are empty
files or key files no decoder takes; of the other 83, 74 hold no data at all. A pass that looks for nothing but pulses of regular width finds stretches of carrier and
noise in them, and no bits. Those remotes were recorded with a radio setting that could not hold their signal,
and no decoder will ever read those files. This page is about not making more of them.

## What a .sub file keeps

A Flipper Zero does not record the radio signal. Its radio chip, a CC1101, demodulates the signal with the
chosen **preset** and hands over levels and durations: carrier for 412 µs, none for 388 µs, and so on. A RAW
`.sub` file is that list. What the preset's settings filtered away is not in the file:

- an FSK remote recorded with an AM preset is one long carrier pulse per burst, whatever it sent;
- an FSK remote recorded with an FM preset of the wrong deviation is noise, or nothing;
- a remote faster than the preset's data rate comes out with its pulses stretched and its pauses shortened, or
  with short pulses missing.

An SDR recording (`.cs8` from a HackRF One, `.cu8` from an RTL-SDR, `.C16` or `.C8` from a PortaPack) keeps
the samples and can be demodulated again another way. KAT records one while it receives with `:record` ([Receive chain](radio.md)).

## Which preset

First find out whether the remote is on-off keyed or FSK. The remote's FCC filing says so in its test report
("Modulation Type: FSK", or ASK), and often gives the deviation; the [FCC ID table](https://kat.karazajac.io/fcc.html)
has the filings that were read. Without a filing, record with `AM650` and with `FM476` and look at both.

| The remote | Preset | What the library shows |
|---|---|---|
| On-off keyed, half bit of 200 µs or more (most remotes) | `AM650` | read: most of the library |
| On-off keyed and fast: half bit near 120 µs (General Motors' [433 MHz](chevrolet_v0.md) and [315 MHz](gm_rfa.md) remotes) | `AM650` | read, with the pulses up to 50 µs out: KAT measures the skew of each burst and takes it out, but the weakest presses are lost |
| 2-FSK, ±30 to ±50 kHz ([Kia Mobis](kia_mobis.md), [Kia Denso](kia_denso.md) and most FSK remotes in the library) | `FM476` | read |
| 2-FSK, about ±15 kHz, fast (Honda's Continental keys) | the custom preset "Honda1": ±15.87 kHz, 15.4 kBaud | read with it ([Continental-KR5V2X](continental_kr5v2x.md), [Honda V5](honda_v5.md)); a 2022 CR-V recorded with `FM476` holds nothing |
| The Continental smart keys of Kia and Hyundai (FCC IDs beginning `SY5`) | not `FM476` | the Soul, Sorento and Carnival recordings made with `FM476` hold nothing. Their filings give 2-FSK at about ±15 kHz and 7.8 kBaud, the figures of the Honda keys above, so "Honda1" should fit. Nobody has tried it yet |
| Toyota and Lexus keys on two channels (312.10 and 314.35 MHz) | not `FM238`, not an AM preset | the Tundra recordings made with `FM238` (±2.38 kHz) and the older ones made with AM presets hold nothing. The filings give 2-FSK at ±30 kHz (TRW) or tones 30 to 50 kHz apart (Denso), which `FM476` is closer to; the TRW filing shows one press on one channel and the next on the other. Not tried yet |

The last two rows are worked out from filings, not from a recording that reads. If you record one of those
remotes with the preset suggested and it reads, or does not, that is worth an issue either way.

## Making the recording

- On the Flipper: **Sub-GHz → Read RAW**, the frequency of the remote's band (315.00, 433.92 or 868.35 MHz) and
  the preset. Hold the remote about an arm's length from the Flipper.
- Press each button at least twice, a second apart, and one button after the other. Two presses show what
  changes from press to press; a third and fourth show whether a counter counts.
- Hold one button down for two seconds in a separate recording. Many remotes send something else while a
  button is held, and some send a different frame when it is let go.
- Name the file after what is in it. The library's names read year, make, model and the presses in order:
  `22_chevy_trail_blazer_l2_u2_rs_p_t.sub` is lock twice, unlock twice, remote start, panic, trunk.
- Note the remote's FCC ID, from its back or from under its battery. It ties the recording to a filing and to
  the cars the remote fits, and it is the one thing that cannot be read from the signal.

## Checking it

`kat --decode file.sub` prints what KAT reads from a file. For a file it cannot read, open the file in KAT and
press `a` on it for [Analyze](analyze.md): it says how the signal is keyed, the pulse widths it found, and
whether they form frames that repeat.

A recording that holds a remote shows the same thing each time a button was pressed: bursts of the same length
at the same distance, and pulse widths in two or three clear groups (one and two half bits, say 200 and 400 µs).
A recording that holds nothing shows a few long pulses per press and widths spread evenly over everything. If
the second picture is what you see, change the preset and record again while the car is still there.

## Sending it in

Recordings go into the repository's [`IMPORTS`](https://github.com/KaraZajac/KAT/tree/main/IMPORTS) folder, one
folder per make, by a pull request or attached to an [issue](https://github.com/KaraZajac/KAT/issues). A
recording of a remote on the [wanted list](https://kat.karazajac.io/coverage.html#wanted) is the most useful,
and so is any recording of a remote KAT reads only by a signal match.

Record only remotes you own or have the owner's permission to record. A rolling-code remote's recording does
not open the car again once the car has heard those presses; a fixed-code remote's does, so think before
publishing one that opens something.
