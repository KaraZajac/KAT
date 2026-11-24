---
layout: default
---

# Recording a remote

How the recordings in the [library](https://kat.karazajac.io/captures.html) are made, which Flipper preset holds
which signal, and how to tell on the spot whether a recording is worth keeping.

## Overview

A decoder starts with a recording. KAT reads 451 of the library's 553 files. Of the other 102, seven hold
nothing (empty files, and key files of zeros), eight hold a remote's signal from which no frame has been read yet,
and 77 hold no data at all: a pass that looks for nothing but pulses of regular width finds stretches of carrier
and noise in them, and no bits. Those remotes were recorded with a radio setting that could not hold their signal,
or on the wrong frequency, and no decoder will ever read those files; [Unread recordings](unidentified_captures.md)
goes through each. This page is about not making more of them.

## What a .sub file keeps

A Flipper Zero does not record the radio signal. Its radio chip, a CC1101, demodulates the signal with the
chosen **preset** and hands over levels and durations: carrier for 412 µs, none for 388 µs, and so on. A RAW
`.sub` file is that list. What the preset's settings filtered away is not in the file:

- an FSK remote recorded with an AM preset is one long carrier pulse per burst, whatever it sent;
- an FSK remote recorded with an FM preset of the wrong deviation is noise, or nothing;
- a remote faster than the preset's data rate comes out with its pulses stretched and its pauses shortened, or
  with short pulses missing.

Two more filters sit between the chip and the file. In asynchronous mode the CC1101 updates its output 8 times
per symbol of the preset's data rate (TI's CC1101 datasheet, SWRS061I p.63: "8 samples per bit"): every 26 µs
with the stock FM presets, every 33 µs with the AM presets. The firmware then joins any pulse under 30 µs to
the one before (`lib/subghz/subghz_worker.c`), and its RAW recorder drops a pulse of 50 µs or less together with
the pulse after it (`lib/subghz/protocols/raw.c`, `te_short`; 20 µs in doctor-firmware). A remote whose half
bits are near 60 µs loses pulses on the way to the file; doctor-firmware keeps more of them.

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
| The Continental smart keys of Kia, Hyundai and Genesis (FCC IDs beginning `SY5`) | "F4" (`FM14K` on doctor-firmware) or "Honda1"; not `FM476` | the Soul, Sorento and Carnival recordings made with `FM476` hold nothing: through each press its output stays HIGH. Their filings give 2-FSK at 7.8 kBaud, Manchester (64 µs half bits, 15.6 kBaud on air) and tone peaks 27.5 to 31 kHz apart, about ±15 kHz, where `FM476` expects ±47.6 kHz at 4.8 kBaud. F4 and Honda1 fit both and read the Ford and Honda keys of the same speed. Not tried on these keys yet |
| Toyota and Lexus keys on two channels (312.10 and 314.35 MHz) | an SDR; with a Flipper, one channel per recording | the Tundra recordings made with `FM238` (±2.38 kHz) and the older ones made with AM presets hold nothing. The Denso smart keys' filings give tones about 30 kHz apart (±15 kHz: "F4" or "Honda1"), the remote-head keys' 50 to 60 kHz and the TRW keys' ±30 kHz ("FM29F" below), and one press on one channel, the next on the other. Read RAW does not hop, so a recording on one channel holds every other press. Not tried yet on these; the European Lexus UX key (433.66 MHz), recorded with `FM476`, reads ([Lexus UX](lexus_ux.md)) |

The last two rows are worked out from filings, not from a recording that reads. If you record one of those
remotes with the preset suggested and it reads, or does not, that is worth an issue either way. The
[Unread recordings](unidentified_captures.md) page goes through every recording in the library that KAT reads
nothing from, and what each would need.

### A custom preset

"F4", "Honda1" and the others below are not built into most firmwares. A preset is added in the Flipper's
`/ext/subghz/assets/setting_user` file (Momentum and doctor-firmware ship a `setting_user.example` to start from):
the header `Filetype: Flipper SubGhz Setting File` and `Version: 1`, then three lines per preset. The data is
register and value pairs, `00 00`, and the eight bytes of the power table; it must not start with `00`, or no
register is written. A recording made with it stores the preset's data in the `.sub`, so KAT knows what it was.

```
Custom_preset_name: F4
Custom_preset_module: CC1101
Custom_preset_data: 02 0D 0B 06 08 32 07 04 14 00 13 02 12 04 11 83 10 6A 15 31 18 18 19 16 1D 91 1C 00 1B 07 20 FB 22 10 21 56 00 00 C0 00 00 00 00 00 00 00

Custom_preset_name: Honda1
Custom_preset_module: CC1101
Custom_preset_data: 02 0D 0B 06 08 32 07 04 14 00 13 02 12 04 11 36 10 69 15 32 18 18 19 16 1D 91 1C 00 1B 07 20 FB 22 10 21 56 00 00 C0 00 00 00 00 00 00 00

Custom_preset_name: FM29F
Custom_preset_module: CC1101
Custom_preset_data: 02 0D 0B 06 08 32 07 04 14 00 13 02 12 04 11 83 10 6A 15 41 18 18 19 16 1D 91 1C 00 1B 07 20 FB 22 10 21 56 00 00 C0 00 00 00 00 00 00 00
```

| Preset | Filter | Data rate | Deviation | Status |
|---|---|---|---|---|
| F4 (doctor-firmware's `FM14K` is the same modem setting) | 270.8 kHz | 38.4 kBaud | ±14.28 kHz | reads Ford V1 and the JLR fobs |
| Honda1 | 270.8 kHz | 15.4 kBaud | ±15.87 kHz | reads the Continental Honda keys and Honda V5 |
| FM29F | 270.8 kHz | 38.4 kBaud | ±28.56 kHz | F4 with the deviation of a ±30 kHz key; worked out from TI's formulas, not tried |
| `FM476` (built in) | 270.8 kHz | 4.8 kBaud | ±47.61 kHz | reads FSK keys of ±30 to ±50 kHz with half bits of 175 µs and more (Kia Mobis, Kia Denso); the fastest key it reads in the library has 100 µs half bits (Honda V2) |

Figures from TI's formulas (fXOSC 26 MHz) on the register values. TI's datasheet says the deviation setting
"should be the same as the TX deviation for demodulation to be performed reliably and robustly" (p.42) and that
the data rate is "the data rate expected in receive" (p.35).

### An SDR recording

When the remote's deviation or speed is not known, when it uses two channels, when its half bits are shorter
than about 60 µs, or when a Flipper recording shows no regular pulses, record IQ samples instead: they keep the
signal as it was, and KAT demodulates it itself, OOK or FSK, with no loss measured up to ±325 kHz off frequency.

- KAT: `:record` while it receives saves the IQ samples ([Receive chain](radio.md)).
- HackRF One with `hackrf_transfer`: 8 MS/s or more (HackRF's documentation does not recommend less), centred
  within a few hundred kHz of the remote.
- RTL-SDR: 2.048 to 2.4 MS/s.
- PortaPack (Mayhem) Capture app: a bandwidth of 500 kHz (Mayhem's advice for most SD cards; no more than 1 MHz),
  the `.C16` format, and keep the `.TXT` beside it.

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
