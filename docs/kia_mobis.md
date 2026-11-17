---
layout: default
---

# Kia Mobis Protocol

**Rust module:** `src/protocols/kia_mobis.rs` (KAT's own decoder, decode only)
**Reference:** none reads this signal. The name is KAT's: the smart keys Hyundai Mobis builds for Kia.

## Overview

Kia smart keys of about 2017 to 2022 (a Niro, a K7 and a Sportage QL in the recordings) that send a **104-bit
Manchester burst** with the press counter and the button in the clear and a 40-bit code that changes with every
press. No checksum is known and the cipher is not known: **decode and display only**.

The decoder was worked out from three recordings in `IMPORTS/KIA/more/` (2026-11, 433.92 MHz, Flipper preset
FM476, ten presses each). Before it, FoxFW's Ford V3 took one clipped burst of two of the files for a frame of its
own (both frames are 104 bits long).

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Half bit | 194.8 µs | 2567 bit/s, the same on all three fobs (± 60 µs in the decoder) |
| Line code | Manchester | either polarity of the two tones is read |
| Burst | 104 bits, 40.5 ms | 16 one bits, the word `0x6AC2`, 72 bits |
| Repeats | up to three per press | 135 to 170 ms apart |

2-FSK, 433.92 MHz.

## FCC filings

| FCC ID | Remote | What the filing shows |
|--------|--------|-----------------------|
| `TQ8-FOB-4F34` | the K7's (Hyundai Mobis "Fob Smart Key", 2019; five buttons: lock, unlock, panic, tailgate, remote start) | three bursts of 41.0 ms, 167.4 ms apart, 0.38 s a press; the lower tone 40.5 kHz under 433.92 MHz; 98.4 kHz of bandwidth |
| `TQ8-FOB-4F17` to `TQ8-FOB-4F26` | other Mobis smart keys of those years | three bursts of 41.5 to 42.0 ms, the two tones 80 kHz apart |

The K7 recording agrees with its filing: 40.5 ms of bits a burst, bursts 159 to 168 ms apart (once 136), up to
three a press. The FCC IDs of the Niro's and the Sportage's remotes are not known.

## Frame (72 bits behind the word)

| Bits | Field | Notes |
|------|-------|-------|
| 0..27 | Counter | up by one with every press on all three fobs (1 to 14 seen). The upper bits were always 0, so the width is a reading, not a finding |
| 28..31 | Button | `1` or `2` seen (two of the recordings change after five presses); which button is which is not known |
| 32..71 | Code | changes with every press; a few of its first bits stay per fob |

No fob ID is in the clear. No checksum was found (CRC-8 of every polynomial over the obvious spans, sum, XOR).

## Decoder

The decoder collects the half bits of a burst (pulses of one or two half bits) and reads the burst when a pulse
of another length ends it (`src/protocols/burst.rs`). A frame needs:

1. at least 6 one bits, then the word `0x6AC2`;
2. 72 bits without one Manchester violation;
3. the burst ending with the frame: at most 8 half bits behind it (an FM receiver's noise sets in as the carrier
   drops, and a few of its pulses pass for half bits).

Both phases and both polarities are tried. There is no checksum, so the registry places Kia Mobis behind every
Flipper reference's decoders.

## Field mapping

`data` = frame bits 8..71, `extra` = frame bits 0..7, `data_count_bit` = 72, `counter` = the counter's low 16
bits, `button` = the nibble, `serial` None, `crc_valid` false (shown as "-"), `encoder_capable` false. No key
file: no firmware has a format for this frame.

## Validation

- `KIA_Niro_late_model_433.92_FM.sub`: 12 frames, counters 2 to 10, button 1.
- `Kia_K7_433.92_FM.sub`: 12 frames, counters 2 to 10 (6 and 8 not caught), buttons 1 then 2.
- `2019-2022_Kia_Sportage_QL__433.92_FM.sub`: 16 frames, counters 5 to 14 (11 not caught), buttons 1 then 2.
- The two false Ford V3 lines on these files are gone; no other file in IMPORTS changes.
- Unit tests: a synthetic burst at either polarity, the gate (five ones, a wrong word, a burst too long or too
  short, a pulse out of tolerance), the three recordings.
