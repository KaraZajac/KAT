---
layout: default
---

# Fiat V2 Protocol

**Rust module:** `src/protocols/fiat_v2.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/fiat_v2.c` (decoder; decode-only there). The encoder
combines Flipper-ARF's `lib/subghz/protocols/fiat_v2.c` known-key hop recompute with doctor-firmware's
replay waveform (`lib/subghz/protocols/hitag2.c`, which since November 2026 holds its Fiat V1, Fiat V2 and
Renault V1 code; `fiat_v2.c` before).

## Overview

Rolling-code keyfobs from FCA (Fiat / Chrysler / Jeep / Alfa Romeo). Each burst carries one 112-bit Manchester
frame at 210/420 µs between long LOW gaps. In the references the frame holds a 32-bit UID, a type nibble, a
2-bit button selector, an 11- or 14-bit counter and a 32-bit hop code; the real fobs send button flags and a
14-bit counter ([below](#the-fca-fobs-layout-kats-from-the-captures)). Real captures in IMPORTS:
`chrysler/19_chrysler_pacifica*`, `jeep/22_jeep_renegade*`, and a key specialist's Alfa Romeo Giulia and Stelvio
(`ALFA ROMEO/more/2016-2025_*`), Jeep Wrangler (`jeep/more/2019-2025_*`) and Maserati Ghibli
(`MASERATI/more/2016-2021_maserati_ghibli_*`, the references' non-FCA layout).

## Timing

| Parameter | Value    | Notes                                        |
|-----------|----------|----------------------------------------------|
| Short     | 210 µs   | ±100 µs (111..309 µs) = 1 half-bit cell      |
| Long      | 420 µs   | ±100 µs (321..519 µs) = 2 cells              |
| Boundary  | ≥900 µs  | LOW gap ending a frame                        |
| Frame     | 112 bits | 224 cells                                     |

## Frame Layout (112 bits = 14 bytes, MSB first)

| Bytes        | Field                                                                    |
|--------------|--------------------------------------------------------------------------|
| raw[0..1]    | markers `0x00 0x01` (15 zero bits + a one)                               |
| raw[2..5]    | UID (serial)                                                             |
| raw[6]       | high nibble: type (`0xD` = FCA layout); low nibble varies with the button |
| raw[7]       | bits 7:6 selector: 1 Trunk, 2 Lock, 3 Unlock; `button` = raw[7] as in the C |
| raw[8..13]   | counter + hop (layout depends on the type, below)                        |

- **non-FCA:** counter = NOT(`(raw[7] & 0x3F) << 5` OR `raw[8] >> 3`) & 0x7FF (11-bit), hop = raw[9..12]
- **FCA (type `0xD`):** counter = NOT(`raw[8] << 6` OR `raw[9] >> 2`) & 0x3FFF (14-bit), hop = raw[10..13]

The non-FCA layout is a real key's: a key specialist's 2016–2021 Maserati Ghibli key sends it (20 presses of each
button). The selector follows the recordings' names (2 Lock, 3 Unlock, 1 Trunk), and the 11-bit counter goes up by
one with every press, on from one recording to the next (59 of 59 steps). Its flag word does not have the FCA fobs'
shape (below), so KAT reads it with the C's fields. None of Flipper-ARF's known keys reproduces its hop, read in
either layout, so it is replayed.

**KAT packing:** `data` = raw[0..8], `data_count_bit` = 112, `extra` = raw[8..14] (48 bits, right-aligned).

### The FCA fobs' layout (KAT's, from the captures)

Five real fobs in IMPORTS fit neither layout above: a 2019 Chrysler Pacifica and a 2022 Jeep Renegade, and a key
specialist's 2016–2025 Alfa Romeo Giulia, 2016–2025 Alfa Romeo Stelvio and 2019–2025 Jeep Wrangler (20 presses of
each button, one recording per button). Read with the non-FCA layout, as ProtoPirate, Flipper-ARF and
doctor-firmware read them, the Pacifica's and the Renegade's counter is one value for the whole recording
(`0x09B`, `0x080`), and their Panic and Remote start presses carry Unlock's selector. Bit-inverted, their bytes
6..9 are a flag word and a 16-bit word:

| Bytes (inverted) | Field |
|---|---|
| `!raw[6..8]` | top nibble: one value per fob (`0xA` Pacifica and Giulia, `0x8` Renegade, `0x4` Wrangler, `0x0` Stelvio), then the button flags (one of bits 11..5), then `0b00100` |
| `!raw[8..10]` | counter, 14 bits: up by one with every press, across all buttons; then 2 bits that change from press to press with no pattern |
| `raw[10..14]` | hop |

| Flag bit | Button | KAT button code |
|---|---|---|
| 5 (`0x020`) | Panic | 1 |
| 6 (`0x040`) | Lock | 2 |
| 7 (`0x080`) | Trunk | 3 |
| 8 (`0x100`) | Unlock | 4 |
| 9 (`0x200`) | Sliding door 1 | 5 |
| 10 (`0x400`) | Sliding door 2 | 6 |
| 11 (`0x800`) | Remote start | 7 |

The recordings are named after the buttons pressed, and the flags follow the names press for press. The counter
goes up by one with every press: on the Pacifica and the Renegade (below), and on the three other fobs over all
171 steps between their presses, on from one recording to the next in the order they were made.

| Recording | Presses (flag, counter) |
|---|---|
| `chrysler/19_chrysler_pacifica_ul2_l2_t2_rs2_sd2_sd2_p2_t2.sub` | Unlock `37F7` `37F8`, Lock `37F9` `37FA`, Trunk `37FB` `37FC`, Remote start `37FD` `37FE`, Sliding door 1 `37FF` `3800`, Sliding door 2 `3801` `3802`, Panic `3803`, Trunk `3804` `3805` |
| `jeep/22_jeep_renegade_ul2_l2_rs2_p2.sub` | Unlock `1F5` `1F6`, Lock `1F7` `1F8`, Remote start `1F9` `1FA`, Panic `1FB` `1FC` |
| The Giulia's, the Stelvio's and the Wrangler's (`*_lock`, `*_unlock`, `*_trunk`) | 20 presses each, every one the button its recording is named after |

The Pacifica's recording is named for two Panic presses, but the counter goes straight on from its one Panic
press to the Trunk press after it: the fob sent one. KAT 1.22 took the whole 16-bit word for the counter, which
went up by 1 to 7 from press to press.

The C's FCA layout (type `0xD`) is this one: its 14-bit counter (`raw[8] << 6` OR `raw[9] >> 2`) and its hop are
these fields, and its selector (raw[7] bits 7:6) is the Trunk and Lock flags seen through two bits (`10` Lock,
`01` Trunk, `11` any other button, which is why the C reads Panic and Remote start as Unlock). Only its gate,
the one top nibble `0xD`, reads none of the five fobs.

A frame whose flag word has that shape (low five bits `00100`, exactly one button bit), whatever its top nibble,
is read so: `counter` = the 14 bits, `button` = the code in the table (a selector byte is never below `0x40`, so
the two kinds of code do not meet). `serial`, `data`, `extra`, the key file and the replay are unchanged, and
such a frame is never forward-encoded with the other layouts' fields. KAT 1.22 read this layout only on the top
nibbles `0xA` and `0x8` (raw `0x5` and `0x7`), so the Stelvio's and the Wrangler's presses showed the C's
fields: the raw button byte and one counter for the whole recording.

FoxFW's Jeep V0 (`jeep_v0.c`) reads the Renegade's frames this way, inverted, with the four codes that fob has
(`0x8104` Unlock, `0x8044` Lock, `0x8804` Remote start, `0x8024` Panic); it takes the 16-bit word's high byte for
a constant `0x07`, and so reads no other fob (not the Pacifica).

On air, a bit b is sent as half-bits (!b, b) (LOW/HIGH = 1). Every genuine frame in IMPORTS uses this polarity,
which the C reaches through its inverted try.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM (OOK)
- **Encryption:** rolling code: a 32-bit hop, which Flipper-ARF and qDoctor take for Fiat V1's HITAG2
  authenticator (not confirmed against a real key); recomputed only when a known or saved key reproduces it,
  replayed otherwise (see Encoder)
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset**: the first short or long pulse clears the cell buffer and starts Data.
2. **Data**: a short pulse pushes one cell of its level, a long pulse pushes two. After each push the last
   224 cells are tried as 112 Manchester pairs, first with bit = first cell, then inverted. A window is valid if
   every pair differs, both markers match, the selector is 1/2/3 and the UID is not 0 / 0xFFFFFFFF.
3. **End of burst**: any other pulse ends the run. A LOW ≥900 µs first pushes one LOW cell (the frame's last
   half-bit merged into the gap).
4. **KAT gate**: the frame is reported only if the valid window ends the burst, with at most one trailing cell
   (doctor-firmware's stop pulse). Identical consecutive frames are suppressed until `reset()` (the C's dedup).
   When the input ends right after the frame (a recording cut there), there is no pulse to end the burst: the
   frame is reported then, as the C, which reports when its window completes, has.

The end-of-burst gate is the one deviation from the C. The C's sliding window also matches 15 bits of any
Manchester preamble plus the first 97 bits of a longer frame. On `LDV/LDV-T80_unlock` it would complete before
Ford V3 does and steal that frame (LDV V0's now) in KAT's first-match registry. Genuine FCA bursts always end
with the frame, so all 47 Pacifica and 29 Renegade frames still decode. doctor-firmware's extra type-nibble exclusions
(0x1/0x7/0xA) were not adopted, because the genuine Renegade frames have type 7.

## Encoder

Frame content from Flipper-ARF, waveform from doctor-firmware:

- **Hop recompute (Flipper-ARF):** if one of ARF's 8 Fiat V1 known HITAG2 keys (`fiat_v1_known_keys`) with
  one of its 4 IV normalisations (raw selector or one-hot remap × counter or complemented counter, epoch 0)
  reproduces the captured hop, the frame is forward-encoded: the requested selector (KAT Lock → 2,
  Unlock → 3, Trunk → 1; other commands keep the captured one), the counter + 1 (ARF's car-emulate scene
  writes Cnt + 1 per press), and the hop recomputed (`fiat_v1_compute_auth`) and patched in; markers, UID
  and type nibble are kept. After ARF's keys KAT tries the key saved for the remote in `hitag2_keys.txt`
  (Fiat V1's byte order) the same way, e.g. one [Recover key (HiTag2)](hitag2_hell.md) found from two
  captures (its IV readings for Fiat V2 are Flipper-ARF's; no reference has confirmed them against a real
  key, so a search may find nothing). The key that verifies a capture is shown in its detail panel's
  **HiTag2** row, in the standard order and as **Fiat order**, the order of `hitag2_keys.txt`
  ([HiTag2 key recovery](hitag2_hell.md#the-key-file)). KAT's Fiat V2 key files carry no `Hitag2 Key` line, and
  KAT does not read the one Flipper-ARF writes.
- **Replay:** otherwise (every capture in IMPORTS — no known key matches them in ARF either) the 14
  captured bytes are replayed unchanged, as ARF and doctor-firmware do.
- **Waveform (doctor-firmware):** each of 6 copies is LOW 2000 µs, then the 112 bits as (!b, b) half-bits of
  210 µs, then a HIGH 210 µs stop pulse if the last half-bit was LOW, then LOW 2000 µs, with same-level runs
  merged. This polarity matches the real captures; Flipper-ARF's upload is inverted behind a 2033 µs
  carrier lead copied from its Fiat V1 encoder, so KAT keeps doctor-firmware's (capture evidence).

## Validation

Decodes REAL IMPORTS captures: the Chrysler Pacifica (UID `0xD737A2C3`, 47 frames, 434.075 MHz) and the Jeep
Renegade (UID `0xD08BE613`, 29 frames, 433.889 MHz), the key specialist's Alfa Romeo Giulia, Alfa Romeo Stelvio
and Jeep Wrangler (8 recordings, 433.92 MHz, 160 presses, 704 frames) and Maserati Ghibli (3 recordings, 60
presses, 241 frames, read in the non-FCA layout); in these files KAT reads nothing but Fiat V2. The UID is stable, and the button flags and the 14-bit counter follow the recordings' names press
for press (the table under "The FCA fobs' layout"; `chrysler_and_jeep_presses_follow_their_recordings_names`,
`alfa_romeo_and_jeep_wrangler_presses_follow_their_recordings_names`). The LDV / Ford V3 captures do not match.
A volunteer's Alfa Romeo Giulia (2015–2026, 433.92 MHz, not in `IMPORTS`) uses Fiat V2
too: 5 lock and 5 unlock codes, the frames doctor-firmware, ProtoPirate and Flipper-ARF read. Its unlock
frames also form a valid Ford V3 US frame in the opposite polarity; KAT 1.8.5 listed 28 of those 30 frames as
Ford V3 ([ford_v3.md](ford_v3.md)). The Fiat Bravo capture (`FIAT/Fiat Bravo 2010 …`) is not Fiat V2: its
bursts are 70-bit frames after a ~756 µs sync. Unit tests cover synthetic frames in both polarities, a
stretched first pulse, the merged-LOW boundary, FCA field parsing, the 6-copy encode→decode round trip (frames
ending HIGH and LOW), and rejection of bad markers, UID, selector and Manchester, plus a window inside a
longer burst, and the known-key forward encoding (both layouts, all 4 IV combos). C oracle: the real captures
decode to the same frames as ProtoPirate's and Flipper-ARF's decoders (KAT's end-of-burst gate only drops
their LDV false positive); on 1200 encodes of randomized frames (840 with a known-key hop) the transmitted
frame equals the `Raw` Flipper-ARF's encoder writes back, byte for byte. doctor-firmware's merged file
(98d1fc1) builds from KAT's key files the upload KAT sends, pulse for pulse (the Pacifica frame and a frame
with the FCA layout), and the key files its build writes for the Pacifica capture (11 distinct frames) load
in KAT with its serial, button and counter.
