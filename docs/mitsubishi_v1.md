---
layout: default
---

# Mitsubishi V1 Protocol

**Rust module:** `src/protocols/mitsubishi_v1.rs`
**Reference:** `FoxFW lib/subghz/protocols/mitsubishi_v1.c` (no other reference has it). KAT keeps FoxFW's line
code, frame layout and key file, and reads every fob: FoxFW's decoder reads the one its author had (below).

## Overview

A 2-FSK Manchester rolling-code keyfob frame of 96 bits: a 32-bit fob id, a constant fix byte, a 4-bit button, a
10-bit cleartext counter and a 42-bit encrypted hop. It is not KAT's `Mitsubishi V0` (the 96-bit PWM pairs of
Flipper-ARF, or the 61-bit Kia V0 type). KAT replays the captured frame; FoxFW has no encoder.

## Timing

| Parameter | Value  | Notes                                                        |
|-----------|--------|--------------------------------------------------------------|
| Short     | 200 µs | 100..=300 µs (delta 100)                                     |
| Long      | 400 µs | >300..=500 µs                                                |
| Glitch    | <100 µs | ignored, state kept                                          |
| Gap       | >500 µs | ends the collection                                          |

Line code: a LONG pulse is one bit equal to its level; of two SHORT pulses the second is one bit equal to its
level (FoxFW's half-bit toggle).

## Frame Layout (96 bits)

| Bits   | Field                                                    |
|--------|----------------------------------------------------------|
| 0..31  | fob id; starts `0001` (FoxFW: "sync" `0x1E4C11A7`)       |
| 32..39 | fix byte, `0xE0`                                         |
| 40..43 | button: `0x2` Lock, `0x8` Unlock, `0x4` Trunk            |
| 44..53 | counter (10 bits, cleartext; wraps from `0x3FF` to 0)    |
| 54..95 | hop (42 bits, encrypted); the frame's last bit is 1      |

**KAT packing:** `data` = the first 64 bits (id, fix, button, counter, 10 hop bits), `extra` = the 42-bit hop,
`data_count_bit` = 96, `serial` = the id.

## A press

Both fobs in IMPORTS send: 248 pairs of 200 µs pulses (about 99 ms), carrier off for about 20 ms, then the frame
four times (more while the button is held). Each copy follows about 1.2 ms of the low tone and 25 short pulses,
and the carrier is off for about 20 ms between copies. In Manchester cells of 400 µs a copy is 13 cells
HIGH-LOW, one LOW-HIGH, then a cell per bit from the frame's second bit on (a 1 is HIGH-LOW). The long LOW where
the two meet is what the reader takes for the frame's first 0.

## FoxFW's "sync word" is a fob's id

FoxFW takes the first 32 bits for a constant, `0x1E4C11A7`, searches the collected bits for it and says the frame
has no per-fob serial. IMPORTS has two fobs (`MITSUBISHI/`):

| Captures | Id | Presses read | Buttons |
|----------|----|--------------|---------|
| `more/KeyFob1_1.sub` … `KeyFob1_6.sub` | `0x1E4C11A7` | 109 | Lock, Unlock, Trunk; three short-press and three held recordings |
| `Mitsh_all_x1_fm476.sub`, `more/Mitsh_lock_x20_f3.sub`, `…unlock…`, `…btn3…` | `0x1260F206` | 60 | Lock, Unlock, Trunk |

`KeyFob1` is by its counts the fob FoxFW's decoder was written from ("6 captures / 111 presses" in the C's
header). The second fob has the same fix byte, the same three button codes, and a counter that goes up by one
from the first press of its first file to the last of its last (`0x36F` … `0x3B8`). The word is the fob's id.

## Decoder (KAT's rule)

When a gap ends the collection, the last 96 bits are a frame if they

- start `0001` and end in a 1 (both hold for all 169 presses in IMPORTS),
- carry the fix byte `0xE0` (FoxFW's gate),
- follow at least 7 equal bits: the preamble, 12 zeros to the reader. When the carrier comes up behind receiver
  noise that reads HIGH, the first preamble pulse is lost in it and the reader pairs the others the other way
  round: 12 ones, and the frame behind them as before.

There is no CRC; the CRC column shows "-".

## Deviations from FoxFW

- **Any id.** `serial` is the 32-bit id; FoxFW shows the fix byte there (`0xE0` for every fob).
- **The frame is the 96 bits in front of the gap.** FoxFW takes up to 96 bits from its sync word wherever that is,
  and reports a frame cut at 88..95 bits with its hop zero-padded. KAT wants all 96 bits and the closing 1, the
  only checks there are on the bits behind the fix byte. Two of the 226 copies of the second fob arrive damaged;
  one still has its fix byte, and only the closing 1 keeps it from being listed as a press of its own.
- **The collection ends at every gap.** FoxFW's decoder resets its buffer only after it found its sync and at
  least 88 bits; with 199 bits and no sync it is stuck until the receiver restarts. A press starts with a 248-bit
  wake-up, so on a whole press that is every time: FoxFW's C reads none of the presses in IMPORTS, its own fob's
  included. KAT starts over at each gap, and past 199 bits it drops the oldest.

## Encoder

The press above with the captured frame, as it is: wake-up, four copies. The hop is encrypted, so there is no
next code to build, and a receiver that has seen the frame may refuse it. Lock / Unlock / Trunk / Panic all send
the same frame. The pauses are sent as the low tone (a level/duration upload has no "carrier off").

## Key files

FoxFW's (`subghz_protocol_decoder_mitsubishi_v1_serialize`): `Protocol: Mitsubishi V1`, `Bit: 96`, `Key` = the
first 64 bits, then `Fix`, `Btn`, `Cnt`, `HopHi`, `HopLo` (decimal). Loading takes Bit 96, the Key and the two
hop words, as FoxFW does.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM and FM in FoxFW's flags; both fobs are 2-FSK (presets FM476 and F3 in the captures)
- **Encryption:** rolling code (the hop); the counter is in the clear
- **Frequencies:** 315 MHz, 433.92 MHz
- **Check:** none (structural: preamble, `0001`, fix byte, closing 1)

## Validation

- The ten captures: every press of both fobs, four copies each in the short-press recordings (68 / 80 / 80
  frames for `KeyFob1_1..3`, 80 / 64 / 72 for the second fob's Lock / Unlock / Trunk recordings), the same frames
  as a separate Python model of the rule. The held recordings give 109, 194 and 203 copies.
- FoxFW's C (host build): reads a single made-up frame with its own id as KAT does (same Key, Fix, Btn, Cnt,
  hop); reads nothing with another id; reads nothing of a whole press, its own id included (the wake-up).
- Unit tests: any id, the preamble in either phase, the fix byte, the closing 1, frames that do not end at the
  gap, a frame behind noise, the encoder's press against the fob's pulse layout, key files.

Every Flipper reference also lists a **Kia V7** frame with key `4C00000000000000` in `Mitsh_btn3_x20_f3.sub`: the
wake-up is a square wave of short pulses, which Kia V7 takes for its preamble. KAT's Kia V7 drops that key
([kia_v7.md](kia_v7.md)).
