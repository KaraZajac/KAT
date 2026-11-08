---
layout: default
---

# Intertechno V3 Protocol

**Rust module:** `src/protocols/intertechno_v3.rs`
**Reference:** `REFERENCES/doctor-firmware/lib/subghz/protocols/intertechno_v3.c` (the stock Flipper protocol,
registered by doctor, Momentum, RogueMaster and FoxFW). Flipper-ARF's `lib` and garage-app copies (both
unregistered), Momentum's, RogueMaster's and FoxFW's (encoder in an SD plugin) decode and encode identically; they
differ in style and in the display (doctor / Momentum / RogueMaster add the channel and On/Off or the dimming
level; ARF shows `SN:` only). ARF's `lib` copy turns on the custom-button D-pad, but every direction re-sends the
captured code; no copy switches buttons. Flipper unit tests: `intertechno_v3_raw.sub`, `intertechno_v3.sub`.

## Overview

The self-learning Intertechno code (KlikAanKlikUit "new" style) of 433 MHz sockets, switches and dimmers: 32-bit
on/off frames and 36-bit dimming frames carrying a 26-bit address, an "all channels" flag, the channel and the
command. No checksum.

## Timing / modulation

T = 275 µs.

| Element | Pulses (HIGH, LOW, …) | Decoder windows |
|---------|-----------------------|-----------------|
| Stop bit | T, 38·T | LOW 37·T ± 2250 µs (7926..12424) |
| Start bit | T, 10·T | HIGH T ± 150, LOW 10·T ± 450 µs |
| 0 | T, T, T, 5·T | |
| 1 | T, 5·T, T, T | |
| Dimm (28th cell of a 36-bit frame) | T, T, T, T | reads as 0 |
| End of frame | a LOW ≥ 11·T (3025 µs) | |

A cell's bit is decided by its first LOW (T ± 150 → 0, 5·T ± 300 → 1, and only for the 28th bit T ± 300 → 0) with
the second HIGH T ± 150; the closing LOW must be T ± 150 or 5·T ± 300. AM (OOK); the C's flags: 315, 433 and
868 MHz.

## Frame Layout

| Frame | Bits | Field |
|-------|------|-------|
| 32 bits | 31..6 | address (KAT `serial`) |
| | 5 | all channels |
| | 4 | command: 1 On, 0 Off (KAT `button`) |
| | 3..0 | channel, inverted (KAT `counter`: ~bits; 32 when all channels) |
| 36 bits | 35..10 | address |
| | 9 | all channels |
| | 8 | the dimm cell (always 0) |
| | 7..4 | channel, inverted |
| | 3..0 | dimming level (KAT `button`; the C shows 6.67 % steps) |

## Decoder Steps

1. Reset: a LOW of |d − 10175| < 2250 (the stop bit).
2. Start bit: HIGH T ± 150, then LOW |d − 2750| < 450.
3. Cells: HIGH T ± 150; LOW (saved); HIGH T ± 150 — the bit as above; a closing LOW.
4. A LOW ≥ 3025 µs where a cell's first LOW belongs ends the frame: exactly 32 or 36 bits are reported. Without a
   report the decoder goes on from the start bit (the long LOW was the next frame's stop bit). With one, the app's
   callback resets the receiver, so the stop bit that closed a frame does not open the next one: of a held button's
   back-to-back copies every other one decodes.

## Field mapping

`subghz_protocol_intertechno_v3_check_remote_controller`: `serial` = the address, `button` = On/Off or the dimming
level, `counter` = the C's `cnt`, the channel (as KAT's Dooya); `data` = the frame, `data_count_bit` 32 or 36;
`crc_valid` false; `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_intertechno_v3_get_upload` (`encoder.repeat` = 3): stop bit, start bit, the cells (bit 8
of a 36-bit frame always as the dimm cell). The captured code is sent: no reference switches buttons. Each copy's
stop bit closes the copy before it, so a receiver decodes copy 1 (copy 2's stop bit was consumed by the report and
copy 3 has none after it).

## Key file

`Protocol: Intertechno_V3`, `Bit: 32` or `36` (the decoder's and encoder's deserialize reject other counts), `Key`.

## Add Manually

None of the references offers an Intertechno type.

## Validation

Host builds of the references' C (`REFERENCES/tools/oracle/bin/subghz_oracle_{doctor,arf,momentum,roguemaster,foxfw}`):
- `intertechno_v3_raw.sub` (ARF, Momentum and RogueMaster unit-test folders): 12 frames through KAT's whole
  registry, all 0x3F86C59F (address 0xFE1B16, channel 0, On), and nothing else; `test_random_raw.sub` adds the same
  12 frames, as the C's random test; `intertechno_v3.sub` loads and its upload decodes back once, as the C.
- Encoder: 64 random key files (32 on/off and 32 dimming frames, every level, both "all channels" values, bit 8
  set and clear) give the same upload, pulse for pulse, in KAT and in doctor's, ARF's, Momentum's and RogueMaster's
  C.
- Decoder: 200 fuzz files (repeats, mixed codes, wrong lengths, other protocols' frames, noise; jitter, dropped,
  split and stretched pulses, timing scaled ±15 %) give the same 176 frames in KAT's decoder and in all five
  references; on the 304 real RAW files and the other protocols' fuzz files it decodes only its own captures.
- Registry: the fuzz streams decode the same through KAT's whole registry as through doctor's.

Parity tools: `src/protocols/f_common.rs` (`F_FILES`, `F_KEYS`).
