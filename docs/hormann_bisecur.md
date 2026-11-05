---
layout: default
---

# Hörmann BiSecur Protocol

**Rust module:** `src/protocols/hormann_bisecur.rs`
**Reference:** `REFERENCES/Momentum-Firmware/lib/subghz/protocols/hormann_bisecur.c`; RogueMaster's copy is
byte-identical. RogueMaster registers it; Momentum (like Unleashed) leaves it commented out of its registry
("disabled ... due to flash space constraints and very limited usefulness", CHANGELOG) — a Flipper constraint KAT
does not have. Both trees ship two key files and one RAW capture in their unit-test resources (no unit test uses
them).

## Overview

Hörmann's BiSecur garage-door remotes (868 MHz, 2-FSK): a 176-bit frame — type byte, 32-bit serial, two 64-bit
encrypted blocks (AES-128 rolling code) and a CRC-8. Nothing beyond the serial is readable without Hörmann's key, so
the reference decodes, displays and re-sends a stored frame; it cannot make a new code.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 208 µs | half-bit; ± te_delta 104 µs |
| te_long | 416 µs | full bit; ± 104 µs |
| Preamble | 21 × (LOW, HIGH) te_short | first LOW 312 µs (1.5·te_short), last HIGH 1664 µs (4·te_long) |
| Sync | 2 × (LOW, HIGH) te_long | the decoder checks LOW, HIGH, LOW, HIGH of 416 µs |
| Data | 176 bits Manchester | MSB first, bits inverted (0 = LOW then HIGH), last half-bit + 312 µs |

FM (2-FSK), 868 MHz (`SubGhzProtocolFlag_868 | FM`). The unit-test captures were recorded with a custom CC1101
preset (DEVIATN 0x40, ±25.39 kHz), which KAT transmits with when a capture carries it; otherwise F3 (±31.74 kHz;
F2 at ±19.04 kHz is equally far, see [transmit.md](transmit.md)).

## Frame Layout (22 bytes)

| Bytes | Field |
|-------|-------|
| 0 | type: 0x50 or 0x70 |
| 1..4 | serial |
| 5..12 | encrypted block 1 (the C's `generic.data`, first "Key:" line) |
| 13..20 | encrypted block 2 (`generic.data_2`, second "Key:" line) |
| 21 | CRC: type 0x50 → !(CRC-8 poly 0x07 init 0 over bytes 1..20 ^ 0x55); type 0x70 → reflected CRC-8 (poly 0x07, init 0xFF) over bytes 0..20 |

## Decoder Steps

`subghz_protocol_decoder_hormann_bisecur_feed`:

1. Reset: a LOW of |d − 312| < 104.
2. Any number of |d − 208| < 104 pulses, then a HIGH of |d − 1664| < 104.
3. A LOW of |d − 416| < 104 starts the sync countdown (`sync_cnt` = 3): HIGH, LOW, HIGH of |d − 416| < 104 (the
   C's `level == (sync_cnt-- & 1)`), then the Manchester decoder is reset (Mid1).
4. Data: short (|d − 208| < 104, or any pulse longer than 208 µs for the 176th bit) / long (|d − 416| < 104)
   events, `level ? ShortLow/LongLow : ShortHigh/LongHigh`, through the C's own table `manchester_advance_alt`
   (CodeAllNight's fix of the toolbox decoder, ported as `advance_alt`). Any other pulse resets the decoder.
5. The 176th bit completes the frame (the C calls back; the receiver then resets the decoder).
6. **KAT gate:** the frame is reported only when its CRC holds (type 0x50 / 0x70). The C reports every 176-bit
   frame and shows "CRC OK" / "WRONG"; KAT's checksum policy for gate remotes. Both real captures pass.

## Field mapping

176 bits fit no `data` + `extra` pair, so KAT keeps the C's own field split: `serial` = bytes 1..4, `data` =
bytes 5..12, `extra` = bytes 13..20, `button` = byte 0 (the frame type; labelled "Type 0x50" / "Type 0x70"),
`counter` None (encrypted), `crc_valid` = the CRC holds; the CRC byte is recomputed from the type when the frame is
written or sent. `data_count_bit` = 176.

## Encoder

`subghz_protocol_encoder_hormann_bisecur_get_upload`: no key, no counter — it re-sends the stored frame (Dynamic
type; "Send next code" replays the decoded frame). Upload: the preamble and sync above, the 176 bits through
Flipper's `manchester_encoder` (inverted), the closing half-bit + 312 µs. `encoder.repeat` = 1 (the C: "original
remote does 3 repeats", left at 1 for the 504.3 ms carrier-off gap it cannot send). `button` is ignored.

## Key file

`Protocol: Hormann BiSecur`, `Bit: 176`, `Key` = the 22 bytes (the C writes the generic 8-byte Key, then updates it
with all 22). Loading needs `Bit: 176` and at least 22 Key bytes; KAT loads only keys whose CRC holds (it keeps no
CRC byte; the C loads and re-sends any 176-bit key — a receiver rejects a frame whose CRC fails). Momentum's /
RogueMaster's `Lat:` / `Lon:` lines are not written.

## Validation

Reference oracles (host builds of the Momentum / RogueMaster C, added this round): `subghz_oracle_momentum
--registry all`, `subghz_oracle_roguemaster`; parity tools in `src/protocols/f_common.rs`.

- Unit-test resources (Momentum = RogueMaster): `hormann_bisecur_1C84A55B_raw.sub` decodes 3 frames through KAT's
  whole registry (FM gate from the custom preset), identical to both; `hormann_bisecur_080ED969.sub` /
  `_1C84A55B.sub` load with the C's fields and write back byte-identical.
- Decode parity on 1338 distinct RAW files: identical (no other file decodes in either).
- Fuzz: 1000 streams (250 per F protocol; a quarter of the BiSecur frames with random bytes, i.e. mostly a wrong
  CRC): identical but for the 13 frames whose CRC fails, which only the C reports (C 68, KAT 55).
- Encoder: both unit-test keys and 100 random keys of both types — KAT's upload equals Momentum's and RogueMaster's
  pulse for pulse.
- Unit tests: both CRC types (and a bitwise reference for the reflected CRC), key files, 64 random encode→decode
  round trips, bad CRC / broken preamble / glitch rejected, the long last half-bit.
