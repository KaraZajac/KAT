---
layout: default
---

# Hollarm Protocol

**Rust module:** `src/protocols/hollarm.rs`
**Reference:** `REFERENCES/Flipper-ARF/lib/subghz/protocols/hollarm.c` (@xMasterX, Unleashed lineage) — the copy
Flipper-ARF's garage app, Momentum and RogueMaster also ship (identical decoder and encoder). ARF ships it
unregistered; Momentum, RogueMaster, doctor-firmware and the stock firmware register it (unit tests
`hollarm_raw.sub`, `hollarm.sub`).

## Overview

42-bit fixed code of Hollarm (GSM alarm) remotes: a 24-bit serial, a fixed nibble, a 4-bit button and a byte sum,
sent as 40 bits plus two trailing 0 bits. The decoder checks the byte sum. Static code.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| HIGH | 200 µs | te_short, ± te_delta 200 µs |
| LOW, bit 0 | 1000 µs | te_long, ± 200 µs |
| LOW, bit 1 | 1600 µs | 8·te_short, ± 200 µs |
| Gap | 2400 µs | 12·te_short; header window ± 400 µs, end-of-frame ± 200 µs |
| Bits | 42 | MSB first: the 40-bit key, then 00 |

AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`).

## Frame Layout (40-bit key)

| Bits | Field |
|------|-------|
| 39..16 | serial (the C: (key & 0xFFFFFFF0000) >> 16) |
| 15..12 | fixed nibble (0x2 on the C's example remote) |
| 11..8 | button: 0x2 Arm, 0x1 Disarm, 0x4 Ringtone/Alarm, 0x8 Ring; settings mode 0x9 Settings mode, 0xA Exit settings, 0xB Vibro sensitivity, 0xC (not used), 0xD Volume |
| 7..0 | (byte 4 + byte 3 + byte 2 + byte 1) & 0xFF |

## Decoder Steps

1. Reset: a LOW of |d − 2400| < 400 starts a frame.
2. HIGH (saved) + LOW pairs: HIGH |d − 200| < 200 with LOW ≈ 1000 → 0, ≈ 1600 → 1.
3. A LOW of |d − 2400| < 200 closes the frame and adds a final 0 bit; with exactly 42 bits, key = bits >> 2 is
   reported when its byte sum holds (else dropped, as the C). Back to Reset: a held button reports every other
   frame.

**doctor-firmware's copy differs (decoder only):** header window ± 200 µs, "degenerate" keys rejected
(`subghz_protocol_blocks_key_is_degenerate`), and after a frame it goes on from the gap that closed it, so every
repeat of a held button decodes (24 vs 12 frames on the unit-test capture; its callback's receiver reset is
overridden by the `SaveDuration` step that follows it). KAT follows Flipper-ARF (reference priority; the copy the
other three firmwares ship) — the decoded codes are the same, KAT's registry resets every decoder after a decode
as ARF's code expects.

## Field mapping

`serial`, `button` as the C; `counter` None; `data` = the 40-bit key, `data_count_bit` = 42; `crc_valid` = the byte
sum holds (always for decodes); `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_hollarm_get_upload` (`encoder.repeat` = 3): key = (key >> 12) << 12 | btn << 8 + the new
byte sum; upload = the 42 bits of key << 2 as HIGH 200 + LOW 1000 / 1600, the last LOW 2400; 3 copies. Button: KAT's
native code 0..=0xF is sent (the references' button editor takes any 4-bit value; their D-pad cycles 0x2 / 0x1 /
0x4 / 0x8), larger codes keep the captured one. "Send button…" lists the four buttons, then the settings-mode
codes. No counter.

## Key file

`Protocol: Hollarm`, `Bit: 42`, `Key` = the 40-bit key; loading requires `Bit: 42`.

## Add Manually

"Hollarm 433MHz" (all four Flipper menus): key `(r & 0x000FFF0000) | 0xF0B0002200` plus the byte sum (button 0x2,
fixed nibble 0x2). See [add_manually.md](add_manually.md).

## Validation

Reference oracles: `subghz_oracle_arf --registry all`, `subghz_oracle_momentum`, `subghz_oracle_roguemaster` (and
doctor-firmware's for comparison); parity tools in `src/protocols/f_common.rs`.

- Flipper unit test `hollarm_raw.sub`: 12 frames through KAT's whole registry, identical to Flipper-ARF, Momentum
  and RogueMaster (doctor-firmware: the same key 24 times, see above); `hollarm.sub` loads, encodes and decodes
  back once (as in the C's encoder test).
- Decode parity on 1338 distinct RAW files: identical with ARF / Momentum / RogueMaster; doctor differs only on
  `hollarm_raw.sub` (24 vs 12).
- Fuzz: 1000 streams (250 per F protocol): identical with ARF / Momentum / RogueMaster (442 frames); doctor reports
  8 more (back-to-back repeats).
- Reference registries (doctor, RogueMaster, G1 protocols first) report only Hollarm on the Hollarm streams.
- Encoder: the unit-test key and 100 random keys (a third with an arbitrary byte sum) — pulse-identical to
  Flipper-ARF, doctor-firmware, Momentum and RogueMaster.
- Unit tests: every button, the C's four example keys, header / end-gap window bounds, bad sum / length / timing
  rejected, key-file round trip.
