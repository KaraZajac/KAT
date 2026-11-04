---
layout: default
---

# Princeton Protocol

**Rust module:** `src/protocols/princeton.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/princeton.c` (primary);
Flipper-ARF `lib/subghz/protocols/princeton.c` and doctor-firmware `lib/subghz/protocols/princeton.c`

## Overview

The 24-bit fixed code of PT2262 / PT2260 / EV1527-style encoders used by countless generic gate, garage
and alarm remotes. The C's false-positive defence is a repeat check: a frame is reported only when the
previous 24-bit frame was identical and non-zero. The decoder also estimates the remote's time base (TE)
and guard time, which the key file stores and the encoder reuses.

doctor-firmware additionally drops "degenerate" keys (all 0/1, `0xAA…`/`0x55…`, or four equal nibbles in
a row, `subghz_protocol_blocks_key_is_degenerate`); the garage app and ARF's lib copy do not, and neither
does KAT (such keys are valid remotes, e.g. `0x0000F1`).

## Timing

| Parameter  | Value              | Notes                                                    |
|------------|--------------------|----------------------------------------------------------|
| te_short   | 390 µs             | ±300 µs                                                  |
| te_long    | 1170 µs            | ±900 µs (3 × te_delta)                                   |
| Start LOW  | 14040 ± 10800 µs   | any guard-like LOW starts a frame                        |
| End LOW    | ≥ 2340 µs          | the guard of the frame                                   |
| Guard      | 30 × TE (default)  | estimated per capture, 15..=72 × TE, else 30             |

## Frame Layout (24 bits, MSB first)

Bit 1 = HIGH 3·TE + LOW TE, bit 0 = HIGH TE + LOW 3·TE; then a stop HIGH TE and the guard LOW.
Fields as `subghz_protocol_princeton_check_remote_controller` splits them:

- low byte 0x30 / 0xC0: 16-bit serial (`data >> 8`) + 8-bit button 0x30 / 0xC0;
- low byte 0x03 / 0x0C: 16-bit serial + button 0xF3 / 0xFC;
- otherwise: 20-bit serial (`data >> 4`) + 4-bit button.

`extra` = TE (low 32 bits) | guard time << 32.

## RF

- **Encoding:** PWM (1:3)
- **RF modulation:** AM (OOK); 315 / 433.92 / 868 MHz (no frequency filter)
- **Encryption:** none (fixed code)
- **Buttons:** labelled as the C prints them ("Btn 4", "Btn C0")

## Decoder Steps

1. **Reset** — a LOW within 14040 ± 10800 µs → SaveDuration (bits, TE sum cleared, guard = 30).
2. **SaveDuration** — a HIGH: remember it, add it to the TE sum.
3. **CheckDuration** — a LOW ≥ 2340 µs ends the frame: with exactly 24 bits equal to `last_data` (non-zero)
   the frame is reported with TE = sum / 97 and guard = round(LOW / TE); `last_data` takes the frame.
   Otherwise HIGH short + LOW long = 0, HIGH long + LOW short = 1, anything else → Reset.

The app resets the receiver inside the callback, so after a decode the next frame needs a new start LOW
while `last_data` (restored right after the callback) keeps the frame: a held button decodes the 2nd, 4th,
6th … frame after the first start LOW. KAT reproduces that (the module keeps the post-callback state
through the registry's reset).

## Encoder

`subghz_protocol_encoder_princeton_get_upload`, `encoder.repeat` = 3: the key is rebuilt from serial and
button (`button` = the native code to send: a 4-bit code for a 4-bit remote, 0x30/0xC0/0xF3/0xFC for the
8-bit type; anything else keeps the captured button), then 24 bits, a stop HIGH TE and the guard LOW
TE × guard time, three times. A key without TE does not load (as in the C).

The garage app's TX plugin is built without its custom-button API, so its stub reports the "original"
button as 0 and the plugin transmits every Princeton key with button 0 (e.g. key `95D5D4` is sent as
`95D5D0`). KAT sends the captured button, as Flipper-ARF's lib copy and doctor-firmware do.

## Key file

`Protocol: Princeton`, `Bit: 24`, `Key` (8 bytes), `TE: <µs>`, `Guard_time: <n>` (the garage serializer's
order; `Guard_time` is optional on load).

## Validation

Against the C oracles (`$S/oracle`, garage and doctor builds, `decode --protocol Princeton`, the decoder
fed alone with the in-callback reset): the 56 Flipper unit-test RAW captures give 37 identical decodes (key,
TE, guard time); 3420 fuzz files (perturbed encoder output of random keys for all twelve G1 protocols plus
900 random pulse soups) give 207 identical decodes; the 108 IMPORTS car captures give none in either. The
encoder's upload equals the doctor oracle's pulse for pulse for 26 key files (the unit test's `princeton.sub`
and 25 random keys incl. 8-bit types and guard times 15..72); the garage oracle differs only in the button
nibble (see Encoder). rtl_433 IQ recordings: `PT2262/01` (3 files) decode as `755555` (rtl_433 tristate
`Z1ZZZZZZZZZZ`, the FSK file through the inverted FM path), `generic_remote/01` (4 files) as `13CDC0`,
`13CD0C`, `13CD03`, `13CD30` (rtl_433 id 5069 = 0x13CD, cmd 192/12/3/48) and
`EV1527-Universal-Remote` (8 files) as the bit-reversed EV1527 codes (`6F3CB1` ↔ rtl_433 `8d3cf6`);
`HT680_remote` decodes in neither KAT nor the C. Unit tests: field split, encode → decode with TE/guard
estimation, button switching within the button type, repeat/zero/timing rejection, the unit-test key file
and the RAW capture.
