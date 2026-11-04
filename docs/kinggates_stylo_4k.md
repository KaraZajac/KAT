---
layout: default
---

# KingGates Stylo4k Protocol

**Rust module:** `src/protocols/kinggates_stylo_4k.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/kinggates_stylo_4k.c` (primary) and
doctor-firmware `lib/subghz/protocols/kinggates_stylo_4k.c` — same air format, frame, check and upload

## Overview

KingGates Stylo4k gate remotes: an 89-bit frame, a 53-bit fixed part (serial, button) and a KeeLoq hop encrypted
with simple learning under the KingGates manufacturer key.

The C calls back for every 89-bit frame and zeroes serial/button/counter in its display when the hop does not decrypt
to the frame's button, the nibble 0xC and the serial's low byte. Per KAT policy a frame is reported only when it
passes that check (16 bits, `crc_valid`).

The garage copy keeps walking the keystore's type-10 entries until one passes the check; doctor-firmware stops at the
first type-10 entry (the same with one KingGates key).

## Timing

| Parameter  | Value    | Notes                                                 |
|------------|----------|-------------------------------------------------------|
| Short      | 400 µs   | ± 140 µs                                              |
| Long       | 1100 µs  | ± 280 µs                                              |
| Preamble   | 400/400  | > 2 pulses (the encoder sends 12)                     |
| Header     | 2200 µs  | ± 280 µs, last preamble LOW                           |
| Start bit  | 800 µs   | HIGH, ± 280 µs (the decoder waits for it)             |
| Bits       | 89       | LOW first: 1 = LOW 400 + HIGH 1100, 0 = LOW 1100 + HIGH 400 |
| End        | ≥ 3300 µs | LOW                                                  |

## Frame Layout (89 bits)

- **53-bit fix**, sent LSB first: `reverse_key(fix, 53)` = 16 zero bits | serial[31:16] | button (4) | 1 | serial[15:0]
- **36 bits**: the 32-bit KeeLoq hop bit-reversed, then 4 zero bits
- **Hop plaintext** (simple learning): button << 28 | 0xC << 24 | serial[7:0] << 16 | counter (16 bits)

KAT fields: `data` = the first 64 wire bits (fix53 << 11 | hop36 >> 25), `extra` = the last 25 wire bits,
`data_count_bit` = 89; serial (32 bits), button (4 bits), counter (16 bits). Key file: `Key` = fix53 (C
`generic.data`), `Data` = hop36 (C `generic.data_2`).

## Key

The C uses the keystore entries of learning type 10 (KEELOQ_LEARNING_SIMPLE_KINGGATES). KAT's embedded keystore holds
that key ("Kingates_Stylo4k", 0x414C455831393830) typed 1, and its only type-10 slot is ProtoPirate's KIA key, which
the KeeLoq view drops; the module therefore takes entries of type 10 or with that name. Evidence: Flipper's unit-test
capture `kinggates_stylo4k_raw.sub` decrypts with it and passes the check (serial 0x130F8127, button 0xE, counter
0x0B9D) — and the C oracle run with that key typed 10 gives the same fields, while with KAT's type 1 it shows zeros.

## RF

- **Encoding:** PWM (LOW-first)
- **RF modulation:** AM (OOK), 433.92 MHz in the C's flags; KAT decodes on every band
- **Encryption:** KeeLoq (simple learning)

## Decoder Steps

1. **Reset** — a HIGH 400 ± 140 µs counts a preamble pulse.
2. **CheckPreambula** — a LOW 400 ± 140 goes back to Reset; after more than 2 pulses a 2200 ± 280 µs pulse is the
   header; anything else resets the count.
3. **CheckStartBit** — wait for the HIGH 800 ± 280 µs start bit.
4. **SaveDuration / CheckDuration** — LOW then HIGH per bit; the first 53 bits are stored at the 53rd; a LOW of
   3300 µs or more ends the frame: 89 bits → analyse and report when the check passes.

## Encoder

`..._deserialize` + `gen_data` + `get_upload`: the frame is analysed again, counter + 1 (0 past 0xFFFF), button = the
requested native code (4 bits; the C's custom buttons cycle 0xE/0xD/0xB/0x7, codes above 0xF keep the captured
button), fix rebuilt, hop re-encrypted with the first KingGates key. Upload: LOW 9500, 12 × (HIGH 400, LOW 400) with
the last LOW 2200, HIGH 800, 53 + 36 bits; `encoder.repeat` = 3.

Flipper-ARF's garage TX plugin (`tx_kinggates_stylo_4k`) is built without `SUBGHZ_GARAGE_HAS_CUSTOM_BTN` and
`SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS`: their fallbacks make it send button 0 and take the "OFEX" counter path (the counter
jumps to 0xFFFE). KAT follows the source's intended path, which is doctor-firmware's behaviour.

## Key file

`Protocol: KingGates Stylo4k`, `Bit: 89` (required), `Key` (fix53) and `Data` (hop36, required). A frame failing the
check is refused.

## Validation

Flipper's `kinggates_stylo4k_raw.sub` decodes 12 times (serial 0x130F8127, button 0xE, counter 0x0B9D);
`test_random_raw.sub` contains the same remote (12 frames). C oracle (doctor and garage, with the key typed 10): decode
parity on the unit-test captures (57 files, 24 frames), IMPORTS (108), UberGuidoZ (1171) and 4200 fuzz files (1357
frames; one C frame failing the check is not reported). Encoder: 300 random key files, KAT equals doctor-firmware pulse
for pulse; the garage oracle (TX plugin flags) differs by the button-0/OFEX fallbacks above.
Key files the C serialized (doctor and garage formats) load in KAT with the C's fields; KAT writes the garage's lines
exactly.
