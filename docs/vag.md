---
layout: default
---

# VAG (VW/Audi/Seat/Skoda) Protocol

**Rust module:** `src/protocols/vag.rs` (+ `src/protocols/aut64.rs`)
**Reference:** `REFERENCES/ProtoPirate/protocols/vag.c`, `aut64.c` (primary); `REFERENCES/Flipper-ARF/lib/subghz/protocols/vag.c` (same state machine; adds 315 MHz and Lock/Unlock/Trunk remapping)

## Overview

VAG keyfobs send an 80-bit Manchester frame: Key1 (64 bits) + Key2 (16 bits). Key1 byte 0 is a
vehicle/type byte (0x00 "VW Passat", 0xC0 VW, 0xC1 Audi, 0xC2 Seat, 0xC3 Skoda); Key1 bytes 1–7 plus
Key2's high byte are an 8-byte encrypted block; Key2's low byte is the dispatch byte. There are four
types in two air formats:

| Type | Air format | Cipher | Dispatch (Lock / Unlock / Boot) |
|------|------------|--------|---------------------------------|
| 1 | 300/600 µs, prefix 0xAF3F, Key1/Key2 inverted | AUT64 (keys 1–3) | 0x2A / 0x1C / 0x46 |
| 2 | 300/600 µs, prefix 0xAF1C, Key1/Key2 inverted | XTEA (fixed key) | 0x2A / 0x1C / 0x46 |
| 3/4 | 500/1000 µs, sync 1000+500 + 3×750 µs, not inverted | AUT64 (key 3 → Type 4, else keys 2, 1) | 0x2B / 0x1D / 0x47 |

Every 80-bit frame is reported, as in ProtoPirate; frames that do not decrypt carry no
serial/button/counter (`crc_valid = false`) and cannot be re-encoded. Flipper-ARF and doctor-firmware
report decrypted frames only.

## Timing

| Parameter | Type 1/2 | Type 3/4 |
|-----------|----------|----------|
| Short | 300 µs (\|d−300\| < 100) | 500 µs (\|d−500\| < 100) |
| Long | 600 µs (d > 400 and \|d−600\| < 200) | 1000 µs (\|d−1000\| < 200) |
| Preamble | ≥151 LOW halves of 300 µs | ≥31 LOW halves of 500 µs |
| Sync | 600 µs LOW (\|d−600\| < 200) | HIGH 1000 (±200) + LOW 500 (±100), then 3 × 750 µs pairs (±150) |
| End | LOW ≥ 4001 µs with exactly 80 bits | the 80th bit |

Inside the data states a pulse that is neither short nor long is ignored (not a reset), like the C.

## Frame Layout (80 bits)

- **Key1[63:56]:** vehicle/type byte (sent in clear)
- **Key1[55:0] + Key2[15:8]:** encrypted block → serial (bytes 0–3, big-endian), counter (bytes 4–6, little-endian, 24 bits), button byte (byte 7)
- **Key2[7:0]:** dispatch byte (must match the button for Types 1/2/4)

Button: Type 1 reports the whole byte (0x10 Unlock, 0x20 Lock, 0x40 Boot — ProtoPirate; ARF keeps the
nibble), Types 2/3/4 the high nibble (1 Unlock, 2 Lock, 4 Boot). The KAT UI labels both forms
(`native_button_name`, "Boot" shown as "Trunk").

## KAT fields

`data` = Key1, `data_count_bit` = 80, `counter` = low 16 bits of the counter. `extra` (saved in `.fob`
as `data_extra`): bits 0–15 Key2, bits 16–23 type, bits 24–31 AUT64 key index (0xFF = none/XTEA), bits
32–39 counter bits 16–23, bit 63 set. `.fob` files from KAT ≤ v1.3.0 (`type | key_idx << 8`) still
encode.

## Decoder Steps

1. **Reset** — HIGH 300 µs → Preamble1; HIGH 500 µs → Preamble2.
2. **Preamble1 → Data1** — count 300 µs halves; a 600 µs LOW after ≥151 enters Data1. Manchester (ProtoPirate's `level ? ShortLow : ShortHigh` on Flipper's machine); at 15 bits a 0x2F3F/0x2F1C prefix sets Type 1/2 and restarts the count; at 64 bits Key1 = ~data; on the ≥4001 µs gap with 80 bits Key2 = ~data → parse.
3. **Preamble2 → Sync2A/B/C → Data2** — count 500 µs halves (≥31), sync 1000/500, three 750 µs pairs; Data2 starts with one implied 1 bit; at 64 bits Key1; at 80 bits Key2 → Type 3 → parse.
4. **Parse** (`vag_parse_data`) — Type 1: AUT64 keys 1,2,3 with `vag_button_valid`; Type 2: XTEA + `vag_button_matches`; Type 3: AUT64 key 3 (→ Type 4), 2, 1 with `vag_button_valid`; Type 4: key 3 + `vag_button_matches`.

## Encoder

Supported. From the capture's type, key index (recovered by re-parsing Key1/Key2 when missing, like the
C), serial, button and counter + 1 (24-bit): builds the plaintext block, encrypts (AUT64 / XTEA),
recomputes the dispatch byte and emits ProtoPirate's upload — Type 1/2: 220 × (HIGH 300, LOW 300),
LOW 300, HIGH 300, prefix + ~Key1 + ~Key2 Manchester at 300 µs, LOW 6000 (635 entries); Type 3/4: two
copies of 45 × (HIGH 500, LOW 500), HIGH 1000, LOW 500, 3 × (HIGH 750, LOW 750), Key1 + Key2 at 500
µs, LOW 10000 (518 entries). The Flipper loops the upload while the button is held (ProtoPirate repeat
10, ARF 1); KAT sends it once.

Buttons (ProtoPirate's emulate mapping): Lock → 0x20 / 2, Unlock → 0x10 / 1, Trunk → 0x40 / 4 (full
byte for Type 1, nibble otherwise), Panic → 8 (nibble types only); other commands keep the captured
button.

**Deviation:** ProtoPirate and ARF convert a Type 1 frame with type byte 0x00 into Type 2 (XTEA, prefix
0xAF1C) before sending. The real Type 1 captures `IMPORTS/VAG/Passat_2015.sub` and `Vw pasat B7.sub`
carry type byte 0x00 and decrypt as Type 1 with AUT64 key 1 (consistent serial, consecutive counters),
so KAT keeps Type 1.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM (OOK)
- **Encryption:** AUT64 (3 keys from the embedded keystore, index 1–3) / XTEA
- **Frequencies:** 433.92 / 434.42 MHz (ProtoPirate), 315 MHz (ARF)

## Validation

Compared with the compiled ProtoPirate `vag.c` + `aut64.c` (host harness): decoder output identical on
every IMPORTS capture in both polarities (198 frames: Golf4 38, Test_55 69, VW_Skoda_Seat_2008 31,
Vw_passat_b5_3bg 27, Vw pasat B7 12, Passat_2015 8, VAG_2 8, Skoda Fabia 5) and on 4508 frames from 3000
C-encoded uploads (plain, ±70/±115 µs jitter, glitched); encoder upload identical pulse-for-pulse on 3000
random vectors (all types, key indices, buttons, counter wrap). `IMPORTS/VAG/Passat_b5_3bg.sub` (a
decoded key file) is one of the `Vw_passat_b5_3bg.sub` frames. The VW Atlas captures decode in no
reference. Unit tests cover the captures, every type's encode→decode round trip, the legacy `extra`
layout and AUT64 golden vectors.
