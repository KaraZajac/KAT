---
layout: default
---

# KeeLoq Protocol

**Rust module:** `src/protocols/keeloq.rs` (+ `src/protocols/keeloq_common.rs`)
**Reference:** `REFERENCES/Flipper-ARF/lib/subghz/protocols/keeloq.c`, `keeloq_common.c`
(Unleashed lineage; ProtoPirate has no KeeLoq protocol)

## Overview

Generic Microchip KeeLoq (HCS2xx/HCS3xx-style) remotes: gates, garage doors, barriers and car alarms. The
decoder identifies the manufacturer from the embedded keystore exactly as ARF's
`check_remote_controller` does and reports `KeeLoq (<manufacturer>)`. Unidentified frames are not
reported (the C lists them as "Unknown"); KAT's separate KeeLoq generic fallback
([keeloq_generic.md](keeloq_generic.md)) still tries captures that no decoder reports.

## Timing

| Parameter | Value    | Notes                                                        |
|-----------|----------|--------------------------------------------------------------|
| Short     | 400 µs   | ±140 µs                                                      |
| Long      | 800 µs   | ±280 µs                                                      |
| Preamble  | > 2 × 400 µs HIGH | LOW ~400 µs (or ~800 µs: Superrollo/HCS361)         |
| Header    | 4000 µs  | ±1400 µs (LOW; Superrollo also a 10·Te HIGH sync)            |
| Bit 1 / 0 | 400 HIGH + 800 LOW / 800 HIGH + 400 LOW |                               |
| End       | LOW ≥ 940 µs | 64..66 bits, first 64 kept                               |

## Frame Layout (64 bits, MSB-first, bit-reversed = fix | hop)

- **fix (32 bits):** button (4) << 28 | serial (28)
- **hop (32 bits):** KeeLoq(button << 28 | discriminator << 16 | counter)

## Identification

1. **AN-Motors** — hop = `cnt8 cnt8 btn 0x404` (no key); counter = hop >> 16.
2. **HCS101** — hop = `cnt16 btn 0x000`; counter = hop >> 16.
3. **Reference selector** — each keystore entry with ONLY its learning type: SIMPLE (Pecinin: 12-bit
   serial check, button from the plaintext), NORMAL (Centurion: discriminator 0x1CE), SECURE (BFT: seed 0,
   then seed = serial), MAGIC_XOR_TYPE_1, MAGIC_SERIAL_TYPE_1/2/3, AERF, ERREKA, PUJOL, SIMPLE_JCM, UNKNOWN
   (simple, normal, secure, magic-xor, each with the key and its byte-mirrored copy). Check: plaintext
   button = fix button and plaintext[23:16] = serial[7:0] or 0.

The first match names the manufacturer. A random 64-bit frame is identified ≈6% of the time, so strongly
gated KeeLoq-PWM protocols (Sheriff CFM) are registered before KeeLoq.

## Keystore learning-type correction (DoorHan)

The selector applies each entry's learning type, so the type must be right. KAT's embedded keystore lists
DoorHan (`8455F43584941223`) as NORMAL (2), but every real DoorHan remote available decrypts with that key
under SIMPLE learning (1) and under nothing else (checked against every keystore key × learning mode × byte
order):

| Capture (Flipper-ARF `applications/debug/unit_tests/resources/unit_tests/subghz/`) | Serial | Button | Counters |
|---|---|---|---|
| `doorhan_raw.sub` | 0D36C681 | 2 | 04BD, 04BE, 04BF |
| `test_random_raw.sub` | 0D36C681 | 2 | 04C0, 04C1, 04C2 |
| `test_random_raw.sub` | 047D94C1 | 8, 1 | 0490, 0491 |
| `doorhan.sub` (decoded key `4850F07233789514`) | 08A91ECC | 2 | 0004 |

Three remotes with consecutive counters and discriminator = serial & 0x3FF leave no doubt, so
`keeloq_common::LEARNING_TYPE_FIXES` maps DoorHan to SIMPLE in the keystore view both KeeLoq and Star Line
use. Without it these remotes are not identified at all.

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK)
- **Encryption:** KeeLoq
- **Frequencies:** 315 MHz, 433.92 MHz, 868.35 MHz

## Encoder

ARF `gen_data` + `encode_to_timings` (counter mode 0): manufacturer, learning mode, counter and seed are
re-derived from the key; counter + 1 (wraps to 0). `fix = button << 28 | serial` (Pecinin: serial only).
Discriminator per manufacturer (default serial & 0x3FF; Aprimatic parity form; 12-bit for DTM_Neo,
FAAC_RC,XT, Mutanco/Clemsa, Came_Space, Genius_Bravo, GSN, Rosh, Rossi, Pecinin, Steelmate, Cardin_S449,
Stilmatic, …; 8-bit for NICE_Smilo/MHOUSE, JCM_Tech, Pujol, Erreka; 0 Beninca/Merlin; 0x1CE Centurion; 0x100
Monarch; Dea_Mio's C19 form). Hop by the entry's learning type; UNKNOWN entries per matched mode with the key
as stored (reference quirk). MAGIC_SERIAL_2/3 have no encode case in the C → the captured frame is
re-sent. AN-Motors/HCS101 rebuild their plaintext hop; unidentified → the
captured hop is replayed. Programming mode (button 0xF): BFT sends its seed as hop, Aprimatic 0x1A2B3C4D,
Dea_Mio 0.

Upload: 11 × (400, 400), 400 HIGH, 4000 LOW, 64 bits, status bit (400/800), 400 HIGH, 16 ms LOW (Sommer
11.6 ms); "Superrollo" uses ARF's HCS361 framing. Sent 3 times (C `encoder.repeat`).

Buttons are 4-bit function codes; KAT Lock/Unlock/Trunk/Panic (1/2/4/8) and codes 1..F are sent as is.
For ARF's "shifted" brands (DoorHan, Pandora_*, Alligator_S-275, …) codes 2/4/8/1 are labelled B1..B4.

## Validation

C oracle (unmodified ARF `keeloq.c`/`keeloq_common.c`, KAT's keystore with the DoorHan correction): every
identification matches field-for-field (manufacturer, serial, button, counter, seed) and every encoder
upload pulse-for-pulse on randomized frames for every keystore entry and learning type. Real remotes in
ARF's unit-test captures: three DoorHan remotes (consecutive counters), an AN-Motors and a Stilmatic
remote. The IMPORTS sweep has no KeeLoq capture.
