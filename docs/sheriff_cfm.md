---
layout: default
---

# Sheriff CFM Protocol

**Rust module:** `src/protocols/sheriff_cfm.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/sheriff_cfm.c`

## Overview

Sheriff is a Russian aftermarket car-alarm brand; CFM covers the ZX-750 and ZX-930 remotes. Flipper-ARF is
the only copy in REFERENCES. The air format is the standard KeeLoq PWM frame (the same state machine as
KAT's `keeloq.rs`), but the 64 bits are not bit-reversed (fix first, MSB-first) and the hop is byte-shuffled
per model before KeeLoq decryption with that model's manufacturer key.

Both keys are already in KAT's embedded keystore (verified equal to the reference's π-obfuscated values):
ZX-750 = entry `ZX-730-750-1055`, ZX-930 = entry `Pandora_M101`. The module looks them up by name; no key
material is copied into the source.

## Timing

| Parameter   | Value                    | Notes                              |
|-------------|--------------------------|------------------------------------|
| Short       | 400 µs                   | ±140 µs                            |
| Long        | 800 µs                   | ±280 µs                            |
| Preamble    | ≥3 × 400 µs HIGH         | 11 pairs + 1 pulse transmitted     |
| Header      | 4000 µs LOW              | ±1400 µs                           |
| Bit 1 / 0   | 400 HIGH + 800 LOW / 800 HIGH + 400 LOW |                     |
| End         | LOW ≥ 940 µs             | 64–66 bits, first 64 kept          |

## Frame Layout (64 bits, MSB-first)

- **bits 63..32 — fix:** 32-bit serial (`serial`)
- **bits 31..0 — hop:** KeeLoq ciphertext after the model's byte shuffle
- **plaintext:** `button << 24 | (serial & 0x3FF) << 16 | counter` — button 0x10 Lock, 0x20 Unlock,
  0x40 Trunk, 0x80 Panic

Byte shuffles (hop as little-endian bytes, decrypt direction): ZX-750 swaps the nibbles of bytes 0 and 2;
ZX-930 inverts byte 0, swaps bytes 1/2, rotates byte 0 right, swaps the nibbles of byte 1 and rotates it
left twice.

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK)
- **Encryption:** KeeLoq (ZX-750 / ZX-930 manufacturer keys)
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset / CheckPreamble** — count 400 µs HIGH pulses; after ≥3, a ~4 ms LOW starts data.
2. **SaveDuration / CheckDuration** — classify HIGH+LOW pairs (short+long = 1, long+short = 0).
3. **End** — a LOW ≥ 940 µs with 64–66 bits triggers the decrypt; for ZX-750 then ZX-930 undo the byte
   shuffle, KeeLoq-decrypt, and accept when the button byte is valid and the 10 serial bits match the fix
   (≈1/32768 for a random frame). Only validated frames are emitted.

## Encoder

The reference rolling-code encoder: model from the decrypt, counter + 1 (wraps to 0 past 0xFFFF), plaintext
re-encrypted with the model key, forward byte shuffle, fix kept. Upload: 11 × (400 HIGH, 400 LOW), 400 HIGH,
4000 LOW, 64 PWM bits, 400 HIGH, 16 ms LOW, sent 3 times (C `encoder.repeat`). KAT buttons map 1:1 onto
the reference codes (Lock 0x10, Unlock 0x20, Trunk 0x40, Panic 0x80).

## Registry interaction

`keeloq.rs` accepts the same KeeLoq-PWM frames and fires on the same terminating pulse, so Sheriff CFM is
registered BEFORE it: KeeLoq's keystore identification accepts a random frame ≈6% of the time, while a
genuine KeeLoq frame passes Sheriff's check ≈1 in 32768. The ignored diagnostic `sheriff_cfm_registry_claims` confirms that 400/400 genuine
frames per band are reported as Sheriff CFM. Star Line and Scher-Khan timings cannot match this decoder;
the generic KeeLoq fallback only reads 68-bit Kia V3/V4 and 250/500 µs Star Line layouts.

## Validation

C oracle (unmodified ARF `sheriff_cfm.c`): decoder field-for-field and encoder pulse-for-pulse (3 copies)
on randomized frames for both models and every button — no mismatch. Unit tests: keystore keys present,
byte-shuffle inverses, synthetic decode for both models, encode→decode for every button (counter + 1, wrap
at 0xFFFF), rejection of corrupted hop / serial bits and 63-bit frames, and a KeeLoq frame never decoding as
Sheriff CFM. No IMPORTS capture is a Sheriff remote.
