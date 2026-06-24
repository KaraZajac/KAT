---
layout: default
---

# Mazda Siemens Protocol

**Rust module:** `src/protocols/mazda_siemens.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/mazda_siemens.c`

## Overview

The Siemens/VDO keyfob cipher used on some Mazda vehicles — a DIFFERENT protocol from KAT's existing
"Mazda V0" (Pandora) decoder, even though both ride a 250/500 µs pair-based stream at 433.92 MHz FM. The
decoder is pair-based: `feed()` ignores `level` and interprets raw durations in pairs (`process_pair`),
collecting bits with *inverted* polarity (`state_bit == 0` → stored 1). 64-bit frame.

Siemens obfuscation (`mazda_xor_deobfuscate`): parity-dependent XOR mask, then bit-deinterleave of bytes
5/6. The inner Siemens cipher's plaintext is left as-is (no key); only this obfuscation/interleave layer
is reversed. The gate is an additive checksum `sum(data[0..7]) == data[7]`, plus the structural
preamble/sync/bit-count constraints, which keeps it from false-matching other 250/500 µs Manchester
protocols.

> **Note:** KAT's existing "Mazda V0" decoder implements the same algorithm and keeps first-match
> priority, so on a shared frame Mazda V0 fires first.

## Timing

| Parameter   | Value  | Notes                       |
|-------------|--------|-----------------------------|
| Short       | 250 µs | ±100 µs (te_delta)          |
| Long        | 500 µs | ±100 µs                     |
| Min bits    | 64     |                             |
| Preamble    | ≥13 short/short pairs |              |
| Completion  | 80–105 collected bits |              |

## Frame Layout (64 bits / 8 bytes, after deobfuscation)

The 14-byte buffer's leading sync byte is discarded, leaving 8 bytes:

- **serial:** `data >> 32` (32-bit)
- **button:** `(data >> 24) & 0xFF` — Lock=0x10, Unlock=0x20, Trunk=0x40
- **counter:** `(data >> 8) & 0xFFFF` (16-bit)
- **byte 7:** additive checksum of bytes 0–6

## RF

- **Encoding:** pair-based Manchester (inverted polarity), 250/500 µs
- **RF modulation:** FM
- **Encryption:** Siemens obfuscation (parity XOR + bit-interleave); gated on the additive checksum
- **Frequencies:** 433.92 MHz only

## Decoder Steps

1. **Reset** — a short pulse begins the preamble.
2. **PreambleSave/PreambleCheck** — count short/short pairs (≥13); a short→long transition seeds the leading sync bit (a 1) and enters data.
3. **DataSave/DataCheck** — process duration pairs into bits; on a non-matching pair, run `check_completion` (discard sync byte, deobfuscate, validate additive checksum) and emit.

## Encoder

Supported (`subghz_protocol_encoder_mazda_siemens_get_upload`). Increments the counter byte, recomputes
the checksum, obfuscates (interleave + XOR), then emits a 12-byte 0xFF preamble, a 50 ms gap, `0xFF 0xFF
0xD7`, the 8 obfuscated bytes transmitted as `255 − byte`, a `0x5A` tail, and a trailing 50 ms gap.
Manchester per byte: bit 1 → (H,L), bit 0 → (L,H) at te_short. As in the C, the decoder and encoder are
not a clean raw-timing round-trip pair (the encoder targets a TX upload).

## Validation

Verified by an encode→decode round-trip / synthetic-frame check (no local capture available). Unit tests
cover the obfuscate/deobfuscate XOR+interleave cipher layer (a true inverse, both parity branches), the
field layout, and the TX upload smoke test.
