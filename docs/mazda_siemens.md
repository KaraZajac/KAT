---
layout: default
---

# Mazda Siemens Protocol

**Rust module:** `src/protocols/mazda_siemens.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/mazda_siemens.c`

## Overview

The Siemens/VDO keyfob layer used on some Mazda vehicles. Its air format is **identical** to
[Mazda V0](mazda_v0.md): the Flipper-ARF Siemens encoder and ProtoPirate's Mazda V0 encoder emit the same
waveform (verified by compiling both C references). What differs is the decoder — Flipper-ARF uses a
pair-based state machine that ignores `level` and interprets raw durations in pairs (`process_pair`),
collecting bits with *inverted* polarity — and the field view: an 8-bit button byte and a 16-bit counter
instead of Mazda V0's button nibble + 20-bit counter.

The Siemens obfuscation (`mazda_xor_deobfuscate`) — parity-dependent XOR mask, then a bit-deinterleave of
bytes 5/6 — is reversed; the gate is the additive checksum `sum(data[0..7]) == data[7]` plus the
preamble/sync/bit-count structure.

> **Registry:** Mazda V0 sits first and also completes a shared frame earlier (after the 64th data bit;
> this decoder waits for the trailing gap), so live captures of this air format are reported as
> "Mazda V0". Mazda Siemens only reports frames Mazda V0 rejects, and provides the Siemens-layout encoder.

## Timing

| Parameter   | Value     | Notes                                   |
|-------------|-----------|-----------------------------------------|
| Short       | 250 µs    | ±100 µs (te_delta)                      |
| Long        | 500 µs    | ±100 µs                                 |
| Gap         | 50 000 µs | LOW, after the wake-up preamble and the frame |
| Min bits    | 64        |                                         |
| Preamble    | ≥13 short/short pairs |                             |
| Completion  | 79–105 collected bits | C: 80–105 (see below)       |

## Frame Layout (64 bits / 8 bytes)

On air: `12 × 0xFF | LOW 50 ms | 0xFF 0xFF 0xD7 | 8 obfuscated bytes sent as 255 − byte | 0x5A | LOW 50 ms`
(MSB first; bit 1 = HIGH→LOW, bit 0 = LOW→HIGH; 250 µs half-bits).

The decoder enters data on the first short→long pair (inside the 0xD7 sync); the 14-byte buffer's leading
7-bit sync byte is discarded, leaving 8 bytes that are de-obfuscated into the cleartext kept in `data`:

- **serial:** bytes 0–3 (`data >> 32`)
- **button:** byte 4 (`(data >> 24) & 0xFF`) — Lock=0x10, Unlock=0x20, Trunk=0x40
- **counter:** bytes 5–6 (`(data >> 8) & 0xFFFF`)
- **byte 7:** additive checksum of bytes 0–6

## RF

- **Encoding:** Manchester, read by a pair-based state machine; 250/500 µs
- **RF modulation:** FM
- **Encryption:** Siemens obfuscation (parity XOR + bit-interleave); gated on the additive checksum
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset** — a short pulse begins the preamble.
2. **PreambleSave/PreambleCheck** — count short/short pairs (≥13); a short→long pair seeds the leading
   sync bit and enters data.
3. **DataSave/DataCheck** — process duration pairs into bits; on a non-matching pair (the trailing gap),
   run `check_completion` (79–105 bits, discard the sync byte, de-obfuscate, validate the checksum) and emit.

### Deviation from the C: completion minimum 79 (C: 80)

The C decoder cannot decode the C encoder's frames. A bit is only collected when the pair holding its
half-bits is classified, and the frame's final half-bit merges into the trailing LOW gap, so the pair holding
the last `0x5A` tail bit never classifies. A complete frame always ends with 79 collected bits (7 sync +
64 data + 7 tail bits), one below `MAZDA_COMPLETION_MIN` = 80. KAT lowers the minimum by one — the data
bytes are complete at 72 and nothing else changes, so KAT still accepts every frame the C accepts. On the
host, the unmodified C decoder decodes 0/2000 C-encoder frames; with only this constant changed it decodes
5000/5000, and 300/300 frames from ProtoPirate's Mazda V0 encoder.

## Encoder

Supported (`subghz_protocol_encoder_mazda_siemens_deserialize` + `get_upload`). The cleartext is rebuilt
from the fields — serial into bytes 0–3, the button into **byte 4**, counter + 1 (16-bit, the fresh press)
into bytes 5–6 — the checksum is recomputed, the bytes are interleaved + XORed (`mazda_xor_obfuscate`) and
the upload above is emitted once (the Flipper repeats it 4×). Generic buttons map like the C's D-pad remap:
0x01 → 0x10 Lock, 0x02 → 0x20 Unlock, 0x04 → 0x40 Trunk; anything else (including Panic, which this fob
lacks) keeps the captured button. Adjacent same-level half-bits are merged; the waveform is otherwise
pulse-for-pulse the C upload.

> Fixed in this revision: the encoder used to write the button into byte 3 (the serial's low byte, the
> comment claimed `data >> 24`, which is byte 4), fell back to Unlock for unknown buttons, and emitted
> unmerged half-bits; together with the completion off-by-one its output never decoded.

## Validation

- Reference C (Flipper-ARF `mazda_siemens.c`) compiled on the host: KAT's `encode()` output equals the C
  upload in 300/300 cases; KAT's decoder decodes 300/300 frames from the C Siemens encoder and 300/300 from
  ProtoPirate's Mazda V0 encoder.
- Unit tests: golden cleartext/air vectors from the C, 500-frame encode→decode round trip (serial, mapped
  button, counter + 1), the byte-4 button regression, unknown/Panic buttons keep the captured one, the
  79-bit completion pinned, corrupted checksum rejected, the obfuscation layer (both parity branches), and
  the registry reporting a shared frame as Mazda V0 with equivalent fields.
- Real captures: none of `IMPORTS/Mazda/*.sub` decode (see [Mazda V0](mazda_v0.md#validation)).
