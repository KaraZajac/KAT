---
layout: default
---

# Mazda V0 Protocol

**Rust module:** `src/protocols/mazda_v0.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/mazda_v0.c` (current ProtoPirate; Flipper-ARF
`lib/subghz/protocols/mazda_v0.c` is the same decoder/encoder)

## Overview

Mazda keyfob frames: a 250/500 µs Manchester burst carrying a 64-bit key — 32-bit serial, 4-bit button,
20-bit rolling counter and an additive checksum — whitened with a parity-selected XOR mask plus a
bit-interleave of the two low counter bytes. The decoder aligns on the `FF FF D7` sync and takes exactly
64 bits; emission is gated on the checksum. ProtoPirate's encoder, Flipper-ARF's Mazda Siemens encoder
and Flipper-ARF's `mazda_infinity.c` all produce this same frame (byte layout, whitening, bit coding).

> **History:** KAT's earlier Mazda V0 was a port of ProtoPirate's *original* pair-based decoder, which
> ProtoPirate replaced in commit 6758a4e ("Fix Mazda V0 decoder and add encoder"). That decoder cannot
> decode this air format: it loses the last bit of every frame to the trailing gap and stops at 79 of the
> 80 bits it requires. The same algorithm lives on as [Mazda Siemens](mazda_siemens.md) (with that
> off-by-one fixed in KAT).

## Timing

| Parameter | Value     | Notes                                          |
|-----------|-----------|------------------------------------------------|
| Short     | 250 µs    | ±120 µs (te_delta 100 + 20, strict)            |
| Long      | 500 µs    | ±120 µs                                        |
| Gap       | 52 000 µs | LOW, after the wake-up preamble and the frame  |
| Preamble  | >15 Manchester ones, then the sync byte 0xD7 |   |
| Data      | 64 bits   |                                                |

## Frame Layout

On air (one upload; MSB first; bit 1 = HIGH→LOW, bit 0 = LOW→HIGH, 250 µs half-bits):

```
12 × 0xFF | LOW 52 ms | 0xFF 0xFF 0xD7 | 8 key bytes, each one's-complemented | 0x5A | LOW 52 ms
```

The 64-bit key (bytes b0..b7, stored in `data` exactly as ProtoPirate stores its `Key`):

1. Un-whiten: parity(b7) odd → XOR b0..b5 with b6; even → XOR b0..b4 and b6 with b5.
2. **serial** = b0..b3 (32 bits); **button** = b4 >> 4; **counter** (20 bits) = `(b4 & 0x0F) << 16 |
   mid << 8 | lo` with `mid = (b6 & 0x55) | (b5 & 0xAA)` and `lo = (b5 & 0x55) | (b6 & 0xAA)`.
3. **b7** = checksum = (Σ serial bytes + mid + lo + (button << 4 | counter[19:16])) mod 256.

| Button | Name   | KAT generic code |
|--------|--------|------------------|
| 0x1    | Lock   | 0x01             |
| 0x2    | Unlock | 0x02             |
| 0x4    | Trunk  | 0x04             |
| 0x8    | Remote | 0x08 (Panic)     |

KAT packing: `serial` 32-bit, `button` = the 4-bit code, `counter` = the low 16 bits of the 20-bit counter
(the top nibble stays in `data`, from which the encoder recovers it), `data_count_bit` = 64.

## RF

- **Encoding:** Manchester, 250/500 µs
- **RF modulation:** AM and FM (ProtoPirate flags)
- **Encryption:** none — XOR whitening + counter bit-interleave, additive checksum
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset** — a HIGH short pulse starts the preamble; the Manchester state is seeded to Mid1.
2. **Preamble** — each pulse becomes a Flipper Manchester event with ProtoPirate's mapping
   (`level ? ShortLow : ShortHigh`, likewise Long). Decoded bits shift into an 8-bit pattern and 1s are
   counted; a 0 before 16 ones restarts the count. With more than 15 ones seen and the pattern equal to
   0xD7, go to Data. A pulse that is neither short nor long returns to Reset.
3. **Data** — collect exactly 64 bits; key = their complement; un-whiten; emit only if the checksum
   matches; reset.

## Encoder

Supported (ProtoPirate encoder + emulate scene). The captured key is decoded, then the `DecodedSignal`
fields override it (as the C lets the file's Serial/Btn/Cnt override the Key). KAT's generic button maps
1:1 onto the frame codes above; any other code keeps the captured button. The counter is bumped by one for
the fresh press and masked to 20 bits, the key is re-encoded (`mazda_v0_encode_key`), and the upload above
is emitted once (the Flipper repeats it 10×). Adjacent same-level half-bits are merged into single pulses;
the waveform is otherwise pulse-for-pulse the C `mazda_v0_build_upload`.

## Validation

- The unmodified reference C (ProtoPirate `mazda_v0.c`) was compiled on the host: its decoder decodes
  2000/2000 of its own encoder's frames. KAT's decoder decodes 300/300 C-encoder frames (and 300/300 from
  the Flipper-ARF Mazda Siemens encoder); KAT's `encode()` output equals the C upload in 300/300 cases.
- Unit tests: golden key vectors from the C (both whitening branches, counter extremes), upload layout,
  500-frame encode→decode round trip, button mapping + counter bump/20-bit wrap, counter-field override,
  corrupted checksum rejected, wrong sync / short preamble rejected, ±100 µs jitter, registry decode at
  315 and 433.92 MHz.
- Real captures: none of `IMPORTS/Mazda/*.sub` decode (with KAT or with the reference C decoders). The
  RX-8 files never hold more than 13 consecutive pulses in the 250/500 µs classes (a frame needs ~250),
  and the CX-50 file is 459 chopped pulses recorded with an OOK preset. No capture in `IMPORTS` produces
  a Mazda decode.
