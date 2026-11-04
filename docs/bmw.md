---
layout: default
---

# BMW Protocol

**Rust module:** `src/protocols/bmw.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/bmw.c` (doctor-firmware ships the same wire protocol)

## Overview

BMW rolling-code keyfobs as decoded by Flipper-ARF's `"BMW"` protocol: 62 µs Manchester chips, a long
alternating preamble, a 16-bit sync word and a 128-bit frame carrying a 32-bit serial, a 2-bit button, a
17-bit counter and a 47-bit encrypted hop. The ARF and doctor-firmware copies are identical on the wire;
they differ only in Flipper custom-button plumbing (ARF: UP→Lock, DOWN→Unlock, LEFT→Trunk). KAT follows
ARF, whose Lock/Unlock/Trunk mapping lines up 1:1 with KAT's generic button codes.

Flipper-ARF added this protocol in October 2026 as the replacement for its BMW CAS4 decoder, which it
had disabled in September; KAT followed and retired its BMW CAS4 port. Every BMW chip is 40–160 µs and
any pulse ≥400 µs is a frame gap here, so slower Manchester streams never enter this decoder.

## Timing

| Parameter        | Value          | Notes                                         |
|------------------|----------------|-----------------------------------------------|
| TE (chip)        | 62 µs          | 1T window 40–90 µs                            |
| 2T               | 124 µs         | window 100–160 µs                             |
| Frame gap        | ≥400 µs        | ends the frame; may complete the last bit     |
| Preamble         | 32–280 chips   | counted LOW chips; encoder sends 212 "1" bits |
| Header hunt      | ≤40 bits       | for sync `0x599E` / inverted `0xA661`         |
| Repeat gap       | 200 ms         | encoder; 3 copies                             |
| Frame            | 128 bits       |                                               |

Any other duration resets the decoder. KAT's live demodulator debounces pulses shorter than 40 µs, so live
capture of this 62 µs protocol needs a clean, strong signal; raw `.sub` imports are unaffected.

## Frame Layout (128 bits, MSB first)

Manchester chips are paired: `(0,1)` = bit 1 (low→high), `(1,0)` = bit 0 (high→low).

| Bits      | Field                | Notes                                              |
|-----------|----------------------|----------------------------------------------------|
| 127–112   | sync                 | `0x599E` (inverted capture: `0xA661`, payload inverted back) |
| 111–80    | serial (32)          |                                                    |
| 79–72     | button byte          | `0x01 \| (xx << 3)`; xx: 1 = Unlock, 2 = Lock, 3 = Trunk |
| 71–64     | fixed                | `0x89`                                             |
| 63–48     | counter[16:1]        |                                                    |
| 47        | counter[0]           | 17-bit counter                                     |
| 46–0      | hop (47)             | encrypted rolling code                             |

KAT packing: `data` = bits 127–64 (sync, serial, button byte, 0x89), `extra` = bits 63–0 (counter + hop),
`data_count_bit` = 128. `button` = xx (the protocol's own code), `counter` = the low 16 bits of the 17-bit
counter (the full counter stays in `extra`).

## RF

- **Encoding:** Manchester (62 µs chips, decoded pairwise)
- **RF modulation:** AM/OOK
- **Encryption:** rolling code (47-bit hop, no key available) — gated on sync, fixed byte and button template
- **Frequencies:** 433.92 MHz (the reference only sets `SubGhzProtocolFlag_433`)

## Decoder Steps

1. **Reset / AGC** — an optional 2T HIGH AGC burst followed by LOWs, or straight into the preamble.
2. **Preamble** — alternating 1T chips; LOW chips are counted (32–280). The first 2T pulse with ≥32
   counted enters **Header**, keeping its second chip as the first half of the first header bit.
3. **Header** — pair chips into bits and shift them into a 16-bit register until it equals `0x599E`
   (normal) or `0xA661` (inverted polarity: all following bits are inverted); give up after 40 bits.
4. **Data** — the 128-bit register is seeded with the 16 sync bits and filled to exactly 128 bits; a
   ≥400 µs gap completes a last bit whose second chip merged into the gap.
5. **Validate** — button byte `& 0xE7 == 0x01` with xx ≠ 0 and fixed byte `0x89`; emit, then reset.

## Encoder

Supported, as in the C (`bmw_get_upload`). The hop is encrypted and no key is available, so the encoder
rebuilds the payload from the captured serial, counter and hop (no counter bump — the C has none) and only
re-stamps the button for a Lock/Unlock/Trunk command (other commands keep the captured button). Upload:
212 preamble "1" bits, the 128 frame bits (62 µs chips, adjacent levels merged) and a 200 ms LOW gap,
repeated 3 times (the C's `BMW_REPEAT_DEFAULT`).

## Validation

No local capture decodes as BMW (none of the IMPORTS captures is a 62 µs BMW fob). Unit tests cover the
field packing, encode→decode round trips for every button mapping (3 decodes per upload), inverted-polarity
decoding via the `0xA661` sync, a last bit completed by the gap, rejection of a corrupted fixed byte /
button template / zero button / too-short and too-long preambles, and a registry check that BMW frames
decode only as `BMW` while a CAS4-rate stream never leaves the BMW decoder's reset state.
