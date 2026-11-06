---
layout: default
---

# Mercedes V1 Protocol

**Rust module:** `src/protocols/mercedes_v1.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/mercedes_v1.c` (doctor-firmware ships a near-identical copy)

## Overview

Mercedes-Benz keyfobs (2-FSK, 433 MHz) using 50 µs Manchester chips: a long alternating preamble (the
encoder sends 561 bits), a 2T sync, and a 122-bit payload. The payload carries a 32-bit serial, a 1-bit button
(Lock/Unlock), a 10-bit counter and a 54-bit hop code. The hop code is opaque (no key), so TX is a replay;
the C can re-stamp the button bit together with its check bit.

**Reference choice:** the ARF and doctor-firmware copies have identical decoders, field layouts, payload
rebuilds and burst builders. They differ only in custom-button glue. KAT follows ARF, which is
self-contained and keeps the original button by default.

## Timing

| Parameter   | Value      | Notes                                                    |
|-------------|------------|----------------------------------------------------------|
| Chip (TE)   | 50 µs      | 1T = 25–79 µs                                            |
| Double chip | 100 µs     | 2T = 80–140 µs; 141–279 µs resets                        |
| Gap         | ≥ 280 µs   | resets the decoder                                       |
| Preamble    | ≥ 32 pairs | counted on each 1T LOW; TX sends 561 "1" bits            |
| Repeat gap  | 50 ms      | after each burst; the C default repeat count is 4        |
| Payload     | 122 bits   |                                                          |

A full burst is (561 + 122) × 100 µs ≈ 68.3 ms.

## Frame Layout (122 bits, stored numbering, MSB first)

- **bit 0:** always 1 (also the polarity reference)
- **bit 1:** button (0 = Lock, 1 = Unlock)
- **bit 2:** counter state
- **bit 3:** check bit = `!(bit1 ^ bit2)`
- **bit 13:** repeat flag (`Rpt = bit13 ? 0 : 1`)
- **bits 16..47:** serial (32-bit)
- **counter (10-bit):** bit 48 → 0x200, bit 49 → 0x100, bits 57..64 → counter bits 0..7 (LSB first)
- **bits 67..120:** hop code (54 bits)
- **bit 121:** always 1

On air the encoder sends `!stored` per bit (HIGH→LOW = 1). The decoder normalises polarity: if received
bit 0 is 0, the frame is inverted. `DecodedSignal` packing: `data` = stored bits 0..63, `extra` = stored
bits 64..121 right-aligned (58 bits), `data_count_bit` = 122, `counter` = the 10-bit counter.

## RF

- **Encoding:** Manchester (chip pairs: HIGH→LOW = 1, LOW→HIGH = 0)
- **RF modulation:** FM (2-FSK, ~20 kbaud; doctor-firmware's dedicated Mercedes CC1101 preset is 2-FSK at
  19.99 kBaud)
- **Encryption:** rolling code (54-bit hop, no key; replay only)
- **Frequencies:** 433.92 MHz (C flags 433 only)

**Live capture:** KAT's live demodulator debounces pulses shorter than 40 µs, which leaves only ~10 µs of
margin under the 50 µs chip. Live decoding therefore needs a clean, strong signal. Raw `.sub` imports are not
debounced and are unaffected.

## Decoder Steps

1. **Gap / chips:** a pulse of 280 µs or more resets. Otherwise classify the pulse as 1T or 2T; anything else
   resets (outside Reset).
2. **Preamble:** alternate PreambleLow and PreambleHigh on 1T pulses, counting each 1T LOW. A 2T pulse of the
   expected level after at least 32 pairs enters Data. Its second chip opens the first data pair.
3. **Data:** each opposite-level chip pair is one logical bit. An equal-level pair resets.
4. **Validate (122 bits):** normalise polarity on bit 0, then require bit 3 = `!(bit1 ^ bit2)` and bit 121 = 1.
   KAT also rejects frames whose leading or trailing constant run is 32 bits or longer: that is preamble
   decoded as data (e.g. after a jitter-stretched 2T mid-preamble), and the C would accept such an all-ones
   frame.
5. **Resync:** a rejected frame whose trailing run is 32 bits or longer is treated as more preamble, so the
   genuine frame that follows still decodes (as in the C).
6. **End (KAT):** the C reports the frame at its 122nd bit. KAT holds it for one more pulse and reports it
   only if that pulse is not a chip (outside 25–140 µs). A chip there means a longer frame: BMW's 128-bit
   frames use the same template at 62 µs chips and would otherwise be cut at bit 122 and accepted by the
   weak field check. KAT 1.8.5 and earlier required a gap of 280 µs or more there, which a real fob does not
   always leave (see Validation).

## Encoder

Supported, as a replay. KAT Lock/Unlock re-stamp the button bit and its check bit 3 (via
`mercedes_v1_rebuild_payload`); other buttons keep the captured bit. There is no counter bump (the C has
none). Each upload is 561 preamble "1" bits + 122 air bits (`!stored`) + a 50 ms LOW gap, emitted 4 times
(the C's default repeat count).

## Validation

**A volunteer's 2015 Mercedes (FSB4 system)**, recorded by a Flipper Zero at 433.95 MHz with a custom 2-FSK
setting (±19 kHz, 20 kBd), is not in `IMPORTS`. doctor-firmware and Flipper-ARF read 9 codes from its two
files (4 lock, 5 unlock; 7 and 6 frames). KAT 1.8.5 read 3: the fob ends most frames with a 190–330 µs LOW
followed by noise, not a gap of 280 µs or more. KAT now reads all 9, with the same frame counts.

The two `IMPORTS/mercedes` files do not decode:

- **`24_Merc_gle350_lulu.sub` (434.075 MHz):** noise only (159 pulses; longest run of chip-valid pulses: 3).
- **`24_merc_gle350_luluttlulu.sub` (314.35 MHz):** recorded with the AM650 (OOK) preset. It contains 23
  solid carrier bursts of 67.3–67.6 ms (mean 67.5 ms). That is within about 1% of a full V1 burst (683 bits ×
  100 µs = 68.3 ms), so these are most likely genuine Mercedes FSK transmissions. However, an OOK receiver captures only
  their envelope: the longest run of chip-valid pulses is 10, and no bits are recoverable. Re-capture with a
  2-FSK preset to decode.

Unit tests cover:

- the field layout round trip
- the synthetic frame, in both polarities
- encode→decode with the captured button, and button re-stamping
- the burst structure
- rejection of bad check bits and a short preamble
- the stretched-preamble-pulse case (rejected, then the real frame decodes)
- a registry-level decode
- a cross-protocol guard: no other protocol's encoder output decodes as Mercedes V0/V1 or Opel V1
