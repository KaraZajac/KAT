---
layout: default
---

# Mercedes V0 Protocol

**Rust module:** `src/protocols/mercedes_v0.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/mercedes_v0.c` (doctor-firmware ships a near-identical copy)

## Overview

Mercedes-Benz keyfobs (2-FSK, 433 MHz). A 112-bit Manchester frame (TE = 500 µs) follows a 3700 µs lead gap
and a 55-pair preamble. The top nibble is a command code that must be in the whitelist "from the PIC
receive classifier" (0xC Lock, 0x8 Unlock, 0xA "??"/Aux, 0x0 Follow-up, 0x4). The remaining 108 bits are an
opaque rolling payload with no known key and no CRC, so TX is a byte-exact replay with only the command
nibble re-stamped.

**Reference choice:** the ARF and doctor-firmware copies have identical decoders, validation and burst
builders. They differ only in custom-button glue: ARF maps the d-pad inline and keeps "OK = replay the
captured command"; doctor moves the map into `custom_btn.c` and drops the replay option. KAT follows ARF.
ARF's disabled `mercedes.c` (a Pandora-firmware RE) documents the same wire format but a generic Pandora field
layout (b0..b3 UID, b4 button, …) that conflicts with V0's command nibble, so it is not ported and no
serial/counter is extracted (the C also sets `serial = 0` and hides the counter).

## Timing

| Parameter        | Value     | Notes                                                        |
|------------------|-----------|--------------------------------------------------------------|
| Chip (TE)        | 500 µs    | 1 chip = 350–650 µs (exclusive)                              |
| Double chip      | 1000 µs   | 2 chips = 850–1150 µs; any other duration resets             |
| TX lead gap      | 3700 µs   | LOW before the preamble                                      |
| Preamble         | 55 pairs  | decoder needs **108–113** single-chip edges before the sync  |
| Follow-up gap    | 300 ms    | between the Unlock burst and its Follow-up burst             |
| Frame            | 112 bits  |                                                              |

## Frame Layout (112 bits, MSB first)

- **bits 111..108:** command nibble (= `button`): 0xC Lock, 0x8 Unlock, 0xA "??"/Aux, 0x0 Follow-up, 0x4 (unnamed;
  KAT shows it as "Other")
- **bits 107..0:** opaque rolling payload (replayed unchanged)

`DecodedSignal` packing: `data` = first 64 wire bits (command nibble on top), `extra` = last 48 wire bits
right-aligned, `data_count_bit` = 112. `serial`/`counter` = none. `crc_valid` = false: the frame has no CRC (the
gate decides emission; the interface shows "-", not a failed check).

## RF

- **Encoding:** Manchester, bit 1 = LOW→HIGH, bit 0 = HIGH→LOW (the sync pair auto-detects polarity)
- **RF modulation:** FM (2-FSK)
- **Encryption:** rolling code (opaque, no key; replay only)
- **Frequencies:** 433.92 MHz (C flags 433 only)

## Decoder Steps

1. **Chips:** classify each pulse as 1 or 2 chips. Any other duration, or two consecutive pulses of the same
   level, resets the decoder.
2. **Preamble:** count single-chip pulses (saturating at 114). A 2-chip pulse activates the decoder only if
   108–113 edges were counted (the encoder produces 109).
3. **Sync:** the 2-chip pulse opens the sync pair. The first completed pair sets `invert = !bit`, so both
   polarities decode to the same frame.
4. **Data:** each pair of opposite-level chips yields `bit = !first && second`, XORed with `invert`. Bits go
   MSB first into `hi` (0..63) and `lo` (64..111).
5. **Gate:** at 112 bits, emit only if the command nibble is whitelisted and the 108-bit payload is not all 0s
   or all 1s (KAT addition; a run of preamble pairs read as data would otherwise pass with command 0x0).
6. **Final bit at a gap (KAT addition):** if the last bit is 0, its LOW half merges into the trailing silence
   and the C can never finish the frame. KAT completes the 112th bit on a ≥1150 µs opposite-level gap.

## Encoder

Supported, as a replay. Burst = 3700 µs LOW + 55 × (HIGH 500 / LOW 500) + sync (LOW 500, HIGH 500) +
112 Manchester bits. Adjacent equal levels merge. KAT Lock → 0xC, Unlock → 0x8, Trunk → 0xA (the C's
"Auxiliary"; its real function is unverified). Any other button keeps the captured command. When Unlock is
selected, the C (and KAT) appends a 300 ms gap and a second burst with command 0x0 (Follow-up). There is no
counter bump, and the upload is a single copy (the C's default `Repeat` is 1; a key file's `Repeat` line sets
the count, in the C and in KAT).

## Key file

The C saves with the generic serializer, and the two references' generic writers differ: Flipper-ARF's writes
`Protocol: Mercedes V0`, `Bit: 112` and `Key` (the first 64 bits) only, losing the last 48; doctor-firmware's
adds `Serial: 0`, `Btn` (the command nibble) and `Cnt: 0` when the command is not 0, `TE: 500`, and `Key_2` (bits
64..111 in its top 48 bits) when that is not 0. KAT writes doctor-firmware's lines, so its files reload
completely; ARF reads only `Bit` and `Key` from them. Loading (`mercedes_v0_deserialize`) takes `Key` and `Key_2`
(or `Key2`; 0 when absent, so an ARF file loads with the last 48 bits 0, as in ARF) and refuses a file unless it
has 112 bits, `Key_2`'s low 16 bits are 0 and the command nibble is in the whitelist. The degenerate-payload
gate is a receive-side filter and does not apply to a loaded key.

## Validation

Verified with synthetic frames and encode→decode round trips; no decodable capture is available.
`IMPORTS/mercedes/24_Merc_gle350_lulu.sub` (434.075 MHz) contains only noise (longest run of chip-valid
pulses: 2). `24_merc_gle350_luluttlulu.sub` (314.35 MHz) is an AM650 (OOK) recording of FSK bursts, so only
the carrier envelope survives and no decoder can recover data from it (see [Mercedes V1](mercedes_v1.md)).
`Mercedes W220 S430 1999 315.sub` (315 MHz) holds noise only ([Unread recordings](unidentified_captures.md#1999-mercedes-benz-s-430-w220)).
Unit tests cover:

- the synthetic frame, in both polarities
- byte-exact replay and command re-stamping
- the Unlock + Follow-up double burst
- the final-bit-at-gap completion
- rejection of non-whitelisted commands, preambles outside the edge window, a corrupted Manchester pair,
  and degenerate payloads
- a registry-level decode at 433.92 MHz
- key files: doctor-firmware's lines round-trip, an ARF file (no `Key_2`) loads with a zero tail, invalid keys
  are refused, and a loaded key transmits
