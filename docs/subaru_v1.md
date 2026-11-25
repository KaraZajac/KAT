---
layout: default
---

# Subaru V1 Protocol

**Rust module:** `src/protocols/subaru_v1.rs`
**Reference:** none. ProtoPirate, Flipper-ARF and doctor-firmware do not read it. The format was worked out from recordings of two keys of a 2014 Subaru (FCC ID CWTWB1U811 family, 315 MHz), with labelled button presses. It is the same family as Wimmenhove's 2005-2011 Subaru finding (command nibble, plaintext counter, XOR-then-+1 check) with a different line code and length. The EU/AU PWM format is [Subaru](subaru.md).

## Overview

The US 2012-2019 remote-head key sends a 57-bit Manchester frame, three times per press, with no encryption: the counter is plaintext and goes up by one per press.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Half-bit | 122 µs | bit period 244 µs |
| Preamble pulse | 244 µs | HIGH first, alternating; accepted 185-305 µs |
| Preamble, copy 1 | ~520 pulses | KAT sends 527 (odd, ends HIGH) |
| Preamble, copies 2 and 3 | 63 pulses | |
| Gap after each copy | ~1340 µs LOW | |
| Frame | 57 bits | start bit 0 + 7 bytes |

## Decoding

After 20 or more preamble pulses, the first pulse that is not a preamble pulse must be a LOW of one half-bit (the start bit's first half). From there the pulses expand into half-bit levels (a pulse of 45-185 µs is one half-bit, 185-305 µs two, longer is the gap, which supplies the last bit's LOW half). Pairs read `10` = 1 and `01` = 0, MSB first. The frame is reported when it has exactly 57 bits, the start bit is 0 and the check byte holds. Any invalid pair, wrong length or failed check drops it.

## Frame layout (bytes b0..b6, after the start bit)

| Field | Bits | Meaning |
|-------|------|---------|
| b0..b2 | 24 | key ID (`serial`) |
| b3 high nibble | 4 | button |
| b3 low nibble, b4, b5 | 20 | counter, +1 per press (KAT's `counter` is the low 16 bits; all 20 are in `data`) |
| b6 | 8 | (b0 ^ b1 ^ b2 ^ b3 ^ b4 ^ b5) + 1, mod 256 |

`data` holds b0..b6 (56 bits), `data_count_bit` is 56.

Example (made-up key): ID `5A3C91`, Unlock, counter `0x00457` is `5A 3C 91 20 04 57 85`.

## Buttons

| Native | KAT | Status |
|--------|-----|--------|
| 0x1 | Lock | confirmed by labelled presses |
| 0x2 | Unlock | confirmed |
| 0xB | Trunk | confirmed |
| 0xA | Panic | recorded (the same code as in Wimmenhove's table for the older format) |

KAT's generic commands map Lock to 0x1, Unlock to 0x2, Trunk to 0xB and Panic to 0xA.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM (OOK), preset AM650 (the recordings were made with OOK270 and OOK650)
- **Encryption:** none
- **Frequency:** 315 MHz

## Encoder

Supported. Counter + 1 (20 bits, wrapping), the requested button, the check byte rebuilt, then three copies as the fob sends them: 527 preamble pulses, the frame, a 1340 µs LOW; 63 preamble pulses, the frame, the gap; again. The upload decodes back through the decoder (three identical frames). `sent_signal` reports the frame sent, so the capture's counter advances in the app.

## Key files

KAT's own format (no firmware has one): `Protocol: Subaru V1`, `Bit: 56`, `Key` (7 bytes), then decimal `Serial`, `Btn` (native nibble) and `Cnt` (20 bits). Everything is read back from the Key; one whose check byte fails does not load.

## Not known

- Panic (0xA) has not been recorded.
- Whether other 2012-2019 US models (other FCC IDs) use the same frame.
- The 2016 Denso HYQ14AHC key (FM, 314.35 MHz) is a different, unidentified format.
