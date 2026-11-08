---
layout: default
---

# Chevrolet V0 Protocol

**Rust module:** `src/protocols/chevrolet_v0.rs`
**Reference:** `FoxFW lib/subghz/protocols/chevrolet_v0.c` (a port of the KeeLoq-RX `arke_mazda_gm.py` classes
`ChevroletV0` / `ChevroletV0Gm`); FoxFW is the only copy in REFERENCES.

## Overview

A GM 433.92 MHz keyfob with Open (`0xA`) and Close (`0xB`) buttons. OOK Manchester at 120/240 µs, an exact
32-bit preamble word and 96 payload bits. The hop is encrypted and there is no clear counter, so this is
**decode and display only**, as the reference (its encoder is NULL). It is a different air format from [gm.md](gm.md)
(PPM, rtl_433's `gm_rke` / ARF's GM).

## Timing

| Parameter | Value  | Notes                                            |
|-----------|--------|--------------------------------------------------|
| Short     | 120 µs | `diff < 71` (the C's `te_delta`)                 |
| Long      | 240 µs | `diff < 71`                                      |
| Preamble  | `0xAAAAAAAB` | exact, from the 32nd decoded bit on        |
| Payload   | 96 bits | every Manchester bit inverted (IEEE 802.3)      |

## Frame (96 bits)

| Bits  | Field  | Notes                                                                  |
|-------|--------|------------------------------------------------------------------------|
| 95..72 | Serial | 24 bits                                                              |
| 71..68 | Button | `0xA` Open, `0xB` Close; anything else drops the frame               |
| 67..32 | Hop    | 36 bits, encrypted                                                   |
| 31..0  | Seed   | 32 bits, encrypted                                                   |

The top 64 bits (serial, button, hop) are the C's `gdata`. A `gdata` of 0 or all ones drops the frame. The C's
`CHEVROLET_V0_GM_ACCEPT_ANY` option (any button nibble, for the wider GM family) is off in FoxFW and in KAT.

## Decoder state machine

1. **Reset step:** any pulse that is short or long starts the preamble (Manchester state `Mid1`, register cleared)
   and is processed as its first event; other pulses are ignored.
2. **Preamble:** each pulse is a Manchester event (`level ? ShortLow : ShortHigh`, long likewise); a pulse that is
   neither short nor long returns to the reset step. Each decoded bit is inverted and shifted into a 32-bit
   register; from the 32nd bit on, the register equal to `0xAAAAAAAB` starts the payload.
3. **Data:** each bit shifts through a 64-bit word into the register (bits 95..64 end in the register, 63..0 in the
   word). A bad pulse drops the frame. After 96 bits the frame is checked (gdata, button) and reported; the
   decoder returns to the reset step.

No checksum exists: the gate is structural, so the registry places Chevrolet V0 after every decoder with a check
(after the gate remotes, before the rtl_433 ports).

## Field mapping

`data` = `gdata` (64 bits), `data_count_bit` = 96, `serial` = 24 bits, `button` = the nibble, `extra` = the seed,
`counter` None, `crc_valid` false (shown as "-"), `encoder_capable` false.

## Key file

`Protocol: Chevrolet V0`, `Bit: 96`, `Key` = `gdata`, then `Serial`, `Btn`, `HopHi` (the hop's 4 high bits),
`HopLo` (its low 32 bits), `Seed` (`subghz_protocol_decoder_chevrolet_v0_serialize` / `_deserialize`; loading needs
`Bit: 96` and reads `Seed` when present).

## Validation

- No deviation from the C.
- Real capture: `IMPORTS/chevy/22_chevy_trail_blazer_l2_u2_rs_p_t.sub` (433.92 MHz, OOK270): 21 frames with the
  constant serial `0xB809C5`, button `0xA` or `0xB`, identical to the host build of FoxFW's C frame for frame.
  Before this decoder KAT read none of it. The other GM captures in IMPORTS (Malibu, Buick, GMC) decode in neither
  FoxFW nor KAT with this decoder; no other file in IMPORTS or Flipper's unit-test captures changes.
- Unit tests: a synthetic frame, the gates (button nibble, zero payload, an out-of-tolerance pulse), key-file round
  trip, the real capture.
