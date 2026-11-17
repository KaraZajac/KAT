---
layout: default
---

# VAG Polo Protocol

**Rust module:** `src/protocols/vag_polo.rs`
**Reference:** `FoxFW lib/subghz/protocols/vag_polo.c` (a port of the KeeLoq-RX `arke_vag.py` class `VagPolo`);
FoxFW is the only copy in REFERENCES.

## Overview

The remote of the 2017 VW Polo 5 and, behind another sync word, of the 2020 VW Atlas (North America, 315 MHz).
OOK Manchester at 433.92 MHz (the owner's Polo captures were recorded at 434.42 MHz), 80 payload bits behind a
cleartext preamble and a 16-bit sync. The button's high nibble is in the clear; bytes 1 to 9 change with every
frame (the four bytes the C names `Serial` among them: 48 values in the 48 Polo frames, 23 in the 23 Atlas
frames): **decode and display only**, as the reference (its encoder is NULL; a rolling-code receiver rejects a replay once it has seen the code).

It is not the VAG protocol of [vag.md](vag.md): none of that framing (the 600 µs LOW after the preamble, the
`0xAF3F` / `0xAF1C` prefixes) is present.

## Timing

| Parameter | Value  | Notes                                            |
|-----------|--------|--------------------------------------------------|
| Short     | 300 µs | every pulse must be within 150 µs of 300 or 590 |
| Long      | 590 µs | (`TE_DELTA` 130 + 20, the C's `EVENT_TOL`)       |
| Preamble  | ≥ 32 cleartext `1` bits | counted by the decoder            |
| Sync      | `0x3F91` (16 bits) | read through a 16-bit shift register; `0x3FCD` on the Atlas (KAT's addition) |
| Payload   | 80 bits | MSB first                                       |

## Frame (80 bits / 10 bytes)

| Bytes   | Field        | Notes                                                              |
|---------|--------------|--------------------------------------------------------------------|
| 0       | Button       | high nibble `0x7` = Unlock, `0xB` = Lock; any other nibble drops the frame. The evidence captures send `0x7D`/`0x7F` and `0xBD`/`0xBF`. Behind the Atlas sync also `0xD` = Trunk and `0xF` = Remote start (`0xDD`/`0xDF`, `0xF5`/`0xF7`) |
| 1..4    | "Serial"     | 32-bit, big endian: the C's name; another value in every frame of every capture |
| 5..9    | Rolling part | 40 bits, encrypted                                                 |

## Decoder state machine

1. **Reset step:** a HIGH pulse within tolerance of the short time starts the preamble (the pulse itself is not an
   event). Manchester state `Mid1`, sync register and ones count cleared.
2. **Preamble:** every pulse is a Manchester event for Flipper's toolbox decoder with the C's polarity
   (`level ? ShortLow : ShortHigh`); a pulse out of tolerance goes back to the reset step. Each decoded bit shifts
   into the sync register; a `1` adds to the ones count, a `0` with fewer than 32 ones counted clears the count and
   the register. When the register equals `0x3F91` and 32 or more ones were counted, the payload starts.
3. **Data:** bits go MSB first into 10 bytes. After 80 bits the frame is reported if the first byte's high nibble
   is `0x7` or `0xB`; the decoder re-arms either way.

There is no checksum: the gate is structural, so the registry places VAG Polo after every decoder with a check
(after the gate remotes, before the rtl_433 ports).

## Field mapping

`data` = payload bytes 2..9 (the C's `generic.data`, 64 bits), `data_count_bit` = 80, `serial` = bytes 1..4,
`button` = 1 (Unlock), 2 (Lock), 3 (Trunk) or 4 (Remote start; 3 and 4 on the Atlas only), `extra` = the C's `Hdr` (bytes 0..1, its high byte is `ButtonRaw`), `counter`
None, `crc_valid` false (shown as "-"), `encoder_capable` false.

## Key file

`Protocol: VAG Polo`, `Bit: 80`, `Key` = `data`, then `Serial`, `Btn`, `ButtonRaw`, `Hdr`
(`subghz_protocol_decoder_vag_polo_serialize` / `_deserialize`; loading needs `Bit: 80`).

## Validation

- One addition to the C, for the 2020 VW Atlas: the same frame behind the sync `0x3FCD`, and behind that sync
  the button nibbles `0xD` (Trunk) and `0xF` (Remote start). Behind `0x3F91` the gate is the C's.
- `IMPORTS/VAG/20_vw_atlas_l2_u2_t2_rs2.sub` (315 MHz, AM650; half bit 293.6 µs, a 214-bit preamble): 23 frames,
  5 Lock, 6 Unlock, 6 Trunk, 6 Remote start. Before, KAT and FoxFW read none of it.
- Real captures: the eight `IMPORTS/VAG/VW Polo 5 2017 lock/unlock*.sub` files (433.92 MHz band, OOK270): 48 frames,
  three per press and fifteen in each x5 file, identical to the host build of FoxFW's C frame for frame (`Key`,
  `Serial`, `Btn`). Before this decoder KAT read none of them (`unlock2.sub` gave one junk PSA frame). No other
  file in IMPORTS or Flipper's unit-test captures changes.
- Unit tests: a synthetic frame (Lock and Unlock), the gates (fewer than 32 ones, a button nibble outside 7/B, an
  out-of-tolerance pulse), key-file round trip, the real captures; the Atlas sync and its buttons (and that Trunk
  and Remote start are not taken behind the Polo's sync), the Atlas capture.
