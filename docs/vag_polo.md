---
layout: default
---

# VAG Polo Protocol

**Rust module:** `src/protocols/vag_polo.rs`
**Reference:** `FoxFW lib/subghz/protocols/vag_polo.c` (a port of the KeeLoq-RX `arke_vag.py` class `VagPolo`);
FoxFW is the only copy in REFERENCES.

## Overview

The remote of the 2017 VW Polo 5. OOK Manchester at 433.92 MHz (the owner's captures were recorded at
434.42 MHz), 80 payload bits behind a cleartext preamble and a 16-bit sync. The button's high nibble is in the
clear, the serial is in the clear, and the 40-bit rolling part is encrypted: **decode and display only**, as the
reference (its encoder is NULL; a rolling-code receiver rejects a replay once it has seen the code).

It is not the VAG protocol of [vag.md](vag.md): none of that framing (the 600 µs LOW after the preamble, the
`0xAF3F` / `0xAF1C` prefixes) is present.

## Timing

| Parameter | Value  | Notes                                            |
|-----------|--------|--------------------------------------------------|
| Short     | 300 µs | every pulse must be within 150 µs of 300 or 590 |
| Long      | 590 µs | (`TE_DELTA` 130 + 20, the C's `EVENT_TOL`)       |
| Preamble  | ≥ 32 cleartext `1` bits | counted by the decoder            |
| Sync      | `0x3F91` (16 bits) | read through a 16-bit shift register   |
| Payload   | 80 bits | MSB first                                       |

## Frame (80 bits / 10 bytes)

| Bytes   | Field        | Notes                                                              |
|---------|--------------|--------------------------------------------------------------------|
| 0       | Button       | high nibble `0x7` = Unlock, `0xB` = Lock; any other nibble drops the frame. The evidence captures send `0x7D`/`0x7F` and `0xBD`/`0xBF` |
| 1..4    | Serial       | 32-bit, big endian                                                 |
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
`button` = 1 (Unlock) or 2 (Lock), `extra` = the C's `Hdr` (bytes 0..1, its high byte is `ButtonRaw`), `counter`
None, `crc_valid` false (shown as "-"), `encoder_capable` false.

## Key file

`Protocol: VAG Polo`, `Bit: 80`, `Key` = `data`, then `Serial`, `Btn`, `ButtonRaw`, `Hdr`
(`subghz_protocol_decoder_vag_polo_serialize` / `_deserialize`; loading needs `Bit: 80`).

## Validation

- No deviation from the C.
- Real captures: the eight `IMPORTS/VAG/VW Polo 5 2017 lock/unlock*.sub` files (433.92 MHz band, OOK270): 48 frames,
  three per press and fifteen in each x5 file, identical to the host build of FoxFW's C frame for frame (`Key`,
  `Serial`, `Btn`). Before this decoder KAT read none of them (`unlock2.sub` gave one junk PSA frame). No other
  file in IMPORTS or Flipper's unit-test captures changes.
- Unit tests: a synthetic frame (Lock and Unlock), the gates (fewer than 32 ones, a button nibble outside 7/B, an
  out-of-tolerance pulse), key-file round trip, the real captures.
