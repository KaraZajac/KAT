---
layout: default
---

# BMW CAS3 Protocol

**Rust module:** `src/protocols/bmw_cas3.rs`
**Reference:** FoxFW `lib/subghz/protocols/bmw_cas3.c` (`BMW_CAS3_PROTOCOL_NAME = "BMW CAS3"`; its header credits
FOBworks for Flipper, GPL-3.0, `flipper_oem_wire.c`, for the gate and timing); transmit: FoxFW's SD-plugin replay encoder
`applications/main/subghz/protocols_ext/enc_bmw_cas3.c`.

## Overview

The PPM key fob of BMW CAS3 / CAS4 cars. The payload is AES-encrypted, so the frame is identified by structure and kept
verbatim: constant marks, a short or long space per bit, 64 to 128 bits, after two intervals of at least 10 ms. It is not
KAT's "BMW" (`bmw.rs`, Flipper-ARF's 62 µs Manchester frame behind the 0x599E sync).

## Timing / modulation

OOK/AM (flags 315 | 433 | 868 | AM), pulse position:

| Pulse | Window |
|-------|--------|
| mark (HIGH) | 100..400 µs |
| space 0 (LOW) | 300..700 µs |
| space 1 (LOW) | 1050..1950 µs |
| sync interval | 10 ms or more (either level) |

## Decoder Steps

`subghz_protocol_decoder_bmw_cas3_feed`:

1. Sync counts consecutive pulses of 10 ms or more (a shorter pulse zeroes the count) until there are two.
2. Mark expects a HIGH mark; Space turns the following LOW into a 0 or a 1, up to 128 bits (the bit is stored MSB first).
3. A bad mark or space ends the frame: it is reported when it holds 64 or more bits (any count from 64 to 128 — a frame
   cut at 70 bits is reported as a 70-bit frame), and the pulse that broke it restarts the sync count at 1 if it is
   itself 10 ms or longer, else 0. A frame that reaches 128 bits is reported at once.
4. The C reports inside `bmw_cas3_finish` and sets the count after that callback, overriding the receiver reset the
   callback does: the 75 ms tail of one frame is the first sync interval of the next. KAT skips the registry's reset
   right after its own report to keep that; otherwise every second back-to-back frame (of the encoder's 3 copies) was
   lost, where the FoxFW decoder reads all of them.

The only gate is this structure: no check and no field is validated. It is the loosest decoder of the FoxFW group and is
registered last.

## Field mapping

`data` = the first 64 bits (the C's `generic.data`), `extra` = the following bits right-aligned (the C's `data_2`),
`data_count_bit` = 64..=128, `serial` = `data >> 32` (the C's display value), `crc_valid` false, `button` and `counter`
None, `encoder_capable` true (every decode transmits).

## Key file

`Protocol: BMW CAS3`, `Bit: 64..128`, `Key` = the first 64 bits, `Data` = 8 bytes of `data_2` (the C requires it for more
than 64 bits).

## Transmit

FoxFW's `subghz_protocol_encoder_bmw_cas3_deserialize` + `bmw_cas3_build_upload`: a replay of the captured bits (the
payload is AES-encrypted; "the encoder replays exactly the captured bits").

- **Reads** what the decoder's deserialize reads: `Bit` 64..128, `Key` and, beyond 64 bits, `Data` (refused without
  it); `Repeat` too (default 3, used when above 0), which KAT does not carry: it sends the default.
- **Sends** 3 copies back to back of: HIGH 25 ms, LOW 20 ms, per bit a 250 µs mark and a 1500 µs (1) or 500 µs (0)
  space (the first 64 bits from `Key`, the rest from `Data`'s low bits; its higher bits are not sent), HIGH 75 ms,
  LOW 30 ms.
- **Changes nothing** (no counter or button in clear): TX Lock / Unlock / Trunk / Panic and "Send next code" all send
  the saved frame, and the capture keeps it.

## Validation

- Reference oracle `subghz_oracle_foxfw`: synthetic frames of 64, 100 and 128 bits and frames broken at bit 66 and bit 70
  give identical key, `data_2` and bit count; a 63-bit frame, a bad mark or space at bit 50 and a frame with one sync
  interval are rejected by both.
- No IMPORTS capture decodes as BMW CAS3 in either decoder; `kat_diff` over IMPORTS and the Flipper unit-test captures
  shows no file changed by it.
- Encoder oracle (`subghz_oracle_foxfwenc encode`, FoxFW's plugin encoders built for the host): 114 made-up key files
  (every bit count from 64 to 128, `Data` with bits above the frame's, 64-bit keys with and without `Data`, the
  all-zero and all-one frames, `Key` / `Data` lines of 7 and 9 values, more than 64 bits without `Data`, bit counts
  32 / 63 / 129 / 200): KAT's transmission equals the C's pulse for pulse on all 105 the C sends (the whole 3 copies)
  and KAT refuses the 9 the C rejects.
- Round trip (KAT's transmission sent as the app sends it: twice, 20 ms apart, levels merged): KAT's registry reads
  all 6 copies as BMW CAS3 with the frame of the key file (`Data` cut to the frame's bits), identical to what the
  oracle's decoder reads from the same stream and to twice its air round trip of the C's transmission.
- Unit tests: the oracle-checked frames and rejections, key-file round trip, one oracle transmission (all 3 copies),
  every button sending the same frame, the refused keys, and every copy decoding back in the registry.
