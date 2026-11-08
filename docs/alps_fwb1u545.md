---
layout: default
---

# Alps FWB1U545 (rtl_433 port, receive only; FoxFW key files)

**Rust module:** `src/protocols/alps_fwb1u545.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/alps_fwb1u545.c` ("Alps FWB1U545 Car Remote", model `Alps-FWB1U545`),
slicer `pulse_slicer_manchester_zerobit` (`src/pulse_slicer.c`); key files: FoxFW `lib/subghz/protocols/alps_rke.c`
("Alps FWB1U545", its own port of the same decoder)

## Overview

Alps Electric FWB1U545 (FCC ID CWTWB1U545), an OEM car remote for Honda: a fixed 32-bit ID, a button code sent
twice and an unencrypted 16-bit sequence number. FoxFW is the only Flipper firmware that has it. KAT decodes it
exactly as rtl_433 does; transmitting means replaying the recorded pulses.

## Timing / modulation

| Parameter   | Value                          | Notes |
|-------------|--------------------------------|-------|
| Modulation  | `FSK_PULSE_MANCHESTER_ZEROBIT` | 2-FSK; the higher tone is the pulse (KAT's FM HIGH) |
| short_width | 500 µs                         | half bit; an edge more than 750 µs after the last data edge is a data edge |
| reset_limit | 1500 µs                        | a longer gap ends the message |
| tolerance   | 0                              | the slicer rejects no pulse |
| Deviation   | ±30–37 kHz (recordings)        | KAT's transmit preset F3 (±31.74 kHz) for Replay |

## Frame Layout (76 bits, one row)

| Bits  | Field |
|-------|-------|
| 0–7   | `0x55` preamble byte (bit 0 is the slicer's hard-coded leading 0) |
| 8–39  | 32-bit ID |
| 40–47 | button byte: high nibble = button code |
| 48–55 | copy of the button byte |
| 56–71 | 16-bit sequence |
| 72–75 | unknown |

## Decoder Steps

1. KAT's FM pairs go through rtl_433's Manchester zerobit slicer (rising edge = 0, falling edge = 1, a hard-coded
   0 bit at the start of every message).
2. A gap over 1500 µs ends the message. The 500/500 µs preamble is a message of its own (it ends at the ~2.5 ms
   gap before the frame) and fails the length check.
3. `decode_fn`: exactly one row of exactly 76 bits; byte 0 = `0x55` and byte 6 = byte 5; ID ≠ 0, ≠ FFFFFFFF
   (all `DECODE_FAIL_SANITY`). The event is reported on the gap that ends the frame.

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` | `id` |
| `button` | `button_code` (byte 5 >> 4); native names = `button_str`: E Lock, C Panic, 5 Panic Held, 1 Unlock, else "?" |
| `counter` | `sequence` |
| `data` / `extra` | the first 64 frame bits / the last 12 (byte 8 + the unknown nibble), `data_count_bit` 76 |
| `crc_valid` | false: rtl_433 reports no integrity check for this remote (no `mic` field; its checks are sanity checks) |

## Key files

FoxFW saves a decode with its generic serializer; KAT writes FoxFW's lines and one of its own after them, and loads
both (`Alps FWB1U545` is an alias of this protocol):

```text
Protocol: Alps FWB1U545
Bit: 76
Key: 13 57 9B DF E3 E3 24 68
Raw: 55 13 57 9B DF E3 E3 24 68 A0
```

`Key` is bytes 1–8 of the frame: the ID, the button byte, its copy and the sequence. FoxFW does not save the `0x55`
byte, and never decodes the 4 unknown bits after the sequence (its window search reads 72 bits), so KAT adds `Raw`:
the frame's 10 bytes (the 76 bits, then 4 zero bits). FoxFW reads only `Bit` and `Key` and ignores the line: on
FoxFW's C (host build with its SD info plugins), a file with `Raw` loads and shows exactly what the same file without
it shows.

Loading is FoxFW's `deserialize` (exactly `Bit: 76`) followed by the decoder's checks (the `0x55` byte, the button
copy, the ID not 0 / FFFFFFFF — FoxFW's decoder applies them before it saves a frame, and also skips IDs 55555555 /
AAAAAAAA, which rtl_433 and KAT read and load):

- **with `Raw`** (KAT's files): on `Raw`, which must also hold the key as bytes 1–8 and nothing after the 76 bits; the
  file loads as the received decode, unknown bits included. A `Raw` that fails is refused.
- **without `Raw`** (FoxFW's files): on the frame rebuilt with the `0x55` byte and **the unknown bits 0**: the same
  serial, button, sequence and `data`, with `extra`'s low nibble (not shown) 0.

FoxFW's decoder locks on 8 half-bit pulses and Manchester-decodes from there; it reads a frame only when a preamble
of 1 bits runs straight into it. Behind the remote's own preamble (0 bits, then a ~2.5 ms gap before the frame) it
loses the frame's first bits, so it reads none of the rtl_433 recordings (KAT's pulses of them, as RAW files); after
a 0-bit preamble without the gap it can report a wrong frame. Its window search keeps the last offset at which a
`0x55` byte, the button copy and the ID pass, which for 4 of 150 made-up frames was 2 or 4 bits into the frame
(possible only when the ID starts with bits `01`): FoxFW then saves the key of those shifted bits.

## Encoder

None. rtl_433 has no encoder, and FoxFW's SD encoder plugin (`applications/main/subghz/protocols_ext/enc_alps_rke.c`)
is a stub ("no TX — decode/save only"): its `deserialize` returns `SubGhzProtocolStatusErrorEncoderGetUpload` for
every key file, so a Flipper running FoxFW sends nothing. KAT does the same (Replay of the recorded pulses and the
RAW exports remain).

## Validation

- rtl_433_tests `alps/fwb1u545/01` (4 recordings, 433.92 MHz, 250 kS/s): `kat --decode` (FM path) gives rtl_433's
  events with the same fields and counts (lock, panic, panic held ×2, unlock).
- KAT's FM pulses match rtl_433's FSK pulses within a few µs (rtl_433 ends the last gap at 3056 µs and adds a 12 µs
  F1 blip; KAT ends the burst with a long LOW — both end the message).
- rtl_433's own pulse packages of the recordings (`rtl_433 -w ook:`) through KAT's decoder reproduce the expected
  `.json` files exactly; KAT's pulses through the C reference (`libr_433.a`) and through KAT give the same events.
- Fuzz against the C reference (slicer + decoder linked from rtl_433's library): 2 × 20 000 synthetic trains
  (valid frames with jitter, flipped / cut bits, retimed, dropped, split pulses, timing edges) — identical output,
  24 667 events.
- No decode on any IMPORTS capture, the Flipper unit-test captures or another device's rtl_433_tests recording.
- Key files against FoxFW's C (host build with its encoder plugins, `subghz_oracle_foxfwenc`): 150 RAW files of
  made-up frames (every named button and others), each holding the frame in both shapes (behind twelve 1 bits for
  FoxFW, then as the remote sends it for rtl_433's slicer). FoxFW's serialized lines and KAT's key file are identical
  for 146; in the other 4 FoxFW's decoder read the frame 2 or 4 bits late (see above: its key there is that of the
  shifted bits, checked bit for bit).
- Transmit against FoxFW's C: 120 made-up key files (110 valid, every named button and others, 10 that KAT refuses);
  FoxFW's encoder refuses every one (`deserialize failed (status -12)`), KAT sends nothing for any. KAT loads the 110
  valid ones and refuses the 10 others.
