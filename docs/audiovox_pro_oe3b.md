---
layout: default
---

# Audiovox PRO-OE3B (rtl_433 port, receive only; FoxFW key files)

**Rust module:** `src/protocols/audiovox_pro_oe3b.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/audiovox_pro_oe3b.c` ("Audiovox PRO-OE3B Car Remote (-f 303.4M)", model
`Audiovox-PROOE3B`), slicer `pulse_slicer_pwm`; key files: FoxFW `lib/subghz/protocols/audiovox.c` ("Audiovox PRO",
its own port of the same decoder)

## Overview

Audiovox PRO-OE3B / PRO-OE4B aftermarket car remotes (FCC ID BGAOE3B, AVX01BT3CL3, ~303 MHz): a fixed 16-bit ID
and button flags, repeated while a button is held; several buttons can be pressed together. FoxFW is the only
Flipper firmware that has it.

## Timing / modulation

| Parameter   | Value          | Notes |
|-------------|----------------|-------|
| Modulation  | `OOK_PULSE_PWM` | pulse width carries the bit |
| short_width | 445 µs         | → 1 (pulses up to 670 µs) |
| long_width  | 895 µs         | → 0 (671–1131 µs) |
| sync_width  | 1368 µs        | longer pulses are a sync (new row) |
| reset_limit / gap_limit | 1790 µs | a longer gap ends the message |
| tolerance   | 0              | the slicer's split points apply (`(short + long) / 2 + 1`, `(long + sync) / 2 + 1`) |
| priority    | 10             | rtl_433 runs it only when no priority-0 decoder decoded the package |

The remote sends ~460/870 µs (1) and ~920/425 µs (0) pulse/gap pairs, 25 per frame, every 48 ms.

## Frame Layout (25 bits, one row; the decoder inverts every bit)

| Bits (inverted) | Field |
|-----------------|-------|
| 0–15  | 16-bit ID |
| 16–23 | `1x1x1x1x`: the `0xAA` bits are always 1; flags `0x40` Trunk, `0x10` Option, `0x04` Unlock, `0x01` Lock |
| 24    | free |

## Decoder Steps

1. rtl_433's PWM slicer over KAT's AM pairs; a gap over 1790 µs ends the message.
2. `decode_fn`: exactly one row of exactly 25 bits; on the wire bits byte 2 has no `0xAA` bit set and is not `0x55`;
   after inversion the ID is not 0000 / FFFF and at least one button flag is set (all `DECODE_FAIL_SANITY`).

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` | `id` |
| `button` | the pressed flags packed in rtl_433's order (bit 0 Lock, 1 Unlock, 2 Option, 3 Trunk); rtl_433 prints only `button_str` ("Lock; Unlock" …), KAT's native names are the same strings |
| `data` | the 25 wire bits (before the decoder's inversion), `data_count_bit` 25 |
| `crc_valid` | false (no integrity check) |

## Key files

FoxFW saves a decode with its generic serializer; KAT writes and loads that file (`Audiovox PRO` is an alias of this
protocol):

```text
Protocol: Audiovox PRO
Bit: 25
Key: 00 00 00 00 01 D9 50 23
```

`Key` is the 25 wire bits (KAT's `data`); FoxFW keeps the button as the inverted byte's flag bits (`& 0x55`) but does
not save it. Loading is FoxFW's `deserialize` (exactly `Bit: 25`) followed by the decoder's sanity checks, which
FoxFW's decoder applies before it saves a frame: a key that fails them, or has bits set above its 25, is refused.
KAT's files round-trip exactly.

FoxFW's decoder starts a frame only on a sync pulse of 1131–1600 µs; the rtl_433 recordings have none (frames start
straight after ~15 ms gaps), so FoxFW reads none of them (KAT's pulses of them, as RAW files). Its key files are the
same either way.

## Encoder

None. rtl_433 has no encoder, and FoxFW's SD encoder plugin (`applications/main/subghz/protocols_ext/enc_audiovox.c`)
is a stub ("fixed-code remote; TX not implemented — load/save only"): its `deserialize` returns
`SubGhzProtocolStatusErrorEncoderGetUpload` for every key file, so a Flipper running FoxFW sends nothing. KAT does
the same: no TX buttons; Replay of recorded pulses remains.

## Validation

- rtl_433's own pulse packages of the three rtl_433_tests recordings (`audiovox/PRO-OE3B/01`) through KAT's decoder
  give the expected `.json` exactly (26 events); `decodes_rtl_433_pulses_of_the_lock_recording` keeps one in the
  unit tests.
- **Through KAT's IQ pipeline** (`kat --decode`): lock 3 / 3, option 10 / 10, unlock 13 / 13 frames, the `.json`'s
  events (ID 2EA9: Lock, Option, Unlock). The remote's carrier drifts from −63 to −101 kHz within one frame and is
  pulled further each time it keys on; KAT's receiver follows the carrier within a frame when it moves
  ([radio.md](radio.md)). Before it did, the first pulse of each frame and the ends of the long (920 µs) pulses were
  lost: lock gave 1 of 3 frames plus a corrupt one that passed the weak checks (ID 0029), option 1 of 10, unlock 0
  of 13.
- Fuzz against the C reference: 2 × 20 000 trains, identical output, 34 155 events.
- No decode on any IMPORTS capture, the Flipper unit-test captures or another device's rtl_433_tests recording.
- Key files against FoxFW's C (host build with its encoder plugins, `subghz_oracle_foxfwenc`): on 150 RAW files of
  made-up frames (all 15 button combinations with either free bit, behind a 1368 µs sync pulse, which rtl_433's
  slicer counts as a sync before the row) FoxFW's serialized lines and KAT's key file are identical.
- Transmit against FoxFW's C: 120 made-up key files (110 valid, every button combination, 10 that KAT refuses);
  FoxFW's encoder refuses every one (`deserialize failed (status -12)`), KAT sends nothing for any. KAT loads the 110
  valid ones and refuses the 10 others.
