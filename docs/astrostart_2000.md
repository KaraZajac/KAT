---
layout: default
---

# Astrostart 2000 (rtl_433 port, receive only; FoxFW key files)

**Rust module:** `src/protocols/astrostart_2000.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/astrostart_2000.c` ("Astrostart 2000 Car Remote (-f 372.4M)", model
`Astrostart-2000`), slicer `pulse_slicer_ppm`; key files: FoxFW `lib/subghz/protocols/astrostart.c` ("AstroStart
2000", its own port of the same decoder)

## Overview

Astroflex Astrostart 2000 / 3000 remote-starter remotes (FCC ID J5F-TX2000), 372.4 MHz: a button code with its
inverse, a 32-bit ID and a 4-bit ID checksum; three messages per press. The remote has a primary and a secondary ID
(the recordings: 3410DE88 and 0410DE88). FoxFW is the only Flipper firmware that has it.

## Timing / modulation

| Parameter   | Value          | Notes |
|-------------|----------------|-------|
| Modulation  | `OOK_PULSE_PPM` | the gap after each ~100 µs pulse carries the bit |
| short_width | 326 µs ± 80    | gap 247–405 µs → 0 |
| long_width  | 526 µs ± 80    | gap 447–605 µs → 1 |
| reset_limit / gap_limit | 541 µs | a gap ≥ 541 µs ends the message (a 541–605 µs gap is first a 1 bit, as in the C); other gaps < 541 µs start a new row |

## Frame Layout (exactly 52 bits, one row)

| Bits  | Field |
|-------|-------|
| 0–7   | button code |
| 8–15  | inverse of the button code |
| 16–47 | 32-bit ID |
| 48–51 | checksum: both nibbles of the four ID bytes added mod 16 |

## Decoder Steps

1. rtl_433's PPM slicer over KAT's AM pairs; the message ends on its long gap (KAT reports there).
2. `decode_fn`: exactly one row of exactly 52 bits; byte 0 = !byte 1 and the checksum (`DECODE_FAIL_MIC`).

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` | `id` |
| `button` | `button_code` (8 bits); native names = `button_str` from rtl_433's map (Lock 2B 03 27 0F 35 37, Panic 1F 35 0D 25 15 2D, Start 13 37 2D 33 3D 3B, Stop 2F 0D 33 03 1D 17, Trunk 23 25 3D 1D 27 07, Unlock 0B 15 3B 17 07 0F, Multiple 3F; a code in several lists names them all, else "?") |
| `data` | the 52 row bits, `data_count_bit` 52 |
| `crc_valid` | true (inverse byte + checksum) |

## Key files

FoxFW saves a decode with its generic serializer; KAT writes and loads that file (`AstroStart 2000` is an alias of
this protocol):

```text
Protocol: AstroStart 2000
Bit: 52
Key: 00 02 BD 41 23 4A BC D8
```

`Key` is the 52 frame bits (KAT's `data`). Loading is FoxFW's `deserialize` (exactly `Bit: 52`) followed by the
decoder's checks (inverse byte, ID checksum), which FoxFW's decoder applies before it saves a frame: a key that fails
them, or has bits set above its 52, is refused. KAT's files round-trip exactly.

FoxFW's decoder only takes pulses of 120–900 µs; this remote's are about 100 µs (KAT measures 98–101 µs, rtl_433
104–108 µs), so FoxFW reads none of the rtl_433 recordings (KAT's pulses of them, as RAW files). Its key files are
the same either way.

## Encoder

None. rtl_433 has no encoder, and FoxFW's SD encoder plugin (`applications/main/subghz/protocols_ext/enc_astrostart.c`)
is a stub ("no TX — rolling button set, replay only"): its `deserialize` returns
`SubGhzProtocolStatusErrorEncoderGetUpload` for every key file, so a Flipper running FoxFW sends nothing. KAT does
the same: no TX buttons; Replay of recorded pulses remains.

## Validation

- rtl_433_tests `astroflex/astrostart_2000/01` (5 recordings, 372.4 MHz, 250 kS/s): `kat --decode` gives rtl_433's
  15 events with the same fields and counts (panic: primary ID twice, secondary once).
- KAT's pulses are 4–8 µs shorter and the gaps 4–8 µs longer than rtl_433's (98–101 vs 104–108 µs pulses), well
  inside the ±80 µs gap windows.
- rtl_433's own pulse packages through KAT's decoder: the expected `.json` files exactly; KAT's pulses through the C
  reference and through KAT: the same events.
- Fuzz against the C reference (gap windows probed at their exclusive bounds): 2 × 20 000 trains, identical output,
  8 036 events.
- No decode on any IMPORTS capture, the Flipper unit-test captures or another device's rtl_433_tests recording.
- Key files against FoxFW's C (host build with its encoder plugins, `subghz_oracle_foxfwenc`): on 150 RAW files of
  made-up frames (every mapped button code, "Multiple", unmapped codes; 200 µs pulses) FoxFW's serialized lines and
  KAT's key file are identical.
- Transmit against FoxFW's C: 120 made-up key files (110 valid, covering the button codes, 10 that KAT refuses);
  FoxFW's encoder refuses every one (`deserialize failed (status -12)`), KAT sends nothing for any. KAT loads the 110
  valid ones and refuses the 10 others.
