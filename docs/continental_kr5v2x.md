---
layout: default
---

# Continental-KR5V2X Protocol

**Rust module:** `src/protocols/continental_kr5v2x.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** `REFERENCES/rtl_433/src/devices/continental_car_remote.c`, `continental_car_remote_decode`
(r_device "Continental KR5V2X Car Remote (-f 313.8M -s 1024k)", decoder 316, enabled by default); key files: FoxFW
`lib/subghz/protocols/continental_rke.c` ("Continental RKE", its own port of the same decoder)

## Overview

Continental 72147-SNA-A01 (FCC ID KR5V2X), an OEM remote for Honda, 313.8 MHz FSK. rtl_433 describes it as a
rolling code with an unencrypted sequence number. A press sends a long wake-up (`0x87` repeated) with the frame,
then the frame again ~1.6 ms later; Trunk and Panic long presses repeat every ~95 ms. Receive only, as rtl_433 and
FoxFW; KAT writes and loads FoxFW's key files.

rtl_433 also has an alternate decoding of the same FCC ID, [Honda-KR5V2X1X](honda_kr5v2x1x.md) (60 µs, other
layout and CRC); it decodes none of these recordings.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Modulation | FSK, `FSK_PULSE_MANCHESTER_ZEROBIT` | upper tone = HIGH |
| Short (half-bit) | 100 µs | an edge > 150 µs after the last data edge is a data edge |
| Long | 200 µs | (not used by this slicer) |
| Reset limit | 1500 µs | a lower-tone stretch longer than this ends the message |
| Tolerance | none | no row splits |

The zero-bit slicer (rtl_433 `pulse_slicer_manchester_zerobit`): every message starts with a hard-coded `0`; a
falling data edge adds `1`, a rising one `0`. See `src/protocols/rtl433/mod.rs` for how KAT's level/duration stream maps
to rtl_433's packages.

## Frame Layout

The decoder searches row 0 for the last 20 bits of the preamble, `0xF0F06`, and reads the 112 bits after it:

| bytes | field |
|-------|-------|
| 0..3 | remote ID (32 bits) |
| 4 | unknown (0x00 in the first frame of a press, 0x10 in the repeat on the recordings) |
| 5 | button (high nibble), unknown (low nibble) |
| 6..8 | sequence (24 bits) |
| 9..12 | encrypted code (32 bits) |
| 13 | checksum: the 14 bytes XOR to 0 |

Buttons: 1 Lock, 3 Unlock, 9 Trunk Long Press, 0xA Trunk/Panic Short Press, 0xB Panic Long Press.

## Decoder Steps

1. Slice the stream into messages (zero-bit Manchester, end at a gap > 1500 µs).
2. Row 0 must have ≥ 132 bits; find `0xF0F06`; ≥ 112 bits must follow it.
3. Sanity (rtl_433 `DECODE_FAIL_SANITY`): ID, button and sequence non-zero; ID and encrypted ≠ `0x0FFFFFFF`
   (rtl_433 compares the 32-bit values with seven F's; kept as is); sequence ≠ `0xFFFFFF`.
4. XOR checksum (`DECODE_FAIL_MIC`). Frames failing any check are dropped.

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` | `id` (`%08X`) |
| `button` | `button_code` |
| `counter` | low 16 bits of `sequence` (24 bits) |
| `crc_valid` = true | `mic: CHECKSUM` |
| `data` = bytes 0..7, `extra` = bytes 8..13, 112 bits | `sequence` = `(data & 0xFFFF) << 8 \| extra >> 40`, `encrypted` = `(extra >> 8) & 0xFFFFFFFF` |

## Key files

FoxFW saves a decode with its generic serializer; KAT writes FoxFW's lines and one of its own after them, and loads
both (`Continental RKE` is an alias of this protocol):

```text
Protocol: Continental RKE
Bit: 64
Key: 12 34 56 78 10 30 AB CD
Raw: 12 34 56 78 10 30 AB CD EF 9A BC DE F0 A9
```

`Key` is bytes 0..7 (ID, byte 4, the button byte, the sequence's top 16 bits) — KAT's `data`. That is all FoxFW keeps:
the sequence's low byte, the encrypted code and the checksum (bytes 8..13) are not in its file, and FoxFW shows `Cnt`
and `Enc` 0 for a loaded file. So KAT adds `Raw`, the whole 14-byte frame, to the files it writes for a received
frame. FoxFW reads only `Bit` and `Key` and ignores the line: on FoxFW's C (host build with its SD info plugins), a
file with `Raw` loads and shows exactly what the same file without it shows, and its app only splits a file at a
`RAW_Data` line.

Loading is FoxFW's `deserialize` (exactly `Bit: 64`), then:

- **with `Raw`** (KAT's files): the frame's checks (the sanity checks and the XOR checksum) run on `Raw`, which must
  also start with the key's 8 bytes; the file loads as the received decode, sequence, code and checksum included. A
  `Raw` that fails is refused.
- **without `Raw`** (FoxFW's files): what the file holds — `data` = the key, 64 bits, `serial`, `button`, no counter
  (its low byte is gone) and no checksum (shown as "-", not FAIL). Such a decode writes FoxFW's file back, without
  `Raw`.

FoxFW's decoder reads the rtl_433 recordings (KAT's pulses of them, as RAW files): its 9 distinct keys are KAT's.

## Encoder

None. rtl_433 has no encoder, and FoxFW's SD encoder plugin (`applications/main/subghz/protocols_ext/enc_continental_rke.c`)
is a stub ("rolling code / encrypted — no TX"): its `deserialize` returns `SubGhzProtocolStatusErrorEncoderGetUpload`
for every key file, so a Flipper running FoxFW sends nothing. KAT does the same: the app offers Replay of the recorded
pulses and export.

## Validation

- **Recordings** (`rtl_433_tests/tests/continental/72147-SNA-A01`, 7 files at 1024 kS/s): all decode through KAT's
  IQ pipeline with rtl_433's fields (ID, encrypted, sequence, button) — 22 frames in KAT, 21 in rtl_433: in
  `panic_long_press` rtl_433's demodulator keeps only 127 bits of one package's first message (it needs 132),
  KAT's keeps 150. Before this port no KAT decoder read them at their real rate (KAT 1.7's "Kia V5" decodes came
  from `--rate 250k` stretching the 1024 kS/s files 4×).
- **Oracle parity** (rtl_433 patched to run its FSK decoders on pulse files: `src/rtl_433.c` treats a file with a `;freq2` header
  line as FSK): 8000 fuzzed packages (frames from
  the C with random fields, bit flips, jitter, glitches, truncations, extra gaps, random trains) give identical
  decodes (643 frames); the slicer gives identical rows for every message (10 658 messages, with and without a
  tolerance).
- **Synthetic IQ**: frames built from the C, at 1024 and 250 kS/s, decode identically in rtl_433 and KAT.
- No decode on any other rtl_433_tests recording, IMPORTS capture or Flipper unit-test capture (decoder run alone,
  both polarities).
- Key files against FoxFW's C (host build with its encoder plugins, `subghz_oracle_foxfwenc`): on 150 RAW files of
  made-up frames (every named button and others, random byte 4, sequences and codes) FoxFW's serialized lines and
  KAT's key file are identical; on KAT's pulses of the 7 recordings FoxFW's 9 distinct keys are KAT's 9.
- Transmit against FoxFW's C: 120 made-up key files (110 with `Bit: 64`, every named button and others, 10 with
  another `Bit`); FoxFW's encoder refuses every one (`deserialize failed (status -12)`), KAT sends nothing for any.
  KAT loads the 110 and refuses the 10.
