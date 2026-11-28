---
layout: default
---

# BelKorn Protocol

**Rust module:** `src/protocols/belkorn.rs`
**Reference:** `REFERENCES/RogueMaster/lib/subghz/protocols/belkorn.c` (RogueMaster added it on 2026-11-13, from
Unleashed; read at its `origin/420`) and FoxFW's `lib/subghz/protocols/belkorn.c` with the SD encoder plugin
`applications/main/subghz/protocols_ext/enc_belkorn.c` (FoxFW v2.1.1, 2026-11-22). The two are the same function
for function: air format, frame, check, counter law and upload. FoxFW moves the encoder into the plugin, prints the
fields differently and has no "Add Manually" entry. Neither reference ships a capture of a remote.

## Overview

A 118-bit rolling code with **no encryption**, from gate and barrier remotes sold as Korn, BelGate, Furniteh and
other rebrands. The frame carries no manufacturer field, so every one of them reads as BelKorn. Each frame holds a
36-bit serial, a one-hot button and a 16-bit press counter that goes up by one per click. The rest of the frame is
a function of the counter and of a per-remote origin, which is solved back from any one frame. So one capture is
enough to make the remote's next press.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Bit 0 | a pulse of 481 µs (te_short) | ± 180 µs (302..660) |
| Bit 1 | a pulse of 962 µs (te_long) | ± 180 µs (783..1141) |
| Levels | one bit per pulse, marks and spaces alike | bit i is HIGH when i is even, LOW when odd |
| Start | a LOW of at least 2500 µs | |
| Bits | 118 | MSB first |
| End | a trailing mark (either width), then a gap | the C sends HIGH 481, LOW 7696 (16 te_short) |

AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`, preset AM650).

## Frame Layout

| Bits | Field |
|------|-------|
| 0..11 | sync `0x801` |
| 12..15 | serial, high nibble |
| 16..47 | serial (32 bits) |
| 48..55, 56..63, 64..71 | L24, low byte first |
| 72..79 | b3 |
| 80..87 | b4 |
| 88..95 | b5 |
| 96..103 | b6 |
| 104..111 | c, the counter's low byte |
| 112..117 | button, one bit set: `0x01`, `0x02`, `0x04`, `0x20` on the remotes |

The 16-bit press counter `n` has `c` as its low byte. Its high byte (the "lap") is in b6 when `c` is even (format A)
and in b5 when `c` is odd (format B).

## Counter law

As the reference builds a frame (`subghz_protocol_belkorn_make_frame`), with `k = n >> 1` and `lap = n >> 8`. Its
authors measured it on recordings:

- **Format A** (`c` even): L24 = origin L24 + k·0x810 + lap·0x1C282C; b3 = origin b3 + k·0x10; b4 = c;
  b5 = b6 = lap.
- **Format B** (`c` odd): L24 = origin L24 + 0x0748CC + k·0x1028 − wraps·0x80800 + jitter(lap, c); (b3, b4), read
  as a 16-bit little-endian value, = (0x5A << 8 | (origin b3 − 0x30)) + wraps·0x108; b5 = lap; b6 = c + 0xBA.
  `wraps` = lap + (1 if c > 0x45), the number of times b6 has wrapped. The jitter is a periodic offset. Each set
  bit of `lap` adds a pair of terms, each gated by one bit of `(c >> 1) − 3`. The rows for lap bits 0..2 are
  measured. Bits 3..7 follow the same law but need a remote past 2048 presses.

L24 is 24 bits, b3 and b4 wrap as bytes, and the jitter's `(c >> 1) − 3` is a 16-bit value, so it wraps below
c = 6. The origin is the remote's serial high nibble and format A's L24 and b3 at counter 0. It is solved back from
any frame (`subghz_protocol_belkorn_check_remote_controller`).

## Decoder Steps

1. Reset: wait for a LOW of 2500 µs or more.
2. Each pulse is one bit: 481 ± 180 µs is a 0, 962 ± 180 µs is a 1. Bit i must be HIGH for even i and LOW for odd
   i. A pulse out of turn or in neither window drops the frame. That pulse never starts a new one, so the decoder
   waits for the next long LOW.
3. At the 118th bit the frame is reported if its top 12 bits are `0x801` and its button has exactly one bit set.
   Then the decoder goes back to Reset. The trailing mark that follows is ignored, and the gap after it starts the
   next frame.

There is no checksum (`NO_CHECKSUM`): `crc_valid` is false. The counter law makes some bytes redundant: in format
A b4 = c and b5 = b6, and in format B b6 = c + 0xBA. KAT does not check these, because the reference does not.

## Field mapping

`data` = the C's `generic.data` (frame bits 0..63). `extra` = `generic.data_2` (bits 64..117). `data_count_bit` =
118. `serial` = the C's `generic.serial`: the low 32 bits of the 36-bit serial. Its high nibble is frame bits
12..15, in `data`, so the C's `Sn:389D183DD` shows in KAT as serial `0x89D183DD` with `data` starting `0x8013…`.
`button` = the 6-bit code (shown as "Btn 0x1", "Btn 0x20": the references name no buttons). `counter` = `n`.
`encoder_capable` = true.

## Encoder

`subghz_protocol_belkorn_encoder_prepare_packets` and `get_upload`, with `encoder.repeat` = 3:

1. The frame is read again and the remote's origin is solved back.
2. The counter goes up by one, `& 0xFFFF`, so 0xFFFF goes to 0.
3. The button is the requested code if it is one bit of the 6-bit field. "Send button…" offers the C's custom
   buttons 0x01, 0x02, 0x04 and 0x20, and the C's button editor takes any value. Otherwise it is the captured
   button.
4. The new counter's frame is built from the origin, format A or B by the counter's parity.

The upload is the frame three times, each as 118 pulses (HIGH first, 481 or 962 µs) + HIGH 481 + LOW 7696. With
`encoder.repeat` = 3 a press is nine frames, as the reference sends it (the remote sends about nine). A key file's
`Repeat` line sets the number of uploads, as in the C. Like the C, the encoder re-derives the whole frame,
including the sync, and does not require the input to pass the check.

## Key file

`Protocol: BelKorn`, `Bit: 118` (required), `Key` (8 bytes: frame bits 0..63), `Data` (8 bytes: bits 64..117,
right-aligned). The C's deserialize requires `Bit: 118` and both lines. KAT also requires the frame to pass the
check, as for every key file it loads.

## Add Manually

`BelKorn` (RogueMaster's "BelKorn"), 433.92 MHz AM650:

- the serial is one `rand()` (31 bits) as 5 bytes, so its high nibble and bit 31 are 0;
- button 0x01, counter 0x0005;
- the origin is the protocol-wide L24 low byte 0xE0 with b3 = 0. The reference's note: "a receiver in learn mode
  takes whatever it is handed".

The first frame is at the counter as given, not advanced. The serial editor takes 5 bytes (36 bits used). See
[Add Manually](add_manually.md).

## Validation

No capture of a BelKorn remote exists in the references (no unit-test recording) or in IMPORTS. KAT is checked
against a host build of the C: `REFERENCES/tools/oracle/bin/subghz_oracle_foxfwenc`, built from FoxFW v2.1.1.
FoxFW's BelKorn is RogueMaster's function for function. The build now compiles FoxFW's vendored mbedtls (AES only)
for its new Kia V6.

- **400 generated key files** cover random origins, serials and counters (both formats, every lap, the b6 wrap at
  c = 0x45/0x46 and 0xFFFF) and all six one-hot buttons. The C reads the same 36-bit serial, button and counter as
  KAT from every one. Its transmission of the next press is the same as KAT's, pulse for pulse (nine frames each,
  `Key` and `Data` rewritten). `Repeat: 1` and `Repeat: 5` give one and five uploads (3 and 15 frames).
- **720 RAW files**, made from the C's transmissions: clean; with one pulse set to a window edge (301/302,
  659..661, 781..783, 1141..1143, 700 or 2499/2500 µs); with a pulse dropped and its neighbours merged; with a
  pulse split by a 60 µs glitch; with ±170 µs jitter on every pulse; and with the gaps cut to 2499/2500/2501 µs.
  KAT and the C give the same decodes in the same order in every file: 5759 decodes.

Unit tests (`src/protocols/belkorn.rs`):

- the frame fields round trip;
- the origin and counter solve back from all 65536 counters' frames;
- the counter law of both formats, the b6 wraps and the jitter's u16 wrap;
- both formats decode;
- broken frames are rejected: the sync, no button or two, a pulse in neither window, a dropped pulse, a missing or
  short gap;
- the encoder's next press, the button rule and the 0xFFFF wrap;
- the upload's timing;
- the key file, both ways;
- "Add Manually";
- the upload through the whole registry at every test frequency: BelKorn alone reads it.
