---
layout: default
---

# Audi V3 Protocol

**Rust module:** `src/protocols/audi_v3.rs`
**Reference:** FoxFW `lib/subghz/protocols/audi_v3.c` (a port of the firmware's `audi_v3.c` via KeeLoq-RX
`arke_vag.py`). **Receive only, as the reference** (the encoder is NULL).

## Overview

A 2-FSK VAG key-fob frame. In FoxFW it is 96 bits: button, a 56-bit encrypted hop, a 24-bit counter and a check byte
the C displays but never verifies, gated only by a button whitelist and a non-degenerate hop. The first key recorded
in IMPORTS, a key specialist's 2008–2015 Audi A4 key (BCM2 system, 868.35 MHz; `VAG/more/2008-2015_audi_a4_bcm2_*`,
20 presses of each button), sends this signal with a frame FoxFW does not read. KAT reads it
([below](#the-bcm2-keys-frame-kats-from-the-captures)): a start bit and 12 bytes, of which the last is the sum of
the others.

## Timing / modulation

FM (2-FSK), flags 433 | 315 | AM | FM. A custom half-bit demodulator, not the Manchester table:

| Pulse | Duration |
|-------|----------|
| preamble | 220..470 µs |
| long | 580..980 µs |
| short | under 580 µs |
| gap | 980 µs or more |

The A4 key's half bit is about 383 µs. Each press sends one burst of about 181 ms: some 270 preamble pulses, then the
frame. After most presses come bursts of 18.8 ms every 39 ms (none to 30 of them), each a short preamble and the same
few bits; they hold no frame.

## Frame Layout

96 bits, MSB first: `button:8 | hop:56 | counter:24 | chk:8`.

### The BCM2 key's frame (KAT's, from the captures)

The A4 key's frame is 97 bits: the bit of the opening long pulse (0, a start bit), then 12 bytes.

| Bytes | Field |
|---|---|
| 0 | button: `0x0A` Lock, `0x08` Unlock, `0x09` Trunk (as the recordings are named) |
| 1–6 | 48 bits that change from press to press with no pattern (encrypted) |
| 7–10 | counter, 32 bits, big-endian: up by 3 with every press |
| 11 | the sum of bytes 0–10, mod 256 |

The sum holds on all 61 frames (21 Lock, 20 Unlock, 20 Trunk). The counter goes up by 3 from every press to the next
(58 of 58 steps) and goes on from one recording to the next in the order they were made, with a few unrecorded presses
between them. No serial is sent in the clear.

Read from the start bit, as the C reads, the fields straddle these bytes: the first eight bits are `0x05` (Lock) or
`0x04` (Unlock, Trunk), which the C's whitelist refuses, so the C reads nothing from this key. Whether the key FoxFW's
frame came from (`0x0C` Unlock, `0x0D` Lock) sent a start bit is not known, so such frames are read as the C reads them.

KAT tries this layout first. A frame is read in it when:

- 97 bits are in. When the window ends at a gap after 96 bits, the gap is the 97th bit's second half (merged into it),
  and that bit is the gap's, `level ? 0 : 1`, as any bit is.
- The start bit is 0. When it is 1, the recording has the other polarity, and all bits are read inverted.
- Byte 0 is `0x08` to `0x0B`, and byte 11 is the sum of bytes 0–10.

Otherwise the C's 96 bits and whitelist decide.

## Decoder Steps

`subghz_protocol_decoder_audi_v3_feed`:

1. Reset: a preamble-length pulse starts the count. Preamble: preamble-length pulses are counted; a long pulse with a
   count of 20 or more opens the data window and supplies bit 0; anything else resets.
2. Data: a pulse of 980 µs or more ends the frame. A long pulse is one bit (a long arriving between a bit's two shorts
   ends the frame); two consecutive short pulses are one bit. A bit's value is `level ? 0 : 1` of the pulse that
   completes it. The C evaluates the frame at 96 bits; KAT reads on to the 97th, or to the gap.
3. KAT: a frame in the BCM2 key's layout (above) is reported with its sum checked. Otherwise, as the C: reported only
   when all 96 bits are in, the button is `0x0C` (Unlock) or `0x0D` (Lock) and the hop is neither all `0x00` nor all
   `0xFF`.

## Field mapping

The C's frame: `data` = `button << 56 | hop` (the C's `generic.data`), `data_count_bit` = 96, `button` = the button
byte, `counter` = the low 16 bits of the 24-bit counter, `extra` = `counter << 8 | chk`, `serial` None, `crc_valid`
false (no check exists), `encoder_capable` false.

A frame in the BCM2 key's layout is packed as the C packs its 96 bits, so its key files are the C's: `data` = bytes
0–7, `extra` = bytes 8–10 `<< 8 |` byte 11, with bit 40 of `extra` set. `button` = byte 0, `counter` = the low 16 bits
of the 32-bit counter, `crc_valid` = the sum. The interface checks the sum on these frames only and shows the C's
frames as not checked.

## Key file

`Protocol: Audi V3`, `Bit: 96`, `Key` = `button << 56 | hop`, then `Btn`, `Cnt` (24 bits) and `Chk`, all decimal.
Loading needs `Bit: 96`; missing `Cnt` / `Chk` read as 0; no gate is applied. A key file of a BCM2 frame holds its 12
bytes the same way and loads with its sum checked again.

## Validation

- Reference oracle `subghz_oracle_foxfw`: synthetic frames for both buttons (made-up hops, counters and check bytes)
  give identical key, button, counter and check; an unlisted button, a zero hop, an all-FF hop, a 19-pulse preamble, a
  wrong opening pulse, a gap inside the data and a frame one bit short are rejected by both.
- The A4 key's 61 frames read with the buttons their recordings are named after, valid sums and a counter that goes up
  by 3 with every press (`the_a4_bcm2_recordings_read_as_their_buttons`); in those files KAT reads nothing else.
  Synthetic BCM2 frames read the same whole, with the 97th bit lost in the gap and in the other polarity, and not with
  a wrong sum (`the_bcm2_keys_frame_reads_with_its_sum`).
- Unit tests: the oracle-checked frames and rejections, key-file round trip, decode-only.
