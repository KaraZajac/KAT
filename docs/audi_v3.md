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
the others. The firmware of a BCM2 module ([the module's side](#the-modules-side-from-its-firmware)) reads the frame
the same way and says what its bytes are: which of the car's keys sent it, the button, the key's status, a 48-bit
AES-128 MAC and the counter. FoxFW's frame is the same frame from a car's second key, read one bit early
([below](#foxfws-frame)).

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
| 0 | `[0][key number:3][status:2][button:2]` ([the module's side](#the-modules-side-from-its-firmware)); the A4 key: `0x0A` Lock, `0x08` Unlock, `0x09` Trunk (as the recordings are named) |
| 1–6 | 48 bits that change from press to press with no pattern: the last 6 bytes of an AES-128 block (a MAC) |
| 7–10 | counter, 32 bits, big-endian: up by 3 with every press |
| 11 | the sum of bytes 0–10, mod 256 |

The sum holds on all 61 frames (21 Lock, 20 Unlock, 20 Trunk). The counter goes up by 3 from every press to the next
(58 of 58 steps) and goes on from one recording to the next in the order they were made, with a few unrecorded presses
between them. No serial is sent in the clear: byte 0 says which of the car's keys sent the frame, and the MAC proves
it.

Read from the start bit, as the C reads, the fields straddle these bytes: the first eight bits are `0x05` (Lock) or
`0x04` (Unlock, Trunk), which the C's whitelist refuses, so the C reads nothing from this key.

### The module's side (from its firmware)

The key specialist also read the firmware of an Audi A5's BCM2 (J393, part 8K0907064D, "BCM2 2.0", software of
June 2010; a Renesas V850ES/FJ3). Its remote routines (receive `0x7E204`, handler `0x7E3E6`, check `0x7E002`):

- **Receive.** The module takes 12 bytes from its receiver and keeps them when byte 11 is the sum of bytes 0–10,
  whatever byte 0 is.
- **Byte 0** is `[0][key number:3][status:2][button:2]`. The key number (0–7) says which of the car's learned keys to
  check the frame against; the module numbers its keys from 1, so number 0 is key 1. The button: 0 Unlock, 1 Trunk,
  2 Lock, 3 a fourth button. The status is not checked: its values `00` and `01` set a fault for that key (`01` with a
  warning), `10` and `11` clear them; most likely the key's battery. The A4 key sends key number 0, status `10`.
- **The MAC.** Bytes 1–6 are the last 6 bytes of an AES-128 block that the module computes, under a 128-bit key of the
  car, from a 4-byte value of the car, the counter (bytes 7–10), the 4-byte ID of the key with that number and the
  button. Nothing in the frame can be read or made without the car's key; the scheme shows no weakness of its own.
- **The counter** must be ahead of the one the module stored for that key, within a window.
- **Held buttons.** The short bursts that follow most presses (above) are "button held" frames: the module compares
  their key number and button with the press it accepted, for comfort opening and closing (holding Unlock opens the
  windows).

The A4 recordings are of another car than the firmware's module, so the MAC is read from the firmware and not yet
checked on frames: that needs the key recordings and the module reads of one car.

### FoxFW's frame

The C's two buttons are this byte 0 read from the start bit, shifted by one: `0x0C` is `0x18` (key 2, Unlock) and
`0x0D` is `0x1A` (key 2, Lock). FoxFW's own decoder (the reference oracle) reads synthetic frames of key 2's Unlock and
Lock as its `0x0C` and `0x0D`, with a counter of half the frame's (`0x8002` for `0x10005`) and a check byte that is
the next bits, not a check. So FoxFW's key was most likely a car's second key, its decoder written from its frames
read one bit early, with the key number taken for part of the button. FoxFW has no recording of its own to confirm it.

### What KAT reads

KAT tries the BCM2 key's layout first. A frame is read in it when:

- 97 bits are in. When the window ends at a gap after 96 bits, the gap is the 97th bit's second half (merged into it),
  and that bit is the gap's, `level ? 0 : 1`, as any bit is.
- The start bit is 0. When it is 1, the recording has the other polarity, and all bits are read inverted.
- Byte 0's bit 7 is clear (it is in no frame seen), and byte 11 is the sum of bytes 0–10: every key number, status and
  button, as the module reads them.
- The MAC (bytes 1–6) is neither all `0x00` nor all `0xFF`, as the C wants of its hop. Twelve zero bytes pass the sum,
  and a run of one level in a recording reads as just that (two Opel remote recordings in IMPORTS have one); an AES
  output is all `0x00` or all `0xFF` once in 2⁴⁷.

Otherwise the C's 96 bits and whitelist decide. A frame read in this layout is named with its key: "Lock, key 2".

## Encoding the BCM2 frame

KAT can make a key's **next press** — the real rolling code, not a replay — but only for a car whose
secrets the owner has loaded. The frame's bytes 1–6 are an AES-128 MAC under a 128-bit key held in
the car's own module, so nothing in the frame can be forged without it, and KAT ships no keys.

**The car key file** `audi_bcm2_keys` in KAT's config folder, one car per line:

```
<32-hex K> <8-hex car_value> <8-hex id_key1> [<8-hex id_key2> …] : name
```

`K` is the immobiliser block's MAC key, `car_value` the per-car value folded into every MAC, and the
IDs are each learned key's 4-byte ID in key-number order (key 1 first, up to 8). All of it is read
from the car's BCM2 module; a `#` line is a comment. KAT reads the file once at startup.

**What KAT does.** When a loaded car key reproduces a captured frame's MAC — KAT recomputes it and
compares — the capture is marked encodable (there is no serial in the frame to match on, so the MAC
is the match). "Send next code" then raises the counter by 3 (the observed step; the module accepts
any counter ahead of the one it stored, within a window), recomputes the MAC with the car's key, and
builds the frame; choosing Lock / Unlock / Trunk re-aims the same key. Without a matching key, or for
the C's frames, KAT offers Replay only.

**Status.** The MAC is read from the module's firmware and reproduces every frame KAT's own encoder
builds (the tests), but it is **not yet confirmed against a real car's frames** — that needs
recordings of a dumped car's own keys. Treat the encoder as experimental until then.

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
0–7, `extra` = bytes 8–10 `<< 8 |` byte 11, with bit 40 of `extra` set. `button` = byte 0, named with its button and
key ("Unlock, key 1" for `0x08`, "Lock, key 2" for `0x1A`), `counter` = the low 16 bits of the 32-bit counter,
`crc_valid` = the sum. The interface checks the sum on these frames only and shows the C's frames as not checked.

## Key file

`Protocol: Audi V3`, `Bit: 96`, `Key` = `button << 56 | hop`, then `Btn`, `Cnt` (24 bits) and `Chk`, all decimal.
Loading needs `Bit: 96`; missing `Cnt` / `Chk` read as 0; no gate is applied. A key file of a BCM2 frame holds its 12
bytes the same way and loads with its sum checked again. A key file does not say which layout it holds, so byte 0
`0x0C` / `0x0D` loads as the C's frame (its two buttons) even when the sum holds.

## Validation

- Reference oracle `subghz_oracle_foxfw`: synthetic frames for both buttons (made-up hops, counters and check bytes)
  give identical key, button, counter and check; an unlisted button, a zero hop, an all-FF hop, a 19-pulse preamble, a
  wrong opening pulse, a gap inside the data and a frame one bit short are rejected by both.
- The A4 key's 61 frames read with the buttons their recordings are named after, valid sums and a counter that goes up
  by 3 with every press (`the_a4_bcm2_recordings_read_as_their_buttons`); in those files KAT reads nothing else.
  Synthetic BCM2 frames read the same whole, with the 97th bit lost in the gap and in the other polarity, and not with
  a wrong sum (`the_bcm2_keys_frame_reads_with_its_sum`).
- Every key number (0–7), status (0–3) and button (0–3), with made-up MAC bytes: each reads whole, with the 97th bit
  lost and in the other polarity, with its sum and the name of its button and key
  (`every_key_number_status_and_button_reads_in_the_bcm2_layout`, 384 frames); byte 0 with bit 7 set, twelve zero
  bytes and a MAC of all `0x00` or `0xFF` read in neither layout; key 2's Unlock and Lock begin with the C's `0x0C` / `0x0D` from the start bit and read whole in KAT
  (`the_cs_two_buttons_are_a_second_keys_unlock_and_lock`), where the FoxFW oracle reads them as its two buttons; a
  BCM2 frame keeps its layout through a key file, and a C key file whose check byte happens to be the sum stays the
  C's (`key_files_keep_the_layout`).
- Encoder: the MAC's AES-128 passes the FIPS-197 C.1 vector (`aes_matches_fips197`) and reproduces the research
  encoder's bytes (`the_mac_matches_the_research_encoder`); with a loaded car key a frame is encodable and "Send next
  code" / a chosen button build the next press, which decodes back with counter + 3 and a MAC the same key reproduces
  (`encodes_the_next_press_for_a_loaded_car`); without a matching key, and for the C's frames, nothing encodes
  (`the_encoder_is_inert_without_a_matching_key`). The key-file parser reads cars and flags bad lines
  (`parse_audi_bcm2_keys_reads_cars_and_flags_bad_lines`).
- Unit tests: the oracle-checked frames and rejections, key-file round trip, the encoder.
