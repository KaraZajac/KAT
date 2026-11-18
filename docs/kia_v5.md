---
layout: default
---

# Kia V5 Protocol

**Rust module:** `src/protocols/kia_v5.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v5.c` (encoder since 2026-05-19); Flipper-ARF
`kia_v5.c` (same upload); doctor-firmware `kia_v5.c` (Lock and Unlock)

## Overview

Kia Manchester keyfob at 400/800 µs: 64 data bits + a 3-bit CRC field. The key is sent bit-reversed; its
reverse ("yek") is `button4 << 60 | serial28 << 32 | hop32`, and the 16-bit counter is the hop decrypted by
the "mixer" cipher keyed with the KIA V5 key (keystore type 13).

## Timing

| Parameter | Value  | Notes   |
|-----------|--------|---------|
| Short     | 400 µs | ±150 µs |
| Long      | 800 µs | ±150 µs |
| Preamble  | > 40 counted pairs | then a long HIGH sync |
| Bits      | 64–67  | 64 key + up to 3 CRC-field bits |

## Frame Layout

- `data` = the 64 key bits as received; serial/button/counter from `yek = reverse(data)`.
- CRC field = `0`, then the 2-bit CRC of the key (`kia_v5_calculate_crc`); kept in `extra`.
- Buttons: 0x1 Lock, 0x2 Unlock, 0x4 Trunk, 0x8 Horn (see below).

## Buttons

| Code | Name | From |
|------|------|------|
| 0x1 | Lock | doctor-firmware `kia_v5.c` ("Close") |
| 0x2 | Unlock | doctor-firmware ("Open") |
| 0x4 | Trunk | Flipper-ARF `kia_v5.c` |
| 0x8 | Horn | Flipper-ARF |

The references do not agree. Flipper-ARF has 0x1 Unlock and 0x2 Lock, which KAT also had until 1.16;
doctor-firmware has 0x1 Close, 0x2 Open and 0x3 Boot; ProtoPirate names no button. KAT now has Lock and Unlock
as doctor-firmware.

It does not take 0x3 for the trunk. The frame is laid out like the code word of Microchip's HCS360 in
Manchester mode: 32 encrypted bits, a 28-bit serial, four button bits, a low-battery bit and a 2-bit CRC, each
field sent lowest bit first, in half bits of 400 µs. The CRC is that chip's: it holds in every recorded frame.
In that layout the four button bits are four inputs, and one button is 1, 2, 4 or 8. (The remote is not that
chip as it comes: it sends 200 cycles of preamble where the chip sends 16, and its "mixer" cipher is KeeLoq
cut to 144 rounds, where the chip runs 528.)

The recordings in IMPORTS hold 0x1 (two remotes, 20 presses each) and 0x2 (one remote's two key files), and
no file says which button was pressed. So no name here is checked against a remote by KAT, and Trunk and
Horn have one source only.

## RF

- **Encoding:** Manchester, common.rs port with the C's sense `level ? ShortHigh : ShortLow`
- **RF modulation:** FM
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps (`kia_protocol_decoder_v5_feed`)

1. **Reset** — a short HIGH starts the preamble; Manchester state Mid1.
2. **CheckPreamble** — count pairs; a long HIGH after more than 40 starts the data.
3. **Data** — Manchester-decode (bits after the 64th go to the CRC field, at most 67); the first
   out-of-tolerance pulse ends the frame, reported if ≥ 64 bits arrived (`data_count_bit` = min(bits, 67)).

The C only displays the CRC; KAT sets `crc_valid` when the received CRC bits match the key's CRC (true for
all 117 complete frames in IMPORTS). KAT's earlier port used a hand-written Manchester table: its serials
changed between presses of the same fob, and it matched none of the C's frames.

**Deviation: a square wave is not a frame.** A run of 800 µs pulses counts as this decoder's preamble and then,
in the data step, as 67 Manchester bits `0101…`: the key `5555555555555555` (serial `AAAAAAA`, button `A`), with
a CRC field that comes out right. The preamble of a [Honda V6](honda_v6.md) frame is such a wave (about 200 cells
of 800 µs). ProtoPirate's, doctor-firmware's, Flipper-ARF's and FoxFW's C each list nine "Kia V5" frames with
that key in the three 300-press Honda recordings of IMPORTS (`honda/more/HondaH1_OEM2_300*.sub`). KAT drops a key
of alternating bits, in either phase: no fob has it, and in KAT's registry the decode would end the Honda V6
frame it sits in.

## Encoder

Supported (new; `kia_protocol_encoder_v5_deserialize` + `get_upload`): hop = `mixer_encode(serial,
counter + 1, button)`, key = reverse(yek), CRC recomputed; 200 × (HIGH 400, LOW 400), LOW 400, HIGH 800,
LOW 400, HIGH 400, then the 64 key bits, `0` and the 2 CRC bits as Manchester (1 → LOW, HIGH; 0 → HIGH,
LOW), closing LOW 400 + HIGH 400; adjacent equal levels merged. KAT's generic button maps to the codes
above: Lock 0x1, Unlock 0x2, Trunk 0x4, Panic 0x8 (Horn); anything else keeps the captured button.

## Validation

C oracle: on every IMPORTS capture of the test corpus (both polarities) KAT reports the C's 119 frames
(serial, button, counter, CRC field), and on the captures in `more/` every frame but the nine above; 600 random C-encoded frames — uploads identical, all decode identically. The encoder's
`mixer_encode` inverts the decoder's `mixer_decode` on the real frames (re-encoding a captured frame's fields
gives its exact key). Flipper-ARF only reports 67-bit frames (117 of the 119).

## Keystore

KIA V5 key (keystore type 13, `kia_v5_key`), for both mixer directions.
