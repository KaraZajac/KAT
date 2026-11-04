---
layout: default
---

# Chrysler V0 Protocol

**Rust module:** `src/protocols/chrysler_v0.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/chrysler_v0.c` (primary); identical decoders in `REFERENCES/Flipper-ARF/lib/subghz/protocols/chrysler.c` ("Chrysler", FOBIK GQ43VT) and `REFERENCES/doctor-firmware/lib/subghz/protocols/chrysler_v0.c`

## Overview

Older Chrysler/Dodge/Jeep FOBIK keyfobs. PWM with a short HIGH pulse and two long-LOW symbols: a "1"
payload bit is HIGH ≈600 µs + LOW ≈3400 µs; a "0" payload bit is HIGH ≈300 µs + LOW ≈3700 µs. ~24
preamble pairs (short HIGH + long LOW) precede each frame. 80-bit frame: the first 64 bits are
`data` (payload bytes 0–7), the last 16 bits (`Key_2`, payload bytes 8–9) are kept right-aligned in
`extra`; `data_count_bit = 80`.

Crypto is a proprietary seed-XOR: `seed = reverse6(key[0] >> 2)` (a 6-bit reversed counter);
`transform_block` XORs the 9 payload bytes with `xor_table[seed & 0x0F]`, with an extra nibble flip for
Lock. Each press sends payload A (even seed, plaintext SnA) and payload B (odd seed = A − 1, plaintext
SnB). A frame is reported only when its structural `check_ok` passes (as in the C); `crc_valid`
reflects it.

The newer FCA fobs in `IMPORTS/chrysler` (Pacifica) and `IMPORTS/jeep` (Renegade) are Fiat V2 frames:
no reference's Chrysler V0 decodes them, in either polarity.

## Timing

| Parameter   | Value   | Notes                  |
|-------------|---------|------------------------|
| Short HIGH  | 300 µs  | ±150 µs (te_delta)     |
| One-short HIGH | 600 µs | "1" bit HIGH width (HIGH > 750 µs ends a frame) |
| Long LOW A  | 3400 µs | ±400 µs (long_delta)   |
| Long LOW B  | 3700 µs | ±400 µs                |
| Gap         | 8000 µs | te_gap                 |
| Frame gap   | 15600 µs|                        |
| Min bits    | 80      |                        |
| Preamble    | 24 pairs| >15 needed             |

## Frame Layout (80 bits / 10 payload bytes)

- **byte 0:** `(reverse6(seed) << 2) | header_low2`
- **bytes 1–9:** XOR-masked plaintext
  - Payload A (even seed, byte 0 MSB clear): check `p0 ^ 0xC3 == p4` and `p0 ^ p5` = 0x04 (Lock) / 0x08 (Unlock); serial = SnA = p0..p3
  - Payload B (odd seed, MSB set): check `p0 == p4` and `p0 ^ p5 == 0x62`; button Unlock when `p8 ^ p3 == 0x10`; serial = SnB = p0, p1, p2, p7

## KAT fields

`serial` = SnA or SnB, `button` = 1 Lock / 2 Unlock (same as KAT's generic codes), `counter` = the 6-bit
seed (the C's `Cnt`), `extra` = `Key_2`.

## RF

- **Encoding:** PWM (short HIGH + dual long LOW)
- **RF modulation:** AM
- **Encryption:** proprietary seed-XOR (`transform_block` + `xor_table`); gated on structural `check_ok`
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset** — a short HIGH pulse begins the seek.
2. **Seek** — count preamble pairs (short HIGH + long LOW); after >15 pairs a non-short HIGH (or a gap) enters Data.
3. **Data** — classify each HIGH/LOW pair into a payload bit (`bit ^ 1`); 64 bits into `data` then 16 into `Key_2`. At 80 bits (or on a terminating gap/invalid pulse with ≥80 bits) run `decode_packet` and emit when `check_ok`.

## Encoder

Supported. Recovers the plaintext from the captured frame, flips the button bits (`plain_a[5] ^= 0x0C`,
`plain_b[3] ^= 0x30`) when the requested button differs, and emits payload A (even counter) then B
(counter − 1), each as 24 preamble pairs + short + 15.6 ms gap + 80 PWM bits + short + 15.6 ms gap
(424 entries; the Flipper repeats the upload twice while held, KAT sends it once). Buttons: Lock → 1,
Unlock → 2; Trunk/Panic keep the captured button (no reference models other buttons).

**Counter — deviation, from real captures.** KAT's counter is seed − 1, made even:

- from a payload-A capture of seed N it sends A(N−2) + B(N−3);
- from a payload-B capture it sends A(N−1) + B(N−2).

The payload of the captured type is the fob's next press of that type. The other payload carries
the captured plaintext, as in the C, but A and B frames don't share their plaintext, so the fob
never sends that frame.

The evidence is the Chrysler PT Cruiser 2006 fob in `IMPORTS/chrysler` (two files, ten presses each,
about 1.5 s apart). It sends one payload per press, alternating A and B, and its seed counts **down**
by one: A(N), B(N−1), A(N−2)… Re-encoding each press reproduces the press two later bit for bit, all
80 bits, in 16 of 16 cases (test `reencode_reproduces_the_fobs_next_press`).

The references count up:

- ProtoPirate's emulate scene and doctor-firmware's encoder add 1 to `Cnt`, which re-sends the
  captured press.
- Flipper-ARF adds 2 ("to keep the seed even and actually move it"), which sends an earlier press.

A fob that counts down has already sent both. The A/B frame layout and the transform are the C's
unchanged.

## Validation

Compared with the compiled ProtoPirate `chrysler_v0.c` and Flipper-ARF `chrysler.c` (host harness):
decoder output identical on 3933 frames from 4500 C-encoded uploads (plain, ±140 µs jitter, glitched)
and on every IMPORTS capture. The two real PT Cruiser 2006 captures decode to the same 20 frames
(80 bits each) in KAT, ProtoPirate, Flipper-ARF and doctor-firmware: payload A serial A11E7D24,
payload B serial 4725BF79, Open = 2 and Close = 1. No other capture decodes as Chrysler V0.

The encoder upload was identical pulse-for-pulse on 4500 random captured frames × buttons
(ProtoPirate with `Btn`/`Cnt` set, and ARF with its D-pad). Only KAT's choice of `Cnt` differs since
then (the counter deviation above); the upload for a given `Cnt` is unchanged. Unit tests cover `reverse6`, `transform_block`, payload-A fields
and the encode → decode of every seed and button.
