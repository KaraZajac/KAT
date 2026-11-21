---
layout: default
---

# Chrysler V0 Protocol

**Rust module:** `src/protocols/chrysler_v0.rs` (a port of the references' signal and key files; KAT reads the frame by its own rule, not by their model)
**Reference:** `REFERENCES/ProtoPirate/protocols/chrysler_v0.c` (primary); identical decoders in `REFERENCES/Flipper-ARF/lib/subghz/protocols/chrysler.c` ("Chrysler", FOBIK GQ43VT) and `REFERENCES/doctor-firmware/lib/subghz/protocols/chrysler_v0.c`

## Overview

Older Chrysler/Dodge/Jeep FOBIK keyfobs: pulse-width coding on AM, an 80-bit frame. The frame is not
encrypted. It is a sum byte, the low byte `n` of a 16-bit press counter, and eight bytes each XORed with `n`; when
the sum is odd, everything behind the sum byte is sent backwards. KAT reads and builds the frame by this rule,
which every one of the 732 frames in the six recordings of a Chrysler PT Cruiser 2006 fob follows, and so shows
the fob's own fields: its fixed code, its button and its counter.

The references read the same frame as a 6-bit seed and nine bytes under one of sixteen XOR masks. That model
shows 160 different serials for the one fob, counts the sum instead of the presses, and builds frames no fob
sends. KAT departs from it ([below](#where-kat-departs-from-the-references)); it keeps their signal, their
state machine and their key files.

The newer FCA fobs in `IMPORTS/chrysler` (Pacifica) and `IMPORTS/jeep` (Renegade) are Fiat V2 frames:
no reference's Chrysler V0 decodes them, in either polarity.

## Timing

| Parameter   | Value   | Notes                  |
|-------------|---------|------------------------|
| Short HIGH  | 300 µs  | ±150 µs (te_delta); a 0 bit, and the preamble |
| One-short HIGH | 600 µs | a 1 bit (HIGH > 750 µs ends a frame) |
| Long LOW A  | 3400 µs | behind a 1; ±400 µs (long_delta) |
| Long LOW B  | 3700 µs | behind a 0 and in the preamble; ±400 µs |
| Gap         | 8000 µs | te_gap: a longer LOW ends a frame |
| Frame gap   | 15600 µs| before each copy of the frame |
| Min bits    | 80      |                        |
| Preamble    | 24 pairs| >15 needed             |

## A press

In the recordings, a press is:

- 24 preamble pairs: HIGH 300 µs, LOW 3.7 ms;
- a short HIGH and a 15.6 ms gap, then the 80 bits;
- a short HIGH and a 15.6 ms gap, then the 80 bits again; the press ends at the last bit.

Of the 738 presses in the six recordings, 731 hold both copies, and in every one the two are the same; the rest
are cut short. The decoder reads the copy behind the preamble; the second has none. Presses 0.88 to 0.92 s apart,
one straight after another (57 in `PT_LCK100.sub`, 11 in `PT_LCK250.sub`), are each the next count: a held
button, it seems, sends press after press. Before the preamble the recordings show a HIGH of very different
lengths (65 µs to 0.96 s, 21 ms in the middle), which the [Not known](#not-known) section takes up.

## Frame (80 bits)

Each byte goes out LSB first. The fob holds nine bytes: `n` and `p1` to `p8`.

| Byte | Content |
|------|---------|
| 0 | the sum (mod 256) of `n` and `p1..p8` |
| 1 | `n`: the low byte of a 16-bit counter, one lower with every press |
| 2..9 | `p1 ^ n` … `p8 ^ n` |

When the sum is odd, bytes 1 to 9 go out backwards, the 72 bits last bit first. Read MSB first, as a receiver
sees them, they are then the nine bytes in reverse order. An even sum is what the references call payload A, an
odd one payload B. The sum goes down by one with every press (by two where the counter's high byte changes), so
the two orders alternate.

In the recorded fob, `p1..p8` are:

| Bytes | Value | What they are |
|-------|-------|---------------|
| `p1..p4` | `FD 3B A1 C3` | the fixed code, the same in every frame (KAT's serial) |
| `p5` | `20` Lock, `10` Unlock | the button |
| `p6` | `3B` down to `38` | the counter's high byte, one lower each time `n` passes from 0 to 255 |
| `p7 p8` | `A1 C3` | `p3 p4` again |

The open and close recordings, one session to judge by their names and counts, show one counter for both
buttons: the open recording's last press (Unlock) has the count 3B86, the close recording's first (Lock) 3B85.

### Two recorded frames by hand

The first two presses of `Chrysler PT Cruiser 2006 close.sub`:

| Received | Sum | Order | `n` | `p1..p8` | Counter |
|----------|-----|-------|-----|----------|---------|
| `07A11E7D2462A57D` `2462` | `E0` (even) | forwards | `85` | `FD 3B A1 C3 20 3B A1 C3` | `3B85` |
| `FB4725BFA44725BF` `7984` | `DF` (odd) | backwards | `84` | `FD 3B A1 C3 20 3B A1 C3` | `3B84` |

First press: `07` backwards is `E0`, even, so the nine bytes come forwards, each backwards: `A1 1E 7D 24 62 A5 7D 24
62` are `85 78 BE 24 46 A5 BE 24 46`. `n` is `85`, and XORed with it the rest are `FD 3B A1 C3 20 3B A1 C3`. Their
sum with `85` is `4E0`, so `E0`. Second press: `FB` backwards is `DF`, odd, so `47 25 BF A4 47 25 BF 79 84` are the
bytes in reverse order: `n` is `84`, then `79 BF 25 47 A4 BF 25 47`, which XORed with `84` give the same eight
bytes.

## Fields

| Field | Value |
|-------|-------|
| `serial` | `p1..p4` (`FD3BA1C3`) |
| `button` | `p5`: `20` is shown as Lock, `10` as Unlock, any other code as Other |
| `counter` | `p6 << 8 \| n` (`3B85`) |
| `crc_valid` | the sum |
| `data` / `extra` | the first 64 frame bits / the last 16 (the references' `Key_2`), as received |

## Where KAT departs from the references

ProtoPirate's `chrysler_v0.c`, Flipper-ARF's `chrysler.c` and doctor-firmware's copy read byte 0 as a 6-bit seed,
`reverse6(key[0] >> 2)`, and two header bits, and bytes 1..9 under one of sixteen XOR masks,
`xor_table[seed & 0x0F]`, with a nibble of it flipped for Lock. In the terms of the rule:

- **Their seed is the sum's low six bits** and their two header bits the sum's top two, backwards. Their counter
  (`Cnt`) is that seed: it counts the sum, not the presses, and skips a value where `p6` changes.
- **Their sixteen masks stand in for `n`** over runs of presses, not over the whole count. Their serials (SnA,
  plaintext bytes 0..3 of payload A; SnB, bytes 0, 1, 2 and 7 of payload B) are fixed bytes XORed with whatever
  `n` and the mask leave, so they change with the count: over the six recordings, 160 different serials for one
  fob, which always sends `FD 3B A1 C3`.
- **Their checks test this fob's bytes.** Payload A's check holds when `p4` is `C3` (and `p5` a button), payload
  B's when `p8` equals `p4` and `p3 ^ p8` is `62`. A fob whose `p3` or `p4` differ fails them at every count.
- **They do not test the sum,** the frame's own check. A recorded frame with a bit their checks do not cover
  changed (byte 2) still passes them, read as another serial.

KAT reads every frame whose sum holds, whatever its fixed bytes, and none whose sum fails. On the recordings both
read the same 732 frames (the [Validation](#validation) section).

## Decoder

1. **Reset** — a short HIGH pulse begins the seek.
2. **Seek** — count preamble pairs (short HIGH + long LOW); after >15 pairs a non-short HIGH (or a gap) enters Data.
3. **Data** — classify each HIGH/LOW pair into a bit (`bit ^ 1`); 64 bits into `data`, then 16 into `extra`. At
   80 bits (or on a terminating gap or invalid pulse with ≥80 bits) the frame is read by the rule and reported when
   its sum holds and `p1..p4` are neither `00000000` nor `FFFFFFFF`.

Steps 1–3 are the C's state machine, unchanged.

## Encoder

Supported: the fob's next press.

- The counter one lower, borrowing from `p6` (and from `0000` to `FFFF`).
- `p5` the requested button: KAT's Lock and Unlock are `20` and `10`. Trunk and Panic keep the captured button:
  no recording shows their codes.
- The other bytes as captured, the new sum, and the order the sum gives.
- Sent as the fob sends a press: 24 preamble pairs, then twice a short HIGH, the 15.6 ms gap and the 80 bits,
  closed by a 15.6 ms gap. The upload is the whole press, sent once; a key file's `Repeat` line is not read.

From a frame whose sum fails (a key file can hold one) KAT sends nothing.

The references re-encode with their model: payload A from the captured plaintext with their seed + 1
(ProtoPirate's emulate scene, doctor-firmware), which sends the captured press again, or + 2 (Flipper-ARF), which
sends an earlier press; then payload B with A's plaintext, a frame no fob sends. Each with its own preamble, and
the whole upload twice.

## Key files

The references' lines, written with their model, since the firmwares read `Plain_x`, `Btn` and `Cnt` back when
they send from a file: `Protocol: Chrysler V0`, `Bit: 80`, `Key` = the first 64 frame bits, `Key_2` (the last 16,
big-endian), `Plain_A` (an even sum) or `Plain_B` (odd) — their nine plaintext bytes — and `Serial` (SnA / SnB),
`Btn` (1 Lock, 2 Unlock) and `Cnt` (their seed). A file KAT writes is the file the references write for the same
frame. Flipper-ARF's `Chrysler` name loads too.

Loading reads `Key` and `Key_2` (required) with any `Bit` (the C forces 80), and reads the frame by the rule; the
file's other lines are not applied. A frame whose sum fails loads with `crc_valid` false (the C loads frames whose
check fails too, shown as `Chk: ERR`).

Two quirks of the C are not copied: it writes `Key_2`/`Plain_x` after `flipper_format_rewind`, over the start of
its own buffer (KAT writes them after `Key`, as doctor-firmware's fixed copy does), and its decoder keeps the
plaintext of earlier frames, so one of its files may carry the other payload's `Plain_x` too.

## RF

- **Encoding:** PWM (short HIGH + two long LOWs)
- **RF modulation:** AM
- **Encryption:** none: a 16-bit counter, its low byte XORed into fixed bytes, and a sum
- **Frequencies:** 315 MHz, 433.92 MHz (the recorded fob: 433.89 MHz)

## Not known

All of this rests on one fob, or two that send the same bytes:

- **Whether `FD 3B A1 C3` identify the fob.** `PT_LCK250.sub` holds Lock presses at every count from 3BF8 to 3AE9
  but one, with the same bytes: the counts of the open and close recordings among them, 27 of `PT_300xUNL.sub`'s
  28 and 101 of `PT_LCK100.sub`'s 105. Their presses are spaced as a hand presses a button, not as a generated
  sequence. One fob sends a count once, so either its counter went back between recordings, or a second fob sends
  the same fixed code with a counter in the same range. The 1999 [Volvo S40](volvo_s40.md) remote, whose frame
  follows the same rule, sends other bytes (`0A 8E CB B8`).
- Whether `p7 p8` repeat `p3 p4` in every such fob.
- The codes of other buttons (Trunk, Panic): only Lock and Unlock were recorded.
- **The carrier before the preamble.** Most recorded presses begin with one, from 65 µs to 0.96 s long. A
  receiver's noise and the fob's start-up both fit; KAT, like the references, does not send it.
- What the fob does after count `0000`; KAT goes on from `FFFF`.
- The fob's maker and part number. Flipper-ARF names the FOBIK GQ43VT, a US 315 MHz fob; the recorded one sends
  at 433.89 MHz, and no recording of a GQ43VT fob is in the library.

## Validation

- **The rule:** every one of the 732 frames KAT reads in the six recordings of the PT Cruiser fob
  (`IMPORTS/chrysler`: open and close, ten presses each; `more/`: 28 to 308 presses each) passes the sum, with
  `FD 3B A1 C3`, the button the file name gives, the counter's high byte and `A1 C3`; the counter goes down from
  press to press (test `the_pt_cruiser_recordings`).
- **The encoder:** each press re-encoded with its own button is the fob's next press, all 80 bits, in all 721
  cases where the recording holds it (`reencode_reproduces_the_fobs_next_press`); the open recording's last press
  (Unlock) re-encoded with Lock is the close recording's first (`a_button_change_continues_the_count`); and the
  upload is, pulse for pulse by kind, the recording's second press (`the_upload_is_the_fobs_press`).
- **Against the C** (host builds): ProtoPirate's and Flipper-ARF's decoders read KAT's press as they read the
  fob's own (`FB4725BFA44725BF7984`, SnB 4725BF79, Lock, Cnt 1F, Chk OK), and KAT's key file of it is line for line
  the C's. On the recordings the C's decoder and KAT's read the same frames; the fields differ as described above.
- **The gate:** any one bit of a recorded frame changed fails the sum; a frame of other fixed bytes is read; an ID
  of `00000000` or `FFFFFFFF` is not (`the_rule_reads_back`, `what_the_decoder_reports`). No other recording in
  `IMPORTS` reads as Chrysler V0.
