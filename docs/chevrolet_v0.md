---
layout: default
---

# Chevrolet V0 Protocol

**Rust module:** `src/protocols/chevrolet_v0.rs`
**Reference:** `FoxFW lib/subghz/protocols/chevrolet_v0.c` (a port of the KeeLoq-RX `arke_mazda_gm.py` classes
`ChevroletV0` / `ChevroletV0Gm`); FoxFW is the only copy in REFERENCES.

## Overview

The GM 433.92 MHz keyfob family (Chevrolet, Buick, GMC, Holden; 2016 to 2024 in the captures). OOK Manchester
at 120/240 µs, an exact 32-bit preamble word and 96 payload bits. FoxFW reads the fobs whose nibble behind the
serial is `0xA` or `0xB` ("Open", "Close"); KAT reads every fob of the family (below). The hop is encrypted and there is no clear counter, so this is
**decode and display only**, as the reference (its encoder is NULL). It is a different air format from [gm.md](gm.md)
(PPM, rtl_433's `gm_rke` / ARF's GM).

## Timing

| Parameter | Value  | Notes                                            |
|-----------|--------|--------------------------------------------------|
| Short     | 120 µs | `diff < 71` (the C's `te_delta`)                 |
| Long      | 240 µs | `diff < 71`                                      |
| Preamble  | `0xAAAAAAAB` | exact, from the 32nd decoded bit on        |
| Payload   | 96 bits | every Manchester bit inverted (IEEE 802.3)      |

## Frame (96 bits)

| Bits  | Field  | Notes                                                                  |
|-------|--------|------------------------------------------------------------------------|
| 95..72 | Serial | 24 bits                                                              |
| 71..68 | Button | FoxFW: `0xA` Open, `0xB` Close, anything else drops the frame. KAT: any value (below) |
| 67..32 | Hop    | 36 bits, encrypted                                                   |
| 31..0  | Seed   | 32 bits, encrypted                                                   |

The top 64 bits (serial, button, hop) are the C's `gdata`. A `gdata` of 0 or all ones drops the frame.

### The nibble is not a button

The C has an option, `CHEVROLET_V0_GM_ACCEPT_ANY`, "to generalize to the whole GM 434 MHz remote family
(Buick/Cadillac/GMC/newer Chevrolet), whose fobs carry a per-remote type/toggle nibble instead of Open/Close". It
is off in FoxFW and **on in KAT**: any nibble is taken.

What the nibble is shows next to [opel_v1.md](opel_v1.md), which reads the same frame as `00010`, a 24-bit serial
and a 64-bit block. Chevrolet V0 sees that frame from its third bit on, and inverted:

| Chevrolet V0 | The frame as Opel V1 frames it |
|---|---|
| Serial (24) | `010` and the serial's first 21 bits, complemented (so it always begins `101`) |
| Button (4) | the serial's last 3 bits and the block's first bit, complemented |
| Hop (36), Seed (32) | the block's other 63 bits, complemented, and the 5 bits that follow the frame |

So three bits of the nibble are the fob's and the fourth changes from frame to frame; none of it follows the
button pressed (in the Trail Blazer capture the nibble is `A` or `B` whether Lock, Unlock, Remote start, Panic or
Trunk is pressed). With the nibble gate, only fobs whose serial ends in `010` are read: one in eight.

The block does not open with Opel V1's XTEA key: of the 285 different frames of the six fobs below, none has
its tail byte. (The one frame Opel V1 lists in the Holden capture comes from a recording in which Chevrolet V0
reads 167 different frames: an 8-bit check passes that often by chance, and its button and counter are no fob's
values.)

## The second reading (KAT)

The frame is 8.3 kBaud, more than twice what a Flipper's AM650 preset filters for (3.79 kBaud). A recording can
have its carrier pulses 30 to 40 µs too long and its pauses as much too short; a long pause is then as long as a
short carrier pulse, and the C's fixed windows (`diff < 71` around 120 and 240 µs) lose the frame. Of the 40
bursts of the 2020 GMC Acadia capture the C's reading takes one.

KAT therefore reads a burst the C's reading dropped once more, whole (`src/protocols/burst.rs`): it takes the
burst's skew out (a Manchester signal is HIGH as long as it is LOW, so the mean difference between its HIGH and
LOW pulses is twice the skew), then wants what the C wants, the 32-bit preamble word and 96 bits, and one thing
more: **the burst ending 3 bits behind the 96**. A burst is the preamble and 101 bits (the 93 Opel V1 frames and 8
more, which change with every frame and are no CRC-8, sum or XOR of it; the C reads 96 from the third on).
Without a checksum, that length is what catches a bit lost or gained inside the frame: a first version without
it read one frame in 30 of the Acadia capture with another serial. A burst the C's reading takes is not read
twice.

The skew correction sorts every pulse into one half bit or two, so a slower remote's pulses, sorted into both,
would pass. A burst is therefore dropped when its one-half pulses, or its two-half pulses, are not that long on
average (to a quarter of a half bit; from three pulses of a length on).

## A preamble is no frame (KAT)

The preamble word is alternating bits and one pair alike, and a pulse of 191 to 310 µs is this decoder's long
pulse. A long run of such pulses with a few shorter ones among them is therefore, to the C, a preamble and 96
bits, and with the nibble `A`, which alternating bits give half the time, FoxFW's own gate passes the "frame".
A PSA remote whose half bit is 218 µs sends about 170 such pulses before every frame: two of ten presses of
`IMPORTS/GROUPE PSA/Delphi_remote_unlock_x10.sub` were listed as Chevrolet V0 frames (`555555556AAB5555` and
`5555555AAAAAD555`), and since the list of decoders starts over after a frame, [PSA](psa.md) lost those two
presses.

KAT does not report a frame whose 96 bits all but alternate: fewer than 16 of its 95 pairs of neighbouring bits
alike. A frame has about half of them alike (47, give or take 5); of the 519 frames this decoder reads from the
seven captures below, none has fewer than 31.

## Decoder state machine

1. **Reset step:** any pulse that is short or long starts the preamble (Manchester state `Mid1`, register cleared)
   and is processed as its first event; other pulses are ignored.
2. **Preamble:** each pulse is a Manchester event (`level ? ShortLow : ShortHigh`, long likewise); a pulse that is
   neither short nor long returns to the reset step. Each decoded bit is inverted and shifted into a 32-bit
   register; from the 32nd bit on, the register equal to `0xAAAAAAAB` starts the payload.
3. **Data:** each bit shifts through a 64-bit word into the register (bits 95..64 end in the register, 63..0 in the
   word). A bad pulse drops the frame. After 96 bits the frame is checked (gdata, button) and reported; the
   decoder returns to the reset step.

No checksum exists: the gate is structural, so the registry places Chevrolet V0 after every decoder with a check
(after the gate remotes, before the rtl_433 ports).

## Field mapping

`data` = `gdata` (64 bits), `data_count_bit` = 96, `serial` = 24 bits, `button` = the nibble, `extra` = the seed,
`counter` None, `crc_valid` false (shown as "-"), `encoder_capable` false.

## Key file

`Protocol: Chevrolet V0`, `Bit: 96`, `Key` = `gdata`, then `Serial`, `Btn`, `HopHi` (the hop's 4 high bits),
`HopLo` (its low 32 bits), `Seed` (`subghz_protocol_decoder_chevrolet_v0_serialize` / `_deserialize`; loading needs
`Bit: 96` and reads `Seed` when present).

## Validation

- Three deviations from the C as FoxFW builds it: its `CHEVROLET_V0_GM_ACCEPT_ANY` is on (above), a burst its
  reading drops is read again whole (below), and a frame of all but alternating bits is not reported (above).
- With it, five more captures decode, each with one serial and a nibble whose last bit alone changes:
  `chevy/18_chevy_malibu_…` (36 frames, serial `A8936E`, nibble 8/9), `buick/24_buick_encore_…` (34, `AA67D0`,
  4/5), `buick/24_buick_envista_…` (38, `B6EC9C`, C/D), `HOLDEN/Holden Barina 2016 AU.sub` (358, `B61B30`, C/D)
  and `gmc/20_gmc_acadia_…` (one frame, `BBC6E9`). No other file in IMPORTS changes.
- The second reading adds: the Acadia 25 frames instead of 1, the Trail Blazer 22 instead of 21, and one frame
  of `buick/15_buick_lacross_…` (a 2015 LaCrosse at 315 MHz, serial `AD1053`; the rest of that capture is
  broken up beyond it). No other file in IMPORTS changes.
- Real capture: `IMPORTS/chevy/22_chevy_trail_blazer_l2_u2_rs_p_t.sub` (433.92 MHz, OOK270): 21 frames with the
  constant serial `0xB809C5`, button `0xA` or `0xB`, identical to the host build of FoxFW's C frame for frame.
  Before this decoder KAT read none of it. FoxFW reads none of the other GM captures in IMPORTS (their nibbles
  are not `A`/`B`).
- Unit tests: a synthetic frame, the gates (zero and all-ones payloads, an out-of-tolerance pulse), key-file round
  trip, the real captures; the second reading on a burst with 55 µs of skew (which the C's reading loses), its
  length check (2, 3 or 4 bits behind the 96), a cut preamble, two bursts, the end of the input; the two
  preamble "frames" of the PSA recording, the fewest pairs alike in the real frames (31), and no frame from the
  three PSA recordings.
