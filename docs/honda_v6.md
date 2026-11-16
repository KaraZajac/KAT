---
layout: default
---

# Honda V6 Protocol

**Rust module:** `src/protocols/honda_v6.rs` (KAT's own decoder and encoder)
**Reference:** none reads this signal. The name is FoxFW's: its `honda_v6.c` takes the same frames on its "313"
path, but as "trits" (below). FoxFW's three paths are ported in `src/protocols/honda_v6_foxfw.rs` and stay
unregistered.

## Overview

Honda / Acura remotes that send one **64-bit encrypted block**, Miller-coded, twice in a frame. Nothing in the
block is in the clear: KAT reports the block, and no serial, button or counter. The decoder was worked out from
the captures in `IMPORTS/honda/more/` (2026-11): an OEM fob at 433.92 MHz, an OEM fob at 313.85 MHz, a KeyDIY
clone and one more aftermarket remote.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Bit cell | 800 µs | 1250 bit/s |
| Line code | Miller (delay modulation) | a 1 = a transition in the middle of its cell; a 0 = none; two 0s in a row = a transition between them |
| Pulse widths | 800 / 1200 / 1600 µs | one, one and a half, two cells (± 170 µs in the decoder) |
| Sync | 2400 µs + 2400 µs | starts on a cell boundary (± 300 µs) |
| Half cell | 400 µs | only in front of a sync, after a block that ends in a 1 |

2-FSK (Flipper preset FM476 in the captures), 433.92 MHz and 313.85 MHz. Only the transitions carry data, so the
levels do not matter: the sync starts on either level, and which tone reads as HIGH is the receiver's choice.

## Frame

One press, about 290 ms:

| Part | Content |
|------|---------|
| Lead-in | about 9.5 ms at one level |
| Preamble | 205 zeros on the OEM 433.92 MHz fob (204 pulses of 800 µs), about 100 at 313.85 MHz |
| Start | `1 0 1` |
| Block | 64 bits |
| Sync | 2400 µs, 2400 µs |
| Second copy | `0 0 0`, `1 0 1`, the same 64 bits, sync |
| End | one zero, about 4070 µs at one level, carrier off |

The KeyDIY clone sends the same frame with another ending; the other aftermarket remote sends the block three
times.

## The block

Over 900 presses of one fob (300 each of Lock, Unlock and Trunk) every one of the 64 bits is 1 about half of the
time, and no two blocks are alike. The whole block is encrypted; the cipher is not known. A held button sends new
frames (42 over the 20 holds of `HondaH1_OEM2_20lck_hold.sub`).

## Decoder Steps

Pulse widths only (levels are ignored):

1. At least 40 pulses of 800 µs: zeros.
2. A pulse of 1200 µs starts the block (the last zero and half of the leading 1). From there every pulse of
   800 / 1200 / 1600 µs adds one or two cells. The cells are counted from the first 1.
3. A sync ends the block. It is taken when the block is exactly 67 cells and starts `1 0 1`. A block that ends
   in a 1 has the 400 µs pulse in front of the sync.
4. Behind a sync, 1 to 8 pulses of 800 µs lead to the next block.
5. A frame is reported when a block equals the one before it, on the first pulse of the sync behind it.

That equality is the check (`crc_valid`): a wrong bit in either copy, a Miller violation, a block of another
length, and nothing is reported.

## Field mapping

`data` = the 64-bit block, `data_count_bit` = 64. No serial, button or counter. Line coding in the list:
"Miller".

## Encoder

The frame above, rebuilt from the block with the OEM fob's timing (9500 µs lead-in, 205 zeros, both copies, the
4070 µs ending), once. It sends the **captured block again as it is**: without the cipher there is no next code to
compute, and a receiver that has already seen the block may refuse it. Lock / Unlock / Trunk / Panic all send the
same frame.

## Key files

None. FoxFW's "Honda V6" key file holds the first 32 pulse widths of a frame, which is less than the block.

## FoxFW's reading

`honda_v6.c`, variant 2 ("313"): the pulse widths behind the preamble as trits (800 → 0, 1200 → 1, 1600 → 2), two
bits each, at most 60, the first 32 in its `Key`; "serial" = the first 16. Its first trits are always 1, 2 (the
`1 0 1` start). Because the widths depend on the block's bits, every frame has another "serial" and another length
(44 to 56 trits): on `HondaH1_OEM2_300lck.sub` FoxFW's C lists 297 frames with 297 serials. The port
(`honda_v6_foxfw.rs`) gives the same frames as the C; its other two paths ("stock45", "pandora") read noise in the
corpus (five frames with five serials in a capture that is Continental-KR5V2X), and the module is not registered.

Every Flipper reference also lists a **Kia V5** frame with key `5555555555555555` in these captures: this
protocol's preamble is a square wave of 800 µs pulses, which Kia V5 reads as its own preamble and then as 67 bits.
KAT's Kia V5 drops that key ([kia_v5.md](kia_v5.md)).

## Validation

Captures (`IMPORTS/honda/more/`; the last one in `IMPORTS/honda/`), every frame read, block for block the same as a separate Python model of the
frame:

| Capture | Remote | Frames |
|---------|--------|--------|
| `HondaH1_OEM2_300lck.sub`, `…300ulk.sub`, `…300trk.sub` | OEM, 433.92 MHz | 300 each, all distinct |
| `HondaH1_OEM2_20lck_hold.sub` | OEM, 20 held presses | 42 |
| `Honda_313_lock_long_20.sub`, `Honda_313_unlock_long_20.sub` | OEM, 313.85 MHz | 40 each |
| `Honda_V1_kd_5_5_5.sub` | KeyDIY clone | 30 |
| `Honda_V1_Ferrari_10_10.sub` | aftermarket, three copies per frame | 11 |

Unit tests: the encoder's frame against the fob's pulse layout, decode at either polarity and with skewed pulse
widths, the report on the second copy's sync, rejection of damaged frames (differing copies, short preamble,
foreign width, half a sync, one copy), three copies as one frame.

## Vulnerabilities

**None identified.** A recorded frame can be sent again (Replay, or the encoder), which a rolling scheme refuses.
