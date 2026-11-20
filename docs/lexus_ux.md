---
layout: default
---

# Lexus UX Protocol

**Rust module:** `src/protocols/lexus_ux.rs` (KAT's own decoder, decode only)
**Reference:** none reads this signal as the recordings hold it. FoxFW's [Lexus V0](lexus_v0.md) reads the first
224 bits of the same frame as they are sent, without the step that turns them into the frame's bits (below). The
name is KAT's.

## Overview

The smart key of a 2021 Lexus UX 250, the European model: two-tone FSK, a **256-bit frame sent four times back to
back** with every press. The frame carries the key's ID and the button in the clear, 128 bits that change with
every press, and a **CRC-16**. The cipher behind the 128 bits is not known: **decode and display only**.

Worked out from two recordings of one key, five presses each: `IMPORTS/Toyota + Lexus/Lexus UX 250 2021 lock.sub`
and `IMPORTS/Toyota + Lexus/Lexus UX 250 2021 unlock.sub` (433.66 MHz, Flipper preset FM476). Until this decoder,
nothing read them ([Unread recordings](unidentified_captures.md)).

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Half bit | 348.4 µs | 1435 bit/s; a press of 2048 half bits lasts 713.6 ms |
| Line code | Differential Manchester | a change of tone in the middle of every bit; no change at the bit's start is a 1, a change a 0 |
| Frame | 256 bits, 178.4 ms | |
| Press | four frames, no gap | 713.6 ms |

**The line code.** Read as Manchester, every bit of a burst has its change of tone in the middle (no violation in
2048 half bits). The frame's bits are those Manchester bits each XORed with the one before. So the frame does not
depend on which tone is which, and the decoder reads either.

**The skew.** In these recordings one tone's pulses come out about 120 µs short and the other's 120 µs long: a
half bit is 226 µs of one tone and 470 µs of the other on average, two half bits 575 and 819 µs. The decoder
measures the skew over each burst and takes it out before counting half bits (`src/protocols/burst.rs`).

## FCC filings

The recorded key is the European 433 MHz model, which has no FCC filing. The parts catalogues give `HYQ14FBZ` for
the US UX of 2020 to 2024, and for Lexus ES, GX, LC and LS models of 2020 to 2025:

| FCC ID | Remote | What the filing shows |
|--------|--------|-----------------------|
| `HYQ14FBZ` | DENSO "Electronic Key" `14FBZ`, 2019; four buttons, variants with three and two | 314.35 or 312.10 MHz (never both at once), FSK (F1D); a press transmits for 0.7333 s, the four frames of this recording (0.7136 s) and 20 ms more; a −20 dB bandwidth of 37 kHz on each channel; 18.37 MHz crystal; a 134.2 kHz receiver for the car's call |

The DENSO keys `HYQ14FBF`, `HYQ14FBX` and `HYQ14FBN` transmit 0.72 to 0.73 s a press too. Whether these US keys
send this frame is not known: the US Toyota and Lexus recordings in `IMPORTS/` were made with AM presets or FM238
and hold nothing KAT reads ([Recording](recording.md)).

## Frame (256 bits)

| Bits | Field | Notes |
|------|-------|-------|
| 0..7 | Lead | 8 zeros |
| 8, 16, …, 248 | `1` | every byte from byte 1 on begins with a 1, the CRC's bytes too |
| 8..47 | Head | `C4 80 80 88 80` in every frame |
| 48..52 | Button | `1` and four bits: `0010` Lock, `0100` Unlock |
| 53..198 | Code | 128 bits without the bytes' first bits; changes with every press |
| 199 | `0` | in every frame |
| 200..231 | ID | `AF A6 9C 8B` in every frame of the one key |
| 232..235 | `1010` | in every frame |
| 236..253 | CRC | 16 bits, around the 1s at 240 and 248 |
| 254..255 | `11` | in every frame |

The first lock press, as bytes: `00 C4 80 80 88 80 95 F5 B1 A1 C7 F0 FA 83 E3 BE BB CE B2 F1 88 B4 8C C9 B0 AF A6
9C 8B A9 B0 A7`.

**The check.** CRC-16 with the CCITT polynomial `0x1021` (`x^16 + x^12 + x^5 + 1`), register 0 at the start, no
reflection, nothing XORed at the end (the parameters of CRC-16/XMODEM), over bits 10 to 235, the bytes' first bits
included. Its 16 bits, highest first, stand at 236–239, 241–247 and 249–253. It holds in all ten frames.

How it was found: the difference of two frames cancels a CRC's start value and anything XORed at its end, so each
polynomial was tested on the differences of the ten frames. The search took every polynomial of 4, 5, 7, 8, 12, 15
and 16 bits; the frame's bits as decoded, as sent, and with each byte inverted where its first bit is 0; with and
without the bytes' first bits, in either order, ending at bit 199, 200, 232 or 236; against the 16 free bits at the
end, 15 of them and pieces of them. One fit: of the 32768 polynomials of 16 bits only `0x1021`, over the bits as
decoded up to bit 235 with the bytes' first bits, against the 16 free bits. With it the register starts at 0 from
bit 10 (bits 10 to 13 are 0, so from any of them). The other hit of the search is the same check read as sent: the
15-bit polynomial `0x701F`, which is `0x1021` divided by `x + 1`.

That factor `x + 1` gives every frame an even number of 1 bits, so the tones end as they began and the four copies
of a press are sent alike.

**The code.** 128 bits: a block of a 128-bit cipher such as AES would fill it, but nothing shows which cipher, if
any. Over the ten presses no bit of it stays, between 50 and 72 of its bits are 1, the ten codes are linearly
independent, and two presses in a row differ in 58 to 69 bits. There is no counter in the clear.

## The frame as sent: Lexus V0

Before the XOR the frame begins `00 87 00 FF 0F 00`, and the ID reads `CA C4 E8 F2`, or `35 3B 17 0D` when the bits
before it hold an odd number of 1s: in four of the ten presses. FoxFW's [Lexus V0](lexus_v0.md) (from KeeLoq-RX's
`arke_toyota.py`) reads exactly that: Manchester at 350/700 µs behind 16 short pulses, 224 bits from the first long
pulse on, which are bits 8 to 231 as sent. Its 32-bit "header" is `87 00 FF 0F`, its 160-bit "rolling code" bytes
5 to 24, its "serial" the ID as sent, and the "complement signature" it checks (the serial alternating between S
and its complement) is the differential line code. It stops before the CRC.

KAT's Lexus V0 reads nothing in these recordings: it is flagged AM, and its 16 short pulses must be 250 to 470 µs
each, where the recordings' half bits of one tone are 226 µs on average. On frames without the skew it reads the
first 224 bits of a press's second, third and fourth copies, with the ID as sent; the first copy has at most 15
short pulses in front, one fewer than it wants (a unit test shows both). The two never meet on an FM recording or
on KAT's FM receive path: Lexus UX is flagged FM, and a decode on the modulation-gated pass settles a burst.

## Decoder

The decoder collects the pulses of a burst, each 0.4 to 2.9 half bits long, and reads it when a pulse of another
length (or the end of the input) ends it. It takes the skew out and counts half bits (`src/protocols/burst.rs`),
reads Manchester bits at both phases, XORs each with the one before, and looks for a frame at every position:

1. bits 1 to 47: seven zeros and the head (bit 0 is not looked at: at a burst's start there is no bit before it);
2. a 1 at every byte's start;
3. the CRC;
4. the closing `11`.

One whole copy is enough; a press gives one decode however many copies it holds. With the CRC, the registry places
Lexus UX among KAT's own decoders with a check.

## Field mapping

`serial` = bits 200..231 (`AFA69C8B`), `button` = bits 49..52 (`2` Lock, `4` Unlock), `data` = the code's first 64
bits, `extra` = its last 64, `data_count_bit` = 256, `counter` None, `crc_valid` true, `encoder_capable` false. The
head, the bytes' first bits and the CRC follow from these; bit 199 is not kept. No key file: no firmware has a
format for this frame.

## Not known

- The cipher, and whether the code holds a counter.
- The other buttons' codes, and so which of bits 49 to 52 the button field is: `0010` and `0100` fit a one-hot
  field and a 3-bit number with a `0` behind it alike.
- Whether bit 199 is filler or a flag that was off: the head, the button and the code are 167 bits, one short of
  the 24 groups of seven bits behind the bytes' 1s from bit 9 to 199. What the head holds.
- Whether the ID is the key's alone: one key was recorded.
- The key's frequency: the recordings are at 433.66 MHz, and the skew may come from the key's tones not sitting
  even about it.
- Whether the US keys (`HYQ14FBZ`, 312.10 / 314.35 MHz) send this frame.

## Validation

| Press | Button | Code (`data`, `extra`) | CRC | Whole copies |
|-------|--------|------------------------|-----|--------------|
| lock 1 | `0010` | `BD58A18FC3D03C6F` `9DCE65C443419258` | `9609` | 4 |
| lock 2 | `0010` | `2D204D0C780AF416` `4AA4295913242547` | `3CBB` | 4 |
| lock 3 | `0010` | `A0B91E9871656046` `6D56182781C0F860` | `E584` | 4 |
| lock 4 | `0010` | `A0D17D8C8EC8F270` `325B9F66D48018CF` | `3E88` | 3 |
| lock 5 | `0010` | `82FF4D0AD5E58BCB` `1BB7CB387B9E7700` | `215C` | 3 |
| unlock 1 | `0100` | `0AD01C90911F5FFC` `5348971838F251F1` | `E6BE` | 4 |
| unlock 2 | `0100` | `2A7B876C637380EE` `BF0FFF4A09824299` | `61BE` | 3 |
| unlock 3 | `0100` | `6920C622F2A1D555` `6C205C2E7AA225EE` | `7484` | 3 |
| unlock 4 | `0100` | `BB993317BC85751B` `9BEAB7EA71827B66` | `01BA` | 4 |
| unlock 5 | `0100` | `E06E6E6000CE1A1C` `F4ECAD2020CA5DF0` | `C498` | 3 |

- Both files: five presses each, all read, the ID `AFA69C8B` in every one. Where a press has three whole copies,
  noise covers the start of its first (or the recording ends in its fourth: unlock 5).
- No other file in `IMPORTS/` changes, the `more/` folders included; the decoder alone, at either polarity, reads
  nothing in the other 437 RAW recordings.
- Unit tests: a press at either polarity and with the recordings' skew either way round; one copy, and a copy the
  end of the input ends; the gate (the lead, the head, a byte's first bit, the code, the ID and the `1010` through
  the CRC, the CRC, the closing `11`, a pulse of another length, another speed); a damaged copy among good ones;
  the CRC against a recorded frame and the CRC-16/XMODEM check value; Lexus V0 on the same frame; the two
  recordings, alone and through the registry.
- The lock recording's first two presses, sent as 2-FSK at 10 dB SNR through the model of the air and KAT's own
  receive pipeline (`src/radio/sim.rs`, `imports_real_timings_decode`): both read, skew and all.
