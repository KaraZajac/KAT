---
layout: default
---

# GM RFA Protocol

**Rust module:** `src/protocols/gm_rfa.rs` (KAT's own decoder, decode only)
**Reference:** none reads this signal. The name is KAT's, after "Remote Function Actuator" in the remotes' FCC
filings.

## Overview

The 315 MHz remotes Omron built for GM (Cadillac, GMC and their sister makes, about 2004 to 2017): an **84-bit
Manchester frame behind 51 zero bits**, a new one every 100 ms while the button is held. A 16-bit fob ID is in the
clear; the 64 bits behind it change from burst to burst. No checksum is known and the cipher is not known:
**decode and display only**.

It is none of KAT's other GM formats: [gm.md](gm.md) is PPM, and [chevrolet_v0.md](chevrolet_v0.md) /
[opel_v1.md](opel_v1.md) read the 433 MHz family's frame, which runs at the same speed.

The decoder was worked out from two recordings: `IMPORTS/cadillac/more/2008-2012_Cadilla_CTS_315mhz_AM650.sub`
(2026-11) and `IMPORTS/gmc/15_gmc_acadia_rs2_l2_u2_t2_p2.sub`.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Half bit | 117.5 µs | 4255 bit/s on both fobs |
| Line code | Manchester | carrier then pause is a 1 |
| Burst | 135 bits, 31.9 ms | 51 zero bits, then the frame |
| Repeats | every 100.3 ms | while the button is held |

On-off keying, 315 MHz.

## FCC filings

| FCC ID | Remote | What the filing shows |
|--------|--------|-----------------------|
| `OUC6000066` | the Cadillac CTS's (Omron "Remote Function Actuator (RFA) Model D6 000066", 2006, "GM Cadillac RFA User Manual"). Parts catalogues list it for Cadillac, Buick and Chevrolet cars from 2006 on, the CTS from 2008 | "approximately 315.0MHz"; one burst of about 32 ms in every 100 ms |
| `OUC6000223`, `OUC60270`, `OUC60221` | other Omron RFA remotes | the 32 ms word every 100 ms; `OUC6000223` gives 120 µs pulses in a 240 µs period |

The FCC ID of the Acadia's remote is not known.

**Recording it.** The signal is 8.5 kBaud, more than twice what the Flipper's AM650 preset filters for
(3.79 kBaud). Both recordings show it: the carrier pulses come out 10 to 50 µs too long and the pauses as much
too short, differently in every burst, so that a long pause and a short carrier pulse can be equally long. The
decoder takes each burst's skew out before it counts half bits (`src/protocols/burst.rs`): a Manchester signal is
HIGH as long as it is LOW, so the mean difference between its HIGH and LOW pulses is twice the skew.

## Frame (84 bits)

| Bits | Field | Notes |
|------|-------|-------|
| 0..3 | Head | `1101` on both fobs |
| 4..19 | Fob | 16 bits, constant per fob (`FA04`, `0178`) |
| 20..83 | Code | 64 bits. Every burst of a press has another one (about half the bits differ), but the last two bursts of a press are alike. No button in the clear |

## Decoder

A frame needs: the burst ending with it (at most two half bits behind it), `1101` behind at least 20 zero bits,
84 bits without one Manchester violation, and the burst coming out at 118 µs per half bit to within an eighth.

## Field mapping

`serial` = the 16 fob bits, `data` = the 64-bit code, `extra` = frame bits 0..19, `data_count_bit` = 84, `button`
None, `counter` None, `crc_valid` false (shown as "-"), `encoder_capable` false. No key file.

## Validation

- Cadillac CTS: 25 of the file's 30 bursts, fob `FA04`, 15 different codes.
- GMC Acadia 2015: 38 of the file's 56 bursts, fob `0178`, 29 different codes.
- The other bursts are broken up beyond what the skew correction mends (the first bursts of a press, mostly).
  Reading the bursts from their rising or falling edges alone gives the same frames and two more.
- No other file in IMPORTS changes.
- Unit tests: a synthetic burst, one with 45 µs of skew, the gate (19 zeros, another head, a burst too long or too
  short, the other polarity), the two recordings.
