---
layout: default
---

# Kia V0 Protocol (Kia V0 / Suzuki V0 / Honda V0 / Mitsubishi V0)

**Rust module:** `src/protocols/kia_v0.rs` (a port; Honda V0's frame and every type's press follow the recordings, where they differ from the references)  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v0.c` (also doctor-firmware `kia_v0.c`; Flipper-ARF
`kia_v0.c` has the first three types). Its Kia type is the official Flipper firmware's "KIA" decoder of 2021;
Honda V0 is the remote rtl_433 decodes as `nidec_car_remote.c` ([Sources](#sources)).

## Overview

One PWM air format at 250/500 µs carrying four frame types, told apart by bit count, check and preamble.
ProtoPirate folded its standalone `suzuki.c` (2026-04-17) and `mitsubishi_v0.c` (2026-09-14) into this
decoder, and KAT does the same: the former standalone **Suzuki** decoder is retired (its name still
resolves to Kia V0 for TX), and the type is shown as the display name.

| Type | Display name | Bits | Check | Frame end |
|------|--------------|------|-------|-----------|
| 1 | Kia V0 | 61 | CRC-8 (poly 0x7F, init 0) over bits 8–55 | HIGH 700–1700 µs |
| 2 | Suzuki V0 | 64 | the same CRC-8 over counter/serial/button | LOW then HIGH 2000–2500 µs |
| 3 | Honda V0 | 72 (label; 64-bit key) | CRC-8 (poly 0x21, init 0) over counter and button | the 56th bit |
| 4 | Mitsubishi V0 | 61 | as Kia | as Kia |

None of the four is encrypted: each frame carries a 16-bit counter that goes up by one with every frame sent,
in the clear, and a CRC. KAT reads and builds all four as the recorded remotes send them. For Kia, Suzuki and
Mitsubishi that is the references' frame; for Honda V0 it is not ([below](#where-kat-departs-from-the-references)).

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short (0) | 250 µs | HIGH and LOW of a bit are equal; ±100 µs, strict `<` |
| Long (1)  | 500 µs | ±100 µs |
| Preamble  | > 14 short pairs | the first long pair is the sync and adds two `1` bits |
| Kia gap   | 700–1700 µs HIGH | ends Kia / Mitsubishi frames |
| Suzuki gap | 2000–2500 µs LOW then HIGH | ends Suzuki frames |

## Frame layouts

- **Kia / Mitsubishi** (`data` as decoded, bit 60 set by the sync): counter = bits 40–55, serial = bits
  12–39 (28), button = bits 8–11, CRC = bits 0–7.
- **Suzuki**: `0xF << 60 | counter16 << 44 | serial28 << 16 | button4 << 12 | crc8 << 4`.
- **Honda**: after the 4 long sync pairs, 56 bits MSB first:

  | Bits | Field |
  |------|-------|
  | 16 | counter, one higher with every frame |
  | 24 | the remote's id |
  | 4 | `5`, in every frame of every remote recorded |
  | 4 | button: 3 Lock, 4 Unlock, 5 a trunk or panic press, F the trunk's long press, 6 the panic's |
  | 8 | CRC-8, polynomial 0x21, init 0, no reflection, over the 20 bits of counter and button |

  The id and the `5` are not under the check. KAT keeps, as `data`, the references' 64-bit key: `F0`, then
  these 56 bits with every byte bit-reversed (what the key files carry).
- **Mitsubishi**: a Kia frame sent twice, the second copy behind an 80-pair preamble. The decoder remembers the
  previous Kia frame; an identical frame behind 72–88 preamble pairs is Mitsubishi V0 ([Mitsubishi V0](mitsubishi_v0.md)).

## A press

From the recordings (pulse counts as received; the receiver cuts the start of each preamble):

| Type | Press |
|------|-------|
| Kia | a carrier of 3.7–4.3 ms, 313–336 short pairs as received, the frame, HIGH 1.5 ms and LOW 1.5 ms, 16 short pairs, the frame again (in `kia_seed_raw.sub` a third time), HIGH 1.5 ms |
| Suzuki | 313–318 short pairs, the frame, LOW 2.2 ms, HIGH 2.2 ms, LOW 250 µs, 15 short pairs, the frame again |
| Mitsubishi | a carrier of about 2.5 ms, 320 short pairs, the frame, HIGH 1.5 / LOW 1.5 ms, 80 short pairs, the frame again, HIGH 1.5 / LOW 1.5 ms |
| Honda | 319–325 short pairs and the 4 long pairs (Omron's "324 bit codes" of wake-up), the 56 bits, a closing HIGH of 250 or 500 µs (the remote's make); about 205 ms. Lock and Unlock send one frame. A trunk press sends 5, and its long press F one count higher, about 300 ms later; panic, by rtl_433's description, 5 and then 6 |

Some Honda remotes send 8 more bits after Lock's frame (`04` on rtl_433's OEM remote, `05` on the 300-press set),
and the Xhorse-made one `00` after every frame; no decoder needs them.

## Where KAT departs from the references

The references are ProtoPirate's `kia_v0.c` and its copies. In order of weight:

- **Honda V0's frame.** The C reads the 56 bits as its key's bytes and so reports the id bit-reversed byte by byte
  (`BD661F` for `BD66F8`), and numbers the buttons from the reversed button byte: a Lock is its "Unlock3", a
  Trunk "Trunk2" or "Trunk3", a Panic "Lock2". Its check is a table that equals the CRC above for buttons 3, 4,
  5 and F and not for 6. KAT shows the frame's own fields and checks the CRC.
- **Honda V0's transmit.** The C sends Lock as button 6 (Panic's long press) and Trunk as 2 (a code no remote
  sends), with checks of its table, behind a 40-pair preamble, three times. KAT sends Lock 3, Unlock 4, Trunk 5
  then F, Panic 5 then 6, with the CRC, behind the remote's 320 short pairs, once.
- **Honda frames that pass the Kia CRC.** The decoder reads 61 bits for every type; one Honda frame in 256 also
  passes the Kia CRC, and the C then waits for a Kia gap that never comes and loses the frame. KAT keeps it
  until the next HIGH: a Kia gap makes it Kia V0, anything else Honda V0. Five such frames are in the
  recordings (`HondaV0_BD661FCA_300lck.sub` 1, `_300ulk.sub` 2, `HondaV0_FA94E9CA_300trk.sub` 2).
- **One press per send.** The C's TX loop sends its upload three times, a long preamble each time. KAT sends the
  press the remote sends, once: Kia's closing HIGH is 1.5 ms (the C: 1.0), Mitsubishi's second preamble 80 pairs
  (the C's 80 plus the short pair of the frame's cleared bit 60 make 81).
- **Mitsubishi buttons.** The C gives Mitsubishi Kia's names and codes. The Pajero recorded sends 3, 4 and 6,
  never 1 or 2, and no source says which is which: KAT shows the numbers and keeps the captured button for every
  request.
- **Send next code** keeps the captured button. The types' own codes overlap KAT's generic ones (0x01 Lock, 0x02
  Unlock, 0x04 Trunk, 0x08 Panic): Suzuki's 4 is Unlock, the generic 0x04 Trunk. Up to 1.20.1 "Send next code"
  on a Suzuki Panic, Trunk or Unlock capture, a Kia Panic capture or a Honda Unlock capture sent another
  button.

## Decoder Steps (`subghz_protocol_decoder_kia_feed`)

1. **Reset** — a short HIGH starts the preamble.
2. **Preamble** — count short/short pairs; a long/long pair after more than 14 starts the data with `11`.
3. **SaveDuration** (HIGH) — a Kia gap ends the frame (61 bits: CRC → Kia/Mitsubishi, else a valid Honda
   key → Honda); a Honda frame that also passed the Kia CRC is reported on any other HIGH; a Suzuki HIGH after a
   Suzuki LOW ends a 64-bit Suzuki frame (CRC checked on the bits as received or shifted left by one);
   otherwise remember the HIGH.
4. **CheckDuration** (LOW) — short/short = 0, long/long = 1; after each bit a 61-bit frame failing the Kia
   CRC is tried as a Honda key (the `5`) and reported at once; a Suzuki LOW is remembered.

KAT's registry resets every decoder after any decode; Kia V0's `reset()` therefore keeps the previous-frame
memory so the Mitsubishi rule works as in the C. Live captures and `.sub` imports list each distinct decode
of a burst, as ProtoPirate's history lists every decoded frame: a Mitsubishi transmission gives Kia V0, then
Mitsubishi V0.

## Encoder

The remote's next press, sent once; the counter + 1 for each frame sent.

- **Kia:** HIGH 750, LOW 750, 319 short pairs, frame (its bit 60, 0, makes the 320th short pair), HIGH 1500,
  LOW 1500, 15 short pairs (16 with bit 60), frame, HIGH 1500.
- **Mitsubishi:** as Kia with a 79-pair second preamble (80 with bit 60) and a closing HIGH 1500 + LOW 1500.
- **Suzuki:** 320 short pairs, frame, LOW 2000, HIGH 2000, LOW 250, 15 short pairs, frame.
- **Honda:** 320 short pairs, 4 long pairs, the 56 bits, HIGH 500; for Trunk and Panic two such frames, 100 ms
  apart.

Buttons: "Send next code" keeps the captured one. TX Lock, Unlock, Trunk and Panic map to Kia 1, 2, 3, 4 and
Suzuki 3, 4, 2, 1 (the C's names, which no recording confirms), Honda 3, 4, 5 + F, 5 + 6 (rtl_433's names), and
Mitsubishi to the captured button. The type comes from the bit count (61 Kia, 64 Suzuki, 72 Honda); Mitsubishi
V0 captures are encoded through the "Mitsubishi V0" registry entry.

The KAT-made presses are not tested on a car. The lead-in differs from the recordings (KAT and the C send 750 µs
of carrier and 750 µs of silence before a Kia or Mitsubishi preamble, the recorded fobs a carrier of 2.5–4 ms),
which the 160 ms preamble behind it makes immaterial to a receiver as far as is known.

## Key files

ProtoPirate's lines (`subghz_protocol_decoder_kia_serialize`): Protocol `Kia V0` for every type, Bit 61 / 64 /
72, Key = the frame, then Serial, Btn, Cnt and Type (1 Kia, 2 Suzuki, 3 Honda, 4 Mitsubishi), decimal. For Honda
V0, Serial and Btn are the C's (the id bytes bit-reversed, its button numbers): the firmwares read them back when
they send from a file, and a file KAT writes is the one they write for the same frame.
Loading refuses other bit counts and takes the type from `Type`, else from a legacy Protocol name (`Suzuki`,
`Honda V0`, `Mitsu V0` …), else from the bit count (the Flipper-ARF `KIA/HYU V0` files in IMPORTS have no
`Type`: 61 bits, Kia).
When the file has Serial, Btn or Cnt the C rebuilds the frame from them (CRC recomputed); KAT does that only
when they differ from the key's own fields, so a key it wrote reloads bit for bit (the C's rebuild also clears
the sync bit the decoder leaves in bit 60 of a Kia frame: same fields, same frame on air).

## RF

- **Encoding:** PWM, equal-length HIGH/LOW per bit
- **RF modulation:** FM (ProtoPirate; Omron's filings for the Honda and Mitsubishi remotes: FSK); Flipper-ARF also flags AM
- **Frequencies:** 315 MHz, 433.92 MHz (the recordings: 433.92 MHz; the Xhorse Honda remote 313.85 MHz)

## Sources

- **Kia:** the official Flipper Zero firmware's "KIA" decoder, decode only, by Skorpionm: commit `4418e73b26`
  (2021-10-10, PR #746), named "KIA Seed" since `3164184bbc` (2022-03-03), still in the firmware. Its test
  recording (`8cc3fd579c`, 2022-04-14) is byte for byte `IMPORTS/KIA/more/kia_seed_raw.sub`, and its comment's
  example frames are the same remote's. The comment says `start_crc 0x08`; the frames check with init 0.
  ProtoPirate's first `kia_v0.c` (`ccaeeee`, 2025-12-06) is that file renamed.
- **Honda:** rtl_433's `hondaremote.c` (2016) and `nidec_car_remote.c` (2023–2026, OUCG8D-344H-A): the
  counter, the 24-bit id, the `5`, the button and its names. `nidec_car_remote.c` calls the last field "not a
  conventional checksum" after testing byte-aligned spans; the CRC over the 20 bits of counter and button
  checks its own OEM recordings (6 of 6). Omron's FCC operational descriptions of its G8D remotes: G8D-380H-A and
  G8D-387H-A, FSK, "The synchronous frame has 324 bit codes that it will be used for the receiver to wake up",
  a 28-bit identification code (the 24-bit id and the `5`?); the test reports of 380H-A and 399H-A measure bursts
  of 204–206 ms. The filing of G8D-344H-A, the remote rtl_433's OEM recordings come from, describes "81
  synchronous codes" and 370 ms instead; its OEM recordings show the long preamble (323–324 short pairs, then the
  4 long).
- **Mitsubishi:** Omron's G8D-620M-A and G8D-571M-A filings (FSK, 0.30 and 0.325 s a press; 571M-A's pulse table
  fits the 80-pair repeat).
- **Suzuki:** ProtoPirate's `suzuki.c` (2025-12-08); no independent source, no OEM recording.

## Not known

- **Kia:** which models and markets send it (every recording is 433.92 MHz, no FCC ID found), and what buttons
  1–4 are: the names are the C's.
- **Suzuki:** whether any Suzuki fob sends it. The one recording is a KEYDIY-made remote (`Suzuki_v0_kd.sub`; its
  id `1530242` shares `153024` with the KEYDIY Honda remote in `Honda_V0_kd_10_10_10.sub`).
- **Mitsubishi:** which of 3, 4 and 6 is Lock or Unlock.
- **Honda:** what the extra byte after some remotes' Lock means; whether the `5` is fixed or the low nibble of a
  28-bit id; a short panic press alone (never recorded); the rule of the TS clones' wrong checks. Whether a car
  takes KAT's presses is not tested.

## Recordings

- **Kia:** `KIA/more/Kia_0_orig.sub` (15 presses, buttons 1, 2, 3 five each, counters 510A–5118) and
  `kia_seed_raw.sub` (4 presses, three copies each): every frame passes the CRC, the counter one higher each press.
- **Suzuki:** `SUZUKI/Suzuki_v0_kd.sub` (15 presses, buttons 3, 4, 2): the same.
- **Mitsubishi:** `MITSUBISHI/Mitsubishi pajero 2011.sub` (4 presses, buttons 4, 3, 6, 6, counters 7–A).
- **Honda:** `honda/more/`: an OEM 2005 Civic remote (id `816BB0`, 81 frames), the 300-press sets of seven remotes,
  a KEYDIY-made and an Xhorse-made remote: 1,360 frames from 9 remotes, every one with the right CRC. Three files
  of aftermarket "TS" clones: 30 of 92 frames pass (the clones are wrong; at the counter the OEM Civic sends `0B`,
  a clone sends `00`). `Honda_v0_RAW_oh_no_sus.sub` has the references' Honda upload on air (a 40-pair preamble,
  a 1000 µs end) and frames with other values where the `5` stands, which no remote sends: a Flipper's
  transmission, it seems, not a remote's.
- **Cut short:** 17 recordings in `honda/more/` are exactly 300 KiB and stop mid-line (the ten `HondaV0_*_300*` and
  TS sets that read 23–42 presses, and seven Honda V2 files): their remotes' other presses were never in the file.

## Validation

- Every press re-encoded with its own button is the remote's next press, all its bits, wherever the recording
  holds it: 760 presses (Kia, Suzuki, Mitsubishi, Honda); a trunk press, 5 then F, from TX Trunk on the press
  before: 10 of 10. The upload is the recorded press pulse for pulse by kind in 725 of the 760; the other 35
  have noise inside their recorded preamble (test `reencode_reproduces_the_next_press`).
- The Honda CRC on the recorded frames above and on rtl_433's OEM recordings; the C's table check agrees with it
  for 3, 4, 5 and F (`honda_check_is_a_crc8_over_counter_and_button`). The five Honda frames that pass the Kia CRC
  are read, and the same bits closed by a Kia gap stay Kia V0 (`honda_frames_that_pass_the_kia_crc`).
- The C (`kia_v0.c` on the host): its encoder's golden vectors decode in KAT to the C's keys (`c_vectors_encode_and_decode`);
  on the recordings KAT reads the C's frames and the five Honda frames more. At the port (v1.4) 3000 random
  frames gave the same uploads and decodes in both; KAT's uploads now differ from the C's where listed above. In `IMPORTS/MITSUBISHI/Mitsubishi pajero 2011.sub` every
  press reads as Kia V0, then Mitsubishi V0. Flipper-ARF's and doctor-firmware's decoders give the same frames
  (ARF names Mitsubishi frames Kia V0). Flipper-ARF's standalone `suzuki.c` reads only HIGH widths and sends an
  asymmetric waveform (LOW always 250 µs) that does not match the recorded remote.
