---
layout: default
---

# Ford V3 Protocol

**Rust module:** `src/protocols/ford_v3.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/ford_v3.c` (current version with the US variant; Flipper-ARF's
copy is the same code)

## Overview

Ford V3 frames are 104 bits (13 bytes) at 240/480 µs, sent as plaintext with no CRC or encryption. Two decoders
see every pulse, in the C's order:

- **US variant:** a half-bit-cell decoder for frames that start with a `0xFF` header. It is gated on the header,
  the serial, the button and a flag bit.
- **EU variant:** the original Manchester decoder (≥30-short preamble, then 104 bits). The C has no further
  checks; KAT has one, for the stream of a Revers RB2 gate remote (Decoder Steps).

Since the EU path checks nothing, KAT holds Ford V3's frames back in a recording or a received burst: a frame is
listed unless another decoder reads the same signal, and a Ford V3 hit resets only the decoders held back this
way (the rtl_433 ports and itself; see Validation).

Decode-only: the ProtoPirate encoder is NULL, so KAT does not transmit Ford V3 (Replay still works on the raw
capture).

## Timing

| Parameter       | Value  | Notes                                              |
|-----------------|--------|----------------------------------------------------|
| Short           | 240 µs | EU ±60 µs; US cells ±120 µs                        |
| Long            | 480 µs | EU ±60 µs; US cells ±120 µs                        |
| Frame           | 104 bits | 13 bytes                                         |
| EU preamble     | ≥30 short pulses | before the first long pulse              |
| US burst        | ≥200 cells | ≥100 valid bits from the first cell (cap 320) |

## Frame Layout (104 bits / 13 bytes)

| Bytes      | EU variant                                 | US variant                    |
|------------|--------------------------------------------|-------------------------------|
| raw[0]     | header                                     | `0xFF`                        |
| raw[1..4]  | serial (32-bit, big-endian)                | serial (≠ 0, ≠ 0xFFFFFFFF)    |
| raw[5]     | —                                          | flag (C: bit 7 must be set; KAT: not checked — real 2022 Escape frames carry 0x08) |
| raw[6]     | button: bit 0 set → Unlock, else Lock      | button: 0x01 Lock, 0x02 Unlock, 0x07 Trunk, 0x06 Panic |
| raw[7..8]  | counter, stored inverted (`~b7`, `~b8`)    | counter (not inverted)        |
| raw[9..12] | hop (not parsed)                           | hop (not parsed)              |

**KAT packing:** `data` = raw[0..8] (the C's `Key`), `extra` = raw[8..13] right-aligned (counter low byte +
hop; the C saves the whole frame as `Raw`), `data_count_bit` = 104 (decode-only).
As in the C's deserialize, the variant can be re-derived from the raw bytes: a frame that passes the US gate is
US.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM and FM (ProtoPirate flags `AM | FM`)
- **Encryption:** none (plaintext)
- **Frequencies:** 315 MHz, 433.92 MHz (the ±2 % match covers 312–315 MHz US captures); a 2026 VW Amarok's key
  sends at about 434.24 MHz (2-FSK, ±24 kHz)

Neither path verifies anything (there is no checksum), so the CRC column shows "none" for Ford V3 captures, and
the EU path takes part only in the normal-polarity pass of KAT's registry: with no check at all, a 240/480 µs
Manchester frame cut short and read inverted is a "Ford V3" frame with nonsense fields where every reference
reports nothing (no reference has an inverted pass).

## Decoder Steps

**US path:**

1. Each short pulse appends one half-bit cell of its level; each long pulse appends two.
2. Any other pulse ends the burst. With ≥200 cells, the burst is decoded from its first cell at phase 0, then
   phase 1, as bit = first cell of each pair, up to 104 bits.
3. Every pair must differ and ≥100 bits are required. The frame must pass the gate:
   `raw[0] == 0xFF`, the serial is not 0 or 0xFFFFFFFF, and `raw[6]` is 0x01/0x02/0x07/0x06 (the C also
   requires `raw[5]` bit 7 and only 0x01/0x02 — see Validation for why KAT relaxes this).
4. **KAT:** the frame must end the burst. No whole bit may follow bit 104 (one unpaired cell may); the C
   reads the first 104 bits of a longer burst and ignores the rest. See Validation.

**EU path:**

1. **Reset**: a short pulse starts the preamble (count = 1).
2. **Preamble**: count short pulses. Once ≥30 are seen, a long pulse seeds the Manchester state to Mid1, is
   fed as `level ? LongHigh : LongLow`, and enters Data.
3. **Data**: events are `level ? ShortHigh/LongHigh : ShortLow/LongLow` (exactly as `ford_v3.c`). Bits are
   packed MSB-first. At 104 bits the frame is committed; a gap pulse ends the frame.
4. **KAT:** a frame that is the stream of a [Revers RB2](revers_rb2.md) remote is dropped. Such a remote sends
   one 64-bit message over and over without a gap, Manchester coded at 250/500 µs: 32 ones, a 22-bit id, the
   stop code `0x200`. Its ones are this path's preamble and the 104 bits behind them its frame. The frame is
   dropped when its bits repeat after 64 and those 64, begun at the right bit, are 32 ones, 22 bits and the
   stop code (a Ford frame would need 40 bits that repeat its first 40, and 42 more of a fixed value). See
   Validation.

The Manchester machine is Flipper's `manchester_advance`, ported literally: states Start1=0, Mid1=1, Mid0=2,
Start0=3; events ShortLow=0, ShortHigh=2, LongLow=4, LongHigh=6; transitions `{0x01, 0x91, 0x9B, 0xFB}`. An
event that maps to the current state resets to Mid1 with no bit; entering Mid1 emits 1 and Mid0 emits 0.

A frame identical to the previously committed one is not reported again until `reset()` (the C's dedup). KAT's
registry resets after every hit, so each repeat in a capture is still listed.

## Encoder

Not supported (ProtoPirate's, Flipper-ARF's and doctor-firmware's encoder is NULL). Decode-only.

FoxFW has an encoder, the SD plugin `enc_ford_v3.c`, which replays the key file's `Raw`. KAT does not use it:
run on a host build of FoxFW with its plugins compiled in, on 132 made-up key files and KAT's key files for the
15 real frames in `IMPORTS` (Escape, LDV), it sends no real frame. Its upload buffer holds 1400 pulses and a
frame with fewer than about 55 bit changes overflows it (`furi_check`: a crash on the Flipper), which is every
real frame. The frames that fit go out without frame bit 0, in the US polarity behind an EU preamble; FoxFW's
own decoder reads them as another frame with another serial, and so does KAT's.

## Validation

- **EU path:** decodes the REAL `IMPORTS/LDV` captures (LDV T80, 4 frames lock / 3 unlock). They give serial
  `0x43A88F90`, `raw[6]` `0x68` lock / `0x70` unlock, and the same values in every repeat. These bits match
  ProtoPirate and a direct half-bit decode of the capture. The earlier port used KAT's shared Manchester table,
  which is not Flipper's, and produced serial `0xDF56FD7D` instead.
- **EU path, a 2026 VW Amarok key.** The second-generation Amarok shares the Ford Ranger's platform and is built
  by Ford; a tester's key (a Valeo flip key, board B6681-2, NXP chip marked F7345FL1020) sends Ford V3 EU
  frames. Recorded with a PortaPack as IQ (centre 434.25 MHz, 250 kS/s) and converted to
  `VAG/more/2026_vw_amarok_lock.sub` / `..._unlock.sub` with an FM discriminator (hysteresis slicer, pulses
  under 30 µs merged): 2-FSK at about 434.24 MHz, ±24 kHz, half bit 240 µs; each press sends the frame three
  times, 100 ms long and 100 ms apart. Header `0x00`, serial, `raw[5]` `0x9B`, button `0xFE` Lock / `0xFD`
  Unlock, the counter inverted and one up per press (20 Lock and 20 Unlock presses, none missing), then the
  32-bit hop. In the IQ files the middle copy of every press has a gap of 0.4 to 1.7 ms at a different place
  (most likely samples the recorder dropped); read straight from the IQ file, one such copy passes as a frame
  with another serial and counter, since the EU path checks nothing.
- **Held back in a stream (KAT).** The EU path checks nothing, so it also reads other remotes' Manchester at
  250/500 µs. On an Xhorse Honda V0 remote's three recordings (`IMPORTS/honda/more/HondaV0_xHorse_*`) it read 7
  noise frames inside the presses (all zeros, all ones, serial 0), and each of its hits reset every decoder,
  Honda V0 in the middle of its frame. KAT now holds Ford V3's frames back as it does the rtl_433 ports'
  (`HELD_LIKE_PORTS`): a frame stands unless another decoder reads the same signal, and a Ford V3 hit resets
  only the held decoders. The Honda recordings now list 4 more Honda V0 presses, one more as Nidec RKE (a press
  Honda V0 does not read; it shows the remote's serial, button and that press's counter), and no Ford V3 frame.
  The LDV captures read as [LDV V0](ldv_v0.md), whose decoder checks the frame's fixed fields, also from a
  stored frame. The 2022 Ford Escape's 20 frames are unchanged.
- **US path:** decodes the REAL 2022 Ford Escape capture `IMPORTS/FORD/22_ford_escape_u2_l2_t2_p2.sub`
  (315 MHz): serial `0x4E2464E4` in every frame, the counter rising by one per press (`0xEB0D` → `0xEB16`),
  and buttons `0x02`, `0x01`, `0x07`, `0x06` in exactly the order the file name records (unlock ×2, lock ×2,
  trunk ×2, panic ×2). **KAT deviation:** the C also requires `raw[5]` bit 7 and only buttons 1/2; every
  Escape frame carries `raw[5] = 0x08` and Trunk/Panic use `0x07`/`0x06`, so the C rejects the whole capture.
  KAT drops the flag check and accepts the two extra buttons (the structural gates are unchanged); no other
  capture changes. The `21_ford_escape` and `25_ford_bronco` captures hold no 240/480 µs OOK frames.
- **US frames end the burst (KAT).** A volunteer's Alfa Romeo Giulia (433.92 MHz, OOK; not in `IMPORTS`)
  sends 112-bit Fiat V2 frames. Read in the opposite polarity, their first 13 bytes start `FF FE` (the
  inverted `00 01` markers), and when the Fiat type byte is `0xFE` its inverse `0x01` is a US button. Fiat V2
  and the US path finish on the same gap after the frame, Ford V3 comes first in KAT's registry, and in
  KAT 1.8.5 Ford V3 took 28 of the car's 30 unlock frames (3 of its 5 codes lost). The Flipper firmwares run
  every decoder side by side: ProtoPirate and Flipper-ARF list one such Ford V3 frame beside the five Fiat V2
  codes (doctor-firmware's Ford V3 has no US path). Every Escape burst is a single frame of 208 or 209
  cells, at most one unpaired cell after bit 104, while the Giulia's carry 16 cells more (Fiat V2's last
  byte). KAT now rejects a US frame that a whole bit follows, and the Giulia decodes as Fiat V2 only. Over the air (Flipper Zero → HackRF, a made-up remote
  with the same layout), 1.8.5 listed all three unlock transmissions as Ford V3 and now lists Fiat V2.
- **EU frames of a Revers RB2 remote (KAT).** rtl_433's two recordings of such a remote
  (`rtl_433_tests/tests/revers/rb2/01`, pulses within 60 µs of 240 and 480 µs) are Ford V3 in ProtoPirate's
  and doctor-firmware's C, three frames each; up to 1.12.0 KAT listed them the same way. They are Revers_RB2 now, 7 messages each, and never Ford V3
  (`rtl433_recordings_decode`). Flipper's capture of a real remote (`revers_rb2_raw.sub`) was never taken: its
  pulses are wider apart. The LDV captures, the only real EU frames at the time, decode as before (and are
  listed as LDV V0, see above).
- **C oracle:** on the LDV captures every frame (fields and all 13 bytes) equals the compiled ProtoPirate and
  Flipper-ARF decoders; on 2000 randomized EU/US bursts the only differences are frames admitted by the US
  relaxation above. No reference has a Ford V3 encoder (all three register `.encoder` with NULL alloc).
- **Unit tests:** cover the Flipper transition table, synthetic EU and US frames with variant-specific fields,
  the preamble requirement, the US gate (including the Escape's 0x08 flag), a Fiat V2 burst whose first 13
  bytes pass that gate, a Revers RB2 stream left alone by the EU path, dedup, and the LDV fields above.
