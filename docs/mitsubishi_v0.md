---
layout: default
---

# Mitsubishi V0 Protocol

**Rust module:** `src/protocols/mitsubishi_v0.rs`  
**Reference:** `REFERENCES/Flipper-ARF/lib/subghz/protocols/mitsubishi_v0.c` (96-bit format);
ProtoPirate `protocols/kia_v0.c` type 4 (61-bit format)

## Overview

Two references use the name "Mitsubishi V0" for two different air formats. KAT keeps both under this one
name:

- **61-bit (ProtoPirate, current):** a Kia V0 frame sent twice, the second copy behind an 80-pair preamble.
  It is decoded by [Kia V0](kia_v0.md), which reports it as "Mitsubishi V0". ProtoPirate removed its old
  standalone `mitsubishi_v0.c` on 2026-09-14 ("merge Mitsubishi protocol into Kia V0").
- **96-bit (Flipper-ARF):** the old ProtoPirate format, still shipped (enabled, with an encoder) by
  Flipper-ARF. ProtoPirate's Kia V0 does not read it, so this module decodes it.

The registry entry owns the name for TX: a 61-bit signal is encoded by Kia V0's Mitsubishi upload, a 96-bit
one by the ARF encoder below.

## 61-bit format

The frame and its CRC-8 are Kia V0's ([Kia V0](kia_v0.md)). A press, as the 2011 Pajero recorded sends it (all
four presses): a carrier of about 2.5 ms, 320 short pairs, the frame, HIGH 1.5 ms and LOW 1.5 ms, 80 short pairs,
the frame again, HIGH 1.5 ms and LOW 1.5 ms. Its counter goes up by one with every press.

- **Transmit:** the fob's next press, sent once: the counter + 1 and the press above (the second preamble 80 pairs;
  the C sends 81, its 80 plus the short pair of the frame's cleared bit 60). Built from the press before, it is
  the Pajero's next press pulse for pulse.
- **Buttons:** shown as numbers ("Btn 0x3"). The Pajero sends 3, 4 and 6; no recording names its buttons and no
  source gives the codes, so Lock, Unlock, Trunk and Panic all keep the captured button. (The C gives Mitsubishi
  Kia's codes, 1 Lock and 2 Unlock, which this fob never sends.) The 96-bit format's codes are not known either.
- **Remotes:** Omron's OUCG8D-620M-A (Outlander 2007–12, Endeavor, Eclipse and Galant 2006–07 in the parts
  catalogues; FSK, 0.30 s a press) and OUCG8D-571M-A (433.92 MHz, 0.325 s; its test report's pulses fit the
  80-pair repeat after a 1.5 ms gap) file this kind of signal. Which one the Pajero has is not known.

## 96-bit format

| Parameter | Value | Notes |
|-----------|-------|-------|
| Bit 1 | HIGH 250 µs + LOW 500 µs | ±100 µs, strict `<` |
| Bit 0 | HIGH 500 µs + LOW 250 µs | |
| Frame | 96 bits (12 bytes) MSB first | no preamble, sync or checksum gate in the decoder |

On air the first 8 bytes are inverted and bytes 0–4 XORed with `hi ^ lo` of the counter. The decoder
inverts bytes 0–7, reads the counter from bytes 4–5 **before** undoing the XOR, then XORs bytes 0–4 —
so byte 4 of the displayed counter is itself scrambled (ARF: "asymmetric scramble"; the displayed
counter's two bytes are always equal). Fields: serial = bytes 0–3, counter = bytes 4–5, button = byte 6.

`data` = wire bytes 0–7 and `extra` = wire bytes 8–11 (ARF's saved `Raw_Frame`). Captures from KAT's
earlier decode-only port kept the unscrambled bytes in `data` and no `extra`.

## RF

- **Encoding:** PWM, complementary HIGH/LOW widths
- **RF modulation:** FM
- **Frequencies:** 315 MHz, 433.92 MHz, 868.35 MHz (ARF flags; KAT's earlier port listed only 868.35)

## Decoder Steps (`subghz_protocol_decoder_mitsubishi_v0_feed`)

1. **Reset** — any HIGH is remembered → DataCheck.
2. **DataCheck** — a LOW completes the pair; an invalid pair discards the payload. The frame is reported on
   the LOW of its 96th pair.
3. **DataSave** — waits for the next HIGH (a LOW here resets).

## Encoder

Supported (ARF `subghz_protocol_encoder_mitsubishi_v0_get_upload`), replay-only like ARF: the captured
12-byte frame is re-sent verbatim — no counter bump, and the captured button for every command (ARF knows
no button codes). Without the raw tail (`extra` = None) the frame is rebuilt from serial/counter/button as
ARF's fallback does (bytes 7–8 = 0, 9 = 0x5A, 10–11 = 0xFF, then scrambled). Upload: 100 + 95 short pairs
(HIGH 250, LOW 250), then the 96 data pairs. The frame has no terminator, so `encode()` returns ARF's whole
transmission (5 copies back to back); on air a receiver completes the first 4. ARF sizes its upload buffer
for 414 entries but writes 582 (a heap overflow on the device); the intended upload is unambiguous.

## Validation

C oracle (ARF `mitsubishi_v0.c` on the host): 2000 random frames (raw replay and field fallback) — KAT's
upload equals ARF's pulse for pulse and KAT's decoder reports the same 2000 frames (fields and raw bytes).
No IMPORTS capture contains the 96-bit format (0 decodes in both C and KAT, both polarities).

The 61-bit format has a real capture since 2026-11: `IMPORTS/MITSUBISHI/Mitsubishi pajero 2011.sub` (FM476,
433.92 MHz), four presses of one fob (serial `DF8ABA0`, counters 7 to 0xA). Each press is the Kia V0 frame and
the same frame again behind the 80-pair preamble: KAT lists the first as Kia V0 and the second as Mitsubishi V0,
both with a valid CRC-8. The references' host builds (ProtoPirate, doctor-firmware, Flipper-ARF, FoxFW) list the
same eight frames, all as Kia V0: that harness resets the receiver after every decode, as the stock Sub-GHz app
does, and the C's reset forgets the first copy (`last_kia_data`). ProtoPirate's own app does not reset there
(`protopirate_scene_receiver.c`), which is the case KAT follows ([kia_v0.md](kia_v0.md)).

**CRC column.** A 61-bit frame carries Kia V0's CRC-8 and shows OK or FAIL; a 96-bit frame has no check and shows
"-". Up to 1.13.0 both showed "-".

Up to 1.20.1 KAT named the 61-bit buttons with Kia's names (the Pajero's 3, 4 and 6 showed as Trunk, Panic and
Other), sent Kia's 1 and 2 for Lock and Unlock, and put 81 short pairs before the second copy.
