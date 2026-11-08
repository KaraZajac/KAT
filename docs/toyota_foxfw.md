---
layout: default
---

# Toyota (FoxFW) Protocol

**Rust module:** `src/protocols/toyota_foxfw.rs`
**Reference:** `FoxFW lib/subghz/protocols/toyota.c` / `toyota.h` (protocol name "Toyota"); its decoder is
Flipper-ARF's retired `toyota-old.c.bak` line for line, plus the `Tail` key-file line.

## Overview

Flipper-ARF's older Toyota decoder: KeeLoq-layout frames (32-bit hop, 28-bit serial, 4-bit button) on 400/800 µs
PWM (variant C: Prius 2006 / Corolla Verso, 433.92 MHz) or 200/390 µs NRZ pulses (variant B: Tundra). ARF replaced
it with the 96-bit Toyota/Lexus protocol of [toyota.md](toyota.md), a different air format; FoxFW still registers
it as "Toyota". KAT registers it as **Toyota (FoxFW)**, **decode and display only**: FoxFW's encoder is an SD-card
plugin (`enc_toyota`) that KAT does not port, and the hop is not decrypted.

It reads the two Prius 2006 captures in `IMPORTS/Toyota + Lexus/`, which no other KAT decoder reads.

## Timing

|                     | Variant C (Prius / Corolla Verso)                    | Variant B (Tundra)                         |
|---------------------|------------------------------------------------------|--------------------------------------------|
| Selected by         | first HIGH 310–499 µs (\|d − 400\| < 100, d ≥ 310)   | first HIGH 141–259 µs (\|d − 200\| < 60)   |
| Preamble            | ≥ 6 pairs: HIGH 200–550 µs + LOW 200–650 µs          | ≥ 10 pairs: HIGH 141–259 µs + LOW 331–449 µs |
| Sync                | LOW 1050–1500 µs                                     | LOW 1700–2200 µs                           |
| Data                | PWM on the HIGH (200–1050 µs, 1 when > 600 µs); the LOWs only separate | every pulse, HIGH or LOW (100–500 µs), is a bit, 1 when > 287 µs |
| Bits                | 66–68 (HIGHs past the 68th are dropped)              | 67                                         |
| Frame end           | a LOW of 1050–1500 µs (the next frame's data follows), any other LOW over 1000 µs, or a HIGH outside 200–1050 µs | the 67th bit |

A first HIGH under 141 µs, of 260–309 µs or of 500 µs and more starts nothing. The real Prius frames have 69 data
HIGHs (68 kept) and are 8.3 ms apart; each is reported on the long LOW after it.

## Frame

| Bits (arrival order, the first is the MSB) | Field  | Notes |
|---|---|---|
| 0–31   | hop    | encrypted; not decrypted |
| 32–59  | serial | 28 bits |
| 60–63  | button | C: 0xB Lock, 0xE Unlock, 0xD Trunk, 0x7 Panic; B: 0xA Lock, 0x5 Unlock, 0xF, 0x4 (the last two named nowhere in FoxFW) |
| 64–67  | tail   | the 2–4 bits past the 64th (`Tail`) |

A frame is dropped when its serial is 0 or 0x0FFFFFFF, its hop 0 or 0xFFFFFFFF, or its button outside its variant's
list (`toyota_frame_plausible`). There is no checksum. FoxFW reads a long HIGH as 1, the inverse of Flipper's KeeLoq
decoder (short HIGH + long LOW = 1): read that way, the Prius serial 0x7231804 is 0x8DCE7FB, bit for bit inverted.

## Decoder state machine

1. **Reset step:** LOWs are ignored; a HIGH selects variant B or C (table) and is the first preamble HIGH.
2. **Variant B:** a short HIGH then a long LOW counts a preamble pair; anything else resets. The sync after 10 or more
   pairs starts the data (after fewer: reset). Each data pulse is one bit; one outside 100–500 µs resets, one of
   1700 µs or more ends the frame; the 67th bit reports it and resets.
3. **Variant C:** a HIGH then a LOW count a preamble pair when both fit their windows (the HIGH is judged on its
   LOW); the sync after 6 or more pairs starts the data. Data HIGHs push bits (68 at most). A LOW of 1050–1500 µs
   (checked first) reports the frame if it has 66 or more bits and starts the next frame's data at once, with no
   preamble; any other LOW over 1000 µs reports it the same way and returns to the preamble step without the reset
   step's variant test; a HIGH outside 200–1050 µs reports it and becomes the first HIGH of a new preamble.

The C sets these states after its decode callback, where the receiver resets every decoder; KAT keeps them through
the registry's reset after its own report. Variant A (Corolla, LS/SL pairs) is dead code in the C (nothing enters
its steps) and is not ported.

## Field mapping

`data` = hop << 32 | serial << 4 | button (the C's `generic.data`, the frame's first 64 bits), `data_count_bit` =
66–68, `extra` = the tail, `serial`, `button`, `counter` = the variant (the C's `generic.cnt`: 1 = B, 2 = C),
`crc_valid` false (shown as "-"), `encoder_capable` false.

## Key file

`Protocol: Toyota`, `Bit`, `Key` = `data`, then `Tail` (decimal); FoxFW's generic serializer writes no Serial / Btn
/ Cnt. Loading needs `Bit` ≥ 66 and an 8-byte `Key` and reads `Tail` (0 when absent, as in files saved before FoxFW
v2.0.12). "Toyota" is the name of KAT's [Toyota](toyota.md) decoder too, so that decoder hands keys of 66–68 bits
here (its own have 96). The C takes the variant from the decoder's previous `cnt` (the file has none); KAT derives
it from the button (the two lists do not overlap), so a reloaded key keeps its variant.

## Registry

Last of the FoxFW gate-only block (after Nissan V0, before the rtl_433 ports): its frame check is a plausibility
gate only, a 4-of-16 button list. Kia V3 has variant C's air format (HIGH 400 + LOW 1200 sync, 400/800 µs PWM) and
its lock / unlock codes read 0xB / 0xD here, so FoxFW's decoder alone reads `IMPORTS/KIA/KIA V3 N1 RAW.sub` (FoxFW's
full registry reports those frames as KIA/HYU V3/V4, which completes first). In KAT those files are FSK and this
decoder AM-only. On KAT's own AM receive path (live, IQ files) Kia V3/V4, FM-flagged, only runs in the ungated
second pass; when Toyota (FoxFW) is all the gated first pass reads, that second pass runs anyway
(`NOT_FINAL_ON_GATED_PASS` in `mod.rs`), and there Kia V3/V4 completes first: an OOK Kia V3 fob still decodes as
Kia V3 (without the rule its frames came out as Toyota (FoxFW)). An AM-preset `.sub` of a Kia V3 fob (none in the
corpus) would read as Toyota (FoxFW), where KAT showed nothing before (the preset gate is strict and Kia V3/V4 is
not fed there).

## Validation

- No deviation from the C in the decoder (the key-file variant above is the only difference).
- Real captures: `Toyota_Prius2006_lock.sub` / `_unlock.sub`, 8 frames each (68 bits, hop 0x3F2E5098 / 0x29CD45FF,
  serial 0x7231804, button 0xB / 0xE, tail 0xF), identical to the host build of FoxFW's C frame for frame. No other
  file in IMPORTS or the Flipper-ARF, Momentum and RogueMaster unit-test captures changes.
- Synthetic frames (made-up values): variant C at 66, 67 and 68 bits with each of its buttons and with 69 data HIGHs,
  variant B at 67 bits with each of its buttons, three restart sequences (sync restart, long-HIGH end, a 1020 µs end
  followed by a 250 µs first preamble HIGH), and eight refused ones (button 0x1, serial 0, hop 0xFFFFFFFF, 65 bits,
  a 1600 µs sync, 5 preamble pairs, a B frame with button 0xB, a 66-bit B frame): identical to the C's output.
- 416 perturbed copies of the 16 base frames (jitter 5–30 %, scale 0.85–1.15, a dropped or glitch-split pulse,
  truncation, noise before, the frame twice): this decoder alone matches the C on every file (305 frames). Through
  KAT's registry 12 files differ only by other decoders: the rtl_433 port Microchip-HCS361 reads 10 copies the C's
  Toyota refuses, and CAME TWEE (doctor-firmware's 12 ms header path, registered earlier) reads 2 after the
  synthetic 10 ms lead-in, one of which the C reads as Toyota.
