---
layout: default
---

# Nord ICE Protocol

**Rust module:** `src/protocols/nord_ice.rs`
**References** (all @xMasterX, 2026.03; `SUBGHZ_PROTOCOL_NORD_ICE_NAME` = "Nord ICE"):
- `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/nord_ice.c` — registered by the garage app (RX
  group g12) as a decoder only (`SUBGHZ_GARAGE_RX_ONLY` compiles its encoder out). The first version: te 300 / 800 µs,
  one gap window 7500 ± 1650 µs, back to Reset after a frame, the key as received. Momentum's, Flipper-ARF `lib`'s
  (unregistered there) and FoxFW's decoders are the same; their encoders send that key once per repeat with a
  7500 µs gap (ARF `lib` adds a ProtoPirate-style D-pad over the 6-bit field).
- `REFERENCES/doctor-firmware/lib/subghz/protocols/nord_ice.c` and RogueMaster's (the same code with comments,
  "measured off six captures of one remote") — the second version: te 270 / 760 µs, gaps 7440 and 8980 µs (± 450),
  decoding on from the gap that closed a frame, the frame flag (bit 14) cleared, a 4-bit one-hot button, and an
  encoder that sends the frame pair with the button switchable. RogueMaster's "Add Manually" creates Nord ICE
  remotes for it.

No reference has a capture of it.

## Overview

33-bit fixed code: a 27-bit serial split around a frame field and a one-hot button (four buttons). A press sends
pairs of frames, `[key | 1 << 14] gap 8980 µs [key] gap 7440 µs`, repeated while held; the two frames differ only in
bit 14. No checksum.

## Timing / modulation

| Parameter | doctor / RogueMaster | garage app / Momentum / ARF `lib` / FoxFW | KAT (union) |
|-----------|----------------------|-------------------------------------------|-------------|
| Short | 270 ± 150 µs | 300 ± 150 µs | either, paired with the same version's long |
| Long | 760 ± 150 µs | 800 ± 150 µs | either |
| Gap | 7440 ± 450, 8980 ± 450 µs | 7500 ± 1650 µs | 5851..=9429 µs |
| After a frame | the gap opens the next frame | Reset | the gap opens the next frame |
| Key | bit 14 cleared | as received | bit 14 cleared |

Bit 1 = HIGH long + LOW short, bit 0 = HIGH short + LOW long, the last bit's LOW being the gap; 33 bits, MSB first.
AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`).

## Frame Layout (33 bits)

| Bits | Field |
|------|-------|
| 32..15 | serial, high 18 bits |
| 14..13 | frame field: bit 14 = the flagged frame of a pair, bit 13 set on every frame |
| 12..9 | button, one hot: 0x4, 0x8, 0x1, 0x2 (the order they sit on the remote) |
| 8..0 | serial, low 9 bits |

Serial = (data >> 15) << 9 | data & 0x1FF (27 bits). The garage version shows bits 14..9 as a 6-bit button.

## Decoder Steps

1. Reset: a gap (either version's window).
2. HIGH (saved) + LOW pairs: a bit when either version reads the pair; anything that is neither a bit half nor a gap
   → Reset.
3. A gap closes the frame: the last bit is read from its HIGH (either version's short / long), and with exactly 33
   bits the key, bit 14 cleared, is reported. The gap then opens the next frame (doctor: its callback's receiver
   reset is overridden by the `SaveDuration` step after it; KAT keeps that state through the registry's reset).

**Union.** Every frame either version reports, KAT reports at the same pulse (the garage's with bit 14 cleared):
the two versions' bit and gap windows never overlap another class, so each version classifies every pulse it
accepts as KAT does. KAT also reads the frames the garage skips (every other one) and frames whose gaps only one
version takes while only the other reads its bits.

## Field mapping

`serial`, `button` as doctor; `counter` None; `data` = the 33 bits (bit 14 clear); `crc_valid` false (no checksum);
`encoder_capable` true. Buttons are shown as numbers (Btn 1, 2, 4, 8).

## Encoder

doctor's / RogueMaster's `subghz_protocol_encoder_nord_ice_get_upload` (`encoder.repeat` = 3): bit 14 cleared and the
button put in bits 12..9, then the flagged frame with the 8980 µs gap and the plain frame with the 7440 µs gap, at
270 / 760 µs; 3 copies of the pair (396 pulses). Button: KAT's native code 0..=0xF is sent (doctor's signal-settings
button editor takes any 4-bit value; its D-pad walks 0x4 → 0x8 → 0x1 → 0x2 from the captured button), larger codes
keep the captured button; "Send button…" lists the four. doctor writes the sent key back to the file (flag cleared,
the sent button), which is the key KAT keeps. The older encoders (garage source, Momentum, ARF `lib`, FoxFW) send one
frame per repeat with a 7500 µs gap.

## Key file

`Protocol: Nord ICE`, `Bit: 33`, `Key` (generic serialize; doctor's serializer also writes `Serial` / `Btn` / `Cnt`,
which KAT does not); loading requires `Bit: 33`. A key saved with bit 14 set (an older build, or the garage app) loads
with it cleared, as doctor's display and encoder clear it.

## Add Manually

"Nord ICE 433MHz" (RogueMaster only): AM650, 433.92 MHz, `Bit: 33`, key `(rand() & 0x1FFFF81FF) | 0x2400` — button 0x2
(the fourth), bit 13 set, bit 14 clear. KAT's key file of it loads, sends each of the four buttons and decodes back
with the same serial. See [add_manually.md](add_manually.md).

## Validation

Reference oracles `subghz_oracle_garage`, `subghz_oracle_{doctor,roguemaster,momentum,foxfw}`, `subghz_oracle_arf
--registry all`; the decode dump is `hb_parity_decode_dump` in `src/protocols/nord_ice.rs` (it also runs each version
alone), the encode dump f_common's `F_KEYS`.

- 1200 fuzz files (300 per protocol of this round; here doctor's frame pairs and the garage's single frames, short
  140..430 / long 620..940 µs before jitter, gaps 5900..9400 µs, 31..35-bit frames, every button, flagged and plain
  keys, jitter, dropped / split / stretched pulses, dropped or doubled bits), every file through each decoder alone:
  the doctor version is identical with doctor and RogueMaster (500 frames), the garage version with the garage app,
  Momentum, FoxFW and ARF (320). KAT's union reports 701 frames and every frame of both versions in all 1200 files;
  78 files give it more frames than either version, 39 a key neither reads (mixed-window frames, see above).
- The 243 RAW files of IMPORTS and the ARF / Momentum / RogueMaster unit tests: no frame in any reference or in KAT.
- Encoder: 80 random key files (every button value, flag set and clear, bits above 33) and 16 RogueMaster Add
  Manually keys — KAT's upload equals doctor's and RogueMaster's pulse for pulse; Momentum's and ARF's single-frame
  encoders differ (all 96), the garage app has none.
- `kat_diff` over IMPORTS and the unit-test folders: no Nord ICE frame anywhere.
