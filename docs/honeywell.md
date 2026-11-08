---
layout: default
---

# Honeywell Sec Protocol (security sensors)

**Rust module:** `src/protocols/honeywell.rs`
**Reference:** `REFERENCES/doctor-firmware/lib/subghz/protocols/honeywell.c` (HTotoo 2023, after rtl_433's
`honeywell.c` and Logan Lamb's "DEFCON 22: Home Insecurity"; @xMasterX 2025; `SUBGHZ_PROTOCOL_HONEYWELL_NAME` =
"Honeywell Sec"), registered by doctor-firmware, with FoxFW's copy (`REFERENCES/FoxFW/lib/subghz/protocols/
honeywell.c`) for its Manchester reset, see below. Flipper-ARF's `lib` copy (unregistered there), its garage-app
copy (not in the garage registry), Momentum's and RogueMaster's have doctor's feed, bit and upload code. No
reference has a capture of it.

Not to be confused with [Honeywell](honeywell_wdb.md) (`honeywell_wdb.c`, the doorbells).

## Overview

64-bit frames of Honeywell / 2GIG door, window and motion sensors: a 16-bit preamble, channel, 20-bit serial, event
flags and a CRC-16. Static code.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 143 µs | ± te_delta 51 µs |
| te_long | 280 µs | ± 102 µs (2·te_delta) |
| Coding | Manchester | Flipper's `manchester_decoder.c` / `manchester_encoder.c` |
| Bits | 64 | MSB first |

AM (OOK); doctor flags 315 and 433 MHz, the other copies 315 MHz only (KAT runs it on any band, like every
Flipper decoder).

## Frame Layout (64 bits)

| Bits | Field |
|------|-------|
| 63..48 | preamble 0xFFFE |
| 47..44 | channel (2, 4, 0xA: 2GIG, CRC poly 0x8050; 8: poly 0x8005) |
| 43..24 | serial (20 bits) |
| 23..16 | event: 0x80 contact open, 0x40 tamper, 0x20 reed, 0x10 alarm, 0x08 low battery, 0x04 heartbeat |
| 15..0 | CRC-16 (init 0, MSB first) over bytes 5..2 |

## Decoder Steps

1. Each pulse is a Manchester event: LOW |d − 143| < 51 → ShortLow, else |d − 280| < 102 → LongLow (HIGH:
   ShortHigh / LongHigh). Any other duration clears the bit register and leaves the Manchester state as it is.
2. Each decoded bit is shifted in. From the 62nd bit on, the register's bits 63..48 must read 0x3FFE, 0x7FFE or
   0xFFFE (14 to 16 preamble bits).
3. Channel 2 / 4 / 0xA → CRC poly 0x8050, channel 8 → 0x8005, any other channel clears the register; a matching CRC
   reports the frame with its top byte set to 0xFF (`FF FE` header), a mismatch clears the register; 64 bits
   without a preamble clear it too.

**Union of doctor's and FoxFW's state machines.** FoxFW's alloc and reset also put the Manchester decoder in
`Mid1`; doctor's starts zero-initialised (`Start1`) and is never reset. After a frame, the 84 ms LOW leaves doctor's
state where the frame's last bit put it, so the next frame syncs two half-bits late and only 60 bits are read (the
check needs 62); FoxFW's reset after the report syncs on the next frame's first HIGH. On doctor's own encoder output
(4 copies of one frame) doctor decodes the first copy only, and nothing when the copies follow a 20 ms gap, where
FoxFW decodes 4 of 4 (8 of 8). Doctor in turn reads one more bit at the start of a fresh decoder (`Start1` takes a
lone short LOW as a bit). KAT runs both ("lanes"), each with its own Manchester state and bit register, reports a
frame as soon as either completes one, and then resets both as each copy's callback would (doctor: the register;
FoxFW: the register and `Mid1`), so a frame is reported once.

## Field mapping

`serial` as the C; `button` / `counter` None (the C sets neither); `data` = the 64 bits (`FF FE` header);
`crc_valid` = the CRC holds (always for decodes); `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_honeywell_get_upload` (`encoder.repeat` = 4): bits 62..0 through Flipper's Manchester
encoder (bit 63 is not sent), the closing half when it is HIGH, then LOW 84 ms (300·te_long). The key as stored; no
button. On air (equal levels merged) doctor's decoder reads none of these copies after a gap and FoxFW's every one —
KAT reads them all.

## Key file

`Protocol: Honeywell Sec`, `Bit: 64`, `Key` (generic serialize; doctor's serializer also writes `Serial` / `Btn` /
`Cnt`, which KAT does not). The decoder's deserialize refuses `Bit` < 62 and reads a 62- or 63-bit key (older builds)
with its header set to `FF FE`; the encoder's deserialize checks no bit count and sends the file's `Key` as it is.
KAT keeps the file's `Key` and `Bit` (what the C transmits) and reads the fields as the decoder rebuilds them.

## Validation

Reference oracles `subghz_oracle_{doctor,foxfw,roguemaster,momentum}` and `subghz_oracle_arf --registry all`; the
decode dump is `hb_parity_decode_dump` in `src/protocols/nord_ice.rs` (it also runs each lane alone), the encode
dump f_common's `F_KEYS`.

- 1200 fuzz files (300 per protocol of this round; here the reference encoder's copies and sensor-like frames with
  12..20 preamble bits, every channel, wrong CRCs, jitter up to ± 60 µs, window-edge durations, dropped / split /
  stretched pulses, dropped or doubled bits), every file through each decoder alone: doctor's lane is identical with
  doctor, RogueMaster, Momentum and ARF (168 frames), FoxFW's lane with FoxFW (266); KAT's union reports 266 frames
  and every frame of both references in all 1200 files.
- The 243 RAW files of IMPORTS and the ARF / Momentum / RogueMaster unit tests: no frame in any reference or in KAT.
- Encoder: 64 key files (valid frames of each channel, wrong CRCs and channels, random keys, 62- and 63-bit keys) —
  KAT's upload equals doctor's, RogueMaster's, Momentum's and ARF's pulse for pulse (FoxFW's SD-plugin encoder has
  doctor's source).
- `kat_diff` over IMPORTS and the unit-test folders: no Honeywell Sec frame anywhere.
