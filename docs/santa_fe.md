---
layout: default
---

# SantaFe 13-16 Protocol

**Rust module:** `src/protocols/santa_fe.rs`
**Reference:** FoxFW `lib/subghz/protocols/santa_fe.c` (`SANTA_FE_PROTOCOL_NAME = "SantaFe 13-16"`) and its SD-card
encoder plugin `applications/main/subghz/protocols_ext/enc_santa_fe.c` (with the `pp_auto_common.c` helpers the
firmware hands it).

## Overview

An 80-bit frame with a CRC-8, for the 2013-2016 Hyundai Santa Fe fob. The reference flags it
`SubGhzProtocolTypeStatic`, `433 | AM`. KAT decodes it and transmits it as FoxFW's encoder does.

**Provenance.** The format comes from Flipper-ARF's `lib/subghz/protocols/auto_rke_protocols.c` ("Additional automotive
RKE protocols — ported from Pandora DXL 5000 firmware": its SantaFe 13-16, ID 0x1A, the Hyundai Santa Fe / Solaris TRW fob), a reverse-engineering file ARF never registers, like its
other Pandora-derived decoders (Audi, Hyundai, Mazda Infinity, which KAT keeps unregistered). A KAT fork (limbo111111/KAT,
written by Google's Jules bot in May 2026) ported it to Rust, "aligned with Flipper-ARF reference: `auto_rke_protocols.c`".
FoxFW's C, whose header says "ported from KAT `src/protocols/santa_fe.rs`", comes from that fork, and KAT's module is
ported from FoxFW's C. KAT registers it because FoxFW does (owner's decision, 2026-11-07). No capture of the remote exists
in any reference or in KAT's corpus.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 125 µs | te_delta 100 µs |
| Long | 375 µs | |
| Sync | HIGH 375 + LOW 12000 µs (±1800) | |
| Bit `1` | HIGH 375 µs | the LOW after it (125 or 375 µs) carries nothing |
| Bit `0` | HIGH 125 µs | |

The frame is reported on the LOW of the 80th pair when the CRC holds (bits past 80 are ignored).

## Frame Layout (10 bytes, MSB first)

| Bytes | Field |
|-------|-------|
| 0..3 | rolling (32 bits) |
| 4..6 | **serial** (24 bits) |
| 7 | **counter** (8 bits) |
| 8 | **button** |
| 9 | CRC-8 (poly 0x31, init 0xFF) over bytes 0..8 |

`DecodedSignal.data` = bytes 0..7 (the C's `generic.data`), `extra` = `button << 8 | crc`, `data_count_bit` = 80.

## Key file

`Protocol: SantaFe 13-16`, `Bit: 80`, `Key` (bytes 0..7), then `Serial`, `Btn`, `Cnt` and `Rolling` (decimal). KAT loads
a file as FoxFW's encoder (`subghz_protocol_encoder_santa_fe_deserialize`) reads it, into the frame that encoder sends:

| Field | From |
|-------|------|
| rolling | `Rolling`, else Key bits 63..32 |
| serial | `Serial` (its low 24 bits), else Key bits 31..8 |
| counter | `Cnt` (its low 8 bits), else Key bits 7..0 |
| button | `Btn` (its low 8 bits), else 0 (the encoder's zeroed `generic.btn`) |
| CRC | recomputed |

Any 8-byte Key at exactly 80 bits loads. FoxFW's decoder `deserialize` takes serial and counter from the Key and reads
no button; on every file FoxFW writes the two agree. The C finds `Rolling` only after the `Key` line, where FoxFW
writes it; KAT reads it anywhere in the file.

## Transmit

`subghz_protocol_encoder_santa_fe_deserialize` + `sf_build_upload`: the frame with the button asked for and a fresh
CRC, **3 copies** (`ppa_encoder_read_repeat`'s default) of:

| Part | Pulses |
|------|--------|
| Sync | HIGH 375, LOW 12000 µs |
| Data | 80 bits MSB first: `1` = HIGH 375 + LOW 125, `0` = HIGH 125 + LOW 375 |
| End | LOW 15000 µs (a pulse of its own after the last bit's LOW, as the C yields it) |

The C sends the file's `Btn` and keeps the rolling word, serial and counter: nothing advances, so a transmission
leaves the capture as it is. KAT's **TX Lock / Unlock / Trunk / Panic send buttons 0x01 / 0x02 / 0x04 / 0x08**, the
codes Flipper-ARF's `auto_rke_protocols.c` names (`SFE_BTN_LOCK`, `_UNLOCK`, `_TRUNK`, `_PANIC`); "Send next code"
sends the capture's own button. A `Repeat` line in a key file is not applied (FoxFW deletes it before saving a file).

## RF

- **Encoding:** PWM (pulse width)
- **RF modulation:** AM (AM650)
- **Encryption:** CRC-8 only; the rolling word is not decrypted

## Validation

- No capture exists in the corpus.
- Transmit: KAT against FoxFW's encoder on the host oracle (`subghz_oracle_foxfwenc encode`) for 110 made-up key files
  (FoxFW's layout with buttons 1, 2, 4, 8 and 0..255; `Serial`, `Cnt`, `Rolling` and `Btn` lines that differ from
  the Key or exceed their field; files without `Btn` or without any line; 5 with 79, 81, 64 or 0 bits or a short
  Key): the 105 the C transmits are equal pulse for pulse, and both refuse the other 5. Sent as the app sends it
  (twice, 20 ms apart, equal levels merged), every one of the 105 decodes in KAT's registry and in FoxFW's decoder
  to the frame the C's own transmission decodes to.
- The unit tests keep the oracle's transmission of FoxFW's key file of a decode and check the button mapping, the
  key-file rules and the registry round trip.
- Synthetic frames (made-up values) decode identically in the FoxFW C decoder (host oracle) and in KAT, including the
  rolling word, serial, counter and button. A wrong CRC, 79 bits and a 9 ms sync are rejected by both.
- No change to any file of IMPORTS or the Flipper unit-test corpus.
