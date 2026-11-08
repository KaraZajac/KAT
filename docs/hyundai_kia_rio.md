---
layout: default
---

# Hyundai/Kia RIO Protocol

**Rust module:** `src/protocols/hyundai_kia_rio.rs`
**Reference:** FoxFW `lib/subghz/protocols/hyundai_kia_rio.c` (`HYUNDAI_KIA_RIO_PROTOCOL_NAME = "Hyundai/Kia RIO"`) and
its SD-card encoder plugin `applications/main/subghz/protocols_ext/enc_hyundai_kia_rio.c` (with the `pp_auto_common.c`
helpers the firmware hands it).

## Overview

A 64-bit fixed-code frame with a 16-bit checksum. The reference flags it `SubGhzProtocolTypeStatic`, `433 | AM`.
KAT decodes it and transmits it as FoxFW's encoder does.

**Provenance.** The format comes from Flipper-ARF's `lib/subghz/protocols/auto_rke_protocols.c` ("Additional automotive
RKE protocols — ported from Pandora DXL 5000 firmware": its Hyundai/KiaRIO, ID 0x11, early Hyundai/Kia fobs: Accent, Rio, Elantra ~2001-2008), a reverse-engineering file ARF never registers, like its
other Pandora-derived decoders (Audi, Hyundai, Mazda Infinity, which KAT keeps unregistered). A KAT fork (limbo111111/KAT,
written by Google's Jules bot in May 2026) ported it to Rust, "aligned with Flipper-ARF reference: `auto_rke_protocols.c`".
FoxFW's C, whose header says "ported from KAT `src/protocols/hyundai_kia_rio.rs`", comes from that fork, and KAT's module is
ported from FoxFW's C. KAT registers it because FoxFW does (owner's decision, 2026-11-07). No capture of the remote exists
in any reference or in KAT's corpus.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 312 µs | te_delta 150 µs |
| Long | 728 µs | |
| Sync | HIGH 312 + LOW 10400 µs (±1560) | |
| Bit `1` | HIGH 728 µs | the LOW after it (312 or 728 µs) carries nothing |
| Bit `0` | HIGH 312 µs | |

The frame is reported on the LOW of the 64th pair when the checksum holds.

## Frame Layout

`serial` (32) | `mask` (16) | `checksum` (16), MSB first. `checksum = !((serial ^ (serial >> 16)) ^ mask)` (16 bits).
**button** = 1, 2, 4, 8 for masks 0x0100, 0x0200, 0x0400, 0x0800, otherwise `mask >> 8`. No counter.

## Key file

`Protocol: Hyundai/Kia RIO`, `Bit: 64`, `Key`, then `Serial` and `Btn` (decimal). Both C deserializers load any 8-byte
Key at exactly 64 bits. The decoder's derives serial and button from the Key; the encoder's takes `Serial` and `Btn`
(its low 8 bits) over them, rebuilds the frame (serial, mask `Btn << 8`, checksum) and writes that Key back into the
file. KAT loads a key whose checksum holds and whose `Serial` / `Btn` lines (when present) agree with it as it is (a
frame the decoder reports); any other key, a bad checksum included, loads as the frame the encoder rebuilds.

## Transmit

`subghz_protocol_encoder_hyundai_kia_rio_deserialize` + `hkr_build_upload`: the serial, the mask
`hkr_button_to_mask(button)` (`button << 8` for every code) and its checksum, **3 copies**
(`ppa_encoder_read_repeat`'s default) of:

| Part | Pulses |
|------|--------|
| Sync | HIGH 312, LOW 10400 µs |
| Data | 64 bits MSB first: `1` = HIGH 728 + LOW 312, `0` = HIGH 312 + LOW 728 |
| End | LOW 10000 µs (a pulse of its own after the last bit's LOW, as the C yields it) |

A mask with a low byte (it decodes as button `mask >> 8`) goes out as `button << 8`, as the C sends it. There is no
counter, so a transmission leaves the capture as it is. KAT's **TX Lock / Unlock / Trunk / Panic send buttons 0x01 /
0x02 / 0x04 / 0x08**, i.e. the masks 0x0100 / 0x0200 / 0x0400 / 0x0800 that Flipper-ARF's `auto_rke_protocols.c`
names `HKR_BTN_LOCK`, `_UNLOCK`, `_TRUNK` and `_PANIC`; "Send next code" sends the capture's own button. A `Repeat`
line in a key file is not applied (FoxFW deletes it before saving a file).

## RF

- **Encoding:** PWM (pulse width)
- **RF modulation:** AM (AM650)
- **Encryption:** fixed code, 16-bit checksum

## Validation

- No capture exists in the corpus.
- Transmit: KAT against FoxFW's encoder on the host oracle (`subghz_oracle_foxfwenc encode`) for 112 made-up key files
  (FoxFW's layout with the four keys and 42 other masks, low bytes included; bad checksums with and without lines;
  `Serial` / `Btn` lines over the Key, `Btn` past 255; 5 with 63, 65, 0 or 80 bits or a short Key): the 107 the C
  transmits are equal pulse for pulse, and both refuse the other 5. Sent as the app sends it (twice, 20 ms apart,
  equal levels merged), every one of the 107 decodes in KAT's registry and in FoxFW's decoder to the frame the C's
  own transmission decodes to.
- The unit tests keep the oracle's transmission of FoxFW's key file of a decode (the same pulses as for mask
  0x0123) and check the button-to-mask mapping, the key-file rules and the registry round trip.
- Synthetic frames (made-up serials, the four keys, an unlisted mask) decode identically in the FoxFW C decoder (host
  oracle) and in KAT. A wrong checksum, 63 bits and a 7 ms sync are rejected by both.
- No change to any file of IMPORTS or the Flipper unit-test corpus.
