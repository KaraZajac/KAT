---
layout: default
---

# MARELLI (Flipper-ARF key files)

**Rust module:** `src/protocols/fiat_marelli.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/fiat_marelli.c` (the same file is in FoxFW, without its
Decodable flag)

## Overview

Flipper-ARF registers a "MARELLI" protocol for Fiat Magneti Marelli BSI remotes (PCF7946, Fiat Panda and
Grande Punto era). It is ProtoPirate's Fiat V1 decoder from before ProtoPirate rewrote it; KAT follows the
rewrite as [Fiat V1](fiat_v1.md). KAT **loads, saves and transmits MARELLI key files** a Flipper-ARF user has
saved, exactly as Flipper-ARF does, and **decodes nothing** as MARELLI, as FoxFW.

## Why nothing decodes it

FoxFW took the Decodable flag away on purpose (`fiat_marelli.c`, v2.0.14): Delphi BSI and Magneti Marelli BSI
remotes use the same Hitag2/PCF7946 transponder, so there is no separate Marelli format on the air, and the
decoder "only ever latched garbage 102-bit frames (misframed, wrong serial) onto Fiat V1 (Delphi) and Renault
captures"; Load, Save and Send stay so that saved MARELLI files still open and transmit.

KAT's captures show the same. Run on its own over all 256 test captures, Flipper-ARF's decoder reads 6 frames
in 3 files, and every one is a frame that KAT, ProtoPirate, doctor-firmware and FoxFW read as Fiat V1
(`IMPORTS/FIAT/Fiat Panda.sub`, `IMPORTS/OPEL/Opel Vectra 2008.sub`) or Ford V3 (the 2022 Ford Escape),
read two bits early. On the Opel Vectra its serial `0x1A3D7494` is Fiat V1's UID `0x468F5D25` shifted left
by two, and its counter is not the remote's. So KAT registers MARELLI only so that its key files resolve:
its decoder never reports a frame.

## Key file

What Flipper-ARF's `serialize` writes (`deserialize` reads `Bit`, `Key` and, when present, `Extra` and `TE`):

| Line | Value |
|------|-------|
| `Protocol` | `MARELLI` |
| `Bit` | 80..104 (the decoder's frame length) |
| `Key` | frame bits 0..63 |
| `Extra` | the frame bits after the 64th, as a 32-bit number: of a frame longer than 96 bits, only the last 32 |
| `Extra_bits` | `Bit` − 64 (written, not read) |
| `TE` | the half-bit in µs, measured on the preamble (0 means 260) |
| `Serial`, `Btn`, `Cnt` | written, not read: re-derived from `Key` |

Fields, as Flipper-ARF shows them: serial = frame bytes 2..5, button = byte 6's high nibble (0x7 Lock, 0xB
Unlock, 0xD Trunk), counter = byte 7 >> 3 (5 bits). The frame's CRC-8 (poly 0x01, init 0x03, bytes 0..11 →
byte 12) exists only for 104-bit frames, and no file holds byte 12, so it cannot be checked: KAT shows "none"
(Flipper-ARF shows "CRC:FAIL" for every 104-bit file).

## Transmit

Replay only, as Flipper-ARF: the rolling part is encrypted, so the encoder keeps the captured button and
counter, and KAT's Lock, Unlock, Trunk and Panic all send the saved frame. Three copies of:

| Part | Pulses |
|------|--------|
| Preamble | 100 HIGH / LOW pairs at TE, the last without its LOW |
| Gap | LOW 13 × TE |
| Sync | HIGH 8 × TE |
| Data | the `Bit` frame bits, Manchester from the Mid1 state: from Mid1, 1 = LOW TE + HIGH TE and 0 = LOW 2 TE; from Mid0, 1 = HIGH 2 TE and 0 = HIGH TE + LOW TE |
| End | LOW 37 × TE (in place of the last LOW when the frame ends in Mid0) |

A file outside 80..104 bits does not transmit (Flipper-ARF's encoder refuses it). Without its own preset a
capture goes out as FM476 (Flipper-ARF flags MARELLI FM).

### Frames longer than 96 bits

They do not survive the key file. `Extra` keeps the last 32 extra bits only, and the encoder's rebuild loop
(frame bits 64..95) tests `extra & (1UL << (extra_bits - 1 - i))`: for the first `Bit − 96` bits that is a
shift by 32 or more, which C leaves undefined. The Flipper's Cortex-M4 shifts a register by 32 or more to 0,
so those bits go out as 0, and so do bits 96 and up, which the loop never reaches (a 104-bit frame's byte 12
is recomputed as the CRC of the rebuilt bytes). KAT sends what a Flipper sends.

## Validation

- KAT's transmission against Flipper-ARF's encoder on the host oracle (`subghz_oracle_arf encode`) for 150
  generated key files, every bit count from 80 to 104 and TE values 0 to 300 µs: 129 equal pulse for pulse
  (all files of 96 bits or fewer, and the longer ones whose `Extra` has the affected low bits clear). The
  other 21 differ only by the shift above: the x86 host build masks the shift count to 5 bits and so puts
  `Extra`'s low bits where a Flipper puts 0. A model of the encoder reproduces the oracle on all 150 files with
  x86 shifts and KAT on all 150 with the Cortex-M4's.
- FoxFW's own encoder (the SD plugin `enc_fiat_marelli.c`, run on the same host build with its plugins compiled
  in) gives exactly Flipper-ARF's transmission on all 150 files.
- The unit tests keep the oracle's upload of an 88-bit file, the long-frame rebuild, the key-file round trip
  (Flipper-ARF's line order) and the registry load.
- Not one capture or key file in `IMPORTS` or the firmwares' unit tests decodes differently.

## RF

- **Encoding:** Manchester
- **RF modulation:** FM (Flipper-ARF's flag), 433.92 MHz
- **Encryption:** Hitag2 rolling code: replay of saved frames only
