---
layout: default
---

# Fiat V1 Protocol (Magneti Marelli BCM / PCF7946, HITAG2)

**Rust module:** `src/protocols/fiat_v1.rs` (cipher: `src/protocols/hitag2.rs`)
**Reference:** `REFERENCES/ProtoPirate/protocols/fiat_v1.c` (current version — rewritten upstream in
62b31bf "Fix Fiat V1 decoder and add HITAG2 key TX support" and 1287d10 "fix Fiat V1 encoder"; later
commits only drop `furi_check`s). Also compared: Flipper-ARF `lib/subghz/protocols/fiat_v1.c` + its
"Hitag2 BF" cascade (`applications/main/subghz/helpers/subghz_hitag2_bf*.c`), and doctor-firmware
`lib/subghz/protocols/fiat_v1.c`.

## Overview

Fiat V1 is the Magneti Marelli BCM keyfob protocol (PCF7946 transponder), found on Fiat Panda, Grande
Punto and other Fiat/Lancia/Alfa models of ~2003-2012. Each press sends a 104-bit Manchester frame
carrying the 32-bit transponder UID, a 4-bit button, a 10-bit counter ("control") and a 32-bit
HITAG2 authenticator ("hop"), protected by an XOR byte. It is a different protocol from Fiat V0.

KAT follows the current ProtoPirate decoder: no sync detection at all, both TE variants decoded in
parallel from half-bit cells, strict frame validation, and verification of the hop against HITAG2
keys. All three references share this decoder (ARF's and doctor-firmware's files are ProtoPirate's
with firmware glue). What they add, and KAT therefore also does:

| Capability | ProtoPirate | Flipper-ARF | doctor-firmware | KAT |
|---|---|---|---|---|
| Embedded keys | 9 | 8 (PP's first 8) | 9 | 9 |
| Typed-in "Hitag2 Key" (+ epoch) | yes | yes (from its BF scene) | yes | no key entry |
| Key dictionary + heuristics (L2/L4) | — | on demand (BF scene) | — | automatic, every frame |
| Hitag2Hell key recovery (L5) | — | on demand, needs 2 captures for a unique key | — | on demand: **Recover key (HiTag2)**, 2 captures ([HiTag2 key recovery](hitag2_hell.md)) |
| UID → key database | — | yes | yes | yes (`hitag2_keys.txt`, same format) |
| TX with a key | fresh frame | fresh frame | fresh frame | fresh frame |
| TX without a key | refuses | byte-identical replay | replay ("Hitag2 replay only") | byte-identical replay |

## Timing

Two variants with the same frame, decoded simultaneously:

| Variant | Short | Long | Tolerance | Encoder lead (HIGH) | Encoder gap (LOW) | Example |
|---------|-------|------|-----------|---------------------|-------------------|---------|
| A | 250 µs | 500 µs | ±100 µs | 2033 µs | 3252 µs | Panda |
| B | 100 µs | 200 µs | ±50 µs | 850 µs | 1230 µs | Grande Punto |

A LOW of ≥ 800 µs that is not a data pulse ends a frame (its first half-bit is counted as one LOW cell).

## Frame Layout (104 wire bits = 13 bytes, MSB first)

| Byte(s) | Content |
|---------|---------|
| 0-1 | `00 01` (sync / family) |
| 2-5 | UID (32 bits) |
| 6 | button (4) ‖ control[9:6] |
| 7 | control[5:0] ‖ hop[31:30] |
| 8-10 | hop[29:6] |
| 11 | hop[5:0] ‖ tail (2 bits, usually `10`) |
| 12 | XOR = `0x01 ^ byte0 ^ … ^ byte11` |

ProtoPirate reports 102 "logical" bits (the frame without the 2 tail bits); KAT reports the 104 wire bits.

**Buttons:** `0x1` Close, `0x2` Trunk, `0x4` Lock, `0x8` Unlock (KAT shows the raw Fiat code).

**Authenticator:** `hop = HITAG2(key, uid, IV)` with register `uid ‖ key[4] key[5]`, injected key bits
`key[0..4]` and `IV = epoch(18) ‖ control(10) ‖ button(4)` (`fiat_v1_bcm_generate_authenticator`; the
embedded keys use epoch 0). See `hitag2.rs` for the shared cipher core.

### KAT `DecodedSignal` packing

| Field | Value |
|-------|-------|
| `serial` | UID |
| `button` | Fiat button code (1/2/4/8) |
| `counter` | control (10 bits) |
| `crc_valid` | XOR verified (always true on emission) |
| `data` | wire bits 0-63 = bytes 0-7 (top 16 bits are `0001`) |
| `extra` | wire bits 64-103 = bytes 8-12, right-aligned, plus the TE variant at bit 40 (0 = A, 1 = B) |
| `data_count_bit` | 104 |
| `encoder_capable` | always true (fresh frames with a key, replay without) |

## RF

- **Encoding:** Manchester (bit 1 = HIGH then LOW)
- **RF modulation:** AM/FM — ProtoPirate flags AM, Flipper-ARF and doctor-firmware AM | FM (the union; every
  capture so far is OOK, and KAT transmits AM650)
- **Encryption:** HITAG2 (32-bit authenticator, 48-bit key)
- **Frequencies:** 315 MHz, 433.92 MHz (ProtoPirate flags 315 | 433)

## Decoder Steps

1. **Reset** — wait for any pulse that is short or long for either variant; clear both windows, enter Data.
2. **Data** — for each variant independently: a short pulse appends one cell (= the pulse level), a long
   pulse two cells, to a sliding 208-cell window; any other pulse clears that variant's window (a LOW
   ≥ 800 µs first appends one LOW cell). If no variant matched, return to Reset.
3. **Window read** — every time a window holds 208 cells it is read as 104 cell pairs; each pair must be
   a transition (`first != second`), bit = `first`. If that is not a valid frame, the inverted read is tried.
4. **Validate** (`fiat_v1_frame_valid`) — bytes 0-1 = `00 01`, XOR byte, button ∈ {1, 2, 4, 8}, UID not
   `00000000`/`FFFFFFFF`. A frame identical to the previous one is suppressed until `reset()` (KAT's
   registry resets every decoder after a hit, so each repeat is reported).

## Encoder

**Key resolution** (`resolve_key`, run by `encode()`): (a) ProtoPirate: each of the 9 embedded keys
(epoch 0), for the frame's button, then for buttons 1/2/4/8; then the key saved for the UID in
`hitag2_keys.txt` (ARF's `hitag2_key_db_find` lookup after its L1; e.g. one **Recover key (HiTag2)**
found), with its epoch, the frame's button or any of 1/2/4/8; (b) Flipper-ARF L2: its 95-entry flash
dictionary (ASCII/default keys, mutations of the known keys; its L3 SD file `assets/hitag2` repeats
L1+L2), epoch 0, the frame's button; (c) ARF L4: `(UID ^ mask) ‖ tail` for 12 masks × 20 tails, each of
ARF's 8 known keys with its last byte ±1..±16, and those 8 keys with epoch 1..7. The first match is
ProtoPirate's "KEY:OK"; ≈1000 authenticators, well under 1 ms in a release build. A random hop matches
by chance with probability ≈ 2^-20. (ARF runs L2-L4 only when the user starts its "Hitag2 BF" scene;
KAT runs them on every transmit.)

**With a key** (ProtoPirate's encoder + emulate plugin; ARF and doctor-firmware build the same frame), a
**fresh** frame is built:

- **Button:** KAT Lock → `0x4`, Unlock → `0x8`, Trunk → `0x2`; Panic (no Fiat V1 code) keeps the captured button.
- **Counter:** captured control + 1, masked to 10 bits (ProtoPirate's emulate plugin increments `Cnt` per
  press; ARF/doctor add the rolling multiplier, 1).
- **Hop:** `fiat_bcm_authenticator(uid, button, control, key, epoch)` with the resolved key and epoch; tail
  bits are kept from the capture.

**Without a key** ProtoPirate refuses; Flipper-ARF replays the captured 13-byte frame byte-for-byte
(`TX(replay)`, any button), and doctor-firmware does the same under its "Hitag2 replay only" setting. KAT
replays. ARF always uses variant-A timing for its upload; KAT keeps the captured variant, as
doctor-firmware does (identical for variant-A captures such as the Panda).

**Waveform** (all paths): lead HIGH (2033 µs A / 850 µs B), 104 Manchester bits at the variant's short TE,
LOW gap (3252 µs A / 1230 µs B), adjacent levels merged — repeated 6 times (all three references).

**Hitag2Hell (ARF L5)** is the signal menu's **Recover key (HiTag2)**, ported from qDoctor's C
([HiTag2 key recovery](hitag2_hell.md)): a guess-and-determine attack (Verstegen et al., 2018) that
enumerates the internal states consistent with one 32-bit hop. One frame leaves about 2^16 equally
valid keys, so KAT pairs the capture with a **second press of the same remote** (same UID, another
hop) from the captures list; ARF scans the saved capture's folder for such siblings and, with one
capture only, accepts the first consistent key (wrong with probability ≈ 1 − 2^-16), which KAT does
not. The key is saved for the UID in `hitag2_keys.txt` in KAT's config folder, the file ARF and
doctor-firmware keep in `subghz/assets/` (same format and byte order, so it can be copied either way),
and the remote's captures then transmit fresh frames. The Panda capture contains a single frame, so
its key cannot be recovered.

**Not ported:** *typed-in keys* (doctor-firmware's "Hitag2 Key" entry, and the `Hitag2 Key` line of
a loaded file): KAT has no key entry; a key can be added to `hitag2_keys.txt` by hand instead.

## Validation

- `IMPORTS/FIAT/Fiat Panda.sub` decodes (variant A, UID `EFB82D66`, button `0x2` Trunk, control `0x08D`,
  hop `299BD53E`, 7 repeats, all the same frame). Its hop is signed by none of ProtoPirate's keys
  (`KEY:??`) nor any ARF L2/L4 candidate, and it is not a Renault serial-derived-key frame either (the
  Renault V1 seed search reports not found). ProtoPirate cannot transmit it; KAT, like ARF, replays it.
- `IMPORTS/VAG/Test_55_unlock_and_55_lock_suran.sub` (a VW Suran) was decoded twice as Fiat V1 by the
  old, checksum-less decoder; both were false matches and it no longer decodes as Fiat V1.
- `IMPORTS/FIAT/Fiat Bravo 2010 S.Croce 433.88 am650.sub` still decodes as nothing: its ~175-200 µs /
  ~350-390 µs Manchester with 6.1 ms gaps fits neither variant.
- The decoder was compared pulse-for-pulse with ProtoPirate's `fiat_v1.c` compiled on the host (same
  decodes at the same pulse on every IMPORTS capture, both polarities).
- C oracles built from the unmodified reference files: key resolution equals ARF's cascade (L1-L4) on
  382 frames (signed with every L1/L2 key, sampled L4 keys incl. epochs 1-7, random hops, the Panda)
  except, as intended, frames signed with ProtoPirate's 9th key (absent from ARF). The encoder equals
  ProtoPirate on 60 keyed frames (both variants, all buttons), ARF on 263 frames whose key only ARF's
  cascade finds, ARF's replay on 35 variant-A frames and doctor-firmware's replay on 61 frames (both
  variants), pulse for pulse.
- Unit tests cover the C upload for both variants, synthetic and inverted frames, duplicate suppression,
  the gate (XOR, sync, button, UID), ARF cascade keys (dictionary, three heuristics, epoch), the key-less
  replay, the encode → decode round trip, and the two captures above.
