---
layout: default
---

# Faac SLH Protocol

**Rust module:** `src/protocols/faac_slh.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/faac_slh.c` (primary) and
doctor-firmware `lib/subghz/protocols/faac_slh.c` — same decoding, display and upload (both carry the Unleashed
"Genius" `Manufacture` mechanism). Flipper-ARF's `lib/subghz/protocols` copy is older (no `Manufacture`).

## Overview

FAAC "SLH" gate remotes: 64-bit KeeLoq frames whose cipher key is derived from the manufacturer key (keystore type 5,
"FAAC_SLH") and a 32-bit seed unique to each remote. The seed is not in a normal frame; a master remote put in
programming mode sends it in a special frame.

A received frame therefore cannot be checked: like the C, every 64-bit frame in this air format is reported
(`crc_valid` = false). A normal frame shows serial and button ("Sd:Unknown", no counter); a programming frame shows
the seed and its "mCnt". From a key file with a `Seed` the hop is decrypted and the counter shown.

## Timing

| Parameter | Value    | Notes                                                    |
|-----------|----------|----------------------------------------------------------|
| Short     | 255 µs   | ± 100 µs                                                 |
| Long      | 595 µs   | ± 100 µs                                                 |
| Preamble  | 1190/1190 | HIGH + LOW, each ± 300 µs                               |
| Bits      | 64       | 0 = HIGH 255 + LOW 595, 1 = HIGH 595 + LOW 255           |
| End       | ≥ 865 µs | HIGH (the next preamble)                                 |

## Frame Layout (64 bits)

- **fix (32)** = serial (28) << 4 | button (4)
- **hop (32)**: KeeLoq, key = FAAC learning of the seed with the manufacturer key
  (`encrypt(seed) << 32 | encrypt(seed[31:16] << 16 | 0x544D)`)
- **Hop plaintext**: three fix nibbles << 20 | counter (20 bits). Even counter: fix[7:4], fix[3:0], fix[11:8]
  (serial nibble 0, button, serial nibble 1); odd counter: fix[23:20], fix[19:16], fix[15:12].
- **Programming frame**: fix[31:16] = 0x520F and hop[7:0] = 0. The seed: bytes hop[23:16], hop[31:24], fix[7:0],
  fix[15:8] as a little-endian word, rotated right by (hop[15:8] & 0xF), each byte XORed with hop[15:8] (mCnt).

KAT fields: `data` = the key, serial (28 bits), button (4 bits); `extra` = the seed when known (bits 0-31; bit 32 =
`AllowZeroSeed`), None otherwise; counter = the low 16 of the 20 bits (seeded keys only). Programming frames: serial
and button None, counter = mCnt, `extra` = the seed from the frame.

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK), 433.92 + 868 MHz in the C's flags; KAT decodes on every band
- **Encryption:** KeeLoq (FAAC seed learning)

## Decoder Steps

1. **Reset** — a HIGH 1190 ± 300 µs.
2. **FoundPreambula** — a LOW 1190 ± 300 µs starts the bits; anything else goes back to Reset.
3. **SaveDuration** — a HIGH of 865 µs or more ends the frame (exactly 64 bits are reported) and is taken as the next
   preamble; a shorter HIGH is a bit's first half.
4. **CheckDuration** — classify the bit; anything else goes back to Reset.

After a decode the registry resets every decoder (the Flipper resets its receiver too), so of back-to-back frames —
whose end HIGH is the next preamble — every other one is reported, exactly like the C oracle.

## Encoder

`..._deserialize` + `gen_data` + `get_upload` (the C's cross-file statics `temp_fix_backup` / `temp_counter_backup`
start from this key alone):

- **seed known** (`Seed` ≠ 0 or `AllowZeroSeed`): counter + 1 (0 past 0xFFFFF), button = the requested native code
  (4 bits), plaintext rebuilt for the new counter's parity, hop re-encrypted;
- **seed unknown**: the captured key is sent unchanged ("send data from buffer") — unless another button is requested,
  which is the C's button override: it builds a frame under seed 0 with the counter not advanced;
- **native code 0x10** ("Prog mode", the C's custom UP button) with a known seed: a programming frame for the seed with
  counter + 1 (programming frame: mCnt + 1);
- **programming frame**: sent unchanged — unless its key file names a remote (doctor-firmware's `Serial`/`Btn` lines,
  which doctor's decoder fills with the remote decoded just before the programming frame): then, as doctor-firmware
  does, that remote's frame under the learnt seed with counter mCnt + 1. The garage app reads no such lines and always
  repeats the programming frame; KAT takes doctor's (larger) behaviour. Another button requested on a programming
  frame is the C's override: a normal frame for that serial (0 when unknown).

Upload: HIGH 1190, LOW 1190, 64 bits, no trailing gap; `encoder.repeat` = 3.

Flipper-ARF's garage TX plugin (`tx_faac_slh`) is built without `SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS`, whose fallback
takes the "OFEX" counter path (the counter jumps to 0, or 0xFFFFF from 0), and without custom buttons (no programming
frames). KAT follows the source's standard path, which is doctor-firmware's behaviour.

## Key file

`Protocol: Faac SLH`, `Bit: 64` (required), `Key`, `Seed` (4 hex bytes, required), optional `AllowZeroSeed` (any
value: seed 0 is a real seed) and `Manufacture` (selects the type-5 key; KAT's keystore has only FAAC_SLH, which is
also the C's fallback). On a programming frame, doctor-firmware's `Serial`/`Btn` lines name the remote (see Encoder).
KAT writes `Seed` from `extra` — 00 00 00 00 for a received frame, as the C's decoder serialize does — and
`AllowZeroSeed: true` when set; for a programming frame naming a remote also doctor's `Serial`, `Btn`, `Cnt` lines
(before `Seed`). A `Manufacture` line is not carried (FAAC_SLH is the default).

## Validation

Flipper's `faac_slh_raw.sub` decodes 8 times (serial 0xA03A052, buttons 8 and 9); `test_random_raw.sub` contains the
same remote once. C oracle (doctor and garage): decode parity on the unit-test captures (57 files, 9 frames), IMPORTS
(108), UberGuidoZ (1171) and 4200 fuzz files (691 frames). Encoder: 300 random key files (seeded, `AllowZeroSeed`,
programming frames, seedless), the 304 key files the C serialized from those decodes, and 134 programming keys with
`Serial`/`Btn` lines (incl. 8-bit `Btn` values): KAT equals doctor-firmware pulse for pulse; the garage oracle differs
by the OFEX counter (TX plugin flags) and by repeating programming keys. KAT loads every C-written key file with the
C's fields and writes the garage's lines exactly. The button-override and UP paths (not reachable from the oracle's
CLI) are covered by unit tests.
