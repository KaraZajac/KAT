---
layout: default
---

# Security+ 1.0 Protocol

**Rust module:** `src/protocols/secplus_v1.rs`
**Reference:** the decoder is doctor-firmware's `lib/subghz/protocols/secplus_v1.c` as of 2026-11 (98d1fc1), which
reads a symbol by its period and pairs the packets anew for every frame. The frame and the encoder are those of
`REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/secplus_v1.c` (Flipper-ARF's garage app), the
same in every reference; Flipper-ARF's `lib/subghz/protocols/secplus_v1.c` adds a D-pad button switch to the
encoder, which KAT also offers. The garage app, Flipper-ARF's `lib`, Momentum, RogueMaster and FoxFW have the older
decoder (see Decoder Steps).
Background: argilo/secplus, rtl_433 `secplus_v1.c`.

## Overview

Chamberlain / LiftMaster / Craftsman garage door openers with Security+ (1.0) remotes and keypads. A press sends
two packets of 21 ternary symbols; together they carry a 20-trit fixed code (remote id + button) and a 20-trit
rolling code. There is no checksum and no cipher: the references decode it and transmit the next code.

## Timing

| Parameter | Value     | Notes                                                    |
|-----------|-----------|----------------------------------------------------------|
| te_short  | 500 µs    | ±100 µs (strict `<`; first HIGH of packet 1)             |
| te_long   | 1500 µs   | ±100 µs (first HIGH of packet 2)                         |
| Symbol    | LOW + HIGH = 2000 µs | the period within ±300 µs (1701..2299), each part at least 250 µs |
| Symbol 0  | LOW 1500 + HIGH 500  | HIGH under 750 µs                             |
| Symbol 1  | LOW 1000 + HIGH 1000 | HIGH 750..1249 µs                             |
| Symbol 2  | LOW 500 + HIGH 1500  | HIGH 1250 µs or more                          |
| Gap       | 60000 µs  | ±12000 µs (te_short × 120 ± te_delta × 120), before each packet |

## Frame Layout

Packet 1 starts with symbol `0` (its HIGH 500 right after the gap), packet 2 with symbol `2` (HIGH 1500). The 20
symbols after each header alternate a rolling digit and a fixed digit coded as `(digit + acc) % 3`, where `acc` is the
running sum of both digits in that packet. The decoder (`subghz_protocol_secplus_v1_decode`):

- `rolling` = the 20 rolling digits (packet 1 then packet 2) as a base-3 number, **bit-reversed over 32 bits**
- `fixed` = the 20 fixed digits as a base-3 number (< 3^20)
- `data = fixed << 32 | rolling`, `Bit: 42` (the references' bit count for a pair)

Fields (`get_string`): `button = fixed % 3` (0 middle, 1 left, 2 right), `id0 = fixed / 3 % 3`, `id1 = fixed / 9 % 3`;
`serial = fixed / 27 % 2187` when `id1 == 0` (keypad; its PIN `fixed / 3^10 % 3^9` is display-only), else
`fixed / 27`; counter = the 32-bit `rolling` (KAT's `counter` holds its low 16 bits, `data` all 32).

## RF

- **Encoding:** ternary pulse-width symbols (KAT: PWM)
- **RF modulation:** AM (OOK); flags 315 MHz (doctor-firmware: 315 + 433 MHz); KAT decodes on every band
- **Encryption:** rolling code, no cipher, no checksum (`crc_valid` is always false)
- **Make:** Chamberlain / LiftMaster

## Decoder Steps

1. **Reset** — a LOW of 60000 ± 12000 µs arms the decoder and clears both packets.
2. **SearchStartBit** — HIGH 500 → packet 1, HIGH 1500 → packet 2 (symbol 0 of the packet); anything else → Reset.
   A packet's first symbol unmarks the packet of its kind that was kept, so a packet that breaks off cannot pair
   an older one.
3. **SaveDuration / DecoderData** — each LOW + HIGH pair is one symbol, read by its period: the two parts together
   within 2000 ± 300 µs, each at least 250 µs, and the HIGH says which symbol (table above). Anything else → Reset.
   A gap after exactly 21 symbols marks that packet as received; once both are marked the 40 symbols are decoded
   and the decode is reported.
4. After a frame both packets are forgotten: the next frame needs a packet 1 and a packet 2 again, in either order.
   A 3-copy upload gives 2 decodes, 3 with a trailing gap (the last packet counts only when a gap follows it). The
   C's `reset` is empty ("you need to get 2 parts of the package"), and so is KAT's.

**The older decoder** (doctor-firmware before 2026-11, and still the garage app, Flipper-ARF's `lib`, Momentum,
RogueMaster and FoxFW; KAT up to 1.12.0) took each part of a symbol by its own window (LOW 1500 ± 300 + HIGH
500 ± 100, LOW 1000 ± 200 + HIGH 1000 ± 200, LOW 500 ± 100 + HIGH 1500 ± 300) and kept both packets marked after
a frame, so every further packet gave another frame with the packet before it, whichever press that one came
from: a 3-copy upload with a trailing gap gave 5 decodes. On Flipper's `security_pls_1_0_raw.sub` it reads 25
frames of 8 codes; the current one reads 20 frames of 10 codes, the two more with counters that fit between their
neighbours.

## Encoder

`subghz_protocol_encoder_secplus_v1_deserialize` → `_encode` → `_get_upload`:

1. Button (Flipper-ARF lib D-pad, KAT's native code 0/1/2): replaces the lowest base-3 digit of `fixed`; any other
   value keeps the captured button.
2. Counter: `rolling + 1` (wrapping — the C's `> 0xFFFFFFFF` test can never fire on a `uint32_t`), then at least
   `0xE6000000` (the references' counter jump).
3. `fixed` must be ≤ 0xCFD41B90 (3^20 − 1), else nothing is sent (`encoder_capable` false).
4. `rolling` is bit-reversed and split into 20 trits (a reversed value ≥ 3^20 loses its top part, as in the C),
   symbols rebuilt with the `acc` coding.
5. Upload: LOW 59500 + HIGH 500, 20 symbols (packet 1); LOW 58000 + HIGH 1500, 20 symbols (packet 2); repeated
   `encoder.repeat` = 3 times (252 entries).

**Counter mode.** The garage app's TX plugin (`tx_secplus_v1` in `application.fam`) is built without
`SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS`, so `subghz_garage_get_rolling_counter_mult()` returns −0x7FFFFFFF and the encoder
runs the "OFEX" overflow mode: counters ≥ 0xE6000000 jump to 0xFFFFFFFE, lower ones +1 without the floor. KAT
follows the source's normal mode (multiplier 1), the behaviour of doctor-firmware, of Flipper-ARF's own firmware and
of the garage app's main build flags; both modes give the same next code for 0xE5FFFFFF and 0xFFFFFFFE.

## Key file

`subghz_block_generic_serialize`: `Protocol: Security+ 1.0`, `Bit: 42`, `Key: <data>` (doctor-firmware also writes
`Serial`/`Btn`/`Cnt` after `get_string`). Loading needs `Bit: 42`.

## Validation

C oracles (`REFERENCES/tools/oracle/bin/subghz_oracle_*`: doctor, garage = Flipper-ARF garage app with its plugin
cdefines, arf = Flipper-ARF lib, momentum):

- **Decoder (doctor-firmware 98d1fc1):** the same decodes, key for key and count for count, as doctor's C on 1200
  fuzz files (packets of random symbols in every order: pairs, lone packets, doubled packets, 3-copy uploads;
  timing scaled ±20 %, the duty of each symbol skewed by up to 400 µs, gaps on both sides of the window's edges,
  short and long packets; jitter, dropped, split and stretched pulses; 262 decodes in 165 files) and on 262 real RAW
  files (IMPORTS, the unit-test folders, rtl_433's recordings; 22 decodes in 2 files). On the same fuzz files the
  older decoder reads 179 (Momentum's and the garage app's C; KAT 1.12.0 was identical to them).
- **Frame and fields (older decoder, 1.9.0):** identical decodes (key and fields) to doctor and garage on all 57 RAW
  unit-test captures, 1136 UberGuidoZ garage/gate RAW files, the rtl_433 IQ bursts and 4786 fuzz files (jitter
  10/25/40 %, dropped and inserted pulses, noise, other protocols' captures, random streams; 1248 decodes).
- **Encoder:** pulse-exact with doctor and Flipper-ARF lib on 235 keys (unit-test key + random fixed/rolling incl. the
  counter edges); 6 keys with `fixed` > 3^20 − 1 are refused by both; pulse-exact with the garage build on 60 keys
  where its OFEX counter step agrees with the normal one.
- **Real captures:** `security_pls_1_0_raw.sub` (unit test) → 20 decodes of 10 codes, as doctor's C; rtl_433
  `secplus/v1/g008_315.05M_250k.cu8` → fixed 0x6E163421, rolling 0x8A51C798, id 0x413C90A (68405514), button 1
  (left), id0 0, id1 2 — all as rtl_433's JSON (KAT and doctor's C report 3 decodes for its 4 messages: the
  first packet has no gap before it and the last one no gap of the right length after it, which leaves six packets;
  the older decoder reports 5). rtl_433's `LiftMaster_4330E`
  recordings are a different Chamberlain format (≈885 µs symbols, 33–35 ms gaps) that no reference decodes.
- No decode on any other protocol's unit-test file (except `test_random_raw.sub`, which contains these frames) and none
  on IMPORTS (golden test unchanged).
