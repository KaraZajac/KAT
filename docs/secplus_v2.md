---
layout: default
---

# Security+ 2.0 Protocol

**Rust module:** `src/protocols/secplus_v2.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/secplus_v2.c` (Flipper-ARF's
garage app, primary; the same logic as Flipper-ARF's `lib/subghz/protocols/secplus_v2.c` behind build shims), plus
doctor-firmware's `lib/subghz/protocols/secplus_v2.c` for the 86-bit keypad frames the garage copy does not handle.
Background: argilo/secplus, rtl_433 `secplus_v2.c`.

## Overview

Chamberlain / LiftMaster / Craftsman openers since 2011 ("yellow learn button"; 310 / 315 / 390 / 433 MHz remotes).
A press sends pairs of Manchester packets; together they carry a 40-bit fixed code (8-bit button + 32-bit remote id)
and a 28-bit rolling counter, scrambled per packet by an order/invert code taken from the counter. No secret key:
the references decode the counter and send the next one.

## Timing

| Parameter   | Value      | Notes                                                        |
|-------------|------------|--------------------------------------------------------------|
| te_short    | 250 µs     | ±110 µs (strict `<`)                                         |
| te_long     | 500 µs     | ±110 µs                                                      |
| Packet end  | ≥ 1110 µs LOW | te_long × 2 + te_delta                                    |
| Sync (arm)  | 65000 ± 11000 µs LOW | te_long × 130 ± te_delta × 100 (garage, doctor)    |
| Sync (86)   | 89500 ± 7920 µs LOW  | te_long × 179 ± te_delta × 72 (doctor, keypads)    |
| TX gap      | 68000 µs (62-bit, garage) / 91000 µs (86-bit, doctor)   | te_long × 136 / × 182 |

## Frame Layout

**62-bit packet** (remotes): bits 61..46 zero (16-bit preamble), 45..42 `1111`, 41..40 packet id (0 = packet 1,
1 = packet 2), 37..34 order code, 33..30 invert code, 29..0 three interleaved 10-bit words. Decoding a packet
(`decode_half`): de-interleave, un-invert and re-order the words per the codes (invalid code → no fields); the
4 code digits + 5 two-bit digits of the third word are 9 rolling trits (a digit of 3 → no fields); the first two words
are the 20-bit fixed half.

**86-bit packet** (keypads, doctor-firmware): a 22-bit preamble `0x3C | packet id`, then a 64-bit payload `01`,
order 61..58, invert 57..54, three interleaved 18-bit words: 10 bits of fixed half + 8 bits of a data half per word,
13 trits of which the last 4 repeat the first 4. The 32-bit data word (packet 1 high, packet 2 low half) holds the
PIN (byte-swapped binary, 0–9999) and a check nibble: its 8 nibbles XOR the button's low nibble to 0.

**Fields** (`remote_controller`): counter = the 18 trits (packet 2 / packet 1 interleaved) as a base-3 number,
< 2^28, bit-reversed over 28 bits; `button = fixed_1 >> 12`; `serial = fixed_1 << 20 | fixed_2` (32 bits — with the
button that is the full 40-bit fixed code). KAT's `counter` holds the low 16 bits of the 28-bit counter.

**DecodedSignal packing:** `data` = the references' `Key` (packet 2: 62 bits, or the 64-bit payload of an 86-bit
packet), `extra` = `Secplus_packet_1`, `data_count_bit` = 62 or 86 (the references' `Bit`).

## RF

- **Encoding:** Manchester (1 = LOW then HIGH)
- **RF modulation:** AM (OOK); flags 315 MHz (doctor-firmware: 315 + 433); KAT decodes on every band
- **Encryption:** rolling code (order/invert scrambling, no key)
- **Make:** Chamberlain / LiftMaster

## Decoder Steps

1. **Reset** — a LOW in a sync window arms the decoder: packet 1 cleared, Manchester primed (Reset, LongHigh,
   ShortLow → Start0).
2. **DecoderData** — pulses feed Flipper's Manchester machine; out-of-tolerance pulses → Reset. At bit 22 the packet id
   of an 86-bit frame is latched.
3. **Packet end** (LOW ≥ 1110 µs) — 62 bits: header mask `0xFFFF3C0000000000 == 0x3C0000000000`; id 0 is stored as
   packet 1, id 1 completes a pair if a 62-bit packet 1 is stored. 86 bits (doctor): payload `>> 62 == 1`; id 0 stored,
   id 1 completes; a stored 86-bit packet 1 is dropped after a gap > 179 ms. A completed pair re-arms the decoder.
4. **KAT gate:** the C reports every completed pair, with zero fields when a packet's codes/digits are invalid or the
   86-bit check fails. KAT reports only pairs whose fields decode (`crc_valid` true): on the oracle fuzz corpus this
   drops 43 of 1157 doctor frames, all with zero fields; real captures are unaffected.

## Encoder

1. Fields from the packets (62-bit: zeros when they do not decode, as the C; 86-bit: must decode, else nothing is sent).
2. Button: KAT's native 8-bit code (the references' button editor takes any 8-bit value; the D-pad offers
   0x68 / 0x80 / 0x81 / 0xE2 / 0x78); the app passes the captured one by default.
3. Counter: + 1; above 0xFFFFFFF → 0xE500000; below 0xE500000 → 0xE500000.
4. 62-bit (garage): both packets rebuilt; upload = Manchester packet 1 (its last half kept only if HIGH — a LOW one is
   overwritten by the gap), LOW 68000, packet 2, LOW 68000; `encoder.repeat` = 3 copies (600 entries).
5. 86-bit (doctor): data word kept (with a key file's `Pin` applied), check nibble recomputed for the button; per packet
   HIGH 250 + LOW 500, 22-bit preamble and 64-bit payload in Manchester, LOW 91000; packet 2 (id 1) first; 4 copies.

**Reference differences.** doctor-firmware sends 62-bit pairs packet 2 first with 69000 µs gaps (te_long × 138);
the garage copy sends packet 1 first with 68000 µs gaps — KAT follows the garage copy. Real remotes (UberGuidoZ 891LM /
893LM / 893MAX / 2668A RAW captures, the unit-test capture) send packet id 1 first and space packets 68.6–69.4 ms
apart, as doctor-firmware does; every reference decoder accepts both orders. The garage app's TX plugin is built
without `SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS` and `SUBGHZ_GARAGE_HAS_CUSTOM_BTN`: as shipped it would clear the button
(custom-button shim returns 0) and step the counter in "OFEX" mode (≥ 0xE500000 → 0xFFFFFFE). KAT follows the
source's normal mode, which Flipper-ARF's firmware and doctor-firmware run.

## Key file

- 62 bits (garage `serialize`): `Protocol: Security+ 2.0`, `Bit: 62`, `Key`, `Secplus_packet_1` (8 hex bytes).
- 86 bits (doctor): `Bit: 86`, `Key`, `Serial`/`Btn`/`Cnt`, `Secplus_packet_1`, `Pin` (4 digits; empty when the
  frame's PIN is not 0000–9999). A valid `Pin` that differs from the frame's is applied, as doctor-firmware's encoder
  does.
- Loading needs `Bit: 62` or `86` and `Secplus_packet_1`.

## Validation

C oracles (doctor, garage, arf builds):

- **Decoder:** identical to doctor and garage on 57 unit-test RAW captures, 1136 UberGuidoZ RAW files (181 decodes from
  the 7 Security+ 2.0 captures) and the rtl_433 bursts; on 4786 fuzz files + 1000 bit-error files identical to doctor
  except 43 zero-field frames (KAT gate). Against garage, the additional differences are the 86-bit keypad frames
  (389 files) and doctor's 89.5 ms sync window (13 files), all equal to doctor.
- **Encoder:** 62-bit uploads pulse-exact with Flipper-ARF lib on 245 keys (unit-test key, 4 UberGuidoZ keys, 240
  random incl. counter edges and undecodable packets), and with the garage build on 60 keys where its OFEX step and
  button shim agree with the normal mode; with packets swapped and 69000 µs gaps they equal doctor's on all 245.
  86-bit uploads pulse-exact with doctor on 120 keys (24 with a `Pin` override).
- **Real captures:** unit-test `security_pls_2_0_raw.sub` (14 decodes); UberGuidoZ 891LM/893LM/893MAX/2668A RAW (all
  decode, incrementing counters) and key files (load; fields equal the oracle's). rtl_433
  `secplus/v2/g002_310.01M_250k.cu8`: no decode — its packets are 8.2 ms apart, outside every reference's sync window
  (the oracles do not decode it either); with the gaps stretched to 69 ms KAT and the oracle decode id 0x74C58200
  (1959100928), button 0x10 (16), counter 0xE500343 (240124739) — rtl_433's values.
- No decode on other protocols' unit-test files (except `test_random_raw.sub`, which contains these frames) or IMPORTS.
