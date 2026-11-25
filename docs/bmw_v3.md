---
layout: default
---

# BMW/Mini V3 Protocol

**Rust module:** `src/protocols/bmw_v3.rs`
**Reference:** FoxFW `lib/subghz/protocols/bmw_v3.c` (`BMW_V3_PROTOCOL_NAME = "BMW/Mini V3"`, a port of the firmware's
`bmw_v3.c` via KeeLoq-RX `arke_bmw.py`). **Receive only, as the reference** (the encoder is NULL).
**Also read from:** the firmware of a BMW CAS3 module (CAS3 0L15Y, MC9S12XDP512), which receives this frame.

## Overview

The remote frame of BMW's CAS3 (Car Access System 3, the E9x era): 2-FSK, an 8-bit `0xCC` header, 64 payload bits and an
XOR check byte. FoxFW reads the header and the 64 bits; the CAS3's firmware shows the rest: the check byte, which
bytes hold the key ID, the authenticator and the counter, and that the authenticator is HITAG2, not AES as FoxFW's
comment says. A different signal from KAT's "BMW" (`bmw.rs`, 62 µs AM frame behind the 0x599E sync) and from KAT's
"BMW CAS3" (`bmw_cas3.rs`, a PPM format FOBworks names after CAS3; the CAS3's own firmware reads this frame).

## Timing / modulation

2-FSK Manchester (flags 433 | 315 | AM | FM; meaningful on FM captures), `te_short` 95 µs, `te_long` 190 µs, tolerance
±40 µs. A LOW of more than 350 µs is the gap before a frame. Measured on the 2004 E60 capture in IMPORTS
(`BMW/more/BmwE60_2004_868_FM_cl.sub`, 868.35 MHz): half bit 96 µs, bit 190 µs (about 5.3 kbit/s). Each copy is a
LOW of about 500 µs, then `0xCC` at once (no preamble bits), 80 bits in all; the copies come 16.5 ms apart, 9 in the
first burst.

## Frame

On air, MSB first: `CC | b1 | b2 b3 | b4 b5 b6 b7 | b8 | b9`.

| Byte | Meaning |
|---|---|
| `b1` | type in bits 7..5 (`000` a key; `010` and `100` a second class of transmitter, two learned slots, with its own ID table), bit 4, and the button nibble in bits 3..0: 1, 2, 4 or 8, and 0 for the telegram sent when the button is released |
| `b2 b3` | the key's 16-bit ID: the CAS3 looks it up among its 10 learned keys |
| `b4 .. b7` | the 32-bit authenticator (HITAG2 on CAS3) |
| `b8` | the low byte of the key's 32-bit counter |
| `b9` | the XOR of `b1 .. b8` |

On the E60 capture every copy recorded to its end has `b9` = XOR(`b1 .. b8`) (6 of 6 distinct frames). The last bit of
`b9` usually runs into the gap after the frame and is lost on a Flipper recording, so most copies end with 7 of its
8 bits.

## What the CAS3 does with it

From the CAS3 0L15Y P-flash (the specialist's dump; scripts and notes in
`REFERENCES/tools/scripts/bmw-cas-porsche-2026-11-24`). The remote receiver sits in the diversity (antenna) module; it
demodulates the frame and passes its bytes to the CAS3 as a UART stream on the FBD line (SCI1, 28.8 kbit/s), after
waking it on the WUP line, with one byte of its own after `b9` that the CAS3 keeps as a minimum / maximum (a signal
level).

1. `FE:87AB` waits for `0xCC` and collects 11 bytes. `FE:8BE6` requires `b1 ^ … ^ b9 == 0`; when that fails it
   re-aligns on the next `0xCC` in the buffer. A frame identical to the last one is dropped.
2. `FC:A953` / `FC:BCEA`: the type, and `b2 b3` against the learned IDs (or the test ID `0x10FF`).
3. `FC:A7ED`: the key's stored 32-bit counter with its low byte replaced by `b8`, plus `0x100` when that is not above
   the stored value.
4. HITAG2 (`FC:9A7B`, 32 init rounds; `FC:9C4D`, 32 output rounds) on `uid(32) ‖ key(48)` with the counter as the
   IV; the 32 output bits are compared with `b4 .. b7`. The key and uid are stored per learned key. The filter is
   HITAG2's (f4a `0x2C79`, f4b `0x6671`, f5c `0x7907287B`, kept as byte-per-entry tables at FD:6658, FD:6668,
   FD:6678, which is why no search for the constants found them), and so are the 16 feedback taps.
5. On a mismatch the counter is raised by `0x100` and tried again while it stays within `0x6D0` of the stored one;
   past that the key is marked out of sync. A match stores the counter and acts on the button nibble (`FE:837D`:
   nibble 0 = release, 1, 2, 4, 8 = four functions, button 2 also setting button 1's by coding).

KAT's `hitag2::bmw_cas3_authenticator(uid, counter, key)` is this function; a test proves it equal, on 20 000 random
inputs, to an instruction-for-instruction port of `FC:9A7B` and `FC:9C4D` with the table bytes copied from the dump.

## Decoder Steps

`subghz_protocol_decoder_bmw_v3_feed`, with KAT's check byte:

1. Reset: a LOW over 350 µs starts WaitGap. WaitGap: the first HIGH short or long pulse primes the accumulator
   (`te_last`); anything else resets.
2. A half-period accumulator as in BMW V2: a long pulse is one bit, two shorts are one bit, a long with a short pending
   completes a bit and leaves a third half pending; the bit is `level ? 0 : 1` of the completing pulse. A pulse that is
   neither short nor long resets (and starts a new gap wait when it is a long LOW).
3. FindHeader slides an 8-bit window over the bits (at least 8, at most 32) until its low byte is `0xCC`; the register
   then clears and the next 64 bits are the payload.
4. FoxFW reports at the 64th payload bit when `b1` is `0x02`, `0x04`, `0x08` or `0x10`. KAT reads 8 more bits (`b9`)
   and reports on the last one, or when the frame ends (a pulse outside the timing, or the end of the input): with
   all 8 bits of `b9`, or the first 7, matching the XOR, the frame is checked and is reported for any CAS3 type
   (`b1` bits 7..5 = `000`, `010` or `100`); otherwise it is reported on FoxFW's whitelist, unchecked, so every frame
   FoxFW reports is still reported.

## Field mapping

`data` = the 64-bit payload `b1 .. b8` (header and `b9` excluded), `data_count_bit` = 64, `button` = `b1`, `serial` =
`b2 b3`, `counter` = `b8`, `crc_valid` = the `b9` check, `encoder_capable` false (true with a loaded key, below).

## Keystore: verify, recover the counter, and encode

Like JLR, Audi BCM2 and the PSA older code, KAT ships no BMW keys and reads them only from the owner's own module. A
fob's HITAG2 secrets go in a `bmw_cas3_keys` file in the config dir — one fob per line, `ID UID KEY48 [COUNTER] [name]`
(ID = the air `b2 b3`; UID, the 48-bit KEY and the 32-bit COUNTER from the module's EEPROM). When a checked frame's ID
matches a loaded fob, KAT reproduces the CAS3's own check: it replaces the stored counter's low byte with `b8`, raises
it by `0x100` through the `0x6D0` resync window, and takes the first candidate whose HITAG2 authenticator
(`hitag2::bmw_cas3_authenticator`) equals `b4..b7`. On a match it resyncs the stored counter, shows the decode
authenticated (a ✓ on the name) and enables the encoder, which builds the fob's next valid code (counter + 1, fresh
authenticator and XOR byte). Inert without a key; the air frame alone stays decode-only.

### Where the secrets live in a CAS3 dump

The CAS3 0L15Y firmware's NvM descriptor table (P-flash FD:9470, 13-byte entries `fe <eep16> .. <ram16> 00 <len> ..`)
maps the RAM key block `0x1915..0x1A16` to fixed EEPROM offsets; on this chip the EEPROM offset is the D-flash file
offset. The 10 learned-key slots are parallel arrays:

| EEPROM | field | width |
|--------|-------|-------|
| `0x0830` | key IDs (`b2 b3`) | 10 × `u16` |
| `0x0848` | HITAG2 key, high 16 bits | 10 × `u16` |
| `0x0860` | HITAG2 key, low 32 bits | 10 × `u32` |
| `0x088C` | counter (HITAG2 IV) | 10 × `u32` (wear-levelled) |
| `0x0A8C` | HITAG2 uid | 10 × `u32` |

`KEY48 = key-hi ‖ key-lo`. The uid is stored only for the "type 2" slots; the others derive it from the VIN at learn
time (not reconstructed). The counter block is wear-levelled, so the stored value can trail the fob by up to ~100
presses — harmless, KAT resyncs on the first real frame. Extractor:
`REFERENCES/tools/scripts/bmw-cas-porsche-2026-11-24/tools/extract_cas3_keys.py`.

## Key file

`Protocol: BMW/Mini V3`, `Bit: 64`, `Key` = the payload, then `Serial`, `Btn`, `Cnt` (decimal; KAT writes `b8`,
FoxFW bits 31..16). Loading needs `Bit: 64`; the fields are carved from the key and the button whitelist is not
applied.

## Deviations from FoxFW

- The check byte `b9` (FoxFW has no check) and reporting on it or at the frame's end, not at the 64th bit.
- Checked frames for any CAS3 type: FoxFW's whitelist has no button 1, which a CAS3 acts on, and drops the release
  telegram (`b1` = 0); bit 4, which it lists, is not part of the CAS3's button nibble.
- `counter` = `b8` (FoxFW: bits 31..16, inside the authenticator).

## Not established

- What the four buttons do (the CAS3 maps nibbles 1, 2, 4, 8 to four body functions; the firmware does not name them).
- The E60 capture is a 2004 car (CAS2): its `b4 b5` keep fixed bits over 7 presses (`1011 0x1x 011x x110`), so a CAS2
  key fills those bytes otherwise than with a HITAG2 output; its `b9` is the same XOR.
- The HITAG2 part is proven against the CAS3's own code, not yet against a captured frame of a *known* key: no capture
  of the dumped E60's own fobs exists. Real Mini CAS3 captures from two other cars decode with the
  XOR check passing and the counter stepping press by press, which confirms the frame structure on real hardware, but
  their keys are not available to verify the authenticator.
- The VIN-derived uid of the non-"type 2" slots (the learn-time derivation from the vehicle bytes) is not
  reconstructed, so those slots are not yet usable from a dump alone.

## Validation

- Reference oracle `subghz_oracle_foxfw`: synthetic frames for four buttons (made-up payloads) and a frame with 5 junk
  bits before the header give identical payload, serial and button (KAT's counter is `b8`); a bad button, a wrong
  header, a header more than 32 bits in and a 63-bit payload are rejected by both.
- The E60 capture: 61 decodes (FoxFW's 57, plus the 4 release telegrams), all of them checked; counters `C0 C2 C3 C4
  C5 C6 C8` in press order.
- Unit tests: the oracle-checked frames and rejections; the check with 8, 7 and 6 bits; a bad check falling back to
  FoxFW's whitelist; release, button 1 and the type gate; a frame cut off by the end of the input; the E60 frames'
  XOR bytes; the key-file round trip; the HITAG2 equality with the CAS3 firmware.
- Keystore unit tests: counter recovery tracks the fob within the `0x6D0` window and rejects a replay or an
  out-of-window counter; the encoder's pulses decode straight back to the frame (the biphase encoder is the decoder's
  inverse); and, end to end with a loaded key, a frame verifies, the stored counter resyncs, and the encoder makes the
  next valid code — all against the firmware-proven HITAG2. The `bmw_cas3_keys` parser is tested too.
