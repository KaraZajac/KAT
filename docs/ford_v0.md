---
layout: default
---

# Ford V0 Protocol

**Rust module:** `src/protocols/ford_v0.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/ford_v0.c`

## Overview

Ford V0 uses Manchester encoding at 250/500 µs. 80 bits total: 64-bit key1 + 16-bit key2. key2 is a
checksum byte (BS) and a CRC byte; the CRC is a GF(2) matrix product over key1 bytes 1–7 and BS. Each
press sends 6 bursts, each with a short preamble (4 long pairs) and a 3500 µs gap before the data.

Manchester is decoded with the shared port of Flipper's `manchester_decoder.c` (`common.rs`), using
ProtoPirate's `pp_manchester_event()` mapping: level true → ShortLow/LongLow, level false →
ShortHigh/LongHigh.

## Timing

| Parameter | Value   | Notes                                                    |
|-----------|---------|----------------------------------------------------------|
| Short     | 250 µs  | ±120 µs (C: ±100; real captures have ~390 µs "longs")     |
| Long      | 500 µs  | ±120 µs                                                  |
| Gap       | 3500 µs | ±250 µs                                                  |
| Min bits  | 64      | key1; 80 on air                                          |

## Frame Layout (80 bits)

On air: `~key1` MSB first (bit 63 is not sent — the decoder seeds it as 1) then `~key2`. On-air bits
are Manchester with 1 = HIGH,LOW.

- **key1 byte 0:** header (0x55 on every capture in `IMPORTS/FORD`)
- **key1 bytes 1–4:** serial
- **key1 byte 5:** button field (bits 7..3, one-hot) | counter bits 18..16 (bits 2..0)
- **key1 bytes 6–7:** counter bits 15..0
- bytes 1–7 are XOR-scrambled and bytes 6/7 bit-interleaved, keyed by the parity of BS (`decode_ford_v0`)
- **key2:** BS (high byte) = byte sum of serial + counter + (button field << 3); CRC (low byte) XOR 0x80

Buttons: the C reads a 4-bit field (`(buf[5] >> 3) & 0x0F`: Panic=1, Lock=2, Unlock=4, Boot=8). Real
captures show a fifth one-hot bit (byte 5 bit 7) for Remote start, which the BS checksum includes. KAT
decodes all five bits and reports KAT's generic button bits:

| On-air field | C name | KAT `button` |
|--------------|--------|--------------|
| 0x01         | Panic  | 0x08 (Panic) |
| 0x02         | Lock   | 0x01 (Lock)  |
| 0x04         | Unlock | 0x02 (Unlock)|
| 0x08         | Boot   | 0x04 (Trunk) |
| 0x10         | —      | 0x10 (Remote start, shown as "Other") |

`DecodedSignal`: `data` = key1 (de-inverted, as the C's `generic.data`), `data_count_bit` = 64 (as the C),
`extra` = key2, `counter` = counter bits 15..0 (the counter is 19 bits; the encoder recovers the full
value from `data` + `extra`).

## RF

- **Encoding:** Manchester 250/500 µs (on-air 1 = HIGH,LOW)
- **RF modulation:** AM/OOK
- **Encryption:** none (scrambled fixed fields + rolling counter); CRC-checked
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset** — short HIGH (C) or long HIGH (KAT: re-sync mid-preamble) → Preamble; Manchester reset to Mid1.
2. **Preamble** — LOW long → PreambleCheck.
3. **PreambleCheck** — HIGH long → Preamble; HIGH short → Gap.
4. **Gap** — LOW ~3500 µs (±250) → Data with the first bit seeded as 1 (bit_count = 1). The Manchester state
   stays Mid1, so the first HIGH after the gap (the seeded bit's first half) is dropped and decoding locks
   on the following LOW.
5. **Data** — Manchester events; any other duration (including the inter-burst gap) → Reset. At 64 bits
   key1 = ~bits; at 80 bits key2 = ~bits, then the CRC is checked.

**CRC / gating:** like the C, only the low 7 bits of the CRC byte are compared — bit 7 is not the matrix
output on real fobs (it differs on ~40% of genuine frames, which an 8-bit compare wrongly rejects). The C
emits every 80-bit frame and only displays CRC OK/BAD; KAT emits only CRC-valid frames, so `crc_valid` is
always true on emitted frames. Every genuine frame in `IMPORTS/FORD` passes.

## Encoder

Supported, ported from `subghz_protocol_encoder_ford_v0_get_upload`. Advances the counter by one per
press (ProtoPirate's emulate scene writes Cnt + 1 before the encoder reads it; Flipper-ARF and
doctor-firmware add the rolling-counter step in the encoder; kept within its 19 bits), maps KAT's generic
Lock/Unlock/Trunk/Panic to the button field (the emulate scene's Up/OK/Down/Left; any other code keeps the
captured field, e.g. Remote start), recomputes BS (`ford_v0_calculate_checksum`) and CRC (matrix ^ 0x80),
re-scrambles key1 (`encode_ford_v0`) and emits 6 bursts: short HIGH, long LOW, 4 long pairs, short HIGH,
3500 µs LOW, then the transition-coded `~key1` bits 62..0 and `~key2`, with a 50 ms LOW between bursts.

One deviation from the C upload: a final on-air 0 bit is finished with its HIGH half, as real fobs do
(`2008_Focus_Lock.sub` bursts end `... -504 +257 -51810`). The C stops after the LOW half, which merges
into the inter-burst gap, so its bursts lose their last bit whenever the CRC's last on-air bit is 0.

## Validation

- `IMPORTS/FORD/{0000_Ford_ts_x4, 16_ford_escape_rs2_l2_u2_t2_p2, 2008_Focus_Lock, 2008_Focus_Trunk,
  2008_Focus_Unlock, 3_unlock_ford, Ford Mondeo}.sub` all decode with `crc_valid=true`, one serial per fob
  (0x43C024F8, 0xFCA7C7BF, 0x4306BE50 for the three Focus files, 0x48189F19, 0x54BF55F4), the buttons named
  in the file names and a counter that increments per press (`decodes_real_captures`). KAT reproduces every
  frame the compiled ProtoPirate, Flipper-ARF and doctor-firmware decoders report on these files; the extra
  KAT frames all pass the CRC (the wider te_delta, see Timing).
- Encoding a genuine press reproduces the fob's genuine NEXT press (counter + 1, same file) exactly: key1,
  the checksum byte and the 7 checked CRC bits, for 7 pairs across 6 captures
  (`reencode_reproduces_next_genuine_press`).
- 492 uploads (every decoded capture frame × Lock/Unlock/Trunk/Panic) equal ProtoPirate's encoder fed by
  its emulate scene pulse for pulse, except exactly the trailing-half-bit deviation above; Flipper-ARF's
  encoder produces the same uploads as ProtoPirate.
- Encode→decode round trip through the full 6-burst waveform for each button; a corrupted key1 bit is
  rejected by the CRC gate.
