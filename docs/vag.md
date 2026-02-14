# VAG (VW/Audi/Seat/Skoda) Protocol

**Rust module:** `src/protocols/vag.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/vag.c`

## Overview

VAG supports multiple types (1, 2, 3, 4) with different timing and encryption. Manchester, 80 bits (key1 64 + key2 16). Type 1/2: 300/600 µs, prefix 0xAF3F / 0xAF1C. Type 3/4: 500 µs, 45 preamble pairs, sync 1000+500 then 3×750 µs; key1/key2 not inverted. Type 1/2 use AUT64 or TEA decryption; type 3/4 use AUT64. Button names: Unlock, Lock, Boot. Keys loaded from keystore (VAG raw 64 bytes = 4×16-byte AUT64 keys; lookup by index 1, 2, 3). **Decode and encode** are both supported; encode uses capture data plus stored vag_type/key_idx.

## Timing

- **Type 1/2:** 300 µs short, 600 µs long, ±79/80; Preamble1→Data1 gap 600 µs ±79; end-of-data gap < 4000 µs.
- **Type 3/4:** 500 µs short, 1000 µs long, ±80; Preamble2 count 500±80; Sync2A 500/1000 µs ±79; Sync2B/Sync2C 750 µs ±79; Data2 short 500±120, long 1000±120.

## Encoding

Manchester; type-dependent short/long and sync sequences. Encoder builds the waveform from **decoded capture data** (serial, button, counter, key1) plus **protocol extra** (vag_type, key_idx) so retransmit works after decoder reset.

## Frame Layout (80 bits)

- **key1 (64 bits)** + **key2 (16 bits)**. Dispatch byte in key2; key index for AUT64 (type 1/2). Type 3/4: AUT64 decrypt.

## Decoder Steps

1. **Reset** — 300 µs or 500 µs (type-dependent) → Preamble1 or Preamble2.
2. **Preamble1** — Count; gap 600 µs → Data1. **Data1** — Manchester; 80 bits; parse type 1/2 (AUT64 or TEA).
3. **Preamble2** — Count 500 µs pulses; Sync2A (500/1000) → Sync2B (750) → Sync2C (750) → Data2. **Data2** — Manchester 80 bits; AUT64 decrypt (type 3/4).

Parse (vag_parse_data): dispatch 0x2A/0x1C/0x46 and 0x2B/0x1D/0x47; try AUT64 with key index 0,1,2; or TEA for type 2.

## Encoder (encode from capture)

- **Supports encoding:** protocol reports `true`; TX Lock/Unlock/Trunk available when the capture has encoder data.
- **Stored state:** On successful decrypt, the decoder sets `DecodedSignal.extra = Some(vag_type | (key_idx << 8))`. The app copies this to `Capture.data_extra`. Without `data_extra` (e.g. old import or undecrypted capture), encode returns `None`.
- **Encode path:** `encode_signal(decoded)` uses `decoded.serial`, `decoded.button`, `decoded.counter`, `decoded.data` (key1 / type_byte), and `decoded.extra` (vag_type, key_idx) to build type 1, 2, or 3/4 waveform. No decoder instance state is used so retransmit works after reset.

## Frequencies

433.92 MHz, 434.42 MHz.

## Keystore

VAG AUT64 keys: 4×16-byte packed keys from embedded blob (“VAG ” + 64 bytes) or file. Lookup by `get_vag_key((key_index+1) as u8)` (index 1, 2, 3).
