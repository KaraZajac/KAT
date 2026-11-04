---
layout: default
---

# Renault V1 (ARF) Protocol (82-bit type-table frames)

**Rust module:** `src/protocols/renault_v1_arf.rs`
**Reference:** `REFERENCES/Flipper-ARF/lib/subghz/protocols/renault_v1.c` (Flipper-ARF firmware)

## Overview

Flipper-ARF's firmware registers a protocol called "Renault V1" right after its Renault V0. It is not
ProtoPirate's Renault V1 (a HITAG2/PCF79xx decoder, KAT's [Renault V1](renault_v1.md)): it reads the
older Renault frame types 0x04, 0x0C, 0x1A, 0x3B and 0x3F, plus a catch-all "Dynamic" type, on the same
Manchester wire as Renault V0. KAT names it **Renault V1 (ARF)** to keep the two apart.

These types used to live in ProtoPirate's Renault V0. ProtoPirate commit 1287d10 ("move newer Renault
variants into Renault V1") dropped them in favour of its HITAG2 decoder. Flipper-ARF instead moved them
into this decoder, and its Renault V0, like KAT's, now only accepts type 0x13. ARF's firmware still
decodes and replays these frames, so KAT does too (union of the references' capabilities). No capture of
such a remote exists in `IMPORTS/`.

ARF calls the frames "Hitag2-based rolling codes", but it can only replay them: its own comments say the
position of the 32-bit hop inside the payload and the IV layout are unknown.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Sync | HIGH ≥ 800 µs | encoder: 1000 µs |
| Short | 53-180 µs | TE profiles 125/250 ±69, 124/248 ±60, 108/250 ±55 µs, tried in order |
| Long | 181-319 µs | |
| Encoder TE | 125 µs (140 µs with a 12-bit preamble) | |
| Burst end | LOW 1200-2000 µs with ≥ 96 bits, any pulse ≥ 5000 µs, > 109 bits, or an unclassifiable pulse | |
| Encoder gap | LOW 1500 µs after each burst | |

## Frame Layout (82 bits after a 12/16/18/20-bit preamble)

The decoder takes the **last 82 bits** of a burst; the number of bits before them (the preamble) must be
12, 16, 18 or 20 (their values are not checked).

| Bits | Content |
|------|---------|
| data[63:40] | serial (24 bits) |
| data[39:32] | button (8 bits) |
| data[31:24] | counter (8 bits) |
| data[23:0] + key2[17:0] | 42 payload bits |

`checksum` = XOR of the 8 data bytes, key2[17:10] and key2[9:2]; `low6 = checksum & 0x3F`,
`hi2xor = (checksum >> 6) ^ (key2 & 3)`.

| Type (`low6`) | `hi2xor` | Buttons | Preamble |
|---------------|----------|---------|----------|
| 0x04 | 0 | 0x04-0x0B | 16 |
| 0x0C | 0 | 0xC4-0xCB | 18 |
| 0x1A | 3 | 0x44-0x4B | 12 |
| 0x3B | 0 | 0x44-0x4B | 20 |
| 0x3F | 0 | 0xC4-0xCB | 20 |
| Dynamic (any other `low6` except 0x13, or a typed tag whose check/button failed) | 0 or 3 | 0x04-0x0B, 0x44-0x4B, 0xC4-0xCB | 12/16/18/20 |

Dynamic frames also need a non-zero serial and are reported only when two consecutive candidates are
identical. `low6 = 0x13` is always rejected: it is Renault V0's type.

**Buttons (ARF names):** low nibble 4-7 "Lock", 8-B "Unlock" (the high nibble is the type family).

### KAT `DecodedSignal` packing

| Field | Value |
|-------|-------|
| `serial` | serial (24 bits) |
| `button` | raw button byte |
| `counter` | counter byte |
| `crc_valid` | C1 && C2 (always true on emission) |
| `data` | the 64 data bits |
| `extra` | key2 (bits 0-17, the last 18 wire bits) \| preamble length << 24 |
| `data_count_bit` | 82 |
| `encoder_capable` | true unless the replay would overflow ARF's upload buffer (see Encoder) |
| `protocol_display_name` | `Renault V1 (ARF) [0x0C]`, … or `Renault V1 (ARF) [Dynamic 0x2A]` (checksum tag) |

`ProtocolRegistry::get()` resolves the display names (the part before ` [`).

## RF

- **Encoding:** Manchester (ARF's inline Renault table), 125 µs TE
- **RF modulation:** AM or FM (ARF flags both)
- **Encryption:** unknown rolling code (ARF: HITAG2-based); KAT replays only
- **Frequencies:** 315 MHz, 433.92 MHz, 868.35 MHz

## Decoder Steps

1. **Reset** — wait for a HIGH ≥ 800 µs (sync), then collect bits from Manchester state Mid1.
2. **Data** — classify each pulse against the three TE profiles; feed ARF's state table
   `{0x01, 0x91, 0x9B, 0xFB}` (HIGH → ShortLow/LongLow, LOW → ShortHigh/LongHigh; a transition to the
   same state is ignored); entering Mid1 records a 1, Mid0 a 0.
3. **Burst end** — on a LOW of 1200-2000 µs with ≥ 96 bits, a pulse ≥ 5000 µs, more than 109 bits, or an
   unclassifiable pulse, judge the buffered bits; a HIGH ≥ 800 µs also starts the next burst.
4. **Candidate** — the last 82 bits after a 12/16/18/20-bit preamble; typed rules, else Dynamic rules
   (above); the typed preamble length must match.
5. **Confirm and emit** — Dynamic frames wait for an identical second candidate; a frame identical to
   the last emitted one is suppressed. `reset()` follows the C: it drops the pending Dynamic candidate
   and key2 but keeps the last data, so after a registry reset the same frame is reported again unless
   its key2 is 0.

## Encoder

A byte-identical **replay**, as ARF does (it disables the D-pad for these frames; any button replays the
capture). The frame must still classify; the preamble is the stored one, or the type's default.

Upload per frame (`renault_v1_build_upload`), 3 bursts: HIGH 1000 µs; the preamble 1-bits and the 82
frame bits through ARF's two-state emitter (state 1: bit 1 → LOW te + HIGH te, bit 0 → LOW 2·te and state
2; state 2: bit 0 → HIGH te + LOW te, bit 1 → HIGH 2·te and state 1); a closing HIGH te in state 2; LOW
1500 µs. Adjacent levels are merged. The upload is sent 10 times (ARF's minimum repeat). ARF builds it
in a 600-level buffer and refuses frames that do not fit (only contrived frames with long runs of equal
bits and a 20-bit preamble); KAT reports those as not encoder-capable.

**Not ported:**

- *Fiat V1 key search* (`renault_v1_verify_hitag2_key` + UID key database): Fiat's 8 keys on 3 guessed
  32-bit hop positions × 4 guessed IV layouts. ARF marks these "pending validation against a real
  capture" and only displays the result; its encoder stays replay-only anyway.
- *"Seed BF" re-encoder* (`subghz_protocol_renault_v1_run_seed_bf`, `renault_v1_encoder_reencode_seed`):
  ProtoPirate's PCF seed search run on this 82-bit frame packed into 11 bytes. The search treats
  data[63:32] (serial ‖ button) as the 32-bit UID, but the re-encoder encrypts with the 24-bit serial
  as UID, a different key, so its "next code" cannot match the remote. ProtoPirate's working seed scheme
  is in KAT's HITAG2 [Renault V1](renault_v1.md).

## Validation

- C oracle: ARF's unmodified `renault_v1.c` compiled on the host.
  - **Encoder:** 500 frames (every type, Dynamic, non-default preambles, invalid frames, long-run
    frames). All agree: 439 replays pulse for pulse including the 10 repeats; 60 frames that do not
    classify and 1 that overflows the buffer are refused by both.
  - **Decoder:** 2390 pulse streams, field for field: clean and jittered (±12 %, TE ×0.9-1.15) C
    uploads, other preamble lengths, gate failures, single/double/interleaved Dynamic bursts, explicit
    resets and 300 random pulse trains. 1151 decoded frames, 0 differences.
  - **IMPORTS:** all 108 captures with RAW data, both polarities. Nothing decodes in either (the sweep is
    unchanged).
- **Registry:**
  - All 439 replays decode only as Renault V1 (ARF) at 315/433/868 MHz, the same 395 frames as the C on
    its own (the other 44 are typed frames with a non-default preamble, which the C encodes but no
    decoder accepts).
  - A cross-protocol guard feeds every other encoder's output through the registry; none is claimed.
- **Renault V0 exclusivity:**
  - A type-0x13 frame decodes as Renault V0 only (from Renault V0's encoder and from ARF's emitter), and
    this decoder rejects it on its own.
  - Every ARF type decodes as Renault V1 (ARF) only.
- **ProtoPirate HITAG2 overlap:**
  - On its own this decoder, exactly like ARF's C (same frame reported), reads two consecutive
    HITAG2/PCF long frames as a Dynamic frame.
  - In the registry, ProtoPirate's Renault V1 completes each long frame first, and the reset after its
    hit drops the pending candidate, so those captures stay "Renault V1".
  - Fiat V1 type-B frames (100/200 µs, inside this decoder's windows) are not claimed even standalone.
