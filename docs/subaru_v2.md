---
layout: default
---

# Subaru V2 Protocol

**Rust module:** `src/protocols/subaru_v2.rs`
**Reference:** KAT's own reading, worked out from a tester's recordings of one 433.92 MHz Australian Subaru remote-head key: three recordings, 22 presses, all of the same key. Which models use it is not known; FoxFW names the remote after the Liberty GT. FoxFW v2.1.1 (2026-11-22) also reads this remote, as "Subaru Liberty GT" (`lib/subghz/protocols/subaru_liberty_gt.c`), in another way: see [FoxFW's reading](#foxfws-reading). ProtoPirate, Flipper-ARF and doctor-firmware do not read it. It is distinct from both the 800/1600 µs PWM [Subaru](subaru.md) remote and the US 315 MHz plaintext-counter [Subaru V1](subaru_v1.md) key.

## Overview

The key sends a **128-bit (16-byte)** frame, repeated several times while the button is held. Unlike Subaru V1, the rolling part is **encrypted**: eight of the sixteen bytes change on every press with no plaintext counter, so KAT reads these **decode-only** — it identifies the remote and reports a 32-bit field that is the same in every press, read as the key ID, but does not interpret or synthesise the rolling code. With one key recorded, that this field differs from key to key is not yet shown (FoxFW reads its own key serial from the same region). KAT reports and can replay a captured press; it does not build fresh codes (there is no plaintext counter, and that would require the key's secret).

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Half-bit | 700 µs | bit period 1400 µs |
| Modulation | OOK | 433.92 MHz |
| Line code | differential Manchester | `HL` = 1 / `LH` = 0 (plain), then `data[i] = plain[i] XOR plain[i-1]` |
| Preamble | run of alternating 700 µs pulses | a long differential-zero run |

## Frame (16 bytes, differential domain)

The frame is anchored on an in-band **sync** run of 8 differential zeros that recurs once per 128-bit period. Anchored at the sync run's end so the payload leads:

| Bytes | Field |
|-------|-------|
| b0–b7 | encrypted rolling payload (changes every press) |
| b8–b11 | **32-bit key ID** (`serial`) — constant across every press of one key |
| b12–b14 | trailer (part of the rolling code or a MAC over it) |
| b15 | sync run (the next frame's 8 zeros) |

## Decoder Steps

1. Accumulate half-bit levels: a ~700 µs pulse is one half-bit, a ~1400 µs pulse two; a gap or off-timing pulse ends the burst.
2. Manchester-decode the burst (the phase with no violations), then differentially decode.
3. Find the sync: a zero run of 8–24 bits that recurs exactly 128 bits later (the preamble and the silence tail are far longer and have no partner one period away).
4. Anchor at the sync run's **end**, read 128 bits, and report the 32-bit ID. `button` and `counter` are not recovered — they are inside the encrypted payload.

`crc_valid` is false (`NO_CHECKSUM`): emission is gated on the framing (differential-Manchester-clean 128-bit period and the twin sync runs one period apart), which is specific enough that the Subaru PWM frames, Subaru V1 and every other decoder are unaffected.

## Encoder

None — decode-only. The rolling payload is encrypted, so there is no plaintext counter to advance, and KAT does not attempt to recover the key's cipher secret. A captured press can be replayed from its recording.

## FoxFW's reading

FoxFW's "Subaru Liberty GT" (v2.1.1) has the same air format, 700 / 1400 µs Manchester and 128-bit frames, but
reads it differently:

| | KAT (Subaru V2) | FoxFW (Subaru Liberty GT) |
|---|---|---|
| Bits | differential (`plain[i] XOR plain[i-1]`) | plain Manchester bits |
| Timing windows | a half-bit 400..1050 µs, two half-bits 1050..1750 µs | 700 ± 100 and 1400 ± 100 µs |
| Frame boundary | the 8-zero differential run that recurs every 128 bits | nine 1s then a 0 at the top of the 128-bit window: the same run, in the plain domain |
| Key ID | differential bytes 8..11 (32 bits) | `Serial` = plain bits 80..111 (32 bits) |
| Button, counter | none | none (0) |
| Key file | not written | `Bit: 128`, `Key`, and `Frame` (the 16 bytes) |
| Encoder | none; Replay re-sends the recorded press | re-sends the stored frame (10 copies) |

Both numbers are constant for one key, so either identifies the remote, but they are different numbers for the
same key, and a FoxFW key file is not loaded by KAT. On the tester's recordings FoxFW's narrower windows read fewer
frames than KAT (1 to 5 per file against 4 to 12). Neither one makes a new code: FoxFW's encoder replays.

## Validation

Against a tester's recordings of one key (three files, 22 presses, 433.92 MHz, kept out of git in `REFERENCES/testers`): every recording decodes as Subaru V2, and every press reports the same 32-bit ID while the rolling payload changes. The three files were first taken for three keys; they are one, so whether the ID field differs between keys needs a recording of a second key. Unit tests cover the differential-Manchester framing, the sync anchoring (robust to the 8-vs-9-zero run length), ID extraction and stability, and rejection of noise and short bursts.
