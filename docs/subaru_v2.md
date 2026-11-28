---
layout: default
---

# Subaru V2 Protocol

**Rust module:** `src/protocols/subaru_v2.rs`
**Reference:** none. ProtoPirate, Flipper-ARF and doctor-firmware do not read it. The format was worked out from a tester's recordings of several 433.92 MHz Australian Subaru remote-head keys (Liberty/Legacy, Impreza/WRX/STI and relatives). It is distinct from both the 800/1600 µs PWM [Subaru](subaru.md) remote and the US 315 MHz plaintext-counter [Subaru V1](subaru_v1.md) key.

## Overview

The key sends a **128-bit (16-byte)** frame, repeated several times while the button is held. Unlike Subaru V1, the rolling part is **encrypted**: eight of the sixteen bytes change on every press with no plaintext counter, so KAT reads these **decode-only** — it identifies the remote and recovers the 32-bit key ID, but does not interpret or synthesise the rolling code. KAT reports and can replay a captured press; it does not build fresh codes (there is no plaintext counter, and that would require the key's secret).

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

## Validation

No public reference. Against a tester's recordings of three keys (several presses each, 433.92 MHz, kept out of git in `REFERENCES/testers`): every recording decodes as Subaru V2, and every press of one key reports the same 32-bit ID while the rolling payload changes. Unit tests cover the differential-Manchester framing, the sync anchoring (robust to the 8-vs-9-zero run length), ID extraction and stability, and rejection of noise and short bursts.
