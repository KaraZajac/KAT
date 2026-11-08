---
layout: default
---

# Nissan V0 Protocol

**Rust module:** `src/protocols/nissan_v0.rs`
**Reference:** `FoxFW lib/subghz/protocols/nissan_v0.c` (no other reference has it)

## Overview

A 64-bit encrypted payload behind a differential Manchester at 500/1000 µs. Its only gate is 12 fixed bits (a
6-bit sync `110001` and a 6-bit marker `000010` 48 bits later), which is weak: FoxFW's C decoder reports a
"frame" in two Flipper-ARF unit-test captures that no Nissan fob made (`nice_one_raw.sub`, a Nice One gate
remote, and `test_random_raw.sub`, random noise). In KAT the decoders before it (Nice One, Star Line) already
read those files and keep them, so the corpus diff is empty; Nissan V0 is registered last of the FoxFW block so
it only sees what every other decoder rejects. No capture of a real Nissan V0 fob exists.

## Line code and frame

Each pulse is expanded into 1..4 half-bit samples of its level (320..=700 µs gives 1, 800..=1230 gives 2,
1300..=1730 gives 3, 1800..=2350 gives 4; anything else ends the burst). The samples (cap 512) are processed at
the end of a burst or at 380 samples; fewer than 140 are dropped.

A stateless differential Manchester pairs them: two differing samples form a bit (0,1 gives 1; 1,0 gives 0),
two equal samples slip by one. At most 300 bits. The decoder takes the first position with 64 bits left where
bits `p..p+6` are the sync `110001` and bits `p+48..p+54` the marker `000010`, and reports those 64 bits. A
payload equal to the previous one is not reported again until `reset()`.

**KAT packing:** `data` = the payload, `data_count_bit` = 64; no serial, button or counter (the C leaves them 0).

## RF

- **Encoding:** Manchester
- **RF modulation:** AM
- **Encryption:** rolling code
- **Frequencies:** 315 MHz, 433.92 MHz
- **Check:** none (structural, 12 bits)
