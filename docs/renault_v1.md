---
layout: default
---

# Renault V1 Protocol (HITAG2 / PCF79xx RKE)

**Rust module:** `src/protocols/renault_v1.rs` (cipher and seed search: `src/protocols/hitag2.rs`)
**Reference:** `REFERENCES/ProtoPirate/protocols/renault_v1.c` (added upstream in 1287d10 "move newer
Renault variants into Renault V1"; later commits only refactor it). Also compared: doctor-firmware
`lib/subghz/protocols/renault_v1.c` (same decoder, plus a key-less replay) and Flipper-ARF (see below).

## Overview

Renault V1 covers the newer Renault keyfobs built on a HITAG2 (PCF79xx) transponder. Each frame starts
with a 1500 µs LOW / 1000 µs HIGH header followed by a 104-bit Manchester frame whose half-bit length the
decoder learns on the fly (nominal 125 µs). The payload carries the 32-bit UID, a 4-bit button, a 10-bit
counter and a 32-bit HITAG2 authenticator ("hop"), closed by an XOR byte.

The 104-bit frame is byte-for-byte the same as Fiat V1's (`00 01 ‖ uid ‖ btn/cnt ‖ hop ‖ tail ‖ xor`);
the protocols differ only in timing and sync.

Some remotes derive their HITAG2 key from the serial. For those, ProtoPirate's "Hitag2 BF" recovers the
hidden IV bits (counter bits 10-15 and a 12-bit seed) and can then sign fresh frames. KAT runs the same
search automatically on every decoded frame, so such remotes are transmittable straight away; any other
frame is replayed byte-for-byte as doctor-firmware does.

| Capability | ProtoPirate | doctor-firmware | KAT |
|---|---|---|---|
| Decode | yes | yes (narrower header windows) | yes (ProtoPirate windows) |
| Serial-derived seed search (2^18) | on demand (minutes on a Flipper) | on demand | automatic (~10 ms) |
| TX with recovered seed | fresh frame | fresh frame | fresh frame (ProtoPirate upload) |
| TX with typed-in "Hitag2 Key" (+ epoch, key DB) | yes (epoch 0) | yes (8-epoch search, UID key DB) | UID key DB (`hitag2_keys.txt`, 8-epoch search); no key entry |
| HiTag2 key recovery (Hitag2Hell) | — | — (qDoctor app) | on demand: **Recover key (HiTag2)**, 2 captures ([HiTag2 key recovery](hitag2_hell.md)) |
| TX without key or seed | refuses | byte-identical replay | byte-identical replay (doctor upload) |

**Flipper-ARF:** its bundled ProtoPirate app ships this exact `renault_v1.c`. ARF's own firmware
`lib/subghz/protocols/renault_v1.c` is a different decoder registered under the same name: the old
82-bit type-table frames (types 04/0C/1A/3B/3F + "Dynamic", 12-20 preamble bits) that ProtoPirate's
Renault V0 carried until 1287d10 replaced them with this HITAG2 decoder. KAT ports it separately as
[Renault V1 (ARF)](renault_v1_arf.md) (`renault_v1_arf.rs`). ARF's `hitag2_seed.c` is a verbatim
copy of ProtoPirate's seed model (ported here); its `hitag2_key_db.c` (UID → key/seed file, from
doctor-firmware) is KAT's `hitag2_keys.txt` (`hitag2_keys.rs`).

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Header LOW | 1150-2200 µs | nominal 1500 µs |
| Header HIGH | 800-1150 µs | nominal 1000 µs |
| TE (half-bit) | 125 µs nominal | adaptive per level, clamped 70-185 µs |
| Initial TE estimate | HIGH 120 µs, LOW 150 µs | reset at every header |
| Long/short threshold | 1.5 × TE, clamped 150-210 µs | `hitag2_data_threshold` |
| Glitch filter | ≤ 49 µs ignored | |
| Frame end | any pulse > 520 µs | re-examined as a possible header |

## Frame Layout (104 Manchester bits, MSB first, decoded words inverted)

| Bits | Content |
|------|---------|
| 0-15 | header word, must equal `0x0001` |
| 16-47 | UID (32 bits) |
| 48-51 | button (4 bits) |
| 52-61 | counter (10 bits) |
| 62-93 | hop (32-bit HITAG2 authenticator) |
| 94-95 | tail (2 bits) |
| 96-103 | XOR of the 10 payload bytes (UID … tail) |

ProtoPirate calls bits 16-79 "Key" (64 bits) and bits 80-103 "Key_2" (24 bits), 88 bits in total.

**Buttons:** `0` Sync, `1` Lock, `2` Unlock, `4` Trunk, `8` Panic (the same values as KAT's generic codes).

### Serial-derived key ("seed") scheme

`hitag2_encrypt_frame(serial, cnt, btn, seed)`: the key is `hitag2_serial_permute(serial)` (6 bytes; the
register starts as `serial ‖ perm[4] perm[5]`, `perm[0..4]` are the injected key bits) and the 32-bit IV is
`iv[0] = cnt[3:0] ‖ btn`, `iv[1] = cnt[11:4]`, `iv[2] = seed ‖ cnt[15:12]`, `iv[3] = seed`. A frame shows
the button and counter bits 0-9, so only 18 IV bits are unknown. ProtoPirate's brute force tries them in
the order `iv[1] |= (c & 3) << 6, iv[2] = c >> 2, iv[3] = c >> 10` for `c = 0 .. 0x3FFFF` and keeps the
first IV whose keystream equals the frame's hop; it then stores "Recovered: 01", "Seed" (= the IV) and
the full 16-bit counter `((seed >> 12) & 0xFF0) | ((seed << 4) & 0xF000) | (seed >> 28)`. A random frame
passes with probability ≈ 2^-14.

### KAT `DecodedSignal` packing

| Field | Value |
|-------|-------|
| `serial` | UID |
| `button` | Renault button code |
| `counter` | 10-bit counter, or the 16-bit counter when the seed was recovered |
| `crc_valid` | XOR verified (always true on emission) |
| `data` | wire bits 0-63 (`0001` header + Key[63:16]) |
| `extra` | wire bits 64-103, right-aligned (Key[15:0] ‖ Key_2) |
| `data_count_bit` | 104 |
| `encoder_capable` | always true (fresh frames with a recovered seed, replay otherwise) |

This is the same layout Fiat V1 uses, so an identical frame packs identically. ProtoPirate's view is
`Key = data << 16 | extra >> 24` and `Key_2 = extra & 0xFFFFFF`. No recovery state is stored: `encode()`
re-runs the deterministic seed search on the frame, so `.fob` files exported by any KAT version work.

## RF

- **Encoding:** Manchester (bit 1 = HIGH then LOW), adaptive TE
- **RF modulation:** AM or FM (ProtoPirate flags both)
- **Encryption:** HITAG2 (32-bit authenticator)
- **Frequencies:** 315 MHz, 433.92 MHz, 868.35 MHz

## Decoder Steps

1. **Reset** — wait for a LOW of 1150-2200 µs.
2. **CheckSync** — a HIGH of 800-1150 µs starts Data (Manchester state `Start1`, TE estimates 120/150 µs).
   Another header-length LOW restarts CheckSync; anything else returns to Reset.
3. **Data** — ignore pulses ≤ 49 µs; a pulse > 520 µs returns to Reset and is examined there. Otherwise
   classify long/short against the level's TE threshold (short pulses also update the TE estimate) and
   run the canonical Flipper Manchester state machine. After 16 bits the inverted word is the header,
   after 80 bits the inverted 64-bit Key, after 104 bits the inverted 24-bit Key_2.
4. **Accept** — header `0x0001`, XOR byte, UID not `00000000`/`FFFFFFFF` (KAT addition), and not identical
   to the previous accepted frame (memory cleared by `reset()`). Counting restarts for the next frame.
5. **Seed search** — ProtoPirate's 2^18 brute force on the payload (same order, same first match; the
   init rounds of shared IV prefixes are computed once and each keystream stops at its first wrong
   bit: ≈10 ms in a release build, ≈65 ms unoptimised, cached per frame so a burst searches once).
   On success `counter` becomes the 16-bit counter.
6. **Emit** — on the pulse after the frame's last mid-bit (see Validation for why).

## Encoder

`encode()` takes the 11-byte payload from `data`/`extra` (it must pass the decoder's gates) and:

1. **Seed recovered** (ProtoPirate encoder with "Recovered: 01" + emulate plugin): a fresh frame
   `hitag2_encrypt_frame(uid, counter16 + 1, button, seed)`. Button: KAT Lock/Unlock/Trunk/Panic =
   Renault `1/2/4/8` (plugin Up/OK/Left/Right); other codes keep the captured button. doctor-firmware
   builds the same frame.
   **Upload** (`renault_v1_encoder_get_upload`): 250 × (HIGH 125, LOW 125), 3 long frames
   [LOW 1500, HIGH 1000, `0x0001` (16 bits), Key (64), Key_2 (24)], 3 short frames [LOW 1500, HIGH 1000,
   `0x0001`, Key[27:18] (10 bits), LOW 21500]; bit 1 = (HIGH, LOW), 0 = (LOW, HIGH); sent once.
2. **Saved key** (doctor-firmware's UID key database: `hitag2_keys.txt` in KAT's config folder, where
   **Recover key (HiTag2)** saves the keys it finds, standard HITAG2 byte order; a key saved in Fiat V1's
   order for the same remote works too): the epoch that reproduces the captured hop
   (`hitag2_hell_find_epoch`: the record's epoch and the next seven), then a fresh frame as
   `hitag2_encoder_hell_next_frame` builds it: the requested button (else the captured one), counter
   `epoch << 10 | cnt10` + 1, hop `hitag2::renault_authenticator(uid, btn, counter, key)` (= Fiat V1's
   cipher with the key rotated by two bytes), the captured tail bits; doctor-firmware's upload as in 3.
   The key file (`to_flipper_key`) then gets doctor-firmware's `Hitag2 Key` + `Hitag2 Epoch` lines.
   ProtoPirate's typed-in "Hitag2 Key" (`hitag2_authenticator(uid, btn, cnt10 + 1, key)`, epoch 0) is not
   reachable: KAT has no key entry.
3. **No seed, no key:** doctor-firmware's encoder has no failure path without a key or seed and replays the
   captured Key/Key_2 byte-for-byte (any button). KAT does the same with doctor-firmware's upload in its
   default "Kangoo" variant: 255 pairs + one HIGH 125, 3 long frames, 8 short frames each closed by
   LOW 20000; sent once. (doctor-firmware switches to its "Old" variant — 2 long frames and a 115.5 ms gap —
   when exactly two identical long frames were received, a per-burst count KAT's per-frame registry
   cannot observe.) ProtoPirate refuses to transmit in this case.

## Validation

- No Renault capture exists in `IMPORTS/`; no other capture decodes as Renault V1 (sweep unchanged).
- The decoder was compared against ProtoPirate's `renault_v1.c` compiled on the host over ~700 randomly
  generated bursts (random fields, TE 95-165 µs, ±20 % jitter, glitches, corrupted XORs, header
  variations): every frame matched, each reported exactly one pulse after the C. Re-checked on 2400
  encoder uploads (clean and ±13 % jitter with a 0.85-1.2 TE scale): identical frames.
- Seed search: equal to ProtoPirate's `hitag2_brute_force_run` on 1600 frames (800 seed-scheme frames —
  same recovered IV — and 800 random frames, all not found).
- Encoder: equal, frame and merged pulse list, to ProtoPirate's encoder after its BF patch + emulate press
  on 600 seed-scheme frames (random buttons incl. non-Renault codes), and to doctor-firmware's encoder on
  600 frames without a seed (replay). The two references build the same seed-path frame.
- **Ordering vs Fiat V1:** Renault's header window also accepts the 1230 µs gap + 850 µs lead of Fiat V1
  type-B bursts, and the frames are identical, so Renault V1 decodes Fiat V1 type-B repeats when run on
  its own. The C would fire one pulse before Fiat V1 completes the same frame; KAT delays Renault V1's
  emission by exactly that pulse so both fire together and Fiat V1, earlier in the registry, wins.
  A recording that ends on the pulse the C fires on still gives the frame: it is reported when the input
  ends.
  A Renault remote fast enough for Fiat V1's 100/200 µs window would likewise be labelled Fiat V1.
- Unit tests: synthetic bursts, adaptive TE from 100 to 150 µs, gate rejection (XOR, header word, UID),
  duplicate memory and reset, emission timing, registry decode at 315/433/868 MHz, Fiat V1 type-B bursts
  staying Fiat V1, seed recovery on C vectors, seed-path TX frames per button and the ProtoPirate upload
  against the C, the doctor-firmware replay upload against the C, invalid signals not encoding, and
  encoder output round-tripping through the registry on all three bands.
