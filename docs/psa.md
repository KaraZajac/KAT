---
layout: default
---

# PSA (Peugeot/Citroën) Protocol

**Rust module:** `src/protocols/psa.rs`
**References:** `REFERENCES/ProtoPirate/protocols/psa.c` + `psa_crypto.c` + `psa_crypto_bf.c` (primary),
`REFERENCES/Flipper-ARF/lib/subghz/protocols/psa.c` ("PSA GROUP"), `REFERENCES/doctor-firmware/lib/subghz/protocols/psa.c`

## Overview

Groupe PSA keyfobs send an 80-bit Manchester frame (the references call it "128 bit": a 64-bit
key1 plus a 64-bit `Key_2` of which 16 bits are used). Two crypto families share the frame:

- **mode 0x23 (XOR):** a byte-shuffle/XOR stage with a 4-bit nibble checksum. Decrypts instantly.
- **mode 0x36 (TEA):** a TEA variant whose key is derived from the fob's serial (BF1: key counter
  `0x23000000 | serial`, two TEA encryptions of constants; BF2: `0xF3000000 | serial` XORed into a
  fixed schedule). The serial is unknown until a 2 × 2^24 brute force finds it.

ProtoPirate decodes 0x23 frames and marks the rest "Type 0x36 — brute force deferred" for a manual
"Brute force" action; Flipper-ARF (`decrypt_file`) and doctor-firmware (in-app helper) do the same.
KAT does the same: a TEA frame is reported undecrypted (no serial, not encoder-capable), and the signal
menu's **Recover key (PSA brute force)** action runs the brute force on a background thread (progress in
the status line, **Cancel key recovery** to stop it; ≤ ~0.5 s on a 16-core PC). On a hit the capture gets
its serial, button and counter and the TX actions appear.

`IMPORTS/GROUPE PSA` holds two genuine fobs: an XOR fob (serial 0x99EB25, counters 0x39B/0x39C)
and a TEA fob (serial 0x014590 recovered by the BF1 brute force, counters 0x9158E53/0x9158E55).

## Timing

| Parameter        | Value            | Notes                                           |
|------------------|------------------|-------------------------------------------------|
| Short            | 250 µs           | 151..349 µs (State2 also takes 350 µs)           |
| Long             | 500 µs           | 351..599 µs                                      |
| Preamble         | > 0x46 short LOWs | then a 401..599 µs LOW (the sync)               |
| End marker       | 1000 µs HIGH     | 801..1199 µs, only at exactly 80 bits           |
| AM path (ARF)    | 60..200 µs half-bit | adaptive te, ≥ 8 preamble cells               |
| Encoder          | 250 / 500 / 1000 µs | 80 preamble pairs, 10 repeats                |

## Frame Layout

Byte buffer (`psa_setup_byte_buffer`): `b0..b7` = key1 MSB first, `b8..b9` = key2. `b0..b1` are not
encrypted; the low nibble of `b1` is the **0xA sync** every genuine frame carries.

- **mode 0x23:** checksum = (sum of the nibbles of `b2..b7`) × 16 must equal `b8`'s high nibble; the
  XOR stage then gives serial = `b2..b4`, counter = `b5..b6` (16 bit), CRC byte = `b7`, button =
  `b8 & 0xF`. The fast path runs only when `b8`'s low nibble is in {0,1,2,4,5,6}
  (`psa_direct_xor_allowed_by_key2`); any other frame is a TEA frame.
- **mode 0x36:** `b2..b9` = TEA(v0, v1); v0 = serial << 8 | button << 4 | counter >> 24, v1 =
  counter << 8 | byte-sum CRC (28-bit counter). BF2 frames carry a CRC-16 (poly 0x8005) instead.

`DecodedSignal`: `data` = key1, `extra` = key2 (the 64-bit `Key_2`, 16 bits used),
`data_count_bit` = 128, `serial`/`button`/`counter` (low 16 bits of the 28-bit TEA counter) when
decrypted. Buttons (Flipper-ARF): 0 = Lock, 1 = Unlock, 2 = Trunk.

## RF

- **Encoding:** Manchester (Flipper `manchester_advance`, `level ? ShortLow : ShortHigh`)
- **RF modulation:** AM and FM (all three references)
- **Encryption:** XOR + nibble checksum (0x23) / TEA with serial-derived keys (0x36)
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder

KAT runs two state machines side by side on every pulse, each exactly as in its C file:

1. **ProtoPirate machine** — State0 picks the path from the first HIGH (151..349 µs → State1,
   85..150 µs → State3). State1 counts 250 µs LOWs; a 500 µs LOW after more than 0x46 of them
   enters State2, which Manchester-decodes 64 key1 + 16 key2 bits and reports the frame on a
   1000 µs end HIGH if the 0xA sync is present. Its States 3/4 (125/250 µs) feed only LOW pulses to
   the Manchester decoder, which then only emits 0 bits, so that path never reports a frame; it is
   ported because it holds the machine like the C does. **Every frame this machine reports is
   reported by KAT.**
2. **Flipper-ARF machine** — the same States 0..2, plus ARF's adaptive AM States 3/4 (ported from
   ARF's `psa.js`): te measured from ≥ 8 preamble cells, Manchester at the measured te, and on a
   terminator an 80- or 88-bit window realigned to the offset with the 0xA nibble, ≥ 4 distinct key1
   bytes and the most leading 1s. **Only these AM-path frames are taken from this machine**, and
   only when they decrypt.

Separate machines matter: once ARF's AM State4 overruns to 121 bits with a non-80/88 window its C
never leaves State4 (`new_state = State0` is not stored), which in one shared machine would also
stop the 250/500 µs path. On a stream of interleaved AM and 250/500 µs frames a single ARF-shaped
machine reports 15 of the 1943 frames ProtoPirate decodes; KAT reports all 1943.

Every reported frame goes through ProtoPirate's routing: the XOR fast path when key2 allows it,
otherwise the brute force.

## Brute Force (key recovery)

`psa_brute_force_run`: BF1 tries key counters 0x23000000..0x23FFFFFF (working key = two TEA
encryptions under the BF1 schedule; accept when the decrypted serial echoes the counter and the
byte-sum CRC matches), then BF2 0xF3000000..0xF3FFFFFF (schedule XOR counter; serial echo + CRC-16).
KAT splits each range into 32768-candidate units over all cores and returns the lowest hit, i.e. the
same result as the C's sequential loop; `brute_force()` takes a cancel flag and a progress counter.

Cost on a 16-core machine: full miss ≈ 0.46 s release (up to ~1.3 s while other builds load the
machine; 3.7 s in a debug build); BF1 hits ≤ 0.33 s (the cost grows with the serial: the genuine TEA
fob, serial 0x014590, is found after 83 344 candidates); the C references take 4.6 s
single-threaded for a full miss. The search runs only on demand ("Recover key"), off the UI thread;
a frame it cannot decrypt stays undecrypted (no serial, not encoder-capable), as in the references.

Once the serial is known, decryption is O(1) (`tea_decrypt_with_serial`), so `encode()` needs only
`data`, `extra` and `serial`.

## Encoder

ProtoPirate `psa_encoder_build_upload` (identical in ARF and doctor-firmware): the plaintext fields
are re-derived from the captured key1/key2 (+ the brute-forced serial for TEA frames), then sent with
counter + 1 (PP emulate plugin `current_counter++`, ARF/doctor `+ mult`) and the requested button
(KAT Lock/Unlock/Trunk → 0/1/2; Panic and others keep the captured button).

- **mode 0x23:** XOR-encrypt `b2..b7`, keeping the captured `b9` (if key2 ≠ 0) and searching `b8`'s
  high nibble (and `b9` when there is no key2) for a checksum fixed point; fallback `b9` = 0x23.
- **mode 0x36:** TEA-encrypt under the BF1 key of `0x23000000 | serial` (all references use the BF1
  key, also for a BF2-recovered frame).
- `b0..b1` (the sync) are kept from the captured key1.
- Upload (325 entries): 80 × (HIGH 250, LOW 250), LOW 250 + HIGH 500 + LOW 250, 64 + 16 bits
  MSB first (1 = HIGH/LOW, 0 = LOW/HIGH, 250 µs halves), HIGH 1000 + LOW 1000; **10 copies**
  (PP/ARF; doctor sends 3).

Re-encoding the TEA fob's first frame with counter 0x9158E55 reproduces its later genuine frame bit
for bit, and the XOR fob's 0x39C frame is reproduced from its 0x39B frame with `b9` = 0x23.

Reference quirk kept: the 1000 µs end HIGH merges with a trailing 0 bit's 250 µs HIGH half into
1250 µs, outside the decoders' 801..1199 µs window, so ProtoPirate's (and KAT's) decoder does not
re-decode such uploads of its own encoder. Genuine fobs send a ~895 µs end HIGH (897 µs, and
1133/1145 µs after a 0 bit, in IMPORTS).

## Deviations from the References (with evidence)

- **ARF's 250/500 µs acceptance and routing not used.** ARF also accepts frames without the 0xA sync
  when the XOR checksum passes and runs the XOR path without ProtoPirate's key2 filter. It
  misdecodes the genuine TEA frame `F47A2EA11A4C7CAF/2927` as serial E3ED81, button 9 (ProtoPirate's
  brute force finds the fob's serial 0x014590), and accepts 7 frames of
  `VAG/Test_55_unlock_and_55_lock_suran.sub` / `VAG/Vw pasat B7.sub` (no 0xA sync).
- **doctor-firmware's publish gate not used.** It reports an undecrypted frame only when key2 fails
  the XOR filter, which drops the genuine TEA frame `F47A25DEC980F0DE/1621`.
- **AM-path frames must decrypt (KAT gate).** ARF's AM completion is weak (8 preamble cells, then any
  80/88-bit window with the 0xA nibble at one of up to 9 offsets) and KAT's registry resets re-arm
  it: on `honda/Lock_honda.sub` it accepts Honda Static chips plus inter-frame noise as
  `4CDA57A60039A20E/4FF4` (the unmodified ARF psa.c, replayed with the registry's reset points,
  reports the same window), which decrypts neither by XOR nor by BF1/BF2.
- **Buttons from ARF.** ProtoPirate's emulate plugin maps Lock/Unlock/Trunk/Panic to 0x1/0x2/0x4/0x8
  (its generic template, shared verbatim with Ford V1; 0x8 cannot even be an XOR-mode button under
  PP's own key2 filter). All four genuine frames carry button 0, which PP's map lacks; ARF's codes
  (0 Lock, 1 Unlock, 2 Trunk) are used.
- **BF2 CRC comparison: either variant.** PP/doctor compare the CRC-16 with `b8:b9`, ARF with
  `b7:b9`; no genuine BF2 frame exists to settle it, so both are accepted (a spurious hit would need
  a ~2^-40 coincidence).

## Retired: PSA2 ("PSA OLD")

Flipper-ARF's `psa2.c` ("PSA OLD") was registered on 2026-04-16 and reverted the next day; the file
is in ARF's tree but not in its registry. PSA decodes the two `IMPORTS/GROUPE PSA` captures PSA2
decoded, with the same serial/button/counter, and additionally reports the TEA fob's frames, whose
key the "Recover key" action finds (serial 0x014590).
`get()` resolves "PSA2", "PSA OLD" (and ARF's "PSA GROUP") to PSA; a PSA2 capture (key1 + fields,
no key2) still encodes in mode 0x23 from its decoded fields.

## Verification

C oracles: the unmodified ProtoPirate, Flipper-ARF and doctor-firmware `psa.c` compiled on the host
with stub SDK headers, compared with KAT through `oracle_dump` in `psa.rs`
(`KAT_PSA_ORACLE=<file> cargo test oracle_dump -- --ignored --nocapture`; `P`/`U`/`E`/`B` lines
for decoder pulses, uploads, `encode()` calls and brute forces, KAT additions switched off for `P`).

- **Decoder:** identical frame for frame (pulse index, key1, key2, decrypted fields) to ProtoPirate's
  frames ∪ the ARF machine's AM frames on 11.4 M pulses of randomized streams (encoder frames with
  jitter, dropped/split/stretched pulses and broken end markers; adaptive-AM frames at te 65..190 µs
  with and without resets; interleaved AM/250 µs frames; random noise; 8 071 frames in the final
  run, repeated with other seeds) and on every IMPORTS capture in both polarities.
- **Encoder:** 6000 random uploads identical pulse for pulse to ProtoPirate's encoder (both modes,
  key2 = 0 / low byte 0xFF / key1 = 0 edge cases); 3000 full `encode()` calls (field re-derivation,
  button map, counter + 1) identical to the C encoder fed with the fields the C decrypts.
- **Brute force:** 2 × 38 frames (each: 24 BF1 from the C encoder, 5 + 5 BF2 in the PP and ARF CRC
  variants, 4 misses) give the C's result (ARF's `decrypt_file` for the ARF variant).
- **Real captures:** both GROUPE PSA files decode to exactly ProtoPirate's key1/key2/fields and
  brute-force results; the encoder reproduces the fobs' genuine frames.
