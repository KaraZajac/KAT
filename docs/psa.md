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

**`autobf`** (`config.ini`, on by default) runs that search without being asked, as qDoctor does with
its `doctor_subghz` library: every undecrypted PSA capture, received live, from an IQ recording, a
`.sub` file or a restored session, gets it in the background, one search at a time (the **Recover key**
action goes first). A hit fills in the fields as above; a full miss marks the capture "no key (PSA
BF1/BF2 searched)". The detail panel's **Recovery** row and the vulnerability panel say which: a
recovered key makes the capture encoder-capable, so it counts as vulnerable and can be exported and
sent. `kat --decode` runs the same search in line and ends the frame's line with `key=BF1`, `key=BF2`
or `key=none`. Ten doctor-firmware key files a tester sorted as crackable with qDoctor's library all
give the serial qDoctor found, in 0.02–0.5 s each.

`IMPORTS/GROUPE PSA` holds two genuine fobs: an XOR fob (serial 0x99EB25, counters 0x39B/0x39C)
and a TEA fob (serial 0x014590 recovered by the BF1 brute force, counters 0x9158E53/0x9158E55).

## Timing

| Parameter        | Value            | Notes                                           |
|------------------|------------------|-------------------------------------------------|
| Short            | 250 µs           | 151..349 µs (State2 also takes 350 µs)           |
| Long             | 500 µs           | 351..599 µs                                      |
| Preamble         | > 0x46 short LOWs | then a 401..599 µs LOW (the sync)               |
| End marker       | 1000 µs HIGH     | 801..1199 µs, and (KAT) 1051..1449 µs: a Flipper's 1000 µs end after a final 0 bit; only at exactly 80 bits. doctor-firmware needs none (frame complete at the 80th bit) |
| 125 µs path (doctor) | 125 / 250 µs | ≥ 0x45 short pulses, then a 250 µs one; complete at the 80th bit (or on a 500 ± 99 µs end) |
| AM path (ARF)    | 60..200 µs half-bit | adaptive te, ≥ 8 preamble cells               |
| Encoder          | 250 / 500 µs, end HIGH 900 µs | 80 preamble pairs, 10 repeats (references: end HIGH 1000 µs) |

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

KAT runs three state machines side by side on every pulse, each exactly as in its C file:

1. **ProtoPirate machine** — State0 picks the path from the first HIGH (151..349 µs → State1,
   85..150 µs → State3). State1 counts 250 µs LOWs; a 500 µs LOW after more than 0x46 of them
   enters State2, which Manchester-decodes 64 key1 + 16 key2 bits and reports the frame on a
   1000 µs end HIGH if the 0xA sync is present. Its States 3/4 (125/250 µs) feed only LOW pulses to
   the Manchester decoder, which then only emits 0 bits, so that path never reports a frame; it is
   ported because it holds the machine like the C does. **Every frame this machine reports is
   reported by KAT.**
2. **doctor-firmware machine** (since 1.8.5) — the same States 0..2 with a `>=` 0x46 preamble
   threshold, and the frame complete as its 80th bit arrives (`psa_try_complete_at_key2`), whatever
   follows: no end pulse, or one of any length. Its States 3/4 are a working 125/250 µs path (State0
   takes a 76..150 µs first HIGH; at least 0x45 short pulses, then a 250 µs one; Manchester on both
   levels; complete at the 80th bit). A 250 µs frame of this machine waits for ProtoPirate's end
   pulse (the first HIGH after the 80th bit, or a LOW over 1200 µs): if the ProtoPirate machine takes
   the frame, it is reported as before; otherwise only the 80th-bit rule read it, and it **must
   decrypt** like an AM-path frame. So do the 125 µs path's frames. A recording that ends before
   that pulse ends the wait: the frame is then the 80th-bit rule's alone (doctor-firmware has reported
   it on that bit).
3. **Flipper-ARF machine** — the same States 0..2, plus ARF's adaptive AM States 3/4 (ported from
   ARF's `psa.js`): te measured from ≥ 8 preamble cells, Manchester at the measured te, and on a
   terminator an 80- or 88-bit window realigned to the offset with the 0xA nibble, ≥ 4 distinct key1
   bytes and the most leading 1s. **Only these AM-path frames are taken from this machine**, and
   only when they decrypt.

"Must decrypt": an XOR frame is reported when the fast path decrypts it; a TEA frame, with `autobf`,
as `PSA [unchecked]`, kept out of the list until the key search finds its key (dropped if none
fits); without `autobf` it is dropped. After a reported frame all three machines start again, as
the registry restarts every decoder, so a frame is reported once.

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
single-threaded for a full miss. The search runs off the UI thread, on demand ("Recover key") or with
`autobf`; a frame it cannot decrypt stays undecrypted (no serial, not encoder-capable), as in the
references.

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
  MSB first (1 = HIGH/LOW, 0 = LOW/HIGH, 250 µs halves), HIGH 900 + LOW 1000 (the references: HIGH
  1000, see Deviations); **10 copies** (PP/ARF; doctor sends 3).

Re-encoding the TEA fob's first frame with counter 0x9158E55 reproduces its later genuine frame bit
for bit, and the XOR fob's 0x39C frame is reproduced from its 0x39B frame with `b9` = 0x23.

The references' 1000 µs end HIGH merges with a trailing 0 bit's 250 µs HIGH half into 1250 µs, outside
their decoders' 801..1199 µs window: no reference decodes such an upload of its own encoder, or a
Flipper sending a saved file of such a frame. Genuine fobs end at ~900 µs (both IMPORTS fobs: medians
893–909 µs after a final 1 bit, ~1140 µs merged with a final 0 bit). KAT fixes both sides (see
Deviations).

## Deviations from the References (with evidence)

- **ARF's 250/500 µs acceptance and routing not used.** ARF also accepts frames without the 0xA sync
  when the XOR checksum passes and runs the XOR path without ProtoPirate's key2 filter. It
  misdecodes the genuine TEA frame `F47A2EA11A4C7CAF/2927` as serial E3ED81, button 9 (ProtoPirate's
  brute force finds the fob's serial 0x014590), and accepts 7 frames of
  `VAG/Test_55_unlock_and_55_lock_suran.sub` / `VAG/Vw pasat B7.sub` (no 0xA sync).
- **doctor-firmware's publish gate not used.** It reports an undecrypted frame only when key2 fails
  the XOR filter, which drops the genuine TEA frame `F47A25DEC980F0DE/1621` (and would hide 2 of
  the 10 key files a tester's doctor-firmware build saved).
- **Frames only doctor-firmware's 80th-bit rule reads must decrypt (KAT gate).** The rule alone
  also reads the first 80 bits of longer Manchester frames at similar timing whose bits 12–15
  happen to be 0xA: on `VAG/VW Polo 5 2017 lock x5.sub` it reports `FCFA7B0B8272C8FF/9020`, which
  decrypts neither by XOR nor by BF1/BF2 (doctor-firmware's own publish gate hides that one; KAT
  does not use the gate, above). A frame ProtoPirate's end-pulse rule also takes is reported as
  before; the others are checked like AM-path frames. The IMPORTS decodes are unchanged without
  `autobf`; with it, two all-ones frames 1.8.4 listed with no key (`Peugeot 307 open.sub`
  `FF5A7C3F9F960FFF/FFFF`, `VW Polo 5 2017 unlock2.sub` `737AD61FFFFFFFFF/FFFF`, an end pulse after
  a long gap) are no longer listed.
- **AM-path frames must decrypt (KAT gate).** ARF's AM completion is weak (8 preamble cells, then any
  80/88-bit window with the 0xA nibble at one of up to 9 offsets) and KAT's registry resets re-arm
  it: on `honda/Lock_honda.sub` it accepts Honda Static chips plus inter-frame noise as
  `4CDA57A60039A20E/4FF4` (the unmodified ARF psa.c, replayed with the registry's reset points,
  reports the same window), which decrypts neither by XOR nor by BF1/BF2. Without `autobf` an AM-path
  frame the fast (XOR) path cannot decrypt is dropped, so a TEA fob whose half-bits only ARF's
  adaptive path takes (60–200 µs) showed none of its frames. With `autobf` it is reported as `PSA
  [unchecked]`, which KAT keeps out of the list while the brute force runs: listed with its fields if
  the key is found (the TEA fob's frame at 70/100/160 µs half-bits: serial 0x014590, BF1), dropped if
  not (the Honda window).
- **The end marker of a frame a Flipper sends (receiver).** Flipper firmwares (doctor-firmware,
  Flipper-ARF, ProtoPirate) transmit a saved PSA file with the encoder above: a frame whose last bit
  is 0 ends in a 1250 µs HIGH, which their decoders and KAT ≤ 1.8.3 reject, so half of all frames a
  Flipper replays from a saved file never appeared. KAT also accepts 1051..1449 µs at the 80th bit.
  Checked over the air (Flipper Zero → HackRF, a made-up TEA remote, the Flipper's upload replayed as
  RAW): KAT 1.8.3 received none of the frames ending in a 0 bit, on AM and FM; with the wider window,
  every copy; frames with the genuine fobs' end decoded in both. The tester's ten PSA key files sent
  through the receive pipeline the same way (`radio/sim.rs` `sweep_psa_key_files`): 5 of 10 → 10 of 10.
- **The end marker KAT sends (transmitter).** KAT's encoder ends a frame with the genuine fobs'
  900 µs HIGH instead of the references' 1000 µs, so a frame whose last bit is 0 ends in ~1150 µs,
  inside every reference decoder's window. Checked over the air (HackRF → Flipper Zero `subghz rx_raw`,
  the recording decoded with the references' window, i.e. KAT 1.8.3's decoder): a 1.8.3 transmission
  of such a frame was not readable, the new one is (9 of 10 copies; frames ending in a 1 bit: both).
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
- **Encoder:** 6000 random uploads identical pulse for pulse to ProtoPirate's encoder before the end
  HIGH changed to 900 µs (both modes,
  key2 = 0 / low byte 0xFF / key1 = 0 edge cases); 3000 full `encode()` calls (field re-derivation,
  button map, counter + 1) identical to the C encoder fed with the fields the C decrypts.
- **Brute force:** 2 × 38 frames (each: 24 BF1 from the C encoder, 5 + 5 BF2 in the PP and ARF CRC
  variants, 4 misses) give the C's result (ARF's `decrypt_file` for the ARF variant).
- **Real captures:** both GROUPE PSA files decode to exactly ProtoPirate's key1/key2/fields and
  brute-force results; the encoder reproduces the fobs' genuine frames.
- **doctor-firmware's rule (1.8.5):** 184 variations of four made-up TEA frames (end pulse none or
  250–5000 µs at 250 µs, none or 125–2500 µs at 125 µs; preambles of 40–72 pairs; ±10–60 µs duty
  skew; half-bits of 100–300 µs) through the unmodified doctor-firmware, ProtoPirate and Flipper-ARF
  `psa.c` (host C harness) and KAT: KAT reads every frame doctor-firmware reads (1.8.4 missed 19 of
  its 68; doctor-firmware's publish gate drops half the frames, KAT reads 144).
- **Over the air (1.8.5):** a Flipper Zero (Momentum, RAW files) sending made-up TEA remotes to KAT
  on a HackRF, live, with `autobf`:
  - the tester's four FSK settings (`UFM` ×3, `F3_31`) × a Flipper's upload, a 250 µs fob, doctor's
    125 µs format, a 125 µs fob and ARF's fast format: 80/80 (1.8.4 the same);
  - the carrier keyed only for the frame, as a fob does (`UFM`, `F3_31`): end pulse 900 µs, none,
    three copies, 125 µs with and without an end: 40/40. 1.8.4 listed the 8 frames without an end
    pulse as Unknown signals; doctor-firmware's C reads all 8 from the pulses KAT received;
  - OOK, stock 2-FSK ±47.6 kHz and `UFM`: no end pulse, 500 / 2000 µs ends, a 71-pair preamble,
    125 µs with a 125 µs end or none: 84/84 (1.8.4 decodes 12 of these files, those with a 900 µs end);
  - 1.8.4's own check files (stock OOK / 2-FSK, Flipper and fob ends, last bit 0 / 1): 8/8;
  - `IMPORTS` replays: every frame of the four GROUPE PSA recordings, with its key (as from the
    files); `honda/Lock_honda.sub` and two VW Polo 5 files give no PSA frame and the same Honda
    Static frames as from the file.
- **Simulator (1.8.5):** the tester's ten key files through the receive pipeline (`radio/sim.rs`
  `sweep_psa_key_files`), now also sent without an end pulse at 250 and 125 µs: 2880/2880. Without
  an end pulse at 250 µs, 1.8.4 decodes 0 of those 360 presses (OOK and both FSK deviations).
