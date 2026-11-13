---
layout: default
---

# KeeLoq Protocol

**Rust module:** `src/protocols/keeloq.rs` (+ `src/protocols/keeloq_common.rs`)
**Reference:** `REFERENCES/Flipper-ARF/lib/subghz/protocols/keeloq.c`, `keeloq_common.c`
(Unleashed lineage; ProtoPirate has no KeeLoq protocol)

## Overview

Generic Microchip KeeLoq (HCS2xx/HCS3xx-style) remotes: gates, garage doors, barriers and car alarms. The
decoder identifies the manufacturer from the embedded keystore exactly as ARF's
`check_remote_controller` does and reports `KeeLoq (<manufacturer>)`. Like the C, it also reports
every frame it cannot identify, as `KeeLoq (Unknown)`: serial and button from the fixed part, no
counter (the hop is encrypted with a key KAT does not have), decode and replay only. It runs on every
band (the C has no frequency filter), so 300–318, 390 MHz and other gate frequencies decode too. KAT's
separate KeeLoq generic fallback ([keeloq_generic.md](keeloq_generic.md)) still tries captures that no
decoder reports.

Validated on real recordings from the rtl_433 test suite (HCS200 remotes, IQ through KAT's receive
pipeline): `Microchip/HCS200` → `KeeLoq (Magic_3)` serial D0921 with consecutive counters,
`keeloq/01` → `KeeLoq (Unknown)` serial 174ACD (rtl_433 reports the same serial).

## Timing

| Parameter | Value    | Notes                                                        |
|-----------|----------|--------------------------------------------------------------|
| Short     | 400 µs   | ±180 µs (ARF's gate-app copy; its library copy: ±140 µs)     |
| Long      | 800 µs   | ±360 µs                                                      |
| Preamble  | > 2 × 400 µs HIGH | LOW ~400 µs (or ~800 µs: Superrollo/HCS361)         |
| Header    | 4000 µs  | ±1800 µs (LOW; Superrollo also a 10·Te HIGH sync)            |
| Bit 1 / 0 | 400 HIGH + 800 LOW / 800 HIGH + 400 LOW |                               |
| End       | LOW ≥ 980 µs | 64..66 bits, first 64 kept                               |

**Half rate (RogueMaster, November 2026).** The HCS200/HCS300/HCS301 encoders have a baud-rate-select field, so
the same 66-bit frame also ships at Te 200 µs. As in RogueMaster's `keeloq.c` (from Unleashed), the preamble is
accepted at either rate (HIGH 400 ± 180 or 200 ± 90 µs; preamble LOWs of 200 ± 90 µs too), and the sync gap
decides which. A gap within 2000 ± 450 µs locks the half rate at the encoder's measured Te (gap / 10). Otherwise
the standard band (4000 ± 1800 µs) locks 400 µs. Every bit and end check then scales with the locked Te: tolerance
Te·180/400, so 90 µs at 200, and the end of frame is a LOW ≥ 2·Te + tolerance. The two gap bands overlap from 2200
to 2450 µs, where the half rate is tried first (RogueMaster's order). A half-rate decode keeps its Te for the
encoder and the key file.
## Frame Layout (64 bits, MSB-first, bit-reversed = fix | hop)

- **fix (32 bits):** button (4) << 28 | serial (28)
- **hop (32 bits):** KeeLoq(button << 28 | discriminator << 16 | counter)

## Identification

1. **AN-Motors** — hop = `cnt8 cnt8 btn 0x404` (no key); counter = hop >> 16.
2. **HCS101** — hop = `cnt16 btn 0x000`; counter = hop >> 16.
3. **Reference selector** — each keystore entry with ONLY its learning type: SIMPLE (Pecinin: 12-bit
   serial check, button from the plaintext), NORMAL (Centurion: discriminator 0x1CE), SECURE (BFT: seed 0,
   then seed = serial), MAGIC_XOR_TYPE_1, MAGIC_SERIAL_TYPE_1/2/3, AERF, ERREKA, PUJOL, SIMPLE_JCM, JCM_GEN2, STAGNOLI,
   TELCOMA_TABLE_HI, UNKNOWN
   (simple, normal, secure, magic-xor, each with the key and its byte-mirrored copy). Check: plaintext
   button = fix button and plaintext[23:16] = serial[7:0] or 0.
   doctor-firmware's and RogueMaster's per-manufacturer rules replace that check for five names, the
   union with Flipper-ARF, which has none of them:
   - **KEY** (SIMPLE): the 12-bit discriminator is 0x2FF.
   - **Monarch** (NORMAL): the discriminator is 0x100 or 0x280 (Dorcas), kept as the seed.
   - **Phantom/Hiland** (NORMAL, RogueMaster; `Phantom_Hiland` in the keystore): the discriminator's low 10
     bits are the serial's, and the whole 12-bit field is kept as the seed. Its top 2 bits differ per remote
     and the receiver ignores them.
   - **Dea_Mio** (SIMPLE, RogueMaster): the discriminator's low 8 bits are the serial's (a low byte of 0 is
     not taken for any serial, as the generic check would), and the whole 12-bit field is kept as the seed.
     Bits 8 and 9 are the serial's too; the top 2 are the HCS300's OVR flags, which the chip sets as its
     counter wraps and never clears, so they differ per remote and the receiver ignores them.

The first match names the manufacturer. A random 64-bit frame is identified ≈6% of the time, so strongly
gated KeeLoq-PWM protocols (Sheriff CFM) are registered before KeeLoq.

## Keystore learning-type correction (DoorHan)

The selector applies each entry's learning type, so the type must be right. KAT's embedded keystore lists
DoorHan (`8455F43584941223`) as NORMAL (2), but every real DoorHan remote available decrypts with that key
under SIMPLE learning (1) and under nothing else (checked against every keystore key × learning mode × byte
order):

| Capture (Flipper-ARF `applications/debug/unit_tests/resources/unit_tests/subghz/`) | Serial | Button | Counters |
|---|---|---|---|
| `doorhan_raw.sub` | 0D36C681 | 2 | 04BD, 04BE, 04BF |
| `test_random_raw.sub` | 0D36C681 | 2 | 04C0, 04C1, 04C2 |
| `test_random_raw.sub` | 047D94C1 | 8, 1 | 0490, 0491 |
| `doorhan.sub` (decoded key `4850F07233789514`) | 08A91ECC | 2 | 0004 |

Three remotes with consecutive counters and discriminator = serial & 0x3FF leave no doubt, so
`keeloq_common::LEARNING_TYPE_FIXES` maps DoorHan to SIMPLE in the keystore view both KeeLoq and Star Line
use. Without it these remotes are not identified at all.

## Your own manufacturer keys

KAT's keystore is built in. Like the Flipper firmwares, KAT also reads a plaintext **user keystore**:
put a file named `keeloq_mfcodes_user` in `~/.config/KAT/`, in the Flipper's format, and its keys are
used after the built-in ones (identification, the Star Line / Sheriff / generic fallbacks, and the
encoder, so identified remotes can be re-encoded):

```text
Filetype: Flipper SubGhz Keystore File
Version: 0
Encryption: 0
0123456789ABCDEF:2:My_Gate_Brand
```

Each line is `KEY:TYPE:NAME`: the 64-bit manufacturer key as 16 hex digits, the learning type in the
Flipper's numbering (1 simple, 2 normal, 3 secure, 4 magic XOR type 1, 6–8 magic serial types 1–3, …;
0 = unknown, KAT tries simple/normal/secure/magic XOR with the key and its byte-mirrored copy) and the
manufacturer name shown as `KeeLoq (<name>)`.

Types 16-19 are doctor-firmware's (ported from its `keeloq_common.c` / `keeloq.c`; type 20 stays KAT's
Star Line alias, not doctor's PANDORA_HOP):

| Type | Name | Learning |
|------|------|----------|
| 16 | JCM_GEN2 | Intermediate key from the key and the fix part, one KeeLoq decrypt, then a byte shuffle into the final key. The learning byte is searched over 0-255 on decode and kept as the seed; the encoder reuses it. |
| 17 | STAGNOLI | Magic XOR type 1 with the fix part's top byte forced to `0xA0`. |
| 18 | TELCOMA_TABLE_HI | Table-driven: 32 rounds of `x ^= table[x & 3]; x = rotl(x, 1)` on the 28-bit serial give the key's high word, 32 more the low word. The 4-word table is the key of this entry (words 0-1) and of a type 19 entry (words 2-3); without both, the entry does nothing. Name the type 18 entry as the brand. |
| 19 | TELCOMA_TABLE_LO | The second half of the Telcoma table. Never matches on its own. | Encrypted Flipper keystores (`Encryption: 1`) cannot be
read. KAT reports how many keys it loaded in the status line at start-up (and on stderr for
`kat --decode`).

## Recovering a magic-serial manufacturer key

Flipper-ARF's **KeeLoq Decrypt** ("BF2") finds the manufacturer key of a KeeLoq remote it has no key
for, when the remote uses one of the "magic serial" learning types. KAT runs the same search on every
CPU core:

- **In the app:** select a `KeeLoq (Unknown)` capture and choose **Recover key (KeeLoq magic
  serial)**. It uses every capture of that remote in the list: same serial, different presses, at
  least two. `:klbf [ID] [auto|6|7|8]` does the same from the command line, and `:klbf stop` cancels.
- **Headless:** `kat --keeloq-bf <files|folders> [--type auto|6|7|8] [--save]` reads `.sub` files
  (RAW or key) and IQ recordings, picks the remote with the most captures and prints the keys that fit.
  `--save` adds the best one to the user keystore. A path that does not exist stops it before the
  search (exit status 2); the exit status is 0 when a key was found, else 1. The progress line
  (percent, keys/s, ETA) is shown only when stderr is a terminal.

For these types the device key is the remote's `fix` combined with part of the manufacturer key, so
only that part is unknown:

| Type | Device key | Unknown | Keys | Time (8-core i9 laptop, AVX2) |
|---|---|---|---|---|
| 6, magic serial 1 | manufacturer key bits 0–31, serial bytes | 32 bits | 2^32 | ~5 s |
| 7, magic serial 2 | manufacturer key bits 0–31, `fix` byte-swapped | 32 bits | 2^32 | ~5 s |
| 8, magic serial 3 | manufacturer key bits 24–63, serial bits 0–23 | 40 bits | 2^40 | ~20 min |

Auto tries type 6, then 7. Type 8 is opt-in and needs three captures.

**How a key is accepted.** A candidate must decrypt every capture to that frame's button (4 bits) and
discriminator `serial & 0x3FF` (10 bits). The counters must also be distinct and within ±256 of the
first capture's. False keys are rare:

- a 2^32 search from two captures leaves about 0.1 false keys, and none from three;
- a 2^40 search from two captures would leave about 30, which is why type 8 needs a third.

When several keys fit, the one with the closest counters is used and the others are listed.

**What happens with the key.** As Flipper-ARF does, KAT names it `BF_<serial>` (seven hex digits) and
adds it to the user keystore, `~/.config/KAT/keeloq_mfcodes_user`, creating the file with the Flipper
header if needed. The remote's captures are decoded again with it: they become
`KeeLoq (BF_<serial>)` with counters, and transmit like gate remotes (Send next code, Send button…).

**Differences from the reference.** ARF's scene takes two saved captures and a type (Auto = 6 → 7 → 8)
and sends them over Bluetooth to a companion app that searches 2^32 keys. That app isn't published,
so KAT's search is its own, built on the learning functions the decoder uses:

- **Type 8 is searched completely (2^40).** The keystore's own type 8 keys (Magic_3, Magic_4) use all
  40 unknown bits, so a 2^32 search could not find them.
- **The check uses ARF's 10-bit discriminator.** The scene prepares it as `serial & 0x3FF` for the
  app; the decoder itself compares 8 bits.

**Validation.**

- The bitsliced decryption (64 or 256 keys per pass) equals the scalar one on every lane.
- Full 2^32 searches find random type 6 and type 7 keys (`full_search_finds_random_keys`, an ignored
  test; run it with `--release`).
- In the app, three presses of a synthetic type 6 remote were recovered, saved and re-decoded.
- rtl_433's real HCS200 remote (`Microchip/HCS200/01`, serial D0921) uses the keystore's Magic_3 key
  (type 8). From three of its recordings, a search around that key finds exactly it
  (`finds_the_real_hcs200_magic_3_key`). The complete 2^40 search over all 12 recordings
  (`kat --keeloq-bf Microchip/HCS200/01 --type 8`) found that key and no other, with all 12 counters
  (0ABE–0AD0), in 23 minutes on 12 threads of the 8-core laptop.

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK)
- **Encryption:** KeeLoq
- **Frequencies:** 315 MHz, 433.92 MHz, 868.35 MHz

## Encoder

ARF `gen_data` + `encode_to_timings` (counter mode 0): manufacturer, learning mode, counter and seed are
re-derived from the key — restricted to the manufacturer the capture names (`KeeLoq (<name>)`, e.g. a
key file's `Manufacture` or an "Add Manually" remote), as ARF's encoder restricts its search to the
file's `Manufacture`, so a key whose frame an earlier keystore entry would also validate, or a BFT key
with its own seed, is transmitted with its own manufacturer; counter + 1 (wraps to 0). `fix = button << 28 | serial` (Pecinin: serial only).
Discriminator per manufacturer (default serial & 0x3FF; Aprimatic parity form; 12-bit for DTM_Neo,
FAAC_RC,XT, Mutanco/Clemsa, Came_Space, Genius_Bravo, GSN, Rosh, Rossi, Pecinin, Steelmate, Cardin_S449,
Stilmatic, …; 8-bit for NICE_Smilo/MHOUSE, JCM_Tech, Pujol, Erreka; 0 Beninca/Merlin; 0x1CE Centurion; 0x2FF
KEY; Monarch's received discriminator (0x100 or 0x280, the seed; 0x100 when the seed holds neither, Flipper-ARF's
fixed value); Phantom/Hiland's received discriminator (the seed when its low 10 bits are the serial's, else
serial & 0x3FF); Dea_Mio's received discriminator (the seed when its low 8 bits are the serial's, else
serial & 0x3FF with both OVR bits set, which is the other firmwares' fixed form)). Hop by the entry's learning type; UNKNOWN entries per matched mode with the key
as stored (reference quirk). MAGIC_SERIAL_2/3 have no encode case in the C → the captured frame is
re-sent. AN-Motors/HCS101 rebuild their plaintext hop; unidentified → the
captured hop is replayed. Programming mode (button 0xF): BFT sends its seed as hop, Aprimatic 0x1A2B3C4D,
Dea_Mio 0, and Erreka its seed as FoxFW does (`PROG_MODE_KEELOQ_ERREKA`).

Upload: 11 × (400, 400), 400 HIGH, 4000 LOW, 64 bits, status bit (400/800), 400 HIGH, 16 ms LOW (Sommer
11.6 ms); "Superrollo" uses ARF's HCS361 framing. Sent 3 times (C `encoder.repeat`). A half-rate capture (or a key
file with `TE: 200`) is sent at Te 200 µs, every pulse and the gap halved, as RogueMaster's encoder does; its
other counter modes (7 and up: DoorHan forced-resync packs) are not ported.

**Key files.** `Bit: 64`, `Key`, then `Seed` (4 hex bytes) for BFT (Flipper-ARF) and, as doctor-firmware and
RogueMaster write it, for Erreka, Monarch and Phantom/Hiland, and as RogueMaster does for Dea_Mio. Then `Manufacture`, then RogueMaster's `TE: 200`
for a half-rate frame only, so standard files keep the layout older builds read. On load, a `TE` other than 200
is the standard rate. An Erreka file's `Seed` decrypts its frame, as doctor-firmware's / RogueMaster's: their
"Add Manually" draws one, while the decode flow tries seed 0.

Buttons are 4-bit function codes; KAT Lock/Unlock/Trunk/Panic (1/2/4/8) and codes 1..F are sent as is.
For ARF's "shifted" brands (DoorHan, Pandora_*, Alligator_S-275, …) codes 2/4/8/1 are labelled B1..B4.

## Validation

C oracle (unmodified ARF `keeloq.c`/`keeloq_common.c`, KAT's keystore with the DoorHan correction): every
identification matches field-for-field (manufacturer, serial, button, counter, seed) and every encoder
upload pulse-for-pulse on randomized frames for every keystore entry and learning type. Real remotes in
ARF's unit-test captures: three DoorHan remotes (consecutive counters), an AN-Motors and a Stilmatic
remote. The IMPORTS sweep has no KeeLoq capture.

The half rate and the four manufacturer rules (November 2026) are checked by the tests `keeloq_half_rate` and
`keeloq_discriminator_rules`: frames at Te 185, 200 and 215 µs decode and are sent back at 200 µs. KEY, Monarch A
and B, and Phantom/Hiland (a made-up key, all four top-bit values) are identified, re-sent with their
discriminator and round-trip through key files. No real half-rate or Phantom/Hiland recording is in the corpus.

Dea_Mio's discriminator (RogueMaster at 0cfd4bf) is checked by the same test and against RogueMaster's C: a
made-up remote with each of the four values of the two OVR bits is identified with its 12 bits as the seed, and
the next code KAT sends for each is the one RogueMaster's encoder sends from KAT's key file (four different
hops; Momentum's encoder sends the both-set form for all four, as KAT did up to 1.12.0). A key file without a
`Seed`, or with one that is not this serial's, is sent in the both-set form. No real DEA Mio recording is in the
corpus.
