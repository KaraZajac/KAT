---
layout: default
---

# Hitag2 Protocol

**Rust module:** `src/protocols/hitag2_rke.rs` (the cipher is in `hitag2.rs`, key recovery in `hitag2_hell.rs`)
**Reference:** FoxFW `lib/subghz/protocols/hitag2.c` (`SUBGHZ_PROTOCOL_HITAG2_NAME = "Hitag2"`), after Garcia et al.,
"Lock It and Still Lose It", USENIX Security 2016, Figure 11.

## Overview

The NXP PCF7946/7947 Hitag2 remote-keyless-entry frame used by many makes. The reference flags it
`SubGhzProtocolTypeDynamic`, `433 | AM`. This is an air-format decoder, distinct from KAT's `hitag2.rs`, which is the
HITAG2 *cipher* that Fiat V1, Fiat V2 and Renault V1 use (and with which KAT checks a key recovered for this
protocol, see [Key recovery](#key-recovery)). Receive only, as in FoxFW (see [Transmit](#transmit)).

## Burst handling

Pulses shorter than 30 µs are ignored; a pulse of 1500 µs or more of either level ends the burst, as do 700 collected pulses.
A burst of at least 88 pulses is processed. `te` is the centre of the most populated 10 µs bin among widths 70..=320 µs
(one decoder for te 90 µs Fiat/Alfa up to te 240 µs Ford Ka). A stateless Manchester slicer then runs in both polarities
(HIGH = 0 first): a pulse within `max(40, 0.45 te)` of `2te` is one bit with its level, a pulse within it of `te` is a
half bit (the second half emits the bit), anything else drops the half bit.

## Frame (103 bits)

`marker (16) = 0x0001 | UID (32) | button (4) | lctr (10) | keystream (32) | 0 (1) | check (8)`

The scan takes each position `cs` (the marker's last bit) where bit `cs+79` is 0, the marker reads 0x0001, the XOR of the
ten bytes from `cs` equals the byte at `cs+80`, the UID is not 0 or 0xFFFFFFFF, and counter + keystream are not both zero
or both all ones.

| Field | Meaning |
|-------|---------|
| **serial** | UID |
| **button** | 4 bits |
| **counter** | `lctr`, the low 10 bits of the rolling counter, in clear |
| `extra` | `check << 32 | keystream` (the keystream needs the remote's key) |
| `data` | the C's `generic.data`: `UID << 32 | btn << 28 | lctr << 18 | keystream >> 14` |

## RF

- **Encoding:** Manchester (G. E. Thomas)
- **RF modulation:** AM (AM650)
- **Encryption:** Hitag2 keystream; XOR-parity check byte

## Transmit

None: FoxFW has no Hitag2 encoder to port. The protocol's encoder table in `hitag2.c` is all NULL, and so is the one in
the SD-plugin source `applications/main/subghz/protocols_ext/enc_hitag2.c` (FoxFW's split-out encoders), which the app's
`application.fam` does not build; the protocol has no `Send` flag. `hitag2.c` gives the reason: "the rolling code is
encrypted with a per-remote key, so there is no roundtrip encoder". The host oracle with every FoxFW plugin encoder
(`subghz_oracle_foxfwenc encode`) answers "no encoder" for each of 100 made-up key files, as KAT does. (FoxFW's
"Hitag-2", which transmits, is its Renault V1: KAT's `renault_v1.rs`.)

## Key recovery

Since 1.17.0 the signal menu's **Recover key (HiTag2)** ([HiTag2 key recovery](hitag2_hell.md)) also takes Hitag2
captures. From a frame the attack takes the UID, the 32-bit keystream as the hop, `lctr` as the counter and the
4-bit button, and it assumes Renault V1's IV, `lctr << 4 | button` (`hitag2::renault_authenticator`). Garcia et
al. give the IV as the remote's 28-bit counter followed by the button; the frame carries only the counter's low
10 bits. The search needs a second capture of the same remote with another keystream; a frame must be the 103
bits of this decoder, with a UID other than `00000000`/`FFFFFFFF`. With a wrong IV it comes back empty (a key
that fits both frames by chance has a probability of about 2^-15), and the message then says that the IV is not
confirmed.

The key found absorbs the counter's 18 bits above `lctr`, which the frame does not carry: it reproduces the
remote's frames until the 10-bit counter wraps, and then the search is offered again. KAT saves it in
`hitag2_keys.txt` in the standard byte order, marks the remote's captures as recovered and shows the key in their
detail panel's **HiTag2** row. It does not transmit Hitag2 RKE with it: the key only confirms the frames.

**Not verified on a real remote.** No recording in `IMPORTS/` holds two presses of a Hitag2 RKE remote (the
three frames KAT reads there are not one's, see Validation), so the IV has not been checked against a real key.
For this protocol the recovery is tested on synthetic frames only: an ignored test (`key_recovery_round_trip`) runs
the full search on two presses made with a known key and gets a key that reproduces both.

## Validation

- Three frames in two recordings decode as Hitag2, the same three in FoxFW's C, and none is a Hitag2 RKE remote's:
  two in `OPEL/more/opel_two_button_remote_unlock_x300.sub`, with two different UIDs, in stretches where no press of
  that [Opel Rolling](opel_rolling.md) remote is read, and one in `RENAULT/more/Renault_Duster_2010_KD_300ulk.sub`,
  a [Renault V1](renault_v1.md) remote's recording, with UID `FFFFFFFE` and keystream `FFFFFFFE`. The one older
  corpus file in the same family (an Opel Vectra) is read by KAT's Fiat V1 and by the older "Hitag-2" decoder of
  FoxFW, never by this one.
- Synthetic bursts (made-up UID, counter and keystream at te 90, 160 and 240 µs, both polarities) decode identically in
  the FoxFW C decoder (host oracle) and in KAT. A wrong check byte, a different marker, a 1 before the check, a zero UID,
  a zero counter with a zero keystream, and a short burst are rejected by both.
- When the decoder was added (1.9.0), what KAT read from IMPORTS and the Flipper unit-test corpus did not change.
