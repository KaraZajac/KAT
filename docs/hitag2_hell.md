---
layout: default
---

# HiTag2 key recovery (HiTag2 Hell)

**Rust modules:** `src/protocols/hitag2_hell.rs` (the attack), `src/protocols/hitag2_keys.rs` (the
key file), `src/app/hitag2_bf.rs` (the signal menu's action)
**References:** qDoctor (`github.com/thedoctorz/qDoctor`, main 7a69dec)
`lib/modules/cpp/doctor_subghz/`: `hitag2_bridge.c` `doctor_hitag2hell_recover`, `hitag2hell_common.c`,
`hitag2hell_kernel.inc`, `hitag2hell_f.inc`, `hitag2hell_cpu_kernel.c` (the CPU path KAT ports), and
the Dart workflow that turns a capture into a request (`hitag2_sub.dart`); Flipper-ARF's "Hitag2 BF"
L5 (`subghz_hitag2_hell.c`, the same attack) and its UID key database `hitag2_key_db.c` (from
doctor-firmware); the paper: Verstegen, Verdult and Bokslag, *Hitag 2 Hell — Brutally Optimizing
Guess-and-Determine Attacks*, WOOT 2018.

## Overview

Fiat V1, Renault V1 and Fiat V2 remotes sign each press with HITAG2, a 48-bit stream cipher, keyed per
car. KAT transmits fresh codes for a remote whose key it knows: one of the references' embedded keys,
the key derived from a Renault remote's serial, or a key saved for the remote. For any other remote it
can only replay the captured frame, as the references do.

**Recover key (HiTag2)** finds that key from two captures of the remote. One frame's 32-bit hop fits
about 2^16 keys; a second press (another counter, any button) leaves only the remote's. KAT saves the
key for the remote in `hitag2_keys.txt`, and from then on the remote's captures transmit fresh codes
with the requested button and the next counter.

## Using it

1. Capture two presses of the remote (any buttons; two presses of the same button work too), or load
   them from files.
2. Select one and press `Enter`: the signal menu offers **Recover key (HiTag2)** for a Fiat V1,
   Renault V1 or Fiat V2 frame whose key KAT does not know. KAT pairs it with another capture of the
   same remote (same UID, another hop): one with another button first, then the newest. With no second
   press in the list, it says so.
3. The search runs in the background on every core. The status bar's title shows its progress and the
   longest it can still take; **Cancel key recovery** in either capture's menu stops it, and quitting
   asks first. PSA key searches (`autobf`) keep running beside it, so PSA frames keep showing up.
4. On success the status line names the key and where it was saved, and every capture of the remote
   gets **Recovery: key recovered (HiTag2 Hell)** in its detail panel. TX Lock / Unlock / Trunk /
   Panic then send fresh codes, and the remote's Flipper key files carry the key on the firmwares'
   `Hitag2 Key` / `Hitag2 Epoch` lines. If no key fits the two captures, both say **no key (HiTag2
   Hell searched)**.

It runs only when asked, never through `autobf`: a search takes minutes of every core.

## Which captures

The attack needs a frame's UID, counter, button and 32-bit hop, as qDoctor reads them from a key file
(`Hitag2Sub.parse`):

| Protocol | UID | Counter, button (the cipher's IV) | Hop | Notes |
|---|---|---|---|---|
| [Fiat V1](fiat_v1.md) | frame bytes 2–5 | 10-bit counter, 4-bit button | 32 bits | exactly qDoctor's fields |
| [Renault V1](renault_v1.md) | same 13-byte frame | same | same | a frame whose serial-derived seed KAT recovers is not offered: KAT already transmits it |
| [Fiat V2](fiat_v2.md) | frame bytes 2–5 | Flipper-ARF's 10-bit counter, with both of ARF's button readings in one search | 32 bits | no reference has confirmed this IV against a real key; a wrong one finds nothing |

Fiat V1 and Renault V1 share frame and cipher, so their captures pair: the header alone decides which
name a frame gets ([Renault V1](renault_v1.md)), and one remote can come out under both. Not offered: [Renault V1
(ARF)](renault_v1_arf.md) (Flipper-ARF's type-table frames have no fixed 32-bit hop and a 24-bit serial
instead of a UID) and [Datsun Lada](datsun_lada.md), whose key is fixed (`0x4606` and the serial).

## How it works

With the UID and the counter known, every 48-bit register state after the cipher's initialisation
corresponds to exactly one key, so the search looks for the states whose first 32 keystream bits are
the hop. HiTag2 Hell guesses the register bits in the order the cipher's filter needs them, one
keystream bit per layer, and drops a branch as soon as a filter output disagrees:

- layer 0: the 20 bits the first filter output depends on, the 2^19 patterns that give the hop's first
  bit (the *candidates*, the search's progress counter);
- per candidate, 1024 kernel calls guess 15 more bits, and the bitsliced kernel completes the register
  over the next keystream bits, 32 or 256 guesses per machine word; the remaining hop bits only check.

About 2^16 states survive per hop. Each gives a key, which must also produce the second press's hop: a
wrong key survives that with probability about 2^-15. qDoctor's CPU path sweeps every candidate and
then checks the states it collected; KAT checks each state as it is found, as qDoctor's GPU drivers do,
and stops at the key, but keeps the C's order (the lowest candidate wins), so the result is the C's.

The key found is the one that works with the counter's upper bits (Fiat V1's "epoch", counter bits 10
and up) at zero: the remote's key when its epoch is 0, otherwise an equivalent key that works while the
epoch stays the same. The attack's counter must fit in 10 bits; Fiat V1 and Renault V1 frames carry
exactly that.

## How long it takes

A search covers at most the 2^19 candidates; it stops at the remote's, on average halfway. Measured on
an Intel i9-9980HK (8 cores, 16 threads, fixed 2.4 GHz), release build, the same candidates for every
row:

| Kernel | one thread | full search, one thread | 16 threads | full search, 16 threads |
|---|---|---|---|---|
| qDoctor's C, gcc -O2 (32 lanes, OpenMP) | 61.6 ms per candidate | 9.0 h | 152 candidates/s | 57 min |
| KAT, 32-lane word (the C's) | 54.4 ms | 7.9 h | 167/s | 52 min |
| KAT, 64-lane word | 53.9 ms | 7.9 h | 172/s | 51 min |
| KAT, 256 lanes, SSE2 | 48.8 ms | 7.1 h | 171/s | 51 min |
| **KAT, 256 lanes, AVX2 (what KAT runs)** | **23.0 ms** | **3.4 h** | **429/s** | **20 min** |

Wider words pay less than their width: after a few keystream bits most lanes of a word are dead, and
the deep levels, which dominate, cost one pass per surviving guess whatever the width; only AVX2's
single-instruction 256-bit operations win (2.8× the C). KAT picks AVX2 when the CPU has it. On a CPU
without it a full search takes about an hour on 16 threads.

Full recoveries on the same machine:

- qDoctor's own sample ("Load Hell sample", two presses of one remote): qDoctor's key `E99BDE9C2813`, in
  17.5 minutes; its candidate lies 91 % into the search.
- `IMPORTS/OPEL/Opel Vectra 2008.sub`, a real remote (six presses, all Fiat V1; one of them was listed
  as Renault V1 up to 1.12.0): the key found from its first two presses signs all six, in 17.3 minutes
  (81 % into the search).

## Is a GPU worth it?

qDoctor runs the same kernel on a GPU (Vulkan on Android and Windows, Metal on Apple), with its CPU path
as the fallback. KAT stays on the CPU: a full search is about 20 minutes on a 16-thread laptop CPU
with AVX2, a one-off per remote that saves its key for good, and a GPU path would add a graphics
dependency and drivers to a terminal tool that otherwise needs only libhackrf.

## The key file

`hitag2_keys.txt` in KAT's config folder (`~/.config/KAT/`, on Windows
`C:\Users\<you>\.config\KAT\`) is Flipper-ARF's and doctor-firmware's UID key database, which they keep
in `subghz/assets/hitag2_keys.txt` on the SD card: same format and byte order, so it can be copied to
and from a Flipper running either.

```
# Flipper Hitag2 UID keys
# UID KEY EPOCH SEED
12345678 0123456789AB 0 -
```

One line per remote: the UID (8 hex digits), the key (12 hex digits, or `-`), the epoch (decimal, or
`-` without a key) and doctor-firmware's Renault seed (8 hex digits, or `-`; KAT keeps it but recovers
seeds from the frame itself). The first line of a UID counts. The key is stored as each firmware's
protocol stores it: Fiat V1 (and Fiat V2) in the order the Fiat cipher takes it, the standard HITAG2
order rotated by two bytes; Renault V1 in the standard order. KAT tries a key in both orders, so a key
found from Fiat V1 captures serves the same remote decoded as Renault V1. Every use checks the key
against the captured hop first; a line that does not verify is ignored, as in the firmwares.

KAT reads the file at startup and writes it when **Recover key (HiTag2)** finds a key: the remote's
line is replaced (its seed kept), or appended; other lines and comments stay. A key can also be added by
hand. The `Hitag2 Key` line of a loaded key file is not read (KAT has no key entry).

**With the key**, the encoders build a fresh frame (otherwise they replay the captured one):

- **Fiat V1:** after its embedded keys, the saved key with its epoch, for the frame's button or any
  button (as doctor-firmware checks), then Flipper-ARF's dictionary and heuristics. The fresh frame:
  requested button, counter + 1, hop signed with the key ([Fiat V1](fiat_v1.md)).
- **Renault V1:** with no serial-derived seed, the saved key with the epoch that reproduces the
  captured hop (doctor-firmware's search over the saved epoch and the next seven), then doctor-firmware's
  fresh frame: requested button, counter + 1 (carrying into the epoch), its hop, the captured tail bits
  ([Renault V1](renault_v1.md)).
- **Fiat V2:** after Flipper-ARF's known keys, the saved key with the first of ARF's four IV readings
  that reproduces the hop ([Fiat V2](fiat_v2.md)).

## Validation

Against qDoctor's unmodified C (main 7a69dec) built with gcc -O2, with and without OpenMP, plus a
harness that includes `hitag2hell_common.c` for its static helpers:

- The kernel's schedule (every sub-filter, register bit and guess, in order and at the same loop
  level, 227 steps) was compared mechanically with `hitag2hell_kernel.inc`.
- Unit tests: the cipher (`hitag2hell_authenticator` / `_init` vectors from the C, and equality with
  KAT's Fiat and Renault authenticators), the filter's sub-functions, layer 0 (count, first, last and
  1000th candidate, and a hash over all 524,288 for a hop of each first bit), state finishing and the
  key formula, the kernel's matches in the C's order for chosen candidates and every word width, and
  end-to-end recovery on the true candidate of known keys, also through the protocol mapping.
- Long runs (release, `#[ignore]`): 7680 candidates of two hops give exactly the C's matches, in the
  C's order; a full sweep over one hop finds 70,473 states, exactly the C's count; full recoveries of
  qDoctor's sample (the C's key, state and UID), of a key generated from KAT's HITAG2 test vectors (the
  C recovers all three such keys), and of the Opel Vectra capture.
- The key file's lines are written and read as `hitag2_key_db.c` writes and reads them; saved keys make
  the three encoders send fresh frames signed with the key.
