---
layout: default
---

# Mazda V2 Protocol

**Rust module:** `src/protocols/mazda_v2.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/mazda_v2.c` (doctor-firmware ships the same wire protocol)

## Overview

Mazda rolling-code keyfobs as decoded by Flipper-ARF's `"Mazda V2"` protocol: a 500/1000 µs bi-phase line
code sampled one level per bit, a long preamble, the sync byte `0xEC` and 63 payload bits (button, header,
an 8-bit rolling index and a 39-bit hop). The ARF and doctor-firmware copies share the timing, sample
parser, field layout, both 256×6 lookup tables and the encoder; they differ only in the Flipper
custom-button mapping. KAT follows ARF (UP→Lock, DOWN→Unlock, LEFT→Trunk), which lines up with KAT's
generic button codes.

This is a different protocol from KAT's `Mazda V0` and `Mazda Siemens` (250/500 µs pair decoders at
433.92 MHz). Their frames are all "short" pulses to this sampler and can never form its sync, and its
1000 µs pulses break their pair decoders, so neither family is affected.

## Timing

| Parameter        | Value        | Notes                                              |
|------------------|--------------|----------------------------------------------------|
| TE (half bit)    | 500 µs       | "short" = 250–749 µs                               |
| 2TE (full bit)   | 1000 µs      | "long" = 750–1499 µs                               |
| Glitch filter    | ≤249 µs      | ignored outright (no state change)                 |
| Gap              | >1499 µs     | ends a burst (encoder: 3000 µs HIGH)               |
| Preamble         | >99 samples  | identical samples before the sync; encoder sends 180 short pairs |
| Frame            | 8 + 63 bits  | sync + payload after the preamble                  |

## Line Code and Sampler

Every bit lasts 2 TE, every bit boundary has a transition, and the level of the **second** half equals the
bit. The decoder stores one level per bit: a long pulse stores its own level; of two short pulses the first
is skipped and the second stored. The first short pulse after idle is the lead-in half bit and only arms the
sampler. Samples collect in a 512-entry buffer that is scanned every ~10 pulses once more than 110 samples
are buffered, when the buffer nears full, and at every gap.

## Frame Layout

On air: preamble (identical samples), sync `0xEC` (`1,1,1,0,1,1,0,0`), then 63 payload bits. A capture with
inverted levels shows a preamble of 1s and the sync `0x13`; its payload is inverted back.

The 63 payload bits are stored left-aligned in `data` (`value << 1`; bit 0 is always 0 and is never
transmitted), `data_count_bit` = 63, as in the C:

| `data` bits | Field         | Notes                                                        |
|-------------|---------------|--------------------------------------------------------------|
| 63–56       | button byte   | `& 0x7F`: 0x04 Lock, 0x08 Unlock, 0x10 Trunk, 0x06 / 0x0A / 0x0E alt Lock / Unlock / Trunk; bit 7 is carried through |
| 55–48       | hdr           |                                                              |
| 47–40       | hop0          |                                                              |
| 39–32       | hop1          |                                                              |
| 31–27       | hop2 (5 bits) |                                                              |
| 26–19       | idx           | 8-bit rolling index (KAT `counter`)                          |
| 18–16       | hop3 (3 bits) |                                                              |
| 15–8        | hop4          |                                                              |
| 7–1         | hop5 (7 bits) |                                                              |
| 0           | —             | always 0                                                     |

`button` = the full button byte, `counter` = idx. `serial` is not set: the frame has no plaintext serial
(the C's `generic.serial = idx` is an internal placeholder it never displays), so KAT does not show the
counter as a serial.

## RF

- **Encoding:** bi-phase (transition at every bit boundary, second-half level = bit), sampled one level per bit
- **RF modulation:** AM and FM
- **Encryption:** rolling code (8-bit index + 39-bit hop advanced through two 256×6 lookup tables)
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Idle** — pulses ≤249 µs are ignored; the first ≤749 µs pulse arms the sampler.
2. **Sample** — short pulses alternate skip/store, long pulses store; a >1499 µs gap triggers a final parse
   and returns to idle; a buffer of ≥508 samples forces a parse and is dropped if nothing is found.
3. **Parse** — find the exact sync (`0xEC`, or `0x13` for inverted levels) preceded by more than 99
   identical preamble samples (0s, or 1s when inverted), read the next 63 samples (inverting if needed), and
   require a valid button byte and a value other than 0 / `0xFFFFFFFF00000002`. Emit and consume the frame.

## Encoder

Supported — a real rolling-code generator, as in the C (`subghz_protocol_encoder_mazda_v2_deserialize` +
`mazda_v2_build_upload`):

- idx is bumped by 1 (the C's default rolling-counter multiplier);
- a Lock/Unlock/Trunk command re-stamps the button, keeping the captured button's alt family and bit 7;
  other commands keep the captured button;
- the new hop comes from the lookup tables: "inline" frames (primary button and `hop0 & 0x3C == 0`) get
  their idx-derived bits rewritten in place; otherwise the table row for the new idx (lock table for Lock
  buttons, default table otherwise) is XORed with the captured fob's delta, which is re-mixed when idx
  bits [2:1] change.

Upload: preamble, sync `0xEC` and the 63 payload bits through the C's two-state level encoder, a one-bit
tail and a 3000 µs HIGH gap, repeated 3 times (the C's default `Repeat`); the app keeps the frame sent on
the capture, so the next press sends the code after. Each copy starts with a 1000 µs HIGH (KAT, since
v1.8.7): the C's upload begins with the preamble's LOW, which on air merges into the silence before the
transmission, so the sampler armed on the first HIGH, read the wrong halves and neither the C decoder nor
KAT's could read the transmission; the idle sampler ignores a pulse over 749 µs, so the LOW after KAT's
lead-in is the pulse that arms it.

## Deviations from the C

- **Sync check.** The C's two-flag pattern loop accepts the normal sync only as an exact `0xEC`, but once the
  first sample matches the inverted pattern it accepts any remaining 7 bits, so an inverted frame is gated
  by a single sync sample. KAT requires the full inverted sync `0x13`. Genuine inverted frames carry the
  full pattern, so this only rejects noise.
- **Encoder preamble.** The C emits 180 (LOW, HIGH) pairs and then starts the sync's first `1` bit with a
  full 1000 µs HIGH, so on air the last preamble HIGH merges into a 1500 µs pulse — just over the
  decoder's own 1499 µs gap threshold. Compiled and run, the C encoder's exact-timing output does not
  decode through the C decoder (it does once that pulse is 1499 µs). The trailing preamble HIGH is a
  redundant half bit, so KAT drops it: the preamble is `LOW, (HIGH, LOW) × 179` and the sync starts with a
  clean 1000 µs HIGH. The C decoder accepts this waveform.
- **Debounce.** The C's 149-tick same-key debounce is omitted: KAT resets every decoder after each
  emission (which in the C also clears the remembered key), so it could never fire.

## Frames without a preamble

What a fob sends (the captures below): a press is a preamble of 180 bits and the frame **twice**, back to back.
While the button is held it goes on with a new frame every 100 ms or so, each behind a preamble of about 90 bits
and with bit 7 of the button byte set (`0x86`, `0x8A`). On the fobs with buttons `0x04 / 0x08 / 0x10` the second
copy starts 74 bits after the first instead of 71.

The C's parser wants more than 99 identical preamble samples directly in front of the sync, so it reads the first
frame of a press and nothing after it: not the second copy, and none of a held button's frames. FoxFW's Mazda V1
decoder (`mazda_v1.rs`) reads the same 71 bits without a preamble, behind a structural gate of its own (the sync
at either polarity, button `0x06 / 0x0A / 0x0E / 0x26`, frame bit 70 set, a marker rule). KAT runs both: a frame
Mazda V1's reader accepts with one of this protocol's buttons is reported **as Mazda V2**, with the fields above,
and can be sent again like any other. On the 300-press recording `more/MazdaV1F4_OEM2_300lck.sub` that is 616
frames instead of 293.

Not covered: the second copy of a frame with button `0x04 / 0x08 / 0x10` (Mazda V1's gate rejects those buttons;
the copies are identical, so no code is missed).

## Validation

Captures (`IMPORTS/Mazda/` and `IMPORTS/Mazda/more/`, added 2026-11): 28 recordings of several fobs (FM476,
433.92 MHz), single presses, 300-press sessions and held buttons. KAT lists 5730 frames on them: 4417 as Mazda V2
(3126 of them only through the frames-without-a-preamble rule above) and 1313 as Mazda V1 (button `0x26`). The
C this decoder is ported from (doctor-firmware's build, the same as Flipper-ARF's) lists 1291; FoxFW's C lists
5679, all as its Mazda V1. Two recordings differ from FoxFW: `more/Mazda_F4.sub` (buttons `0x04 / 0x08 / 0x10`,
which FoxFW's gate rejects: 46 frames, as in the C) and `more/MazdaF4V1_OEM_20ulk_long.sub` (64 against 59).
The older `IMPORTS/Mazda` RX-8 (433.889 MHz) and CX-50 (314.98 MHz) captures contain no usable burst at any
timing: the longest run of in-window pulses is ≤12.

The lookup tables were generated from the C source by a script, and a unit test checks their byte sums and
FNV-1a hashes against values computed from the C text. Golden vectors from the compiled C reference
(`extract` / `pack` / `advance_hop` / `mix_hop` / inline path, with the deserialize logic mirrored) check
`next_key` for 40 frames × 4 commands, and an upload fingerprint checks one encoder copy against the C
`mazda_v2_build_upload` output (minus the dropped half bit). Further tests cover encode→decode round trips
(3 decodes per upload, inline and table-driven frames, both tables, idx wrap, alt and bit-7 buttons),
inverted levels, rejection of corrupted sync bits / invalid buttons / a 99-sample preamble (100 accepted),
the strict inverted-sync check, registry decoding at 315 and 433.92 MHz, and silence on Mazda Siemens TX
frames.
