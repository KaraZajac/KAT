---
layout: default
---

# Compustar 1WG3R (rtl_433 port, receive only; FoxFW key files)

**Rust module:** `src/protocols/compustar_1wg3r.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/compustar_1wg3r.c` ("Compustar 1WG3R Car Remote", model `Compustar-1WG3R`),
slicer `pulse_slicer_pwm`; key files: FoxFW `lib/subghz/protocols/compustar.c` ("Compustar", its own port of the
same decoder)

## Overview

Compustar 1WG3R-SH / 1WAMR-1900 car-alarm and remote-start remotes, 433.92 MHz: a fixed 16-bit ID and a button code
with its inverse. Button combinations, a "long press" and a "secondary mode" (batches of codes) give further codes.
The air format is the one the Flipper references read as **Scher-Khan "MAGIC CODE, Static"** (35 bits, no fields),
see [Overlap with Scher-Khan](#overlap-with-scher-khan).

## Timing / modulation

| Parameter   | Value          | Notes |
|-------------|----------------|-------|
| Modulation  | `OOK_PULSE_PWM` | |
| short_width | 708 µs         | → 1 (pulses up to 892 µs) |
| long_width  | 1076 µs        | → 0 (893–1262 µs) |
| sync_width  | 1448 µs        | longer pulses are a sync (new row) |
| reset_limit | 1532 µs        | a longer gap ends the message |
| tolerance   | 0              | the slicer's split points apply |

The remote sends ~720/760 µs (1) and ~1090/1140 µs (0) pulse/gap pairs; every 35-bit frame follows three
~1470/1495 µs sync pairs; a press is one long message.

## Frame Layout (35 bits per row)

| Bits  | Field |
|-------|-------|
| 0–15  | 16-bit ID |
| 16–18 | `111` |
| 19–26 | inverted button code |
| 27–34 | button code (bit 7: secondary mode) |

## Decoder Steps

1. rtl_433's PWM slicer over KAT's AM pairs (and rtl_433's pulse-detector package rules).
2. For every row (rtl_433's `decode_fn` loop): a 5-bit row of 1s is the batch separator (it clears the duplicate
   memory); rows under 35 bits are skipped; `111` and bit 39 = 0 (`DECODE_ABORT_EARLY`), ID ≠ 0000 / FFFF
   (`DECODE_FAIL_SANITY`), inverse byte (`DECODE_FAIL_MIC`); a row equal (35 bits) to the previous valid row of
   the message is not reported again.
3. KAT reports each new event as soon as its row is final, keeping the message's duplicate memory across the
   registry's reset after its own report — so a press gives what rtl_433 outputs for it (one Lock event for a
   held Lock button; every batch of a secondary-mode press).

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` | `id` |
| `button` | `button_code` (8 bits); native names = `button_str` (rtl_433's map on `code & 0x7F`: Lock, Panic, Start, Trunk, Unlock, Long Press — one code can name several — then "Secondary Mode" for bit 0x80; "?Secondary Mode" for an unmapped code, as the C writes it) |
| `data` | the row's first 35 bits, `data_count_bit` 35 |
| `crc_valid` | true (inverse byte) |
| `extra` | `1` when the row has a 36th bit and it is 1, else none (kept for FoxFW's key; not shown) |

## Key files

FoxFW decodes the same remote and saves it with its generic serializer; KAT writes and loads that file
(`Compustar` is an alias of this protocol):

```text
Protocol: Compustar
Bit: 36
Key: 00 00 00 05 A3 CE F1 0E
```

`Key` holds the 35 frame bits followed by one more bit: FoxFW reads 35- and 36-bit frames, appends a 0 to a 35-bit
one and keeps a 36th bit as received (it does not check it), so KAT writes the row's 36th bit, 0 for a 35-bit row.
Loading is FoxFW's `deserialize` (exactly `Bit: 36`) followed by the decoder's checks on the 35 frame bits (`111`,
ID, inverse byte), which FoxFW's decoder also applies before it saves a frame: a key that fails them, or has bits
set above its 36, is refused. A loaded key is the decode of its frame, so KAT's files round-trip exactly.

## Encoder

None. rtl_433 has no encoder, and FoxFW's SD encoder plugin (`applications/main/subghz/protocols_ext/enc_compustar.c`)
is a stub: its `deserialize` returns `SubGhzProtocolStatusErrorEncoderGetUpload` for every key file ("the PWM gap
timing is not published by rtl_433 — TX would be guesswork"), so a Flipper running FoxFW sends nothing for a
Compustar file. KAT does the same: no TX buttons, no "Send next code"; Replay of recorded pulses remains.

## Overlap with Scher-Khan

**Why KAT read these remotes as Scher-Khan.** Scher-Khan (ProtoPirate = Flipper-ARF = doctor-firmware) reads PWM
750/1100 µs ± 160 with a 1500 µs header: the same air format. Its 35-bit "MAGIC CODE, Static" frame is the
Compustar frame with every bit inverted (Scher-Khan reads a short pulse as 0; its start bit is the frame's first
bit): Compustar Lock `9EB4 111 F0 0F` = Scher-Khan `30A580FF0`. The references report it with Serial / Btn / Cnt 0
and nothing to check.

**The references do the same.** KAT's pulses of the five recordings, saved as RAW `.sub`: the ProtoPirate,
Flipper-ARF and doctor-firmware oracles decode them as Scher-Khan 35-bit with KAT's keys and counts (lock ×3,
long press ×8, secondary mode ×16 + ×8 + ×1, trunk ×7, unlock ×7); the garage app has no Scher-Khan.

**What Compustar adds.** The ID, the button code and an integrity check (inverse byte + `111` marker), valid on
every frame of these recordings. No IMPORTS or Flipper unit-test capture is a Scher-Khan 35-bit frame, so there is
no capture of a "MAGIC CODE, Static" remote that does not fit the Compustar layout.

**What KAT does (a deviation from the references).** Scher-Khan sits earlier in the registry and would report the
frames it locks onto first, one feed before Compustar's row is final (on the sync pulse that ends each frame), with
Compustar reporting the rest (the first frame of a burst lacks Scher-Khan's two header pulses, the last its stop
pulse): one press showed both, e.g. lock: Compustar-1WG3R ×2 + Scher-Khan ×3. So Scher-Khan leaves to Compustar
every 35-bit frame whose inverted bits pass Compustar's checks (`compustar_1wg3r::is_valid_frame`, called at
Scher-Khan's stop bit): these recordings decode as Compustar-1WG3R only, with rtl_433's fields (lock 15, long press
25, secondary mode 135 / 130 / 132, trunk 2, unlock 7), and IMPORTS is unchanged. A 35-bit frame that fails the
checks is still Scher-Khan's "MAGIC CODE, Static". The cost: those frames have no Scher-Khan encoder or Scher-Khan
key-file export (Compustar is receive only and saves FoxFW's Compustar key file; Replay of the recorded pulses
remains). A Scher-Khan key file still loads and transmits as before (`decode_key` does not go through the stream).

## Validation

- rtl_433_tests `compustar/1wg3r/01` (5 recordings) through KAT's receive pipeline into this decoder alone: the
  distinct events equal rtl_433's (lock 15; long press 25; secondary mode 135, 130, 132; trunk 2; unlock 7). Counts
  differ where KAT's bursts split differently (long press 1 vs 6, secondary mode 4 vs 6).
- rtl_433's own pulse packages through KAT's decoder: the expected `.json` files exactly (15 events, the duplicate
  and separator logic included); KAT's pulses through the C reference and through KAT: the same events.
- Fuzz against the C reference: 2 × 20 000 trains (repeated rows, separators, presses past 1200 pulses, messages
  past the bit buffer's 50 rows), identical output, 184 301 events; a third seed (89 700 events) also identical.
- No decode on any IMPORTS capture, the Flipper unit-test captures or another device's rtl_433_tests recording.
- Key files against FoxFW's C (host build with its encoder plugins, `subghz_oracle_foxfwenc`): on 160 RAW files of
  made-up frames (every mapped button code, secondary mode, other codes; presses, lone 35-bit rows, 36-bit rows with
  either last bit — 13 with a 1) FoxFW's serialized lines and KAT's key file are identical. FoxFW reads only the
  last row of a press (the next row's sync pulse makes it drop the one before), KAT the first: same frame. On KAT's
  pulses of the 5 recordings (as RAW files) FoxFW reads 4 distinct keys, in 3 of them; all 4 are among KAT's.
- Transmit against FoxFW's C: 120 made-up key files (110 valid, covering the button codes, 10 that KAT refuses);
  FoxFW's encoder refuses every one (`deserialize failed (status -12)`), KAT sends nothing for any. KAT loads the 110
  valid ones and refuses the 10 others.
