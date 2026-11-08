# Unidentified captures

What is known about the captures from the owner's key-captures collection in `IMPORTS/` that
nothing decodes: not KAT, and not ProtoPirate, Flipper-ARF or doctor-firmware (each checked with the
reference C). KAT decodes what the references decode, so none of these formats is implemented. Each
needs at least its cipher before a decoder could report more than raw bits. The files stay in
`IMPORTS/`, where the decode regression test (`imports_decode_golden`) records them as "(none)".
If a decoder ever starts claiming one, rightly or as a false match, that test flags it.

## No usable signal in the file

| File(s) | Recorded | Contents |
|---|---|---|
| `RENAULT/Renault Megane Scenic 2005 *.sub` | 433.075 MHz, AM650 | an FSK fob recorded with an AM preset |
| `BYD/BYD Sealion lock x4.sub`, `unlock x3.sub` | 433.889 MHz, AM650 | an FSK fob (per the sender) recorded with an AM preset. Every press is the same five carrier bursts of ~82 ms (gaps 85, 0.85, 0.87 and 4.95 ms), on lock and unlock alike: the data is in the FSK modulation |
| `BYD/BYD apertura.sub` | 433.92 MHz, AM650 | the same 82.6 ms bursts, plus more of them 34.45 ms apart |
| `BYD/BYD Seal 2025 lock x5.sub`, `unlock x5.sub` | 433.075 MHz, AM650 | a few 5–6 ms carrier fragments at the end of each file; the fob probably transmits outside the recording's bandwidth (the other BYD fobs were recorded at 433.92 and 433.889 MHz) |
| `Mazda/Rx8_2007_10x_*.sub` | 433.889 MHz, AM650 | the same: carrier blocks up to 344 ms, no pulse train longer than 11 pulses |
| `BYD/BYD Atto 3 2022 *.sub` | 433.92 MHz, "Custom" preset without its register lines | noise, and ~70 ms bursts with a pulse pattern between them that repeats from burst to burst. Without the preset's register values those pulses can't be turned into bits (the sender reports a Hitag2 fob, FSK, Manchester) |
| `mercedes/Mercedes W220 S430 1999 315.sub` | 315 MHz, AM650 | noise only: no pulse train longer than 34 pulses |

An FSK fob has to be recorded with an FM preset (FM238 or FM476). With an AM preset the receiver
only sees the carrier switching on and off. A `.sub` file's preset also decides which decoders KAT
runs on it (AM or FM), as on the Flipper. No reference implements a BYD protocol, so even a good FM
recording of these fobs would be material for research, like the formats below, rather than a decode.

## Formats no reference implements

### VW Polo 5 2017

> FoxFW's **VAG Polo** decoder reads these captures (see [vag_polo.md](vag_polo.md)); KAT ports it. The
> analysis below is the owner's own, from before that port, and stays for the wire-level detail.

`VAG/VW Polo 5 2017 *.sub` (lock 1–3, unlock 1–3, five presses of each), 434.42 MHz, recorded with
AM270. Before VAG Polo, PSA read one junk frame in `unlock2.sub` (`737AD61FFFFFFFFF…`, no valid fields), as it
does in ProtoPirate and Flipper-ARF; the Polo frames now come first.

- **Modulation.** OOK Manchester, TE ≈ 290 µs (bit 1 = HIGH then LOW). The capture lengthens every
  HIGH by ~70 µs, so it reads 363/218 µs.
- **Per press.** Three frames about 11 ms apart, all different.
- **Preamble.** About 214 HIGH/LOW pairs of 290 µs. The last HIGH runs straight into the first data
  bit; there is no sync gap.
- **96 data bits** (one more bit can follow at the end):
  - a 16-bit constant, `0x9FC8`;
  - 7 button bits: `1101111` on lock, `1011111` on unlock (one bit cleared per button);
  - 73 bits that differ in every frame.
- **Examples.** Lock `9FC8DEA444EBC56007596118`, unlock `9FC8BFA4804190A78BDDFA16`.

The preamble is the length VAG Type 1/2 uses (220 pairs at 300 µs), but none of the VAG framing is
there:

- no 600 µs LOW after the preamble;
- no `0xAF3F` / `0xAF1C` prefix;
- the button comes first, where VAG ends with a dispatch byte.

Apart from `0x9FC8`, nothing stays the same between presses, so any serial is inside the encrypted
part.

### Lexus UX 250 2021

`Toyota + Lexus/Lexus UX 250 2021 lock.sub` and `unlock.sub`, 433.66 MHz, recorded with 2FSK
Dev476.

- **Modulation.** Manchester, TE ≈ 350 µs. The capture's FSK slicer makes one tone ~115 µs shorter
  than the other: HIGH 235/575 µs, LOW 470/815 µs.
- **Per press.** One burst of four identical 256-bit frames back to back (~0.7 s).
- **Encoding.** The bits are differentially encoded. XOR each bit with the one before it and every
  frame lines up:

| Bytes | Content |
|---|---|
| 0–5 | header, sent as `00 87 00 FF 0F 00` |
| 6 | button in the high nibble: `9` = lock, `A` = unlock |
| 7–24 | 18 bytes that change with every press |
| 25–28 | `AF A6 9C 8B` in every frame (the fob's ID?) |
| 29–31 | change with every press (check bytes?) |

As sent (without the XOR), the ID bytes read `CA C4 E8 F2` or their complement `35 3B 17 0D`,
depending on the bits before them.

### Volvo S40 1999

`VOLVO/Volvo S40 1999 unlock.sub`, 433.92 MHz, recorded with AM650. The air format has the Chrysler
V0 timing of `chrysler/Chrysler PT Cruiser 2006 *.sub`:

- 24 preamble pulses (HIGH 300 µs, LOW 3.7 ms), then a 15.9 ms gap;
- PWM bits of 4 ms each: HIGH 300 µs = 0, HIGH 600 µs = 1;
- each frame sent twice.

The frames are 64 bits long instead of 80. The file holds two of them, `2D376746E42A3386` and
`CD66CB532065E1EB`. Read as Chrysler V0, their first bytes hold the reversed 6-bit counters 52 and
51. That is the even/odd pair one Chrysler V0 press sends, but the other bytes fail Chrysler V0's
checks. Several presses of each button would show whether the rest follows a related scheme.

## Not added

`23_bmw_x5_lock.sub` and `23_bmw_x5_unlock.sub`, shared with these captures, are not recordings.
Their 394 and 544 values use 7 distinct durations: exactly 250, 500 and 1500 µs plus fixed fillers,
with none of the jitter of a real receiver. Their lock and unlock frames also have different lengths
(64 and 89 bits).
