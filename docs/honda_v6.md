---
layout: default
---

# Honda V6 Protocol (ported, not registered)

**Rust module:** `src/protocols/honda_v6.rs`
**Reference:** `REFERENCES/FoxFW/lib/subghz/protocols/honda_v6.c` / `.h` (FoxFW only; itself a port of KeeLoq-RX
`arke_honda.py` `HondaV6`). **Decode only, as the reference** (`.encoder = NULL`). **No path has a check.**

## Status

The decoder is ported and its tests run, but it is **not in the registry**. The only captures FoxFW's decoder reads in
the corpus (16 to 17 frames in five files: three Toyota Estima captures, an Opel Vectra, an owner Subaru 2016 recording)
are noise through the 313 path (keys such as `40 00 00 14 00 00 00 10`); the Toyota and Opel files are decoded by
KAT's Toyota, Fiat V1 and Renault V1 decoders. With no checksum and a one-pulse-class preamble, registering the
decoder would relabel those decodes. Enabling it is one line in `src/protocols/mod.rs`.

## Paths

One decoder, three paths told apart by the pulses that open a burst:

| Variant | Radio | Format |
|---|---|---|
| 0, "stock45" | FM, 434 MHz | at least 20 short (100 +- 45 µs) pulses, then Manchester `{0x01, 0x91, 0x9B, 0xFB}`; 43..48 bits give a 45-bit word: counter = `norm[44:41]`, serial = `hop >> 9` (32 bits, not 0), button from hop bits 34 / 29 / 31 (0x02 / 0x01 / 0x10, else 0x03) |
| 1, "pandora" | FM | 118..140 bits, optional 1000..2000 µs mid gap after 45..65 bits, ends 78 bits after it; a frame under 135 bits is shifted right (at most 16) to 135 and must carry the skeleton `lo[63:40] == 0x1E1E1F`, `lo[7:0] == 0x51`, `hi[7:0] == 0xFE`; serial = `lo[63:32]`, counter = top `bits - 119` (at most 16) bits of `hi` |
| 2, "313" | 313 MHz | at least 15 shorts of 800 +- 250 µs, then trits of 800 / 1200 / 1600 µs packed 2 bits each into `d2` / `d1` (at most 60); a non-trit pulse ends the burst; at least 30 trits and `d2[63:32] != 0`; serial = `d2[63:32]`, counter = the trit count |

## Field mapping

`data` = the low data word (the C `generic.data`), `extra` = the high word (pandora `hi`, 313 `d1`, 0 for stock45),
`data_count_bit` 45 (stock45), 135..140 (pandora) or 2 x trits (313), `crc_valid` false. The variant is not stored; the
bit count tells it. Key files are not supported (the C's key file keeps only the low word).

## Evidence

The five corpus files: the FoxFW oracle and the KAT port give the same frames on all of them (17 frames, 0 differences,
all variant 2). Synthetic sequences for the three paths (made-up values; stock45 and pandora frames built to the C's
Manchester polarity) give the same fields in the oracle and in KAT, and short preambles, short frames, zero serials and
a damaged pandora skeleton are rejected.
