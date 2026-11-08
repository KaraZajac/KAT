---
layout: default
---

# HondaV2&V3 Protocol

**Rust module:** `src/protocols/honda_v2_v3.rs`
**Reference:** `REFERENCES/FoxFW/lib/subghz/protocols/honda_v2_v3.c` / `.h` (FoxFW only). **Decode only**: FoxFW's
encoder is an SD-card plugin that KAT does not port.

## Overview

One FoxFW protocol, two Honda/Acura variants told apart at run time (433.92 MHz, 2-FSK, flagged FM). Not related to
KAT's "Honda V2" (ProtoPirate's differential-Manchester protocol, see [honda_v2.md](honda_v2.md)).

| | V2 | V3 |
|---|---|---|
| te short / long / delta | 96 / 192 / 50 µs | 250 / 500 / 80 µs |
| Frame ends on a gap above | 500 µs | 1500 µs |
| Preamble (not checked) | ~116 x 1460 µs | ~230 x 500 µs |
| Frame | 148 bits, sync + 13 bytes | 120 bits, 15 bytes |
| Check | button pair only | CRC |

The first pulse in Reset that falls within either variant's short window (else long window) selects the variant (a tie
goes to V2). Pulses run through the Honda Manchester state machine (`{0x01, 0x91, 0x9B, 0xFB}`; high short 0, low short
2, high long 4, low long 6) into at most 200 bits. A pulse that fits neither window, a gap, or 200 bits ends the burst;
with at least 100 bits the frame is processed.

## V2 frame

Sync `0E 1E 1E 1E 0D 6C` at any byte offset (without it, the first offset whose bytes 3 and 4 are a button pair), then
`serial` (24 bits), `btn1`, `btn2`, 2 padding bytes, `counter` (8 bits), `rolling` (32 bits), `tail`. The pair
(btn1, btn2) must be (0x40, 0x20) lock, (0x41, 0x40) unlock, (0x41, 0x20) unlock retransmit or (0x40, 0x60) trunk. There
is no CRC: `crc_valid` false. `data` = `serial << 40 | counter << 32 | rolling`, `button` = `btn2`, `extra` = `tail |
btn1 << 8 | padding << 16 | has_sync << 32`.

## V3 frame

15 bytes from the first `0x6A` (trailing `0x55` idle bytes stripped down to 15): sync, counter byte (`counter << 2 |
tx_flag << 1 | bit0`), 12 payload bytes, CRC. The CRC is a CRC-16 (MSB first, poly 0x2627, init 0) over bytes 1..14
folded to 8 bits by the parity masks `0x8181, 0x4141, 0x2121, 0x80F2, 0xAF15, 0x0606, 0x0202, 0x0000` xor `0x1B`;
emission is gated on it. A fixed frame (`d[4] & 0xF0 == 0x80` and `d[5]` 0x82 or 0x83) has `serial = d[2..4]` and `button
= d[4]`; a hopping frame is encrypted throughout (serial and button 0). `data` = `serial << 48 | button << 40 | counter
byte << 32 | crc << 24`, `counter` = the counter byte >> 2, `crc_valid` true, `extra` = bytes 6..14.

## Not kept

The C keeps all 12 payload bytes of a V3 frame (bytes 2..14) and writes them to its key file; a KAT decode holds 8 of
them (`extra`), so key files are not supported for this protocol.

## Evidence

None: no capture of this protocol exists in the test corpus, so the port is validated on synthetic pulse sequences only:
the FoxFW oracle (with `--protocol HondaV2&V3`) and KAT give the same Key, variant fields and counter on a V2 unlock
frame and on a fixed and a hopping V3 frame; a corrupted V3 CRC, a wrong sync byte, a V3 frame cut at 10 bytes, an
unknown V2 button pair, a damaged V2 sync and a short V2 frame are rejected. In the full oracle registry other
protocols (Hitag2, Fiat V2) read these synthetic pulses first. In KAT's registry the V3 frames go to Ford V3 instead,
whose EU path (250/500 µs Manchester, no check) completes on the 104th bit, before the 120-bit V3 frame ends: of
29 copies of the fixed and hopping V3 frames that FoxFW's decoder reads alone, KAT lists 11 as HondaV2&V3: those
jittered by 10 %, stretched by 8-14 % or with one pulse split, which break Ford V3's frame first. The clean copies and
those jittered by 5 %, compressed, doubled or behind noise are Ford V3's. A genuine V3 capture would show whether
HondaV2&V3 needs priority over Ford V3. The IMPORTS sweep shows no
change.
