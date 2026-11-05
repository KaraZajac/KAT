---
layout: default
---

# Akhan 100F14 remotes (rtl_433 `Akhan-100F14`) = Princeton

rtl_433's `src/devices/akhan_100F14.c` ("Akhan 100F14 remote keyless entry", model `Akhan-100F14`) reads an
HS1527 (EV1527-type) OTP encoder: OOK PWM, short 316 µs / long 1020 µs (± 100), 25 bits = a 20-bit ID, a 4-bit
command and the stop pulse, bits inverted (long pulse = 1). It accepts commands 0x1 Lock, 0x2 Unlock, 0x4 Mute and
0x8 Alarm. rtl_433 ships it **disabled** (`.disabled = 1`, "false positives with generic EV1527 devices"); its
recordings decode only with `-R 48`.

KAT has no separate decoder for it: the existing **Princeton** decoder (PT2262/EV1527, Flipper firmwares) reads
these remotes with the same fields, so a duplicate was not added (rtl_433 ports only add what the Flipper
references' decoders cannot read).

| rtl_433 `Akhan-100F14` | KAT `Princeton` |
|------------------------|-----------------|
| `id` (20 bits, decimal) | `serial` (hex): 704303 = 0xABF2F |
| `data` "0x2 (Unlock)" … | `button`: 2, 1, 4, 8 |
| (25th bit = stop pulse) | `data` = the 24 code bits (0xABF2F2 …) |

rtl_433_tests `akhan_remote/01` (gfile001–004, 250 kS/s): rtl_433 `-R 48` gives ID 704303 with Unlock, Lock, Mute,
Alarm; `kat --decode` gives Princeton sn=ABF2F btn=2 / 1 / 4 / 8 (once, once, once, twice: Princeton reports a frame
when its repeat confirms it, rtl_433 every frame). The Flipper references do the same on KAT's pulses of these
files: doctor-firmware and the Flipper-ARF garage app decode them as Princeton, Serial 704303, Btn 2 / 1 / 4 / 8
(ProtoPirate and the Flipper-ARF car registry have no Princeton). Princeton accepts any 4-bit command; Akhan's
names (Lock, Unlock, Mute, Alarm) are the only thing it would add.
