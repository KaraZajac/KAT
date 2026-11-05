---
layout: default
---

# rtl_433 Nidec-OUCG8D and Honda-CarRemote → KAT Honda V0

rtl_433's `nidec_car_remote.c` ("Nidec Car Remote", model `Nidec-OUCG8D`, decoder 304) and `hondaremote.c`
("Honda Car Key", model `Honda-CarRemote`, decoder 64) read the same Honda remotes (Nidec OUCG8D-344H-A, a 2004
Honda Accord fob), FSK 250/500 µs PWM. Both are disabled by default in rtl_433 (Nidec: the trailing security field
is mostly truncated and unverifiable; Honda: no MIC, weak sanity checks). KAT already reads every one of their
frames as **Honda V0** (Kia V0's Honda type, ProtoPirate `kia_v0.c`), which has priority as a Flipper reference,
so KAT has no port of them.

## Field mapping (all 9 frames of `rtl_433_tests/tests/nidec` and `honda_remote`)

On the wire: 28 short + 4 long pairs, then 64 bits (Kia polarity: short pair 0, long pair 1; rtl_433's PWM slicer
reads short = 1 and Nidec inverts the row). KAT's Honda V0 key = byte 0 `F0`, then every wire byte bit-reversed.

| rtl_433 Nidec | wire bits after the sync | KAT Honda V0 |
|---------------|--------------------------|--------------|
| `sequence` | 0..15 | `counter` (equal) |
| `id` (24 bits) | 16..39 | `serial` = `rev8` of each `id` byte (Nidec `CCCE80` = KAT `337301`) |
| constant `5` + `button_code` | 40..47 | header byte = `rev8(0x50 \| button)`; KAT `button` = header >> 5 (Nidec 3 Lock → 6, 4 Unlock → 1, 5 → 5, 6 → 3, 0xF → 7) |
| `security` bits 15..8 | 48..55 | the key's last byte (`rev8`), Honda V0's CRC byte (`crc_valid` on 8 of the 9) |
| `security` bits 7..0 | 56..63 | not read by Honda V0 (rtl_433: `security_bits` 9..16, i.e. 1..8 of these are on air) |

rtl_433 Honda-CarRemote takes fixed positions of a 385..394-bit PWM row: its `id` = row bytes 44..45 = the inverted
low 16 bits of the Nidec `id` (56154 = `~0x24A5` for ID `9824A5`), its `code` = `~(0x50 | button) − 0xAA`
(boot / unlock / lock for buttons 5 / 4 / 3, "unknown" otherwise).

| recording | rtl_433 Nidec | rtl_433 Honda-CarRemote | KAT Honda V0 |
|-----------|---------------|-------------------------|--------------|
| honda_remote/01/hondacar-bootunlock | id 9824A5 seq 22799 btn 5 | id 56154 boot | sn 1924A5 cnt 590F btn 5 |
| honda_remote/01/hondacar-lock2 | id 9824A5 seq 22792 btn 3 | id 56154 lock | sn 1924A5 cnt 5908 btn 6 |
| honda_remote/01/hondacar-unlock | id 9824A5 seq 22798 btn 4 | id 56154 unlock | sn 1924A5 cnt 590E btn 1 |
| nidec/…/01/lock | id CCCE80 seq 34375 btn 3 | id 12671 lock | sn 337301 cnt 8647 btn 6 |
| nidec/…/01/trunk_long_press | id CCCE80 seq 34378 btn 15 | id 12671 unknown | sn 337301 cnt 864A btn 7 |
| nidec/…/01/trunk_panic_short_press | id CCCE80 seq 34377 btn 5 | id 12671 boot | sn 337301 cnt 8649 btn 5 |
| nidec/…/01/unlock | id CCCE80 seq 34376 btn 4 | id 12671 unlock | sn 337301 cnt 8648 btn 1 |
| nidec/…/02/lock | id DD4151 seq 5445 btn 3 | id 48814 lock | sn BB828A cnt 1545 btn 6 |
| nidec/…/02/panic_long_press | id DD4151 seq 5450 btn 6 | id 48814 unknown | sn BB828A cnt 154A btn 3 (crc 0) |

`honda_remote/01/hondacar-lock.cu8` holds only a preamble (235 equal 248 µs pulses): nothing decodes it.

The nidec recordings are 1024 kS/s: decode them with the rate from the file name (`kat --decode <file>`);
`--rate 250k` stretches them 4× (KAT 1.7 then reported 4 of them as unrelated "Honda V1" frames).

Neither decoder adds frames: Honda V0's gate checks the same structure as Nidec's (the long-pair sync, header low
nibble `A` = the Nidec `5`) and decodes every frame rtl_433's Nidec decodes in REFERENCES; on IMPORTS and the
Flipper unit-test captures (converted to rtl_433 pulse files) Nidec decodes nothing, and Honda-CarRemote only fires
on three captures of other makes (Kia V5, Suzuki V0, Lexus UX 250) with code "unknown".
