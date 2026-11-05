---
layout: default
---

# rtl_433 Ford-CarRemote → KAT Ford V0

rtl_433's `fordremote.c` ("Ford Car Key", model `Ford-CarRemote`, decoder 93, disabled by default: "does not attempt
to decrypt the rolling code, id/code semantics are unconfirmed") reads the 250/500 µs Manchester Ford fobs that KAT
decodes as **Ford V0** (ProtoPirate `ford_v0.c`, a Flipper reference with priority), so KAT has no port of it.

rtl_433 slices the frame as differential Manchester (`OOK_PULSE_DMC`) into rows `{1} {9} {1} {78}`. The 78-bit row is
exactly the XNOR of consecutive Manchester wire bits of Ford V0's frame (`~key1` bits 62..0, then `~key2`), on all
six `rtl_433_tests/tests/fordremote` recordings: rtl_433's `id` (row bits 0..23) and `code` (row byte 7) are
transitions of the scrambled key, so they change with every press, while Ford V0 descrambles the constant serial,
the button and the counter (and checks the CRC).

| recording | rtl_433 id / code | KAT Ford V0 |
|-----------|-------------------|-------------|
| ford-unlock001 | 261663 / 181 | sn 481849C8 btn 1 cnt F695 |
| ford-unlock002 | 178520 / 11 | sn 481849C8 btn 1 cnt F696 |
| ford-lock003 | 178520 / 11 | sn 481849C8 btn 4 cnt F697 |
| ford-lock004 | 253953 / 155 | sn 481849C8 btn 4 cnt F698 |
| ford-boot005 | 165220 / 55 | sn 481849C8 btn 2 cnt F699 |
| ford-boot006 | 253953 / 151 | sn 481849C8 btn 2 cnt F69A |

The counter runs F695..F69A in recording order. Both read each press 4 times. On IMPORTS, rtl_433's Ford-CarRemote
fires on five FORD captures, which Ford V0 decodes too.

**Button names.** KAT shows its generic codes (1 Lock, 2 Unlock, 4 Trunk), mapped from Ford V0's on-air field as
ProtoPirate names it (`ford_v0.c`: 0x02 Lock, 0x04 Unlock, 0x08 Boot, 0x01 Panic). These recordings' file names
disagree with that naming (the "unlock" files carry ProtoPirate's Lock field, "lock" its Boot field, "boot" its
Unlock field), but the owner's own captures agree with it: IMPORTS `FORD/2008_Focus_Lock`, `_Unlock` and `_Trunk`
and `3_unlock_ford` decode as Lock, Unlock and Trunk. KAT keeps the reference's names; the rtl_433_tests remote may
assign its buttons differently, or its files may be misnamed (rtl_433 decodes no button for it).
