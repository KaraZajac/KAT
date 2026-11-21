---
layout: default
---

# Unread recordings

Every recording in the library (`IMPORTS/`) that KAT reads nothing from, what each one holds, and why. Most
were made with a Flipper preset that cannot hold the remote's signal: an FSK remote recorded with an AM preset,
or with an FM preset of the wrong deviation or speed, or a recording on the wrong frequency. For each car the page
gives what the remote sends according to its FCC filing, what the recording shows, what to record it with, and
which of KAT's decoders should read a good recording. [Recording a remote](recording.md) explains the presets.

The files stay in `IMPORTS/`, where the decode regression tests record what KAT reads from each (`(none)` for
these). If a decoder ever starts claiming one, rightly or as a false match, they flag it.

## At a glance

How the files were sorted: a file holds data when it has runs of pulses whose widths fall into a few classes, as
the bits of a frame do. In a sample of 20 recordings KAT reads, the longest such run was 109 to 1,864 pulses; in
the 74 files that hold no data it is 56 at most, the length noise reaches by chance. Where there is no data, the
bursts of a remote still show as stretches of carrier: their lengths and spacing were measured and compared with
the remote's filing. Of the 524 recordings in the library, KAT reads
437.

| What the files hold | Files | Section |
|---|---|---|
| The remote's signal, from which no frame has been read yet | 6 | [A signal, no frame yet](#a-signal-no-frame-yet) |
| No data: only the carrier, or noise | 74 | [Recordings that hold no data](#recordings-that-hold-no-data) |
| Nothing: an empty file, or a key file holding zeros | 7 | [Files with nothing in them](#files-with-nothing-in-them), [Key files](#key-files) |
| Read since this page was last written (four files) | – | [Read now](#read-now) |

## A signal, no frame yet

### Vauxhall Zafira 2008

`OPEL/2008_vauxhall_zafira_sri_lock.sub` and `OPEL/2008_vauxhall_zafira_sri_unlock.sub`, 433.92 MHz, recorded
with AM650; 15 presses
in each, sent in by a tester. KAT reads nothing from them, and neither does FoxFW's [Opel V0](opel_v0.md),
whose description of its signal fits these recordings (below). The same remote recorded with FM476
(`OPEL/2008_vauxhall_zafira_sri_lock_fm476.sub`, four presses, and `..._unlock_fm476.sub`, three) shows
nothing more (below).

- **What the recording holds.** Through a press the carrier stays on, and all there is in it are dips of
  about 95 µs (60 to 140). The time from one dip to the next is a whole number of units of 106.7 µs
  (1/9375 s). What the remote does between two dips, an AM recording does not show.
- **A press.** About 150 dips 853 µs apart (8 units; 0.13 s), then a frame, 1.7 ms without carrier, and the
  same frame again. Three short bursts of about 3.5 ms follow, 5.5 ms apart. Then a second frame, also
  twice. 0.2 s in all.
- **A frame.** Four intervals of 8 units, then intervals of 2 to about 10 units. The first eleven are the same in
  every frame of both files, `2 7 3 2 2 3 3 3 4 3 5` (37 units); about twenty more change with every press.
  The longest frames are 92 units behind the four (9.8 ms).
- **Lock and unlock.** Just behind those 37 units most lock frames have their next dips 3, 7 and 11 units
  on, most unlock frames 3, 7 and 12 units on (the first two frames of an unlock press often one more, 5
  units on).
- **Why no bits.** The dips are not all there, and not always on time. The two copies of a frame, 15 ms
  apart, often differ in which dips they show, most of all where two dips are 2 units apart. Of 86 frames,
  53 have the dips at 2, 9 and 37 units that every frame should have.

- **The FM recordings.** They were made to see what the remote does between two dips. Through a press the
  FM receiver's output stays on one side of the centre frequency (HIGH for 0.77 of the time; noise takes the
  rest) and drops where the AM recordings have their dips. The presses are too noisy to read a frame from,
  but the constant start shows in the mean: at 28 places where every other dip of the start lines up, the
  dips between them, which had no part in the lining up, are there too (HIGH for 0.49 of the time, 0.51
  with the two halves exchanged, against 0.77 and 0.79 beside the dips). Between the presses the same search
  finds nothing (0.42 against 0.43). The stretches between the dips all look alike: no second tone.

The remote does not switch between two tones on either side of 433.92 MHz: an FM receiver would show them
as HIGH and LOW. (A Flipper's FM output tells only on which side of the centre frequency the signal is; two
tones on one side would look alike.) Two readings are left, and neither is proved:

- The carrier is off for about a unit at each dip and nothing else happens: the code is in the distances
  between the gaps.
- The signal has two states that neither receiver tells apart, two strengths of carrier or two phases, and
  the receivers mark only one kind of change. In the five pairs of copies that agree in every dip, the 63
  distances behind the constant start are 2 to 9 units and 4.0 on average. A stream of one bit per unit
  that shows only its changes from 1 to 0 gives distances of 2 and more and 4 on average; five frames do
  not make that more than a guess. Such a frame would hold about 90 bits, 37 of them the constant start.

A delay code (Miller), the candidate named here before, is unlikely: its changes are 2, 3 or 4 half bits
apart, and the pairs that agree in every dip have distances of 5, 6, 7 and 9 units.

FoxFW's Opel V0 describes the same recording as on-off keying: "carrier frequency ~1200 Hz (period ~830us)",
"bursts of ~4 cycles (~3500us) separated by ~5400us gaps", "preamble: continuous carrier >50000us". Those
are the 853 µs intervals, the three short bursts between the two frames and the preamble. It takes each
burst for a bit and wants a frame of 62 to 66 of them; a press has three.

What would settle it: IQ samples from an SDR (a PortaPack's Capture app, a HackRF or an RTL-SDR; KAT opens
all three), 500 kHz wide or more, a few presses of each button near the antenna. They hold the carrier's
strength and phase between the dips, which no Flipper recording does: its AM and its FM receiver have both
been tried now.

### BYD Atto 3 2022

`BYD/BYD Atto 3 2022 unlock.sub` (0.92 s) and `boot.sub` (0.29 s), 433.92 MHz. The preset is "Custom", and the
lines that would give its register values (modulation, deviation, bandwidth) are not in the files. The sender
reports a Hitag2 fob, FSK, Manchester.

- **Long stretches at one level.** HIGH for 69.5 to 70.6 ms (twice in unlock, three times in boot; in boot the
  first is cut in two by a 66 µs dip) and LOW for 71.39 to 71.43 ms (five times, all in unlock), with one HIGH of
  28 ms in unlock. Where they follow each other, the next begins 82.0 and 81.3 ms after the last: the ~82 ms
  bursts of the BYD Sealion's recordings, where an AM receiver shows only the carrier. So each burst is about
  70 ms in which the receiver's output does not change, then about 10 ms of pulses, then noise.
- **The pulses behind a long stretch repeat.** In boot, the 50 pulses behind the first and the last long HIGH
  agree within 100 µs; in unlock, the first 26 behind the LOWs that end at 590 and 699 ms. Behind every long HIGH,
  in both files, comes a pulse of about 1550 µs, in five of the six then pulses of about 490 and 480 µs, and one
  burst of boot matches one of unlock in 27 pulses. But the pulses have no bit length: of the 485, at most 31 %
  lie within 12 % of a whole multiple of any unit from 40 to 400 µs, where the noise between the bursts gives
  20 %. They cannot be turned into bits, so whether unlock and boot send different frames cannot be told.
- **The "106-bit Manchester burst"** found earlier is 213 pulses of 65.5 µs, a square wave of 131 µs, for 13.9 ms
  (321.7 ms into unlock). As Manchester at 65.5 µs a half bit it is 106 bits all alike (all 0s or all 1s, by
  where one starts): no data. The same square wave runs for 19.2 ms in unlock (from 740 ms) and 9.4 ms in boot
  (from 189.6 ms), broken by single pulses of 5.5 and 7 half bits (360 and 460 µs; LOW in unlock, HIGH in boot).
  Manchester has pulses of one or two half bits, NRZ of whole ones, and neither has 5.5.
- **Hitag2.** A HITAG2 remote-keyless-entry frame ([Hitag2](hitag2_rke.md)) is 103 Manchester bits behind the
  marker `0x0001`: a UID of 32 bits, a button, a 10-bit counter and a 32-bit keystream. Nothing in either file
  slices into one: as Manchester with half bits of 65 to 250 µs, the only runs of more than 16 bits are the
  square wave's.

A receiver set for FSK holds one level while the signal stays on one side of the frequency it is tuned to. If
this preset was one (the sender reports an FSK fob), the fob's signal stayed above 433.92 MHz for 70 ms in the
bursts held HIGH and below it in those held LOW: it sends at more than one frequency. What it sends there, the
files do not show.

What would settle it: an IQ recording (a PortaPack's Capture app, a HackRF or an RTL-SDR; KAT opens all three),
2 MHz wide or more around 433.92 MHz, a few presses of each button close to the antenna. It shows at which
frequencies the fob sends and how it modulates them, which a receiver tuned to one frequency does not. The
preset's register values, if the original file still has its `Custom_preset_data` line, would show only what
this recording's receiver was set to.

## Read now

### Lexus UX 250 2021

> KAT reads these recordings with its own decoder, [Lexus UX](lexus_ux.md), which has the whole
> frame. The notes that stood here held, but for the slicer: it made one tone about 120 µs short and
> the other about 120 µs long. The frame has more to it: every byte from byte 1 on begins with a 1,
> the button is bits 49–52 (`0010` lock, `0100` unlock), the code that changes is 128 bits, and
> bytes 29–31 are `1010`, a CRC-16 (polynomial `0x1021`) and `11`.

`Toyota + Lexus/Lexus UX 250 2021 lock.sub` and `unlock.sub`, 433.66 MHz, recorded with 2FSK
Dev476: differential Manchester, 348.4 µs per half bit, four 256-bit frames back to back a press.
As sent (without the XOR), the ID bytes read `CA C4 E8 F2` or their complement `35 3B 17 0D`,
depending on the bits before them: what FoxFW's [Lexus V0](lexus_v0.md) calls its complement
signature.

### Volvo S40 1999

> KAT's own [Volvo S40](volvo_s40.md) decoder reads this recording, now that the rule of its frame is known
> (below). No reference reads it.

`VOLVO/Volvo S40 1999 unlock.sub`, 433.92 MHz, recorded with AM650: **two presses**, 2.26 s apart, of the button
the file name calls unlock. The air format is Chrysler V0's, as in `chrysler/Chrysler PT Cruiser 2006 *.sub`:

- 24 preamble pairs (HIGH 300 µs, LOW 3.7 ms), then a short HIGH and a 15.9 ms gap;
- PWM bits of 4 ms each: HIGH 300 µs = 0, HIGH 600 µs = 1;
- the frame, a short HIGH and a 15.9 ms gap, and the frame again, without a preamble.

The frames are 64 bits long instead of 80: `2D376746E42A3386` in the first press, `CD66CB532065E1EB` in the
second. Read by the references' Chrysler V0 model, their first bytes hold the seeds 52 and 51, its payloads A
and B, and the other bytes fail its checks. The two were once taken for the A/B pair of one press, which is what
the references' Chrysler V0 encoders send; but each has its own preamble and its own second copy, and the PT
Cruiser's fob too sends one payload per press.

They follow the rule of the PT Cruiser's frames ([Chrysler V0](chrysler_v0.md#frame-80-bits)),
with six fixed bytes instead of eight. With each byte read LSB first, the remote holds seven bytes: `n`, the low
byte of a counter (236 in the first press, 235 in the second), and `0A 8E CB B8 20 8D`. Byte 0 of the frame is
their sum (180, 179); then come `n` and the six bytes each XORed with `n`, the 56 bits backwards when the sum is
odd. Both frames follow the rule in all 64 bits. Which byte is the button, one button's recording cannot tell:
the [Volvo S40](volvo_s40.md) page has the frame, what is not known, and the recording that would settle it.

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

- `VAG/more/VAG_032.sub`, a VAG Type 4 key file saved with `Bit: 0`, is read as 80 bits since KAT checks that its
  frame decrypts to the file's own serial, button and counter ([VAG](vag.md)). Every reference's loader refuses it.

## Recordings that hold no data

These recordings hold carrier, receiver noise or nothing. Either the preset could not demodulate the key, or the
recording was made on the wrong frequency, so no decoder will read them. Each section gives the key, what the
recording shows, how to record the key again and which decoder should read a good recording.

What a key sends comes from its FCC filing where one is known ([FCC ID table](https://kat.karazajac.io/fcc.html)).
How sure the tie between car and FCC ID is: **catalogue** (a parts catalogue or seller lists the key for the car),
**filing** (the filing names the car), **timing** (the bursts in the recording match the filing) or **guess**. Burst
lengths and spacings were measured on the recordings. The presets are those of [Recording a remote](recording.md);
**untested** means no recording made that way exists yet.

### BYD Sealion, Seal 2025 and one unnamed BYD

`BYD/BYD Sealion lock x4.sub`, `BYD/BYD Sealion unlock x3.sub` (433.889 MHz, AM650); `BYD/BYD apertura.sub`
(433.92 MHz, AM650); `BYD/BYD Seal 2025 lock x5.sub`, `BYD/BYD Seal 2025 unlock x5.sub` (433.075 MHz, AM650). The
Atto 3's two recordings are under [BYD Atto 3 2022](#byd-atto-3-2022).

- **Key.** No FCC ID: BYD sells no cars in the USA. Whether these are one kind of key is not known.
- **Recording.**
  - Sealion and "apertura": every press is frames of unbroken carrier, 82.6 ms long and 83.5 ms apart, with the
    second slot empty. A press is five frames, about 0.5 s; "apertura" has up to ten, about 1.0 s. No bits reach an
    AM preset, so the key is most probably FSK (not documented).
  - Seal: the wrong frequency. 433.075 MHz is 845 kHz below 433.92 MHz. The file is receiver noise with one quiet hole
    of 0.47 to 0.54 s per press: the length of a BYD press, where the key's signal outside the passband turned the
    receiver's gain down.
- **Record with.** An SDR at 433.92 MHz (deviation and data rate not known). With a Flipper: `FM476`, then "FM12K";
  both untested.
- **Decoder.** None: no reference implements a BYD protocol.

### 2021 Ford Escape

`FORD/21_ford_escape.sub` (312.00 MHz; a custom 2-FSK preset with "Honda1"'s registers: ±15.87 kHz, 15.37 kBaud).

- **Key.** M3N-A2C931423 (Continental; catalogue). One channel, 314.95 MHz. A button press is six ASK frames of
  51.0 ms, 100 ms apart, Manchester with pulses of 0.2425 and 0.4775 ms (test report p.6, p.11). Its replies to the
  car's LF field are FSK.
- **Recording.** Receiver noise from start to end. 312.00 MHz is 2.95 MHz from the key, and an FSK preset does not
  demodulate on-off keying.
- **Record with.** `AM650` at 315.00 MHz. The 2022 Escape recording made that way
  (`FORD/22_ford_escape_u2_l2_t2_p2.sub`, 314.98 MHz) reads.
- **Decoder.** [Ford V3](ford_v3.md), US path. It reads the 2022 Escape, whose key is the same FCC ID in the
  catalogue, and its 240/480 µs timing is the filed one. Near certain.

### 2025 Ford Bronco

`FORD/25_ford_bronco_u2_l2_rs2_p2.sub` (433.42 MHz, AM650).

- **Key.** M3N-A3C108397 (Continental; frequency and timing). The filing calls it the "MY21 Ford PK CGEA 1.3 ...
  434MHz PASE fob" (operational description p.1). The catalogue lists only the 314.95 MHz M3N-A2C931423 for the
  Bronco, plus two 902–903 MHz keys for 2021–24; none of those can be heard at 433.42 MHz.
  - Two channels, 433.589 and 434.251 MHz, FSK, tones about 22 kHz apart.
  - A press is six frames of 50.9 ms, 100 ms apart, two at a time on each channel in turn (434.251, 433.589,
    434.251), 0.55 s (test report p.7, p.12–13, p.16).
- **Recording.** Two unbroken bursts of 50.4–51.4 ms per press, 100 ms apart: the two frames on 433.589 MHz, 169 kHz
  from the recording frequency. The frames on 434.251 MHz are outside the passband. An AM preset shows the FSK frames
  as carrier only.
- **Record with.** "F4" (±14.28 kHz, near the key's ±11 kHz), untested. Record at 434.25 MHz, which holds four of the
  six frames, and again at 433.59 MHz, which holds the other two. Or an SDR centred on 433.92 MHz, which holds both
  channels.
- **Decoder.** [Ford V3](ford_v3.md), probably. The frame is as long as the 314.95 MHz key's, which Ford V3 reads, and
  comes from the same "PK CGEA 1.3" family. Ford V3 has not been tried on an FSK recording of it.

### 2019 Genesis G90

`genesis/19_genesis_g90_l2_u2_t2_p2.sub` (433.92 MHz, AM270).

- **Key.** SY5HIFGE04 (Continental Korea; a seller listing, and timing). Other sellers list TQ8-FOB-4F16 for
  "G70 / G90", but its label reads "IK" (the G70) and its three 43.5 ms bursts do not fit.
  - 433.92 MHz FSK, tones 433.911 and 433.941 MHz: about ±15 kHz.
  - Four bursts per press, about 31–34 ms and 22–25 ms by turns, 99–100 ms apart, 321 ms (test report p.5, p.7,
    p.17–18, p.20).
- **Recording.** Bursts of 32.8–34.4 and 21.7–21.8 ms, 100–101 ms apart, each one unbroken carrier.
- **Record with.** "F4" or "Honda1" at 433.92 MHz, preferably on doctor-firmware; untested. This maker's Kia keys
  send 64 µs half bits (see the Kia section below). Or an SDR.
- **Decoder.** None yet.

### 2024 Genesis GV80

`genesis/24_genesis_gv80_hatch.sub`, `_lock.sub`, `_remote_start.sub`, `_unlock.sub` (433.92 MHz, AM650).

- **Key.** TQ8-FOB-4F35 (Hyundai Mobis; seller listings for the 2021–24 GV80, and timing). Sellers also list the
  facelift keys TQ8-FOB-4F75M44 and 4F75U44 for 2024–25; their bursts (23 ms, 140 ms apart) do not fit.
  - 433.92 MHz FSK, 20 dB bandwidth 94.79 kHz.
  - Three bursts of 44.93 ms per press, 168.84 ms apart, 0.37 s (test report p.3, p.12, p.15, p.19).
- **Recording.** Bursts of up to 45.0 ms, 165.5 ms apart. Lock and unlock give 2–3 per press, the held hatch and
  remote-start buttons up to 5. Unbroken carrier only.
- **Record with.** `FM476` at 433.92 MHz. It reads the older Mobis keys, whose tones are about 80 kHz apart (±40 kHz)
  and whose half bits are 194.8 µs. Untested on this key.
- **Decoder.** [Kia Mobis](kia_mobis.md), probably. It has the same maker and burst rhythm as the K7's
  TQ8-FOB-4F34, which Kia Mobis reads (41.0 ms bursts, 167.4 ms apart). The 4F35's burst is 3.9 ms longer, so it may
  carry more bits than Kia Mobis takes; not known.

### 2017 Honda Pilot, 2018 Honda Civic, 2022–24 Honda CR-V, 2024 Honda HR-V and 2022 Acura RDX

`honda/17_honda_pilot_lllluuuuaaattt.sub` (434.177 MHz, AM650); `honda/18_honda_civic_ll_uu_rs_t_rs_t_pp.sub`
(433.657 MHz, AM270); `honda/24_honda_crv.sub` (434.177 MHz, AM650); `honda/more/2022-2024_Honda_CR-V_433.66_FM476.sub`
(433.657 MHz, FM476); `honda/24_honda_hrv_l2_rs_t_p_ul2.sub` (434.177 MHz, AM650); `acura/22_acura_rdx_l_ul_t_p_l_ul.sub`
(433.657 MHz, AM650). The two empty CR-V files are under [Files with nothing in them](#files-with-nothing-in-them).

- **Key.** Continental two-channel keys.
  - Pilot and Civic: KR5V2X (catalogue, frequency and timing). The catalogue also lists KR5V1X for the Civic, but
    that key works at 313.55 and 314.15 MHz.
  - 2023–24 CR-V and HR-V: KR5TP-4. RDX: KR5TP-2. Both are seller listings, and the KR5TP-2 test report is not
    public (the mirror asks for a human check).
  - Signal: 433.66 and 434.18 MHz FSK, one burst of about 24.5 ms every 100 ms, two on one channel and then two on
    the other. KR5V2X: "24.493 ms" in each 100 ms, for as long as the button is held, about 1 s in the lab's test
    (test report p.6; periodic operation p.4). KR5TP-4: eight bursts, 0.7536 s (test report p.13, p.30–31).
- **Recording.**
  - AM files: pairs of 23.7–25.7 ms bursts 100 ms apart, the next pair 400 ms on; the bursts on the other channel are
    outside the passband. Unbroken carrier, no bits.
  - The Civic file also has 0.35 to 0.8 s of unbroken carrier before five presses. No filing shows that carrier.
  - FM476 file: each press is two stretches of output pinned LOW, 22–26 ms long. `FM476`'s 4.8 kBaud is too slow for
    these keys.
- **Record with.** "Honda1", which is proven on these keys. Record at 433.66 MHz, then again at 434.18 MHz: Read RAW
  does not hop, so each recording holds half the bursts. Or an SDR centred on 433.92 MHz, which holds both channels.
- **Decoder.** [Honda V5](honda_v5.md).
  - It reads two "Honda1" recordings of a Honda Continental key (`honda/Lock_honda.sub`, `honda/Unlock_honda.sub`)
    and a 2019 Civic EX's IQ recording (`honda/2019_honda_civic_ex_lock_x3_unlock_x2.sub`), whose key the catalogue
    also gives as KR5V2X. Near certain for the Pilot and Civic.
  - [Continental-KR5V2X](continental_kr5v2x.md) reads other recordings of KR5V2X keys.
  - The KR5TP keys (CR-V, HR-V, RDX) share the maker, the RF chip and the burst timing, so Honda V5 probably reads
    them too. Their frame is not known: the KR5TP-2 manual allows "HT3 or AES" encryption.

### 2018 Honda Accord

`honda/18_Honda_accord_l2_u2_rs2_t2_p2.sub` (433.92 MHz, AM650).

- **Key.** CWTWB1G0090 (Alps; seller listings, and the frequency and burst shape fit). The catalogue's KR5V1X works
  at 313.55 and 314.15 MHz.
  - 433.92 MHz FSK, 20 dB bandwidth 54.5 kHz, one hump; the deviation is not stated.
  - One block of 0.233 s per press (test report p.4, p.11, p.14). Bit rate not stated.
- **Recording.** One unbroken carrier per press, 156–175 ms. The rest of the filed 0.233 s is not seen; why is not
  known.
- **Record with.** An SDR at 433.92 MHz, since neither the deviation nor the rate is filed. With a Flipper: "Honda1"
  or "F4" (the narrow bandwidth suggests a small deviation), else `FM476`; all untested.
- **Decoder.** None yet. Which Honda decoder, if any, fits is not known. KAT's [Alps-FWB1U545](alps_fwb1u545.md)
  reads an older Alps Honda remote.

### 2018 Hyundai Santa Fe

`HYUNDAI/18_hyun_santafe_ll_uu_t_p.sub`, `HYUNDAI/18_hyunai_santa_fe_l2_u2_t2_p2.sub` (433.889 MHz, AM650).

- **Key.** SY5DMFNA433 (Continental Korea; catalogue and timing). The catalogue's other ID, SY5DMFNA04, is the
  315 MHz version of the same key. The manuals call it the "KH_DM_Fob" (DM is the Santa Fe's platform code).
  - 433.92 MHz FSK, "141k0F1D", tones about 75 kHz apart.
  - One unbroken block of 0.463 s per press (test report p.5, p.15–16).
  - NXP PCF7953 and PCF7900 (block diagram). Bit rate not stated.
- **Recording.** One unbroken carrier of 454–468 ms per press in the first file. The second file is carrier most of
  the time, with quiet holes. Carrier only.
- **Record with.** `FM476` at 433.92 MHz (±47.6 kHz is near the key's ±37.5 kHz; the rate is not filed), untested; or
  an SDR.
- **Decoder.** None known. KAT's [SantaFe 13-16](santa_fe.md) is for an on-off keyed TRW fob, not this key.

### 2017 Hyundai Elantra

`HYUNDAI/2017_Hyundai_Elantra_panic.sub` (433.92 MHz, AM650).

- **Key.** CQOFD00120 (Denso Korea; catalogue and timing). The catalogue also lists SY5MDFNA433, but it sends one
  0.460 s block, which does not fit.
  - 433.92 MHz FSK, tones about 60 kHz apart (about ±30 kHz).
  - One unbroken block of 0.39 s per press (test report p.3, p.14, p.16).
- **Recording.** 20 unbroken carriers of 384–407 ms (panic held, or pressed again and again).
- **Record with.** `FM476` at 433.92 MHz. It reads Denso Korea's ±30 kHz keys ([Kia Denso](kia_denso.md), 174.8 µs
  half bits). Untested on this key.
- **Decoder.** [Kia Denso](kia_denso.md), perhaps. It has the same maker and tone spacing, and 0.39 s is ten of Kia
  Denso's 39.2 ms frames, as on the 0.392 s Kia Forte keys (CQOFD00430, CQOTD00660). That is reasoning, not known.

### 2024 Kia Sorento, 2025 Kia Carnival and 2025 Hyundai Sonata

`KIA/24_kia_sorento_l2_u2_rs2_p2.sub` (433.889 MHz, AM270); `KIA/25_kia_carnval_l2_u2_t2_p2_rs2_lr2_rr2.sub`
(433.889 MHz, AM650); `HYUNDAI/25_hyundai_sonata_l2_u2_rs2_p2.sub` (433.92 MHz, AM270).

- **Key.** Hyundai Mobis keys of 2022–23: TQ8-FOB-4F81M44 for the Sorento and Carnival, TQ8-FOB-4F89U44 or 4F61M43
  for the Sonata (seller listings, and timing). The catalogue's tie of the 2024 Sorento to SY5MQ4FGE05 is wrong.
  - 433.92 MHz FSK, tones 80 kHz apart (about ±40 kHz).
  - Three bursts of 22.5–23.2 ms, 139.7–149.5 ms apart, 0.30–0.32 s (4F81M44 test report p.3, p.13, p.17, p.21).
  - The manuals of 4F75M44, 4F75U44, 4F89U44 and 4F81U44 (p.2) give "Baud Rate : 7.8 kbps", "Coding Method :
    Manchester" and "178bit/frame": half bits of 64 µs.
  - The "U44" keys also carry UWB at 6.5 GHz.
- **Recording.**
  - Sorento and Sonata: 22.9 ms bursts, 141.7 and 139.1 ms apart.
  - Carnival: bursts of 40–41 ms. Each holds a 17.5 ms part in front of the filed 23 ms burst, which no filing shows.
  - Most presses begin with 0.1 to 0.8 s of unbroken carrier, also in no filing.
  - Carrier only.
- **Record with.** An SDR at 433.92 MHz: these keys have 64 µs half bits at ±40 kHz, and an SDR would also show what
  the long carrier is. With a Flipper: "FM476F" (±47.61 kHz at 38.4 kBaud), untested. `FM476` (4.8 kBaud) is too slow.
- **Decoder.** None yet. [Kia Mobis](kia_mobis.md) reads the older Mobis keys (194.8 µs half bits, 40.5 ms bursts),
  not these.

### 2019–2023 Kia Sorento, Sportage, Soul and Carnival

`KIA/23_kia_sorento_l2_ul2_t2_p2.sub` (433.92 MHz, AM650); `KIA/23_kia_sportage_l2_u2_t2_p2.sub` (433.889 MHz,
AM270); `KIA/more/2019-2022_KIA_Soul_433.92_FM.sub`, `KIA/more/2020-2022_Kia_Sorento_433.92_FM.sub`,
`KIA/more/2023__Kia_Soul_433.92_FM.sub`, `KIA/more/Kia_Carnival_433.92_FM.sub` (433.92 MHz, FM476).

- **Key.** Continental Korea smart keys and flip keys, FCC IDs beginning SY5: SY5MQ4AFGE04, SY5MQ4FGE05,
  SY5MQ4FGE07, SY5SKFGE04, SY5SKRGE03, SY5SKRGE04, SY5KA4FGE06 (catalogue and timing). Which one each car has is
  not known.
  - 433.92 MHz FSK, "Deviation NA : 30kHz", "Baud rate of RF : 7.8kbaud" (SY5SKFGE04 user manual p.7), Manchester
    (SY5SKRGE03 and SY5SKRGE04 manuals).
  - Four bursts per press: 32.8, about 22, 32.8 and about 22 ms, 100 ms apart, 0.32 s (e.g. SY5MQ4FGE05 test report
    p.9, p.15).
  - SY5MQ4AMFGE04, which the catalogue gives for the 2023–24 Soul, sends the four twice (0.821 s, test report p.13).
    The 2023 Soul file shows four per press, which fits SY5MQ4AFGE04, the sellers' ID for that car.
- **Recording.**
  - AM files: the four bursts as unbroken carrier (32.8 and 21.8 ms, 99.6–100.9 ms apart). Several presses begin with
    0.12 to 0.64 s of carrier that is in no filing.
  - FM476 files: the output is pinned LOW through each burst (33 and 22 ms). `FM476`'s 4.8 kBaud and ±47.6 kHz do not
    fit a key that sends 15.6 kBaud on the air at about ±15 kHz.
- **Record with.** "F4" or "Honda1" at 433.92 MHz, preferably on doctor-firmware, which keeps pulses of 21–50 µs;
  untested on these keys. Or an SDR.
- **Decoder.** None yet.

### Lamborghini (model and year not known)

`Lamborghini/Lambo_Lock.sub`, `Lamborghini/Lambo_Trunk.sub`, `Lamborghini/Lambo_Unlock.sub` (315.00 MHz, AM650).

- **Key.** Not known. Sellers give one FCC ID for a Lamborghini, MYT8Z0837231, listed for the Gallardo; the filing
  itself names Audi. It is an ASK key: 1.8 kBaud Manchester, one 250.76 ms word per press (test report p.4–5).
  Through AM650 it would show its pulses, so these recordings are of another key.
- **Recording.** Each press is one unbroken carrier of 180.4 ms, then 19.2 ms of carrier every 39.0 ms, 2 to 15
  times. The receiver is silent between presses. Carrier only, so the key is most probably FSK (not documented).
- **Record with.** An SDR at 315.00 MHz. With a Flipper: `FM476`, then "FM12K"; both untested.
- **Decoder.** None.

### 2025 Mazda CX-50

`Mazda/25mazda_cx50_ll_uu_tt_pp_x2.sub` (314.98 MHz, AM270).

- **Key.** WAZSKE11D01 (Mitsubishi Electric; catalogue, which lists the CX-30, CX-50, CX-90 and MX-5 of 2019–26).
  - 314.93 MHz, a wide FSK: 20 dB bandwidth 192 kHz, tones roughly 115–145 kHz apart (read from the plot).
  - One block of 0.3375 s per press. Symbols of about 0.13 ms: the duty test shows pulses of 0.139, 0.264 and
    0.389 ms (test report p.4–5, p.11, p.14–15).
- **Recording.** Unbroken carriers of 156–818 ms; whole presses are 284–333 ms. Carrier only.
- **Record with.** An SDR at 315.00 MHz, since no preset comes near the key's deviation of about ±60 kHz. With a
  Flipper: "FM476F" (untested), else `FM476` (whose 4.8 kBaud is below the key's rate).
- **Decoder.** None yet. KAT's Mazda decoders read keys with half bits of 250 to 505 µs.

### 2007 Mazda RX-8 (433 MHz key)

`Mazda/Rx8_2007_10x_alarm.sub`, `_lock.sub`, `_trunk.sub`, `_unlock.sub` (433.889 MHz, AM650).

- **Key.** No FCC ID: a 433.9 MHz key, from a European or Australian car. The US RX-8's key is another radio:
  KPU41788 (Visteon), 313.85 MHz, 314 ms a press, modulation not stated (report p.9).
- **Recording.** One unbroken carrier of about 344 ms per press (329–395 ms). Carrier only, so the key is most probably
  FSK (not documented).
- **Record with.** `FM476` at 433.92 MHz, which reads a Visteon Mazda fob (`Mazda/more/Mazda_Visteon_TS.sub`).
  Untested on this key.
- **Decoder.** [Mazda V0](mazda_v0.md) or [Mazda Siemens](mazda_siemens.md), perhaps: they read that Visteon fob.
  Not known.

### 2024 Mercedes-Benz GLE 350

`mercedes/24_merc_gle350_luluttlulu.sub` (314.35 MHz, AM650); `mercedes/24_Merc_gle350_lulu.sub` (434.075 MHz, AM650).

- **Key.** NBGDM3 (Hella) or IYZ-MS2 (Marquardt); seller listings, low certainty.
  - Both: three channels at 314.00, 314.45 and 314.90 MHz, FSK.
  - NBGDM3: three parts of about 70 ms in 210.5 ms, with a period of 334.05 ms (test report p.7, p.47).
  - IYZ-MS2: two bursts of 208 ms, 268 ms apart (test report p.5, p.16).
- **Recording.**
  - The 314.35 MHz file: three unbroken bursts of 67.3–67.6 ms per press, 128 and 138 ms apart. Most presses begin
    with 0.15 to 0.8 s of carrier. Each burst is as long as a [Mercedes V1](mercedes_v1.md) burst (683 bits of 100 µs:
    68.3 ms).
  - The 434.075 MHz file is 0.1 s long.
- **Record with.** An SDR centred on 314.45 MHz, at least 1 MHz wide: there are three channels, and Mercedes V1's
  chips are 50 µs. With a Flipper: doctor-firmware's "FM20K" Mercedes preset at 314.45 MHz, untested.
- **Decoder.** [Mercedes V1](mercedes_v1.md), perhaps: the burst length fits, but V1 is known only from 433 MHz keys.
  Not known.

### 1999 Mercedes-Benz S 430 (W220)

`mercedes/Mercedes W220 S430 1999 315.sub` (315.00 MHz, AM650).

- **Key.** IYZ3312 (Marquardt). A guess from seller listings for Mercedes keys of 1997–2014; the filing itself names
  the W 203.
  - 315.00 MHz. The modulation is not stated: the block diagram reads "FM oder AM Transmitter". No timing is filed
    (test report p.3–4).
  - Another Marquardt ID, IYZ3302 (1996), is possible; its exhibits are not public.
- **Recording.** 54 unbroken bursts of 151.2–151.3 ms, one at a time or, with a button held, in a steady rhythm. The
  carrier is never switched off inside a burst.
- **Record with.** An SDR at 315.00 MHz. With a Flipper: `FM476`, then "FM12K"; both untested.
- **Decoder.** None known. KAT's [Mercedes V0](mercedes_v0.md) and [Mercedes V1](mercedes_v1.md) read later 433 MHz
  keys.

### 2018 Nissan Rogue and 2021 Nissan Versa

`nissan/18_nissan_rogue_rs2_l2_u2_t2_p2.sub`, `nissan/18_nisssan_rogue_rs3_l3_u3_t3_p3.sub` (433.92 MHz, AM650);
`nissan/21_nissan_versa_l2_u2_t2_p2.sub` (433.657 MHz, AM650).

- **Key.** KR5S180144106 for the Rogue and KR5TXN1 for the Versa (Continental; catalogue and timing, within 1 ms).
  - 433.92 MHz. A 40.1 ms burst, then 113 ms after it a 43.5 ms burst in three parts, then 8.4–9 ms bursts every
    84.5 ms while the button is held (KR5S180144106 periodic operation p.6; KR5TXN1 test report p.12, p.16).
  - Modulation not stated. KR5TXN1's spectrum has its two strongest lines about 77 kHz apart (p.14).
- **Recording.** That pattern exactly, as unbroken carrier.
- **Record with.** `FM476` at 433.92 MHz, untested: the tones are about ±38 kHz and the rate is not filed. Or an SDR.
- **Decoder.** None yet.

### 2021–2022 Nissan Frontier

`nissan/22_nissan_frontier_lock.sub`, `nissan/22_nissan_frotier_unlock.sub` (433.92 MHz, AM650);
`nissan/Nissan_frontier_lock_2time.sub`, `nissan/Nissan_frotier_unlock_4.sub` (434.177 MHz, AM650; no year).

- **Key.** KR5TXN7 (Continental; catalogue and timing).
  - 433.92 MHz. Four bursts of 32.9 ms and one of 18.4 ms, 103.2 ms apart, 431.2 ms in all. "Additional telegrams"
    follow while the button is held (test report p.22, p.27; manual p.4).
  - Modulation not stated; two humps about 70 kHz apart (p.24–25).
- **Recording.** That pattern exactly, as unbroken carrier. The two files without a year also hold the held-button
  train: 20 ms bursts every 100 ms.
- **Record with.** `FM476` at 433.92 MHz, untested; or an SDR.
- **Decoder.** None yet.

### Nissan Skyline (model and year not known)

`Skyline/Skyline_close_US.sub`, `Skyline/Skyline_open_US.sub` (314.98 MHz, AM650); the same files are in `nissan/`.

- **Key.** Not known (guess). KR55WK48903 (Siemens VDO) covers the "Nissan D-Plateform i-key": a US version at
  315 MHz and a Japanese one at 314.85 MHz. Its manual gives FSK, ±25 kHz, 2 kBd, Manchester (p.4); no burst timing.
- **Recording.** Unbroken carriers of 404 ms (twice) to 1216 ms. Carrier only.
- **Record with.** An SDR centred on 314.9 MHz, which covers both versions. With a Flipper: `FM476` at 315.00 or
  314.85 MHz, untested.
- **Decoder.** None yet.

### 2005 Renault Megane Scenic

`RENAULT/Renault Megane Scenic 2005 lock.sub`, `... trunk.sub`, `... unlock.sub` (433.075 MHz, AM650).

- **Key.** No FCC ID: Renault sells no cars in the USA. No document for this key's frequency or modulation; European
  keys of the period use 433.92 MHz.
- **Recording.** The wrong frequency. The files are receiver noise with quiet holes of 0.1 to 0.32 s, one group per
  press, where the key's signal outside the passband turned the gain down. The key itself is not in the files.
- **Record with.** An SDR at 433.92 MHz. With a Flipper: both `AM650` and `FM476` at 433.92 MHz, since the
  modulation is not known.
- **Decoder.** Not known. KAT's Hitag2-based [Renault V0](renault_v0.md) and [Renault V1](renault_v1.md) (on-off
  keyed) read other Renault keys. FoxFW's decoder for "Renault Megane fobs", [Renault V3](renault_v3.md) (2-FSK,
  52 µs), is ported but not registered, and no recording of it exists.

### 2020 Subaru Outback

`SUBARU/20_subaru_outback_ll_tt_ulul_pp.sub` (433.657 MHz, AM650).

- **Key.** HYQ14AHK (Denso smart key; catalogue and timing). The catalogue's other ID, the CWTB1G077 remote-head key,
  sends 0.195 s, which does not fit.
  - 433.92 MHz FSK, tones about 50 kHz apart (read from the plot).
  - One unbroken block of 0.7333 s per press (test report p.4, p.11, p.14).
- **Recording.** Stretches of carrier 0.71–0.86 s long; carrier only. The key at 433.92 MHz is 263 kHz from the
  recording frequency, inside AM650's passband.
- **Record with.** "FM29F" at 433.92 MHz (±28.56 kHz, the nearest to the key's ±25 kHz), or "F4"; both untested. Or
  an SDR.
- **Decoder.** None yet.

### 2016–2023 Toyota and Lexus

`Toyota + Lexus/18_toyota_tacoma_l2_ul2_p2.sub`, `19_lexus_is_sk_l_u_t_p.sub`, `19_toyota_camry_hv_ul2_l2_t2_p2.sub`
(312.10 MHz); `2017 Lexus NX200t_312.sub`, `23_toyota_tacoma_l2_u2_p2.sub` (312.00 MHz); `2017 Lexus NX200t_314.sub`,
`21_lexus_is300_lluulluut.sub`, `23_toyota_rav4_l2_ul2_t2_p2.sub` (314.35 MHz); `21_toyota_tacoma_l_u.sub`
(315.00 MHz); all AM650; `19_toyota_camry_l2_u2_t2_p2.sub` (312.00 MHz, AM270). `Toyota_Lexus NX200t_2017_312.sub`
and `_314.sub` repeat the two NX200t recordings.
`more/Tundra19_lock.sub`, `Tundra19_unlock2.sub` (312.10 MHz) and `Tundra19_lock2.sub`, `Tundra19_unlock.sub`
(314.35 MHz) were made with FM238.

- **Key.** Catalogue only: which key each car has is not known.
  - Denso smart keys HYQ14FBA, FBB, FBC, FLA, FLB, FBX; Denso remote-head keys HYQ12BDM, BFB, BFW, BGF, BGG; TRW
    GQ4-52T and GQ4-73T.
  - All FSK on 312.10 and 314.35 MHz, one channel per press ("not emitted simultaneously"; the TRW keys change
    channel with every press). One unbroken frame of 0.70–0.77 s per press.
  - Tones about 30 kHz apart on the smart keys and 50–60 kHz on the remote-head keys; ±30 kHz on the TRW keys. Symbol
    0.35 ms (HYQ14FBA and HYQ12BDM duty tests).
  - The exception is HYQ12BDP, a remote-head key the catalogue gives for the Tacoma, Camry and RAV4: ASK on 314.35 MHz
    only, 0.535 s, pulses of 0.350 and 0.694 ms (test report p.3, p.10, p.13).
- **Recording.**
  - AM files: one unbroken carrier of 0.70–0.77 s per press, longer where it runs into noise. The 2018 Tacoma's
    frames are on 312.10 MHz and unbroken, so its key is not HYQ12BDP.
  - 2021 Tacoma: recorded at 315.00 MHz, outside the passband for both channels, so only pieces show.
  - Camry `l2_u2_t2_p2`: each press is the key's 0.65 to 0.93 s of carrier, then about 0.6 s of a steady
    375/128 µs square wave that is the same in every press and carries nothing.
  - Tundra: each press is one 707–721 ms frame with the output pinned LOW. FM238's ±2.38 kHz is far from the keys'
    deviation.
- **Record with.**
  - An SDR at 4 MS/s or more centred on 313.2 MHz, which holds both channels. With a Flipper, one recording per
    channel; each holds every other press.
  - Presets: "F4" or "Honda1" for the smart keys, "FM29F" for the remote-head keys; all untested.
  - For the 2021 Tacoma, also `AM650` at 314.35 MHz, in case its key is HYQ12BDP.
- **Decoder.** Perhaps [Lexus UX](lexus_ux.md), KAT's own decoder of the European Lexus UX 250 key: 433.66 MHz,
  FSK, a 256-bit frame four times a press (0.71 s, about the length of these keys' frames), differential Manchester
  at 348 µs (the 0.35 ms symbol of the duty tests above). Whether the US keys send that frame on 312.10 and 314.35
  MHz is not known; an FSK recording of one would show it. FoxFW's [Lexus V0](lexus_v0.md) is the same frame read
  without its differential step. [Toyota](toyota.md) (on-off keyed, 793 µs) does not fit these keys.

### 2024 Volkswagen Atlas

`VAG/24_vw_atlas_l2_t2_ul2.sub` (433.42 MHz, AM650).

- **Key.** NBGFS1901S (Hella; seller listings for the 2022–26 Atlas, and frequency). The catalogue's KR5FS14 is a
  434.42 MHz ASK key (test report p.6). The 2020 Atlas that KAT reads was recorded at 314.98 MHz, so it has yet
  another key.
  - 433.46, 433.92 and 434.36 MHz FSK, on "two of the available RF channels" per press.
  - "+/- 7.8 KHz" at "7.8 Kbit / s" for a button press (user manual p.5, p.10).
  - The burst timing is not public.
- **Recording.** Two unbroken 45.5 ms bursts per press, 200 ms apart: the ones on 433.46 MHz, 40 kHz from the
  recording frequency. Carrier only.
- **Record with.** An SDR centred on 433.92 MHz, at least 1 MHz wide: the key uses two channels, and its deviation is
  narrower than any preset's. With a Flipper: "F4" at 433.46 MHz and again at 433.92 MHz, untested (twice the key's
  deviation).
- **Decoder.** None yet.

### Files with nothing in them

- `FIAT/Fiat type 0 N2.sub`: 176 bytes, all zero. The recording was lost (a failed copy or card write).
- `KIA/Kia0_orig.sub`: three header lines and no samples; no frequency or preset.
- `honda/24_hoda_crv_l2_u2_rs_t_p.sub` and `honda/24_honda_crv_l2_u2_rs_t_p.sub` (the same file twice): a header
  (433.22 MHz, AM650) and no samples. 433.22 MHz is also 440 kHz below the key's nearer channel. Record as in the
  Honda section.

Neither the Fiat file nor the Kia file says which car it is from.

### Summary of the recordings that hold no data

| Car | Files | Why no data | Record with | Decoder that should read it |
|---|---|---|---|---|
| BYD Sealion, unnamed BYD | 3 | FSK (by inference) through AM650: carrier only | SDR at 433.92; `FM476`, then "FM12K" (untested) | none |
| BYD Seal 2025 | 2 | wrong frequency (433.075 MHz) | as above | none |
| Ford Escape 2021 | 1 | wrong frequency (312.00 MHz) and an FSK preset for an ASK key | `AM650` at 315.00 | Ford V3 (near certain) |
| Ford Bronco 2025 | 1 | FSK through AM650; one of two channels in range | "F4" at 434.25 and 433.59 (untested); SDR | Ford V3 (probably) |
| Genesis G90 2019 | 1 | FSK through AM270 | "F4" or "Honda1" (untested); SDR | none yet |
| Genesis GV80 2024 | 4 | FSK through AM650 | `FM476` | Kia Mobis (probably) |
| Honda Pilot, Civic, CR-V, HR-V; Acura RDX | 6 | FSK through AM; `FM476` too slow | "Honda1", one recording per channel (433.66, 434.18); SDR | Honda V5 (near certain for KR5V2X; probably for KR5TP) |
| Honda Accord 2018 | 1 | FSK through AM650 | SDR; "Honda1" or "F4" (untested) | none yet |
| Hyundai Santa Fe 2018 | 2 | FSK through AM650 | `FM476` (untested); SDR | none known |
| Hyundai Elantra 2017 | 1 | FSK through AM650 | `FM476` (untested) | Kia Denso (perhaps) |
| Kia Sorento 2024, Carnival 2025, Hyundai Sonata 2025 | 3 | FSK through AM | SDR; "FM476F" (untested) | none yet |
| Kia Sorento, Sportage, Soul, Carnival 2019–23 | 6 | FSK through AM; `FM476` pinned | "F4" or "Honda1" on doctor-firmware (untested); SDR | none yet |
| Lamborghini | 3 | FSK (by inference) through AM650 | SDR at 315.00; `FM476`, "FM12K" (untested) | none |
| Mazda CX-50 2025 | 1 | wide FSK through AM270 | SDR; "FM476F" (untested) | none yet |
| Mazda RX-8 2007 | 4 | FSK (by inference) through AM650 | `FM476` at 433.92 (untested) | Mazda V0 or Mazda Siemens (perhaps) |
| Mercedes GLE 350 2024 | 2 | FSK through AM650; second file 0.1 s | SDR centred on 314.45; "FM20K" on doctor-firmware (untested) | Mercedes V1 (perhaps) |
| Mercedes S 430 1999 | 1 | not on-off keyed: carrier only | SDR; `FM476`, "FM12K" (untested) | none known |
| Nissan Rogue 2018, Versa 2021 | 3 | carrier only through AM650 | `FM476` (untested); SDR | none yet |
| Nissan Frontier | 4 | carrier only through AM650 | `FM476` (untested); SDR | none yet |
| Nissan Skyline | 4 | carrier only through AM650 | SDR at 314.9; `FM476` (untested) | none yet |
| Renault Megane Scenic 2005 | 3 | wrong frequency (433.075 MHz) | SDR; `AM650` and `FM476` at 433.92 | not known |
| Subaru Outback 2020 | 1 | FSK through AM650 | "FM29F" or "F4" (untested); SDR | none yet |
| Toyota and Lexus, nine cars | 16 | FSK through AM; Tacoma 2021 off frequency; Tundra `FM238` pinned | SDR for both channels; "F4", "Honda1" or "FM29F" per channel (untested) | perhaps Lexus UX (not known) |
| Volkswagen Atlas 2024 | 1 | FSK through AM650; one channel in range | SDR; "F4" per channel (untested) | none yet |
| Fiat "type 0 N2", Kia0_orig, two CR-V 2024 files | 4 | no samples | (Fiat and Kia: car not known) | – |

## Key files

- `VAG/more/32 Audi V1.sub` (88 bits) and `MISC/more/36 Audiovox PRO.sub` (25 bits): their `Key` (and `Hop`) are all
  zeros, the empty form a firmware writes when a key is made by hand. There is nothing to read.
- `VAG/more/audi_v1.sub` (64 bits): the key `12 34 56 78 9A BC DE F0`, serial 74565 (0x12345), button 1, counter 7:
  a test pattern, not a recording. FoxFW's Audi V1 frames are Fiat V1's ([Audi V1](audi_v1.md)).

## Not added

`23_bmw_x5_lock.sub` and `23_bmw_x5_unlock.sub`, shared with these captures, are not recordings.
Their 394 and 544 values use 7 distinct durations: exactly 250, 500 and 1500 µs plus fixed fillers,
with none of the jitter of a real receiver. Their lock and unlock frames also have different lengths
(64 and 89 bits).
