# Changelog

All notable changes to KAT are documented here.

## [1.11.0] - 2026-11-10

KAT uses every radio it finds, each on its own frequency. **Analyze** says what a signal is, also one no decoder knows. And a pass over the interface: the settings of each kind of radio, commands, typing keys, key hints, messages and small terminals.

### Added

- **Several radios at once** ([radio.md](docs/radio.md#several-radios)): KAT uses every HackRF, RTL-SDR and Flipper Zero it finds, each tuned on its own (frequency, gains, hop list, recording, receiving on or off), so it can listen on 315 and 433.92 MHz at the same time, or transmit with one radio while another listens. Each radio is found by what tells it from another of its kind (a HackRF's serial number, an RTL-SDR's EEPROM serial or USB order, a Flipper's port) and opened by it for every receive run and transmission. The header has a line per radio (▸ the selected one, `TX` the transmitter), the RX meter a column per radio, the captures list a Radio column and the detail panel the radio that heard the capture. `r`, `:freq`, `:lna`, `:vga`, `:amp`, `:hop`, `:record` and the `Tab` settings act on the selected radio, chosen with `1`–`9`, `:radio <n|name>` or the settings' new Radio tab; `R` starts or stops every radio. One radio transmits (the first HackRF, else a Flipper; `:txradio`, config `tx_radio`), and only it pauses receiving to do so. A capture another radio hears while it transmits nearby is marked **Own TX** (no key search runs on it) and the status line says which radio heard the transmission. `:radios` lists the radios (state, serial or port, settings, frequency coverage, why a kind is not in use) and adds radios plugged in since KAT started. Config: `device` takes a list of kinds, and `[radio.<kind>]` / `[radio.<kind>.<serial>]` sections set how single radios or kinds start (`frequency`, `lna_gain`, `vga_gain`, `amp`). With one radio KAT looks and works as before. Not yet tried on hardware, with one radio or several.

- **Analyze** ([analyze.md](docs/analyze.md)): `a` on a capture, **Analyze** in its menu or `:analyze [ID]` says what the signal is, for a decoded capture and for one no decoder knows. The report gives the keying (on a HackRF or RTL-SDR it is measured from the burst: 2-FSK with its two tones and deviation, or OOK, with the carrier's offset from the tuned frequency and the level over the noise; for a `.sub` recording it is the preset's, for a Flipper the receive path's), the pulse widths of HIGH and LOW with their counts and ranges, the base TE and how far the receiver skews the two levels, the symbol rate, the line coding (PWM, PWM with equal-length pairs or with a fixed gap, PPM, Manchester, NRZ), the rows between pauses and how they repeat ("4 frames of 29 bits, all identical, every 71.1 ms"), a trial decode of the bits in hex and binary, a drawn waveform and the raw durations. A decoded capture is compared with its protocol's timing; an unknown one gets the protocols KAT knows with similar widths, as a place to start. `←` / `→` in the report show the captures next to this one; the list selects the capture shown, so closing the report leaves it there. An unknown capture's detail panel says how to get there ("Not decoded. Press a to analyze it.").

- **Command history**: `↑` / `↓` on the command line bring back the commands run before (the last 50 of this run of KAT), and back to what was being typed. `Backspace` on the empty line closes it, as in vi.

- **Line editing keys**: `Ctrl-U` clears what is typed and `Ctrl-W` (or `Alt-Backspace`) deletes its last word, in the command line, the export and vehicle forms, the filter and fields of `:new` and the code of **Send button…**. They had `Backspace` only.

### Changed

- **`device = auto` (the default) uses every radio found**, not only the first of HackRF, RTL-SDR, Flipper; `device = hackrf, rtlsdr` limits it to those kinds.
- **Starting KAT opens no radio.** A Flipper's console is opened the first time it receives or transmits in KAT (then kept until KAT quits); KAT 1.10 connected at startup when the Flipper was its only radio, which would end what another program (qFlipper, a terminal) was running on it. A Flipper that cannot be used says why when it is used and in `:radios`, and is tried again on the next use.
- **"Timing analysis" is part of Analyze.** The Signal menu's entry is **Analyze**, and `:timing [ID]` still works as another name of `:analyze`. The comparison with the protocol's definition (ProtoPirate's Timing Tuner) is the report's **Protocol timing** section ([timing_analysis.md](docs/timing_analysis.md)).
- **A protocol sent with two timings is compared with the one the capture fits**: VAG's Type 1/2 (300/600 µs) beside its Type 3/4 (500/1000 µs), and Fiat V1's variant B (100/200 µs) beside variant A (250/500 µs). Against the other one a capture has no "long" durations at all, so half of the VAG decodes in IMPORTS and most of the Fiat V1 ones read INCOMPLETE. The report says when there was a choice.
- **The TE and line-coding estimate**, checked on the 1170 decodes of IMPORTS (it agrees with KAT's own encoding label for 1162 of them). NRZ is recognised from the durations themselves, where clusters could not tell (runs of four and five TEs share one, and its mean is no multiple of the TE). A stretch with data in it is preferred over one that is all preamble, which is as regular as pulses get and says nothing of the coding. A constant skew between HIGH and LOW (an FSK slicer off the centre of the two tones) is fitted together with the TE, instead of a TE per level that fits neither. A width the recorder's timer steps split in two (100 and 133 µs for a TE of 104 µs) counts once. Manchester is told from other codings of the same widths by its 1 TE pulses coming in twos.
- **Times are on this computer's clock**: the list's Time column, the detail panel and the Analyze report ("2026-11-04 10:31:07 UTC+01:00", with the clock's offset from UTC) and the restore prompt. They were UTC. `.fob` files and the saved session keep UTC.
- **Small terminals.** The overlays (help, License, Credits, radios, Analyze) take all of an 80×24 terminal but its edge (78×22 where it was 64×20), so the help and the reports no longer wrap there; the help fits the smallest terminal KAT runs in (60×16) without a wrapped line, and an overlay's title is cut with "…" where it is longer than the border. The help bar shows the fullest text that fits in every mode, as the list's did: the export form and the new-remote forms cut their last keys off in a narrow window. The new-remote picker has shorter lines for a narrow window, and its "more" hint counts the remote types out of view, not the headings between them.
- **The detail panel is as tall as what it shows**, up to the height it had, and leaves the other rows to the list. The captures of one remote take the same height, so the list does not move while the selection goes through them. A recording shows the preset it was made with, and an unknown capture says how to analyze it.
- **An unknown capture's menu and forms** name it by what it has (`#07 Unknown | 433.92 MHz`, where it read `#07 Unknown | 433.92 MHz | Unknown | 0x-`), and its export form says "none decoded (the recorded pulses are saved)" where a decoded capture has its key.
- **The suggested `.fob` file name follows the export form**: `Year_Make_Model_Region_Command_8HEX` is built again from the vehicle details as they are typed (it kept the "Unknown" parts it opened with), until the name itself is edited.
- **Without a radio**, the startup warning says that `:radios` finds one connected later, with no restart, and what KAT does until then (load, decode, analyze, export). It names the kinds of radio KAT looks for: with `device = hackrf, rtlsdr` it no longer reports a missing Flipper Zero or suggests connecting one. Receiving, transmitting, `:hop` and `:record` point to `:radios` as well ("No radio connected (:radios looks for one)").
- **Messages count in plain words**: "Restored 1 capture from the last session", where it read "1 capture(s)"; the same for the messages of `:clear`, an IQ file, the key files and the key searches, and for the startup import prompt. The Status and Command boxes have their titles spaced like every other box.
- **A radio has the settings of its kind.** The `Tab` settings show LNA, VGA and AMP for a HackRF, one **Gain** for an RTL-SDR and the frequency alone for a Flipper Zero, with TX power when the radio that transmits is a HackRF. They offered a HackRF's settings for every radio, and setting one the radio does not have said it was set. `:lna` and `:amp` on an RTL-SDR, and `:lna`, `:vga` and `:amp` on a Flipper, say why they do not apply ("RTL-SDR has one gain: :vga <0-62> sets it"). `Shift-Tab` goes back through the settings.
- **An RTL-SDR's gain is the VGA value**: `:vga`, the Gain setting, `default_vga_gain`, and `vga_gain` in a `[radio.rtlsdr]` section; the header shows it as "Gain:20". The LNA and the VGA value both set its one tuner gain before, so the one set last won (at startup the VGA default: an RTL-SDR still starts at 20 dB). `:lna` and `default_lna_gain` no longer reach an RTL-SDR.
- **`:lna` and `:vga` say what the radio takes**: `:lna 20` answers "LNA gain set to 16 dB (8 dB steps)". A HackRF's LNA goes in 8 dB steps and its VGA in 2 dB steps, and the answer gave the fitted value without saying why. `:lna 16dB` works, and a value that is no number gives the usage with the range and the steps, where it read "Invalid LNA gain value".
- **Commands**: the command word works in any case (`:FREQ 315` was an unknown command) and with a second `:` typed after the prompt's. `:x`, `:exit` and `:wq` quit as `:q` does, `:q!` quits without asking, `:?` opens the help and `:del` is `:delete`.
- **Messages name a capture as the list does**, "#07": "Capture #99 not found", "Replayed #03 (…)", "Recovering PSA key for #12…", where they read "Capture 99" and "capture 3". `:klbf #3` takes the number with its `#`, like the other commands, and the error of a transmission the protocol cannot encode names the protocol in every case (one read "Protocol does not support encoding").
- **A long message wraps in the status bar**, which grows to three lines for it: an error with a file's path and the reason after it was cut in the middle. In a terminal too short to spare the rows the bar keeps its one line, as before.
- **Key hints.** The list's help bar names `Tab: Settings` at 80 columns (it read "Tab: Radio" there), the command line's names `Ctrl-U`, and the prompts' say that `Esc` answers no ("y: Quit | n/Esc: Back"). Hints are written one way: "Key: Action" in the help bar, lower case inside a box.
- **An empty list says what to do while a radio receives**: "No captures yet — receiving: press a button on the remote". It went on with "press r to receive", and `r` stops the radio.
- **The Signal menu says why it has no transmit actions** when no radio transmits ("TX: needs a HackRF or a Flipper" on its border), and a title wider than the terminal ends with "…".
- **`:new` shows the whole reason a type cannot be created**, on the two lines under the list: one line cut "no usable Erreka key in the keystore (add one to ~/.config/KAT/keeloq_mfcodes_user)" off at the file's name.
- **A gate remote's button** reads "Btn: 0x09" in the detail panel, where it read "Btn: Btn 0x9".

### Fixed

- **With `ignore_duplicates = false`, two receive sessions running at once reset each other's 500 ms repeat check** (an IQ file decoded while receiving live), so one press could be listed twice: each session now keeps its own.
- **`Enter`, `i` and `d` with no capture selected** (an empty list) say "No capture selected"; they did nothing. `:delete all` with one capture says "Deleted the only capture", where it read "Deleted all 1 captures".

## [1.10.0] - 2026-11-08

A Flipper Zero plugged in over USB can be KAT's radio: it receives and transmits through its console, with no app to install.

### Added

- **A Flipper Zero as the radio** ([flipper_radio.md](docs/flipper_radio.md)): plugged in over USB, it receives and transmits for KAT through its console, with no app to install (config `device = flipper`, or found when no HackRF or RTL-SDR is; `flipper_port` when its name is unusual). Receive: `subghz rx_raw` streams the CC1101's pulses, noise included between transmissions; KAT's decoders read the stream as it comes (`StreamLocator`: each pulse once, as a burst's), cut every frame they read out of the noise (the shortest stretch its decoder still reads, plus the rest of a regular preamble) and hand it to the app as a burst, so it is decoded, listed and de-duplicated like an SDR's. Streamed this way, the 127 AM Flipper recordings in IMPORTS give exactly the 329 frames KAT decodes from the files. FM reception needs doctor-firmware, whose `rx_raw` takes a preset (`flipper_preset = FM476`; KAT checks the firmware's usage text first). Transmit: KAT's own transmission (encoders, copies, rolling-code advance) is written as a RAW `.sub` to the SD card and played with `subghz tx_from_file`, with AM650 for OOK and, for 2-FSK, the preset of its deviation (FM238 / FM476 by name, others as a register image); receiving pauses meanwhile, and a refusal (region lock, frequency) shows the Flipper's words. There is no signal meter, `:record` or hopping with a Flipper. Tested on a simulated Flipper console (pseudo-terminal) and the IMPORTS recordings.

### Fixed

- **Changing the frequency while receiving, to one the radio refuses, left the header on RX** with nothing received (a Flipper's CC1101 has gaps between 300 and 928 MHz): the receiver goes back to idle and the status line gives the radio's reason.

## [1.9.0] - 2026-11-08

42 more remotes: 27 car remotes, most of them ported from FoxFW, and 15 gate, home, sensor and key-finder remotes from the Flipper firmwares and rtl_433. Car remotes now send as many copies per press as their reference firmware, and a key file's `Repeat` line sets the count.

### Added

- **Toyota (FoxFW): the two Prius 2006 captures decode** (decode only). FoxFW's "Toyota" is not KAT's Toyota (Flipper-ARF's and doctor-firmware's 96-bit Estima decoder): it is Flipper-ARF's retired KeeLoq-layout Toyota decoder (`toyota-old.c.bak`) plus a variant C, registered under the same name, which is how the FoxFW review missed it. Variant C (Prius / Corolla Verso: 400/800 µs PWM on the HIGH behind ≥ 6 preamble pairs and a 1050-1500 µs sync, 66..68 bits) and variant B (Tundra: NRZ behind a 1.7-2.2 ms sync, 67 bits) are ported, gated on the C's plausibility check (hop and serial neither all zeros nor all ones, a per-variant button list); its variant A cannot be reached from the C's reset state and is not. `IMPORTS/Toyota + Lexus/Toyota_Prius2006_lock.sub` and `_unlock.sub`, which no KAT decoder read, now give FoxFW's 8 frames each (serial 0x7231804, buttons 0xB Lock / 0xE Unlock, 68 bits); nothing else in the corpus changes. FoxFW's `Toyota` key files (Bit 66..68) load through KAT's Toyota, which hands them over. On KAT's own AM receive path an OOK Kia V3 fob (same air format, buttons on Toyota's list) still decodes as Kia V3: a burst only Toyota (FoxFW) reads in the modulation-gated pass also gets the ungated one, where Kia V3/V4 comes first. Checked against FoxFW's C: identical on 28 made-up frames (variants, buttons, lengths, 8 the C refuses) and on 416 distorted versions of them. Page: [toyota_foxfw.md](docs/toyota_foxfw.md).
- **KeeLoq keystore learning types 16-19** from doctor-firmware: JCM Gen2 (16, the learning byte searched on decode and kept as the seed), Stagnoli (17) and the table-driven Telcoma pair (18 high half, 19 low half), for decoding and transmitting keystore entries of those types. Type 20 is unchanged (Star Line). See [docs/keeloq.md](docs/keeloq.md).
- **VAG Polo and Chevrolet V0, two decode-only car remotes ported from FoxFW.** Both read frames KAT could not: the eight `VW Polo 5 2017` lock and unlock captures (48 frames, serial and Lock / Unlock; OOK Manchester at 300/590 µs, 32+ preamble ones, sync `0x3F91`, 80 bits, no checksum) and the Chevrolet Trail Blazer capture (21 frames, serial `0xB809C5`, Open / Close; OOK Manchester at 120/240 µs behind the exact preamble `0xAAAAAAAB`, 96 bits, encrypted hop). Every frame matches a host build of FoxFW's C. The hop and the rolling part are encrypted, so there is no transmit (as in FoxFW; Replay still sends the recorded pulses). Neither decoder has a checksum, so both are registered after every decoder that has one, before the rtl_433 ports; no other capture in `IMPORTS/` or Flipper's unit tests changes except that `VW Polo 5 2017 unlock2.sub` no longer shows a junk all-ones PSA frame next to its Polo frames. Pages: [vag_polo.md](docs/vag_polo.md), [chevrolet_v0.md](docs/chevrolet_v0.md).
- **Honda V5** (FoxFW `honda_v5.c`, decode only): 135-bit Manchester frames (63/126 µs) with a CRC-8, reporting serial, button, a 16-bit counter and the rolling word; the 168-bit alternate frame is read too. `IMPORTS/honda/Lock_honda.sub` and `Unlock_honda.sub` decode as Honda V5 now: CRC valid on all 40 frames, one serial (0xB53EFF72), button 0x21 (Lock) / 0x22 (Unlock), and a counter that rises with every press. The registry tries it before Honda Static.
- **Faac TM and Freezer, two gate remotes from RogueMaster** (decode and transmit). Faac TM: the 12 DIP-switch code of FAAC TM remotes, AM at 433.92 / 868 MHz, CAME's short/long grammar at a Te measured per frame (nominally 400 µs) behind a 35-Te header; key files carry the Te (`TE`), and "Add Manually" gains RogueMaster's FAAC TM 433MHz / 868MHz types. Freezer: a 24-bit code (20-bit serial, 4-bit button), AM at 433.92 MHz, reported only when a frame repeats. Both match a host build of RogueMaster's C: identical uploads for 120 key files, identical decodes of 300 perturbed recordings, and RogueMaster's `freezer_raw.sub` gives the same five frames. Pages: [faac_tm.md](docs/faac_tm.md), [freezer.md](docs/freezer.md).
- **Twelve home, sensor and key-finder remotes the Flipper firmwares register** (decode, transmit, key files): Honeywell Sec (security sensors, CRC-16), Magellan (Paradox security sensors, CRC-8), X10, Honeywell (doorbells / PIR), Legrand, Doorbell32, Intertechno V3, KeyFinder (with Flipper-ARF's four colour buttons), Treadmill37, Nord ICE, Feron and KeyFinder2. Flipper-ARF's garage app registers X10, Doorbell32, KeyFinder, KeyFinder2 and Nord ICE; doctor-firmware, Momentum, RogueMaster and FoxFW the stock ones too. Each matches the reference C: every Flipper unit-test capture (Honeywell, Magellan, Legrand ×2, Intertechno V3, Feron) and key file decodes and transmits as the C does, 60–96 random key files per protocol give the same upload, and 200–1200 perturbed recordings per protocol give the same decodes; nothing else in the corpus changes. Where the copies differ, KAT reads what either reads: Honeywell Sec runs doctor-firmware's and FoxFW's Manchester state side by side (FoxFW re-arms it on every reset and reads every repeat), and Nord ICE takes the union of the garage app's and doctor-firmware's timing windows and sends doctor-firmware's two-frame upload with any button. RogueMaster's "Nord ICE 433MHz" Add Manually type now works (86 types available). Pages: [honeywell.md](docs/honeywell.md), [magellan.md](docs/magellan.md), [x10.md](docs/x10.md), [honeywell_wdb.md](docs/honeywell_wdb.md), [legrand.md](docs/legrand.md), [doorbell32.md](docs/doorbell32.md), [intertechno_v3.md](docs/intertechno_v3.md), [keyfinder.md](docs/keyfinder.md), [treadmill37.md](docs/treadmill37.md), [nord_ice.md](docs/nord_ice.md), [feron.md](docs/feron.md), [keyfinder2.md](docs/keyfinder2.md).
- **Urmet transmits and saves key files**, as doctor-firmware and FoxFW do. Its frame (up to 40 bytes and 192 gaps) does not fit a decode, so KAT keeps it in the capture's recording: a loaded Urmet key file becomes a capture whose Replay sends exactly the reference encoder's transmission (gaps rebuilt from `Raw` when the file has none), and a received frame saves with `Raw` and `Gaps`. On 120 recordings and 339 key files KAT and doctor-firmware's C agree line for line and pulse for pulse.
- **Cardin S466-TX2** (rtl_433 `cardin.c`, receive only): the 27.195 MHz fixed-code garage remote with nine tri-state DIP switches; on 400 synthetic messages KAT and rtl_433 (`-s 1M`) give the same 172 decodes. Page: [cardin_s466.md](docs/cardin_s466.md).
- **KeeLoq at half rate** (from RogueMaster): HCS200/HCS300/HCS301 remotes set to Te 200 µs decode (the sync gap locks the rate at the measured Te), are sent back at that rate, and their key files carry RogueMaster's `TE: 200`. On 160 synthetic recordings at Te 180–420 µs KAT and RogueMaster's C read the same 305 frames.
- **KeeLoq manufacturer rules from doctor-firmware and RogueMaster**: KEY (discriminator 0x2FF), Monarch's two discriminators (0x100 and Dorcas' 0x280, kept as the seed) and RogueMaster's new Phantom/Hiland (HCS300: the serial's low 10 bits, the top 2 kept as the seed; the key goes in `keeloq_mfcodes_user`). They are identified, re-sent with their discriminator and written with a `Seed` line, as both firmwares do; given KAT's keystore order, RogueMaster's C names the same manufacturer and counter on 72 test files. "Add Manually" can now create doctor's KEY and Dorcas remotes, and lists RogueMaster's KL: Phantom/Hiland and FoxFW's plain KeeLoq "KL: Erreka 433/868MHz" (112 types; 86 available without user files, with Nord ICE below).
- **KeeLoq Erreka programming mode**, as FoxFW has it: button 0xF on an Erreka remote sends its seed as the hop (like BFT's), for captured remotes and new ones.
- **23 more decode-only car remotes ported from FoxFW:** Tesla Charge, Hyundai/Kia RIO, SantaFe 13-16, Hitag2 (PCF7946/7947 RKE frame), Nidec RKE, Renault/xHorse V4, Fiat Ducato V0, Renault V2, Opel V0, Honda V4, V7, V9, V2&V3 and V10, Mitsubishi V1, LDV V0, Lexus V0, Mazda V1, Nissan V0, BMW V2, BMW/Mini V3, Audi V3 and BMW CAS3. No capture in the test corpus contains these remotes, so each decoder was checked on made-up frames decoded by both FoxFW's C and KAT (identical fields; corrupted frames rejected by both), and none changes any decode of the 241 corpus captures. Six further FoxFW decoders are ported but not registered because they claimed other protocols' frames: Honda V6, Honda V8, Renault V3, Fiat Multi, Audi V1 and Audi V2 (each module's docs give the evidence). FoxFW's Mercedes fob is disabled in FoxFW itself and was not ported; GM ABO1502T, Porsche Touareg, Mitsubishi V0-a, Toyota V0, Peugeot V0 and Jeep V0 duplicate decoders KAT already has.

- **MARELLI key files from Flipper-ARF** (load, save, transmit): Flipper-ARF still registers ProtoPirate's Fiat V1 decoder from before its rewrite as "MARELLI" (`fiat_marelli.c`), and its users' saved MARELLI files now open in KAT, show Flipper-ARF's fields (serial, button, 5-bit counter) and replay as its encoder sends them: three copies of 100 preamble pairs, a 13-TE gap, an 8-TE sync and the Manchester frame at the file's TE. KAT has no MARELLI decoder, as FoxFW, which dropped its Decodable flag because there is no separate Marelli format on the air: run alone over KAT's 256 test captures, Flipper-ARF's decoder reads 6 frames, all of them Fiat V1 or Ford V3 frames read two bits early (on the Opel Vectra 2008 its serial is Fiat V1's UID shifted left by two). On 150 generated key files KAT's transmission equals Flipper-ARF's encoder (host oracle) pulse for pulse, except that frames longer than 96 bits are rebuilt as a Flipper's Cortex-M4 rebuilds them: the C's shift by 32 or more, undefined in C, is 0 there and something else on the x86 host. Page: [fiat_marelli.md](docs/fiat_marelli.md).

- **Tesla Charge, Hyundai/Kia RIO and SantaFe 13-16 transmit**, ported from FoxFW's encoder plugins (`applications/main/subghz/protocols_ext/enc_*.c`) and identical to them pulse for pulse on 314 made-up key files (checked on a host build of FoxFW with its plugin encoders compiled in). Tesla Charge sends the saved code 5 times at the key's TE; Hyundai/Kia RIO sends its serial with the button mask (`button << 8`) and a fresh checksum; SantaFe 13-16 sends the captured rolling word, serial and counter with the chosen button and a fresh CRC (3 copies each). TX Lock / Unlock / Trunk / Panic are buttons 1 / 2 / 4 / 8, Flipper-ARF's names for both remotes. Their key files load as FoxFW's encoder reads them (SantaFe's Rolling / Serial / Cnt / Btn lines, RIO's rebuilt frame). Nidec RKE stays receive only: FoxFW's encoder plugin for it refuses every key.

- **BMW CAS3 transmits with FoxFW's encoder**: a replay of the saved frame (3 copies, any button), identical to FoxFW's `enc_bmw_cas3.c` on 114 made-up key files. Its decoder now keeps the C's resync after a report (in the C the receiver's reset runs inside the callback, before it), so back-to-back frames all read back; before, every second copy was lost. Hitag2 and Opel V0 stay receive only: FoxFW has no Hitag2 encoder and its Opel V0 plugin refuses every key. FoxFW's Ford V3 encoder is not used: on 147 key files, the real Escape and LDV frames included, it overflows its 1400-pulse upload (a crash on a Flipper) on every real frame, and the frames that fit are read as other frames, by FoxFW's own decoder too (see [ford_v3.md](docs/ford_v3.md)).

- **FoxFW key files for five rtl_433 remotes**: Compustar-1WG3R, Astrostart-2000, Audiovox-PROOE3B, Alps-FWB1U545 and Continental-KR5V2X save and load the files FoxFW writes for its own ports of them (`Compustar`, `AstroStart 2000`, `Audiovox PRO`, `Alps FWB1U545`, `Continental RKE`). They stay receive only: FoxFW's encoder plugins for all five refuse every key file. On 760 made-up frames FoxFW's C writes KAT's lines, except 4 Alps frames its own decoder reads 2–4 bits late. FoxFW's Alps files drop 4 unknown bits and its Continental files keep only bytes 0–7 (losing the sequence's low byte, the rolling code and the checksum), so KAT adds a `Raw` line with the whole frame: KAT's files reload exactly, and FoxFW, which reads only `Bit` and `Key`, opens them and shows the same as for its own (checked on FoxFW's C with its SD info plugins). FoxFW's own files load with what they hold.

### Changed

- **Car remotes send the references' copies per press.** KAT sent one upload where the references send several: 1 of ProtoPirate's 3 for Kia V0 (and its Suzuki, Honda and Mitsubishi types), Honda Static and Kia V7, 1 of 6 for Kia V5, 1 of 2 for Chrysler V0, 1 of Flipper-ARF's 4 for Mazda Siemens and 5 for GM, and 1 of 10 for Fiat V0, Ford V0 / V1 / V2, Honda V2, Kia V1 / V2 / V6, Mazda V0, Subaru and VAG. It now sends the references' counts, except 3 instead of 10 for that last group: ten copies last up to 12 s (a Kia V1 upload takes 1.2 s), ProtoPirate's own emulate screen stops at the button's release once 666 ms have passed, and doctor-firmware's copies of these encoders send 3. Measured on the references' C for 268 key files of 99 encoders: KAT's transmissions now have the references' airtime; gate, home and the other encoders already did. See "Copies per press" in [transmit.md](docs/transmit.md).
- **A key file's `Repeat` line sets the copies**, as on a Flipper: the count of every encoder whose reference reads it (all but KeeLoq, PSA, VAG, Porsche Cayenne, Telcoma/Cardin EDGE, Doorbell32, MARELLI, Renault V1 (ARF) and Kia V3/V4), matching the references' C at `Repeat` 1, 2, 3 and 5. The line stays with the capture and is written back when it is saved as a key file; `Repeat: 0` sends nothing, and a count past the 60 s transmit limit is refused.
- **Kia V1 and Kia V2 use doctor-firmware's timing windows**: ±300 µs instead of ProtoPirate's ±200 for Kia V1 (with doctor-firmware's rule that only CRC-valid frames are reported), ±200 instead of ±150 for Kia V2. Tested on KAT's simulated radio against both settings: Kia V1 decodes 87 % instead of 29 % of frames with SNR 4 dB, 60 µs jitter and a ±10 % fob clock, and fob clocks 10–15 % off that the narrow window never read; Kia V2 98 % instead of 54–62 % with 50 µs jitter. Neither loses a frame the narrow window read, and no capture in the corpus changes. Ford V0's wider gap window in doctor-firmware was tested the same way and not taken: every real gap is well inside ProtoPirate's, and the wide one admitted an all-ones false frame. Pages: [kia_v1.md](docs/kia_v1.md), [kia_v2.md](docs/kia_v2.md).
- **Numbers in key files read as on a Flipper**: every firmware reads `TE`, `Cnt`, `Bit`, `Frequency` … with `strint_to_uint32(…, 10)`, which skips a `0x` prefix and reads the digits in decimal, so `TE: 0x190` is 190 µs there; KAT read it as hex (400). Checked by sending such a file through Flipper-ARF's, FoxFW's and Momentum's encoders. No firmware writes a prefix; this only changes hand-edited files, which now behave in KAT as on a Flipper.
- **CAME leaves slow 12-bit frames with a Faac TM gap alone** (RogueMaster's rule): a 12-bit frame at a mean Te of 340 µs or more is CAME's only when the gap after it is CAME's 47-Te repeat (42–52 Te), so fast FAAC TM remotes are read as Faac TM. Frames at CAME's own timing are unaffected; on 72 synthetic files KAT's CAME gives the same 82 frames as RogueMaster's.
- **Hörmann HSM accepts remotes whose 8th preamble bit is 0**: RogueMaster's mask `0xFE000000003` instead of `0xFF000000003`, a superset of what Flipper-ARF and doctor-firmware read.
- **Subaru is flagged AM + FM** as in Flipper-ARF and FoxFW (ProtoPirate: AM), so an FM-preset recording reaches its decoder; transmission stays AM650 unless the capture has its own preset. **Fiat Ducato V0 also listens on 315 MHz**, as FoxFW flags it.

### Fixed

- **Renault V0 lost every other frame of bursts sent close together.** A burst's sync (a HIGH of 800 µs or more) ends the previous frame and opens the next; KAT's registry reset after the report wiped that restart, so of three bursts 200 µs apart KAT listed 2 where ProtoPirate, Flipper-ARF and doctor-firmware list 3 (in the C the receiver's reset runs inside the callback, before the restart). Found by distorting car frames against the C decoders (about 720,000 runs over 40 car decoders); BMW CAS3 and Toyota (FoxFW) needed the same fix.
- **Fiat V2 lost a frame the recording ended right after.** KAT reports a Fiat V2 frame when its burst ends (its documented gate); without a pulse after the frame, it never did, where every C copy reports it. The end of the input now ends the burst. Same sweep.
- **Opel V0 overflowed on extreme pulse widths** (a panic in a debug build, a silent wrap in release): its burst sums now wrap like the C's `uint32_t`.
- **A capture or key file with a corrupt timing could crash KAT when sent.** KAT builds a transmission's samples before keying the radio (2 per µs), with no limit: a key file whose TE reads 10^9 µs (MARELLI, Tesla Charge and other encoders take the file's TE, as the references do), or a RAW file of million-µs pulses, asked for gigabytes, and a failed allocation ends the program. A transmission longer than 60 s on air is now refused with a message before any sample exists; KAT's longest protocol transmissions last a few seconds. (Found by fuzzing every encoder with random key files.)
- **With `autobf` on (the default), a PSA frame waiting for its key search could cost another remote its frame.** A PSA frame that counts only if the search finds its key (`PSA [unchecked]`) was a registry hit like any decode: it reset every other decoder and, when no key fitted, was dropped, so the frame another decoder was reading at that moment was lost and nothing was listed instead. PSA's doctor-firmware path reads such frames inside VW Polo 5 bursts: one damaged pulse in `VW Polo 5 2017 unlock2.sub` made KAT list 1 of its VAG Polo frames with `autobf` on and 2 with it off. An unchecked frame also counted as a decode when KAT decided whether to try the inverted polarity, or every protocol after those flagged for the receive path, so those passes could be skipped. An unchecked frame now resets no decoder, takes no pulses from the frames around it and decides no pass: every other protocol's decodes, and their stored pulses, are the same with `autobf` on or off (tested on every IMPORTS capture that has unchecked frames, on the file and receiver paths). Live receive, IQ files and imports all went through this.
- **Frames KAT holds back for one more pulse were lost when a recording ended first.** Mazda V1 (held until its burst ends, so Mazda V2 keeps its frames), Mercedes V1 (until a pulse that is not a chip shows the frame is not longer), Renault V1 (one pulse, so Fiat V1 keeps its frames) and PSA's doctor-firmware path (waiting for ProtoPirate's end pulse) report later than their reference, which reports on the frame's last bit. When the input ended before that pulse, for example a recording cut while the button was held, the frame was never reported. The end of the input now releases it. No capture in `IMPORTS` or the firmwares' unit tests changes; FoxFW's C and KAT now agree on Mazda V1 recordings cut inside a burst.
- **"FIAT SPA" key files load**: Flipper-ARF and FoxFW register that name for their older copy of Fiat V0; KAT reads it as Fiat V0.
- **KeeLoq Erreka key files with a seed** (doctor-firmware's / RogueMaster's "Add Manually" draws one) now load with that seed: the frame decrypts and is transmitted with it.
- **Honda keyfob presses read as Honda Static with the wrong fields.** Honda Static's bit-reversed fallback (the one path ProtoPirate's C does not checksum) misread those V5 frames: button 1 in both the Lock and the Unlock capture, a counter that never moved, and a "serial" that changed within one press (the V5 frame's retransmit byte). Honda Static still decodes its own checksum-verified fixed-code frames. Honda-KR5V2X1X frames (rtl_433's port of the same remotes) are reported as Honda V5 too, since a Flipper decoder reads them.

## [1.8.7] - 2026-11-05

A review of every protocol module against its reference C (ProtoPirate, Flipper-ARF, doctor-firmware, Momentum, RogueMaster, rtl_433), with each reference's decoders and encoders run on the real captures, the Flipper unit-test files and KAT's own transmissions. What it found and changed:

### Fixed

- **Imported captures lost their frame's last half-bit, so replaying them worked on no Flipper.** A capture from a `.sub` file keeps the pulses up to the one on which its decoder reported the frame; Manchester decoders built on the Flipper table report at the middle of the last bit, so when that middle is a LOW the stored pulses ended there, without the last bit's HIGH half. Replay sent them that way, and no reference (nor KAT) read a replayed Ford V3 EU or Ford V0 frame of that kind (2 of the 4 frames of `IMPORTS/LDV/LDV-T80_lock.sub`, about half of all Ford V0 frames); the other half, ending on a HIGH, was never affected. The stored pulses now extend over the frame's last half-bit (the pulses must decode again when sent as Replay sends them, with silence after).
- **A decoder reporting a frame now resets the others before they see the same pulse, as a Flipper does.** KAT fed every decoder the pulse first and reset afterwards, so a decoder placed after the reporting one never saw a frame-ending gap in its reset state — its next frame's header. Of a 42-bit CAME / Prastel press every other frame was lost and the Prastel fields never shown.
- **rtl_433 ports no longer hide the Flipper decoders' frames.** The receive-only Audiovox-PROOE3B port reads GateTX and Princeton frames (its checks are weak) and its hit reset every decoder on the pulse GateTX and Princeton needed, so such remotes were listed as "Audiovox-PROOE3B" and never as GateTX / Princeton. A frame an rtl_433 port reads is now held back and reported only if no Flipper decoder reads the signal; an rtl_433 hit resets the rtl_433 ports only.
- **Mazda V2: the first copy of the upload decoded by luck.** The upload (as the ARF encoder's) began with the preamble's LOW, which merges into whatever precedes the transmission: behind silence the receiver's sampler armed on the first HIGH and read the sync behind a preamble of ones (KAT's own receiver never read the first copy), behind a Flipper's receiver noise it depended on the noise's parity. Each copy now starts with a 3000 µs HIGH, which the sampler takes for a gap in every state, like the gap between the copies (after which the second and third copies always decoded). On a receiver whose gain settles during the first copy's preamble — the test rig's Flipper drops the first ~40 ms of every transmission — the first copy stays a matter of luck with any lead-in.
- **Scher-Khan captures never advanced after "send".** The upload (as the C's) carries the start bit plus every key bit and decodes back as a frame one bit longer without fields, so the frame sent could not be matched to the capture; every press re-sent counter + 1. The encoder now reports the key it sent, as the C's emulate scene writes it back into its file.
- **Flipper-ARF's VAG key files of Type 1 fobs loaded as another, undecryptable frame.** ARF writes a Type 1 button as its high nibble (`Btn: 2` for 0x20); KAT compared it with the whole byte and rebuilt the frame around plaintext 0x02.
- **KeeLoq key files with a manufacturer name KAT's keystore lacks (doctor-firmware's own brand names, its "Star" …), or whose key does not validate the frame, were refused.** They load like a frame on air now, as the references load them.
- **Marantec24:** the decoder's gap window sat 12 µs above the real remote's longest gap and under every firmware's own transmission (KAT's included, so the capture never recorded what was sent); RogueMaster's window, centred on that gap, is used.
- **Nero Radio:** the first frame of every press in the Flipper unit-test capture ends in a 1.0–1.2 ms LOW and was dropped (frame end at 1250 µs); RogueMaster's 900 µs bound reads it, and no data LOW is longer than 480 µs.
- **LinearDelta3:** a 3–4 frame tap never decoded and a held button gave every third frame; RogueMaster's decoder fix (clear the bits after each 8-bit frame) decodes every frame after the first.
- **Fiat V0** is fed captures at 315 MHz again (every reference flags 315 | 433); **Fiat V1** is AM/FM as Flipper-ARF and doctor-firmware flag it (a 2-FSK `.sub` of one now decodes).
- **Kia V3/V4:** a capture stored before the version was kept re-encoded as V4 whatever its name; the name decides now.
- The detail panel's **Key/Data** showed frames longer than 64 bits zero-padded (`0x000000F339…` for an 88-bit Datsun Lada frame: the zeros were made up and the last 24 bits are in `extra`); it shows the first 64 bits, like the references' `Key` line. Saved `.fob` files written with the padding still match.
- **CRC column:** "OK" was shown for protocols that verify nothing (Fiat V0, Fiat V2, Ford V2, Ford V3, Mercedes V0, Mitsubishi V0, Subaru) and "FAIL" for frames no check applies to (Nice FloR-S 52-bit frames, Scher-Khan's non-PRO variants, Kia V5 frames cut at 64 bits, KeeLoq frames no key identifies, Faac SLH without its seed, RojaFlex bridge commands, which carry no CRC but are checked by their token sum). All of these show "none" now (the bridge command "OK"); their decodes report `crc_valid` false in `--decode` and `.fob` files.
- **Kia V7:** button 8 is labelled "Trunk", as every reference names it (KAT said "Panic", the ProtoPirate emulate key that sends it; KAT's Panic command still sends 8).
- **Ford V3:** its EU path, which has no check at all, is left out of the inverted-polarity pass (no reference has one): a 240/480 µs Manchester frame cut short read inverted was a "Ford V3" frame with nonsense fields.
- The generic KeeLoq fallback's "Keeloq (<key>)" decodes no longer claim to be re-encodable (no encoder takes them).

### Known, not changed

- `counter` is 16 bits: VAG (24), Security+ 1.0/2.0 (32), Opel V1, Fiat V0's hop, Honda Static, BMW, PSA and Kia V6 counters show their low 16 bits.
- KAT writes key files with ProtoPirate's protocol names (`PSA`, `VAG`, `Chrysler V0`, `Kia Vn`…), which a Flipper-ARF build does not load (it matches names exactly; doctor-firmware has aliases).
- Fiat V1 does not read a key file's `Hitag2 Key` / `Hitag2 Epoch` lines (the references verify that key and send fresh frames); the saved-keys file does the same job after one "Recover key (HiTag2)".
- A Fiat-family remote at a ~125 µs half-bit (an Opel Vectra 2008 in `IMPORTS`: two of its thirteen frames) decodes as "Renault V1" rather than Fiat V1, in every reference too; a third Fiat V1 timing variant would take them.

## [1.8.6] - 2026-11-05

Volunteers' recordings: KAT now reads every code doctor-firmware and Flipper-ARF read from a Mercedes, an Alfa Romeo Giulia and a Lada.

### Fixed

- **Mercedes V1 frames that a fob ends without a long gap were dropped.** A volunteer's 2015 Mercedes (FSB4 system), recorded with a Flipper Zero: doctor-firmware and Flipper-ARF read 9 codes from its lock and unlock files, KAT 1.8.5 only 3. KAT reports a Mercedes V1 frame only if no further chip follows its 122nd bit, so that BMW's longer frames are not cut short, and it waited for a gap of 280 µs or more. This fob ends most frames with a 190–330 µs LOW and then noise. Any pulse that is not a chip now ends the frame: all 9 codes, with the references' frame counts ([docs/mercedes_v1.md](docs/mercedes_v1.md)).
- **Fiat V2 frames could be listed as Ford V3.** A volunteer's Alfa Romeo Giulia sends Fiat V2 frames. Read inverted, the first 13 bytes of its unlock frames are also a valid Ford V3 US frame, and Ford V3, earlier in KAT's registry, took 28 of the 30 unlock frames (3 of the 5 codes lost). A genuine Ford V3 US burst is a single 104-bit frame (`IMPORTS/FORD/22_ford_escape…`), so Ford V3 now rejects a frame that further bits follow. The Giulia decodes as Fiat V2 only: 5 lock and 5 unlock codes ([docs/ford_v3.md](docs/ford_v3.md)). Over the air, Flipper Zero → HackRF, with a made-up remote of the same layout: 1.8.5 listed every unlock transmission as Ford V3, now Fiat V2.
- **Datsun Lada frames behind a preamble cut short by noise were dropped.** To keep Fiat Panda frames out, KAT required 64 short pulses before a Datsun Lada frame. In a real Lada recording (posted by a doctor-firmware user), noise had cut the preamble of two first frames to 25 and 59, and doctor-firmware reads them. A frame with a shorter preamble is now accepted when it passes Datsun Lada's cipher check, which the Fiat frame does not: 19 of 19 frames, as in doctor-firmware (1.8.5: 17). Over the air, with a made-up remote, a press whose two frames both had cut preambles was listed as Fiat V1 by 1.8.5 and is now Datsun Lada ([docs/datsun_lada.md](docs/datsun_lada.md)).

## [1.8.5] - 2026-11-05

PSA as doctor-firmware reads it: KAT now decodes every PSA frame doctor-firmware decodes, including frames a fob ends without the end pulse ProtoPirate's decoder waits for.

### Fixed

- **PSA frames without the expected end pulse were not decoded.** A tester's PSA remotes work with doctor-firmware but not all of them with KAT. KAT took its PSA decoder from ProtoPirate, which finishes a frame only on an end pulse of about 1000 µs after its 80th bit (KAT: 801–1449 µs). doctor-firmware finishes the frame as its 80th bit arrives, whatever follows. A frame that ends without that pulse, or with one of another length, was listed by doctor-firmware and was an Unknown signal in KAT. Over the air, made-up frames sent from a Flipper Zero with the carrier switched off after the last bit, as a fob does, were 0 of 8 in KAT 1.8.4. doctor-firmware's own decoder (its C code, on the pulses KAT received) read 8 of 8. KAT now runs doctor-firmware's decoder beside ProtoPirate's and Flipper-ARF's, including its 125 µs path, which completes frames the same way ([docs/psa.md](docs/psa.md)):
  - a frame ProtoPirate's end-pulse rule takes is reported as before;
  - a frame only doctor-firmware's rule reads must decrypt, like the fast-timing paths' frames: an XOR frame at once, a TEA frame once `autobf` finds its key. That covers no end pulse or one of another length, a 70-pulse preamble, and the 125 µs path. On its own, the rule also reads the first 80 bits of longer frames (a VW Polo 5 remote in `IMPORTS`), which never decrypt.
  - Compared with doctor-firmware's C on 184 variations of made-up frames (end pulses, preamble lengths, timings, duty-cycle skew), KAT now reads every frame doctor-firmware reads. 1.8.4 missed 19 of its 68.
  - Checked over the air, Flipper Zero → HackRF, with made-up remotes. They used the tester's four FSK settings, OOK and stock FSK: 212 of 212 transmissions decoded, 1.8.4's own checks included, and frames without an end pulse 8 of 8 (1.8.4: 0 of 8). The four real PSA recordings in `IMPORTS`, replayed over the air, give every frame with its key; the Honda and VW Polo recordings give no PSA frame. The tester's ten key files in the receiver simulator: 2880 of 2880 presses, in every style. Sent at 250 µs without an end pulse, 1.8.4 decoded 0 of their 360 presses, 1.8.5 all 360.
- With `autobf`, two all-ones frames that 1.8.4 listed with no key (in `Peugeot 307 open.sub` and `VW Polo 5 2017 unlock2.sub`) are no longer listed.

## [1.8.4] - 2026-11-05

PSA between KAT and Flipper Zeros: KAT now receives the PSA frames a Flipper sends from saved files, and Flipper firmwares can read KAT's PSA transmissions, whatever a frame's last bit.

### Fixed

- **PSA frames a Flipper sends from a saved file were lost when their last bit was 0.** Flipper firmwares (doctor-firmware, Flipper-ARF, ProtoPirate) end each PSA frame they transmit with a 1000 µs pulse; after a final 0 bit it runs into the bit's last half, 1250 µs on the air, past the end window KAT took from those firmwares (801–1199 µs; they cannot read such a frame from each other either). So half of all PSA frames played from a Flipper's saved files never appeared, though the same frames from the fobs themselves (which end at about 900 µs) did. KAT now also accepts that merged end. Checked over the air, Flipper Zero → HackRF, with a made-up remote: KAT 1.8.3 received none of the frames ending in a 0 bit, on AM or FM, and now receives every copy; frames sent with a fob's timing decode in both. The tester's ten PSA key files sent through the receiver simulator the way a Flipper sends them: 5 of 10 before, 10 of 10 now ([docs/psa.md](docs/psa.md)).
- **KAT's PSA transmissions ending in a 0 bit could not be read by Flipper firmwares.** KAT's PSA encoder used the same 1000 µs end; it now ends frames as the genuine fobs do, with 900 µs (measured on both PSA fobs in `IMPORTS`). Checked over the air, HackRF → Flipper Zero (its raw recording decoded with the firmwares' end window): such a frame from KAT 1.8.3 was unreadable, the new one is read (9 of 10 copies, as with frames ending in a 1 bit).

## [1.8.3] - 2026-11-05

Key recovery for the testers: PSA keys are now found on their own (`autobf`), and Fiat V1, Renault V1 and Fiat V2 remotes get **Recover key (HiTag2)**, the HiTag2 Hell attack, with the recovered keys saved per remote.

### Added

- **`autobf`: PSA keys recovered without being asked** (`config.ini`, `[general]`, default `true`). Every undecrypted PSA (TEA) frame — received live, from an IQ recording, a `.sub` or key file, or a restored session — gets ProtoPirate's BF1/BF2 key search in the background, one at a time (well under a second each on a 16-core PC), also while a long KeeLoq or HiTag2 key search runs, as qDoctor's `doctor_subghz` library gives its users. A found key fills in the serial, button and counter and makes the capture encoder-capable, so it counts as vulnerable and can be exported (`.sub`, `.fob`, Flipper key) and sent; a full miss marks it "no key". The detail panel's new **Recovery** row and the vulnerability panel say which (KeeLoq keys found by **Recover key** are marked the same way), and the session keeps it. `kat --decode` runs the search in line and ends the frame's line with `key=BF1` / `key=BF2` / `key=none`. Ten doctor-firmware PSA key files a tester sorted as crackable with qDoctor's library all give the serial qDoctor found, in 0.02–0.5 s each. `false` leaves the search to the **Recover key** action.
- **Recover key (HiTag2)** for Fiat V1, Renault V1 and Fiat V2 remotes KAT has no key for (until now it could only replay them): the HiTag2 Hell attack (Verstegen, Verdult and Bokslag, 2018), ported from qDoctor's `doctor_subghz` library and checked against its C on the same inputs. KAT pairs the capture with another press of the same remote from the list (same UID, another hop): one frame fits about 65,000 keys, the second singles out the remote's. On a 16-thread laptop CPU with AVX2 the search covers its 2^19 candidates in at most 20 minutes (qDoctor's C on the same CPU: 57) and stops at the key, about halfway on average; the status bar shows its progress and the time left, and the signal menu cancels it. It runs only when asked, never through `autobf`. qDoctor's own sample capture gives qDoctor's key in 17.5 minutes (its key is 91 % into the search), and the real Opel Vectra 2008 remote in `IMPORTS` gives, from its first two presses, the key that signs all six, in 17.3 minutes ([docs/hitag2_hell.md](docs/hitag2_hell.md)).
- **HiTag2 keys per remote** (`hitag2_keys.txt` in KAT's config folder): Flipper-ARF's and doctor-firmware's key database, same format and byte order, so the file can be copied to and from a Flipper. **Recover key (HiTag2)** saves the keys it finds there, and KAT reads it at startup. A Fiat V1, Renault V1 or Fiat V2 remote whose key is in it transmits fresh codes with the requested button and counter + 1 instead of replaying the captured frame; its Flipper key files carry the key on the firmwares' `Hitag2 Key` / `Hitag2 Epoch` lines, and the detail panel's Recovery row says the key was recovered (or that none fits the two captures).

### Fixed

- **PSA fobs on the adaptive AM path showed no TEA frames.** A PSA frame that only Flipper-ARF's adaptive AM path reads (half-bits of 60–200 µs) was dropped unless it decrypted at once, so a TEA fob of that kind never appeared (that check keeps out a false PSA decode in a Honda capture). With `autobf` such a frame waits outside the list while the brute force runs, and is listed with its fields if the key is found; the Honda window never decrypts and stays out.

## [1.8.2] - 2026-11-05

Two settings asked for by the first Windows tester: list every press of a remote, and hide the signals no protocol decodes.

### Added

- **`ignore_duplicates`** (`config.ini`, `[general]`, default `true`). With `false`, every press of a remote is listed, as a Flipper's receive history lists it (Flipper-ARF, Momentum): a decoded signal is only merged with the same one (protocol, data, serial, button) that was on the air less than 500 ms before it — the same press still going, the AM and FM paths both reporting it, or both sides of a long transmission's 3-second cut. By default, as before, a signal already in the list is not added again. To tell a held button from a second press, received captures now carry when their pulses were on the air, counted in the receiver's samples. With duplicates kept, importing a file again adds its captures again. Checked over the air with a Flipper Zero and a HackRF: three presses of a Princeton remote give three captures (one by default), and a 6-second replay of a four-button remote, cut twice by the receiver, gives each of its twelve presses once, in order.
- **`show_unknown_signals`** (default `true`): the setting that lists unknown signals, under a name that says what it does. `false` lists only the signals a protocol decodes, live, from IQ files and from imports. It was called `research_mode`, which is still read when the new key is absent ([README](README.md#configuration)).

## [1.8.1] - 2026-11-05

KAT for Windows: releases now include a Windows x64 build. Also in this release: the website's move to [kat.karazajac.io](https://kat.karazajac.io), and no more "FAIL" in the CRC column for protocols that have no checksum.

### Added

- **Windows.** Releases include `kat-<version>-x86_64-windows.zip`: `kat.exe` with the three DLLs it loads next to it (`libhackrf.dll`, `libusb-1.0.dll` and MinGW's `libwinpthread-1.dll`), built in MSYS2's UCRT64 environment with its HackRF library 2026.01.3. CI builds KAT and runs the whole test suite on Windows for every push, and uploads that build as a test artifact. The radio needs a WinUSB driver, installed once with Zadig; the startup warning says so on Windows instead of naming udev rules and the DVB-T driver ([README](README.md#radio-hardware)). Settings and captures are in `C:\Users\<you>\.config\KAT`. New remotes (`:new`) take their random serial from the system's randomness on every platform (it was `/dev/urandom`). This is the first Windows release; please report what doesn't work.

### Changed

- **KAT's website moved to [kat.karazajac.io](https://kat.karazajac.io)**, rebuilt in the style of the other karazajac.io project pages, with the protocol list linking every protocol's page. The pages keep their names (`radio.html`, `kia_v0.html` …), and the old `karazajac.github.io/KAT/` addresses forward to the same page there. The site is built from `docs/` (`docs/build.mjs`); GitHub Pages now only serves the forwarding pages.
- **No "FAIL" for protocols without a checksum.** The CRC column and the detail panel said FAIL for every frame of a protocol that has no checksum to check — fixed-code gate remotes such as Princeton and CAME, Security+ 1.0, CAME Atomo, and the Alps, Audiovox, Code Alarm and Opel Mokka ports — as if the decode were wrong. They now show "-" (detail panel: "none"); OK and FAIL remain for protocols with a check. `--decode` output and `.fob` files are unchanged.

### Fixed

- The receiver page (`docs/radio.md`) was cut short on the website: a `<reason>` placeholder was read as an HTML tag and hid the rest of the page, the over-the-air results included.

## [1.8.0] - 2026-11-05

Two more Flipper firmwares, Momentum and RogueMaster, join the references for the protocols KAT lacked, and rtl_433 for receive-only decoders of remotes no Flipper firmware reads. KAT's receiver was also tested over the air for the first time, with a Flipper Zero replaying 67 `.sub` files to a HackRF: KAT 1.7 received 138 of their 250 frames, this release 239 ([docs/radio.md](docs/radio.md)). What that turned up is fixed below.

### Added

- **Gate and alarm remotes from the Flipper firmwares:** GangQi and Hollarm (doctor-firmware / Flipper-ARF: decode, Send next code, any button), Hörmann BiSecur (868 MHz FSK, Momentum / RogueMaster: decode, and re-send the stored frame as the C does) and Cardin S508 (868 MHz FSK, RogueMaster: decode only, as in the C).
- **Add Manually has 107 types** (was 101): GangQi 433MHz and Hollarm 433MHz now create remotes, and RogueMaster's six types are listed — Prastel 433MHz creates remotes; Erreka 868MHz, the three Nice O-Code types and Nord ICE say why KAT can't make them. The picker marks RogueMaster-only types "RM" and names the menus that list the selected type.
- **Receive-only decoders ported from rtl_433:** Alps FWB1U545, Continental KR5V2X and Honda KR5V2X/1X (Honda OEM remotes), Siemens 5WY72XX, Chrysler 315 MHz remotes, Opel Mokka, MIC 6SC2, Code Alarm FRDPC2002, Compustar 1WG3R, Astrostart 2000, Audiovox PRO-OE3B, the Microchip HCS361 and HCS362 KeeLoq encoders, and the Elero, Somfy io-homecontrol and RojaFlex blind and shutter remotes. They are exact models of rtl_433's pulse slicers and bit buffer, give rtl_433's fields on its test recordings where there are any, and match its C code on fuzzed pulse trains. They come last in the registry, so a frame a Flipper reference reads keeps that decoder; the capture can be replayed and exported. HCS361 and HCS362 frames go through KeeLoq identification and key recovery like an HCS200's. Markisol is ported too but, as in rtl_433, not enabled (false positives).
- rtl_433's Ford, GM, Honda, Nidec and Akhan decoders read the same frames as Ford V0, GM, Honda V0 and Princeton, so they are not duplicated; the field mappings are documented ([docs/ford_car_remote.md](docs/ford_car_remote.md), [docs/nidec_oucg8d.md](docs/nidec_oucg8d.md), [docs/akhan_100f14.md](docs/akhan_100f14.md), [docs/gm.md](docs/gm.md)).

### Changed

- **Live receive lists every frame.** A burst that holds several different frames (presses in quick succession with new counters, a fob alternating two frames, a Flipper replaying a recording) now gives one capture per frame, as a `.sub` import and ProtoPirate's receive history do; before, only the burst's first frame was kept. Each capture holds its frame's pulses and repeats, so a press that repeats one frame is still one capture of the whole burst. Found by transmitting `.sub` files from a Flipper Zero to KAT's HackRF receiver.
- **Compustar remotes are no longer reported as Scher-Khan.** Compustar 1WG3R remotes send the 35-bit frame the Flipper references read as Scher-Khan "MAGIC CODE, Static" (no fields) with every bit inverted. A frame that passes Compustar's checks now decodes as Compustar, with its ID and button; other 35-bit frames are still Scher-Khan's ([docs/compustar_1wg3r.md](docs/compustar_1wg3r.md)).

### Fixed

- **Remotes whose carrier drifts.** A cheap OOK remote whose oscillator drifts during a press, or is pulled off frequency each time it keys on, lost its first pulses and the ends of its long pulses in KAT's receiver. On rtl_433's Audiovox PRO-OE3B recordings (the carrier moves from −63 to −101 kHz within one frame) KAT decoded 1 of 3, 1 of 10 and 0 of 13 frames, plus one corrupt frame. The receiver now follows the carrier's frequency within a frame when it moves, and decodes all of them; a steady carrier gives the same pulses as before. In simulation, a Kia V1 fob that is pulled at key-on now decodes at a signal about 10 dB weaker.
- **Transmissions longer than 3 seconds.** When a burst reached the 3-second limit with the carrier still on (an FSK fob held down, a Flipper replaying a long recording), the receiver took the carrier for the new noise floor and lost the rest. The burst now continues as a new one from its last frame gap: a simulated 10-second FSK transmission decodes 99–100 of 100 frames (was 9). A fob held down that sends its frames back to back, with no gap (a Kia V1 sends a new code every ~380 ms), lost the frame across each cut; the next part now starts 500 ms back, so that frame is whole in it.
- **Recordings that start with silence.** Dead (all-zero) samples or a receiver still settling at the start of a file no longer hide the bursts after them: rtl_433's HCS361 `04/unlock` recording now decodes (all three frames), and GM, Nutek and PT2262 recordings that start the same way get proper bursts.
- **Fast FSK.** An FSK burst whose bits are too short for the normal FM path is sliced again without smoothing, so Elero's 13 µs bits come through: KAT decodes rtl_433's three Elero recordings with rtl_433's ID, command and counter. So do Somfy io-homecontrol's 26 µs bits, which the normal path showed as FSK but merged (no recording exists; checked in simulation).
- **Short pulses far apart.** The receiver's second pass estimated each frame's element length from the time its levels take, so a frame of short pulses between long gaps got the gap as its element and lost every pulse. A Chrysler V0 remote's preamble (320 µs pulses 3.7 ms apart) vanished this way: 3 of the 10 frames of `IMPORTS/chrysler/Chrysler PT Cruiser 2006 close.sub` never decoded, over the air and in simulation. When the estimate would erase a frame's pulses (or its gaps), the element length now comes from those alone.

### Known issues

- A FAAC SLH `.sub` replayed by a Flipper Zero doesn't decode in KAT: the Flipper's CC1101 puts every edge it sends on a 33 µs grid (8× the preset's data rate), which stretches some gaps past FAAC SLH's tolerance. The file itself decodes.

## [1.7.0] - 2026-11-04

A sweep through the interface for bugs and usability: KAT now fits an 80×24 terminal, no longer ends on a failed export or a radio error, never overwrites a file, and has help built in.

### Added

- **Help** (`?`, `F1` or `:help`): the keys and commands, in the app; an unknown command points to it.
- **Undo** (`u`) for the last delete (`d`, `:delete`, `:delete all`). `:delete` takes ID lists, and `:delete all` asks first.
- **ID lists** in every command: commas or spaces, `#4`, ranges either way (`1-5`, `5-1`). A command given several ends with a summary ("Replayed 3 of 5 (#04, #05 not found)").
- **Moving around**: PgUp/PgDn, Home/End and g/G in the captures list; Tab/Shift-Tab and ↑/↓ between form fields; ←/Backspace to the parent folder in the `:load` browser, which now always offers `..` (an empty import folder used to be a dead end).
- **Ctrl-C** quits (in menus and forms it works as Esc). **Quitting asks first** while a key search, an IQ file, a recording or a transmission is running.
- **Progress without hiding messages**: queued transmissions count up ("Transmitting 3/20…", Esc cancels the rest), and key searches and IQ files show their progress in the status bar's title. Before, `:replay 1-20` froze the screen, and a key search rewrote the status line every 100 ms.
- The no-device warning says why each radio couldn't be opened (permissions, busy, the DVB-T driver).

### Changed

- **The layout fits 80×24.** The header, status line, command line and key hints always keep their rows; under about 33 rows they used to disappear, so `:` commands were typed blind. The captures table drops low-priority columns when narrow instead of cutting every column, so protocol names and button codes stay whole; the detail panel shrinks or hides; below 60×16 KAT says the terminal is too small.
- **Sending a rolling code advances it.** After Send next code, Send button… or TX Lock … Panic, the capture keeps the code it sent, so the next press sends the one after, as Flipper-ARF, doctor-firmware and ProtoPirate do ([docs/transmit.md](docs/transmit.md#after-a-transmission)). Before, every press repeated the same code, so a remote made with `:new` worked only once after pairing.
- **Messages**: each action clears the last message, so an old error no longer hides later results; long messages keep both ends; the status bar always shows the selected capture's position ("#07 · 3/15").
- **Frequencies** are read the same way everywhere (`433.92`, `433.92M`, `433.92 MHz`, `433920000`): `:freq`, `:hop`, `:iq`, `--freq` and config.ini; they're shown in full (303.875 MHz, not 303.88).
- **config.ini**: switches accept yes/no/on/off/1/0, `default_frequency` takes MHz or Hz, gains are fitted to the radio's steps, and typos or bad values are reported at startup instead of being ignored silently.
- **Exports never overwrite** (`_2`, `_3`, … are added); Flipper key files default to `<name>_key.sub`; names are safe for a Flipper's SD card; the message gives the full path. `:record` never overwrites either and always uses the device's file extension, and transmitting while recording is refused instead of ending the recording.
- **Imports say what they did** ("Imported 15 captures from Kia3.sub", "Nothing imported from x.fob: …"); `.SUB` and `.FOB` files load; re-importing a file no longer adds an "Unknown" copy of it; the first imported capture is selected.
- Light terminal themes: normal text uses the terminal's own colour, and every list uses the same selection style.
- Command line: `kat rec.cu8` and `--iq x.sub` suggest the right option, `--decode` on a folder with nothing to decode says so, `--rate` is checked up front, and `--help` lists the exit codes.

### Fixed

- **Crashes**: a failed export (unplugged SD card, full or read-only disk) ended KAT, and so did a radio that failed to start or retune; two panics in the `:load` browser (non-ASCII folder names, an unreadable folder); a huge ID range (`:replay 1-4000000000`) aborted; a startup error left the terminal in raw mode.
- **The action menu could act on the wrong capture**: a capture received while the menu was open became the selection, so Enter replayed, exported or deleted it — "Delete Signal" when its menu was shorter. The menu now stays on its capture, and a new capture is selected only when the selection is on the last row.
- **The settings dropdown changed the value it showed**: when the current value wasn't a preset (after `:freq 433.42`, `:vga 30`, `:txpower 35`), the first entry was highlighted and Enter applied it — 300 MHz, 0 dB, or full TX power. The current value is now listed first.
- `:license` and `:credits` couldn't scroll to the end; the no-device warning hid "Press any key"; the `i` form said "Step 5/4"; the export file name hid its end and the cursor; the recording indicator was the first thing cut from a narrow header; the RSSI bar froze after receiving stopped, and the header kept "● RX" after the receiver failed.
- `kat --iq FILE` without a radio skipped the session-restore prompt and turned autosave off for the whole run.
- The session-restore prompt took Esc as "No", which moves the session to `session.json.bak`; only `n` declines now.
- The vulnerability panel lists each CVE once and says "+N more" when they don't fit.

## [1.6.0] - 2026-11-04

KeeLoq key recovery for "magic serial" remotes, prebuilt binaries for every release, the plain BSD 3-Clause license and a reorganised README.

### Added

- **KeeLoq key recovery** (Flipper-ARF's KeeLoq Decrypt, "BF2"): for a KeeLoq remote whose "magic serial" manufacturer key KAT doesn't have, **Recover key (KeeLoq magic serial)** in the signal menu, `:klbf` and `kat --keeloq-bf` search the unknown part of the key from two or more captures. The search tests 256 keys per pass with bitsliced AVX2 code on every core: types 6 and 7 (2^32 keys each, the Auto search) take about 5 s each on an 8-core laptop, and type 8 (2^40, opt-in, three captures) about 20 minutes. ARF's companion app, which does the search for the Flipper, isn't published, so KAT's search is its own. As ARF does, the key is saved to the user keystore as `BF_<serial>`, and the remote's captures then decode and transmit. The complete type 8 search over rtl_433's 12 real HCS200 recordings finds exactly that remote's real key (the keystore's Magic_3) and nothing else; types 6 and 7 were checked on synthetic remotes. See [docs/keeloq.md](docs/keeloq.md).
- **Prebuilt binaries** for Linux x86_64 and arm64 (glibc 2.35 or newer) and macOS on Apple Silicon, built by GitHub Actions for every release (`.github/workflows/release.yml`); v1.5.0's were added afterwards. See *Installation* in the README.

### Changed

- **License**: KAT is now under the plain BSD 3-Clause License; the extra clause about military and government use is removed.
- **README** reorganised: features grouped by task, a quick start, installation and building in one place, and the keys, signal actions, commands and config checked against the app. Newly documented: `d` (delete), `i` (capture metadata), `:lna`, `:vga`, `:amp`, `:license`, `:credits` and `:delete all`. Corrected: building needs Rust 1.88 or newer (it said 1.75); without a HackRF the transmit actions are hidden (not marked "(no TX)"); the example config shows research mode on, its default.

### Fixed

- `:load <file>` imports that file, as the README always said; it opened the file browser and ignored the argument. `:load <dir>` opens the browser in that directory.

## [1.5.0] - 2026-11-04

A new receive chain (FM fobs now decode live, OOK is far more sensitive, and fobs are found up to ±325 kHz off frequency), FSK transmit at each protocol's reference deviation, **47 garage door / gate / barrier / shutter remote protocols** with Add Manually, Flipper key files, headless decoding, IQ recordings, frequency hopping, session autosave, saved-fob matching and ProtoPirate's timing analysis. Every new protocol was checked against the reference C compiled on a PC (host builds of doctor-firmware, Flipper-ARF, its garage app and ProtoPirate), on Flipper's own unit-test captures and on real recordings from the rtl_433 test suite. Radio performance is measured on synthetic and recorded signals; none of it has been checked on KAT's hardware yet.

### Added

- **Garage door, gate, barrier and shutter remotes** — 47 protocols from Flipper-ARF's garage app (`applications/system/subghz_garage`, its newest copies) and doctor-firmware, each with decoder, encoder (where the references have one) and Flipper key files. Decodes match the reference C field for field on Flipper's unit-test captures, real recordings and fuzzed streams; uploads match pulse for pulse. They run on any frequency and are registered after every car protocol.
  - **Rolling codes:** Security+ 1.0 and 2.0 (Chamberlain/LiftMaster, incl. doctor's 2.0 keypad frames), Somfy Telis (RTS) and Keytis, FAAC SLH (seed, programming frames), KingGates Stylo4k, Jarolift, Beninca ARC (AES-128), Nice FloR-S / Nice One (+ doctor's Nice O-Code keys), CAME Atomo, Alutech AT-4N, Ditec GOL4, Phoenix V2, Hay21, Prastel, iDo 117/111 (decode only, as in the C) — "Send next code" sends counter + 1. Nice FloR-S and Alutech AT-4N need their 32-byte tables, which the Flipper ships encrypted: put a plaintext copy in `~/.config/KAT/nice_flor_s.bin` / `alutech_at_4n.bin`; without it KAT decodes what a Flipper without the file decodes.
  - **Fixed codes:** Princeton (PT2262/EV1527), CAME (12/18/24/25/42-bit), CAME TWEE, Nice FLO, GateTX, Linear, Linear Delta3, MegaCode, Holtek, Holtek HT12X, Chamberlain Code, SMC5326, Marantec (CRC-8 checked), Marantec24, BETT, Doitrand, Clemsa, Ansonic, Mastercode, Dickert MAHS, Nero Sketch, Nero Radio, Hörmann HSM, Elplast, Roger, Revers RB2, Telcoma/Cardin EDGE, Allstar Firefly, Power Smart, Dooya, Urmet (replay only).
  - Gate captures get **Replay**, **Send next code** and **Send button…** in the signal menu instead of the car commands. Send button… sends any of the remote's native buttons (the references' names, or "Btn 0xN" where they name none) or a typed code.
  - Where Flipper-ARF's garage-app transmit plugins are built without their counter/button helpers (they send button 0 or jump the counter to 0xFFFE), KAT follows the source's intended behaviour, as doctor-firmware does.
  - Real recordings that now decode: rtl_433's Security+ 1.0, Somfy Situo, Dickert MAHS, PT2262 and EV1527 remotes (fields agree with rtl_433), the UberGuidoZ Security+ 2.0 garage and LiftMaster MegaCode parking-gate captures.
- **Add Manually** (`:new`): Flipper-ARF's and doctor-firmware's 101 remote types, 78 of them available in KAT (the rest are listed with the reason: no protocol, no manufacturer key, or a table file needed). Each new remote gets a fresh serial, the manufacturer key and a first code computed as the references do. 231 of 234 generated key files are byte-identical to the references' own generation code; the other 3 are Beninca ARC, which matches doctor-firmware but not Flipper-ARF's hardware-AES copy. Remotes can be edited before creating and are transmitted like captures. See [docs/add_manually.md](docs/add_manually.md).
- **KeeLoq**: frames from manufacturers KAT has no key for are listed as **KeeLoq (Unknown)** (serial and button; replay), as Flipper-ARF does — before they were dropped. KeeLoq decodes on every band (300–318, 390 MHz gates), with the ±180 µs timing tolerance of Flipper-ARF's garage app (±140 µs in its library copy). Checked on real HCS200 recordings from rtl_433 (`KeeLoq (Magic_3)` with consecutive counters; an unknown remote with rtl_433's serial).
- **Your own KeeLoq keys**: a Flipper-format plaintext user keystore `~/.config/KAT/keeloq_mfcodes_user` (`KEY:TYPE:NAME` lines) is added to the built-in keys for identification, the Star Line / Sheriff / generic fallbacks and the encoder.
- **Flipper key files**: `.sub` files holding a decoded key (how ProtoPirate and Flipper-ARF save every decode) load like RAW captures (`:load`, the import folder, `kat --decode`) for all 40 car protocols and the gate protocols, with each reference's checks (bit count, CRC, KeeLoq / AUT64 / AES / TEA) and the names ProtoPirate, Flipper-ARF (`KIA/HYU V5`, `VAG GROUP` …) and doctor-firmware write; the 12 key files in `IMPORTS/` load (the Audi A3 2012 one only in KAT: its writer spells `Key2` as `Key_2`, which ProtoPirate and doctor-firmware don't read). Loaded keys have no pulses (Replay / RAW export / timing hidden); TX re-encodes from the fields. **Export Flipper key** writes a decode as its reference's `serialize` does. See [docs/flipper_key_files.md](docs/flipper_key_files.md).
- **BinRAW `.sub` files** (signals the Flipper saved as bit blocks) load as pulses, built the way Flipper-ARF's encoder sends them.
- **Headless decoding**: `kat --decode <files|folders> [--freq] [--rate] [--all] [--verbose]` decodes `.sub` files (RAW, BinRAW, key) and IQ recordings without the TUI and prints one line per distinct decode; `--verbose` dumps each IQ burst and its pulses.
- **New real captures in `IMPORTS/`** (from the owner's key-captures collection): Subaru Forester 2012 and WRX 2018 (AU), Holden Barina 2016 (Opel V1), Toyota Estima 2000 (ACR30, three captures), Peugeot 307 (PSA TEA — **Recover key** decrypts them in ~30 ms: serial 02F0AB), Opel Vectra 2008 (Fiat V1 / Marelli), Chrysler PT Cruiser 2006 (Chrysler V0, open and close) and an Audi A3 2012 VAG key file (serial 3BF8E083, as the sender's screenshot shows). Not decodable yet, see [docs/unidentified_captures.md](docs/unidentified_captures.md): VW Polo 5 2017 (a 96-bit Manchester format no reference implements), Lexus UX 250 2021 (256-bit FSK frames, differentially encoded), Volvo S40 1999 (64-bit frames in Chrysler V0 timing), Renault Megane Scenic 2005, BYD Sealion, BYD Seal 2025 and a BYD "apertura" capture (FSK fobs recorded with an AM preset), and Mercedes W220 1999 and BYD Atto 3 (no signal in the file).
- **New receive pipeline** shared by the HackRF, the RTL-SDR and IQ files: offset tuning (LO fs/8 below the target, away from the DC spike); a wide-band FFT detector with a noise floor per bin that finds fobs anywhere in the band (at 2 MS/s from 1.15 MHz below to 0.65 MHz above the target — a Flipper's ±325 kHz OOK window included — with each signal its own burst, so two fobs at once are two captures, and I/Q images and spurs dropped); then per burst the carrier mixed to 0 Hz, a ±100 kHz channel filter and slicing: OOK between noise and carrier level, FSK with both tones found per burst (automatic frequency correction), then each frame re-sliced with smoothing matched to its element length. Fobs decode equally well up to ±325 kHz off the target (synthetic sweeps); in rtl_433's test recordings six HCS200 presses ~120 kHz off frequency now decode. The HackRF's samples are processed on their own thread, as the RTL-SDR's already were. See [docs/radio.md](docs/radio.md).
  - **FM/2-FSK receive works**: KAT 1.4's FM demodulator never closed a capture once there was noise. Now half the presses of ±20 and ±47.6 kHz FSK decode at about −6 dB SNR (2 MHz bandwidth), at any fob offset up to ±325 kHz.
  - **OOK is far more sensitive**: half the presses decode at about −8.5 dB, at any offset up to ±325 kHz, where 1.4 needed 12–18 dB.
  - Noise alone produces no captures, and one burst no longer yields both an AM and an FM capture.
- **IQ recordings**: decode `.cs8` (hackrf_transfer), `.cu8` (rtl_sdr) and `.cf32` recordings in the background, with or without a radio (`:iq <file> [MHz] [rate]`, `kat --iq <file> --freq … --rate …`, or the `:load` browser); `:record [file]` writes the raw IQ while receiving, named so `:iq` and `hackrf_transfer -t` replay it at the right frequency.
- **Frequency hopping** (`:hop on|off`, `:hop <MHz> …`, `:hop dwell <ms>`, a **Hop** setting): Flipper-ARF's hopper list (315.00, 390.00, 430.50, 433.92, 434.25, 434.42, 868.35 MHz) with ProtoPirate's behaviour (stay on a frequency while a signal is on air, then pause after a decode); the header shows the frequency being heard.
- **Command line**: `--iq`, `--freq`, `--rate`, `--help`, `--version`.
- **FSK transmit**: FM protocols now go out as continuous-phase 2-FSK at the deviation the references use for them (FM476 ±47.6 kHz for the Kia family, F4 ±14.3 kHz for Ford V1–V3 and Honda V2, Honda1 ±15.9 kHz for Honda Static, F3 ±31.7 kHz for PSA's FM variant, F2 ±19.0 kHz for Mazda V0, doctor-firmware's FM20K ±12.7 kHz for Mercedes, FM476 where a reference names none). KAT 1.4 sent every protocol as OOK. The modulation comes from the capture's own preset (`.sub` imports), else the path it was received on, else the protocol's primary preset; see [docs/transmit.md](docs/transmit.md).
- **TX power settings**: `tx_vga_gain` (0–47 dB) and `tx_amp` in `config.ini`, a **TX** tab in the radio settings and `:txpower <0-47> [amp on|off]`. Defaults keep 1.4's full power (47 dB + amp).
- **Session autosave and restore**: all captures are saved to `~/.config/KAT/session.json` (atomic writes, owner-only permissions, at most once a second and on exit) and offered for restore at the next start; declining keeps a `.bak`. `:clear` empties the list and deletes the saved session. `[session] session_autosave` turns it off; `max_captures` caps the saved session. See [docs/session.md](docs/session.md).
- **Saved-fob matching** (ProtoPirate's "check saved" rules: same protocol, same serial or key, counter within ±50): decoded captures that match a `.fob` in the export or import folder get a ★ in the list and the saved vehicle details in the detail panel; a live match shows a status message.
- **Timing analysis** (ProtoPirate's Timing Tuner): for any capture, from the signal menu or `:timing [ID]`, compares the measured short/long pulses with the protocol's definition and gives ProtoPirate's verdict; unknown signals get pulse clusters, a base-TE estimate and a line-coding guess. See [docs/timing_analysis.md](docs/timing_analysis.md).
- `.sub` import keeps the file's preset (stock and custom CC1101 presets), `.sub` export writes the capture's preset back (KAT 1.4 always wrote `Ook270`), and `.fob` stores the receive path and preset.

### Changed

- **Modulation gate** (Flipper-ARF's receiver filter, after ProtoPirate's per-preset registry): decoders only see captures of their modulation — a `.sub` file's preset decides; KAT's receiver tries the protocols of the burst's AM or FM path first and every protocol only when none decodes (protocol flags are approximate: real OOK Hyundai fobs send Kia V3 frames, flagged FM). No IMPORTS decode changed.
- The inverted-polarity retry (for captures recorded with carrier-on as LOW) is kept for the car protocols; the gate protocols take part only on FSK input, where either tone can be "on" (the Flipper has no such pass).
- **Kia V2** reports only CRC-valid frames (the C displays "CRC BAD" and emits them): both real Kia V2 keys in IMPORTS carry a valid CRC, while real EV1527 garage remotes and a Hyundai fob recorded by rtl_433 produced CRC-invalid "Kia V2" frames.
- **Mitsubishi V0** (ARF 96-bit, no checksum) drops 96 identical symbols: a real GM fob's preamble decoded as an all-ones Mitsubishi frame.
- **Opel V1** reports every button whose XTEA block validates, like the C: KAT's Trunk/Lock/Unlock whitelist dropped a real 2016 Holden Barina fob (button 0x73), which Flipper-ARF and doctor-firmware decode.
- **KingGates Stylo4k**'s keystore entry is typed 10 (`SIMPLE_KINGGATES`) as the references expect (the dump said 1); the real `kinggates_stylo4k_raw.sub` remote decodes only that way.
- Prastel is registered after CAME (both references do), so 42-bit frames both accept are CAME's, as on a Flipper.
- **KeeLoq transmits with the manufacturer the capture or key file names**, as Flipper-ARF's encoder does with `Manufacture`, instead of re-identifying it from the frame. About 10 % of new KeeLoq remotes would otherwise have gone out with another manufacturer whose key also matches. Named key files and BFT keys with their own seed are now transmittable.
- **Chrysler V0 transmits the fob's next press** (deviation, from real captures): the counter is seed − 1, made even, so the payload of the captured type is the next one the fob would send. The real Chrysler PT Cruiser 2006 fob in `IMPORTS/chrysler` counts its seed **down** by one per press, alternating payloads A and B, and re-encoding each of its presses reproduces its press two later bit for bit (16 of 16). The references count up: ProtoPirate and doctor-firmware add 1, which re-sends the captured press; Flipper-ARF adds 2, which KAT followed and which sends an earlier press. A real fob has already sent both. See [docs/chrysler_v0.md](docs/chrysler_v0.md).
- The RTL-SDR reads on its own thread (it retunes between reads when hopping).
- **Protocol metadata in one place**: each protocol module declares its line coding, RF, encryption label, make, native buttons and TX presets (`protocols::info`), replacing name-matched tables in `capture.rs`, `app.rs` and `protocols/mod.rs`. A test pins every value to KAT 1.4's except the intended fixes below.
- **Offset transmit**: the HackRF LO is tuned 500 kHz below the carrier and the signal is shifted digitally, so LO leakage no longer sits on the carrier during OOK "off" time. A transmission now ends when the last sample has left the device instead of after a fixed sleep.
- **Kia V2** is tagged AM/FM and transmits FM476 by default: ProtoPirate's help text, its README and both real Kia V2 captures in `IMPORTS/` use FM476 (its catalog flag says AM).
- **Honda V1** is labelled "Rolling Code" (it carries a 16-bit counter; ProtoPirate's label) instead of "Fixed Code".
- Queued transmissions (`:replay 1-5`, …) go out in order; they were sent last-first.

### Fixed

- Exporting a capture whose make contains a slash (Kia/Hyundai, Honda/Acura …) ended KAT with "No such file or directory".
- Gate and garage protocols without button names show "Btn 0xN" instead of Lock/Unlock/Trunk/Panic. The KeeLoq barrier list had a typo (`faacrctx` → `faacrcxt`, so FAAC RC,XT remotes missed the gate menu) and lacked 9 brands, which now get the gate menu too.
- `.sub` files with commas between RAW values (some Flipper unit-test captures) failed to load; IQ file names with `-` separators (rtl_433's `…-868.3M_1000k.cu8`) lost their frequency.
- **HackRF stop race**: the vendored libhackrf wrapper freed its callback context before stopping the stream (and freed it as the wrong type), so a late callback could touch freed memory. It now stops first and frees the context properly, for RX and TX.
- A failed transmission shows "Transmit failed: …" and RX resumes, instead of ending KAT.
- Stale metadata for the retired BMW CAS4; legacy names such as "PSA OLD", "Mitsu V0" and "KIA/HYU V4" now get their protocol's full details.

### Maintenance

- **Flipper's own Sub-GHz unit test** runs in KAT's suite (when `REFERENCES/` is present): every decoder case decodes its capture and every encoder case's key file encodes and decodes back (three encoder cases that fail in the reference C itself are left out, see the test).
- **CI**: a GitHub Actions workflow builds KAT and runs the full test suite on every push and pull request.
- **Decode regression test**: `imports_decode_golden` decodes every `IMPORTS/*.sub` capture and compares every field of every decode (1,086 decodes from 157 files) with `testdata/imports_decodes.txt`; regenerate it with `KAT_UPDATE_GOLDEN=1 cargo test imports_decode_golden` after an intended change.
- **Dependencies**: removed the unused `tokio`, `bitvec`, `thiserror` and `toml` crates, and replaced the unmaintained `atty` with the standard library's `IsTerminal` (Cargo.lock: 185 → 154 packages).
- **Line endings**: all text files are LF, enforced by `.gitattributes` (`.sub`/`.fob` captures are kept byte-for-byte). Removed the tracked macOS `.DS_Store` files.

## [1.4.0] - 2026-11-03

Capability parity with the references in `REFERENCES/` — [ProtoPirate](https://protopirate.net/ProtoPirate/ProtoPirate) (primary), [Flipper-ARF](https://github.com/D4C1-Labs/Flipper-ARF) and [doctor-firmware](https://github.com/thedoctorz/doctor-firmware): KAT decodes and transmits what they do, mirrors their retirements, and every protocol was checked against the reference C compiled on a PC (decoders field-for-field, encoders pulse-for-pulse) and against the `IMPORTS/` captures. 40 protocol decoders are registered.

### Added

- **New protocols:**
  - **From ProtoPirate:** **Renault V0** (type-13 frames; encoder with counter + 1); **Renault V1** (HITAG2; the 2^18 serial-derived seed search runs automatically, ~10 ms, so KAT transmits fresh frames; replay otherwise); **Fiat V2** (FCA). Fiat V2 decodes the real **2019 Chrysler Pacifica** and **2022 Jeep Renegade** captures; it forward-encodes when one of ARF's HITAG2 keys verifies, else replays.
  - **From Flipper-ARF / doctor-firmware:**
    - **BMW** (62 µs Manchester, 128-bit; ARF's replacement for BMW CAS4).
    - **Mazda V2** (table-driven next code).
    - **Mercedes V0** and **Mercedes V1**.
    - **Opel V1** (XTEA, decode-only like ARF).
    - **GM** (Chevrolet/Buick/GMC/Cadillac, rtl_433 format; replay).
    - **Datsun Lada**.
    - **Sheriff CFM** (ZX-750/930 alarm, KeeLoq).
    - **Renault V1 (ARF)** (ARF's 82-bit type-table frames 0x04/0x0C/0x1A/0x3B/0x3F + Dynamic; replay).
- **Transmit where KAT could only receive:** Kia V5, Mitsubishi V0 (ARF 96-bit; replay), Scher-Khan (ARF Magic Code PRO1/PRO2), Toyota, Renault V1, Fiat V1 (ProtoPirate's 9 keys plus ARF's key dictionary and heuristics; replay otherwise) and Porsche Cayenne.
- **Counter + 1 on every re-encode** where the references bump it before sending (ProtoPirate's emulate scene; ARF/doctor encoders): Ford V0/V1/V2, Honda Static/V1/V2, Kia V0–V7, VAG (24-bit), Subaru, Chrysler V0 (seed + 2), Mazda V0/V2, Renault V0/V1, PSA, Star Line, KeeLoq. Re-encoding a real Ford V0 press reproduces the fob's real next press bit for bit (7 pairs).
- **Kia V0** decodes ProtoPirate's four frame types: Kia V0, **Suzuki V0**, **Honda V0** and **Mitsubishi V0**, with their display names and encoders.
- **PSA "Recover key" action**: ProtoPirate's TEA key brute force, offered as an on-demand signal-menu action like the references do. It runs on a background thread (two 2^24 ranges, multithreaded, ≈0.5 s worst case on a 16-core PC) with progress in the status line and a cancel action. The Groupe PSA captures hold two fobs: serial 0x99EB25 (XOR, decoded directly) and 0x014590 (TEA, via Recover key).
- **Star Line** searches the whole keystore like the C ("Star Line [manufacturer]").
- **KeeLoq** (gates/garage doors/alarms):
  - Flipper-ARF's per-key learning types, AN-Motors, HCS101, Superrollo and the special manufacturers.
  - The full ARF encoder.
  - DoorHan's learning type is corrected from three real remotes.
- **Real-capture decodes** that were previously empty: 2022 Ford Escape (Ford V3 US), Fiat Bravo (Fiat V0), VW Passat 2015 and Passat B7 (VAG Type 1), VAG_2 and the Suran capture (VAG Type 2), Chrysler Pacifica and Jeep Renegade (Fiat V2).
- **Shared HITAG2 module** (`hitag2.rs`) for Fiat V1, Fiat V2 and Renault V1.
- **Native button names** for protocols that report their own button codes.
- **`.fob` keeps encoder state** (`signal.data_extra`). Imported `.fob` captures with raw pairs are re-decoded with the current decoders, so old exports pick up renames and new encoders.
- **Tests and diagnostics:**
  - A keystore regression test pins the reference keystore dump (Kia V3–V6, Star Line / alarm KeeLoq keys, Sheriff ZX, VAG AUT64).
  - The `detail_decodes` diagnostic prints decoded fields per capture.
  - Cross-protocol guard tests check that every encoder's output is claimed only by its own decoder.

### Changed

- **Re-synced with the current references** (most older ports predated later reference fixes):
  - **Kia V1, V2, V5 and V6** previously decoded garbage and now match the C.
  - **Kia V3/V4** now requires the KIA KeeLoq check, as both references do, and reports "Kia V3" / "Kia V4".
  - **VAG** is an exact port of ProtoPirate's state machine (all four types decode and transmit; ARF's 315 MHz). Its Type 1 frames stay Type 1 because the real Passat captures prove it.
  - **Subaru:** ProtoPirate's reader with ARF's as a fallback. The encoder's counter rule is taken from the real Impreza capture (9/9 frames reproduced).
  - **Chrysler V0:** counter = seed.
  - **Ford V1:** `try_last_byte_variants`; the encoder sends burst 0 like the C.
  - **Ford V2:** ARF's encoder.
  - **Ford V3** gains ProtoPirate's US variant. KAT relaxes its flag/button gate because the real 2022 Escape carries flag 0x08 and Trunk/Panic codes 0x07/0x06.
  - **Fiat V1:** ProtoPirate's HITAG2 re-port.
  - **Mazda V0:** ProtoPirate's current Manchester decoder, plus an encoder.
  - **Mazda Siemens:** completes at 79 bits (ARF's C never completes its own 80-bit frames); the encoder's button-byte bug is fixed.
  - **Renault V0:** ARF's alternate frame reading.
  - **Toyota:** ARF's current Toyota/Lexus protocol, with its encoder.
  - **Porsche Cayenne:** synced with ARF (every frame transmittable, 315 MHz).
- **Mirrored retirements, renames and disables** (legacy names still resolve, so old captures and `.fob` files keep working):
  - **Land Rover V0 → Honda V2** (ProtoPirate rename).
  - **BMW CAS4** retired; ARF replaced it with **BMW**.
  - **Porsche Touareg** retired; ProtoPirate removed it, and it was the same frame as Porsche Cayenne.
  - **Suzuki** retired; ProtoPirate folded it into Kia V0.
  - **PSA2** retired; ARF never registers it, and PSA now decodes those captures with the same fields.
  - The **KeeLoq Toyota** was retired; ARF replaced it with the current Toyota.
  - **Audi**, **Mazda Infinity** and **Hyundai** (ARF's Pandora reverse-engineering, shipped disabled) and **Land Rover RKE** (an ARF helper that ARF never registers) are kept as code and docs but not registered, as in ARF.
- **Registry order:** Sheriff CFM sits before the KeeLoq decoder; Renault V1 (ARF) sits right after Renault V0.

### Fixed

- **Manchester decoding.** Several modules carried a hand-written table that is not Flipper's `manchester_decoder.c` and decoded nothing. Ford V0 previously "decoded" the real Ford captures with failing CRCs and changing serials. `common.rs` now holds one exact port.
  - All 7 real Ford V0 captures decode with valid CRCs (7-bit, as the C), one serial per fob, buttons matching the file names and counters rising per press.
- **Bogus decodes removed:**
  - Isuzu and Prius 2006 frames were claimed by Kia V3/V4 without its KeeLoq check.
  - A VAG capture decoded as Fiat V1.
  - A Camry noise burst decoded as the old Toyota.
- **Scher-Khan 51-bit encoder** (documented deviation from Flipper-ARF): ARF masks the key with `0x7FFFFFFF0000`, which drops serial bits 47–50 (its own sample frame `0E8C1619E830C` would go out as `068C…`, a different remote). KAT keeps bits 16–50 and updates the inverted-button nibble together with the button.
- **Mercedes V1 vs BMW:** Mercedes V1 now requires an exact 122-bit frame, so it no longer cuts BMW's 128-bit frames.
- **Panics in debug builds:** Fiat V0 on a 799 µs gap, and Subaru's rotation count.
- **TX lookup** of display names with a variant suffix (e.g. "Porsche Cayenne [First]", "Star Line [Cenmax]").
- **Labels:**
  - Ford V0/V1/V2, Fiat V0, Kia V0–V2/V5/V7 and Toyota are rolling codes, not fixed codes.
  - Ford V0 and Fiat V0 are AM.
  - PSA is TEA/XOR, AM/FM.

## [1.3.0] - 2026-06-23

### Added

- **14 new protocol decoders** (total now 32), ported from the [ProtoPirate](https://protopirate.net/ProtoPirate/ProtoPirate) reference and the [Flipper-ARF](https://github.com/D4C1-Labs/Flipper-ARF) firmware:
  - **ProtoPirate (8):** **Kia V7** (Manchester 250/500, header 0x4C + CRC8), **Ford V1** (Manchester 65/130, descramble + CRC16/CCITT), **Ford V2** (Manchester 200/400, 0x7FA7 sync), **Ford V3** (Manchester 240/480, plaintext, decode-only), **Chrysler V0** (PWM, seed-XOR transform; Dodge/Jeep), **Honda Static** (FM, XOR checksum), **Honda V1** (PWM/PPM, CRC-fold), **Land Rover V0** (differential Manchester, 3-bit check polynomial).
  - **Flipper-ARF (6):** **Toyota** (dual PWM + NRZ variants, KeeLoq, decode-only; Lexus), **Land Rover RKE** (PWM, KeeLoq), **Mazda Siemens** (Siemens XOR/interleave), **BMW CAS4** (Manchester, 0x30/0xC5 markers, decode-only), **Porsche Cayenne** (PWM, VAG rolling-register cipher; adds an encoder to the shared Touareg air protocol), **PSA2 / "PSA OLD"** (Manchester, TEA with bounded brute-force gated behind a structural/checksum check).
- **Real-capture validation** — decoders verified against `IMPORTS/` samples where they exist: **Ford V3** (LDV T80), **Honda Static** (Honda), **Toyota** (Camry NRZ variant), **PSA2** (Groupe PSA, recovered serial 0x99EB25 + rolling counter). The rest are validated by in-module encode→decode round-trip and synthetic-frame unit tests.
- **Metadata, make-suggestion, and docs** — each new protocol is tagged with Encoding / RF (AM/FM) / Encryption in the signal detail panel, mapped to a Make suggestion (Honda/Acura, Chrysler/Dodge/Jeep, Toyota/Lexus, Land Rover, BMW, Porsche, Mazda, Peugeot/Citroën), and documented under `docs/`.

### Fixed

- **KeeLoq decrypt debug-panic** — `keeloq_common::keeloq_decrypt` computed `15 - r` (with `r` up to 527), underflowing and panicking in debug builds / `cargo test`; now uses `wrapping_sub` to match the reference's unsigned wraparound (release behavior, which silently wrapped, is unchanged).

### Notes

- All new decoders are gated (CRC / checksum / fixed markers / frame structure) so they do not false-match existing protocols — every previously-decoding `IMPORTS/*.sub` capture decodes unchanged. Where two decoders share a wire protocol (Porsche Cayenne ↔ Touareg; Mazda Siemens ↔ Mazda V0; Toyota variant-A ↔ Kia V3/V4 KeeLoq), the pre-existing decoder keeps first-match priority.

## [1.1.3] - 2026-02-20

### Added

- **Command field** — Capture metadata (press **i**) and .fob export now include **Command** (e.g. Unlock, Lock, Trunk, Panic). Export filename for unknown protocol uses Year_Make_Model_Region_Command; 8-hex suffix is shown in the filename field and saved in uppercase (e.g. `…_A1B2C3D4.fob`). .fob vehicle info and import support optional `command`.

### Changed

- **Unknown signals** — Shown by default (`research_mode` default is now `true`). Config comment and storage docs clarify that no keystore directory is used or created (keys are embedded).
- **Signal separation** — End-of-signal gap increased from 20 ms to **80 ms** so one button press (multiple bursts with 25–50 ms gaps) produces a single capture instead of 3–4.
- **Short signals** — Demodulator now emits captures with **5+** level-duration pairs (was 10), so short or weak unknown keyfob bursts are no longer dropped (RSSI spike but no capture).

### Fixed

- **Export filename** — Unknown-protocol .fob exports always get the 8-hex suffix (including when filename ends with `Unknown`); suffix is uppercase.

---

## [1.1.2] - 2026-02-20

### Added

- **Vulnerability database (CVE)** — Expanded CVE coverage: unique `id` per vuln row; year range (year_start/year_end) and arrays for makes/models; source URL per CVE. New/updated CVEs: **CVE-2022-38766** (Renault ZOE), **CVE-2022-27254** (Honda Civic), **CVE-2022-37418** (RollBack, Honda/Hyundai/Kia/Nissan — per-model year ranges), **CVE-2022-36945** (RollBack, Mazda, three consecutive signals through 2020), **CVE-2019-20626** (Honda/Acura static-code replay; confirmed vehicles per Unoriginal-Rice-Patty). Vulnerability panel shows NVD source link for each match. README table of CVEs with NVD links.

### Changed

- **Commands** — `:replay`, `:lock`, `:unlock`, `:trunk` (and `:panic`) accept multiple IDs: single (`1`), comma list (`1, 3, 5`), range (`1-5`), executed in order.
- **RSSI bar** — Shows **TX** with red bar and border while transmitting; draw-before-transmit so TX state is visible.

---

## [1.1.1] - 2026-02-13

### Changed

- **UI updates** — Vulnerability panel (green border when vuln found, green “encryption broken” text); signal action menu shows TX Lock/Unlock/Trunk/Panic only for encoder-capable captures (unknown/decoded get Replay only).

---

## [1.1.0] - 2026-02-13

### Added

- **RSSI Bar** — Live received signal strength indicator in the UI.
- **KeeLoq Decodes** — KeeLoq generic fallback and keystore-based decoding (see 1.0.1); listed here for 1.1.0 release.
- **Vulnerability Database** — Built-in CVE database (Year/Make/Model/Region). New **Vuln Found** column (Yes/No). Press **i** on a capture to set Year/Make/Model/Region; matching CVEs appear in the detail panel. Same metadata used for .fob export.

### Updated

- **Keystore** — Keystore improvements and additional keys.

### Fixed

- **VAG decoding** — Fixes and improvements for VAG protocol decoding.

---

## [1.0.2] - 2026-02-13

### Added

- **RTL-SDR support (receive-only)** — KAT can use an RTL-SDR (e.g. RTL433-style dongles) when no HackRF is present. Device selection: HackRF first, then RTL-SDR. With RTL-SDR, capture and decode work as normal; transmit (Lock/Unlock/Trunk/Panic, Replay) is disabled with a clear message. Header shows **RTL-SDR (RX only)**; signal menu shows **(no TX)** on transmit actions when using RTL-SDR. Dependency: `rtl-sdr-rs`. README updated with supported hardware and Linux DVB-T note.

### Fixed

- **:q / :quit terminal state** — `:q` and `:quit` now request quit via the main loop instead of `std::process::exit(0)`, so the terminal is properly restored (raw mode off, alternate screen left, cursor shown), matching behavior of pressing `q`.

### Changed

- **UI** — DISCONNECTED status in the header is now shown in red. Startup no-device warning text updated to "or continue without TX/RX support" (was "or continue in demo mode"). Header displays the active device name (HackRF, RTL-SDR (RX only), or No device).

---

## [1.0.1] - 2026-02-13

### Fixed

- **Ford V0** — Decoder fix for Ford keyfob signals (BinRAW/RAW .sub handling and decode alignment).

### Added

- **KeeLoq generic fallback** — When a capture does not decode as any known protocol, KAT now tries to decode it as KeeLoq using every manufacturer key in the embedded keystore (Kia V3/V4 and Star Line air formats). Successful decodes appear in the capture list as **Keeloq (*keystore name*)** (e.g. Keeloq (Alligator), Keeloq (Pandora_PRO)). Implemented in `keeloq_generic.rs` using the `keeloq_common` helper only.

### Changed

- **Embedded keystore** — Updated with additional manufacturer keys (including Pandora and other entries). KeeLoq generic fallback uses all KeeLoq MF keys (types 0, 1, 2, 10, 20) from the keystore.

---

## [1.0.0] - 2025

Initial release. Multi-protocol decoding (Kia V0–V6, Ford V0, Fiat V0, Subaru, Suzuki, VAG, PSA, Scher-Khan, Star Line), HackRF capture and transmit, .fob and Flipper .sub export, embedded keystore, research mode, TUI.
