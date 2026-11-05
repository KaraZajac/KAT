# Changelog

All notable changes to KAT are documented here.

## [Unreleased]

### Added

- **Windows builds, for testing.** CI builds KAT on Windows x64 (MSYS2's UCRT64 environment, with its HackRF library and libusb), runs the test suite there, and uploads a test build: `kat.exe` with the DLLs it loads. Releases get a `kat-<version>-x86_64-windows.zip`. The radio-not-found hints name the Windows step (a WinUSB driver, installed with Zadig) instead of udev rules and the DVB-T driver, and new remotes take their random serial from the system's randomness on every platform.

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
