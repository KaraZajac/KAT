# KAT — Keyfob Analysis Toolkit

A terminal-based RF signal analysis tool for capturing, decoding, and retransmitting car keyfob and garage door / gate remote signals. Built in Rust with a real-time TUI powered by `ratatui`. Protocol decoders are aligned with the [ProtoPirate](REFERENCES/ProtoPirate/) reference.

**Supported hardware:** KAT uses **HackRF One** (or compatible) when present for full receive and transmit. For **receive-only** operation, **RTL433** (or any compatible RTL-SDR dongle) is supported: if no HackRF is found, KAT uses the first available RTL-SDR for capture and decode; transmit (Lock/Unlock/Trunk/Panic, Replay) is disabled when using RTL433/RTL-SDR.

![Keyfob Analysis Toolkit screenshot](images/kat-screenshot.png)

---

## Security and legal disclaimer

**Use KAT only on systems and vehicles you own or have explicit, written permission to test.** Capturing, decoding, or transmitting keyfob and vehicle-access signals without authorization may be illegal in your jurisdiction (e.g. computer misuse, unauthorized access, or radio regulations). You are solely responsible for ensuring your use complies with all applicable laws. KAT is intended for security research, authorized penetration testing, education, and legitimate testing on your own equipment. The authors and contributors assume no liability for misuse or damage arising from use of this software.

---

## Features

- **Real-time capture, AM/OOK and FM/2-FSK** — one receive pipeline for HackRF, RTL-SDR and IQ recordings. A wide-band detector finds fobs anywhere in the received band: at 2 MS/s from 1.15 MHz below to 0.65 MHz above the tuned frequency, two at once included. Each burst is then filtered around its own carrier and sliced with automatic frequency correction, so OOK and FSK fobs decode up to ±325 kHz off frequency ([docs/radio.md](docs/radio.md))
- **Multi-protocol decoding** — 87 protocol decoders: 40 car keyfob and alarm protocols — Kia V0–V7 (Kia V0 also decodes Suzuki V0, Honda V0 and Mitsubishi V0), Ford V0–V3 (V3 with EU + US variants), Chrysler V0 (Dodge/Jeep), GM (Chevrolet/Buick/GMC/Cadillac), Honda Static, Honda V1, Honda V2, Toyota (Lexus), Fiat V0/V1/V2, Renault V0/V1 (+ Flipper-ARF’s type-table Renault V1), Mazda V0, Mazda V2, Mazda Siemens, Mitsubishi V0, BMW, Mercedes V0/V1, Opel V1, Datsun Lada, Porsche Cayenne, Subaru, VAG (VW/Audi/Seat/Skoda), PSA, Scher-Khan, Star Line, Sheriff CFM, KeeLoq (gates/garage doors/alarms) — and 47 garage door, gate, barrier and shutter remote protocols from Flipper-ARF's garage app and doctor-firmware (Security+ 1.0/2.0, Somfy Telis/Keytis, FAAC SLH, Nice FLO/FloR-S, CAME/TWEE/Atomo, Hörmann HSM, Marantec, Beninca ARC, KingGates, Jarolift, Princeton/PT2262 and more; see [Supported Protocols](#supported-protocols)), every one checked against the reference C compiled on a PC
- **KeeLoq generic fallback** — when a signal doesn’t match any known protocol, KAT tries decoding it as KeeLoq using every manufacturer key in the embedded keystore (Kia V3/V4 and Star Line air formats); successful decodes appear as **Keeloq (keystore name)** in the capture list
- **RF modulation metadata** — each protocol is tagged AM, FM or both and carries the TX presets the references use for it; shown in signal detail and exported in .fob
- **Rich signal detail** — encoding (PWM/Manchester), RF (AM/FM), encryption, serial, counter, key data, CRC, frequency, and raw level/duration pairs
- **Signal retransmission** — transmit Lock, Unlock, Trunk, and Panic commands from decoded captures, or Replay raw capture (HackRF only; when using RTL-SDR these show “(no TX)” and a receive-only message). Rolling-code protocols re-encode the next code with the requested button wherever the references do (e.g. VAG all types, Subaru, Chrysler V0, Kia, Ford, Mazda, Renault V0, Sheriff CFM); protocols without an available key re-send the captured frame, as the references do. FM protocols go out as continuous-phase 2-FSK at the references’ deviation, OOK ones as OOK, at a configurable TX power ([docs/transmit.md](docs/transmit.md)).
- **Add Manually** — `:new` creates a new gate/garage remote of any of Flipper-ARF's and doctor-firmware's 101 types (78 available in KAT), to pair with a receiver. Each gets a fresh serial, the manufacturer key and a first code computed as the references do. Gate remotes get **Send button…** for their native buttons ([docs/add_manually.md](docs/add_manually.md))
- **Export formats** — `.fob` (versioned JSON with vehicle metadata, signal info, optional raw pairs), `.sub` RAW (Flipper Zero compatible) and Flipper **key files** (the decode as ProtoPirate / Flipper-ARF save it, so a Flipper can load and send it)
- **Import support** — load `.fob` files with automatic v1/v2 format detection, and Flipper `.sub` files: RAW captures, BinRAW captures and decoded **key files** (`Protocol`/`Bit`/`Key` + the protocol's lines, as ProtoPirate, Flipper-ARF and doctor-firmware write them; loaded keys transmit through the protocol's encoder). The file's CC1101 preset is kept for transmit and re-export ([docs/flipper_key_files.md](docs/flipper_key_files.md))
- **Headless decoding** — `kat --decode <files or folders>` prints every decode of `.sub` files and IQ recordings without the TUI, one line per distinct decode (`--verbose` dumps each IQ burst's pulses) — for scripting and for checking capture collections
- **IQ recordings** — decode `.cs8` (hackrf_transfer), `.cu8` (rtl_sdr / rtl_433) and `.cf32` recordings without a radio (`:iq`, `kat --iq`, or the `:load` browser), and record the raw IQ while receiving (`:record`)
- **Frequency hopping** — `:hop` cycles Flipper's hopper list (315.00, 390.00, 430.50, 433.92, 434.25, 434.42, 868.35 MHz) and stays on a frequency while a signal is on air, like ProtoPirate
- **Session autosave** — every capture is saved to `~/.config/KAT/session.json` (owner-only) and offered for restore at the next start; `:clear` wipes it ([docs/session.md](docs/session.md))
- **Saved-fob matching** — decoded signals that match one of your `.fob` files (ProtoPirate's rules: protocol, serial or key, counter within ±50) get a ★ and show the saved vehicle
- **Timing analysis** — ProtoPirate's Timing Tuner for any capture, plus pulse clusters, a base-TE estimate and a line-coding guess for unknown signals ([docs/timing_analysis.md](docs/timing_analysis.md))
- **Research mode** — config option to show unknown (unidentified) signals in addition to successfully decoded ones
- **INI configuration** — `~/.config/KAT/config.ini` (auto-created with comments on first run): export path, max captures, research_mode, radio defaults, export format
- **Embedded keystore** — manufacturer keys (Kia, VAG, KeeLoq brands, etc.) built in for decoding; add your own KeeLoq manufacturer keys in a Flipper-format `~/.config/KAT/keeloq_mfcodes_user` ([docs/keeloq.md](docs/keeloq.md))
- **VIM-style command line** — `:freq`, `:hop`, `:iq`, `:record`, `:lock`, `:unlock`, `:replay`, `:txpower`, `:timing`, `:load`, `:new`, `:delete`, `:clear`, `:q` / `:quit`, and more
- **Interactive TUI** — captures list with detail panel (protocol, freq, mod, RF, encryption), signal action menu, radio settings, fob export form; header shows device (HackRF / RTL-SDR (RX only) / No device), status (DISCONNECTED in red when no device), hopping and recording state

## Requirements

- **Radio hardware (one of):**
  - **HackRF One** (or compatible) — full receive and transmit
  - **RTL433 / RTL-SDR** — receive-only; no transmit (RTL433 dongles and compatible RTL-SDR hardware)
- **Rust 1.75+** (for building from source)
- **libhackrf** — HackRF C library and headers (required at build time even when using RTL-SDR)
- **libusb** — for RTL433/RTL-SDR (usually provided by OS)

At runtime, KAT tries **HackRF first**; if none is found, it uses the first available **RTL433/RTL-SDR** for receive-only. If neither is connected, it runs without TX/RX support (no live capture); a startup warning offers to continue or you can connect a device and restart.

### Installing Dependencies

**macOS:**

```bash
brew install hackrf
# RTL433/RTL-SDR: no extra system lib required; rtl-sdr-rs uses rusb
```

**Debian / Ubuntu:**

```bash
sudo apt install libhackrf-dev pkg-config libusb-1.0-0-dev
```

**Fedora:**

```bash
sudo dnf install hackrf-devel pkg-config libusb1-devel
```

**Arch Linux:**

```bash
sudo pacman -S hackrf libusb
```

On **Linux**, if using RTL433/RTL-SDR, you may need to unload DVB-T kernel modules so the device is not claimed by the kernel (see [rtl-sdr-rs](https://crates.io/crates/rtl-sdr-rs)).

## Building

```bash
git clone <repo-url> && cd KAT
cargo build --release
```

The binary is placed at `target/release/kat`.

## Usage

```bash
./target/release/kat                                        # start the TUI
./target/release/kat --iq fob.cs8 --freq 433.92 --rate 2M   # also decode an IQ recording at startup
./target/release/kat --help                                 # options; --version prints the version
./target/release/kat --decode IMPORTS/ fob.cu8 --rate 250k  # decode files/folders without the TUI
```

`--decode` prints `file | frequency | source | protocol | xcount | data=…/bits sn= btn= cnt= crc= tx=` per distinct decode (`source` is `raw`, `key`, or the IQ path `am`/`fm`); `--all` prints every decode in stream order, `--verbose` also dumps each IQ burst (time, length, SNR, carrier offset, FSK deviation, pulses) to stderr. Files whose name carries no rate (e.g. rtl_433 `gfile001.cu8`) need `--rate`.

`--freq` and `--rate` default to values in the recording's file name (e.g. `kat_433670000Hz_2000000sps_….cs8`), else the configured frequency and 2 MS/s. Live receive and IQ decoding need a release build (`cargo build --release`); a debug build is about 20× slower.

KAT starts in an interactive terminal UI. If no HackRF or RTL433/RTL-SDR is detected, a warning appears (“No HackRF or RTL-SDR detected”) and you can press any key to continue without TX/RX support—you can still view, import, and export captures. The header shows the active device (**HackRF**, **RTL-SDR (RX only)** when using RTL433/RTL-SDR, or **No device**) and radio status; when no device is connected, the status is shown in red (DISCONNECTED). When using RTL433/RTL-SDR, transmit actions (Replay, TX Lock/Unlock/Trunk/Panic) show “(no TX)” in the signal menu and display a receive-only message if selected.

### Keyboard Controls

| Key | Action |
|---|---|
| `j` / `k` or Arrow Up / Down | Navigate captures list |
| `Enter` | Open signal action menu on selected capture |
| `Tab` | Open radio settings menu (Frequency, Hop, LNA, VGA, AMP, TX power) |
| `r` | Toggle receive mode (start/stop RX) |
| `:` | Enter VIM-style command mode |
| `Esc` | Close menu / cancel current action |
| `q` | Quit (restores terminal) |
| `:q` or `:quit` | Quit from command line (same as `q`; restores terminal) |

### Signal Action Menu

Press `Enter` on a capture to open the action menu. When using RTL-SDR (receive-only), transmit actions show **(no TX)** and are disabled.

| Action | Description |
|---|---|
| Recover key (PSA brute force) | PSA TEA frame without a key: run ProtoPirate’s key brute force in the background (progress in the status line; **Cancel key recovery** stops it). On success the capture gets its serial/button/counter and TX actions |
| Replay | Re-transmit raw capture (HackRF only; “no TX” with RTL-SDR) |
| TX Lock | Transmit lock command |
| TX Unlock | Transmit unlock command |
| TX Trunk | Transmit trunk release command |
| TX Panic | Transmit panic alarm command |
| Send next code | Gate, garage and barrier remotes (instead of TX Lock … Panic): transmit the code after the stored one |
| Send button… | Gate and garage remotes: pick one of the remote's native buttons, or type a hex code, and send the next code with it (the list stays open) |
| Export .fob | Export signal with full vehicle + signal metadata |
| Export .sub | Export in Flipper Zero SubGHz format (with the capture’s preset) |
| Timing analysis | Compare the capture’s pulse timing with its protocol’s definition (ProtoPirate’s Timing Tuner); for unknown signals: pulse clusters, a TE estimate and a line-coding guess |
| Delete | Remove capture from the list |

### Fob Export

When exporting to `.fob`, a metadata form collects filename and optional vehicle info:

- **File** — output filename (extension added by format). For unknown protocol, a unique 8-character hex suffix (e.g. `A1B2C3D4`) is shown in the field and appended to the filename so each export has a distinct file.
- **Year** — vehicle model year
- **Make** — manufacturer (auto-suggested from protocol)
- **Model** — vehicle model
- **Region** — region/market
- **Command** — button/command label (e.g. Unlock, Lock, Trunk, Panic); used in the default filename for unknown protocol and stored in the .fob vehicle section.
- **Notes** — free-form notes

The exported `.fob` file is a versioned JSON document (v2.0, format `kat-fob`) containing:

```json
{
  "version": "2.0",
  "format": "kat-fob",
  "signal": {
    "protocol": "Kia V3/V4",
    "frequency": 433920000,
    "frequency_mhz": "433.92MHz",
    "modulation": "PWM",
    "rf_modulation": "AM/FM",
    "encryption": "KeeLoq",
    "data_bits": 64,
    "data_hex": "...",
    "serial": "0x1A2B3C",
    "key": "0x...",
    "button": 1,
    "button_name": "Lock",
    "counter": 1234,
    "crc_valid": true,
    "encoder_capable": true
  },
  "vehicle": {
    "year": 2023,
    "make": "Kia",
    "model": "Sportage",
    "region": "NA",
    "command": "Lock",
    "notes": ""
  },
  "capture": {
    "timestamp": "2026-02-07T12:00:00Z",
    "raw_data_hex": "0x...",
    "raw_pair_count": 0,
    "raw_pairs": [{"level": true, "duration_us": 400}, {"level": false, "duration_us": 800}]
  }
}
```

- **vehicle.command** — optional; user-editable command label (e.g. Unlock, Lock). Set in the export form or via **i** (capture metadata). Used for unknown-protocol export filenames (`Year_Make_Model_Region_Command_8HEX.fob`).
- **rf_modulation** — AM, FM, or AM/FM per protocol (from ProtoPirate). Omitted when unknown.
- **Optional signal fields (1.5)** — `data_extra` (encoder state for frames over 64 bits), `received_rf` (the path KAT received it on: AM or FM), `preset` with `custom_preset_module` / `custom_preset_data` (the Flipper preset the signal was recorded or will be sent with) and `tx_modulation` (informational, e.g. `2-FSK ±47.61 kHz`). Files without them still load.
- **capture** — `raw_data_hex`, `raw_pair_count`, and optionally `raw_pairs` when config `include_raw_pairs` is true.

### VIM-Style Commands

Transmit commands (`:lock`, `:unlock`, `:trunk`, `:panic`) require HackRF; with RTL-SDR they report that transmit is not available.

| Command | Description |
|---|---|
| `:freq <MHz>` | Set receive frequency (e.g. `:freq 433.92`) |
| `:hop on` / `:hop off` | Frequency hopping over the hop list; KAT stays on a frequency while a signal is on air |
| `:hop <MHz> <MHz> …` | Set the hop list (`:hop default` restores Flipper's list; `:hop dwell <ms>` sets the time per quiet frequency) |
| `:iq <file> [MHz] [rate]` | Decode a `.cs8` / `.cu8` / `.cf32` IQ recording in the background (no radio needed); `:iq stop` cancels |
| `:record [file]` / `:record stop` | Record the raw IQ as received to the export folder (`.cs8` for HackRF, `.cu8` for RTL-SDR) |
| `:lock <ID>` | Transmit lock signal (ID: single, comma list, or range; e.g. `1`, `1, 3, 5`, `1-5`) |
| `:unlock <ID>` | Transmit unlock signal (same ID formats) |
| `:trunk <ID>` | Transmit trunk release (same ID formats) |
| `:panic <ID>` | Transmit panic alarm (same ID formats) |
| `:replay <ID>` | Replay raw capture(s) by ID in order (same ID formats; HackRF only) |
| `:delete <ID>` | Delete capture from list |
| `:clear` | Clear all captures and the saved session (asks first) |
| `:txpower <0-47> [amp on/off]` | TX VGA gain and RF amp for every transmission; no argument shows the setting |
| `:timing [ID]` | Timing analysis of a capture (default: the selected one) |
| `:load <file>` | Import capture from `.fob` or `.sub` file (`:load` alone opens a file browser, which also opens IQ recordings) |
| `:new [type…]` | Create a new gate/garage remote ("Add Manually"): opens the type picker, or creates the one type the words name (e.g. `:new princeton 433`) |
| `:q` or `:quit` | Quit application (terminal restored cleanly, same as pressing `q`) |

## Configuration

On first launch, KAT creates the config directory and a default config file. Captures are autosaved to `session.json` and offered for restore at the next start; set `session_autosave = false` under `[session]` to keep them in memory only.

```
~/.config/KAT/
├── config.ini      # Application settings (auto-generated with comments)
├── session.json    # Autosaved captures (owner-only; see docs/session.md)
└── exports/        # Default export directory for .fob / .sub files
```

Example `config.ini` (all keys optional; defaults are used if missing):

```ini
[general]
export_directory = ~/.config/KAT/exports
import_directory = ~/.config/KAT/import
max_captures = 100
research_mode = false

[radio]
default_frequency = 433920000
default_lna_gain = 24
default_vga_gain = 20
default_amp = false
hop_frequencies = 315.00, 390.00, 430.50, 433.92, 434.25, 434.42, 868.35
hop_dwell_ms = 100
tx_vga_gain = 47
tx_amp = true

[export]
default_format = fob
include_raw_pairs = true

[session]
session_autosave = true
```

- **import_directory** — directory scanned at startup for .fob and .sub files to import (default `~/.config/KAT/import`). Exports are still saved to **export_directory**.
- **research_mode** — when `true` (default), unknown (unidentified) signals are shown in addition to decoded ones; when `false`, only successfully decoded signals appear.
- **include_raw_pairs** — when `true`, .fob exports include raw level/duration pairs for replay.
- **hop_frequencies / hop_dwell_ms** — the `:hop` list (MHz) and the time spent on a quiet frequency.
- **tx_vga_gain / tx_amp** — transmit power (0–47 dB TX VGA, RF amp); the defaults are the HackRF's maximum, as in KAT ≤ 1.4.
- **session_autosave** — autosave captures to `session.json` (default on); **max_captures** also caps the saved session.

## Supported Protocols

Protocol behavior and RF modulation (AM/FM) follow the ProtoPirate reference; protocols ProtoPirate does not carry are ported from [Flipper-ARF](https://github.com/D4C1-Labs/Flipper-ARF) and [doctor-firmware](https://github.com/thedoctorz/doctor-firmware). KAT receives AM/OOK and FM/2-FSK and transmits each protocol with the modulation and FSK deviation the references use ([docs/transmit.md](docs/transmit.md)). “Replay” encoders re-send the captured rolling code (optionally re-stamping the button) because no manufacturer key is available.

| Protocol | Encoding | RF | Encryption | Frequency |
|---|---|---|---|---|
| Kia V0 / Suzuki V0 / Honda V0 / Mitsubishi V0 (61-bit) | PWM | AM/FM | Rolling Code (CRC8; encoder: counter + 1) | 315 / 433.92 MHz |
| Kia V1 | Manchester | AM | Rolling Code (CRC; encoder: counter + 1) | 315 / 433.92 MHz |
| Kia V2 | Manchester | AM/FM | Rolling Code (CRC; encoder: counter + 1) | 315 / 433.92 MHz |
| Kia V3 / Kia V4 | PWM | FM | KeeLoq (KIA key check, as in the references) | 315 / 433.92 MHz |
| Kia V5 | Manchester | FM | Rolling Code (mixer; encoder) | 315 / 433.92 MHz |
| Kia V6 | Manchester | FM | AES-128 (encoder: counter + 1) | 315 / 433.92 MHz |
| Kia V7 | Manchester | FM | Rolling Code (CRC8; encoder: counter + 1) | 315 / 433.92 MHz |
| Ford V0 | Manchester | AM | Rolling Code (7-bit matrix CRC) | 315 / 433.92 MHz |
| Ford V1 | Manchester | FM | Rolling Code (descramble + CRC16) | 315 / 433.92 MHz |
| Ford V2 | Manchester | FM | Rolling Code (plaintext 16-bit counter; encoder: counter + 1) | 315 / 433.92 MHz |
| Ford V3 (EU + US) | Manchester | AM/FM | Rolling Code (decode-only) | 315 / 433.92 MHz |
| Chrysler V0 (Dodge/Jeep) | PWM | AM | Rolling Code (seed-XOR) | 315 / 433.92 MHz |
| Honda Static | Manchester | FM | Fixed Code | 315 / 433.92 MHz |
| Honda V1 | PWM | AM | Rolling Code | 315 / 433.92 MHz |
| Toyota (Lexus) | Bi-phase (diff. Manchester) | AM | Rolling Code (7-bit counter masks; encoder) | 315 / 433.92 MHz |
| Honda V2 (formerly "Land Rover V0") | Diff. Manchester | FM | Rolling Code (3-bit check) | 315 / 433.92 MHz |
| Fiat V0 | Manchester | AM | Rolling Code (replay) | 433.92 MHz |
| Fiat V1 (Magneti Marelli BSI) | Manchester | AM | HITAG2 (PP 9 keys + ARF dictionary/heuristics → fresh frame; replay otherwise) | 315 / 433.92 MHz |
| Fiat V2 (FCA: Fiat/Chrysler/Jeep) | Manchester | AM | Rolling Code (HITAG2 forward-encode with a known key, else replay) | 315 / 433.92 MHz |
| Renault V0 | Manchester | AM | Rolling Code | 315 / 433.92 / 868.35 MHz |
| Renault V1 | Manchester | AM/FM | HITAG2 (serial-derived seed auto-recovered → fresh frame; replay otherwise) | 315 / 433.92 / 868.35 MHz |
| Renault V1 (ARF) — Flipper-ARF type table 0x04/0x0C/0x1A/0x3B/0x3F + Dynamic | Manchester | AM/FM | Rolling Code (replay only) | 315 / 433.92 / 868.35 MHz |
| Mazda V0 | Manchester | AM/FM | XOR whitening (encoder) | 315 / 433.92 MHz |
| Mazda Siemens | Manchester | FM | Siemens XOR/interleave | 315 / 433.92 MHz |
| Mazda V2 | Diff. Manchester | AM/FM | Rolling Code (table-driven next code) | 315 / 433.92 MHz |
| Mitsubishi V0 (ARF 96-bit) | PWM | FM | Bit Negation + XOR (replay encoder) | 315 / 433.92 / 868.35 MHz |
| BMW | Manchester | AM | Rolling Code (replay) | 433.92 MHz |
| Mercedes V0 | Manchester | FM | Rolling Code (replay) | 433.92 MHz |
| Mercedes V1 | Manchester | FM | Rolling Code (replay) | 433.92 MHz |
| Opel V1 | Manchester | AM | XTEA (decode-only) | 433.92 MHz |
| GM (Chevrolet/Buick/GMC/Cadillac) | PPM | AM | Rolling Code (replay) | 315 / 433.92 MHz |
| Datsun Lada | Manchester | FM | HITAG2-family | 433.92 MHz |
| Porsche Cayenne (ARF “Porsche AG”) | PWM | AM | VAG rolling register (encoder) | 315 / 433.92 / 868.35 MHz |
| Subaru | PWM | AM | Rolling Code (encoder: counter + 1) | 315 / 433.92 MHz |
| VAG (VW/Audi/Seat/Skoda) | Manchester | AM | AUT64/XTEA (encoder: counter + 1, all types) | 315 / 433.92 / 434.42 MHz |
| Scher-Khan (Magic Code, PRO1/PRO2) | PWM | AM/FM | Magic Code / PRO cipher + CRC (encoder) | 315 / 433.92 MHz |
| Star Line | PWM | AM | KeeLoq (whole keystore searched; encoder: counter + 1) | 315 / 433.92 MHz |
| Sheriff CFM (ZX-750/930 alarm) | PWM | AM | KeeLoq | 315 / 433.92 MHz |
| KeeLoq (gates, garage doors, alarms) | PWM | AM | KeeLoq (keystore + your `keeloq_mfcodes_user`; per-key learning type as in Flipper-ARF; unidentified remotes listed as `KeeLoq (Unknown)`, replay only) | any band (300–868 MHz) |
| PSA (Peugeot/Citroën) | Manchester | AM/FM | TEA/XOR (TEA key via the “Recover key” brute force; encoder: counter + 1) | 315 / 433.92 MHz |

### Garage door, gate, barrier and shutter remotes

Ported from Flipper-ARF's garage app (`applications/system/subghz_garage`, its newest copies), with doctor-firmware where the app lacks a protocol or doctor can do more; every decoder matches the reference C compiled on a PC (field for field, on Flipper's unit-test captures, real recordings and fuzzed streams) and every encoder its upload pulse for pulse. They run on any frequency, like the Flipper. Their transmit actions are **Replay**, **Send next code** (counter + 1 where the protocol rolls) and **Send button…** (any of the remote's native buttons, or a typed code). `:new` creates a new remote of any of Flipper-ARF's and doctor-firmware's 101 types to pair with a receiver (**Add Manually**, [docs/add_manually.md](docs/add_manually.md)). Where Flipper-ARF's garage-app TX plugins are built without their counter/button helpers (they always send button 0 or jump the counter), KAT follows the source's intended behaviour, as doctor-firmware does.

| Protocol | Encoding | RF | Code | Bands (reference flags) | TX |
|---|---|---|---|---|---|
| Security+ 1.0 / Security+ 2.0 (Chamberlain/LiftMaster) | PWM ternary / Manchester | AM | Rolling Code | 315 / 390 / 433.92 MHz | next code |
| Somfy Telis (RTS) / Somfy Keytis | Manchester | AM | Rolling Code (obfuscation + checksum) | 433.42 / 868 MHz | next code, any button |
| FAAC SLH | PWM | AM | KeeLoq (FAAC learning, seed) | 433.92 / 868 MHz | next code with the seed; programming frames |
| KingGates Stylo4k | PWM | AM | KeeLoq | 433.92 MHz | next code |
| Jarolift (shutters) | PWM | AM | KeeLoq (normal learning) | 433.92 MHz | next code |
| Beninca ARC | PWM | AM | AES-128 | 433.92 MHz | next code |
| Nice FloR-S / Nice One (+ doctor's Nice O-Code keys) | PWM | AM | Rolling Code (needs `~/.config/KAT/nice_flor_s.bin`) | 433.92 / 868 MHz | next code with the table |
| CAME Atomo | Manchester | AM | Rolling Code | 433.92 / 868 MHz | next code |
| Alutech AT-4N | PWM | AM | TEA-style (needs `~/.config/KAT/alutech_at_4n.bin`) | 433.92 MHz | next code with the table |
| Ditec GOL4 | PWM | AM | Rolling Code | 433.92 MHz | next code |
| Phoenix V2, Hay21, Prastel | PWM | AM | Rolling Code | 433.92 MHz (Prastel also 315) | next code |
| iDo 117/111 | PWM | AM | Rolling Code | 433.92 MHz | decode only (as in the C) |
| CAME (12/18/24/25/42-bit) / CAME TWEE | PWM / Manchester | AM | Fixed Code | 315 / 433.92 / 868 MHz | yes |
| Nice FLO (12/24-bit) | PWM | AM | Fixed Code | 315 / 433.92 MHz | yes |
| Princeton (PT2262/EV1527), SMC5326, Holtek, Holtek HT12X | PWM | AM (HT12X AM/FM) | Fixed Code | 315 / 433.92 / 868 MHz | yes |
| Linear, Linear Delta3, MegaCode, Chamberlain Code | PWM | AM | Fixed Code | 300–390 MHz | yes |
| Hörmann HSM, Marantec, Marantec24 | PWM / Manchester | AM | Fixed Code (Marantec CRC-8) | 433.92 / 868 MHz | yes |
| GateTX, BETT, Doitrand, Clemsa, Mastercode, Ansonic, Dickert MAHS, Elplast, Roger | PWM | AM (Ansonic AM/FM) | Fixed Code | 315 / 433.92 / 868 MHz | yes |
| Nero Sketch, Nero Radio, Revers RB2, Telcoma/Cardin EDGE, Allstar Firefly | PWM / Manchester | AM | Fixed Code | 315 / 433.92 MHz | yes |
| Power Smart, Dooya (shutters/blinds) | Manchester / PWM | AM | Fixed Code | 315 / 433.92 MHz | yes |
| Urmet | PWM | AM | Fixed Code | 433.92 MHz | replay only (its frame does not fit KAT's capture format) |

Each protocol has its own page in [docs/](docs/README.md). Not ported (in the references but not remotes for gates, doors or shutters): doorbells, alarm sensors, lights, key finders, X10, treadmill and freezer remotes.

**Modulation gate:** like Flipper-ARF, decoders are only fed captures of their own modulation — a `.sub` file's preset decides (AM650 → AM protocols, FM476 → FM protocols); KAT's own receiver tries the protocols of the burst's path (AM or FM) first and the rest only when none decodes.

**KeeLoq generic fallback:** If no protocol decodes a capture, KAT tries KeeLoq with every keystore manufacturer key (Kia V3/V4 and Star Line bit layouts). On success the protocol is shown as **Keeloq (*keystore name*)** (e.g. Keeloq (Alligator), Keeloq (Pandora_PRO)). See [docs/keeloq_generic.md](docs/keeloq_generic.md).

**Not registered (mirrors Flipper-ARF):** **Audi**, **Mazda Infinity** and **Hyundai** (ARF “Hundai”) — reverse-engineered by ARF from Pandora alarm firmware and shipped disabled — and **Land Rover RKE** (an ARF helper that ARF never registers) are ported and tested but kept out of KAT’s registry, as in ARF. Mazda Infinity’s air format is decoded by Mazda V0 anyway; Hyundai’s frame has no checksum. Each is a one-line change in `ProtocolRegistry::new()` to enable (see their docs).

**Retired (mirrors the references):** **BMW CAS4** — Flipper-ARF disabled `bmw_cas4.c` and replaced it with **BMW**; **Porsche Touareg** — ProtoPirate removed `porsche_touareg.c`, a port of the same `porsche_cayenne.c` that KAT keeps as **Porsche Cayenne** (old “Porsche Touareg” captures resolve to it); the **KeeLoq Toyota** decoder — Flipper-ARF replaced it with the current Toyota/Lexus protocol above; **PSA2** (“PSA OLD”) — Flipper-ARF never registers `psa2.c`, and PSA now decodes those frames with the same fields (old “PSA2” captures resolve to PSA); **Suzuki** — ProtoPirate folded `suzuki.c` into Kia V0 (now the “Suzuki V0” frame type; old “Suzuki” captures resolve to Kia V0).

### Vulnerability database

KAT matches capture metadata (Year / Make / Model / Region, set via **i** on a capture) against a built-in CVE list. The **Vuln Found** column and **Vulnerability** detail panel show matches; each CVE links to the NVD for further reading.

| CVE | Make(s) | Models | Year range | Description |
|-----|---------|--------|------------|-------------|
| [CVE-2022-38766](https://nvd.nist.gov/vuln/detail/CVE-2022-38766) | Renault | ZOE | 2020–2022 | Replay attack: same rolling code set per door-open request (433.92 MHz). |
| [CVE-2022-27254](https://nvd.nist.gov/vuln/detail/CVE-2022-27254) | Honda | Civic | 2016–2019 | Replay attack: same RF signal per door-open request (related to CVE-2019-20626). |
| [CVE-2019-20626](https://nvd.nist.gov/vuln/detail/CVE-2019-20626) | Honda, Acura | HR-V, Civic, Accord; TSX | 2009, 2016–2020 (by model) | Replay attack: same RF signal per door-open request (static code, no rolling code). Confirmed vehicles per [Unoriginal-Rice-Patty](https://github.com/HackingIntoYourHeart/Unoriginal-Rice-Patty). |
| [CVE-2022-37418](https://nvd.nist.gov/vuln/detail/CVE-2022-37418) | Honda, Hyundai, Kia, Nissan | Fit (hybrid), Fit, City, Vezel; Elantra; Cerato, Forte, K3; Latio, Sylphy | 2007–2022 (varies by model) | **RollBack attack:** RKE allows unlock and resync after capturing two consecutive key fob signals; attacker can unlock indefinitely. |
| [CVE-2022-36945](https://nvd.nist.gov/vuln/detail/CVE-2022-36945) | Mazda | 3, 2 Sedan, 2 HB (facelift), Cx-3, Cx-5 | 2018–2020 (varies by model) | **RollBack attack:** RKE allows unlock and resync after capturing **three** consecutive key fob signals; attacker can unlock indefinitely. |

Details (exact model/year scope) are in `src/vuln_db.rs`. The app shows the NVD URL for each matched CVE in the Vulnerability panel.

### Cryptographic modules

- **KeeLoq** — encrypt/decrypt with normal, secure, FAAC, and magic serial/XOR learning key derivation (keeloq_common, keys). Unknown signals are tried as KeeLoq with every keystore key via **keeloq_generic** (uses keeloq_common only).
- **AES-128** — full encrypt/decrypt for Kia V6 (key derived from keystore A/B with XOR masks)
- **TEA / XOR** — PSA (Peugeot/Citroën): XOR fast path on decode; TEA frames are recovered on demand with ProtoPirate’s key brute force (two 2^24 ranges, multithreaded, ≈0.5 s worst case) from the signal menu’s **Recover key** action
- **AUT64** — 12-round block cipher for VAG type 1/3/4 (aut64)
- **HITAG2** — keystream/authenticator shared by Fiat V1 (hop verified against ProtoPirate’s 9 keys plus Flipper-ARF’s dictionary/heuristics → fresh frame) and Renault V1 (ProtoPirate’s 2^18 serial-derived seed search, ~10 ms, runs on every frame → fresh frame); without a key/seed both replay the captured frame, as the references do (hitag2)
- **XTEA** — Opel V1 frames decrypted with the published Bosch key (opel_v1)
- **Keystore** — manufacturer keys (Kia, VAG, etc.) built in; see `src/keystore/`

### Receiver

One receive pipeline serves the HackRF, the RTL-SDR and IQ recordings ([docs/radio.md](docs/radio.md)):

- **Offset tuning** — the LO sits fs/8 below the target (250 kHz at 2 MS/s), away from the HackRF/RTL-SDR DC spike; captures always carry the target frequency
- **Wide-band detection** — an FFT detector (31 kHz bins, a noise floor per bin) watches the whole receive band: at 2 MS/s a fob is found from 1.15 MHz below to 0.65 MHz above the target, which covers a Flipper's ±325 kHz OOK window. Each signal is its own burst, so two fobs on different frequencies at once give two captures; a strong fob's I/Q image and spurs are dropped. Files recorded at the fob frequency are searched over their whole band
- **Channel filter** — each burst's carrier is mixed to 0 Hz and filtered to ~250 kS/s, passband ±100 kHz (FSK tones and drift)
- **Squelch** — bursts open and close on carrier presence in their own bins (no captures from noise; a burst closes after 80 ms without carrier)
- **AM and FM per burst** — OOK is sliced between noise and carrier level; FSK finds both tones per burst (automatic frequency correction), with hysteresis at 10 % of the tone separation; each frame is then re-sliced with smoothing matched to its element length
- **Measured on synthetic signals** — OOK and 2FSK fobs decode equally well anywhere from 325 kHz below to 325 kHz above the target: half the presses decode at about −8.5 dB SNR for OOK and −6 dB for FSK (±20 and ±47.6 kHz; SNR over 2 MHz); KAT 1.4 needed 12–18 dB for OOK and decoded no FSK once there was noise
- **Real recordings** — rtl_433's test recordings decode as before, plus six HCS200 KeeLoq presses whose carrier is ~120 kHz below the file's frequency

## IMPORTS folder

The `IMPORTS/` directory holds sample `.sub` captures by manufacturer for testing. If you see `desktop.ini` files in subfolders (e.g. from Windows), you can delete them; they are ignored by git (see `.gitignore`).

## Project Structure

```
src/
├── main.rs              # Entry point, event loop, key handling
├── app.rs               # Application state, radio events, signal actions
├── app/                 # rx.rs (:iq, :record, :hop), tx.rs (transmit queue, :txpower), persistence.rs (session),
│                        # fob_matching.rs (saved-fob ★), timing_view.rs (timing overlay), key_files.rs (Flipper key
│                        # files), add_manually.rs (:new, Send button…)
├── headless.rs          # kat --decode: decode .sub files and IQ recordings without the TUI
├── analysis/timing.rs   # Timing analysis (ProtoPirate Timing Tuner, pulse clusters, line-coding guess)
├── session.rs           # session.json format and atomic writer
├── saved_fobs.rs        # Index of saved .fob files for matching
├── capture.rs           # Capture data (data_extra for protocol encode state), encoding/RF modulation
├── storage.rs           # Config (INI), export dir, resolve_config_dir, Storage
├── keystore/
│   ├── mod.rs           # Keystore trait and access
│   └── embedded.rs      # Embedded manufacturer keys (Kia, VAG, etc.)
├── export/
│   ├── fob.rs           # .fob JSON export/import (v1 + v2, rf_modulation)
│   ├── flipper.rs       # Flipper Zero .sub export/import (RAW, key files)
│   └── binraw.rs        # Flipper BinRAW captures -> level/duration pairs
├── protocols/
│   ├── mod.rs           # Protocol registry, decoder trait, duration_diff macro
│   ├── info.rs          # Protocol metadata (coding, RF, encryption, make, buttons, TX presets) from each module's INFO
│   ├── common.rs        # DecodedSignal (extra for VAG encode), CRC, bit helpers, button codes
│   ├── keeloq_common.rs # KeeLoq cipher + learning key algorithms (used by Kia V3/V4, Star Line, keeloq_generic)
│   ├── keeloq_generic.rs # KeeLoq fallback: try every keystore key when no protocol matches (uses keeloq_common)
│   ├── aut64.rs         # AUT64 block cipher (VAG)
│   ├── keys.rs          # Key loading (embedded + optional file), KIA/VAG
│   ├── kia_v0..kia_v7.rs
│   ├── ford_v0.rs, fiat_v0.rs, fiat_v1.rs, subaru.rs
│   ├── mazda_v0.rs, mitsubishi_v0.rs, porsche_cayenne.rs
│   ├── vag.rs           # VAG decoder/encoder (4 sub-types; encode from capture via extra)
│   ├── scher_khan.rs, star_line.rs, psa.rs
│   ├── renault_v0.rs, renault_v1.rs, fiat_v2.rs, hitag2.rs   # HITAG2 shared by Fiat V1 / Renault V1
│   ├── bmw.rs, mercedes_v0.rs, mercedes_v1.rs, opel_v1.rs, gm.rs, mazda_v2.rs, datsun_lada.rs
│   ├── sheriff_cfm.rs, audi.rs, mazda_infinity.rs, hyundai.rs   # hyundai: not registered
│   ├── princeton.rs, came.rs, secplus_v2.rs, somfy_telis.rs, faac_slh.rs, …   # 47 gate/garage protocols
│   ├── g1_common.rs … g6_common.rs   # helpers shared by the gate protocols
│   ├── flipper_key.rs   # Flipper key-file (.sub Protocol/Bit/Key) reader and writer
│   ├── add_manually.rs  # "Add Manually" type list and new-remote generation (ARF / doctor-firmware)
│   ├── gate_buttons.rs  # Native button lists for "Send button…"
│   └── ...
├── radio/
│   ├── hackrf.rs        # HackRF device control (RX/TX)
│   ├── rtlsdr.rs        # RTL-SDR device control (RX only)
│   ├── dsp.rs           # IQ formats, DC removal, mixer, decimating FIR filters
│   ├── pipeline.rs      # Receive pipeline: wide-band detector, per-burst channel filter, AM/FM slicing -> bursts
│   ├── rx.rs            # Live receive glue (pipeline + hopper + recorder -> captures)
│   ├── hopper.rs, iq_file.rs, recorder.rs   # Frequency hopping, IQ file input, IQ recording
│   ├── preset.rs        # CC1101 presets (.sub Preset / Custom_preset_data) -> TX modulation
│   ├── tx.rs            # TX waveform (OOK / continuous-phase 2-FSK on a digital offset), TX stream
│   ├── sim.rs           # Synthetic-signal tests and sensitivity sweeps (test only)
│   ├── demodulator.rs   # KAT 1.4's demodulators (comparison baseline in the sweeps)
│   └── modulator.rs    # Level/duration -> TX waveform helpers
└── ui/
    ├── layout.rs        # TUI layout, fob metadata form
    ├── captures_list.rs # Captures table + detail (protocol, mod, RF, enc)
    ├── signal_menu.rs   # Signal action menu
    ├── settings_menu.rs # Radio settings (Freq, Hop, LNA, VGA, AMP, TX)
    ├── app_prompts.rs   # Session restore / clear prompts
    ├── command.rs       # VIM-style command line
    ├── add_manually.rs  # :new type picker and field editor, Send button… list
    └── status_bar.rs    # Status bar
```

## Call for researchers

**We need your help.** KAT’s protocol decoders and future analysis depend on real-world keyfob captures. If you use KAT for security research, authorized testing, or protocol development, please consider:

- **Contributing to protocol analysis** — share timing, encoding, or decoder feedback; report bugs or suggest new protocols.
- **Contributing captures to the research library** — add your `.fob` files to the **[FOB Research Library](https://github.com/KaraZajac/FOB_Research_Library)** so others can use them for decoder development, CVE validation, and protocol studies.

The [FOB Research Library](https://github.com/KaraZajac/FOB_Research_Library) is a community collection of KAT `.fob` captures organized by manufacturer. Your submissions (with proper metadata: year, make, model, region, command) help grow test vectors and support the wider keyfob research community.

---

## Credits

KAT is developed by **Kara Zajac (.kara)**. KAT would not be possible without [ProtoPirate](https://protopirate.net/ProtoPirate/ProtoPirate)—the protocol decoders, reference implementations, and community work are the foundation this tool is built on. Protocols that ProtoPirate does not carry are ported from [Flipper-ARF](https://github.com/D4C1-Labs/Flipper-ARF) and [The Doctor’s firmware](https://github.com/thedoctorz/doctor-firmware). I am truly standing on the shoulders of giants.

---

## License

[BSD-3-Clause NO MILITARY NO GOVERNMENT](LICENSE)
