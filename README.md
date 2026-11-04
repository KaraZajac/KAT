# KAT — Keyfob Analysis Toolkit

A terminal-based RF signal analysis tool for capturing, decoding, and retransmitting automotive keyfob signals. Built in Rust with a real-time TUI powered by `ratatui`. Protocol decoders are aligned with the [ProtoPirate](REFERENCES/ProtoPirate/) reference.

**Supported hardware:** KAT uses **HackRF One** (or compatible) when present for full receive and transmit. For **receive-only** operation, **RTL433** (or any compatible RTL-SDR dongle) is supported: if no HackRF is found, KAT uses the first available RTL-SDR for capture and decode; transmit (Lock/Unlock/Trunk/Panic, Replay) is disabled when using RTL433/RTL-SDR.

![Keyfob Analysis Toolkit screenshot](images/kat-screenshot.png)

---

## Security and legal disclaimer

**Use KAT only on systems and vehicles you own or have explicit, written permission to test.** Capturing, decoding, or transmitting keyfob and vehicle-access signals without authorization may be illegal in your jurisdiction (e.g. computer misuse, unauthorized access, or radio regulations). You are solely responsible for ensuring your use complies with all applicable laws. KAT is intended for security research, authorized penetration testing, education, and legitimate testing on your own equipment. The authors and contributors assume no liability for misuse or damage arising from use of this software.

---

## Features

- **Real-time capture** — receive and demodulate AM/OOK keyfob signals at configurable frequencies (HackRF uses AM envelope detection; FM protocols are tagged for display and may decode when signal is strong)
- **Multi-protocol decoding** — 40 protocol decoders: Kia V0–V7 (Kia V0 also decodes Suzuki V0, Honda V0 and Mitsubishi V0), Ford V0–V3 (V3 with EU + US variants), Chrysler V0 (Dodge/Jeep), GM (Chevrolet/Buick/GMC/Cadillac), Honda Static, Honda V1, Honda V2, Toyota (Lexus), Fiat V0/V1/V2, Renault V0/V1 (+ Flipper-ARF’s type-table Renault V1), Mazda V0, Mazda V2, Mazda Siemens, Mitsubishi V0, BMW, Mercedes V0/V1, Opel V1, Datsun Lada, Porsche Cayenne, Subaru, VAG (VW/Audi/Seat/Skoda), PSA, Scher-Khan, Star Line, Sheriff CFM, KeeLoq (gates/garage doors/alarms); adaptive demodulation for real-world conditions
- **KeeLoq generic fallback** — when a signal doesn’t match any known protocol, KAT tries decoding it as KeeLoq using every manufacturer key in the embedded keystore (Kia V3/V4 and Star Line air formats); successful decodes appear as **Keeloq (keystore name)** in the capture list
- **RF modulation metadata** — each protocol tagged as AM, FM, or both (from ProtoPirate); shown in signal detail and exported in .fob
- **Rich signal detail** — encoding (PWM/Manchester), RF (AM/FM), encryption, serial, counter, key data, CRC, frequency, and raw level/duration pairs
- **Signal retransmission** — transmit Lock, Unlock, Trunk, and Panic commands from decoded captures, or Replay raw capture (HackRF only; when using RTL-SDR these show “(no TX)” and a receive-only message). Rolling-code protocols re-encode the next code with the requested button wherever the references do (e.g. VAG all types, Subaru, Chrysler V0, Kia, Ford, Mazda, Renault V0, Sheriff CFM); protocols without an available key re-send the captured frame, as the references do.
- **Export formats** — `.fob` (versioned JSON with vehicle metadata, signal info, optional raw pairs) and `.sub` (Flipper Zero compatible)
- **Import support** — load `.fob` files with automatic v1/v2 format detection
- **Research mode** — config option to show unknown (unidentified) signals in addition to successfully decoded ones
- **INI configuration** — `~/.config/KAT/config.ini` (auto-created with comments on first run): export path, max captures, research_mode, radio defaults, export format
- **Embedded keystore** — manufacturer keys (Kia, VAG, etc.) built in for decoding
- **VIM-style command line** — `:freq`, `:lock`, `:unlock`, `:replay`, `:save`, `:load`, `:delete`, `:q` / `:quit`, and more
- **Interactive TUI** — captures list with detail panel (protocol, freq, mod, RF, encryption), signal action menu, radio settings, fob export form; header shows device (HackRF / RTL-SDR (RX only) / No device) and status (DISCONNECTED in red when no device)

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
./target/release/kat
```

KAT starts in an interactive terminal UI. If no HackRF or RTL433/RTL-SDR is detected, a warning appears (“No HackRF or RTL-SDR detected”) and you can press any key to continue without TX/RX support—you can still view, import, and export captures. The header shows the active device (**HackRF**, **RTL-SDR (RX only)** when using RTL433/RTL-SDR, or **No device**) and radio status; when no device is connected, the status is shown in red (DISCONNECTED). When using RTL433/RTL-SDR, transmit actions (Replay, TX Lock/Unlock/Trunk/Panic) show “(no TX)” in the signal menu and display a receive-only message if selected.

### Keyboard Controls

| Key | Action |
|---|---|
| `j` / `k` or Arrow Up / Down | Navigate captures list |
| `Enter` | Open signal action menu on selected capture |
| `Tab` | Open radio settings menu (Frequency, LNA, VGA, AMP) |
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
| Export .fob | Export signal with full vehicle + signal metadata |
| Export .sub | Export in Flipper Zero SubGHz format |
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
- **capture** — `raw_data_hex`, `raw_pair_count`, and optionally `raw_pairs` when config `include_raw_pairs` is true.

### VIM-Style Commands

Transmit commands (`:lock`, `:unlock`, `:trunk`, `:panic`) require HackRF; with RTL-SDR they report that transmit is not available.

| Command | Description |
|---|---|
| `:freq <MHz>` | Set receive frequency (e.g. `:freq 433.92`) |
| `:lock <ID>` | Transmit lock signal (ID: single, comma list, or range; e.g. `1`, `1, 3, 5`, `1-5`) |
| `:unlock <ID>` | Transmit unlock signal (same ID formats) |
| `:trunk <ID>` | Transmit trunk release (same ID formats) |
| `:panic <ID>` | Transmit panic alarm (same ID formats) |
| `:replay <ID>` | Replay raw capture(s) by ID in order (same ID formats; HackRF only) |
| `:save <ID>` | Save capture to file |
| `:delete <ID>` | Delete capture from list |
| `:load <file>` | Import capture from `.fob` or `.sub` file |
| `:q` or `:quit` | Quit application (terminal restored cleanly, same as pressing `q`) |

## Configuration

On first launch, KAT creates the config directory and a default config file. Captures are **in-memory only** and are not written to disk unless you export them.

```
~/.config/KAT/
├── config.ini      # Application settings (auto-generated with comments)
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

[export]
default_format = fob
include_raw_pairs = true
```

- **import_directory** — directory scanned at startup for .fob and .sub files to import (default `~/.config/KAT/import`). Exports are still saved to **export_directory**.
- **research_mode** — when `true` (default), unknown (unidentified) signals are shown in addition to decoded ones; when `false`, only successfully decoded signals appear.
- **include_raw_pairs** — when `true`, .fob exports include raw level/duration pairs for replay.

## Supported Protocols

Protocol behavior and RF modulation (AM/FM) follow the ProtoPirate reference; protocols ProtoPirate does not carry are ported from [Flipper-ARF](https://github.com/D4C1-Labs/Flipper-ARF) and [doctor-firmware](https://github.com/thedoctorz/doctor-firmware). KAT’s receiver is AM/OOK only; FM protocols may still decode when the signal produces a usable envelope. “Replay” encoders re-send the captured rolling code (optionally re-stamping the button) because no manufacturer key is available.

| Protocol | Encoding | RF | Encryption | Frequency |
|---|---|---|---|---|
| Kia V0 / Suzuki V0 / Honda V0 / Mitsubishi V0 (61-bit) | PWM | AM/FM | Rolling Code (CRC8; encoder: counter + 1) | 315 / 433.92 MHz |
| Kia V1 | Manchester | AM | Rolling Code (CRC; encoder: counter + 1) | 315 / 433.92 MHz |
| Kia V2 | Manchester | AM | Rolling Code (CRC; encoder: counter + 1) | 315 / 433.92 MHz |
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
| Honda V1 | PWM | AM | Fixed Code | 315 / 433.92 MHz |
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
| KeeLoq (gates, garage doors, alarms) | PWM | AM | KeeLoq (keystore; per-key learning type as in Flipper-ARF) | 315 / 433.92 / 868.35 MHz |
| PSA (Peugeot/Citroën) | Manchester | AM/FM | TEA/XOR (TEA key via the “Recover key” brute force; encoder: counter + 1) | 315 / 433.92 MHz |

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

### Demodulator

The **AM/OOK** demodulator turns IQ samples into level/duration pairs for protocol decoders. FM/2FSK is not demodulated; protocols are tagged AM/FM for display and export.

- **Envelope detection** — magnitude from I/Q for AM
- **Adaptive threshold** — transition-based updates to handle varying signal levels
- **Exponential moving average** — magnitude smoothing
- **Schmitt trigger hysteresis** — reduces chattering at the decision boundary
- **Debounce** — 40µs minimum pulse width to reject noise spikes
- **Gap detection** — 80 ms gap treated as end of signal (multi-burst keyfob presses stay one capture)

## IMPORTS folder

The `IMPORTS/` directory holds sample `.sub` captures by manufacturer for testing. If you see `desktop.ini` files in subfolders (e.g. from Windows), you can delete them; they are ignored by git (see `.gitignore`).

## Project Structure

```
src/
├── main.rs              # Entry point, event loop, key handling
├── app.rs               # Application state, radio events, signal actions
├── capture.rs           # Capture data (data_extra for protocol encode state), encoding/RF modulation
├── storage.rs           # Config (INI), export dir, resolve_config_dir, Storage
├── keystore/
│   ├── mod.rs           # Keystore trait and access
│   └── embedded.rs      # Embedded manufacturer keys (Kia, VAG, etc.)
├── export/
│   ├── fob.rs           # .fob JSON export/import (v1 + v2, rf_modulation)
│   └── flipper.rs      # Flipper Zero .sub export
├── protocols/
│   ├── mod.rs           # Protocol registry, decoder trait, duration_diff macro
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
│   └── ...
├── radio/
│   ├── hackrf.rs        # HackRF device control (RX/TX)
│   ├── rtlsdr.rs        # RTL-SDR device control (RX only)
│   ├── demodulator.rs   # AM/OOK demodulator (IQ -> level/duration)
│   └── modulator.rs    # Level/duration -> TX waveform
└── ui/
    ├── layout.rs        # TUI layout, fob metadata form
    ├── captures_list.rs # Captures table + detail (protocol, mod, RF, enc)
    ├── signal_menu.rs   # Signal action menu
    ├── settings_menu.rs # Radio settings (Freq, LNA, VGA, AMP)
    ├── command.rs       # VIM-style command line
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
