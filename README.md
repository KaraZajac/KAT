# KAT — Keyfob Analysis Toolkit

A terminal-based RF signal analysis tool for capturing, decoding, and retransmitting automotive keyfob signals using HackRF One. Built in Rust with a real-time TUI powered by `ratatui`.

---

## Features

- **Real-time capture** — receive and demodulate AM/OOK keyfob signals at configurable frequencies
- **Multi-protocol decoding** — 14 protocol decoders covering Kia, Ford, Fiat, Subaru, Suzuki, VAG (VW/Audi/Seat/Skoda), PSA, Scher-Khan, and Star Line with adaptive demodulation for real-world signal conditions
- **Rich signal detail** — modulation type, encryption method, serial, counter, key data, CRC, frequency, and raw level/duration pairs
- **Signal retransmission** — transmit Lock, Unlock, Trunk, and Panic commands from decoded captures
- **Export formats** — `.fob` (rich JSON with vehicle metadata, signal info, and capture data) and `.sub` (Flipper Zero compatible)
- **Import support** — load `.fob` files with automatic v1/v2 format detection
- **Persistent storage** — automatic capture saving to `~/.config/kat/captures/`
- **INI configuration** — human-readable config at `~/.config/kat/config.ini` (auto-created with comments on first run)
- **VIM-style command line** — `:freq`, `:lock`, `:unlock`, `:save`, `:load`, `:delete`, and more
- **Interactive TUI** — captures list with detail panel, signal action menu, radio settings menu, and fob export form

## Requirements

- **HackRF One** (or compatible SDR)
- **Rust 1.75+** (for building from source)
- **libhackrf** — HackRF C library and headers

### Installing Dependencies

**macOS:**

```bash
brew install hackrf
```

**Debian / Ubuntu:**

```bash
sudo apt install libhackrf-dev pkg-config
```

**Fedora:**

```bash
sudo dnf install hackrf-devel pkg-config
```

**Arch Linux:**

```bash
sudo pacman -S hackrf
```

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

KAT starts in an interactive terminal UI. If a HackRF device is not connected, the application runs in demo/offline mode so you can still view, import, and export captures.

### Keyboard Controls

| Key | Action |
|---|---|
| `j` / `k` or Arrow Up / Down | Navigate captures list |
| `Enter` | Open signal action menu on selected capture |
| `Tab` | Open radio settings menu (Frequency, LNA, VGA, AMP) |
| `r` | Toggle receive mode (start/stop RX) |
| `:` | Enter VIM-style command mode |
| `Esc` | Close menu / cancel current action |
| `q` | Quit |

### Signal Action Menu

Press `Enter` on a capture to open the action menu:

| Action | Description |
|---|---|
| TX Lock | Transmit lock command |
| TX Unlock | Transmit unlock command |
| TX Trunk | Transmit trunk release command |
| TX Panic | Transmit panic alarm command |
| Export .fob | Export signal with full vehicle + signal metadata |
| Export .sub | Export in Flipper Zero SubGHz format |
| Delete | Remove capture from the list |

### Fob Export

When exporting to `.fob`, a 6-step metadata form collects:

1. **Year** — vehicle model year
2. **Make** — manufacturer (auto-suggested from protocol)
3. **Model** — vehicle model
4. **Color** — vehicle color
5. **Trim** — trim level / package
6. **Notes** — free-form notes

The exported `.fob` file is a versioned JSON document (v2.0) containing:

```json
{
  "version": "2.0",
  "format": "KAT Fob Signal",
  "signal": {
    "protocol": "Kia V3/V4",
    "modulation": "PWM",
    "encryption": "KeeLoq",
    "frequency_mhz": 433.92,
    "serial": "0x1A2B3C",
    "key": "0xDEADBEEF...",
    "button": 1,
    "counter": 1234,
    "encoder_capable": true
  },
  "vehicle": {
    "year": "2023",
    "make": "Kia",
    "model": "Sportage",
    "color": "White",
    "trim": "EX",
    "notes": ""
  },
  "capture": {
    "timestamp": "2026-02-07T12:00:00Z",
    "raw_pairs": [[true, 400], [false, 800]]
  }
}
```

### VIM-Style Commands

| Command | Description |
|---|---|
| `:freq <MHz>` | Set receive frequency (e.g. `:freq 433.92`) |
| `:lock <ID>` | Transmit lock signal for capture ID |
| `:unlock <ID>` | Transmit unlock signal for capture ID |
| `:trunk <ID>` | Transmit trunk release signal |
| `:panic <ID>` | Transmit panic alarm signal |
| `:save <ID>` | Save capture to file |
| `:delete <ID>` | Delete capture from list |
| `:load <file>` | Import capture from `.fob` or `.sub` file |
| `:q` | Quit application |

## Configuration

On first launch, KAT creates the following directory structure:

```
~/.config/kat/
├── config.ini      # Application settings (auto-generated with comments)
├── captures/       # Persistent capture storage
└── exports/        # Default export directory for .fob / .sub files
```

The `config.ini` file is a commented INI file with the following settings:

```ini
[radio]
frequency = 433920000       # Default receive frequency in Hz
lna_gain = 32               # LNA gain (0-40 dB, step 8)
vga_gain = 40               # VGA gain (0-62 dB, step 2)
amp_enable = true           # RF amplifier on/off

[storage]
export_directory = ~/.config/kat/exports   # Where .fob/.sub files are saved
```

## Supported Protocols

| Protocol | Encoding | Encryption | Frequency |
|---|---|---|---|
| Kia V0 | PWM | Fixed Code | 433.92 MHz |
| Kia V1 | Manchester | Rolling Code | 433.92 MHz |
| Kia V2 | PWM | Rolling Code | 433.92 MHz |
| Kia V3/V4 | PWM | KeeLoq | 433.92 MHz |
| Kia V5 | PWM | Custom Mixer | 433.92 MHz |
| Kia V6 | PWM | AES-128 | 433.92 MHz |
| Ford V0 | Manchester | Fixed Code | 315 / 433.92 MHz |
| Subaru | PWM | Rolling Code | 315 / 433.92 MHz |
| Suzuki | Manchester | Rolling Code | 433.92 MHz |
| Fiat V0 | Diff. Manchester | Rolling Code | 433.92 MHz |
| VAG (VW/Audi/Seat/Skoda) | Manchester | AUT64 / TEA | 433.92 / 434.42 MHz |
| Scher-Khan | PWM | Magic Code | 433.92 MHz |
| Star Line | PWM | KeeLoq | 433.92 MHz |
| PSA (Peugeot/Citroen) | PWM | Rolling Code | 433.92 MHz |

### Cryptographic Modules

- **KeeLoq** — full encrypt/decrypt with normal, secure, FAAC, and magic serial/XOR learning key derivation
- **AUT64** — 12-round block cipher for VAG type 1/3/4 signals
- **Key Store** — global thread-safe key management for manufacturer keys (KIA, VAG)

### Demodulator

The AM/OOK demodulator uses an adaptive threshold with transition-based updates for accurate pulse detection across varying signal conditions:

- **Exponential moving average** — magnitude smoothing for stable signal tracking
- **Schmitt trigger hysteresis** — prevents noise-induced chattering at threshold crossings
- **Fast threshold convergence** — α=0.3 transition-based updates for rapid adaptation after silence periods
- **Debounce filtering** — 40µs minimum pulse width to reject noise spikes

## Project Structure

```
src/
├── main.rs              # Entry point, event loop, key handling
├── app.rs               # Application state, radio events, signal actions
├── capture.rs           # Capture data structure, modulation/encryption helpers
├── storage.rs           # Config management, capture persistence, INI read/write
├── export/
│   ├── fob.rs           # .fob JSON export/import (v1 + v2 format support)
│   └── flipper.rs       # Flipper Zero .sub export
├── protocols/
│   ├── mod.rs           # Protocol registry, decoder trait, duration_diff macro
│   ├── common.rs        # Shared CRC, bit helpers, button codes
│   ├── keeloq_common.rs # KeeLoq cipher + learning key algorithms
│   ├── aut64.rs         # AUT64 block cipher implementation
│   ├── keys.rs          # Global key store (KIA, VAG key management)
│   ├── kia_v0.rs        # Kia V0 decoder
│   ├── kia_v1.rs        # Kia V1 decoder (Manchester)
│   ├── kia_v2.rs        # Kia V2 decoder
│   ├── kia_v3_v4.rs     # Kia V3/V4 decoder (KeeLoq)
│   ├── kia_v5.rs        # Kia V5 decoder (mixer cipher)
│   ├── kia_v6.rs        # Kia V6 decoder (AES-128)
│   ├── ford_v0.rs       # Ford V0 decoder
│   ├── subaru.rs        # Subaru decoder
│   ├── suzuki.rs        # Suzuki decoder
│   ├── fiat_v0.rs       # Fiat V0 decoder (diff. Manchester)
│   ├── vag.rs           # VAG decoder/encoder (4 sub-types)
│   ├── scher_khan.rs    # Scher-Khan decoder
│   ├── star_line.rs     # Star Line decoder
│   └── psa.rs           # PSA decoder
├── radio/
│   ├── hackrf.rs        # HackRF One device control (RX/TX)
│   ├── demodulator.rs   # AM/OOK demodulator (IQ -> level/duration pairs)
│   └── modulator.rs     # Signal modulator (level/duration -> TX waveform)
└── ui/
    ├── layout.rs        # Main TUI layout, fob metadata form overlay
    ├── captures_list.rs # Captures table + signal detail panel
    ├── signal_menu.rs   # Signal action popup menu
    ├── settings_menu.rs # Radio settings popup menu
    ├── command.rs       # VIM-style command line renderer
    └── status_bar.rs    # Bottom status bar (radio state, frequency, gains)
```

## License

BSD-3-Clause
