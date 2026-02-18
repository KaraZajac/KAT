# Changelog

All notable changes to KAT are documented here.

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
