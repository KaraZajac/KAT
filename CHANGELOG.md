# Changelog

All notable changes to KAT are documented here.

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
