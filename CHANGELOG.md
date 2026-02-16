# Changelog

All notable changes to KAT are documented here.

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
