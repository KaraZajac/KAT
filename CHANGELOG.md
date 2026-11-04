# Changelog

All notable changes to KAT are documented here.

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
