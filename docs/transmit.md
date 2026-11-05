---
layout: default
---

# Transmit: OOK, 2-FSK and presets

**Rust modules:** `src/radio/tx.rs` (waveform, TX stream), `src/radio/preset.rs` (CC1101 presets),
`src/radio/hackrf.rs` (HackRF TX path), `src/protocols/info.rs` + each protocol module's `INFO` (per-protocol
presets), `src/app/tx.rs` (queue, TX power)

KAT transmits with a HackRF. Since v1.5 it keys the carrier the way the protocol (or the capture) calls for:
on-off keying for AM protocols, continuous-phase 2-FSK with the right deviation for FM protocols. Up to v1.4
every transmission was OOK at full power, so FM protocols went out as OOK.

## The waveform

A transmission is the protocol encoder's level/duration pairs (or a capture's raw pairs, for Replay) turned into
8-bit IQ at **2 MS/s, exactly 2 samples per µs** (pair boundaries are rounded from the cumulative time, so
nothing drifts).

- **OOK:** carrier on while the level is high, exact zeros while it is low.
- **2-FSK:** the carrier never switches off; **high = carrier + deviation, low = carrier − deviation**, with
  continuous phase across every bit boundary. This is the CC1101's asynchronous 2-FSK mode that the reference
  firmwares transmit with, and KAT's FM receive convention (frequency above the carrier = high). Leading and
  trailing low time is sent as the lower tone, as on a Flipper.
- **Offset TX:** the HackRF LO is tuned **500 kHz below the carrier** and the waveform is shifted up digitally.
  The HackRF leaks some LO (a CW tone at the LO frequency); with the LO on the carrier, that leakage filled every
  OOK "off" period. At 500 kHz away it is outside a key fob receiver's channel (also outside the 650 kHz-wide
  AM650 filter). 500 kHz = fs/4 keeps the shifted signal (±47.6 kHz at most) well inside the 1.75 MHz baseband
  filter KAT sets for TX, and puts the IQ-imbalance image 1 MHz below the carrier.
- **Completion:** KAT waits for the TX callback's progress, not a fixed sleep. libhackrf keeps 4 USB transfers in
  flight and calls the TX callback to refill one each time one completes; when the callback that wrote the last
  sample was call *L*, that buffer has left the host at call *L* + 4. KAT then waits 25 ms for the HackRF's own
  USB buffer (32 KB ≈ 8 ms) and stops. A stalled stream times out after the air time + 2.3 s and is reported.
- **TX power:** `tx_vga_gain` (0–47 dB) and `tx_amp` (RF amplifier, about +11 dB) in `config.ini`, the **TX**
  settings tab, or `:txpower <0-47> [amp on|off]`. The defaults (47 dB, amp on) are KAT ≤ 1.4's full power.

## Choosing the modulation

For every transmission (Lock/Unlock/…, Send next code, Replay) KAT picks a preset, and with it OOK or 2-FSK and
the deviation (`radio::preset::choose_tx_preset`):

1. **The capture's own preset.** Imported `.sub` files carry one (`Preset:`, plus `Custom_preset_module` /
   `Custom_preset_data` for a custom CC1101 preset); `.fob` files written by KAT 1.5 persist one. It is used
   whenever KAT can decode it.
2. **The receive path of a live capture** (`received_rf`, no deviation): AM → the protocol's OOK preset;
   FM → the protocol's FSK preset (its deviation). A protocol the references only know as AM keeps its OOK
   preset (an FM-path decode of an OOK burst is possible, the reverse is not); an unknown signal received as FM
   gets FM476.
3. **The protocol's primary preset** (first in the table below), or AM650 for an unknown protocol.

The status line names the choice, e.g. `Transmitted Unlock for capture 3 (2-FSK ±14.28 kHz, F4)`.

## Presets

A CC1101 preset is a register image: address/value pairs, `00 00`, then the 8-byte PA table. **MDMCFG2**
(0x12) bits 6:4 give the modulation (0 2-FSK, 1 GFSK, 3 ASK/OOK, 4 4-FSK, 7 MSK) and **DEVIATN** (0x15) the
deviation: f_dev = 26 MHz / 2^17 × (8 + m) × 2^e, m = bits 2:0, e = bits 6:4. GFSK, 4-FSK and MSK presets are
transmitted as 2-FSK with that deviation. A custom preset without MDMCFG2 counts as unknown (like ProtoPirate,
KAT does not guess); an FSK preset without DEVIATN gets the CC1101 reset value 0x47.

| Preset | Modulation | MDMCFG2 | DEVIATN | Deviation | `.sub` name | Source |
|--------|------------|---------|---------|-----------|-------------|--------|
| AM650 | OOK | 0x30 | — | — | `FuriHalSubGhzPresetOok650Async` | firmware `cc1101_configs.c` (stock) |
| AM270 | OOK | 0x30 | — | — | `FuriHalSubGhzPresetOok270Async` | firmware `cc1101_configs.c` (stock) |
| FM238 | 2-FSK | 0x04 | 0x04 | ±2.38 kHz | `FuriHalSubGhzPreset2FSKDev238Async` | firmware `cc1101_configs.c` (stock) |
| FM476 | 2-FSK | 0x04 | 0x47 | ±47.61 kHz | `FuriHalSubGhzPreset2FSKDev476Async` | firmware `cc1101_configs.c` (stock) |
| F4 | 2-FSK | 0x04 | 0x31 | ±14.28 kHz | custom | doctor-firmware `setting_user.example` |
| Honda1 | 2-FSK | 0x04 | 0x32 | ±15.87 kHz | custom | doctor-firmware `setting_user.example`, Flipper-ARF `setting_user` |
| F3 | 2-FSK | 0x80 | 0x42 | ±31.74 kHz | custom | doctor-firmware `setting_user.example` (ARF: same DEVIATN) |
| F2 | 2-FSK | 0x00 | 0x34 | ±19.04 kHz | custom | doctor-firmware `setting_user.example` |
| FM12K | 2-FSK | 0x04 | 0x30 | ±12.70 kHz | `FuriHalSubGhzPreset2FSKDev12KAsync` | Flipper-ARF / doctor-firmware built-in |
| FM14K | 2-FSK | 0x04 | 0x31 | ±14.28 kHz | `FM14K` | doctor-firmware built-in (same image as F4) |
| FM20K | 2-FSK | 0x00 | 0x30 | ±12.70 kHz | `FM20K` | doctor-firmware built-in "mercedes" preset (20 kBaud) |
| AM101 | OOK | 0x30 | — | — | `AM101` | doctor-firmware built-in |

On `.sub` export KAT writes the four presets every Flipper firmware has (AM650, AM270, FM238, FM476) by name
and every other preset as `FuriHalSubGhzPresetCustom` + `Custom_preset_module: CC1101` + its registers, so the
file loads on any firmware. Note that ProtoPirate's help legend calls F4 "4FSK": its MDMCFG2 (0x04) is 2-FSK.

## Per-protocol presets

The presets come from ProtoPirate's help text (`scenes/plugins/protopirate_welcome_plugin.c`, e.g.
"V1 [F4 315/433]"), its catalog routes (`protocols/protocol_items.c`), the references' protocol flags
(`SubGhzProtocolFlag_AM` / `_FM` in each protocol's `.c`) and real captures in `IMPORTS/`. Where a reference
flags a protocol FM without naming a preset, KAT uses FM476: the default FM preset of all three references
(doctor-firmware's dual receive defaults to AM650 + FM476, Flipper-ARF hops AM650 / FM476 / FM95, ProtoPirate's
emulate scene falls back to AM650, then FM476). The first preset is the primary one.

### FM and AM/FM protocols

| Protocol | RF | Presets | FSK deviation | Sources | Notes |
|----------|----|---------|---------------|---------|-------|
| Kia V0 | AM/FM | FM476 + AM650 | ±47.61 kHz | PP help "V0 [FM476 315/433]", PP flag FM, catalog FM default; ARF flags AM + FM | IMPORTS `KIA/HYU V0` key files were received with FM476 |
| Suzuki V0 | AM/FM | FM476 + AM650 | ±47.61 kHz | Kia V0 frame type (PP `kia_v0.c`) | IMPORTS `SUZUKI/Suzuki_v0_kd.sub` was recorded with FM476 |
| Honda V0 | AM/FM | FM476 + AM650 | ±47.61 kHz | Kia V0 frame type (PP `kia_v0.c`) | |
| Kia V2 | AM/FM | FM476 + AM650 | ±47.61 kHz | PP help "V2 [FM476 315/433]" and README (FM table); PP flag AM and catalog AM default; ARF flag AM; doctor "AM" | The references disagree; both IMPORTS Kia V2 key files (`HYU V2 N1.sub`, `HYU V2_random_habar.sub`) were received with FM476. KAT ≤ 1.4 said AM. |
| Kia V3/V4 | FM | FM476 | ±47.61 kHz | PP help "V3/V4 [FM476]", PP flag FM; ARF flag FM | IMPORTS `Kia3_…`, `Kia4_…`, `KIA V3/V4 N1 RAW` recorded with FM476 |
| Kia V5 | FM | FM476 | ±47.61 kHz | PP help "V5 [FM476]", PP flag FM; ARF flag FM | IMPORTS `KIA V5 N1/N2 RAW`, `KIA_V5_x20` recorded with FM476 |
| Kia V6 | FM | FM476 | ±47.61 kHz | PP help "V6 [FM476]", PP flag FM; ARF flag FM | IMPORTS Kia V6 key files received with FM476 |
| Kia V7 | FM | FM476 | ±47.61 kHz | PP help "V7 [FM476]", PP flag FM; ARF flag FM | |
| Mitsubishi V0 | FM | FM476 | ±47.61 kHz | PP help "Mitsubishi V0 [FM476]"; ARF `mitsubishi_v0.c` flag FM | both the 61-bit (Kia V0 type) and 96-bit formats |
| Ford V1 | FM | F4 | ±14.28 kHz | PP help "V1 [F4 315/433]", catalog FM F4; ARF flag FM | |
| Ford V2 | FM | F4 | ±14.28 kHz | PP help "V2 [F4 434]", catalog FM F4; ARF flag FM | |
| Ford V3 | AM/FM | AM650 + F4 | ±14.28 kHz | PP help "V3 [AM650+F4 434]", PP flags AM + FM, catalog FM F4; ARF flags AM + FM | |
| Honda Static | FM | Honda1 | ±15.87 kHz | PP help "Static [Honda1 315/433]", catalog FM Honda1; ARF flag FM | IMPORTS `honda/*_honda.sub` were recorded with exactly Honda1's registers |
| Honda V2 | FM | F4 | ±14.28 kHz | PP help "V2 [F4 315/433]", catalog FM F4; ARF flag FM | formerly "Land Rover V0" |
| Scher-Khan | AM/FM | FM476 + AM650 | ±47.61 kHz | PP help "Scher-Khan [FM 315/433]", PP flag FM, catalog FM default; ARF flags FM + AM | no named FM preset → FM476 |
| PSA | AM/FM | AM650 + F3 | ±31.74 kHz | PP help "[AM650+F3]", PP flags AM + FM, catalog by modulation; ARF flags AM + FM | IMPORTS `GROUPE PSA` captures are OOK |
| Mazda V0 | AM/FM | AM650 + F2 | ±19.04 kHz | PP help "Mazda V0 [AM650+F2 315/433]", PP flags AM + FM; ARF flag FM | F2 from doctor-firmware's `setting_user.example` (PP's README: "FM (F2?)") |
| Mazda Siemens | FM | FM476 | ±47.61 kHz | ARF `mazda_siemens.c` flag FM | no named preset → FM476 |
| Mazda V2 | AM/FM | AM650 + FM476 | ±47.61 kHz | ARF `mazda_v2.c` flags AM + FM; doctor "AM/FM" | no named FM preset → FM476 |
| Mercedes V0 | FM | FM20K | ±12.70 kHz | ARF / doctor `mercedes_v0.c` flag FM (433 MHz); doctor's built-in "mercedes" preset (`subghz_device_cc1101_preset_mercedes_async_regs`, registered as FM20K) | |
| Mercedes V1 | FM | FM20K | ±12.70 kHz | ARF / doctor `mercedes_v1.c` flag FM (433 MHz); doctor's "mercedes" preset | FM20K's 20 kBaud matches V1's 50 µs chips |
| Renault V1 | AM/FM | AM650 + FM476 | ±47.61 kHz | PP flags AM + FM, catalog by modulation, README AM650; doctor `renault_v1.c` (HITAG2) flags AM + FM | no named FM preset → FM476 |
| Renault V1 (ARF) | AM/FM | AM650 + FM476 | ±47.61 kHz | ARF `renault_v1.c` flags AM + FM | no named FM preset → FM476 |
| Datsun Lada | FM | FM476 | ±47.61 kHz | doctor `datsun_lada.c` flag FM (433 MHz) | no named preset → FM476 (doctor's default FM receive preset) |
| Holtek_HT12X | AM/FM | AM650 + FM476 | ±47.61 kHz | Flipper-ARF garage app / lib and doctor `holtek_ht12x.c` flag AM + FM | gate remote (HT12E); no named FM preset → FM476 |
| Ansonic | AM/FM | AM650 + FM476 | ±47.61 kHz | Flipper-ARF `ansonic.c` (garage app and lib) flag FM; doctor `ansonic.c` flag AM | gate remote; the Flipper unit-test capture `ansonic_raw.sub` was recorded with OOK650 and the UberGuidoZ Ansonic brute-force files use it; no named FM preset → FM476 |
| Hormann BiSecur | FM | F3 | ±31.74 kHz | Momentum / RogueMaster `hormann_bisecur.c` flag FM (868 MHz); their unit-test captures use a custom preset with DEVIATN 0x40 (±25.39 kHz) | that deviation lies exactly between F2 (±19.04) and F3 (±31.74); KAT transmits with a capture's own preset whenever it has one (the key files and the RAW capture carry it) |
| Cardin S508 | FM | FM12K | ±12.70 kHz | RogueMaster `cardin_s508.c` flag FM (868 MHz); its header: "read with the FM12K preset" | decode-only (rolling AES-128): Replay of the recording only |
| Continental-KR5V2X | FM | FM476 | ±47.61 kHz | rtl_433 `continental_car_remote.c` (`FSK_PULSE_MANCHESTER_ZEROBIT`) | rtl_433 port, receive only (replay of the recorded pulses); KAT measures ±43 kHz on the rtl_433_tests recordings → FM476 |
| Honda-KR5V2X1X | FM | FM476 | ±47.61 kHz | rtl_433 `continental_car_remote.c` `honda_keyfob` (`FSK_PULSE_MANCHESTER_ZEROBIT`) | rtl_433 port, receive only; no recording, deviation unknown → FM476 |
| Opel-Mokka | FM | FM476 | ±47.61 kHz | rtl_433 `opel_mokka.c` (`FSK_PULSE_MANCHESTER_ZEROBIT`) | rtl_433 port, receive only; no recording, deviation unknown → FM476 |
| Alps-FWB1U545 | FM | F3 | ±31.74 kHz | rtl_433 `alps_fwb1u545.c` (`FSK_PULSE_MANCHESTER_ZEROBIT`); receive only | Replay only: the rtl_433_tests recordings show ±30–37 kHz, closest to F3 |
| Elero | FM | F3 | ±31.74 kHz | rtl_433 `elero.c` (FSK_PULSE_PCM, 13 µs bits) | shutter remote, receive only (rtl_433 port; Replay uses the preset): the rtl_433_tests recordings show ±38 kHz, F3 is the closest |
| Somfy-IOHC | FM | F2 | ±19.04 kHz | rtl_433 `somfy_iohc.c` (FSK_PULSE_PCM, 26 µs bits) | io-homecontrol (±19.2 kHz), receive only (rtl_433 port) |
| RojaFlex-Remote | FM | FM476 | ±47.61 kHz | rtl_433 `rojaflex.c` (FSK_PULSE_PCM, 100 µs bits) | shutter remote, receive only (rtl_433 port); no deviation documented → FM476 |

### AM protocols

AM650 (OOK): Kia V1, VAG, Ford V0, Honda V1, Subaru, Fiat V0, Fiat V1, Fiat V2, Star Line, Sheriff CFM,
KeeLoq (registry and generic fallback), Chrysler V0, Porsche Cayenne, Toyota, BMW, Opel V1, Renault V0, GM, and
the unregistered Land Rover RKE, Audi, Hyundai and Mazda Infinity. ProtoPirate's help lists AM650 for every one
it carries. Flipper-ARF also flags Subaru and Fiat V1 FM (and doctor-firmware lists Fiat V1/V2 as AM/FM); no
reference names an FM preset for them and no capture shows one, so KAT keeps them AM. A capture of any of them
with its own FSK preset (an imported `.sub`) still transmits FSK.

`src/protocols/info.rs` checks that this table lists every FM and AM/FM protocol with the presets the code uses.

## After a transmission

What the references do once they have transmitted a decoded signal (rather than a raw recording):

- **Flipper-ARF and doctor-firmware** (`applications/main/subghz/helpers/subghz_txrx.c`): the encoder's
  `deserialize` builds the upload from the signal's data and, for a rolling code, advances the counter and
  writes the new `Key` (and fields such as `Cnt`) back into that data, so the next transmission of the same
  signal sends the code after. When TX stops, a `SubGhzProtocolTypeDynamic` protocol's data is saved back to
  the file it was loaded from (`subghz_txrx_tx_stop`: "if protocol dynamic then we save the last upload";
  doctor-firmware only when it sent its own data, `tx_from_internal_fff`). Static (fixed-code) protocols are
  not saved back.
- **ProtoPirate** (`scenes/plugins/protopirate_emulate_plugin.c`, car remotes): emulation runs on a saved
  file, which it opens for writing; every button press increments the counter (Renault V0: 8 bits) and writes
  `Cnt` and the pressed `Btn` into the file before transmitting (`emulate_update_data`).

KAT does the same for every encoder transmission — Send next code, Send button…, TX Lock / Unlock / Trunk /
Panic and `:lock` … `:panic` (`src/app/tx.rs`): once the transmission went out (a failed one changes
nothing), the capture holds the frame sent — counter, frame (`data`, `data_extra`), button and check flag, as
a receiver decodes it — while its recorded pulses, name, frequency, preset and metadata stay. The session
autosave keeps it, so the next press, or the next session, sends the code after; exports made afterwards
(`.fob`, Flipper key file) carry the advanced state. Up to v1.6.0 every press sent the same frame, and a
remote made with `:new` worked once after pairing.

- Only rolling codes change (the counter moved). Fixed codes keep their capture, also after Send button…; so
  do encoders that replay the captured frame because no key verifies it (Fiat V0, and Fiat V1 / V2 and
  Renault V1 without a verified key send it byte for byte, as the references do; a key saved in
  `hitag2_keys.txt`, e.g. by [Recover key (HiTag2)](hitag2_hell.md), verifies theirs).
- **Replay** sends the recorded pulses and changes nothing.
- KAT learns the frame sent by reading the upload with the protocol's own decoder (`sent_frame`). Encoder
  state the air does not carry stays the capture's: FAAC SLH's seed, with which the protocol re-reads the
  frame through its key file to get the counter. PSA reports the frame it built
  (`ProtocolDecoder::sent_signal`): a TEA frame only decrypts under the serial "Recover key" found.
  KAT's PSA frames end like the genuine fobs' (900 µs, not the references' 1000 µs), so Flipper
  firmwares can read them also when the last bit is 0 ([docs/psa.md](psa.md)).
- Transmitting is refused while `:record` runs: TX stops the receiver, which would end the recording.

## Files

- **`.sub` import** reads `Preset:` (stock names, doctor-firmware's `FM20K` / `FM14K` / `AM101`, short names
  such as `AM650`) and `Custom_preset_module` / `Custom_preset_data`, and keeps them on the capture
  (`Capture::rf_preset`).
- **`.sub` export** writes the capture's own preset back verbatim; otherwise the preset KAT transmits with
  (KAT ≤ 1.4 wrote `FuriHalSubGhzPresetOok270Async` for everything).
- **`.fob`** adds optional `signal.received_rf` ("AM" / "FM", live captures), `signal.preset`,
  `signal.custom_preset_module`, `signal.custom_preset_data` (hex, as in `.sub`) and an informational
  `signal.tx_modulation` ("2-FSK ±14.28 kHz"). For a live capture KAT persists the preset it chose, so the file
  transmits the same way after re-import. Files from KAT ≤ 1.4 (no such fields) load as before; their protocol's
  primary preset applies.

## Verification

Synthetic only — no hardware in the loop:

- Sample counts are exact (2 per µs); OOK "off" samples are exact zeros; the OOK tone sits at +500 kHz.
- 2-FSK: every sample's phase follows the ideal continuous-phase trajectory within 8-bit quantisation
  (< 0.001 cycles), so there is no phase jump at bit boundaries and no frequency drift; the per-pair average
  frequency is the offset ± deviation within 20 Hz; the envelope is constant.
- Loopback: real IMPORTS frames (Kia V5, Kia V4, Suzuki V0 at FM476; Honda Static at Honda1) and a Ford V1 frame
  (F4) are re-encoded, transmitted as 2-FSK on the offset, shifted back to the carrier and fed through KAT's
  receive-side `FmDemodulator` (no noise) and the registry: every one decodes with the original serial.
- The TX stream's buffer/zero filling and completion rule are unit-tested with simulated libhackrf callbacks.
- After a transmission (`src/app/tx.rs` tests): every rolling-code remote `:new` creates, and the IMPORTS car
  key files (Kia V0 / V2 / V5 / V6, VAG), send a new counter on each of three presses once the frame sent is
  kept; fixed codes keep their capture. PSA's reported frame equals the decode of its upload where that
  decodes, and two presses from the TEA fob's first frame give the fob's own later frame.

Not verified: the HackRF's actual RF output (spectrum, LO leakage level, power at each TX setting), libhackrf's
callback timing on real hardware, and reception by a car.
