# KAT Protocol Documentation

This folder describes how each keyfob and gate/garage remote protocol supported by KAT works. Each document corresponds to a decoder/encoder in `src/protocols/`.

**Radio and app features:** [radio.md](radio.md) (receive pipeline, IQ recordings, `:record`, frequency hopping), [transmit.md](transmit.md) (OOK / 2-FSK transmit, per-protocol presets and deviations, TX power), [session.md](session.md) (session autosave and saved-fob matching), [timing_analysis.md](timing_analysis.md) (ProtoPirate's Timing Tuner and line-coding guesses), [add_manually.md](add_manually.md) (`:new`: new gate / garage remotes of 107 Flipper-ARF / doctor-firmware / RogueMaster types, and the signal menu's "Send button…"), [hitag2_hell.md](hitag2_hell.md) (Recover key (HiTag2): the HiTag2 Hell attack for Fiat V1 / Renault V1 / Fiat V2 remotes, and the `hitag2_keys.txt` key file).

**Capture/encode model:** Decoded signals expose optional `extra` (e.g. VAG: Key2 + type + key index + counter high byte; protocols with frames longer than 64 bits keep the remaining bits there). When present, the app stores it in `Capture.data_extra` (persisted in `.fob` as `signal.data_extra`) so retransmit can encode from the capture without decoder instance state. See [vag.md](vag.md) for the VAG encode-from-capture flow.

**Flipper key files:** [flipper_key_files.md](flipper_key_files.md) — loading and saving decoded keys (`Protocol` / `Bit` / `Key` `.sub` files, as ProtoPirate and Flipper-ARF save every decode): the name and lines each protocol writes, and what loads.

**Unidentified captures:** [unidentified_captures.md](unidentified_captures.md) — the owner-collection captures in `IMPORTS/` that no decoder (KAT's or a reference's) reads: which were recorded with the wrong preset, and what is known of the formats no reference implements (VW Polo 5 2017, Lexus UX 250 2021, Volvo S40 1999).

| Protocol | Rust module | Doc |
|----------|-------------|-----|
| Kia V0 (+ Suzuki V0, Honda V0, Mitsubishi V0 frame types) | `kia_v0.rs` | [kia_v0.md](kia_v0.md) |
| Kia V1 | `kia_v1.rs` | [kia_v1.md](kia_v1.md) |
| Kia V2 | `kia_v2.rs` | [kia_v2.md](kia_v2.md) |
| Kia V3/V4 | `kia_v3_v4.rs` | [kia_v3_v4.md](kia_v3_v4.md) |
| Kia V5 | `kia_v5.rs` | [kia_v5.md](kia_v5.md) |
| Kia V6 | `kia_v6.rs` | [kia_v6.md](kia_v6.md) |
| Kia V7 | `kia_v7.rs` | [kia_v7.md](kia_v7.md) |
| Ford V0 | `ford_v0.rs` | [ford_v0.md](ford_v0.md) |
| Ford V1 | `ford_v1.rs` | [ford_v1.md](ford_v1.md) |
| Ford V2 | `ford_v2.rs` | [ford_v2.md](ford_v2.md) |
| Ford V3 | `ford_v3.rs` | [ford_v3.md](ford_v3.md) |
| Honda Static | `honda_static.rs` | [honda_static.md](honda_static.md) |
| Honda V1 | `honda_v1.rs` | [honda_v1.md](honda_v1.md) |
| Honda V2 (formerly Land Rover V0) | `honda_v2.rs` | [honda_v2.md](honda_v2.md) |
| Chrysler V0 | `chrysler_v0.rs` | [chrysler_v0.md](chrysler_v0.md) |
| GM (Chevrolet/Buick/GMC/Cadillac) | `gm.rs` | [gm.md](gm.md) |
| Subaru | `subaru.rs` | [subaru.md](subaru.md) |
| Toyota | `toyota.rs` | [toyota.md](toyota.md) |
| VAG | `vag.rs` | [vag.md](vag.md) |
| Fiat V0 | `fiat_v0.rs` | [fiat_v0.md](fiat_v0.md) |
| Fiat V1 | `fiat_v1.rs` | [fiat_v1.md](fiat_v1.md) |
| Fiat V2 (FCA: Fiat/Chrysler/Jeep) | `fiat_v2.rs` | [fiat_v2.md](fiat_v2.md) |
| Renault V0 | `renault_v0.rs` | [renault_v0.md](renault_v0.md) |
| Renault V1 (HITAG2) | `renault_v1.rs` | [renault_v1.md](renault_v1.md) |
| Renault V1 (ARF) — Flipper-ARF type-table frames | `renault_v1_arf.rs` | [renault_v1_arf.md](renault_v1_arf.md) |
| Mazda V0 | `mazda_v0.rs` | [mazda_v0.md](mazda_v0.md) |
| Mazda Siemens | `mazda_siemens.rs` | [mazda_siemens.md](mazda_siemens.md) |
| Mazda V2 | `mazda_v2.rs` | [mazda_v2.md](mazda_v2.md) |
| Mitsubishi V0 | `mitsubishi_v0.rs` | [mitsubishi_v0.md](mitsubishi_v0.md) |
| Land Rover RKE (**not registered** — ARF never registers it) | `land_rover_rke.rs` | [land_rover_rke.md](land_rover_rke.md) |
| BMW | `bmw.rs` | [bmw.md](bmw.md) |
| Mercedes V0 | `mercedes_v0.rs` | [mercedes_v0.md](mercedes_v0.md) |
| Mercedes V1 | `mercedes_v1.rs` | [mercedes_v1.md](mercedes_v1.md) |
| Opel V1 | `opel_v1.rs` | [opel_v1.md](opel_v1.md) |
| Datsun Lada | `datsun_lada.rs` | [datsun_lada.md](datsun_lada.md) |
| Porsche Cayenne | `porsche_cayenne.rs` | [porsche_cayenne.md](porsche_cayenne.md) |
| Scher-Khan | `scher_khan.rs` | [scher_khan.md](scher_khan.md) |
| Star Line | `star_line.rs` | [star_line.md](star_line.md) |
| Sheriff CFM (car alarm) | `sheriff_cfm.rs` | [sheriff_cfm.md](sheriff_cfm.md) |
| KeeLoq (gates, garage doors, alarms) | `keeloq.rs` | [keeloq.md](keeloq.md) |
| PSA | `psa.rs` | [psa.md](psa.md) |
| Audi (experimental, Pandora RE — **not registered**) | `audi.rs` | [audi.md](audi.md) |
| Mazda Infinity (experimental, Pandora RE — **not registered**) | `mazda_infinity.rs` | [mazda_infinity.md](mazda_infinity.md) |
| Hyundai (experimental, Pandora RE — **not registered**) | `hyundai.rs` | [hyundai.md](hyundai.md) |
| KeeLoq generic (fallback) | `keeloq_generic.rs` | [keeloq_generic.md](keeloq_generic.md) |

### Garage door, gate, barrier and shutter remotes

Ported from Flipper-ARF's garage app (`applications/system/subghz_garage/protocols/`) and doctor-firmware, and from Momentum and RogueMaster for Hörmann BiSecur and Cardin S508; registered after every car protocol, most strongly gated first. Shared helpers: `g1_common.rs` … `g6_common.rs`, `f_common.rs`.

| Protocol | Rust module | Doc |
|----------|-------------|-----|
| Beninca ARC | `beninca_arc.rs` | [beninca_arc.md](beninca_arc.md) |
| KingGates Stylo4k | `kinggates_stylo_4k.rs` | [kinggates_stylo_4k.md](kinggates_stylo_4k.md) |
| Jarolift | `jarolift.rs` | [jarolift.md](jarolift.md) |
| Faac SLH | `faac_slh.rs` | [faac_slh.md](faac_slh.md) |
| Security+ 2.0 | `secplus_v2.rs` | [secplus_v2.md](secplus_v2.md) |
| Somfy Keytis | `somfy_keytis.rs` | [somfy_keytis.md](somfy_keytis.md) |
| Somfy Telis | `somfy_telis.rs` | [somfy_telis.md](somfy_telis.md) |
| Security+ 1.0 | `secplus_v1.rs` | [secplus_v1.md](secplus_v1.md) |
| Alutech AT-4N | `alutech_at_4n.rs` | [alutech_at_4n.md](alutech_at_4n.md) |
| Nice FloR-S / Nice One | `nice_flor_s.rs` | [nice_flor_s.md](nice_flor_s.md) |
| Ditec GOL4 | `ditec_gol4.rs` | [ditec_gol4.md](ditec_gol4.md) |
| CAME Atomo | `came_atomo.rs` | [came_atomo.md](came_atomo.md) |
| Power Smart | `power_smart.rs` | [power_smart.md](power_smart.md) |
| Urmet | `urmet.rs` | [urmet.md](urmet.md) |
| Revers RB2 | `revers_rb2.rs` | [revers_rb2.md](revers_rb2.md) |
| Telcoma/Cardin EDGE | `telcoma_edge.rs` | [telcoma_edge.md](telcoma_edge.md) |
| Phoenix V2 | `phoenix_v2.rs` | [phoenix_v2.md](phoenix_v2.md) |
| Dooya | `dooya.rs` | [dooya.md](dooya.md) |
| Roger | `roger.rs` | [roger.md](roger.md) |
| Hay21 | `hay21.rs` | [hay21.md](hay21.md) |
| Allstar Firefly | `allstar_firefly.rs` | [allstar_firefly.md](allstar_firefly.md) |
| Marantec | `marantec.rs` | [marantec.md](marantec.md) |
| Hörmann HSM | `hormann.rs` | [hormann.md](hormann.md) |
| Nero Radio | `nero_radio.rs` | [nero_radio.md](nero_radio.md) |
| Nero Sketch | `nero_sketch.rs` | [nero_sketch.md](nero_sketch.md) |
| iDo 117/111 | `ido.rs` | [ido.md](ido.md) |
| Dickert MAHS | `dickert_mahs.rs` | [dickert_mahs.md](dickert_mahs.md) |
| Doitrand | `doitrand.rs` | [doitrand.md](doitrand.md) |
| Mastercode | `mastercode.rs` | [mastercode.md](mastercode.md) |
| Marantec24 | `marantec24.rs` | [marantec24.md](marantec24.md) |
| Clemsa | `clemsa.rs` | [clemsa.md](clemsa.md) |
| BETT | `bett.rs` | [bett.md](bett.md) |
| Elplast | `elplast.rs` | [elplast.md](elplast.md) |
| Ansonic | `ansonic.rs` | [ansonic.md](ansonic.md) |
| Hörmann BiSecur (Momentum / RogueMaster) | `hormann_bisecur.rs` | [hormann_bisecur.md](hormann_bisecur.md) |
| GangQi | `gangqi.rs` | [gangqi.md](gangqi.md) |
| Hollarm | `hollarm.rs` | [hollarm.md](hollarm.md) |
| Cardin S508 (RogueMaster) | `cardin_s508.rs` | [cardin_s508.md](cardin_s508.md) |
| Holtek | `holtek.rs` | [holtek.md](holtek.md) |
| CAME TWEE | `came_twee.rs` | [came_twee.md](came_twee.md) |
| MegaCode | `megacode.rs` | [megacode.md](megacode.md) |
| Chamberlain Code (Cham_Code) | `chamberlain_code.rs` | [chamberlain_code.md](chamberlain_code.md) |
| GateTX | `gate_tx.rs` | [gate_tx.md](gate_tx.md) |
| Holtek HT12X | `holtek_ht12x.rs` | [holtek_ht12x.md](holtek_ht12x.md) |
| CAME | `came.rs` | [came.md](came.md) |
| Prastel | `prastel.rs` | [prastel.md](prastel.md) |
| Nice FLO | `nice_flo.rs` | [nice_flo.md](nice_flo.md) |
| Linear | `linear.rs` | [linear.md](linear.md) |
| Linear Delta3 | `linear_delta3.rs` | [linear_delta3.md](linear_delta3.md) |
| SMC5326 | `smc5326.rs` | [smc5326.md](smc5326.md) |
| Princeton (PT2262 / EV1527) | `princeton.rs` | [princeton.md](princeton.md) |

### Receive-only remotes from rtl_433

Ports of [rtl_433](https://github.com/merbanan/rtl_433) decoders (`src/devices/`) for remotes no Flipper firmware decodes; receive only, registered last. They share one exact port of rtl_433's receive chain, `src/protocols/rtl433/`: the pulse detector, the pulse slicers (PWM, PPM, Manchester zero-bit, PIWM_DC, PCM) and the bit buffer, run over KAT's level/duration stream.

| Protocol | Rust module | Doc |
|----------|-------------|-----|
| Alps FWB1U545 | `alps_fwb1u545.rs` | [alps_fwb1u545.md](alps_fwb1u545.md) |
| Continental KR5V2X | `continental_kr5v2x.rs` | [continental_kr5v2x.md](continental_kr5v2x.md) |
| Honda KR5V2X/1X | `honda_kr5v2x1x.rs` | [honda_kr5v2x1x.md](honda_kr5v2x1x.md) |
| Siemens 5WY72XX | `siemens_5wy72xx.rs` | [siemens_5wy72xx.md](siemens_5wy72xx.md) |
| Chrysler car remote (315 MHz) | `chrysler_car_remote.rs` | [chrysler_car_remote.md](chrysler_car_remote.md) |
| Opel Mokka | `opel_mokka.rs` | [opel_mokka.md](opel_mokka.md) |
| MIC 6SC2 | `mic6sc2.rs` | [mic6sc2.md](mic6sc2.md) |
| Code Alarm FRDPC2002 | `code_alarm_frdpc2002.rs` | [code_alarm_frdpc2002.md](code_alarm_frdpc2002.md) |
| Compustar 1WG3R | `compustar_1wg3r.rs` | [compustar_1wg3r.md](compustar_1wg3r.md) |
| Astrostart 2000 | `astrostart_2000.rs` | [astrostart_2000.md](astrostart_2000.md) |
| Audiovox PRO-OE3B | `audiovox_pro_oe3b.rs` | [audiovox_pro_oe3b.md](audiovox_pro_oe3b.md) |
| Microchip HCS361 (KeeLoq) | `microchip_hcs361.rs` | [microchip_hcs361.md](microchip_hcs361.md) |
| Microchip HCS362 (KeeLoq) | `microchip_hcs362.rs` | [microchip_hcs362.md](microchip_hcs362.md) |
| Elero (blinds, awnings) | `elero.rs` | [elero.md](elero.md) |
| Somfy io-homecontrol | `somfy_iohc.rs` | [somfy_iohc.md](somfy_iohc.md) |
| RojaFlex (shutters) | `rojaflex.rs` | [rojaflex.md](rojaflex.md) |
| Markisol (not registered) | `markisol.rs` | [markisol.md](markisol.md) |

rtl_433 decoders KAT does not duplicate (an existing decoder reads the same frames): [Ford-CarRemote → Ford V0](ford_car_remote.md), [Nidec-OUCG8D and Honda-CarRemote → Honda V0](nidec_oucg8d.md), [Akhan-100F14 → Princeton](akhan_100f14.md); rtl_433's GM-ABO1502T reads the frames of [GM](gm.md) with the same fields.

**Add Manually** (new remotes of a chosen type): see [add_manually.md](add_manually.md).

**Shared cipher modules:** `hitag2.rs` (HITAG2 keystream/authenticator, used by Fiat V1 and Renault V1), `aut64.rs`, `keeloq_common.rs`.

**Not registered (mirrors the references):** Audi, Mazda Infinity and Hyundai (Pandora-firmware reverse engineering, shipped disabled by ARF) and Land Rover RKE (an ARF helper that is never registered) are ported and tested but kept out of KAT's registry, exactly as ARF does, and so is Markisol, which rtl_433 ships disabled for its false positives; each is a one-line change in `ProtocolRegistry::new()` to enable. Retired: **BMW CAS4** (ARF replaced it with **BMW**), **Porsche Touareg** (ProtoPirate removed it; same frame as Porsche Cayenne), **Suzuki** (ProtoPirate folded it into Kia V0) and **PSA2** (PSA decodes those frames).

**KeeLoq generic** is not a registered decoder; it runs when no protocol matches and tries KeeLoq with every keystore manufacturer key (using `keeloq_common`). Successful decodes appear as **Keeloq (*keystore name*)**.

Implementations are aligned with the ProtoPirate reference in `REFERENCES/ProtoPirate/protocols/`.
