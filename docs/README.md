# KAT Protocol Documentation

This folder describes how each keyfob protocol supported by KAT works. Each document corresponds to a decoder/encoder in `src/protocols/`.

**Capture/encode model:** Decoded signals expose optional `extra` (e.g. VAG: Key2 + type + key index + counter high byte; protocols with frames longer than 64 bits keep the remaining bits there). When present, the app stores it in `Capture.data_extra` (persisted in `.fob` as `signal.data_extra`) so retransmit can encode from the capture without decoder instance state. See [vag.md](vag.md) for the VAG encode-from-capture flow.

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

**Shared cipher modules:** `hitag2.rs` (HITAG2 keystream/authenticator, used by Fiat V1 and Renault V1), `aut64.rs`, `keeloq_common.rs`.

**Not registered (mirrors Flipper-ARF):** Audi, Mazda Infinity and Hyundai (Pandora-firmware reverse engineering, shipped disabled by ARF) and Land Rover RKE (an ARF helper that is never registered) are ported and tested but kept out of KAT's registry, exactly as ARF does; each is a one-line change in `ProtocolRegistry::new()` to enable. Retired: **BMW CAS4** (ARF replaced it with **BMW**), **Porsche Touareg** (ProtoPirate removed it; same frame as Porsche Cayenne), **Suzuki** (ProtoPirate folded it into Kia V0) and **PSA2** (PSA decodes those frames).

**KeeLoq generic** is not a registered decoder; it runs when no protocol matches and tries KeeLoq with every keystore manufacturer key (using `keeloq_common`). Successful decodes appear as **Keeloq (*keystore name*)**.

Implementations are aligned with the ProtoPirate reference in `REFERENCES/ProtoPirate/protocols/`.
