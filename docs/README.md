# KAT Protocol Documentation

This folder describes how each keyfob protocol supported by KAT works. Each document corresponds to a decoder/encoder in `src/protocols/`.

**Capture/encode model:** Decoded signals expose optional `extra` (e.g. VAG: vag_type + key_idx). When present, the app stores it in `Capture.data_extra` so retransmit can encode from the capture without decoder instance state. See [vag.md](vag.md) for the VAG encode-from-capture flow.

| Protocol | Rust module | Doc |
|----------|-------------|-----|
| Kia V0 | `kia_v0.rs` | [kia_v0.md](kia_v0.md) |
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
| Chrysler V0 | `chrysler_v0.rs` | [chrysler_v0.md](chrysler_v0.md) |
| Subaru | `subaru.rs` | [subaru.md](subaru.md) |
| Toyota | `toyota.rs` | [toyota.md](toyota.md) |
| VAG | `vag.rs` | [vag.md](vag.md) |
| Fiat V0 | `fiat_v0.rs` | [fiat_v0.md](fiat_v0.md) |
| Fiat V1 | `fiat_v1.rs` | [fiat_v1.md](fiat_v1.md) |
| Mazda V0 | `mazda_v0.rs` | [mazda_v0.md](mazda_v0.md) |
| Mazda Siemens | `mazda_siemens.rs` | [mazda_siemens.md](mazda_siemens.md) |
| Mitsubishi V0 | `mitsubishi_v0.rs` | [mitsubishi_v0.md](mitsubishi_v0.md) |
| Land Rover V0 | `land_rover_v0.rs` | [land_rover_v0.md](land_rover_v0.md) |
| Land Rover RKE | `land_rover_rke.rs` | [land_rover_rke.md](land_rover_rke.md) |
| BMW CAS4 | `bmw_cas4.rs` | [bmw_cas4.md](bmw_cas4.md) |
| Porsche Touareg | `porsche_touareg.rs` | [porsche_touareg.md](porsche_touareg.md) |
| Porsche Cayenne | `porsche_cayenne.rs` | [porsche_cayenne.md](porsche_cayenne.md) |
| Suzuki | `suzuki.rs` | [suzuki.md](suzuki.md) |
| Scher-Khan | `scher_khan.rs` | [scher_khan.md](scher_khan.md) |
| Star Line | `star_line.rs` | [star_line.md](star_line.md) |
| PSA | `psa.rs` | [psa.md](psa.md) |
| PSA2 (PSA OLD) | `psa2.rs` | [psa2.md](psa2.md) |
| KeeLoq generic (fallback) | `keeloq_generic.rs` | [keeloq_generic.md](keeloq_generic.md) |

**KeeLoq generic** is not a registered decoder; it runs when no protocol matches and tries KeeLoq with every keystore manufacturer key (using `keeloq_common`). Successful decodes appear as **Keeloq (*keystore name*)**.

Implementations are aligned with the ProtoPirate reference in `REFERENCES/ProtoPirate/protocols/`.
