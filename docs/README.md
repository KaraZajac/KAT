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
| Ford V0 | `ford_v0.rs` | [ford_v0.md](ford_v0.md) |
| Subaru | `subaru.rs` | [subaru.md](subaru.md) |
| VAG | `vag.rs` | [vag.md](vag.md) |
| Fiat V0 | `fiat_v0.rs` | [fiat_v0.md](fiat_v0.md) |
| Suzuki | `suzuki.rs` | [suzuki.md](suzuki.md) |
| Scher-Khan | `scher_khan.rs` | [scher_khan.md](scher_khan.md) |
| Star Line | `star_line.rs` | [star_line.md](star_line.md) |
| PSA | `psa.rs` | [psa.md](psa.md) |
| KeeLoq generic (fallback) | `keeloq_generic.rs` | [keeloq_generic.md](keeloq_generic.md) |

**KeeLoq generic** is not a registered decoder; it runs when no protocol matches and tries KeeLoq with every keystore manufacturer key (using `keeloq_common`). Successful decodes appear as **Keeloq (*keystore name*)**.

Implementations are aligned with the ProtoPirate reference in `REFERENCES/ProtoPirate/protocols/`.
