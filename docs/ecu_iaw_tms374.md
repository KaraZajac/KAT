---
layout: default
---

# IAW on TMS374 (Magneti Marelli)

[ECU & immobiliser reference](ecu.md) › IAW TMS374

One of the most common small petrol‑ECU immobilisers on 1990s–2000s Fiat‑group and PSA cars:
**Magneti Marelli IAW** engine ECUs built around a **Texas Instruments TMS374** microcontroller,
whose immobiliser record sits in the MCU's **on‑chip EEPROM** (no external memory chip).

## Where it is used

| Badge | ECU variants (TMS374) |
|---|---|
| Fiat / Alfa / Lancia | IAW `1ABB.92`, `1ABG.81`, `1AF.13`, `1AF.15`, `1AF.17`, `1AF.1G` |
| Peugeot / Citroën | IAW `1AP.20`, `.40`, `.41`, `.43`, `.80`, `.81`, `.83`, `.90` |

Close relatives store the same kind of record in an external SPI/parallel chip instead of the TMS374
EEPROM — e.g. IAW `49F.B4` (95040), IAW `48P2.xx` (29F400 full flash on the Peugeot 1.1) — and are
read the same way once you know where the record is.

## Memory layout

The dump is the TMS374's **256‑byte internal EEPROM**. Across the corpus the structure is consistent:

| Offset | Field |
|---|---|
| `0x00`–`0x06` | header / status bytes |
| `0x07`–`0x0E` | **immobiliser code** — 8 characters `0`–`9`/`A`–`Z` on the `1AP` set and `1AF.1G` (the per‑ECU secret) |
| `0x0F`–`0x60` | mostly `0x00`; adaptation / counters |
| `0x61` | family marker byte sequence (`!1AQw`‑style) shared across the `1AP` set |
| … | zero‑filled to the end, with a checksum region |

Entropy is low (~2 bits/byte): a short code plus a lot of zero padding. A **programmed** ECU has the
8‑character field populated; a **virgin / prepared‑for‑coding** ECU has that field and its checksum
reset so the dealer tool (or the car on first start) can learn a key.

> The real code values are not reproduced here — they are specific to individual vehicles and kept in
> the local reference set, the same way KAT keeps real key‑fob IDs out of the RF pages.

## Reading a dump

1. Confirm the device: a 256‑byte image, low entropy, with a readable 8‑character field at `0x07`.
2. The code field is mirrored/checksummed; on the `1AP` set the `0x61` marker confirms the family and
   byte order.
3. "Repair / prepare for coding" rewrites the code field and its checksum to a learnable state; it
   does not touch the engine maps elsewhere in the ECU (those live in separate flash).
4. TMW's *PSA Calculator* repairs a `1AP` differently: it overwrites the whole 256‑byte EEPROM with a
   complete donor image, the donor's code included. The two images it carries, and the 34 bytes in
   which they differ, are described on the [PSA page](ecu_psa.md).

## Corpus (local reference)

Dumps in the reference set, with their on‑chip code field located (values redacted):

`IAW_1AP.40`, `IAW_1AP.80`, `IAW_1AP.81`, `IAW_1ABB.92`, `IAW_1AF.1G`, `IAW_TMS374` (all TMS374; the
8‑character code at `0x07`–`0x0E` on the `1AP` dumps and `1AF.1G`, the `0x61` marker on the `1AP`
dumps only; `IAW_1AP.40` and `IAW_1AP.80` are the PSA Calculator's two repair images);
`IAW_49F.B4` (95040, code at `0x36`); `IAW_48P2.xx` (29F400, full‑flash image).
Board photographs that identify the MCU and the memory on each ECU are held locally (they are TMW
*Immo Tool* material and are not republished).
