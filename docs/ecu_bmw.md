---
layout: default
---

# BMW (EWS & MS41.0)

[ECU & immobiliser reference](ecu.md) › BMW

Two different BMW points in the corpus: a small **diesel ECU with an EWS‑linked immobiliser** in a
93C46, and a full **Siemens MS41.0 petrol DME** image in 29F200 flash.

## 318 TDS — Bosch, EWS (93C46)

The E36 **318 TDS** diesel runs a Bosch ECU (`0 281 001 243`) whose immobiliser data sits in a
**93C46** microwire EEPROM (`B58323`), 128 bytes. It is tied to BMW's **EWS** immobiliser: the ECU
only fuels once EWS authorises it, and the 93C46 holds the ECU's side of that link.

- **Size / shape:** 128 B, ~59% `0xFF` — a small programmed record in a mostly‑erased chip.
- **Record:** the EWS/ISN bytes with a checksum; "repair" fixes the record so the ECU pairs with the
  car's EWS (or runs standalone on the bench).

## MS41.0 — Siemens DME (29F200)

The 2.0–2.8 L petrol **MS41.0 DME** is a full engine‑management image in **29F200** parallel flash,
256 KB. Here the immobiliser state is embedded in the firmware image rather than a tiny EEPROM:

- **Size / shape:** 256 KB, ~6 bits/byte entropy — real code and maps, not a bare immo record.
- The EWS‑sync region is a small part of the image; "repair" clears it so a second‑hand DME can be
  adapted, leaving the engine calibration intact.

## Reading a dump

1. 93C46 (128 B, mostly `0xFF`) → small EWS record; locate the ISN + checksum.
2. 29F200 (256 KB) → full DME firmware; the immobiliser/EWS‑sync bytes are a small region inside it,
   not the whole file. Work from a map of the MS41 image rather than scanning blind.

> BMW EWS/ISN values are vehicle‑specific and kept in the local reference set, not published. Board
> photographs identifying the chips are held locally (TMW *Immo Tool* material, not republished).
