---
layout: default
---

# Engine-ECU families

[ECU & immobiliser reference](ecu.md) › Engine ECUs

The immobiliser PIN that pairs an engine ECU to the body module lives inside the ECU — in its serial
EEPROM on the older families, or in the microcontroller's own flash on the newer ones. Knowing the ECU
family tells you where the PIN is and how it is read. This page catalogs the engine-ECU families by
supplier, with their microcontroller, memory and where the PIN sits; a family links to KAT's PIN page
where KAT has documented it.

The microcontroller, memory and PIN-storage facts are the established characteristics of each family.
The **makes** and counts are what a field reference corpus of about 434 ECU reads (diesel and petrol
tuning dumps, kept locally — no dump, VIN or PIN is published) actually showed, so the list reflects
real cars rather than a datasheet. KAT reads the ECU to document structure, not to defeat a vehicle's
security: the PIN pages give the storage location and encoding, never a real key.

ECU_CATALOG_TABLE

## How the families relate

- **Bosch** split diesel into the EEPROM era (**EDC15**, **EDC16** — PIN in a 95xxx serial chip) and
  the TriCore era (**EDC17** — PIN inside the microcontroller). Petrol went the same way: **ME7** (PIN
  in EEPROM) then **MED9 / MED17 / MEV17** (TriCore).
- **Siemens / Continental** diesel is **SID** (SID801/803/805/807, SID30x) plus the VAG-specific
  **PPD** (pump-düse) and **Simos PCR** (common rail) — all TriCore.
- **Magneti Marelli** is the Fiat group: **MJD** diesel and **IAW** petrol.
- **Delphi DCM** is the common-rail ECU on many Renault and Korean diesels.

KAT has documented the PIN storage for the PSA EDC15, EDC16, ME7.4.4 and SID803 families
([PSA engine ECUs and BSI](ecu_psa.md)) and the Marelli IAW TMS374 immobiliser
([IAW TMS374](ecu_iaw_tms374.md)). The others are cataloged here for identification; their PIN handling
is model-specific and read by BDM, boot-mode or OBD.
