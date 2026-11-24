---
layout: default
---

# Mercedes Temic (Sprinter / Vito)

[ECU & immobiliser reference](ecu.md) › Mercedes Temic

The Mercedes **Sprinter** and **Vito** vans of the late‑1990s / 2000s use **Temic** engine ECUs built
on Motorola **HC05 / HC08** microcontrollers, with the immobiliser record in a small serial EEPROM
(24C04) or in the MCU, depending on the plug/variant. A mismatched immobiliser here presents as the
classic **"start error"** — the engine cranks but will not run — which is why these dumps are labelled
that way and make a good reference for a *fault state* rather than a clean programmed one.

A separate Mercedes architecture, the **C/E‑class infra‑red** system, is two parts: a **Lucas engine
ECU on a Motorola HC11E9** plus an **immo box on a Motorola HC05 (Siemens)**; repairing it is a
two‑step job across both dumps.

## Variants in the corpus

| Dump | Vehicle / part | Memory | Size | State |
|---|---|---|---|---|
| `5P08C3_Sprinter` | Sprinter, 5‑plug ECU | ST 5P08C3 (95P08) | 1 KB | start‑error |
| `24C04_05_Vito` | Vito, 5‑plug | 24C04 | 512 B | start‑error |
| `24C04_08_Vito` | Vito | 24C04 | 512 B | start‑error |
| `CE_HC05_immo` | C/E infra‑red immo box | HC05 internal | 256 B | programmed |
| `CE_HC11E9_ecu` | C/E Lucas engine ECU | HC11E9 internal | 512 B | programmed |

## What a "start error" dump looks like

The Vito/Sprinter dumps are **programmed but locked**: entropy is moderate (~4.6 bits/byte on the
`24C04_05` Vito, ~4.8 on the Sprinter), with ASCII part/calibration strings (`184.21`, `083.10`,
`188.23`) and a long data record — not a blank chip. The fault is a *pairing* mismatch between the
ECU and the immobiliser, not missing data. "Prepare for coding / unlock" rewrites the record so the
ECU will accept a fresh pairing; the engine calibration is untouched.

The C/E infra‑red pair shows the two‑sided design: the HC05 immo box holds the shared secret (a
code‑like block appears around `0x8E` in `CE_HC05_immo`), and the HC11E9 ECU holds its counterpart —
both must line up for the car to start.

## Reading a dump

1. Identify the memory from the plug/variant (1‑plug Sprinter = 24C02; 5‑plug = 5P08C3/95P08; Vito
   5‑plug = 24C04).
2. A "start error" image is a full, programmed record — look for the pairing/counter region, not a
   blank. Compare against a known‑good same‑variant dump to see which bytes carry the lock.
3. For the C/E system, read **both** the HC11E9 ECU and the HC05 immo box; the repair spans the pair.

> Part/calibration strings are shown above as they identify the ECU; the vehicle‑specific secret and
> any VIN are kept in the local reference set, not published here.
