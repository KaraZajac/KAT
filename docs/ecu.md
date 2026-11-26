---
layout: default
---

# ECU & Immobiliser Reference

KAT's protocol pages document car **key‑fob RF** — the signal a remote sends over the air. This
section is the companion on the **wired side of the car**: the engine ECU and the immobiliser data
it holds. Where the RF pages answer "what does this remote transmit and how is it checked," the ECU
pages answer "how does a given engine‑control unit store its immobiliser secret, how is it laid out
in memory, and how do you read that structure from a dump."

It is a **research reference**, in the same spirit as the rest of these docs: it describes how these
systems are built and how a memory dump is analysed. It is **not** a how‑to for defeating a car's
security — concrete per‑vehicle secrets (codes, PINs, VINs) from the reference dumps are kept out of
the public pages, the same way real key IDs are kept out of the RF pages.

## How an immobiliser fits together

A modern (1995‑onward) car ties three parts together so the engine only runs with a valid key:

- the **transponder** in the key head (a passive RFID chip: Megamos, Hitag2, Texas 4C/4D, Philips),
- the **immobiliser / body unit** (a BCM, BSI, or a stand‑alone immo box) that challenges the key,
- the **engine ECU**, which will only inject fuel / fire the coils once the immobiliser tells it the
  key is authentic.

The ECU and the immobiliser share a secret — variously called the **ISN** (Immobiliser Serial
Number), **PIN**, or **secret code** — stored in non‑volatile memory on each side. When a mechanic
replaces an ECU, clears a crash flag, or codes a new key, that stored secret (and a set of
checksums) is what has to line up. The dump of that memory is what this section documents.

## Where the data lives: memory devices

An ECU's immobiliser data is almost always in a small external serial‑memory chip beside the MCU,
or in the MCU's own on‑chip EEPROM. The families you meet:

| Family | Bus | Typical sizes | Seen on |
|---|---|---|---|
| **24Cxx** (24C01…24C16) | I²C | 128 B – 2 KB | Bosch MA/MP/ME, Mercedes Vito, VAG TDI |
| **93Cxx** (93C46/56/76/86) | Microwire | 128 B – 2 KB | BMW EWS, Rover MEMS, Opel Simtec, Peugeot MP5 |
| **95xxx** (95040/080/160/320/640) | SPI | 512 B – 8 KB | Magneti Marelli IAW, Sagem, EDC15/16, BSI |
| **29Fxxx** (29F200/400) | Parallel flash | 256 KB – 512 KB | Siemens MS41, IAW 48P2 — full ECU firmware |
| **MCU‑internal** | — | varies | TMS374/370, Motorola HC05/HC08/HC11, ST, NEC |

A memory *type* in a dump's name (e.g. `…_24C02`, `…_95160`, `…_TMS374`) tells you the bus width,
the likely size, and where in the chip the immobiliser record tends to sit.

## Reading a dump: state signatures

Before decoding fields, the overall shape of a dump tells you its state:

- **Erased / virgin** — almost all `0xFF`. A blank or never‑programmed chip (e.g. an erased 24C02).
- **Default / no‑key** — large runs of `0x55`. Several Bosch/Kia immo EEPROMs ship or reset to a
  `0x55` fill meaning "no key learned."
- **Programmed** — a low‑entropy record with a short, readable code field and mirror/checksum bytes.
- **Fault state** — e.g. the Mercedes Vito/Sprinter "start error" dumps: a programmed but
  mismatched/locked immobiliser, useful as a reference for what a broken pairing looks like.

Entropy per byte, the `0xFF`/`0x00`/`0x55` fill ratio, and the ASCII‑readable regions are the first
three things to measure. (KAT's dump tooling reports exactly these.)

## Data structure, in general terms

Across the families, the immobiliser record follows a recurring shape:

- a short **code / ISN field** (often 8 ASCII‑range characters) at a fixed per‑family offset,
- **mirror copies** of that field elsewhere in the record (redundancy the ECU cross‑checks), or, on
  some Bosch Motronic units, every value stored three times in a row for two‑out‑of‑three voting,
- a **key / adaptation table** with a recognisable default fill, and
- one or more **checksums** over the record.

"Remove immo" / "virgin" operations zero the record's key table and rewrite the code field and its
checksums; a before/after pair of dumps shows exactly which bytes carry the immobiliser state. The
per‑family offsets and checksum rules are documented on the family pages (without the real codes).

### Locating the record by its checksum

The immobiliser entry often isn't at a fixed address, but it carries its own self‑check that lets you
find it. A common form (Bosch I²C ECUs, and visible in several dumps here) stores a value immediately
followed by its **one's‑complement**, so the pair sums to `0xFFFF`:

```
  code  complement
  0B 04  F4 FB      →  0B04 + F4FB = FFFF   ✓  (this is the immo entry)
```

Scanning a dump for adjacent `word, ~word` pairs finds the record wherever it sits. Clearing the
immobiliser then means zeroing the value while keeping the sum valid (`00 00  FF FF`), so the ECU
reads a "no code" entry whose checksum still passes. The same idea recurs as mirror copies (the code
repeated at several offsets) and as byte sums over the record; each family page gives the exact rule.

## Operations (what a dump tool does)

The repair/locksmith tooling in this space performs a small set of operations, which is a useful way
to classify each ECU:

- **Calculate PIN** — read the stored secret out of the dump (BSI, EDC15/16, Sagem).
- **Program PIN** — write a new secret into the record, in every copy the ECU keeps.
- **Decode / encode** — undo a storage coding (a byte complement on some PSA BSI dumps) before
  reading, and redo it after editing.
- **Repair / Prepare for coding** — fix checksums / clear a learned‑key table so a new key can be
  taught by the dealer tool.
- **Remove Immo / Clear Immo** — rewrite the record to a state where the ECU no longer demands a key
  (used in engine‑swap and bench repair; documented here as *structure*, not as a procedure).

## Coverage

The ECU families documented so far, grouped by make (the ones with their own page are linked):

ECU_FAMILIES_TABLE

Each family gets its own page with the memory map, the immobiliser record layout, and a worked
example from the corpus. Started with:

- [PSA engine ECUs and BSI](ecu_psa.md) — where EDC15, EDC16, SID803 and ME7.4.4 keep the PIN, the
  Motronic MA/MP immobiliser word, the IAW 1AP repair images and the BSI dump coding, recovered from
  TMW's *PSA Calculator*
- [IAW TMS374](ecu_iaw_tms374.md) — Magneti Marelli, TI TMS374 internal EEPROM (Fiat/Alfa/Peugeot)
- [Mercedes Temic](ecu_mercedes_temic.md) — Sprinter/Vito "start error", Motorola HC05/HC08
- [BMW EWS & MS41](ecu_bmw.md) — 318 TDS (93C46) and the Siemens MS41.0 engine ECU (29F200)

For identifying the engine ECU itself — which Bosch / Siemens / Marelli / Delphi family a car carries,
its microcontroller and memory, and where in it the PIN sits — see the
[engine-ECU families](ecu_catalog.md) catalog.

## Sources

The corpus is grounded in a reference set of real ECU dumps and in the public catalogue and
"solution" notes of TMW electronic's *Immo Tool* (a commercial locksmith/repair product;
`tmw.netlook.pl`), which documents its coverage and methods for "educational purposes." The PSA page
also draws on TMW's *PSA Calculator* 16.04.07, analysed statically and never run. KAT does not
redistribute these tools or their board photographs; the pages here describe the memory structures and
analysis methodology in KAT's own words, and the real per‑vehicle codes, PINs and VINs from the
reference dumps are kept out of the published pages. This is a reference work for researchers, not a
procedure for defeating a vehicle's security.
