---
layout: default
---

# Research reports

KAT's decoders come from reading real signals and, where it helps, the firmware of the module that
receives them. When KAT turns out to be the first to document how a system works — or finds that it
is weak — the finding is written up as a report. Each one describes the radio, the frame, the cipher
and what the weakness means, and names the decoder and encoder in KAT that implement it.

Real per‑vehicle keys, VINs and manufacturer key material are kept **out** of these reports and out
of KAT, the same way the protocol pages keep real key IDs out. KAT ships no car keys: the decoders
that need one read it from a file the owner supplies.

| Report | Vehicles | What it finds | Protocol |
|---|---|---|---|
| [Ford V1 (JLR): a 32‑bit key](reports/jlr.pdf) | Jaguar / Land Rover keyless fobs, ~2015–2018 | The remote rolling code is enciphered under a 16‑byte key that carries only **32 bits** of entropy (it has the form X‖~X‖~X‖X). Two captured button presses recover the car's key in about a minute, after which any future code can be generated. | [Ford V1 (JLR)](ford_v1_jlr.md) |
| [PSA legacy two‑button remotes: a global key](reports/psa-older-code.pdf) | Peugeot / Citroën, ~2001–2012 | The remote's frame is enciphered with a block cipher whose 16‑byte key is **global to the whole platform** — no per‑vehicle secret. A single captured press is enough to read the remote's identity and counter and generate its next valid code. | [PSA](psa.md) |
| [Subaru V1: a plaintext rolling code](reports/subaru-v1.pdf) | Subaru US 2012–2019 remote‑head key | The 56‑bit frame carries the identity, a **plaintext** 20‑bit counter and a byte‑XOR check, with no cryptography. A captured press is replayable with the counter advanced. | [Subaru V1](subaru_v1.md) |

Each report is a PDF, linked above. The same finding, with the frame layout and KAT's handling, is on
the protocol's own page. The proof of concept is the open‑source KAT decoder and encoder for each
protocol — there is nothing to download separately.

## How KAT handles what it finds

A finding like these is a double‑edged thing: documenting it helps owners, locksmiths and researchers
understand a real exposure, and it also describes an attack. KAT's line is the same one it draws for
every protocol it reads:

- The **method** is published — the radio, the frame, the cipher, the weakness.
- The **secrets are not** — real keys, VINs and manufacturer key tables stay out of the report and out
  of the tool. KAT ships none; a decoder that needs a key reads it from a file the owner provides, from
  their own vehicle's module.

This is a research reference, not a procedure for defeating a vehicle's security.
