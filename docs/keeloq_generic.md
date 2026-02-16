# KeeLoq Generic Fallback

**Rust module:** `src/protocols/keeloq_generic.rs`  
**Helper:** `src/protocols/keeloq_common.rs` (decrypt, normal learning)

## Overview

When no known protocol decodes a captured signal, KAT tries to decode it as **KeeLoq** using **every** manufacturer key in the embedded keystore, **regardless of frequency**. All decryption is done via the **keeloq_common** helper (no protocol-specific decrypt code). Two air formats are tried (in order):

1. **Kia V3/V4 format** — 68 bits, 400/800 µs PWM  
2. **Star Line format** — 64 bits, 250/500 µs PWM  

Bit collection reuses the existing Kia V3/V4 and Star Line state machines; decryption and validation use only **keeloq_common** (`keeloq_decrypt`, `keeloq_normal_learning`, `reverse_key`, `reverse8`). Each key is tried in **both byte orders** (as stored in the keystore, and byte-swapped) so that either big-endian or little-endian key sources can match.

## When it runs

- After all registered protocol decoders have been tried (both normal and inverted polarity).
- Only if the signal was not decoded as Kia V3/V4, Star Line, or any other protocol.

## Display

On first successful decrypt with a given key, the capture is shown in the list with:

- **Protocol:** `Keeloq (keystore name)` — e.g. `Keeloq (Alligator)`, `Keeloq (Pandora_PRO)`, `Keeloq (KIA)`.
- Serial, button, counter, CRC, and data as for other KeeLoq decodes.

## Keystore

Uses **all** KeeLoq manufacturer keys from the embedded blob (types 0, 1, 2, 10, 20). Key names come from `keystore::KEY_ENTRY_NAMES` and are used only for the displayed protocol label. Keys are stored in the blob as 8 bytes little-endian; the resulting u64 matches reference/Pandora hex (MSB-first notation).

## Encoding

Captures decoded as **Keeloq (*name*)** are decode-only in this path. Retransmit (Lock/Unlock/Trunk/Panic) is supported only for the named protocols **Kia V3/V4** and **Star Line**; to encode, the signal would need to have been decoded by those decoders (or you would need a separate encoder mapping for generic KeeLoq).

## Flow

1. Get `keeloq_mf_keys_with_names()` from the keystore.
2. If frequency is 315 or 433.92 MHz: for each polarity, run `collect_kia_v3_v4_bits`; for each key, run Kia V3/V4 byte layout + `keeloq_decrypt`; on validation match, return `("Keeloq (name)", decoded)`.
3. If frequency is 433.92 MHz: for each polarity, run `collect_star_line_bits`; for each key, try simple then normal learning with `keeloq_decrypt` / `keeloq_normal_learning`; on match, return `("Keeloq (name)", decoded)`.
4. If no key validates, return `None` (capture stays unknown or is shown as unknown in research mode).

Implementations are aligned with the ProtoPirate reference; keeloq_common matches `REFERENCES/ProtoPirate/protocols/keeloq_common.c`.
