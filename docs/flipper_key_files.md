# Flipper key files

A Flipper SubGhz **key file** is a `.sub` that holds a *decoded* capture instead of RAW pulses:
`Protocol`, `Bit` and `Key` plus the protocol's own lines (`Key_2`, `Type`, `CRC`, `Manufacture`,
`Seed`, `Serial`, `Btn`, `Cnt` …). ProtoPirate and Flipper-ARF save every decode that way (each
protocol's `serialize`), and load it back with the protocol's `deserialize` to show or transmit it.

```text
Filetype: Flipper SubGhz Key File
Version: 1
Frequency: 434390000
Preset: FuriHalSubGhzPresetOok650Async
Protocol: VAG
Bit: 80
Key: C0 9E BD 07 92 CA 7F C3
Key_2: 00 00 00 00 00 00 A7 2B
Type: 11011
```

KAT loads and saves key files for every car protocol it registers, following the reference each
module is ported from (the module's doc comment names it). A `.sub` with `RAW_Data` lines (or
`Protocol: RAW`) is a RAW capture and is decoded from its pulses as before.

## Loading

Key files load wherever RAW `.sub` files do: the `:load` browser, the import folder offered at
startup, and `kat --decode <files|dirs>` (headless; key decodes are printed with source `key`).

- **Protocol names.** KAT's own names and display names (`Suzuki V0`, `Star Line [Cenmax]` …),
  every `info.rs` alias (ProtoPirate's catalog aliases such as `StarLine`, `KIA/HYU V3`, `VW`, and
  retired names such as `Land Rover V0`, `PSA2`), plus the names other firmwares write
  (`flipper_key::registry_name`): Flipper-ARF's `KIA/HYU V0` … `KIA/HYU V6`, `VAG GROUP`,
  `Porsche AG`, `MazdaSiemens`, `Chrysler`; doctor-firmware's `Toyota Lexus` and the legacy
  aliases its `registry.c` still loads (`Korea V0` … `Korea V7`, `Hitag-2`, `Renault`, `ScherKhan`,
  `HondaCVE`, `Passat`, `VAG v1`/`v2`, `Audi` → VAG). Names are matched case-insensitively.
- **What loading does** is the reference's `deserialize`: the bit count is checked, the protocol's
  own checks run (CRC, KeeLoq / AUT64 / AES / TEA decryption with the keystore …) and a key that
  fails them is refused, as on the Flipper. Where the reference rebuilds the key from edited
  `Serial` / `Btn` / `Cnt` lines, KAT does too; a file whose lines agree with its key loads exactly
  as written.
- **The capture** carries the protocol, the decoded fields and the file's frequency and preset
  (custom CC1101 registers included), and **no raw pairs** (the detail panel shows
  `Raw Data: None (key file)`). It is EncoderCapable when the protocol's encoder can transmit it.
- A key file whose protocol KAT does not know, or that does not validate, is not imported; the
  status line says why.

## Using a loaded key

Without raw pairs there is nothing to replay, so the signal menu hides **Replay**, the RAW
**Export .sub** and **Timing analysis** for such captures (`.fob` files saved without raw pairs
behave the same). Everything that re-encodes works from the fields: **TX Lock / Unlock / Trunk /
Panic** for car keyfobs and **Send next code** for gate / alarm remotes, with the counter step and
button mapping of each protocol's encoder (as for any decoded capture). Sessions and `.fob`
exports keep the capture; a `.fob` without raw pairs re-imports as EncoderCapable when its encoder
still produces a frame from the stored fields.

## Saving

**Export Flipper key (.sub)** in the signal menu (shown only when the capture's protocol writes key
files) saves `<name>.sub` in the export directory, as the reference's `serialize` would:

- header, `Frequency`, `Preset` (a custom preset adds `Custom_preset_module` /
  `Custom_preset_data` right after it), then `Protocol`, `Bit`, `Key` (8 bytes, the reference's
  `generic.data`) and the protocol's lines in the order the reference writes them;
- the `Protocol` name is the one the ported reference writes: ProtoPirate's names for its
  protocols (Suzuki V0 / Honda V0 / Mitsubishi V0 frames are `Kia V0` with `Type: 2/3/4`, as in
  ProtoPirate), Flipper-ARF's for its own (`MazdaSiemens`, `Porsche AG`, `KeeLoq` +
  `Manufacture: …`), doctor-firmware's for Datsun Lada.

A Flipper, ProtoPirate or Flipper-ARF loads these files like its own saves. (ProtoPirate's own
`.psf` saves carry the same lines but put `Protocol` / `Bit` / `Key` before `Frequency` /
`Preset`; the Flipper's reader does not care about the order.)

## Per protocol

`Protocol` and `Bit` as written, then the lines after `Key` in the order KAT writes them (the
reference's `serialize` order; numbers decimal unless noted). "Word" means `Key` is written as
one 16-digit hex word, as that reference does; otherwise the Flipper's spaced bytes. Each module's
doc comment has the details.

| KAT protocol (reference) | `Protocol` / `Bit` | Lines after `Key` | Loading notes |
|---|---|---|---|
| Kia V0, Suzuki V0, Honda V0, Mitsubishi V0 61-bit (PP) | `Kia V0` / 61, 64, 72 | Serial, Btn, Cnt, Type (1 Kia, 2 Suzuki, 3 Honda, 4 Mitsubishi) | type from `Type`, a legacy name (`Suzuki`, `Honda V0`, `Mitsu V0` …) or the bit count; edited Serial/Btn/Cnt rebuild the key (CRC recomputed) |
| Kia V1 (PP) | `Kia V1` / 57 | Serial, Btn, Cnt | edited fields rebuild the key |
| Kia V2 (PP) | `Kia V2` / 53 | Serial, Btn, Cnt, CRC, RawCnt | Flipper-ARF's Key-only files load |
| Kia V3/V4 (PP) | `Kia V3` or `Kia V4` / 68, word | Serial, Btn, Cnt, Encrypted, Decrypted, KIAVersion | must pass the KeeLoq check; Bit 64 or 68; no `CRC` line (KAT does not keep the per-repeat CRC nibble) |
| Kia V5 (PP) | `Kia V5` / 64–67 | Serial, Btn, Cnt, CRC, DataHi, DataLo, YekHi, YekLo | |
| Kia V6 (PP) | `Kia V6` / 144, word | Serial, Btn, Cnt (32 bit), Key_2, Key_3, Key_4, Fx | parts 2/3 are re-encrypted from the fields; the older ARF layout (Key_2/Key_3 as hex) loads |
| Kia V7 (PP) | `Kia V7` / 64 | Serial, Btn, Cnt, Repeat, Disp | header 0x4C + CRC checked; fields rebuild the key |
| VAG (PP) | `VAG` / 80 | Key2, Type, then Serial, Btn, Cnt, KeyIdx when decrypted | `Key_2` accepted; decrypted as on air |
| Ford V0 (PP) | `Ford V0` / 64 | Checksum, CRC, Serial, Btn, Cnt | key2 rebuilt from Checksum/CRC |
| Ford V1 (PP) | `Ford V1` / 136 | Serial, Btn, Cnt, Key_2, Key_3 | also the C's older byte-swapped layout |
| Ford V2 (PP) | `Ford V2` / 104 | Serial, Btn, Cnt, Tail31, TailRaw | |
| Ford V3 (PP) | `Ford V3` / 104 | Raw, Serial, Btn, Cnt, Variant (0 EU, 1 US) | decode-only: listed and exported, not transmitted |
| Honda Static (PP) | `Honda Static` / 64 | Serial, Btn, Cnt, Checksum | Checksum written only for checksum-verified frames (KAT's `crc_valid`; no reference reads it back) |
| Honda V1 (PP) | `Honda V1` / 68 | Serial, Btn, Cnt, Crc, Key_2, Disp | |
| Honda V2 (PP) | `Honda V2` / 81 | Serial, Btn, BtnSig, Cnt, Check, Tail, ExtraBit | `Land Rover V0` files load |
| Subaru (PP) | `Subaru` / 64 | Serial, Btn, Cnt, Disp | |
| Fiat V0 (PP) | `Fiat V0` / 71 | Serial, Btn, Cnt, EndByte, Disp | Bit 64 or 71; button from EndByte, else `CRC`, else Btn |
| Fiat V1 (PP) | `Fiat V1` / 102 | Serial, Btn, Cnt, Type, Hop, XOR, Tail Bits, Raw, Hitag2 Key + Epoch (when a key verifies), Disp | Raw validated; ARF files (no Type) load as variant B |
| Fiat V2 (PP) | `Fiat V2` / 112 | Serial, Btn, Cnt, Hop, Raw | Raw required |
| Scher-Khan (PP) | `Scher-Khan` / 35–64, word | Serial, Btn, Cnt | frames over 64 bits are not written (the C keeps only 64) |
| Star Line (PP) | `Star Line` / 64, word | Serial, Btn, Cnt, Manufacture | Manufacture restricts the key search, as in the C |
| PSA (PP) | `PSA` / 128, word | Key_2 (word), ValidationField; when decrypted Serial, Btn, Cnt, Type, CRC, Seed | a TEA frame with its Serial loads recovered; ARF's `PSA GROUP` files load |
| Chrysler V0 (PP) | `Chrysler V0` / 80 | Key_2, Plain_A or Plain_B, Serial, Btn, Cnt | |
| Mazda V0 (PP) | `Mazda V0` / 64 | Checksum, Serial, Btn, Cnt, Disp | |
| Renault V0 (PP) | `Renault V0` / 82 | Serial, Btn, Cnt, Key2, Disp | checksum required |
| Renault V1 (PP) | `Renault V1` / 88 | Key_2, Recovered, Seed, Serial, Btn, Cnt, Hop | the seed search reruns on load; `Bit: 82` keys go to Renault V1 (ARF) |
| Sheriff CFM (ARF) | `Sheriff CFM` / 64 | Model, Serial, Btn, Cnt | must decrypt |
| KeeLoq (ARF) | `KeeLoq` / 64 | Seed (BFT), Manufacture | the named manufacturer's keys must validate the frame (TX then uses that manufacturer, as ARF); `Unknown` / no line: identified as on air |
| Mazda Siemens (ARF) | `MazdaSiemens` / 64 | Serial, Btn, Cnt | |
| Mitsubishi V0 96-bit (ARF) | `Mitsubishi V0` / 96 | Serial, Cnt, Btn, Raw_Frame | Key is all zero, as ARF writes it; 61-bit keys load as Kia V0 type 4 |
| Porsche Cayenne (ARF) | `Porsche AG` / 64 | Serial, Btn, Counter, Cnt | |
| Toyota (ARF) | `Toyota` / 96 | Key_2, Btn, ToyotaBtn, Disp | Key_2 always written (ARF writes it only for a zero tail and cannot reload its other files) |
| BMW (doctor) | `BMW` / 128 | Serial, Btn, Cnt, ValidationField, TE, Key_2, Key_3 | |
| Mazda V2 (ARF) | `Mazda V2` / 63 | Hdr, Disp | |
| Mercedes V0 (doctor) | `Mercedes V0` / 112 | Serial, Btn, Cnt, TE, Key_2 | |
| Mercedes V1 (doctor) | `Mercedes V1` / 122 | Serial, Btn, Cnt, TE, Extra, Key_2, Key_3 | |
| Opel V1 (doctor) | `Opel V1` / 64 | Serial, Btn, Cnt, XteaKey, Disp, Burst | decode-only |
| Renault V1 (ARF) | `Renault V1` / 82 | Key2, Preamble, Serial, Btn, Cnt, Recovered | |
| GM (ARF) | `GM` / 112 | Data, Serial, Btn, Cnt | Data required, both checksums checked |
| Datsun Lada (doctor) | `Datsun Lada` / 88 | Serial, Btn, Cnt, Raw | Raw + XOR checksum required |

BMW, Mercedes V0/V1 and Opel V1 are written with doctor-firmware's lines: Flipper-ARF's copy of
the same file saves only `Key` (the first 64 bits), so its files cannot be reloaded completely;
ARF still reads `Bit` / `Key` from doctor's.

Differences from the references, all deliberate and documented in the modules: KAT always writes
the full Flipper preset name (ProtoPirate writes `FM476` / `AM650` for Kia V3/V4, Kia V6, PSA,
Star Line and Scher-Khan, which the stock SubGhz app cannot load); Kia V3/V4 has no `CRC` line;
Honda Static writes `Checksum` only for verified frames; Toyota always writes `Key_2`; Chrysler V0
writes its lines in order (ProtoPirate's serializer overwrites the start of its own file); Renault
V1 is written as ProtoPirate's file reads after its seed search.

Checked against the reference C (host builds of ProtoPirate, Flipper-ARF and doctor-firmware run
on the same RAW files): the lines KAT writes for the IMPORTS captures and for encoder-made frames
are identical to the reference's `serialize` output apart from the differences above and from
fields where KAT's decoder already differs from the C (Porsche Cayenne's full 16-bit counter
recovery, Ford V3's US frames, KeeLoq DoorHan's learning type). Not checkable that way: VAG
AUT64 frames (the ProtoPirate build has no AUT64 keystore; their XTEA frames match) and Mazda
Siemens / Mitsubishi V0, whose ARF decoders do not decode their own encoders' frames (ARF's
encoder loads KAT's Mazda Siemens files unchanged; its Mitsubishi V0 encoder overflows its
upload buffer on these keys).

## Tests

- every IMPORTS key file loads with the fields the references show (and, where a RAW capture of
  the same remote exists, the fields of that capture's decode);
- every decode of every IMPORTS RAW capture survives `to_flipper_key` → file text → parse →
  `from_flipper_key` unchanged (every field), and the reloaded decode transmits a frame that
  decodes again (`protocols::tests::imports_decodes_round_trip_through_key_files`);
- each protocol module round-trips encoder-made frames, loads a file written the way its
  reference writes it, and checks the exact lines it writes.
