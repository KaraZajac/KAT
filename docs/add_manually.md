# Add Manually — new gate / garage remotes

`:new` creates a brand-new virtual remote of a chosen type — a fresh random serial with the type's
default button, counter, manufacturer key and seed — that KAT can transmit, typically to pair it with
a gate / garage receiver in learning mode. It ports the SubGHz app's **Add Manually** of Flipper-ARF
(`applications/main/subghz/scenes/subghz_scene_set_type.c`, `helpers/subghz_gen_info.c`,
`helpers/subghz_txrx_create_protocol_key.c` and the protocols' `*_create_data`) and of
doctor-firmware (the same files; its list is a superset). Code: `src/protocols/add_manually.rs`
(types, parameters, key files), `src/app/add_manually.rs` and `src/ui/add_manually.rs` (picker).

## Using it

- `:new` opens the type picker: the 101 types in the references' menu order, under the headings
  Rolling code, KeeLoq manufacturers (KL:), Nice, CAME, Fixed code and Security+. Each row shows the
  protocol (KeeLoq: the manufacturer key), frequency and preset; doctor-firmware-only types say
  "doctor", types KAT cannot create "✗ n/a" (the reason is shown under the list).
- Type to filter (all words must match the name), Up/Down/PgUp/PgDn to select.
- **Enter** creates the selected type with fresh random values (doctor-firmware's "Add Manually").
- **Tab** first shows its fields — Key (fixed codes), Serial, Button, Counter, Seed, as Flipper-ARF's
  `set_key` / `set_serial` / `set_button` / `set_counter` / `set_seed` screens — prefilled with the
  drawn values, as hex of the C field's width; Enter creates it, Esc goes back. (Security+ 1.0 has
  no fields: its fixed code is drawn when it is created, as in the C.)
- `:new <words>` creates the one available type the words name directly (`:new princeton 433`),
  else opens the picker filtered by them.

The new remote is the type's Flipper key file loaded like any key file (`ProtocolRegistry::decode_key`
→ a capture without raw pairs, at the type's frequency and preset). Its **File** line says
`Add Manually: <type>`. Its signal menu offers **Send next code**, **Send button…** (the remote's
native buttons, below), **Export .fob** and **Export Flipper key**, so the same remote can also be
loaded on a Flipper. Send next code transmits the code after the stored one, and once it went out the
capture keeps it (counter, key, button), so every press sends a new code and the remote stays in step
with the receiver it was paired to — as Flipper-ARF, doctor-firmware and ProtoPirate keep the last code
they sent ([transmit.md](transmit.md#after-a-transmission)). The session autosave keeps the counter
across restarts; export the key file again after using the remote in KAT if a Flipper is to go on from
there.

## How a key is made

1. **Parameters** (`subghz_scene_set_type_fill_generation_infos`): `r = rand()`, then GangQi's and
   Marantec's own `rand()` draws (made for every type, so the draws come in the C's order), then the
   type's preset, frequency and fields from `r` with the C's masks (table below). `rand()` is newlib's
   (0..0x7FFFFFFF): KAT draws 31-bit values from a SplitMix64 generator seeded from `/dev/urandom`
   (the time and process id if it cannot be read); tests inject a seed or the C's exact values.
2. **Key file** (`subghz_scene_set_type_generate_protocol_from_infos`):
   - fixed codes (`GenData`): `Protocol`, `Bit`, `Key` (Princeton / ZKTeco also `TE` and
     `Guard_time: 30`, the lines its decoder writes);
   - Security+ 1.0: `rand()` until `subghz_protocol_secplus_v1_check_fixed` accepts it (id1 ≠ 0) as
     the fixed code, rolling code 0xE6000000;
   - the rest through the protocol's `create_data`, the first code computed as the C does:
     KeeLoq (`keeloq_create_data`, the manufacturer's first keystore entry and learning type,
     programming mode for button 0xF of BFT / Aprimatic / Dea_Mio; `Seed` + `Manufacture`), FAAC SLH
     (FAAC learning of the seed; `Seed`, `AllowZeroSeed: true`), CAME Atomo, Alutech AT-4N, Somfy
     Telis / Keytis, KingGates Stylo4k, Beninca ARC, Jarolift, Ditec GOL4, Nice FloR-S / One,
     Security+ 2.0, Phoenix V2 — each a `create_data` helper in its module. Counter steps as in each
     C function: FAAC SLH (Flipper-ARF), Somfy, Ditec GOL4, Security+ 2.0 and Alutech advance the
     counter before the first code, the others store it as given.

## Differences from the references (and why)

- **One list**: Flipper-ARF's 78 types plus doctor-firmware's 23 (Genius, Erreka, Somfy Keytis,
  Ditec GOL4, Superrollo, Dorcas, KEY, HomeGate, Pujol, Pujol Vario, ET Blue (Mix), ATA PTX4, Seav,
  Wisniowski, Fadini, Mc Garcia, Clemsa Mutancode, Doormatic, Elvox, Verex, Telcoma EDGE) where
  doctor's menu puts them. Where both have a type, ARF's name and parameters are used (they differ
  only in "KL: Genius Bravo": ARF button 0x06, doctor 0x09 / "KL: Genius TX4RC 433M.").
- **All types work in KAT that its protocols allow.** Flipper-ARF's firmware registers only its car
  protocols and KeeLoq, so on a real ARF device every non-KeeLoq type fails ("Function requires an SD
  card with fresh databases") or crashes in `subghz_transmitter_free(NULL)` (FAAC SLH, Somfy Telis,
  KingGates, Beninca ARC, Jarolift, Alutech, Phoenix V2, Nice FloR-S / One, CAME Atomo — reference
  gen-oracle finding); doctor-firmware registers them all, and so does KAT.
- **FAAC SLH**: Flipper-ARF's `create_data` advances the counter before the first code, doctor's
  does not; KAT follows ARF for the FAAC SLH types and doctor for its Genius types (which also write
  `Manufacture: Genius`; KAT's keystore has no Genius key, so — as doctor without one — they use
  the FAAC_SLH key).
- **Beninca ARC**: Flipper-ARF's `lib` copy encrypts with the STM32 hardware AES, whose word-swapped
  key gives frames neither doctor-firmware, the garage app nor KAT decode; KAT creates doctor's /
  the garage app's (software AES) frame, which its codec decodes and transmits.
- **doctor's generic serializer** also writes `Serial` / `Btn` / `Cnt` (and a decimal `Seed` line
  before the hex one, which a hex reader then trips on); KAT writes the lines the protocols' own
  serializers write (Flipper-ARF's form).
- **Not created** (listed with the reason): GangQi and Hollarm (no KAT protocol); Nice FloR-S /
  Nice One and Alutech AT-4N without the user's rainbow table (`~/.config/KAT/nice_flor_s.bin`,
  `alutech_at_4n.bin`, see `nice_flor_s.md` / `alutech_at_4n.md`; the C would save a key encrypted
  with nothing); KeeLoq manufacturers without a usable key in KAT's keystore (the C saves an all-zero
  key) — they become available when `~/.config/KAT/keeloq_mfcodes_user` adds the key; doctor's KEY
  and Dorcas, whose hops use doctor-only KeeLoq discriminators (0x2FF; Monarch 0x280 kept in the seed)
  while KAT's KeeLoq codec follows Flipper-ARF's.
- **KeeLoq transmission** uses the manufacturer the remote names (as ARF's encoder uses the key
  file's `Manufacture`). On air, receivers that walk a whole keystore (Flipper, KAT) name the first
  entry whose key validates the frame — for about one remote in ten an earlier keystore entry
  (e.g. a generated Beninca remote shown as Mutanco_Mutancode); doctor's and ARF's decoders name
  the same first match as KAT. The gate's receiver, which has only its own key, is not affected.

## Verification

- Unit tests (`add_manually.rs`, `gate_buttons.rs`): every type with three fixed seeds — the key file
  loads (text round trip), is encoder-capable, names the type's protocol (KeeLoq: its manufacturer),
  and its first transmission decodes on air (`process_signal_stream`) as the same remote: same
  serial and button, counter + 1 for rolling codes (KeeLoq first-match cases checked against the
  keystore order and the named manufacturer). Revers RB2 and Marantec24 fail their own round trip in
  every reference too (whether Revers RB2's decoder re-syncs to back-to-back frames depends on the
  key; Marantec24's encoder gap sits on the decoder's exclusive bound): Marantec24 is checked with a
  receiver's few µs of slack, Revers RB2 through the reference decoders, which decode exactly the
  same transmissions as KAT.
  Every listed button of every created remote is sent and decoded with that button.
- `src/app/tx.rs` `rolling_codes_advance_on_every_press`: every available rolling-code type sends a
  new counter on each of three presses once the frame sent is kept (FAAC SLH keeping its seed); fixed
  codes keep their capture.
- Reference gen oracle (Flipper-ARF's and doctor-firmware's `subghz_gen_info.c` and
  `subghz_txrx_create_protocol_key.c` compiled unmodified, `rand()` scripted, KAT's keystore): for
  three `rand()` sequences, 231 of 234 generated key files are byte-identical to the reference's
  (doctor's generic `Serial`/`Btn`/`Cnt` lines aside); the 3 others are Beninca ARC against ARF's
  hardware-AES copy (identical to doctor's).
- Reference decoders on KAT's transmissions (`subghz_oracle_{garage,doctor,arf}`, three seeds):
  every created remote decodes with the same protocol and serial (KeeLoq: Flipper-ARF and
  doctor-firmware name the type's manufacturer, or the same first match as KAT), except the
  reference failures above (Marantec24, Revers RB2 for some keys; BETT is absent from the garage app;
  Beninca ARC in ARF's hardware-AES copy). Reference encoders on KAT's key files: 150 of 156 uploads
  pulse-identical per seed; the rest are known reference differences (Beninca ARC in ARF's lib,
  Security+ 2.0's packet order and CAME TWEE's repeat count in doctor-firmware).

## Send button…

For gate / garage remotes (and KeeLoq barrier / alarm manufacturers) the signal menu has **Send
button…** next to Send next code: the protocol's native buttons — its labels where the references
name them (Somfy My/Up/Down/Prog, Jarolift Learn/Down/Stop/Up, Alutech buttons 1–5, …), else the codes
its encoder accepts as "Btn 0xN" (`src/protocols/gate_buttons.rs`) — or a typed hex code. Enter sends
the next code with that button, which the capture then keeps along with the counter (as the references
save the last code sent); the list stays open. The button column shows "Btn 0xN" for gate
protocols without button names (KAT's car labels Lock/Unlock/Trunk/Panic are for car remotes only).

## Types

| Type (menu name) | Protocol (manufacturer key) | MHz | Preset | Parameters (`r` = `rand()`) | Menu | KAT |
|---|---|---|---|---|---|---|
| FAAC SLH 868MHz | Faac SLH | 868.35 | AM650 | serial `((r & 0x00FFFFF0) \| 0xA0000006) >> 4`, button `0x06`, counter `0x02`, seed `r` | both | ✓ |
| FAAC SLH 433MHz | Faac SLH | 433.92 | AM650 | serial `((r & 0x00FFFFF0) \| 0xA0000006) >> 4`, button `0x06`, counter `0x02`, seed `r` | both | ✓ |
| Genius 433MHz | Faac SLH (Genius) | 433.92 | AM650 | serial `((r & 0x00FFFFF0) \| 0xA0000006) >> 4`, button `0x06`, counter `0x02`, seed `r` | doctor | ✓ |
| Genius 868MHz | Faac SLH (Genius) | 868.35 | AM650 | serial `((r & 0x00FFFFF0) \| 0xA0000006) >> 4`, button `0x06`, counter `0x02`, seed `r` | doctor | ✓ |
| BFT Mitto 433MHz | KeeLoq (BFT) | 433.92 | AM650 | serial `r & 0x000FFFFF`, button `0x02`, counter `0x02`, seed `r & 0x000FFFFF` | both | ✓ |
| Erreka 433MHz | KeeLoq (Erreka) | 433.92 | AM650 | serial `r & 0x000FFFFF`, button `0x02`, counter `0x02`, seed `r & 0x000FFFFF` | doctor | ✗ needs a Erreka key in `keeloq_mfcodes_user` |
| Somfy Telis 433MHz | Somfy Telis | 433.42 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | both | ✓ |
| Somfy Keytis 433MHz | Somfy Keytis | 433.42 | AM650 | serial `(r & 0x0000FFFF) \| 0x00D50000`, button `0x04`, counter `0x03` | doctor | ✓ |
| KingGates Stylo4k 433M. | KingGates Stylo4k | 433.92 | AM650 | serial `r & 0xFFFFFFFF`, button `0x0E`, counter `0x03` | both | ✓ |
| Beninca ARC 433MHz | Beninca ARC | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | both | ✓ |
| Jarolift 433MHz | Jarolift | 433.92 | AM650 | serial `r & 0xFFFFF00`, button `0x02`, counter `0x03` | both | ✓ |
| Ditec GOL4 433MHz | Ditec GOL4 | 433.92 | AM650 | serial `(r & 0x0000FFFF) \| 0xCC090000`, button `0x01`, counter `0xC200` | doctor | ✓ |
| AN-Motors AT4 433MHz | KeeLoq (AN-Motors) | 433.92 | AM650 | serial `(r & 0x000FFFFF) \| 0x04700000`, button `0x02`, counter `0x21` | both | ✓ |
| Alutech AT4N 433MHz | Alutech AT-4N | 433.92 | AM650 | serial `(r & 0x000FFFFF) \| 0x00100000`, button `0x44`, counter `0x03` | both | ✗ needs `alutech_at_4n.bin` |
| V2 Phoenix 433MHz | Phoenix_V2 | 433.92 | AM650 | serial `(r & 0x0FFFFFFF) \| 0xB0000000`, counter `0x025D` | both | ✓ |
| KL: HCS101 433MHz | KeeLoq (HCS101) | 433.92 | AM650 | serial `r & 0x000FFFFF`, button `0x02`, counter `0x03` | both | ✓ |
| KL: DoorHan 315MHz | KeeLoq (DoorHan) | 315.00 | AM650 | serial `r & 0x0FFFFFFF`, button `0x02`, counter `0x03` | both | ✓ |
| KL: DoorHan 433MHz | KeeLoq (DoorHan) | 433.92 | AM650 | serial `r & 0x0FFFFFFF`, button `0x02`, counter `0x03` | both | ✓ |
| KL: Superrollo 433MHz | KeeLoq (Superrollo) | 433.92 | AM650 | serial `r & 0x0FFFFFFF`, button `0x0A`, counter `0x03` | doctor | ✗ needs a Superrollo key in `keeloq_mfcodes_user` |
| KL: Beninca 433MHz | KeeLoq (Beninca) | 433.92 | AM650 | serial `(r & 0x000FFF00) \| 0x00800080`, button `0x01`, counter `0x05` | both | ✓ |
| KL: Beninca 868MHz | KeeLoq (Beninca) | 868.35 | AM650 | serial `(r & 0x000FFF00) \| 0x00800080`, button `0x01`, counter `0x05` | both | ✓ |
| KL: Comunello 433MHz | KeeLoq (Comunello) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x08`, counter `0x05` | both | ✓ |
| KL: Comunello 868MHz | KeeLoq (Comunello) | 868.46 | AM650 | serial `r & 0x00FFFFFF`, button `0x08`, counter `0x05` | both | ✓ |
| KL: Allmatic 433MHz | KeeLoq (Beninca) | 433.92 | AM650 | serial `(r & 0x00FFFF00) \| 0x01000011`, button `0x0C`, counter `0x05` | both | ✓ |
| KL: Allmatic 868MHz | KeeLoq (Beninca) | 868.35 | AM650 | serial `(r & 0x00FFFF00) \| 0x01000011`, button `0x0C`, counter `0x05` | both | ✓ |
| KL: Centurion 433MHz | KeeLoq (Centurion) | 433.92 | AM650 | serial `(r & 0x0000FFFF)`, button `0x02`, counter `0x03` | both | ✓ |
| KL: Monarch 433MHz | KeeLoq (Monarch) | 433.92 | AM650 | serial `(r & 0x0000FFFF)`, button `0x0A`, counter `0x03` | both | ✓ |
| KL: Dorcas 433MHz | KeeLoq (Monarch) | 433.92 | AM650 | serial `(r & 0x0000FFFF)`, button `0x0A`, counter `0x03`, seed `0x280` | doctor | ✗ doctor's Monarch discriminator 0x280 (KAT's KeeLoq is ARF's) |
| KL: KEY 433MHz | KeeLoq (KEY) | 433.92 | AM650 | serial `(r & 0x0FFFFFFF)`, button `0x01`, counter `0x03` | doctor | ✗ doctor's KEY discriminator 0x2FF (KAT's KeeLoq is ARF's) |
| KL: Jolly Mot. 433MHz | KeeLoq (Jolly_Motors) | 433.92 | AM650 | serial `(r & 0x000FFFFF)`, button `0x02`, counter `0x03` | both | ✓ |
| KL: Motorline 433MHz | KeeLoq (Motorline) | 433.92 | AM650 | serial `r & 0x0FFFFFFF`, button `0x01`, counter `0x03` | both | ✓ |
| KL: Sommer 434MHz | KeeLoq (Sommer) | 434.42 | FM476 | serial `(r & 0x0000FFFF) \| 0x01700000`, button `0x02`, counter `0x03` | both | ✓ |
| KL: Sommer 868MHz | KeeLoq (Sommer) | 868.80 | FM476 | serial `(r & 0x0000FFFF) \| 0x01700000`, button `0x02`, counter `0x03` | both | ✓ |
| KL: Sommer fm2 434Mhz | KeeLoq (Sommer) | 434.42 | FM12K | serial `r & 0x0000FFFF`, button `0x02`, counter `0x03` | both | ✓ |
| KL: Sommer fm2 868Mhz | KeeLoq (Sommer) | 868.80 | FM12K | serial `r & 0x0000FFFF`, button `0x02`, counter `0x03` | both | ✓ |
| KL: Stilmatic 433MHz | KeeLoq (Stilmatic) | 433.92 | AM650 | serial `r & 0x0FFFFFFF`, button `0x01`, counter `0x03` | both | ✓ |
| KL: IronLogic 433MHz | KeeLoq (IronLogic) | 433.92 | AM650 | serial `r & 0x00FFFFF0`, button `0x04`, counter `0x05` | both | ✓ |
| KL: IronLogic SM 433MHz | KeeLoq (IL-100(Smart)) | 433.92 | AM650 | serial `r & 0x00FFFFF0`, button `0x04`, counter `0x05` | both | ✓ |
| KL: DEA Mio 433MHz | KeeLoq (Dea_Mio) | 433.92 | AM650 | serial `(r & 0x0FFFF000) \| 0x00000869`, button `0x02`, counter `0x03` | both | ✓ |
| KL: DTM Neo 433MHz | KeeLoq (DTM_Neo) | 433.92 | AM650 | serial `r & 0x000FFFFF`, button `0x02`, counter `0x05` | both | ✓ |
| KL: Gibidi 433MHz | KeeLoq (Gibidi) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | both | ✓ |
| KL: GSN 433MHz | KeeLoq (GSN) | 433.92 | AM650 | serial `r & 0x0FFFFFFF`, button `0x02`, counter `0x03` | both | ✓ |
| KL: HomeGate 433MHz | KeeLoq (HomeGate) | 433.92 | AM650 | serial `r & 0x0FFFFFFF`, button `0x02`, counter `0x03` | doctor | ✗ needs a HomeGate key in `keeloq_mfcodes_user` |
| KL: Aprimatic 433MHz | KeeLoq (Aprimatic) | 433.92 | AM650 | serial `(r & 0x000FFFFF) \| 0x00600000`, button `0x08`, counter `0x03` | both | ✓ |
| KL: Elmes (PL) 433MHz | KeeLoq (Elmes_Poland) | 433.92 | AM650 | serial `(r & 0x00FFFFFF) \| 0x02000000`, button `0x02`, counter `0x03` | both | ✓ |
| KL: Normstahl 433MHz | KeeLoq (Normstahl) | 433.92 | AM650 | serial `r & 0x0000FFFF`, button `0x04`, counter `0x03` | both | ✓ |
| KL: JCM Tech 433MHz | KeeLoq (JCM_Tech) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | both | ✓ |
| KL: Novoferm 433MHz | KeeLoq (Novoferm) | 433.92 | AM650 | serial `(r & 0x0000FFFF) \| 0x018F0000`, button `0x01`, counter `0x03` | both | ✓ |
| KL: Hor. EcoStar 433MHz | KeeLoq (EcoStar) | 433.92 | AM650 | serial `(r & 0x000FFFFF) \| 0x02200000`, button `0x04`, counter `0x03` | both | ✓ |
| KL: Cardin S449 433MHz | KeeLoq (Cardin_S449) | 433.92 | FM12K | serial `(r & 0x000FFFFF)`, button `0x02`, counter `0x03` | both | ✓ |
| KL: Pujol 433MHz | KeeLoq (Pujol) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | doctor | ✗ needs a Pujol key in `keeloq_mfcodes_user` |
| KL: Pujol Vario 433MHz | KeeLoq (Pujol_Vario) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | doctor | ✗ needs a Pujol_Vario key in `keeloq_mfcodes_user` |
| KL: ET Blue 433MHz | KeeLoq (ET_Blue) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x04`, counter `0x03` | doctor | ✗ needs a ET_Blue key in `keeloq_mfcodes_user` |
| KL: ET Blue Mix 433MHz | KeeLoq (ET_Blue_Mix) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x04`, counter `0x03` | doctor | ✗ needs a ET_Blue_Mix key in `keeloq_mfcodes_user` |
| KL: ATA PTX4 433MHz | KeeLoq (ATA_PTX4) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | doctor | ✗ needs a ATA_PTX4 key in `keeloq_mfcodes_user` |
| KL: Seav 433MHz | KeeLoq (Seav) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | doctor | ✗ needs a Seav key in `keeloq_mfcodes_user` |
| KL: Wisniowski 433MHz | KeeLoq (Wisniowski) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | doctor | ✗ needs a Wisniowski key in `keeloq_mfcodes_user` |
| KL: Fadini 433MHz | KeeLoq (Fadini) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | doctor | ✗ needs a Fadini key in `keeloq_mfcodes_user` |
| KL: Mc Garcia 433MHz | KeeLoq (Mc_Garcia) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | doctor | ✗ needs a Mc_Garcia key in `keeloq_mfcodes_user` |
| KL: Clm.Mutancode 433M. | KeeLoq (Clemsa_Mutancode) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | doctor | ✗ needs a Clemsa_Mutancode key in `keeloq_mfcodes_user` |
| KL: Doormatic 433MHz | KeeLoq (Doormatic) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | doctor | ✗ needs a Doormatic key in `keeloq_mfcodes_user` |
| KL: Elvox 433MHz | KeeLoq (Elvox) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | doctor | ✗ needs a Elvox key in `keeloq_mfcodes_user` |
| KL: Verex 433MHz | KeeLoq (Verex) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | doctor | ✗ needs a Verex key in `keeloq_mfcodes_user` |
| KL: FAAC RC,XT 433MHz | KeeLoq (FAAC_RC,XT) | 433.92 | AM650 | serial `(r & 0x0000FFFF) \| 0x00100000`, button `0x02`, counter `0x03` | both | ✓ |
| KL: FAAC RC,XT 868MHz | KeeLoq (FAAC_RC,XT) | 868.35 | AM650 | serial `(r & 0x0000FFFF) \| 0x00100000`, button `0x02`, counter `0x03` | both | ✓ |
| KL: Genius Bravo 433MHz | KeeLoq (Genius_Bravo) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x06`, counter `0x03` | both | ✓ |
| KL: Mhouse 433MHz | KeeLoq (NICE_MHOUSE) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x09`, counter `0x03` | both | ✓ |
| KL: Nice Smilo 433MHz | KeeLoq (NICE_Smilo) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x02`, counter `0x03` | both | ✓ |
| Nice FloR-S 433MHz | Nice FloR-S | 433.92 | AM650 | serial `r & 0x0FFFFFFF`, button `0x01`, counter `0x03` | both | ✗ needs `nice_flor_s.bin` |
| Nice One 433MHz | Nice FloR-S | 433.92 | AM650 | serial `r & 0x0FFFFFFF`, button `0x01`, counter `0x03`, Nice One (72 bits) | both | ✗ needs `nice_flor_s.bin` |
| Nice Flo 12bit 433MHz | Nice FLO | 433.92 | AM650 | key `(r & 0x00000FF0) \| 0x1`, 12 bits | both | ✓ |
| Nice Flo 24bit 433MHz | Nice FLO | 433.92 | AM650 | key `(r & 0x00FFFFF0) \| 0x4`, 24 bits | both | ✓ |
| CAME 12bit 433MHz | CAME | 433.92 | AM650 | key `(r & 0x00000FF0) \| 0x1`, 12 bits | both | ✓ |
| CAME 24bit 433MHz | CAME | 433.92 | AM650 | key `(r & 0x00FFFFF0) \| 0x4`, 24 bits | both | ✓ |
| CAME 12bit 868MHz | CAME | 868.35 | AM650 | key `(r & 0x00000FF0) \| 0x1`, 12 bits | both | ✓ |
| CAME 24bit 868MHz | CAME | 868.35 | AM650 | key `(r & 0x00FFFFF0) \| 0x4`, 24 bits | both | ✓ |
| CAME TWEE 433MHz | CAME TWEE | 433.92 | AM650 | key `0x003FFF7200000000 \| ((r & 0x0FFFFFF0) ^ 0xE0E0E0EE)`, 54 bits | both | ✓ |
| CAME Atomo 433MHz | CAME Atomo | 433.92 | AM650 | serial `(r & 0x0FFFFFFF) \| 0x10000000`, counter `0x03` | both | ✓ |
| CAME Atomo 868MHz | CAME Atomo | 868.35 | AM650 | serial `(r & 0x0FFFFFFF) \| 0x10000000`, counter `0x03` | both | ✓ |
| KL: CAME Space 433MHz | KeeLoq (Came_Space) | 433.92 | AM650 | serial `r & 0x00FFFFFF`, button `0x04`, counter `0x03` | both | ✓ |
| Princeton 315MHz | Princeton | 315.00 | AM650 | key `(r & 0x00FFFFF0) \| 0x4`, 24 bits, TE 400 µs | both | ✓ |
| Princeton 433MHz | Princeton | 433.92 | AM650 | key `(r & 0x00FFFFF0) \| 0x4`, 24 bits, TE 400 µs | both | ✓ |
| ZKTeco 430MHz | Princeton | 430.50 | AM650 | key `(r & 0x00FFFF00) \| 0x30`, 24 bits, TE 357 µs | both | ✓ |
| BETT 433MHz | BETT | 433.92 | AM650 | key `(r & 0x0000FFF0)`, 18 bits | both | ✓ |
| GangQi 433MHz | GangQi | 433.92 | AM650 | 16-bit serial from `r₂`, button 0xD, byte-sum check, 34 bits | both | ✗ KAT has no GangQi protocol |
| Hollarm 433MHz | Hollarm | 433.92 | AM650 | key `(r & 0x000FFF0000) \| 0xF0B0002200` + byte-sum check, 42 bits | both | ✗ KAT has no Hollarm protocol |
| Revers RB2 433MHz | Revers_RB2 | 433.92 | AM650 | key `(r & 0x00000FFFFFFFF000) \| 0xFFFFF00000000000 \| 0x0000000000000A00`, 64 bits | both | ✓ |
| Marantec24 868MHz | Marantec24 | 868.35 | AM650 | key `(r & 0xFFFFF0) \| 0x000008`, 24 bits | both | ✓ |
| Marantec 433MHz | Marantec | 433.92 | AM650 | 20-bit serial from `r₃`, constant 0x130 / 0x86, button 0x4, CRC-8, 49 bits | both | ✓ |
| Marantec 868MHz | Marantec | 868.35 | AM650 | 20-bit serial from `r₃`, constant 0x130 / 0x86, button 0x4, CRC-8, 49 bits | both | ✓ |
| Roger 433MHz | Roger | 433.92 | AM650 | key `(r & 0xFFFF000) \| 0x0000101`, 28 bits | both | ✓ |
| Linear 300MHz | Linear | 300.00 | AM650 | key `(r & 0x3FF)`, 10 bits | both | ✓ |
| Telcoma EDGE 433MHz | Telcoma/Cardin EDGE | 433.92 | AM650 | key `0xFF000000 \| (r & 0x00FFFFF8)`, 32 bits | doctor | ✓ |
| Gate TX 433MHz | GateTX | 433.92 | AM650 | key `reverse((r & 0x00F0FF00) \| 0xF0040, 24)`, 24 bits | both | ✓ |
| Security+1.0 315MHz | Security+ 1.0 | 315.00 | AM650 | fixed code: `r` until `check_fixed` (id1 ≠ 0), rolling 0xE6000000 | both | ✓ |
| Security+1.0 390MHz | Security+ 1.0 | 390.00 | AM650 | fixed code: `r` until `check_fixed` (id1 ≠ 0), rolling 0xE6000000 | both | ✓ |
| Security+1.0 433MHz | Security+ 1.0 | 433.92 | AM650 | fixed code: `r` until `check_fixed` (id1 ≠ 0), rolling 0xE6000000 | both | ✓ |
| Security+2.0 310MHz | Security+ 2.0 | 310.00 | AM650 | serial `(r & 0x7FFFF3FC)`, button `0x68`, counter `0xE500000` | both | ✓ |
| Security+2.0 315MHz | Security+ 2.0 | 315.00 | AM650 | serial `(r & 0x7FFFF3FC)`, button `0x68`, counter `0xE500000` | both | ✓ |
| Security+2.0 390MHz | Security+ 2.0 | 390.00 | AM650 | serial `(r & 0x7FFFF3FC)`, button `0x68`, counter `0xE500000` | both | ✓ |
| Security+2.0 433MHz | Security+ 2.0 | 433.92 | AM650 | serial `(r & 0x7FFFF3FC)`, button `0x68`, counter `0xE500000` | both | ✓ |
