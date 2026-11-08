//! Alps FWB1U545 car remote (receive only; FoxFW key files).
//!
//! Port of rtl_433 `src/devices/alps_fwb1u545.c` (`r_device` "Alps FWB1U545 Car Remote", model
//! `Alps-FWB1U545`; FCC ID CWTWB1U545, an Alps Electric OEM remote for Honda). FoxFW has its own
//! port of the same decoder ("Alps FWB1U545", see **Key files**); no other Flipper reference has it.
//!
//! **Modulation** — `FSK_PULSE_MANCHESTER_ZEROBIT`: short_width 500 µs, reset_limit 1500 µs, no
//! tolerance (the slicer rejects no pulse). rtl_433's zerobit slicer ([super::rtl433])
//! runs on KAT's FM pairs (higher tone = pulse, as rtl_433's FSK detector); a gap > 1500 µs ends
//! the message. The recordings are 433.92 MHz 2-FSK, about ±30–37 kHz.
//!
//! **Frame** (`alps_fwb1u545_car_remote_decode`) — exactly one row of exactly 76 bits:
//! `PP IIIIIIII bb bb SSSS C`: byte 0 the 0x55 preamble byte, bytes 1–4 the 32-bit ID, byte 5 the
//! button code, byte 6 its copy, bytes 7–8 the 16-bit sequence, then 4 unknown bits. Checks
//! (`DECODE_FAIL_SANITY`): byte 0 = 0x55, byte 6 = byte 5, ID ≠ 0 and ≠ FFFFFFFF. Button =
//! byte 5 >> 4: 0xE Lock, 0xC Panic, 0x5 Panic Held, 0x1 Unlock, else "?". The long 500/500 µs
//! preamble is its own message (it ends at the ~2.5 ms gap before the frame) and fails the length
//! check.
//!
//! **Fields** — `serial` = ID, `button` = byte 5 >> 4, `counter` = sequence. `data` = the first 64
//! frame bits (bytes 0–7), `extra` = the last 12 (byte 8 and the unknown nibble), right-aligned;
//! `data_count_bit` = 76. `crc_valid` = false: rtl_433 reports no integrity check for this remote
//! (its checks are sanity checks; it prints no `mic` field). rtl_433's `button_str` is the native
//! button name.
//!
//! **Key files** — FoxFW's (`lib/subghz/protocols/alps_rke.c`, `ALPS_RKE_PROTOCOL_NAME` "Alps
//! FWB1U545", an alias here), written by its generic serializer: `Protocol: Alps FWB1U545`,
//! `Bit: 76`, `Key` (8 bytes) = bytes 1–8: ID, the button byte, its copy, the sequence. FoxFW never
//! decodes the 4 unknown bits after the sequence and does not save the 0x55 byte, so KAT adds one
//! line after FoxFW's: `Raw` = the frame's 10 bytes (the 76 bits, then 4 zero bits). FoxFW's
//! `deserialize` reads only `Bit` and `Key` and ignores it (checked on FoxFW's C). Loading is
//! FoxFW's `deserialize` (exactly `Bit: 76`), then the decoder's checks (the 0x55 byte, the button
//! copy, the ID; FoxFW saves only frames that pass them, and also drops IDs 55555555 / AAAAAAAA,
//! which rtl_433 and KAT read) on `Raw`, which must hold the key as bytes 1–8 (a `Raw` that fails
//! is refused), giving the received decode back exactly. A file without `Raw` (FoxFW's own) is
//! checked as the frame rebuilt with the 0x55 byte and **the unknown bits 0**, so it loads with
//! `extra`'s low nibble (not displayed) cleared; a key that fails the checks is refused.
//!
//! **Encoder** — none. rtl_433 has none, and FoxFW's SD encoder plugin
//! (`applications/main/subghz/protocols_ext/enc_alps_rke.c`) is a stub ("no TX — decode/save
//! only"): its `deserialize` returns `SubGhzProtocolStatusErrorEncoderGetUpload` for every key file,
//! so a Flipper running FoxFW sends nothing. `supports_encoding()` is false, `encode` returns None
//! and nothing is encoder-capable; KAT offers Replay of the recorded pulses.

use super::rtl433::{frame_signal, BitBuffer, DecodeFn, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "Alps-FWB1U545";

/// FoxFW's protocol name (`ALPS_RKE_PROTOCOL_NAME`), written in key files.
const FOXFW_NAME: &str = "Alps FWB1U545";

/// The frame length (`ALPS_RKE_FRAME_BITS`, FoxFW's key `Bit`).
const BITS: usize = 76;

/// Metadata (see `info.rs`): 2-FSK; F3 (±31.74 kHz) is the closest preset to the recordings'
/// deviation.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Alps FWB1U545 Car Remote", FOXFW_NAME],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Honda",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&F3],
};

/// The `r_device`; a burst's FM pairs are one package (see `rtl433`).
const DEVICE: Device = Device {
    modulation: Modulation::FskManchesterZerobit,
    short_width: 500,
    long_width: 0,
    reset_limit: 1500,
    gap_limit: 0,
    sync_width: 0,
    tolerance: 0,
    gaps: Gaps::AtOnce,
    ends: Ends::None,
    resync: false,
};

/// rtl_433's `button_str`.
fn button_str(code: u8) -> &'static str {
    match code {
        0xe => "Lock",
        0xc => "Panic",
        0x5 => "Panic Held",
        0x1 => "Unlock",
        _ => "?",
    }
}

fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(button_str(code))
}

/// `alps_fwb1u545_car_remote_decode`.
#[derive(Debug, Default)]
struct Alps;

impl DecodeFn for Alps {
    fn message(&mut self, bits: &mut BitBuffer) -> Option<DecodedSignal> {
        if bits.bits_per_row(0) != 76 || bits.num_rows() > 1 {
            return None; // DECODE_ABORT_LENGTH
        }
        check(&bits.bytes(0))
    }
}

/// The C's sanity checks on the frame's 10 bytes (preamble byte, button copy, ID) and the decode
/// they give.
fn check(b: &[u8; 10]) -> Option<DecodedSignal> {
    // check preamble and button XOR
    if b[0] != 0x55 || b[5] != b[6] {
        return None; // DECODE_FAIL_SANITY
    }
    let id = u32::from_be_bytes([b[1], b[2], b[3], b[4]]);
    if id == 0 || id == 0xffff_ffff {
        return None; // DECODE_FAIL_SANITY
    }
    let mut d = frame_signal(b, BITS);
    d.serial = Some(id);
    d.button = Some(b[5] >> 4);
    d.counter = Some(u16::from_be_bytes([b[7], b[8]]));
    Some(d)
}

/// Alps FWB1U545 decoder (receive only).
pub struct AlpsFwb1u545Decoder {
    rx: Receiver,
    decode: Alps,
}

impl AlpsFwb1u545Decoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE), decode: Alps }
    }
}

impl Default for AlpsFwb1u545Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for AlpsFwb1u545Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 500, te_long: 1000, te_delta: 0, min_count_bit: 76 }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.rx.reset(&mut self.decode);
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.rx.feed(&mut self.decode, level, duration)
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    /// None: rtl_433 has no encoder and FoxFW's plugin refuses every key (see "Encoder").
    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// FoxFW's `subghz_protocol_decoder_alps_rke_deserialize` (`Bit: 76`), then the decoder's checks
    /// on KAT's `Raw` line, which must hold the key as bytes 1–8, or, without it (FoxFW's own files),
    /// on the frame rebuilt with the 0x55 byte and the unknown bits 0 (see "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != BITS {
            return None;
        }
        let k = key.key_u64()?.to_be_bytes();
        if key.field("Raw").is_some() {
            let raw: [u8; 10] = key.hex_bytes_field("Raw")?.try_into().ok()?;
            return (raw[1..9] == k && raw[9] & 0x0F == 0).then(|| check(&raw)).flatten();
        }
        let mut b = [0u8; 10];
        b[0] = 0x55;
        b[1..9].copy_from_slice(&k);
        check(&b)
    }

    /// FoxFW's `subghz_protocol_decoder_common_serialize` of the frame (`Bit: 76`, `Key` = bytes
    /// 1–8: `id << 32 | button byte << 24 | copy << 16 | sequence`), then `Raw` = the frame's 10
    /// bytes (KAT's line, see "Key files"). None for a decode the checks refuse (it would not load).
    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        let extra = d.extra?;
        if d.data_count_bit != BITS || d.data >> 56 != 0x55 || extra >> 12 != 0 {
            return None;
        }
        let mut raw = [0u8; 10];
        raw[..8].copy_from_slice(&d.data.to_be_bytes());
        raw[8] = (extra >> 4) as u8;
        raw[9] = ((extra & 0xF) << 4) as u8;
        check(&raw)?;
        let mut key = FlipperKey::for_protocol(FOXFW_NAME, BITS, d.data << 8 | extra >> 4);
        key.push_hex_bytes("Raw", &raw);
        Some(key)
    }
}

/// rtl_433's JSON for a decode (`data_print_jsons`, no `time`), for the oracle comparisons.
#[cfg(test)]
pub(super) fn rtl433_json(d: &DecodedSignal) -> String {
    let btn = d.button.unwrap_or(0);
    format!(
        "{{\"model\":\"{NAME}\",\"id\":\"{:08X}\",\"button_code\":{btn},\"button_str\":\"{}\",\"sequence\":{}}}",
        d.serial.unwrap_or(0),
        button_str(btn),
        d.counter.unwrap_or(0)
    )
}

#[cfg(test)]
mod tests {
    use super::super::rtl433::test_util::*;
    use super::*;

    /// The 76 frame bits for an ID, button and sequence (C layout; `button` is byte 5's high nibble,
    /// `low` its low nibble, `tail` the unknown nibble).
    fn frame(id: u32, button: u8, low: u8, seq: u16, tail: u8) -> Vec<bool> {
        let b5 = button << 4 | low;
        let bytes = [0x55, (id >> 24) as u8, (id >> 16) as u8, (id >> 8) as u8, id as u8, b5, b5, (seq >> 8) as u8, seq as u8, tail << 4];
        bits_of_bytes(&bytes, 76)
    }

    /// A transmission as the recordings show it: 500/500 µs preamble, a 2.5 ms gap, the frame.
    fn transmission(row: &[bool], half: u32) -> Vec<(u32, u32)> {
        let mut pairs = vec![(half, half); 200];
        pairs.last_mut().unwrap().1 = 2500;
        pairs.extend(manchester_zerobit(row, half, 80_000));
        pairs
    }

    #[test]
    fn decodes_every_button() {
        for (button, label) in [(0xE, "Lock"), (0xC, "Panic"), (0x5, "Panic Held"), (0x1, "Unlock"), (0x7, "?")] {
            let row = frame(0x4817_E61E, button, 0x3, 34_788, 0xA);
            let got = feed_pairs(&mut AlpsFwb1u545Decoder::new(), &transmission(&row, 500));
            assert_eq!(got.len(), 1, "{label}");
            let d = &got[0];
            assert_eq!((d.serial, d.button, d.counter), (Some(0x4817_E61E), Some(button), Some(34_788)));
            assert_eq!((d.data_count_bit, d.crc_valid, d.encoder_capable), (76, false, false));
            // bytes 0..7 = 55 48 17 E6 1E b5 b5 87, then E4 and the unknown nibble A
            let b5 = u64::from(button << 4 | 3);
            assert_eq!(d.data, 0x55_4817_E61E_u64 << 24 | b5 << 16 | b5 << 8 | 0x87);
            assert_eq!(d.extra, Some(0xE4A));
            assert_eq!(native_buttons(NAME, button), Some(label));
            assert!(rtl433_json(d).contains(&format!("\"button_str\":\"{label}\"")));
        }
    }

    #[test]
    fn tolerates_timing_like_the_c() {
        // No tolerance: only the 1.5 × short edge spacing matters.
        let row = frame(0x0102_0304, 0x1, 0, 7, 0);
        for half in [420, 500, 580] {
            let got = feed_pairs(&mut AlpsFwb1u545Decoder::new(), &transmission(&row, half));
            assert_eq!(got.len(), 1, "half {half}");
            assert_eq!(got[0].serial, Some(0x0102_0304));
        }
    }

    #[test]
    fn rejects_failed_checks() {
        let decode = |row: &[bool]| feed_pairs(&mut AlpsFwb1u545Decoder::new(), &transmission(row, 500));
        // button copy differs
        let mut row = frame(0x4817_E61E, 0xE, 0, 1, 0);
        row[47] = !row[47];
        assert!(decode(&row).is_empty());
        // preamble byte
        let mut row = frame(0x4817_E61E, 0xE, 0, 1, 0);
        row[1] = !row[1];
        assert!(decode(&row).is_empty());
        // ID 0 and FFFFFFFF
        assert!(decode(&frame(0, 0xE, 0, 1, 0)).is_empty());
        assert!(decode(&frame(0xFFFF_FFFF, 0xE, 0, 1, 0)).is_empty());
        // 75 / 77 bits
        let row = frame(0x4817_E61E, 0xE, 0, 1, 0);
        assert!(decode(&row[..75]).is_empty());
        let mut long = row.clone();
        long.push(true);
        assert!(decode(&long).is_empty());
        // a gap over 1500 µs inside the frame splits it
        let mut pairs = transmission(&row, 500);
        let n = pairs.len();
        pairs[n - 20].1 = 1600;
        assert!(feed_pairs(&mut AlpsFwb1u545Decoder::new(), &pairs).is_empty());
    }

    /// rtl_433_tests `alps/fwb1u545/01`: the four recordings through KAT's receive pipeline and the
    /// registry give rtl_433's events (`.json` next to each file).
    #[test]
    #[ignore]
    fn decodes_the_rtl_433_recordings() {
        let Some(root) = rtl433_tests() else {
            eprintln!("Skip: rtl_433_tests not found");
            return;
        };
        let dir = root.join("alps/fwb1u545/01");
        for f in ["lock", "panic", "panic_held", "unlock"] {
            let path = dir.join(format!("{f}_433.92M_250k.cu8"));
            let got: Vec<serde_json::Value> = registry_decodes(&path)
                .iter()
                .map(|(n, d)| {
                    assert_eq!(n, NAME, "{f}");
                    serde_json::from_str(&rtl433_json(d)).unwrap()
                })
                .collect();
            let want = expected_json(&path);
            assert_eq!(distinct(&got), distinct(&want), "{f}");
        }
    }

    /// A made-up frame whose RAW file (written to `$KAT_SYNTH_DIR` when set) FoxFW's decoder read with
    /// `subghz_oracle_foxfwenc decode --protocol "Alps FWB1U545"`: KAT writes FoxFW's lines; reading
    /// them back gives the decode with the 4 unknown bits 0 (FoxFW's key has no room for them).
    #[test]
    fn key_files_are_foxfws() {
        const FOXFW: &str = "Frequency: 433920000; Preset: FuriHalSubGhzPreset2FSKDev476Async; Protocol: Alps FWB1U545; Bit: 76; Key: 13 57 9B DF E3 E3 24 68";
        let air = vector_wire(&frame(0x1357_9BDF, 0xE, 0x3, 0x2468, 0xA));
        crate::protocols::foxfw_common::test_support::dump("alps_unit.sub", 433_920_000, "FuriHalSubGhzPreset2FSKDev476Async", &air);
        let got = decode_all(&mut AlpsFwb1u545Decoder::new(), &air);
        assert_eq!(got.len(), 1, "only the recordings' shape is rtl_433's frame");
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter, d.extra), (Some(0x1357_9BDF), Some(0xE), Some(0x2468), Some(0x68A)));
        let dec = AlpsFwb1u545Decoder::new();
        let mut key = dec.to_flipper_key(d).unwrap();
        key.frequency = 433_920_000;
        key.preset = Some("FuriHalSubGhzPreset2FSKDev476Async".into());
        assert_eq!(key.to_text().lines().skip(2).collect::<Vec<_>>().join("; "), format!("{FOXFW}; Raw: 55 13 57 9B DF E3 E3 24 68 A0"));
        // KAT's file keeps the 4 unknown bits (0xA) in `Raw`: the received decode comes back.
        crate::protocols::flipper_key::testing::assert_round_trip(&dec, d);
        // FoxFW's file has no room for them: they read 0.
        let cleared = DecodedSignal { extra: Some(0x680), ..d.clone() };
        let text = format!("Filetype: Flipper SubGhz Key File\nVersion: 1\n{}\n", FOXFW.replace("; ", "\n"));
        assert_eq!(dec.from_flipper_key(&crate::protocols::flipper_key::testing::key(&text)), Some(cleared.clone()));
        crate::protocols::flipper_key::testing::assert_round_trip(&dec, &cleared);
        let reg = super::super::ProtocolRegistry::new();
        assert_eq!(reg.get(FOXFW_NAME).map(|p| p.name()), Some(NAME));
    }

    /// Keys FoxFW's `deserialize` refuses (another `Bit`), keys whose frame fails the checks (FoxFW
    /// writes none) and a `Raw` line that is not the key's frame or fails them: not loaded. IDs
    /// 55555555 / AAAAAAAA, which FoxFW's decoder skips but rtl_433's reads, load.
    #[test]
    fn refuses_other_keys() {
        let dec = AlpsFwb1u545Decoder::new();
        let good = 0x1357_9BDF_u64 << 32 | 0xE3E3 << 16 | 0x2468;
        let load = |bits: usize, k: u64| dec.from_flipper_key(&FlipperKey::new(FOXFW_NAME, bits, k, 433_920_000, "FuriHalSubGhzPreset2FSKDev476Async"));
        assert_eq!(load(76, good).and_then(|d| d.counter), Some(0x2468));
        assert!(load(72, good).is_none(), "Bit 72");
        assert!(load(76, good ^ 1 << 16).is_none(), "button copy");
        assert!(load(76, good & 0xFFFF_FFFF).is_none(), "ID 0");
        assert!(load(76, good | 0xFFFF_FFFF << 32).is_none(), "ID FFFFFFFF");
        assert_eq!(load(76, 0x5555_5555 << 32 | 0xE3E3 << 16 | 1).and_then(|d| d.serial), Some(0x5555_5555));
        assert!(dec.to_flipper_key(&DecodedSignal::new(0, 64)).is_none());
        let with_raw = |k: u64, raw: &[u8]| {
            let mut key = FlipperKey::new(FOXFW_NAME, 76, k, 433_920_000, "FuriHalSubGhzPreset2FSKDev476Async");
            key.push_hex_bytes("Raw", raw);
            dec.from_flipper_key(&key)
        };
        let raw = [0x55, 0x13, 0x57, 0x9B, 0xDF, 0xE3, 0xE3, 0x24, 0x68, 0xA0];
        assert_eq!(with_raw(good, &raw).and_then(|d| d.extra), Some(0x68A));
        assert!(with_raw(good ^ 1, &raw).is_none(), "Raw is another frame than Key");
        assert!(with_raw(good, &raw[..9]).is_none(), "Raw too short");
        assert!(with_raw(good, &[0x54, 0x13, 0x57, 0x9B, 0xDF, 0xE3, 0xE3, 0x24, 0x68, 0xA0]).is_none(), "no 0x55 byte");
        assert!(with_raw(good, &[0x55, 0x13, 0x57, 0x9B, 0xDF, 0xE3, 0xE3, 0x24, 0x68, 0xA1]).is_none(), "bits past the 76");
    }

    /// FoxFW's encoder plugin refuses every Alps FWB1U545 key (`subghz_oracle_foxfwenc encode`:
    /// "deserialize failed (status -12)" for each of 120 made-up files) and rtl_433 has no encoder:
    /// nothing transmits, for no button.
    #[test]
    fn transmits_nothing_as_foxfw() {
        let dec = AlpsFwb1u545Decoder::new();
        assert!(!dec.supports_encoding());
        let d = decode_all(&mut AlpsFwb1u545Decoder::new(), &vector_wire(&frame(0x1357_9BDF, 0x1, 0, 7, 0))).remove(0);
        let loaded = dec.from_flipper_key(&dec.to_flipper_key(&d).unwrap()).unwrap();
        for x in [&d, &loaded] {
            assert!(!x.encoder_capable);
            for button in [0x01, 0x02, 0x04, 0x08, x.button.unwrap()] {
                assert!(dec.encode(x, button).is_none());
            }
        }
    }

    /// A RAW file's pulses for a frame, in the two shapes the decoders read: FoxFW's decoder locks
    /// on 8 half-bit pulses and Manchester-decodes from there, so it reads a frame only when a
    /// preamble of 1 bits runs straight into it (after a preamble of 0 bits, or behind the
    /// recordings' 2.5 ms gap, it loses the frame's first bits); rtl_433's slicer reads only a
    /// message of exactly the 76 frame bits, as the remote sends it ([transmission]). So: the frame
    /// behind twelve 1 bits, 20 ms of LOW, then [transmission].
    fn vector_wire(row: &[bool]) -> Vec<LevelDuration> {
        let mut foxfw = vec![false];
        foxfw.extend([true; 12]);
        foxfw.extend_from_slice(row);
        let mut pairs = manchester_zerobit(&foxfw, 500, 20_000);
        pairs.extend(transmission(row, 500));
        pairs.last_mut().unwrap().1 = 20_000;
        with_idle(&wire(&pairs), 20_000)
    }

    /// Made-up frames for the FoxFW comparison, as RAW files in `$KAT_SYNTH_DIR`: every named button
    /// and others, random IDs, low nibbles, sequences and unknown bits.
    #[test]
    #[ignore = "oracle harness"]
    fn dump_key_vectors() {
        let mut rng = Rng(0x00A1_9545);
        for i in 0..150u64 {
            let id = loop {
                let id = rng.next() as u32;
                if id != 0 && id != 0xFFFF_FFFF && id != 0x5555_5555 && id != 0xAAAA_AAAA {
                    break id;
                }
            };
            let button = if i < 64 { [0xE, 0xC, 0x5, 0x1][(i % 4) as usize] } else { rng.below(16) as u8 };
            let row = frame(id, button, rng.below(16) as u8, rng.next() as u16, rng.below(16) as u8);
            crate::protocols::foxfw_common::test_support::dump(&format!("alps_{i:03}.sub"), 433_920_000, "FuriHalSubGhzPreset2FSKDev476Async", &vector_wire(&row));
        }
    }

    /// `KAT_ALPS_RAW=<file listing RAW .sub files>`: KAT's key-file lines for each frame this decoder
    /// reads in them, to diff with `subghz_oracle_foxfwenc decode --protocol "Alps FWB1U545"`.
    #[test]
    #[ignore = "oracle harness"]
    fn key_lines_of_raw_files() {
        print_raw_key_lines("KAT_ALPS_RAW", &mut AlpsFwb1u545Decoder::new());
    }

    /// `KAT_ALPS_ENCODE=<file listing key files>`: KAT's transmission of each (`(none)`: it never
    /// transmits), to diff with `subghz_oracle_foxfwenc encode <file>`, which refuses each.
    #[test]
    #[ignore = "oracle harness"]
    fn encode_key_files() {
        print_key_file_encodes("KAT_ALPS_ENCODE", &AlpsFwb1u545Decoder::new());
    }

    /// Fuzz dump for the C harness (see `rtl433::test_util::dump_fuzz_trains`).
    #[test]
    #[ignore]
    fn fuzz_dump() {
        let interesting = [1, 499, 500, 501, 749, 750, 751, 1000, 1499, 1500, 1501, 2500];
        dump_fuzz_trains(
            "alps",
            20_000,
            |rng, i| {
                let id = match rng.below(20) {
                    0 => 0,
                    1 => 0xFFFF_FFFF,
                    _ => rng.next() as u32,
                };
                let button = [0xE, 0xC, 0x5, 0x1][rng.below(4) as usize] ^ if rng.chance(10) { rng.below(16) as u8 } else { 0 };
                let mut row = frame(id, button, rng.below(16) as u8, rng.next() as u16, rng.below(16) as u8);
                if rng.chance(15) {
                    let k = 1 + rng.below(75) as usize;
                    row[k] = !row[k];
                }
                if rng.chance(5) {
                    row.truncate(70 + rng.below(6) as usize);
                }
                let half = rng.jitter(500, if i % 3 == 0 { 120 } else { 30 });
                let mut pairs = if rng.chance(50) { transmission(&row, half) } else { manchester_zerobit(&row, half, 3000) };
                for p in pairs.iter_mut() {
                    *p = (rng.jitter(p.0, 15), rng.jitter(p.1, 15));
                }
                if rng.chance(60) {
                    mutate(rng, &mut pairs, &interesting);
                }
                pairs
            },
            || Box::new(AlpsFwb1u545Decoder::new()),
            rtl433_json,
        );
    }
}
