//! MARELLI: Flipper-ARF's "Fiat Marelli" key files, loaded, saved and replayed. KAT has no
//! decoder for it (it registers the protocol with a `feed` that reports nothing), as FoxFW has none.
//!
//! Ported from Flipper-ARF `lib/subghz/protocols/fiat_marelli.c` / `fiat_marelli.h`
//! (`FIAT_MARELLI_PROTOCOL_NAME` = "MARELLI"): the key-file `serialize` / `deserialize`, the fields
//! its `get_string` shows and the encoder. The file is ProtoPirate's Fiat V1 decoder from before
//! ProtoPirate rewrote it ("Magneti Marelli BSI keyfob protocol (PCF7946)"; KAT ported that version
//! in v1.2.0 and followed the rewrite, `fiat_v1.rs`). Flipper-ARF still registers it as a decoder.
//! FoxFW carries the same file without its Decodable flag.
//!
//! **Why nothing decodes it.** FoxFW dropped the flag on purpose (`fiat_marelli.c`, v2.0.14): "Delphi
//! BSI and Magneti Marelli BSI Fiat systems both use the same Hitag2/PCF7946 transponder, so there is
//! no distinct Marelli over-the-air protocol: the Fiat V0/V1 decoders already cover every Fiat Hitag2
//! remote. With Decodable set, this decoder only ever latched garbage 102-bit frames (misframed,
//! wrong serial) onto Fiat V1 (Delphi) and Renault captures [...] Load|Save|Send keep existing saved
//! MARELLI .sub files openable and emulatable." KAT's test captures show the same: run alone on all
//! 256 of them, Flipper-ARF's decoder reads 6 frames in 3 files, every one a frame that KAT,
//! ProtoPirate, doctor-firmware and FoxFW read as Fiat V1 (`Fiat Panda.sub`, `Opel Vectra 2008.sub`)
//! or Ford V3 (the 2022 Ford Escape), read two bits early: on the Opel Vectra its serial 0x1A3D7494
//! is Fiat V1's UID 0x468F5D25 shifted left by two, and its counter is not the remote's.
//!
//! **Key files** (`subghz_protocol_decoder_fiat_marelli_serialize` / `_deserialize`): `Protocol:
//! MARELLI`, `Bit` (the decoder writes 80..=104), `Key` (frame bits 0..63), `Extra` (32 bits: the
//! frame bits after the 64th as the decoder shifts them in, so of a frame longer than 96 bits only
//! the last 32), `Extra_bits` (Bit − 64), `TE` (µs: the decoder's half-bit, measured on the preamble;
//! 0 = 260), then `Serial`, `Btn` and `Cnt`. Loading reads Bit and Key (any bit count, as
//! `subghz_block_generic_deserialize`), then `Extra` and `TE` when present; the C re-derives the
//! other three. KAT keeps `extra` = TE << 32 | Extra.
//!
//! **Fields** (`fiat_marelli_rebuild_raw_data`, for frames of 56 bits or more): serial = frame
//! bytes 2..5, button = byte 6's high nibble (the C names 0x7 Lock, 0xB Unlock and 0xD Trunk),
//! counter = byte 7 >> 3 (5 bits). The frame's CRC-8 (poly 0x01, init 0x03, over bytes 0..11 = byte
//! 12) covers 104-bit frames only and cannot be checked from a file, whose `Extra` never holds byte
//! 12; the C then shows "CRC:FAIL" for every 104-bit file, KAT "none".
//!
//! **Transmit** (`subghz_protocol_encoder_fiat_marelli_deserialize` +
//! `fiat_marelli_encoder_get_upload`), replay only, as the C: it keeps the captured button and
//! counter ("the rolling security lives in the ENCRYPTED payload bytes 8-11 [...] we replay the
//! captured frame byte-identically"), so KAT's Lock / Unlock / Trunk / Panic all send the saved
//! frame. The frame bytes are rebuilt from Key and Extra (`fiat_marelli_encoder_rebuild_raw_data`,
//! which recomputes byte 12 as the CRC for 104 bits), then sent 3 times: 100 HIGH/LOW pairs at TE
//! (the last pair without its LOW), LOW 13·TE, HIGH 8·TE (the sync), the bits Manchester-coded from
//! the Mid1 state (from Mid1: 1 = LOW TE, HIGH TE and 0 = LOW 2·TE; from Mid0: 1 = HIGH 2·TE and 0 =
//! HIGH TE, LOW TE), and a LOW of 37·TE (in place of the last LOW when the frame ends in Mid0).
//! Bit counts outside 80..=104 do not transmit (the C's upload fails).
//!
//! **Frames longer than 96 bits, as a Flipper sends them.** They do not survive the key file:
//! `Extra` keeps the last 32 extra bits, and the rebuild (frame bits 64..95, `i < 32`) tests
//! `extra & (1UL << (extra_bits - 1 - i))`, a shift by 32 or more for the first `extra_bits - 32`
//! bits, which C leaves undefined. The Flipper's Cortex-M4 shifts a register by 32 or more to 0,
//! so those bits go out as 0, as do bits 96 and up (never rebuilt; for 104 bits byte 12 is the CRC
//! of the rebuilt bytes). KAT sends what a Flipper sends. The x86 host build of the C masks the
//! shift count instead and puts `Extra`'s low bits there; on every file of 96 bits or fewer, and
//! on longer ones whose `Extra` has those low bits clear, KAT's transmission equals that build's
//! pulse for pulse (checked on generated key files, see `docs/fiat_marelli.md`). FoxFW's SD-plugin
//! encoder (`enc_fiat_marelli.c`) sends exactly what Flipper-ARF's does on all of them.

use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// `FIAT_MARELLI_PROTOCOL_NAME` (Flipper-ARF and FoxFW).
pub(super) const NAME: &str = "MARELLI";

/// Metadata (see `info.rs`). Flipper-ARF flags it 433 MHz FM.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Fiat Marelli"],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "Replay (saved frames; no decoder)",
    make: "Fiat",
    category: Category::Vehicle,
    buttons: Some(button_name),
    presets: &[&FM476],
};

/// `fiat_marelli_button_name`.
fn button_name(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0x7 => Some("Lock"),
        0xB => Some("Unlock"),
        0xD => Some("Trunk"),
        _ => None,
    }
}

/// `subghz_protocol_fiat_marelli_const.te_short`: the half-bit when the file has no `TE`.
const TE_DEFAULT: u32 = 260;
/// `FIAT_MARELLI_MIN_DATA_BITS` / `FIAT_MARELLI_MAX_DATA_BITS`: what the encoder sends.
const MIN_BITS: usize = 80;
const MAX_BITS: usize = 104;
/// `FIAT_MARELLI_PREAMBLE_PAIRS`, `FIAT_MARELLI_ENCODER_REPEAT`.
const PREAMBLE_PAIRS: usize = 100;
const REPEAT: usize = 3;
/// The encoder's gap (`te * 12`) and sync (`te * 8`).
const GAP_TE: u32 = 12;
const SYNC_TE: u32 = 8;
/// 13 frame bytes (104 bits).
const FRAME_BYTES: usize = 13;

/// `fiat_marelli_crc8`: CRC-8, poly 0x01, init 0x03, MSB first.
fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0x03;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x01 } else { crc << 1 };
        }
    }
    crc
}

/// `fiat_marelli_rebuild_raw_data` (`with_crc` false, the decoder's deserialize) and
/// `fiat_marelli_encoder_rebuild_raw_data` (`with_crc` true): the frame bytes from Key and Extra,
/// with the C's 32-bit `Extra` and the Cortex-M4's shifts (a shift by 32 or more is 0; see the module
/// doc).
fn rebuild(key: u64, extra: u32, bits: usize, with_crc: bool) -> [u8; FRAME_BYTES] {
    let mut raw = [0u8; FRAME_BYTES];
    raw[..8].copy_from_slice(&key.to_be_bytes());
    // `uint8_t extra_bits = data_count_bit > 64 ? data_count_bit - 64 : 0`.
    let extra_bits = (bits.saturating_sub(64) & 0xFF) as u32;
    for i in 0..extra_bits.min(32) {
        let shift = extra_bits - 1 - i;
        if shift < 32 && extra & (1u32 << shift) != 0 {
            raw[8 + (i / 8) as usize] |= 1 << (7 - i % 8);
        }
    }
    if with_crc && bits >= MAX_BITS {
        raw[12] = crc8(&raw[..12]);
    }
    raw
}

/// The decode a key file stands for (the C's deserialize, then `get_string`'s fields).
fn decode_of(key: u64, extra: u32, bits: usize, te: u32) -> DecodedSignal {
    let raw = rebuild(key, extra, bits, false);
    let fields = bits >= 56;
    DecodedSignal {
        serial: fields.then(|| u32::from_be_bytes([raw[2], raw[3], raw[4], raw[5]])),
        button: fields.then_some(raw[6] >> 4),
        counter: fields.then_some(u16::from((raw[7] >> 3) & 0x1F)),
        crc_valid: false,
        data: key,
        data_count_bit: bits,
        encoder_capable: (MIN_BITS..=MAX_BITS).contains(&bits),
        extra: Some((u64::from(te) << 32) | u64::from(extra)),
        protocol_display_name: None,
    }
}

/// `fiat_marelli_encoder_get_upload`: one copy of the transmission (None outside 80..=104 bits).
fn upload(raw: &[u8; FRAME_BYTES], bits: usize, te: u32) -> Option<Vec<LevelDuration>> {
    if !(MIN_BITS..=MAX_BITS).contains(&bits) {
        return None;
    }
    let te = if te == 0 { TE_DEFAULT } else { te };
    // The C's `uint32_t` arithmetic (a file's TE is not range-checked): wrapping, never a panic.
    let (short, long) = (te, te.wrapping_mul(2));
    let mut up = Vec::with_capacity(2 * PREAMBLE_PAIRS + 2 * bits + 3);
    for i in 0..PREAMBLE_PAIRS {
        up.push(LevelDuration::new(true, short));
        if i < PREAMBLE_PAIRS - 1 {
            up.push(LevelDuration::new(false, short));
        }
    }
    up.push(LevelDuration::new(false, short.wrapping_add(GAP_TE.wrapping_mul(te))));
    up.push(LevelDuration::new(true, SYNC_TE.wrapping_mul(te)));
    let mut in_mid1 = true;
    for i in 0..bits {
        let bit = (raw[i / 8] >> (7 - i % 8)) & 1 == 1;
        match (in_mid1, bit) {
            (true, true) => up.extend([LevelDuration::new(false, short), LevelDuration::new(true, short)]),
            (true, false) => {
                up.push(LevelDuration::new(false, long));
                in_mid1 = false;
            }
            (false, true) => {
                up.push(LevelDuration::new(true, long));
                in_mid1 = true;
            }
            (false, false) => up.extend([LevelDuration::new(true, short), LevelDuration::new(false, short)]),
        }
    }
    let end = LevelDuration::new(false, short.wrapping_add((3 * GAP_TE).wrapping_mul(te)));
    if in_mid1 {
        up.push(end);
    } else {
        *up.last_mut()? = end;
    }
    Some(up)
}

/// Key files and transmissions only: `feed` never reports (see the module doc).
pub struct FiatMarelliDecoder;

impl FiatMarelliDecoder {
    pub fn new() -> Self {
        Self
    }
}

impl Default for FiatMarelliDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for FiatMarelliDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE_DEFAULT, te_long: 2 * TE_DEFAULT, te_delta: 80, min_count_bit: MIN_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[433_920_000]
    }

    fn reset(&mut self) {}

    /// No decoding, as FoxFW (no Decodable flag): Flipper-ARF's decoder reads Fiat V1 and Ford V3
    /// frames two bits early.
    fn feed(&mut self, _level: bool, _duration_us: u32) -> Option<DecodedSignal> {
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// The saved frame, 3 times, whatever `button` asks (replay only, as the C).
    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        let packed = decoded.extra?;
        let (te, extra) = ((packed >> 32) as u32, packed as u32);
        let raw = rebuild(decoded.data, extra, decoded.data_count_bit, true);
        let one = upload(&raw, decoded.data_count_bit, te)?;
        Some(one.iter().copied().cycle().take(REPEAT * one.len()).collect())
    }

    /// `subghz_protocol_decoder_fiat_marelli_deserialize`.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits > 0).then(|| {
            decode_of(key.key, key.u32_field("Extra").unwrap_or(0), key.bits, key.u32_field("TE").unwrap_or(0))
        })
    }

    /// `subghz_protocol_decoder_fiat_marelli_serialize`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let packed = decoded.extra?;
        let mut key = FlipperKey::for_protocol(NAME, decoded.data_count_bit, decoded.data);
        key.push_u32("Extra", packed as u32);
        key.push_u32("Extra_bits", decoded.data_count_bit.saturating_sub(64) as u32);
        key.push_u32("TE", (packed >> 32) as u32);
        key.push_u32("Serial", decoded.serial.unwrap_or(0));
        key.push_u32("Btn", decoded.button.unwrap_or(0).into());
        key.push_u32("Cnt", decoded.counter.unwrap_or(0).into());
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    /// A Flipper-ARF key file (the layout its serialize writes; made-up values).
    fn key_file(bits: usize, key: &str, extra: u32, te: u32) -> FlipperKey {
        let extra_bits = bits.saturating_sub(64);
        FlipperKey::parse(&format!(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPreset2FSKDev476Async\nProtocol: MARELLI\nBit: {bits}\nKey: {key}\n\
             Extra: {extra}\nExtra_bits: {extra_bits}\nTE: {te}\nSerial: 1\nBtn: 2\nCnt: 3\n"
        ))
        .unwrap()
    }

    /// The registry loads a MARELLI file and shows the C's fields; nothing decodes it on air.
    #[test]
    fn key_files_load_through_the_registry_with_the_c_fields() {
        let reg = ProtocolRegistry::new();
        let key = key_file(96, "FF FD 12 34 56 78 B5 98", 0x1234_5678, 104);
        let (name, d) = reg.decode_key(&key).expect("MARELLI loads");
        assert_eq!(name, "MARELLI");
        // serial = bytes 2..5, button = byte 6 >> 4, counter = byte 7 >> 3 & 0x1F.
        assert_eq!((d.serial, d.button, d.counter), (Some(0x1234_5678), Some(0xB), Some(0x13)));
        assert_eq!((d.data, d.data_count_bit, d.extra), (0xFFFD_1234_5678_B598, 96, Some((104 << 32) | 0x1234_5678)));
        assert!(d.encoder_capable && !d.crc_valid);
        assert_eq!(INFO.button_name("MARELLI", 0xB), Some("Unlock"));
        // Saved again, it reads back the same; the file text keeps Flipper-ARF's line order.
        let dec = reg.get("MARELLI").unwrap();
        let (text, back) = crate::protocols::flipper_key::testing::round_trip(dec, &d).unwrap();
        assert_eq!(back, d);
        let order: Vec<&str> = text.lines().filter_map(|l| l.split(':').next()).collect();
        assert_eq!(order[order.len() - 8..], ["Bit", "Key", "Extra", "Extra_bits", "TE", "Serial", "Btn", "Cnt"]);
        // A file without TE sends at the C's default 260 µs.
        let no_te = FlipperKey::parse(&key.to_text().replace("TE: 104\n", "")).unwrap();
        let d = dec.from_flipper_key(&no_te).unwrap();
        assert_eq!(dec.encode(&d, 1).unwrap()[0].duration_us, 260);
    }

    #[test]
    fn nothing_decodes_on_air() {
        let mut dec = FiatMarelliDecoder::new();
        let key = key_file(81, "FF FD A3 DE E5 E1 83 8B", 0x3710, 136);
        let d = dec.from_flipper_key(&key).unwrap();
        let tx = dec.encode(&d, 0).unwrap();
        assert!(tx.iter().all(|p| dec.feed(p.level, p.duration_us).is_none()));
        assert!(dec.flush().is_none());
    }

    /// The upload of the C's encoder, checked against Flipper-ARF's `fiat_marelli.c` on the x86 host
    /// build (`subghz_oracle_arf encode`): for this 88-bit file its `RAW_Data` is 3 copies of 199
    /// preamble pulses of 120 µs, a LOW of 1560, a HIGH of 960, the 88 bits below and a final LOW
    /// of 4440 µs.
    #[test]
    fn upload_matches_the_reference_encoder() {
        let dec = FiatMarelliDecoder::new();
        let key = key_file(88, "FF FC 5A 3C 96 E1 0F 2D", 0x0081_F0E1, 120);
        let d = dec.from_flipper_key(&key).unwrap();
        let signed = |v: &[LevelDuration]| -> Vec<i64> {
            v.iter().map(|p| if p.level { i64::from(p.duration_us) } else { -i64::from(p.duration_us) }).collect()
        };
        let tx = signed(&dec.encode(&d, 0x08).unwrap());
        let one = tx.len() / REPEAT;
        assert_eq!(tx.len() % REPEAT, 0);
        assert!(tx[..one] == tx[one..2 * one] && tx[..one] == tx[2 * one..]);
        let copy = &tx[..one];
        assert!(copy[..199].iter().enumerate().all(|(i, &v)| v == if i % 2 == 0 { 120 } else { -120 }));
        assert_eq!(copy[199..201], [-1560, 960]);
        assert_eq!(&copy[201..], ORACLE_88_DATA);
        // Replay only: the button asked for changes nothing.
        assert_eq!(dec.encode(&d, 0x01).map(|t| signed(&t)), Some(tx));
    }

    /// `subghz_oracle_arf encode` of the 88-bit file above: the pulses after the sync.
    const ORACLE_88_DATA: &[i64] = &[
        -120, 120, -120, 120, -120, 120, -120, 120, -120, 120, -120, 120, -120, 120, -120, 120, -120, 120, -120,
        120, -120, 120, -120, 120, -120, 120, -120, 120, -240, 120, -120, 120, -120, 240, -240, 240, -120, 120,
        -240, 240, -240, 120, -120, 120, -120, 240, -120, 120, -120, 120, -120, 120, -240, 120, -120, 240, -240,
        120, -120, 240, -240, 240, -120, 120, -240, 240, -120, 120, -120, 120, -240, 120, -120, 120, -120, 120,
        -120, 240, -240, 120, -120, 120, -120, 120, -120, 240, -120, 120, -120, 120, -120, 120, -240, 120, -120,
        240, -240, 240, -120, 120, -240, 240, -120, 120, -240, 120, -120, 120, -120, 120, -120, 120, -120, 120,
        -120, 240, -120, 120, -120, 120, -120, 120, -120, 120, -240, 120, -120, 120, -120, 120, -120, 240, -120,
        120, -120, 120, -240, 120, -120, 120, -120, 120, -120, 240, -4440,
    ];

    /// 104 bits: `Extra` holds bits 72..103, the C rebuilds bits 72..95 from it, sends bits 64..71 and
    /// 96..103 as 0 on a Flipper, and recomputes byte 12 as the CRC of the rebuilt bytes.
    #[test]
    fn long_frames_go_out_as_a_flipper_rebuilds_them() {
        let raw = rebuild(0x0123_4567_89AB_CDEF, 0xA5C3_F00F, 104, true);
        assert_eq!(raw[8..12], [0x00, 0xA5, 0xC3, 0xF0]);
        assert_eq!(raw[12], crc8(&raw[..12]));
        // Up to 96 bits nothing is lost: 88 bits keep all 24 extra bits.
        assert_eq!(rebuild(0, 0x00AB_CDEF, 88, true)[8..12], [0xAB, 0xCD, 0xEF, 0x00]);
        // Outside 80..=104 bits the C's upload fails: no transmission.
        let dec = FiatMarelliDecoder::new();
        for bits in [79, 105] {
            let d = dec.from_flipper_key(&key_file(bits, "FF FD 00 00 00 01 70 00", 0, 260)).unwrap();
            assert!(!d.encoder_capable && dec.encode(&d, 0).is_none(), "{bits}");
        }
    }

    /// Generator for the oracle comparison: `KAT_MARELLI_ENCODE=<file listing key files>` prints KAT's
    /// transmission for each (`KATENC <file> | <signed pulses>`), to diff with `subghz_oracle_arf
    /// encode --registry all <file>`.
    #[test]
    #[ignore = "oracle harness"]
    fn encode_key_files() {
        let Ok(list) = std::env::var("KAT_MARELLI_ENCODE") else { return };
        let dec = FiatMarelliDecoder::new();
        for file in std::fs::read_to_string(list).unwrap().lines().filter(|l| !l.is_empty()) {
            let key = FlipperKey::parse(&std::fs::read_to_string(file).unwrap()).unwrap();
            let out = dec.from_flipper_key(&key).and_then(|d| dec.encode(&d, 0));
            let text = out.map_or("(none)".to_string(), |tx| {
                tx.iter()
                    .map(|p| if p.level { p.duration_us.to_string() } else { format!("-{}", p.duration_us) })
                    .collect::<Vec<_>>()
                    .join(" ")
            });
            println!("KATENC {file} | {text}");
        }
    }
}
