//! Continental KR5V2X car remote decoder (receive only; FoxFW key files).
//!
//! Port of rtl_433 `src/devices/continental_car_remote.c` (`continental_car_remote_decode`,
//! r_device "Continental KR5V2X Car Remote (-f 313.8M -s 1024k)", model `Continental-KR5V2X`):
//! Continental 72147-SNA-A01, FCC ID KR5V2X, an OEM remote for Honda (313.8 MHz). rtl_433 calls it a
//! rolling code with an unencrypted sequence number.
//!
//! **Modulation** — FSK, rtl_433's Manchester zero-bit slicer (`FSK_PULSE_MANCHESTER_ZEROBIT`,
//! short 100 µs, long 200 µs, reset limit 1500 µs, no tolerance), see [super::rtl433]: the upper
//! tone is HIGH, a falling data edge is `1`, a rising one `0`, every message starts with the
//! slicer's hard-coded `0`, and a lower-tone stretch over 1.5 ms (or the end of the carrier) ends
//! the message. A press sends a long wake-up (`0x87` repeated, ~1000 bits) with the frame, then the
//! frame again after a ~1.6 ms gap; Trunk/Panic long presses repeat that every ~95 ms.
//!
//! **Frame** — the decoder looks for the last 20 bits of the preamble, `0xF0F06`, in row 0 and reads
//! the 112 bits after it (14 bytes, MSB first):
//!
//! | bytes | field |
//! |-------|-------|
//! | 0..3  | remote ID (32 bits) |
//! | 4     | unknown |
//! | 5     | button (high nibble), unknown (low nibble) |
//! | 6..8  | sequence (24 bits, plain counter) |
//! | 9..12 | encrypted code (32 bits) |
//! | 13    | checksum: the 14 bytes XOR to 0 |
//!
//! **Gating** (the C's checks, in order; a frame failing one is dropped, as rtl_433 returns
//! `DECODE_ABORT_*` / `DECODE_FAIL_SANITY` / `DECODE_FAIL_MIC`): row 0 has ≥ 132 bits; ≥ 112 bits
//! follow the pattern (when it is missing `bitbuffer_search` returns the row length, which fails
//! this too); ID, button and sequence non-zero, ID and encrypted ≠ `0x0FFFFFFF` (the C compares the
//! 32-bit values with seven F's; kept as is, so an all-ones ID passes) and sequence ≠ `0xFFFFFF`;
//! then the XOR checksum. rtl_433 outputs `mic: CHECKSUM`.
//!
//! **Field mapping** — `serial` = ID (rtl_433 `id`, printed `%08X`), `button` = the button nibble
//! (`button_code`: 1 Lock, 3 Unlock, 9 Trunk Long Press, 0xA Trunk/Panic Short Press, 0xB Panic
//! Long Press, others "?"), `counter` = the low 16 bits of the 24-bit `sequence`, `crc_valid` = true
//! (the checksum passed). `data` = bytes 0..7, `extra` = bytes 8..13 (48 bits, right-aligned),
//! `data_count_bit` = 112: rtl_433's `sequence` = `(data & 0xFFFF) << 8 | extra >> 40` and
//! `encrypted` = `(extra >> 8) & 0xFFFFFFFF`.
//!
//! **Key files** — FoxFW's (`lib/subghz/protocols/continental_rke.c`, its own port of the same
//! rtl_433 decoder; `CONTINENTAL_RKE_PROTOCOL_NAME` "Continental RKE", an alias here), written by
//! its generic serializer: `Protocol: Continental RKE`, `Bit: 64`, `Key` = bytes 0..7 (ID, byte 4,
//! the button byte, the sequence's top 16 bits) — `data`. FoxFW keeps nothing else in its file: the
//! sequence's low byte, the encrypted code and the checksum (bytes 8..13) are lost, and FoxFW shows
//! `Cnt` / `Enc` 0 for a loaded file. So KAT adds one line after FoxFW's for a received frame:
//! `Raw` = all 14 bytes. FoxFW's `deserialize` reads only `Bit` and `Key` and ignores it (checked on
//! FoxFW's C), so KAT's files still open there as FoxFW's own. Loading is FoxFW's `deserialize`
//! (exactly `Bit: 64`), then: with `Raw`, the frame's checks (the sanity checks and the XOR
//! checksum) on `Raw`, which must also start with the key's 8 bytes, giving the received decode
//! back exactly (a `Raw` that fails is refused); without `Raw` (FoxFW's own files), the decode of
//! what the file holds: `data` = the key, `data_count_bit` = 64, `serial` = ID, `button`, no
//! counter (its low byte is gone), `crc_valid` false (shown as "-", not FAIL), no `extra`, which
//! writes the same file back.
//!
//! **Encoder** — none. rtl_433 has none, and FoxFW's SD encoder plugin
//! (`applications/main/subghz/protocols_ext/enc_continental_rke.c`) is a stub ("rolling code /
//! encrypted — no TX"): its `deserialize` returns `SubGhzProtocolStatusErrorEncoderGetUpload` for
//! every key file, so a Flipper running FoxFW sends nothing. `supports_encoding()` is false, `encode`
//! returns None and nothing is encoder-capable; the app offers Replay of the recorded pulses.
//!
//! rtl_433 also has an alternate decoder for the same FCC ID, "Honda Keyfob KR5V2X/1X"
//! (`honda_kr5v2x1x.rs`, 60/120 µs, a different layout and CRC); it decodes none of the
//! Continental recordings.

use super::rtl433::{frame_signal, xor_bytes, BitBuffer, Decode, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "Continental-KR5V2X";

/// FoxFW's protocol name (`CONTINENTAL_RKE_PROTOCOL_NAME`), written in key files.
const FOXFW_NAME: &str = "Continental RKE";

/// FoxFW's key length (`CONTINENTAL_RKE_KEY_BITS`): bytes 0..7 of the frame.
const KEY_BITS: usize = 64;

/// Metadata (see `info.rs`): FSK (2-FSK, ~±45 kHz on the recordings → FM476), receive only.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Continental KR5V2X Car Remote", FOXFW_NAME],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Honda/Acura",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&FM476],
};

/// rtl_433's `button_str`.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x1 => "Lock",
        0x3 => "Unlock",
        0x9 => "Trunk Long Press",
        0xA => "Trunk/Panic Short Press",
        0xB => "Panic Long Press",
        _ => "Other",
    })
}

/// r_device timing: `short_width` (half-bit), `long_width`, `reset_limit` (µs).
const SHORT_US: u32 = 100;
const LONG_US: u32 = 200;
const RESET_US: u32 = 1500;
const FRAME_BITS: usize = 112;

/// The `r_device`; a burst's FM pairs are one package, consecutive LOWs one gap (see `rtl433`).
const DEVICE: Device = Device {
    modulation: Modulation::FskManchesterZerobit,
    short_width: SHORT_US,
    long_width: LONG_US,
    reset_limit: RESET_US,
    gap_limit: 0,
    sync_width: 0,
    tolerance: 0,
    gaps: Gaps::Merged,
    ends: Ends::None,
    resync: false,
};

/// `continental_car_remote_decode` on one message's bitbuffer.
fn decode(bits: &BitBuffer) -> Option<DecodedSignal> {
    let len = bits.bits_per_row(0);
    if len < 132 {
        return None; // DECODE_ABORT_LENGTH
    }
    let offset = bits.search(0, 0, &[0xF0, 0xF0, 0x60], 20) + 20;
    if len < offset + FRAME_BITS {
        return None; // DECODE_ABORT_EARLY
    }
    let mut b = [0u8; 14];
    bits.extract_bytes(0, offset, &mut b, FRAME_BITS);
    check(&b)
}

/// The C's checks on the frame's 14 bytes (sanity, then the XOR checksum) and the decode they give.
fn check(b: &[u8; 14]) -> Option<DecodedSignal> {
    let id = u32::from_be_bytes([b[0], b[1], b[2], b[3]]);
    let button = b[5] >> 4;
    let sequence = u32::from_be_bytes([0, b[6], b[7], b[8]]);
    let encrypted = u32::from_be_bytes([b[9], b[10], b[11], b[12]]);
    if id == 0
        || button == 0
        || sequence == 0
        || id == 0x0FFF_FFFF
        || encrypted == 0x0FFF_FFFF
        || sequence == 0xFF_FFFF
    {
        return None; // DECODE_FAIL_SANITY
    }
    if xor_bytes(b) != 0 {
        return None; // DECODE_FAIL_MIC
    }
    let mut d = frame_signal(b, FRAME_BITS);
    d.serial = Some(id);
    d.button = Some(button);
    d.counter = Some(sequence as u16);
    d.crc_valid = true;
    Some(d)
}

/// Continental KR5V2X decoder: rtl_433's zero-bit slicer + `continental_car_remote_decode`.
pub struct ContinentalKr5v2xDecoder {
    rx: Receiver,
}

impl ContinentalKr5v2xDecoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE) }
    }
}

impl Default for ContinentalKr5v2xDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for ContinentalKr5v2xDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    /// Manchester half-bit / full bit; the slicer splits them at 1.5 half-bits (± 50 µs).
    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: SHORT_US,
            te_long: LONG_US,
            te_delta: SHORT_US / 2,
            min_count_bit: FRAME_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.rx.reset(&mut Decode(decode));
    }

    fn feed(&mut self, level: bool, duration_us: u32) -> Option<DecodedSignal> {
        self.rx.feed(&mut Decode(decode), level, duration_us)
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    /// None: rtl_433 has no encoder and FoxFW's plugin refuses every key (see "Encoder").
    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// FoxFW's `subghz_protocol_decoder_continental_rke_deserialize` (`Bit: 64`), plus KAT's `Raw`
    /// line (see "Key files"): with `Raw`, the decode of the whole frame, which must start with the
    /// key's 8 bytes and pass the checks; without it, the decode of bytes 0..7 alone (FoxFW's own
    /// files).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != KEY_BITS {
            return None;
        }
        let k = key.key_u64()?;
        if key.field("Raw").is_some() {
            let raw: [u8; 14] = key.hex_bytes_field("Raw")?.try_into().ok()?;
            return (raw[..8] == k.to_be_bytes()).then(|| check(&raw)).flatten();
        }
        let mut d = DecodedSignal::new(k, KEY_BITS);
        d.serial = Some((k >> 32) as u32);
        d.button = Some((k >> 20) as u8 & 0xF);
        Some(d)
    }

    /// FoxFW's `subghz_protocol_decoder_common_serialize` of the frame (`Bit: 64`, `Key` = bytes
    /// 0..7, `data`), for a received frame or one loaded from such a file; for a whole frame, then
    /// `Raw` = its 14 bytes (KAT's line, see "Key files").
    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        if ![FRAME_BITS, KEY_BITS].contains(&d.data_count_bit) {
            return None;
        }
        let mut key = FlipperKey::for_protocol(FOXFW_NAME, KEY_BITS, d.data);
        if let (FRAME_BITS, Some(extra)) = (d.data_count_bit, d.extra) {
            let mut raw = [0u8; 14];
            raw[..8].copy_from_slice(&d.data.to_be_bytes());
            raw[8..].copy_from_slice(&extra.to_be_bytes()[2..]);
            if extra >> 48 == 0 && check(&raw).is_some() {
                key.push_hex_bytes("Raw", &raw);
            }
        }
        Some(key)
    }
}

/// rtl_433's JSON fields of a decode, rebuilt from KAT's (the field mapping, for the parity tests).
#[cfg(test)]
pub(super) fn rtl433_fields(d: &DecodedSignal) -> String {
    let extra = d.extra.unwrap_or(0);
    format!(
        "id={:08X} encrypted={:08X} sequence={} button_code={}",
        d.serial.unwrap_or(0),
        (extra >> 8) & 0xFFFF_FFFF,
        (d.data & 0xFFFF) << 8 | extra >> 40,
        d.button.unwrap_or(0),
    )
}

#[cfg(test)]
mod tests {
    use super::super::rtl433::test_util::*;
    use super::*;

    /// The 14 frame bytes for these fields, checksum included.
    fn frame(id: u32, button: u8, sequence: u32, encrypted: u32) -> [u8; 14] {
        let mut b = [0u8; 14];
        b[..4].copy_from_slice(&id.to_be_bytes());
        b[4] = 0x10;
        b[5] = button << 4;
        b[6..9].copy_from_slice(&sequence.to_be_bytes()[1..]);
        b[9..13].copy_from_slice(&encrypted.to_be_bytes());
        b[13] = xor_bytes(&b[..13]);
        b
    }

    /// Message bits like the recordings' repeat (`00 F0 F0 F0 6` + frame), after a wake-up of
    /// `wake` `0x87` bytes (the first message of a press).
    fn message(frame: &[u8; 14], wake: usize) -> Vec<bool> {
        let mut bytes = vec![0x00];
        bytes.extend(std::iter::repeat_n(0x87, wake));
        bytes.extend([0xF0, 0xF0, 0xF0, 0x60]);
        let head = bytes.len() * 8 - 4;
        let mut bits = bits_of_bytes(&bytes, head);
        bits.extend(bits_of_bytes(frame, 112));
        bits
    }

    fn air(frame: &[u8; 14], wake: usize) -> Vec<LevelDuration> {
        with_idle(
            &wire(&manchester_zerobit(&message(frame, wake), SHORT_US, 0)),
            100_000,
        )
    }

    #[test]
    fn decodes_frames_from_the_c_layout() {
        for (button, wake) in [
            (0x1, 0),
            (0x3, 3),
            (0x9, 0),
            (0xA, 125),
            (0xB, 140),
            (0x5, 0),
        ] {
            let f = frame(0xF42E_6789, button, 1522, 0x9391_771C);
            let got = decode_all(&mut ContinentalKr5v2xDecoder::new(), &air(&f, wake));
            assert_eq!(got.len(), 1, "button {button:#x}, wake-up {wake}");
            let d = &got[0];
            assert_eq!(
                (d.serial, d.button, d.counter, d.crc_valid),
                (Some(0xF42E_6789), Some(button), Some(1522), true)
            );
            assert_eq!((d.data_count_bit, d.encoder_capable), (112, false));
            let p = frame_signal(&f, 112);
            assert_eq!((p.data, p.extra), (d.data, d.extra));
            assert_eq!(
                rtl433_fields(d),
                format!("id=F42E6789 encrypted=9391771C sequence=1522 button_code={button}")
            );
        }
        // A 24-bit sequence: the counter keeps its low 16 bits, `extra` the rest.
        let d = &decode_all(
            &mut ContinentalKr5v2xDecoder::new(),
            &air(&frame(0x2A79_1780, 1, 0x12_D483, 0xC5B7_F6D2), 0),
        )[0];
        assert_eq!(
            (
                d.counter,
                rtl433_fields(d).contains(&format!("sequence={}", 0x12_D483))
            ),
            (Some(0xD483), true)
        );
        // The C's seven-F sanity constant: an all-ones ID passes, 0x0FFFFFFF does not.
        assert_eq!(
            decode_all(
                &mut ContinentalKr5v2xDecoder::new(),
                &air(&frame(u32::MAX, 1, 7, 1), 0)
            )
            .len(),
            1
        );
        assert!(decode_all(
            &mut ContinentalKr5v2xDecoder::new(),
            &air(&frame(0x0FFF_FFFF, 1, 7, 1), 0)
        )
        .is_empty());
    }

    #[test]
    fn rejects_bad_frames() {
        let good = frame(0xF42E_6789, 1, 1522, 0x9391_771C);
        let mut dec = ContinentalKr5v2xDecoder::new();
        assert_eq!(decode_all(&mut dec, &air(&good, 0)).len(), 1);
        // Checksum, sanity (zero ID / button / sequence, seven-F encrypted, all-ones sequence).
        let mut bad = good;
        bad[7] ^= 0x04;
        assert!(decode_all(&mut dec, &air(&bad, 0)).is_empty(), "checksum");
        for f in [
            frame(0, 1, 1522, 1),
            frame(0xF42E_6789, 0, 1522, 1),
            frame(0xF42E_6789, 1, 0, 1),
            frame(0xF42E_6789, 1, 1522, 0x0FFF_FFFF),
            frame(0xF42E_6789, 1, 0xFF_FFFF, 1),
        ] {
            assert!(decode_all(&mut dec, &air(&f, 0)).is_empty(), "{f:02X?}");
        }
        // Wrong preamble, a truncated frame, the wrong rate.
        let mut bits = message(&good, 0);
        bits[34] = !bits[34];
        assert!(
            decode_all(
                &mut dec,
                &with_idle(&wire(&manchester_zerobit(&bits, SHORT_US, 0)), 100_000)
            )
            .is_empty(),
            "preamble"
        );
        let bits = message(&good, 0);
        let short = &bits[..bits.len() - 3];
        assert!(
            decode_all(
                &mut dec,
                &with_idle(&wire(&manchester_zerobit(short, SHORT_US, 0)), 100_000)
            )
            .is_empty(),
            "truncated"
        );
        assert!(
            decode_all(
                &mut dec,
                &with_idle(&wire(&manchester_zerobit(&bits, 160, 0)), 100_000)
            )
            .is_empty(),
            "160 µs half-bits"
        );
        // A lower-tone stretch over the reset limit inside the frame ends the message there.
        let mut wire = wire(&manchester_zerobit(&bits, SHORT_US, 0));
        let k = (wire.len() * 3 / 4..).find(|&i| !wire[i].level).unwrap();
        wire[k].duration_us = 1_600;
        assert!(
            decode_all(&mut dec, &with_idle(&wire, 100_000)).is_empty(),
            "split by a gap"
        );
    }

    #[test]
    fn registry_reports_it_on_fm() {
        let f = frame(0x2A79_1780, 3, 54405, 0xB900_57D2);
        let got = registry(
            &air(&f, 125),
            313_800_000,
            Some(crate::capture::RfModulation::FM),
        );
        assert_eq!(
            got.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>(),
            [NAME]
        );
        assert_eq!(got[0].1.serial, Some(0x2A79_1780));
    }

    /// A made-up frame whose RAW file (written to `$KAT_SYNTH_DIR` when set) FoxFW's decoder read with
    /// `subghz_oracle_foxfwenc decode --protocol "Continental RKE"`: KAT writes FoxFW's lines (bytes
    /// 0..7), then `Raw` (the whole frame), and reads its file back to the received decode. FoxFW's
    /// own file (no `Raw`) gives the 64-bit decode of bytes 0..7, which writes that file back.
    #[test]
    fn key_files_are_foxfws() {
        const FOXFW: &str = "Frequency: 313800000; Preset: FuriHalSubGhzPreset2FSKDev476Async; Protocol: Continental RKE; Bit: 64; Key: 12 34 56 78 10 30 AB CD";
        let f = frame(0x1234_5678, 0x3, 0xAB_CDEF, 0x9ABC_DEF0);
        let air = vector_wire(&f);
        crate::protocols::foxfw_common::test_support::dump("continental_unit.sub", 313_800_000, "FuriHalSubGhzPreset2FSKDev476Async", &air);
        let got = decode_all(&mut ContinentalKr5v2xDecoder::new(), &air);
        assert_eq!(got.len(), 2, "sent twice");
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter, d.crc_valid, d.data_count_bit), (Some(0x1234_5678), Some(3), Some(0xCDEF), true, 112));
        let dec = ContinentalKr5v2xDecoder::new();
        let mut key = dec.to_flipper_key(d).unwrap();
        key.frequency = 313_800_000;
        key.preset = Some("FuriHalSubGhzPreset2FSKDev476Async".into());
        let raw: Vec<String> = f.iter().map(|b| format!("{b:02X}")).collect();
        assert_eq!(key.to_text().lines().skip(2).collect::<Vec<_>>().join("; "), format!("{FOXFW}; Raw: {}", raw.join(" ")));
        // KAT's file gives the received decode back: sequence 0xABCDEF, the code and the checksum.
        crate::protocols::flipper_key::testing::assert_round_trip(&dec, d);
        // FoxFW's file holds bytes 0..7 (ID 12345678, byte 4 0x10, button 3, the sequence's top 16
        // bits 0xABCD); the sequence's low byte (0xEF) and the code are not in it.
        let mut held = DecodedSignal::new(0x1234_5678_1030_ABCD, 64);
        (held.serial, held.button) = (Some(0x1234_5678), Some(3));
        let text = format!("Filetype: Flipper SubGhz Key File\nVersion: 1\n{}\n", FOXFW.replace("; ", "\n"));
        assert_eq!(dec.from_flipper_key(&crate::protocols::flipper_key::testing::key(&text)), Some(held.clone()));
        let written = crate::protocols::flipper_key::testing::assert_round_trip(&dec, &held);
        assert!(!written.contains("Raw"), "a 64-bit decode writes FoxFW's file only");
        // shown as unchecked ("-"), not as a failed checksum
        assert!(crate::protocols::has_checksum_for(NAME, 112, d.extra));
        assert!(!crate::protocols::has_checksum_for(NAME, 64, None));
        let reg = super::super::ProtocolRegistry::new();
        assert_eq!(reg.get(FOXFW_NAME).map(|p| p.name()), Some(NAME));
    }

    /// Keys FoxFW's `deserialize` refuses (any `Bit` but 64, KAT's own 112-bit frames included), and
    /// a `Raw` line that does not hold the key's frame or fails the frame's checks.
    #[test]
    fn refuses_other_keys() {
        let dec = ContinentalKr5v2xDecoder::new();
        let key = |bits: usize| FlipperKey::new(FOXFW_NAME, bits, 0x1234_5678_1030_00AB, 313_800_000, "FuriHalSubGhzPreset2FSKDev476Async");
        assert_eq!(dec.from_flipper_key(&key(64)).and_then(|d| d.serial), Some(0x1234_5678));
        for bits in [112, 63, 65, 32] {
            assert!(dec.from_flipper_key(&key(bits)).is_none(), "Bit {bits}");
        }
        let f = frame(0x1234_5678, 0x3, 0xAB_CDEF, 0x9ABC_DEF0);
        let with_raw = |key_word: u64, raw: &[u8]| {
            let mut k = FlipperKey::new(FOXFW_NAME, 64, key_word, 313_800_000, "FuriHalSubGhzPreset2FSKDev476Async");
            k.push_hex_bytes("Raw", raw);
            dec.from_flipper_key(&k)
        };
        let word = u64::from_be_bytes(f[..8].try_into().unwrap());
        assert_eq!(with_raw(word, &f).map(|d| (d.counter, d.data_count_bit)), Some((Some(0xCDEF), 112)));
        assert!(with_raw(word ^ 1 << 40, &f).is_none(), "Raw is another frame than Key");
        assert!(with_raw(word, &f[..13]).is_none(), "Raw too short");
        let mut bad = f;
        bad[11] ^= 0x01;
        assert!(with_raw(word, &bad).is_none(), "Raw fails the checksum");
        assert!(dec.to_flipper_key(&DecodedSignal::new(0, 76)).is_none());
    }

    /// FoxFW's encoder plugin refuses every Continental RKE key (`subghz_oracle_foxfwenc encode`:
    /// "deserialize failed (status -12)" for each of 120 made-up files) and rtl_433 has no encoder:
    /// nothing transmits, for no button.
    #[test]
    fn transmits_nothing_as_foxfw() {
        let dec = ContinentalKr5v2xDecoder::new();
        assert!(!dec.supports_encoding());
        let d = decode_all(&mut ContinentalKr5v2xDecoder::new(), &vector_wire(&frame(0x1234_5678, 0x1, 7, 1))).remove(0);
        let loaded = dec.from_flipper_key(&dec.to_flipper_key(&d).unwrap()).unwrap();
        for x in [&d, &loaded] {
            assert!(!x.encoder_capable);
            for button in [0x01, 0x02, 0x04, 0x08, x.button.unwrap()] {
                assert!(dec.encode(x, button).is_none());
            }
        }
    }

    /// A RAW file's pulses for a frame: [air] after a wake-up of 6 `0x87` bytes, sent twice, 20 ms
    /// of LOW around and between.
    fn vector_wire(f: &[u8; 14]) -> Vec<LevelDuration> {
        let one = wire(&manchester_zerobit(&message(f, 6), SHORT_US, 20_000));
        let mut both = one.clone();
        both.extend(one);
        with_idle(&both, 20_000)
    }

    /// Made-up frames for the FoxFW comparison, as RAW files in `$KAT_SYNTH_DIR`: every named button
    /// and others, random IDs, byte 4 values, sequences and codes.
    #[test]
    #[ignore = "oracle harness"]
    fn dump_key_vectors() {
        let mut rng = Rng(0xC0_4752);
        for i in 0..150u64 {
            let button = if i < 60 { [0x1, 0x3, 0x9, 0xA, 0xB][(i % 5) as usize] } else { 1 + rng.below(15) as u8 };
            let mut f = frame(rng.next() as u32 | 1, button, 1 + rng.below(0xFF_FFFE) as u32, rng.next() as u32);
            f[4] = rng.below(256) as u8;
            f[5] |= rng.below(16) as u8;
            f[13] = xor_bytes(&f[..13]);
            crate::protocols::foxfw_common::test_support::dump(&format!("continental_{i:03}.sub"), 313_800_000, "FuriHalSubGhzPreset2FSKDev476Async", &vector_wire(&f));
        }
    }

    /// `KAT_CONTINENTAL_RAW=<file listing RAW .sub files>`: KAT's key-file lines for each frame this
    /// decoder reads in them, to diff with `subghz_oracle_foxfwenc decode --protocol "Continental RKE"`.
    #[test]
    #[ignore = "oracle harness"]
    fn key_lines_of_raw_files() {
        print_raw_key_lines("KAT_CONTINENTAL_RAW", &mut ContinentalKr5v2xDecoder::new());
    }

    /// `KAT_CONTINENTAL_ENCODE=<file listing key files>`: KAT's transmission of each (`(none)`: it
    /// never transmits), to diff with `subghz_oracle_foxfwenc encode <file>`, which refuses each.
    #[test]
    #[ignore = "oracle harness"]
    fn encode_key_files() {
        print_key_file_encodes("KAT_CONTINENTAL_ENCODE", &ContinentalKr5v2xDecoder::new());
    }

    /// rtl_433_tests `continental/72147-SNA-A01`: the seven recordings decode through KAT's IQ
    /// pipeline with the fields of rtl_433's `.json` files.
    #[test]
    #[ignore = "IQ recordings, ~5 s in a debug build"]
    fn decodes_rtl433_recordings() {
        let json = |v: &serde_json::Value| {
            format!(
                "id={} encrypted={} sequence={} button_code={}",
                v["id"].as_str().unwrap(),
                v["encrypted"].as_str().unwrap(),
                v["sequence"],
                v["button_code"]
            )
        };
        if let Some(n) = check_recordings(&["continental"], NAME, NAME, rtl433_fields, json) {
            assert_eq!(n, 7);
        }
    }
}
