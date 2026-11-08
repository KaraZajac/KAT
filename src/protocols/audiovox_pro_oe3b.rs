//! Audiovox PRO-OE3B / PRO-OE4B car remote (receive only; FoxFW key files).
//!
//! Port of rtl_433 `src/devices/audiovox_pro_oe3b.c` (`r_device` "Audiovox PRO-OE3B Car Remote",
//! model `Audiovox-PROOE3B`; FCC ID BGAOE3B, AVX01BT3CL3, ~303 MHz). FoxFW has its own port of the
//! same decoder ("Audiovox PRO", see **Key files**); no other Flipper reference has it.
//!
//! **Modulation** — `OOK_PULSE_PWM`: short_width 445 µs (pulse → 1), long_width 895 µs (→ 0),
//! sync_width 1368 µs, reset_limit = gap_limit = 1790 µs, no tolerance, so the slicer's split points
//! apply: a pulse up to 670 µs is a 1, 671–1131 µs a 0, longer a sync (a new row). A gap > 1790 µs
//! ends the message. rtl_433 runs it at priority 10 (only when no priority-0 decoder decoded the
//! package); in KAT it is in the last registry block anyway.
//!
//! **Frame** (`audiovox_pro_oe3b_decode`) — exactly one row of exactly 25 bits. Checks
//! (`DECODE_FAIL_SANITY`): on the wire bits, byte 2 has none of the 0xAA bits set and is not 0x55;
//! after inverting every bit, the 16-bit ID (bytes 0–1) is not 0 or FFFF, and at least one button
//! flag of inverted byte 2 is set: 0x01 Lock, 0x04 Unlock, 0x10 Option, 0x40 Trunk (several can be
//! pressed at once; rtl_433 joins the names with "; "). Fixed code, repeated while held.
//!
//! **Fields** — `serial` = the ID, `button` = the pressed flags packed in rtl_433's order: bit 0
//! Lock, bit 1 Unlock, bit 2 Option, bit 3 Trunk (rtl_433 prints only the names). `data` = the 25
//! wire bits (before the decoder's inversion) right-aligned, `data_count_bit` = 25. `crc_valid` =
//! false (no integrity check in the C).
//!
//! **Key files** — FoxFW's (`lib/subghz/protocols/audiovox.c`, `AUDIOVOX_PROTOCOL_NAME` "Audiovox
//! PRO", an alias here), written by its generic serializer: `Protocol: Audiovox PRO`, `Bit: 25`,
//! `Key` (8 bytes) = the 25 wire bits, i.e. `data` (FoxFW keeps the button as the inverted byte's
//! flag bits, `& 0x55`, but does not save it). Loading is FoxFW's `deserialize` (exactly `Bit: 25`),
//! then the decoder's sanity checks — FoxFW saves only frames that pass them; a key that fails
//! them, or has bits set above its 25, is refused. The file round-trips exactly.
//!
//! **Encoder** — none. rtl_433 has none, and FoxFW's SD encoder plugin
//! (`applications/main/subghz/protocols_ext/enc_audiovox.c`) is a stub ("fixed-code remote; TX not
//! implemented — load/save only"): its `deserialize` returns `SubGhzProtocolStatusErrorEncoderGetUpload`
//! for every key file, so a Flipper running FoxFW sends nothing. `supports_encoding()` is false,
//! `encode` returns None and nothing is encoder-capable; Replay of recorded pulses remains.

use super::rtl433::{button_table, frame_signal, join_names, BitBuffer, DecodeFn, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "Audiovox-PROOE3B";

/// FoxFW's protocol name (`AUDIOVOX_PROTOCOL_NAME`), written in key files.
const FOXFW_NAME: &str = "Audiovox PRO";

/// The frame length (`AUDIOVOX_FRAME_BITS`, FoxFW's key `Bit`).
const BITS: usize = 25;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Audiovox PRO-OE3B Car Remote", FOXFW_NAME],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Audiovox",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The `r_device`.
const DEVICE: Device = Device {
    modulation: Modulation::OokPwm,
    short_width: 445,
    long_width: 895,
    reset_limit: 1790,
    gap_limit: 1790,
    sync_width: 1368,
    tolerance: 0,
    gaps: Gaps::AtOnce,
    ends: Ends::Ook,
    resync: false,
};

/// rtl_433's `button_strings`, in mask order (0x01, 0x04, 0x10, 0x40 of the inverted byte 2).
const BUTTONS: [&str; 4] = ["Lock", "Unlock", "Option", "Trunk"];

/// rtl_433's `button_str` for KAT's packed flags (bit i = `BUTTONS[i]`).
fn button_str(flags: u8) -> String {
    join_names(BUTTONS.iter().enumerate().filter(|(i, _)| flags >> i & 1 == 1).map(|(_, n)| *n), "")
}

fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    static NAMES: std::sync::OnceLock<Vec<&'static str>> = std::sync::OnceLock::new();
    Some(button_table(&NAMES, button_str)[code as usize])
}

/// `audiovox_pro_oe3b_decode`.
#[derive(Debug, Default)]
struct Audiovox;

impl DecodeFn for Audiovox {
    fn message(&mut self, bits: &mut BitBuffer) -> Option<DecodedSignal> {
        if bits.bits_per_row(0) != 25 {
            return None; // DECODE_ABORT_LENGTH
        }
        if bits.num_rows() != 1 {
            return None; // DECODE_ABORT_EARLY
        }
        check(&bits.bytes(0))
    }
}

/// The C's sanity checks on the 25 wire bits (`wire`, their 4 bytes) and the decode they give.
fn check(wire: &[u8; 4]) -> Option<DecodedSignal> {
    if wire[2] & 0xaa != 0 || wire[2] == 0x55 {
        return None; // DECODE_FAIL_SANITY
    }
    // bitbuffer_invert
    let b = [!wire[0], !wire[1], !wire[2]];
    let id = u16::from_be_bytes([b[0], b[1]]);
    if id == 0 || id == 0xffff {
        return None; // DECODE_FAIL_SANITY
    }
    // mask 0x01, 0x04, 0x10, 0x40 → flags bit 0..3
    let flags = (0..4).filter(|i| b[2] >> (2 * i) & 1 == 1).fold(0u8, |f, i| f | 1 << i);
    if flags == 0 {
        return None; // DECODE_FAIL_SANITY
    }
    let mut d = frame_signal(wire, BITS);
    d.serial = Some(u32::from(id));
    d.button = Some(flags);
    Some(d)
}

/// Audiovox PRO-OE3B decoder (receive only).
pub struct AudiovoxProOe3bDecoder {
    rx: Receiver,
    decode: Audiovox,
}

impl AudiovoxProOe3bDecoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE), decode: Audiovox }
    }
}

impl Default for AudiovoxProOe3bDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for AudiovoxProOe3bDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 445, te_long: 895, te_delta: 0, min_count_bit: 25 }
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

    /// FoxFW's `subghz_protocol_decoder_audiovox_deserialize` (`Bit: 25`), then the decoder's
    /// checks (see "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let k = key.key_u64()?;
        if key.bits != BITS || k >> BITS != 0 {
            return None;
        }
        // The 25 key bits as the first bits of the frame's 4 bytes.
        check(&((k as u32) << 7).to_be_bytes())
    }

    /// FoxFW's `subghz_protocol_decoder_common_serialize` of the frame: `Bit: 25`, `Key` = `data`.
    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        (d.data_count_bit == BITS && d.data >> BITS == 0).then(|| FlipperKey::for_protocol(FOXFW_NAME, BITS, d.data))
    }
}

/// rtl_433's JSON for a decode (`data_print_jsons`, no `time`), for the oracle comparisons.
#[cfg(test)]
pub(super) fn rtl433_json(d: &DecodedSignal) -> String {
    format!(
        "{{\"model\":\"{NAME}\",\"id\":\"{:04X}\",\"button_str\":\"{}\"}}",
        d.serial.unwrap_or(0),
        button_str(d.button.unwrap_or(0))
    )
}

#[cfg(test)]
mod tests {
    use super::super::rtl433::test_util::*;
    use super::*;

    /// The 25 wire bits for an ID and packed button flags (`last`: the free wire bit 24).
    fn frame(id: u16, flags: u8, last: bool) -> Vec<bool> {
        let inv2 = (0..4).filter(|i| flags >> i & 1 == 1).fold(0xAAu8, |b, i| b | 1 << (2 * i));
        let wire = [!(id >> 8) as u8, !id as u8, !inv2, if last { 0x80 } else { 0 }];
        bits_of_bytes(&wire, 25)
    }

    /// PWM as the remote sends it: 1 = 460 µs pulse + 870 µs gap, 0 = 920 + 425, then 10 ms.
    fn wire_pairs(bits: &[bool]) -> Vec<(u32, u32)> {
        let mut pairs: Vec<(u32, u32)> = bits.iter().map(|&b| if b { (460, 870) } else { (920, 425) }).collect();
        pairs.last_mut().unwrap().1 = 10_000;
        pairs
    }

    fn decode(pairs: &[(u32, u32)]) -> Vec<DecodedSignal> {
        feed_pairs(&mut AudiovoxProOe3bDecoder::new(), pairs)
    }

    #[test]
    fn decodes_the_recorded_lock_frame() {
        let got = decode(&wire_pairs(&frame(0x2EA9, 0b0001, true)));
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.crc_valid, d.encoder_capable), (Some(0x2EA9), Some(1), false, false));
        assert_eq!((d.data, d.data_count_bit), (0xD1_5654 << 1 | 1, 25));
        assert_eq!(rtl433_json(d), r#"{"model":"Audiovox-PROOE3B","id":"2EA9","button_str":"Lock"}"#);
    }

    #[test]
    fn decodes_every_button_combination() {
        for flags in 1..16u8 {
            let got = decode(&wire_pairs(&frame(0x1234, flags, flags & 1 == 0)));
            assert_eq!(got.iter().map(|d| d.button).collect::<Vec<_>>(), [Some(flags)], "{flags:#b}");
        }
        assert_eq!(button_str(0b1010), "Unlock; Trunk");
        assert_eq!(native_buttons(NAME, 0b0100), Some("Option"));
    }

    #[test]
    fn rejects_failed_checks() {
        // no button
        assert!(decode(&wire_pairs(&frame(0x2EA9, 0, false))).is_empty());
        // a 0xAA bit of byte 2 set on the wire
        let mut bits = frame(0x2EA9, 1, false);
        bits[16] = true;
        assert!(decode(&wire_pairs(&bits)).is_empty());
        // ID 0000 / FFFF
        assert!(decode(&wire_pairs(&frame(0, 1, false))).is_empty());
        assert!(decode(&wire_pairs(&frame(0xFFFF, 1, false))).is_empty());
        // 24 / 26 bits
        let bits = frame(0x2EA9, 1, false);
        assert!(decode(&wire_pairs(&bits[..24])).is_empty());
        let mut long = bits.clone();
        long.push(true);
        assert!(decode(&wire_pairs(&long)).is_empty());
        // a sync pulse inside makes two rows
        let mut pairs = wire_pairs(&bits);
        pairs[10].0 = 1400;
        assert!(decode(&pairs).is_empty());
        // gap boundaries: 1790 µs keeps the message, 1791 ends it
        let mut pairs = wire_pairs(&bits);
        pairs[12].1 = 1790;
        assert_eq!(decode(&pairs).len(), 1);
        pairs[12].1 = 1791;
        assert!(decode(&pairs).is_empty());
    }

    /// rtl_433's own pulses for the first frame of `audiovox/PRO-OE3B/01/lock_303.3M_250k.cu8`
    /// (`rtl_433 -w ook:…`, µs; the last gap is the pulse detector's end of package).
    const RTL_433_LOCK_PULSES: &str = "472 872 456 424 920 860 468 416 920 432 912 428 920 864 464 424 920 864 464 424 \
        920 860 468 424 916 864 464 872 468 428 916 424 920 864 464 428 916 868 464 428 916 864 464 428 920 428 916 428 920 10004";

    #[test]
    fn decodes_rtl_433_pulses_of_the_lock_recording() {
        let v: Vec<u32> = RTL_433_LOCK_PULSES.split_whitespace().map(|x| x.parse().unwrap()).collect();
        let pairs: Vec<(u32, u32)> = v.chunks(2).map(|c| (c[0], c[1])).collect();
        let got = decode(&pairs);
        assert_eq!(got.iter().map(rtl433_json).collect::<Vec<_>>(), [r#"{"model":"Audiovox-PROOE3B","id":"2EA9","button_str":"Lock"}"#]);
    }

    /// rtl_433_tests `audiovox/PRO-OE3B/01` through KAT's receive pipeline and the registry: every
    /// recording's rtl_433 events (lock 3, option 10, unlock 13; ID 2EA9). Until v1.8.0's receiver
    /// fixes KAT's AM chain distorted these frames (a missed first pulse, shortened long pulses as
    /// the remote's carrier drifted) and this test pinned a missed Unlock and a corrupt ID 0029.
    #[test]
    #[ignore]
    fn decodes_the_rtl_433_recordings() {
        let Some(root) = rtl433_tests() else {
            eprintln!("Skip: rtl_433_tests not found");
            return;
        };
        let dir = root.join("audiovox/PRO-OE3B/01");
        for f in ["lock", "option", "unlock"] {
            let path = dir.join(format!("{f}_303.3M_250k.cu8"));
            let got: Vec<serde_json::Value> = registry_decodes(&path)
                .iter()
                .map(|(n, d)| {
                    assert_eq!(n, NAME, "{f}");
                    serde_json::from_str(&rtl433_json(d)).unwrap()
                })
                .collect();
            assert_eq!(distinct(&got), distinct(&expected_json(&path)), "{f}");
        }
    }

    /// A made-up frame whose RAW file (written to `$KAT_SYNTH_DIR` when set) FoxFW's decoder read with
    /// `subghz_oracle_foxfwenc decode --protocol "Audiovox PRO"`: KAT writes FoxFW's lines and reads
    /// them back to the same decode; FoxFW's name resolves to this decoder.
    #[test]
    fn key_files_are_foxfws() {
        const FOXFW: &str = "Frequency: 303000000; Preset: FuriHalSubGhzPresetOok650Async; Protocol: Audiovox PRO; Bit: 25; Key: 00 00 00 00 01 D9 50 23";
        let air = vector_wire(&frame(0x1357, 0b1010, true));
        crate::protocols::foxfw_common::test_support::dump("audiovox_unit.sub", 303_000_000, "FuriHalSubGhzPresetOok650Async", &air);
        let got = decode_all(&mut AudiovoxProOe3bDecoder::new(), &air);
        assert_eq!(got.len(), 2, "sent twice");
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.data_count_bit), (Some(0x1357), Some(0b1010), 25));
        let dec = AudiovoxProOe3bDecoder::new();
        let mut key = dec.to_flipper_key(d).unwrap();
        key.frequency = 303_000_000;
        key.preset = Some("FuriHalSubGhzPresetOok650Async".into());
        assert_eq!(key.to_text().lines().skip(2).collect::<Vec<_>>().join("; "), FOXFW);
        crate::protocols::flipper_key::testing::assert_round_trip(&dec, d);
        let text = format!("Filetype: Flipper SubGhz Key File\nVersion: 1\n{}\n", FOXFW.replace("; ", "\n"));
        assert_eq!(dec.from_flipper_key(&crate::protocols::flipper_key::testing::key(&text)).as_ref(), Some(d));
        let reg = super::super::ProtocolRegistry::new();
        assert_eq!(reg.get(FOXFW_NAME).map(|p| p.name()), Some(NAME));
    }

    /// Keys FoxFW's `deserialize` refuses (another `Bit`) and keys whose frame fails the sanity
    /// checks or has bits above its 25 (FoxFW writes neither): not loaded.
    #[test]
    fn refuses_other_keys() {
        let dec = AudiovoxProOe3bDecoder::new();
        // ~0x1357, then ~(0xAA | Unlock 0x04 | Trunk 0x40) = 0x11, then the free bit
        let good = 0xECA8_u64 << 9 | 0x11 << 1 | 1;
        let load = |bits: usize, k: u64| dec.from_flipper_key(&FlipperKey::new(FOXFW_NAME, bits, k, 303_000_000, "FuriHalSubGhzPresetOok650Async"));
        assert_eq!(load(25, good).and_then(|d| d.button), Some(0b1010));
        assert!(load(24, good >> 1).is_none(), "Bit 24");
        assert!(load(25, good | 0x02 << 1).is_none(), "a 0xAA bit of byte 2 set on the wire");
        assert!(load(25, 0xECA8 << 9 | 0x55 << 1).is_none(), "no button (byte 2 = 0x55)");
        assert!(load(25, 0xFFFF << 9 | 0x11 << 1).is_none(), "ID 0000");
        assert!(load(25, 0x11 << 1).is_none(), "ID FFFF");
        assert!(load(25, good | 1 << 25).is_none(), "bit above the 25");
        assert!(dec.to_flipper_key(&DecodedSignal::new(0, 64)).is_none());
    }

    /// FoxFW's encoder plugin refuses every Audiovox PRO key (`subghz_oracle_foxfwenc encode`:
    /// "deserialize failed (status -12)" for each of 120 made-up files) and rtl_433 has no encoder:
    /// nothing transmits, for no button.
    #[test]
    fn transmits_nothing_as_foxfw() {
        let dec = AudiovoxProOe3bDecoder::new();
        assert!(!dec.supports_encoding());
        let d = decode_all(&mut AudiovoxProOe3bDecoder::new(), &vector_wire(&frame(0x1357, 0b0001, false))).remove(0);
        let loaded = dec.from_flipper_key(&dec.to_flipper_key(&d).unwrap()).unwrap();
        for x in [&d, &loaded] {
            assert!(!x.encoder_capable);
            for button in [0x01, 0x02, 0x04, 0x08, x.button.unwrap()] {
                assert!(dec.encode(x, button).is_none());
            }
        }
    }

    /// A RAW file's pulses for a frame: a 1368 µs sync pulse (FoxFW starts a frame only on one;
    /// rtl_433's slicer counts it as a sync before the row), [wire_pairs], sent twice, 20 ms of LOW
    /// around and between.
    fn vector_wire(bits: &[bool]) -> Vec<LevelDuration> {
        let mut pairs = vec![(1368, 425)];
        pairs.extend(wire_pairs(bits));
        pairs.last_mut().unwrap().1 = 20_000;
        let again = pairs.clone();
        pairs.extend(again);
        with_idle(&wire(&pairs), 20_000)
    }

    /// Made-up frames for the FoxFW comparison, as RAW files in `$KAT_SYNTH_DIR`: every button
    /// combination with either free bit, random IDs.
    #[test]
    #[ignore = "oracle harness"]
    fn dump_key_vectors() {
        let mut rng = Rng(0xA0D1_0E3B);
        for i in 0..150u32 {
            let id = loop {
                let id = rng.next() as u16;
                if id != 0 && id != 0xFFFF {
                    break id;
                }
            };
            let flags = (i % 15 + 1) as u8;
            let wire = vector_wire(&frame(id, flags, (i / 15) % 2 == 1));
            crate::protocols::foxfw_common::test_support::dump(&format!("audiovox_{i:03}.sub"), 303_000_000, "FuriHalSubGhzPresetOok650Async", &wire);
        }
    }

    /// `KAT_AUDIOVOX_RAW=<file listing RAW .sub files>`: KAT's key-file lines for each frame this
    /// decoder reads in them, to diff with `subghz_oracle_foxfwenc decode --protocol "Audiovox PRO"`.
    #[test]
    #[ignore = "oracle harness"]
    fn key_lines_of_raw_files() {
        print_raw_key_lines("KAT_AUDIOVOX_RAW", &mut AudiovoxProOe3bDecoder::new());
    }

    /// `KAT_AUDIOVOX_ENCODE=<file listing key files>`: KAT's transmission of each (`(none)`: it
    /// never transmits), to diff with `subghz_oracle_foxfwenc encode <file>`, which refuses each.
    #[test]
    #[ignore = "oracle harness"]
    fn encode_key_files() {
        print_key_file_encodes("KAT_AUDIOVOX_ENCODE", &AudiovoxProOe3bDecoder::new());
    }

    /// Fuzz dump for the C harness (see `rtl433::test_util::dump_fuzz_trains`).
    #[test]
    #[ignore]
    fn fuzz_dump() {
        let interesting = [1, 669, 670, 671, 672, 1130, 1131, 1132, 1133, 1789, 1790, 1791];
        dump_fuzz_trains(
            "audiovox",
            20_000,
            |rng, i| {
                let mut bits = frame(rng.next() as u16, rng.below(16) as u8, rng.chance(50));
                if rng.chance(20) {
                    let k = rng.below(25) as usize;
                    bits[k] = !bits[k];
                }
                if rng.chance(5) {
                    bits.truncate(22 + rng.below(3) as usize);
                }
                let (one, zero, spread) = if i % 3 == 0 { (445, 895, 200) } else { (460, 920, 40) };
                let mut pairs: Vec<(u32, u32)> = bits
                    .iter()
                    .map(|&b| if b { (rng.jitter(one, spread), rng.jitter(870, 300)) } else { (rng.jitter(zero, spread), rng.jitter(425, 200)) })
                    .collect();
                pairs.last_mut().unwrap().1 = 1791 + rng.below(20_000) as u32;
                if rng.chance(40) {
                    let again = pairs.clone();
                    pairs.extend(again);
                }
                if rng.chance(60) {
                    mutate(rng, &mut pairs, &interesting);
                }
                pairs
            },
            || Box::new(AudiovoxProOe3bDecoder::new()),
            rtl433_json,
        );
    }
}
