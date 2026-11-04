//! Security+ 1.0 (Chamberlain / LiftMaster / Craftsman garage door openers with a purple, red or
//! orange learn button, their remotes and keypads): ternary rolling code.
//!
//! Port of `secplus_v1.c` from Flipper-ARF's garage app
//! (`applications/system/subghz_garage/protocols/`, the primary reference), which is
//! doctor-firmware's `lib/subghz/protocols/secplus_v1.c` with the same decoder and encoder.
//! Flipper-ARF's `lib/subghz/protocols/secplus_v1.c` adds a D-pad button switch to the encoder;
//! KAT takes that union (see Encoder). The references cite argilo/secplus and rtl_433's
//! `secplus_v1.c`.
//!
//! **Air format** (AM/OOK): ternary symbols of 2 ms, LOW then HIGH: `0` = LOW 1500 + HIGH 500,
//! `1` = LOW 1000 + HIGH 1000, `2` = LOW 500 + HIGH 1500 (te_short 500, te_long 1500, te_delta 100,
//! strict `<`). A transmission is two packets of 21 symbols, each led by a ~60 ms LOW gap; packet 1
//! starts with symbol `0`, packet 2 with symbol `2`.
//!
//! **Decoder** (`subghz_protocol_decoder_secplus_v1_feed`): a LOW of 60000 ± 12000 µs arms it;
//! the first HIGH (500 → packet 1, 1500 → packet 2) is symbol 0 of the packet, then (LOW, HIGH)
//! symbol pairs; a packet counts once 21 symbols are followed by the next 60 ms gap. When both
//! packets have been seen the 40 data symbols are decoded and the C calls back — and since its
//! `reset` is empty ("you need to get 2 parts of the package") and the decode path then continues in
//! `SearchStartBit` with both packets still marked, every following packet completes another decode
//! (mixing the new packet with the previous one), until a pulse out of tolerance sends it back to
//! the gap search. KAT's `reset()` is equally empty.
//!
//! **Frame** (`subghz_protocol_secplus_v1_decode`): per packet, the 20 symbols after the header
//! alternate rolling digit / `(60 + symbol - acc) % 3` fixed digit (`acc` = running sum of both
//! digits in that packet). `rolling` = the 20 rolling digits base 3, bit-reversed over 32 bits;
//! `fixed` = the 20 fixed digits base 3. `data = fixed << 32 | rolling`, `data_count_bit` = 42
//! (the references' `Bit`). Fields as `get_string` shows them: `button = fixed % 3` (0 middle,
//! 1 left, 2 right), `id1 = fixed / 9 % 3`; serial = `fixed / 27 % 2187` when `id1 == 0`
//! (keypad: PIN in `fixed / 3^10 % 3^9`, display only), else `fixed / 27`; the counter is the
//! 32-bit `rolling` — KAT's `counter` holds its low 16 bits (`data` has all 32). There is no
//! checksum: `crc_valid` is false and, like the C, every structurally complete pair is reported.
//!
//! **Encoder** (`subghz_protocol_encoder_secplus_v1_deserialize` + `_encode` + `_get_upload`):
//! rolling + 1 (the C's `0xFFFFFFFF` overflow test can never fire on a `uint32_t`, so the value
//! wraps), then raised to at least 0xE6000000 (the references' "we don't know the counter size"
//! jump); `fixed` must be ≤ 0xCFD41B90 (3^20 − 1) or nothing is sent. `button` (native code 0, 1
//! or 2) replaces the lowest base-3 digit of `fixed` — Flipper-ARF lib's D-pad (Up 0, Down 1,
//! Left 2); any other value keeps the captured button. The rolling value is bit-reversed and split
//! into 20 base-3 digits (a reversed value ≥ 3^20 loses its top part, as in the C), the symbols
//! rebuilt, and the upload is LOW 59500 + HIGH 500 (gap + packet-1 header), 20 symbols, LOW 58000 +
//! HIGH 1500 (gap + packet-2 header), 20 symbols; `encoder.repeat` = 3 copies. (The garage app's
//! TX plugins are built without `SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS`, which turns the counter step
//! into the "OFEX" overflow mode; KAT follows the source's normal mode, the one doctor-firmware and
//! Flipper-ARF's firmware run with the default multiplier 1. See docs/secplus_v1.md.)
//!
//! **Key file** (`subghz_block_generic_serialize`): `Protocol: Security+ 1.0`, `Bit: 42`,
//! `Key: <data>`; loading requires `Bit: 42` (`deserialize_check_count_bit`).

use super::flipper_key::FlipperKey;
use super::g4_common::{push, reverse_key};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`). Flags: 315 MHz, AM (doctor-firmware also flags 433).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Security+ 1.0",
    decoder: "Security+ 1.0",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Chamberlain/LiftMaster",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// `get_string`: `Btn:middle` (0), `left` (1), `right` (2).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0 => "Middle",
        1 => "Left",
        2 => "Right",
        _ => "Other",
    })
}

const NAME: &str = "Security+ 1.0";
const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1500;
const TE_DELTA: u32 = 100;
/// `min_count_bit_for_found`: symbols per packet.
const PACKET_SYMBOLS: u8 = 21;
/// The references' `Bit` for a decoded pair (`2 * min_count_bit_for_found`).
const BIT_COUNT: usize = 42;
/// Gap before each packet: te_short * 120 ± te_delta * 120.
const GAP: u32 = TE_SHORT * 120;
const GAP_DELTA: u32 = TE_DELTA * 120;

const BIT_0: u8 = 0;
const BIT_1: u8 = 1;
const BIT_2: u8 = 2;
const PACKET_1_INDEX_BASE: u8 = 0;
const PACKET_2_INDEX_BASE: u8 = 21;
const PACKET_1_ACCEPTED: u8 = 1 << 0;
const PACKET_2_ACCEPTED: u8 = 1 << 1;
/// Largest encodable `fixed` (3^20 − 1).
const FIXED_MAX: u32 = 0xCFD4_1B90;
/// The references' counter floor after the step.
const ROLLING_FLOOR: u32 = 0xE600_0000;
/// `encoder.repeat`.
const TX_REPEAT: usize = 3;

/// `SecPlus_v1DecoderStep`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    SearchStartBit,
    SaveDuration,
    DecoderData,
}

/// Security+ 1.0 decoder (`SubGhzProtocolDecoderSecPlus_v1`).
pub struct SecPlusV1Decoder {
    step: Step,
    te_last: u32,
    decode_count_bit: u8,
    base_packet_index: u8,
    packet_accepted: u8,
    data_array: [u8; 44],
}

impl SecPlusV1Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_count_bit: 0,
            base_packet_index: 0,
            packet_accepted: 0,
            data_array: [0; 44],
        }
    }

    /// Store one symbol of the current packet and go wait for the next LOW.
    fn store(&mut self, symbol: u8) {
        let index = self.decode_count_bit as usize + self.base_packet_index as usize;
        self.data_array[index] = symbol;
        self.decode_count_bit += 1;
        self.step = Step::SaveDuration;
    }
}

impl Default for SecPlusV1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `subghz_protocol_secplus_v1_decode`: (fixed, rolling) from the two packets' symbols.
fn decode_symbols(data_array: &[u8; 44]) -> (u32, u32) {
    let mut rolling: u32 = 0;
    let mut fixed: u32 = 0;
    for start in [1usize, 22] {
        let mut acc: u32 = 0;
        for i in (start..start + 20).step_by(2) {
            let digit = data_array[i] as u32;
            rolling = rolling * 3 + digit;
            acc += digit;
            let digit = (60 + data_array[i + 1] as u32 - acc) % 3;
            fixed = fixed * 3 + digit;
            acc += digit;
        }
    }
    (fixed, reverse_key(rolling as u64, 32) as u32)
}

/// The decode the references show for a key: fields as `get_string` derives them.
fn decoded_from_data(data: u64) -> DecodedSignal {
    let fixed = (data >> 32) as u32;
    let rolling = data as u32;
    let id1 = (fixed / 9) % 3;
    let serial = if id1 == 0 {
        (fixed / 27) % 2187
    } else {
        fixed / 27
    };
    DecodedSignal {
        serial: Some(serial),
        button: Some((fixed % 3) as u8),
        counter: Some(rolling as u16),
        crc_valid: false,
        data,
        data_count_bit: BIT_COUNT,
        encoder_capable: fixed <= FIXED_MAX,
        extra: None,
        protocol_display_name: None,
    }
}

/// `subghz_protocol_secplus_v1_check_fixed`: a fixed code the SubGHz app's "Add Manually" accepts
/// as a new remote's (`id1 != 0`; the button digit is always 0..2).
pub(super) fn check_fixed(fixed: u32) -> bool {
    !(fixed / 9).is_multiple_of(3)
}

/// The encoder's counter step (normal mode, multiplier 1).
fn next_rolling(rolling: u32) -> u32 {
    rolling.wrapping_add(1).max(ROLLING_FLOOR)
}

/// `subghz_protocol_secplus_v1_encode` after the counter step: the 42 symbols.
fn encode_symbols(fixed: u32, rolling: u32) -> [u8; 44] {
    let mut rolling = reverse_key(rolling as u64, 32) as u32;
    let mut fixed = fixed;
    let mut rolling_array = [0u8; 20];
    let mut fixed_array = [0u8; 20];
    for i in (0..20).rev() {
        rolling_array[i] = (rolling % 3) as u8;
        rolling /= 3;
        fixed_array[i] = (fixed % 3) as u8;
        fixed /= 3;
    }
    let mut data_array = [0u8; 44];
    data_array[PACKET_1_INDEX_BASE as usize] = BIT_0;
    data_array[PACKET_2_INDEX_BASE as usize] = BIT_2;
    let mut acc: u32 = 0;
    for i in 1..11 {
        acc += rolling_array[i - 1] as u32;
        data_array[i * 2 - 1] = rolling_array[i - 1];
        acc += fixed_array[i - 1] as u32;
        data_array[i * 2] = (acc % 3) as u8;
    }
    acc = 0;
    for i in 11..21 {
        acc += rolling_array[i - 1] as u32;
        data_array[i * 2] = rolling_array[i - 1];
        acc += fixed_array[i - 1] as u32;
        data_array[i * 2 + 1] = (acc % 3) as u8;
    }
    data_array
}

/// One symbol: LOW then HIGH.
fn push_symbol(out: &mut Vec<LevelDuration>, symbol: u8) {
    let (low, high) = match symbol {
        BIT_0 => (TE_SHORT * 3, TE_SHORT),
        BIT_1 => (TE_SHORT * 2, TE_SHORT * 2),
        _ => (TE_SHORT, TE_SHORT * 3),
    };
    push(out, false, low);
    push(out, true, high);
}

/// `subghz_protocol_encoder_secplus_v1_get_upload`: both packets, once.
fn push_upload(out: &mut Vec<LevelDuration>, data_array: &[u8; 44]) {
    push(out, false, TE_SHORT * (116 + 3));
    push(out, true, TE_SHORT);
    for &symbol in &data_array[1..21] {
        push_symbol(out, symbol);
    }
    push(out, false, TE_SHORT * 116);
    push(out, true, TE_SHORT * 3);
    for &symbol in &data_array[22..42] {
        push_symbol(out, symbol);
    }
}

impl ProtocolDecoder for SecPlusV1Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: PACKET_SYMBOLS as usize,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    /// `subghz_protocol_decoder_secplus_v1_reset` does nothing: the two packets of a pair arrive
    /// separately, and the state (including accepted packets) survives the receiver reset.
    fn reset(&mut self) {}

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, GAP) < GAP_DELTA {
                    self.step = Step::SearchStartBit;
                    self.decode_count_bit = 0;
                    self.packet_accepted = 0;
                    self.data_array = [0; 44];
                }
            }
            Step::SearchStartBit => {
                if level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.base_packet_index = PACKET_1_INDEX_BASE;
                    self.store(BIT_0);
                } else if level && duration_diff!(duration, TE_LONG) < TE_DELTA {
                    self.base_packet_index = PACKET_2_INDEX_BASE;
                    self.store(BIT_2);
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if level {
                    self.step = Step::Reset;
                } else if duration_diff!(duration, GAP) < GAP_DELTA {
                    let mut result = None;
                    if self.decode_count_bit == PACKET_SYMBOLS {
                        if self.base_packet_index == PACKET_1_INDEX_BASE {
                            self.packet_accepted |= PACKET_1_ACCEPTED;
                        }
                        if self.base_packet_index == PACKET_2_INDEX_BASE {
                            self.packet_accepted |= PACKET_2_ACCEPTED;
                        }
                        if self.packet_accepted == PACKET_1_ACCEPTED | PACKET_2_ACCEPTED {
                            let (fixed, rolling) = decode_symbols(&self.data_array);
                            result = Some(decoded_from_data((fixed as u64) << 32 | rolling as u64));
                        }
                    }
                    // The C sets Reset after its callback, then SearchStartBit unconditionally.
                    self.step = Step::SearchStartBit;
                    self.decode_count_bit = 0;
                    return result;
                } else {
                    self.te_last = duration;
                    self.step = Step::DecoderData;
                }
            }
            Step::DecoderData => {
                if level && self.decode_count_bit <= PACKET_SYMBOLS {
                    let last = self.te_last;
                    if duration_diff!(last, TE_SHORT * 3) < TE_DELTA * 3
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.store(BIT_0);
                    } else if duration_diff!(last, TE_SHORT * 2) < TE_DELTA * 2
                        && duration_diff!(duration, TE_SHORT * 2) < TE_DELTA * 2
                    {
                        self.store(BIT_1);
                    } else if duration_diff!(last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT * 3) < TE_DELTA * 3
                    {
                        self.store(BIT_2);
                    } else {
                        self.step = Step::Reset;
                    }
                } else {
                    self.step = Step::Reset;
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        let mut fixed = (decoded.data >> 32) as u32;
        // Flipper-ARF lib's D-pad: replace the lowest base-3 digit (the button).
        if button <= 2 {
            fixed = fixed - fixed % 3 + button as u32;
        }
        let rolling = next_rolling(decoded.data as u32);
        if fixed > FIXED_MAX {
            return None;
        }
        let data_array = encode_symbols(fixed, rolling);
        let mut out = Vec::with_capacity(TX_REPEAT * 84);
        for _ in 0..TX_REPEAT {
            push_upload(&mut out, &data_array);
        }
        Some(out)
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == BIT_COUNT).then(|| decoded_from_data(key.key))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == BIT_COUNT).then(|| {
            FlipperKey::new(
                NAME,
                BIT_COUNT,
                decoded.data,
                0,
                "FuriHalSubGhzPresetOok650Async",
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    fn feed_all(dec: &mut SecPlusV1Decoder, wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        wire.iter()
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect()
    }

    /// The upload as the C builds it for `(fixed, rolling)` with the step already applied.
    fn upload_for(fixed: u32, rolling: u32, copies: usize) -> Vec<LevelDuration> {
        let symbols = encode_symbols(fixed, rolling);
        let mut out = Vec::new();
        for _ in 0..copies {
            push_upload(&mut out, &symbols);
        }
        out
    }

    /// The unit-test key file (`security_pls_1_0.sub`).
    const UT_KEY: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Security+ 1.0\nBit: 42\n\
        Key: 1C 41 D2 39 E6 A3 8B CC\n";

    #[test]
    fn symbols_round_trip() {
        for &(fixed, rolling) in &[
            (0u32, 0u32),
            (1, 0x8000_0000),
            (0x6E15_6A21, 0x8A51_F098),
            (FIXED_MAX, 0xFFFF_FFFE),
            (0x1C41_D239, 0xE6A3_8BCD),
        ] {
            let symbols = encode_symbols(fixed, rolling);
            assert!(symbols.iter().all(|&s| s <= 2));
            let (f, r) = decode_symbols(&symbols);
            assert_eq!(f, fixed, "fixed {fixed:#X}");
            // The 20 base-3 rolling digits hold the bit-reversed value modulo 3^20.
            let reversed = reverse_key(rolling as u64, 32) % 3u64.pow(20);
            assert_eq!(r as u64, reverse_key(reversed, 32), "rolling {rolling:#X}");
        }
    }

    /// Synthetic upload → decoder: the C completes a decode at every packet after the first pair
    /// (packets 1, 2, 1, 2, 1 + gaps → 4 decodes; the last packet has no trailing gap).
    #[test]
    fn decodes_synthetic_frames_like_the_c() {
        let (fixed, rolling) = (0x6E15_6A21u32, 0xE8A5_1F09u32);
        let mut dec = SecPlusV1Decoder::new();
        let mut wire = upload_for(fixed, rolling, 3);
        let got = feed_all(&mut dec, &wire);
        assert_eq!(got.len(), 4);
        for d in &got {
            assert_eq!(d.data, (fixed as u64) << 32 | rolling as u64);
            assert_eq!(d.data_count_bit, 42);
            assert_eq!(d.button, Some((fixed % 3) as u8));
            assert_eq!(d.counter, Some(rolling as u16));
            assert!(!d.crc_valid && d.encoder_capable);
        }
        // fixed / 9 % 3 != 0 → serial = fixed / 27.
        assert_eq!(got[0].serial, Some(fixed / 27));
        // A trailing gap completes the last packet too.
        wire.push(LevelDuration::new(false, 60_000));
        assert_eq!(feed_all(&mut SecPlusV1Decoder::new(), &wire).len(), 5);
    }

    #[test]
    fn rejects_bad_timing_and_single_packets() {
        let wire = upload_for(0x6E15_6A21, 0xE8A5_1F09, 1);
        // One pair only: packet 1 completes at packet 2's gap, packet 2 has no trailing gap.
        assert!(feed_all(&mut SecPlusV1Decoder::new(), &wire).is_empty());
        // Symbol out of tolerance in packet 2 of the second copy: that packet completes nothing
        // (copy 1's packet 2 and copy 2's packet 1 each complete a pair).
        let mut wire = upload_for(0x6E15_6A21, 0xE8A5_1F09, 2);
        let n = wire.len();
        wire[n - 10].duration_us += 400;
        wire.push(LevelDuration::new(false, 60_000));
        let got = feed_all(&mut SecPlusV1Decoder::new(), &wire);
        assert_eq!(got.len(), 2);
        let mut intact = upload_for(0x6E15_6A21, 0xE8A5_1F09, 2);
        intact.push(LevelDuration::new(false, 60_000));
        assert_eq!(feed_all(&mut SecPlusV1Decoder::new(), &intact).len(), 3);
        // Gaps outside 60000 ± 12000 never arm it.
        let stretched: Vec<LevelDuration> = upload_for(0x6E15_6A21, 0xE8A5_1F09, 3)
            .into_iter()
            .map(|p| {
                if p.duration_us > 50_000 {
                    LevelDuration::new(p.level, 73_000)
                } else {
                    p
                }
            })
            .collect();
        assert!(feed_all(&mut SecPlusV1Decoder::new(), &stretched).is_empty());
    }

    #[test]
    fn encode_steps_the_counter_and_switches_buttons() {
        let key = FlipperKey::parse(UT_KEY).unwrap();
        let d = SecPlusV1Decoder::new().from_flipper_key(&key).unwrap();
        assert_eq!(d.data, 0x1C41_D239_E6A3_8BCC);
        let fixed = 0x1C41_D239u32;
        assert_eq!(d.button, Some((fixed % 3) as u8));
        for (button, want_btn) in [
            (d.button.unwrap(), fixed % 3),
            (0, 0),
            (1, 1),
            (2, 2),
            (7, fixed % 3),
        ] {
            let wire = SecPlusV1Decoder::new().encode(&d, button).unwrap();
            let mut dec = SecPlusV1Decoder::new();
            let mut got = feed_all(&mut dec, &wire);
            assert_eq!(got.len(), 4, "button {button}");
            let g = got.remove(0);
            assert_eq!(g.button, Some(want_btn as u8), "button {button}");
            assert_eq!(
                g.data,
                ((fixed - fixed % 3 + want_btn) as u64) << 32 | 0xE6A3_8BCD
            );
        }
        // Counter step: +1 with wrap, then at least 0xE6000000.
        assert_eq!(next_rolling(0x1234_5678), 0xE600_0000);
        assert_eq!(next_rolling(0xE600_0000), 0xE600_0001);
        assert_eq!(next_rolling(0xFFFF_FFFF), 0xE600_0000);
        // fixed > 3^20 - 1 cannot be sent.
        let big = decoded_from_data(0xD000_0000_0000_0000);
        assert!(!big.encoder_capable);
        assert!(SecPlusV1Decoder::new().encode(&big, 0xFF).is_none());
        // Wrong bit count.
        let mut wrong = d.clone();
        wrong.data_count_bit = 40;
        assert!(SecPlusV1Decoder::new().encode(&wrong, 0).is_none());
    }

    /// Upload layout: gap + header, 20 symbols, gap + header, 20 symbols; 3 copies.
    #[test]
    fn upload_layout_matches_the_c() {
        let d = decoded_from_data(0x1C41_D239_E6A3_8BCC);
        let wire = SecPlusV1Decoder::new().encode(&d, 0xFF).unwrap();
        assert_eq!(wire.len(), 3 * 84);
        assert_eq!((wire[0].level, wire[0].duration_us), (false, 59_500));
        assert_eq!((wire[1].level, wire[1].duration_us), (true, 500));
        assert_eq!((wire[42].level, wire[42].duration_us), (false, 58_000));
        assert_eq!((wire[43].level, wire[43].duration_us), (true, 1500));
        // Every symbol is a LOW + HIGH pair of 2000 µs (headers: gap + symbol).
        for pair in wire.chunks(2) {
            assert!(!pair[0].level && pair[1].level);
            assert!(
                pair[0].duration_us + pair[1].duration_us == 2000 || pair[0].duration_us >= 58_000
            );
        }
    }

    #[test]
    fn key_file_round_trip() {
        let dec = SecPlusV1Decoder::new();
        let key = FlipperKey::parse(UT_KEY).unwrap();
        let d = dec.from_flipper_key(&key).unwrap();
        assert_eq!(
            (d.serial, d.counter),
            (Some(0x1C41_D239 / 27), Some(0x8BCC))
        );
        let text = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(
            text.contains("Protocol: Security+ 1.0\nBit: 42\nKey: 1C 41 D2 39 E6 A3 8B CC\n"),
            "{text}"
        );
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(
            (
                back.data,
                back.data_count_bit,
                back.serial,
                back.button,
                back.counter,
                back.crc_valid,
                back.encoder_capable,
                back.extra
            ),
            (
                d.data,
                d.data_count_bit,
                d.serial,
                d.button,
                d.counter,
                d.crc_valid,
                d.encoder_capable,
                d.extra
            )
        );
        let mut wrong = key.clone();
        wrong.bits = 40;
        assert!(dec.from_flipper_key(&wrong).is_none());
        // The registry resolves the key file's protocol name.
        let reg = ProtocolRegistry::new();
        assert_eq!(reg.decode_key(&key).map(|(n, _)| n), Some(NAME.to_string()));
    }

    /// Flipper's unit test: `security_pls_1_0_raw.sub` must decode (skipped without REFERENCES).
    #[test]
    fn decodes_flipper_unit_test_capture() {
        let path = std::path::Path::new(
            "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz/security_pls_1_0_raw.sub",
        );
        if !path.exists() {
            eprintln!("Skip: {path:?} not found");
            return;
        }
        let (freq, raw) = crate::export::flipper::import_sub_raw(path).unwrap();
        let pairs: Vec<LevelDuration> = raw
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let mut dec = SecPlusV1Decoder::new();
        let own: Vec<DecodedSignal> = feed_all(&mut dec, &pairs);
        assert!(!own.is_empty(), "no Security+ 1.0 frame");
        let mut reg = ProtocolRegistry::new();
        let hits = reg.process_signal_stream(&pairs, freq);
        assert!(
            hits.iter().any(|(n, _, _)| n == NAME),
            "{:?}",
            hits.iter().map(|h| &h.0).collect::<Vec<_>>()
        );
    }
}
