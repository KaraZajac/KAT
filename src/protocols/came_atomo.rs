//! CAME Atomo decoder + encoder (rolling code, self-contained cipher).
//!
//! Primary reference: Flipper-ARF's garage app,
//! `applications/system/subghz_garage/protocols/came_atomo.c`. Union with doctor-firmware's and
//! ARF's `lib/subghz/protocols/came_atomo.c`, which go further on the wire: a third header
//! length (~19 ms, TOPD4REN remotes) and buttons 5/6 (codes 0xC/0xE) — KAT accepts both.
//!
//! **Air format** (AM 433 MHz): Manchester, te 600/1200 µs (te_delta 250), decoded with
//! Flipper's `manchester_decoder.c` (common.rs) and stored inverted (`!bit`). A frame is a long
//! LOW header (12 ms ± 5, 19.2 ms ± 2.5 or 72 ms ± 10), a 1200 µs HIGH start (decodes as a 1),
//! 60 data bits and a LOW ≥ 2650 µs, which ends the frame: exactly 62 counted bits (the count
//! starts at 1) are reported, and the decoder stays in the data state, so back-to-back frames
//! (the encoder's 68 ms pause) decode one after the other. `data` = the 61 received bits (bit 60
//! is the start bit), `data_count_bit` = 62, as in the C.
//!
//! **Fields** (`remote_controller`): `pack = BE bytes of (!data << 4)` decrypted with
//! `atomo_decrypt` (an LFSR keystream XORed over bits 8..58, keyed by byte 0): byte 0 = the
//! hold-cycle counter (0..0x7F, rotates while the button is held), bytes 1..2 = the 16-bit
//! counter, 3..6 = the 32-bit serial, the high nibble of byte 7 = the button code 0, 2, 4, 6, 0xC,
//! 0xE → button 1..6 (the C leaves `btn` unchanged for any other code: KAT keeps the previous
//! frame's button too). No checksum (`crc_valid` false); every frame is reported, like the C.
//!
//! **Encoder** (`subghz_protocol_encoder_came_atomo_deserialize` + `get_upload`): decrypt the key,
//! counter + 1 (`CounterMode` 1: 0 → 1 → 0xFFFE → 0xFFFF → 0, 2: 0x807B → 0x807C → 0x007B →
//! 0x007C → 0x807B, ≥ 3: frozen), button = the requested native button 1..6 (else the captured
//! one) mapped back to its code; upload = HIGH 18000, LOW 72000, then 8 frames with hold-cycle
//! bytes 10, 30, 125, 126, 127, 0, 1, 3: `atomo_encrypt`, invert, `>> 4`, HIGH 1200, LOW 600,
//! Manchester of the 60 inverted bits (Flipper's `manchester_encoder.c`; a final LOW half is
//! dropped), LOW 68000. One copy (`encoder.repeat` = 1). `extra` = `CounterMode << 8` from a
//! key file (None otherwise).
//! The garage app's TX plugins are built without the app's custom-button and firmware-extension
//! flags, so their compat shims make this encoder send button 0 and the "OFEX" counter (0 → 1 →
//! 0xFFFE → 0xFFFF → 0) whatever was captured (oracle-confirmed); KAT does not copy that build
//! artefact and sends the captured or requested button with counter + 1, as doctor-firmware does.

use super::common::{ManchesterEvent, ManchesterState};
use super::g6_common::{self, push};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): SubGhzProtocolFlag_AM | 433.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "CAME Atomo",
    decoder: "CAME Atomo",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "CAME",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The C's `generic.btn` (codes 0, 2, 4, 6 = left-up, right-up, left-down, right-down; 0xC/0xE
/// = buttons 5/6 on the Unleashed-lineage copies).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        1 => "Button 1",
        2 => "Button 2",
        3 => "Button 3",
        4 => "Button 4",
        5 => "Button 5",
        6 => "Button 6",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 600;
const TE_LONG: u32 = 1200;
const TE_DELTA: u32 = 250;
const MIN_COUNT_BIT: u8 = 62;
/// Hold-cycle bytes of the 8 frames the encoder sends.
const HOLD_CYCLE: [u8; 8] = [10, 30, 125, 126, 127, 0, 1, 3];

/// `atomo_decrypt`.
fn atomo_decrypt(buff: &mut [u8; 8]) {
    buff[0] = (buff[0] ^ 5) & 0x7F;
    let tmp_b = buff[0].wrapping_neg() & 0x7F;
    atomo_keystream(buff, tmp_b);
}

/// `atomo_encrypt`.
fn atomo_encrypt(buff: &mut [u8; 8]) {
    let tmp_b = (!buff[0]).wrapping_add(1) & 0x7F;
    atomo_keystream(buff, tmp_b);
    buff[0] = (buff[0] ^ 5) & 0x7F;
}

/// The LFSR both `atomo_*` functions XOR over bits 8..58 (MSB first).
fn atomo_keystream(buff: &mut [u8; 8], mut tmp_b: u8) {
    for bit_cnt in 8..59usize {
        if tmp_b & 0x18 != 0 && (tmp_b >> 3) & 3 != 3 {
            tmp_b = (tmp_b << 1) | 1;
        } else {
            tmp_b <<= 1;
        }
        if tmp_b & 0x80 != 0 {
            buff[bit_cnt / 8] ^= 0x80 >> (bit_cnt & 7);
        }
    }
}

/// Button code (high nibble of the decrypted byte 7) → the C's `generic.btn`.
fn button_from_code(code: u8) -> Option<u8> {
    match code {
        0x0 => Some(1),
        0x2 => Some(2),
        0x4 => Some(3),
        0x6 => Some(4),
        0xC => Some(5),
        0xE => Some(6),
        _ => None,
    }
}

/// `generic.btn` → the code `get_upload` sends (other values go out unchanged).
fn code_from_button(btn: u8) -> u8 {
    match btn {
        1 => 0x0,
        2 => 0x2,
        3 => 0x4,
        4 => 0x6,
        5 => 0xC,
        6 => 0xE,
        other => other,
    }
}

/// The fields `remote_controller` reads out of a received/stored key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Fields {
    hold_cycle: u8,
    counter: u16,
    serial: u32,
    /// None for an unknown button code (the C keeps its previous `btn`).
    button: Option<u8>,
}

/// `subghz_protocol_came_atomo_remote_controller` (doctor's non-mutating form).
fn fields(data: u64) -> Fields {
    let mut pack = ((!data) << 4).to_be_bytes();
    atomo_decrypt(&mut pack);
    Fields {
        hold_cycle: pack[0],
        counter: u16::from_be_bytes([pack[1], pack[2]]),
        serial: u32::from_be_bytes([pack[3], pack[4], pack[5], pack[6]]),
        button: button_from_code(pack[7] >> 4),
    }
}

/// One encrypted frame (`get_upload`'s pack → `generic.data`): 60 bits.
fn frame_data(hold_cycle: u8, counter: u16, serial: u32, code: u8) -> u64 {
    let c = counter.to_be_bytes();
    let s = serial.to_be_bytes();
    let mut pack = [hold_cycle, c[0], c[1], s[0], s[1], s[2], s[3], code << 4];
    atomo_encrypt(&mut pack);
    ((!u64::from_be_bytes(pack)) >> 4) & 0x0FFF_FFFF_FFFF_FFFF
}

/// `subghz_protocol_came_atomo_create_data` (the SubGHz app's "Add Manually"): a new remote's key
/// file — hold-cycle byte 0x7E, the counter as given (not advanced), the serial and button 1
/// (code 0), encrypted like an upload frame; `Bit: 62`, `Key`.
pub(super) fn create_data(serial: u32, cnt: u16) -> FlipperKey {
    FlipperKey::for_protocol("CAME Atomo", MIN_COUNT_BIT as usize, frame_data(0x7E, cnt, serial, 0))
}

/// Flipper's `manchester_encoder.c` (`ManchesterEncoderState`): `advance` returns the result and
/// whether the bit was consumed (the caller re-sends it when not).
struct ManchesterEncoder {
    step: u8,
    prev_bit: bool,
}

/// `ManchesterEncoderResult` as the level of the half-bit pulse and whether it is long.
type EncoderResult = (bool, bool);

impl ManchesterEncoder {
    fn new() -> Self {
        Self { step: 0, prev_bit: false }
    }

    /// `manchester_encoder_advance`. Results: ShortLow (0b00) = (low, short), LongLow (0b01) =
    /// (low, long), LongHigh (0b10) = (high, long), ShortHigh (0b11) = (high, short).
    fn advance(&mut self, curr_bit: bool) -> (bool, EncoderResult) {
        match self.step {
            0 => {
                self.prev_bit = curr_bit;
                self.step = 1;
                (true, (!curr_bit, false))
            }
            1 => {
                let result = match (self.prev_bit, curr_bit) {
                    (false, false) => (false, false),
                    (false, true) => (false, true),
                    (true, false) => (true, true),
                    (true, true) => (true, false),
                };
                if curr_bit == self.prev_bit {
                    self.step = 2;
                    (false, result)
                } else {
                    self.prev_bit = curr_bit;
                    (true, result)
                }
            }
            _ => {
                self.prev_bit = curr_bit;
                self.step = 1;
                (true, (!curr_bit, false))
            }
        }
    }

    /// `manchester_encoder_finish`: the closing half of the last bit.
    fn finish(&mut self) -> EncoderResult {
        self.step = 0;
        (self.prev_bit, false)
    }
}

/// `subghz_protocol_encoder_came_atomo_add_duration_to_upload`.
fn push_result(out: &mut Vec<LevelDuration>, (high, long): EncoderResult) {
    push(out, high, if long { TE_LONG } else { TE_SHORT });
}

/// The counter `get_upload` sends (`CounterMode` 0 with multiplier 1, 1, 2, ≥ 3).
fn next_counter(mode: u8, cnt: u16) -> u16 {
    let cnt = u32::from(cnt);
    (match mode {
        0 => g6_common::counter_increment(cnt),
        1 => g6_common::counter_ofex(cnt),
        2 => {
            if cnt != 0x807B && cnt != 0x807C && cnt != 0x007B {
                0x807B
            } else if cnt == 0x807C {
                0x007B
            } else {
                cnt + 1
            }
        }
        _ => cnt,
    }) as u16
}

/// `subghz_protocol_encoder_came_atomo_get_upload` for a counter, serial and button code.
fn build_upload(counter: u16, serial: u32, code: u8) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 + 8 * 64);
    push(&mut out, true, TE_LONG * 15);
    push(&mut out, false, TE_LONG * 60);
    let mut enc = ManchesterEncoder::new();
    for hold_cycle in HOLD_CYCLE {
        let data = frame_data(hold_cycle, counter, serial, code);
        push(&mut out, true, TE_LONG);
        push(&mut out, false, TE_SHORT);
        for i in (0..u32::from(MIN_COUNT_BIT) - 2).rev() {
            let bit = (data >> i) & 1 == 0;
            let (consumed, result) = enc.advance(bit);
            push_result(&mut out, result);
            if !consumed {
                let (_, result) = enc.advance(bit);
                push_result(&mut out, result);
            }
        }
        let last = enc.finish();
        if last.0 {
            push_result(&mut out, last);
        }
        push(&mut out, false, TE_DELTA * 272);
    }
    out
}

/// Decoder steps (`CameAtomoDecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    DecoderData,
}

/// CAME Atomo decoder (`SubGhzProtocolDecoderCameAtomo`).
pub struct CameAtomoDecoder {
    step: Step,
    decode_data: u64,
    /// `SubGhzBlockDecoder.decode_count_bit` is a `uint8_t`.
    decode_count_bit: u8,
    manchester: ManchesterState,
    /// `generic.btn`: survives frames whose button code is unknown (and `reset()`), as in the C.
    btn: u8,
}

impl CameAtomoDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            decode_data: 0,
            decode_count_bit: 0,
            manchester: ManchesterState::Mid1,
            btn: 0,
        }
    }

    /// Manchester `Reset` then `ShortLow` (the C's start-of-frame sequence; ends in Mid1).
    fn restart_manchester(&mut self) {
        self.manchester.advance(ManchesterEvent::Reset);
        self.manchester.advance(ManchesterEvent::ShortLow);
    }

    /// The decode of a key (`get_string`); `btn` is the button kept from earlier frames.
    fn signal(data: u64, extra: Option<u64>, btn: u8) -> DecodedSignal {
        let f = fields(data);
        DecodedSignal {
            serial: Some(f.serial),
            button: Some(f.button.unwrap_or(btn)),
            counter: Some(f.counter),
            crc_valid: false,
            data,
            data_count_bit: MIN_COUNT_BIT as usize,
            encoder_capable: true,
            extra,
            protocol_display_name: None,
        }
    }
}

impl Default for CameAtomoDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for CameAtomoDecoder {
    fn name(&self) -> &'static str {
        "CAME Atomo"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: MIN_COUNT_BIT as usize,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    /// `subghz_protocol_decoder_came_atomo_reset`: parser step and Manchester state.
    fn reset(&mut self) {
        self.step = Step::Reset;
        self.manchester.advance(ManchesterEvent::Reset);
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                // 12 ms (TOP44RBN), 72 ms (TOP42R, TOP44R) — the garage copy — and doctor/ARF-lib's
                // 19 ms (TOPD4REN).
                if !level
                    && (duration_diff!(duration, TE_LONG * 10) < TE_DELTA * 20
                        || duration_diff!(duration, TE_LONG * 16) < TE_DELTA * 10
                        || duration_diff!(duration, TE_LONG * 60) < TE_DELTA * 40)
                {
                    self.step = Step::DecoderData;
                    self.decode_data = 0;
                    self.decode_count_bit = 1;
                    self.restart_manchester();
                }
                None
            }
            Step::DecoderData => {
                let mut result = None;
                let event = if !level {
                    if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                        Some(ManchesterEvent::ShortLow)
                    } else if duration_diff!(duration, TE_LONG) < TE_DELTA {
                        Some(ManchesterEvent::LongLow)
                    } else if duration >= TE_LONG * 2 + TE_DELTA {
                        if self.decode_count_bit == MIN_COUNT_BIT {
                            let data = self.decode_data;
                            let f = fields(data);
                            if let Some(b) = f.button {
                                self.btn = b;
                            }
                            result = Some(Self::signal(data, None, self.btn));
                        }
                        self.decode_data = 0;
                        self.decode_count_bit = 1;
                        self.restart_manchester();
                        None
                    } else {
                        self.step = Step::Reset;
                        None
                    }
                } else if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    Some(ManchesterEvent::ShortHigh)
                } else if duration_diff!(duration, TE_LONG) < TE_DELTA {
                    Some(ManchesterEvent::LongHigh)
                } else {
                    self.step = Step::Reset;
                    None
                };
                if let Some(event) = event {
                    if let Some(bit) = self.manchester.advance(event) {
                        self.decode_data = (self.decode_data << 1) | u64::from(!bit);
                        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
                    }
                }
                result
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT as usize {
            return None;
        }
        let f = fields(decoded.data);
        let mode = (decoded.extra.unwrap_or(0) >> 8) as u8;
        // A fresh encoder's `generic.btn` is 0 when the key's button code is unknown.
        let btn = if (1..=6).contains(&button) { button } else { f.button.unwrap_or(0) };
        Some(build_upload(next_counter(mode, f.counter), f.serial, code_from_button(btn)))
    }

    /// The decoder's deserialize (`Bit` must be 62; `CounterMode`) + `get_string`.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != MIN_COUNT_BIT as usize {
            return None;
        }
        let mode = g6_common::counter_mode(key);
        Some(Self::signal(key.key, (mode != 0).then_some(u64::from(mode) << 8), 0))
    }

    /// The garage serialize (`Protocol`, `Bit`, `Key` = the received bits) + the key's
    /// `CounterMode`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT as usize {
            return None;
        }
        let mut key = FlipperKey::new(
            "CAME Atomo",
            MIN_COUNT_BIT as usize,
            decoded.data,
            433_920_000,
            "FuriHalSubGhzPresetOok650Async",
        );
        let mode = (decoded.extra.unwrap_or(0) >> 8) as u8;
        if mode != 0 {
            key.push("CounterMode", mode.to_string());
        }
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::g6_common::{decode_all, tuples, unit_test_capture};
    use crate::protocols::ProtocolRegistry;

    /// The C's worked example (comment in `remote_controller`): received 0x1FF08D9924984115 →
    /// inverted 0x00F7266DB67BEEA0 → decrypted 0x0501FD0000A08300: hold cycle 5, counter 0x01FD,
    /// serial 0x0000A083, button code 0 (button 1).
    #[test]
    fn decrypts_the_c_example() {
        let f = fields(0x1FF0_8D99_2498_4115);
        assert_eq!(f, Fields { hold_cycle: 5, counter: 0x01FD, serial: 0x0000_A083, button: Some(1) });
    }

    #[test]
    fn cipher_round_trips() {
        for hold in [0u8, 1, 3, 10, 30, 0x7D, 0x7E, 0x7F] {
            for (cnt, serial, code) in [(0x01FDu16, 0x0000_A083u32, 0u8), (0xFFFF, 0xFFFF_FFFF, 0xE), (0, 0, 6)] {
                let data = frame_data(hold, cnt, serial, code);
                assert_eq!(data >> 60, 0);
                let f = fields(data);
                assert_eq!((f.hold_cycle, f.counter, f.serial), (hold, cnt, serial));
                assert_eq!(f.button, button_from_code(code));
            }
        }
    }

    #[test]
    fn manchester_encoder_matches_flipper() {
        // 1 = Low then High, 0 = High then Low; equal neighbours make the encoder re-send.
        let mut enc = ManchesterEncoder::new();
        let mut halves = Vec::new();
        for bit in [true, true, false, false, true] {
            let (consumed, r) = enc.advance(bit);
            halves.push(r);
            if !consumed {
                halves.push(enc.advance(bit).1);
            }
        }
        halves.push(enc.finish());
        let f = (false, false);
        let ll = (false, true);
        let lh = (true, true);
        let h = (true, false);
        assert_eq!(halves, vec![f, h, f, lh, f, h, ll, h]);
    }

    /// encode → decode: 4 of the 8 frames decode back in a Flipper-style receiver (it resets after
    /// each decode, so the pause that ends a frame cannot also start the next one), with the
    /// counter + 1 and the button.
    #[test]
    fn encode_decode_round_trip() {
        let key = (1u64 << 60) | frame_data(5, 0x01FD, 0x0000_A083, 0x2);
        let src = CameAtomoDecoder::signal(key, None, 0);
        assert_eq!(src.button, Some(2));
        for (button, want) in [(0u8, 2u8), (1, 1), (4, 4), (5, 5), (6, 6), (7, 2)] {
            for (mode, cnt_in, cnt_out) in [(0u8, 0x01FDu16, 0x01FEu16), (0, 0xFFFF, 0), (1, 0x01FD, 0xFFFE), (2, 0x807C, 0x007B), (3, 0x01FD, 0x01FD)] {
                let key = (1u64 << 60) | frame_data(5, cnt_in, 0x0000_A083, 0x2);
                let src = CameAtomoDecoder::signal(key, (mode != 0).then_some(u64::from(mode) << 8), 0);
                let wire = CameAtomoDecoder::new().encode(&src, button).unwrap();
                let got = decode_all(&mut CameAtomoDecoder::new(), &wire);
                assert_eq!(got.len(), 4, "button {button} mode {mode}");
                for (d, hold) in got.iter().zip([10u8, 125, 127, 1]) {
                    assert_eq!((d.serial, d.counter, d.button), (Some(0x0000_A083), Some(cnt_out), Some(want)));
                    assert_eq!(fields(d.data).hold_cycle, hold);
                    assert_eq!(d.data >> 60, 1, "start bit");
                }
                // Without the reset after each decode (Flipper's unit test feeds the decoder
                // directly) all 8 frames decode.
                let mut dec = CameAtomoDecoder::new();
                let all: Vec<_> = wire.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect();
                assert_eq!(all.len(), 8);
            }
        }
    }

    #[test]
    fn upload_layout_matches_the_c() {
        let key = (1u64 << 60) | frame_data(0x55, 0x1234, 0xCAFE_F00D, 0x4);
        let wire = CameAtomoDecoder::new().encode(&CameAtomoDecoder::signal(key, None, 0), 0).unwrap();
        let t = tuples(&wire);
        assert_eq!(&t[..3], &[(true, 18_000), (false, 72_000), (true, 1200)]);
        // 8 frames, each closed by a 68 ms pause that always follows a HIGH half-bit.
        assert_eq!(t.iter().filter(|p| **p == (false, 68_000)).count(), 8);
        assert_eq!(t.last(), Some(&(false, 68_000)));
        assert!(t.iter().all(|&(_, d)| [600, 1200, 18_000, 72_000, 68_000].contains(&d)), "{t:?}");
        // Each frame lasts 1800 µs of start + 60 bits × 1200 µs, minus a dropped final LOW half.
        let frame_us: Vec<u32> = t[2..]
            .split(|p| *p == (false, 68_000))
            .filter(|f| !f.is_empty())
            .map(|f| f.iter().map(|p| p.1).sum())
            .collect();
        assert_eq!(frame_us.len(), 8);
        assert!(frame_us.iter().all(|&us| us == 1800 + 72_000 || us == 1800 + 72_000 - 600), "{frame_us:?}");
    }

    /// Hand-built frame in the C's layout, and rejected variants.
    #[test]
    fn synthetic_frame_and_negative_cases() {
        let data = frame_data(0x10, 0x0BAD, 0x1234_5678, 0x6);
        let mut wire = Vec::new();
        push(&mut wire, false, 12_000);
        push(&mut wire, true, 1200);
        push(&mut wire, false, 600);
        let mut enc = ManchesterEncoder::new();
        for i in (0..60).rev() {
            let bit = (data >> i) & 1 == 0;
            let (c, r) = enc.advance(bit);
            push_result(&mut wire, r);
            if !c {
                push_result(&mut wire, enc.advance(bit).1);
            }
        }
        let last = enc.finish();
        if last.0 {
            push_result(&mut wire, last);
        }
        push(&mut wire, false, 20_000);
        let got = decode_all(&mut CameAtomoDecoder::new(), &wire);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].data, (1 << 60) | data);
        assert_eq!((got[0].serial, got[0].counter, got[0].button), (Some(0x1234_5678), Some(0x0BAD), Some(4)));
        // 19.2 ms header (doctor/ARF-lib TOPD4REN) too.
        let mut d19 = wire.clone();
        d19[0].duration_us = 19_200;
        assert_eq!(decode_all(&mut CameAtomoDecoder::new(), &d19).len(), 1);
        // Header too short (6 ms) / too long (25 ms): nothing.
        for gap in [6_000, 25_000] {
            let mut g = wire.clone();
            g[0].duration_us = gap;
            assert!(decode_all(&mut CameAtomoDecoder::new(), &g).is_empty(), "{gap}");
        }
        // A pulse out of tolerance aborts; a frame cut short ends with the wrong bit count.
        let mut bad = wire.clone();
        bad[10].duration_us = 900;
        assert!(decode_all(&mut CameAtomoDecoder::new(), &bad).is_empty());
        let mut cut = wire[..wire.len() - 6].to_vec();
        push(&mut cut, false, 20_000);
        assert!(decode_all(&mut CameAtomoDecoder::new(), &cut).is_empty());
    }

    #[test]
    fn unknown_button_code_keeps_the_previous_button() {
        let mut dec = CameAtomoDecoder::new();
        let mut wire = build_upload(0x10, 0xAA55_AA55, 0x6);
        wire.extend(build_upload(0x11, 0xAA55_AA55, 0x9));
        let got = decode_all(&mut dec, &wire);
        assert_eq!(got.len(), 8);
        assert!(got.iter().all(|d| d.button == Some(4)), "{got:?}");
        assert_eq!(fields(got[7].data).button, None);
    }

    const KEY: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: CAME Atomo\nBit: 62\n\
        Key: 1F F0 8D 99 24 98 41 15\n";

    #[test]
    fn key_file_loads_and_round_trips() {
        let dec = CameAtomoDecoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(KEY).unwrap()).unwrap();
        assert_eq!((d.serial, d.counter, d.button, d.extra), (Some(0xA083), Some(0x01FD), Some(1), None));
        assert!(dec.from_flipper_key(&FlipperKey::parse(&KEY.replace("Bit: 62", "Bit: 61")).unwrap()).is_none());
        for text in [KEY.to_string(), KEY.to_string() + "CounterMode: 2\n"] {
            let a = dec.from_flipper_key(&FlipperKey::parse(&text).unwrap()).unwrap();
            let written = dec.to_flipper_key(&a).unwrap().to_text();
            assert_eq!(written.lines().skip(2).collect::<Vec<_>>(), text.lines().skip(2).collect::<Vec<_>>());
            let b = dec.from_flipper_key(&FlipperKey::parse(&written).unwrap()).unwrap();
            assert_eq!(format!("{a:?}"), format!("{b:?}"));
        }
    }

    /// Flipper unit test `came_atomo_raw.sub` decodes through the registry.
    #[test]
    fn unit_test_capture_decodes() {
        let Some((freq, pulses)) = unit_test_capture("came_atomo_raw.sub") else { return };
        let mut reg = ProtocolRegistry::new();
        let hits: Vec<_> = reg
            .process_signal_stream(&pulses, freq)
            .into_iter()
            .filter(|(n, _, _)| n == "CAME Atomo")
            .collect();
        assert!(!hits.is_empty());
        assert!(hits.iter().all(|(_, d, _)| d.data_count_bit == 62 && d.data >> 60 == 1));
    }
}
