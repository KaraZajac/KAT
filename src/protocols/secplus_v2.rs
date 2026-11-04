//! Security+ 2.0 (Chamberlain / LiftMaster / Craftsman openers since 2011, "yellow learn button"
//! remotes; 310 / 315 / 390 / 433 MHz): two Manchester packets carrying a 40-bit fixed code (8-bit
//! button + 32-bit remote id) and a 28-bit rolling counter, mixed by a per-packet order/invert code.
//!
//! Port of `secplus_v2.c` from Flipper-ARF's garage app
//! (`applications/system/subghz_garage/protocols/`, the primary reference; its logic is Flipper-ARF
//! lib's copy behind compatibility shims), plus doctor-firmware's extension of the same file
//! (`lib/subghz/protocols/secplus_v2.c`) for the 86-bit frames of Security+ 2.0 keypads, which the
//! garage copy does not decode (KAT takes the union). The references cite argilo/secplus and
//! rtl_433's `secplus_v2.c`.
//!
//! **Air format** (AM/OOK): Manchester, te_short 250 / te_long 500 µs (te_delta 110, strict `<`),
//! Flipper convention (1 = LOW then HIGH). Each packet follows a long LOW: remotes send 62-bit
//! packets (16 zero bits, `1111`, 2 unused bits, packet id in bits 41..40, 2 unused bits, then the
//! 38-bit payload: order 37..34, invert 33..30, 30 interleaved bits); keypads send 86 bits (a 22-bit
//! preamble `0x3C | packet id`, then a 64-bit payload `01`, order 61..58, invert 57..54, 54
//! interleaved bits with a 16-bit data field per packet — PIN and a check nibble).
//!
//! **Decoder** (`subghz_protocol_decoder_secplus_v2_feed`, doctor-firmware's superset): a LOW of
//! 65000 ± 11000 µs (garage and doctor) or 89500 ± 7920 µs (doctor, keypads) arms it (packet 1 is
//! cleared, Manchester state primed with Reset/LongHigh/ShortLow); pulses then feed Flipper's
//! Manchester machine; any LOW ≥ 1110 µs ends a packet. A 62-bit packet must match the 0x3C header
//! mask; id 0 is stored as packet 1, id 1 completes the pair when a 62-bit packet 1 is stored. An
//! 86-bit packet (doctor) needs `payload >> 62 == 1`; preamble id 0 is stored, id 1 completes the
//! pair; a stored 86-bit packet 1 is dropped after a gap > 179 ms. On a pair the C calls back and
//! re-arms (its `reset` is empty, as is KAT's). On 62-bit-only streams doctor's decoder reports
//! exactly the garage copy's frames, plus pairs behind an 81.6–97.4 ms gap.
//!
//! **Fields** (`subghz_protocol_secplus_v2_remote_controller` / doctor's `_86`): each packet is
//! de-interleaved into three words, un-inverted and re-ordered per its order/invert code; the
//! rolling digits of both packets give the 28-bit counter (bit-reversed base-3 number), packet 1's
//! fixed half gives `button = fixed_1 >> 12` and the id `serial = fixed_1 << 20 | fixed_2` (32 bits,
//! as the C's `uint32_t` keeps it; with the button that is the whole 40-bit fixed code). KAT's
//! `counter` holds the low 16 bits of the 28-bit counter (`data`/`extra` carry all of it). The C
//! reports every header-valid pair and shows zeros when a packet's order/invert code or a digit is
//! invalid (and, for 86 bits, when the data nibbles XOR with the button to non-zero); KAT emits only
//! pairs whose fields decode (`crc_valid` = true) — see docs/secplus_v2.md for the oracle counts.
//!
//! **Packing**: `data` = the references' `Key` (packet 2, `generic.data`: 62 bits, or the 64-bit
//! payload of an 86-bit packet), `extra` = `Secplus_packet_1` (packet 1, same form),
//! `data_count_bit` = the references' `Bit` (62 or 86).
//!
//! **Encoder**: fields from the packets (62 bits: zeros if they do not decode, as in the C; 86 bits:
//! must decode or nothing is sent, as in doctor-firmware); `button` = the native 8-bit button code
//! (the references' button editor accepts any 8-bit value; the D-pad offers 0x68/0x80/0x81/0xE2/
//! 0x78); counter + 1, above 0xFFFFFFF → 0xE500000, below 0xE500000 → 0xE500000. 62 bits (garage):
//! both packets rebuilt, upload = Manchester packet 1, its last half kept only if HIGH, LOW
//! 500 × 136; the same for packet 2; `encoder.repeat` = 3 copies. 86 bits (doctor): the data word
//! keeps its PIN, its check nibble is recomputed for the button; per packet HIGH 250 + LOW 500,
//! the 22-bit preamble and 64-bit payload in Manchester, LOW 500 × 182; packet 2 (id 1) first, then
//! packet 1; 4 copies. (doctor-firmware sends 62-bit pairs packet 2 first with 500 × 138 gaps, as
//! real remotes do; KAT keeps the garage order, which every reference decoder also accepts — see the
//! docs. The garage app's TX plugins are built without `SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS` /
//! `_CUSTOM_BTN`, which would put the counter in "OFEX" mode and clear the button; KAT follows the
//! source's normal mode.)
//!
//! **Key file**: 62 bits (garage `serialize`): `Bit: 62`, `Key`, `Secplus_packet_1`. 86 bits
//! (doctor): `Bit: 86`, `Key`, `Serial`/`Btn`/`Cnt` (doctor's generic optional fields), then
//! `Secplus_packet_1` and `Pin` (4 digits, empty when the frame's PIN is not 0000-9999). Loading
//! needs `Secplus_packet_1`; a valid `Pin` replaces the frame's PIN (doctor's encoder does).

use super::common::{ManchesterEvent, ManchesterState};
use super::flipper_key::FlipperKey;
use super::g4_common::{push, reverse_key, ManchesterEncoder, ManchesterResult};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`). Flags: 315 MHz, AM (doctor-firmware also flags 433). The references
/// do not name the 8-bit button codes.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Security+ 2.0",
    decoder: "Security+ 2.0",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Chamberlain/LiftMaster",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Security+ 2.0";
const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 110;
const BITS_62: u8 = 62;
const BITS_86: u8 = 86;

const HEADER: u64 = 0x3C00_0000_0000;
const HEADER_MASK: u64 = 0xFFFF_3C00_0000_0000;
const PACKET_1: u64 = 0x0000_0000_0000;
const PACKET_2: u64 = 0x0100_0000_0000;
const PACKET_MASK: u64 = 0x0300_0000_0000;

/// 86-bit frames: 22-bit preamble `0x3C | packet id` before the 64-bit payload.
const PREAMBLE_86: u64 = 0x3C;
const PREAMBLE_86_BITS: u8 = 22;

/// Sync gaps (× te_long), as in doctor-firmware; the garage copy has only the 62-bit window.
const SYNC_62: u32 = 130;
const SYNC_62_DELTA: u32 = 100;
const SYNC_86: u32 = 179;
const SYNC_86_DELTA: u32 = 72;
/// A stored 86-bit packet 1 is dropped after a gap longer than te_long × 358.
const PAIR_TIMEOUT: u32 = 358;

/// Gap after each packet in the upload (× te_long): garage 62-bit, doctor 86-bit.
const GAP_62: u32 = 136;
const GAP_86: u32 = 182;
const REPEAT_62: usize = 3;
const REPEAT_86: usize = 4;

const P_MASK_62: u32 = 0x03FF;
const P_MASK_86: u32 = 0x3FFFF;

const COUNTER_MAX: u32 = 0xFFF_FFFF;
const COUNTER_FLOOR: u32 = 0xE50_0000;

/// `SecPlus_v2DecoderStep`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    DecoderData,
}

/// Security+ 2.0 decoder (`SubGhzProtocolDecoderSecPlus_v2`, doctor-firmware layout).
pub struct SecPlusV2Decoder {
    step: Step,
    manchester: ManchesterState,
    decode_data: u64,
    /// `SubGhzBlockDecoder.decode_count_bit` is a `uint8_t`.
    decode_count_bit: u8,
    packet_1: u64,
    packet_1_count_bit: u8,
    packet_id: u8,
}

impl SecPlusV2Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            manchester: ManchesterState::Start1,
            decode_data: 0,
            decode_count_bit: 0,
            packet_1: 0,
            packet_1_count_bit: 0,
            packet_id: 0,
        }
    }

    /// Manchester Reset, LongHigh, ShortLow (the C's three `manchester_advance` calls).
    fn prime_manchester(&mut self) {
        self.manchester = ManchesterState::Mid1;
        self.manchester.advance(ManchesterEvent::LongHigh);
        self.manchester.advance(ManchesterEvent::ShortLow);
    }

    /// `subghz_protocol_secplus_v2_check_packet` (doctor-firmware).
    fn check_packet(&mut self) -> bool {
        if self.decode_count_bit == BITS_86 {
            if self.decode_data >> 62 != 1 {
                return false;
            }
            if self.packet_id == 0 {
                self.packet_1 = self.decode_data;
                self.packet_1_count_bit = BITS_86;
                return false;
            }
            return self.packet_id == 1 && self.packet_1 != 0 && self.packet_1_count_bit == BITS_86;
        }
        if self.decode_data & HEADER_MASK == HEADER {
            if self.decode_data & PACKET_MASK == PACKET_1 {
                self.packet_1 = self.decode_data;
                self.packet_1_count_bit = BITS_62;
            } else if self.decode_data & PACKET_MASK == PACKET_2
                && self.packet_1 != 0
                && self.packet_1_count_bit == BITS_62
            {
                return true;
            }
        }
        false
    }
}

impl Default for SecPlusV2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `subghz_protocol_secplus_v2_mix_invet`.
fn mix_invert(invert: u8, p: &mut [u32; 3], mask: u32) -> bool {
    let flip: &[usize] = match invert {
        0x00 => &[0, 1],
        0x01 => &[1],
        0x02 => &[2],
        0x04 => &[0, 1, 2],
        0x05 | 0x0A => &[0, 2],
        0x06 => &[1, 2],
        0x08 => &[0],
        0x09 => &[],
        _ => return false,
    };
    for &i in flip {
        p[i] = !p[i] & mask;
    }
    true
}

/// `subghz_protocol_secplus_v2_mix_order_decode`.
fn mix_order_decode(order: u8, p: &mut [u32; 3]) -> bool {
    let [a, b, c] = *p;
    *p = match order {
        0x06 | 0x09 => [c, b, a],
        0x08 | 0x04 => [c, a, b],
        0x01 => [b, c, a],
        0x00 => [a, c, b],
        0x05 => [b, a, c],
        0x02 | 0x0A => [a, b, c],
        _ => return false,
    };
    true
}

/// `subghz_protocol_secplus_v2_mix_order_encode`.
fn mix_order_encode(order: u8, p: &mut [u32; 3]) -> bool {
    let [p0, p1, p2] = *p;
    *p = match order {
        0x06 | 0x09 => [p2, p1, p0],
        0x08 | 0x04 => [p1, p2, p0],
        0x01 => [p2, p0, p1],
        0x00 => [p0, p2, p1],
        0x05 => [p1, p0, p2],
        0x02 | 0x0A => [p0, p1, p2],
        _ => return false,
    };
    true
}

/// De-interleave `words` 3-bit groups from bit `top` down into three words.
fn deinterleave(data: u64, top: u32, words: u32) -> [u32; 3] {
    let mut p = [0u32; 3];
    for k in 0..words {
        let i = top - 3 * k;
        p[0] = p[0] << 1 | ((data >> i) & 1) as u32;
        p[1] = p[1] << 1 | ((data >> (i - 1)) & 1) as u32;
        p[2] = p[2] << 1 | ((data >> (i - 2)) & 1) as u32;
    }
    p
}

/// Interleave the low `bits` bits of the three words, MSB first, 3 bits per step.
fn interleave(p: &[u32; 3], bits: u32) -> u64 {
    (0..bits).fold(0u64, |data, k| {
        let i = bits - 1 - k;
        (data << 3)
            | (((p[0] >> i) & 1) as u64 * 4)
            | (((p[1] >> i) & 1) as u64 * 2)
            | ((p[2] >> i) & 1) as u64
    })
}

/// The four header digits (order and invert codes, 2 bits each); None if one is 3.
fn header_digits(order: u8, invert: u8) -> Option<[u8; 4]> {
    let d = [order >> 2, order & 3, invert >> 2, invert & 3];
    d.iter().all(|&x| x != 3).then_some(d)
}

/// `subghz_protocol_secplus_v2_decode_half`: (roll digits, 20-bit fixed half).
fn decode_half(data: u64) -> Option<([u8; 9], u32)> {
    let order = ((data >> 34) & 0xF) as u8;
    let invert = ((data >> 30) & 0xF) as u8;
    let mut p = deinterleave(data, 29, 10);
    if !mix_invert(invert, &mut p, P_MASK_62) || !mix_order_decode(order, &mut p) {
        return None;
    }
    let mut roll = [0u8; 9];
    roll[..4].copy_from_slice(&header_digits(order, invert)?);
    for k in 0..5 {
        roll[4 + k] = ((p[2] >> (8 - 2 * k)) & 3) as u8;
        if roll[4 + k] == 3 {
            return None;
        }
    }
    Some((roll, p[0] << 10 | p[1]))
}

/// `subghz_protocol_secplus_v2_decode_half_86` (doctor): (roll digits, 20-bit fixed, 16-bit data).
fn decode_half_86(data: u64) -> Option<([u8; 9], u32, u16)> {
    if data >> 62 != 1 {
        return None;
    }
    let invert = ((data >> 54) & 0xF) as u8;
    let order = ((data >> 58) & 0xF) as u8;
    let mut p = deinterleave(data, 53, 18);
    if !mix_invert(invert, &mut p, P_MASK_86) || !mix_order_decode(order, &mut p) {
        return None;
    }
    let mut digits = [0u8; 13];
    digits[..4].copy_from_slice(&header_digits(order, invert)?);
    for k in 0..9 {
        digits[4 + k] = ((p[2] >> (16 - 2 * k)) & 3) as u8;
        if digits[4 + k] == 3 {
            return None;
        }
    }
    // The tail repeats the head.
    if digits[..4] != digits[9..13] {
        return None;
    }
    let mut roll = [0u8; 9];
    roll.copy_from_slice(&digits[..9]);
    let fixed = ((p[0] << 2) & 0xF_FC00) | ((p[1] >> 8) & 0x3FF);
    let data_part = ((p[0] << 8) | (p[1] & 0xFF)) as u16;
    Some((roll, fixed, data_part))
}

/// The 18 counter digits from both packets' roll digits → the 28-bit counter (None if ≥ 2^28).
fn rolling_join(roll_1: &[u8; 9], roll_2: &[u8; 9]) -> Option<u32> {
    let mut d = [0u8; 18];
    d[0] = roll_2[8];
    d[1] = roll_1[8];
    d[2..6].copy_from_slice(&roll_2[4..8]);
    d[6..10].copy_from_slice(&roll_1[4..8]);
    d[10..14].copy_from_slice(&roll_2[..4]);
    d[14..18].copy_from_slice(&roll_1[..4]);
    let v = d.iter().fold(0u32, |v, &x| v * 3 + x as u32);
    (v < 0x1000_0000).then(|| reverse_key(v as u64, 28) as u32)
}

/// The inverse: the counter's digits split into both packets' roll digits.
fn rolling_split(counter: u32) -> ([u8; 9], [u8; 9]) {
    let mut rolling = reverse_key(counter as u64, 28) as u32;
    let mut d = [0u8; 18];
    for i in (0..18).rev() {
        d[i] = (rolling % 3) as u8;
        rolling /= 3;
    }
    let (mut roll_1, mut roll_2) = ([0u8; 9], [0u8; 9]);
    roll_2[8] = d[0];
    roll_1[8] = d[1];
    roll_2[4..8].copy_from_slice(&d[2..6]);
    roll_1[4..8].copy_from_slice(&d[6..10]);
    roll_2[..4].copy_from_slice(&d[10..14]);
    roll_1[..4].copy_from_slice(&d[14..18]);
    (roll_1, roll_2)
}

/// Fields of a 62-bit pair: `subghz_protocol_secplus_v2_remote_controller` when it succeeds.
struct Fields {
    counter: u32,
    button: u8,
    serial: u32,
    /// 86-bit frames: the 32-bit data word (PIN, check nibble).
    data_word: u32,
}

/// `subghz_protocol_secplus_v2_remote_controller` (None where the C sets zeros).
fn fields_62(packet_1: u64, packet_2: u64) -> Option<Fields> {
    let (roll_1, fixed_1) = decode_half(packet_1)?;
    let (roll_2, fixed_2) = decode_half(packet_2)?;
    let counter = rolling_join(&roll_1, &roll_2)?;
    Some(Fields {
        counter,
        button: (fixed_1 >> 12) as u8,
        serial: fixed_1 << 20 | fixed_2,
        data_word: 0,
    })
}

/// The data word's check: its 8 nibbles XOR the button's low nibble to zero.
fn data_check(button: u8, data_word: u32) -> u8 {
    (0..32)
        .step_by(4)
        .fold(button & 0xF, |c, i| c ^ ((data_word >> i) & 0xF) as u8)
}

/// `subghz_protocol_secplus_v2_remote_controller_86` (doctor; None where it returns false).
fn fields_86(packet_1: u64, packet_2: u64) -> Option<Fields> {
    let (roll_1, fixed_1, data_1) = decode_half_86(packet_1)?;
    let (roll_2, fixed_2, data_2) = decode_half_86(packet_2)?;
    let counter = rolling_join(&roll_1, &roll_2)?;
    let data_word = (data_1 as u32) << 16 | data_2 as u32;
    let button = (fixed_1 >> 12) as u8;
    (data_check(button, data_word) == 0).then_some(Fields {
        counter,
        button,
        serial: fixed_1 << 20 | fixed_2,
        data_word,
    })
}

/// `subghz_protocol_secplus_v2_pin_extract`: the byte-swapped top half, if 0000-9999.
fn pin_extract(data_word: u32) -> Option<u16> {
    let high = (data_word >> 16) as u16;
    Some(high.swap_bytes()).filter(|&pin| pin <= 9999)
}

/// `subghz_protocol_secplus_v2_pin_apply` (the check nibble is fixed by the encoder).
fn pin_apply(data_word: u32, pin: u16) -> u32 {
    (pin.swap_bytes() as u32) << 16 | (data_word & 0xFFFF)
}

/// `subghz_protocol_secplus_v2_pin_read`: `atoi` of the trimmed `Pin` line, if 0..=9999.
fn pin_read(key: &FlipperKey) -> Option<u16> {
    let text = key.field("Pin")?.trim();
    if text.is_empty() {
        return None;
    }
    let (negative, digits) = match text.as_bytes()[0] {
        b'-' => (true, &text[1..]),
        b'+' => (false, &text[1..]),
        _ => (false, text),
    };
    let value = digits
        .bytes()
        .take_while(u8::is_ascii_digit)
        .fold(0i64, |v, b| (v * 10 + (b - b'0') as i64).min(1 << 40));
    (!negative && value <= 9999).then_some(value as u16)
}

/// `subghz_protocol_secplus_v2_encode_half`.
fn encode_half(roll: &[u8; 9], fixed: u32) -> u64 {
    let order = roll[0] << 2 | roll[1];
    let invert = roll[2] << 2 | roll[3];
    let mut p = [
        (fixed >> 10) & 0x3FF,
        fixed & 0x3FF,
        (roll[4] as u32) << 8
            | (roll[5] as u32) << 6
            | (roll[6] as u32) << 4
            | (roll[7] as u32) << 2
            | roll[8] as u32,
    ];
    if !mix_order_encode(order, &mut p) || !mix_invert(invert, &mut p, P_MASK_62) {
        return 0;
    }
    interleave(&p, 10) | (order as u64) << 34 | (invert as u64) << 30
}

/// `subghz_protocol_secplus_v2_encode_half_86` (doctor).
fn encode_half_86(roll: &[u8; 9], fixed: u32, data_part: u16) -> u64 {
    let order = roll[0] << 2 | roll[1];
    let invert = roll[2] << 2 | roll[3];
    let r = |i: usize| roll[i] as u32;
    let mut p = [
        ((fixed >> 10) & 0x3FF) << 8 | ((data_part >> 8) & 0xFF) as u32,
        (fixed & 0x3FF) << 8 | (data_part & 0xFF) as u32,
        r(4) << 16
            | r(5) << 14
            | r(6) << 12
            | r(7) << 10
            | r(8) << 8
            | r(0) << 6
            | r(1) << 4
            | r(2) << 2
            | r(3),
    ];
    if !mix_order_encode(order, &mut p) || !mix_invert(invert, &mut p, P_MASK_86) {
        return 0;
    }
    interleave(&p, 18) | (order as u64) << 58 | (invert as u64) << 54 | 1 << 62
}

/// The fixed halves for a button and 32-bit id: `btn << 12 | serial >> 20`, `serial & 0xFFFFF`.
fn fixed_halves(button: u8, serial: u32) -> (u32, u32) {
    ((button as u32) << 12 | serial >> 20, serial & 0xF_FFFF)
}

/// `subghz_protocol_secplus_v2_encode` after the counter step: (packet 1, packet 2).
fn encode_62(serial: u32, button: u8, counter: u32) -> (u64, u64) {
    let (fixed_1, fixed_2) = fixed_halves(button, serial);
    let (roll_1, roll_2) = rolling_split(counter);
    (
        HEADER | PACKET_1 | encode_half(&roll_1, fixed_1),
        HEADER | PACKET_2 | encode_half(&roll_2, fixed_2),
    )
}

/// `subghz_protocol_secplus_v2_encode_86` (doctor): (packet 1, packet 2), check nibble recomputed.
fn encode_86(serial: u32, button: u8, counter: u32, data_word: u32) -> (u64, u64) {
    let (fixed_1, fixed_2) = fixed_halves(button, serial);
    let (roll_1, roll_2) = rolling_split(counter);
    let word = data_word & !0xF000;
    let word = word | (data_check(button, word) as u32 & 0xF) << 12;
    (
        encode_half_86(&roll_1, fixed_1, (word >> 16) as u16),
        encode_half_86(&roll_2, fixed_2, word as u16),
    )
}

/// The encoders' counter step (the references' normal mode, multiplier 1).
fn next_counter(counter: u32) -> u32 {
    let next = if counter + 1 > COUNTER_MAX {
        COUNTER_FLOOR
    } else {
        counter + 1
    };
    next.max(COUNTER_FLOOR)
}

/// `subghz_protocol_secplus_v2_create_data` (the SubGHz app's "Add Manually"): `encode` with its
/// counter step (+ 1, at least 0xE500000); `Bit: 62`, `Key` (packet 2), `Secplus_packet_1`.
pub(super) fn create_data(serial: u32, btn: u8, cnt: u32) -> FlipperKey {
    let (packet_1, packet_2) = encode_62(serial, btn, next_counter(cnt));
    let mut key = FlipperKey::for_protocol(NAME, BITS_62 as usize, packet_2);
    key.push_hex("Secplus_packet_1", packet_1, 8);
    key
}

/// One packet of the upload: optional 86-bit lead-in and preamble, Manchester payload, the final
/// half only if HIGH (a LOW one is overwritten by the gap), then the gap.
fn push_packet(
    out: &mut Vec<LevelDuration>,
    payload: u64,
    bits: u32,
    preamble: Option<u64>,
    gap: u32,
) {
    let mut enc = ManchesterEncoder::new();
    let emit = |out: &mut Vec<LevelDuration>, r: ManchesterResult| {
        let (level, duration) = r.level_duration(TE_SHORT, TE_LONG);
        push(out, level, duration);
    };
    if let Some(preamble) = preamble {
        push(out, true, TE_SHORT);
        push(out, false, TE_LONG);
        for i in (0..PREAMBLE_86_BITS as u32).rev() {
            enc.encode_bit((preamble >> i) & 1 == 1, |r| emit(out, r));
        }
    }
    for i in (0..bits).rev() {
        enc.encode_bit((payload >> i) & 1 == 1, |r| emit(out, r));
    }
    let last = enc.finish();
    if last.level_duration(TE_SHORT, TE_LONG).0 {
        emit(out, last);
    }
    push(out, false, TE_LONG * gap);
}

/// The decode for a pair (fields as the references display them).
fn decoded(packet_1: u64, packet_2: u64, bits: u8, fields: Option<&Fields>) -> DecodedSignal {
    DecodedSignal {
        serial: Some(fields.map_or(0, |f| f.serial)),
        button: Some(fields.map_or(0, |f| f.button)),
        counter: Some(fields.map_or(0, |f| f.counter as u16)),
        crc_valid: fields.is_some(),
        data: packet_2,
        data_count_bit: bits as usize,
        // 62 bits: the C sends (zero fields when undecodable); 86 bits: only a decodable pair.
        encoder_capable: bits == BITS_62 || fields.is_some(),
        extra: Some(packet_1),
        protocol_display_name: None,
    }
}

fn fields_for(packet_1: u64, packet_2: u64, bits: u8) -> Option<Fields> {
    if bits == BITS_86 {
        fields_86(packet_1, packet_2)
    } else {
        fields_62(packet_1, packet_2)
    }
}

impl ProtocolDecoder for SecPlusV2Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: BITS_62 as usize,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    /// `subghz_protocol_decoder_secplus_v2_reset` does nothing (doctor-firmware only clears the
    /// PIN it shows for a loaded file): a pair's packets arrive one gap apart.
    fn reset(&mut self) {}

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level
                    && (duration_diff!(duration, TE_LONG * SYNC_62) < TE_DELTA * SYNC_62_DELTA
                        || duration_diff!(duration, TE_LONG * SYNC_86) < TE_DELTA * SYNC_86_DELTA)
                {
                    self.step = Step::DecoderData;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.packet_1 = 0;
                    self.packet_1_count_bit = 0;
                    self.prime_manchester();
                }
                None
            }
            Step::DecoderData => {
                let short = duration_diff!(duration, TE_SHORT) < TE_DELTA;
                let long = duration_diff!(duration, TE_LONG) < TE_DELTA;
                if !short && !long {
                    if level || duration < TE_LONG * 2 + TE_DELTA {
                        self.step = Step::Reset;
                        return None;
                    }
                    // End of a packet.
                    let mut result = None;
                    let bits = self.decode_count_bit;
                    if bits == BITS_62 || bits == BITS_86 {
                        let packet_2 = self.decode_data;
                        if self.check_packet() {
                            let fields = fields_for(self.packet_1, packet_2, bits);
                            // KAT: only pairs whose fields decode (the C shows zeros otherwise).
                            if let Some(f) = &fields {
                                result = Some(decoded(self.packet_1, packet_2, bits, Some(f)));
                            }
                            self.step = Step::Reset;
                        }
                    }
                    if self.packet_1_count_bit == BITS_86 && duration > TE_LONG * PAIR_TIMEOUT {
                        self.packet_1 = 0;
                        self.packet_1_count_bit = 0;
                    }
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.prime_manchester();
                    return result;
                }
                let event = ManchesterEvent::from_half_bit(level, long);
                if let Some(bit) = self.manchester.advance(event) {
                    self.decode_data = self.decode_data << 1 | bit as u64;
                    self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
                    if self.decode_count_bit == PREAMBLE_86_BITS {
                        self.packet_id = (self.decode_data & 0x03) as u8;
                    }
                }
                None
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let packet_1 = decoded.extra?;
        let mut out = Vec::new();
        match decoded.data_count_bit {
            62 => {
                let (counter, serial) =
                    fields_62(packet_1, decoded.data).map_or((0, 0), |f| (f.counter, f.serial));
                let (p1, p2) = encode_62(serial, button, next_counter(counter));
                for _ in 0..REPEAT_62 {
                    push_packet(&mut out, p1, BITS_62 as u32, None, GAP_62);
                    push_packet(&mut out, p2, BITS_62 as u32, None, GAP_62);
                }
            }
            86 => {
                let f = fields_86(packet_1, decoded.data)?;
                let (p1, p2) = encode_86(f.serial, button, next_counter(f.counter), f.data_word);
                for _ in 0..REPEAT_86 {
                    push_packet(&mut out, p2, 64, Some(PREAMBLE_86 | 1), GAP_86);
                    push_packet(&mut out, p1, 64, Some(PREAMBLE_86), GAP_86);
                }
            }
            _ => return None,
        }
        Some(out)
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let bits = match key.bits {
            62 => BITS_62,
            86 => BITS_86,
            _ => return None,
        };
        let mut packet_1 = key.hex_field("Secplus_packet_1")?;
        let mut packet_2 = key.key;
        let mut fields = fields_for(packet_1, packet_2, bits);
        if let (Some(f), Some(pin)) = (fields.as_ref().filter(|_| bits == BITS_86), pin_read(key)) {
            // doctor-firmware's encoder transmits the file's PIN: rebuild the packets with it.
            if pin_extract(f.data_word) != Some(pin) {
                (packet_1, packet_2) =
                    encode_86(f.serial, f.button, f.counter, pin_apply(f.data_word, pin));
                fields = fields_86(packet_1, packet_2);
            }
        }
        Some(decoded(packet_1, packet_2, bits, fields.as_ref()))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let packet_1 = decoded.extra?;
        let preset = "FuriHalSubGhzPresetOok650Async";
        match decoded.data_count_bit {
            62 => {
                let mut key = FlipperKey::new(NAME, 62, decoded.data, 0, preset);
                key.push_hex("Secplus_packet_1", packet_1, 8);
                Some(key)
            }
            86 => {
                let mut key = FlipperKey::new(NAME, 86, decoded.data, 0, preset);
                let fields = fields_86(packet_1, decoded.data);
                if let Some(f) = &fields {
                    if f.serial != 0 || f.button != 0 || f.counter != 0 {
                        key.push("Serial", f.serial.to_string());
                        key.push("Btn", f.button.to_string());
                        key.push("Cnt", f.counter.to_string());
                    }
                }
                key.push_hex("Secplus_packet_1", packet_1, 8);
                let pin = fields.and_then(|f| pin_extract(f.data_word));
                key.push("Pin", pin.map(|p| format!("{p:04}")).unwrap_or_default());
                Some(key)
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    fn feed_all(dec: &mut SecPlusV2Decoder, wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        wire.iter()
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect()
    }

    /// The unit-test key file (`security_pls_2_0.sub`).
    const UT_KEY: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Security+ 2.0\nBit: 62\n\
        Key: 00 00 3D 29 0F B9 BE EE\nSecplus_packet_1: 00 00 3C 01 6B 19 DD 60\n";

    fn key_decode(text: &str) -> DecodedSignal {
        SecPlusV2Decoder::new()
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap()
    }

    #[test]
    fn mixing_round_trips() {
        for order in [0u8, 1, 2, 4, 5, 6, 8, 9, 10] {
            for invert in [0u8, 1, 2, 4, 5, 6, 8, 9, 10] {
                let orig = [0x155, 0x2AA, 0x0F0];
                let mut p = orig;
                assert!(mix_order_encode(order, &mut p));
                assert!(mix_invert(invert, &mut p, P_MASK_62));
                assert!(mix_invert(invert, &mut p, P_MASK_62));
                assert!(mix_order_decode(order, &mut p));
                assert_eq!(p, orig, "order {order:#X} invert {invert:#X}");
            }
        }
        for bad in [3u8, 7, 11, 12, 13, 14, 15] {
            assert!(!mix_invert(bad, &mut [0; 3], P_MASK_62));
            assert!(!mix_order_decode(bad, &mut [0; 3]));
        }
    }

    #[test]
    fn fields_round_trip_62_and_86() {
        for &(serial, button, counter) in &[
            (0x74C5_8200u32, 0x10u8, 0xE50_0343u32),
            (0, 0, 0),
            (0xFFFF_FFFF, 0xFF, COUNTER_MAX),
            (0x1234_5678, 0x68, 0x00A_BCDE),
        ] {
            let (p1, p2) = encode_62(serial, button, counter);
            let f = fields_62(p1, p2).expect("62-bit fields");
            assert_eq!((f.serial, f.button, f.counter), (serial, button, counter));
            for data_word in [0u32, 0xD204_0000, 0xFFFF_0FFF] {
                let (p1, p2) = encode_86(serial, button, counter, data_word);
                let f = fields_86(p1, p2).expect("86-bit fields");
                assert_eq!((f.serial, f.button, f.counter), (serial, button, counter));
                assert_eq!(f.data_word & !0xF000, data_word & !0xF000);
            }
        }
        // The PIN is the binary value, byte-swapped: 1234 = 0x04D2 → 0xD204.
        assert_eq!(pin_extract(0xD204_0000), Some(1234));
        assert_eq!(pin_extract(pin_apply(0xFFFF_ABCD, 42)), Some(42));
        assert_eq!(pin_extract(0xFFFF_0000), None);
    }

    /// rtl_433_tests `secplus/v2/g002_310.01M_250k.json`: id 1959100928 (0x74C58200), button 16,
    /// fixed 70678577664 (0x10_74C5_8200), rolling 240124739 (0xE500343). The same values must
    /// survive KAT's packet build and decode.
    #[test]
    fn rtl_433_fields() {
        let (p1, p2) = encode_62(0x74C5_8200, 0x10, 0xE50_0343);
        let f = fields_62(p1, p2).unwrap();
        assert_eq!(((f.button as u64) << 32 | f.serial as u64), 70_678_577_664);
        assert_eq!(f.counter, 240_124_739);
    }

    #[test]
    fn decodes_the_encoders_upload() {
        let d = key_decode(UT_KEY);
        assert!(d.crc_valid && d.encoder_capable);
        let wire = SecPlusV2Decoder::new()
            .encode(&d, d.button.unwrap())
            .unwrap();
        let mut dec = SecPlusV2Decoder::new();
        let got = feed_all(&mut dec, &wire);
        // Copy 1's packet 1 has no leading gap; copy 2 completes a pair; copy 3 re-arms at its
        // packet 1's gap and its packet 2 finds no packet 1.
        assert_eq!(got.len(), 1);
        let g = &got[0];
        assert_eq!((g.serial, g.button), (d.serial, d.button));
        assert_eq!(g.data_count_bit, 62);
        let f = fields_62(g.extra.unwrap(), g.data).unwrap();
        let f0 = fields_62(d.extra.unwrap(), d.data).unwrap();
        assert_eq!(f.counter, next_counter(f0.counter));
        // Button switch.
        let other = SecPlusV2Decoder::new().encode(&d, 0x68).unwrap();
        let got = feed_all(&mut SecPlusV2Decoder::new(), &other);
        assert_eq!(got[0].button, Some(0x68));
        assert_eq!(got[0].serial, d.serial);
    }

    #[test]
    fn upload_layout_62() {
        let d = key_decode(UT_KEY);
        let wire = SecPlusV2Decoder::new()
            .encode(&d, d.button.unwrap())
            .unwrap();
        let gaps: Vec<usize> = (0..wire.len())
            .filter(|&i| wire[i].duration_us == 68_000)
            .collect();
        assert_eq!(gaps.len(), 6, "2 packets x 3 copies");
        assert_eq!(gaps.last(), Some(&(wire.len() - 1)));
        assert!(wire.windows(2).all(|w| w[0].level != w[1].level));
        assert!(wire
            .iter()
            .all(|p| [250, 500, 68_000].contains(&p.duration_us)));
        // Packet 1 first: its header bits (16 zeros, 1111, 00, id 00) start every copy.
        let packets: Vec<u64> = decode_wire_packets(&wire);
        let (p1, p2) = (packets[0], packets[1]);
        assert_eq!(p1 & PACKET_MASK, PACKET_1);
        assert_eq!(p2 & PACKET_MASK, PACKET_2);
    }

    /// Manchester-decode each packet of an upload (independent of the decoder state machine):
    /// half-bits at 250 µs, starting with the first HIGH after each gap, 62 bits per packet.
    fn decode_wire_packets(wire: &[LevelDuration]) -> Vec<u64> {
        let mut packets = Vec::new();
        let mut halves: Vec<bool> = Vec::new();
        for p in wire {
            if p.duration_us > 1000 {
                halves.resize(124, false);
                let bits = halves
                    .chunks(2)
                    .fold(0u64, |acc, h| acc << 1 | (!h[0] && h[1]) as u64);
                packets.push(bits);
                halves.clear();
            } else {
                halves.extend(std::iter::repeat_n(p.level, (p.duration_us / 250) as usize));
            }
        }
        packets
    }

    #[test]
    fn keypad_86_bit_round_trip() {
        let (serial, button, counter, word) =
            (0x0102_0304u32, 0x70u8, 0xE50_1000u32, pin_apply(0, 1234));
        let (p1, p2) = encode_86(serial, button, counter, word);
        let d = decoded(p1, p2, BITS_86, fields_86(p1, p2).as_ref());
        assert!(d.crc_valid && d.encoder_capable);
        let wire = SecPlusV2Decoder::new().encode(&d, button).unwrap();
        assert_eq!(
            wire.iter().filter(|p| p.duration_us == 91_000).count(),
            8,
            "2 packets x 4 copies"
        );
        let got = feed_all(&mut SecPlusV2Decoder::new(), &wire);
        assert!(!got.is_empty(), "doctor's 86-bit upload decodes");
        for g in &got {
            assert_eq!(g.data_count_bit, 86);
            let f = fields_86(g.extra.unwrap(), g.data).unwrap();
            assert_eq!(
                (f.serial, f.button, f.counter),
                (serial, button, next_counter(counter))
            );
            assert_eq!(pin_extract(f.data_word), Some(1234));
        }
        // Key file: Pin line; a different Pin is applied to the packets.
        let dec = SecPlusV2Decoder::new();
        let text = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(
            text.contains("Bit: 86\n") && text.contains("\nPin: 1234\n"),
            "{text}"
        );
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!((back.data, back.extra), (d.data, d.extra));
        let edited = dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Pin: 1234", "Pin: 0042")).unwrap())
            .unwrap();
        let f = fields_86(edited.extra.unwrap(), edited.data).unwrap();
        assert_eq!(
            (pin_extract(f.data_word), f.counter, f.serial),
            (Some(42), counter, serial)
        );
        // A bad check nibble: not decodable, not sendable.
        let bad = decoded(p1, p2 ^ 1, BITS_86, fields_86(p1, p2 ^ 1).as_ref());
        assert!(!bad.encoder_capable && dec.encode(&bad, button).is_none());
    }

    #[test]
    fn rejects_undecodable_and_unpaired_packets() {
        let d = key_decode(UT_KEY);
        // Packet 2 with an invalid invert code (0xF): the C reports it with zero fields; KAT not.
        let bad_p2 = d.data | 0xF << 30;
        let mut wire = Vec::new();
        push(&mut wire, false, 65_000);
        for p in [d.extra.unwrap(), bad_p2] {
            push_packet(&mut wire, p, 62, None, GAP_62);
        }
        assert!(feed_all(&mut SecPlusV2Decoder::new(), &wire).is_empty());
        // The same frame with the valid packet 2 decodes.
        let mut good = Vec::new();
        push(&mut good, false, 65_000);
        for p in [d.extra.unwrap(), d.data] {
            push_packet(&mut good, p, 62, None, GAP_62);
        }
        assert_eq!(feed_all(&mut SecPlusV2Decoder::new(), &good).len(), 1);
        // Packet 2 alone, or out-of-tolerance pulses, never complete a pair.
        let mut lone = Vec::new();
        push(&mut lone, false, 65_000);
        push_packet(&mut lone, d.data, 62, None, GAP_62);
        assert!(feed_all(&mut SecPlusV2Decoder::new(), &lone).is_empty());
        let stretched: Vec<LevelDuration> = good
            .iter()
            .map(|p| {
                LevelDuration::new(
                    p.level,
                    if p.duration_us == 500 {
                        640
                    } else {
                        p.duration_us
                    },
                )
            })
            .collect();
        assert!(feed_all(&mut SecPlusV2Decoder::new(), &stretched).is_empty());
        // A key file without Secplus_packet_1 or with a wrong Bit does not load.
        let dec = SecPlusV2Decoder::new();
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&UT_KEY.replace("Secplus_packet_1", "X")).unwrap())
            .is_none());
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&UT_KEY.replace("Bit: 62", "Bit: 64")).unwrap())
            .is_none());
    }

    #[test]
    fn key_file_round_trip() {
        let dec = SecPlusV2Decoder::new();
        let d = key_decode(UT_KEY);
        assert_eq!(
            (d.data, d.extra),
            (0x3D29_0FB9_BEEE, Some(0x3C01_6B19_DD60))
        );
        let text = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(
            text.contains("Protocol: Security+ 2.0\nBit: 62\nKey: 00 00 3D 29 0F B9 BE EE\nSecplus_packet_1: 00 00 3C 01 6B 19 DD 60\n"),
            "{text}"
        );
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(
            (
                back.data,
                back.extra,
                back.serial,
                back.button,
                back.counter,
                back.crc_valid,
                back.encoder_capable
            ),
            (
                d.data,
                d.extra,
                d.serial,
                d.button,
                d.counter,
                d.crc_valid,
                d.encoder_capable
            )
        );
        let reg = ProtocolRegistry::new();
        assert_eq!(
            reg.decode_key(&FlipperKey::parse(UT_KEY).unwrap())
                .map(|(n, _)| n),
            Some(NAME.to_string())
        );
    }

    /// Flipper's unit test: `security_pls_2_0_raw.sub` must decode (skipped without REFERENCES).
    #[test]
    fn decodes_flipper_unit_test_capture() {
        let path = std::path::Path::new(
            "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz/security_pls_2_0_raw.sub",
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
        assert!(
            !feed_all(&mut SecPlusV2Decoder::new(), &pairs).is_empty(),
            "no Security+ 2.0 frame"
        );
        let mut reg = ProtocolRegistry::new();
        let hits = reg.process_signal_stream(&pairs, freq);
        assert!(
            hits.iter().any(|(n, _, _)| n == NAME),
            "{:?}",
            hits.iter().map(|h| &h.0).collect::<Vec<_>>()
        );
    }
}
