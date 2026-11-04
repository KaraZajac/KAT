//! Nice FloR-S / Nice One decoder + encoder (rolling code, Nice table cipher).
//!
//! Primary reference: Flipper-ARF's garage app,
//! `applications/system/subghz_garage/protocols/nice_flor_s.c` (= ARF's
//! `lib/subghz/protocols/nice_flor_s.c` without the custom-button bookkeeping). Union with
//! doctor-firmware's `lib/subghz/protocols/nice_flor_s.c`, which adds **Nice O-Code**: a key file
//! carrying an `IC` (installer code) line is decrypted and re-encoded with a per-parcel mask
//! (doctor's `nice_o_code` app finds the IC by brute force; KAT only uses a given one).
//! Wire-level difference: doctor's receiver reports a 52-bit frame only when the frame before it
//! had the same low 32 bits (its "skip O-Code" filter: the first parcel of a press only primes it,
//! the receiver's reset after a decode clears it, so it reports every second parcel; O-Code
//! frames, whose cipher text changes per parcel, never pass). The garage copy, followed here,
//! reports every frame (oracle: 998 vs doctor's 719 decodes on the corpus).
//!
//! **Air format** (AM, 433/868 MHz): PWM with te 500/1000 µs (te_delta 300). A parcel is a
//! ~19 ms LOW gap (te_short × 38 ± te_delta × 38; the encoder sends × 37), a 1500 µs HIGH +
//! 1500 µs LOW start, 52 data bits MSB first (1 = HIGH 1000 + LOW 500, 0 = HIGH 500 + LOW 1000)
//! and a 1500 µs HIGH stop bit (± te_delta). Nice One appends 20 bits before the stop bit
//! (72 bits). A press sends 16 parcels.
//!
//! **Frame** (52 bits): `btn4 ‖ P1 4 ‖ enc44`. `btn` is the button (1, 2, 4, 8; 3 on some
//! remotes), `P1 = 0xF ^ btn ^ n` numbers the parcel (n = 0..15) and `enc` encrypts
//! `serial28 ‖ cnt16` with the table cipher (two rounds of table lookups XORed over bytes 1..5,
//! a byte swap and an inversion; see [decrypt_ic]). Nice One's 20 extra bits are
//! `k8 ‖ (hold ^ k << 4) 4 ‖ crc8`, `k = n ^ (n < 4 ? 0x8F : 0x80)`, `hold` = 0 on the first
//! parcel; the CRC-8 (reflected poly 0x97, init 0xFF) covers the 64 bits before it.
//!
//! **Fields need the table** (see `g6_common.rs`): without `~/.config/KAT/nice_flor_s.bin` the C
//! reports the frame with serial/button/counter 0 (KAT: `None`) and KAT does not transmit (the C
//! would send `enc` = 0). With it: `serial`, `button`, `counter` as the C decrypts them.
//!
//! **KAT packing**: 52-bit frames: `data` = the frame. 72-bit frames: `data` = the first 64 wire
//! bits (`frame52 << 12 | tail20 >> 8`), `extra` bits 0..7 = the last 8 wire bits (= the Nice One
//! CRC). `extra` bits 8..15 = `CounterMode` (key files), bits 16..31 = O-Code `IC`, bit 32 = O-Code
//! flag; `extra` is None for a plain 52-bit frame. `crc_valid` = the Nice One CRC holds (52-bit
//! frames have no check; the C does not check the CRC either and reports every frame).
//!
//! **Encoder** (`subghz_protocol_encoder_nice_flor_s_deserialize` + `get_upload`): decrypt the
//! key (serial, counter, button), counter + 1 (`CounterMode` 1: 0xFFFE ↔ 1, ≥ 2: 1 ↔ 0), button
//! = the requested native nibble (1..=0xF; else the captured one), then 16 parcels n = 0..15 with
//! `P1 = 0xF ^ btn ^ n` (Nice One: tail for n with hold = n), each `LOW 18500, HIGH 1500,
//! LOW 1500, bits, HIGH 1500`; one copy (`encoder.repeat` = 1). O-Code keys re-encrypt every
//! parcel with its mask (doctor).
//! The garage app's TX plugins are built without the app's custom-button and firmware-extension
//! flags, so their compat shims make this encoder send button 0 and the "OFEX" counter (0 → 1 →
//! 0xFFFE → 0xFFFF → 0) whatever was captured (oracle-confirmed); KAT does not copy that build
//! artefact and sends the captured or requested button with counter + 1, as doctor-firmware does.

use super::g6_common::{self, push, Table};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;
use std::sync::OnceLock;

use super::info::prelude::*;

/// Metadata (see `info.rs`): SubGhzProtocolFlag_AM | 433 | 868.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Nice FloR-S",
    decoder: "Nice FloR-S",
    // The C's display names: 72-bit frames show as "Nice One", doctor's O-Code keys as
    // "Nice O-Code" (both are Nice FloR-S key files).
    aliases: &["Nice One", "Nice O-Code"],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Nice",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The button nibble (`P0`: 1, 2, 4, 8 = buttons 1..4; the references also offer 3).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x1 => "Button 1",
        0x2 => "Button 2",
        0x4 => "Button 3",
        0x8 => "Button 4",
        0x3 => "Buttons 1+2",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1000;
const TE_DELTA: u32 = 300;
const MIN_COUNT_BIT: u8 = 52;
const NICE_ONE_COUNT_BIT: u8 = 72;
/// The installer code of a plain (non O-Code) remote (`NICE_FLOR_S_IC_PLAIN`).
const IC_PLAIN: u16 = 0xFFFF;
const PARCELS: u8 = 16;
const MASK_52: u64 = (1 << 52) - 1;
/// `extra` bit 32: the decode carries an O-Code IC (bits 16..31).
const EXTRA_O_CODE: u64 = 1 << 32;

/// The plaintext table from `~/.config/KAT/nice_flor_s.bin`, read once.
fn config_table() -> Option<Table> {
    static TABLE: OnceLock<Option<Table>> = OnceLock::new();
    *TABLE.get_or_init(|| g6_common::read_config_table("nice_flor_s.bin"))
}

/// `subghz_protocol_decoder_nice_flor_s_magic_xor`: bytes 1..5 ^= k.
fn magic_xor(p: &mut [u8; 8], k: u8) {
    for b in &mut p[1..6] {
        *b ^= k;
    }
}

/// doctor's `subghz_protocol_nice_flor_s_decrypt_ic_buffer` (with `ic` = 0xFFFF it is the garage
/// copy's `subghz_protocol_nice_flor_s_decrypt`). `p` is the little-endian byte view of `data`.
fn decrypt_ic(data: u64, ic: u16, table: &Table) -> u64 {
    let mut p = data.to_le_bytes();
    let mut k = p[4] ^ (ic >> 8) as u8;
    p[5] = !p[5];
    p[4] = !p[2];
    p[2] = !p[0];
    p[0] = k;
    k = p[3] ^ ic as u8;
    p[3] = !p[1];
    p[1] = k;
    for y in 0..2 {
        k = table[(p[0] >> 3) as usize].wrapping_add(0x25);
        magic_xor(&mut p, k);
        p[5] &= 0x0F;
        p[0] ^= k & 0x7;
        k = table[(p[0] & 0x1F) as usize];
        magic_xor(&mut p, k);
        p[5] &= 0x0F;
        p[0] ^= k & 0xE0;
        if y == 0 {
            p.swap(0, 1);
        }
    }
    u64::from_le_bytes(p)
}

/// doctor's `subghz_protocol_nice_flor_s_encrypt_ic` (`ic` = 0xFFFF: the garage copy's
/// `subghz_protocol_nice_flor_s_encrypt`).
fn encrypt_ic(data: u64, ic: u16, table: &Table) -> u64 {
    let mut p = data.to_le_bytes();
    for y in 0..2 {
        let mut k = table[(p[0] & 0x1F) as usize];
        magic_xor(&mut p, k);
        p[5] &= 0x0F;
        p[0] ^= k & 0xE0;
        k = table[(p[0] >> 3) as usize].wrapping_add(0x25);
        magic_xor(&mut p, k);
        p[5] &= 0x0F;
        p[0] ^= k & 0x7;
        if y == 0 {
            p.swap(0, 1);
        }
    }
    p[5] = !p[5] & 0x0F;
    let mut k = !p[4];
    p[4] = p[0] ^ (ic >> 8) as u8;
    p[0] = !p[2];
    p[2] = k;
    k = p[1] ^ ic as u8;
    p[1] = !p[3];
    p[3] = k;
    u64::from_le_bytes(p)
}

/// doctor's `subghz_protocol_nice_o_mask`: the O-Code serial mask (28 bits) and IC mask of parcel
/// `n`.
fn nice_o_mask(n: u8) -> (u32, u16) {
    let n = n as u32;
    let a = 0x0Eu32.wrapping_sub(n) & 0x0F;
    let b = n + 2;
    let odd = n & 1;
    let c = b + (odd << 1);
    let bn = b & 0x0F;
    let cn = c & 0x0F;
    let mut m = (a << 8) | (bn << 4) | (bn << 20) | (a << 24) | cn | (cn << 16);
    m |= ((((b ^ 0x0Eu32.wrapping_sub(n) ^ c) & 0x0F) + (odd << 2)) << 12) & 0xFFFF;
    (m & 0x0FFF_FFFF, (((m & 0xFF) << 8) | ((m >> 8) & 0xFF)) as u16)
}

/// doctor's `subghz_protocol_nice_o_get_parcel`: the parcel number from `btn` and `P1`.
fn nice_o_parcel(data: u64) -> u8 {
    ((((data >> 48) & 0x0F) ^ ((data >> 44) & 0x0F)) ^ 0x0F) as u8
}

/// `subghz_protocol_nice_one_crc` over the 64 bits it covers (bits 4..67 of the byte array
/// `P0..P9`, i.e. the first 64 wire bits of a Nice One frame), MSB first.
fn nice_one_crc(wire64: u64) -> u8 {
    let mut crc_data: u8 = 0xFF;
    for i in (0..64).rev() {
        let crc = if (wire64 >> i) & 1 != 0 { crc_data ^ 1 } else { crc_data };
        crc_data >>= 1;
        if crc & 0x01 != 0 {
            crc_data ^= 0x97;
        }
    }
    crc_data.reverse_bits()
}

/// `subghz_protocol_nice_one_get_data` for a 52-bit parcel: the 24-bit `data_2` (`P7 P8 P9`) of
/// parcel `num_parcel` (the encoder passes `hold_bit` = `num_parcel`); the upload sends bits
/// 23..4.
fn nice_one_tail(frame: u64, num_parcel: u8, hold_bit: u8) -> u32 {
    let mut p = [0u8; 10];
    for (i, b) in p.iter_mut().take(7).enumerate() {
        *b = (frame >> (48 - i * 8)) as u8;
    }
    p[1] = (p[1] & 0x0F) | ((0x0F ^ (p[0] & 0x0F) ^ num_parcel) << 4);
    let mut k: u8 = if num_parcel < 4 { 0x8F } else { 0x80 };
    let hold: u8 = if hold_bit != 0 { 0x10 } else { 0 };
    k ^= num_parcel;
    p[7] = k;
    p[8] = hold ^ (k << 4);
    let wire64 = (u64::from(p[0] & 0x0F) << 60)
        | (u64::from_be_bytes([p[1], p[2], p[3], p[4], p[5], p[6], p[7], 0]) >> 4)
        | u64::from(p[8] >> 4);
    let crc = nice_one_crc(wire64);
    p[8] |= crc >> 4;
    p[9] = crc << 4;
    (u32::from(p[7]) << 16) | (u32::from(p[8]) << 8) | u32::from(p[9])
}

/// One frame as the C's `generic` holds it: the 52-bit `data`, Nice One's 20-bit `data_2`,
/// the bit count, and the key file's `CounterMode` / doctor `IC`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Frame {
    key: u64,
    tail: u32,
    bits: u8,
    counter_mode: u8,
    ic: Option<u16>,
}

/// Fields a table recovers (`remote_controller`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Fields {
    serial: u32,
    counter: u16,
    button: u8,
}

impl Frame {
    /// Unpack a [DecodedSignal] (see the module doc).
    fn from_decoded(d: &DecodedSignal) -> Option<Frame> {
        let extra = d.extra.unwrap_or(0);
        let (key, tail) = match d.data_count_bit {
            52 => (d.data, 0),
            72 => (d.data >> 12, (((d.data & 0xFFF) << 8) | (extra & 0xFF)) as u32),
            _ => return None,
        };
        Some(Frame {
            key,
            tail,
            bits: d.data_count_bit as u8,
            counter_mode: (extra >> 8) as u8,
            ic: (extra & EXTRA_O_CODE != 0).then_some((extra >> 16) as u16),
        })
    }

    /// The first 64 wire bits of a Nice One frame.
    fn wire64(&self) -> u64 {
        ((self.key & MASK_52) << 12) | u64::from(self.tail >> 8)
    }

    /// `subghz_protocol_nice_flor_s_remote_controller` (+ doctor's O-Code branch); None without
    /// a table (the C shows 0s).
    fn fields(&self, table: Option<&Table>) -> Option<Fields> {
        let table = table?;
        Some(match self.ic {
            None => {
                let dec = decrypt_ic(self.key, IC_PLAIN, table);
                Fields {
                    serial: ((dec >> 16) & 0x0FFF_FFFF) as u32,
                    counter: dec as u16,
                    button: ((dec >> 48) & 0x0F) as u8,
                }
            }
            Some(ic) => {
                let (mask_32, mask_16) = nice_o_mask(nice_o_parcel(self.key));
                let dec = decrypt_ic(self.key, ic ^ mask_16, table);
                Fields {
                    serial: (((dec >> 16) ^ u64::from(mask_32)) as u32) & 0x0FFF_FFFF,
                    counter: dec as u16,
                    button: ((self.key >> 48) & 0x0F) as u8,
                }
            }
        })
    }

    /// The decode KAT reports (what the C's `get_string` shows).
    fn to_decoded(self, table: Option<&Table>) -> DecodedSignal {
        let fields = self.fields(table);
        let mut extra = (u64::from(self.counter_mode) << 8)
            | self.ic.map_or(0, |ic| EXTRA_O_CODE | (u64::from(ic) << 16));
        let (data, crc_valid) = if self.bits == NICE_ONE_COUNT_BIT {
            extra |= u64::from(self.tail & 0xFF);
            (self.wire64(), nice_one_crc(self.wire64()) == (self.tail & 0xFF) as u8)
        } else {
            (self.key, false)
        };
        let display = if self.bits == NICE_ONE_COUNT_BIT {
            Some("Nice One")
        } else if self.ic.is_some() {
            Some("Nice O-Code")
        } else {
            None
        };
        DecodedSignal {
            serial: fields.map(|f| f.serial),
            button: fields.map(|f| f.button),
            counter: fields.map(|f| f.counter),
            crc_valid,
            data,
            data_count_bit: self.bits as usize,
            encoder_capable: fields.is_some(),
            extra: (self.bits == NICE_ONE_COUNT_BIT || extra != 0).then_some(extra),
            protocol_display_name: display.map(str::to_string),
        }
    }

    /// The counter the encoder sends (`get_upload`'s counter modes, multiplier 1).
    fn next_counter(&self, cnt: u16) -> u16 {
        let cnt = u32::from(cnt);
        (match self.counter_mode {
            0 => g6_common::counter_increment(cnt),
            1 => {
                if cnt == 0xFFFE {
                    0x0001
                } else {
                    0xFFFE
                }
            }
            _ => {
                if cnt >= 0x0001 {
                    0
                } else {
                    cnt + 1
                }
            }
        }) as u16
    }

    /// `subghz_protocol_encoder_nice_flor_s_get_upload` after deserialize: 16 parcels, with
    /// `encrypt(plain, ic)` as the cipher.
    fn upload(&self, encrypt: impl Fn(u64, u16) -> u64, fields: Fields, button: u8) -> Vec<LevelDuration> {
        let btn = if (1..=0x0F).contains(&button) { button } else { fields.button };
        let cnt = self.next_counter(fields.counter);
        let plain = encrypt((u64::from(fields.serial) << 16) | u64::from(cnt), IC_PLAIN);
        let mut out = Vec::with_capacity(PARCELS as usize * 148);
        for n in 0..PARCELS {
            let enc = match self.ic {
                None => plain,
                Some(ic) => {
                    let (mask_32, mask_16) = nice_o_mask(n);
                    let o = ((u64::from(fields.serial ^ mask_32) & 0x0FFF_FFFF) << 16) | u64::from(cnt);
                    encrypt(o, ic ^ mask_16)
                }
            };
            let byte = ((u32::from(btn) << 4) | (0x0F ^ u32::from(btn) ^ u32::from(n))) as u8;
            let frame = (u64::from(byte) << 44) | enc;
            push(&mut out, false, TE_SHORT * 37);
            push(&mut out, true, TE_SHORT * 3);
            push(&mut out, false, TE_SHORT * 3);
            push_bits(&mut out, frame, 52);
            if self.bits == NICE_ONE_COUNT_BIT {
                push_bits(&mut out, u64::from(nice_one_tail(frame, n, n) >> 4), 20);
            }
            push(&mut out, true, TE_SHORT * 3);
        }
        out
    }
}

/// The upload the C builds when the table file is missing (for oracle comparisons): the fields
/// are 0, the cipher returns 0 (`SUBGHZ_NO_NICE_FLOR_S_RAINBOW_TABLE`) and the button is 0.
#[cfg(test)]
pub(super) fn c_upload_without_table(decoded: &DecodedSignal) -> Option<Vec<LevelDuration>> {
    let frame = Frame::from_decoded(decoded)?;
    Some(frame.upload(|_, _| 0, Fields { serial: 0, counter: 0, button: 0 }, 0))
}

/// `subghz_protocol_nice_flor_s_create_data` (the SubGHz app's "Add Manually"): `serial << 16 |
/// cnt` (as given) encrypted with the table, the button byte `btn << 4 | (0xF ^ btn ^ 3)`; Nice
/// One (72 bits) adds `nice_one_get_data(…, 0, 0)` as `Data` (the 20 bits it saves, decimal).
/// `Bit: 52` or `72`, `Key`, [`Data`]. Needs the table.
pub(super) fn create_data(serial: u32, btn: u8, cnt: u16, nice_one: bool, table: &Table) -> FlipperKey {
    let enc = encrypt_ic((u64::from(serial) << 16) | u64::from(cnt), IC_PLAIN, table);
    let byte = (btn << 4) | (0x0F ^ btn ^ 0x3);
    let data = (u64::from(byte) << 44) | enc;
    let bits = if nice_one { NICE_ONE_COUNT_BIT } else { MIN_COUNT_BIT };
    let mut key = FlipperKey::for_protocol("Nice FloR-S", bits as usize, data);
    if nice_one {
        key.push("Data", ((nice_one_tail(data, 0, 0) >> 4) & 0xF_FFFF).to_string());
    }
    key
}

/// The user's table (`~/.config/KAT/nice_flor_s.bin`), if present.
pub(super) fn user_table() -> Option<Table> {
    config_table()
}

/// The low `count` bits of `value`, MSB first, as PWM pairs.
fn push_bits(out: &mut Vec<LevelDuration>, value: u64, count: u32) {
    for i in (0..count).rev() {
        if (value >> i) & 1 != 0 {
            push(out, true, TE_LONG);
            push(out, false, TE_SHORT);
        } else {
            push(out, true, TE_SHORT);
            push(out, false, TE_LONG);
        }
    }
}

/// Decoder steps (`NiceFlorSDecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    CheckHeader,
    FoundHeader,
    SaveDuration,
    CheckDuration,
}

/// Nice FloR-S decoder (`SubGhzProtocolDecoderNiceFlorS`).
pub struct NiceFlorSDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    /// `SubGhzBlockDecoder.decode_count_bit` is a `uint8_t`.
    decode_count_bit: u8,
    /// `generic.data_2`: the first 52 bits once bit 52 is in; at the stop bit Nice One's tail.
    data_2: u64,
    table: Option<Table>,
}

impl NiceFlorSDecoder {
    /// A decoder with the user's table, if `~/.config/KAT/nice_flor_s.bin` exists.
    pub fn new() -> Self {
        Self::with_table(config_table())
    }

    /// A decoder with the given plaintext table (None: as a Flipper without the asset file).
    pub(super) fn with_table(table: Option<Table>) -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            data_2: 0,
            table,
        }
    }

    /// `subghz_protocol_blocks_add_bit`.
    fn add_bit(&mut self, bit: u64) {
        self.decode_data = (self.decode_data << 1) | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for NiceFlorSDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for NiceFlorSDecoder {
    fn name(&self) -> &'static str {
        "Nice FloR-S"
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

    /// The garage copy's reset: the parser step only (`generic` survives, as in the C).
    fn reset(&mut self) {
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_SHORT * 38) < TE_DELTA * 38 {
                    self.step = Step::CheckHeader;
                }
            }
            Step::CheckHeader => {
                self.step = if level && duration_diff!(duration, TE_SHORT * 3) < TE_DELTA * 3 {
                    Step::FoundHeader
                } else {
                    Step::Reset
                };
            }
            Step::FoundHeader => {
                if !level && duration_diff!(duration, TE_SHORT * 3) < TE_DELTA * 3 {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if level {
                    if duration_diff!(duration, TE_SHORT * 3) < TE_DELTA {
                        // Stop bit.
                        self.step = Step::Reset;
                        let bits = self.decode_count_bit;
                        if bits == MIN_COUNT_BIT || bits == NICE_ONE_COUNT_BIT {
                            // generic.data = data_2; data_2 = decode_data; decode_data = data.
                            std::mem::swap(&mut self.data_2, &mut self.decode_data);
                            let frame = Frame {
                                key: self.decode_data,
                                tail: (self.data_2 & 0xF_FFFF) as u32,
                                bits,
                                counter_mode: 0,
                                ic: None,
                            };
                            return Some(frame.to_decoded(self.table.as_ref()));
                        }
                    } else {
                        self.te_last = duration;
                        self.step = Step::CheckDuration;
                    }
                }
            }
            Step::CheckDuration => {
                if !level {
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA
                    {
                        self.add_bit(0);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(1);
                        self.step = Step::SaveDuration;
                    } else {
                        self.step = Step::Reset;
                    }
                } else {
                    self.step = Step::Reset;
                }
                if self.decode_count_bit == MIN_COUNT_BIT {
                    self.data_2 = self.decode_data;
                    self.decode_data = 0;
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        self.table.is_some()
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let table = self.table.as_ref()?;
        let frame = Frame::from_decoded(decoded)?;
        let fields = frame.fields(Some(table))?;
        Some(frame.upload(|plain, ic| encrypt_ic(plain, ic, table), fields, button))
    }

    /// The garage decoder's deserialize (`Bit` 52 or 72, `Data` for 72, `CounterMode`) + doctor's
    /// `IC`, then the fields as `get_string` shows them.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let bits = match key.bits {
            52 => MIN_COUNT_BIT,
            72 => NICE_ONE_COUNT_BIT,
            _ => return None,
        };
        let tail = if bits == NICE_ONE_COUNT_BIT { key.u32_field("Data")? & 0xF_FFFF } else { 0 };
        let ic = match key.field("IC") {
            Some(v) => match super::flipper_key::parse_hex_vec(v)?.as_slice() {
                [hi, lo] => Some(u16::from_be_bytes([*hi, *lo])),
                _ => return None,
            },
            None => None,
        };
        let frame = Frame { key: key.key, tail, bits, counter_mode: g6_common::counter_mode(key), ic };
        Some(frame.to_decoded(self.table.as_ref()))
    }

    /// The garage serialize: `Protocol`, `Bit`, `Key` (the 52-bit frame), Nice One's `Data` (the
    /// 20-bit tail), then doctor's `IC` and the `CounterMode` the key came with.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let frame = Frame::from_decoded(decoded)?;
        let mut key = FlipperKey::new(
            "Nice FloR-S",
            frame.bits as usize,
            frame.key,
            433_920_000,
            "FuriHalSubGhzPresetOok650Async",
        );
        if frame.bits == NICE_ONE_COUNT_BIT {
            key.push("Data", frame.tail.to_string());
        }
        if let Some(ic) = frame.ic {
            key.push_hex("IC", u64::from(ic), 2);
        }
        if frame.counter_mode != 0 {
            key.push("CounterMode", frame.counter_mode.to_string());
        }
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::g6_common::{decode_all, tuples, unit_test_capture};
    use crate::protocols::ProtocolRegistry;

    /// A synthetic table (the real one is secret): any 32 bytes make a valid cipher.
    const TABLE: Table = [
        0x3A, 0x91, 0x5C, 0xE7, 0x08, 0x6D, 0xB2, 0x4F, 0x17, 0xC8, 0x2E, 0x99, 0x70, 0x05, 0xDB, 0x63,
        0xA4, 0x1F, 0x86, 0x3C, 0xF0, 0x58, 0x2B, 0xCE, 0x94, 0x47, 0x0E, 0xB9, 0x61, 0xD5, 0x7A, 0x13,
    ];

    fn frame_with(serial: u32, counter: u16, button: u8, parcel: u8, ic: Option<u16>) -> u64 {
        let enc = match ic {
            None => encrypt_ic((u64::from(serial) << 16) | u64::from(counter), IC_PLAIN, &TABLE),
            Some(ic) => {
                let (m32, m16) = nice_o_mask(parcel);
                encrypt_ic((u64::from((serial ^ m32) & 0x0FFF_FFFF) << 16) | u64::from(counter), ic ^ m16, &TABLE)
            }
        };
        let byte = (button << 4) | (0x0F ^ button ^ parcel);
        (u64::from(byte) << 44) | enc
    }

    fn signal(frame: Frame, table: Option<&Table>) -> DecodedSignal {
        frame.to_decoded(table)
    }

    /// Golden vectors from the C: doctor-firmware's cipher functions (`encrypt_ic`,
    /// `decrypt_ic_buffer`, `nice_o_mask`, `nice_one_get_data`) extracted verbatim and compiled on
    /// the host with [TABLE] in place of the keystore read.
    #[rustfmt::skip]
    const C_CIPHER: &[(u64, u16, u64, u64, u64)] = &[ // (plain, ic, encrypt, decrypt(x), x)
        (0x002BC585C9CE6, 0xFFFF, 0x00E6E90E00400, 0x00050E24985DBAE3, 0x5B9B99B03BF7A),
        (0x000917DBAAF99, 0x0000, 0x00C17755DB176, 0x000A09260F50B59C, 0xA8B88C8B49DC2),
        (0x00912BB4B0750, 0x5A3C, 0x002001409A050, 0x00050FC76B099DF9, 0x576B1B4BE1270),
        (0x00AF643AD8ABC, 0xBEEF, 0x002CB6DEE5BB5, 0x00030686C52FB00E, 0x32DFD56BDFE14),
        (0x000B41B77FB5A, 0xFFFF, 0x00A30560EA1CD, 0x00040A0DA8BCCA8C, 0x4109ABED77266),
        (0x002CB41A15BC5, 0x0000, 0x00BC2FE129878, 0x00030C9A0FAC0E85, 0x31C84472ABF1C),
        (0x008BB81271C3A, 0x5A3C, 0x0069D09350FA9, 0x000805CC927CDC9C, 0x838B92B81DF31),
        (0x002C6232EAF48, 0xBEEF, 0x00EAA7B1AFFF2, 0x0006082F0111260A, 0x6C425A8032D3D),
    ];
    #[rustfmt::skip]
    const C_MASKS: [(u32, u16); 16] = [
        (0xE22EE22, 0x22EE),
        (0xD35FD35, 0x35FD),
        (0xC44CC44, 0x44CC),
        (0xB57DB57, 0x57DB),
        (0xA66AA66, 0x66AA),
        (0x979B979, 0x79B9),
        (0x8888888, 0x8888),
        (0x79B979B, 0x9B97),
        (0x6AA66AA, 0xAA66),
        (0x5BD75BD, 0xBD75),
        (0x4CC44CC, 0xCC44),
        (0x3DF53DF, 0xDF53),
        (0x2EE22EE, 0xEE22),
        (0x1F131F1, 0xF131),
        (0x0000000, 0x0000),
        (0xF131F13, 0x131F),
    ];
    #[rustfmt::skip]
    const C_TAILS: &[(u64, u8, u8, u32)] = &[ // (frame, parcel, hold, P7 P8 P9)
        (0x5A71338906A10, 4, 0, 0x844660),
        (0x1B88CB65B5D9E, 14, 14, 0x8EF010),
        (0x2947B2516609C, 12, 0, 0x8CC710),
        (0x633A84237E267, 8, 8, 0x889930),
        (0x0A0CD9ABE4E5C, 10, 0, 0x8AA550),
        (0xE6EEA2FD80696, 7, 7, 0x876DB0),
    ];

    #[test]
    fn matches_the_c_golden_vectors() {
        for &(plain, ic, enc, dec, x) in C_CIPHER {
            assert_eq!(encrypt_ic(plain, ic, &TABLE), enc, "encrypt {plain:X} ic {ic:04X}");
            assert_eq!(decrypt_ic(x, ic, &TABLE), dec, "decrypt {x:X} ic {ic:04X}");
        }
        for (n, mask) in C_MASKS.iter().enumerate() {
            assert_eq!(nice_o_mask(n as u8), *mask, "parcel {n}");
        }
        for &(frame, n, hold, tail) in C_TAILS {
            assert_eq!(nice_one_tail(frame, n, hold), tail, "{frame:X}");
        }
    }

    #[test]
    fn cipher_round_trips_for_every_ic() {
        for (serial, counter) in [(0x0436_C682u32 & 0x0FFF_FFFF, 0x0444u16), (0x0FFF_FFFF, 0xFFFF), (0, 0), (0x0123_4567, 0x8000)] {
            for ic in [IC_PLAIN, 0x0000, 0x1234, 0xBEEF] {
                let plain = (u64::from(serial) << 16) | u64::from(counter);
                let enc = encrypt_ic(plain, ic, &TABLE);
                assert_eq!(enc >> 44, 0, "the cipher text is 44 bits");
                assert_eq!(decrypt_ic(enc, ic, &TABLE), plain, "serial {serial:07X} cnt {counter:04X} ic {ic:04X}");
            }
        }
    }

    /// The C's own example: `data 0x1c5783607f7b3 => decrypt 0x10436c6820444` (button 1, serial
    /// 0x0436C682, counter 0x0444) holds with the button/P1 byte untouched by the cipher.
    #[test]
    fn decrypt_keeps_the_button_byte() {
        let frame = frame_with(0x0436_C682 & 0x0FFF_FFFF, 0x0444, 0x1, 3, None);
        let dec = decrypt_ic(frame, IC_PLAIN, &TABLE);
        assert_eq!(dec >> 48, 0x1);
        assert_eq!((dec >> 16) & 0x0FFF_FFFF, 0x0436_C682 & 0x0FFF_FFFF);
        assert_eq!(dec & 0xFFFF, 0x0444);
    }

    /// The C's Nice One examples from a real remote (comment in `remote_controller`):
    /// `0x1724A7D9A522F 899 D6` (parcel 9, hold bit 0: button just pressed) and
    /// `0x1424A7D9A522F 8AB 03` (parcel 0xA, hold bit 1): the tail is `nice_one_get_data` of the
    /// frame and its last byte the CRC of the 64 bits before it.
    #[test]
    fn nice_one_tail_matches_the_c_examples() {
        let first = 0x1724A7D9A522Fu64;
        assert_eq!(nice_o_parcel(first), 9);
        assert_eq!(nice_one_tail(first, 9, 0) >> 4, 0x899D6);
        let held = 0x1424A7D9A522Fu64;
        assert_eq!(nice_o_parcel(held), 0xA);
        assert_eq!(nice_one_tail(held, 0xA, 1) >> 4, 0x8AB03);
        for (frame, tail) in [(first, 0x899D6u32), (held, 0x8AB03)] {
            let f = Frame { key: frame, tail, bits: 72, counter_mode: 0, ic: None };
            assert!(signal(f, None).crc_valid);
            let bad = Frame { tail: tail ^ 0x100, ..f };
            assert!(!signal(bad, None).crc_valid);
        }
    }

    #[test]
    fn o_code_masks_are_distinct_per_parcel() {
        let masks: std::collections::HashSet<(u32, u16)> = (0..16).map(nice_o_mask).collect();
        assert_eq!(masks.len(), 16);
        assert_eq!(nice_o_parcel(frame_with(1, 2, 0x4, 9, None)), 9);
    }

    /// encode → decode with the synthetic table: every parcel decodes back with the counter + 1
    /// and the requested (or captured) button, for 52/72 bits, plain and O-Code, every counter mode.
    #[test]
    fn encode_decode_round_trip_with_a_table() {
        for bits in [52u8, 72] {
            for ic in [None, Some(0x5A3C)] {
                for (mode, from, to) in [(0u8, 0x1233u16, 0x1234u16), (0, 0xFFFF, 0), (1, 0x0010, 0xFFFE), (1, 0xFFFE, 1), (2, 5, 0), (2, 0, 1)] {
                    for (button, want) in [(0u8, 0x2u8), (0x2, 0x2), (0x8, 0x8), (0x10, 0x2)] {
                        let key = frame_with(0x0ABC_DEF1 & 0x0FFF_FFFF, from, 0x2, 0, ic);
                        let tail = if bits == 72 { nice_one_tail(key, 0, 0) >> 4 } else { 0 };
                        let src = signal(Frame { key, tail, bits, counter_mode: mode, ic }, Some(&TABLE));
                        assert!(src.encoder_capable);
                        let dec = NiceFlorSDecoder::with_table(Some(TABLE));
                        let wire = dec.encode(&src, button).expect("encode");
                        let mut rx = NiceFlorSDecoder::with_table(Some(TABLE));
                        // The registry resets after each decode, and the 16 parcels decode one by one.
                        let got = decode_all(&mut rx, &wire);
                        assert_eq!(got.len(), PARCELS as usize, "bits {bits} ic {ic:?}");
                        for (n, d) in got.iter().enumerate() {
                            let f = Frame::from_decoded(d).unwrap();
                            assert_eq!(d.data_count_bit, bits as usize);
                            assert_eq!(nice_o_parcel(f.key), n as u8);
                            if bits == 72 {
                                assert!(d.crc_valid, "parcel {n}");
                                assert_eq!(d.protocol_display_name.as_deref(), Some("Nice One"));
                            }
                            if ic.is_none() {
                                assert_eq!((d.serial, d.counter, d.button), (Some(0x0ABC_DEF1 & 0x0FFF_FFFF), Some(to), Some(want)));
                            } else {
                                // The decoder sees O-Code frames as plain FloR-S; the key file's IC decrypts them.
                                let f = Frame { ic, ..f };
                                let fields = f.fields(Some(&TABLE)).unwrap();
                                assert_eq!((fields.serial, fields.counter, fields.button), (0x0ABC_DEF1 & 0x0FFF_FFFF, to, want));
                            }
                        }
                    }
                }
            }
        }
    }

    /// Without a table the frames still decode (fields unknown, not transmittable), as on a
    /// Flipper without the asset file.
    #[test]
    fn no_table_decodes_without_fields_and_does_not_encode() {
        let key = frame_with(0x0123_4567, 0x0042, 0x4, 0, None);
        let with = signal(Frame { key, tail: 0, bits: 52, counter_mode: 0, ic: None }, Some(&TABLE));
        let wire = NiceFlorSDecoder::with_table(Some(TABLE)).encode(&with, 0x4).unwrap();
        let mut rx = NiceFlorSDecoder::with_table(None);
        let got = decode_all(&mut rx, &wire);
        assert_eq!(got.len(), 16);
        assert!(got.iter().all(|d| d.serial.is_none() && d.button.is_none() && d.counter.is_none()));
        assert!(got.iter().all(|d| !d.encoder_capable && !d.crc_valid && d.extra.is_none()));
        let plain = NiceFlorSDecoder::with_table(None);
        assert!(!plain.supports_encoding());
        assert!(plain.encode(&got[0], 0x4).is_none());
    }

    /// Hand-built 52/72-bit frames in the C's layout decode; a bad stop bit, a wrong bit count or
    /// an out-of-tolerance pulse is rejected.
    #[test]
    fn synthetic_frames_and_negative_cases() {
        let key = 0x1C5783607F7B3u64;
        let mut wire = Vec::new();
        push(&mut wire, false, 19_000);
        push(&mut wire, true, 1500);
        push(&mut wire, false, 1500);
        push_bits(&mut wire, key, 52);
        push(&mut wire, true, 1500);
        let mut dec = NiceFlorSDecoder::with_table(None);
        let got = decode_all(&mut dec, &wire);
        assert_eq!(got.len(), 1);
        assert_eq!((got[0].data, got[0].data_count_bit), (key, 52));

        // 51 bits: no frame.
        let mut short = Vec::new();
        push(&mut short, false, 19_000);
        push(&mut short, true, 1500);
        push(&mut short, false, 1500);
        push_bits(&mut short, key, 51);
        push(&mut short, true, 1500);
        assert!(decode_all(&mut NiceFlorSDecoder::with_table(None), &short).is_empty());

        // A data pulse out of tolerance (te_short 500 ± 300 → 850 is out) aborts the frame.
        let mut bad = wire.clone();
        bad[5] = LevelDuration::new(bad[5].level, 850);
        assert!(decode_all(&mut NiceFlorSDecoder::with_table(None), &bad).is_empty());

        // Stop bit outside 1500 ± 300 (here 1850): not a stop bit, the frame never completes.
        let mut stop = wire.clone();
        stop.last_mut().unwrap().duration_us = 1850;
        assert!(decode_all(&mut NiceFlorSDecoder::with_table(None), &stop).is_empty());

        // Header gap must be 19000 ± 11400 µs: 7000 is too short.
        let mut gap = wire.clone();
        gap[0].duration_us = 7000;
        assert!(decode_all(&mut NiceFlorSDecoder::with_table(None), &gap).is_empty());
    }

    const KEY_52: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Nice FloR-S\nBit: 52\n\
        Key: 00 0E 4D 3A 1B 2C 5D 6E\n";
    const KEY_72: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Nice FloR-S\nBit: 72\n\
        Key: 00 01 72 4A 7D 9A 52 2F\nData: 563670\n";

    /// Key files in the garage format load (no unit-test key file exists for Nice FloR-S), and
    /// `to_flipper_key` writes back what `from_flipper_key` reads.
    #[test]
    fn key_files_load_and_round_trip() {
        let dec = NiceFlorSDecoder::with_table(None);
        let k52 = dec.from_flipper_key(&FlipperKey::parse(KEY_52).unwrap()).unwrap();
        assert_eq!((k52.data, k52.data_count_bit, k52.extra), (0x000E_4D3A_1B2C_5D6E, 52, None));
        let k72 = dec.from_flipper_key(&FlipperKey::parse(KEY_72).unwrap()).unwrap();
        assert_eq!(k72.data_count_bit, 72);
        assert_eq!(k72.data, (0x1724A7D9A522Fu64 << 12) | (0x899D6 >> 8));
        assert_eq!(k72.extra, Some(0xD6));
        assert!(k72.crc_valid);
        assert_eq!(k72.protocol_display_name.as_deref(), Some("Nice One"));
        // Bit count other than 52/72, or a 72-bit key without Data: rejected (as the C).
        assert!(dec.from_flipper_key(&FlipperKey::parse(&KEY_52.replace("Bit: 52", "Bit: 51")).unwrap()).is_none());
        assert!(dec.from_flipper_key(&FlipperKey::parse(&KEY_72.replace("Data: 563670\n", "")).unwrap()).is_none());

        let with = NiceFlorSDecoder::with_table(Some(TABLE));
        let o_code = KEY_52.to_string() + "IC: 5A 3C\nCounterMode: 2\n";
        for text in [KEY_52, KEY_72, o_code.as_str()] {
            for d in [&dec, &with] {
                let a = d.from_flipper_key(&FlipperKey::parse(text).unwrap()).unwrap();
                let written = d.to_flipper_key(&a).unwrap().to_text();
                let b = d.from_flipper_key(&FlipperKey::parse(&written).unwrap()).unwrap();
                assert_eq!(format!("{a:?}"), format!("{b:?}"), "{written}");
                assert!(written.contains("Protocol: Nice FloR-S\n"), "{written}");
            }
        }
        let oc = with.from_flipper_key(&FlipperKey::parse(&o_code).unwrap()).unwrap();
        assert_eq!(oc.protocol_display_name.as_deref(), Some("Nice O-Code"));
        assert_eq!(oc.extra, Some(EXTRA_O_CODE | 0x5A3C << 16 | 2 << 8));
        let text = with.to_flipper_key(&oc).unwrap().to_text();
        assert!(text.ends_with("Key: 00 0E 4D 3A 1B 2C 5D 6E\nIC: 5A 3C\nCounterMode: 2\n"), "{text}");
    }

    /// Flipper unit tests `nice_flor_s_raw.sub` and `nice_one_raw.sub` (decoder "Nice FloR-S")
    /// decode through the registry.
    #[test]
    fn unit_test_captures_decode() {
        for (file, bits, display) in [("nice_flor_s_raw.sub", 52, "Nice FloR-S"), ("nice_one_raw.sub", 72, "Nice One")] {
            let Some((freq, pulses)) = unit_test_capture(file) else { return };
            let mut reg = ProtocolRegistry::new();
            let hits: Vec<_> = reg
                .process_signal_stream(&pulses, freq)
                .into_iter()
                .filter(|(n, _, _)| n == display)
                .collect();
            assert!(!hits.is_empty(), "{file}");
            assert!(hits.iter().all(|(_, d, _)| d.data_count_bit == bits), "{file}");
            if bits == 72 {
                assert!(hits.iter().all(|(_, d, _)| d.crc_valid), "{file}: Nice One CRC");
            }
        }
    }

    #[test]
    fn upload_layout_matches_the_c() {
        let key = frame_with(0x0123_4567, 0x0100, 0x1, 0, None);
        let src = signal(Frame { key, tail: 0, bits: 52, counter_mode: 0, ic: None }, Some(&TABLE));
        let wire = NiceFlorSDecoder::with_table(Some(TABLE)).encode(&src, 0).unwrap();
        let t = tuples(&wire);
        // 16 × (gap + start pair + 104 bit pulses + stop), no merges.
        assert_eq!(t.len(), 16 * (3 + 104 + 1));
        assert_eq!(&t[..3], &[(false, 18_500), (true, 1500), (false, 1500)]);
        assert_eq!(t[107], (true, 1500));
        assert_eq!(t[108], (false, 18_500));
    }
}
