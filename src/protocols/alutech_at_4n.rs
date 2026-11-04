//! Alutech AT-4N decoder + encoder (rolling code, TEA-style cipher keyed by a secret table).
//!
//! Primary reference: Flipper-ARF's garage app,
//! `applications/system/subghz_garage/protocols/alutech_at_4n.c`; doctor-firmware's
//! `lib/subghz/protocols/alutech_at_4n.c` decodes and encodes the same on the wire (it only caches
//! the field analysis).
//!
//! **Air format** (AM 433 MHz): PWM, te 400/800 µs (te_delta 140; long pulses ± 280). ≥ 10
//! HIGH/LOW 400 µs preamble pairs, a LOW of 4000 µs ± 1400, then 72 bits: 1 = HIGH 400 + LOW 800,
//! 0 = HIGH 800 + LOW 400; the frame ends with a LOW ≥ 940 µs (the last bit is read from its
//! HIGH alone). `data` = the first 64 bits, `extra` bits 0..7 = the last 8 (the CRC),
//! `data_count_bit` = 72. Like the C, a frame equal to the previous one the decoder reported is
//! not reported again (the encoder sends 3 copies), and that memory survives `reset()`.
//!
//! **CRC and cipher** (`remote_controller`): the frame is sent LSB first, so the C bit-reverses
//! it: `d = reverse64(data)`, `crc = reverse8(last byte)`; the CRC is CRC-8 (poly 0x31, init
//! 0xFF) over `d`'s little-endian bytes. A valid frame is decrypted with the table (TEA-like
//! rounds, see [decrypt]) to `crc1 ‖ serial32 ‖ cnt16 ‖ btn8`, where `crc1` = `!crc8(cnt & 0xFF)`
//! (poly 0x31) is an inner check. Buttons: 0xFF, 0x11, 0x22, 0x33, 0x44 = buttons 1..5.
//!
//! **Table**: `~/.config/KAT/alutech_at_4n.bin`, 32 bytes = 8 big-endian words m0..m7
//! (`get_magic_data_from_buffer`): decryption runs `sum = m0`, keys m1/m2 (second word) and m4/m5
//! (first word), `sum += m3` until 0; encryption `sum += m6` from 0 until m0 (so m6 = -m3, m0 a
//! multiple of m6); m7 is unused. A table whose loops do not end within [MAX_ROUNDS] is ignored.
//! Without a table the C decodes the frame but shows serial/button/counter 0 (KAT: None) and
//! would transmit an all-ones cipher block; KAT does not transmit.
//!
//! **KAT gate**: the C reports CRC failures too (only skipping their decryption); KAT reports
//! only frames with a valid CRC (every frame of the unit-test capture `alutech_at_4n_raw.sub`
//! passes it), keeping the C's repeat memory evolving as in the C.
//!
//! **Encoder** (`deserialize` + `get_upload` + `gen_data`): decrypt the key (it must pass the
//! inner check), counter + 1 (`CounterMode` 1: 0 → 1 → 0xFFFE → 0xFFFF → 0, ≥ 2: 0..5 cycle),
//! button = the requested native code (one of the five, else the captured one), re-encrypt, CRC,
//! bit-reverse; upload = 12 × (HIGH 400, LOW 400) with the last LOW 4000, 64 + 8 bits, the last
//! LOW + 16000 µs; 3 copies (`encoder.repeat` = 3). `extra` bits 8..15 = `CounterMode`.
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

/// Metadata (see `info.rs`): SubGhzProtocolFlag_AM | 433.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Alutech AT-4N",
    decoder: "Alutech AT-4N",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "TEA",
    make: "Alutech",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The button byte (C: "key 1=0xff, 2=0x11, 3=0x22, 4=0x33, 5=0x44").
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0xFF => "Button 1",
        0x11 => "Button 2",
        0x22 => "Button 3",
        0x33 => "Button 4",
        0x44 => "Button 5",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 800;
const TE_DELTA: u32 = 140;
const MIN_COUNT_BIT: u8 = 72;
const BUTTONS: [u8; 5] = [0xFF, 0x11, 0x22, 0x33, 0x44];
/// Upper bound on the cipher's rounds for a user table (the real cipher is TEA-like: 32).
const MAX_ROUNDS: u32 = 1024;
/// The encoder's `encoder.repeat`.
const REPEAT: usize = 3;

/// The plaintext table from `~/.config/KAT/alutech_at_4n.bin`, read once and validated.
fn config_table() -> Option<Table> {
    static TABLE: OnceLock<Option<Table>> = OnceLock::new();
    *TABLE.get_or_init(|| {
        let table = g6_common::read_config_table("alutech_at_4n.bin")?;
        if rounds(&table).is_none() {
            tracing::warn!("alutech_at_4n.bin: the cipher loops do not end; table ignored");
            return None;
        }
        Some(table)
    })
}

/// `subghz_protocol_alutech_at_4n_get_magic_data_from_buffer`: big-endian word `n`.
fn magic(table: &Table, n: usize) -> u32 {
    u32::from_be_bytes([table[4 * n], table[4 * n + 1], table[4 * n + 2], table[4 * n + 3]])
}

/// The number of rounds of decryption and encryption, when both loops end within [MAX_ROUNDS]
/// (else the C would spin for up to 2^32 rounds or forever).
fn rounds(table: &Table) -> Option<(u32, u32)> {
    let count = |start: u32, step: u32, end: u32| {
        let mut x = start;
        (1..=MAX_ROUNDS).find(|_| {
            x = x.wrapping_add(step);
            x == end
        })
    };
    Some((count(magic(table, 0), magic(table, 3), 0)?, count(0, magic(table, 6), magic(table, 0))?))
}

/// `subghz_protocol_alutech_at_4n_decrypt` (`data` in the bit-reversed, MSB form).
fn decrypt(data: u64, table: &Table) -> u64 {
    let p = data.to_le_bytes();
    let mut data1 = u32::from_be_bytes([p[0], p[1], p[2], p[3]]);
    let mut data2 = u32::from_be_bytes([p[4], p[5], p[6], p[7]]);
    let m = |n| magic(table, n);
    let mut i = m(0);
    for _ in 0..MAX_ROUNDS {
        data2 = data2.wrapping_sub(
            m(1).wrapping_add(data1 << 4) ^ (m(2).wrapping_add(data1 >> 5) ^ data1.wrapping_add(i)),
        );
        let data3 = data2.wrapping_add(i);
        i = i.wrapping_add(m(3));
        data1 = data1.wrapping_sub(m(4).wrapping_add(data2 << 4) ^ (m(5).wrapping_add(data2 >> 5) ^ data3));
        if i == 0 {
            break;
        }
    }
    let (a, b) = (data1.to_be_bytes(), data2.to_be_bytes());
    u64::from_le_bytes([a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]])
}

/// `subghz_protocol_alutech_at_4n_encrypt`.
fn encrypt(data: u64, table: &Table) -> u64 {
    let p = data.to_le_bytes();
    let mut data2 = u32::from_be_bytes([p[0], p[1], p[2], p[3]]);
    let mut data3 = u32::from_be_bytes([p[4], p[5], p[6], p[7]]);
    let m = |n| magic(table, n);
    let mut data1 = 0u32;
    for _ in 0..MAX_ROUNDS {
        data1 = data1.wrapping_add(m(6));
        data2 = data2.wrapping_add(
            m(4).wrapping_add(data3 << 4) ^ (m(5).wrapping_add(data3 >> 5) ^ data1.wrapping_add(data3)),
        );
        data3 = data3.wrapping_add(
            m(1).wrapping_add(data2 << 4) ^ (m(2).wrapping_add(data2 >> 5) ^ data1.wrapping_add(data2)),
        );
        if data1 == m(0) {
            break;
        }
    }
    let (a, b) = (data2.to_be_bytes(), data3.to_be_bytes());
    u64::from_le_bytes([a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]])
}

/// `subghz_protocol_alutech_at_4n_crc`: CRC-8 poly 0x31, init 0xFF over the little-endian bytes.
fn frame_crc(data: u64) -> u8 {
    super::common::crc8(&data.to_le_bytes(), 0x31, 0xFF)
}

/// `subghz_protocol_alutech_at_4n_decrypt_data_crc`: the inner check byte of a counter.
fn data_crc(byte: u8) -> u8 {
    !super::common::crc8(&[byte], 0x31, 0x00)
}

/// `subghz_protocol_blocks_reverse_key(key, 64)` / `(crc, 8)`.
fn reverse64(v: u64) -> u64 {
    v.reverse_bits()
}

/// Fields decrypted from a key (`serial`, `counter`, `button`), when the inner check passes.
fn fields(data: u64, table: &Table) -> Option<(u32, u16, u8)> {
    let dec = decrypt(reverse64(data), table);
    ((dec >> 56) as u8 == data_crc((dec >> 8) as u8)).then_some(((dec >> 24) as u32, (dec >> 8) as u16, dec as u8))
}

/// The counter `gen_data` sends (`CounterMode` 0 with multiplier 1, 1, ≥ 2).
fn next_counter(mode: u8, cnt: u16) -> u16 {
    let cnt = u32::from(cnt);
    (match mode {
        0 => g6_common::counter_increment(cnt),
        1 => g6_common::counter_ofex(cnt),
        _ => {
            if cnt >= 0x0005 {
                0
            } else {
                cnt + 1
            }
        }
    }) as u16
}

/// The frame `gen_data` builds: (`generic.data`, `crc`) as sent.
fn build_frame(serial: u32, counter: u16, button: u8, table: &Table) -> (u64, u8) {
    let plain = (u64::from(data_crc(counter as u8)) << 56)
        | (u64::from(serial) << 24)
        | (u64::from(counter) << 8)
        | u64::from(button);
    let enc = encrypt(plain, table);
    (reverse64(enc), frame_crc(enc).reverse_bits())
}

/// `subghz_protocol_alutech_at_4n_create_data` (the SubGHz app's "Add Manually"): `gen_data` on a
/// fresh encoder — it first decrypts the empty key and, should that pass the inner check, takes
/// its serial and counter (the button stays the one given) — counter + 1, the frame for `btn`;
/// `Bit: 72`, `Key`, `CRC` (decimal). Needs the table.
pub(super) fn create_data(serial: u32, btn: u8, cnt: u16, table: &Table) -> FlipperKey {
    let (serial, cnt) = fields(0, table).map_or((serial, cnt), |(s, c, _)| (s, c));
    let (data, crc) = build_frame(serial, next_counter(0, cnt), btn, table);
    let mut key = FlipperKey::for_protocol("Alutech AT-4N", MIN_COUNT_BIT as usize, data);
    key.push("CRC", crc.to_string());
    key
}

/// The user's table (`~/.config/KAT/alutech_at_4n.bin`), if present and valid.
pub(super) fn user_table() -> Option<Table> {
    config_table()
}

/// The bits of `value`'s low `count` bits, MSB first, as PWM (1 = short HIGH + long LOW).
fn push_bits(out: &mut Vec<LevelDuration>, value: u64, count: u32) {
    for i in (0..count).rev() {
        if (value >> i) & 1 != 0 {
            push(out, true, TE_SHORT);
            push(out, false, TE_LONG);
        } else {
            push(out, true, TE_LONG);
            push(out, false, TE_SHORT);
        }
    }
}

/// `subghz_protocol_encoder_alutech_at_4n_get_upload` (one copy).
fn build_upload(data: u64, crc: u8) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(24 + 144);
    for _ in 0..12 {
        push(&mut out, true, TE_SHORT);
        push(&mut out, false, TE_SHORT);
    }
    out.last_mut().unwrap().duration_us += TE_SHORT * 9;
    push_bits(&mut out, data, 64);
    push_bits(&mut out, u64::from(crc), 8);
    out.last_mut().unwrap().duration_us += TE_LONG * 20;
    out
}

/// The upload the C builds when the table file is missing (for oracle comparisons): its cipher
/// returns all ones (`SUBGHZ_NO_ALUTECH_AT_4N_RAINBOW_TABLE`), whatever the key.
#[cfg(test)]
pub(super) fn c_upload_without_table() -> Vec<LevelDuration> {
    build_upload(u64::MAX, frame_crc(u64::MAX).reverse_bits()).repeat(REPEAT)
}

/// Decoder steps (`Alutech_at_4nDecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    CheckPreambula,
    SaveDuration,
    CheckDuration,
}

/// Alutech AT-4N decoder (`SubGhzProtocolDecoderAlutech_at_4n`).
pub struct AlutechAt4nDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    /// `SubGhzBlockDecoder.decode_count_bit` is a `uint8_t`.
    decode_count_bit: u8,
    /// `header_count` (`uint16_t`); not cleared by `reset()`, as in the C.
    header_count: u16,
    /// `generic.data_2`: the first 64 bits of the frame being received.
    data_2: u64,
    /// `generic.data`: the last frame reported (0 at start), for the repeat check.
    last_data: u64,
    table: Option<Table>,
}

impl AlutechAt4nDecoder {
    /// A decoder with the user's table, if `~/.config/KAT/alutech_at_4n.bin` exists and is valid.
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
            header_count: 0,
            data_2: 0,
            last_data: 0,
            table: table.filter(|t| rounds(t).is_some()),
        }
    }

    /// `subghz_protocol_blocks_add_bit` as the C's bit paths use it: after bit 64 the first 64
    /// bits move to `generic.data_2` and the CRC byte collects from 0.
    fn add_bit(&mut self, bit: u64) {
        self.add_bit_raw(bit);
        if self.decode_count_bit == 64 {
            self.data_2 = self.decode_data;
            self.decode_data = 0;
        }
    }

    /// `subghz_protocol_blocks_add_bit` alone (the end-of-frame path adds the last bit so).
    fn add_bit_raw(&mut self, bit: u64) {
        self.decode_data = (self.decode_data << 1) | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }

    /// The decode of a frame (`remote_controller` + `get_string`).
    fn signal(&self, data: u64, crc: u8, counter_mode: u8) -> DecodedSignal {
        let crc_valid = crc.reverse_bits() == frame_crc(reverse64(data));
        let decrypted = if crc_valid { self.table.as_ref().and_then(|t| fields(data, t)) } else { None };
        DecodedSignal {
            serial: decrypted.map(|f| f.0),
            button: decrypted.map(|f| f.2),
            counter: decrypted.map(|f| f.1),
            crc_valid,
            data,
            data_count_bit: MIN_COUNT_BIT as usize,
            // `gen_data` decrypts the key whatever its outer CRC and needs the inner check.
            encoder_capable: self.table.as_ref().is_some_and(|t| fields(data, t).is_some()),
            extra: Some(u64::from(crc) | (u64::from(counter_mode) << 8)),
            protocol_display_name: None,
        }
    }
}

#[cfg(test)]
impl AlutechAt4nDecoder {
    /// The decode of a fresh frame for these fields (needs the decoder's table).
    pub(super) fn test_frame(&self, serial: u32, counter: u16, button: u8) -> DecodedSignal {
        let (data, crc) = build_frame(serial, counter, button, self.table.as_ref().unwrap());
        self.signal(data, crc, 0)
    }
}

impl Default for AlutechAt4nDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for AlutechAt4nDecoder {
    fn name(&self) -> &'static str {
        "Alutech AT-4N"
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

    /// The garage copy's reset: the parser step only.
    fn reset(&mut self) {
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.step = Step::CheckPreambula;
                    self.header_count = self.header_count.wrapping_add(1);
                }
            }
            Step::CheckPreambula => {
                if !level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.step = Step::Reset;
                } else if self.header_count > 9 && duration_diff!(duration, TE_SHORT * 10) < TE_DELTA * 10 {
                    // Found header (the C does not check the level here).
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                } else {
                    self.step = Step::Reset;
                    self.header_count = 0;
                }
            }
            Step::SaveDuration => {
                if level {
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                }
            }
            Step::CheckDuration => {
                if !level {
                    if duration >= TE_SHORT * 2 + TE_DELTA {
                        // End of the frame: the last bit from its HIGH alone.
                        if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA {
                            self.add_bit_raw(1);
                        } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA * 2 {
                            self.add_bit_raw(0);
                        }
                        self.step = Step::Reset;
                        let mut result = None;
                        if self.decode_count_bit == MIN_COUNT_BIT {
                            if self.last_data != self.data_2 {
                                self.last_data = self.data_2;
                                let d = self.signal(self.data_2, self.decode_data as u8, 0);
                                // KAT: report only CRC-valid frames (the C reports every one).
                                if d.crc_valid {
                                    result = Some(d);
                                }
                            }
                            self.decode_data = 0;
                            self.decode_count_bit = 0;
                            self.header_count = 0;
                        }
                        return result;
                    } else if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA * 2
                    {
                        self.add_bit(1);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA * 2
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(0);
                        self.step = Step::SaveDuration;
                    } else {
                        self.step = Step::Reset;
                        self.header_count = 0;
                    }
                } else {
                    self.step = Step::Reset;
                    self.header_count = 0;
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
        if decoded.data_count_bit != MIN_COUNT_BIT as usize {
            return None;
        }
        let (serial, counter, captured) = fields(decoded.data, table)?;
        let mode = (decoded.extra.unwrap_or(0) >> 8) as u8;
        let btn = if BUTTONS.contains(&button) { button } else { captured };
        let (data, crc) = build_frame(serial, next_counter(mode, counter), btn, table);
        Some(build_upload(data, crc).repeat(REPEAT))
    }

    /// The decoder's deserialize (`Bit` 72, `CRC` required, `CounterMode`) + `get_string`.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != MIN_COUNT_BIT as usize {
            return None;
        }
        let crc = key.u32_field("CRC")?;
        Some(self.signal(key.key, crc as u8, g6_common::counter_mode(key)))
    }

    /// The serialize: `Protocol`, `Bit`, `Key`, `CRC` (decimal), then the key's `CounterMode`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT as usize {
            return None;
        }
        let extra = decoded.extra?;
        let mut key = FlipperKey::new(
            "Alutech AT-4N",
            MIN_COUNT_BIT as usize,
            decoded.data,
            433_920_000,
            "FuriHalSubGhzPresetOok650Async",
        );
        key.push("CRC", (extra & 0xFF).to_string());
        let mode = (extra >> 8) as u8;
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

    /// A synthetic TEA-shaped table (the real one is secret): m0 = 32 × delta, m3 = −delta,
    /// m6 = delta, arbitrary keys m1, m2, m4, m5 and an unused m7.
    fn table() -> Table {
        let delta = 0x9E37_79B9u32;
        let words = [delta.wrapping_mul(32), 0xA56B_3C91, 0x1F2E_3D4C, delta.wrapping_neg(), 0x7788_99AA, 0x0BAD_F00D, delta, 0xDEAD_BEEF];
        let mut t = [0u8; 32];
        for (i, w) in words.iter().enumerate() {
            t[4 * i..4 * i + 4].copy_from_slice(&w.to_be_bytes());
        }
        t
    }

    /// Golden vectors from the C: the garage copy's `subghz_protocol_alutech_at_4n_encrypt`,
    /// `_decrypt` and `_crc` extracted verbatim and compiled on the host with [table].
    #[rustfmt::skip]
    const C_VECTORS: &[(u64, u64, u64, u8)] = &[ // (x, encrypt(x), decrypt(x), crc(x))
        (0xAA6DB2B434168DEF, 0x58B7372B4603C3F9, 0xF2007AADCDCBA385, 0xF5),
        (0x4824139843F30ACA, 0x962B1662D1200381, 0x5C875F554E04FA65, 0xA4),
        (0xA88FFF785F53547B, 0xC94EF7E2ED526A34, 0x43B293F715774C15, 0x23),
        (0x5994F6CCB416DECE, 0x890383730BA9A028, 0x223E4A9D466C67E4, 0x19),
        (0x0A57F890C936FB35, 0x615B29E35E61EECD, 0x95776BB4B0585430, 0x9D),
        (0x08336D83BDEEE67C, 0xC1CC433CB5044CED, 0xFC8E5EC818EF95E2, 0x40),
        (0xB0E402E6154D8407, 0x7C3D7D0F6BE9C736, 0x5E238355D73B1568, 0xB6),
        (0x88B66519D122B1F8, 0x3AFA1F77DC05887B, 0xF29DE28E6F8F2AC5, 0x20),
    ];

    #[test]
    fn matches_the_c_golden_vectors() {
        let t = table();
        for &(x, enc, dec, crc) in C_VECTORS {
            assert_eq!(encrypt(x, &t), enc, "encrypt {x:X}");
            assert_eq!(decrypt(x, &t), dec, "decrypt {x:X}");
            assert_eq!(frame_crc(x), crc, "crc {x:X}");
        }
    }

    #[test]
    fn cipher_round_trips() {
        let t = table();
        assert_eq!(rounds(&t), Some((32, 32)));
        for plain in [0u64, u64::MAX, 0x0123_4567_89AB_CDEF, 0xC6EF_3720_0000_0001] {
            assert_eq!(decrypt(encrypt(plain, &t), &t), plain);
        }
        // A table whose loops never end is rejected.
        let mut bad = t;
        bad[12..16].copy_from_slice(&0u32.to_be_bytes());
        assert!(rounds(&bad).is_none());
        assert!(AlutechAt4nDecoder::with_table(Some(bad)).table.is_none());
    }

    #[test]
    fn crc_helpers_match_the_c() {
        // decrypt_data_crc(0xFF): 8 shifts of 0xFF with poly 0x31 → 0xAC, inverted 0x53.
        assert_eq!(data_crc(0xFF), 0x53);
        assert_eq!(frame_crc(0), super::super::common::crc8(&[0; 8], 0x31, 0xFF));
    }

    fn key(serial: u32, counter: u16, button: u8) -> (u64, u8) {
        build_frame(serial, counter, button, &table())
    }

    /// encode → decode with the synthetic table: counter + 1 (and the CounterMode sequences), the
    /// requested or captured button; 3 copies decode once (the repeat check).
    #[test]
    fn encode_decode_round_trip_with_a_table() {
        let t = table();
        for (mode, from, to) in [(0u8, 0x0041u16, 0x0042u16), (0, 0xFFFF, 0), (1, 0x0041, 0xFFFE), (1, 0, 1), (2, 5, 0), (2, 3, 4)] {
            for (button, want) in [(0u8, 0x22u8), (0x11, 0x11), (0x44, 0x44), (0x12, 0x22)] {
                let (data, crc) = key(0x1234_5678, from, 0x22);
                let src = AlutechAt4nDecoder::with_table(Some(t)).signal(data, crc, mode);
                assert!(src.crc_valid && src.encoder_capable);
                let wire = AlutechAt4nDecoder::with_table(Some(t)).encode(&src, button).unwrap();
                let got = decode_all(&mut AlutechAt4nDecoder::with_table(Some(t)), &wire);
                assert_eq!(got.len(), 1, "3 identical copies decode once");
                let d = &got[0];
                assert_eq!((d.serial, d.counter, d.button), (Some(0x1234_5678), Some(to), Some(want)));
                assert!(d.crc_valid && d.encoder_capable);
            }
        }
    }

    #[test]
    fn upload_layout_matches_the_c() {
        let (data, crc) = key(0xCAFE_F00D, 7, 0xFF);
        let t = tuples(&build_upload(data, crc));
        assert_eq!(t.len(), 24 + 144);
        assert_eq!(t[22], (true, 400));
        assert_eq!(t[23], (false, 4000));
        let last = t.last().unwrap();
        assert!(*last == (false, 16_400) || *last == (false, 16_800), "{last:?}");
    }

    /// Without a table: frames decode (CRC checked, fields unknown) and nothing is transmitted.
    #[test]
    fn no_table_decodes_without_fields() {
        let (data, crc) = key(0x0BAD_CAFE, 0x1000, 0x33);
        let wire = build_upload(data, crc).repeat(REPEAT);
        let mut dec = AlutechAt4nDecoder::with_table(None);
        let got = decode_all(&mut dec, &wire);
        assert_eq!(got.len(), 1);
        assert_eq!((got[0].data, got[0].extra), (data, Some(u64::from(crc))));
        assert!(got[0].crc_valid && !got[0].encoder_capable);
        assert_eq!((got[0].serial, got[0].button, got[0].counter), (None, None, None));
        assert!(dec.encode(&got[0], 0x33).is_none());
        assert!(!dec.supports_encoding());
        // The same frame again is not reported (the C's repeat check survives reset()).
        dec.reset();
        assert!(decode_all(&mut dec, &wire).is_empty());
    }

    /// Hand-built frames: a bad CRC is not reported (KAT gate); short preamble, wrong bit count
    /// and out-of-tolerance pulses are rejected.
    #[test]
    fn synthetic_frames_and_negative_cases() {
        let (data, crc) = key(0x0102_0304, 0x0506, 0x11);
        let good = build_upload(data, crc);
        assert_eq!(decode_all(&mut AlutechAt4nDecoder::with_table(None), &good).len(), 1);
        let bad_crc = build_upload(data, crc ^ 0x01);
        assert!(decode_all(&mut AlutechAt4nDecoder::with_table(None), &bad_crc).is_empty());
        // 9 preamble pairs are not enough (header_count must exceed 9).
        let short_pre = good[6..].to_vec();
        assert!(decode_all(&mut AlutechAt4nDecoder::with_table(None), &short_pre).is_empty());
        // A data pulse out of tolerance (neither 400 ± 140 nor 800 ± 280).
        let mut bad = good.clone();
        bad[30].duration_us = 1200;
        assert!(decode_all(&mut AlutechAt4nDecoder::with_table(None), &bad).is_empty());
        // 71 bits.
        let mut short = good[..good.len() - 2].to_vec();
        short.last_mut().unwrap().duration_us += 16_000;
        assert!(decode_all(&mut AlutechAt4nDecoder::with_table(None), &short).is_empty());
    }

    const KEY: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Alutech AT-4N\nBit: 72\n\
        Key: 01 23 45 67 89 AB CD EF\nCRC: 90\n";

    #[test]
    fn key_files_load_and_round_trip() {
        let dec = AlutechAt4nDecoder::with_table(None);
        let d = dec.from_flipper_key(&FlipperKey::parse(KEY).unwrap()).unwrap();
        assert_eq!((d.data, d.data_count_bit, d.extra), (0x0123_4567_89AB_CDEF, 72, Some(90)));
        assert!(dec.from_flipper_key(&FlipperKey::parse(&KEY.replace("CRC: 90\n", "")).unwrap()).is_none());
        assert!(dec.from_flipper_key(&FlipperKey::parse(&KEY.replace("Bit: 72", "Bit: 64")).unwrap()).is_none());
        let (data, crc) = key(0x0BAD_CAFE, 0x1000, 0x33);
        let with = AlutechAt4nDecoder::with_table(Some(table()));
        let valid = KEY.replace("01 23 45 67 89 AB CD EF", &format!("{:016X}", data)).replace("CRC: 90", &format!("CRC: {crc}"));
        for d in [&dec, &with] {
            for text in [KEY.to_string(), valid.clone(), valid.clone() + "CounterMode: 1\n"] {
                let a = d.from_flipper_key(&FlipperKey::parse(&text).unwrap()).unwrap();
                let written = d.to_flipper_key(&a).unwrap().to_text();
                let b = d.from_flipper_key(&FlipperKey::parse(&written).unwrap()).unwrap();
                assert_eq!(format!("{a:?}"), format!("{b:?}"), "{written}");
            }
        }
        let v = with.from_flipper_key(&FlipperKey::parse(&valid).unwrap()).unwrap();
        assert_eq!((v.serial, v.counter, v.button, v.crc_valid), (Some(0x0BAD_CAFE), Some(0x1000), Some(0x33), true));
    }

    /// Flipper unit test `alutech_at_4n_raw.sub` decodes through the registry (CRC valid).
    #[test]
    fn unit_test_capture_decodes() {
        let Some((freq, pulses)) = unit_test_capture("alutech_at_4n_raw.sub") else { return };
        let mut reg = ProtocolRegistry::new();
        let hits: Vec<_> = reg
            .process_signal_stream(&pulses, freq)
            .into_iter()
            .filter(|(n, _, _)| n == "Alutech AT-4N")
            .collect();
        assert!(!hits.is_empty());
        assert!(hits.iter().all(|(_, d, _)| d.crc_valid && d.data_count_bit == 72));
    }
}
