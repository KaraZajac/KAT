//! Audi V3 (FoxFW `audi_v3.c`, `AUDI_V3_PROTOCOL_NAME = "Audi V3"`): 2-FSK VAG key fob, 96 bits —
//! **decode only**, as the reference (its encoder is NULL).
//!
//! Ported from FoxFW (itself a port of the firmware's `audi_v3.c` via KeeLoq-RX `arke_vag.py`).
//! **Air format** (FM, te ~383 / ~766 µs; flags 433 | 315 | AM | FM): a custom half-bit demodulator,
//! not the toolbox Manchester table. A preamble of at least 20 pulses of 220..=470 µs, then one long
//! pulse (580..=980 µs) that opens the data window and supplies the first bit; after that one long
//! pulse is one bit, two consecutive short pulses (under 580 µs) are one bit, and the bit's value is
//! `level ? 0 : 1` of the pulse that completes it. A pulse of 980 µs or more ends the frame; so does
//! a long pulse arriving between the two shorts of a bit.
//!
//! **Frame** (96 bits, MSB first): `button:8 | hop:56 | counter:24 | chk:8`. A frame is reported
//! only when the button is 0x0C (Unlock) or 0x0D (Lock) and the 56-bit hop is neither all 0x00 nor
//! all 0xFF. The `chk` byte is displayed by the C but never verified, so no check exists. The hop
//! is encrypted.
//!
//! **KAT fields**: `data` = `button << 56 | hop` (the C's `generic.data`), `data_count_bit` = 96,
//! `button` = the button byte, `counter` = the low 16 bits of the 24-bit counter, `extra` =
//! `counter << 8 | chk` (the full counter and the check byte), `serial` None, `crc_valid` false.
//!
//! The frame is gated only by the button whitelist and the non-degenerate hop. A gap, or a long
//! pulse between a bit's two shorts, ends the window and reports a frame only if all 96 bits are in
//! (`audi_v3_process` returns below 96).
//!
//! **Deviation: the BCM2 key's frame.** A key specialist's 2008–2015 Audi A4 key (BCM2 system,
//! 868.35 MHz; `IMPORTS/VAG/more/2008-2015_audi_a4_bcm2_*`, 20 presses of each button) sends this
//! signal, with a frame the C does not read: its first eight bits are 0x05 or 0x04, outside the
//! whitelist. The frame is 97 bits, the opening long pulse's bit (0, a start bit) and 12 bytes:
//! ```text
//! byte 0        button: 0x0A Lock, 0x08 Unlock, 0x09 Trunk (as the recordings are named)
//! bytes 1..7    48 bits that change from press to press with no pattern (a MAC, below)
//! bytes 7..11   counter, 32 bits: up by 3 with every press, on from one recording to the next
//! byte 11       the sum of bytes 0..11, mod 256
//! ```
//! All 61 frames pass the sum.
//!
//! **The module's side** (the firmware of an Audi A5 BCM2, 8K0907064D, read by the same key
//! specialist; routines `0x7E204` receive, `0x7E3E6` handler, `0x7E002` check). It reads the frame
//! exactly so: 12 bytes, kept when byte 11 is the sum of bytes 0..11, whatever byte 0 is. Byte 0 is
//! `[0][key number:3][status:2][button:2]`: the key number (0..=7; the module numbers its keys
//! from 1, so 0 is key 1) picks which learned key to check against; the button is 0 Unlock,
//! 1 Trunk, 2 Lock, 3 a fourth button; the status is not checked, and its values 00 and 01 set
//! a per-key fault (01 with a warning), 10 and 11 clear them (most likely the key's battery). The
//! A4 key sends key number 0, status 10. Bytes 1..7 are the last 6 bytes of an AES-128 block the
//! module computes from a value of the car, the counter, the key's ID and the button, under a key
//! of the car: a MAC, so nothing in the frame can be read without the module's key. The counter
//! must be ahead of the one stored for that key. The short bursts after a press are "button held"
//! frames the module compares with the press (key number and button) for comfort opening/closing.
//!
//! Read from the start bit, as the C reads, the fields straddle these bytes. The C's two buttons
//! are this byte 0 shifted by one: 0x0C is 0x18 (key 2, Unlock) and 0x0D is 0x1A (key 2, Lock).
//! So FoxFW's key was most likely a car's second key, read one bit early (not proven: FoxFW has no
//! recording). KAT tries this layout first: the 97 bits (a 97th bit lost in the closing gap is the
//! gap's, `level ? 0 : 1`, as for any bit) with a start bit of 0 (a recording of the other
//! polarity, start bit 1, is read inverted), byte 0 with bit 7 clear (never seen set), a MAC that
//! is not all 0x00 or 0xFF (12 zero bytes pass the sum) and the sum: every key number, status and
//! button ([bcm2_valid]). Such a frame is packed as the C packs its 96
//! bits (`data` = bytes 0..8, `extra` = bytes 8..11 << 8 | byte 11, so key files are the C's), with
//! bit [BCM2_FLAG] of `extra` set; `button` = byte 0 (named with its key, [capture_button_name]),
//! `counter` = the counter's low 16 bits, `crc_valid` = the sum, which `info.rs` checks for these
//! frames only (`PARTIAL_CHECK`). A key file says nothing of its layout, so its 12 bytes are taken
//! as this layout by the same test, except byte 0 0x0C / 0x0D, which stay the C's.

use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): flags 433 | 315 | AM | FM, dynamic.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::Both,
    encryption: "Encrypted (rolling)",
    make: "Audi",
    category: Category::Vehicle,
    buttons: Some(buttons),
    presets: &[&AM650, &FM476],
};

/// The two buttons the C accepts (its header: 0x0C Unlock, 0x0D Lock), and the BCM2 key's byte 0
/// for its first key (module doc). A capture is named by [capture_button_name], which knows every
/// key number; this table is what the protocol lists.
fn buttons(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0x0C | 0x08 => Some("Unlock"),
        0x0D | 0x0A => Some("Lock"),
        0x09 => Some("Trunk"),
        0x0B => Some("Btn 4"),
        _ => None,
    }
}

/// A frame read in the BCM2 key's layout ([BCM2_FLAG] in `extra`): its byte 0's button (bits 0..2)
/// with the key number (bits 4..7), counted from 1 as the module counts its keys. None for the
/// C's frames, whose names are [buttons].
pub(super) fn capture_button_name(code: u8, extra: Option<u64>) -> Option<&'static str> {
    const LABELS: [[&str; 4]; 8] = [
        ["Unlock, key 1", "Trunk, key 1", "Lock, key 1", "Btn 4, key 1"],
        ["Unlock, key 2", "Trunk, key 2", "Lock, key 2", "Btn 4, key 2"],
        ["Unlock, key 3", "Trunk, key 3", "Lock, key 3", "Btn 4, key 3"],
        ["Unlock, key 4", "Trunk, key 4", "Lock, key 4", "Btn 4, key 4"],
        ["Unlock, key 5", "Trunk, key 5", "Lock, key 5", "Btn 4, key 5"],
        ["Unlock, key 6", "Trunk, key 6", "Lock, key 6", "Btn 4, key 6"],
        ["Unlock, key 7", "Trunk, key 7", "Lock, key 7", "Btn 4, key 7"],
        ["Unlock, key 8", "Trunk, key 8", "Lock, key 8", "Btn 4, key 8"],
    ];
    let bcm2 = extra.is_some_and(|e| (e >> BCM2_FLAG) & 1 == 1);
    bcm2.then(|| LABELS[usize::from((code >> 4) & 7)][usize::from(code & 3)])
}

const NAME: &str = "Audi V3";
const PRE_LO: u32 = 220;
const PRE_HI: u32 = 470;
const LONG_LO: u32 = 580;
const LONG_HI: u32 = 980;
const GAP: u32 = 980;
const MIN_PRE: u16 = 20;
/// The half-bit durations the encoder sends (the A4 key's ~383 / ~766 µs).
const SHORT_US: u32 = 383;
const LONG_US: u32 = 766;
const DATA_BITS: usize = 96;
/// The BCM2 key's frame: a start bit and 12 bytes (module doc).
const FRAME_BITS: usize = DATA_BITS + 1;
/// Bit of `extra` set on a frame read in the BCM2 key's layout (checked by its sum).
pub(super) const BCM2_FLAG: u32 = 40;

/// The BCM2 key's 12 bytes pass as the module reads them (module doc): byte 11 = the sum of the
/// others, and byte 0's bit 7 clear (any key number, status and button). The MAC (bytes 1..7) is
/// neither all 0x00 nor all 0xFF, as the C wants of its hop: a run of one level in a recording
/// reads as 12 zero bytes, whose sum holds (an Opel remote's recordings have one), and an AES
/// output is all 0x00 or 0xFF once in 2^47.
fn bcm2_valid(bytes: &[u8; 12]) -> bool {
    let mac = &bytes[1..7];
    bytes[0] & 0x80 == 0
        && mac != [0x00; 6]
        && mac != [0xFF; 6]
        && bytes[..11].iter().fold(0u8, |sum, &b| sum.wrapping_add(b)) == bytes[11]
}

/// A key file's 12 bytes taken as the BCM2 key's frame: [bcm2_valid], but byte 0 0x0C / 0x0D stay
/// the C's two buttons (a key file does not say which layout its bytes are in).
fn bcm2_key_file(bytes: &[u8; 12]) -> bool {
    !matches!(bytes[0], 0x0C | 0x0D) && bcm2_valid(bytes)
}

/// The 12 bytes of a frame in the BCM2 key's layout: 97 bits, a start bit of 0 (1 = a recording of
/// the other polarity, read inverted), then the bytes, which must pass [bcm2_valid].
fn bcm2_bytes(bits: &[u8]) -> Option<[u8; 12]> {
    if bits.len() != FRAME_BITS {
        return None;
    }
    let invert = bits[0] & 1;
    let mut bytes = [0u8; 12];
    for (i, &bit) in bits[1..].iter().enumerate() {
        bytes[i / 8] = (bytes[i / 8] << 1) | ((bit ^ invert) & 1);
    }
    bcm2_valid(&bytes).then_some(bytes)
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Preamble,
    Data,
}

/// Audi V3 decoder (`SubGhzProtocolDecoderAudiV3`).
pub struct AudiV3Decoder {
    step: Step,
    pre: u16,
    half: bool,
    /// The C's 96 bits, and the BCM2 key's 97th (module doc).
    bits: [u8; FRAME_BITS],
    nbits: usize,
}

impl AudiV3Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            pre: 0,
            half: false,
            bits: [0; FRAME_BITS],
            nbits: 0,
        }
    }

    /// `audi_v3_clear`.
    fn clear(&mut self) {
        self.pre = 0;
        self.half = false;
        self.nbits = 0;
    }

    /// `audi_v3_append`: the bit is `level ? 0 : 1`.
    fn append(&mut self, level: bool) {
        if self.nbits < FRAME_BITS {
            self.bits[self.nbits] = u8::from(!level);
            self.nbits += 1;
        }
    }

    /// The frame in the BCM2 key's layout (KAT, module doc), else `audi_v3_process`: the C's
    /// frame, if all 96 bits are in and the button and hop pass. `gap` = the level of the gap that
    /// ended the window, whose bit is the 97th when only 96 came before it.
    fn process(&self, gap: Option<bool>) -> Option<DecodedSignal> {
        let mut bits = self.bits[..self.nbits].to_vec();
        if let (DATA_BITS, Some(level)) = (bits.len(), gap) {
            bits.push(u8::from(!level));
        }
        if let Some(bytes) = bcm2_bytes(&bits) {
            let data = u64::from_be_bytes(bytes[..8].try_into().unwrap());
            let counter = u32::from_be_bytes([0, bytes[8], bytes[9], bytes[10]]);
            return Some(signal(data, counter, bytes[11], true));
        }
        if self.nbits < DATA_BITS {
            return None;
        }
        let field = |from: usize, len: usize| {
            self.bits[from..from + len]
                .iter()
                .fold(0u64, |acc, &b| (acc << 1) | u64::from(b & 1))
        };
        let button = field(0, 8) as u8;
        let hop = field(8, 56);
        let counter = field(64, 24) as u32;
        let chk = field(88, 8) as u8;
        if hop == 0 || hop == 0x00FF_FFFF_FFFF_FFFF {
            return None;
        }
        if button != 0x0C && button != 0x0D {
            return None;
        }
        Some(signal((u64::from(button) << 56) | hop, counter, chk, false))
    }
}

impl Default for AudiV3Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for AudiV3Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: 383,
            te_long: 766,
            te_delta: 180,
            min_count_bit: DATA_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.clear();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let in_pre = (PRE_LO..=PRE_HI).contains(&duration);
        let is_long = (LONG_LO..=LONG_HI).contains(&duration);
        let is_short = duration < LONG_LO;

        match self.step {
            Step::Reset => {
                if in_pre {
                    self.pre = 1;
                    self.step = Step::Preamble;
                }
            }
            Step::Preamble => {
                if in_pre {
                    self.pre = self.pre.wrapping_add(1);
                } else if is_long && self.pre >= MIN_PRE {
                    self.step = Step::Data;
                    self.nbits = 0;
                    self.append(level); // the first bit comes from the opening long pulse
                    self.half = false;
                } else {
                    self.clear();
                    self.step = Step::Reset;
                }
            }
            Step::Data => {
                if duration >= GAP {
                    let out = self.process(Some(level));
                    self.clear();
                    self.step = Step::Reset;
                    return out;
                }
                if is_long {
                    if self.half {
                        let out = self.process(None);
                        self.clear();
                        self.step = Step::Reset;
                        return out;
                    }
                    self.append(level);
                } else if is_short {
                    if self.half {
                        self.append(level);
                        self.half = false;
                    } else {
                        self.half = true;
                    }
                }
                // The C stops at 96 bits; KAT reads on to the BCM2 key's 97th (module doc). A
                // 96-bit frame of the C's ends at the gap, or at the next bit, with the same fields.
                if self.nbits >= FRAME_BITS {
                    let out = self.process(None);
                    self.clear();
                    self.step = Step::Reset;
                    return out;
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// Make the next press of a BCM2 frame's key for `button` (a KAT generic code), using the car's
    /// loaded key ([encode_frame]). None for the C's frames (encrypted hop, Replay only) and for a
    /// BCM2 frame whose car key is not loaded.
    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let bytes = bcm2_bytes_of(decoded)?;
        encode_frame(&bytes, map_button(button, bytes[0] & 3))
    }

    /// "Send next code": the captured button exactly (the Audi button codes overlap KAT's generic
    /// ones, so the default `encode` with the raw byte would remap them).
    fn encode_next(&self, decoded: &DecodedSignal) -> Option<Vec<LevelDuration>> {
        let bytes = bcm2_bytes_of(decoded)?;
        encode_frame(&bytes, bytes[0] & 3)
    }

    /// `subghz_protocol_decoder_audi_v3_deserialize`: `Bit: 96`, the key (`button << 56 | hop`) and
    /// the optional `Cnt` (24 bits) and `Chk` lines (0 when absent); no gate is applied.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != DATA_BITS {
            return None;
        }
        let cnt = key.u32_field("Cnt").unwrap_or(0) & 0xFF_FFFF;
        let chk = key.u32_field("Chk").unwrap_or(0) as u8;
        Some(signal(key.key, cnt, chk, bcm2_key_file(&pack(key.key, cnt, chk))))
    }

    /// `subghz_protocol_decoder_audi_v3_serialize`: the generic Key, then `Btn`, `Cnt`, `Chk`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != DATA_BITS {
            return None;
        }
        let extra = decoded.extra?;
        let mut key = FlipperKey::for_protocol(NAME, DATA_BITS, decoded.data);
        key.push_u32("Btn", u32::from(decoded.button?));
        key.push_u32("Cnt", (extra >> 8) as u32 & 0xFF_FFFF);
        key.push_u32("Chk", (extra & 0xFF) as u32);
        Some(key)
    }
}

/// The 12 bytes the C's fields hold: `data` (`button << 56 | hop`), the 24-bit counter, the check.
fn pack(data: u64, counter: u32, chk: u8) -> [u8; 12] {
    let mut bytes = [0u8; 12];
    bytes[..8].copy_from_slice(&data.to_be_bytes());
    bytes[8..11].copy_from_slice(&counter.to_be_bytes()[1..]);
    bytes[11] = chk;
    bytes
}

/// A decode of `data` (`button << 56 | hop`), the 24-bit counter and the check byte. `bcm2`: the
/// frame was read in the BCM2 key's layout (module doc), so its sum holds and [BCM2_FLAG] is set.
/// `encoder_capable` is true for a BCM2 frame when a loaded car key (the keystore) reproduces its
/// MAC: KAT can then make that key's next press ([encode_frame]).
fn signal(data: u64, counter: u32, chk: u8, bcm2: bool) -> DecodedSignal {
    let encoder_capable = bcm2 && {
        let bytes = pack(data, counter, chk);
        car_for(&bytes, super::keys::get_keystore().audi_bcm2_keys()).is_some()
    };
    DecodedSignal {
        serial: None,
        button: Some((data >> 56) as u8),
        counter: Some((counter & 0xFFFF) as u16),
        crc_valid: bcm2,
        data,
        data_count_bit: DATA_BITS,
        encoder_capable,
        extra: Some((u64::from(bcm2) << BCM2_FLAG) | (u64::from(counter & 0xFF_FFFF) << 8) | u64::from(chk)),
        protocol_display_name: None,
    }
}

// ─── Encoder (the module's RF MAC, 0x7E002) ────────────────────────────────────────────────────
//
// A BCM2 frame's bytes 1..7 are a 48-bit AES-128 MAC over
// `car_value ‖ counter(BE) ‖ key_id ‖ [button|0x0C, 0,0,0]`, under the car's 128-bit key. The
// inputs are secret to the car and come from its own module ([AudiBcm2Car], a user key file), so
// KAT can forge a frame only for a car whose key the user loaded, and it checks the key fits a
// capture by recomputing the MAC ([car_for]). The MAC itself is read from the firmware and not yet
// confirmed against a real car's frames; it reproduces every frame KAT's own encoder builds (the
// tests), and awaits recordings of a dumped car's keys to confirm it on the air.

/// How much the counter rises per press (the step seen in the A4 recordings; the module accepts any
/// counter ahead of the one it stored, within a window).
const COUNTER_STEP: u32 = 3;
/// Preamble pulses the encoder sends before the opening long (the recorded key sends ~270).
const PREAMBLE_PULSES: usize = 274;

/// The frame's 48-bit MAC: AES-128 of `car_value ‖ counter(BE) ‖ key_id ‖ [button|0x0C,0,0,0]`,
/// bytes 10..16 (`button` the 2-bit code 0 Unlock / 1 Trunk / 2 Lock / 3 fourth).
fn authenticator(key: &[u8; 16], car_value: &[u8; 4], counter: u32, key_id: &[u8; 4], button: u8) -> [u8; 6] {
    let mut m = [0u8; 16];
    m[0..4].copy_from_slice(car_value);
    m[4..8].copy_from_slice(&counter.to_be_bytes());
    m[8..12].copy_from_slice(key_id);
    m[12] = (button & 3) | 0x0C;
    aes::encrypt(key, &m)[10..16].try_into().unwrap()
}

/// Assemble the 12 bytes: `[key_num<<4 | status<<2 | button][MAC 6][counter(BE) 4][sum of 0..10]`.
fn build_bcm2(key_num: u8, status: u8, button: u8, counter: u32, mac: &[u8; 6]) -> [u8; 12] {
    let mut b = [0u8; 12];
    b[0] = ((key_num & 7) << 4) | ((status & 3) << 2) | (button & 3);
    b[1..7].copy_from_slice(mac);
    b[7..11].copy_from_slice(&counter.to_be_bytes());
    b[11] = b[..11].iter().fold(0u8, |s, &x| s.wrapping_add(x));
    b
}

/// The 12 bytes of a decoded BCM2 frame (the inverse of [signal]'s packing); None for the C's frames.
fn bcm2_bytes_of(d: &DecodedSignal) -> Option<[u8; 12]> {
    let extra = d.extra?;
    if (extra >> BCM2_FLAG) & 1 == 0 {
        return None;
    }
    let mut b = [0u8; 12];
    b[..8].copy_from_slice(&d.data.to_be_bytes());
    b[8] = (extra >> 24) as u8;
    b[9] = (extra >> 16) as u8;
    b[10] = (extra >> 8) as u8;
    b[11] = extra as u8;
    Some(b)
}

/// The loaded car whose key reproduces `bytes`' MAC, with the frame's key number (0..=7) and 32-bit
/// counter. The MAC is one-way, so this is how KAT tells a key belongs to a capture.
fn car_for<'a>(bytes: &[u8; 12], cars: &'a [super::keys::AudiBcm2Car]) -> Option<(&'a super::keys::AudiBcm2Car, u8, u32)> {
    let key_num = (bytes[0] >> 4) & 7;
    let button = bytes[0] & 3;
    let counter = u32::from_be_bytes([bytes[7], bytes[8], bytes[9], bytes[10]]);
    let mac: [u8; 6] = bytes[1..7].try_into().unwrap();
    cars.iter()
        .find(|c| c.key_ids.get(key_num as usize).is_some_and(|id| authenticator(&c.key, &c.car_value, counter, id, button) == mac))
        .map(|c| (c, key_num, counter))
}

/// Build the next press of the key that sent `bytes`, for the 2-bit `button` (keeping the captured
/// key number and status): raise the counter by [COUNTER_STEP], recompute the MAC with the car's
/// key, and emit the pulses. None unless a loaded car key reproduces `bytes`' MAC.
fn encode_frame(bytes: &[u8; 12], button: u8) -> Option<Vec<LevelDuration>> {
    let keys = super::keys::get_keystore().audi_bcm2_keys().to_vec();
    let (car, key_num, counter) = car_for(bytes, &keys)?;
    let status = (bytes[0] >> 2) & 3;
    let next = counter.wrapping_add(COUNTER_STEP);
    let mac = authenticator(&car.key, &car.car_value, next, &car.key_ids[key_num as usize], button);
    Some(bcm2_pulses(&build_bcm2(key_num, status, button, next, &mac), PREAMBLE_PULSES))
}

/// KAT's generic button code → the Audi 2-bit button; anything else keeps the captured 2 bits.
fn map_button(requested: u8, captured: u8) -> u8 {
    match requested {
        0x01 => 2, // Lock
        0x02 => 0, // Unlock
        0x04 => 1, // Trunk
        _ => captured,
    }
}

/// The pulses the decoder reads back as `bytes` (module doc): `n_pre` short preamble pulses, the
/// opening long (its level makes the start bit 0), then the 96 data bits as long pulses or short
/// pairs. Every pulse flips level; a bit is `level ? 0 : 1` of the completing pulse.
fn bcm2_pulses(bytes: &[u8; 12], n_pre: usize) -> Vec<LevelDuration> {
    let mut bits = Vec::with_capacity(FRAME_BITS);
    bits.push(false); // the start bit (0)
    bits.extend(bytes.iter().flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1 == 1)));
    let level0 = !bits[0]; // the opening long's level, with bit0 = !level0
    let mut out: Vec<LevelDuration> =
        (0..n_pre).map(|k| LevelDuration::new(level0 == (n_pre - k).is_multiple_of(2), SHORT_US)).collect();
    out.push(LevelDuration::new(level0, LONG_US));
    let mut cur = level0; // the level of the last pulse; every pulse flips it
    for &bit in &bits[1..] {
        if bit == cur {
            out.push(LevelDuration::new(!cur, LONG_US));
            cur = !cur;
        } else {
            out.push(LevelDuration::new(!cur, SHORT_US));
            out.push(LevelDuration::new(cur, SHORT_US));
        }
    }
    out.push(LevelDuration::new(false, GAP + 1_020)); // the closing gap
    out
}

/// AES-128 encryption (FIPS-197), encrypt only — the MAC needs no decrypt. A self-contained copy
/// (as in `beninca_arc.rs`), self-contained and not shared with any other protocol. Proven by the FIPS-197 C.1
/// known-answer vector in the tests.
mod aes {
    const SBOX: [u8; 256] = [
        0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76, 0xca, 0x82,
        0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26,
        0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15, 0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96,
        0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0,
        0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb,
        0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf, 0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f,
        0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff,
        0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
        0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32,
        0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d,
        0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, 0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6,
        0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, 0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e,
        0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e,
        0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf, 0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f,
        0xb0, 0x54, 0xbb, 0x16,
    ];
    const RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];

    fn xtime(x: u8) -> u8 {
        (x << 1) ^ ((x >> 7) * 0x1b)
    }

    /// One 16-byte block under a 16-byte key, column-major state (FIPS-197).
    pub(super) fn encrypt(key: &[u8; 16], block: &[u8; 16]) -> [u8; 16] {
        let mut rk = [0u8; 176];
        rk[..16].copy_from_slice(key);
        for i in 4..44 {
            let mut t = [rk[(i - 1) * 4], rk[(i - 1) * 4 + 1], rk[(i - 1) * 4 + 2], rk[(i - 1) * 4 + 3]];
            if i % 4 == 0 {
                t = [SBOX[t[1] as usize] ^ RCON[i / 4 - 1], SBOX[t[2] as usize], SBOX[t[3] as usize], SBOX[t[0] as usize]];
            }
            for j in 0..4 {
                rk[i * 4 + j] = rk[(i - 4) * 4 + j] ^ t[j];
            }
        }
        let mut s = *block;
        for (b, k) in s.iter_mut().zip(&rk[..16]) {
            *b ^= *k;
        }
        for round in 1..=10 {
            for b in s.iter_mut() {
                *b = SBOX[*b as usize];
            }
            s = [
                s[0], s[5], s[10], s[15], s[4], s[9], s[14], s[3], s[8], s[13], s[2], s[7], s[12], s[1], s[6], s[11],
            ];
            if round != 10 {
                for c in 0..4 {
                    let col = [s[c * 4], s[c * 4 + 1], s[c * 4 + 2], s[c * 4 + 3]];
                    for r in 0..4 {
                        s[c * 4 + r] = xtime(col[r]) ^ (xtime(col[(r + 1) % 4]) ^ col[(r + 1) % 4]) ^ col[(r + 2) % 4] ^ col[(r + 3) % 4];
                    }
                }
            }
            for (b, k) in s.iter_mut().zip(&rk[round * 16..round * 16 + 16]) {
                *b ^= *k;
            }
        }
        s
    }
}

/// Synthetic captures for the tests.
#[cfg(test)]
pub(super) mod testing {
    use super::*;
    use crate::protocols::foxfw_common::test_support::{bits_of, pulses_for_bits};

    pub const SHORT: u32 = super::SHORT_US;
    pub const LONG: u32 = super::LONG_US;

    /// Pulses of a frame `button | hop (56 bits) | counter (24) | chk`: `n_pre` 383 µs preamble
    /// pulses (alternating, starting HIGH), the opening long pulse (its level makes the first bit),
    /// the other 95 bits as long pulses or short pairs, then a 2 ms LOW.
    pub fn frame(button: u8, hop: u64, counter: u32, chk: u8, n_pre: usize) -> Vec<LevelDuration> {
        let mut bits = bits_of(u64::from(button), 8);
        bits.extend(bits_of(hop, 56));
        bits.extend(bits_of(u64::from(counter), 24));
        bits.extend(bits_of(u64::from(chk), 8));
        frame_bits(&bits, n_pre)
    }

    /// A frame in the BCM2 key's layout: the real encoder's pulses ([super::bcm2_pulses]).
    pub fn bcm2_frame(bytes: [u8; 12], n_pre: usize) -> Vec<LevelDuration> {
        super::bcm2_pulses(&bytes, n_pre)
    }

    /// Pulses of any bit string, as [frame] builds them.
    pub fn frame_bits(bits: &[bool], n_pre: usize) -> Vec<LevelDuration> {
        // The opening long pulse's level gives bit 0 (`level ? 0 : 1`); the preamble alternates
        // up to it.
        let level = !bits[0];
        let mut out: Vec<LevelDuration> = (0..n_pre)
            .map(|k| LevelDuration::new(level == ((n_pre - k).is_multiple_of(2)), SHORT))
            .collect();
        out.push(LevelDuration::new(level, LONG));
        // After the opening long: `half` false. A state is `half`; a bit is `level ? 0 : 1`.
        let rest = pulses_for_bits(false, level, &bits[1..], SHORT, LONG, |half, lvl, long| {
            let bit = Some(!lvl);
            match (long, half) {
                (true, true) => None,
                (true, false) => Some((false, bit)),
                (false, true) => Some((false, bit)),
                (false, false) => Some((true, None)),
            }
        });
        out.extend(rest);
        out.push(LevelDuration::new(false, 2_000));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::foxfw_common::test_support::*;

    const HOP: u64 = 0x0011_2233_4455_6677;

    /// Made-up frames: (name, pulses, expected (button, data, counter, chk)).
    #[allow(clippy::type_complexity)]
    fn cases() -> Vec<(&'static str, Vec<LevelDuration>, Option<(u8, u64, u32, u8)>)> {
        let mut short_frame = testing::frame(0x0C, HOP, 0x00AB_CDEF, 0x5A, 20);
        let n = short_frame.len();
        short_frame.drain(n - 9..n - 1); // the 96th bit never arrives
        let mut early_gap = testing::frame(0x0C, HOP, 0x00AB_CDEF, 0x5A, 20);
        early_gap[40].duration_us = 1_500; // a gap in the middle of the data window
        let mut bad_open = testing::frame(0x0C, HOP, 0x00AB_CDEF, 0x5A, 20);
        bad_open[20].duration_us = 1_200; // the opening pulse is out of the long window
        vec![
            ("unlock", testing::frame(0x0C, HOP, 0x00AB_CDEF, 0x5A, 20), Some((0x0C, 0x0C11_2233_4455_6677, 0x00AB_CDEF, 0x5A))),
            ("lock", testing::frame(0x0D, 0x0A_0B0C_0D0E_0F10, 0x0000_0102, 0xC3, 33), Some((0x0D, 0x0D0A_0B0C_0D0E_0F10, 0x0102, 0xC3))),
            ("bad_button", testing::frame(0x0B, HOP, 0x00AB_CDEF, 0x5A, 20), None),
            ("hop_zero", testing::frame(0x0C, 0, 0x00AB_CDEF, 0x5A, 20), None),
            ("hop_ff", testing::frame(0x0D, 0x00FF_FFFF_FFFF_FFFF, 0x00AB_CDEF, 0x5A, 20), None),
            ("short_preamble", testing::frame(0x0C, HOP, 0x00AB_CDEF, 0x5A, 19), None),
            ("bad_open", bad_open, None),
            ("early_gap", early_gap, None),
            ("short_frame", short_frame, None),
        ]
    }

    #[test]
    fn key_file_round_trip_and_decode_only() {
        let dec = AudiV3Decoder::new();
        let d = signal(0x0C11_2233_4455_6677, 0x00AB_CDEF, 0x5A, false);
        let text = crate::protocols::flipper_key::testing::assert_round_trip(&dec, &d);
        // the lines the C's serialize writes (checked on the oracle's output)
        assert!(text.contains("Protocol: Audi V3\nBit: 96\nKey: 0C 11 22 33 44 55 66 77\nBtn: 12\nCnt: 11259375\nChk: 90\n"), "{text}");
        let mut wrong_bits = FlipperKey::parse(&text).unwrap();
        wrong_bits.bits = 64;
        assert!(dec.from_flipper_key(&wrong_bits).is_none());
        // The protocol encodes (BCM2 next presses), but a C frame's hop is encrypted: no encode.
        assert!(dec.supports_encoding() && dec.encode(&d, 1).is_none());
    }

    /// 12 bytes in the BCM2 key's layout: `button`, a made-up encrypted part, the counter, the sum.
    fn bcm2_bytes_for(button: u8, counter: u32) -> [u8; 12] {
        let mut b = [button, 0x3C, 0xA5, 0x0F, 0x96, 0x5A, 0xC3, 0, 0, 0, 0, 0];
        b[7..11].copy_from_slice(&counter.to_be_bytes());
        b[11] = b[..11].iter().fold(0u8, |s, &x| s.wrapping_add(x));
        b
    }

    /// A frame in the BCM2 key's layout on air three ways: whole, with its 97th bit lost in the
    /// closing gap (the last bit's second half merged into the closing LOW; a bit 1 ends LOW), and
    /// in the other polarity.
    fn bcm2_wires(bytes: [u8; 12]) -> [(&'static str, Vec<LevelDuration>); 3] {
        let whole = testing::bcm2_frame(bytes, 274);
        let mut lost = whole.clone();
        let gap = lost.pop().unwrap();
        if !lost.last().unwrap().level {
            lost.last_mut().unwrap().duration_us += gap.duration_us;
        } else {
            lost.push(gap);
        }
        let inverted = whole.iter().map(|p| LevelDuration::new(!p.level, p.duration_us)).collect();
        [("whole", whole), ("lost bit", lost), ("inverted", inverted)]
    }

    /// The BCM2 key's frame (module doc): read with its button, counter and sum; also when its
    /// 97th bit is lost in the closing gap, and in the other polarity. A wrong sum is not read, by
    /// this layout or the C's (whose whitelist the first eight bits, 0x05, fail).
    #[test]
    fn the_bcm2_keys_frame_reads_with_its_sum() {
        for (button, counter) in [(0x0A, 0x0004_4769u32), (0x08, 0x0004_47A8), (0x09, 0x0004_47ED)] {
            let bytes = bcm2_bytes_for(button, counter);
            for (case, wire) in bcm2_wires(bytes) {
                let got = decode_all(&mut AudiV3Decoder::new(), &wire);
                assert_eq!(got.len(), 1, "{button:#04x} {case}");
                let d = &got[0];
                assert_eq!((d.button, d.counter, d.crc_valid), (Some(button), Some((counter & 0xFFFF) as u16), true), "{case}");
                assert_eq!(d.data, u64::from_be_bytes(bytes[..8].try_into().unwrap()), "{case}");
                assert_eq!(d.extra, Some((1 << BCM2_FLAG) | (u64::from(counter & 0xFF_FFFF) << 8) | u64::from(bytes[11])));
                assert!(crate::protocols::info::has_checksum_for(NAME, d.data_count_bit, d.extra));
            }
            let mut bad = bytes;
            bad[11] ^= 0x10;
            assert!(decode_all(&mut AudiV3Decoder::new(), &testing::bcm2_frame(bad, 274)).is_empty(), "{button:#04x}");
        }
        assert_eq!([buttons(NAME, 0x0A), buttons(NAME, 0x08), buttons(NAME, 0x09)], [Some("Lock"), Some("Unlock"), Some("Trunk")]);
        // The C's frames keep the C's fields and are not checked.
        let d = signal(0x0C11_2233_4455_6677, 0x00AB_CDEF, 0x5A, false);
        assert!(!d.crc_valid && !crate::protocols::info::has_checksum_for(NAME, d.data_count_bit, d.extra));
    }

    /// The module's byte 0 (module doc): every key number (0..=7), status (0..=3) and button (0..=3)
    /// reads in the BCM2 key's layout, with its sum, named with its key (counted from 1), whole,
    /// with the 97th bit lost and in the other polarity. The six bytes after byte 0 are made up (no
    /// key's MAC); the counter goes up by 3 from frame to frame, as a key's does.
    #[test]
    fn every_key_number_status_and_button_reads_in_the_bcm2_layout() {
        const NAMES: [&str; 4] = ["Unlock", "Trunk", "Lock", "Btn 4"];
        let mut n = 0u32;
        for key in 0..8u8 {
            for status in 0..4u8 {
                for button in 0..4u8 {
                    let b0 = (key << 4) | (status << 2) | button;
                    let counter = 0x0001_0005 + 3 * n;
                    let mut bytes = bcm2_bytes_for(b0, counter);
                    for (i, b) in bytes[1..7].iter_mut().enumerate() {
                        *b = (n as u8).wrapping_mul(37).wrapping_add(0x29 * i as u8) ^ 0x5C;
                    }
                    bytes[11] = bytes[..11].iter().fold(0u8, |s, &x| s.wrapping_add(x));
                    let label = format!("{}, key {}", NAMES[usize::from(button)], key + 1);
                    for (case, wire) in bcm2_wires(bytes) {
                        let got = decode_all(&mut AudiV3Decoder::new(), &wire);
                        assert_eq!(got.len(), 1, "{b0:#04x} {case}");
                        let d = &got[0];
                        assert_eq!((d.button, d.crc_valid), (Some(b0), true), "{b0:#04x} {case}");
                        assert_eq!(d.data, u64::from_be_bytes(bytes[..8].try_into().unwrap()), "{b0:#04x} {case}");
                        assert_eq!(d.extra, Some((1 << BCM2_FLAG) | (u64::from(counter & 0xFF_FFFF) << 8) | u64::from(bytes[11])));
                        assert_eq!(crate::protocols::capture_button_name(NAME, b0, d.data, d.extra), Some(label.as_str()));
                    }
                    n += 1;
                }
            }
        }
    }

    /// Byte 0's bit 7 is never set in a key's frame: such a frame is read in neither layout. Nor is
    /// a frame whose MAC is all 0x00 or all 0xFF, though its sum holds: 12 zero bytes are what a
    /// run of one level reads as (`IMPORTS/OPEL/more/opel_two_button_remote_*_x300.sub` have one).
    #[test]
    fn a_frame_with_bit_7_set_or_a_degenerate_mac_is_not_read() {
        let degenerate = |fill: u8| {
            let mut b = bcm2_bytes_for(0x0A, 0x0001_0005);
            b[1..7].fill(fill);
            b[11] = b[..11].iter().fold(0u8, |s, &x| s.wrapping_add(x));
            b
        };
        for (what, bytes) in [
            ("bit 7, 0x88", bcm2_bytes_for(0x88, 0x0001_0005)),
            ("bit 7, 0x9A", bcm2_bytes_for(0x9A, 0x0001_0005)),
            ("bit 7, 0xFB", bcm2_bytes_for(0xFB, 0x0001_0005)),
            ("all zero", [0u8; 12]),
            ("MAC all 0x00", degenerate(0x00)),
            ("MAC all 0xFF", degenerate(0xFF)),
        ] {
            for (case, wire) in bcm2_wires(bytes) {
                assert!(decode_all(&mut AudiV3Decoder::new(), &wire).is_empty(), "{what} {case}");
            }
        }
    }

    /// The C's two buttons are a second key's frame read from its start bit (module doc): key 2's
    /// Unlock (byte 0 0x18) and Lock (0x1A) begin, from the start bit, with 0x0C and 0x0D. KAT reads
    /// them whole, with their sum and their key. `KAT_SYNTH_DIR` dumps them for the FoxFW oracle.
    #[test]
    fn the_cs_two_buttons_are_a_second_keys_unlock_and_lock() {
        for (b0, c_button, label) in [(0x18u8, 0x0Cu8, "Unlock, key 2"), (0x1A, 0x0D, "Lock, key 2")] {
            let bytes = bcm2_bytes_for(b0, 0x0001_0005);
            let wire = testing::bcm2_frame(bytes, 274);
            dump(&format!("audi_v3_bcm2_{b0:02x}.sub"), 868_350_000, "FuriHalSubGhzPreset2FSKDev476Async", &wire);
            // The first eight bits from the start bit: the start bit (0), then byte 0's top seven.
            assert_eq!(b0 >> 1, c_button);
            assert_eq!(buttons(NAME, c_button), Some(&label[..label.find(',').unwrap()]));
            let got = decode_all(&mut AudiV3Decoder::new(), &wire);
            assert_eq!(got.len(), 1, "{label}");
            assert!(got[0].crc_valid, "{label}");
            assert_eq!(crate::protocols::capture_button_name(NAME, b0, got[0].data, got[0].extra), Some(label));
        }
    }

    /// A key file keeps the layout: a frame of key 3 comes back read in the BCM2 key's layout with
    /// its sum; a key file of the C's whose check byte happens to be the sum stays the C's frame.
    #[test]
    fn key_files_keep_the_layout() {
        let dec = AudiV3Decoder::new();
        let bytes = bcm2_bytes_for(0x2A, 0x0001_0005);
        let d = decode_all(&mut AudiV3Decoder::new(), &testing::bcm2_frame(bytes, 274)).remove(0);
        let back = dec.from_flipper_key(&dec.to_flipper_key(&d).unwrap()).unwrap();
        assert_eq!((back.data, back.extra, back.button, back.crc_valid), (d.data, d.extra, Some(0x2A), true));
        assert_eq!(crate::protocols::capture_button_name(NAME, 0x2A, back.data, back.extra), Some("Lock, key 3"));
        let mut c = pack(0x0C11_2233_4455_6677, 0x00AB_CDEF, 0);
        c[11] = c[..11].iter().fold(0u8, |s, &x| s.wrapping_add(x));
        let back = dec.from_flipper_key(&dec.to_flipper_key(&signal(0x0C11_2233_4455_6677, 0x00AB_CDEF, c[11], false)).unwrap()).unwrap();
        assert!(!back.crc_valid && back.extra.is_some_and(|e| (e >> BCM2_FLAG) & 1 == 0));
        assert_eq!(crate::protocols::capture_button_name(NAME, 0x0C, back.data, back.extra), Some("Unlock"));
    }

    /// The key specialist's 2008–2015 Audi A4 (BCM2) recordings: every press reads with the button
    /// its recording is named after and a valid sum, and the counter goes up by 3 with every press
    /// and on from one recording to the next (presses between them were not recorded).
    #[test]
    fn the_a4_bcm2_recordings_read_as_their_buttons() {
        let mut last: Option<u32> = None;
        for (rel, button, presses) in [
            ("VAG/more/2008-2015_audi_a4_bcm2_lock.sub", 0x0A, 21),
            ("VAG/more/2008-2015_audi_a4_bcm2_unlock.sub", 0x08, 20),
            ("VAG/more/2008-2015_audi_a4_bcm2_trunk.sub", 0x09, 20),
        ] {
            let path = std::path::Path::new("IMPORTS").join(rel);
            if !path.exists() {
                eprintln!("Skip: {path:?} not found (run from crate root)");
                return;
            }
            let (_, pairs) = crate::export::flipper::import_sub_raw(&path).unwrap();
            let mut dec = AudiV3Decoder::new();
            let got: Vec<DecodedSignal> = pairs.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect();
            assert_eq!(got.len(), presses, "{rel}");
            assert!(got.iter().all(|d| d.button == Some(button) && d.crc_valid), "{rel}");
            let counters: Vec<u32> =
                got.iter().map(|d| ((d.data as u32 & 0xFF) << 24) | (d.extra.unwrap() >> 8) as u32 & 0xFF_FFFF).collect();
            assert!(counters.windows(2).all(|w| w[1] == w[0] + 3), "{rel}: {counters:?}");
            assert!(last.is_none_or(|prev| counters[0] > prev), "{rel}");
            last = counters.last().copied();
        }
    }

    use crate::protocols::keys::AudiBcm2Car;

    /// A made-up car key. `key_ids[1]` is the ID the MAC vector below uses (key number 1).
    fn demo_car() -> AudiBcm2Car {
        AudiBcm2Car {
            name: "demo".into(),
            key: [0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F],
            car_value: [0xDE, 0xAD, 0xBE, 0xEF],
            key_ids: vec![[9, 9, 9, 9], [1, 2, 3, 4]],
        }
    }

    #[test]
    fn aes_matches_fips197() {
        // FIPS-197 Appendix C.1.
        let key = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
        let pt = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        assert_eq!(aes::encrypt(&key, &pt), [0x69, 0xC4, 0xE0, 0xD8, 0x6A, 0x7B, 0x04, 0x30, 0xD8, 0xCD, 0xB7, 0x80, 0x70, 0xB4, 0xC5, 0x5A]);
    }

    #[test]
    fn the_mac_matches_the_research_encoder() {
        // The byte-for-byte MAC the Python `encoder.py` built and KAT decoded (keys 1/2/3 frames).
        let car = demo_car();
        assert_eq!(authenticator(&car.key, &car.car_value, 0x0001_0005, &car.key_ids[1], 2), [0x91, 0x9A, 0xAC, 0x50, 0x13, 0xC8]);
    }

    /// With a car key loaded, a BCM2 frame is `encoder_capable`, and "Send next code" makes the key's
    /// next press (counter + 3, a fresh MAC the same key reproduces); a chosen button re-aims it.
    #[test]
    fn encodes_the_next_press_for_a_loaded_car() {
        let _lk = crate::protocols::keeloq_common::KEYSTORE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let car = demo_car();
        crate::protocols::keys::get_keystore_mut().load_audi_bcm2_keys(vec![car.clone()]);

        // A capture: key number 1, status 2 (clear), button 2 (Lock), counter 0x00010005.
        let mac = authenticator(&car.key, &car.car_value, 0x0001_0005, &car.key_ids[1], 2);
        let capture = build_bcm2(1, 2, 2, 0x0001_0005, &mac);
        let d = decode_all(&mut AudiV3Decoder::new(), &testing::bcm2_frame(capture, 274)).remove(0);
        assert_eq!(d.button, Some(0x1A));
        assert!(d.encoder_capable, "a loaded car key makes the capture encodable");
        assert_eq!(crate::protocols::capture_button_name(NAME, 0x1A, d.data, d.extra), Some("Lock, key 2"));

        // "Send next code": same key and button, counter + 3, and the car key reproduces its MAC.
        let next = decode_all(&mut AudiV3Decoder::new(), &AudiV3Decoder::new().encode_next(&d).expect("next press")).remove(0);
        assert_eq!(next.button, Some(0x1A));
        let full = |s: &DecodedSignal| ((s.data as u32 & 0xFF) << 24) | (s.extra.unwrap() >> 8) as u32 & 0xFF_FFFF;
        assert_eq!(full(&next), 0x0001_0008);
        assert!(next.crc_valid && next.encoder_capable);
        assert_eq!(bcm2_bytes_of(&next).and_then(|b| car_for(&b, std::slice::from_ref(&car)).map(|_| ())), Some(()));

        // A chosen button (Unlock = KAT 0x02) re-aims the same key: byte 0 0x18.
        let unlock = decode_all(&mut AudiV3Decoder::new(), &AudiV3Decoder::new().encode(&d, 0x02).expect("unlock")).remove(0);
        assert_eq!(unlock.button, Some(0x18));
        assert_eq!(crate::protocols::capture_button_name(NAME, 0x18, unlock.data, unlock.extra), Some("Unlock, key 2"));

        crate::protocols::keys::get_keystore_mut().load_audi_bcm2_keys(Vec::new());
    }

    /// Without a loaded car key the encoder is inert: a BCM2 frame reads but is not encodable, and a
    /// car key whose MAC does not match the capture does not make it encodable either.
    #[test]
    fn the_encoder_is_inert_without_a_matching_key() {
        let _lk = crate::protocols::keeloq_common::KEYSTORE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crate::protocols::keys::get_keystore_mut().load_audi_bcm2_keys(Vec::new());
        let bytes = bcm2_bytes_for(0x1A, 0x0001_0005); // a made-up MAC, no car loaded
        let d = decode_all(&mut AudiV3Decoder::new(), &testing::bcm2_frame(bytes, 274)).remove(0);
        assert!(!d.encoder_capable && AudiV3Decoder::new().encode_next(&d).is_none());
        // A real car key, but its MAC does not match this capture's made-up bytes.
        crate::protocols::keys::get_keystore_mut().load_audi_bcm2_keys(vec![demo_car()]);
        let d = decode_all(&mut AudiV3Decoder::new(), &testing::bcm2_frame(bytes, 274)).remove(0);
        assert!(!d.encoder_capable && AudiV3Decoder::new().encode(&d, 0x01).is_none());
        // The C's frames never encode (encrypted hop, Replay only).
        let c = signal(0x0C11_2233_4455_6677, 0x00AB_CDEF, 0x5A, false);
        assert!(AudiV3Decoder::new().encode_next(&c).is_none());
        crate::protocols::keys::get_keystore_mut().load_audi_bcm2_keys(Vec::new());
    }

    #[test]
    fn synthetic_frames_match_the_foxfw_oracle() {
        for (name, wire, want) in cases() {
            dump(&format!("audi_v3_{name}.sub"), 433_920_000, "FuriHalSubGhzPreset2FSKDev476Async", &wire);
            let got = decode_all(&mut AudiV3Decoder::new(), &wire);
            eprintln!("CASE {name}: {:?}", got.iter().map(|d| (format!("{:016X}", d.data), d.extra, d.button, d.counter)).collect::<Vec<_>>());
            assert_eq!(got.len(), usize::from(want.is_some()), "{name}");
            if let Some((button, data, counter, chk)) = want {
                let d = &got[0];
                assert_eq!(d.button, Some(button), "{name}");
                assert_eq!(d.data, data, "{name}");
                assert_eq!(d.extra, Some((u64::from(counter) << 8) | u64::from(chk)), "{name}");
                assert_eq!(d.counter, Some((counter & 0xFFFF) as u16), "{name}");
                assert_eq!(d.data_count_bit, 96);
            }
        }
    }
}
