//! BelKorn: a 118-bit rolling code with no encryption — gate / barrier remotes sold as Korn, BelGate,
//! Furniteh and other rebrands (the frame carries no manufacturer field, so every one reads as
//! "BelKorn").
//!
//! References: RogueMaster `lib/subghz/protocols/belkorn.c` (primary; added 2026-11-13 from
//! Unleashed, read at `origin/420`) and FoxFW `lib/subghz/protocols/belkorn.c` with its SD encoder
//! plugin `applications/main/subghz/protocols_ext/enc_belkorn.c` (FoxFW v2.1.1, 2026-11-22). The two
//! are function for function the same air format, frame, check, counter law and upload; FoxFW moves
//! the encoder into the plugin, formats the display differently and has no "Add Manually" entry
//! (RogueMaster's `subghz_protocol_belkorn_create_data`, ported here as [create_data]).
//!
//! **Air format** (`subghz_protocol_decoder_belkorn_feed`): one bit per pulse, on marks and spaces
//! alike — a pulse within 180 µs of 481 µs (te_short) is a 0, within 180 µs of 962 µs (te_long) a 1.
//! A frame starts after a LOW of at least 2500 µs; bit i must be HIGH when i is even and LOW when it
//! is odd (a level out of turn, or a pulse in neither window, drops the frame and the decoder waits
//! for the next long LOW — the breaking pulse itself never restarts it). The first 64 bits go to
//! `data`, bits 64..117 to `data_2`, and the frame is reported at the 118th bit. The remote then
//! sends a trailing mark (either width) and a gap; the C's encoder sends te_short and 16 te_short.
//!
//! **Frame** (118 bits, MSB first): sync 0x801 (12) | serial high nibble (4) | serial (32) |
//! L24 low byte (8) | L24 middle byte (8) | L24 high byte (8) | b3 | b4 | b5 | b6 | c (8 each) |
//! button (6, one-hot). The press counter `n` is 16 bits: `c` is its low byte; its high byte (the
//! "lap") is b6 when `c` is even (format A) and b5 when `c` is odd (format B).
//!
//! **Counter law** (`subghz_protocol_belkorn_make_frame`, measured by the reference's authors on
//! recordings): with `k = n >> 1` and `lap = n >> 8`,
//! - format A (`c` even): L24 = origin L24 + k·0x810 + lap·0x1C282C, b3 = origin b3 + k·0x10,
//!   b4 = c, b5 = b6 = lap;
//! - format B (`c` odd): L24 = origin L24 + 0x0748CC + k·0x1028 − wraps·0x80800 + jitter(lap, c),
//!   (b3, b4) as a 16-bit little-endian value = (0x5A << 8 | (origin b3 − 0x30)) + wraps·0x108,
//!   b5 = lap, b6 = c + 0xBA, where `wraps` counts the wraps of b6 (`lap + (c > 0x45)`) and the
//!   jitter is the reference's periodic offset over the set bits of `lap` (rows for bits 0..2
//!   measured, bits 3..7 extrapolated by the same law: they need a remote past 2048 presses).
//!
//! All arithmetic as the C's: L24 is 24 bits (mod 2^24), b3/b4 bytes wrap, the jitter's
//! `(c >> 1) − 3` is taken as a u16. The per-remote origin (the serial's high nibble and format A's
//! L24 and b3 at counter 0) is solved back from any frame
//! (`subghz_protocol_belkorn_check_remote_controller`), so one capture is enough to make the next
//! press.
//!
//! **Gating** (`subghz_protocol_belkorn_frame_valid`): a 118-bit frame is reported when its top 12
//! bits are the sync 0x801 and its button is one non-zero bit. There is no checksum
//! (`NO_CHECKSUM`): `crc_valid` stays false. KAT does not add the redundancy the counter law implies
//! (format A's b4 = c and b5 = b6, format B's b6 = c + 0xBA) as a gate, since the reference does not.
//!
//! **KAT fields**: `data` = the C's `generic.data` (frame bits 0..63), `extra` = `generic.data_2`
//! (bits 64..117, right-aligned), `data_count_bit` = 118, `serial` = the C's `generic.serial` (the
//! low 32 bits of the 36-bit serial; its high nibble is frame bits 12..15, in `data`), `button` = the
//! 6-bit one-hot code, `counter` = `n`.
//!
//! **Encoder** (`subghz_protocol_belkorn_encoder_prepare_packets` + `get_upload`): the frame is
//! analysed again, the counter goes up by one (`& 0xFFFF`: 0xFFFF → 0), the button is the requested
//! native code when it is one bit of the 6-bit field (the C's custom buttons cycle 0x01, 0x02,
//! 0x04, 0x20; its button editor takes any value, so any single bit is sent) and otherwise the
//! captured button, and the frame for the new counter is built from the solved origin (format A or
//! B by the counter's parity). Upload: the frame three times, each as 118 pulses (HIGH first, 481 /
//! 962 µs) + HIGH 481 + LOW 7696; `encoder.repeat` = 3, so `encode` returns three uploads, nine
//! frames, as the reference sends a press (the remote's ~9). Like the C, it re-derives the whole
//! frame (sync included) and does not require the input to pass the check.
//!
//! **Key file** (`subghz_protocol_decoder_common_serialize_data_2`): `Protocol: BelKorn`,
//! `Bit: 118`, `Key` (8 bytes, `data`), `Data` (8 bytes, `data_2`); the C's deserialize requires
//! `Bit: 118` and both lines. KAT also requires the frame to pass the check, as it does for every
//! key file it loads ("does not validate" → not loaded).

use super::common::DecodedSignal;
use super::flipper_key::FlipperKey;
use super::g5_common::{hex8_field, key8, next_counter, push};
use super::{ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "BelKorn",
    decoder: "BelKorn",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code (plaintext counter)",
    make: "BelKorn",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "BelKorn";
const TE_SHORT: u32 = 481;
const TE_LONG: u32 = 962;
const TE_DELTA: u32 = 180;
const MIN_COUNT_BIT: usize = 118;
/// The LOW that starts a frame.
const GAP_MIN: u32 = 2500;
const SYNC: u64 = 0x801;

const L24_MASK: u32 = 0xFF_FFFF;
/// L24 per counter pair, format A / B.
const STEP_A: u32 = 0x810;
const STEP_B: u32 = 0x1028;
/// L24 per counter byte wrap (format A) / per b6 wrap (format B).
const WRAP_A: u32 = 0x1C_282C;
const WRAP_B: u32 = 0x8_0800;
/// b3 per counter pair, format A.
const B3_STEP_A: u32 = 0x10;
/// (b3, b4) as 16-bit little-endian per b6 wrap, format B.
const B34_WRAP_B: u32 = 0x108;
/// Format B: b6 = counter low byte + this bias.
const B6_BIAS: u8 = 0xBA;
/// Format B: b4 at counter 0, protocol wide.
const B4_BASE: u16 = 0x5A;
/// L24 at counter 0: format B minus format A.
const L24_AB: u32 = 0x07_48CC;
/// b3 at counter 0: format A minus format B.
const B3_AB: u8 = 0x30;
/// Low byte of L24 at counter 0, protocol wide ("Add Manually" starts from it).
const L24_ORIGIN: u32 = 0xE0;
/// The C's custom buttons (`subghz_protocol_belkorn_btn_codes`).
pub(super) const BUTTONS: [u8; 4] = [0x01, 0x02, 0x04, 0x20];
/// The frame's copies in one upload (`BELKORN_FRAME_REPEAT`) and the C's `encoder.repeat`.
const FRAME_REPEAT: usize = 3;
const TX_REPEAT: usize = 3;
/// Levels of one frame in the upload: 118 bits, the trailing mark and the gap.
const FRAME_LEVELS: usize = MIN_COUNT_BIT + 2;

/// The per-remote constants (`BelKornOrigin`): the serial's top nibble and format A's L24 and b3
/// at counter 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Origin {
    serial_hi: u8,
    l24: u32,
    b3: u8,
}

/// `belkorn_get`: `len` bits at frame position `pos` (MSB first; 0..63 in `data`, 64..117 in
/// `data_2`).
fn get(data: u64, data_2: u64, pos: u32, len: u32) -> u32 {
    let mask = (1u64 << len) - 1;
    let v = if pos + len <= 64 { data >> (64 - pos - len) } else { data_2 >> (118 - pos - len) };
    (v & mask) as u32
}

/// `belkorn_put`: the frame's two words from its fields.
#[allow(clippy::too_many_arguments)] // the C's field list, one argument per frame field
fn put(serial_hi: u8, serial: u32, l24: u32, b3: u8, b4: u8, b5: u8, b6: u8, c: u8, btn: u8) -> (u64, u64) {
    let data = (SYNC << 52)
        | (u64::from(serial_hi & 0xF) << 48)
        | (u64::from(serial) << 16)
        | (u64::from(l24 & 0xFF) << 8)
        | u64::from((l24 >> 8) & 0xFF);
    let data_2 = (u64::from((l24 >> 16) & 0xFF) << 46)
        | (u64::from(b3) << 38)
        | (u64::from(b4) << 30)
        | (u64::from(b5) << 22)
        | (u64::from(b6) << 14)
        | (u64::from(c) << 6)
        | u64::from(btn & 0x3F);
    (data, data_2)
}

/// `subghz_protocol_belkorn_b6_wraps`: format B's b6 is the counter's low byte plus a bias, so it
/// wraps once per lap plus once more inside the lap in progress.
fn b6_wraps(n: u16) -> u16 {
    (n >> 8) + u16::from((n & 0xFF) > 0x45)
}

/// `subghz_protocol_belkorn_jitter`: format B's periodic L24 offset, additive over the set bits
/// of the counter's high byte, each gated by one bit of the low byte.
fn jitter(lap: u8, c: u8) -> u32 {
    // C: (uint16_t)((c >> 1) - 3), negative for c < 6
    let v = (i32::from(c >> 1) - 3) as u16;
    let mut jitter = u32::from(lap) << 21;
    for i in 0..8u32 {
        if (lap >> i) & 1 == 0 {
            continue;
        }
        if (v >> (i + 1)) & 1 == 0 {
            jitter = jitter.wrapping_add(257 << (5 + i));
        }
        if i != 0 && (v >> (i - 1)) & 1 == 0 {
            jitter = jitter.wrapping_add(1 << (3 + i));
        }
    }
    jitter
}

/// `subghz_protocol_belkorn_make_frame`: the frame of press `n` of the remote `origin`.
fn make_frame(origin: &Origin, serial: u32, n: u16, btn: u8) -> (u64, u64) {
    let c = n as u8;
    let lap = (n >> 8) as u8;
    let k = u32::from(n >> 1);
    if c & 1 == 1 {
        let wraps = b6_wraps(n);
        let l24 = origin
            .l24
            .wrapping_add(L24_AB)
            .wrapping_add(k.wrapping_mul(STEP_B))
            .wrapping_sub(u32::from(wraps).wrapping_mul(WRAP_B))
            .wrapping_add(jitter(lap, c))
            & L24_MASK;
        let b34 = ((B4_BASE << 8) | u16::from(origin.b3.wrapping_sub(B3_AB)))
            .wrapping_add((u32::from(wraps).wrapping_mul(B34_WRAP_B)) as u16);
        put(origin.serial_hi, serial, l24, b34 as u8, (b34 >> 8) as u8, lap, c.wrapping_add(B6_BIAS), c, btn)
    } else {
        let l24 = origin.l24.wrapping_add(k.wrapping_mul(STEP_A)).wrapping_add(u32::from(lap).wrapping_mul(WRAP_A))
            & L24_MASK;
        let b3 = origin.b3.wrapping_add(k.wrapping_mul(B3_STEP_A) as u8);
        put(origin.serial_hi, serial, l24, b3, c, lap, lap, c, btn)
    }
}

/// What `subghz_protocol_belkorn_check_remote_controller` reads from a frame: serial, button and
/// press counter, and the remote's origin solved back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Fields {
    serial: u32,
    btn: u8,
    n: u16,
    origin: Origin,
}

fn analyze(data: u64, data_2: u64) -> Fields {
    let serial_hi = get(data, data_2, 12, 4) as u8;
    let serial = get(data, data_2, 16, 32);
    let btn = get(data, data_2, 112, 6) as u8;
    let l24 = get(data, data_2, 48, 8) | (get(data, data_2, 56, 8) << 8) | (get(data, data_2, 64, 8) << 16);
    let b3 = get(data, data_2, 72, 8) as u8;
    let b4 = get(data, data_2, 80, 8) as u8;
    let b5 = get(data, data_2, 88, 8) as u8;
    let b6 = get(data, data_2, 96, 8) as u8;
    let c = get(data, data_2, 104, 8) as u8;
    let n = (u16::from(if c & 1 == 1 { b5 } else { b6 }) << 8) | u16::from(c);
    let k = u32::from(n >> 1);
    let lap = n >> 8;
    let origin = if c & 1 == 1 {
        let wraps = b6_wraps(n);
        let l24 = l24
            .wrapping_sub(k.wrapping_mul(STEP_B))
            .wrapping_add(u32::from(wraps).wrapping_mul(WRAP_B))
            .wrapping_sub(jitter(lap as u8, c))
            .wrapping_sub(L24_AB)
            & L24_MASK;
        let b34 = u32::from(b3) | (u32::from(b4) << 8);
        let b3 = (b34.wrapping_sub(u32::from(wraps).wrapping_mul(B34_WRAP_B)) as u8).wrapping_add(B3_AB);
        Origin { serial_hi, l24, b3 }
    } else {
        let l24 =
            l24.wrapping_sub(k.wrapping_mul(STEP_A)).wrapping_sub(u32::from(lap).wrapping_mul(WRAP_A)) & L24_MASK;
        let b3 = b3.wrapping_sub(k.wrapping_mul(B3_STEP_A) as u8);
        Origin { serial_hi, l24, b3 }
    };
    Fields { serial, btn, n, origin }
}

/// `subghz_protocol_belkorn_frame_valid`: the sync, and a one-hot button.
fn frame_valid(data: u64, data_2: u64) -> bool {
    let btn = (data_2 & 0x3F) as u8;
    data >> 52 == SYNC && btn.is_power_of_two()
}

fn signal(data: u64, data_2: u64) -> DecodedSignal {
    let f = analyze(data, data_2);
    DecodedSignal {
        serial: Some(f.serial),
        button: Some(f.btn),
        counter: Some(f.n),
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: Some(data_2),
        protocol_display_name: None,
    }
}

/// `subghz_protocol_belkorn_create_data` (RogueMaster's "Add Manually"): a 36-bit serial, the
/// button and counter as given (not advanced), the origin the protocol-wide L24 low byte 0xE0 with
/// b3 = 0 ("a receiver in learn mode takes whatever it is handed"); `Bit: 118`, `Key`, `Data`.
pub(super) fn create_data(serial: u64, btn: u8, cnt: u16) -> FlipperKey {
    let origin = Origin { serial_hi: ((serial >> 32) & 0xF) as u8, l24: L24_ORIGIN, b3: 0 };
    let (data, data_2) = make_frame(&origin, serial as u32, cnt, btn);
    let mut k = FlipperKey::for_protocol(NAME, MIN_COUNT_BIT, data);
    k.push_hex("Data", data_2, 8);
    k
}

/// One copy of the C's upload: the frame `FRAME_REPEAT` times.
fn upload_once(data: u64, data_2: u64, out: &mut Vec<LevelDuration>) {
    for _ in 0..FRAME_REPEAT {
        for i in 0..MIN_COUNT_BIT {
            let set = if i < 64 { (data >> (63 - i)) & 1 == 1 } else { (data_2 >> (117 - i)) & 1 == 1 };
            push(out, i.is_multiple_of(2), if set { TE_LONG } else { TE_SHORT });
        }
        // The trailing mark (the one pulse the remote sends at either width) and the gap.
        push(out, true, TE_SHORT);
        push(out, false, TE_SHORT * 16);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    Data,
}

pub struct BelKornDecoder {
    step: Step,
    decode_data: u64,
    decode_count_bit: usize,
    /// The C's `generic.data` while receiving: the first 64 bits, stored at the 64th.
    data: u64,
}

impl BelKornDecoder {
    pub fn new() -> Self {
        Self { step: Step::Reset, decode_data: 0, decode_count_bit: 0, data: 0 }
    }
}

impl Default for BelKornDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for BelKornDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE_SHORT, te_long: TE_LONG, te_delta: TE_DELTA, min_count_bit: MIN_COUNT_BIT }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration >= GAP_MIN {
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::Data;
                }
            }
            Step::Data => {
                // Bits alternate mark and space: a level out of turn breaks the frame.
                if level != self.decode_count_bit.is_multiple_of(2) {
                    self.step = Step::Reset;
                    return None;
                }
                let bit = if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    0
                } else if duration_diff!(duration, TE_LONG) < TE_DELTA {
                    1
                } else {
                    self.step = Step::Reset;
                    return None;
                };
                self.decode_data = (self.decode_data << 1) | bit;
                self.decode_count_bit += 1;
                if self.decode_count_bit == 64 {
                    self.data = self.decode_data;
                    self.decode_data = 0;
                } else if self.decode_count_bit == MIN_COUNT_BIT {
                    self.step = Step::Reset;
                    let data_2 = self.decode_data;
                    if frame_valid(self.data, data_2) {
                        return Some(signal(self.data, data_2));
                    }
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let f = analyze(decoded.data, decoded.extra?);
        let btn = if button.is_power_of_two() && button <= 0x20 { button } else { f.btn };
        let n = next_counter(u32::from(f.n), 0xFFFF) as u16;
        let (data, data_2) = make_frame(&f.origin, f.serial, n, btn);
        let mut out = Vec::with_capacity(TX_REPEAT * FRAME_REPEAT * FRAME_LEVELS);
        for _ in 0..TX_REPEAT {
            upload_once(data, data_2, &mut out);
        }
        Some(out)
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        // `subghz_protocol_decoder_belkorn_deserialize`: Bit 118, Key and Data (8 bytes each).
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        let (data, data_2) = (key8(key)?, hex8_field(key, "Data")?);
        frame_valid(data, data_2).then(|| signal(data, data_2))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let mut k = FlipperKey::new(NAME, MIN_COUNT_BIT, decoded.data, 0, "");
        k.push_hex("Data", decoded.extra?, 8);
        Some(k)
    }
}

/// A key file for a random valid frame (oracle driver fixtures): a random origin and serial, any
/// counter (both formats, every lap, the b6 wrap at c = 0x45/0x46 and 0xFFFF among them) and any
/// one-hot button.
#[cfg(test)]
pub(super) fn fixture_key(r: u64) -> FlipperKey {
    let origin = Origin { serial_hi: (r >> 60) as u8 & 0xF, l24: (r >> 36) as u32 & L24_MASK, b3: (r >> 28) as u8 };
    let n = match r & 0x7 {
        0 => 0xFFFF,
        1 => 0x0045 | ((r >> 8) as u16 & 0xFF00),
        2 => 0x0046 | ((r >> 8) as u16 & 0xFF00),
        _ => (r >> 8) as u16,
    };
    let btn = 1 << ((r >> 24) % 6);
    let serial = (r as u32).rotate_left(13) ^ (r >> 32) as u32;
    let (data, data_2) = make_frame(&origin, serial, n, btn);
    BelKornDecoder::new().to_flipper_key(&signal(data, data_2)).unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    fn decode_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = BelKornDecoder::new();
        signal.iter().filter_map(|ld| dec.feed(ld.level, ld.duration_us)).collect()
    }

    /// A press as the remote sends it: a gap, then the frame (with its trailing mark and gap) the
    /// upload's three times.
    fn press(data: u64, data_2: u64) -> Vec<LevelDuration> {
        let mut s = vec![LevelDuration::new(false, 10_000)];
        upload_once(data, data_2, &mut s);
        s
    }

    const ORIGIN: Origin = Origin { serial_hi: 0x9, l24: 0x3A_51E0, b3: 0x7C };

    #[test]
    fn frame_fields_round_trip() {
        let (data, data_2) = put(0xA, 0x1234_5678, 0xABCDEF, 0x11, 0x22, 0x33, 0x44, 0x55, 0x20);
        assert_eq!(data >> 52, SYNC);
        assert_eq!(get(data, data_2, 12, 4), 0xA);
        assert_eq!(get(data, data_2, 16, 32), 0x1234_5678);
        // L24 is sent low byte first.
        assert_eq!((get(data, data_2, 48, 8), get(data, data_2, 56, 8), get(data, data_2, 64, 8)), (0xEF, 0xCD, 0xAB));
        let bytes: Vec<u32> = (0..5).map(|i| get(data, data_2, 72 + 8 * i, 8)).collect();
        assert_eq!(bytes, [0x11, 0x22, 0x33, 0x44, 0x55]);
        assert_eq!(get(data, data_2, 112, 6), 0x20);
        assert!(data_2 < 1 << 54);
    }

    /// The origin solved from any frame is the one that made it, and so is the counter, in both
    /// formats and every lap (all 65536 counters).
    #[test]
    fn origin_and_counter_solve_back_from_every_frame() {
        for n in 0..=0xFFFFu16 {
            let (data, data_2) = make_frame(&ORIGIN, 0x0BAD_F00D, n, 0x04);
            let f = analyze(data, data_2);
            assert_eq!((f.n, f.origin, f.serial, f.btn), (n, ORIGIN, 0x0BAD_F00D, 0x04), "n = {n:#06x}");
        }
    }

    #[test]
    fn counter_law_by_format() {
        // Format A (even c): b4 = c, b5 = b6 = lap, b3 steps 0x10 per pair.
        let (data, data_2) = make_frame(&ORIGIN, 1, 0x0204, 0x01);
        let byte = |pos| get(data, data_2, pos, 8);
        assert_eq!((byte(80), byte(88), byte(96), byte(104)), (0x04, 0x02, 0x02, 0x04));
        assert_eq!(byte(72), u32::from(ORIGIN.b3.wrapping_add((0x0102u32 * 0x10) as u8)));
        // Format B (odd c): b5 = lap, b6 = c + 0xBA, b4 from the protocol-wide 0x5A.
        let (data, data_2) = make_frame(&ORIGIN, 1, 0x0001, 0x01);
        let byte = |pos| get(data, data_2, pos, 8);
        assert_eq!((byte(88), byte(96), byte(104)), (0x00, 0xBB, 0x01));
        assert_eq!(byte(80), 0x5A);
        assert_eq!(b6_wraps(0x0045), 0);
        assert_eq!(b6_wraps(0x0046), 1);
        assert_eq!(b6_wraps(0x0345), 3);
        // The jitter's (c >> 1) - 3 wraps as a u16 below c = 6 (c = 1: 0xFFFD, bit 1 clear, so the
        // lap bit 0 term is added); c = 11 gives v = 2, bit 1 set, so it is not.
        assert_eq!(jitter(0, 5), 0);
        assert_eq!(jitter(1, 1), (1 << 21) + (257 << 5));
        assert_eq!(jitter(1, 11), 1 << 21);
        assert_eq!(jitter(1, 7), (1 << 21) + (257 << 5));
        // Lap bit 1 (v = 0): both of its terms.
        assert_eq!(jitter(2, 7), (2 << 21) + (257 << 6) + (1 << 4));
    }

    #[test]
    fn decodes_both_formats() {
        for (n, btn) in [(0x0000u16, 0x01u8), (0x0005, 0x02), (0x0146, 0x04), (0x0145, 0x20), (0xFFFF, 0x08)] {
            let (data, data_2) = make_frame(&ORIGIN, 0x1234_5678, n, btn);
            let got = decode_all(&press(data, data_2));
            assert_eq!(got.len(), FRAME_REPEAT, "n = {n:#x}");
            for d in &got {
                assert_eq!((d.serial, d.button, d.counter), (Some(0x1234_5678), Some(btn), Some(n)));
                assert_eq!((d.data, d.extra, d.data_count_bit), (data, Some(data_2), 118));
                assert!(!d.crc_valid && d.encoder_capable);
            }
        }
    }

    #[test]
    fn rejects_bad_frames() {
        let (data, data_2) = make_frame(&ORIGIN, 0x1234_5678, 0x10, 0x02);
        // Another sync.
        assert!(decode_all(&press(data ^ 1 << 60, data_2)).is_empty());
        // No button, or two.
        assert!(decode_all(&press(data, data_2 & !0x3F)).is_empty());
        assert!(decode_all(&press(data, data_2 | 0x03)).is_empty());
        // A pulse in neither window (700 µs) breaks the frame; the next one, after its own gap,
        // still decodes.
        let mut s = press(data, data_2);
        s[40].duration_us = 700;
        assert_eq!(decode_all(&s).len(), FRAME_REPEAT - 1);
        // One pulse dropped (117 bits, levels out of turn from there).
        let mut s = vec![LevelDuration::new(false, 10_000)];
        upload_once(data, data_2, &mut s);
        s.remove(30);
        assert!(decode_all(&s[..118]).is_empty());
        // No gap before the first frame: only the frames after a long LOW count.
        let mut s = Vec::new();
        upload_once(data, data_2, &mut s);
        assert_eq!(decode_all(&s).len(), FRAME_REPEAT - 1);
        // A gap under 2500 µs does not start a frame.
        let mut s = press(data, data_2);
        s[0].duration_us = 2400;
        assert_eq!(decode_all(&s).len(), FRAME_REPEAT - 1);
    }

    #[test]
    fn encode_next_press_and_buttons() {
        let dec = BelKornDecoder::new();
        let (data, data_2) = make_frame(&ORIGIN, 0x0055_AA00, 0x0144, 0x02);
        let captured = signal(data, data_2);
        for (button, want) in [(0x02u8, 0x02u8), (0x01, 0x01), (0x04, 0x04), (0x20, 0x20), (0x10, 0x10), (0x03, 0x02), (0x40, 0x02), (0, 0x02)]
        {
            let wire = dec.encode(&captured, button).unwrap();
            assert_eq!(wire.len(), TX_REPEAT * FRAME_REPEAT * FRAME_LEVELS);
            let mut s = vec![LevelDuration::new(false, 10_000)];
            s.extend(wire);
            let got = decode_all(&s);
            assert_eq!(got.len(), TX_REPEAT * FRAME_REPEAT, "button {button:#x}");
            // The next press of the same remote (format B now), the frame its make_frame gives.
            let (nd, nd2) = make_frame(&ORIGIN, 0x0055_AA00, 0x0145, want);
            for d in &got {
                assert_eq!((d.serial, d.button, d.counter, d.data, d.extra), (Some(0x0055_AA00), Some(want), Some(0x0145), nd, Some(nd2)));
            }
        }
        // The counter wraps past 0xFFFF to 0 (format A again).
        let (data, data_2) = make_frame(&ORIGIN, 7, 0xFFFF, 0x01);
        let mut s = vec![LevelDuration::new(false, 10_000)];
        s.extend(dec.encode(&signal(data, data_2), 0x01).unwrap());
        let got = decode_all(&s);
        assert_eq!(got.len(), TX_REPEAT * FRAME_REPEAT);
        assert!(got.iter().all(|d| d.counter == Some(0) && d.data == make_frame(&ORIGIN, 7, 0, 0x01).0));
        // Not a BelKorn frame / no Data word.
        assert!(dec.encode(&DecodedSignal::new(data, 64), 0x01).is_none());
        assert!(dec.encode(&DecodedSignal { extra: None, ..signal(data, data_2) }, 0x01).is_none());
    }

    #[test]
    fn upload_timing() {
        let (data, data_2) = make_frame(&ORIGIN, 0x1234_5678, 0x22, 0x01);
        let mut s = Vec::new();
        upload_once(data, data_2, &mut s);
        assert_eq!(s.len(), FRAME_REPEAT * FRAME_LEVELS);
        // Sync 0x801: long, ten shorts, long; HIGH first.
        let head: Vec<(bool, u32)> = s[..12].iter().map(|l| (l.level, l.duration_us)).collect();
        let mut want = vec![(true, 962)];
        want.extend((1..11).map(|i| (i % 2 == 0, 481)));
        want.push((false, 962));
        assert_eq!(head, want);
        assert_eq!((s[118].level, s[118].duration_us, s[119].level, s[119].duration_us), (true, 481, false, 7696));
    }

    #[test]
    fn key_file_round_trip() {
        let dec = BelKornDecoder::new();
        let key = fixture_key(0x5EED_1234_ABCD_0003);
        let text = key.to_text();
        assert!(text.contains("Protocol: BelKorn\nBit: 118\nKey: 80"), "{text}");
        let d = dec.from_flipper_key(&FlipperKey::parse(&text).unwrap()).unwrap();
        let back = dec.to_flipper_key(&d).unwrap();
        assert_eq!(back.to_text(), text);
        assert_eq!(format!("{:?}", dec.from_flipper_key(&back).unwrap()), format!("{d:?}"));
        // Wrong bit count, missing Data, a frame that fails the check.
        assert!(dec.from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 118", "Bit: 117")).unwrap()).is_none());
        assert!(dec.from_flipper_key(&FlipperKey::parse(&text.replace("Data:", "Dat:")).unwrap()).is_none());
        let mut bad = FlipperKey::for_protocol(NAME, MIN_COUNT_BIT, d.data);
        bad.push_hex("Data", d.extra.unwrap() & !0x3F, 8);
        assert!(dec.from_flipper_key(&bad).is_none());
        let reg = ProtocolRegistry::new();
        assert_eq!(reg.decode_key(&key).map(|(n, _)| n), Some(NAME.to_string()));
    }

    /// RogueMaster's "Add Manually" key: the counter and button as given, the protocol-wide origin.
    #[test]
    fn create_data_makes_a_frame_at_the_given_counter() {
        let key = create_data(0x7_89AB_CDEF, 0x01, 0x05);
        let d = BelKornDecoder::new().from_flipper_key(&key).unwrap();
        assert_eq!((d.serial, d.button, d.counter), (Some(0x89AB_CDEF), Some(0x01), Some(0x05)));
        assert_eq!(get(d.data, 0, 12, 4), 0x7);
        let f = analyze(d.data, d.extra.unwrap());
        assert_eq!(f.origin, Origin { serial_hi: 0x7, l24: L24_ORIGIN, b3: 0 });
    }

    /// Its uploads, through the whole registry at every test frequency, decode only as BelKorn.
    #[test]
    fn registry_attributes_belkorn_uploads_to_belkorn() {
        let dec = BelKornDecoder::new();
        for r in [0x0123_4567_89AB_CDEFu64, 0xFEDC_BA98_7654_3211, 0x5EED_0000_0045_0001, 0x0000_0000_0000_0000] {
            let d = dec.from_flipper_key(&fixture_key(r)).unwrap();
            let mut wire = vec![LevelDuration::new(false, 10_000)];
            wire.extend(dec.encode(&d, d.button.unwrap()).unwrap());
            for freq in crate::protocols::test_frequencies(crate::protocols::ANY_FREQUENCY) {
                let names: Vec<String> =
                    ProtocolRegistry::new().process_signal_stream(&wire, freq).into_iter().map(|(n, _, _)| n).collect();
                assert!(!names.is_empty() && names.iter().all(|n| n == NAME), "{r:#x} @ {freq}: {names:?}");
            }
        }
    }
}
