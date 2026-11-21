//! Kia V0 / Suzuki V0 / Honda V0 / Mitsubishi V0 decoder + encoder.
//!
//! Port of ProtoPirate `protocols/kia_v0.c` (HEAD 2026-10, after commits f81a706 "Refactor Kia V0 to
//! include shared logic for Suzuki V0 and Honda V0" and 1287d10 "merge Mitsubishi protocol into Kia
//! V0"). Its Kia type is older than ProtoPirate: the official Flipper firmware's "KIA" decoder
//! (Skorpionm, 2021-10, `lib/subghz/protocols/kia.c`, "KIA Seed" since 2022), of which ProtoPirate's
//! first `kia_v0.c` is a copy; its test recording is `IMPORTS/KIA/more/kia_seed_raw.sub`.
//! ProtoPirate retired its standalone `suzuki.c` and `mitsubishi_v0.c` in favour of these Kia V0
//! types; doctor-firmware's `kia_v0.c` carries the same four types and Flipper-ARF's `kia_v0.c` the
//! first three (no Mitsubishi), so this module is the union. **Where KAT departs from the C**: Honda
//! V0 is read and built by the remote's own frame (below), and every type sends the remote's press,
//! once; both from the recordings (`docs/kia_v0.md`).
//!
//! Air format: PWM pairs at 315 / 433.92 MHz (FM): each bit is HIGH+LOW of equal length,
//! te_short = 250µs → 0, te_long = 500µs → 1 (te_delta 100, strict `<`). A run of short pairs is the
//! preamble; the first long pair is the sync and contributes two `1` bits.
//!
//! | Type | Display name | Bits | Layout (`data`) | Check | End of frame |
//! |------|--------------|------|-----------------|-------|--------------|
//! | 1 | Kia V0 | 61 | `cnt16 << 40 \| serial28 << 12 \| btn4 << 8 \| crc8` | CRC-8 poly 0x7F over bits 8..55 | HIGH 700..1700µs |
//! | 2 | Suzuki V0 | 64 | `0xF << 60 \| cnt16 << 44 \| serial28 << 16 \| btn4 << 12 \| crc8 << 4` | CRC-8 0x7F over cnt/serial/btn | LOW then HIGH 2000..2500µs |
//! | 3 | Honda V0 | 72 (C label; the key is 64 bits) | the C's key `F0 rev(…)`, see below | CRC-8 0x21 over cnt/btn | the 56th bit |
//! | 4 | Mitsubishi V0 | 61 | as Kia | as Kia | as Kia, see below |
//!
//! **Honda V0** is the Nidec / Omron G8D remote of Hondas and Acuras (OUCG8D-344H-A and kin; rtl_433's
//! `nidec_car_remote.c` and `hondaremote.c`). After 320 short pairs and 4 long pairs (Omron's filings:
//! "324 bit codes … for the receiver to wake up") it sends 56 bits, `counter16 | id24 | 0x5 | button4 |
//! check8`, MSB first; the check is a CRC-8 (polynomial 0x21, init 0) over the counter and the button
//! nibble ([HondaFrame::check_of]), which every frame of the OEM and tool-made remotes recorded passes
//! (aftermarket "TS" clones send wrong ones) and rtl_433's OEM recordings too. Buttons (rtl_433's
//! names; the recordings' file names): 3 Lock, 4 Unlock, 5 a trunk or panic press, then F for the
//! trunk's long press, 6 for the panic's. The decoder reads the 61 bits as the C does; `data` stays
//! the C's key (`F0`, then the 56 bits with every byte bit-reversed), which the key files carry, and
//! the fields are the frame's: `serial` = the id, `button` = the nibble, `counter`, `crc_valid` = the
//! check. The C reads the id bytes bit-reversed, numbers the buttons from the reversed byte (a Lock is
//! its "Unlock3", a Panic its "Lock2"), checks with a table that is this CRC for 3, 4, 5 and F, and
//! sends Lock as 6 (Panic's long press) and Trunk as 2 (no remote's code), behind 40 preamble pairs.
//! Its decoder also loses a Honda frame whose 61 bits pass the Kia CRC (one in 256), waiting for a Kia
//! gap that never comes; KAT holds such a frame until the next HIGH (`honda_pending`).
//!
//! Mitsubishi V0 is the Kia frame sent twice, the second copy behind a 72..88 pair preamble (the C's
//! `last_kia_data` memory): the first copy decodes as Kia V0, an identical second one with that
//! preamble length as Mitsubishi V0.
//!
//! Decoder (`subghz_protocol_decoder_kia_feed`), ported step for step: Reset → Preamble (counts
//! short pairs, >14 then a long pair starts data with `11`) → SaveDuration (HIGH) / CheckDuration
//! (LOW). It emits on the same pulse as the C: the Kia gap (Kia / Mitsubishi / Honda), the Suzuki
//! HIGH gap, or the LOW completing a Honda-valid 61st bit; and, for a frame that is both, on the
//! HIGH after it.
//!
//! KAT deviation (registry semantics only): the registry resets every decoder after any decode,
//! which in the C would only clear the state machine (`kia_v0_decoder_state_clear`). `reset()`
//! therefore keeps the previous-frame memory (`last_kia_data`), as the C does across its own
//! decodes, so the Mitsubishi V0 repeat rule still works. A live capture (`process_signal`) reports
//! its first decode only, which for a Mitsubishi transmission is the first copy (Kia V0); `.sub`
//! imports list both, as ProtoPirate's history does.
//!
//! Encoder: the remote's press, sent once (`tx_copies.rs`; the C's TX loop sends its upload 3 times),
//! adjacent equal levels merged (identical on air). Each is the recordings' press pulse for pulse
//! (test `reencode_reproduces_the_next_press`):
//! - Kia: HIGH 750, LOW 750, 319 short pairs, frame (its bit 60, 0, is the 320th short pair),
//!   HIGH 1500, LOW 1500, 15 short pairs (+ bit 60: 16), frame, HIGH 1500 (the C: 1000).
//! - Mitsubishi: as Kia with a 79-pair second preamble (80 with bit 60, as the Pajero sends; the C's
//!   80 make 81) and a closing HIGH 1500 + LOW 1500.
//! - Suzuki: 320 short pairs, frame, LOW 2000, HIGH 2000, LOW 250, 15 short pairs, frame.
//! - Honda: 320 short pairs, 4 long pairs, the 56 bits, HIGH 500; a trunk or panic request sends the
//!   press's 5 and then its long press's F or 6, one count apart, 100 ms between them.
//!
//! The frame is rebuilt from serial/button/counter (CRC recomputed) with the counter + 1 per frame
//! (16 bits). "Send next code" ([ProtocolDecoder::encode_next]) keeps the captured button; KAT's
//! generic buttons map to the type's codes: Kia Lock 1, Unlock 2, Trunk 3, Panic 4 and Suzuki Lock 3,
//! Unlock 4, Trunk 2, Panic 1 (the C's names), Honda Lock 3, Unlock 4, Trunk 5 + F, Panic 5 + 6
//! (rtl_433's), Mitsubishi none (the Pajero sends 3, 4 and 6, named by no source: the captured
//! button); anything else keeps the captured button. The type comes from the bit count (61 Kia, 64
//! Suzuki, 72 Honda); Mitsubishi V0 captures reach [`encode_type`] through the "Mitsubishi V0"
//! registry entry (`mitsubishi_v0.rs`), which owns that name.

use super::common::{crc8, crc8_kia};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`). ProtoPirate flags Kia V0 FM (preset FM476), Flipper-ARF AM + FM.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Kia V0",
    decoder: "Kia V0",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::Both,
    encryption: "Rolling Code",
    make: "Kia/Hyundai",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&FM476, &AM650],
};

/// Kia V0 type 2 (display name "Suzuki V0"; "Suzuki" is ProtoPirate's retired standalone).
pub(super) const SUZUKI_V0_INFO: ProtocolInfo = ProtocolInfo {
    name: "Suzuki V0",
    decoder: "Kia V0",
    aliases: &["Suzuki"],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::Both,
    encryption: "Rolling Code",
    make: "Suzuki",
    category: Category::Vehicle,
    buttons: Some(suzuki_buttons),
    presets: &[&FM476, &AM650],
};

/// Kia V0 type 3 (display name "Honda V0").
pub(super) const HONDA_V0_INFO: ProtocolInfo = ProtocolInfo {
    name: "Honda V0",
    decoder: "Kia V0",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::Both,
    encryption: "Rolling Code",
    make: "Honda/Acura",
    category: Category::Vehicle,
    buttons: Some(honda_buttons),
    presets: &[&FM476, &AM650],
};

/// Kia V0 / Mitsubishi V0 buttons (ProtoPirate kia_v0_button_name; Panic = the emulate D-pad's Left code).
pub(super) fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x1 => "Lock",
        0x2 => "Unlock",
        0x3 => "Trunk",
        0x4 => "Panic",
        _ => "Other",
    })
}

/// Suzuki V0 buttons (kia_v0_button_name, Suzuki type).
fn suzuki_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x3 => "Lock",
        0x4 => "Unlock",
        0x2 => "Trunk",
        0x1 => "Panic",
        _ => "Other",
    })
}

/// Honda V0 buttons: the function nibble as sent, with rtl_433's names (`nidec_car_remote.c`); the
/// recordings' file names give 3, 4, 5 and F. A trunk or panic press sends 5, its long press then F
/// (trunk) or 6 (panic).
fn honda_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x3 => "Lock",
        0x4 => "Unlock",
        0x5 => "Trunk/Panic (short)",
        0x6 => "Panic (long)",
        0xF => "Trunk (long)",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;

pub(super) const BIT_COUNT_KIA: usize = 61;
pub(super) const BIT_COUNT_SUZUKI: usize = 64;
pub(super) const BIT_COUNT_HONDA: usize = 72;

/// A Kia gap the decoder takes (700–1700 µs; the C's encoder closed a Kia press with it), for the
/// tests.
#[cfg(test)]
const KIA_GAP: u32 = 1000;
const KIA_GAP_BASE: u32 = 700;
const KIA_GAP_SPAN: u32 = 1000;
const SUZUKI_GAP: u32 = 2000;
const SUZUKI_GAP_SPAN: u32 = 500;

const TYPE1_SYNC: u32 = 750;
const TYPE1_PREAMBLE_PAIRS: usize = 0x13F;
const TYPE2_PREAMBLE_PAIRS: usize = 0x140;
const TAIL_PREAMBLE_PAIRS: usize = 0x0F;
/// 79 pairs, then the frame's bit 60 (0) is the 80th: the Pajero's 80 (the C's 0x50 sends 81).
const MITSU_TAIL_PREAMBLE_PAIRS: usize = 0x4F;
const MITSU_PREAMBLE_MIN: u16 = 72;
const MITSU_PREAMBLE_MAX: u16 = 88;
/// The HIGH and LOW between two copies of a Kia / Mitsubishi frame, and the HIGH that ends the Kia
/// press (both Kia fobs; the C ends with 1000 µs).
const KIA_FRAME_GAP: u32 = 1500;
/// Honda V0: 320 short pairs and the 4 long pairs, Omron's "324 bit codes" of wake-up (the C: 40).
const HONDA_PREAMBLE_PAIRS: usize = 320;
const HONDA_SYNC_PAIRS: usize = 4;
const HONDA_WIRE_BITS: u32 = 56;
/// The HIGH that ends a Honda press (the OEM Civic remote and rtl_433's second OEM recording).
const HONDA_END: u32 = 500;
/// The silence between a trunk or panic press's two frames (the recordings: 297–310 ms from the
/// start of one to the start of the next, a frame lasting about 205 ms).
const HONDA_LONG_PRESS_GAP: u32 = 100_000;

/// Honda CRC fold table (`kia_v0_honda_crc_table`).
const HONDA_CRC_TABLE: [u8; 16] = [
    0x4A, 0x25, 0x96, 0x4B, 0xA1, 0xD4, 0x6A, 0x35, 0x9E, 0x4F, 0xA3, 0xD5, 0xEE, 0x77, 0xBF, 0xDB,
];

/// The four frame types of `kia_v0.c` (`KIA_V0_TYPE_*`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum KiaV0Type {
    Kia = 1,
    Suzuki = 2,
    Honda = 3,
    Mitsubishi = 4,
}

impl KiaV0Type {
    /// `kia_v0_protocol_name`: the name the C displays and saves.
    pub(super) fn display_name(self) -> &'static str {
        match self {
            KiaV0Type::Kia => "Kia V0",
            KiaV0Type::Suzuki => "Suzuki V0",
            KiaV0Type::Honda => "Honda V0",
            KiaV0Type::Mitsubishi => "Mitsubishi V0",
        }
    }

    fn bit_count(self) -> usize {
        match self {
            KiaV0Type::Suzuki => BIT_COUNT_SUZUKI,
            KiaV0Type::Honda => BIT_COUNT_HONDA,
            KiaV0Type::Kia | KiaV0Type::Mitsubishi => BIT_COUNT_KIA,
        }
    }

    /// Type for a saved bit count (`kia_v0_infer_type_from_bits`, limited to the C's allowed bits).
    fn from_bits(bits: usize) -> Option<Self> {
        match bits {
            BIT_COUNT_KIA => Some(KiaV0Type::Kia),
            BIT_COUNT_SUZUKI => Some(KiaV0Type::Suzuki),
            BIT_COUNT_HONDA => Some(KiaV0Type::Honda),
            _ => None,
        }
    }
}

/// `KiaV0Fields`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Fields {
    serial: u32,
    counter: u16,
    button: u8,
    crc: u8,
    crc_valid: bool,
}

#[inline]
fn is_short(d: u32) -> bool {
    duration_diff!(d, TE_SHORT) < TE_DELTA
}

#[inline]
fn is_long(d: u32) -> bool {
    duration_diff!(d, TE_LONG) < TE_DELTA
}

/// `kia_v0_is_kia_gap`: 700..=1700µs.
#[inline]
fn is_kia_gap(d: u32) -> bool {
    d >= KIA_GAP_BASE && d - KIA_GAP_BASE <= KIA_GAP_SPAN
}

/// `kia_v0_is_suzuki_gap_strict`: 2000..=2500µs.
#[inline]
fn is_suzuki_gap(d: u32) -> bool {
    d >= SUZUKI_GAP && d - SUZUKI_GAP <= SUZUKI_GAP_SPAN
}

/// `pp_reverse_bits8`.
#[inline]
fn rev8(v: u8) -> u8 {
    v.reverse_bits()
}

/// `kia_v0_calculate_crc_poly`: CRC-8 (poly 0x7F, init 0) over bytes 6..1 (bits 55..8).
fn crc_poly(data: u64) -> u8 {
    let bytes = [
        (data >> 48) as u8,
        (data >> 40) as u8,
        (data >> 32) as u8,
        (data >> 24) as u8,
        (data >> 16) as u8,
        (data >> 8) as u8,
    ];
    crc8_kia(&bytes)
}

/// `kia_v0_verify_crc_poly`.
fn verify_crc_poly(data: u64) -> bool {
    crc_poly(data) == data as u8
}

/// `kia_v0_suzuki_verify_shifted_crc_words` / `kia_v0_suzuki_shifted_crc_valid`.
fn suzuki_shifted_crc_valid(shifted: u64) -> bool {
    let lo = shifted as u32;
    let hi = (shifted >> 32) as u32;
    let r3 = ((lo >> 16) | (hi << 16)) & !0xF000_0000u32;
    let r2 = (lo >> 12) & 0x0F;
    let mut r1 = (hi >> 12) & 0xFFFF;
    let r4 = (lo >> 4) as u8;
    r1 = ((r1 & 0xFF) << 8) | ((r1 >> 8) & 0xFF);
    let buf = [
        r1 as u8,
        (r1 >> 8) as u8,
        (r3 >> 20) as u8,
        (r3 >> 12) as u8,
        (r3 >> 4) as u8,
        ((r3 << 4) | (r2 & 0x0F)) as u8,
    ];
    crc8_kia(&buf) == r4
}

/// `kia_v0_suzuki_crc8_from_fields`.
fn suzuki_crc8_from_fields(serial: u32, button: u8, counter: u32) -> u8 {
    let cnt = (counter & 0xFFFF) as u16;
    let c_sw = cnt.rotate_left(8);
    let buf = [
        c_sw as u8,
        (c_sw >> 8) as u8,
        (serial >> 20) as u8,
        (serial >> 12) as u8,
        (serial >> 4) as u8,
        button | (serial << 4) as u8,
    ];
    crc8_kia(&buf)
}

/// `kia_v0_suzuki_shifted_key_from_fields`.
fn suzuki_shifted_key(serial: u32, button: u8, counter: u32, crc: u8) -> u64 {
    let r8 = (serial << 16) | ((crc as u32) << 4) | (((button as u32) & 0x0F) << 12);
    let r5 = ((serial >> 16) & 0xFFF) | ((counter & 0xFFFF) << 12) | 0xF000_0000;
    ((r5 as u64) << 32) | r8 as u64
}

/// `kia_v0_suzuki_resolve_shifted`: the frame as received, or shifted left by one.
fn suzuki_resolve_shifted(decode_data: u64) -> Option<u64> {
    if suzuki_shifted_crc_valid(decode_data) {
        return Some(decode_data);
    }
    let from_wire = decode_data << 1;
    suzuki_shifted_crc_valid(from_wire).then_some(from_wire)
}

/// `kia_v0_honda_transform`: bit-reverse every byte.
fn honda_transform(data: u64) -> u64 {
    u64::from_be_bytes(data.to_be_bytes().map(rev8))
}

/// `kia_v0_honda_fold_counter`.
fn honda_fold_counter(counter: u16) -> u8 {
    HONDA_CRC_TABLE
        .iter()
        .enumerate()
        .filter(|(i, _)| (counter >> i) & 1 != 0)
        .fold(0, |acc, (_, &v)| acc ^ v)
}

/// `kia_v0_honda_crc`.
fn honda_crc(header: u8, counter: u16) -> u8 {
    honda_fold_counter(counter)
        ^ match header {
            0xAA => 0xA5,
            0x2A => 0x21,
            0x6A => 0x15,
            0xFA => 0x73,
            _ => 0xC6,
        }
}

/// `kia_v0_honda_header`.
fn honda_header(button: u8) -> u8 {
    let b = button & 0x07;
    let base = if b == 0x07 { 0x1A } else { 0x0A };
    ((b << 5) & 0xE0) | base
}

/// A Honda V0 frame as the remote sends it (see the module doc): the 56 bits after the four long
/// sync pairs, `counter16 | id24 | five4 | button4 | check8`, MSB first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct HondaFrame {
    counter: u16,
    id: u32,
    /// 5 in every frame of every remote recorded.
    five: u8,
    button: u8,
    check: u8,
}

impl HondaFrame {
    /// From the C's 64-bit key: `F0`, then the 56 bits with every byte bit-reversed.
    fn from_key(key: u64) -> Self {
        let b = key.to_be_bytes().map(rev8);
        Self {
            counter: u16::from_be_bytes([b[1], b[2]]),
            id: u32::from_be_bytes([0, b[3], b[4], b[5]]),
            five: b[6] >> 4,
            button: b[6] & 0x0F,
            check: b[7],
        }
    }

    /// The C's key of this frame (what `data` and the key files hold).
    fn key(self) -> u64 {
        let [_, i2, i1, i0] = self.id.to_be_bytes();
        let [c1, c0] = self.counter.to_be_bytes();
        u64::from_be_bytes([0x0F, c1, c0, i2, i1, i0, (self.five << 4) | self.button, self.check].map(rev8))
    }

    /// The check: CRC-8, polynomial 0x21, init 0, over the counter and the button nibble (20 bits).
    fn check_of(counter: u16, button: u8) -> u8 {
        crc8(&[(counter >> 12) as u8, (counter >> 4) as u8, ((counter as u8) << 4) | (button & 0x0F)], 0x21, 0)
    }

    fn check_ok(self) -> bool {
        self.check == Self::check_of(self.counter, self.button)
    }

    /// The remote's next frame with button `button`: the counter one higher, the check recomputed.
    fn next(self, button: u8) -> Self {
        let counter = self.counter.wrapping_add(1);
        Self { counter, button: button & 0x0F, check: Self::check_of(counter, button), ..self }
    }
}

/// `kia_v0_build_kia_raw`.
fn build_kia_raw(serial: u32, button: u8, counter: u16, crc: u8) -> u64 {
    let high = 0x0F00_0000u32 | ((counter as u32) << 8) | ((serial >> 20) & 0xFF);
    let low = ((serial & 0x000F_FFFF) << 12) | (((button as u32) & 0x0F) << 8) | crc as u32;
    ((high as u64) << 32) | low as u64
}

/// `kia_v0_build_honda_key`.
fn build_honda_key(serial: u32, button: u8, counter: u16) -> u64 {
    let header = honda_header(button);
    let crc = honda_crc(header, counter);
    u64::from_be_bytes([
        0xF0,
        rev8((counter >> 8) as u8),
        rev8(counter as u8),
        (serial >> 16) as u8,
        (serial >> 8) as u8,
        serial as u8,
        header,
        crc,
    ])
}

/// `kia_v0_honda_key_valid`.
fn honda_key_valid(key: u64) -> bool {
    key >> 60 == 0x0F && (key >> 56) & 0x0F == 0 && (key >> 8) & 0x0F == 0x0A
}

/// `kia_v0_parse_data` (with `kia_v0_parse_family_raw` / `kia_v0_parse_honda_key`).
fn parse_fields(data: u64, ty: KiaV0Type) -> Fields {
    match ty {
        KiaV0Type::Honda => {
            let b = data.to_be_bytes();
            let counter = ((rev8(b[1]) as u16) << 8) | rev8(b[2]) as u16;
            Fields {
                serial: ((b[3] as u32) << 16) | ((b[4] as u32) << 8) | b[5] as u32,
                counter,
                button: b[6] >> 5,
                crc: b[7],
                crc_valid: honda_crc(b[6], counter) == b[7],
            }
        }
        KiaV0Type::Suzuki => Fields {
            serial: ((data >> 16) & 0x0FFF_FFFF) as u32,
            counter: (data >> 44) as u16,
            button: ((data >> 12) & 0x0F) as u8,
            crc: (data >> 4) as u8,
            crc_valid: suzuki_shifted_crc_valid(data),
        },
        KiaV0Type::Kia | KiaV0Type::Mitsubishi => Fields {
            serial: ((data >> 12) & 0x0FFF_FFFF) as u32,
            counter: (data >> 40) as u16,
            button: ((data >> 8) & 0x0F) as u8,
            crc: data as u8,
            crc_valid: verify_crc_poly(data),
        },
    }
}

/// The decode reported for frame `data` of type `ty` (`kia_v0_decoder_commit` → `kia_v0_parse_data`;
/// Honda V0: the frame's fields as sent, see the module doc).
fn signal(data: u64, ty: KiaV0Type) -> DecodedSignal {
    let f = if ty == KiaV0Type::Honda {
        let h = HondaFrame::from_key(data);
        Fields { serial: h.id, counter: h.counter, button: h.button, crc: h.check, crc_valid: h.check_ok() }
    } else {
        parse_fields(data, ty)
    };
    DecodedSignal {
        serial: Some(f.serial),
        button: Some(f.button),
        counter: Some(f.counter),
        crc_valid: f.crc_valid,
        data,
        data_count_bit: ty.bit_count(),
        encoder_capable: true,
        extra: None,
        protocol_display_name: (ty != KiaV0Type::Kia).then(|| ty.display_name().to_string()),
    }
}

/// Decoder states (`KiaV0DecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    SaveDuration,
    CheckDuration,
}

/// Kia V0 family decoder (`SubGhzProtocolDecoderKIA`).
pub struct KiaV0Decoder {
    step: DecoderStep,
    te_last: u32,
    decode_data: u64,
    /// `SubGhzBlockDecoder.decode_count_bit` is a `uint8_t`.
    decode_count_bit: u8,
    preamble_pairs: u16,
    last_kia_data: u64,
    have_last_kia: bool,
    /// A 61-bit frame that passes both the Kia CRC and the Honda check (Honda key), waiting for
    /// the next HIGH: a Kia gap makes it Kia V0, anything else Honda V0.
    honda_pending: Option<u64>,
}

impl KiaV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            preamble_pairs: 0,
            last_kia_data: 0,
            have_last_kia: false,
            honda_pending: None,
        }
    }

    /// `kia_v0_decoder_state_clear`.
    fn state_clear(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.preamble_pairs = 0;
        self.honda_pending = None;
    }

    /// `subghz_protocol_blocks_add_bit`.
    fn add_bit(&mut self, bit: bool) {
        self.decode_data = (self.decode_data << 1) | bit as u64;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }

    /// `kia_v0_decoder_commit`: report the frame and clear the state machine.
    fn commit(&mut self, data: u64, ty: KiaV0Type) -> DecodedSignal {
        self.state_clear();
        signal(data, ty)
    }

    /// `kia_v0_decoder_finish_kia_or_honda_at_gap`.
    fn finish_at_gap(&mut self) -> Option<DecodedSignal> {
        if self.decode_count_bit as usize != BIT_COUNT_KIA {
            self.state_clear();
            return None;
        }
        let data = self.decode_data;
        if verify_crc_poly(data) {
            let ty = if self.have_last_kia
                && self.last_kia_data == data
                && (MITSU_PREAMBLE_MIN..=MITSU_PREAMBLE_MAX).contains(&self.preamble_pairs)
            {
                KiaV0Type::Mitsubishi
            } else {
                KiaV0Type::Kia
            };
            self.last_kia_data = data;
            self.have_last_kia = true;
            return Some(self.commit(data, ty));
        }
        let key = honda_transform(data & 0x0FFF_FFFF_FFFF_FFFF);
        if honda_key_valid(key) {
            return Some(self.commit(key, KiaV0Type::Honda));
        }
        self.state_clear();
        None
    }

    /// `kia_v0_decoder_try_honda`: a 61-bit frame that fails the Kia CRC but is a valid Honda key is
    /// Honda V0 at once. One that passes the Kia CRC too (one Honda frame in 256 does) and also the
    /// Honda check waits for the next HIGH (`honda_pending`); the C waits for a Kia gap only, which a
    /// Honda frame never has, and loses it.
    fn try_honda(&mut self) -> Option<DecodedSignal> {
        if self.decode_count_bit as usize != BIT_COUNT_KIA {
            return None;
        }
        let key = honda_transform(self.decode_data & 0x0FFF_FFFF_FFFF_FFFF);
        if !honda_key_valid(key) {
            return None;
        }
        if !verify_crc_poly(self.decode_data) {
            return Some(self.commit(key, KiaV0Type::Honda));
        }
        if HondaFrame::from_key(key).check_ok() {
            self.honda_pending = Some(key);
        }
        None
    }
}

impl ProtocolDecoder for KiaV0Decoder {
    fn name(&self) -> &'static str {
        "Kia V0"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: BIT_COUNT_KIA,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433
        &[315_000_000, 433_920_000]
    }

    /// Clears the state machine but keeps the previous-frame memory (see the module docs).
    fn reset(&mut self) {
        self.state_clear();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                self.state_clear();
                if level && is_short(duration) {
                    self.step = DecoderStep::Preamble;
                    self.te_last = duration;
                    self.preamble_pairs = 0;
                }
            }
            DecoderStep::Preamble => {
                if level {
                    if is_short(duration) || is_long(duration) {
                        self.te_last = duration;
                    } else {
                        self.state_clear();
                    }
                } else if is_short(duration) && is_short(self.te_last) {
                    self.preamble_pairs = self.preamble_pairs.wrapping_add(1);
                } else if is_long(duration) && is_long(self.te_last) {
                    if self.preamble_pairs > 14 {
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.add_bit(true);
                        self.add_bit(true);
                        self.step = DecoderStep::SaveDuration;
                    } else {
                        self.state_clear();
                    }
                } else {
                    self.state_clear();
                }
            }
            DecoderStep::SaveDuration => {
                if !level {
                    self.state_clear();
                    return None;
                }
                if is_kia_gap(duration) {
                    return self.finish_at_gap();
                }
                if let Some(key) = self.honda_pending.take() {
                    return Some(self.commit(key, KiaV0Type::Honda));
                }
                if is_suzuki_gap(duration) && is_suzuki_gap(self.te_last) {
                    if self.decode_count_bit as usize == BIT_COUNT_SUZUKI {
                        if let Some(shifted) = suzuki_resolve_shifted(self.decode_data) {
                            return Some(self.commit(shifted, KiaV0Type::Suzuki));
                        }
                    }
                    self.state_clear();
                    return None;
                }
                if is_short(duration) || is_long(duration) {
                    self.te_last = duration;
                    self.step = DecoderStep::CheckDuration;
                } else {
                    self.state_clear();
                }
            }
            DecoderStep::CheckDuration => {
                if level {
                    self.state_clear();
                    return None;
                }
                let pair = if is_short(self.te_last) && is_short(duration) {
                    Some(false)
                } else if is_long(self.te_last) && is_long(duration) {
                    Some(true)
                } else {
                    None
                };
                if let Some(bit) = pair {
                    self.add_bit(bit);
                    let honda = self.try_honda();
                    if honda.is_none() {
                        self.step = DecoderStep::SaveDuration;
                    }
                    return honda;
                }
                if is_suzuki_gap(duration) {
                    self.te_last = duration;
                    self.step = DecoderStep::SaveDuration;
                    return None;
                }
                let honda = self.try_honda();
                if honda.is_none() {
                    self.state_clear();
                }
                return honda;
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        encode_type(decoded, Request::Button(button), KiaV0Type::from_bits(decoded.data_count_bit)?)
    }

    /// "Send next code": the captured button (the types' codes overlap KAT's generic ones).
    fn encode_next(&self, decoded: &DecodedSignal) -> Option<Vec<LevelDuration>> {
        encode_type(decoded, Request::Captured, KiaV0Type::from_bits(decoded.data_count_bit)?)
    }

    /// `subghz_protocol_decoder_kia_deserialize`: Bit must be 61, 64 or 72; the type comes from
    /// `Type` (1..=4), else from a legacy Protocol name ("Suzuki", "Honda V0", "Mitsu V0" …), else
    /// from the bit count. When the file has Serial/Btn/Cnt the C rebuilds the key from them (CRC
    /// recomputed); KAT does that only when they differ from the key's own fields, so a key it
    /// wrote reloads bit for bit (the C's rebuild would also clear the sync bit the decoder leaves
    /// in bit 60 of a Kia frame — same fields, same frame on air).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let by_bits = KiaV0Type::from_bits(key.bits)?;
        let ty = match key.u32_field("Type") {
            Some(1) => KiaV0Type::Kia,
            Some(2) => KiaV0Type::Suzuki,
            Some(3) => KiaV0Type::Honda,
            Some(4) => KiaV0Type::Mitsubishi,
            _ => type_from_protocol_name(&key.protocol).unwrap_or(by_bits),
        };
        let mut data = key.key_u64()?;
        let f = parse_fields(data, ty);
        let (serial, button, counter) = (key.u32_field("Serial"), key.u32_field("Btn"), key.u32_field("Cnt"));
        if serial.is_some() || button.is_some() || counter.is_some() {
            let serial = serial.unwrap_or(f.serial);
            let button = button.map_or(f.button, |b| b as u8);
            let counter = counter.map_or(f.counter, |c| c as u16);
            let mask = if ty == KiaV0Type::Honda { 0x07 } else { 0x0F };
            let edited = (serial, button & mask, counter) != (f.serial, f.button, f.counter);
            if edited {
                data = build_frame(ty, serial, button & mask, counter);
            }
        }
        Some(signal(data, ty))
    }

    /// `subghz_protocol_decoder_kia_serialize`: Protocol "Kia V0" for every type (the C's
    /// `generic.protocol_name`), Bit 61/64/72, Key = the frame, then `pp_serialize_fields`'
    /// Serial, Btn, Cnt and Type (1 Kia, 2 Suzuki, 3 Honda, 4 Mitsubishi), decimal.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let ty = match decoded.protocol_display_name.as_deref().map(super::info::strip_variant) {
            Some("Suzuki V0") => KiaV0Type::Suzuki,
            Some("Honda V0") => KiaV0Type::Honda,
            Some("Mitsubishi V0") => KiaV0Type::Mitsubishi,
            _ => KiaV0Type::from_bits(decoded.data_count_bit)?,
        };
        if ty.bit_count() != decoded.data_count_bit {
            return None;
        }
        let f = parse_fields(decoded.data, ty);
        let mut key = FlipperKey::for_protocol("Kia V0", ty.bit_count(), decoded.data);
        key.push_u32("Serial", f.serial);
        key.push_u32("Btn", f.button.into());
        key.push_u32("Cnt", f.counter.into());
        key.push_u32("Type", ty as u32);
        Some(key)
    }
}

/// `kia_v0_type_from_protocol_name`: the type a legacy / display Protocol name stands for.
fn type_from_protocol_name(name: &str) -> Option<KiaV0Type> {
    match name.trim() {
        "Mitsu v0" | "Mitsu V0" | "Mitsubishi v0" | "Mitsubishi V0" => Some(KiaV0Type::Mitsubishi),
        "Suzuki" | "Suzuki V0" => Some(KiaV0Type::Suzuki),
        "Honda v0" | "Honda V0" => Some(KiaV0Type::Honda),
        _ => None,
    }
}

impl Default for KiaV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

/// KAT generic button (0x01 Lock, 0x02 Unlock, 0x04 Trunk, 0x08 Panic) → the type's code, from the
/// C's button names and the emulate scene's D-pad map; `None` keeps the captured button. Mitsubishi
/// V0 maps none: the Pajero recorded sends 3, 4 and 6, and no source says which is which (the C gives
/// it Kia's 1 and 2). Honda V0 is [encode_honda]'s.
fn map_button(ty: KiaV0Type, button: u8) -> Option<u8> {
    match (ty, button) {
        (KiaV0Type::Kia, 0x01) => Some(0x1),
        (KiaV0Type::Kia, 0x02) => Some(0x2),
        (KiaV0Type::Kia, 0x04) => Some(0x3),
        (KiaV0Type::Kia, 0x08) => Some(0x4),
        (KiaV0Type::Suzuki, 0x01) => Some(0x3),
        (KiaV0Type::Suzuki, 0x02) => Some(0x4),
        (KiaV0Type::Suzuki, 0x04) => Some(0x2),
        (KiaV0Type::Suzuki, 0x08) => Some(0x1),
        _ => None,
    }
}

/// Wire frame (`generic.data`) for the fields, exactly as `kia_v0_encoder_apply_fields` builds it.
fn build_frame(ty: KiaV0Type, serial: u32, button: u8, counter: u16) -> u64 {
    match ty {
        KiaV0Type::Honda => build_honda_key(serial, button & 0x07, counter),
        KiaV0Type::Suzuki => {
            let crc = suzuki_crc8_from_fields(serial, button & 0x0F, counter as u32);
            suzuki_shifted_key(serial, button & 0x0F, counter as u32, crc)
        }
        KiaV0Type::Kia | KiaV0Type::Mitsubishi => {
            let crc = crc_poly(build_kia_raw(serial, button & 0x0F, counter, 0));
            build_kia_raw(serial, button & 0x0F, counter, crc)
        }
    }
}

/// Append one level, merging with the previous entry when the level repeats (identical on air).
fn push(out: &mut Vec<LevelDuration>, level: bool, duration: u32) {
    if let Some(last) = out.last_mut() {
        if last.level == level {
            last.duration_us += duration;
            return;
        }
    }
    out.push(LevelDuration::new(level, duration));
}

/// `kia_v0_append_short_pairs`.
fn push_short_pairs(out: &mut Vec<LevelDuration>, pairs: usize) {
    for _ in 0..pairs {
        push(out, true, TE_SHORT);
        push(out, false, TE_SHORT);
    }
}

/// `kia_v0_append_data_pairs`: bits `bits-1..0`, each HIGH+LOW of te_long (1) or te_short (0).
fn push_data_pairs(out: &mut Vec<LevelDuration>, data: u64, bits: u32) {
    for bit in (0..bits).rev() {
        let d = if (data >> bit) & 1 != 0 { TE_LONG } else { TE_SHORT };
        push(out, true, d);
        push(out, false, d);
    }
}

/// The C upload for a frame of the given type (`kia_v0_build_{kia,mitsu,suzuki,honda}_upload`).
fn build_upload(ty: KiaV0Type, frame: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(1024);
    match ty {
        KiaV0Type::Kia | KiaV0Type::Mitsubishi => {
            let (tail, end) = if ty == KiaV0Type::Mitsubishi {
                (MITSU_TAIL_PREAMBLE_PAIRS, [(true, KIA_FRAME_GAP), (false, KIA_FRAME_GAP)])
            } else {
                (TAIL_PREAMBLE_PAIRS, [(true, KIA_FRAME_GAP), (false, 0)])
            };
            push(&mut out, true, TYPE1_SYNC);
            push(&mut out, false, TYPE1_SYNC);
            push_short_pairs(&mut out, TYPE1_PREAMBLE_PAIRS);
            push_data_pairs(&mut out, frame, BIT_COUNT_KIA as u32);
            push(&mut out, true, KIA_FRAME_GAP);
            push(&mut out, false, KIA_FRAME_GAP);
            push_short_pairs(&mut out, tail);
            push_data_pairs(&mut out, frame, BIT_COUNT_KIA as u32);
            for (level, d) in end {
                if d > 0 {
                    push(&mut out, level, d);
                }
            }
        }
        KiaV0Type::Suzuki => {
            push_short_pairs(&mut out, TYPE2_PREAMBLE_PAIRS);
            push_data_pairs(&mut out, frame, BIT_COUNT_SUZUKI as u32);
            push(&mut out, false, SUZUKI_GAP);
            push(&mut out, true, SUZUKI_GAP);
            push(&mut out, false, TE_SHORT);
            push_short_pairs(&mut out, TAIL_PREAMBLE_PAIRS);
            push_data_pairs(&mut out, frame, BIT_COUNT_SUZUKI as u32);
        }
        KiaV0Type::Honda => push_honda_press(&mut out, frame),
    }
    out
}

/// One Honda V0 frame (the C's key `key`) as the remote sends it: 320 short pairs, the 4 long sync
/// pairs, the 56 bits, HIGH 500 µs.
fn push_honda_press(out: &mut Vec<LevelDuration>, key: u64) {
    push_short_pairs(out, HONDA_PREAMBLE_PAIRS);
    for _ in 0..HONDA_SYNC_PAIRS {
        push(out, true, TE_LONG);
        push(out, false, TE_LONG);
    }
    push_data_pairs(out, honda_transform(key), HONDA_WIRE_BITS);
    push(out, true, HONDA_END);
}

/// What a transmission asks of [encode_type]: KAT's generic button (TX Lock 0x01, Unlock 0x02,
/// Trunk 0x04, Panic 0x08; any other value keeps the captured button), or the captured button
/// ("Send next code", [ProtocolDecoder::encode_next]). The types' own codes overlap the generic
/// ones (Suzuki's 4 is Unlock, the generic 0x04 Trunk), so the two must not be told apart by value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Request {
    Button(u8),
    Captured,
}

/// Honda V0: the remote's next frame of the requested button, or for trunk and panic its long
/// press, 5 then F or 6, each frame one count higher (see the module doc).
fn encode_honda(decoded: &DecodedSignal, request: Request) -> Option<Vec<LevelDuration>> {
    // The decode's fields (the frame's as sent), else the key's; the nibble after the id is 5 in
    // every remote's frames.
    let key = HondaFrame::from_key(decoded.data);
    let mut frame = HondaFrame {
        counter: decoded.counter.unwrap_or(key.counter),
        id: decoded.serial.map_or(key.id, |s| s & 0x00FF_FFFF),
        five: 0x5,
        button: decoded.button.unwrap_or(key.button) & 0x0F,
        check: 0,
    };
    let buttons: &[u8] = match request {
        Request::Button(0x01) => &[0x3],
        Request::Button(0x02) => &[0x4],
        Request::Button(0x04) => &[0x5, 0xF],
        Request::Button(0x08) => &[0x5, 0x6],
        Request::Button(_) | Request::Captured => &[frame.button],
    };
    let mut out = Vec::with_capacity(1600);
    for (i, &button) in buttons.iter().enumerate() {
        frame = frame.next(button);
        if i > 0 {
            push(&mut out, false, HONDA_LONG_PRESS_GAP);
        }
        push_honda_press(&mut out, frame.key());
    }
    Some(out)
}

/// Encode a Kia V0 family frame of type `ty` for KAT's generic `button` (see the module docs).
/// Fields missing from `decoded` are parsed from its `data` (the C's raw-key path).
pub(super) fn encode_type(
    decoded: &DecodedSignal,
    request: Request,
    ty: KiaV0Type,
) -> Option<Vec<LevelDuration>> {
    if ty == KiaV0Type::Honda {
        return encode_honda(decoded, request);
    }
    let parsed = parse_fields(decoded.data, ty);
    let serial = decoded.serial.unwrap_or(parsed.serial);
    let captured_button = decoded.button.unwrap_or(parsed.button);
    let button = match request {
        Request::Button(b) => map_button(ty, b).unwrap_or(captured_button),
        Request::Captured => captured_button,
    };
    let counter = decoded.counter.unwrap_or(parsed.counter).wrapping_add(1);
    Some(build_upload(ty, build_frame(ty, serial, button, counter)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    fn signal(ty: KiaV0Type, serial: u32, button: u8, counter: u16) -> DecodedSignal {
        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: true,
            data: 0,
            data_count_bit: ty.bit_count(),
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    fn feed_all(dec: &mut KiaV0Decoder, wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        wire.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    /// type, serial, btn, cnt, key, decodes, last type, key, serial, button, counter, crc_valid.
    type CVector = (u64, u32, u8, u16, u64, usize, u64, u64, u32, u8, u16, bool);

    /// Golden vectors from the C (`kia_v0.c` built on the host, encoder fed Type/Serial/Btn/Cnt as the
    /// emulate scene does): type, serial, btn, cnt, the encoder's `generic.data`, how many frames the
    /// C decoder reports on that one (merged) upload, and the last report: type, key, serial, button,
    /// counter, crc_valid. Kia sends two copies (both Kia V0), Mitsubishi two (Kia V0 then
    /// Mitsubishi V0); Suzuki's second copy has no closing HIGH gap, Honda sends one.
    #[rustfmt::skip]
    const C_VECTORS: &[CVector] = &[
        (1, 0x01234567, 0x2, 0x1234, 0x0F1234123456724E, 2, 1, 0x1F1234123456724E, 0x1234567, 0x2, 0x1234, true),
        (1, 0x0FEDCBA9, 0x3, 0x0001, 0x0F0001FEDCBA93D2, 2, 1, 0x1F0001FEDCBA93D2, 0xFEDCBA9, 0x3, 0x0001, true),
        (1, 0xA5A5A5A5, 0x4, 0x8000, 0x0F80005A5A5A542D, 2, 1, 0x1F80005A5A5A542D, 0x5A5A5A5, 0x4, 0x8000, true),
        (1, 0x00000001, 0x7, 0xFFFF, 0x0FFFFF000000172B, 2, 1, 0x1FFFFF000000172B, 0x0000001, 0x7, 0xFFFF, true),
        (2, 0x01234567, 0x3, 0xFFFF, 0xFFFFF12345673900, 1, 2, 0xFFFFF12345673900, 0x1234567, 0x3, 0xFFFF, true),
        (2, 0x0FEDCBA9, 0x4, 0x1234, 0xF1234FEDCBA94080, 1, 2, 0xF1234FEDCBA94080, 0xFEDCBA9, 0x4, 0x1234, true),
        (2, 0xA5A5A5A5, 0x7, 0x0001, 0xF00015A5A5A57560, 1, 2, 0xF00015A5A5A57560, 0x5A5A5A5, 0x7, 0x0001, true),
        (2, 0x00000001, 0x8, 0x8000, 0xF800000000018D30, 1, 2, 0xF800000000018D30, 0x0000001, 0x8, 0x8000, true),
        (3, 0x01234567, 0x4, 0x8000, 0xF001002345678A1D, 1, 3, 0xF001002345678A1D, 0x0234567, 0x4, 0x8000, true),
        (3, 0x0FEDCBA9, 0x7, 0xFFFF, 0xF0FFFFEDCBA9FAB1, 1, 3, 0xF0FFFFEDCBA9FAB1, 0x0EDCBA9, 0x7, 0xFFFF, true),
        (3, 0xA5A5A5A5, 0x8, 0x1234, 0xF0482CA5A5A50A84, 1, 3, 0xF0482CA5A5A50A84, 0x0A5A5A5, 0x0, 0x1234, true),
        (3, 0x00000001, 0x1, 0x0001, 0xF000800000012A6B, 1, 3, 0xF000800000012A6B, 0x0000001, 0x1, 0x0001, true),
        (4, 0x01234567, 0x7, 0x0001, 0x0F0001123456776A, 2, 4, 0x1F0001123456776A, 0x1234567, 0x7, 0x0001, true),
        (4, 0x0FEDCBA9, 0x8, 0x8000, 0x0F8000FEDCBA98D0, 2, 4, 0x1F8000FEDCBA98D0, 0xFEDCBA9, 0x8, 0x8000, true),
        (4, 0xA5A5A5A5, 0x1, 0xFFFF, 0x0FFFFF5A5A5A5152, 2, 4, 0x1FFFFF5A5A5A5152, 0x5A5A5A5, 0x1, 0xFFFF, true),
        (4, 0x00000001, 0x2, 0x1234, 0x0F12340000001276, 2, 4, 0x1F12340000001276, 0x0000001, 0x2, 0x1234, true),
    ];

    /// The C's key builder (the key files' rebuild) gives the C's keys; the decoder reads them back.
    /// Honda V0 is reported with the frame's own fields (the C's are still those of the key, which
    /// `to_flipper_key` writes); the C's encoder sends Honda buttons 4, 7, 0 and 1 as the nibbles 1, F,
    /// 0 and 4, of which only F and 4 are a remote's.
    #[test]
    fn c_vectors_encode_and_decode() {
        for &(ty, serial, btn, cnt, key, n, dty, dkey, dser, dbtn, dcnt, dok) in C_VECTORS {
            let ty = ty_of(ty);
            assert_eq!(build_frame(ty, serial, btn, cnt), key, "{ty:?} {serial:#x}");
            let got = feed_all(&mut KiaV0Decoder::new(), &build_upload(ty, key));
            assert_eq!(got.len(), n, "{ty:?} {serial:#x}");
            let last = got.last().unwrap();
            let name = last.protocol_display_name.as_deref().unwrap_or("Kia V0");
            assert_eq!(name, ty_of(dty).display_name());
            assert_eq!(last.data, dkey);
            assert_eq!(last.data_count_bit, ty.bit_count());
            let c = parse_fields(dkey, ty);
            assert_eq!((c.serial, c.button, c.counter), (dser, dbtn, dcnt), "the C's fields of the key");
            if ty == KiaV0Type::Honda {
                let h = HondaFrame::from_key(dkey);
                assert_eq!(h.five, 5);
                assert_eq!((last.serial, last.button, last.counter), (Some(h.id), Some(h.button), Some(h.counter)));
                assert_eq!(last.crc_valid, h.check_ok());
            } else {
                assert_eq!((last.serial, last.button, last.counter, last.crc_valid), (Some(dser), Some(dbtn), Some(dcnt), dok));
            }
        }
    }

    fn ty_of(n: u64) -> KiaV0Type {
        match n {
            1 => KiaV0Type::Kia,
            2 => KiaV0Type::Suzuki,
            3 => KiaV0Type::Honda,
            _ => KiaV0Type::Mitsubishi,
        }
    }

    /// encode(): counter + 1 and KAT's generic button mapped to the type's code (unknown keeps the
    /// captured one); the result decodes to exactly those fields.
    /// encode(): the counter + 1 per frame and KAT's generic button mapped to the type's codes
    /// (unknown keeps the captured one); Honda's trunk and panic send the long press, 5 then F or 6;
    /// the frames sent decode to exactly those fields. Mitsubishi V0 keeps the captured button.
    #[test]
    fn encode_bumps_counter_and_maps_buttons() {
        let cases: &[(KiaV0Type, u8, &[u8])] = &[
            (KiaV0Type::Kia, 0x01, &[0x1]),
            (KiaV0Type::Kia, 0x02, &[0x2]),
            (KiaV0Type::Kia, 0x04, &[0x3]),
            (KiaV0Type::Kia, 0x08, &[0x4]),
            (KiaV0Type::Kia, 0x40, &[0x9]),
            (KiaV0Type::Suzuki, 0x01, &[0x3]),
            (KiaV0Type::Suzuki, 0x02, &[0x4]),
            (KiaV0Type::Suzuki, 0x04, &[0x2]),
            (KiaV0Type::Suzuki, 0x08, &[0x1]),
            (KiaV0Type::Honda, 0x01, &[0x3]),
            (KiaV0Type::Honda, 0x02, &[0x4]),
            (KiaV0Type::Honda, 0x04, &[0x5, 0xF]),
            (KiaV0Type::Honda, 0x08, &[0x5, 0x6]),
            (KiaV0Type::Honda, 0x40, &[0x5]),
        ];
        for &(ty, generic, want) in cases {
            let serial = if ty == KiaV0Type::Honda { 0x00AB_CDEF } else { 0x0ABC_DEF1 };
            let src = signal(ty, serial, if ty == KiaV0Type::Honda { 0x5 } else { 0x9 }, 0xFFFF);
            let wire = KiaV0Decoder::new().encode(&src, generic).unwrap();
            let mut got = feed_all(&mut KiaV0Decoder::new(), &wire);
            got.dedup(); // a Kia / Suzuki press holds its frame twice
            let fields: Vec<(Option<u8>, Option<u16>)> = got.iter().map(|d| (d.button, d.counter)).collect();
            let expected: Vec<(Option<u8>, Option<u16>)> = (0..).zip(want).map(|(i, &b)| (Some(b), Some(i))).collect();
            assert_eq!(fields, expected, "{ty:?} generic {generic:#x}: 0xFFFF + 1 wraps, one count a frame");
            assert!(got.iter().all(|d| d.serial == Some(serial) && d.crc_valid), "{ty:?} {got:?}");
        }
        // Mitsubishi V0 (its registry entry): every request keeps the captured button.
        let reg = ProtocolRegistry::new();
        let src = signal(KiaV0Type::Mitsubishi, 0x0123_4567, 0x6, 0x0041);
        for generic in [0x01u8, 0x02, 0x04, 0x08] {
            let wire = reg.get("Mitsubishi V0").unwrap().encode(&src, generic).unwrap();
            assert!(feed_all(&mut KiaV0Decoder::new(), &wire).iter().all(|d| d.button == Some(0x6) && d.counter == Some(0x42)));
        }
    }

    /// "Send next code" (`encode_next`) keeps the captured button even where its code is a generic
    /// one of another button: Kia's Panic 4 (generic Trunk), Suzuki's Panic 1, Trunk 2 and Unlock 4
    /// (generic Lock, Unlock and Trunk), Honda's Unlock 4, Mitsubishi's 4.
    #[test]
    fn send_next_code_keeps_the_captured_button() {
        let reg = ProtocolRegistry::new();
        for (ty, captured) in [
            (KiaV0Type::Kia, 0x4u8),
            (KiaV0Type::Suzuki, 0x1),
            (KiaV0Type::Suzuki, 0x2),
            (KiaV0Type::Suzuki, 0x4),
            (KiaV0Type::Honda, 0x4),
            (KiaV0Type::Mitsubishi, 0x4),
        ] {
            let serial = if ty == KiaV0Type::Honda { 0x00AB_CDEF } else { 0x0ABC_DEF1 };
            let src = DecodedSignal { protocol_display_name: Some(ty.display_name().into()), ..signal(ty, serial, captured, 0x0100) };
            let entry = if ty == KiaV0Type::Mitsubishi { "Mitsubishi V0" } else { "Kia V0" };
            let wire = super::super::tx_copies::transmission(reg.get(entry).unwrap(), &src, None, None).unwrap();
            let got = feed_all(&mut KiaV0Decoder::new(), &wire);
            assert!(!got.is_empty() && got.iter().all(|d| d.button == Some(captured) && d.counter == Some(0x0101)), "{ty:?} {captured}: {got:?}");
        }
    }

    /// Mitsubishi V0 = a repeated Kia frame behind a 72..88-pair preamble. Through the registry
    /// (which resets every decoder after each decode) the upload still reports Kia V0 then
    /// Mitsubishi V0, and the "Mitsubishi V0" name routes TX back to the Mitsubishi upload.
    #[test]
    fn registry_detects_mitsubishi_and_routes_tx() {
        let frame = build_frame(KiaV0Type::Mitsubishi, 0x0123_4567, 0x2, 0x0042);
        let wire = build_upload(KiaV0Type::Mitsubishi, frame);
        let mut reg = ProtocolRegistry::new();
        let names: Vec<String> =
            reg.process_signal_stream(&wire, 433_920_000).into_iter().map(|r| r.0).collect();
        assert_eq!(names, ["Kia V0", "Mitsubishi V0"]);

        let kia = build_upload(KiaV0Type::Kia, frame);
        let names: Vec<String> =
            reg.process_signal_stream(&kia, 315_000_000).into_iter().map(|r| r.0).collect();
        assert_eq!(names, ["Kia V0", "Kia V0"], "15-pair tail preamble stays Kia V0");

        let reg = ProtocolRegistry::new();
        let src = signal(KiaV0Type::Mitsubishi, 0x0123_4567, 0x2, 0x0041);
        let tx = reg.get("Mitsubishi V0").unwrap().encode(&src, 0x02).unwrap();
        let pulses = |v: &[LevelDuration]| v.iter().map(|p| (p.level, p.duration_us)).collect::<Vec<_>>();
        assert_eq!(pulses(&tx), pulses(&wire), "Mitsubishi V0 61-bit capture → Kia V0 Mitsubishi upload");
    }

    /// ProtoPirate's aliases: the retired standalone names and the type display names all reach
    /// the Kia V0 encoder, which picks the type from the bit count.
    #[test]
    fn legacy_and_display_names_resolve_to_kia_v0() {
        let reg = ProtocolRegistry::new();
        for name in ["Suzuki", "Suzuki V0", "SUZUKI", "Honda V0", "Kia V0"] {
            assert_eq!(reg.get(name).map(|d| d.name()), Some("Kia V0"), "{name}");
        }
        // A legacy "Suzuki" capture (old standalone decoder: same 64-bit layout and fields).
        let src = DecodedSignal {
            data: 0xF001_9153_0242_3C10,
            ..signal(KiaV0Type::Suzuki, 0x153_0242, 0x3, 0x0019)
        };
        let wire = reg.get("Suzuki").unwrap().encode(&src, 0x02).unwrap();
        let got = feed_all(&mut KiaV0Decoder::new(), &wire);
        assert_eq!(got[0].protocol_display_name.as_deref(), Some("Suzuki V0"));
        assert_eq!(
            (got[0].serial, got[0].button, got[0].counter),
            (Some(0x153_0242), Some(0x4), Some(0x001A))
        );
    }

    /// Native button labels for the Kia family (mod.rs `native_button_name`).
    #[test]
    fn native_button_names() {
        use crate::protocols::native_button_name as n;
        assert_eq!(n("Kia V0", 0x3), Some("Trunk"));
        assert_eq!(n("Mitsubishi V0", 0x4), Some("Btn 0x4"), "no source names the Mitsubishi codes");
        assert_eq!(n("Suzuki V0", 0x3), Some("Lock"));
        assert_eq!(n("Suzuki", 0x4), Some("Unlock"));
        assert_eq!(n("Honda V0", 0x3), Some("Lock"), "rtl_433's names of the nibble sent");
        assert_eq!(n("Honda V0", 0x4), Some("Unlock"));
        assert_eq!(n("Honda V0", 0xF), Some("Trunk (long)"));
        assert_eq!(n("Honda V0", 0x6), Some("Panic (long)"));
        assert_eq!(n("Kia V1", 0x3), Some("Trunk"));
        assert_eq!(n("Kia V4", 0x8), Some("Horn"));
        assert_eq!(n("Kia V3/V4", 0x3), Some("Trunk"));
        assert_eq!(n("Kia V5", 0x1), Some("Lock"));
        assert_eq!(n("Kia V5", 0x2), Some("Unlock"));
        assert_eq!(n("Kia V6", 0x4), Some("Panic"));
        assert_eq!(n("Kia V7", 0x3), Some("Trunk"));
        assert_eq!(n("Kia V7", 0x8), Some("Trunk"), "the references name 8 Trunk (KAT's Panic sends it)");
    }

    /// Gates: a Kia-shaped frame with a wrong CRC that is not a Honda key, a short preamble, and a
    /// wrong bit count are all rejected.
    #[test]
    fn rejects_bad_frames() {
        let good = build_frame(KiaV0Type::Kia, 0x0123_4567, 0x1, 0x0010);
        let bad_crc = good ^ 0x01;
        assert!(!honda_key_valid(honda_transform(bad_crc & 0x0FFF_FFFF_FFFF_FFFF)));
        assert!(feed_all(&mut KiaV0Decoder::new(), &build_upload(KiaV0Type::Kia, bad_crc)).is_empty());

        // 14 preamble pairs (+ the leading 0 bit) is not enough; 15 is.
        for (pairs, want) in [(13usize, 0usize), (14, 1)] {
            let mut w = Vec::new();
            push_short_pairs(&mut w, pairs);
            push_data_pairs(&mut w, good, BIT_COUNT_KIA as u32);
            push(&mut w, true, KIA_GAP);
            assert_eq!(feed_all(&mut KiaV0Decoder::new(), &w).len(), want, "{pairs} pairs");
        }

        // 60 bits then the gap: no decode.
        let mut w = Vec::new();
        push_short_pairs(&mut w, 20);
        push_data_pairs(&mut w, good >> 1, BIT_COUNT_KIA as u32 - 1);
        push(&mut w, true, KIA_GAP);
        assert!(feed_all(&mut KiaV0Decoder::new(), &w).is_empty());
    }

    /// Honda V0 is reported on the LOW that completes the 61st bit (`kia_v0_decoder_try_honda`),
    /// before any gap.
    #[test]
    fn honda_commits_on_the_61st_bit() {
        let key = build_honda_key(0x00AB_CDEF, 0x3, 0x1234);
        let wire = build_upload(KiaV0Type::Honda, key);
        let mut dec = KiaV0Decoder::new();
        let at = wire.iter().position(|p| dec.feed(p.level, p.duration_us).is_some()).unwrap();
        assert_eq!(at, wire.len() - 2, "the last data LOW, before the closing HIGH 1000");
    }

    /// Key files (ProtoPirate `subghz_protocol_decoder_kia_serialize` / `_deserialize`): every
    /// type's decode round-trips with the C's lines, a Flipper-ARF file loads, and an edited Cnt
    /// rebuilds the key (CRC included) as the C does.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = KiaV0Decoder::new();
        for &(ty, _, _, _, frame, ..) in C_VECTORS {
            let ty = ty_of(ty);
            for d in feed_all(&mut KiaV0Decoder::new(), &build_upload(ty, frame)) {
                let text = assert_round_trip(&dec, &d);
                assert!(text.contains("\nProtocol: Kia V0\n"), "{text}");
            }
        }
        let first = feed_all(&mut KiaV0Decoder::new(), &build_upload(KiaV0Type::Kia, C_VECTORS[0].4)).remove(0);
        assert!(assert_round_trip(&dec, &first).ends_with(
            "Protocol: Kia V0\nBit: 61\nKey: 1F 12 34 12 34 56 72 4E\nSerial: 19088743\nBtn: 2\nCnt: 4660\nType: 1\n"
        ));
        // Mitsubishi V0 is saved as Kia V0 type 4 and reloads under its display name.
        let mitsu = DecodedSignal {
            protocol_display_name: Some("Mitsubishi V0".into()),
            ..signal_of(KiaV0Type::Mitsubishi, C_VECTORS[12].4)
        };
        let text = assert_round_trip(&dec, &mitsu);
        assert!(text.ends_with("Type: 4\n"), "{text}");

        // IMPORTS/HYUNDAI/HYU V0 N1.sub (Flipper-ARF "KIA/HYU V0", no Type line: 61 bits = Kia).
        let arf = key(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPreset2FSKDev476Async\nProtocol: KIA/HYU V0\nBit: 61\n\
             Key: 0F 73 91 67 BD 75 B2 BD\n",
        );
        let d = dec.from_flipper_key(&arf).unwrap();
        assert_eq!(d.protocol_display_name, None);
        assert_eq!((d.data, d.data_count_bit), (0x0F73_9167_BD75_B2BD, 61));
        assert_eq!((d.serial, d.button, d.counter, d.crc_valid), (Some(0x67B_D75B), Some(0x2), Some(0x7391), true));

        // An edited counter: the C rebuilds the key from Serial/Btn/Cnt.
        let mut edited = dec.to_flipper_key(&d).unwrap();
        edited.fields.retain(|(n, _)| n != "Cnt");
        edited.push_u32("Cnt", 0x7392);
        let e = dec.from_flipper_key(&edited).unwrap();
        assert_eq!((e.serial, e.button, e.counter, e.crc_valid), (Some(0x67B_D75B), Some(0x2), Some(0x7392), true));
        assert_eq!(e.data, build_frame(KiaV0Type::Kia, 0x67B_D75B, 0x2, 0x7392));
        // The C rejects any other bit count.
        let mut bad = edited.clone();
        bad.bits = 60;
        assert!(dec.from_flipper_key(&bad).is_none());
    }

    fn signal_of(ty: KiaV0Type, data: u64) -> DecodedSignal {
        super::signal(data, ty)
    }

    /// Real fob data: the IMPORTS key files `HYUNDAI/HYU V0 N1.sub` and `HYU V0_great_konserva.sub`
    /// (saved by Flipper-ARF as "KIA/HYU V0"). Their CRC checks and the encoder rebuilds each key
    /// exactly from its fields.
    #[test]
    fn real_key_files_rebuild() {
        for key in [0x0F73_9167_BD75_B2BDu64, 0x0F6F_4667_BD75_B2BF] {
            let f = parse_fields(key, KiaV0Type::Kia);
            assert!(f.crc_valid);
            assert_eq!((f.serial, f.button), (0x67B_D75B, 0x2));
            assert_eq!(build_frame(KiaV0Type::Kia, f.serial, f.button, f.counter), key);
        }
    }

    /// A Honda V0 frame from its 56 bits as sent.
    fn honda(wire: u64) -> HondaFrame {
        HondaFrame::from_key(honda_transform((0x0F << 56) | wire))
    }

    /// The Honda check is a CRC-8 (polynomial 0x21, init 0) over the counter and the button nibble:
    /// recorded frames (IMPORTS/honda/more: an OEM 2005 Civic remote, KEYDIY- and Xhorse-made ones, a
    /// 300-press set) and rtl_433's two OEM remotes (rtl_433_tests, Nidec OUCG8D-344H-A), every button
    /// seen (3, 4, 5, 6, F). An aftermarket "TS" clone sends a wrong check. The C's table check is this
    /// CRC for 3, 4, 5 and F, not for 6.
    #[test]
    fn honda_check_is_a_crc8_over_counter_and_button() {
        for (wire, what) in [
            (0x0470_BD66_F853_5Eu64, "HondaV0_BD661FCA_300lck #0"),
            (0x0471_BD66_F853_0C, "HondaV0_BD661FCA_300lck #1"),
            (0x059D_BD66_F854_F8, "HondaV0_BD661FCA_300ulk #0"),
            (0x059E_BD66_F854_0E, "HondaV0_BD661FCA_300ulk #1"),
            (0x02F2_1530_2455_A7, "Honda_v0_1tr_kd_hnd #0, trunk"),
            (0x02F3_1530_245F_9E, "Honda_v0_1tr_kd_hnd #1, trunk long"),
            (0x2C55_816B_B053_0B, "honda_civic_2005_27 #11, OEM"),
            (0x0090_12B7_4356_EF, "HondaV0_xHorse_LUTP #2, panic long"),
        ] {
            let h = honda(wire);
            assert!(h.check_ok() && h.five == 5, "{what}");
            let c = honda_crc(rev8((h.five << 4) | h.button), h.counter);
            assert_eq!(c == rev8(h.check), h.button != 6, "{what}: the C's table check");
        }
        // rtl_433's OEM recordings: counter, button, check.
        for (cnt, btn, check) in
            [(0x8647u16, 0x3u8, 0x46u8), (0x864A, 0xF, 0x02), (0x8649, 0x5, 0x9F), (0x8648, 0x4, 0xEC), (0x1545, 0x3, 0xC5), (0x154A, 0x6, 0x2D)]
        {
            assert_eq!(HondaFrame::check_of(cnt, btn), check, "{cnt:04X} {btn:X}");
        }
        // Honda_v0_Invalid_Chk_Honda_TS_sn3_300 #9: a clone at the OEM remote's counter, check 00.
        assert!(!honda(0x2C55_816B_B053_00).check_ok());
    }

    /// One Honda frame in 256 also passes the Kia CRC (its 61 decoder bits); the C then waits for a
    /// Kia gap that never comes and loses it. These five, from the recordings, are read now, and the
    /// same 61 bits closed by a Kia gap are still Kia V0.
    #[test]
    fn honda_frames_that_pass_the_kia_crc() {
        for wire in [0x058B_BD66_F853_57u64, 0x05C6_BD66_F854_0F, 0x0664_BD66_F854_A7, 0x038A_5F29_9755_F4, 0x0393_5F29_975F_9A] {
            assert!(verify_crc_poly((0x1F << 56) | wire), "{wire:014X} passes the Kia CRC");
            let h = honda(wire);
            assert!(h.check_ok());
            let mut press = Vec::new();
            push_honda_press(&mut press, h.key());
            let got = feed_all(&mut KiaV0Decoder::new(), &press);
            assert_eq!(got.len(), 1, "{wire:014X}");
            assert_eq!(got[0].protocol_display_name.as_deref(), Some("Honda V0"));
            assert_eq!(
                (got[0].serial, got[0].button, got[0].counter, got[0].crc_valid),
                (Some(h.id), Some(h.button), Some(h.counter), true)
            );
            let mut kia = press.clone();
            kia.pop();
            push(&mut kia, true, KIA_FRAME_GAP);
            let got = feed_all(&mut KiaV0Decoder::new(), &kia);
            assert_eq!((got.len(), got[0].protocol_display_name.as_deref()), (1, None), "Kia V0");
        }
    }

    /// Pulse kinds, HIGH upper case: short `s`, long `l`, the Kia gap `g`, the Suzuki gap `w`.
    fn kinds(wire: &[LevelDuration]) -> String {
        wire.iter()
            .map(|p| {
                let c = match p.duration_us {
                    151..=349 => 's',
                    401..=599 => 'l',
                    1300..=1700 => 'g',
                    1900..=2500 => 'w',
                    _ => '?',
                };
                if p.level { c.to_ascii_uppercase() } else { c }
            })
            .collect()
    }

    /// Every press re-encoded with its own button ("Send next code") is the remote's next press, all
    /// its bits, wherever the recording holds it (one count higher): Kia (two fobs), Suzuki (a
    /// KEYDIY-made remote), Mitsubishi (the 2011 Pajero), Honda (an OEM 2005 Civic, KEYDIY- and
    /// Xhorse-made remotes, two complete 300-press sets). A Honda trunk press is 5 then F, and TX Trunk
    /// from the press before sends both. The upload is also the remote's press pulse for pulse by kind,
    /// from 300 short pairs before its first long pair to its last data HIGH (the recordings cut the
    /// start of each preamble; the closing HIGH differs between Honda remotes, 250 or 500 µs).
    #[test]
    fn reencode_reproduces_the_next_press() {
        use super::super::f_common::test_support::{feed_resetting, sub_pulses};
        let (mut pairs, mut trunk, mut shapes) = (0, 0, 0);
        for rel in [
            "KIA/more/Kia_0_orig.sub",
            "KIA/more/kia_seed_raw.sub",
            "SUZUKI/Suzuki_v0_kd.sub",
            "MITSUBISHI/Mitsubishi pajero 2011.sub",
            "honda/more/honda_civic_2005_27.sub",
            "honda/more/honda_civic_2005_28.sub",
            "honda/more/honda_civic_2005_29.sub",
            "honda/more/honda_civic_2005_30.sub",
            "honda/more/Honda_V0_kd_10_10_10.sub",
            "honda/more/Honda_v0_1tr_kd_hnd.sub",
            "honda/more/HondaV0_xHorse_20lck.sub",
            "honda/more/HondaV0_xHorse_20trk.sub",
            "honda/more/HondaV0_BD661FCA_300lck.sub",
            "honda/more/HondaV0_BD661FCA_300ulk.sub",
        ] {
            let path = std::path::Path::new("IMPORTS").join(rel);
            if !path.is_file() {
                eprintln!("Skip: IMPORTS not found (run from crate root)");
                return;
            }
            let (_, wire) = sub_pulses(&path);
            let recording = kinds(&wire);
            let mut got = feed_resetting(&mut KiaV0Decoder::new(), &wire);
            got.retain(|d| d.protocol_display_name.as_deref() != Some("Mitsubishi V0")); // the Pajero's second copies
            got.dedup(); // a Kia / Suzuki press holds its frame two or three times
            let reg = ProtocolRegistry::new();
            let p = reg.get(if rel.contains("pajero") { "Mitsubishi V0" } else { "Kia V0" }).unwrap();
            let honda = got.first().and_then(|d| d.protocol_display_name.as_deref()) == Some("Honda V0");
            for (i, w) in got.windows(2).enumerate() {
                let (now, next) = (&w[0], &w[1]);
                if next.serial != now.serial || next.counter != now.counter.map(|c| c.wrapping_add(1)) {
                    continue;
                }
                if next.button == now.button {
                    let upload = p.encode_next(now).unwrap();
                    let mut sent = feed_all(&mut KiaV0Decoder::new(), &upload);
                    sent.retain(|d| d.protocol_display_name.as_deref() != Some("Mitsubishi V0"));
                    sent.dedup();
                    assert_eq!(sent, [next.clone()], "{rel} #{i}");
                    pairs += 1;
                    // from 300 short pairs before the first long HIGH to the last data HIGH
                    let us = kinds(&upload);
                    let b = us.as_bytes();
                    let start = us.find('L').unwrap() - 600;
                    let end = (0..b.len() - 1).rev().find(|&k| matches!(b[k], b'S' | b'L') && matches!(b[k + 1], b's' | b'l')).unwrap() + 1;
                    shapes += usize::from(recording.contains(&us[start..end]));
                } else if honda && next.button == Some(0x5) {
                    let after = got.get(i + 2).filter(|a| a.button == Some(0xF) && a.counter == next.counter.map(|c| c.wrapping_add(1)));
                    if let Some(after) = after {
                        let sent = feed_all(&mut KiaV0Decoder::new(), &p.encode(now, 0x04).unwrap());
                        assert_eq!(sent, [next.clone(), after.clone()], "{rel} #{i}: a trunk press");
                        trunk += 1;
                    }
                }
            }
        }
        // The 35 presses whose shape is not found have noise inside their recorded preamble (fewer than
        // 300 clean pairs before the sync: two of the Civic files, both Xhorse files); their frames match.
        assert_eq!((pairs, trunk, shapes), (760, 10, 725), "next presses, trunk presses, shapes found");
    }
}
