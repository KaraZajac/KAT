//! Ford V3 protocol decoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/ford_v3.c` and `ford_v3.h`
//! (current version: EU Manchester path + US cell path; Flipper-ARF's `ford_v3.c` is the same
//! code, doctor-firmware's has no US path). 240/480µs, 104 bits (13 bytes), AM/FM (C flags
//! AM|FM, 315|433). Decode-only — the reference encoder is NULL.
//!
//! Both decoders see every pulse (C order: US cell path first, then EU Manchester path):
//!
//! **EU path** (`ford_v3_manchester_feed`, te_delta=60µs): Reset → Preamble (≥30 short pulses) →
//! the first long pulse seeds the Manchester state to Mid1 and is fed as `level ? LongHigh :
//! LongLow` → Data, events `level ? High : Low`. The C uses Flipper's `manchester_advance`
//! (`lib/toolbox/manchester_decoder.c`); this module ports that function literally — Flipper state
//! numbering (Start1=0, Mid1=1, Mid0=2, Start0=3), event codes (ShortLow=0, ShortHigh=2,
//! LongLow=4, LongHigh=6), transition bytes {0x01, 0x91, 0x9B, 0xFB}, "same state → reset to Mid1,
//! no bit", entering Mid1/Mid0 emits 1/0. (The earlier port used KAT's shared table, which is not
//! equivalent: on `IMPORTS/LDV` it produced bits that are no Manchester decode of the stream,
//! e.g. serial DF56FD7D, where the C and a direct half-bit decode both give 43A88F90.) Bits are
//! packed MSB-first; at 104 bits the frame is committed as EU. No other check exists in the C.
//!
//! **US path** (`ford_v3_cell_feed/cell_process/cell_decode`, te_delta=120µs): every short pulse
//! appends one half-bit cell of its level, every long pulse two (capacity 320 cells, extra cells
//! dropped). Any other pulse ends the burst: with ≥200 cells the burst is decoded from its first
//! cell (phase 0, then phase 1) as bit = first cell of each pair, stopping at 104 bits; every pair
//! must differ and ≥100 bits are required. Gate (`ford_v3_cell_frame_valid`): raw[0] == 0xFF,
//! serial raw[1..=4] ≠ 0 / 0xFFFFFFFF, raw[6] ∈ {0x01 Lock, 0x02 Unlock, 0x07 Trunk, 0x06 Panic}.
//!
//! **KAT deviation (US gate), from real-capture evidence.** The C additionally requires raw[5]
//! bit 7 and accepts only buttons 1/2. The genuine 2022 Ford Escape capture
//! `IMPORTS/FORD/22_ford_escape_u2_l2_t2_p2.sub` (315 MHz) carries raw[5] = 0x08 in every frame and
//! uses 0x07/0x06 for Trunk/Panic, so the C rejects all of it. With the flag check dropped and the
//! two buttons added it decodes as one fob: serial 0x4E2464E4 in every frame, the counter rising by
//! one per press (0xEB0D → 0xEB16), and buttons 2,1,7,6 in exactly the order the file name records
//! (u2 l2 t2 p2); no other IMPORTS capture changes. The structural gates (burst-anchored ≥100-bit
//! Manchester cell frame at 240/480µs, 0xFF header, sane serial) are unchanged.
//!
//! **KAT deviation (US frame ends the burst), from real-capture evidence.** The C reads the first
//! 104 bits of any burst of ≥200 cells and ignores the rest. A volunteer's Alfa Romeo Giulia
//! capture (433.92 MHz, not in IMPORTS) showed what that costs in KAT's first-match registry: the
//! car's 112-bit Fiat V2 frames, read in the opposite polarity, start FF FE (the inverted 00 01
//! markers), and whenever the Fiat type byte is 0xFE its inverse 0x01 is a valid US button. Fiat
//! V2 and this path finish on the same gap after the frame, Ford V3 comes first in the registry,
//! and 28 of the 30 unlock frames came out as Ford V3 (3 of its 5 codes lost). Genuine US bursts
//! (`IMPORTS/FORD/22_ford_escape*`) are exactly one frame: 208 or 209 cells, at most one unpaired
//! cell after bit 104. KAT therefore rejects a US frame when a whole bit follows it in the burst;
//! the Giulia bursts carry 16 more cells (Fiat V2's last byte) and decode as Fiat V2.
//!
//! **Fields** (`ford_v3_parse_fields`): serial = raw[1..=4] (both variants).
//! - EU: counter = `~raw[7] << 8 | ~raw[8]`, button = raw[6] bit 0 ? Unlock : Lock.
//! - US: flag = raw[5], counter = `raw[7] << 8 | raw[8]`, button = raw[6].
//!
//! raw[9..=12] is the hop (displayed by the C, not parsed).
//!
//! **Dedup** (`ford_v3_commit_frame`): a frame identical to the last committed one (either
//! variant) is not reported again until `reset()` — KAT's registry resets every decoder after
//! each hit, so in a capture stream each repeat is still reported, as before.
//!
//! **Packing**: `data` = raw[0..8] big-endian (the C's `Key`), `extra` = raw[8..13] right-aligned
//! (40 bits: the counter's low byte and the hop — the C saves the whole frame as `Raw`),
//! `data_count_bit` = 104. The variant is not stored; as in the C's deserialize it can be
//! re-derived (`cell_frame_valid(raw)` ⇒ US).
//!
//! **Key files** (`subghz_protocol_decoder_ford_v3_serialize` / `_deserialize`; Flipper-ARF's are
//! the same): `Protocol: Ford V3`, `Bit: 104`, `Key` = raw[0..8], then `Raw` (all 13 bytes),
//! `Serial`, `Btn`, `Cnt` and `Variant` (0 EU, 1 US). KAT writes the variant whose fields the
//! decode reports (the two read the counter bytes inverted vs plain, so exactly one matches).
//! Loading needs Bit 104, takes the frame from Key (+ zeros) overlaid by `Raw`, the variant from
//! `Variant`, and — without that line — from the C's own US check (`ford_v3_cell_frame_valid`:
//! 0xFF header, sane serial, flag bit 7, button 1/2; stricter than KAT's US decoder gate), then
//! parses the fields as the decoder does (the file's Serial/Btn/Cnt are not read, as in the C).
//! Decode-only, like the reference: a loaded key can be listed and exported, not transmitted.

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): ProtoPirate "V3 [AM650+F4 434]" (EU + US variants).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Ford V3",
    decoder: "Ford V3",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::Both,
    encryption: "Rolling Code",
    make: "Ford",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650, &F4],
};

/// US variant; the EU variant reports 0x01/0x02 (generic Lock/Unlock).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x07 => "Trunk",
        0x06 => "Panic",
        _ => return None,
    })
}

const TE_SHORT: u32 = 240;
const TE_LONG: u32 = 480;
const TE_DELTA: u32 = 60;
const CELL_TE_DELTA: u32 = 120;
const DATA_BITS: usize = 104;
const DATA_BYTES: usize = 13;
const PREAMBLE_MIN: u16 = 30;
const CELL_CAP: usize = 320;
const CELL_MIN: usize = 200;
const CELL_MIN_BITS: usize = 100;

const BTN_LOCK: u8 = 0x01;
const BTN_UNLOCK: u8 = 0x02;

/// Flipper `ManchesterState` (lib/toolbox/manchester_decoder.h numbering).
#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Start1 = 0,
    Mid1 = 1,
    Mid0 = 2,
    Start0 = 3,
}

/// Flipper `ManchesterEvent` values.
const EVENT_SHORT_LOW: u8 = 0;
const EVENT_SHORT_HIGH: u8 = 2;
const EVENT_LONG_LOW: u8 = 4;
const EVENT_LONG_HIGH: u8 = 6;

/// Flipper manchester_decoder.c `transitions[]`.
const MANCHESTER_TRANSITIONS: [u8; 4] = [0x01, 0x91, 0x9B, 0xFB];

/// Decoder step states (matches FordV3DecoderStep in ford_v3.c)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    Data,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Variant {
    Eu,
    Us,
}

/// Ford V3 protocol decoder (matches SubGhzProtocolDecoderFordV3)
pub struct FordV3Decoder {
    step: DecoderStep,
    manchester_state: ManchesterState,
    manchester_raw: [u8; DATA_BYTES],
    manchester_bit_count: usize,
    preamble_count: u16,
    cells: [bool; CELL_CAP],
    cell_count: usize,
    last_raw: Option<[u8; DATA_BYTES]>,
}

impl FordV3Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: ManchesterState::Mid1,
            manchester_raw: [0; DATA_BYTES],
            manchester_bit_count: 0,
            preamble_count: 0,
            cells: [false; CELL_CAP],
            cell_count: 0,
            last_raw: None,
        }
    }

    fn is_short(duration: u32) -> bool {
        duration_diff!(duration, TE_SHORT) < TE_DELTA
    }

    fn is_long(duration: u32) -> bool {
        duration_diff!(duration, TE_LONG) < TE_DELTA
    }

    fn is_cell_short(duration: u32) -> bool {
        duration_diff!(duration, TE_SHORT) < CELL_TE_DELTA
    }

    fn is_cell_long(duration: u32) -> bool {
        duration_diff!(duration, TE_LONG) < CELL_TE_DELTA
    }

    /// Flipper `manchester_advance` (lib/toolbox/manchester_decoder.c), verbatim.
    fn manchester_advance(&mut self, event: u8) -> Option<bool> {
        let state = self.manchester_state as u8;
        let next = (MANCHESTER_TRANSITIONS[state as usize] >> event) & 0x03;
        if next == state {
            self.manchester_state = ManchesterState::Mid1;
            return None;
        }
        self.manchester_state = match next {
            0 => ManchesterState::Start1,
            1 => ManchesterState::Mid1,
            2 => ManchesterState::Mid0,
            _ => ManchesterState::Start0,
        };
        match self.manchester_state {
            ManchesterState::Mid1 => Some(true),
            ManchesterState::Mid0 => Some(false),
            _ => None,
        }
    }

    /// ford_v3_reset_manchester
    fn reset_manchester(&mut self) {
        self.manchester_raw = [0; DATA_BYTES];
        self.manchester_bit_count = 0;
        self.preamble_count = 0;
        self.manchester_state = ManchesterState::Mid1;
    }

    /// ford_v3_add_manchester_bit: MSB-first into the 13-byte buffer.
    fn add_manchester_bit(&mut self, bit: bool) {
        if self.manchester_bit_count >= DATA_BITS {
            return;
        }
        if bit {
            let i = self.manchester_bit_count;
            self.manchester_raw[i / 8] |= 1 << (7 - (i % 8));
        }
        self.manchester_bit_count += 1;
    }

    /// ford_v3_cell_frame_valid (the US gate; also the C's variant detector).
    fn cell_frame_valid(raw: &[u8; DATA_BYTES]) -> bool {
        if raw[0] != 0xFF {
            return false;
        }
        let serial = u32::from_be_bytes([raw[1], raw[2], raw[3], raw[4]]);
        if serial == 0 || serial == u32::MAX {
            return false;
        }
        if !matches!(raw[6], 0x01 | 0x02 | 0x06 | 0x07) {
            return false;
        }
        true
    }

    /// The C's `ford_v3_cell_frame_valid` as written (flag bit 7 set, buttons 1/2 only): how its
    /// deserialize picks the variant of a key file without a `Variant` line.
    fn c_cell_frame_valid(raw: &[u8; DATA_BYTES]) -> bool {
        Self::cell_frame_valid(raw) && raw[5] & 0x80 != 0 && matches!(raw[6], BTN_LOCK | BTN_UNLOCK)
    }

    /// ford_v3_cell_decode: phase 0 then phase 1, bit = first cell of each pair.
    fn cell_decode(cells: &[bool]) -> Option<[u8; DATA_BYTES]> {
        for phase in 0..2 {
            let mut frame = [0u8; DATA_BYTES];
            let mut bit_count = 0usize;
            let mut ok = true;
            let mut i = phase;
            while i + 1 < cells.len() && bit_count < DATA_BITS {
                let first = cells[i];
                if first == cells[i + 1] {
                    ok = false;
                    break;
                }
                if first {
                    frame[bit_count >> 3] |= 1 << (7 - (bit_count & 7));
                }
                bit_count += 1;
                i += 2;
            }
            // KAT: the frame must end the burst, i.e. no whole bit after it (see the module doc,
            // "frame ends the burst"); `i` is the first cell not read.
            if !ok
                || bit_count < CELL_MIN_BITS
                || cells.len() - i > 1
                || !Self::cell_frame_valid(&frame)
            {
                continue;
            }
            return Some(frame);
        }
        None
    }

    /// ford_v3_commit_frame: dedup against the last committed frame, then build the signal.
    fn commit_frame(&mut self, raw: [u8; DATA_BYTES], variant: Variant) -> Option<DecodedSignal> {
        if self.last_raw == Some(raw) {
            return None;
        }
        self.last_raw = Some(raw);
        Some(Self::build_signal(&raw, variant))
    }

    /// ford_v3_parse_fields + serialize packing.
    fn build_signal(b: &[u8; DATA_BYTES], variant: Variant) -> DecodedSignal {
        let serial = u32::from_be_bytes([b[1], b[2], b[3], b[4]]);
        let (counter, button) = match variant {
            Variant::Us => (u16::from_be_bytes([b[7], b[8]]), b[6]),
            Variant::Eu => (
                u16::from_be_bytes([!b[7], !b[8]]),
                if b[6] & 0x01 != 0 { BTN_UNLOCK } else { BTN_LOCK },
            ),
        };
        let mut data_bytes = [0u8; 8];
        data_bytes.copy_from_slice(&b[..8]);
        let extra = b[8..].iter().fold(0u64, |acc, &x| (acc << 8) | x as u64);

        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: true, // Ford V3 is plaintext with no CRC
            data: u64::from_be_bytes(data_bytes),
            data_count_bit: DATA_BITS,
            encoder_capable: false,
            extra: Some(extra),
            protocol_display_name: None,
        }
    }

    /// ford_v3_cell_feed (US path).
    fn cell_feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if Self::is_cell_short(duration) {
            if self.cell_count < CELL_CAP {
                self.cells[self.cell_count] = level;
                self.cell_count += 1;
            }
        } else if Self::is_cell_long(duration) {
            if self.cell_count + 2 <= CELL_CAP {
                self.cells[self.cell_count] = level;
                self.cells[self.cell_count + 1] = level;
                self.cell_count += 2;
            }
        } else {
            // ford_v3_cell_process
            let frame = if self.cell_count >= CELL_MIN {
                Self::cell_decode(&self.cells[..self.cell_count])
            } else {
                None
            };
            self.cell_count = 0;
            if let Some(raw) = frame {
                return self.commit_frame(raw, Variant::Us);
            }
        }
        None
    }

    /// ford_v3_manchester_emit_if_ready
    fn manchester_emit_if_ready(&mut self) -> Option<DecodedSignal> {
        if self.manchester_bit_count < DATA_BITS {
            return None;
        }
        self.commit_frame(self.manchester_raw, Variant::Eu)
    }

    /// ford_v3_manchester_feed (EU path).
    fn manchester_feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if Self::is_short(duration) {
                    self.reset_manchester();
                    self.preamble_count = 1;
                    self.step = DecoderStep::Preamble;
                }
            }

            DecoderStep::Preamble => {
                if Self::is_short(duration) {
                    self.preamble_count = self.preamble_count.wrapping_add(1);
                } else if self.preamble_count >= PREAMBLE_MIN && Self::is_long(duration) {
                    self.manchester_state = ManchesterState::Mid1;
                    let event = if level { EVENT_LONG_HIGH } else { EVENT_LONG_LOW };
                    if let Some(bit) = self.manchester_advance(event) {
                        self.add_manchester_bit(bit);
                    }
                    self.step = DecoderStep::Data;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::Data => {
                let short = Self::is_short(duration);
                if !short && !Self::is_long(duration) {
                    // Gap / out-of-range pulse ends the frame. (The C's follow-up "if short,
                    // restart the preamble" can never trigger here.)
                    self.step = DecoderStep::Reset;
                    return self.manchester_emit_if_ready();
                }

                let event = match (level, short) {
                    (true, true) => EVENT_SHORT_HIGH,
                    (true, false) => EVENT_LONG_HIGH,
                    (false, true) => EVENT_SHORT_LOW,
                    (false, false) => EVENT_LONG_LOW,
                };
                if let Some(bit) = self.manchester_advance(event) {
                    self.add_manchester_bit(bit);
                    if self.manchester_bit_count >= DATA_BITS {
                        self.step = DecoderStep::Reset;
                        return self.manchester_emit_if_ready();
                    }
                }
            }
        }
        None
    }
}

impl ProtocolDecoder for FordV3Decoder {
    fn name(&self) -> &'static str {
        "Ford V3"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: DATA_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.reset_manchester();
        self.cell_count = 0;
        self.last_raw = None;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        // C order: cell (US) path first, then the Manchester (EU) path; both always run. They
        // cannot both complete on the same pulse (the EU path completes on an in-range pulse,
        // the US path only on an out-of-range one), so returning the first hit loses nothing.
        let us = self.cell_feed(level, duration);
        let eu = self.manchester_feed(level, duration);
        us.or(eu)
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// `subghz_protocol_decoder_ford_v3_deserialize` (see the module doc, "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != DATA_BITS {
            return None;
        }
        let mut raw = [0u8; DATA_BYTES];
        raw[..8].copy_from_slice(&key.key_u64()?.to_be_bytes());
        if let Some(full) = key.hex_bytes_field("Raw").filter(|r| r.len() == DATA_BYTES) {
            raw.copy_from_slice(&full);
        }
        let variant = match key.u32_field("Variant") {
            Some(0) => Variant::Eu,
            Some(1) => Variant::Us,
            _ if Self::c_cell_frame_valid(&raw) => Variant::Us,
            _ => Variant::Eu,
        };
        Some(Self::build_signal(&raw, variant))
    }

    /// `subghz_protocol_decoder_ford_v3_serialize` (see the module doc, "Key files").
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let extra = decoded.extra?;
        if decoded.data_count_bit != DATA_BITS || extra >> 40 != 0 {
            return None;
        }
        let mut raw = [0u8; DATA_BYTES];
        raw[..8].copy_from_slice(&decoded.data.to_be_bytes());
        raw[8..].copy_from_slice(&extra.to_be_bytes()[3..]);
        let variant = [Variant::Us, Variant::Eu]
            .into_iter()
            .find(|&v| {
                let s = Self::build_signal(&raw, v);
                (s.button, s.counter) == (decoded.button, decoded.counter)
            })
            .unwrap_or(if Self::c_cell_frame_valid(&raw) { Variant::Us } else { Variant::Eu });
        let s = Self::build_signal(&raw, variant);
        let mut key = FlipperKey::for_protocol("Ford V3", DATA_BITS, decoded.data);
        key.push_hex_bytes("Raw", &raw);
        key.push_u32("Serial", s.serial?);
        key.push_u32("Btn", s.button?.into());
        key.push_u32("Cnt", s.counter?.into());
        key.push_u32("Variant", if variant == Variant::Us { 1 } else { 0 });
        Some(key)
    }
}

impl Default for FordV3Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export::flipper::import_sub_raw;
    use crate::protocols::ProtocolRegistry;
    use std::path::Path;

    /// The first LDV-T80 lock frame as ProtoPirate decodes it.
    const RAW_EU: [u8; DATA_BYTES] =
        [0x44, 0x43, 0xA8, 0x8F, 0x90, 0x65, 0x68, 0x00, 0x00, 0x09, 0x48, 0xED, 0xD7];
    /// A US-layout frame (0xFF header, flag bit 7 set, Unlock).
    const RAW_US: [u8; DATA_BYTES] =
        [0xFF, 0x4E, 0x24, 0x64, 0xE4, 0x88, 0x02, 0xEB, 0x0D, 0xD7, 0xDC, 0xEF, 0xA6];

    fn bit(raw: &[u8; DATA_BYTES], i: usize) -> bool {
        (raw[i / 8] >> (7 - (i % 8))) & 1 != 0
    }

    fn cells_to_pulses(cells: &[bool], out: &mut Vec<LevelDuration>) {
        for &c in cells {
            match out.last_mut() {
                Some(last) if last.level == c => {
                    *last = LevelDuration::new(c, last.duration_us + TE_SHORT);
                }
                _ => out.push(LevelDuration::new(c, TE_SHORT)),
            }
        }
    }

    /// EU air format (matches the LDV captures): short-pulse preamble, a sync "1" whose first
    /// half-bit merges with the last preamble LOW into the first long pulse (no bit emitted),
    /// then each bit b as half-bits (!b, b).
    fn eu_pulses(raw: &[u8; DATA_BYTES]) -> Vec<LevelDuration> {
        let mut cells = Vec::new();
        for _ in 0..20 {
            cells.extend([true, false]);
        }
        cells.extend([false, true]); // sync bit
        for i in 0..DATA_BITS {
            let b = bit(raw, i);
            cells.extend([!b, b]);
        }
        let mut out = vec![LevelDuration::new(false, 10_000)];
        cells_to_pulses(&cells, &mut out);
        out.push(LevelDuration::new(!out.last().unwrap().level, 10_000));
        out
    }

    /// US air format (matches IMPORTS/FORD/22_ford_escape*): each bit b as half-bits (b, !b),
    /// framed by gaps.
    fn us_pulses(raw: &[u8; DATA_BYTES]) -> Vec<LevelDuration> {
        let mut cells = Vec::new();
        for i in 0..DATA_BITS {
            let b = bit(raw, i);
            cells.extend([b, !b]);
        }
        let mut out = vec![LevelDuration::new(false, 10_000)];
        cells_to_pulses(&cells, &mut out);
        out.push(LevelDuration::new(!out.last().unwrap().level, 10_000));
        out
    }

    fn decode(pulses: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut decoder = FordV3Decoder::new();
        pulses
            .iter()
            .filter_map(|p| decoder.feed(p.level, p.duration_us))
            .collect()
    }

    #[test]
    fn manchester_advance_matches_flipper_table() {
        // (state, event) -> (next state, bit), straight from manchester_decoder.c.
        use ManchesterState::*;
        let cases = [
            (Start1, EVENT_SHORT_LOW, Mid1, Some(true)),
            (Start1, EVENT_SHORT_HIGH, Mid1, None),
            (Mid1, EVENT_SHORT_HIGH, Start1, None),
            (Mid1, EVENT_LONG_HIGH, Mid0, Some(false)),
            (Mid1, EVENT_LONG_LOW, Mid1, None),
            (Mid0, EVENT_SHORT_LOW, Start0, None),
            (Mid0, EVENT_LONG_LOW, Mid1, Some(true)),
            (Mid0, EVENT_LONG_HIGH, Mid1, None),
            (Start0, EVENT_SHORT_HIGH, Mid0, Some(false)),
            (Start0, EVENT_LONG_LOW, Mid1, None),
        ];
        for (state, event, next, bit) in cases {
            let mut d = FordV3Decoder::new();
            d.manchester_state = state;
            assert_eq!(d.manchester_advance(event), bit, "{state:?} ev {event}");
            assert_eq!(d.manchester_state, next, "{state:?} ev {event}");
        }
    }

    #[test]
    fn eu_synthetic_frame_decodes() {
        let decoded = decode(&eu_pulses(&RAW_EU));
        assert_eq!(decoded.len(), 1);
        let d = &decoded[0];
        assert_eq!(d.serial, Some(0x43A8_8F90));
        assert_eq!(d.data, 0x4443_A88F_9065_6800);
        assert_eq!(d.counter, Some(0xFFFF)); // ~0x00, ~0x00
        assert_eq!(d.button, Some(BTN_LOCK)); // 0x68 bit 0 clear
        assert_eq!(d.data_count_bit, DATA_BITS);
        assert!(!d.encoder_capable);

        let mut unlock = RAW_EU;
        unlock[6] = 0x69;
        unlock[7] = 0x12;
        let d = &decode(&eu_pulses(&unlock))[0];
        assert_eq!(d.button, Some(BTN_UNLOCK));
        assert_eq!(d.counter, Some(0xEDFF));
    }

    #[test]
    fn eu_requires_preamble() {
        // Drop most of the preamble: fewer than 30 shorts before the first long.
        let pulses = eu_pulses(&RAW_EU);
        let trimmed: Vec<LevelDuration> = pulses[..1].iter().chain(&pulses[20..]).copied().collect();
        assert!(decode(&trimmed).is_empty());
    }

    #[test]
    fn us_synthetic_frame_decodes_with_us_fields() {
        let decoded = decode(&us_pulses(&RAW_US));
        assert_eq!(decoded.len(), 1);
        let d = &decoded[0];
        assert_eq!(d.serial, Some(0x4E24_64E4));
        assert_eq!(d.button, Some(BTN_UNLOCK));
        assert_eq!(d.counter, Some(0xEB0D)); // not inverted for US
        assert_eq!(d.data, 0xFF4E_2464_E488_02EB);
        assert_eq!(d.data_count_bit, DATA_BITS);
    }

    #[test]
    fn us_gate_rejects_invalid_frames() {
        // KAT accepts the flag byte without bit 7 and the Trunk/Panic codes (real 2022 Escape).
        for (flag, btn) in [(0x08u8, 0x02u8), (0x88, 0x07), (0x08, 0x06)] {
            let mut ok = RAW_US;
            ok[5] = flag;
            ok[6] = btn;
            assert!(FordV3Decoder::cell_frame_valid(&ok), "flag {flag:02X} btn {btn:02X}");
        }
        let mut bad_button = RAW_US;
        bad_button[6] = 0x03;
        let mut header = RAW_US;
        header[0] = 0xFE;
        let mut serial_ones = RAW_US;
        serial_ones[1..5].copy_from_slice(&[0xFF; 4]);
        for raw in [bad_button, header, serial_ones] {
            assert!(!FordV3Decoder::cell_frame_valid(&raw));
            assert!(decode(&us_pulses(&raw)).is_empty(), "{raw:02X?}");
        }
        // A broken Manchester pair: one pulse gains/loses a half-bit, misaligning the pairs.
        let pulses = us_pulses(&RAW_US);
        let mut broken = pulses.clone();
        let stretched = if broken[40].duration_us == TE_SHORT { TE_LONG } else { TE_SHORT };
        broken[40] = LevelDuration::new(broken[40].level, stretched);
        assert!(decode(&broken).is_empty());
        // Too short a burst (< 200 cells) is never decoded.
        let short: Vec<LevelDuration> = pulses[..90].iter().copied().chain([LevelDuration::new(false, 10_000)]).collect();
        assert!(decode(&short).is_empty());
    }

    /// A Fiat V2 burst (112 bits, doctor-firmware's waveform: bit b as 210µs half-bits (!b, b))
    /// whose first 13 bytes, read in this path's polarity, are a valid US frame: FF FE header and
    /// the Fiat type byte 0xFE inverted to button 0x01. Made-up UID and hop; the Alfa Romeo Giulia
    /// capture that showed it is a volunteer's, not in IMPORTS. The US path must leave the burst to
    /// Fiat V2, yet still read those 13 bytes when they are the whole burst.
    #[test]
    fn us_path_leaves_longer_fiat_v2_bursts_alone() {
        let fiat: [u8; 14] =
            [0x00, 0x01, 0x5E, 0xED, 0x01, 0x23, 0xFE, 0xFB, 0xF9, 0x12, 0x34, 0x56, 0x78, 0x9A];
        let burst = |bytes: &[u8]| {
            let mut out = vec![LevelDuration::new(false, 20_000)];
            for i in 0..bytes.len() * 8 {
                let b = (bytes[i / 8] >> (7 - i % 8)) & 1 != 0;
                for c in [!b, b] {
                    match out.last_mut() {
                        Some(last) if last.level == c => *last = LevelDuration::new(c, last.duration_us + 210),
                        _ => out.push(LevelDuration::new(c, 210)),
                    }
                }
            }
            match out.last_mut() {
                Some(last) if !last.level => *last = LevelDuration::new(false, last.duration_us + 20_000),
                _ => out.push(LevelDuration::new(false, 20_000)),
            }
            out
        };
        let whole = burst(&fiat);
        assert!(decode(&whole).is_empty(), "16 cells follow the 104th bit");
        let names: Vec<String> = ProtocolRegistry::new()
            .process_signal_stream(&whole, 433_920_000)
            .into_iter()
            .map(|(n, _, _)| n)
            .collect();
        assert_eq!(names, ["Fiat V2"]);

        let first13 = decode(&burst(&fiat[..13]));
        assert_eq!(first13.len(), 1, "the same bits as a burst of their own");
        assert_eq!((first13[0].serial, first13[0].button), (Some(0xFEA1_12FE), Some(BTN_LOCK)));
    }

    #[test]
    fn duplicate_frames_are_suppressed_until_reset() {
        let mut pulses = us_pulses(&RAW_US);
        pulses.extend(us_pulses(&RAW_US));
        assert_eq!(decode(&pulses).len(), 1);

        let mut decoder = FordV3Decoder::new();
        let mut hits = 0;
        for p in &pulses {
            if decoder.feed(p.level, p.duration_us).is_some() {
                hits += 1;
                decoder.reset();
            }
        }
        assert_eq!(hits, 2);
    }

    #[test]
    fn ldv_t80_frames_match_protopirate_fields() {
        // With Flipper's manchester_advance the LDV-T80 frames decode to the same bits as the C
        // (serial 43A88F90 in both captures, button byte 0x68 lock / 0x70 unlock).
        for (rel, count, b6) in [("LDV/LDV-T80_lock.sub", 4, 0x68u64), ("LDV/LDV-T80_unlock.sub", 3, 0x70)] {
            let path = Path::new("IMPORTS").join(rel);
            if !path.exists() {
                eprintln!("Skip: {:?} not found (run from crate root)", path);
                continue;
            }
            let (freq, raw_pairs) = import_sub_raw(&path).unwrap();
            let pairs: Vec<LevelDuration> = raw_pairs
                .iter()
                .map(|p| LevelDuration::new(p.level, p.duration_us))
                .collect();
            let results = ProtocolRegistry::new().process_signal_stream(&pairs, freq);
            let ford: Vec<_> = results.iter().filter(|(n, _, _)| n == "Ford V3").collect();
            assert_eq!(ford.len(), count, "{rel}");
            for (_, d, _) in ford {
                assert_eq!(d.serial, Some(0x43A8_8F90), "{rel}");
                assert_eq!(d.data >> 56, 0x44, "{rel}");
                assert_eq!((d.data >> 8) & 0xFF, b6, "{rel}");
            }
        }
    }

    /// Genuine 2022 Ford Escape (US, 315 MHz): one fob, counter rising per press, buttons in the
    /// order the file name records (u2 l2 t2 p2 → 0x02, 0x01, 0x07, 0x06). Guards the US gate.
    #[test]
    fn us_decodes_real_2022_escape_capture() {
        use crate::export::flipper::import_sub_raw;
        use crate::protocols::ProtocolRegistry;
        let path = std::path::Path::new("IMPORTS/FORD/22_ford_escape_u2_l2_t2_p2.sub");
        if !path.exists() {
            eprintln!("Skip: {path:?} not found (run from crate root)");
            return;
        }
        let (freq, raw_pairs) = import_sub_raw(path).unwrap();
        let pairs: Vec<LevelDuration> = raw_pairs
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let mut reg = ProtocolRegistry::new();
        let hits: Vec<DecodedSignal> = reg
            .process_signal_stream(&pairs, freq)
            .into_iter()
            .filter(|(n, _, _)| n == "Ford V3")
            .map(|(_, d, _)| d)
            .collect();
        assert!(hits.len() >= 8, "expected the Escape's presses to decode, got {}", hits.len());
        assert!(hits.iter().all(|d| d.serial == Some(0x4E24_64E4)));
        let counters: Vec<u16> = hits.iter().map(|d| d.counter.unwrap()).collect();
        assert!(counters.windows(2).all(|w| w[0] <= w[1]), "counter must not go backwards: {counters:?}");
        let mut order: Vec<u8> = Vec::new();
        for d in &hits {
            let b = d.button.unwrap();
            if order.last() != Some(&b) {
                order.push(b);
            }
        }
        assert_eq!(order, vec![0x02, 0x01, 0x07, 0x06]);
    }

    /// Key files (ProtoPirate `subghz_protocol_decoder_ford_v3_serialize` / `_deserialize`): EU and
    /// US decodes round-trip with the C's lines (Raw, Serial, Btn, Cnt, Variant), a file without
    /// Variant / Raw loads as the C loads it, and Bit must be 104.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = FordV3Decoder::new();
        let eu = decode(&eu_pulses(&RAW_EU)).remove(0);
        let text = assert_round_trip(&dec, &eu);
        assert!(
            text.ends_with(
                "Protocol: Ford V3\nBit: 104\nKey: 44 43 A8 8F 90 65 68 00\n\
                 Raw: 44 43 A8 8F 90 65 68 00 00 09 48 ED D7\nSerial: 1135120272\nBtn: 1\nCnt: 65535\nVariant: 0\n"
            ),
            "{text}"
        );
        let us = decode(&us_pulses(&RAW_US)).remove(0);
        let text = assert_round_trip(&dec, &us);
        assert!(text.ends_with("Serial: 1311007972\nBtn: 2\nCnt: 60173\nVariant: 1\n"), "{text}");
        let (name, loaded) = crate::protocols::ProtocolRegistry::new().decode_key(&key(&text)).unwrap();
        assert_eq!((name.as_str(), &loaded), ("Ford V3", &us));

        // No Variant line: the C's own US check decides (RAW_US has flag bit 7 and button 2).
        for (d, raw) in [(&eu, RAW_EU), (&us, RAW_US)] {
            let mut k = dec.to_flipper_key(d).unwrap();
            k.fields.retain(|(n, _)| n != "Variant");
            assert_eq!(dec.from_flipper_key(&k).as_ref(), Some(d), "{raw:02X?}");
            // No Raw either: only the Key's 8 bytes, the tail is zero.
            k.fields.retain(|(n, _)| n != "Raw");
            let short = dec.from_flipper_key(&k).unwrap();
            assert_eq!((short.serial, short.data, short.extra), (d.serial, d.data, Some(0)));
        }
        // A real 2022 Escape frame (flag 0x08, KAT's relaxed US gate): its Variant line keeps it US.
        let mut escape = RAW_US;
        escape[5] = 0x08;
        let e = decode(&us_pulses(&escape)).remove(0);
        assert_round_trip(&dec, &e);
        let mut wrong = dec.to_flipper_key(&eu).unwrap();
        wrong.bits = 64;
        assert!(dec.from_flipper_key(&wrong).is_none());
        assert!(!eu.encoder_capable, "decode-only, as the reference");
    }

    /// Reference-oracle vectors (real IMPORTS frames: Ford V3 has no encoder):
    /// `KAT_KEY_VECTORS=<dir> cargo test oracle_vectors -- --ignored`.
    #[test]
    #[ignore]
    fn oracle_vectors() {
        let dec = FordV3Decoder::new();
        for (n, rel) in ["LDV/LDV-T80_lock.sub", "FORD/22_ford_escape_u2_l2_t2_p2.sub"].iter().enumerate() {
            let path = Path::new("IMPORTS").join(rel);
            let Ok((freq, raw_pairs)) = import_sub_raw(&path) else { continue };
            let pairs: Vec<LevelDuration> =
                raw_pairs.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
            let mut reg = ProtocolRegistry::new();
            let Some((_, d, seg)) = reg.process_signal_stream(&pairs, freq).into_iter().find(|(n, _, _)| n == "Ford V3")
            else {
                continue;
            };
            let mut key = dec.to_flipper_key(&d).unwrap();
            key.frequency = freq;
            key.preset = crate::export::flipper::import_sub(&path).unwrap().preset.map(|p| p.name);
            super::super::ford_v0::write_oracle_vector(&format!("ford_v3_{}", n + 1), &seg, &key);
        }
    }
}
