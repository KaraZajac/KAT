//! Fiat V1 protocol decoder (Magneti Marelli BSI / PCF7946)
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/fiat_v1.c` and
//! `fiat_v1.h`. Found on Fiat Panda, Grande Punto, and possibly other Fiat/Lancia/Alfa ~2003-2012.
//!
//! RF: 433.92 MHz, Manchester encoding.
//! Two timing variants with identical frame structure:
//!   Type A (e.g. Panda):        te_short ~260us, te_long ~520us
//!   Type B (e.g. Grande Punto): te_short ~100us, te_long ~200us
//! TE is auto-detected from preamble pulse averaging (boundary at 180us).
//!
//! Frame layout (103-104 bits = 13 bytes):
//!   Bytes 0-1:  0xFFFF/0xFFFC preamble residue
//!   Bytes 2-5:  Serial (32 bits)
//!   Byte 6:     [Button:4 | Epoch:4]
//!   Byte 7:     [Counter:5 | Scramble:2 | Fixed:1]
//!   Bytes 8-12: Encrypted payload (40 bits)
//!
//! State machine: Reset -> Preamble -> Sync -> Data (-> RetxSync -> Data)
//! No encoder (decode-only).
//!
//! Original C implementation by @lupettohf

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

// Default timing constants (Type A)
const TE_SHORT: u32 = 260;
#[allow(dead_code)]
const TE_LONG: u32 = 520;
#[allow(dead_code)]
const TE_DELTA: u32 = 80;
#[allow(dead_code)]
const MIN_COUNT_BIT: usize = 80;

// Preamble / sync / data constants (from C defines)
const PREAMBLE_PULSE_MIN: u32 = 50;
const PREAMBLE_PULSE_MAX: u32 = 350;
const PREAMBLE_MIN: u16 = 80;
const MAX_DATA_BITS: u8 = 104;
const MIN_DATA_BITS: u8 = 80;
const GAP_TE_MULT: u32 = 4;
const SYNC_TE_MIN_MULT: u32 = 4;
const SYNC_TE_MAX_MULT: u32 = 12;
const RETX_GAP_MIN: u32 = 5000;
const RETX_SYNC_MIN: u32 = 400;
const RETX_SYNC_MAX: u32 = 2800;
#[allow(dead_code)]
const TE_TYPE_AB_BOUNDARY: u32 = 180;

/// Manchester state machine (Flipper-style, same as fiat_v0.rs / common.rs).
#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Mid0 = 0,
    Mid1 = 1,
    Start0 = 2,
    Start1 = 3,
}

/// Decoder state machine steps.
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    Sync,
    Data,
    RetxSync,
}

pub struct FiatV1Decoder {
    step: DecoderStep,
    manchester_state: ManchesterState,

    // Preamble tracking
    preamble_count: u16,

    // Auto-detected TE from preamble averaging
    te_sum: u32,
    te_count: u16,
    te_detected: u32,

    // Data accumulation
    raw_data: [u8; 13],
    bit_count: u8,
    data: u64,       // first 64 bits
    extra_data: u32, // bits 65+

    // Parsed fields
    serial: u32,
    btn: u8,
    cnt: u8,

    te_last: u32,
}

impl FiatV1Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: ManchesterState::Mid1,
            preamble_count: 0,
            te_sum: 0,
            te_count: 0,
            te_detected: 0,
            raw_data: [0u8; 13],
            bit_count: 0,
            data: 0,
            extra_data: 0,
            serial: 0,
            btn: 0,
            cnt: 0,
            te_last: 0,
        }
    }

    /// Advance the Manchester state machine by one event.
    /// Event encoding: 0=ShortLow, 1=ShortHigh, 2=LongLow, 3=LongHigh.
    /// Returns `Some(bit)` when a data bit is emitted, `None` otherwise.
    fn manchester_advance(&mut self, event: u8) -> Option<bool> {
        let (new_state, emit) = match (self.manchester_state, event) {
            (ManchesterState::Mid0, 0) => (ManchesterState::Mid0, false),
            (ManchesterState::Mid0, 1) => (ManchesterState::Start1, true),
            (ManchesterState::Mid0, 2) => (ManchesterState::Mid0, false),
            (ManchesterState::Mid0, 3) => (ManchesterState::Mid1, true),

            (ManchesterState::Mid1, 0) => (ManchesterState::Start0, true),
            (ManchesterState::Mid1, 1) => (ManchesterState::Mid1, false),
            (ManchesterState::Mid1, 2) => (ManchesterState::Mid0, true),
            (ManchesterState::Mid1, 3) => (ManchesterState::Mid1, false),

            (ManchesterState::Start0, 0) => (ManchesterState::Mid0, false),
            (ManchesterState::Start0, 1) => (ManchesterState::Mid0, false),
            (ManchesterState::Start0, 2) => (ManchesterState::Mid0, false),
            (ManchesterState::Start0, 3) => (ManchesterState::Mid1, false),

            (ManchesterState::Start1, 0) => (ManchesterState::Mid0, false),
            (ManchesterState::Start1, 1) => (ManchesterState::Mid1, false),
            (ManchesterState::Start1, 2) => (ManchesterState::Mid0, false),
            (ManchesterState::Start1, 3) => (ManchesterState::Mid1, false),

            _ => (ManchesterState::Mid1, false),
        };

        self.manchester_state = new_state;
        if emit {
            Some((event & 1) == 1)
        } else {
            None
        }
    }

    /// Reset Manchester state machine (to Mid1, matching C reset).
    fn manchester_reset(&mut self) {
        self.manchester_state = ManchesterState::Mid1;
    }

    /// Prepare for data reception: clear accumulators, reset Manchester, enter Data step.
    /// Matches `fiat_marelli_prepare_data` in the C reference.
    fn prepare_data(&mut self) {
        self.bit_count = 0;
        self.extra_data = 0;
        self.data = 0;
        self.raw_data = [0u8; 13];
        self.manchester_reset();
        self.step = DecoderStep::Data;
    }

    /// Get the effective te_short (auto-detected or default).
    fn te_short(&self) -> u32 {
        if self.te_detected != 0 {
            self.te_detected
        } else {
            TE_SHORT
        }
    }

    /// Get the effective te_long (2 * te_short).
    fn te_long(&self) -> u32 {
        self.te_short() * 2
    }

    /// Get the effective te_delta (te_short / 2, minimum 30).
    fn te_delta(&self) -> u32 {
        let d = self.te_short() / 2;
        if d < 30 { 30 } else { d }
    }

    /// Map button code to name (matches C fiat_marelli_button_name).
    #[allow(dead_code)]
    fn button_name(btn: u8) -> &'static str {
        match btn {
            0x7 => "Lock",
            0xB => "Unlock",
            0xD => "Trunk",
            _ => "Unknown",
        }
    }

    /// Build a DecodedSignal from the current state.
    fn parse_data(&self) -> DecodedSignal {
        // Pack extra bytes (bits 65+) into extra field
        let extra = if self.bit_count > 64 {
            Some(self.extra_data as u64)
        } else {
            None
        };

        DecodedSignal {
            serial: Some(self.serial),
            button: Some(self.btn),
            counter: Some(self.cnt as u16),
            crc_valid: true, // no CRC defined for this protocol
            data: self.data,
            data_count_bit: self.bit_count as usize,
            encoder_capable: false,
            extra,
            protocol_display_name: None,
        }
    }
}

impl ProtocolDecoder for FiatV1Decoder {
    fn name(&self) -> &'static str {
        "Fiat V1"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: MIN_COUNT_BIT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.preamble_count = 0;
        self.bit_count = 0;
        self.extra_data = 0;
        self.te_last = 0;
        self.te_sum = 0;
        self.te_count = 0;
        self.te_detected = 0;
        self.data = 0;
        self.raw_data = [0u8; 13];
        self.serial = 0;
        self.btn = 0;
        self.cnt = 0;
        self.manchester_state = ManchesterState::Mid1;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let te_short = self.te_short();
        let te_long = self.te_long();
        let te_delta = self.te_delta();

        match self.step {
            // =========================================================
            // Reset: wait for a HIGH pulse in preamble range, or detect
            // retransmission gap (LOW > 5000us with valid te_detected).
            // =========================================================
            DecoderStep::Reset => {
                if level {
                    if duration >= PREAMBLE_PULSE_MIN && duration <= PREAMBLE_PULSE_MAX {
                        self.step = DecoderStep::Preamble;
                        self.preamble_count = 1;
                        self.te_sum = duration;
                        self.te_count = 1;
                        self.te_last = duration;
                    }
                } else if duration > RETX_GAP_MIN && self.te_detected != 0 {
                    self.step = DecoderStep::RetxSync;
                    self.te_last = duration;
                }
            }

            // =========================================================
            // Preamble: accumulate pulses in 50-350us range for TE
            // averaging. When count >= 80 and we see a LOW gap >= 4*TE,
            // transition to Sync.
            // =========================================================
            DecoderStep::Preamble => {
                if duration >= PREAMBLE_PULSE_MIN && duration <= PREAMBLE_PULSE_MAX {
                    // Pulse in valid preamble range (either HIGH or LOW)
                    self.preamble_count += 1;
                    self.te_sum += duration;
                    self.te_count += 1;
                    self.te_last = duration;
                } else if !level {
                    // LOW pulse outside preamble range -- check if it's the gap after preamble
                    if self.preamble_count >= PREAMBLE_MIN && self.te_count > 0 {
                        self.te_detected = self.te_sum / (self.te_count as u32);
                        let gap_threshold = self.te_detected * GAP_TE_MULT;

                        if duration > gap_threshold {
                            self.step = DecoderStep::Sync;
                            self.te_last = duration;
                        } else {
                            self.step = DecoderStep::Reset;
                        }
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    // HIGH pulse outside preamble range
                    self.step = DecoderStep::Reset;
                }
            }

            // =========================================================
            // Sync: expect HIGH sync pulse of te_detected*4 to
            // te_detected*12 duration. On match, prepare data decoder.
            // =========================================================
            DecoderStep::Sync => {
                let sync_min = self.te_detected * SYNC_TE_MIN_MULT;
                let sync_max = self.te_detected * SYNC_TE_MAX_MULT;

                if level && duration >= sync_min && duration <= sync_max {
                    self.prepare_data();
                    self.te_last = duration;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            // =========================================================
            // RetxSync: after retransmission gap (>5000us LOW), look for
            // HIGH sync pulse in 400-2800us range.
            // =========================================================
            DecoderStep::RetxSync => {
                if level && duration >= RETX_SYNC_MIN && duration <= RETX_SYNC_MAX {
                    self.prepare_data();
                    self.te_last = duration;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            // =========================================================
            // Data: Manchester decode up to 104 bits. Complete frame on
            // 104 bits reached or gap with >= 80 bits collected.
            // =========================================================
            DecoderStep::Data => {
                let mut event: u8 = 4; // 4 = ManchesterEventReset (invalid)
                let mut frame_complete = false;

                // Check for short pulse match
                let diff_short = duration_diff!(duration, te_short);
                if diff_short < te_delta {
                    // Flipper convention: level=HIGH -> ShortLow(0), level=LOW -> ShortHigh(1)
                    event = if level { 0 } else { 1 };
                } else {
                    // Check for long pulse match
                    let diff_long = duration_diff!(duration, te_long);
                    if diff_long < te_delta {
                        event = if level { 2 } else { 3 };
                    }
                }

                if event != 4 {
                    // Valid Manchester event
                    if let Some(data_bit) = self.manchester_advance(event) {
                        let new_bit: u32 = if data_bit { 1 } else { 0 };

                        // Store bit into raw_data byte array (MSB-first)
                        if self.bit_count < MAX_DATA_BITS {
                            let byte_idx = (self.bit_count / 8) as usize;
                            let bit_pos = 7 - (self.bit_count % 8);
                            if new_bit != 0 {
                                self.raw_data[byte_idx] |= 1u8 << bit_pos;
                            }
                        }

                        // Also accumulate into u64 data / u32 extra_data
                        if self.bit_count < 64 {
                            self.data = (self.data << 1) | (new_bit as u64);
                        } else {
                            self.extra_data = (self.extra_data << 1) | new_bit;
                        }

                        self.bit_count += 1;
                        if self.bit_count >= MAX_DATA_BITS {
                            frame_complete = true;
                        }
                    }
                } else if self.bit_count >= MIN_DATA_BITS {
                    // Gap or invalid pulse but we have enough bits
                    frame_complete = true;
                } else {
                    // Not enough bits and invalid pulse -- abort
                    self.step = DecoderStep::Reset;
                }

                if frame_complete {
                    // Parse serial, button, counter from raw_data
                    self.serial = ((self.raw_data[2] as u32) << 24)
                        | ((self.raw_data[3] as u32) << 16)
                        | ((self.raw_data[4] as u32) << 8)
                        | (self.raw_data[5] as u32);
                    self.btn = (self.raw_data[6] >> 4) & 0x0F;
                    self.cnt = (self.raw_data[7] >> 3) & 0x1F;

                    let result = self.parse_data();
                    self.step = DecoderStep::Reset;
                    self.te_last = duration;
                    return Some(result);
                }

                self.te_last = duration;
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

impl Default for FiatV1Decoder {
    fn default() -> Self {
        Self::new()
    }
}
