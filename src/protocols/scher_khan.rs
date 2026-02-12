//! Scher-Khan protocol decoder (decode-only)
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/scher_khan.c`.
//! Decode logic (PWM preamble/sync, short=0/long=1, variable bit count) matches reference.
//! No encoder in protopirate.
//!
//! Protocol characteristics:
//! - PWM encoding: 750µs = 0, 1100µs = 1; preamble uses 2× short then alternating
//! - Variable bit count (35, 51, 57, 63, 64, 81, 82); only 51-bit format parsed for serial/button/counter
//!
//! References: https://phreakerclub.com/72

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 750;
const TE_LONG: u32 = 1100;
const TE_DELTA: u32 = 160;
const MIN_COUNT_BIT: usize = 35;

/// Decoder states (matches protopirate's ScherKhanDecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckPreamble,
    SaveDuration,
    CheckDuration,
}

/// Scher-Khan protocol decoder
pub struct ScherKhanDecoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    decode_data: u64,
    decode_count_bit: usize,
}

impl ScherKhanDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    /// Parse payload by bit count; 51-bit format yields serial/button/counter (matches scher_khan.c)
    fn parse_data(data: u64, bit_count: usize) -> DecodedSignal {
        let (serial, btn, cnt) = match bit_count {
            51 => {
                // 51-bit "MAGIC CODE" / Dynamic format: serial(28) | button(4) | counter(16) — matches reference
                let serial =
                    ((data >> 24) & 0xFFFFFF0) as u32 | ((data >> 20) & 0x0F) as u32;
                let btn = ((data >> 24) & 0x0F) as u8;
                let cnt = (data & 0xFFFF) as u16;
                (Some(serial), Some(btn), Some(cnt))
            }
            _ => (None, None, None),
        };

        DecodedSignal {
            serial,
            button: btn,
            counter: cnt,
            crc_valid: true,
            data,
            data_count_bit: bit_count,
            encoder_capable: false,
        }
    }
}

impl ProtocolDecoder for ScherKhanDecoder {
    fn name(&self) -> &'static str {
        "Scher-Khan"
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
        self.te_last = 0;
        self.header_count = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && duration_diff!(duration, TE_SHORT * 2) < TE_DELTA {
                    self.step = DecoderStep::CheckPreamble;
                    self.te_last = duration;
                    self.header_count = 0;
                }
            }

            DecoderStep::CheckPreamble => {
                if level {
                    if duration_diff!(duration, TE_SHORT * 2) < TE_DELTA
                        || duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else if duration_diff!(duration, TE_SHORT * 2) < TE_DELTA
                    || duration_diff!(duration, TE_SHORT) < TE_DELTA
                {
                    if duration_diff!(self.te_last, TE_SHORT * 2) < TE_DELTA {
                        self.header_count += 1;
                    } else if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA {
                        // Found start bit
                        if self.header_count >= 2 {
                            self.step = DecoderStep::SaveDuration;
                            self.decode_data = 0;
                            self.decode_count_bit = 1;
                        } else {
                            self.step = DecoderStep::Reset;
                        }
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::SaveDuration => {
                if level {
                    if duration >= (TE_DELTA * 2 + TE_LONG) {
                        // Found stop bit
                        self.step = DecoderStep::Reset;
                        if self.decode_count_bit >= MIN_COUNT_BIT {
                            let result =
                                Self::parse_data(self.decode_data, self.decode_count_bit);
                            self.decode_data = 0;
                            self.decode_count_bit = 0;
                            return Some(result);
                        }
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                    } else {
                        self.te_last = duration;
                        self.step = DecoderStep::CheckDuration;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::CheckDuration => {
                if !level {
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        // Bit 0
                        self.decode_data = (self.decode_data << 1) | 0;
                        self.decode_count_bit += 1;
                        self.step = DecoderStep::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA
                    {
                        // Bit 1
                        self.decode_data = (self.decode_data << 1) | 1;
                        self.decode_count_bit += 1;
                        self.step = DecoderStep::SaveDuration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None // Scher-Khan decode-only in protopirate
    }
}

impl Default for ScherKhanDecoder {
    fn default() -> Self {
        Self::new()
    }
}
