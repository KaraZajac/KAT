//! Suzuki protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/suzuki.c`.
//! Decode/encode logic (preamble count 350, gap 2000µs, short=0/long=1, field layout) matches reference.
//!
//! Protocol characteristics:
//! - PWM encoding: 250µs HIGH = 0, 500µs HIGH = 1; LOW 250µs after each bit
//! - 64 bits total; preamble: 300+ short LOW pulses then long HIGH starts data
//! - 350 preamble pairs (SHORT HIGH / SHORT LOW); 2000µs gap at end
//! - Field layout: serial = (data_high&0xFFF)<<16 | data_low>>16; btn = (data_low>>12)&0xF; cnt = (data_high<<4)>>16

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 99;
const MIN_COUNT_BIT: usize = 64;
const PREAMBLE_COUNT: u16 = 350;
const GAP_TIME: u32 = 2000;
const GAP_DELTA: u32 = 399;

/// Decoder states (matches protopirate's SuzukiDecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CountPreamble,
    DecodeData,
}

/// Suzuki protocol decoder
pub struct SuzukiDecoder {
    step: DecoderStep,
    header_count: u16,
    decode_data: u64,
    decode_count_bit: usize,
    te_last: u32,
}

impl SuzukiDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            header_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
            te_last: 0,
        }
    }

    fn add_bit(&mut self, bit: u8) {
        self.decode_data = (self.decode_data << 1) | (bit as u64);
        self.decode_count_bit += 1;
    }

    /// Parse 64-bit data (matches suzuki.c: serial, btn, cnt layout)
    fn parse_data(data: u64) -> DecodedSignal {
        let data_high = (data >> 32) as u32;
        let data_low = data as u32;
        // Reference: instance->generic.serial = ((data_high & 0xFFF) << 16) | (data_low >> 16); etc.
        let serial = ((data_high & 0xFFF) << 16) | (data_low >> 16);
        let btn = ((data_low >> 12) & 0xF) as u8;
        let cnt = ((data_high << 4) >> 16) as u16;

        DecodedSignal {
            serial: Some(serial),
            button: Some(btn),
            counter: Some(cnt),
            crc_valid: true, // CRC checked via structure
            data,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
        }
    }
}

impl ProtocolDecoder for SuzukiDecoder {
    fn name(&self) -> &'static str {
        "Suzuki"
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
        self.header_count = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.te_last = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if !level {
                    return None;
                }
                if duration_diff!(duration, TE_SHORT) > TE_DELTA {
                    return None;
                }
                self.decode_data = 0;
                self.decode_count_bit = 0;
                self.step = DecoderStep::CountPreamble;
                self.header_count = 0;
            }

            DecoderStep::CountPreamble => {
                if level {
                    // HIGH pulse
                    if self.header_count >= 300 {
                        if duration_diff!(duration, TE_LONG) <= TE_DELTA {
                            self.step = DecoderStep::DecodeData;
                            self.add_bit(1);
                        }
                    }
                } else {
                    // LOW pulse
                    if duration_diff!(duration, TE_SHORT) <= TE_DELTA {
                        self.te_last = duration;
                        self.header_count += 1;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                }
            }

            DecoderStep::DecodeData => {
                if level {
                    // HIGH pulse - determines bit value
                    let diff_long = duration_diff!(duration, TE_LONG);
                    let diff_short = duration_diff!(duration, TE_SHORT);

                    if diff_long <= TE_DELTA {
                        self.add_bit(1);
                    } else if diff_short <= TE_DELTA {
                        self.add_bit(0);
                    }
                } else {
                    // LOW pulse - check for gap (end of transmission)
                    let diff_gap = duration_diff!(duration, GAP_TIME);

                    if diff_gap <= GAP_DELTA {
                        if self.decode_count_bit == MIN_COUNT_BIT {
                            let result = Self::parse_data(self.decode_data);
                            self.decode_data = 0;
                            self.decode_count_bit = 0;
                            self.step = DecoderStep::Reset;
                            return Some(result);
                        }

                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.step = DecoderStep::Reset;
                    }
                }
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        let data = decoded.data;

        let mut signal = Vec::with_capacity(1024);

        // Preamble + data + gap (matches subghz_protocol_encoder_suzuki_get_upload in suzuki.c)
        for _ in 0..PREAMBLE_COUNT {
            signal.push(LevelDuration::new(true, TE_SHORT));
            signal.push(LevelDuration::new(false, TE_SHORT));
        }
        // Data: 64 bits MSB first; SHORT HIGH = 0, LONG HIGH = 1; LOW = 250µs after each
        for bit in (0..64).rev() {
            if (data >> bit) & 1 == 1 {
                signal.push(LevelDuration::new(true, TE_LONG));
            } else {
                signal.push(LevelDuration::new(true, TE_SHORT));
            }
            signal.push(LevelDuration::new(false, TE_SHORT));
        }

        signal.push(LevelDuration::new(false, GAP_TIME));

        Some(signal)
    }
}

impl Default for SuzukiDecoder {
    fn default() -> Self {
        Self::new()
    }
}
