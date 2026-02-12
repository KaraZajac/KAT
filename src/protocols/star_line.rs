//! Star Line protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/star_line.c`.
//! Decode/encode logic (PWM header, fix/hop split, KeeLoq simple/normal learning) matches reference.
//!
//! Protocol characteristics:
//! - PWM encoding: 250µs = 0, 500µs = 1
//! - 64 bits total: key_fix (32) + key_hop (32), sent MSB-first (reversed on air)
//! - Header: 6 pairs of 1000µs HIGH + 1000µs LOW
//! - KeeLoq: fix = serial(24) + button(8); hop encrypted with MF key or normal-learning derived key

use super::keeloq_common::{keeloq_decrypt, keeloq_encrypt, keeloq_normal_learning, reverse_key};
use super::keys;
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 120;
const MIN_COUNT_BIT: usize = 64;
const HEADER_DURATION: u32 = 1000; // te_long * 2

/// Decoder states (matches protopirate's StarLineDecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckPreamble,
    SaveDuration,
    CheckDuration,
}

/// Star Line protocol decoder
pub struct StarLineDecoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    decode_data: u64,
    decode_count_bit: usize,
}

impl StarLineDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    /// Star Line manufacturer key from keystore (type 20)
    fn get_mf_key() -> u64 {
        keys::get_keystore().get_star_line_mf_key()
    }

    /// Parse 64-bit payload: reverse_key then fix(32)/hop(32); KeeLoq decrypt (simple then normal learning) — matches star_line.c
    fn parse_data(data: u64) -> DecodedSignal {
        // Data is MSB-first on air; reverse to get fix|hop
        let reversed = reverse_key(data, MIN_COUNT_BIT);
        let key_fix = (reversed >> 32) as u32;
        let key_hop = (reversed & 0xFFFFFFFF) as u32;

        let serial = key_fix & 0x00FFFFFF;
        let btn = (key_fix >> 24) as u8;

        // Attempt KeeLoq decryption
        let mf_key = Self::get_mf_key();
        let counter = if mf_key != 0 {
            // Try simple learning first
            let decrypt = keeloq_decrypt(key_hop, mf_key);
            let dec_btn = (decrypt >> 24) as u8;
            let dec_serial_lsb = ((decrypt >> 16) & 0xFF) as u8;
            let serial_lsb = (serial & 0xFF) as u8;

            if dec_btn == btn && dec_serial_lsb == serial_lsb {
                Some((decrypt & 0xFFFF) as u16)
            } else {
                // Try normal learning
                let man_key = keeloq_normal_learning(key_fix, mf_key);
                let decrypt = keeloq_decrypt(key_hop, man_key);
                let dec_btn = (decrypt >> 24) as u8;
                let dec_serial_lsb = ((decrypt >> 16) & 0xFF) as u8;

                if dec_btn == btn && dec_serial_lsb == serial_lsb {
                    Some((decrypt & 0xFFFF) as u16)
                } else {
                    None
                }
            }
        } else {
            None
        };

        DecodedSignal {
            serial: Some(serial),
            button: Some(btn),
            counter: counter.or(Some(0)),
            crc_valid: counter.is_some() || mf_key == 0,
            data,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
        }
    }
}

impl ProtocolDecoder for StarLineDecoder {
    fn name(&self) -> &'static str {
        "Star Line"
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
                if level {
                    if duration_diff!(duration, HEADER_DURATION) < TE_DELTA * 2 {
                        self.step = DecoderStep::CheckPreamble;
                        self.header_count += 1;
                    } else if self.header_count > 4 {
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.te_last = duration;
                        self.step = DecoderStep::CheckDuration;
                    }
                } else {
                    self.header_count = 0;
                }
            }

            DecoderStep::CheckPreamble => {
                if !level && duration_diff!(duration, HEADER_DURATION) < TE_DELTA * 2 {
                    // Found preamble pair
                    self.step = DecoderStep::Reset;
                } else {
                    self.header_count = 0;
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::SaveDuration => {
                if level {
                    if duration >= (TE_LONG + TE_DELTA) {
                        // End of data - check if we have enough bits
                        self.step = DecoderStep::Reset;
                        if self.decode_count_bit >= MIN_COUNT_BIT
                            && self.decode_count_bit <= MIN_COUNT_BIT + 2
                        {
                            let result = Self::parse_data(self.decode_data);
                            self.decode_data = 0;
                            self.decode_count_bit = 0;
                            self.header_count = 0;
                            return Some(result);
                        }
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.header_count = 0;
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
                        // Bit 0: short HIGH + short LOW
                        if self.decode_count_bit < MIN_COUNT_BIT {
                            self.decode_data = (self.decode_data << 1) | 0;
                            self.decode_count_bit += 1;
                        } else {
                            self.decode_count_bit += 1;
                        }
                        self.step = DecoderStep::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA
                    {
                        // Bit 1: long HIGH + long LOW
                        if self.decode_count_bit < MIN_COUNT_BIT {
                            self.decode_data = (self.decode_data << 1) | 1;
                            self.decode_count_bit += 1;
                        } else {
                            self.decode_count_bit += 1;
                        }
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
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let serial = decoded.serial?;
        let counter = decoded.counter.unwrap_or(0).wrapping_add(1);

        let fix = ((button as u32) << 24) | (serial & 0x00FFFFFF);
        let plaintext = ((button as u32) << 24)
            | (((serial & 0xFF) as u32) << 16)
            | (counter as u32);

        let mf_key = Self::get_mf_key();
        let hop = if mf_key != 0 {
            keeloq_encrypt(plaintext, mf_key)
        } else {
            // Without a key, replay the original hop
            let reversed = reverse_key(decoded.data, MIN_COUNT_BIT);
            (reversed & 0xFFFFFFFF) as u32
        };

        let yek = ((fix as u64) << 32) | (hop as u64);
        let data = reverse_key(yek, MIN_COUNT_BIT);

        let mut signal = Vec::with_capacity(256);

        // Header: 6 pairs 1000µs HIGH + 1000µs LOW (matches protopirate star_line encode)
        for _ in 0..6 {
            signal.push(LevelDuration::new(true, HEADER_DURATION));
            signal.push(LevelDuration::new(false, HEADER_DURATION));
        }

        // Data: 64 bits PWM, MSB first (1=long, 0=short)
        for bit in (0..64).rev() {
            if (data >> bit) & 1 == 1 {
                // Bit 1: LONG HIGH + LONG LOW
                signal.push(LevelDuration::new(true, TE_LONG));
                signal.push(LevelDuration::new(false, TE_LONG));
            } else {
                // Bit 0: SHORT HIGH + SHORT LOW
                signal.push(LevelDuration::new(true, TE_SHORT));
                signal.push(LevelDuration::new(false, TE_SHORT));
            }
        }

        Some(signal)
    }
}

impl Default for StarLineDecoder {
    fn default() -> Self {
        Self::new()
    }
}
