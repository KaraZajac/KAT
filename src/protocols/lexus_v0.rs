//! Lexus V0 decoder (FoxFW, decode-only).
//!
//! Ported from FoxFW `lib/subghz/protocols/lexus_v0.c` (`SUBGHZ_PROTOCOL_LEXUS_V0_NAME = "Lexus V0"`;
//! the header credits the firmware `lexus_v0.c` via KeeLoq-RX `arke_toyota.py`). No other reference
//! has it. It is a different air format from KAT's "Toyota" (Flipper-ARF / doctor-firmware
//! `toyota.c`: ~793/1587 µs bi-phase, 96-bit payload repeated 10 times), so the two read different
//! frames: this one is ~350/700 µs, 224 bits. AM, 315 / 433.92 MHz.
//!
//! **Line code** (`feed`, custom bands, not `DURATION_DIFF`): preamble pulse 250..=470 µs; LONG
//! 500..=980 µs; SHORT anything under 500 µs; GAP >= 980 µs. At least 16 preamble pulses, then the
//! first LONG starts the data and is itself the first bit (its level). In data, a LONG is one bit
//! equal to its level (a LONG arriving between two SHORTs, `half` set, is a framing error that ends
//! the frame); two SHORTs are one bit equal to the second one's level. A GAP ends the frame, so
//! does the 224th bit; the frame is processed when it holds all 224 bits.
//!
//! **Frame** (224 bits): `[0..31]` header (discarded) | `[32..191]` 160-bit encrypted rolling code
//! (20 bytes) | `[192..223]` 32-bit serial. There is no CRC. The only validator is the Toyota
//! complement signature: the serial alternates between S and ~S on successive frames, so a frame is
//! "complement verified" once its ~S partner has been seen. A serial of all zeros or all ones is
//! dropped as noise.
//!
//! **DecodedSignal**: `data` = the top 64 bits of the rolling code (the first 8 of the 20 payload
//! bytes: the C's `generic.data`), `data_count_bit` = 224, `serial` = the 32-bit serial. The C sets
//! button and counter to 0 ("encrypted, no cleartext"); KAT leaves them `None`. The other 96 bits of
//! the rolling code (the C writes them in a `Rolling` hex line) are not kept. `extra` = 1 when the
//! complement signature verified this frame, else 0, and `crc_valid` mirrors it, so a verified frame
//! shows as checked and an unverified one as "-" (`info::PARTIAL_CHECK`).
//!
//! `prev_serial` / `have_prev` persist across `reset()`, as in the C (cleared only on allocation):
//! the signature is a cross-frame check. KAT's registry resets every decoder after a hit, which the
//! C does not do between frames of one capture, hence this memory survives it.

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Lexus V0",
    decoder: "Lexus V0",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Lexus",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const PRE_LO: u32 = 250;
const PRE_HI: u32 = 470;
const LONG_LO: u32 = 500;
const LONG_HI: u32 = 980;
const GAP: u32 = 980;
const MIN_PRE: u16 = 16;
const HEADER_BITS: usize = 32;
const DATA_BITS: usize = 160;
const SERIAL_BITS: usize = 32;
const FRAME_BITS: usize = 224;

#[derive(Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Preamble,
    Data,
}

pub struct LexusV0Decoder {
    step: Step,
    preamble_count: u16,
    half_bit: bool,
    bit_count: usize,
    bits: [u8; FRAME_BITS],
    prev_serial: u32,
    have_prev: bool,
}

impl LexusV0Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            preamble_count: 0,
            half_bit: false,
            bit_count: 0,
            bits: [0; FRAME_BITS],
            prev_serial: 0,
            have_prev: false,
        }
    }

    /// `lexus_v0_restart`.
    fn restart(&mut self) {
        self.step = Step::Reset;
        self.preamble_count = 0;
        self.half_bit = false;
        self.bit_count = 0;
    }

    /// `lexus_v0_process_frame`.
    fn process_frame(&mut self) -> Option<DecodedSignal> {
        if self.bit_count < FRAME_BITS {
            return None;
        }
        let base = HEADER_BITS + DATA_BITS;
        let mut serial = 0u32;
        for i in 0..SERIAL_BITS {
            serial = (serial << 1) | u32::from(self.bits[base + i] & 1);
        }
        if serial == 0 || serial == 0xFFFF_FFFF {
            return None;
        }
        let mut data = 0u64;
        for i in 0..64 {
            data = (data << 1) | u64::from(self.bits[HEADER_BITS + i] & 1);
        }
        let compl_ok = self.have_prev && serial == !self.prev_serial;
        self.prev_serial = serial;
        self.have_prev = true;
        Some(DecodedSignal {
            serial: Some(serial),
            button: None,
            counter: None,
            crc_valid: compl_ok,
            data,
            data_count_bit: FRAME_BITS,
            encoder_capable: false,
            extra: Some(u64::from(compl_ok)),
            protocol_display_name: None,
        })
    }
}

impl Default for LexusV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for LexusV0Decoder {
    fn name(&self) -> &'static str {
        "Lexus V0"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 350, te_long: 700, te_delta: 150, min_count_bit: FRAME_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        // prev_serial / have_prev persist (see the module doc).
        self.restart();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let in_pre = (PRE_LO..=PRE_HI).contains(&duration);
        let is_long = (LONG_LO..=LONG_HI).contains(&duration);
        let is_short = duration < LONG_LO;

        match self.step {
            Step::Reset => {
                if in_pre {
                    self.preamble_count = 1;
                    self.step = Step::Preamble;
                }
            }
            Step::Preamble => {
                if in_pre {
                    self.preamble_count = self.preamble_count.wrapping_add(1);
                } else if is_long && self.preamble_count >= MIN_PRE {
                    self.step = Step::Data;
                    self.bit_count = 0;
                    self.half_bit = false;
                    self.bits[0] = u8::from(level);
                    self.bit_count = 1;
                } else {
                    self.restart();
                }
            }
            Step::Data => {
                if duration >= GAP {
                    let r = self.process_frame();
                    self.restart();
                    return r;
                }
                if is_long {
                    if self.half_bit {
                        let r = self.process_frame();
                        self.restart();
                        return r;
                    }
                    self.bits[self.bit_count] = u8::from(level);
                    self.bit_count += 1;
                } else if is_short {
                    if self.half_bit {
                        self.bits[self.bit_count] = u8::from(level);
                        self.bit_count += 1;
                        self.half_bit = false;
                    } else {
                        self.half_bit = true;
                    }
                }
                if self.bit_count >= FRAME_BITS {
                    let r = self.process_frame();
                    self.restart();
                    return r;
                }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::dd_testutil::{bits_of, dump_sub, push, toggle_bits, Pulse};

    const PRESET: &str = "FuriHalSubGhzPresetOok650Async";
    /// Made-up 160-bit rolling code (20 bytes) and serial.
    const ROLL: [u8; 20] = [
        0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x18, 0x29, 0x3A, 0x4B, 0x5C, 0x6D, 0x7E, 0x8F,
        0x90, 0x01, 0x12, 0x23, 0x34,
    ];
    const SERIAL: u32 = 0x1357_9BDF;

    /// 40 preamble pulses of 350 µs, then the 224 bits (header, rolling code, serial) as the
    /// half-bit-toggle line code; the first data bit is a LONG, so the line is forced to its
    /// opposite level first, and a 2 ms gap ends the frame.
    fn wire(header: u32, roll: &[u8; 20], serial: u32, bits_wanted: usize) -> Vec<Pulse> {
        let mut bits = bits_of(u64::from(header), 32);
        for b in roll {
            bits.extend(bits_of(u64::from(*b), 8));
        }
        bits.extend(bits_of(u64::from(serial), 32));
        bits.truncate(bits_wanted);
        let mut p: Vec<Pulse> = Vec::new();
        for i in 0..40 {
            push(&mut p, i % 2 == 0, 350);
        }
        // Make the first data bit a LONG: force the line level to be the opposite of bit 0.
        let first = bits[0] != 0;
        if p.last().unwrap().0 == first {
            // Same level as the first bit: stretch with one more preamble pulse.
            p.push((!first, 350));
        }
        toggle_bits(&mut p, &bits, 350, 700);
        let end = !p.last().unwrap().0;
        p.push((end, 2000));
        p
    }

    /// Frames back to back (equal levels at the joins merge).
    fn join(frames: &[Vec<Pulse>]) -> Vec<Pulse> {
        let mut acc = Vec::new();
        for (l, us) in frames.iter().flatten() {
            push(&mut acc, *l, *us);
        }
        acc
    }

    fn run(d: &mut LexusV0Decoder, p: &[Pulse]) -> Vec<DecodedSignal> {
        p.iter().filter_map(|&(l, us)| d.feed(l, us)).collect()
    }

    fn expected_data() -> u64 {
        ROLL[..8].iter().fold(0, |a, &b| (a << 8) | u64::from(b))
    }

    #[test]
    fn decodes_a_made_up_frame_and_verifies_the_complement_on_the_next() {
        let a = wire(0x8000_0001, &ROLL, SERIAL, 224);
        let b = wire(0x8000_0001, &ROLL, !SERIAL, 224);
        dump_sub("lexus_v0_ok", 433_920_000, PRESET, &join(&[a.clone(), b.clone()]));
        let mut d = LexusV0Decoder::new();
        let r = run(&mut d, &a);
        assert_eq!(r.len(), 1, "{r:?}");
        assert_eq!(r[0].serial, Some(SERIAL));
        assert_eq!(r[0].data, expected_data());
        assert_eq!(r[0].data_count_bit, 224);
        assert!(!r[0].crc_valid);
        // The registry resets after a hit; the complement memory survives it.
        d.reset();
        let r = run(&mut d, &b);
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].serial, Some(!SERIAL));
        assert!(r[0].crc_valid);
        assert_eq!(r[0].extra, Some(1));
    }

    #[test]
    fn the_registry_reports_both_frames_and_checks_the_second() {
        let p = join(&[wire(0x8000_0001, &ROLL, SERIAL, 224), wire(0x8000_0001, &ROLL, !SERIAL, 224)]);
        let pairs: Vec<LevelDuration> = p.iter().map(|&(l, us)| LevelDuration::new(l, us)).collect();
        let out = crate::protocols::ProtocolRegistry::new().process_signal_stream(&pairs, 433_920_000);
        let got: Vec<(&str, bool)> = out.iter().map(|(n, d, _)| (n.as_str(), d.crc_valid)).collect();
        assert_eq!(got, [("Lexus V0", false), ("Lexus V0", true)]);
    }

    #[test]
    fn rejects_degenerate_serials_short_frames_and_short_preambles() {
        for (name, serial) in [("zero", 0u32), ("ones", 0xFFFF_FFFF)] {
            let p = wire(0x8000_0001, &ROLL, serial, 224);
            dump_sub(&format!("lexus_v0_serial_{name}"), 433_920_000, PRESET, &p);
            assert!(run(&mut LexusV0Decoder::new(), &p).is_empty(), "serial {serial:#x}");
        }
        let p = wire(0x8000_0001, &ROLL, SERIAL, 223);
        dump_sub("lexus_v0_223", 433_920_000, PRESET, &p);
        assert!(run(&mut LexusV0Decoder::new(), &p).is_empty(), "223-bit frame");

        // 12 preamble pulses only (< 16).
        let mut p = wire(0x8000_0001, &ROLL, SERIAL, 224);
        p.drain(..28);
        dump_sub("lexus_v0_short_preamble", 433_920_000, PRESET, &p);
        assert!(run(&mut LexusV0Decoder::new(), &p).is_empty(), "short preamble");
    }
}
