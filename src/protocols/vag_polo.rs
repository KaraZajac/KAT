//! VAG Polo (VW Polo 5, 2017): 433 MHz OOK Manchester rolling-code remote — **decode and display
//! only**, as the reference.
//!
//! Ported from FoxFW `lib/subghz/protocols/vag_polo.c` (itself a port of the KeeLoq-RX
//! `arke_vag.py` class `VagPolo`); the protocol name is FoxFW's, "VAG Polo". The C is decode-only
//! (`.encoder = NULL`; flags 433 | 315 | AM | Decodable | Load | Save, type Dynamic) and has no
//! checksum: the gate is structural.
//!
//! **Air format**: OOK, Manchester with `te_short` 300 µs / `te_long` 590 µs. Every pulse must be
//! within 150 µs (`VAG_POLO_TE_DELTA` 130 + 20) of one of them, else the frame is dropped. The
//! frame is a run of at least 32 cleartext `1` preamble bits, the 16-bit sync `0x3F91`, then 80
//! payload bits: `Button(8) | Serial(32) | rolling(40)`. The button's high nibble is in the clear
//! (`0x7` Unlock, `0xB` Lock; the evidence captures send 0x7D or 0x7F for Unlock, 0xBD or 0xBF for Lock: bit 1 differs between presses); the rolling part is
//! encrypted, so there is no clone.
//!
//! **Decoder** (`subghz_protocol_decoder_vag_polo_feed`): in the reset step a HIGH pulse within
//! tolerance of `te_short` starts the preamble (that pulse is not an event itself; Manchester
//! state `Mid1`). In the preamble each pulse is a Manchester event for Flipper's toolbox decoder
//! with the C's "core" polarity (`level ? ShortLow : ShortHigh`); every bit slides into a 16-bit
//! sync register, a `1` counts towards the 32 preamble ones, and a `0` before 32 ones were seen
//! restarts the count. When the register reads `0x3F91` with at least 32 ones counted, the
//! payload is collected MSB first into 10 bytes. After 80 bits the frame is reported if the first
//! byte's high nibble is `0x7` or `0xB`; either way the decoder re-arms (reset step).
//!
//! **KAT fields**: `data` = payload bytes 2..9 (the C's `generic.data`, 64 bits),
//! `data_count_bit` = 80, `serial` = bytes 1..4, `button` = 1 for a high nibble of 7 (Unlock), 2
//! otherwise (Lock), `extra` = the C's `Hdr` (payload bytes 0..1; its high byte is `ButtonRaw`),
//! `counter` None, `crc_valid` false, `encoder_capable` false.
//!
//! **Key file** (`serialize` / `deserialize`): `Protocol: VAG Polo`, `Bit: 80`, `Key` = `data`,
//! then `Serial`, `Btn`, `ButtonRaw`, `Hdr`.
//!
//! **Deviations**: none. **Evidence**: the eight "VW Polo 5 2017 lock/unlock" captures in
//! IMPORTS (48 frames, three per press; fifteen for the x5 files); the button's high nibble is constant
//! per action (the byte alternates between two values from press to press) and the rest changes
//! every frame. Decodes match the host build of FoxFW's C frame
//! for frame (`Serial`, `Btn`, `ButtonRaw`, `Hdr`, `Key`).

use super::common::{ManchesterEvent, ManchesterState};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | 315 | AM`, `SubGhzProtocolTypeDynamic`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling code (encrypted)",
    make: "Volkswagen",
    category: Category::Vehicle,
    buttons: Some(button_name),
    presets: &[&AM650],
};

const NAME: &str = "VAG Polo";
const TE_SHORT: u32 = 300;
const TE_LONG: u32 = 590;
const EVENT_TOL: u32 = 150;
const SYNC: u16 = 0x3F91;
const PRE_ONES: u16 = 32;
const DATA_BITS: u16 = 80;
const BYTES: usize = 10;

fn button_name(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        1 => Some("Unlock"),
        2 => Some("Lock"),
        _ => None,
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    Preamble,
    Data,
}

/// VAG Polo decoder (`SubGhzProtocolDecoderVagPolo`).
pub struct VagPoloDecoder {
    step: Step,
    manchester: ManchesterState,
    sync_reg: u16,
    pre_ones: u16,
    payload: [u8; BYTES],
    pbits: u16,
}

/// `vag_polo_event`: the C's "core" polarity.
fn event(duration: u32, level: bool) -> ManchesterEvent {
    if duration.abs_diff(TE_SHORT) < EVENT_TOL {
        ManchesterEvent::from_half_bit(!level, false)
    } else if duration.abs_diff(TE_LONG) < EVENT_TOL {
        ManchesterEvent::from_half_bit(!level, true)
    } else {
        ManchesterEvent::Reset
    }
}

/// `vag_polo_commit`: the KAT view of a collected payload.
fn signal(p: &[u8; BYTES]) -> DecodedSignal {
    let serial = u32::from_be_bytes([p[1], p[2], p[3], p[4]]);
    let data = p[2..].iter().fold(0u64, |acc, &b| (acc << 8) | u64::from(b));
    let hdr16 = (u16::from(p[0]) << 8) | u16::from(p[1]);
    DecodedSignal {
        serial: Some(serial),
        button: Some(if (p[0] >> 4) & 0x0F == 0x7 { 1 } else { 2 }),
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: usize::from(DATA_BITS),
        encoder_capable: false,
        extra: Some(u64::from(hdr16)),
        protocol_display_name: None,
    }
}

impl VagPoloDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            manchester: ManchesterState::Mid1,
            sync_reg: 0,
            pre_ones: 0,
            payload: [0; BYTES],
            pbits: 0,
        }
    }

    /// `vag_polo_rearm`.
    fn rearm(&mut self) {
        self.step = Step::Reset;
        self.manchester = ManchesterState::Mid1;
        self.sync_reg = 0;
        self.pre_ones = 0;
        self.pbits = 0;
        self.payload = [0; BYTES];
    }
}

impl Default for VagPoloDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for VagPoloDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: 130,
            min_count_bit: usize::from(DATA_BITS),
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.rearm();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if level && duration.abs_diff(TE_SHORT) < EVENT_TOL {
                    self.step = Step::Preamble;
                    self.manchester = ManchesterState::Mid1;
                    self.sync_reg = 0;
                    self.pre_ones = 0;
                }
            }
            Step::Preamble => {
                let ev = event(duration, level);
                if ev == ManchesterEvent::Reset {
                    self.step = Step::Reset;
                    return None;
                }
                if let Some(bit) = self.manchester.advance(ev) {
                    self.sync_reg = (self.sync_reg << 1) | u16::from(bit);
                    if bit {
                        self.pre_ones = self.pre_ones.wrapping_add(1);
                    } else if self.pre_ones < PRE_ONES {
                        self.pre_ones = 0;
                        self.sync_reg = 0;
                        return None;
                    }
                    if self.sync_reg == SYNC && self.pre_ones >= PRE_ONES {
                        self.pbits = 0;
                        self.payload = [0; BYTES];
                        self.step = Step::Data;
                    }
                }
            }
            Step::Data => {
                let ev = event(duration, level);
                if ev == ManchesterEvent::Reset {
                    self.step = Step::Reset;
                    return None;
                }
                if let Some(bit) = self.manchester.advance(ev) {
                    let bi = usize::from(self.pbits >> 3);
                    if bi < BYTES {
                        self.payload[bi] = (self.payload[bi] << 1) | u8::from(bit);
                    }
                    self.pbits += 1;
                    if self.pbits >= DATA_BITS {
                        let hi = (self.payload[0] >> 4) & 0x0F;
                        let out = (hi == 0x7 || hi == 0xB).then(|| signal(&self.payload));
                        self.rearm();
                        return out;
                    }
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

    /// `subghz_protocol_decoder_vag_polo_deserialize`: `Bit: 80`; `Serial`, `Btn` and `Hdr` are
    /// read as the C does (a missing line reads 0).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != usize::from(DATA_BITS) {
            return None;
        }
        Some(DecodedSignal {
            serial: Some(key.u32_field("Serial").unwrap_or(0)),
            button: Some(key.u32_field("Btn").unwrap_or(0) as u8),
            counter: None,
            crc_valid: false,
            data: key.key,
            data_count_bit: usize::from(DATA_BITS),
            encoder_capable: false,
            extra: Some(u64::from(key.u32_field("Hdr").unwrap_or(0) as u16)),
            protocol_display_name: None,
        })
    }

    /// `subghz_protocol_decoder_vag_polo_serialize`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != usize::from(DATA_BITS) {
            return None;
        }
        let hdr = decoded.extra? as u32;
        let mut key = FlipperKey::for_protocol(NAME, usize::from(DATA_BITS), decoded.data);
        key.push_u32("Serial", decoded.serial?);
        key.push_u32("Btn", decoded.button?.into());
        key.push_u32("ButtonRaw", hdr >> 8);
        key.push_u32("Hdr", hdr);
        Some(key)
    }
}

/// Synthetic captures for the tests.
#[cfg(test)]
pub(super) mod testing {
    use super::*;

    /// Pulses of a frame: `ones` preamble ones, the sync, then the 10 payload bytes, in the C's
    /// polarity (a `1` is HIGH then LOW), ending with a long LOW.
    pub fn frame(ones: usize, payload: &[u8; BYTES]) -> Vec<LevelDuration> {
        let mut bits = vec![true; ones];
        bits.extend((0..16).rev().map(|i| (SYNC >> i) & 1 == 1));
        for b in payload {
            bits.extend((0..8).rev().map(|i| (b >> i) & 1 == 1));
        }
        let mut halves = Vec::new();
        for b in bits {
            halves.extend([b, !b]);
        }
        let mut out = Vec::new();
        let mut i = 0;
        while i < halves.len() {
            let level = halves[i];
            let run = if halves.get(i + 1) == Some(&level) { 2 } else { 1 };
            out.push(LevelDuration::new(level, if run == 2 { TE_LONG } else { TE_SHORT }));
            i += run;
        }
        out.push(LevelDuration::new(false, 5_000));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    const P: [u8; BYTES] = [0xBD, 0x48, 0x89, 0xD7, 0x8A, 0xC0, 0x0E, 0xB2, 0xC2, 0x31];

    #[test]
    fn decodes_a_synthetic_frame() {
        let got = feed_all(&mut VagPoloDecoder::new(), &testing::frame(40, &P));
        assert_eq!(got.len(), 1, "{got:?}");
        let d = &got[0];
        assert_eq!(d.serial, Some(0x4889_D78A));
        assert_eq!((d.button, d.counter, d.extra), (Some(2), None, Some(0xBD48)));
        assert_eq!(d.data, 0x89D7_8AC0_0EB2_C231);
        assert!(!d.encoder_capable && !d.crc_valid);
        let mut unlock = P;
        unlock[0] = 0x7D;
        let got = feed_all(&mut VagPoloDecoder::new(), &testing::frame(40, &unlock));
        assert_eq!(got[0].button, Some(1));
    }

    #[test]
    fn gates_match_the_c() {
        // fewer than 32 preamble ones: no sync lock
        assert!(feed_all(&mut VagPoloDecoder::new(), &testing::frame(20, &P)).is_empty());
        // button high nibble outside {7, B}
        let mut bad = P;
        bad[0] = 0x5D;
        assert!(feed_all(&mut VagPoloDecoder::new(), &testing::frame(40, &bad)).is_empty());
        // an out-of-tolerance pulse drops the frame
        let mut wire = testing::frame(40, &P);
        let mid = wire.len() / 2;
        wire[mid].duration_us = 1_000;
        assert!(feed_all(&mut VagPoloDecoder::new(), &wire).is_empty());
        assert!(VagPoloDecoder::new().encode(&DecodedSignal::new(0, 80), 1).is_none());
    }

    #[test]
    fn key_file_round_trip() {
        let dec = VagPoloDecoder::new();
        let d = feed_all(&mut VagPoloDecoder::new(), &testing::frame(40, &P)).remove(0);
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(text.contains("Protocol: VAG Polo\nBit: 80\nKey: 89 D7 8A C0 0E B2 C2 31\n"), "{text}");
        assert!(text.contains("Serial: 1216993162\nBtn: 2\nButtonRaw: 189\nHdr: 48456\n"), "{text}");
    }

    /// FoxFW's C on the eight Polo captures (48 frames): `Serial`, `Key` and `Hdr` of the frames of
    /// `lock1`, and the button byte's two values per action.
    #[test]
    fn imports_polo_captures_match_the_reference() {
        let dir = std::path::Path::new("IMPORTS/VAG");
        if !dir.is_dir() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let mut total = 0;
        for (file, frames, btn, raw) in [
            ("VW Polo 5 2017 lock1.sub", 3, 2, [0xBD, 0xBD]),
            ("VW Polo 5 2017 lock2.sub", 3, 2, [0xBF, 0xBF]),
            ("VW Polo 5 2017 lock3.sub", 3, 2, [0xBD, 0xBD]),
            ("VW Polo 5 2017 lock x5.sub", 15, 2, [0xBD, 0xBF]),
            ("VW Polo 5 2017 unlock1.sub", 3, 1, [0x7F, 0x7F]),
            ("VW Polo 5 2017 unlock2.sub", 3, 1, [0x7D, 0x7D]),
            ("VW Polo 5 2017 unlock3.sub", 3, 1, [0x7F, 0x7F]),
            ("VW Polo 5 2017 unlock x5.sub", 15, 1, [0x7D, 0x7F]),
        ] {
            let (_, pairs) = crate::export::flipper::import_sub_raw(&dir.join(file)).unwrap();
            let wire: Vec<LevelDuration> =
                pairs.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
            let got = feed_resetting(&mut VagPoloDecoder::new(), &wire);
            assert_eq!(got.len(), frames, "{file}");
            for d in &got {
                let b = d.extra.map(|e| (e >> 8) as u8).unwrap();
                assert!(d.button == Some(btn) && raw.contains(&b), "{file}: {d:?}");
            }
            if file == "VW Polo 5 2017 lock1.sub" {
                assert_eq!(got[0].serial, Some(1_216_993_162));
                assert_eq!(got[0].data, 0x89D7_8AC0_0EB2_C231);
                assert_eq!(got[0].extra, Some(48_456));
                assert_eq!(got[1].serial, Some(1_368_698_743));
                assert_eq!(got[2].serial, Some(1_750_845_385));
            }
            total += got.len();
        }
        assert_eq!(total, 48);
    }
}
