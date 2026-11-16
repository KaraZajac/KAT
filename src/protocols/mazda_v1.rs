//! Mazda V1 decoder (FoxFW) and a replay encoder (KAT's).
//!
//! Ported from FoxFW `lib/subghz/protocols/mazda_v1.c` (`SUBGHZ_PROTOCOL_MAZDA_V1_NAME = "Mazda
//! V1"`; the header credits the firmware `mazda_v1.c` via KeeLoq-RX `arke_mazda_gm.py`). No other
//! reference has it. 315 / 433.92 MHz, AM in FoxFW's flags, 2-FSK on the fobs captured (below).
//!
//! **Relation to Mazda V2** (Flipper-ARF, `mazda_v2.rs`): the same family of fobs and the same air
//! format: 505/1010 µs bits, the 0xEC sync and a 71-bit frame, with the button bytes 0x06 / 0x0A /
//! 0x0E that V2 calls its "alt" Lock / Unlock / Trunk. The two decoders gate differently, which is
//! why V1 is kept as its own module: V1 needs no long preamble (it scans the bit stream for the
//! sync at every position, both polarities) and additionally takes button 0x26 (Btn5 / Trunk), which
//! V2's six button codes reject; V2 in turn takes buttons 0x04 / 0x08 / 0x10, which V1 rejects. A
//! frame both know is listed as Mazda V2 (below). It is not KAT's Mazda V0 / Mazda Siemens
//! (250/500 µs pair decoders).
//!
//! **Line code** (`feed`): a half-bit-toggle Manchester at ts 505 / tl 1010, delta 200 (`<=`). The
//! first SHORT (305..=705 µs) after reset starts collecting with `half_bit` set; a LONG
//! (810..=1210 µs) is one bit equal to its level; of two SHORTs the second is one bit equal to its
//! level; any other duration ends the burst and runs the full scan. The bit buffer is capped at 400
//! (the scan also runs at 380 bits).
//!
//! **Frame** (71 bits): `sync8 = 0xEC | btn8 | serial6 + 2 more bits | ...`. After every added bit the
//! newest 71-bit window is tried (`streaming_check`); at the end of the burst the whole buffer is
//! scanned (`process_frame`). A window is accepted at polarity 0 or 1 (polarity 1 inverts every bit)
//! when: the 8 sync bits equal 0xEC; `btn & 0x7F` is 0x06, 0x0A, 0x0E or 0x26; frame bit 70 is 1; and
//! either the "v1 marker" (bits 18..21 = 1,0,0,0 and bit 37 = 0) or the "v3 marker" (bits 18..21 are
//! anything but 1,0,0,0) holds. No CRC. After an accepted frame, windows starting less than
//! 142 (2 x 71) bits later are skipped. An accepted polarity-1 window is inverted in place in the
//! buffer, as the C does.
//!
//! **Fields**: `serial` = the 6-bit per-remote constant `(byte2 >> 2) & 0x3F`, `button` = the full
//! button byte (bits 8..15), `counter` = a 5-bit cleartext field built from frame bits 49..53 (the
//! top bit is bit53 XOR bit49). `data` = the first 64 frame bits, `extra` = the 7 tail bits (frame
//! bits 64..70), `data_count_bit` = 71.
//!
//! **Reports** as the C does: a frame is reported when its last bit arrives. KAT's registry resets
//! every decoder after a decode, so the second copy of a frame sent twice is collected anew and
//! reported too (the FoxFW oracle resets the same way and lists both; on a Flipper the C skips
//! windows that start less than 142 bits after an accepted one, and shows one).
//!
//! **Frames that are also Mazda V2's are listed as Mazda V2.** A frame with button 0x06 / 0x0A /
//! 0x0E is the frame Mazda V2 reads behind its preamble, and Mazda V2 can send it again, so such a
//! frame is reported with Mazda V2's fields and under its name ([super::mazda_v2::
//! frame_without_preamble]: `data` = the 63 bits after the sync, left-aligned; no serial; counter =
//! its idx). "Mazda V1" is left with button 0x26, which Mazda V2 does not know. Mazda V2's own
//! reader wants more than 99 preamble samples before the sync and so reads only the first frame of
//! a press; this reader needs no preamble and gives the rest: the second copy of every frame, and
//! the new frames a held button sends every ~100 ms behind a preamble of about 90 bits. Until KAT
//! 1.13 this decoder held the first frame of a burst back until the burst ended, to let Mazda V2
//! report first, and so listed one frame per burst; the frames behind it were never listed.
//!
//! **Transmit** (KAT's; FoxFW has no encoder): the captured frame again, as it is, in Mazda V2's
//! upload ([super::mazda_v2::replay_upload]: a 180-bit preamble, the sync and the 63 bits, three
//! copies). The frame's rolling part is not understood for button 0x26 (Mazda V2's tables cover
//! its six buttons), so there is no next code to build, and a receiver that has seen the frame may
//! refuse it. The fob itself sends one preamble and the frame twice back to back.
//!
//! **AM and FM.** FoxFW flags the protocol AM only (its receiver does not gate by modulation). The
//! captures of real fobs in IMPORTS (`Mazda/`, FM476) are 2-FSK, so KAT reads it on both.
//!
//! IMPORTS: `Mazda/MazdaV1F4_OEM2_LU.sub` (one lock, one unlock: two frames, each twice),
//! `Mazda/mazda_key3_10x_trunk.sub` (button 0x26: 10 presses, 20 frames), and the 300-press and
//! held-button recordings in `Mazda/more/`. Counts against the references: `docs/mazda_v2.md`.

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Mazda V1",
    decoder: "Mazda V1",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::Both,
    encryption: "Rolling Code",
    make: "Mazda",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650, &FM476],
};

const TE_SHORT: u32 = 505;
const TE_LONG: u32 = 1010;
const TE_DELTA: u32 = 200;
const FRAME_BITS: usize = 71;
const CAP: usize = 400;
const PROCESS_AT: usize = 380;

/// 0xEC = 1110 1100, MSB first.
const SYNC: [u8; 8] = [1, 1, 1, 0, 1, 1, 0, 0];

pub struct MazdaV1Decoder {
    collecting: bool,
    half_bit: bool,
    all_count: usize,
    last_pos: i32,
    bits: [u8; CAP],
}

/// `mazda_v1_extract_cnt`: frame bits 49..53 (MSB-indexed in the 64-bit word).
fn extract_cnt(data: u64) -> u8 {
    let bit = |n: u32| ((data >> (63 - n)) & 1) as u8;
    let (fb53, fb52, fb51, fb50, fb49) = (bit(53), bit(52), bit(51), bit(50), bit(49));
    let b4 = fb53 ^ fb49;
    (b4 << 4) | (fb50 << 3) | (fb51 << 2) | (fb52 << 1) | fb53
}

impl MazdaV1Decoder {
    pub fn new() -> Self {
        Self { collecting: false, half_bit: false, all_count: 0, last_pos: -32767, bits: [0; CAP] }
    }

    /// `mazda_v1_try_emit_at`: lock a 71-bit frame at `sync_pos` under `polarity` (0 / 1).
    fn try_emit_at(&mut self, sync_pos: usize, polarity: u8) -> Option<DecodedSignal> {
        if sync_pos + FRAME_BITS > self.all_count {
            return None;
        }
        let at = |s: &Self, i: usize| s.bits[sync_pos + i] ^ polarity;

        if (0..8).any(|j| at(self, j) != SYNC[j]) {
            return None;
        }
        let btn = (0..8).fold(0u8, |a, j| (a << 1) | at(self, 8 + j));
        let btn7 = btn & 0x7F;
        if !matches!(btn7, 0x06 | 0x0A | 0x0E | 0x26) {
            return None;
        }
        if at(self, 70) != 1 {
            return None;
        }
        let (b18, b19, b20, b21, b37) =
            (at(self, 18), at(self, 19), at(self, 20), at(self, 21), at(self, 37));
        let v1_marker = b18 == 1 && b19 == 0 && b20 == 0 && b21 == 0 && b37 == 0;
        let v3_marker = !(b18 == 1 && b19 == 0 && b20 == 0 && b21 == 0);
        if !v1_marker && !v3_marker {
            return None;
        }

        if polarity == 1 {
            for k in sync_pos..sync_pos + FRAME_BITS {
                self.bits[k] ^= 1;
            }
        }
        let mut data = 0u64;
        for i in 0..64 {
            data = (data << 1) | u64::from(self.bits[sync_pos + i] & 1);
        }
        let mut tail = 0u8;
        for i in 64..FRAME_BITS {
            tail = (tail << 1) | (self.bits[sync_pos + i] & 1);
        }
        // A frame Mazda V2 reads too goes out as Mazda V2's (see the module doc).
        let payload = (8..FRAME_BITS).fold(0u64, |a, i| (a << 1) | u64::from(self.bits[sync_pos + i] & 1));
        if let Some(signal) = super::mazda_v2::frame_without_preamble(payload) {
            return Some(signal);
        }
        let btn_out = (data >> 48) as u8;
        let b2 = (data >> 40) as u8;
        Some(DecodedSignal {
            serial: Some(u32::from((b2 >> 2) & 0x3F)),
            button: Some(btn_out),
            counter: Some(u16::from(extract_cnt(data))),
            crc_valid: false,
            data,
            data_count_bit: FRAME_BITS,
            encoder_capable: true,
            extra: Some(u64::from(tail)),
            protocol_display_name: None,
        })
    }

    /// `mazda_v1_streaming_check`: only the newest window.
    fn streaming_check(&mut self) -> Option<DecodedSignal> {
        let n = self.all_count;
        if n < FRAME_BITS {
            return None;
        }
        let sync_pos = n - FRAME_BITS;
        if self.last_pos >= 0 && (sync_pos as i32) < self.last_pos + 2 * FRAME_BITS as i32 {
            return None;
        }
        for polarity in 0..=1 {
            if let Some(s) = self.try_emit_at(sync_pos, polarity) {
                self.last_pos = sync_pos as i32;
                return Some(s);
            }
        }
        None
    }

    /// `mazda_v1_process_frame`: the whole buffer (end of burst / buffer full).
    fn process_frame(&mut self) -> Option<DecodedSignal> {
        let n = self.all_count;
        if n < FRAME_BITS {
            return None;
        }
        let mut first = None;
        for i in 0..=(n - FRAME_BITS) {
            if self.last_pos >= 0 && (i as i32) < self.last_pos + 2 * FRAME_BITS as i32 {
                continue;
            }
            for polarity in 0..=1 {
                if let Some(s) = self.try_emit_at(i, polarity) {
                    self.last_pos = i as i32;
                    first.get_or_insert(s);
                    break;
                }
            }
        }
        first
    }

    /// The end of a burst: the full scan (the C reports every frame it finds there; behind the
    /// per-bit check there is none left, and KAT reports the first).
    fn finish(&mut self) -> Option<DecodedSignal> {
        let scanned = self.process_frame();
        self.collecting = false;
        scanned
    }
}

impl Default for MazdaV1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for MazdaV1Decoder {
    fn name(&self) -> &'static str {
        "Mazda V1"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: FRAME_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.collecting = false;
        self.half_bit = false;
        self.all_count = 0;
        self.last_pos = -32767;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let is_short = duration.abs_diff(TE_SHORT) <= TE_DELTA;
        let is_long = duration.abs_diff(TE_LONG) <= TE_DELTA;

        if !self.collecting {
            if is_short {
                self.collecting = true;
                self.all_count = 0;
                self.last_pos = -32767;
                self.half_bit = true;
            }
            return None;
        }
        if self.all_count >= PROCESS_AT {
            return self.finish();
        }
        let mut bit_added = false;
        if is_long {
            if self.all_count < CAP {
                self.bits[self.all_count] = u8::from(level);
                self.all_count += 1;
                bit_added = true;
            }
            self.half_bit = false;
        } else if is_short {
            if self.half_bit {
                if self.all_count < CAP {
                    self.bits[self.all_count] = u8::from(level);
                    self.all_count += 1;
                    bit_added = true;
                }
                self.half_bit = false;
            } else {
                self.half_bit = true;
            }
        } else {
            return self.finish();
        }
        if bit_added {
            return self.streaming_check();
        }
        None
    }

    fn reads_frames_of(&self) -> Option<&'static str> {
        Some("Mazda V2")
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// The captured frame again, as it is, in Mazda V2's upload (module doc, "Transmit").
    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != FRAME_BITS || (decoded.data >> 56) as u8 != 0xEC {
            return None;
        }
        let tail = decoded.extra.filter(|t| *t < 1 << (FRAME_BITS - 64))?;
        Some(super::mazda_v2::replay_upload(((decoded.data & 0x00FF_FFFF_FFFF_FFFF) << 7) | tail))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::dd_testutil::{bits_of, dump_sub, push, toggle_bits, Pulse};

    const PRESET: &str = "FuriHalSubGhzPresetOok650Async";

    /// 71 frame bits: sync, button, 6-bit serial + 2 bits, then filler built from `body`.
    /// `body` = the 55 bits after the button byte (bits 16..70); bit 70 (the last) is forced to 1.
    fn frame_bits(btn: u8, body: u64) -> Vec<u8> {
        let mut bits = bits_of(0xEC, 8);
        bits.extend(bits_of(u64::from(btn), 8));
        let mut tail = bits_of(body, 55);
        *tail.last_mut().unwrap() = 1;
        bits.extend(tail);
        bits
    }

    /// Lead-in of alternating 505 µs pulses, the frame sent twice back to back, a 4 ms gap.
    fn wire(bits: &[u8], copies: usize, invert: bool) -> Vec<Pulse> {
        let mut p: Vec<Pulse> = Vec::new();
        for i in 0..12 {
            push(&mut p, i % 2 == 0, 505);
        }
        let mut all = Vec::new();
        for _ in 0..copies {
            all.extend_from_slice(bits);
        }
        if invert {
            all.iter_mut().for_each(|b| *b ^= 1);
        }
        toggle_bits(&mut p, &all, 505, 1010);
        let end = !p.last().unwrap().0;
        p.push((end, 4000));
        p
    }

    fn run(p: &[Pulse]) -> Vec<DecodedSignal> {
        let mut d = MazdaV1Decoder::new();
        p.iter().filter_map(|&(l, us)| d.feed(l, us)).collect()
    }

    // Bits 16..23 = 0b0101_1000 (serial 0b010110), 18..21 = 0,1,1,0 (v3 marker: not 1000).
    const BODY: u64 = 0x0058_0ABC_DEF1_2345 & ((1 << 55) - 1);

    #[test]
    fn decodes_a_made_up_frame_sent_twice() {
        let bits = frame_bits(0x26, BODY);
        let p = wire(&bits, 2, false);
        dump_sub("mazda_v1_ok", 433_920_000, PRESET, &p);
        let r = run(&p);
        // Fed without a reset in between (as on a Flipper), the second copy starts less than 142
        // bits behind the first and is skipped.
        assert_eq!(r.len(), 1);
        let s = &r[0];
        let mut data = 0u64;
        for &b in &bits[..64] {
            data = (data << 1) | u64::from(b);
        }
        assert_eq!(s.data, data);
        assert_eq!(s.button, Some(0x26));
        assert_eq!(s.data_count_bit, 71);
        assert_eq!(s.extra, Some(bits[64..].iter().fold(0u64, |a, &b| (a << 1) | u64::from(b))));
        assert_eq!(s.serial, Some(((data >> 42) & 0x3F) as u32));
        assert!(!s.crc_valid && s.encoder_capable && s.protocol_display_name.is_none());
        // What the FoxFW oracle reports for the same pulses (Key, Serial, Btn, Cnt), twice: like
        // KAT's registry it resets its decoders after each decode, so both copies are listed.
        assert_eq!((s.data, s.serial, s.button, s.counter), (0xEC26_B015_79BD_E246, Some(44), Some(0x26), Some(24)));
        assert_eq!(crate::protocols::dd_testutil::registry_names(&p, 433_920_000), ["Mazda V1", "Mazda V1"]);
    }

    /// The encoder sends the captured frame as it is, in Mazda V2's upload: three copies, each
    /// read back with every bit (key and tail) and as Mazda V1; no button changes the frame.
    #[test]
    fn replays_the_captured_frame() {
        let src = run(&wire(&frame_bits(0x26, BODY), 2, false)).remove(0);
        let dec = MazdaV1Decoder::new();
        assert!(dec.supports_encoding());
        let tx = dec.encode(&src, 0x02).unwrap();
        assert_eq!(dec.encode(&src, 0x01).unwrap(), tx);
        let mut reg = crate::protocols::ProtocolRegistry::new();
        let got = reg.process_signal_stream(&tx, 433_920_000);
        assert_eq!(got.len(), 3, "{:?}", got.iter().map(|(n, ..)| n).collect::<Vec<_>>());
        for (name, d, _) in &got {
            assert_eq!(name, "Mazda V1");
            assert_eq!((d.data, d.extra, d.data_count_bit, d.button), (src.data, src.extra, 71, Some(0x26)));
        }
        // Not a frame of this decoder: another length, no sync, a tail of more than 7 bits.
        assert!(dec.encode(&DecodedSignal { data_count_bit: 63, ..src.clone() }, 0).is_none());
        assert!(dec.encode(&DecodedSignal { data: src.data ^ (1 << 60), ..src.clone() }, 0).is_none());
        assert!(dec.encode(&DecodedSignal { extra: Some(0x80), ..src.clone() }, 0).is_none());
        assert!(dec.encode(&DecodedSignal { extra: None, ..src.clone() }, 0).is_none());
    }

    /// A frame is reported on its last bit: a recording cut inside the second copy has its first.
    #[test]
    fn a_frame_is_reported_on_its_last_bit() {
        let one = wire(&frame_bits(0x26, BODY), 1, false);
        let mut d = MazdaV1Decoder::new();
        let at = one.iter().position(|&(l, us)| d.feed(l, us).is_some());
        // The last bit's pulse: the burst-ending gap behind it is not needed.
        assert_eq!(at, Some(one.len() - 2));
        let mut p = wire(&frame_bits(0x26, BODY), 2, false);
        p.truncate(p.len() * 4 / 5);
        assert_eq!(crate::protocols::dd_testutil::registry_names(&p, 433_920_000), ["Mazda V1"]);
        // Cut before the first copy's last bit: nothing was complete, nothing is reported.
        let mut d = MazdaV1Decoder::new();
        assert!(one[..one.len() - 3].iter().all(|&(l, us)| d.feed(l, us).is_none()));
        assert!(d.flush().is_none());
    }

    /// Button 0x06 / 0x0A / 0x0E frames are Mazda V2's: this reader reports them with Mazda V2's
    /// fields and under its name, with or without Mazda V2's preamble, and every copy of them.
    #[test]
    fn frames_mazda_v2_knows_are_listed_as_mazda_v2() {
        let names = crate::protocols::dd_testutil::registry_names;
        let bits = frame_bits(0x0A, BODY);
        let p = wire(&bits, 2, false);
        dump_sub("mazda_v1_v2frame", 433_920_000, PRESET, &p);
        let s = run(&p).remove(0);
        // The 63 bits behind the sync, left-aligned: what Mazda V2's own reader stores.
        let payload = bits[8..].iter().fold(0u64, |a, &b| (a << 1) | u64::from(b));
        assert_eq!((s.data, s.data_count_bit, s.serial, s.button), (payload << 1, 63, None, Some(0x0A)));
        assert_eq!(s.protocol_display_name.as_deref(), Some("Mazda V2"));
        assert!(s.encoder_capable && s.crc_valid);
        assert_eq!(names(&p, 433_920_000), ["Mazda V2", "Mazda V2"]);
        // Button 0x26 is not one of Mazda V2's six button codes.
        let p = wire(&frame_bits(0x26, BODY), 2, false);
        assert_eq!(names(&p, 433_920_000)[0], "Mazda V1");

        // A press as the fobs in IMPORTS send it: 180 short pairs of preamble, the frame twice.
        // Mazda V2's own reader gives the first copy only (it wants its preamble before the sync).
        let mut p: Vec<Pulse> = Vec::new();
        // 361 alternating pulses starting LOW: V2's sampler stores 180 LOW samples, then the sync.
        for i in 0..361 {
            push(&mut p, i % 2 == 1, 505);
        }
        let mut twice = bits.clone();
        twice.extend_from_slice(&bits);
        toggle_bits(&mut p, &twice, 505, 1010);
        let end = !p.last().unwrap().0;
        p.push((end, 4000));
        dump_sub("mazda_v1_v2style", 433_920_000, PRESET, &p);
        let pairs: Vec<LevelDuration> = p.iter().map(|&(l, d)| LevelDuration::new(l, d)).collect();
        let listed = crate::protocols::ProtocolRegistry::new().process_signal_stream(&pairs, 433_920_000);
        assert_eq!(listed.len(), 2);
        assert!(listed.iter().all(|(name, d, _)| name == "Mazda V2" && d.data == payload << 1));
        let mut v2 = crate::protocols::mazda_v2::MazdaV2Decoder::new();
        let own: Vec<DecodedSignal> = pairs.iter().filter_map(|x| v2.feed(x.level, x.duration_us)).collect();
        assert_eq!(own.len(), 1);
        assert_eq!(own[0].data, payload << 1);
        // The registry lists this reader's two frames exactly as Mazda V2's own decode (the
        // display name that carried them there is dropped: `reads_frames_of`).
        assert!(listed.iter().all(|(_, d, _)| *d == own[0]), "{listed:?}");
    }

    #[test]
    fn takes_button_0x26_and_inverted_levels() {
        let p = wire(&frame_bits(0x26, BODY), 2, false);
        dump_sub("mazda_v1_btn26", 433_920_000, PRESET, &p);
        assert_eq!(run(&p)[0].button, Some(0x26));
        let p = wire(&frame_bits(0x0E, BODY), 2, true);
        dump_sub("mazda_v1_inverted", 433_920_000, PRESET, &p);
        assert_eq!(run(&p)[0].button, Some(0x0E));
        let p = wire(&frame_bits(0x26, BODY), 2, true);
        assert_eq!(run(&p)[0].data, 0xEC26_B015_79BD_E246);
    }

    #[test]
    fn rejects_bad_sync_button_end_marker_and_short_frames() {
        // Button 0x04 (V2's Lock) is not on V1's whitelist.
        let p = wire(&frame_bits(0x04, BODY), 2, false);
        dump_sub("mazda_v1_badbtn", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "button 0x04");
        // One sync bit wrong (0xEC -> 0xEE).
        let mut bits = frame_bits(0x0A, BODY);
        bits[6] ^= 1;
        let p = wire(&bits, 2, false);
        dump_sub("mazda_v1_badsync", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "bad sync");
        // Bit 70 = 0.
        let mut bits = frame_bits(0x0A, BODY);
        bits[70] = 0;
        let p = wire(&bits, 2, false);
        dump_sub("mazda_v1_badend", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "bit 70 clear");
        // Bits 18..21 = 1,0,0,0 without bit 37 = 0 fail both markers.
        let mut bits = frame_bits(0x0A, BODY);
        bits[18..22].copy_from_slice(&[1, 0, 0, 0]);
        bits[37] = 1;
        let p = wire(&bits, 2, false);
        dump_sub("mazda_v1_badmarker", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "v1 marker without bit 37 = 0");
        // 70 bits.
        let mut bits = frame_bits(0x0A, BODY);
        bits.truncate(70);
        let p = wire(&bits, 1, false);
        dump_sub("mazda_v1_70bits", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "70-bit frame");
    }
}
