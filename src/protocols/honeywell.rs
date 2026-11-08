//! Honeywell Sec: 64-bit frames of Honeywell / 2GIG security sensors (door / window contacts,
//! motion), decoder + encoder.
//!
//! Reference: doctor-firmware `lib/subghz/protocols/honeywell.c` (HTotoo 2023, after rtl_433's
//! `honeywell.c` and Logan Lamb's "DEFCON 22: Home Insecurity"; @xMasterX 2025;
//! `SUBGHZ_PROTOCOL_HONEYWELL_NAME` = "Honeywell Sec"), which doctor registers. Not to be confused
//! with "Honeywell" (`honeywell_wdb.c`, the doorbells: `honeywell_wdb.rs`). The copies in
//! Flipper-ARF `lib` (unregistered there), the garage app (not in its registry), Momentum,
//! RogueMaster and FoxFW (encoder in its SD plugin `enc_honeywell`) have the same feed, bit and
//! upload code. Differences: doctor flags 315 and 433 MHz, the others 315 only (KAT, like every
//! Flipper decoder, runs on any band); ARF's `lib` and FoxFW display less; and FoxFW's alloc and
//! reset also put the Manchester decoder in `Mid1`, where doctor's (and every other copy's) starts
//! zero-initialised (`Start1`) and is never reset.
//!
//! **Union of doctor and FoxFW.** That one line decides whether a repeated frame decodes: the 84 ms
//! LOW after a frame leaves doctor's Manchester state where the frame's last bit put it, so the next
//! frame syncs two half-bits late and only 60 of its bits are read (the check needs 62), while
//! FoxFW's reset after the report puts it in `Mid1`, which syncs on the first HIGH. On doctor's own
//! encoder output (4 copies of one frame) doctor decodes the first copy only, and nothing when the
//! copies follow a 20 ms gap; FoxFW decodes every copy (4, and 8 of 8). Doctor in turn reads one more
//! bit at the very start of a fresh decoder (`Start1` takes a lone short LOW as a bit, `Mid1` does not).
//! KAT runs both state machines ("lanes") on the same pulses, each with its own Manchester state and
//! bit register, and reports a frame as soon as either lane completes one; the report then resets
//! both lanes the way each copy's callback would (doctor: the register; FoxFW: the register and
//! `Mid1`), so a frame is reported once.
//!
//! **Air format** (AM): Manchester, te_short 143 µs, te_long 280 µs (te_delta 51 µs). A frame is 64
//! bits, MSB first: 16-bit preamble 0xFFFE, 4-bit channel, 20-bit serial, 8-bit event, 16-bit CRC.
//!
//! **Decoder** (`subghz_protocol_decoder_honeywell_feed` / `_addbit`): each pulse is an event of
//! Flipper's Manchester machine (`lib/toolbox/manchester_decoder.c`, [super::common]): LOW |d − 143|
//! < 51 → ShortLow, else |d − 280| < 102 → LongLow (HIGH: ShortHigh / LongHigh); any other duration
//! clears the bit register but leaves the Manchester state as it is. Each decoded bit is shifted in;
//! from the 62nd bit on, the register's bits 63..48 must be 0x3FFE, 0x7FFE or 0xFFFE (14 to 16
//! preamble bits: "the sensors are not that good"); then channel 2, 4 or 0xA (2GIG) takes CRC-16 poly
//! 0x8050, channel 8 poly 0x8005 (init 0, MSB first, over bytes 5..2), any other channel clears the
//! register; a matching CRC reports the frame with its top byte set to 0xFF (`FF FE` header), a
//! mismatch clears the register, and 64 bits without a preamble clear it too.
//!
//! **Fields** (`subghz_protocol_decoder_honeywell_get_string`): serial = bits 43..24 (20 bits),
//! channel = bits 47..44, event = bits 23..16 (0x80 contact open, 0x40 tamper, 0x20 reed, 0x10
//! alarm, 0x08 low battery, 0x04 heartbeat). KAT: `serial` as the C, `button` / `counter` None
//! (the C sets neither), `crc_valid` = the CRC holds (always for decodes).
//!
//! **Encoder** (`subghz_protocol_encoder_honeywell_get_upload`, `encoder.repeat` = 4): bits 62..0
//! through Flipper's Manchester encoder (bit 63 is not sent), the closing half when it is HIGH, then
//! a LOW of 300·te_long = 84 ms. The key as stored; nothing to switch (`button` is ignored).
//!
//! **Key file**: `Protocol: Honeywell Sec`, `Bit: 64`, `Key` (`subghz_block_generic_serialize`;
//! doctor's serializer also writes `Serial` / `Btn` / `Cnt`, which KAT does not). Loading: the
//! decoder's deserialize refuses `Bit` < 62 and reads a 62- or 63-bit key (older builds) with its
//! header set to `FF FE`; the encoder's deserialize checks no bit count and sends the file's `Key` as
//! it is. KAT keeps the file's `Key` and `Bit` (what the C transmits) and reads the fields as the
//! decoder's deserialize rebuilds them.

use super::common::{ManchesterEvent, ManchesterState};
use super::f_common::{push, with_repeats};
use super::g3_common::ManchesterEncoder;
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): doctor's `SubGhzProtocolFlag_315 | 433 | AM | Sensors`,
/// `SubGhzProtocolTypeStatic`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Honeywell",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

pub(super) const NAME: &str = "Honeywell Sec";
const TE_SHORT: u32 = 143;
const TE_LONG: u32 = 280;
const TE_DELTA: u32 = 51;
const BIT_COUNT: usize = 64;
/// The decoder's deserialize accepts 62..=64-bit keys (older builds saved partial preambles).
const MIN_KEY_BITS: usize = 62;
/// The LOW after each frame: 300·te_long.
const DELAY: u32 = TE_LONG * 300;
/// `subghz_protocol_encoder_common_alloc(…, 4, 64 * 2 + 10)`.
const REPEAT: usize = 4;

/// One copy's state machine (`SubGhzProtocolDecoderHoneywell`): its Manchester state and bit
/// register.
#[derive(Debug, Clone, Copy)]
struct Lane {
    manchester: ManchesterState,
    decode_data: u64,
    decode_count_bit: u8,
    /// FoxFW: alloc and reset put the Manchester state in `Mid1` (doctor: `Start1`, never reset).
    reset_to_mid1: bool,
}

/// Honeywell Sec decoder: doctor's lane and FoxFW's lane (see the module doc).
pub struct HoneywellDecoder {
    lanes: Vec<Lane>,
}

/// `subghz_protocol_honeywell_crc16`: MSB first, init 0.
fn crc16(message: &[u8], polynomial: u16) -> u16 {
    let mut remainder = 0u16;
    for &byte in message {
        remainder ^= u16::from(byte) << 8;
        for _ in 0..8 {
            remainder = if remainder & 0x8000 != 0 { (remainder << 1) ^ polynomial } else { remainder << 1 };
        }
    }
    remainder
}

/// The CRC the frame's channel calls for, over bytes 5..2; None for a channel without one.
fn crc_of(data: u64) -> Option<u16> {
    let bytes = [(data >> 40) as u8, (data >> 32) as u8, (data >> 24) as u8, (data >> 16) as u8];
    match (data >> 44) & 0xF {
        0x2 | 0x4 | 0xA => Some(crc16(&bytes, 0x8050)),
        0x8 => Some(crc16(&bytes, 0x8005)),
        _ => None,
    }
}

/// "Removing possible artifacts from higher bits and setting header to FF FE": the top byte 0xFF.
fn with_header(data: u64) -> u64 {
    0xFF00_0000_0000_0000 | (data & 0x00FF_FFFF_FFFF_FFFF)
}

/// A decode of the 64-bit `data`, or a loaded key (`data` / `bits` as stored; the fields read from
/// the deserialized form).
fn signal(data: u64, bits: usize) -> DecodedSignal {
    let rebuilt = if bits == BIT_COUNT { data } else { with_header(data) };
    DecodedSignal {
        serial: Some(((rebuilt >> 24) & 0xF_FFFF) as u32),
        button: None,
        counter: None,
        crc_valid: crc_of(rebuilt) == Some(rebuilt as u16),
        data,
        data_count_bit: bits,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One `get_upload`: bits 62..0 Manchester-coded, the closing half when HIGH, the 84 ms delay.
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut enc = ManchesterEncoder::new();
    let mut results = Vec::with_capacity(2 * BIT_COUNT);
    for i in (1..BIT_COUNT).rev() {
        enc.encode_bit((data >> (i - 1)) & 1 == 1, &mut results);
    }
    let mut out = Vec::with_capacity(2 * BIT_COUNT + 2);
    for r in results {
        let p = r.pulse(TE_SHORT, TE_LONG);
        push(&mut out, p.level, p.duration_us);
    }
    let last = enc.finish().pulse(TE_SHORT, TE_LONG);
    if last.level {
        push(&mut out, true, last.duration_us);
    }
    push(&mut out, false, DELAY);
    out
}

impl Lane {
    fn new(reset_to_mid1: bool) -> Self {
        Self {
            manchester: if reset_to_mid1 { ManchesterState::Mid1 } else { ManchesterState::Start1 },
            decode_data: 0,
            decode_count_bit: 0,
            reset_to_mid1,
        }
    }

    fn clear(&mut self) {
        self.decode_data = 0;
        self.decode_count_bit = 0;
    }

    /// `subghz_protocol_decoder_honeywell_reset` of this lane's copy.
    fn reset(&mut self) {
        self.clear();
        if self.reset_to_mid1 {
            self.manchester = ManchesterState::Mid1;
        }
    }

    /// `subghz_protocol_decoder_honeywell_feed`: the event (None = `ManchesterEventReset`).
    fn feed(&mut self, event: Option<ManchesterEvent>) -> Option<DecodedSignal> {
        match event {
            Some(event) => {
                let bit = self.manchester.advance(event)?;
                self.add_bit(bit)
            }
            None => {
                // The register is cleared, the Manchester state is not touched.
                self.clear();
                None
            }
        }
    }

    /// `subghz_protocol_decoder_honeywell_addbit`.
    fn add_bit(&mut self, bit: bool) -> Option<DecodedSignal> {
        self.decode_data = (self.decode_data << 1) | u64::from(bit);
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
        if self.decode_count_bit < 62 {
            return None;
        }
        let d = self.decode_data;
        if matches!((d >> 48) & 0xFFFF, 0x3FFE | 0x7FFE | 0xFFFE) {
            let ok = crc_of(d).is_some_and(|crc| crc == d as u16);
            self.clear();
            return ok.then(|| signal(with_header(d), BIT_COUNT));
        }
        if self.decode_count_bit >= 64 {
            self.clear();
        }
        None
    }
}

impl HoneywellDecoder {
    pub fn new() -> Self {
        Self { lanes: vec![Lane::new(false), Lane::new(true)] }
    }

    /// One reference's decoder alone (parity tests): doctor's lane or FoxFW's.
    #[cfg(test)]
    pub(super) fn single(foxfw: bool) -> Self {
        Self { lanes: vec![Lane::new(foxfw)] }
    }
}

impl Default for HoneywellDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HoneywellDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: BIT_COUNT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    /// `subghz_protocol_decoder_honeywell_reset` of each lane: the bit registers (and FoxFW's
    /// Manchester state).
    fn reset(&mut self) {
        for lane in &mut self.lanes {
            lane.reset();
        }
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let event = if duration.abs_diff(TE_SHORT) < TE_DELTA {
            Some(ManchesterEvent::from_half_bit(level, false))
        } else if duration.abs_diff(TE_LONG) < TE_DELTA * 2 {
            Some(ManchesterEvent::from_half_bit(level, true))
        } else {
            None
        };
        let mut out = None;
        for lane in &mut self.lanes {
            if let Some(d) = lane.feed(event) {
                out.get_or_insert(d);
            }
        }
        if out.is_some() {
            // The callback's receiver reset, as each copy applies it.
            self.reset();
        }
        out
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        Some(with_repeats(&upload(decoded.data), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits >= MIN_KEY_BITS).then(|| signal(key.key, key.bits))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        Some(FlipperKey::for_protocol(NAME, decoded.data_count_bit, decoded.data))
    }
}

#[cfg(test)]
pub(super) mod testing {
    use super::*;

    /// A 64-bit frame (`FF FE` header) of `channel` (2, 4, 0xA or 8), `serial` and `event` with its CRC.
    pub fn frame(channel: u8, serial: u32, event: u8) -> u64 {
        let body = 0xFFFE_0000_0000_0000
            | (u64::from(channel & 0xF) << 44)
            | (u64::from(serial & 0xF_FFFF) << 24)
            | (u64::from(event) << 16);
        body | u64::from(crc_of(body).expect("channel 2, 4, 0xA or 8"))
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::testing::frame;
    use super::*;

    /// (level, µs) of each pulse (`LevelDuration` has no `PartialEq`).
    fn levels(wire: &[LevelDuration]) -> Vec<(bool, u32)> {
        wire.iter().map(|p| (p.level, p.duration_us)).collect()
    }

    fn on_air(data: u64, copies: usize) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, 20_000)];
        for p in with_repeats(&upload(data), copies) {
            push(&mut wire, p.level, p.duration_us);
        }
        wire
    }

    #[test]
    fn crc16_of_both_polynomials() {
        // Poly 0x8005, init 0, MSB first, no final XOR is CRC-16/UMTS: check value 0xFEE8.
        assert_eq!(crc16(b"123456789", 0x8005), 0xFEE8);
        assert_eq!(crc16(&[], 0x8050), 0);
        // doctor's encoder round trip of this frame: "Key: FF FE 81 23 45 80 8B 86 … Sn:0074565  Ch:8".
        assert_eq!(frame(0x8, 0x12345, 0x80), 0xFFFE_8123_4580_8B86);
        let f = frame(0x8, 0x12345, 0x80);
        assert_eq!(f >> 48, 0xFFFE);
        assert_eq!(crc_of(f), Some(f as u16));
        assert_ne!(crc_of(frame(0x2, 0x12345, 0x80)), crc_of(f));
        assert_eq!(crc_of(f & !(0xF << 44) | (0x3 << 44)), None, "channel 3 has no CRC");
    }

    /// The encoder's copies after a gap: FoxFW's lane reads every copy (as the FoxFW oracle does, 4 of
    /// 4), doctor's lane none of them (the doctor oracle: none) — its Manchester state, left by the
    /// previous frame or by the start, syncs two half-bits late.
    #[test]
    fn every_copy_decodes_with_its_fields() {
        for (ch, sn, ev) in [(0x8u8, 0x12345u32, 0x80u8), (0x2, 0xFFFFF, 0x04), (0x4, 0x00001, 0x00), (0xA, 0xABCDE, 0xFF)] {
            let data = frame(ch, sn, ev);
            let wire = on_air(data, 4);
            let got = feed_all(&mut HoneywellDecoder::new(), &wire);
            assert_eq!(got.len(), 4, "{data:016X}");
            for d in &got {
                assert_eq!((d.data, d.data_count_bit, d.serial), (data, 64, Some(sn)));
                assert!(d.crc_valid && d.encoder_capable && d.button.is_none());
            }
            assert_eq!(feed_resetting(&mut HoneywellDecoder::single(true), &wire).len(), 4);
            assert!(feed_resetting(&mut HoneywellDecoder::single(false), &wire).is_empty());
            // Without the gap the very first half-bit is a lone short LOW: doctor's fresh `Start1`
            // takes it as a bit and reads the first copy (the doctor oracle on its own encoder output).
            let bare = HoneywellDecoder::new().encode(&signal(data, 64), 0).unwrap();
            assert_eq!(feed_resetting(&mut HoneywellDecoder::single(false), &bare).len(), 1);
            assert_eq!(feed_all(&mut HoneywellDecoder::new(), &bare).len(), 4);
        }
    }

    #[test]
    fn rejects_a_wrong_crc_and_an_unknown_channel() {
        let good = frame(0x8, 0x5A5A5, 0x80);
        assert_eq!(feed_all(&mut HoneywellDecoder::new(), &on_air(good, 1)).len(), 1);
        assert!(feed_all(&mut HoneywellDecoder::new(), &on_air(good ^ 1, 2)).is_empty(), "CRC");
        let other = (good & !(0xF << 44)) | (0x3 << 44);
        assert!(feed_all(&mut HoneywellDecoder::new(), &on_air(other, 2)).is_empty(), "channel 3");
    }

    #[test]
    fn upload_layout() {
        let up = upload(frame(0x8, 0x12345, 0x80));
        // Bit 62 (1) opens with its LOW half, merged on air into the delay before it.
        assert_eq!((up[0].level, up[0].duration_us), (false, TE_SHORT));
        assert_eq!((up[1].level, up[1].duration_us), (true, TE_SHORT));
        assert_eq!(levels(&up[up.len() - 1..]), [(false, DELAY)]);
        let tx = HoneywellDecoder::new().encode(&signal(frame(0x8, 0x12345, 0x80), 64), 0).unwrap();
        assert_eq!(levels(&tx), levels(&with_repeats(&up, 4)));
    }

    #[test]
    fn key_file_round_trip_and_partial_preambles() {
        let dec = HoneywellDecoder::new();
        let data = frame(0x2, 0x0BEEF, 0x84);
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &signal(data, 64));
        assert!(text.contains(&format!("Protocol: Honeywell Sec\nBit: 64\nKey: {}\n", hex(data))), "{text}");
        // A 62-bit key of an older build: loaded, fields read from the FF FE form, sent as stored.
        let old = FlipperKey::parse(&text.replace("Bit: 64", "Bit: 62")).unwrap();
        let mut old = old;
        old.key = data & 0x3FFF_FFFF_FFFF_FFFF;
        old.key_bytes = old.key.to_be_bytes().to_vec();
        let d = dec.from_flipper_key(&old).unwrap();
        assert_eq!((d.data, d.data_count_bit, d.serial), (old.key, 62, Some(0x0BEEF)));
        assert!(d.crc_valid);
        assert_eq!(levels(&dec.encode(&d, 0).unwrap()), levels(&with_repeats(&upload(old.key), 4)));
        let mut short = old.clone();
        short.bits = 61;
        assert!(dec.from_flipper_key(&short).is_none());
    }

    fn hex(v: u64) -> String {
        v.to_be_bytes().iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
    }

    /// The upload through KAT's whole registry: Honeywell Sec, every copy after the first (whose
    /// first half-bits the decoder needs to sync).
    #[test]
    fn upload_decodes_back_through_the_registry() {
        for data in [frame(0x8, 0x12345, 0x80), frame(0xA, 0x00F0F, 0x08)] {
            let tx = HoneywellDecoder::new().encode(&signal(data, 64), 0).unwrap();
            let mut reg = super::super::ProtocolRegistry::new();
            let hits = reg.process_signal_stream(&tx, 345_000_000);
            assert!(hits.len() >= 3, "{hits:?}");
            assert!(hits.iter().all(|(n, x, _)| n == NAME && x.data == data), "{hits:?}");
        }
    }

    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0x5EC0_0001);
        for _ in 0..50 {
            let ch = [0x2u8, 0x4, 0x8, 0xA][rng.below(4) as usize];
            let data = frame(ch, rng.next() as u32, rng.next() as u8);
            let wire = jitter(&on_air(data, 4), &mut rng, 30);
            let got = feed_all(&mut HoneywellDecoder::new(), &wire);
            assert!(!got.is_empty(), "{data:016X}");
            assert!(got.iter().all(|d| d.data == data));
        }
    }
}
