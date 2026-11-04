//! Telcoma / Cardin EDGE (EDGE2 / EDGE4 / EDGE20) 433.92 MHz Manchester fixed-code remote.
//!
//! Primary reference: Flipper-ARF's garage app copy
//! `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/telcoma_edge.c`; the
//! doctor-firmware and Flipper-ARF `lib/subghz/protocols/telcoma_edge.c` copies decode and
//! transmit identically. Not Cardin's S449 (FSK KeeLoq) or S466 (27 MHz PWM) products.
//!
//! **Line coding**: Manchester with a 1 270 µs half bit (te_short) and 2 540 µs full bit
//! (te_long), te_delta = 520 µs (750–1 790 µs short, 2 020–3 060 µs long). 1 = HIGH then LOW,
//! 0 = LOW then HIGH. On air: 0xFF, a 21-bit serial and a one-hot channel marker. The gate
//! channel is 32 bits plus a trailing HIGH stop half bit; the other channels are 33 bits (the
//! marker adds a bit) without the stop half.
//!
//! **Decoder** (`subghz_protocol_decoder_telcoma_edge_feed`): not Flipper's Manchester state
//! machine. Each short pulse adds one half-bit sample of its level, each long pulse two; samples
//! are paired: high→low = 1, low→high = 0, a same-level pair is dropped. Any other pulse (the
//! ~5 ms inter-frame gap, or noise of either level) ends the burst: with 32 bits the key is the
//! bits, with 33 bits it is the bits >> 1, and it is reported when its top byte is 0xFF. The
//! bit buffer and the pairing restart after every such pulse. The bit count is the C's `uint8_t`.
//!
//! **Fields** (`subghz_protocol_decoder_telcoma_edge_get_string`, doctor/garage): serial = key
//! bits 3..23 (21 bits), button = the channel, key bits 0..2 (0 = gate, else one-hot 1/2/4).
//!
//! **Encoder** (`subghz_protocol_encoder_telcoma_edge_get_upload`): channel 0 → the 32 key bits
//! plus a HIGH stop half; other channels → (key << 1) as 33 bits, no stop; half bits merged
//! into runs, then a LOW guard of 4·te_short (5 080 µs); sent 6 times. Static: the references
//! have no button switch, so KAT replays the captured channel whatever `button` asks for.
//!
//! **Key file**: `Protocol: Telcoma/Cardin EDGE`, `Bit: 32`, `Key`.

use super::g3_common::{duration_diff, repeat_upload, UploadBuilder};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Telcoma/Cardin EDGE",
    decoder: "Telcoma/Cardin EDGE",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Telcoma/Cardin",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The channel field (key bits 0..2).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0 => "Gate",
        1 => "Ch 0x1",
        2 => "Ch 0x2",
        4 => "Ch 0x4",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 1270;
const TE_LONG: u32 = 2540;
const TE_DELTA: u32 = 520;
const BIT_COUNT: usize = 32;
/// `subghz_protocol_encoder_telcoma_edge_alloc`: repeat = 6.
const REPEAT: usize = 6;

/// Telcoma/Cardin EDGE decoder (`SubGhzProtocolDecoderTelcomaEdge`).
pub struct TelcomaEdgeDecoder {
    decode_data: u64,
    decode_count_bit: u8,
    half_pending: bool,
    half_level: bool,
}

fn signal(key: u32) -> DecodedSignal {
    DecodedSignal {
        serial: Some((key >> 3) & 0x1F_FFFF),
        button: Some((key & 0x07) as u8),
        counter: None,
        crc_valid: false,
        data: u64::from(key),
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One upload (`get_upload`).
fn upload(key: u32) -> Vec<LevelDuration> {
    let (value, nbits, add_stop) = if key & 0x07 == 0 {
        (u64::from(key), 32, true)
    } else {
        (u64::from(key) << 1, 33, false)
    };
    let mut b = UploadBuilder::new();
    for i in (0..nbits).rev() {
        let bit = (value >> i) & 1 == 1;
        b.push(bit, TE_SHORT);
        b.push(!bit, TE_SHORT);
    }
    if add_stop {
        b.push(true, TE_SHORT);
    }
    b.push(false, TE_SHORT * 4);
    b.finish()
}

impl TelcomaEdgeDecoder {
    pub fn new() -> Self {
        Self {
            decode_data: 0,
            decode_count_bit: 0,
            half_pending: false,
            half_level: false,
        }
    }

    /// `subghz_protocol_decoder_telcoma_edge_add_half`.
    fn add_half(&mut self, level: bool) {
        if !self.half_pending {
            self.half_pending = true;
            self.half_level = level;
            return;
        }
        self.half_pending = false;
        if self.half_level && !level {
            self.add_bit(1);
        } else if !self.half_level && level {
            self.add_bit(0);
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = (self.decode_data << 1) | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for TelcomaEdgeDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for TelcomaEdgeDecoder {
    fn name(&self) -> &'static str {
        "Telcoma/Cardin EDGE"
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

    fn reset(&mut self) {
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.half_pending = false;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if duration_diff(duration, TE_SHORT) < TE_DELTA {
            self.add_half(level);
            return None;
        }
        if duration_diff(duration, TE_LONG) < TE_DELTA {
            self.add_half(level);
            self.add_half(level);
            return None;
        }
        // End of a burst.
        let count = self.decode_count_bit;
        let mut found = None;
        if count == 32 || count == 33 {
            let key = if count == 33 {
                (self.decode_data >> 1) as u32
            } else {
                self.decode_data as u32
            };
            if key >> 24 == 0xFF {
                found = Some(signal(key));
            }
        }
        self.reset();
        found
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        (decoded.data_count_bit == BIT_COUNT)
            .then(|| repeat_upload(&upload(decoded.data as u32), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        // The decoder side keeps the full 64-bit Key; only the low 32 bits are the frame.
        (key.bits == BIT_COUNT).then(|| DecodedSignal {
            data: key.key,
            ..signal(key.key as u32)
        })
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == BIT_COUNT).then(|| {
            FlipperKey::new(
                "Telcoma/Cardin EDGE",
                BIT_COUNT,
                decoded.data,
                433_920_000,
                "FuriHalSubGhzPresetOok650Async",
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::super::g3_common::test_support::*;
    use super::*;

    /// telcoma_edge_raw.sub (Unleashed unit test, gate channel): its first two frames.
    const GATE_RAW: &[i32] = &[
        1293, -1244, 1299, -1212, 1305, -1240, 1307, -1244, 1285, -1230, 1303, -1250, 1299, -1246,
        1283, -2498, 1313, -1254, 2539, -1242, 1295, -2504, 1303, -1252, 1275, -1250, 1305, -1236,
        2561, -2510, 1295, -1246, 2547, -1228, 1301, -1252, 1303, -1246, 1293, -1234, 1297, -1250,
        1273, -1250, 1283, -2522, 1281, -1260, 1275, -1250, 1303, -1244, 1283, -1240, 1309, -1244,
        2565, -5046, 1275, -1232, 1305, -1244, 1285, -1268, 1277, -1240, 1299, -1248, 1285, -1268,
        1271, -1244, 1291, -2506, 1305, -1242, 2581, -1236, 1309, -2500, 1275, -1228, 1297, -1270,
        1283, -1250, 2543, -2528, 1283, -1230, 2593, -1224, 1313, -1218, 1299, -1240, 1297, -1228,
        1301, -1250, 1299, -1242, 1293, -2510, 1305, -1210, 1321, -1218, 1325, -1210, 1311, -1220,
        1313, -1228, 2595, -5028,
    ];

    /// telcoma_edge_ch_raw.sub (Unleashed unit test, another channel): its second frame.
    const CHANNEL_RAW: &[i32] = &[
        -5062, 1301, -1248, 1299, -1246, 1261, -1264, 1293, -1246, 1299, -1246, 1279, -1240, 1307,
        -1242, 1279, -2518, 1285, -1282, 2529, -1246, 1289, -2534, 1283, -1240, 1307, -1244, 1281,
        -1262, 2557, -2502, 1317, -1236, 2563, -1232, 1301, -1236, 1305, -1240, 1299, -1254, 1273,
        -1260, 1269, -1272, 1263, -2514, 1319, -1248, 1275, -1250, 1281, -1268, 1275, -1244, 2575,
        -2520, 1301, -5012,
    ];

    #[test]
    fn decodes_the_unleashed_gate_capture() {
        let frames = feed_all(&mut TelcomaEdgeDecoder::new(), &pulses(GATE_RAW));
        assert_eq!(frames.len(), 2);
        for f in &frames {
            assert_eq!((f.data, f.data_count_bit), (0xFF30_9FC0, 32));
            assert_eq!(
                (f.serial, f.button),
                (Some((0x30_9FC0 >> 3) & 0x1F_FFFF), Some(0))
            );
        }
    }

    #[test]
    fn decodes_the_unleashed_channel_capture() {
        let frames = feed_all(&mut TelcomaEdgeDecoder::new(), &pulses(CHANNEL_RAW));
        assert_eq!(frames.len(), 1);
        let f = &frames[0];
        assert_eq!(f.data >> 24, 0xFF);
        assert_ne!(f.button, Some(0), "a non-gate channel");
        assert_eq!(
            f.serial,
            Some(((0xFF30_9FC0u64 >> 3) & 0x1F_FFFF) as u32),
            "same remote"
        );
    }

    #[test]
    fn encoder_reproduces_the_captured_frames() {
        // Gate: the encoder's frame has the capture's pulse pattern (in te units).
        let quantize = |p: &[LevelDuration]| -> Vec<(bool, u32)> {
            p.iter()
                .map(|x| (x.level, (x.duration_us + TE_SHORT / 2) / TE_SHORT))
                .collect()
        };
        let gate = upload(0xFF30_9FC0);
        let captured = pulses(&GATE_RAW[..58]);
        assert_eq!(quantize(&gate)[..57], quantize(&captured)[..57]);
        assert_eq!((gate[57].level, gate[57].duration_us), (false, 5080));
        let ch = feed_all(&mut TelcomaEdgeDecoder::new(), &pulses(CHANNEL_RAW))[0].clone();
        let up = upload(ch.data as u32);
        assert_eq!(
            quantize(&up)[..up.len() - 1],
            quantize(&pulses(&CHANNEL_RAW[1..]))[..up.len() - 1]
        );
    }

    #[test]
    fn encode_round_trip_every_channel() {
        let d = TelcomaEdgeDecoder::new();
        for key in [
            0xFF30_9FC0u32,
            0xFF30_9FC1,
            0xFF30_9FC2,
            0xFF30_9FC4,
            0xFF00_0000,
            0xFFFF_FFF8,
        ] {
            let tx = d.encode(&signal(key), 0x4).unwrap();
            let frames = feed_all(&mut TelcomaEdgeDecoder::new(), &tx);
            assert_eq!(frames.len(), 6, "{key:08X}");
            assert!(frames.iter().all(|f| f.data == u64::from(key)), "{key:08X}");
        }
    }

    #[test]
    fn rejects_noise_and_other_prefixes() {
        let mut p = upload(0xFE30_9FC0);
        assert!(
            feed_all(&mut TelcomaEdgeDecoder::new(), &p).is_empty(),
            "top byte not 0xFF"
        );
        p = upload(0xFF30_9FC0);
        p.insert(10, LevelDuration::new(true, 300)); // a glitch splits the burst
        assert!(feed_all(&mut TelcomaEdgeDecoder::new(), &p).is_empty());
    }

    #[test]
    fn key_file_round_trip() {
        let d = TelcomaEdgeDecoder::new();
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
            Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Telcoma/Cardin EDGE\nBit: 32\n\
            Key: 00 00 00 00 FF 30 9F C0\n";
        let s = d
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!((s.data, s.button), (0xFF30_9FC0, Some(0)));
        let back_text = d.to_flipper_key(&s).unwrap().to_text();
        assert!(back_text
            .contains("Protocol: Telcoma/Cardin EDGE\nBit: 32\nKey: 00 00 00 00 FF 30 9F C0\n"));
        let back = d
            .from_flipper_key(&FlipperKey::parse(&back_text).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{s:?}"));
        let mut wrong = FlipperKey::parse(text).unwrap();
        wrong.bits = 33;
        assert!(d.from_flipper_key(&wrong).is_none());
    }
}
