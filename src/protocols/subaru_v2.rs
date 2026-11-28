//! Subaru V2 protocol decoder (decode-only): the Australian/European Subaru remote-head key of the
//! 2010s (Liberty/Legacy, Impreza/WRX/STI and relatives), 433.92 MHz. Not in ProtoPirate,
//! Flipper-ARF or doctor-firmware, and distinct from both [super::subaru] (the 800/1600µs PWM
//! remote) and [super::subaru_v1] (the US 315 MHz 122µs plaintext-counter key). Worked out from a
//! tester's recordings of several keys (`docs/subaru_v2.md`).
//!
//! # Air format
//!
//! OOK, **differential Manchester** with a 700µs half-bit (1400µs bit): a long preamble of
//! alternating 700µs pulses, then the frame repeats several times while the button is held. The
//! wire bit is recovered as a plain Manchester bit (`HL` = 1, `LH` = 0 in half-bit order) and then
//! differentially decoded (`data[i] = plain[i] XOR plain[i-1]`), which is where the key ID sits
//! still. Each repeat is **128 data bits = 16 bytes**, delimited by an in-band sync run of ≥8
//! differential zeros that recurs once per 128-bit period.
//!
//! # Frame (16 bytes b0..b15, differential domain, rotated so the sync run leads)
//!
//! * `b0`: header/sync byte (the run of zeros plus a fixed nibble; constant across keys).
//! * `b1..b9`: the **rolling payload** — eight bytes that change every press with no plaintext
//!   counter visible. This is an encrypted/hopping rolling code; KAT does not interpret it.
//! * `b9..b13`: the **32-bit key ID** (`serial`) — constant across every press of one key.
//! * `b13..b16`: a varying trailer (part of the rolling code or a MAC over it).
//!
//! KAT reports the ID and the raw 16-byte frame; `button` and `counter` are not recovered (they are
//! inside the encrypted payload). **Decode-only**: there is no plaintext counter to advance and no
//! encoder, so KAT identifies and replays these but does not synthesise new codes. `crc_valid` is
//! false (`NO_CHECKSUM`): emission is gated on the framing (preamble, the differential-Manchester
//! clean 128-bit period and the twin sync runs 128 bits apart), which is specific enough that the
//! Subaru PWM frames, Subaru V1 and every other decoder are unaffected.

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Subaru V2",
    decoder: "Subaru V2",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code (encrypted)",
    make: "Subaru",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const HALF_US: u32 = 700;
/// A pulse is one half-bit up to this, two up to [DOUBLE_MAX]; outside is a burst boundary.
const HALF_MIN: u32 = 400;
const HALF_MAX: u32 = 1050;
const DOUBLE_MAX: u32 = 1750;
/// Frame length in differential data bits (16 bytes).
const FRAME_BITS: usize = 128;
const FRAME_BYTES: usize = FRAME_BITS / 8;
/// The in-band sync: a run of [SYNC_ZEROS]..=[SYNC_MAX] differential zeros that recurs once per
/// 128-bit period (`b0`'s zeros). The upper bound keeps the long preamble and the silence tail,
/// which are far longer, from being taken for the sync.
const SYNC_ZEROS: usize = 8;
const SYNC_MAX: usize = 24;
/// Byte offset of the 32-bit key ID within the frame (anchored at the sync run's end, so the sync's
/// 8-vs-9-zero length does not shift it).
const ID_OFFSET: usize = 8;
/// A burst must carry at least one full period plus a sync before it is worth decoding.
const MIN_HALVES: usize = 2 * (FRAME_BITS + SYNC_ZEROS);

/// Subaru V2 decoder (decode-only): accumulates half-bit levels for a burst and decodes it when a
/// boundary (gap or off-timing pulse) arrives.
pub struct SubaruV2Decoder {
    halves: Vec<bool>,
}

impl SubaruV2Decoder {
    pub fn new() -> Self {
        Self { halves: Vec::new() }
    }

    /// Plain Manchester bits of the half-bit buffer at `phase` (0 or 1): `HL` = 1, `LH` = 0; None
    /// on the first invalid pair (a clean burst has one valid phase).
    fn manchester(&self, phase: usize) -> Option<Vec<bool>> {
        let h = &self.halves;
        if h.len() < phase + 2 {
            return None;
        }
        let mut bits = Vec::with_capacity((h.len() - phase) / 2);
        let mut i = phase;
        while i + 1 < h.len() {
            match (h[i], h[i + 1]) {
                (true, false) => bits.push(true),
                (false, true) => bits.push(false),
                _ => break, // transition lost: the burst ends here
            }
            i += 2;
        }
        (bits.len() >= FRAME_BITS + SYNC_ZEROS).then_some(bits)
    }

    /// The differential of a plain Manchester bit vector: `d[i] = plain[i] XOR plain[i-1]`.
    fn differential(plain: &[bool]) -> Vec<bool> {
        (1..plain.len()).map(|i| plain[i] != plain[i - 1]).collect()
    }

    /// Maximal zero runs whose length is [SYNC_ZEROS]..=[SYNC_MAX] in `d`, as `(start, end)`.
    fn sync_runs(d: &[bool]) -> Vec<(usize, usize)> {
        let mut out = Vec::new();
        let mut i = 0;
        while i < d.len() {
            if !d[i] {
                let start = i;
                while i < d.len() && !d[i] {
                    i += 1;
                }
                if (SYNC_ZEROS..=SYNC_MAX).contains(&(i - start)) {
                    out.push((start, i));
                }
            } else {
                i += 1;
            }
        }
        out
    }

    /// The 16-byte frame of the burst, anchored at the **end** of a sync run that recurs exactly one
    /// period on (so the payload leads and the sync's 8-vs-9-zero length does not shift the fields).
    /// A silence tail or preamble is too long to be a sync and has no partner 128 bits away.
    fn frame_bytes(&self) -> Option<[u8; FRAME_BYTES]> {
        let plain = (0..2).find_map(|p| self.manchester(p))?;
        let d = Self::differential(&plain);
        let runs = Self::sync_runs(&d);
        let starts: std::collections::BTreeSet<usize> = runs.iter().map(|&(s, _)| s).collect();
        let &(_, end) = runs.iter().find(|&&(s, e)| e + FRAME_BITS <= d.len() && starts.contains(&(s + FRAME_BITS)))?;
        let mut bytes = [0u8; FRAME_BYTES];
        for (k, bit) in d[end..end + FRAME_BITS].iter().enumerate() {
            if *bit {
                bytes[k / 8] |= 1 << (7 - k % 8);
            }
        }
        Some(bytes)
    }

    /// A decoded frame: the 32-bit key ID and the opaque 16-byte payload.
    fn frame(bytes: [u8; FRAME_BYTES]) -> DecodedSignal {
        let serial = u32::from_be_bytes([bytes[ID_OFFSET], bytes[ID_OFFSET + 1], bytes[ID_OFFSET + 2], bytes[ID_OFFSET + 3]]);
        let data = bytes[..8].iter().fold(0u64, |a, &b| (a << 8) | u64::from(b));
        let extra = bytes[8..].iter().fold(0u64, |a, &b| (a << 8) | u64::from(b));
        DecodedSignal {
            serial: Some(serial),
            button: None,  // inside the encrypted payload
            counter: None, // inside the encrypted payload
            crc_valid: false,
            data,
            data_count_bit: FRAME_BITS,
            encoder_capable: false,
            extra: Some(extra),
            protocol_display_name: None,
        }
    }

    /// Decode the accumulated burst, clearing it.
    fn flush_burst(&mut self) -> Option<DecodedSignal> {
        let out = (self.halves.len() >= MIN_HALVES)
            .then(|| self.frame_bytes())
            .flatten()
            .map(Self::frame);
        self.halves.clear();
        out
    }
}

impl Default for SubaruV2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for SubaruV2Decoder {
    fn name(&self) -> &'static str {
        "Subaru V2"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: HALF_US, te_long: 2 * HALF_US, te_delta: 300, min_count_bit: FRAME_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.halves.clear();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if (HALF_MIN..HALF_MAX).contains(&duration) {
            self.halves.push(level);
            None
        } else if (HALF_MAX..DOUBLE_MAX).contains(&duration) {
            self.halves.push(level);
            self.halves.push(level);
            None
        } else {
            // A gap or off-timing pulse ends the burst.
            self.flush_burst()
        }
    }

    fn supports_encoding(&self) -> bool {
        false // encrypted rolling code: KAT reads and replays, but does not synthesise new codes
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a burst as the key sends it: a preamble of `pre` alternating 700µs pulses, then
    /// `copies` repeats of the 16-byte frame, each as differential-Manchester half-bits, ended by a
    /// gap. `bytes` is the differential-domain frame (a sync run of zeros must lead it).
    fn burst(bytes: &[u8; FRAME_BYTES], pre: usize, copies: usize) -> Vec<LevelDuration> {
        // differential frame bits -> plain Manchester bits (running XOR) -> half-bit levels.
        let dbits: Vec<bool> = (0..FRAME_BITS).map(|k| bytes[k / 8] >> (7 - k % 8) & 1 == 1).collect();
        let mut up = Vec::new();
        for _ in 0..pre {
            up.push(LevelDuration::new(true, HALF_US));
            up.push(LevelDuration::new(false, HALF_US));
        }
        let mut plain = true; // preamble ends HIGH; the first differential bit toggles from here
        for _ in 0..copies {
            for &db in &dbits {
                if db {
                    plain = !plain;
                }
                // Manchester: 1 -> HL, 0 -> LH (of the plain bit).
                up.push(LevelDuration::new(plain, HALF_US));
                up.push(LevelDuration::new(!plain, HALF_US));
            }
        }
        up.push(LevelDuration::new(false, 20_000)); // inter-press gap
        up
    }

    fn decode(up: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut d = SubaruV2Decoder::new();
        up.iter().filter_map(|p| d.feed(p.level, p.duration_us)).collect()
    }

    /// A frame (differential domain, run-end anchored): 8 payload bytes b0..b8 (b0's MSB set so the
    /// preceding sync run is exactly 8 zeros), the 32-bit ID at b8..b12, a 3-byte trailer, and
    /// b15 = 0x00 (the next frame's sync run).
    fn frame_bytes(id: u32, payload: [u8; 8], trailer: [u8; 3]) -> [u8; FRAME_BYTES] {
        let i = id.to_be_bytes();
        [payload[0] | 0x80, payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7],
         i[0], i[1], i[2], i[3], trailer[0], trailer[1], trailer[2], 0x00]
    }

    // High-entropy rolling payloads, as an encrypted code sends (no long zero runs that would form
    // a false sync — real captures behave this way).
    const PAY_A: [u8; 8] = [0x5A, 0xC3, 0x3C, 0xA5, 0x96, 0x69, 0xB4, 0x4B];
    const PAY_B: [u8; 8] = [0xD2, 0x71, 0x8E, 0x5B, 0x6D, 0xA6, 0x39, 0xC7];

    #[test]
    fn decodes_the_key_id_from_a_press() {
        let bytes = frame_bytes(0x1234_5678, PAY_A, [0xAB, 0xCD, 0xEF]);
        let got = decode(&burst(&bytes, 40, 5));
        assert_eq!(got.len(), 1, "one frame per burst: {got:?}");
        let d = &got[0];
        assert_eq!(d.serial, Some(0x1234_5678));
        assert_eq!((d.button, d.counter, d.crc_valid, d.encoder_capable), (None, None, false, false));
        assert_eq!(d.data_count_bit, FRAME_BITS);
    }

    #[test]
    fn the_id_is_stable_across_presses_but_the_payload_rolls() {
        let id = 0x0A4B_2C7D;
        let a = decode(&burst(&frame_bytes(id, PAY_A, [0x3A, 0x5C, 0x96]), 60, 5));
        let b = decode(&burst(&frame_bytes(id, PAY_B, [0xC5, 0x69, 0xA3]), 60, 5));
        assert_eq!(a[0].serial, b[0].serial, "same key -> same ID");
        assert_ne!(a[0].data, b[0].data, "rolling payload differs");
    }

    #[test]
    fn noise_and_short_bursts_do_not_decode() {
        // A short Manchester burst (no full frame) is ignored.
        let mut up = Vec::new();
        for i in 0..60 {
            up.push(LevelDuration::new(i % 2 == 0, HALF_US));
        }
        up.push(LevelDuration::new(false, 20_000));
        assert!(decode(&up).is_empty());
        // Irregular pulse widths never form half-bits.
        let noise: Vec<LevelDuration> = (0..400).map(|i| LevelDuration::new(i % 2 == 0, 90 + (i * 37 % 300) as u32)).collect();
        assert!(decode(&noise).is_empty());
    }

    /// The tester's recordings (`REFERENCES/testers/2026-11-27_subaru_au`, git-ignored): every
    /// recording decodes, and all presses of one key report the same ID. Masked: no captured ID,
    /// payload or counter is printed.
    #[test]
    #[ignore = "reads local tester recordings kept out of git"]
    fn tester_recordings() {
        use std::collections::BTreeSet;
        let dir = std::path::Path::new("REFERENCES/testers/2026-11-27_subaru_au");
        if !dir.is_dir() {
            eprintln!("Skip: tester recordings not present");
            return;
        }
        for name in ["subaru_gt_sti", "subaru_liberty_gt", "subaru_geetee"] {
            let path = dir.join(format!("{name}.sub"));
            let text = std::fs::read_to_string(&path).unwrap();
            let pairs: Vec<LevelDuration> = text
                .lines()
                .filter(|l| l.starts_with("RAW_Data:"))
                .flat_map(|l| l[9..].split_whitespace().map(|v| v.parse::<i64>().unwrap()).collect::<Vec<_>>())
                .map(|v| LevelDuration::new(v > 0, v.unsigned_abs() as u32))
                .collect();
            let mut d = SubaruV2Decoder::new();
            let ids: BTreeSet<Option<u32>> = pairs.iter().filter_map(|p| d.feed(p.level, p.duration_us)).map(|s| s.serial).collect();
            eprintln!("{name}: {} distinct ID(s) across its presses", ids.len());
            assert_eq!(ids.len(), 1, "{name}: every press of one key should carry the same ID");
        }
    }
}
