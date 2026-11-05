//! Opel Mokka car key decoder (receive only).
//!
//! Port of rtl_433 `src/devices/opel_mokka.c` (`opel_mokka_decode`, r_device "Opel Mokka Car Key",
//! model `Opel-Mokka`). rtl_433: presumably a HITAG AES (4A, NCF29A1M) transponder key, so other
//! brands using it may decode too; it reads the key ID and the event type, the payload is
//! encrypted. Not KAT's `Opel V1` (Flipper-ARF, AM 120/240 µs, 93 bits).
//!
//! **Modulation** — FSK, rtl_433's Manchester zero-bit slicer (`FSK_PULSE_MANCHESTER_ZEROBIT`,
//! short 100 µs, reset limit 1000 µs, no tolerance; its `long_width` is also 100 and unused), see
//! [super::rtl433].
//!
//! **Frame** — the message must be exactly 268 bits: the slicer's hard-coded `0`, 88 preamble
//! zeros, then the packet twice with 15 zeros between (`S iiiiiiiiiii ttttt c…c E`):
//!
//! | row bits | field |
//! |----------|-------|
//! | 0..=88   | zeros (the C checks bytes 0..10) |
//! | 89       | start bit 1 |
//! | 90..=100 | key ID (11 bits) |
//! | 101..=105 | event type (5 bits): 26 for both lock and unlock, 3 for a zero-filled packet the fob sends periodically (proximity?) |
//! | 106..=169 | encrypted payload (64 bits) |
//! | 170      | end bit 1 |
//! | 171..=185 | 15 zeros |
//! | 186..=267 | the packet again (its key ID at 187..=197) |
//!
//! rtl_433's `code` is the 64 bits from row bit 107 (`90 + 17`), i.e. payload bits 1..63 and the end
//! bit: one bit after the payload (11 + 5 = 16 bits follow bit 90). KAT reports the same bits.
//!
//! **Gating** (the C's checks; a row failing one is skipped): exactly 268 bits; bytes 0..10 zero;
//! both key IDs equal; key ID ≠ 0. There is no checksum (rtl_433 prints no `mic`).
//!
//! **Field mapping** — `serial` = key ID (rtl_433 `id`), `button` = event type (`event`), no
//! counter, `crc_valid` = false (no integrity check beyond the repeated ID). `data`/`extra` = the
//! first packet from its start bit (row bits 89..=170, 82 bits: the first 64 in `data`, the last 18
//! right-aligned in `extra`), `data_count_bit` = 82; rtl_433's `code` = `(data << 18 | extra)` (its low
//! 64 bits).
//!
//! The C reports every 268-bit row of a bitbuffer; this slicer (no tolerance) only ever makes one
//! row per message, so KAT's one decode per message is the same.
//!
//! Receive only (rtl_433 has no encoder). No recording exists (rtl_433_tests has none): the tests
//! use frames built from the C and its documented example packet. The exact 268-bit length makes
//! the decode depend on how the package starts: rtl_433 counts a lower-tone lead-in longer than
//! 150 µs as one more `0` (its FSK package then starts with `pulse[0] = 0`), which KAT's FM path
//! merges into the idle LOW (see `rtl433`); unverified on a real fob.

use super::rtl433::{frame_signal, BitBuffer, Decode, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "Opel-Mokka";

/// Metadata (see `info.rs`): FSK of unknown deviation (no recording) → FM476, receive only.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Opel Mokka Car Key"],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Opel",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&FM476],
};

/// Event types as rtl_433's doc comment describes them (it prints the number).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        26 => "Lock/Unlock",
        3 => "Proximity",
        _ => "Other",
    })
}

/// r_device timing: `short_width` (half-bit) and `reset_limit` (µs).
const SHORT_US: u32 = 100;
const RESET_US: u32 = 1000;
const ROW_BITS: usize = 268;
/// KAT's frame: the first packet, start bit to end bit.
const FRAME_START: usize = 89;
const FRAME_BITS: usize = 82;

/// The `r_device`; a burst's FM pairs are one package, consecutive LOWs one gap (see `rtl433`).
const DEVICE: Device = Device {
    modulation: Modulation::FskManchesterZerobit,
    short_width: SHORT_US,
    long_width: SHORT_US,
    reset_limit: RESET_US,
    gap_limit: 0,
    sync_width: 0,
    tolerance: 0,
    gaps: Gaps::Merged,
    ends: Ends::None,
    resync: false,
};

/// `opel_mokka_decode` on one message's bitbuffer (the first row that passes).
fn decode(bits: &BitBuffer) -> Option<DecodedSignal> {
    (0..bits.num_rows()).find_map(|i| decode_row(bits, i))
}

fn decode_row(bits: &BitBuffer, i: usize) -> Option<DecodedSignal> {
    if bits.bits_per_row(i) != ROW_BITS {
        return None; // invalid length
    }
    let b = bits.row(i);
    if b[..11].iter().any(|&x| x != 0) {
        return None;
    }
    let id_at = |pos: usize| {
        let mut t = [0u8; 2];
        bits.extract_bytes(i, pos, &mut t, 11);
        u16::from(t[0]) << 3 | u16::from(t[1] >> 5)
    };
    let key_id = id_at(90);
    if key_id != id_at(90 + 12 * 8 + 1) || key_id == 0 {
        return None;
    }
    let event = (b[12] & 0x07) << 2 | (b[13] & 0xC0) >> 6;
    let mut f = [0u8; 11];
    bits.extract_bytes(i, FRAME_START, &mut f, FRAME_BITS);
    let mut d = frame_signal(&f, FRAME_BITS);
    d.serial = Some(u32::from(key_id));
    d.button = Some(event);
    Some(d)
}

/// Opel Mokka decoder: rtl_433's zero-bit slicer + `opel_mokka_decode`.
pub struct OpelMokkaDecoder {
    rx: Receiver,
}

impl OpelMokkaDecoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE) }
    }
}

impl Default for OpelMokkaDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for OpelMokkaDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    /// Manchester half-bit / full bit; the slicer splits them at 1.5 half-bits (± 50 µs).
    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: SHORT_US,
            te_long: 2 * SHORT_US,
            te_delta: SHORT_US / 2,
            min_count_bit: FRAME_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.rx.reset(&mut Decode(decode));
    }

    fn feed(&mut self, level: bool, duration_us: u32) -> Option<DecodedSignal> {
        self.rx.feed(&mut Decode(decode), level, duration_us)
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    /// rtl_433 decodes only.
    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

/// rtl_433's JSON fields of a decode, rebuilt from KAT's (the field mapping, for the parity tests).
#[cfg(test)]
pub(super) fn rtl433_fields(d: &DecodedSignal) -> String {
    let code = d.data << 18 | d.extra.unwrap_or(0);
    format!(
        "id={} event={} code={code:016x}",
        d.serial.unwrap_or(0),
        d.button.unwrap_or(0)
    )
}

#[cfg(test)]
mod tests {
    use super::super::rtl433::test_util::*;
    use super::*;

    /// One packet: start bit, key ID, event, payload, end bit.
    fn packet(id: u16, event: u8, payload: u64) -> Vec<bool> {
        let mut bits = vec![true];
        bits.extend((0..11).rev().map(|i| id >> i & 1 != 0));
        bits.extend((0..5).rev().map(|i| event >> i & 1 != 0));
        bits.extend((0..64).rev().map(|i| payload >> i & 1 != 0));
        bits.push(true);
        bits
    }

    /// The 268-bit message: hard-coded 0, 88 zeros, the packet, 15 zeros, the packet again.
    fn message(first: &[bool], second: &[bool]) -> Vec<bool> {
        let mut bits = vec![false; 89];
        bits.extend(first);
        bits.extend([false; 15]);
        bits.extend(second);
        bits
    }

    fn air(bits: &[bool]) -> Vec<LevelDuration> {
        with_idle(&wire(&manchester_zerobit(bits, SHORT_US, 0)), 30_000)
    }

    /// rtl_433's documented frame (opel_mokka.c): key ID 1074, event 26.
    const DOC_PACKET: &str =
        "1 10000110010 11010 0000010100011111101001100000010100101011001000011100100101010110 1";

    fn doc_packet() -> Vec<bool> {
        DOC_PACKET
            .chars()
            .filter(|c| !c.is_whitespace())
            .map(|c| c == '1')
            .collect()
    }

    #[test]
    fn decodes_the_documented_frame() {
        let p = doc_packet();
        assert_eq!(p.len(), 82);
        let got = decode_all(&mut OpelMokkaDecoder::new(), &air(&message(&p, &p)));
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!(
            (d.serial, d.button, d.counter, d.crc_valid),
            (Some(1074), Some(26), None, false)
        );
        assert_eq!((d.data_count_bit, d.encoder_capable), (82, false));
        // `code` = the 64 bits after payload bit 0: payload bits 1..63 + the end bit.
        let payload = u64::from_str_radix(DOC_PACKET.split(' ').nth(3).unwrap(), 2).unwrap();
        assert_eq!(
            rtl433_fields(d),
            format!("id=1074 event=26 code={:016x}", payload << 1 | 1)
        );
    }

    #[test]
    fn decodes_frames_from_the_c_layout() {
        for (id, event, payload) in [
            (1, 26, 0xDEAD_BEEF_0123_4567u64),
            (0x7FF, 3, 0),
            (0x2AA, 31, u64::MAX),
        ] {
            let p = packet(id, event, payload);
            let got = decode_all(&mut OpelMokkaDecoder::new(), &air(&message(&p, &p)));
            assert_eq!(got.len(), 1, "{id} {event}");
            let d = &got[0];
            assert_eq!((d.serial, d.button), (Some(u32::from(id)), Some(event)));
            let frame: u128 = p.iter().fold(0, |acc, &b| acc << 1 | u128::from(b));
            assert_eq!(
                (d.data, d.extra),
                ((frame >> 18) as u64, Some((frame & 0x3FFFF) as u64))
            );
        }
        // Only the key IDs must match: the second packet's event and payload are not checked.
        let got = decode_all(
            &mut OpelMokkaDecoder::new(),
            &air(&message(&packet(9, 26, 1), &packet(9, 3, 2))),
        );
        assert_eq!(got.len(), 1);
    }

    #[test]
    fn rejects_bad_frames() {
        let p = packet(1074, 26, 0x0123_4567_89AB_CDEF);
        let mut dec = OpelMokkaDecoder::new();
        assert_eq!(decode_all(&mut dec, &air(&message(&p, &p))).len(), 1);
        assert!(
            decode_all(&mut dec, &air(&message(&p, &packet(1075, 26, 0)))).is_empty(),
            "key IDs differ"
        );
        let z = packet(0, 3, 0);
        assert!(
            decode_all(&mut dec, &air(&message(&z, &z))).is_empty(),
            "key ID 0"
        );
        // Exactly 268 bits: one preamble zero more or less, or a trailing bit, fails.
        let m = message(&p, &p);
        assert!(decode_all(&mut dec, &air(&m[1..])).is_empty(), "267 bits");
        let mut longer = vec![false];
        longer.extend(&m);
        assert!(decode_all(&mut dec, &air(&longer)).is_empty(), "269 bits");
        let mut trailing = m.clone();
        trailing.push(false);
        assert!(decode_all(&mut dec, &air(&trailing)).is_empty(), "trailing 0");
        // A 1 in the preamble (bytes 0..10).
        let mut one = m.clone();
        one[40] = true;
        assert!(decode_all(&mut dec, &air(&one)).is_empty(), "preamble");
        // Lower-tone stretch over 1 ms inside the message.
        let mut wire = wire(&manchester_zerobit(&m, SHORT_US, 0));
        let k = (wire.len() / 2..).find(|&i| !wire[i].level).unwrap();
        wire[k].duration_us = 1_100;
        assert!(
            decode_all(&mut dec, &with_idle(&wire, 30_000)).is_empty(),
            "split"
        );
    }

    /// rtl_433_tests has no Opel Mokka recording; on the FSK car-remote recordings it has (Nidec,
    /// Honda) rtl_433 reports no Opel-Mokka frame, and neither does KAT.
    #[test]
    #[ignore = "IQ recordings, ~5 s in a debug build"]
    fn rtl433_recordings_as_rtl433() {
        let json = |v: &serde_json::Value| {
            format!(
                "id={} event={} code={}",
                v["id"],
                v["event"],
                v["code"].as_str().unwrap()
            )
        };
        if let Some(n) =
            check_recordings(&["nidec", "honda_remote"], NAME, NAME, rtl433_fields, json)
        {
            assert_eq!(n, 0);
        }
    }

    #[test]
    fn registry_reports_it_on_fm() {
        let p = doc_packet();
        let got = registry(
            &air(&message(&p, &p)),
            433_920_000,
            Some(crate::capture::RfModulation::FM),
        );
        assert_eq!(
            got.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>(),
            [NAME]
        );
    }
}
