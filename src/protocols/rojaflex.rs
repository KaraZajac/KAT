//! RojaFlex shutters, remotes and bridge (receive only).
//!
//! Port of rtl_433 `src/devices/rojaflex.c` (`REFERENCES/rtl_433`, GPL-2.0; r_device "RojaFlex
//! shutter and remote devices", `-R 187`; models "RojaFlex-Remote", "-Shutter", "-Bridge",
//! "-Other"). No recordings exist (rtl_433_tests has none); validated on synthetic frames built
//! from the C's layout, through rtl_433 itself (see `docs/rojaflex.md`).
//!
//! **Air format**: FSK_PULSE_PCM, 100 µs per bit (433.92 MHz), `reset_limit` 102.4 ms: each FSK
//! package is one row of bits (upper tone = 1). On KAT's FM path a LOW over 32 bit periods
//! (3.2 ms) is the carrier stopping and ends the package. The frame's final zero bits merge into
//! that LOW, while rtl_433 sees the carrier stop right after them (its classic FSK detector, used
//! below 800 MHz, keeps them in the last segment): the package's last LOW is read as 0 … 32
//! trailing zero bits, and the shortest reading that passes the CRC wins, else the shortest reading
//! that makes a 72-bit frame without CRC whose token sum checks (`rtl433::FskFrame`).
//! KAT deviation, forced by that unknown tail: the C reports any CRC-less frame of 72..86 bits
//! (`& 0xFE`), KAT only the bridge's 9 bytes with a correct Token II (byte 8 = Σ bytes 1..7, as the
//! C documents it) — a shorter tail would otherwise turn an 88-bit frame that fails its CRC into an
//! "unchecked" one, and a longer one pad a cut frame up to 72 bits.
//!
//! **Frame**: `aaaa d391d391` (preamble end + sync, searched) then `SS KKKKKK ?CDDDD TTTT CCCC`:
//! size (0x08), 28-bit installation ID, channel (4 bits), command, value, 16-bit token, CRC-16/CMS
//! (poly 0x8005, init 0xFFFF). The C takes `(row bits − sync end) & 0xFE` bits as the frame (a
//! byte count: the row's length modulo 256), accepts 72..88: with exactly 88 the CRC must match
//! (else the frame is dropped); shorter frames (the bridge sends no CRC) are reported unchecked.
//! The model: command low nibble 0x5 = Shutter (a status reply), 0xA = Remote with CRC / Bridge
//! without, else Other.
//!
//! **Fields**: `serial` = rtl_433 `id` (28 bits); `button` = rtl_433 `cmd_id` (labels: rtl_433's
//! `cmd_name`); `counter`: none. rtl_433's `channel` (low nibble of frame byte 4), `cmd_value`
//! (byte 6) and `token` (bytes 7 and 8) have no field of their own: `data` = frame bytes 0..8
//! (size, ID ×3, ID/channel, command, value, token high), `extra` = the rest right-aligned (token
//! low, CRC), `data_count_bit` = 88, or 72 without CRC (72..86 only in a package the 1200-pair limit
//! cuts, which ends without a carrier-off LOW); `crc_valid` = the CRC was checked (88-bit frames). The display name is the model unless it is "RojaFlex-Remote".
//!
//! **Encoder**: none — receive only, as rtl_433 (its `GENERATE_COMMANDS` cloner is compiled out).

use super::rtl433::{crc16, frame_signal, BitBuffer, Decode, Device, Ends, FskFrame, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// rtl_433's model name for the remotes (the frames KAT is after); the other models are aliases.
pub(super) const NAME: &str = "RojaFlex-Remote";

/// Metadata (see `info.rs`): FSK NRZ, fixed code, shutter remote,
/// receive only. No deviation is documented: FM476.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["RojaFlex-Shutter", "RojaFlex-Bridge", "RojaFlex-Other", "RojaFlex shutter and remote devices"],
    name_prefix: None,
    encoding: Coding::Nrz,
    rf: Rf::FM,
    encryption: "Fixed Code",
    make: "RojaFlex",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&FM476],
};

/// rtl_433's `cmd_name` for each command ID.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x0a => "Stop",
        0x1a => "Up",
        0x8a => "Down",
        0x9a => "Save/Unsave position",
        0xda => "Go saved position",
        0xea => "Request Status",
        0x85 => "Pos. Status 0%",
        0x95 => "Pos. Status 20%",
        0xA5 => "Pos. Status 40%",
        0xB5 => "Pos. Status 60%",
        0xC5 => "Pos. Status 80%",
        0xD5 => "Pos. Status 100%",
        _ => "unknown",
    })
}

/// `r_device rojaflex`; on KAT's FM path the package ends with the carrier (see `rtl433`).
const DEVICE: Device = Device {
    modulation: Modulation::FskPcm(FRAME),
    short_width: 100,
    long_width: 100,
    reset_limit: 102_400,
    gap_limit: 0,
    sync_width: 0,
    tolerance: 0,
    gaps: Gaps::Merged,
    ends: Ends::Fsk { carrier_off: CARRIER_OFF_US },
    resync: true,
};
/// On FM input, a LOW longer than 32 bit periods is the carrier stopping.
const CARRIER_OFF_US: u32 = 32 * 100;
/// `DATAFRAME_BITCOUNT_INCL_CRC`, `MESSAGE_CRC_BITCOUNT`.
const FRAME_BITS_CRC: usize = 88;
const CRC_BITS: usize = 16;

/// The end of the preamble and the sync word, searched first.
const PREAMBLE: [u8; 6] = [0xaa, 0xaa, 0xd3, 0x91, 0xd3, 0x91];

/// The FSK reading of the package's last gap (`rtl433::FskFrame`): a frame without CRC is taken
/// only as a bridge frame whose token sum checks ([bridge_frame]); the smallest frame is the sync
/// and 72 bits.
const FRAME: FskFrame = FskFrame {
    unchecked: Some(bridge_frame),
    sync: &PREAMBLE,
    sync_bits: 48,
    min_bits: 48 + FRAME_BITS_CRC - CRC_BITS,
};

/// A CRC-less frame KAT reads on the FM path: the bridge's 9 bytes (72 bits) whose Token II (byte 8)
/// is the sum of bytes 1..7 — the frame format the C documents (and its cloner builds). The C itself
/// checks nothing here; KAT needs the sum to tell the frame's last zero bits from the carrier-off
/// LOW (see the module doc).
fn bridge_frame(d: &DecodedSignal) -> bool {
    let sum = (1..8).map(|i| (d.data >> (56 - 8 * i)) as u8).fold(0u8, u8::wrapping_add);
    d.data_count_bit == FRAME_BITS_CRC - CRC_BITS && d.extra == Some(u64::from(sum))
}

/// `rojaflex_decode`.
fn rojaflex_decode(bits: &BitBuffer) -> Option<DecodedSignal> {
    if bits.num_rows() != 1 {
        return None; // DECODE_ABORT_EARLY
    }
    let row_bits = bits.bits_per_row(0);
    let start = bits.search(0, 0, &PREAMBLE, PREAMBLE.len() * 8);
    if start >= row_bits {
        return None; // no preamble
    }
    // `uint8_t dataframe_bitcount = (bits - start - 48) & 0xFE`.
    let count = (row_bits - start - PREAMBLE.len() * 8) & 0xFE;
    if !(FRAME_BITS_CRC - CRC_BITS..=FRAME_BITS_CRC).contains(&count) {
        return None; // DECODE_ABORT_LENGTH
    }
    let mut msg = [0u8; 11];
    bits.extract_bytes(0, start + PREAMBLE.len() * 8, &mut msg, count);
    let has_crc = count == FRAME_BITS_CRC;
    if has_crc {
        let crc_message = u16::from(msg[9]) << 8 | u16::from(msg[10]);
        if crc_message != crc16(&msg[..9], 0x8005, 0xffff) {
            return None; // DECODE_FAIL_MIC
        }
    }
    let id = u32::from(msg[1]) << 20 | u32::from(msg[2]) << 12 | u32::from(msg[3]) << 4 | u32::from(msg[4]) >> 4;
    let model = match msg[5] & 0xF {
        0x5 => "RojaFlex-Shutter",
        0xa if has_crc => "RojaFlex-Remote",
        0xa => "RojaFlex-Bridge",
        _ => "RojaFlex-Other",
    };
    let mut d = frame_signal(&msg, count);
    d.serial = Some(id);
    d.button = Some(msg[5]);
    d.crc_valid = has_crc;
    d.protocol_display_name = (model != NAME).then(|| model.to_string());
    Some(d)
}

/// RojaFlex decoder (rtl_433 `rojaflex.c`).
pub struct RojaflexDecoder {
    rx: Receiver,
}

impl RojaflexDecoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE) }
    }
}

impl Default for RojaflexDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for RojaflexDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 100, te_long: 100, te_delta: 25, min_count_bit: FRAME_BITS_CRC - CRC_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.rx.reset(&mut Decode(rojaflex_decode));
    }

    fn feed(&mut self, level: bool, duration_us: u32) -> Option<DecodedSignal> {
        self.rx.feed(&mut Decode(rojaflex_decode), level, duration_us)
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    use crate::protocols::rtl433::test_util::{bits_of_bytes, decode_all, nrz};

    /// A frame as the C's cloner builds one: `aaaaaaaa d391d391 08 ID(3) ID/ch cmd value token sum
    /// crc` (the sum: ID bytes + ID/ch + cmd + value + token, as `msg_new[15]`), without the CRC
    /// for a bridge frame.
    pub(crate) fn frame(id: u32, channel: u8, cmd: u8, value: u8, token: u8, with_crc: bool) -> Vec<u8> {
        let mut m = vec![0x08, (id >> 20) as u8, (id >> 12) as u8, (id >> 4) as u8, ((id as u8 & 0xF) << 4) | (channel & 0xF), cmd, value, token];
        let sum = m[1..8].iter().fold(0u8, |a, &b| a.wrapping_add(b));
        m.push(sum);
        if with_crc {
            let crc = crc16(&m, 0x8005, 0xffff);
            m.extend([(crc >> 8) as u8, crc as u8]);
        }
        let mut f = vec![0xaa, 0xaa, 0xaa, 0xaa, 0xd3, 0x91, 0xd3, 0x91];
        f.extend(m);
        f
    }

    /// FM-path pulses: idle LOW, the frame's bits at `bit_us`, the carrier off.
    pub(crate) fn wire(bytes: &[u8], bit_us: u32) -> Vec<LevelDuration> {
        let mut w = vec![LevelDuration::new(false, 20_000)];
        w.extend(nrz(&bits_of_bytes(bytes, bytes.len() * 8), bit_us));
        w.push(LevelDuration::new(false, 80_000));
        w
    }

    #[test]
    fn decodes_remote_shutter_and_bridge_frames() {
        // A remote's commands (with CRC), every channel.
        for (cmd, label) in [(0x0a, "Stop"), (0x1a, "Up"), (0x8a, "Down"), (0x9a, "Save/Unsave position"), (0xda, "Go saved position"), (0xea, "Request Status")] {
            for channel in [0u8, 1, 7, 15] {
                for bit_us in [100, 96, 104] {
                    let got = decode_all(&mut RojaflexDecoder::new(), &wire(&frame(0x1A2B3C4, channel, cmd, 0x01, cmd, true), bit_us));
                    assert_eq!(got.len(), 1, "{label} ch {channel} {bit_us} µs");
                    let d = &got[0];
                    assert_eq!((d.serial, d.button, d.crc_valid), (Some(0x1A2B3C4), Some(cmd), true), "{label}");
                    assert_eq!(d.protocol_display_name, None, "RojaFlex-Remote");
                    assert_eq!(native_buttons(NAME, cmd), Some(label));
                    assert_eq!(((d.data >> 24) & 0xF) as u8, channel);
                    assert_eq!(d.data_count_bit, 88);
                }
            }
        }
        // A bridge frame ending in 9 zero bits (token 0x1a, sum 0x00) keeps its 72 bits.
        let d = &decode_all(&mut RojaflexDecoder::new(), &wire(&frame(0x0123456, 2, 0x1a, 1, 0x1a, false), 100))[0];
        assert_eq!((d.protocol_display_name.as_deref(), d.data_count_bit, d.extra), (Some("RojaFlex-Bridge"), 72, Some(0)));
        // A shutter's position status, and a bridge's command (no CRC).
        let d = &decode_all(&mut RojaflexDecoder::new(), &wire(&frame(0x0000001, 3, 0xB5, 60, 0x02, true), 100))[0];
        assert_eq!((d.protocol_display_name.as_deref(), d.button), (Some("RojaFlex-Shutter"), Some(0xB5)));
        let d = &decode_all(&mut RojaflexDecoder::new(), &wire(&frame(0x0ABCDEF, 2, 0x1a, 1, 0x1a, false), 100))[0];
        assert_eq!((d.protocol_display_name.as_deref(), d.crc_valid, d.data_count_bit), (Some("RojaFlex-Bridge"), false, 72));
        assert_eq!(d.serial, Some(0x0ABCDEF));
    }

    #[test]
    fn trailing_zero_bits_are_recovered_from_the_carrier_off_low() {
        // Frames whose CRC ends in 0 bits: the decoder finds the CRC-checked 88-bit frame.
        let mut ends = 0;
        for token in 0..=255u8 {
            let f = frame(0x0123456, 1, 0x8a, 1, token, true);
            let zeros = f.last().unwrap().trailing_zeros().min(8);
            if zeros == 0 {
                continue;
            }
            ends += 1;
            let got = decode_all(&mut RojaflexDecoder::new(), &wire(&f, 100));
            assert_eq!(got.len(), 1, "token {token:02X}");
            assert!(got[0].crc_valid && got[0].protocol_display_name.is_none(), "token {token:02X}: {:?}", got[0]);
        }
        assert!(ends > 100);
    }

    #[test]
    fn failed_checks_are_dropped() {
        let f = frame(0x0123456, 5, 0x1a, 1, 0x1a, true);
        // A flipped bit in an 88-bit frame: CRC.
        for i in [8 * 8 + 3, 8 * 12, 8 * 17 + 7] {
            let mut bits = bits_of_bytes(&f, f.len() * 8);
            bits[i] ^= true;
            let mut w = vec![LevelDuration::new(false, 20_000)];
            w.extend(nrz(&bits, 100));
            w.push(LevelDuration::new(false, 80_000));
            assert!(decode_all(&mut RojaflexDecoder::new(), &w).is_empty(), "bit {i}");
        }
        // No sync.
        let mut bad = f.clone();
        bad[5] ^= 0x10;
        assert!(decode_all(&mut RojaflexDecoder::new(), &wire(&bad, 100)).is_empty());
        // Too short (5 bytes after the sync): zeros cannot pad it to a bridge frame.
        assert!(decode_all(&mut RojaflexDecoder::new(), &wire(&f[..13], 100)).is_empty());
        // A CRC-less (bridge) frame whose token sum is wrong.
        let mut b = frame(0x0123456, 2, 0x1a, 1, 0x1a, false);
        *b.last_mut().unwrap() ^= 0x40;
        assert!(decode_all(&mut RojaflexDecoder::new(), &wire(&b, 100)).is_empty());
        // An 88-bit frame that fails its CRC is not read as a shorter CRC-less frame either.
        let mut c = frame(0x0123456, 2, 0x1a, 1, 0x1a, true);
        c[15] ^= 0x01;
        assert!(decode_all(&mut RojaflexDecoder::new(), &wire(&c, 100)).is_empty());
        // Wrong bit rate.
        assert!(decode_all(&mut RojaflexDecoder::new(), &wire(&f, 160)).is_empty());
    }
}
