//! Somfy / Velux io-homecontrol (receive only).
//!
//! Port of rtl_433 `src/devices/somfy_iohc.c` (`REFERENCES/rtl_433`, GPL-2.0; model "Somfy-IOHC",
//! r_device "Somfy io-homecontrol", `-R 189`), e.g. the Velux KI 313 remote. No recordings exist
//! (rtl_433_tests' `somfy/` holds Somfy RTS only, which KAT's Somfy Telis decodes); validated on
//! the C's example packets and synthetic frames, through rtl_433 itself (see `docs/somfy_iohc.md`).
//!
//! **Air format**: FSK_PULSE_PCM, 26 µs per bit (38.4 kbit/s, 868.25–868.95 MHz),
//! `reset_limit` 300 µs (UART coding has at most 9 zeros in a row); one row per message. On KAT's
//! FM path a LOW over 32 bit periods (832 µs) is the carrier stopping and ends the package; its
//! last LOW is read as 0 … 32 trailing zero bits, the shortest reading that passes the CRC wins
//! (`rtl433::FskFrame`; a frame ends on a UART stop bit, so 0 does).
//!
//! **Frame** (`somfy_iohc_decode`): exactly one row; after the pattern 0x57FD99 (preamble end +
//! the sync word 0xFF33, UART coded) the message is UART 8N1 (start 0, 8 data bits LSB first, stop
//! 1): control byte 1 (end / start / mode flags, 5-bit length `msg_len`), control byte 2 (beacon /
//! routed / low-power flags, version), 24-bit destination, 24-bit source, command ID, payload,
//! in one-way mode with ≥ 8 payload bytes a 16-bit counter and a 48-bit MAC, and a CRC-16 (KERMIT:
//! reflected 0x1021, init 0) over the `msg_len + 3` bytes. Checks, as the C: ≥ 11 UART bytes,
//! `8 ≤ msg_len`, the bytes present, the CRC (residue 0).
//!
//! **Fields**: `serial` = rtl_433 `id` (source address, 24 bits); `button` = rtl_433 `msg_type`
//! (command ID); `counter` = rtl_433 `counter` (the 16-bit MAC counter, one-way mode only, 0
//! when the payload is too short to carry it — as the C); `dst_id`, `msg`, `mac`, `mode`, `version`
//! and the flags have no field: `data` = message bytes 0..8 (control 1, control 2, destination,
//! source), `extra` = the message's last 8 bytes (MAC tail and CRC), `data_count_bit` = 8 ·
//! (`msg_len` + 3); `crc_valid` = the CRC passed.
//!
//! **Receiver note**: like Elero, 26 µs bits are below what KAT's IQ pipeline resolves (20 µs
//! smoothing, 30 µs debounce).
//!
//! **Encoder**: none — receive only, as rtl_433 (and the frames carry an AES MAC).

use super::rtl433::{crc16lsb, extract_bytes_uart_8n1, frame_signal, BitBuffer, Decode, Device, Ends, FskFrame, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// rtl_433's model name.
pub(super) const NAME: &str = "Somfy-IOHC";

/// Metadata (see `info.rs`): FSK UART-framed NRZ, AES-128 MAC,
/// shutter / window remote, receive only. FM preset with the closest deviation (io-homecontrol:
/// ±19.2 kHz): F2 (±19.04 kHz).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Somfy io-homecontrol"],
    name_prefix: None,
    encoding: Coding::Nrz,
    rf: Rf::FM,
    encryption: "AES-128",
    make: "Somfy",
    category: Category::Gate,
    buttons: None,
    presets: &[&F2],
};

/// `r_device somfy_iohc`; on KAT's FM path the package ends with the carrier (see `rtl433`).
const DEVICE: Device = Device {
    modulation: Modulation::FskPcm(FRAME),
    short_width: 26,
    long_width: 26,
    reset_limit: 300,
    gap_limit: 0,
    sync_width: 0,
    tolerance: 0,
    gaps: Gaps::Merged,
    ends: Ends::Fsk { carrier_off: CARRIER_OFF_US },
    resync: true,
};
/// On FM input, a LOW longer than 32 bit periods is the carrier stopping.
const CARRIER_OFF_US: u32 = 32 * 26;

/// The end of the preamble and the UART-coded sync word 0xFF33, searched first.
const PREAMBLE: [u8; 3] = [0x57, 0xfd, 0x99];

/// The FSK reading of the package's last gap (`rtl433::FskFrame`): the smallest frame is the
/// pattern and 11 UART bytes.
const FRAME: FskFrame = FskFrame { unchecked: None, sync: &PREAMBLE, sync_bits: 24, min_bits: 24 + 11 * 10 };

/// `somfy_iohc_decode`.
fn somfy_iohc_decode(bits: &BitBuffer) -> Option<DecodedSignal> {
    if bits.num_rows() != 1 {
        return None; // DECODE_ABORT_EARLY
    }
    let row_bits = bits.bits_per_row(0);
    let offset = bits.search(0, 0, &PREAMBLE, 24) + 24;
    if offset >= row_bits {
        return None;
    }
    let mut b = [0u8; 1 + 31 + 2]; // length, payload, CRC
    let num_bits = (row_bits - offset).min(b.len() * 10);
    let len = extract_bytes_uart_8n1(bits.row(0), offset, num_bits, &mut b);
    if len < 11 {
        return None; // DECODE_ABORT_LENGTH
    }
    let msg_len = usize::from(b[0] & 0x1f);
    if len < msg_len + 3 || msg_len < 8 {
        return None;
    }
    let len = msg_len + 3;
    let protocol_mode = (b[0] & 0x20) >> 5;
    let src = u32::from(b[5]) << 16 | u32::from(b[6]) << 8 | u32::from(b[7]);
    let cmd_id = b[8];
    let mut data_length = msg_len - 8;
    let mut seq_nr = 0u16;
    if protocol_mode != 0 && data_length >= 8 {
        data_length -= 8;
        seq_nr = u16::from(b[9 + data_length]) << 8 | u16::from(b[9 + data_length + 1]);
    }
    if crc16lsb(&b[..len], 0x8408, 0x0000) != 0 {
        return None; // DECODE_FAIL_MIC
    }
    let mut d = frame_signal(&b[..len], len * 8);
    d.serial = Some(src);
    d.button = Some(cmd_id);
    // `counter` is output in one-way mode only (DATA_COND msg_protocol_mode == 1).
    d.counter = (protocol_mode == 1).then_some(seq_nr);
    d.crc_valid = true;
    Some(d)
}

/// Somfy io-homecontrol decoder (rtl_433 `somfy_iohc.c`).
pub struct SomfyIohcDecoder {
    rx: Receiver,
}

impl SomfyIohcDecoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE) }
    }
}

impl Default for SomfyIohcDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for SomfyIohcDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 26, te_long: 26, te_delta: 6, min_count_bit: 11 * 8 }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.rx.reset(&mut Decode(somfy_iohc_decode));
    }

    fn feed(&mut self, level: bool, duration_us: u32) -> Option<DecodedSignal> {
        self.rx.feed(&mut Decode(somfy_iohc_decode), level, duration_us)
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
    use crate::protocols::rtl433::test_util::{decode_all, nrz};

    /// Example packets from the C's documentation (Velux remotes; the sync ff33 stripped): message
    /// bytes incl. CRC. `rtl_433_c_example_packets` runs all 44 when REFERENCES is present.
    pub(crate) const EXAMPLES: &[&str] = &[
        "f62000003fdacdea000161000000000bddfd8ef56f15adaa1e",
        "f60000003fdacdea000161000000000bddfd8ef56f15ad4f9c",
        "f62000003fdacdea000161c80000000bbd8aa3a9732e1026d2",
        "f80000007fe1f573000161d40080c800000d6c2c3a3123e6ab7f1e",
        "f80000007fc58967000161d40080c800000c6304e867ed64adf055",
        "f60000003f708758000161d20000003bd2e6b62cef54c8a937",
        "f60000003f17f523000147c800000018c438789cb680ccbc74",
        "f80000003f17f5232002ff0143010e000018c5045ee107363d59b4",
        "f80000007fe1f573000161d40080c800000daaeb6adf4f8fad3404",
    ];

    pub(crate) fn hex(s: &str) -> Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap()).collect()
    }

    /// UART 8N1 bits of `bytes` (start 0, data LSB first, stop 1).
    fn uart(bytes: &[u8]) -> Vec<bool> {
        bytes.iter().flat_map(|&b| std::iter::once(false).chain((0..8).map(move |i| (b >> i) & 1 == 1)).chain(std::iter::once(true))).collect()
    }

    /// An air frame: 0x55 preamble, the UART-coded sync ff33, the UART-coded message.
    pub(crate) fn frame_bits(msg: &[u8]) -> Vec<bool> {
        let mut bits: Vec<bool> = (0..64).map(|i| (0x55u8 >> (7 - i % 8)) & 1 == 1).collect();
        bits.extend(uart(&[0xff, 0x33]));
        bits.extend(uart(msg));
        bits
    }

    pub(crate) fn wire(bits: &[bool], bit_us: u32) -> Vec<LevelDuration> {
        let mut w = vec![LevelDuration::new(false, 20_000)];
        w.extend(nrz(bits, bit_us));
        w.push(LevelDuration::new(false, 80_000));
        w
    }

    /// Decode one example packet at several bit rates; check the fields the C outputs.
    fn check_packet(ex: &str) {
        let msg = hex(ex);
        for bit_us in [26, 25, 27] {
            let got = decode_all(&mut SomfyIohcDecoder::new(), &wire(&frame_bits(&msg), bit_us));
            assert_eq!(got.len(), 1, "{ex} {bit_us} µs");
            let d = &got[0];
            let src = u32::from(msg[5]) << 16 | u32::from(msg[6]) << 8 | u32::from(msg[7]);
            assert_eq!((d.serial, d.button, d.crc_valid), (Some(src), Some(msg[8]), true), "{ex}");
            // One-way mode (0x20 flag) with an 8-byte+ payload: the counter before the MAC.
            let mode = msg[0] & 0x20 != 0;
            let want = mode.then(|| {
                let dl = usize::from(msg[0] & 0x1f) - 8;
                if dl >= 8 { u16::from(msg[9 + dl - 8]) << 8 | u16::from(msg[10 + dl - 8]) } else { 0 }
            });
            assert_eq!(d.counter, want, "{ex}");
            assert_eq!(d.data_count_bit, msg.len() * 8);
        }
    }

    #[test]
    fn decodes_the_c_example_packets() {
        for ex in EXAMPLES {
            check_packet(ex);
        }
    }

    /// Every example packet of `REFERENCES/rtl_433/src/devices/somfy_iohc.c` (44; skipped when
    /// the reference is absent).
    #[test]
    fn rtl_433_c_example_packets() {
        let Ok(c) = std::fs::read_to_string("REFERENCES/rtl_433/src/devices/somfy_iohc.c") else {
            eprintln!("Skip: REFERENCES/rtl_433 not found");
            return;
        };
        let packets: Vec<String> = c
            .lines()
            .map(str::trim)
            .filter(|l| l.starts_with("ff33 "))
            .map(|l| l.split('[').next().unwrap().split_whitespace().skip(1).collect())
            .collect();
        assert_eq!(packets.len(), 44);
        for p in &packets {
            check_packet(p);
        }
    }

    #[test]
    fn failed_checks_are_dropped() {
        let msg = hex(EXAMPLES[0]);
        let bits = frame_bits(&msg);
        // A flipped data bit (inside a UART byte): CRC.
        for i in [64 + 20 + 3, 64 + 20 + 10 * 7 + 4, 64 + 20 + 10 * 20 + 8] {
            let mut bad = bits.clone();
            bad[i] ^= true;
            assert!(decode_all(&mut SomfyIohcDecoder::new(), &wire(&bad, 26)).is_empty(), "bit {i}");
        }
        // A broken stop bit: fewer than msg_len + 3 UART bytes.
        let mut bad = bits.clone();
        bad[64 + 20 + 10 * 5 + 9] = false;
        assert!(decode_all(&mut SomfyIohcDecoder::new(), &wire(&bad, 26)).is_empty());
        // msg_len < 8.
        let mut short = msg.clone();
        short[0] = (short[0] & 0xe0) | 7;
        assert!(decode_all(&mut SomfyIohcDecoder::new(), &wire(&frame_bits(&short), 26)).is_empty());
        // Wrong bit rate.
        assert!(decode_all(&mut SomfyIohcDecoder::new(), &wire(&bits, 40)).is_empty());
    }
}
