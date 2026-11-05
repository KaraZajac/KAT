//! Elero bidirectional 868 / 915 MHz blind / awning remotes (receive only).
//!
//! Port of rtl_433 `src/devices/elero.c` (`REFERENCES/rtl_433`, GPL-2.0; model "Elero",
//! r_device "Elero bidirectional blinds/awning remote (Silent Gliss and others)", `-R 372`).
//!
//! **Air format**: FSK_PULSE_PCM, 13 µs per bit (76.8 kbit/s GFSK, ±38 kHz in the rtl_433_tests
//! recordings), `reset_limit` 4000 µs; the PCM slicer (`rtl433::Modulation::FskPcm`) measures the
//! bit period on the 0x55 preamble and turns the package into one row of bits (upper tone = 1).
//! On KAT's FM path a LOW over 32 bit periods (416 µs) is the carrier stopping and ends the
//! package (rtl_433's FSK packages end with the carrier); the zeros that end the frame cannot be
//! told from the silence after it, so the package's last LOW is read as 0 … 32 trailing zero bits
//! and the shortest reading that passes the CRC wins (`rtl433::FskFrame`).
//!
//! **Frame** (`elero_decode`): exactly one row; the sync word 0xA723A723 — only its first 31 bits,
//! the payload starts one bit early — then up to 40 bytes, de-whitened with the IBM PN9 sequence:
//! `len cnt typ typ2 hop syst chl src(3) bwd(3) fwd(3) ndst dst(ndst) p1 p2 enc(8) crc(2)`
//! (`len + 3` bytes). Checks, as the C: the frame fits the row, CRC-16 (poly 0x8005, init 0xFFFF)
//! over every byte before it, and the destination list fits. `enc` is de-obfuscated (nibble
//! table, two key subtractions, XOR); its byte 2 is the command: 0x20 Up, 0x10 Stop, 0x40 Down.
//!
//! **Fields**: `serial` = rtl_433 `id` (`src`, 24 bits); `button` = the command byte (labels Up /
//! Stop / Down; rtl_433 prints "?" for others); `counter` = rtl_433 `counter` (`cnt`, 8 bits);
//! rtl_433's `channel` (the `ndst` destination bytes as hex, e.g. "11" or "112203") has no field:
//! the sending channel `chl` is byte 6 of `data`. `data` = frame bytes 0..8 (de-whitened), `extra`
//! = the last 8 frame bytes (`enc` bytes 2..8 and the CRC), `data_count_bit` = `8 · (len + 3)`;
//! `crc_valid` = the CRC passed (always, for a reported frame).
//!
//! **Receiver note**: KAT's IQ pipeline averages over 20 µs and debounces 30 µs (its shortest
//! protocol element was 50 µs), so it cannot deliver 13 µs bits: the decoder works on pulse
//! streams that resolve them (tests: the rtl_433_tests codes and recordings).
//!
//! **Encoder**: none — receive only, as rtl_433.

use super::rtl433::{crc16, frame_signal, ibm_whitening, BitBuffer, Decode, Device, Ends, FskFrame, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// rtl_433's model name.
pub(super) const NAME: &str = "Elero";

/// Metadata (see `info.rs`): GFSK NRZ, 8-bit counter + fixed
/// obfuscation (no key), shutter remote, receive only. FM preset with the closest deviation
/// (±38 kHz measured): F3 (±31.7 kHz).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Elero bidirectional blinds/awning remote (Silent Gliss and others)"],
    name_prefix: None,
    encoding: Coding::Nrz,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Elero",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&F3],
};

/// rtl_433's command names.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0x20 => Some("Up"),
        0x10 => Some("Stop"),
        0x40 => Some("Down"),
        _ => None,
    }
}

/// `r_device elero`; on KAT's FM path the package ends with the carrier (see `rtl433`).
const DEVICE: Device = Device {
    modulation: Modulation::FskPcm(FRAME),
    short_width: 13,
    long_width: 13,
    reset_limit: 4000,
    gap_limit: 0,
    sync_width: 0,
    tolerance: 0,
    gaps: Gaps::Merged,
    ends: Ends::Fsk { carrier_off: CARRIER_OFF_US },
    resync: true,
};
/// On FM input, a LOW longer than 32 bit periods is the carrier stopping.
const CARRIER_OFF_US: u32 = 32 * 13;

/// Nibble substitution table (QuadCorei8085/elero_protocol's `xor_decode.py`).
const NIBBLE_TABLE: [u8; 16] = [0x0a, 0x03, 0x01, 0x0c, 0x0d, 0x07, 0x0f, 0x06, 0x00, 0x08, 0x0b, 0x0e, 0x09, 0x02, 0x05, 0x04];

/// `elero_decode_command`: de-obfuscate the 8-byte command block in place.
fn decode_command(msg: &mut [u8; 8]) {
    for b in msg.iter_mut() {
        let nh = NIBBLE_TABLE[usize::from(*b >> 4)];
        let nl = NIBBLE_TABLE[usize::from(*b & 0x0f)];
        *b = (nh << 4) | nl;
    }
    let mut key: u8 = 0xfe;
    for b in msg.iter_mut().take(2) {
        let ln = b.wrapping_sub(key) & 0x0f;
        let hn = (*b & 0xf0).wrapping_sub(key & 0xf0);
        *b = hn | ln;
        key = key.wrapping_sub(0x22);
    }
    let (xor_b0, xor_b1) = (msg[0], msg[1]);
    for i in (0..8).step_by(2) {
        msg[i] ^= xor_b0;
        msg[i + 1] ^= xor_b1;
    }
    let mut key: u8 = 0xba;
    for b in msg.iter_mut().skip(2) {
        let ln = b.wrapping_sub(key) & 0x0f;
        let hn = (*b & 0xf0).wrapping_sub(key & 0xf0);
        *b = hn | ln;
        key = key.wrapping_sub(0x22);
    }
}

/// Sync word 0xa723a723, but only its first 31 bits are sync.
const SYNC: [u8; 4] = [0xa7, 0x23, 0xa7, 0x23];
const SYNC_BITS: usize = 31;

/// The FSK reading of the package's last gap (`rtl433::FskFrame`): the smallest frame is the sync
/// and 29 bytes (`17 + ndst + 2 + 8 + 2` with no destination).
const FRAME: FskFrame = FskFrame { unchecked: None, sync: &SYNC, sync_bits: SYNC_BITS, min_bits: SYNC_BITS + 29 * 8 };

/// `elero_decode`.
fn elero_decode(bits: &BitBuffer) -> Option<DecodedSignal> {
    if bits.num_rows() != 1 {
        return None; // DECODE_ABORT_EARLY
    }
    let row_bits = bits.bits_per_row(0);
    let start = bits.search(0, 0, &SYNC, SYNC_BITS);
    if start == row_bits {
        return None;
    }
    let start = start + SYNC_BITS;
    let avail_bits = row_bits - start;
    if avail_bits < 8 {
        return None; // DECODE_ABORT_LENGTH
    }
    let mut frame = [0u8; 40];
    let avail_bytes = (avail_bits / 8).min(frame.len());
    bits.extract_bytes(0, start, &mut frame, avail_bytes * 8);
    ibm_whitening(&mut frame[..avail_bytes]);
    let total = usize::from(frame[0]) + 3; // len byte + len bytes + 2-byte CRC
    if total > frame.len() || avail_bytes < total {
        return None; // truncated capture
    }
    let crc_calc = crc16(&frame[..total - 2], 0x8005, 0xffff);
    let crc_recv = u16::from(frame[total - 2]) << 8 | u16::from(frame[total - 1]);
    if crc_calc != crc_recv {
        return None; // DECODE_FAIL_MIC
    }
    let ndst = usize::from(frame[16]);
    if 17 + ndst + 2 + 8 + 2 > total {
        return None; // DECODE_FAIL_SANITY: dst list doesn't fit the frame
    }
    let src = u32::from(frame[7]) << 16 | u32::from(frame[8]) << 8 | u32::from(frame[9]);
    let mut enc = [0u8; 8];
    enc.copy_from_slice(&frame[17 + ndst + 2..17 + ndst + 10]);
    decode_command(&mut enc);
    let mut d = frame_signal(&frame[..total], total * 8);
    d.serial = Some(src);
    d.button = Some(enc[2]);
    d.counter = Some(u16::from(frame[1]));
    d.crc_valid = true;
    Some(d)
}

/// Elero decoder (rtl_433 `elero.c`).
pub struct EleroDecoder {
    rx: Receiver,
}

impl EleroDecoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE) }
    }
}

impl Default for EleroDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for EleroDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 13, te_long: 13, te_delta: 3, min_count_bit: 29 * 8 }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.rx.reset(&mut Decode(elero_decode));
    }

    fn feed(&mut self, level: bool, duration_us: u32) -> Option<DecodedSignal> {
        self.rx.feed(&mut Decode(elero_decode), level, duration_us)
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
    use crate::protocols::rtl433::test_util::{bits_of_bytes, decode_all, expected_json, nrz, rtl433_tests};

    /// The inverse of [decode_command] (to build test frames): each step undone in reverse.
    fn encode_command(plain: &[u8; 8]) -> [u8; 8] {
        // Search byte by byte: decode_command is a bijection per position given the earlier
        // bytes' XOR keys (bytes 0 and 1 decode to zero, so only bytes 2..8 carry data).
        let mut best = [0u8; 8];
        // Bytes 0..2 are XORed with themselves (always 0); choose 0x00 inputs' images.
        for i in 2..8 {
            for v in 0..=255u8 {
                let mut m = best;
                m[i] = v;
                let mut d = m;
                decode_command(&mut d);
                if d[i] == plain[i] {
                    best[i] = v;
                    break;
                }
            }
        }
        best
    }

    /// A frame: `len cnt 45 10 05 01 chl src×3 ndst dst p1=00 p2=03 enc crc`, whitened, behind the
    /// 0x55 preamble and the sync (31 bits).
    pub(crate) fn frame_bits(cnt: u8, chl: u8, src: u32, dst: &[u8], command: u8) -> Vec<bool> {
        let mut f = vec![0, cnt, 0x45, 0x10, 0x05, 0x01, chl];
        for _ in 0..3 {
            f.extend([(src >> 16) as u8, (src >> 8) as u8, src as u8]);
        }
        f.push(dst.len() as u8);
        f.extend_from_slice(dst);
        f.extend([0x00, 0x03]);
        f.extend(encode_command(&[0, 0, command, 0, 0, 0, 0, 0]));
        // len: the bytes after it, up to the CRC (the frame is len + 3 bytes).
        f[0] = (f.len() - 1) as u8;
        let crc = crc16(&f, 0x8005, 0xffff);
        f.extend([(crc >> 8) as u8, crc as u8]);
        ibm_whitening(&mut f);
        let mut bits = bits_of_bytes(&[0x55; 12], 96);
        bits.extend(bits_of_bytes(&[0xa7, 0x23, 0xa7, 0x23], 31));
        bits.extend(bits_of_bytes(&f, f.len() * 8));
        bits.extend(bits_of_bytes(&[0x55, 0x55], 16)); // tail
        bits
    }

    /// FM-path pulses of a frame: idle LOW, the bits at `bit_us`, the carrier off.
    pub(crate) fn wire(bits: &[bool], bit_us: u32) -> Vec<LevelDuration> {
        let mut w = vec![LevelDuration::new(false, 20_000)];
        w.extend(nrz(bits, bit_us));
        w.push(LevelDuration::new(false, 80_000));
        w
    }

    #[test]
    fn decode_command_matches_rtl_433() {
        // rtl_433_tests elero/01 (Ch1 Up, counter 106): the C's command block decodes to 0x20.
        let code = "{367}555555555555555555555555a723a723c916b115d10845407256add4422708b6aad85bb6c78b9cd8ab22b31ac1b4";
        let d = elero_decode(&BitBuffer::parse(code)).unwrap();
        assert_eq!((d.serial, d.button, d.counter), (Some(0x84D351), Some(0x20), Some(106)));
        assert_eq!(d.data >> 56, 0x1b, "len");
        assert_eq!((d.data_count_bit, d.data >> 8 & 0xFF), (30 * 8, 0x11), "chl");
    }

    #[test]
    fn decodes_synthetic_frames_for_every_channel_and_command() {
        for (dst, chl) in [(&[0x11u8][..], 0x11u8), (&[0x22], 0x22), (&[0x03], 0x03), (&[0x11, 0x22, 0x03], 0xff)] {
            for (command, label) in [(0x20u8, "Up"), (0x10, "Stop"), (0x40, "Down")] {
                for bit_us in [13, 12, 14] {
                    let bits = frame_bits(0x7A, chl, 0x84D351, dst, command);
                    let got = decode_all(&mut EleroDecoder::new(), &wire(&bits, bit_us));
                    assert_eq!(got.len(), 1, "{dst:X?} {label} {bit_us} µs");
                    let d = &got[0];
                    assert_eq!((d.serial, d.button, d.counter, d.crc_valid), (Some(0x84D351), Some(command), Some(0x7A), true));
                    assert_eq!(native_buttons(NAME, command), Some(label));
                    assert_eq!((d.data >> 8) as u8, chl);
                    assert!(!d.encoder_capable);
                }
            }
        }
    }

    #[test]
    fn failed_checks_are_dropped() {
        let bits = frame_bits(1, 0x11, 0x123456, &[0x11], 0x20);
        let sync_end = 96 + 31;
        // One flipped payload bit: CRC.
        for i in [sync_end + 3, sync_end + 60, sync_end + 200] {
            let mut bad = bits.clone();
            bad[i] ^= true;
            assert!(decode_all(&mut EleroDecoder::new(), &wire(&bad, 13)).is_empty(), "bit {i}");
        }
        // Truncated frame (last CRC byte cut, carrier off).
        let cut = &bits[..sync_end + 29 * 8];
        assert!(decode_all(&mut EleroDecoder::new(), &wire(cut, 13)).is_empty());
        // Broken sync.
        let mut bad = bits.clone();
        bad[96 + 5] ^= true;
        assert!(decode_all(&mut EleroDecoder::new(), &wire(&bad, 13)).is_empty());
        // Wrong bit rate (20 µs).
        assert!(decode_all(&mut EleroDecoder::new(), &wire(&bits, 20)).is_empty());
    }

    /// rtl_433_tests `elero/codes_test.txt`: the 38 CRC-valid codes of the reporter's captures, as
    /// 13 µs NRZ pulse trains, against `codes_test.json` (rtl_433's output for them).
    #[test]
    fn rtl_433_tests_codes() {
        let Some(root) = rtl433_tests() else {
            eprintln!("Skip: rtl_433_tests not found");
            return;
        };
        let Ok(codes) = std::fs::read_to_string(root.join("elero/codes_test.txt")) else { return };
        let json = std::fs::read_to_string(root.join("elero/codes_test.json")).unwrap();
        let codes: Vec<&str> = codes.lines().filter(|l| l.starts_with('{')).collect();
        let expected: Vec<&str> = json.lines().collect();
        assert_eq!(codes.len(), 38);
        assert_eq!(expected.len(), 38);
        for (code, want) in codes.iter().zip(expected) {
            let row = BitBuffer::parse(code);
            let bits = bits_of_bytes(row.row(0), row.bits_per_row(0));
            let got = decode_all(&mut EleroDecoder::new(), &wire(&bits, 13));
            assert_eq!(got.len(), 1, "{code}");
            let d = &got[0];
            let want: serde_json::Value = serde_json::from_str(want).unwrap();
            let id = u32::from_str_radix(want["id"].as_str().unwrap(), 16).unwrap();
            let counter = want["counter"].as_u64().unwrap() as u16;
            let command = match want["command"].as_str().unwrap() {
                "Up" => 0x20,
                "Stop" => 0x10,
                "Down" => 0x40,
                other => panic!("{other}"),
            };
            assert_eq!((d.serial, d.button, d.counter), (Some(id), Some(command), Some(counter)), "{code}");
            // The channel list rtl_433 prints: the ndst bytes after byte 16 (in data/extra or
            // the frame bytes; checked on the decoded frame).
            let channel = want["channel"].as_str().unwrap();
            let f = elero_frame_bytes(code);
            let ndst = usize::from(f[16]);
            let list: String = f[17..17 + ndst].iter().map(|b| format!("{b:02X}")).collect();
            assert_eq!(list, channel, "{code}");
        }
    }

    /// The de-whitened frame bytes of an rtl_433 code (test helper).
    fn elero_frame_bytes(code: &str) -> Vec<u8> {
        let b = BitBuffer::parse(code);
        let start = b.search(0, 0, &[0xa7, 0x23, 0xa7, 0x23], 31) + 31;
        let mut f = vec![0u8; 40];
        let n = ((b.bits_per_row(0) - start) / 8).min(40);
        b.extract_bytes(0, start, &mut f, n * 8);
        ibm_whitening(&mut f[..n]);
        f
    }

    /// The rtl_433_tests recordings (2.048 MS/s GFSK): KAT's IQ pipeline cannot resolve 13 µs
    /// bits, so this demodulates them with a plain FM discriminator (|Δφ| between samples,
    /// smoothed over 5 samples, sliced halfway between the tones while the carrier is on) and
    /// checks the decoder on those pulses against the recordings' JSON.
    #[test]
    #[ignore]
    fn rtl_433_tests_recordings_with_a_test_demodulator() {
        let Some(root) = rtl433_tests() else {
            eprintln!("Skip: rtl_433_tests not found");
            return;
        };
        for rel in ["elero/01/g003_869.4M_2048k", "elero/02/g010_869.4M_2048k", "elero/03/g011_869.4M_2048k"] {
            let path = root.join(format!("{rel}.cu8"));
            let Ok(iq) = std::fs::read(&path) else { return };
            let want = &expected_json(&path)[0];
            let pulses = fm_demod_cu8(&iq, 2_048_000.0);
            let got = decode_all(&mut EleroDecoder::new(), &pulses);
            assert_eq!(got.len(), 1, "{rel}: {got:?}");
            let d = &got[0];
            let id = u32::from_str_radix(want["id"].as_str().unwrap(), 16).unwrap();
            let counter = want["counter"].as_u64().unwrap() as u16;
            assert_eq!((d.serial, d.counter), (Some(id), Some(counter)), "{rel}");
            assert_eq!(native_buttons(NAME, d.button.unwrap()), want["command"].as_str(), "{rel}");
        }
    }

    /// Test-only FM demodulator for a short cu8 recording with one strong FSK burst.
    pub(crate) fn fm_demod_cu8(iq: &[u8], fs: f64) -> Vec<LevelDuration> {
        let z: Vec<(f32, f32)> = iq.chunks_exact(2).map(|c| (c[0] as f32 - 127.5, c[1] as f32 - 127.5)).collect();
        let mag: Vec<f32> = z.iter().map(|(i, q)| (i * i + q * q).sqrt()).collect();
        let mut sorted = mag.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let (noise, peak) = (sorted[sorted.len() / 2], sorted[sorted.len() * 999 / 1000]);
        let on: Vec<bool> = mag.iter().map(|&m| m > (noise + peak) / 2.0).collect();
        let mut freq = vec![0f32; z.len()];
        for k in 1..z.len() {
            let (a, b) = (z[k], z[k - 1]);
            let (re, im) = (a.0 * b.0 + a.1 * b.1, a.1 * b.0 - a.0 * b.1);
            freq[k] = im.atan2(re);
        }
        let w = 5;
        let smooth: Vec<f32> = (0..z.len()).map(|k| freq[k.saturating_sub(w / 2)..(k + w / 2 + 1).min(z.len())].iter().sum::<f32>() / w as f32).collect();
        let on_f: Vec<f32> = smooth.iter().zip(&on).filter(|(_, &o)| o).map(|(&f, _)| f).collect();
        let mut s = on_f.clone();
        s.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mid = (s[s.len() / 10] + s[s.len() * 9 / 10]) / 2.0;
        let us = 1e6 / fs;
        // Runs of equal level, timed from sample counts.
        let mut retimed = Vec::new();
        let mut run_start = 0usize;
        let mut k = 0usize;
        while k < z.len() {
            let level = on[k] && smooth[k] > mid;
            let mut j = k;
            while j < z.len() && (on[j] && smooth[j] > mid) == level {
                j += 1;
            }
            let dur = ((j - run_start) as f64 * us).round() as u32;
            retimed.push(LevelDuration::new(level, dur.max(1)));
            run_start = j;
            k = j;
        }
        retimed.push(LevelDuration::new(false, 80_000));
        retimed
    }
}
