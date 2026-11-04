//! Flipper "BinRAW" files (`Protocol: BinRAW`): a signal the Flipper could not decode, saved as
//! bit blocks at one element time `TE` — `Bit_RAW: <n>` + `Data_RAW: <hex bytes>` pairs, each
//! block's `n` bits right-aligned in its `ceil(n / 8)` bytes. Flipper-ARF's encoder
//! (`lib/subghz/protocols/bin_raw.c`, `subghz_protocol_encoder_bin_raw_get_upload` +
//! `subghz_protocol_blocks_get_upload_from_bit_array`) glues the blocks into one bit sequence and
//! sends every bit as `TE` µs of carrier (1) or silence (0), `Repeat` times (default 3). KAT turns
//! a BinRAW file into the same pulses, so it decodes and replays like a RAW file.

use crate::capture::StoredLevelDuration;

/// Leading/trailing silence around the pulses (decoders expect an idle gap before a frame).
const IDLE_US: u32 = 10_000;
/// The encoder's default `encoder.repeat`.
const DEFAULT_REPEAT: u32 = 3;
/// Bound on the pulses built from one file (the Flipper's data buffer is 512 bytes).
const MAX_BITS: usize = 512 * 8;

fn hex_bytes(s: &str) -> Option<Vec<u8>> {
    s.split_whitespace().map(|b| u8::from_str_radix(b, 16).ok()).collect()
}

/// The pulses a BinRAW file's encoder sends (with idle gaps around them), or None when the text is
/// not a valid BinRAW file.
pub fn binraw_pairs(text: &str) -> Option<Vec<StoredLevelDuration>> {
    let mut is_binraw = false;
    let mut te: Option<u32> = None;
    let mut repeat = DEFAULT_REPEAT;
    let mut pending_bits: Option<usize> = None;
    let mut bits: Vec<bool> = Vec::new();
    for line in text.lines() {
        let Some((name, value)) = line.split_once(':') else { continue };
        let value = value.trim();
        match name.trim() {
            "Protocol" => is_binraw = value == "BinRAW",
            "TE" => te = value.parse().ok(),
            "Repeat" => repeat = value.parse().ok()?,
            "Bit_RAW" => pending_bits = Some(value.parse().ok()?),
            "Data_RAW" => {
                let n = pending_bits.take()?;
                let bytes = hex_bytes(value)?;
                let full = n.div_ceil(8);
                if bytes.len() < full || bits.len() + n > MAX_BITS {
                    return None;
                }
                let bias = full * 8 - n;
                bits.extend((bias..bias + n).map(|i| bytes[i / 8] >> (7 - i % 8) & 1 == 1));
            }
            _ => {}
        }
    }
    let te = te.filter(|&t| t > 0)?;
    if !is_binraw || bits.is_empty() {
        return None;
    }
    let mut pairs = vec![StoredLevelDuration { level: false, duration_us: IDLE_US }];
    let mut push = |level: bool, duration_us: u32| match pairs.last_mut() {
        Some(last) if last.level == level => last.duration_us = last.duration_us.saturating_add(duration_us),
        _ => pairs.push(StoredLevelDuration { level, duration_us }),
    };
    for _ in 0..repeat.clamp(1, 20) {
        for &b in &bits {
            push(b, te);
        }
    }
    push(false, IDLE_US);
    Some(pairs)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builds_the_encoder_pulses() {
        // Two blocks: 0b101 (3 bits, right-aligned in 0x05) and 0b1100_0001_1 (9 bits in 01 83).
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: BinRAW\nBit: 12\nTE: 400\n\
                    Bit_RAW: 3\nData_RAW: 05\nBit_RAW: 9\nData_RAW: 01 83\nRepeat: 1\n";
        let got: Vec<(bool, u32)> = binraw_pairs(text).unwrap().iter().map(|p| (p.level, p.duration_us)).collect();
        // bits 1 0 1 | 1 1 0 0 0 0 0 1 1
        assert_eq!(
            got,
            [(false, 10_000), (true, 400), (false, 400), (true, 1200), (false, 2000), (true, 800), (false, 10_000)]
        );
        // Default repeat (3): the copies follow each other with no gap (levels merge at the seams).
        let three = binraw_pairs(&text.replace("Repeat: 1\n", "")).unwrap();
        assert_eq!(three.iter().filter(|p| p.level).count(), 3 * 3 - 2);
        assert!(binraw_pairs("Protocol: RAW\nTE: 400\nBit_RAW: 3\nData_RAW: 05\n").is_none());
        assert!(binraw_pairs("Protocol: BinRAW\nBit_RAW: 3\nData_RAW: 05\n").is_none(), "no TE");
        assert!(binraw_pairs("Protocol: BinRAW\nTE: 400\nBit_RAW: 16\nData_RAW: 05\n").is_none(), "short data");
    }
}
