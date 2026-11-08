//! Test-only helpers shared by the FoxFW decoder ports (Lexus V0, Mazda V1, Mitsubishi V1, Nissan
//! V0, LDV V0): building pulse sequences from made-up frames and writing them as Flipper RAW
//! `.sub` files, so that the same bytes can be fed to the FoxFW host oracle and to KAT.

use super::common::{ManchesterEvent, ManchesterState};
use std::io::Write;

/// One pulse: level (true = HIGH) and duration in µs.
pub(super) type Pulse = (bool, u32);

/// Append a pulse, merging it into the previous one when the level is the same.
pub(super) fn push(p: &mut Vec<Pulse>, level: bool, us: u32) {
    match p.last_mut() {
        Some(last) if last.0 == level => last.1 += us,
        _ => p.push((level, us)),
    }
}

/// Level after the last pulse of `p` (the line idles LOW before any pulse).
pub(super) fn last_level(p: &[Pulse]) -> bool {
    p.last().is_some_and(|l| l.0)
}

/// The "half-bit toggle" line code of Lexus V0, Mazda V1 and Mitsubishi V1: a bit is either one
/// LONG pulse at the bit's level (when that level differs from the line's current level) or two
/// SHORT pulses `!bit`, `bit` (when it equals the current level).
pub(super) fn toggle_bits(p: &mut Vec<Pulse>, bits: &[u8], short: u32, long: u32) {
    for &b in bits {
        let b = b != 0;
        if b != last_level(p) {
            p.push((b, long));
        } else {
            p.push((!b, short));
            p.push((b, short));
        }
    }
}

/// `value`'s lowest `n` bits, MSB first.
pub(super) fn bits_of(value: u64, n: usize) -> Vec<u8> {
    (0..n).rev().map(|i| ((value >> i) & 1) as u8).collect()
}

/// The protocol names the full registry reports for `pulses` at `freq` Hz.
pub(super) fn registry_names(pulses: &[Pulse], freq: u32) -> Vec<String> {
    let pairs: Vec<crate::radio::demodulator::LevelDuration> =
        pulses.iter().map(|&(l, d)| crate::radio::demodulator::LevelDuration::new(l, d)).collect();
    super::ProtocolRegistry::new()
        .process_signal_stream(&pairs, freq)
        .into_iter()
        .map(|(n, _, _)| n)
        .collect()
}

/// Write `pulses` as a Flipper RAW `.sub` file (positive = HIGH) when the `KAT_DD_DUMP` directory
/// is set; the test suite stays file-free otherwise.
pub(super) fn dump_sub(name: &str, freq: u32, preset: &str, pulses: &[Pulse]) {
    let Ok(dir) = std::env::var("KAT_DD_DUMP") else { return };
    let mut f = std::fs::File::create(std::path::Path::new(&dir).join(format!("{name}.sub"))).unwrap();
    writeln!(f, "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: {freq}\nPreset: {preset}\nProtocol: RAW").unwrap();
    for chunk in pulses.chunks(256) {
        let line: Vec<String> =
            chunk.iter().map(|&(l, d)| if l { d.to_string() } else { format!("-{d}") }).collect();
        writeln!(f, "RAW_Data: {}", line.join(" ")).unwrap();
    }
}

/// Standard Flipper Manchester (`level ? ShortLow : ShortHigh`, as LDV V0 maps it): append pulses to
/// `p` so that `manchester_advance`, starting from `state`, emits exactly `bits` (one bit per
/// step, found by trying one or two pulses of SHORT / LONG length at the next level). Returns the
/// final state.
pub(super) fn flipper_manchester(
    p: &mut Vec<Pulse>,
    mut state: ManchesterState,
    bits: &[u8],
    short: u32,
    long: u32,
) -> ManchesterState {
    for &want in bits {
        let want = want != 0;
        let lvl = !last_level(p);
        let mut found = None;
        'search: for n in 1..=2 {
            for combo in 0..(1u32 << n) {
                let mut st = state;
                let mut level = lvl;
                let mut emitted = None;
                let mut pulses = Vec::new();
                for k in 0..n {
                    let is_long = (combo >> k) & 1 == 1;
                    let ev = ManchesterEvent::from_half_bit(!level, is_long);
                    let bit = st.advance(ev);
                    pulses.push((level, if is_long { long } else { short }));
                    if bit.is_some() {
                        emitted = bit;
                    }
                    level = !level;
                }
                if emitted == Some(want) {
                    found = Some((st, pulses));
                    break 'search;
                }
            }
        }
        let (st, pulses) = found.expect("no pulse pattern gives the wanted bit");
        state = st;
        p.extend(pulses);
    }
    state
}
