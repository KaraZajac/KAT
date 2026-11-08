//! Helpers shared by the decoders ported from FoxFW (Audi V1/V2/V3, BMW V2, BMW/Mini V3, BMW CAS3).

/// FoxFW's `DURATION_DIFF(x, y)`: the absolute difference of two durations.
pub(super) const fn duration_diff(x: u32, y: u32) -> u32 {
    x.abs_diff(y)
}

#[cfg(test)]
pub(super) mod test_support {
    //! Test helpers: pulse-vector builders and the `.sub` dump the oracle comparison uses.
    use crate::protocols::common::{manchester_advance, ManchesterEvent, ManchesterState};
    use crate::protocols::{DecodedSignal, ProtocolDecoder};
    use crate::radio::demodulator::LevelDuration;

    /// Pulses that make a decoder's bit accumulator, standing in `state` after a pulse of level
    /// `prev_level`, emit `bits` one by one. `step(state, level, long)` is the decoder's reaction
    /// to one pulse: None for a path that resets it, else the next state and the bit it emitted.
    /// Levels alternate; each bit takes the first run of one to three short/long pulses that emits
    /// it (and nothing before it).
    pub fn pulses_for_bits<S: Copy>(
        mut state: S,
        mut prev_level: bool,
        bits: &[bool],
        te_short: u32,
        te_long: u32,
        step: impl Fn(S, bool, bool) -> Option<(S, Option<bool>)>,
    ) -> Vec<LevelDuration> {
        let mut out = Vec::new();
        for (index, &want) in bits.iter().enumerate() {
            let mut found = false;
            'search: for n in 1..=3u32 {
                for combo in 0..(1u32 << n) {
                    let (mut st, mut lvl) = (state, prev_level);
                    let mut pulses = Vec::new();
                    let mut got = None;
                    for k in 0..n {
                        lvl = !lvl;
                        let long = (combo >> k) & 1 == 1;
                        let Some((next, bit)) = step(st, lvl, long) else {
                            break;
                        };
                        st = next;
                        pulses.push(LevelDuration::new(lvl, if long { te_long } else { te_short }));
                        if bit.is_some() {
                            got = bit;
                            break;
                        }
                    }
                    if got == Some(want) {
                        state = st;
                        prev_level = lvl;
                        out.extend(pulses);
                        found = true;
                        break 'search;
                    }
                }
            }
            assert!(found, "no pulses make the decoder emit bit {index} ({want})");
        }
        out
    }

    /// [pulses_for_bits] for the toolbox Manchester decoder fed `from_half_bit(level, long)`.
    pub fn manchester_pulses(
        state: ManchesterState,
        prev_level: bool,
        bits: &[bool],
        te_short: u32,
        te_long: u32,
    ) -> Vec<LevelDuration> {
        pulses_for_bits(state, prev_level, bits, te_short, te_long, |st, lvl, long| {
            let (next, bit) = manchester_advance(st, ManchesterEvent::from_half_bit(lvl, long));
            // (Mid1, no bit) is the decoder's error path.
            (next != ManchesterState::Mid1 || bit.is_some()).then_some((next, bit))
        })
    }

    /// The `n` most significant of `width` bits of `value`, MSB first.
    pub fn bits_of(value: u64, width: u32) -> Vec<bool> {
        (0..width).rev().map(|i| (value >> i) & 1 == 1).collect()
    }

    /// Feed `wire` to `dec` (reset after each frame, as the oracle's unit-test callback does).
    pub fn decode_all(dec: &mut dyn ProtocolDecoder, wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        crate::protocols::f_common::test_support::feed_resetting(dec, wire)
    }

    /// Writes `wire` as a Flipper RAW `.sub` file in `$KAT_SYNTH_DIR` (nothing when it is unset),
    /// for `subghz_oracle_foxfw decode` to read back.
    pub fn dump(name: &str, frequency: u32, preset: &str, wire: &[LevelDuration]) {
        let Ok(dir) = std::env::var("KAT_SYNTH_DIR") else {
            return;
        };
        let mut text = format!(
            "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: {frequency}\nPreset: {preset}\nProtocol: RAW\n"
        );
        let values: Vec<String> = wire
            .iter()
            .map(|p| {
                if p.level {
                    p.duration_us.to_string()
                } else {
                    format!("-{}", p.duration_us)
                }
            })
            .collect();
        for chunk in values.chunks(512) {
            text += &format!("RAW_Data: {}\n", chunk.join(" "));
        }
        std::fs::write(std::path::Path::new(&dir).join(name), text).unwrap();
    }
}
