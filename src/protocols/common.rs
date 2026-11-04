//! Common utilities for protocol implementations.
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/protocols_common.c`
//! and `protocols_common.h`. The reference provides only Flipper preset name mapping
//! (`protopirate_get_short_preset_name`); we implement the same mapping in `short_preset_name`.
//! This module also holds shared types and helpers used by multiple protocol decoders.
//!
//! **CRC / add_bit**: `crc8_kia` matches `kia_crc8` in kia_v0.c (polynomial 0x7F, init 0x00);
//! `add_bit` matches the common shift-left-and-append pattern used in the reference decoders.
//!
//! **Manchester**: [`ManchesterState`] / [`ManchesterEvent`] / [`manchester_advance`] are an exact
//! port of Flipper's `lib/toolbox/manchester_decoder.c`, which every C reference (ProtoPirate,
//! Flipper-ARF) decodes Manchester through. Protocol modules share this one implementation and keep
//! only their own level→event mapping, initial state and reset points, mirroring their C file.

/// Decoded signal information
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecodedSignal {
    /// Serial number / device ID
    pub serial: Option<u32>,
    /// Button code
    pub button: Option<u8>,
    /// Rolling counter
    pub counter: Option<u16>,
    /// CRC is valid
    pub crc_valid: bool,
    /// Raw data (up to 64 bits)
    pub data: u64,
    /// Number of bits in data
    pub data_count_bit: usize,
    /// Whether encoding is supported
    pub encoder_capable: bool,
    /// Protocol-specific extra data for encoding (e.g. VAG: vag_type + key_idx)
    pub extra: Option<u64>,
    /// Optional protocol display name (e.g. "KeeLoq (DoorHan)"). When set, used as the protocol name for this decode.
    pub protocol_display_name: Option<String>,
}

impl DecodedSignal {
    #[allow(dead_code)]
    pub fn new(data: u64, bit_count: usize) -> Self {
        Self {
            serial: None,
            button: None,
            counter: None,
            crc_valid: false,
            data,
            data_count_bit: bit_count,
            encoder_capable: false,
            extra: None,
            protocol_display_name: None,
        }
    }
}

/// CRC8 calculation with custom polynomial (MSB-first, shift-left style).
///
/// # Arguments
/// * `data` - Data bytes to calculate CRC over
/// * `poly` - CRC polynomial (e.g. 0x7F for Kia)
/// * `init` - Initial CRC value (e.g. 0x00)
pub fn crc8(data: &[u8], poly: u8, init: u8) -> u8 {
    let mut crc = init;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// CRC8 for Kia protocol (matches kia_v0.c kia_crc8: polynomial 0x7F, init 0x00)
pub fn crc8_kia(data: &[u8]) -> u8 {
    crc8(data, 0x7F, 0x00)
}

/// Add a bit to the decoder's data accumulator (shift-left, LSB last; matches reference add_bit pattern)
#[inline]
#[allow(dead_code)]
pub fn add_bit(data: &mut u64, count: &mut usize, bit: bool) {
    *data = (*data << 1) | (bit as u64);
    *count += 1;
}

// =============================================================================
// Preset name mapping (protocols_common.c: protopirate_get_short_preset_name)
// =============================================================================

/// Short preset name for display. Matches `protopirate_get_short_preset_name`.
/// Returns "UNKNOWN" for null/empty or unknown preset names (reference returns the
/// original pointer for unknown; we use "UNKNOWN" to avoid allocation).
#[allow(dead_code)]
#[inline]
pub fn short_preset_name(preset: Option<&str>) -> &'static str {
    let p = match preset {
        None => return "UNKNOWN",
        Some(s) if s.is_empty() => return "UNKNOWN",
        Some(s) => s,
    };
    match p {
        "FuriHalSubGhzPresetOok270Async" => "AM270",
        "FuriHalSubGhzPresetOok650Async" => "AM650",
        "FuriHalSubGhzPreset2FSKDev238Async" => "FM238",
        "FuriHalSubGhzPreset2FSKDev12KAsync" => "FM12K",
        "FuriHalSubGhzPreset2FSKDev476Async" => "FM476",
        "FuriHalSubGhzPresetCustom" => "CUSTOM",
        _ => "UNKNOWN",
    }
}

// =============================================================================
// Manchester decoder: exact port of Flipper lib/toolbox/manchester_decoder.c
// =============================================================================
// Reference: REFERENCES/Flipper-ARF/lib/toolbox/manchester_decoder.{c,h}.
//
// The C keeps a 2-bit next state per (state, event) in `transitions[]`; the event value is the
// shift into that byte. Expanded, the table is (row = current state):
//
//   state \ event | ShortLow (0) | ShortHigh (2) | LongLow (4)  | LongHigh (6)
//   --------------+--------------+---------------+--------------+-------------
//   Start1 (0)    | Mid1, emit 1 | reset         | reset        | reset
//   Mid1   (1)    | reset        | Start1        | reset        | Mid0, emit 0
//   Mid0   (2)    | Start0       | reset         | Mid1, emit 1 | reset
//   Start0 (3)    | reset        | Mid0, emit 0  | reset        | reset
//
// "reset" is the C's error path: the table maps back to the current state, so the state becomes
// Mid1 and no bit is emitted. `ManchesterEvent::Reset` also forces Mid1.
//
// In Flipper's own terms a 1 bit is a Low half followed by a High half and a 0 bit is High then Low;
// which RF level is a "Low" event is decided by each protocol's level→event mapping (ProtoPirate's
// `pp_manchester_event()` is `level ? ShortLow : ShortHigh`; Ford V1/V2 and Kia V7 use
// `level ? ShortHigh : ShortLow`). A bit is emitted on reaching Mid0/Mid1, i.e. at mid-bit.

/// Flipper `ManchesterState` (manchester_decoder.h): same variants and discriminants.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ManchesterState {
    Start1 = 0,
    Mid1 = 1,
    Mid0 = 2,
    Start0 = 3,
}

/// Flipper `ManchesterEvent` (manchester_decoder.h). The discriminant is the shift into the
/// transition-table byte, exactly as in C.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ManchesterEvent {
    ShortLow = 0,
    ShortHigh = 2,
    LongLow = 4,
    LongHigh = 6,
    Reset = 8,
}

impl ManchesterEvent {
    /// Data event for one pulse: `high` selects the High/Low event sense and `long` the
    /// Long/Short event length.
    ///
    /// `high` is the Manchester *event* sense, not necessarily the RF level: every protocol mirrors
    /// its C mapping at the call site, e.g. ProtoPirate `level ? ShortLow : ShortHigh` becomes
    /// `ManchesterEvent::from_half_bit(!level, long)`.
    pub const fn from_half_bit(high: bool, long: bool) -> Self {
        match (high, long) {
            (false, false) => Self::ShortLow,
            (true, false) => Self::ShortHigh,
            (false, true) => Self::LongLow,
            (true, true) => Self::LongHigh,
        }
    }
}

/// `transitions[]` from manchester_decoder.c: the 2-bit next state for event shift 0/2/4/6.
const MANCHESTER_TRANSITIONS: [u8; 4] = [0b0000_0001, 0b1001_0001, 0b1001_1011, 0b1111_1011];

impl ManchesterState {
    /// State from its 2-bit C encoding.
    const fn from_bits(bits: u8) -> Self {
        match bits & 0x3 {
            0 => Self::Start1,
            1 => Self::Mid1,
            2 => Self::Mid0,
            _ => Self::Start0,
        }
    }

    /// In-place [`manchester_advance`]: updates `self` and returns the decoded bit, if any.
    pub fn advance(&mut self, event: ManchesterEvent) -> Option<bool> {
        let (next, bit) = manchester_advance(*self, event);
        *self = next;
        bit
    }
}

/// Exact port of Flipper's `manchester_advance()`. Returns `(next_state, Some(bit))` when a data
/// bit is decoded and `(next_state, None)` otherwise.
pub fn manchester_advance(
    state: ManchesterState,
    event: ManchesterEvent,
) -> (ManchesterState, Option<bool>) {
    if event == ManchesterEvent::Reset {
        return (ManchesterState::Mid1, None);
    }
    let next = ManchesterState::from_bits(MANCHESTER_TRANSITIONS[state as usize] >> (event as u8));
    if next == state {
        // No progress: the C treats this as an error and resets to Mid1 without a bit.
        return (ManchesterState::Mid1, None);
    }
    match next {
        ManchesterState::Mid0 => (next, Some(false)),
        ManchesterState::Mid1 => (next, Some(true)),
        _ => (next, None),
    }
}

/// Button names for common keyfob buttons
#[allow(dead_code)]
pub fn get_button_name(btn: u8) -> &'static str {
    match btn {
        0x01 => "Lock",
        0x02 => "Unlock",
        0x03 => "Lock+Unlock",
        0x04 => "Trunk",
        0x08 => "Panic",
        _ => "Unknown",
    }
}

/// Button code constants
#[allow(dead_code)]
pub mod buttons {
    pub const LOCK: u8 = 0x01;
    pub const UNLOCK: u8 = 0x02;
    pub const TRUNK: u8 = 0x04;
    pub const PANIC: u8 = 0x08;
}

#[cfg(test)]
mod tests {
    use super::ManchesterEvent::{LongHigh, LongLow, Reset, ShortHigh, ShortLow};
    use super::ManchesterState::{Mid0, Mid1, Start0, Start1};
    use super::*;

    const ALL_STATES: [ManchesterState; 4] = [Start1, Mid1, Mid0, Start0];
    const DATA_EVENTS: [ManchesterEvent; 4] = [ShortLow, ShortHigh, LongLow, LongHigh];

    /// Line-for-line transcription of manchester_decoder.c, independent of the implementation's
    /// constant and enum conversions: `new = transitions[state] >> event & 0x3`, then
    /// `new == state` → Mid1 (reset), `new == Mid0` → 0, `new == Mid1` → 1, else no bit.
    fn c_reference(state: u8, event: u8) -> (u8, Option<bool>) {
        const TRANSITIONS: [u8; 4] = [0b00000001, 0b10010001, 0b10011011, 0b11111011];
        if event == 8 {
            return (1, None);
        }
        let new_state = TRANSITIONS[state as usize] >> event & 0x3;
        if new_state == state {
            (1, None)
        } else if new_state == 2 {
            (new_state, Some(false))
        } else if new_state == 1 {
            (new_state, Some(true))
        } else {
            (new_state, None)
        }
    }

    /// Every state × event, written out from the C table (see the table in the comment above
    /// `ManchesterState`). This is the regression guard against hand-written look-alike tables.
    #[test]
    fn manchester_table_matches_flipper_c() {
        let expected: [(ManchesterState, ManchesterEvent, ManchesterState, Option<bool>); 16] = [
            (Start1, ShortLow, Mid1, Some(true)),
            (Start1, ShortHigh, Mid1, None),
            (Start1, LongLow, Mid1, None),
            (Start1, LongHigh, Mid1, None),
            (Mid1, ShortLow, Mid1, None),
            (Mid1, ShortHigh, Start1, None),
            (Mid1, LongLow, Mid1, None),
            (Mid1, LongHigh, Mid0, Some(false)),
            (Mid0, ShortLow, Start0, None),
            (Mid0, ShortHigh, Mid1, None),
            (Mid0, LongLow, Mid1, Some(true)),
            (Mid0, LongHigh, Mid1, None),
            (Start0, ShortLow, Mid1, None),
            (Start0, ShortHigh, Mid0, Some(false)),
            (Start0, LongLow, Mid1, None),
            (Start0, LongHigh, Mid1, None),
        ];
        for (state, event, next, bit) in expected {
            assert_eq!(
                manchester_advance(state, event),
                (next, bit),
                "{state:?} + {event:?}"
            );
            // The written-out table must also agree with the raw C bytes.
            let (c_next, c_bit) = c_reference(state as u8, event as u8);
            assert_eq!((c_next, c_bit), (next as u8, bit), "C bytes: {state:?} + {event:?}");
        }
        for state in ALL_STATES {
            assert_eq!(manchester_advance(state, Reset), (Mid1, None), "{state:?} + Reset");
        }
    }

    /// Enum discriminants are the C values (states 0..3, events = shift amounts 0/2/4/6, Reset 8).
    #[test]
    fn manchester_enums_match_c_values() {
        assert_eq!(
            ALL_STATES.map(|s| s as u8),
            [0, 1, 2, 3],
            "ManchesterStateStart1/Mid1/Mid0/Start0"
        );
        assert_eq!(DATA_EVENTS.map(|e| e as u8), [0, 2, 4, 6]);
        assert_eq!(Reset as u8, 8);
        for (i, high, long) in [(0, false, false), (1, true, false), (2, false, true), (3, true, true)] {
            assert_eq!(ManchesterEvent::from_half_bit(high, long), DATA_EVENTS[i]);
        }
    }

    /// `ManchesterState::advance` is the in-place form of `manchester_advance`.
    #[test]
    fn manchester_advance_in_place_matches_pure_form() {
        for state in ALL_STATES {
            for event in DATA_EVENTS.into_iter().chain([Reset]) {
                let mut s = state;
                let bit = s.advance(event);
                assert_eq!((s, bit), manchester_advance(state, event));
            }
        }
    }

    /// xorshift64* — a tiny deterministic PRNG so the property test needs no extra crates.
    struct XorShift64(u64);

    impl XorShift64 {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 >> 12;
            self.0 ^= self.0 << 25;
            self.0 ^= self.0 >> 27;
            self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
        }
    }

    /// Independent Manchester encoder in Flipper's convention (1 = Low half then High half,
    /// 0 = High then Low): expand `bits` into half-bits (after an optional `lead` half), merge
    /// equal neighbours into Long events and emit one event per pulse.
    fn encode_events(lead: Option<bool>, bits: &[bool]) -> Vec<ManchesterEvent> {
        let mut halves: Vec<bool> = lead.into_iter().collect();
        for &bit in bits {
            halves.extend(if bit { [false, true] } else { [true, false] });
        }
        let mut events = Vec::new();
        let mut i = 0;
        while i < halves.len() {
            let run = if halves.get(i + 1) == Some(&halves[i]) { 2 } else { 1 };
            assert!(halves.get(i + run) != Some(&halves[i]), "Manchester never has 3 equal halves");
            events.push(ManchesterEvent::from_half_bit(halves[i], run == 2));
            i += run;
        }
        events
    }

    fn decode_events(start: ManchesterState, events: &[ManchesterEvent]) -> Vec<bool> {
        let mut state = start;
        events.iter().filter_map(|&event| state.advance(event)).collect()
    }

    /// Property: random bit streams survive encode→decode. From the Mid1 reset state the stream is
    /// led by the High half that ends a virtual 1 bit (that is what Mid1 means); from Start1/Start0
    /// it needs no lead-in. The hand-written table this replaces decoded none of these.
    #[test]
    fn manchester_encode_decode_random_streams() {
        let mut rng = XorShift64(0x9E37_79B9_7F4A_7C15);
        for _ in 0..500 {
            let len = 1 + (rng.next() % 128) as usize;
            let bits: Vec<bool> = (0..len).map(|_| rng.next() & 1 == 1).collect();

            let from_mid1 = decode_events(Mid1, &encode_events(Some(true), &bits));
            assert_eq!(from_mid1, bits, "from Mid1");

            let start = if bits[0] { Start1 } else { Start0 };
            let from_start = decode_events(start, &encode_events(None, &bits));
            assert_eq!(from_start, bits, "from {start:?}");
        }
    }

    /// A pulse that does not fit the stream (e.g. a glitch) costs at most the bits around it:
    /// the decoder falls back to Mid1 and re-locks, never emitting more bits than were sent.
    #[test]
    fn manchester_resyncs_after_error() {
        let bits = [true, false, false, true, true, false, true, false, false, false, true, true];
        let mut events = encode_events(Some(true), &bits);
        events.insert(5, ShortHigh); // two High halves in a row: impossible in Manchester
        let decoded = decode_events(Mid1, &events);
        assert!(decoded.len() <= bits.len());
        assert_eq!(&decoded[decoded.len() - 4..], &bits[bits.len() - 4..], "re-locked by the tail");
    }
}
