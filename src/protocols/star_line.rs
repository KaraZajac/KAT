//! Star Line protocol decoder/encoder
//!
//! References: ProtoPirate `REFERENCES/ProtoPirate/protocols/star_line.c` (primary) and Flipper-ARF
//! `lib/subghz/protocols/star_line.c` (same decoder and key search; adds 315 MHz, the rolling-counter
//! multiplier and button remapping in the encoder). doctor-firmware carries the plain Flipper copy.
//!
//! **Air format**: PWM, te_short 250 µs (bit 0 = 250 HIGH + 250 LOW), te_long 500 µs (bit 1 = 500
//! HIGH + 500 LOW), te_delta 120 µs. Header: > 4 pairs of 1000 ± 240 µs HIGH/LOW; the first non-header
//! HIGH starts the data. A HIGH ≥ 620 µs (te_long + te_delta, i.e. the next header pulse) ends the
//! frame, which is reported when it holds 64..66 bits (only the first 64 are kept). Every such frame is
//! reported, as in the C; `crc_valid` says whether a keystore key validated it. (The C also drops a
//! frame identical to the previous one even across receiver resets; KAT reports every repeat.)
//!
//! **Frame**: 64 bits sent MSB-first; bit-reversed it is `fix(32) | hop(32)`; fix = button (8 bits) <<
//! 24 | serial (24 bits); hop = KeeLoq(button << 24 | serial[7:0] << 16 | counter).
//!
//! **Key search** (`subghz_protocol_star_line_check_remote_controller_selector`): every keystore entry
//! in order ([`keeloq_keystore`]); SIMPLE (type 1) entries decrypt with the key, NORMAL (2) with the
//! normal-learning key for `fix`, UNKNOWN (0) entries try simple, simple with the byte-mirrored key,
//! normal, normal with the mirrored key. Other learning types are skipped. Accepted when the
//! plaintext's top byte equals the button AND bits 23..16 equal `fix & 0xFF` (16-bit check). The
//! first match names the manufacturer (shown as `Star Line [<name>]`) and gives the counter; with no
//! match the counter is 0 (C "MF:Unknown").
//!
//! **Encoder** (`subghz_protocol_star_line_gen_data` + `get_upload`): the manufacturer is re-derived by
//! the same key search (the C re-runs it on the saved key with the saved "Manufacture"). Counter + 1
//! (0 once it would pass 0xFFFF); `fix = button << 24 | serial`; plaintext `button << 24 |
//! serial[7:0] << 16 | counter`; SIMPLE → encrypt with the key, NORMAL → with the normal-learning key,
//! UNKNOWN → per the matched mode (simple or normal) but always with the key as stored — the C does
//! not use the mirrored key here, so a remote matched via the mirrored key gets a hop its receiver
//! will not accept (reference behaviour, kept). No manufacturer → the captured hop is replayed under
//! the new fix ("Unknown"). A zero hop makes the C refuse to build an upload (`None` here).
//! Upload: 6 × (1000 HIGH, 1000 LOW) then the 64 bits; the C yields it 40 times (`encoder.repeat`),
//! so the KAT waveform holds 40 copies (each copy's first header pulse terminates the previous copy).
//!
//! **Buttons** (ARF `star_line_btn_name`): 0x01 Lock, 0x02 Unlock, 0x03 Trunk, 0x04 Panic; "2-way"
//! remotes (bit 5 set): 0x21 Lock, 0x22 Unlock, 0x23 Trunk, 0x24 Start, 0x25 Stop, 0x26 Extra.
//! KAT's generic Lock/Unlock/Trunk/Panic map like ARF's d-pad (`star_line_custom_to_btn`): 0x01/0x02/
//! 0x03/0x04, OR 0x20 when the captured button is a 2-way one; native 0x03 and 0x21..0x26 pass
//! through; anything else keeps the captured button.
//!
//! **Key files** (`subghz_protocol_decoder_star_line_serialize` / `_deserialize`, identical in
//! ProtoPirate and Flipper-ARF): `Protocol: Star Line`, `Bit: 64`, `Key`, then `Serial`, `Btn`,
//! `Cnt` (decimal) and `Manufacture` (the matched keystore name or "Unknown"). Loading keeps the
//! C's semantics: Serial/Btn come from the key; a named manufacturer restricts the key search to
//! that name (no match → "Unknown", counter 0), "Unknown" keeps the file's Cnt without searching,
//! and a file without the line is searched like a frame on air. `Key` is one 16-digit word, as both
//! references write it (`%016llX`). ProtoPirate writes the preset by its short name ("AM650"); KAT
//! writes the full Flipper name, which every reference reads.

use super::keeloq_common::{
    keeloq_decrypt, keeloq_encrypt, keeloq_keystore, keeloq_normal_learning, learning_types,
    mirrored_key, reverse_key, MfKey,
};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`); "StarLine" is ProtoPirate's catalog alias.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Star Line",
    decoder: "Star Line",
    aliases: &["StarLine"],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "KeeLoq",
    make: "Star Line",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// Flipper-ARF star_line_btn_name (0x2x = 2-way remotes).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x01 | 0x21 => "Lock",
        0x02 | 0x22 => "Unlock",
        0x03 | 0x23 => "Trunk",
        0x04 => "Panic",
        0x24 => "Start",
        0x25 => "Stop",
        0x26 => "Extra",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 120;
const MIN_COUNT_BIT: usize = 64;
const HEADER_DURATION: u32 = 1000; // te_long * 2
/// C `encoder.repeat`.
const TX_REPEAT: usize = 40;

// KAT generic button codes.
const BTN_LOCK: u8 = 0x01;
const BTN_UNLOCK: u8 = 0x02;
const BTN_TRUNK: u8 = 0x04;
const BTN_PANIC: u8 = 0x08;

/// How a keystore entry validated a frame (the C's `kl_type` for UNKNOWN entries).
#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
    Simple,
    Normal,
}

/// A successful key search: keystore entry, learning mode, counter.
struct KeyMatch {
    entry: &'static MfKey,
    mode: Mode,
    cnt: u16,
}

/// `subghz_protocol_star_line_check_remote_controller_selector` over the keystore.
fn find_key(fix: u32, hop: u32) -> Option<KeyMatch> {
    find_key_where(fix, hop, |_| true)
}

/// The selector over the keystore entries `keep` accepts (the C with `keystore->mfname` set to a
/// key file's "Manufacture" tries only that manufacturer's entries).
fn find_key_where(fix: u32, hop: u32, keep: impl Fn(&MfKey) -> bool) -> Option<KeyMatch> {
    let end_serial = fix & 0xFF;
    let btn = fix >> 24;
    let check = |decrypt: u32| decrypt >> 24 == btn && (decrypt >> 16) & 0xFF == end_serial;
    for entry in keeloq_keystore().iter().filter(|e| keep(e)) {
        let mut attempts: [(u64, Mode); 4] = [(0, Mode::Simple); 4];
        let n = match entry.learning {
            learning_types::SIMPLE => {
                attempts[0] = (entry.key, Mode::Simple);
                1
            }
            learning_types::NORMAL => {
                attempts[0] = (keeloq_normal_learning(fix, entry.key), Mode::Normal);
                1
            }
            learning_types::UNKNOWN => {
                let rev = mirrored_key(entry.key);
                attempts = [
                    (entry.key, Mode::Simple),
                    (rev, Mode::Simple),
                    (keeloq_normal_learning(fix, entry.key), Mode::Normal),
                    (keeloq_normal_learning(fix, rev), Mode::Normal),
                ];
                4
            }
            _ => 0,
        };
        for &(key, mode) in &attempts[..n] {
            let decrypt = keeloq_decrypt(hop, key);
            if check(decrypt) {
                return Some(KeyMatch {
                    entry,
                    mode,
                    cnt: decrypt as u16,
                });
            }
        }
    }
    None
}

/// Split a received 64-bit key into (fix, hop) (`subghz_protocol_blocks_reverse_key`).
fn fix_hop(data: u64) -> (u32, u32) {
    let reversed = reverse_key(data, MIN_COUNT_BIT);
    ((reversed >> 32) as u32, reversed as u32)
}

/// KAT generic button → Star Line code (see the module doc).
fn map_button(button: u8, original: u8) -> u8 {
    let twage = original & 0x20;
    match button {
        BTN_LOCK => 0x01 | twage,
        BTN_UNLOCK => 0x02 | twage,
        BTN_TRUNK => 0x03 | twage,
        BTN_PANIC => 0x04 | twage,
        0x03 | 0x21..=0x26 => button,
        _ => original,
    }
}

/// Decoder states (matches StarLineDecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckPreamble,
    SaveDuration,
    CheckDuration,
}

/// Star Line protocol decoder
pub struct StarLineDecoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    decode_data: u64,
    decode_count_bit: usize,
}

impl StarLineDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    /// `subghz_protocol_star_line_check_remote_controller` → the reported signal.
    fn parse_data(data: u64) -> DecodedSignal {
        let (fix, hop) = fix_hop(data);
        Self::signal_for(data, find_key(fix, hop), 0)
    }

    /// The reported signal for `data` and its key-search result; `unknown_cnt` is the counter
    /// shown without a match (0, or a key file's Cnt under "Manufacture: Unknown").
    fn signal_for(data: u64, found: Option<KeyMatch>, unknown_cnt: u16) -> DecodedSignal {
        let (fix, _) = fix_hop(data);
        DecodedSignal {
            serial: Some(fix & 0x00FF_FFFF),
            button: Some((fix >> 24) as u8),
            counter: Some(found.as_ref().map_or(unknown_cnt, |m| m.cnt)),
            crc_valid: found.is_some(),
            data,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: found.map(|m| format!("Star Line [{}]", m.entry.name)),
        }
    }

    /// The 64-bit key the C encoder sends for `button` (None when it refuses: zero hop).
    fn encode_key(decoded: &DecodedSignal, button: u8) -> Option<u64> {
        let (fix_c, hop_c) = fix_hop(decoded.data);
        let serial = fix_c & 0x00FF_FFFF;
        let found = find_key(fix_c, hop_c);
        // Counter base: the decrypted counter (C check_remote_controller), or the saved one when
        // the manufacturer is unknown (0 for a KAT capture).
        let cnt_base = match &found {
            Some(m) => m.cnt,
            None => decoded.counter.unwrap_or(0),
        };
        let cnt = cnt_base.checked_add(1).unwrap_or(0);
        let btn = u32::from(map_button(button, (fix_c >> 24) as u8));
        let fix = (btn << 24) | serial;
        let plain = (btn << 24) | ((serial & 0xFF) << 16) | u32::from(cnt);
        let hop = match found {
            None => hop_c,
            Some(m) => match (m.entry.learning, m.mode) {
                (learning_types::NORMAL, _) | (learning_types::UNKNOWN, Mode::Normal) => {
                    keeloq_encrypt(plain, keeloq_normal_learning(fix, m.entry.key))
                }
                _ => keeloq_encrypt(plain, m.entry.key),
            },
        };
        if hop == 0 {
            return None;
        }
        Some(reverse_key(
            (u64::from(fix) << 32) | u64::from(hop),
            MIN_COUNT_BIT,
        ))
    }
}

/// Collect 64-bit Star Line payload from level+duration pairs (for keeloq_generic fallback).
pub fn collect_star_line_bits(
    pairs: &[LevelDuration],
    invert_level: bool,
) -> Option<u64> {
    let mut step = DecoderStep::Reset;
    let mut header_count = 0u16;
    let mut decode_data = 0u64;
    let mut decode_count_bit = 0usize;
    let mut te_last = 0u32;

    for pair in pairs {
        let level = if invert_level { !pair.level } else { pair.level };
        let duration = pair.duration_us;

        match step {
            DecoderStep::Reset => {
                if level {
                    if duration_diff!(duration, HEADER_DURATION) < TE_DELTA * 2 {
                        step = DecoderStep::CheckPreamble;
                        header_count += 1;
                    } else if header_count > 4 {
                        decode_data = 0;
                        decode_count_bit = 0;
                        te_last = duration;
                        step = DecoderStep::CheckDuration;
                    }
                } else {
                    header_count = 0;
                }
            }
            DecoderStep::CheckPreamble => {
                if !level && duration_diff!(duration, HEADER_DURATION) < TE_DELTA * 2 {
                    step = DecoderStep::Reset;
                } else {
                    header_count = 0;
                    step = DecoderStep::Reset;
                }
            }
            DecoderStep::SaveDuration => {
                if level {
                    if duration >= (TE_LONG + TE_DELTA) {
                        step = DecoderStep::Reset;
                        if (MIN_COUNT_BIT..=MIN_COUNT_BIT + 2).contains(&decode_count_bit) {
                            return Some(decode_data);
                        }
                        decode_data = 0;
                        decode_count_bit = 0;
                        header_count = 0;
                    } else {
                        te_last = duration;
                        step = DecoderStep::CheckDuration;
                    }
                } else {
                    step = DecoderStep::Reset;
                }
            }
            DecoderStep::CheckDuration => {
                if !level {
                    if duration_diff!(te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        if decode_count_bit < MIN_COUNT_BIT {
                            decode_data <<= 1;
                            decode_count_bit += 1;
                        } else {
                            decode_count_bit += 1;
                        }
                        step = DecoderStep::SaveDuration;
                    } else if duration_diff!(te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA
                    {
                        if decode_count_bit < MIN_COUNT_BIT {
                            decode_data = (decode_data << 1) | 1;
                            decode_count_bit += 1;
                        } else {
                            decode_count_bit += 1;
                        }
                        step = DecoderStep::SaveDuration;
                    } else {
                        step = DecoderStep::Reset;
                    }
                } else {
                    step = DecoderStep::Reset;
                }
            }
        }
    }
    None
}

impl ProtocolDecoder for StarLineDecoder {
    fn name(&self) -> &'static str {
        "Star Line"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: MIN_COUNT_BIT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.header_count = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level {
                    if duration_diff!(duration, HEADER_DURATION) < TE_DELTA * 2 {
                        self.step = DecoderStep::CheckPreamble;
                        self.header_count = self.header_count.wrapping_add(1);
                    } else if self.header_count > 4 {
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.te_last = duration;
                        self.step = DecoderStep::CheckDuration;
                    }
                } else {
                    self.header_count = 0;
                }
            }

            DecoderStep::CheckPreamble => {
                if !level && duration_diff!(duration, HEADER_DURATION) < TE_DELTA * 2 {
                    // Found preamble pair
                    self.step = DecoderStep::Reset;
                } else {
                    self.header_count = 0;
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::SaveDuration => {
                if level {
                    if duration >= (TE_LONG + TE_DELTA) {
                        // End of data - check if we have enough bits
                        self.step = DecoderStep::Reset;
                        let result = (self.decode_count_bit >= MIN_COUNT_BIT
                            && self.decode_count_bit <= MIN_COUNT_BIT + 2)
                            .then(|| Self::parse_data(self.decode_data));
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.header_count = 0;
                        return result;
                    } else {
                        self.te_last = duration;
                        self.step = DecoderStep::CheckDuration;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::CheckDuration => {
                if !level {
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        // Bit 0: short HIGH + short LOW
                        if self.decode_count_bit < MIN_COUNT_BIT {
                            self.decode_data <<= 1;
                        }
                        self.decode_count_bit += 1;
                        self.step = DecoderStep::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA
                    {
                        // Bit 1: long HIGH + long LOW
                        if self.decode_count_bit < MIN_COUNT_BIT {
                            self.decode_data = (self.decode_data << 1) | 1;
                        }
                        self.decode_count_bit += 1;
                        self.step = DecoderStep::SaveDuration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let data = Self::encode_key(decoded, button)?;
        let mut signal = Vec::with_capacity(TX_REPEAT * (12 + MIN_COUNT_BIT * 2));
        for _ in 0..TX_REPEAT {
            // Header: 6 pairs of te_long * 2 HIGH + LOW.
            for _ in 0..6 {
                signal.push(LevelDuration::new(true, HEADER_DURATION));
                signal.push(LevelDuration::new(false, HEADER_DURATION));
            }
            // Data: 64 bits MSB first (1 = long/long, 0 = short/short).
            for bit in (0..MIN_COUNT_BIT).rev() {
                let te = if (data >> bit) & 1 == 1 { TE_LONG } else { TE_SHORT };
                signal.push(LevelDuration::new(true, te));
                signal.push(LevelDuration::new(false, te));
            }
        }
        Some(signal)
    }

    /// `subghz_protocol_decoder_star_line_deserialize` (ProtoPirate; Flipper-ARF's is the same
    /// with optional Serial/Btn/Cnt) + the `check_remote_controller` its display runs. The key must
    /// be 16 hex digits and non-zero; the frame is always 64 bits (the C ignores Bit's value).
    /// "Manufacture" restricts the key search to that keystore name (no match → the C's "Unknown",
    /// counter 0); "Manufacture: Unknown" skips the search and keeps the file's Cnt; without the
    /// line the whole keystore is searched, as on air. Serial and button always come from the key.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let data = key.key_u64().filter(|&k| k != 0)?;
        let (fix, hop) = fix_hop(data);
        Some(match key.field("Manufacture").map(str::trim) {
            None | Some("") => Self::parse_data(data),
            Some("Unknown") => {
                Self::signal_for(data, None, key.u32_field("Cnt").unwrap_or(0) as u16)
            }
            Some(mf) => Self::signal_for(data, find_key_where(fix, hop, |e| e.name == mf), 0),
        })
    }

    /// `subghz_protocol_decoder_star_line_serialize` (ProtoPirate = Flipper-ARF): Protocol
    /// "Star Line", Bit 64, Key, then Serial, Btn, Cnt (decimal) and Manufacture (the decode's
    /// `[<manufacturer>]`, or "Unknown"). Key is one 16-digit word, as the C writes it.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let (fix, hop) = fix_hop(decoded.data);
        let named = decoded
            .protocol_display_name
            .as_deref()
            .and_then(|n| n.strip_prefix("Star Line ["))
            .and_then(|n| n.strip_suffix(']'));
        // The C re-runs the search with the saved manufacturer (the whole keystore for a decode).
        let found = match named {
            Some(mf) => find_key_where(fix, hop, |e| e.name == mf),
            None if decoded.crc_valid => find_key(fix, hop),
            None => None,
        };
        let cnt = found.as_ref().map_or(decoded.counter.unwrap_or(0), |m| m.cnt);
        let mut key = FlipperKey::for_protocol("Star Line", MIN_COUNT_BIT, decoded.data);
        key.key_word = true;
        key.push_u32("Serial", fix & 0x00FF_FFFF);
        key.push_u32("Btn", fix >> 24);
        key.push_u32("Cnt", cnt.into());
        key.push("Manufacture", found.map_or("Unknown", |m| m.entry.name.as_str()));
        Some(key)
    }
}

impl Default for StarLineDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// C-oracle test vector for key files (used by the KeeLoq-family and PSA / Renault V1 tests):
/// `<dir>/<name>.sub`, a Flipper RAW file of `upload` sent twice (20 ms LOW before, between and
/// after; adjacent equal levels merged, ≤ 512 values per line), and `<dir>/<name>.kat.txt`, KAT's
/// key file for the decode of that upload, with the same Frequency / Preset.
#[cfg(test)]
pub(super) fn write_oracle_vector(
    dir: &std::path::Path,
    name: &str,
    frequency: u32,
    preset: &str,
    upload: &[LevelDuration],
    key: &FlipperKey,
) {
    crate::protocols::flipper_key::testing::write_vector(dir, name, frequency, preset, upload, key);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = StarLineDecoder::new();
        let mut out = Vec::new();
        for ld in signal {
            if let Some(d) = dec.feed(ld.level, ld.duration_us) {
                out.push(d);
                dec.reset(); // registry behaviour
            }
        }
        out
    }

    fn entry(name: &str) -> &'static MfKey {
        keeloq_keystore()
            .iter()
            .find(|e| e.name == name)
            .unwrap_or_else(|| panic!("keystore entry {name}"))
    }

    /// A captured frame for (key mode, button, serial, counter), built like a real remote.
    fn frame(e: &MfKey, mode: Mode, mirrored: bool, btn: u8, serial: u32, cnt: u16) -> u64 {
        let key = if mirrored { mirrored_key(e.key) } else { e.key };
        let fix = (u32::from(btn) << 24) | (serial & 0xFF_FFFF);
        let plain = (u32::from(btn) << 24) | ((serial & 0xFF) << 16) | u32::from(cnt);
        let hop = match mode {
            Mode::Simple => keeloq_encrypt(plain, key),
            Mode::Normal => keeloq_encrypt(plain, keeloq_normal_learning(fix, key)),
        };
        reverse_key((u64::from(fix) << 32) | u64::from(hop), 64)
    }

    /// One header + frame + a terminating header pulse.
    fn capture(data: u64) -> Vec<LevelDuration> {
        let mut s = Vec::new();
        for _ in 0..6 {
            s.push(LevelDuration::new(true, 1000));
            s.push(LevelDuration::new(false, 1000));
        }
        for bit in (0..64).rev() {
            let te = if (data >> bit) & 1 == 1 { 500 } else { 250 };
            s.push(LevelDuration::new(true, te));
            s.push(LevelDuration::new(false, te));
        }
        s.push(LevelDuration::new(true, 1000));
        s
    }

    #[test]
    fn star_line_keystore_search_simple_normal_unknown() {
        // SIMPLE entry (SL_A2-A4), NORMAL entry (Mongoose), UNKNOWN entry (Teco) in all 4 modes.
        let cases = [
            ("SL_A2-A4", Mode::Simple, false),
            ("Mongoose", Mode::Normal, false),
            ("Teco", Mode::Simple, false),
            ("Teco", Mode::Simple, true),
            ("Teco", Mode::Normal, false),
            ("Teco", Mode::Normal, true),
        ];
        for (name, mode, mirrored) in cases {
            let data = frame(entry(name), mode, mirrored, 0x22, 0x00AB_CDEF, 0x1234);
            let d = decode_all(&capture(data));
            assert_eq!(d.len(), 1, "{name} {mode:?} {mirrored}");
            let d = &d[0];
            assert!(d.crc_valid, "{name} {mode:?} {mirrored}");
            assert_eq!(d.serial, Some(0x00AB_CDEF));
            assert_eq!(d.button, Some(0x22));
            assert_eq!(d.counter, Some(0x1234));
            assert_eq!(
                d.protocol_display_name.as_deref(),
                Some(format!("Star Line [{name}]").as_str())
            );
        }
    }

    #[test]
    fn star_line_unknown_manufacturer_still_reported() {
        let d = decode_all(&capture(0x0123_4567_89AB_CDEF));
        assert_eq!(d.len(), 1);
        let d = &d[0];
        if !d.crc_valid {
            assert_eq!(d.counter, Some(0));
            assert_eq!(d.protocol_display_name, None);
        }
    }

    #[test]
    fn star_line_encode_roundtrip_and_buttons() {
        let enc = StarLineDecoder::new();
        for (name, mode, twage) in [
            ("SL_A6-A9/Tomahawk_9010", Mode::Simple, false),
            ("Guard_RF-311A", Mode::Normal, true),
            ("Leopard", Mode::Normal, false),
        ] {
            let orig_btn = if twage { 0x21 } else { 0x01 };
            let data = frame(entry(name), mode, false, orig_btn, 0x0012_3456, 0x00FF);
            let captured = decode_all(&capture(data)).pop().unwrap();
            assert!(captured.crc_valid, "{name}");
            let tw = if twage { 0x20 } else { 0 };
            for (button, code) in [
                (BTN_LOCK, 0x01 | tw),
                (BTN_UNLOCK, 0x02 | tw),
                (BTN_TRUNK, 0x03 | tw),
                (BTN_PANIC, 0x04 | tw),
                (0x25, 0x25),
                (0x10, orig_btn),
            ] {
                let signal = enc.encode(&captured, button).expect("encode");
                assert_eq!(signal.len(), TX_REPEAT * (12 + 128));
                // Each copy is terminated by the next copy's header; the last one is not.
                let d = decode_all(&signal);
                assert_eq!(d.len(), TX_REPEAT - 1, "{name}");
                let d = &d[0];
                assert!(d.crc_valid, "{name} button {button:#x}");
                assert_eq!(d.serial, Some(0x0012_3456));
                assert_eq!(d.button, Some(code), "{name} button {button:#x}");
                assert_eq!(d.counter, Some(0x0100));
                assert_eq!(
                    d.protocol_display_name.as_deref(),
                    Some(format!("Star Line [{name}]").as_str())
                );
            }
        }
    }

    #[test]
    fn star_line_encoder_unknown_replays_hop_and_wraps_counter() {
        let enc = StarLineDecoder::new();
        let data = 0x0123_4567_89AB_CDEF;
        let captured = decode_all(&capture(data)).pop().unwrap();
        if !captured.crc_valid {
            let key = StarLineDecoder::encode_key(&captured, BTN_LOCK).unwrap();
            let (fix, hop) = fix_hop(key);
            let (fix0, hop0) = fix_hop(data);
            assert_eq!(hop, hop0, "unknown manufacturer: hop replayed");
            // Lock keeps the captured button's 2-way bit (0x20), as ARF's d-pad mapping does.
            assert_eq!(fix, ((0x01 | (fix0 >> 24) & 0x20) << 24) | (fix0 & 0xFF_FFFF));
            assert!(enc.encode(&captured, BTN_LOCK).is_some());
        }
        // Counter wraps to 0 past 0xFFFF.
        let data = frame(entry("Pantera"), Mode::Simple, false, 0x02, 0x000A_0B0C, 0xFFFF);
        let captured = decode_all(&capture(data)).pop().unwrap();
        let d = decode_all(&enc.encode(&captured, BTN_UNLOCK).unwrap());
        assert_eq!(d[0].counter, Some(0));
    }

    #[test]
    fn star_line_mirrored_unknown_key_encodes_with_stored_key() {
        // Reference quirk: a remote matched via the mirrored key of an UNKNOWN entry is re-encoded
        // with the key as stored, so the new hop no longer validates under the mirrored key.
        let e = entry("Faraon");
        let data = frame(e, Mode::Simple, true, 0x01, 0x00C0_FFEE, 0x0010);
        let captured = decode_all(&capture(data)).pop().unwrap();
        assert_eq!(captured.protocol_display_name.as_deref(), Some("Star Line [Faraon]"));
        let key = StarLineDecoder::encode_key(&captured, BTN_LOCK).unwrap();
        let (fix, hop) = fix_hop(key);
        let plain = (1u32 << 24) | (0xEE << 16) | 0x0011;
        assert_eq!(fix, (1 << 24) | 0x00C0_FFEE);
        assert_eq!(hop, keeloq_encrypt(plain, e.key));
    }

    /// "0123456789ABCDEF": the key as the C writes it (`%016llX`).
    fn word(data: u64) -> String {
        format!("{data:016X}")
    }

    /// Key files (ProtoPirate = Flipper-ARF `subghz_protocol_decoder_star_line_serialize` /
    /// `_deserialize` + `check_remote_controller`): identified and unidentified decodes round-trip
    /// with the C's lines, a file the C wrote (one-word Key, short preset name) loads, the
    /// Manufacture line restricts the key search like the C, and TX from a loaded key decodes back.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = StarLineDecoder::new();
        for (name, mode, mirrored) in [
            ("SL_A2-A4", Mode::Simple, false),
            ("Mongoose", Mode::Normal, false),
            ("Teco", Mode::Normal, true),
        ] {
            let data = frame(entry(name), mode, mirrored, 0x22, 0x00AB_CDEF, 0x1234);
            let d = decode_all(&capture(data)).pop().unwrap();
            assert!(d.crc_valid, "{name}");
            let text = assert_round_trip(&dec, &d);
            let tail = format!(
                "Protocol: Star Line\nBit: 64\nKey: {}\nSerial: 11259375\nBtn: 34\nCnt: 4660\nManufacture: {name}\n",
                word(data)
            );
            assert!(text.ends_with(&tail), "{text}");
        }
        // An unidentified frame: "Manufacture: Unknown", Cnt 0 (C `instance->cnt = 0`).
        let unknown = decode_all(&capture(0x0123_4567_89AB_CDEF)).pop().unwrap();
        if !unknown.crc_valid {
            let text = assert_round_trip(&dec, &unknown);
            assert!(text.ends_with("Cnt: 0\nManufacture: Unknown\n"), "{text}");
        }

        // A file as ProtoPirate writes it (+ its storage header): one-word Key, short preset.
        let data = frame(entry("SL_A2-A4"), Mode::Simple, false, 0x01, 0x0012_3456, 0x00FF);
        let pp = format!(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\nPreset: AM650\n\
             Protocol: Star Line\nBit: 64\nKey: {data:016X}\nSerial: 1193046\nBtn: 1\nCnt: 255\n\
             Manufacture: SL_A2-A4\n"
        );
        let loaded = dec.from_flipper_key(&key(&pp)).unwrap();
        assert_eq!(loaded, decode_all(&capture(data)).pop().unwrap());
        assert_eq!(loaded.protocol_display_name.as_deref(), Some("Star Line [SL_A2-A4]"));
        assert_eq!((loaded.serial, loaded.button, loaded.counter), (Some(0x12_3456), Some(0x01), Some(0x00FF)));
        // TX from the loaded key: counter + 1, same remote.
        let tx = decode_all(&dec.encode(&loaded, BTN_UNLOCK).unwrap());
        assert_eq!(tx[0].protocol_display_name.as_deref(), Some("Star Line [SL_A2-A4]"));
        assert_eq!((tx[0].serial, tx[0].button, tx[0].counter), (Some(0x12_3456), Some(0x02), Some(0x0100)));

        // Another manufacturer named in the file: the C searches only its entries → Unknown.
        let wrong = dec.from_flipper_key(&key(&pp.replace("SL_A2-A4", "Pantera"))).unwrap();
        assert_eq!((wrong.crc_valid, wrong.counter, wrong.protocol_display_name), (false, Some(0), None));
        // "Unknown": no search, the file's Cnt is shown.
        let unk = dec.from_flipper_key(&key(&pp.replace("SL_A2-A4", "Unknown"))).unwrap();
        assert_eq!((unk.crc_valid, unk.counter, unk.serial), (false, Some(255), Some(0x12_3456)));
        // No Manufacture line: searched like a frame on air.
        let bare = pp.replace("Manufacture: SL_A2-A4\n", "");
        assert_eq!(dec.from_flipper_key(&key(&bare)).unwrap(), loaded);
        // The C rejects a zero key.
        let zero = pp.replace(&format!("{data:016X}"), "0000000000000000");
        assert!(dec.from_flipper_key(&key(&zero)).is_none());
    }

    /// `KAT_KEY_VECTORS=<dir> cargo test star_line_oracle_vectors -- --ignored`: RAW uploads and
    /// KAT's key files for the C oracle (`decode <raw.sub>` prints the reference's lines).
    #[test]
    #[ignore]
    fn star_line_oracle_vectors() {
        let Some(dir) = std::env::var_os("KAT_KEY_VECTORS") else { return };
        let dir = std::path::PathBuf::from(dir);
        let dec = StarLineDecoder::new();
        let cases = [
            frame(entry("SL_A2-A4"), Mode::Simple, false, 0x01, 0x0012_3456, 0x00FF),
            frame(entry("Mongoose"), Mode::Normal, false, 0x22, 0x00AB_CDEF, 0x1234),
        ];
        for (i, data) in cases.into_iter().enumerate() {
            let src = decode_all(&capture(data)).pop().unwrap();
            let upload = dec.encode(&src, BTN_LOCK).unwrap();
            let d = decode_all(&upload).pop().unwrap();
            let key = dec.to_flipper_key(&d).unwrap();
            write_oracle_vector(&dir, &format!("star_line_{}", i + 1), 433_920_000, "FuriHalSubGhzPresetOok650Async", &upload, &key);
        }
    }
}
