//! PSA 2-Button: a two-button Valeo flip key for Citroën and Peugeot with a 76-bit frame —
//! **decode, decipher and generate the next code**. The 64-bit rolling code is the same block cipher
//! as PSA's [older code](super::psa) under a 16-byte table of its own (recovered from one key's
//! presses); with the table KAT reads the press counter and makes the next valid code, otherwise it
//! replays the captured frame.
//!
//! **KAT's own decoder**, worked out from two recordings of one Citroën key that a tester sent in
//! (`IMPORTS/GROUPE PSA/citroen_two_button_valeo_key_unlock_x50.sub` and `..._lock_x50.sub`,
//! 433.9 MHz, on-off keying, 50 presses each). No reference reads the signal; the name is KAT's.
//! Not the 80-bit frame of [PSA](super::psa). See the report under `reports/2026-psa-2button-valeo`.
//!
//! **Air format**: Manchester (carrier then pause is a 1), 207 µs per half bit (2415 bit/s). A
//! press is about 78 ms of carrier, 6.9 ms of pause, then one frame: half a bit of carrier, the 76
//! bits, and 2.5 ms of carrier (twelve half bits).
//!
//! **Frame (76 bits)**:
//!
//! | Bits | Field |
//! |---|---|
//! | 8 | `CE` in every frame of the one key recorded |
//! | 4 | button: `C` Unlock, `3` Lock |
//! | 64 | the code: changes with every press |
//!
//! **The code** is the PSA older-code block cipher (four rounds of a byte mix and a popcount-driven
//! rotation; see [`super::psa`]) applied to the 64-bit code, under this key family's own 16-byte
//! table ([`crate::keystore::PSA_2BUTTON_TABLE`]). Deciphered, the eight bytes are a per-remote
//! constant, the 16-bit press counter (bytes 2 and 3), and two equal check bytes that are a fixed
//! function of the plaintext byte-sum ([`cipher::check_byte`]); the counter runs once across both
//! buttons, the button
//! itself being the clear nibble in the header. The table was recovered by differential
//! cryptanalysis of one key's 96 presses (`docs/psa_two_button.md`); whether it is shared across the
//! family is not established.
//!
//! **Gate**: a burst that begins behind a pause with the half bit of carrier, holds exactly 76
//! Manchester bits without a violation, and is ended by a carrier pulse of 2.0 to 3.3 ms. With the
//! table loaded the deciphered block's two check bytes are also verified. (Bytes 2 and 3 are the
//! counter, byte 2 the high byte; the check byte is a function of the plaintext byte-sum.)
//!
//! **Fields**: `data` = the code, `extra` = the 12 bits before it, `data_count_bit` = 76,
//! `button` = the button nibble, `counter` = the deciphered press counter (when the table reads it,
//! else none), `crc_valid` = the cipher verified, `encoder_capable` true.
//!
//! **Transmit**: [`PsaTwoButtonDecoder::encode`] sends the frame [`TX_COPIES`] times behind one
//! wake-up carrier. When the frame deciphered (counter known, table loaded) it generates the **next
//! code** (counter + 1, this remote's constants, a fresh check byte, re-enciphered); otherwise it
//! replays the captured code. Works from a live decode or a reloaded `.fob`; there is no Flipper
//! key-file form (the 76-bit frame does not fit the generic `Key`).

use super::burst::{self, BurstReader};
use super::info::prelude::*;
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling code (PSA older cipher)",
    make: "Peugeot/Citroen",
    category: Category::Vehicle,
    buttons: Some(button_name),
    presets: &[&AM650],
};

const NAME: &str = "PSA 2-Button";
/// Half a bit: a pulse and the pause behind it are 414 µs together in the recordings (a Flipper's
/// receiver gave 120 to 240 µs of carrier and 200 to 320 µs of pause for one half bit each).
const TE: u32 = 207;
const FRAME_BITS: usize = 76;
/// The carrier behind the frame: 2.5 ms, and 2.7 ms when the frame's last bit is a 0 (its second
/// half is carrier too).
const END_MIN: u32 = 2_000;
const END_MAX: u32 = 3_300;
/// Transmit (replay): the pause before a frame, the closing carrier, and the wake-up carrier a press
/// opens with. The frame is sent [`TX_COPIES`] times, as a held button repeats it.
const PAUSE_US: u32 = 6_900;
const END_US: u32 = 2_500;
const LEADIN_US: u32 = 78_000;
const TX_COPIES: usize = 3;

/// The cipher behind the 64-bit code: the PSA older-code block cipher ([`super::psa`]) under this
/// key family's table. Inert without a table — [`decode`] and [`next_code`] return `None`, and the
/// decoder falls back to reporting/replaying the raw code.
mod cipher {
    use crate::protocols::keys::get_keystore;
    use crate::protocols::psa::{older_decrypt, older_encrypt};

    /// The two equal check bytes the deciphered block carries (bytes 6 and 7): a function of the
    /// 8-bit sum of the other six plaintext bytes and the counter's low byte. Recovered from about
    /// 540 presses of the one key across a wide counter range; the earlier closed form in the counter
    /// alone was this rule with the key's byte 2 (the counter's high byte) held fixed. When the
    /// generic term lands on zero the byte is sent as `0xFF`.
    pub fn check_byte(pt: &[u8; 8]) -> u8 {
        let sum = pt[..6].iter().fold(0u8, |a, &b| a.wrapping_add(b));
        let h = (pt[3] << 3) ^ (0u8.wrapping_sub(sum >> 2) & 0x3F);
        if h == 0 {
            0xFF
        } else {
            h
        }
    }

    /// Decipher the code and return `(counter, plaintext)` when the block verifies: bytes 6 and 7
    /// are equal and are the block's [`check_byte`]. The counter is 16-bit (byte 2 the high byte,
    /// byte 3 the low). `None` without a loaded table, or when the block does not verify (a frame of
    /// another protocol, or a wrong table).
    pub fn decode(code: u64) -> Option<(u16, [u8; 8])> {
        let table = get_keystore().psa_2button_table()?;
        let pt = older_decrypt(code.to_be_bytes(), &table);
        let counter = (u16::from(pt[2]) << 8) | u16::from(pt[3]);
        (pt[6] == pt[7] && pt[6] == check_byte(&pt)).then_some((counter, pt))
    }

    /// The 64-bit code of the next press (counter + 1, carrying into byte 2), keeping this remote's
    /// own constant bytes and a fresh check byte. `None` without a loaded table (the caller then
    /// replays the captured code).
    pub fn next_code(code: u64) -> Option<u64> {
        let table = get_keystore().psa_2button_table()?;
        let mut pt = older_decrypt(code.to_be_bytes(), &table);
        let next = (((u16::from(pt[2]) << 8) | u16::from(pt[3])).wrapping_add(1)).to_be_bytes();
        pt[2] = next[0];
        pt[3] = next[1];
        pt[6] = check_byte(&pt);
        pt[7] = pt[6];
        Some(u64::from_be_bytes(older_encrypt(pt, &table)))
    }
}

fn button_name(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0xC => Some("Unlock"),
        0x3 => Some("Lock"),
        _ => None,
    }
}

/// PSA 2-Button decoder: reads a burst when the carrier behind the frame ends it.
pub struct PsaTwoButtonDecoder {
    reader: BurstReader,
}

impl PsaTwoButtonDecoder {
    pub fn new() -> Self {
        Self { reader: BurstReader::skewed(TE, 2, 1024) }
    }
}

impl Default for PsaTwoButtonDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// The frame of a burst that a long carrier pulse ended: the pause before the burst, the half bit
/// of carrier, 76 bits. The pulse that ended the burst is its last half: the second half of a last
/// 0 bit, or one half more behind a last 1 bit.
fn read(halves: &[bool]) -> Option<DecodedSignal> {
    let whole = 2 + 2 * FRAME_BITS;
    if !(halves.len() == whole || halves.len() == whole + 1) || halves[0] || !halves[1] {
        return None;
    }
    let frame = burst::take(&burst::bits(halves, 2, false), 0, FRAME_BITS)?;
    let code = burst::number(&frame[12..]);
    // With the key table loaded, decipher the code to its press counter; otherwise report the raw
    // code (and encode will replay it).
    let (counter, crc_valid) = cipher::decode(code).map_or((None, false), |(c, _)| (Some(c), true));
    Some(DecodedSignal {
        serial: None,
        button: Some(burst::number(&frame[8..12]) as u8),
        counter,
        crc_valid,
        data: code,
        data_count_bit: FRAME_BITS,
        encoder_capable: true,
        extra: Some(burst::number(&frame[..12])),
        protocol_display_name: None,
    })
}

/// The pulses of one frame: the 6.9 ms pause, the half bit of carrier, the 76 Manchester bits
/// (`CE` ‖ button ‖ code; carrier-then-pause is a 1), and the 2.5 ms closing carrier. The inverse of
/// [`read`] over the header `extra` and the 64-bit `code` the caller supplies (the captured code for
/// a replay, or the next press's for a rolling send). Copies are chained after a single wake-up
/// carrier (below), the end-carrier→pause boundary keeping each frame separate.
fn frame_pulses(extra: u64, code: u64) -> Vec<LevelDuration> {
    let mut bits = burst::pattern(extra, 12);
    bits.extend(burst::pattern(code, 64));
    let mut wire = vec![LevelDuration::new(false, PAUSE_US), LevelDuration::new(true, TE)];
    wire.extend(bits.iter().flat_map(|&b| [LevelDuration::new(b, TE), LevelDuration::new(!b, TE)]));
    wire.push(LevelDuration::new(true, END_US));
    wire
}

impl ProtocolDecoder for PsaTwoButtonDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE, te_long: 2 * TE, te_delta: TE / 2, min_count_bit: FRAME_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.reader.reset();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let halves = self.reader.feed(level, duration)?;
        (level && (END_MIN..=END_MAX).contains(&duration)).then(|| read(&halves)).flatten()
    }

    /// A burst the input ends has no carrier behind it: no frame.
    fn flush(&mut self) -> Option<DecodedSignal> {
        self.reader.flush();
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// Transmit the frame [`TX_COPIES`] times behind one wake-up carrier. When the frame deciphered
    /// (its `counter` is set and the key table is loaded), the code sent is the **next** press's
    /// (counter + 1); otherwise the captured code is replayed. The button stays the decoded frame's
    /// (it is the clear header nibble; the requested button is ignored). `None` if the decode is not
    /// a 76-bit frame or has no `extra`.
    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != FRAME_BITS {
            return None;
        }
        let extra = decoded.extra?;
        // Next code when the frame deciphered and the table is still loaded; else replay the capture.
        let code = decoded.counter.and_then(|_| cipher::next_code(decoded.data)).unwrap_or(decoded.data);
        // One wake-up carrier, then the frame TX_COPIES times (each ended by its 2.5 ms carrier,
        // which the next frame's pause separates from it).
        let mut wire = vec![LevelDuration::new(true, LEADIN_US)];
        for _ in 0..TX_COPIES {
            wire.extend(frame_pulses(extra, code));
        }
        Some(burst::merged(wire))
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    /// The first whole frame of the unlock recording.
    const HEAD: u64 = 0xCEC;
    const CODE: u64 = 0xB328_FDAE_A203_3C71;

    /// The pulses of a frame of `bits`: the pause, the half bit of carrier, the bits, `end` µs of
    /// carrier and silence.
    fn wire_of(bits: &[bool], pause: u32, end: u32) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, pause), LevelDuration::new(true, TE)];
        wire.extend(bits.iter().flat_map(|&b| [LevelDuration::new(b, TE), LevelDuration::new(!b, TE)]));
        wire.push(LevelDuration::new(true, end));
        wire.push(LevelDuration::new(false, 100_000));
        burst::merged(wire)
    }

    fn frame_bits(head: u64, code: u64) -> Vec<bool> {
        let mut bits = burst::pattern(head, 12);
        bits.extend(burst::pattern(code, 64));
        bits
    }

    fn press(head: u64, code: u64) -> Vec<LevelDuration> {
        wire_of(&frame_bits(head, code), 6_900, 2_500)
    }

    #[test]
    fn decodes_a_press() {
        // Serialize with the cipher tests (shared keystore) and read with no table loaded.
        let _lk = crate::protocols::keeloq_common::KEYSTORE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crate::protocols::keys::get_keystore_mut().load_psa_2button_table(None);
        let got = feed_all(&mut PsaTwoButtonDecoder::new(), &press(HEAD, CODE));
        assert_eq!(got.len(), 1, "{got:?}");
        let d = &got[0];
        assert_eq!((d.data, d.extra, d.data_count_bit), (CODE, Some(HEAD), 76));
        assert_eq!((d.serial, d.button, d.counter), (None, Some(0xC), None));
        assert!(!d.crc_valid && d.encoder_capable);
        assert_eq!(feed_resetting(&mut PsaTwoButtonDecoder::new(), &press(HEAD, CODE)).len(), 1);
        // a frame that ends with a 0 bit: its second half is part of the carrier behind it
        let got = feed_all(&mut PsaTwoButtonDecoder::new(), &press(0xCE3, CODE ^ 1));
        assert_eq!(got.len(), 1);
        assert_eq!((got[0].data, got[0].button), (CODE ^ 1, Some(0x3)));
        // as a receiver gives it: carrier pulses 70 µs short, pauses 70 µs long
        let mut wire = press(HEAD, CODE);
        let n = wire.len();
        for p in &mut wire[1..n - 2] {
            p.duration_us = if p.level { p.duration_us - 70 } else { p.duration_us + 70 };
        }
        let got = feed_all(&mut PsaTwoButtonDecoder::new(), &wire);
        assert_eq!(got.iter().map(|d| d.data).collect::<Vec<_>>(), [CODE]);
        assert_eq!(button_name(NAME, 0xC), Some("Unlock"));
        assert_eq!(button_name(NAME, 0x3), Some("Lock"));
        assert_eq!(button_name(NAME, 0x1), None);
    }

    /// Replay encoder: the transmitted copies decode back to the captured frame (same code, head and
    /// button), and a decode that is not a 76-bit frame refuses.
    #[test]
    fn replay_round_trips() {
        let _lk = crate::protocols::keeloq_common::KEYSTORE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crate::protocols::keys::get_keystore_mut().load_psa_2button_table(None);
        let dec = PsaTwoButtonDecoder::new();
        let decoded = feed_all(&mut PsaTwoButtonDecoder::new(), &press(HEAD, CODE)).pop().unwrap();
        assert!(decoded.encoder_capable);
        let wire = dec.encode(&decoded, 0).expect("replay pulses");
        let back = feed_all(&mut PsaTwoButtonDecoder::new(), &wire);
        assert_eq!(back.len(), TX_COPIES, "one decode per copy: {back:?}");
        for d in &back {
            assert_eq!((d.data, d.extra, d.button), (CODE, Some(HEAD), Some(0xC)));
        }
        // the lock frame (ends on a 0 bit) round-trips too, button and all
        let lock = feed_all(&mut PsaTwoButtonDecoder::new(), &press(0xCE3, CODE ^ 1)).pop().unwrap();
        let back = feed_all(&mut PsaTwoButtonDecoder::new(), &dec.encode(&lock, 0).unwrap());
        assert!(back.iter().all(|d| (d.data, d.button) == (CODE ^ 1, Some(0x3))));
        // a non-frame decode cannot be replayed
        let mut wrong = decoded.clone();
        wrong.data_count_bit = 64;
        assert!(dec.encode(&wrong, 0).is_none());
    }

    /// A dummy 16-byte table (not the shipped one). The cipher mechanics are table-independent.
    const DUMMY: [u8; 16] =
        [0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF];

    /// A 64-bit code built from a counter under `table`: this test's plaintext layout (constants, the
    /// 16-bit counter in bytes 2 and 3, two check bytes) enciphered as the key does.
    fn code_for(counter: u16, table: &[u8; 16]) -> u64 {
        let mut pt = [0x11, 0x22, (counter >> 8) as u8, counter as u8, 0x44, 0x55, 0, 0];
        pt[6] = cipher::check_byte(&pt);
        pt[7] = pt[6];
        u64::from_be_bytes(crate::protocols::psa::older_encrypt(pt, table))
    }

    /// With a key table loaded, a frame deciphers to its press counter, and "send next code" makes
    /// the next press (counter + 1) which re-decodes. Uses the dummy table, not the shipped one.
    #[test]
    fn deciphers_and_sends_next_code() {
        let _lk = crate::protocols::keeloq_common::KEYSTORE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crate::protocols::keys::get_keystore_mut().load_psa_2button_table(Some(DUMMY));

        let code = code_for(0x2A, &DUMMY);
        let dec = PsaTwoButtonDecoder::new();
        let d = feed_all(&mut PsaTwoButtonDecoder::new(), &press(HEAD, code)).pop().expect("decode");
        assert_eq!(d.counter, Some(0x2A), "counter deciphered");
        assert!(d.crc_valid, "the block's check bytes verified");

        // "Send next code": the transmitted copies carry counter + 1, and decode back to it.
        let back = feed_all(&mut PsaTwoButtonDecoder::new(), &dec.encode(&d, 0).expect("next"));
        assert_eq!(back.len(), TX_COPIES);
        for b in &back {
            assert_eq!(b.counter, Some(0x2B), "counter advanced");
            assert_eq!(b.data, code_for(0x2B, &DUMMY), "the next press's code");
            assert_eq!(b.button, Some(0xC), "button kept");
        }
        // The counter is 16-bit: incrementing 0x00FF carries into byte 2 (to 0x0100), and the next
        // code re-decodes to it. (The long recordings showed byte 2 stepping as the low byte wraps.)
        let wrap = feed_all(&mut PsaTwoButtonDecoder::new(), &press(HEAD, code_for(0x00FF, &DUMMY))).pop().unwrap();
        assert_eq!(wrap.counter, Some(0x00FF));
        let back = feed_all(&mut PsaTwoButtonDecoder::new(), &dec.encode(&wrap, 0).unwrap());
        assert!(back.iter().all(|b| b.counter == Some(0x0100) && b.data == code_for(0x0100, &DUMMY)));

        // A random code does not spuriously verify (≈ 2^-16).
        assert!(feed_all(&mut PsaTwoButtonDecoder::new(), &press(HEAD, CODE)).pop().unwrap().counter.is_none());

        crate::protocols::keys::get_keystore_mut().load_psa_2button_table(None);
    }

    /// The table KAT ships ([`crate::keystore::PSA_2BUTTON_TABLE`]) reads a real recorded press: the
    /// first whole unlock frame deciphers to its counter. References the shipped const; its bytes are
    /// not written here.
    #[test]
    fn the_shipped_table_reads_a_real_press() {
        let _lk = crate::protocols::keeloq_common::KEYSTORE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crate::protocols::keys::get_keystore_mut().load_psa_2button_table(Some(crate::keystore::PSA_2BUTTON_TABLE));

        let d = feed_all(&mut PsaTwoButtonDecoder::new(), &press(HEAD, CODE)).pop().expect("decode");
        assert_eq!(d.counter, Some(0xB316), "the recorded 16-bit counter of this frame");
        assert!(d.crc_valid);

        crate::protocols::keys::get_keystore_mut().load_psa_2button_table(None);
    }

    #[test]
    fn the_gate() {
        let read_one = |wire: Vec<LevelDuration>| feed_all(&mut PsaTwoButtonDecoder::new(), &wire).len();
        let bits = frame_bits(HEAD, CODE);
        assert_eq!(read_one(wire_of(&bits, 6_900, 2_500)), 1);
        // the carrier behind the frame
        assert_eq!(read_one(wire_of(&bits, 6_900, 2_000)), 1);
        assert_eq!(read_one(wire_of(&bits, 6_900, 3_300)), 1);
        assert_eq!(read_one(wire_of(&bits, 6_900, 1_500)), 0, "a short end");
        assert_eq!(read_one(wire_of(&bits, 6_900, 4_500)), 0, "a long end");
        // a bit more or less
        assert_eq!(read_one(wire_of(&bits[1..], 6_900, 2_500)), 0, "75 bits");
        let mut longer = vec![true];
        longer.extend(&bits);
        assert_eq!(read_one(wire_of(&longer, 6_900, 2_500)), 0, "77 bits");
        // a pulse of another length in the frame: the burst begins behind it, and is too short
        let mut wire = wire_of(&bits, 6_900, 2_500);
        wire[40].duration_us = 4 * TE;
        assert_eq!(read_one(wire), 0);
        // no carrier behind the frame: the pause, or the end of the input
        let mut wire = wire_of(&bits, 6_900, 2_500);
        let n = wire.len();
        wire.truncate(n - 2);
        let mut dec = PsaTwoButtonDecoder::new();
        assert!(feed_all(&mut dec, &wire).is_empty() && dec.flush().is_none());
        wire.push(LevelDuration::new(false, 50_000));
        assert_eq!(read_one(wire), 0);
        // another speed
        let slow: Vec<LevelDuration> = wire_of(&bits, 6_900, 2_500)
            .into_iter()
            .map(|p| if p.duration_us < 1_000 { LevelDuration::new(p.level, p.duration_us * 5 / 4) } else { p })
            .collect();
        assert_eq!(read_one(slow), 0);
    }

    /// The two recordings: 50 presses each, every frame read with the head `CE`, the file's button
    /// and a code of its own.
    #[test]
    fn imports_recordings() {
        let dir = std::path::Path::new("IMPORTS/GROUPE PSA");
        let files = [("citroen_two_button_valeo_key_unlock_x50.sub", 0xC, 40), ("citroen_two_button_valeo_key_lock_x50.sub", 0x3, 49)];
        if !dir.join(files[0].0).is_file() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let mut codes = std::collections::HashSet::new();
        for (file, button, least) in files {
            let (_, wire) = sub_pulses(&dir.join(file));
            let got = feed_resetting(&mut PsaTwoButtonDecoder::new(), &wire);
            eprintln!("PSA 2-Button {file}: {} frames", got.len());
            assert!(got.len() >= least && got.len() <= 50, "{file}: {}", got.len());
            assert!(got.iter().all(|d| d.extra == Some(0xCE0 | button) && d.button == Some(button as u8)));
            codes.extend(got.iter().map(|d| d.data));
            if button == 0xC {
                assert!(got.iter().any(|d| d.data == CODE));
            }
        }
        assert!(codes.len() >= 89, "{}", codes.len());
    }
}
