//! Compustar 1WG3R / 1WAMR-1900 car-alarm remote (receive only).
//!
//! Port of rtl_433 `src/devices/compustar_1wg3r.c` (`r_device` "Compustar 1WG3R Car Remote",
//! model `Compustar-1WG3R`; 433.92 MHz). No Flipper reference names it, but the air format is the
//! one ProtoPirate / Flipper-ARF read as Scher-Khan "MAGIC CODE, Static" (35 bits, no fields):
//! Scher-Khan's 750/1100 µs PWM with a 1500 µs header, which reads a short pulse as 0. Scher-Khan
//! is earlier in KAT's registry and would report most such frames first (on the sync pulse that
//! ends each frame), so it leaves the 35-bit frames that pass this decoder's checks
//! ([is_valid_frame]) to it; see "Overlap with Scher-Khan" in `docs/compustar_1wg3r.md`.
//!
//! **Modulation** — `OOK_PULSE_PWM`: short_width 708 µs (pulse → 1), long_width 1076 µs (→ 0),
//! sync_width 1448 µs, reset_limit 1532 µs, no tolerance, so the slicer's split points apply: a
//! pulse up to 892 µs is a 1, 893–1262 µs a 0, longer a sync (a new row). A gap > 1532 µs ends the
//! message. A press is one long message of 35-bit frames, each behind three sync pulses.
//!
//! **Frame** (`compustar_1wg3r_decode`) — every row of 35 or more bits: `IIII xxx iiiiiiii
//! bbbbbbbb`: the 16-bit ID, three 1 bits, the inverted button code, the button code. Checks: the
//! 111 marker and bit 39 = 0 (`DECODE_ABORT_EARLY`), ID not 0000 / FFFF (`DECODE_FAIL_SANITY`), the
//! inverse byte (`DECODE_FAIL_MIC`). Within a message, a row equal (35 bits) to the previous valid
//! row is not reported again; a 5-bit row of 1s (the batch separator in "secondary mode") ends
//! that. Buttons: rtl_433's map on `button & 0x7F` (Lock, Panic, Start, Trunk, Unlock, Long Press —
//! codes can name several), "?" when none, then "Secondary Mode" when bit 0x80 is set (appended
//! with "; " only after a name, so an unknown code reads "?Secondary Mode", as in the C).
//!
//! **Fields** — `serial` = the ID, `button` = the 8-bit button code, no counter (fixed code).
//! `data` = the row's first 35 bits (the frame) right-aligned, `data_count_bit` = 35.
//! `crc_valid` = true (the inverse byte matched).
//!
//! **Reports** — row-wise ([super::rtl433]): each new event is reported as soon as its
//! row is final, with rtl_433's per-message duplicate suppression, so KAT reports what rtl_433
//! outputs for the message (every batch of a secondary-mode press).
//!
//! **Encoder** — receive only, as rtl_433.

use super::rtl433::{button_table, frame_signal, join_names, BitBuffer, DecodeFn, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "Compustar-1WG3R";

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Compustar 1WG3R Car Remote"],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Compustar",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The `r_device`.
const DEVICE: Device = Device {
    modulation: Modulation::OokPwm,
    short_width: 708,
    long_width: 1076,
    reset_limit: 1532,
    gap_limit: 0,
    sync_width: 1448,
    tolerance: 0,
    gaps: Gaps::AtOnce,
    ends: Ends::Ook,
    resync: false,
};

/// rtl_433's `button_map` (matched against `button & 0x7f`).
const BUTTON_MAP: [(&str, &[u8]); 6] = [
    ("Lock", &[0x03, 0x05, 0x09, 0x0b, 0x0d, 0x0f, 0x1f, 0x17, 0x13, 0x15, 0x19, 0x1b, 0x1d]),
    ("Panic", &[0x18]),
    ("Start", &[0x09, 0x0a, 0x0c, 0x0b, 0x0e, 0x0d, 0x04, 0x1f, 0x08, 0x19, 0x1a, 0x1c, 0x1b, 0x1e, 0x1d, 0x12]),
    ("Trunk", &[0x05, 0x06, 0x0c, 0x0e, 0x0d, 0x1f, 0x17, 0x02, 0x15, 0x16, 0x1c, 0x1e, 0x1d, 0x08, 0x14]),
    ("Unlock", &[0x03, 0x06, 0x0a, 0x0b, 0x0e, 0x1f, 0x07, 0x17, 0x13, 0x16, 0x1a, 0x1b, 0x1e]),
    (
        "Long Press",
        &[
            0x23, 0x31, 0x13, 0x16, 0x17, 0x1a, 0x1b, 0x1e, 0x15, 0x16, 0x1c, 0x1e, 0x1d, 0x08, 0x14, 0x08, 0x19, 0x1a, 0x1c, 0x1b,
            0x1e, 0x1d, 0x12, 0x13, 0x15, 0x19, 0x1b, 0x1d,
        ],
    ),
];

/// rtl_433's `button_str`.
fn button_str(button: u8) -> String {
    let code = button & 0x7f;
    let matched = BUTTON_MAP.iter().any(|(_, v)| v.contains(&code));
    let mut s = join_names(BUTTON_MAP.iter().filter(|(_, v)| v.contains(&code)).map(|(n, _)| *n), "?");
    if button & 0x80 != 0 {
        if matched {
            s += "; ";
        }
        s += "Secondary Mode";
    }
    s
}

fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    static NAMES: std::sync::OnceLock<Vec<&'static str>> = std::sync::OnceLock::new();
    Some(button_table(&NAMES, button_str)[code as usize])
}

/// `compustar_1wg3r_decode`, one row at a time; `previous_row` is the C's per-message state.
#[derive(Debug, Default)]
struct Compustar {
    previous_row: Option<usize>,
}

impl DecodeFn for Compustar {
    const ROW_WISE: bool = true;

    fn message_start(&mut self) {
        self.previous_row = None;
    }

    fn row(&mut self, bits: &BitBuffer, row: usize) -> Option<DecodedSignal> {
        let b: [u8; 5] = bits.bytes(row);
        let n = bits.bits_per_row(row);
        // reset row count for the row separator
        if n == 5 && b[0] & 0xf8 == 0xf8 {
            self.previous_row = None;
            return None;
        }
        if n < 35 {
            return None; // DECODE_ABORT_LENGTH
        }
        if b[2] & 0xe0 != 0xe0 || b[4] & 1 != 0 {
            return None; // DECODE_ABORT_EARLY
        }
        if (b[0] == 0xff && b[1] == 0xff) || (b[0] == 0x00 && b[1] == 0x00) {
            return None; // DECODE_FAIL_SANITY
        }
        let button_inverse = b[2] << 3 | b[3] >> 5;
        let button = b[3] << 3 | b[4] >> 5;
        if !button_inverse != button {
            return None; // DECODE_FAIL_MIC
        }
        if self.previous_row.is_some_and(|prev| bits.compare_rows(prev, row, 35)) {
            return None; // duplicate of previous valid message
        }
        self.previous_row = Some(row);
        let mut d = frame_signal(&b, 35);
        d.serial = Some(u32::from(u16::from_be_bytes([b[0], b[1]])));
        d.button = Some(button);
        d.crc_valid = true;
        Some(d)
    }
}

/// Whether a 35-bit frame, right-aligned as this decoder reads it (`IIII xxx iiiiiiii bbbbbbbb`),
/// passes the C's checks: the 111 marker, an ID other than 0000 / FFFF and the inverse button
/// byte (bit 39 lies past a 35-bit frame). Scher-Khan reads the same frame with every bit inverted
/// and leaves the ones that pass to this decoder.
pub(super) fn is_valid_frame(frame: u64) -> bool {
    let id = (frame >> 19) & 0xFFFF;
    let (inverse, button) = ((frame >> 8) & 0xFF, frame & 0xFF);
    (frame >> 16) & 0b111 == 0b111 && id != 0 && id != 0xFFFF && inverse == !button & 0xFF
}

/// Compustar 1WG3R decoder (receive only).
pub struct Compustar1wg3rDecoder {
    rx: Receiver,
    decode: Compustar,
}

impl Compustar1wg3rDecoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE), decode: Compustar::default() }
    }
}

impl Default for Compustar1wg3rDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for Compustar1wg3rDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 708, te_long: 1076, te_delta: 0, min_count_bit: 35 }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.rx.reset(&mut self.decode);
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.rx.feed(&mut self.decode, level, duration)
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    /// Receive only, as rtl_433.
    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

/// rtl_433's JSON for a decode (`data_print_jsons`, no `time`), for the oracle comparisons.
#[cfg(test)]
pub(super) fn rtl433_json(d: &DecodedSignal) -> String {
    let btn = d.button.unwrap_or(0);
    format!(
        "{{\"model\":\"{NAME}\",\"id\":\"{:04X}\",\"button_code\":{btn},\"button_str\":\"{}\",\"mic\":\"CHECKSUM\"}}",
        d.serial.unwrap_or(0),
        button_str(btn)
    )
}

#[cfg(test)]
mod tests {
    use super::super::rtl433::test_util::*;
    use super::*;

    /// The 35 frame bits of an ID and button code (inverse byte computed).
    fn frame(id: u16, button: u8) -> Vec<bool> {
        let v = u64::from(id) << 19 | 0b111 << 16 | u64::from(!button) << 8 | u64::from(button);
        bits_of(v, 35)
    }

    /// PWM as the remote sends it: three 1470/1495 µs syncs before every row, 1 = 720/760 µs,
    /// 0 = 1090/1140 µs; `rows` back to back, then a 10 ms gap.
    fn press(rows: &[Vec<bool>]) -> Vec<(u32, u32)> {
        let mut pairs = Vec::new();
        for r in rows {
            pairs.extend([(1470, 1495); 3]);
            pairs.extend(r.iter().map(|&b| if b { (720, 760) } else { (1090, 1140) }));
        }
        pairs.last_mut().unwrap().1 = 10_000;
        pairs
    }

    fn decode(pairs: &[(u32, u32)]) -> Vec<DecodedSignal> {
        feed_pairs(&mut Compustar1wg3rDecoder::new(), pairs)
    }

    /// The batch separator: five short pulses as a row of their own.
    fn separator() -> Vec<bool> {
        vec![true; 5]
    }

    #[test]
    fn decodes_the_recorded_lock_frame_once_per_press() {
        let lock = frame(0x9EB4, 0x0F);
        // KAT's Scher-Khan reads the same 35 bits inverted: 30A580FF0
        assert_eq!(bits_of(!0x3_0A58_0FF0_u64 & ((1 << 35) - 1), 35), lock);
        let got = decode(&press(&vec![lock; 8]));
        assert_eq!(got.len(), 1, "duplicates suppressed within the message");
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter, d.crc_valid), (Some(0x9EB4), Some(0x0F), None, true));
        assert_eq!((d.data, d.data_count_bit), (0x4F5A_7F00F, 35));
        assert_eq!(rtl433_json(d), r#"{"model":"Compustar-1WG3R","id":"9EB4","button_code":15,"button_str":"Lock","mic":"CHECKSUM"}"#);
        // a new message reports it again
        let mut two = press(&vec![frame(0x9EB4, 0x0F); 2]);
        two.extend(press(&vec![frame(0x9EB4, 0x0F); 2]));
        assert_eq!(decode(&two).len(), 2);
    }

    /// The check Scher-Khan applies before reporting a 35-bit frame agrees with this decoder's.
    #[test]
    fn scher_khan_leaves_valid_frames_to_compustar() {
        let value = |id: u16, button: u8| u64::from(id) << 19 | 0b111 << 16 | u64::from(!button) << 8 | u64::from(button);
        assert!(is_valid_frame(value(0x9EB4, 0x0F)));
        assert!(is_valid_frame(!0x3_0A58_0FF0_u64 & ((1 << 35) - 1)), "Scher-Khan's 30A580FF0 inverted");
        assert!(!is_valid_frame(value(0x9EB4, 0x0F) ^ 0x100), "inverse byte");
        assert!(!is_valid_frame(value(0x9EB4, 0x0F) & !(1 << 17)), "111 marker");
        assert!(!is_valid_frame(value(0x0000, 0x0F)) && !is_valid_frame(value(0xFFFF, 0x0F)), "ID sanity");
        // Through the registry, a press of Compustar frames is Compustar's alone; a 35-bit frame that
        // fails its checks stays Scher-Khan's.
        let registry_names = |rows: &[Vec<bool>]| -> Vec<String> {
            let mut stream = vec![LevelDuration::new(false, 20_000)];
            for (h, l) in press(rows) {
                stream.extend([LevelDuration::new(true, h), LevelDuration::new(false, l)]);
            }
            let mut reg = super::super::ProtocolRegistry::new();
            reg.process_signal_stream(&stream, 433_920_000).into_iter().map(|(n, _, _)| n).collect()
        };
        let names = registry_names(&vec![frame(0x9EB4, 0x0F); 4]);
        assert!(!names.is_empty() && names.iter().all(|n| n == NAME), "{names:?}");
        // The same air format with a wrong inverse byte: not Compustar, so Scher-Khan reports it.
        let mut bad = frame(0x9EB4, 0x0F);
        bad[27] = !bad[27];
        let names = registry_names(&vec![bad; 4]);
        assert!(!names.is_empty() && names.iter().all(|n| n.starts_with("Scher-Khan")), "{names:?}");
    }

    #[test]
    fn secondary_mode_batches_like_rtl_433() {
        // rtl_433's secondary_mode recording: 135, 130, 130, 135, 132 in one message
        let (u, t, s) = (frame(0x9EB4, 0x87), frame(0x9EB4, 0x82), frame(0x9EB4, 0x84));
        let rows = vec![u.clone(), u.clone(), t.clone(), t.clone(), separator(), t.clone(), t, u, s.clone(), s];
        let got: Vec<Option<u8>> = decode(&press(&rows)).iter().map(|d| d.button).collect();
        assert_eq!(got, [Some(135), Some(130), Some(130), Some(135), Some(132)]);
        assert_eq!(button_str(135), "Unlock; Secondary Mode");
        assert_eq!(button_str(0x19), "Lock; Start; Long Press");
        assert_eq!(button_str(0x80), "?Secondary Mode");
        assert_eq!(button_str(0x20), "?");
        assert_eq!(native_buttons(NAME, 0x82), Some("Trunk; Secondary Mode"));
    }

    /// In the registry each row is a report of its own: the registry's reset after it does not
    /// cost the message its duplicate suppression.
    #[test]
    fn registry_reset_after_own_report_keeps_the_message() {
        let mut dec = Compustar1wg3rDecoder::new();
        let rows = vec![frame(0x9EB4, 0x87), frame(0x9EB4, 0x87), frame(0x9EB4, 0x82), frame(0x9EB4, 0x82)];
        let mut got = Vec::new();
        dec.reset();
        for p in wire(&press(&rows)) {
            if let Some(d) = dec.feed(p.level, p.duration_us) {
                got.push(d.button);
                dec.reset(); // what the registry does after every decode
            }
        }
        assert_eq!(got, [Some(0x87), Some(0x82)]);
        // a reset that does not follow a report starts over
        dec.reset();
        let again: Vec<_> = wire(&press(&rows[..2])).iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect();
        assert_eq!(again.len(), 1);
    }

    #[test]
    fn rejects_failed_checks() {
        // inverse byte
        let mut r = frame(0x9EB4, 0x0F);
        r[22] = !r[22];
        assert!(decode(&press(&[r])).is_empty());
        // 111 marker
        let mut r = frame(0x9EB4, 0x0F);
        r[17] = false;
        assert!(decode(&press(&[r])).is_empty());
        // ID 0000 / FFFF
        assert!(decode(&press(&[frame(0, 0x0F)])).is_empty());
        assert!(decode(&press(&[frame(0xFFFF, 0x0F)])).is_empty());
        // 34 bits
        assert!(decode(&press(&[frame(0x9EB4, 0x0F)[..34].to_vec()])).is_empty());
        // bit 39 set (a longer row): rejected; 36..39 clear: accepted
        let mut r = frame(0x9EB4, 0x0F);
        r.extend([false, false, false, false, true]);
        assert!(decode(&press(&[r])).is_empty());
        let mut r = frame(0x9EB4, 0x0F);
        r.extend([true, true, true, false]);
        assert_eq!(decode(&press(&[r])).len(), 1);
        // pulse split points: 892 µs is a 1, 893 a 0 (the ID's top bit); a gap over 1532 µs ends
        // the message
        let mut pairs = press(&[frame(0x9EB4, 0x0F)]);
        pairs[3].0 = 892;
        assert_eq!(decode(&pairs)[0].serial, Some(0x9EB4));
        pairs[3].0 = 893;
        assert_eq!(decode(&pairs)[0].serial, Some(0x1EB4));
        let mut pairs = press(&[frame(0x9EB4, 0x0F)]);
        pairs[10].1 = 1533;
        assert!(decode(&pairs).is_empty());
    }

    /// rtl_433_tests `compustar/1wg3r/01` through KAT's receive pipeline into this decoder alone
    /// equal rtl_433's events; through the registry they decode as Compustar only (Scher-Khan
    /// leaves them to it, see `docs/compustar_1wg3r.md`).
    #[test]
    #[ignore]
    fn decodes_the_rtl_433_recordings() {
        let Some(root) = rtl433_tests() else {
            eprintln!("Skip: rtl_433_tests not found");
            return;
        };
        let dir = root.join("compustar/1wg3r/01");
        for f in ["lock", "lock_start_long_press", "secondary_mode", "trunk", "unlock"] {
            let path = dir.join(format!("{f}_433.92M_250k.cu8"));
            let got: Vec<serde_json::Value> = decoder_decodes(&mut Compustar1wg3rDecoder::new(), &path, true)
                .iter()
                .map(|d| serde_json::from_str(&rtl433_json(d)).unwrap())
                .collect();
            assert_eq!(distinct(&got), distinct(&expected_json(&path)), "{f}");
            let names: Vec<String> = registry_decodes(&path).into_iter().map(|(n, _)| n).collect();
            assert!(!names.is_empty() && names.iter().all(|n| n == NAME), "{f}: {names:?}");
        }
    }

    /// Fuzz dump for the C harness (see `rtl433::test_util::dump_fuzz_trains`).
    #[test]
    #[ignore]
    fn fuzz_dump() {
        let interesting = [1, 891, 892, 893, 894, 1261, 1262, 1263, 1264, 1531, 1532, 1533];
        dump_fuzz_trains(
            "compustar",
            20_000,
            |rng, i| {
                let id = match rng.below(20) {
                    0 => 0,
                    1 => 0xFFFF,
                    _ => rng.next() as u16,
                };
                let codes = [0x0F, 0x87, 0x82, 0x84, 0x19, 0x02, 0x07, 0x18];
                let mut rows = Vec::new();
                if rng.chance(5) {
                    // many short rows first: past the bit buffer's 50 rows within one package
                    for _ in 0..40 + rng.below(30) {
                        rows.push((0..1 + rng.below(6)).map(|_| rng.chance(50)).collect::<Vec<bool>>());
                    }
                }
                // sometimes past 1200 pulses (the package limit)
                let max_rows = if rng.chance(10) { 40 } else { 8 };
                let n_rows = 1 + rng.below(max_rows);
                for _ in 0..n_rows {
                    let mut r = if rng.chance(10) {
                        separator()
                    } else {
                        let b = if rng.chance(80) { codes[rng.below(8) as usize] } else { rng.below(256) as u8 };
                        frame(if rng.chance(85) { id } else { rng.next() as u16 }, b)
                    };
                    if rng.chance(10) {
                        let k = rng.below(r.len() as u64) as usize;
                        r[k] = !r[k];
                    }
                    if rng.chance(5) {
                        r.extend((0..1 + rng.below(6)).map(|_| rng.chance(50)));
                    }
                    let n = 1 + rng.below(3);
                    rows.extend(std::iter::repeat_n(r, n as usize));
                }
                let mut pairs = press(&rows);
                let spread = if i % 3 == 0 { 150 } else { 30 };
                for p in pairs.iter_mut() {
                    *p = (rng.jitter(p.0, spread), rng.jitter(p.1, 40));
                }
                if rng.chance(60) {
                    mutate(rng, &mut pairs, &interesting);
                }
                pairs
            },
            || Box::new(Compustar1wg3rDecoder::new()),
            rtl433_json,
        );
    }
}
