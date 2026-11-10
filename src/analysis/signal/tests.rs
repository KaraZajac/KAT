//! Tests of the signal report, on made-up pulse trains.

use super::render::MIN_WIDTH;
use super::*;
use crate::capture::RfMeasurement;

fn p(level: bool, duration_us: u32) -> StoredLevelDuration {
    StoredLevelDuration { level, duration_us }
}

/// Small deterministic PRNG (xorshift32) for reproducible bits and jitter.
struct Rng(u32);

impl Rng {
    fn next(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }

    fn bits(&mut self, n: usize) -> Vec<bool> {
        (0..n).map(|_| self.next() & 1 == 1).collect()
    }

    /// `d` give or take up to `pct` percent.
    fn near(&mut self, d: u32, pct: u32) -> u32 {
        let span = d * pct / 100;
        d - span + self.next() % (2 * span + 1)
    }
}

/// One PWM frame: 12 preamble pairs, a sync gap of 10 TE, then the bits (a long HIGH is 1).
fn pwm_frame(te: u32, bits: &[bool], jitter: u32, rng: &mut Rng) -> Vec<StoredLevelDuration> {
    let mut v = Vec::new();
    for _ in 0..12 {
        v.push(p(true, rng.near(te, jitter)));
        v.push(p(false, rng.near(te, jitter)));
    }
    v.push(p(true, rng.near(te, jitter)));
    v.push(p(false, 10 * te));
    for bit in bits {
        let (h, l) = if *bit { (2 * te, te) } else { (te, 2 * te) };
        v.push(p(true, rng.near(h, jitter)));
        v.push(p(false, rng.near(l, jitter)));
    }
    v
}

/// One Manchester frame (HIGH then LOW is 1): `preamble` ones, then the bits.
fn manchester_frame(te: u32, preamble: usize, bits: &[bool], jitter: u32, rng: &mut Rng) -> Vec<StoredLevelDuration> {
    let mut halves: Vec<bool> = Vec::new();
    for bit in std::iter::repeat_n(&true, preamble).chain(bits) {
        halves.extend(if *bit { [true, false] } else { [false, true] });
    }
    let mut v: Vec<StoredLevelDuration> = Vec::new();
    for half in halves {
        match v.last_mut() {
            Some(last) if last.level == half => last.duration_us += te,
            _ => v.push(p(half, te)),
        }
    }
    for d in &mut v {
        d.duration_us = rng.near(d.duration_us, jitter);
    }
    v
}

/// NRZ at one bit per `te` (HIGH is 1): the bits as runs.
fn nrz_frame(te: u32, bits: &[bool]) -> Vec<StoredLevelDuration> {
    let mut v: Vec<StoredLevelDuration> = Vec::new();
    for bit in bits {
        match v.last_mut() {
            Some(last) if last.level == *bit => last.duration_us += te,
            _ => v.push(p(*bit, te)),
        }
    }
    v
}

/// The frames with a pause before, between and after them.
fn capture(frames: &[Vec<StoredLevelDuration>], pause_us: u32) -> Vec<StoredLevelDuration> {
    let mut v = vec![p(false, pause_us)];
    for f in frames {
        v.extend(f.iter().copied());
        v.push(p(false, pause_us));
    }
    v
}

fn noise(n: usize, seed: u32) -> Vec<StoredLevelDuration> {
    let mut rng = Rng(seed);
    (0..n).map(|i| p(i % 2 == 0, 60 + rng.next() % 241)).collect()
}

fn am() -> Source {
    Source { frequency: 433_920_000, path: Some(RfModulation::AM), ..Source::default() }
}

fn measured(deviation_hz: Option<f32>, both_paths: bool) -> Option<RfMeasurement> {
    Some(RfMeasurement { offset_hz: 12_500.0, snr_db: 31.0, deviation_hz, burst_us: 180_000, both_paths })
}

fn known() -> Vec<KnownTiming> {
    let k = |name, te_short, te_long, te_delta| KnownTiming {
        name,
        te_short,
        te_long,
        te_delta,
        coding: "PWM".to_string(),
        rf: "AM".to_string(),
    };
    vec![k("Made Up A", 250, 500, 100), k("Made Up B", 400, 800, 120), k("Made Up C", 380, 760, 60)]
}

fn report(pairs: &[StoredLevelDuration], source: Source) -> SignalReport {
    analyze(pairs, source, None, &[], None, &known())
}

/// The rendered report as plain text rows.
fn text(r: &SignalReport, width: usize) -> Vec<String> {
    r.render(width).iter().map(|line| line.iter().map(|s| s.text.as_str()).collect()).collect()
}

fn joined(r: &SignalReport, width: usize) -> String {
    text(r, width).join("\n")
}

/// Three copies of one 64-bit PWM frame (TE 400 µs), 30 ms apart.
fn pwm_capture() -> (Vec<StoredLevelDuration>, Vec<bool>) {
    let mut rng = Rng(0x2468_ACE1);
    let bits = rng.bits(64);
    let frames: Vec<_> = (0..3).map(|_| pwm_frame(400, &bits, 6, &mut rng)).collect();
    (capture(&frames, 30_000), bits)
}

#[test]
fn ook_pwm_frames_are_split_read_and_matched() {
    let (pairs, bits) = pwm_capture();
    let r = report(&pairs, Source { live: true, measured: measured(None, false), ..am() });
    assert_eq!(r.keying, Keying::Ook);
    assert!(matches!(r.timing.coding, LineCoding::Pwm { .. }), "{:?}", r.timing.coding);
    assert!((r.te().expect("TE").te_us - 400.0).abs() < 20.0);
    assert!(r.is_regular());
    // Each frame is a preamble row and a data row: the sync gap is a pause.
    assert_eq!(r.rows.len(), 6, "{:#?}", r.rows);
    assert!(matches!(r.rows[0].kind, RowKind::Preamble { .. }), "{:?}", r.rows[0].kind);
    let data: Vec<_> = r.data_rows().collect();
    assert_eq!(data.iter().map(|(n, _, _)| *n).collect::<Vec<_>>(), [2, 4, 6]);
    for (_, _, b) in &data {
        assert_eq!(b.data(), bits.as_slice());
    }
    assert!(matches!(data[1].1.kind, RowKind::Data { same_as: Some(1), .. }), "{:?}", data[1].1.kind);
    assert_eq!(r.repeat.map(|rep| (rep.rows, rep.times)), Some((2, 3)));
    assert_eq!(r.fragments, 0);
    assert_eq!(r.frames_line().as_deref().map(|l| l.starts_with("3 frames of 64 bits, all identical")), Some(true));
    assert!(r.headline().starts_with("OOK, PWM, TE ~"), "{}", r.headline());
    // Nearest first, and only the protocols whose tolerance holds both widths.
    assert_eq!(r.similar.iter().map(|k| k.name).collect::<Vec<_>>(), ["Made Up B", "Made Up C"]);
    let all = joined(&r, 76);
    for part in ["SUMMARY", "MODULATION", "PULSE WIDTHS", "ROWS", "BITS", "WAVEFORM", "SIMILAR TIMING", "RAW"] {
        assert!(all.contains(part), "no {part} in:\n{all}");
    }
    assert!(all.contains("OOK / ASK"), "{all}");
    assert!(all.contains("433.933 MHz (+12.5 kHz"), "{all}");
    assert!(!all.contains("PROTOCOL TIMING"), "{all}");
}

/// The protocols with similar timing are a table: its columns line up whatever the names and
/// codings are, and on the narrowest overlay it is the name that gives way.
#[test]
fn the_similar_protocols_line_up() {
    let (pairs, _) = pwm_capture();
    let mut known = known();
    known[1].coding = "Diff. Manchester".to_string();
    known[1].rf = "AM/FM".to_string();
    known[2].name = "A Made Up Protocol With A Long Name";
    let r = analyze(&pairs, am(), None, &[], None, &known);
    for width in [render::MIN_WIDTH, 76, 110] {
        let rows = text(&r, width);
        let at = rows.iter().position(|l| l.contains("SIMILAR TIMING")).expect("the section");
        let table = &rows[at + 1..at + 3];
        assert!(table[0].starts_with(" Made Up B") && table[0].ends_with("  Diff. Manchester  AM/FM"), "{table:#?}");
        assert!(table[1].ends_with("  PWM               AM"), "{width}: {table:#?}");
        let column = |row: &str, word: &str| row.find(word).map(|i| row[..i].chars().count());
        assert_eq!(column(&table[0], "400/800 us"), column(&table[1], "380/760 us"), "{width}: {table:#?}");
        assert!(table.iter().all(|l| l.chars().count() <= width && !l.ends_with('…')), "{width}: {table:#?}");
        // The long name is whole where there is room for it.
        assert_eq!(table[1].contains("A Made Up Protocol With A Long Name"), width >= 76, "{width}: {table:#?}");
    }
}

/// The waveform and the raw durations show the frame: the row with the most bits (the first of
/// them), not a shorter run before it that also reads as bits.
#[test]
fn the_waveform_shows_the_longest_row_of_bits() {
    let mut rng = Rng(0x1357_9BDF);
    let (short, long) = (rng.bits(12), rng.bits(64));
    let frames = [
        manchester_frame(300, 4, &short, 4, &mut rng),
        manchester_frame(300, 8, &long, 4, &mut rng),
        manchester_frame(300, 8, &long, 4, &mut rng),
    ];
    let r = report(&capture(&frames, 20_000), am());
    let rows: Vec<(usize, usize)> = r.data_rows().map(|(n, _, bits)| (n, bits.data().len())).collect();
    assert_eq!(rows.len(), 3, "{:#?}", r.rows);
    assert!(rows[0].1 < rows[1].1 && rows[1].1 == rows[2].1, "{rows:?}");
    let all = joined(&r, 76);
    assert!(all.contains(&format!("WAVEFORM   row #{}, one column", rows[1].0)), "{all}");
    assert!(all.contains(&format!("RAW   row #{} in us", rows[1].0)), "{all}");
    // Rows of several lengths are counted as rows, the commonest length named.
    let line = r.frames_line().unwrap();
    assert!(line.starts_with(&format!("3 rows of bits, 2 of them {} bits long, 2 distinct", rows[1].1)), "{line}");
    // Two rows of two lengths: both lengths, and nothing about how alike they are.
    let r = report(&capture(&frames[..2], 20_000), am());
    assert_eq!(r.frames_line(), Some(format!("2 rows of bits: {} and {} bits", rows[0].1, rows[1].1)));
}

/// A recording that starts with more long pulses than the rows table lists: the frame after them
/// is in the table all the same (the waveform, the bits and the raw durations name it by number),
/// and the line under the table says what was left out.
#[test]
fn the_rows_table_keeps_the_rows_of_bits() {
    let mut rng = Rng(0x2468_ACE0);
    let bits = rng.bits(64);
    // Long pulses that are no whole number of TEs, each followed by a pause.
    let mut pairs: Vec<_> = (0..14).flat_map(|i| [p(true, 1_400 + 400 * (i % 5)), p(false, 60_000)]).collect();
    let frames: Vec<_> = (0..4).map(|_| pwm_frame(400, &bits, 4, &mut rng)).collect();
    pairs.extend(capture(&frames, 20_000));
    let r = report(&pairs, am());
    let data: Vec<usize> = r.data_rows().map(|(n, _, _)| n).collect();
    // Eleven pulses are long enough to list; each frame is a preamble row and a row of bits.
    assert_eq!((r.rows.len(), data.as_slice()), (19, &[13, 15, 17, 19][..]), "{:#?}", r.rows);
    let all = joined(&r, 76);
    for n in &data {
        assert!(all.lines().any(|l| l.trim_start().starts_with(&format!("{n} ")) && l.contains("64 bits")), "#{n}: {all}");
    }
    // The frames' preambles come next, and four of the long pulses fill the table; the seven left
    // out are named for what they are.
    assert_eq!(all.lines().filter(|l| l.contains("one width:")).count(), 4, "{all}");
    assert_eq!(all.lines().filter(|l| l.contains("long pulse:")).count(), 4, "{all}");
    assert!(all.contains("+ 7 more rows without bits"), "{all}");
    assert!(all.contains("WAVEFORM   row #13, one column"), "{all}");
    assert!(all.lines().any(|l| l.trim_start().starts_with("#13") && l.contains("64 bits")), "{all}");
}

/// Carrier blocks in a recording mean "FSK through an AM receiver" for an unknown capture. A decoded
/// one's signal is known, so its report says nothing of the blocks around the frame.
#[test]
fn a_decoded_captures_carrier_blocks_are_left_alone() {
    let (frames, _) = pwm_capture();
    let mut pairs: Vec<_> = (0..5).flat_map(|_| [p(true, 80_000), p(false, 20_000)]).collect();
    pairs.extend(frames);
    let source = Source { preset: Some("AM650".to_string()), ..am() };
    let unknown = analyze(&pairs, source.clone(), None, &[], None, &known());
    assert_eq!(unknown.carrier_block_us, Some(80_000));
    assert!(joined(&unknown, 76).contains("long blocks without on/off data"));
    let reference = ProtocolTiming { te_short: 400, te_long: 800, te_delta: 120, min_count_bit: 64 };
    let decoded = analyze(&pairs, source, Some("Made Up B"), &[reference], None, &known());
    assert_eq!(decoded.carrier_block_us, None);
    let all = joined(&decoded, 76);
    assert!(!all.contains("long blocks without on/off data") && all.contains("PROTOCOL TIMING"), "{all}");
}

#[test]
fn a_decoded_capture_is_compared_with_its_protocol() {
    let (pairs, _) = pwm_capture();
    let reference = ProtocolTiming { te_short: 400, te_long: 800, te_delta: 120, min_count_bit: 64 };
    let r = analyze(&pairs, am(), Some("Made Up B"), &[reference], None, &known());
    assert!(r.timing.comparison.is_some());
    assert!(r.similar.is_empty());
    let all = joined(&r, 76);
    assert!(all.contains("PROTOCOL TIMING   Made Up B: 400/800 us, +/-120 us"), "{all}");
    assert!(all.contains("EXCELLENT") || all.contains("OK"), "{all}");
    assert!(!all.contains("SIMILAR TIMING"), "{all}");
    // A decoded protocol KAT has no timing definition for says so.
    let r = analyze(&pairs, am(), Some("Made Up Z"), &[], None, &known());
    assert!(joined(&r, 76).contains("no timing definition for Made Up Z"));
}

#[test]
fn a_protocol_sent_with_two_timings_is_compared_with_the_one_that_fits() {
    // A protocol defined as 500/1000 us that is also sent as 300/600 us, like VAG's Type 1/2.
    let mut rng = Rng(0x0BAD_5EED);
    let bits = rng.bits(64);
    let frames: Vec<_> = (0..2).map(|_| manchester_frame(300, 12, &bits, 6, &mut rng)).collect();
    let pairs = capture(&frames, 30_000);
    let main = ProtocolTiming { te_short: 500, te_long: 1000, te_delta: 80, min_count_bit: 64 };
    let other = ProtocolTiming { te_short: 300, te_long: 600, te_delta: 100, min_count_bit: 64 };

    let alone = joined(&analyze(&pairs, am(), Some("Made Up M"), &[main], None, &known()), 76);
    assert!(alone.contains("PROTOCOL TIMING   Made Up M: 500/1000 us, +/-80 us"), "{alone}");
    assert!(!alone.contains("is sent with"), "{alone}");

    let both = joined(&analyze(&pairs, am(), Some("Made Up M"), &[main, other], None, &known()), 76);
    assert!(both.contains("PROTOCOL TIMING   Made Up M: 300/600 us, +/-100 us"), "{both}");
    assert!(both.contains("Timing matches fob!") || both.contains("Within tolerance."), "{both}");
    assert!(both.contains("Made Up M is sent with 2 timings: this is the one the capture fits."), "{both}");
}

#[test]
fn manchester_rows_lose_their_preamble_and_keep_their_bits() {
    let mut rng = Rng(0x0BAD_5EED);
    let mut bits = rng.bits(64);
    bits[0] = false; // the preamble of ones ends here
    let frames: Vec<_> = (0..2).map(|_| manchester_frame(300, 16, &bits, 5, &mut rng)).collect();
    let r = report(&capture(&frames, 25_000), am());
    assert_eq!(r.timing.coding, LineCoding::Manchester);
    let data: Vec<_> = r.data_rows().collect();
    assert_eq!(data.len(), 2, "{:#?}", r.rows);
    assert_eq!(data[0].2.preamble, 16);
    assert_eq!(data[0].2.data(), bits.as_slice());
    assert!(matches!(data[1].1.kind, RowKind::Data { same_as: Some(0), .. }));
    let all = joined(&r, 76);
    assert!(all.contains("16 preamble + 64 bits"), "{all}");
    assert!(all.contains("OOK by the receive path"), "{all}");
    // A second frame with other bits is counted as different, and its distance is given.
    let mut other = bits.clone();
    for i in [5, 17, 40] {
        other[i] = !other[i];
    }
    let frames = [manchester_frame(300, 16, &bits, 5, &mut rng), manchester_frame(300, 16, &other, 5, &mut rng)];
    let r = report(&capture(&frames, 25_000), am());
    let data: Vec<_> = r.data_rows().collect();
    assert!(matches!(data[1].1.kind, RowKind::Data { same_as: None, differs: Some((0, 3)), .. }), "{:?}", data[1].1.kind);
    assert!(r.frames_line().unwrap().starts_with("2 frames of 64 bits, all different"), "{:?}", r.frames_line());
}

/// The Manchester frames of TE 100 with every duration replaced: `high` and `low` give the widths
/// of a 1 TE and a 2 TE pulse of that level, the 1 TE one in two variants picked at random.
fn reshaped(frames: &[Vec<StoredLevelDuration>], high: [u32; 3], low: [u32; 3], rng: &mut Rng) -> Vec<Vec<StoredLevelDuration>> {
    let mut pick = |d: &StoredLevelDuration| {
        let w = if d.level { high } else { low };
        let us = if d.duration_us == 200 { w[2] } else { w[(rng.next() >> 7) as usize % 2] };
        p(d.level, us + rng.next() % 5)
    };
    frames.iter().map(|f| f.iter().map(&mut pick).collect()).collect()
}

#[test]
fn a_slicer_off_the_middle_of_the_tones_still_gives_manchester() {
    let mut rng = Rng(0x2468_ACE1);
    let mut bits = rng.bits(64);
    bits[0] = false;
    let plain: Vec<_> = (0..2).map(|_| manchester_frame(100, 16, &bits, 0, &mut rng)).collect();
    // TE 350 with every HIGH 110 us short and every LOW 110 us long: no TE of its own fits a level.
    let frames = reshaped(&plain, [240, 240, 590], [460, 460, 810], &mut rng);
    let source = Source { path: Some(RfModulation::FM), preset_deviation_hz: Some(47_600.0), ..am() };
    let r = report(&capture(&frames, 25_000), source);
    assert_eq!(r.timing.coding, LineCoding::Manchester);
    let te = r.te().unwrap();
    assert!((te.te_us - 352.0).abs() < 8.0 && (te.skew_us + 110.0).abs() < 8.0, "{te:?}");
    let data: Vec<_> = r.data_rows().collect();
    assert_eq!(data.len(), 2, "{:#?}", r.rows);
    assert_eq!(data[0].2.data(), bits.as_slice());
    let all = joined(&r, 76);
    assert!(all.contains("every HIGH is ~1") && all.contains("shorter and every LOW as much longer"), "{all}");
    assert!(all.contains("2 frames of 64 bits, all identical"), "{all}");
}

#[test]
fn a_width_the_recorder_split_in_two_is_one_width() {
    let mut rng = Rng(0x0F1E_2D3C);
    let mut bits = rng.bits(64);
    bits[0] = false;
    let plain: Vec<_> = (0..3).map(|_| manchester_frame(100, 16, &bits, 0, &mut rng)).collect();
    // One TE comes out as either of two widths on both levels: three widths a level, two of them 1 TE.
    let frames = reshaped(&plain, [98, 132, 226], [70, 104, 198], &mut rng);
    let r = report(&capture(&frames, 25_000), am());
    assert_eq!(r.timing.coding, LineCoding::Manchester, "{:?}", r.timing.te);
    let data: Vec<_> = r.data_rows().collect();
    assert_eq!(data.len(), 3, "{:#?}", r.rows);
    assert_eq!(data[0].2.data(), bits.as_slice());
    assert!(matches!(data[2].1.kind, RowKind::Data { same_as: Some(0), .. }));
}

#[test]
fn fsk_nrz_is_read_one_bit_per_te() {
    let mut rng = Rng(0x1357_9BDF);
    let mut bits = rng.bits(96);
    (bits[0], bits[95]) = (true, true);
    let frames: Vec<_> = (0..2).map(|_| nrz_frame(100, &bits)).collect();
    let source = Source {
        frequency: 433_920_000,
        path: Some(RfModulation::FM),
        live: true,
        measured: measured(Some(47_600.0), true),
        sibling: Some(7),
        ..Source::default()
    };
    let r = report(&capture(&frames, 30_000), source);
    assert_eq!(r.keying, Keying::Fsk(Some(47_600.0)));
    assert_eq!(r.timing.coding, LineCoding::Nrz, "{:?}", r.timing.coding);
    assert!(r.is_regular(), "{:?}", r.timing.te);
    let data: Vec<_> = r.data_rows().collect();
    assert_eq!(data.len(), 2, "{:#?}", r.rows);
    assert_eq!(data[0].2.data(), bits.as_slice());
    assert_eq!(r.carrier_block_us, None);
    let all = joined(&r, 76);
    assert!(all.contains("2-FSK: two tones +/-47.6 kHz"), "{all}");
    assert!(all.contains("also gave pulses on the AM path (capture #07)"), "{all}");
    assert!(all.contains("HIGH (upper tone)"), "{all}");
}

#[test]
fn noise_is_not_a_signal() {
    let r = report(&noise(900, 77), am());
    assert!(!r.is_regular(), "{:?} coding {:?}", r.timing.te, r.timing.coding);
    assert_eq!(r.headline(), "no regular pulse train");
    assert_eq!(r.data_rows().count(), 0);
    assert!(r.similar.is_empty());
    let all = joined(&r, 76);
    assert!(all.contains("no regular pulse train here"), "{all}");
    assert!(!all.contains("BITS"), "{all}");
    // Nothing at all, and a single pulse, give a report too.
    for pairs in [Vec::new(), vec![p(true, 300)], vec![p(false, 0), p(true, 0)]] {
        let r = report(&pairs, Source::default());
        assert!(!r.is_regular());
        assert!(!text(&r, 60).is_empty());
    }
}

#[test]
fn carrier_blocks_are_called_carrier() {
    // An FSK fob through an OOK receiver: carrier for as long as each frame lasts.
    let frames: Vec<_> = (0..5).map(|i| vec![p(true, 82_000 + 100 * i)]).collect();
    let source = Source { preset: Some("AM650".to_string()), ..am() };
    let r = report(&capture(&frames, 20_000), source);
    assert_eq!(r.carrier_block_us, Some(82_400));
    assert_eq!(r.headline(), "carrier without on/off data");
    assert_eq!(r.rows.len(), 5, "{:#?}", r.rows);
    assert!(r.rows.iter().all(|row| matches!(row.kind, RowKind::Long { .. })));
    assert_eq!(r.frames_line().as_deref(), Some("5 long pulses, no data"));
    let all = joined(&r, 76);
    assert!(all.contains("OOK by the recording's preset (AM650)"), "{all}");
    assert!(all.contains("An FSK transmitter looks like this"), "{all}");
    assert!(!all.contains("WAVEFORM   row #1, one column is 0 us"), "{all}");
}

#[test]
fn no_rendered_row_is_wider_than_asked() {
    let (pwm, _) = pwm_capture();
    let mut rng = Rng(99);
    let bits = rng.bits(200);
    let manchester = capture(&[manchester_frame(250, 24, &bits, 4, &mut rng)], 40_000);
    let reference = ProtocolTiming { te_short: 400, te_long: 800, te_delta: 120, min_count_bit: 64 };
    let reports = [
        report(&pwm, Source { live: true, measured: measured(Some(2_000.0), true), sibling: Some(3), ..am() }),
        analyze(&pwm, am(), Some("A Protocol With A Long Name V12"), &[reference], None, &[]),
        report(&manchester, Source { preset: Some("FM476".to_string()), preset_deviation_hz: Some(47_607.0), ..am() }),
        report(&noise(700, 3), Source::default()),
        report(&capture(&[vec![p(true, 90_000)]], 20_000), am()),
    ];
    for (i, r) in reports.iter().enumerate() {
        for width in [MIN_WIDTH, 56, 60, 76, 100, 160] {
            for row in text(r, width) {
                assert!(row.chars().count() <= width, "report {i} at {width}: {:?} is {}", row, row.chars().count());
            }
        }
        // Narrower than the least width is rendered at the least width.
        assert_eq!(text(r, 10), text(r, MIN_WIDTH));
    }
}

#[test]
fn keying_comes_from_the_measurement_then_the_preset_then_the_path() {
    let with = |path, measured, preset_deviation_hz| Keying::of(&Source { path, measured, preset_deviation_hz, ..Source::default() });
    let (amp, fmp) = (Some(RfModulation::AM), Some(RfModulation::FM));
    assert_eq!(with(amp, measured(Some(20_000.0), true), None), Keying::Fsk(Some(20_000.0)));
    assert_eq!(with(amp, measured(None, false), Some(47_607.0)), Keying::Ook);
    assert_eq!(with(fmp, measured(None, false), None), Keying::Fsk(None));
    assert_eq!(with(amp, None, Some(47_607.0)), Keying::Fsk(Some(47_607.0)));
    assert_eq!(with(fmp, None, None), Keying::Fsk(None));
    assert_eq!(with(amp, None, None), Keying::Ook);
    assert_eq!(with(None, None, None), Keying::Unknown);
    assert_eq!(Keying::Fsk(Some(47_607.0)).label(), "2-FSK +/-47.6 kHz");
}

#[test]
fn widths_class_durations_by_the_nearer_width() {
    let w = LevelWidths { short: 400.0, long: Some(800.0) };
    assert_eq!([400, 310, 520].map(|d| w.class(d)), [Some(0); 3]);
    assert_eq!([800, 640, 1050].map(|d| w.class(d)), [Some(1); 3]);
    assert_eq!([0, 200, 590, 1200, 20_000].map(|d| w.class(d)), [None; 5]);
    let one = LevelWidths { short: 400.0, long: None };
    assert_eq!([400, 800].map(|d| one.class(d)), [Some(0), None]);
}

#[test]
fn bits_print_first_bit_first() {
    let bits = |s: &str, preamble| Bits { bits: s.chars().map(|c| c == '1').collect(), preamble };
    assert_eq!(bits("101001011", 0).hex(), "A58");
    assert_eq!(bits("101001011", 0).binary(), "101001011");
    assert_eq!(bits("1111000011110000", 4).hex(), "0F0");
    assert_eq!(bits("", 0).hex(), "");
    assert_eq!(bits("11", 5).data(), &[] as &[bool]);
}

/// Not a test: prints the report of every RAW .sub file under IMPORTS whose path holds the text in
/// KAT_ANALYZE (all of them without it), as an unknown capture would be reported.
///   KAT_ANALYZE=Lexus cargo test analysis::signal::tests::sweep -- --ignored --nocapture
#[test]
#[ignore = "prints reports for the files in IMPORTS"]
fn sweep_imports_reports() {
    fn walk(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
        let Ok(entries) = std::fs::read_dir(dir) else { return };
        for e in entries.flatten() {
            let path = e.path();
            if path.is_dir() {
                walk(&path, out);
            } else if path.extension().is_some_and(|x| x == "sub") {
                out.push(path);
            }
        }
    }
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("IMPORTS");
    let mut files = Vec::new();
    walk(&root, &mut files);
    files.sort();
    let want = std::env::var("KAT_ANALYZE").unwrap_or_default();
    let width: usize = std::env::var("KAT_WIDTH").ok().and_then(|w| w.parse().ok()).unwrap_or(76);
    for path in files.iter().filter(|f| f.to_string_lossy().contains(&want)) {
        let Ok(sub) = crate::export::flipper::import_sub(path) else { continue };
        let pairs = sub.pairs;
        let source = Source {
            frequency: sub.frequency,
            path: sub.preset.as_ref().and_then(|p| p.rf()),
            preset: sub.preset.as_ref().map(|p| p.label()),
            preset_deviation_hz: sub.preset.as_ref().and_then(|p| match p.modulation() {
                Some(crate::radio::preset::TxModulation::Fsk { deviation_hz }) => Some(deviation_hz),
                _ => None,
            }),
            ..Source::default()
        };
        let r = report(&pairs, source);
        println!("\n===== {} ({} pairs): {}", path.strip_prefix(&root).unwrap().display(), pairs.len(), r.headline());
        if std::env::var("KAT_BRIEF").is_ok() {
            println!("      {}", r.frames_line().unwrap_or_default());
            continue;
        }
        for row in text(&r, width) {
            println!("{row}");
        }
    }
}

/// Long pulses of all lengths at no pace are no repeat, the widths get no TE multiples, and the
/// waveform shows the capture rather than one long pulse.
#[test]
fn carrier_blocks_are_reported_as_what_they_are() {
    let mut rng = Rng(0x51ab);
    let mut pulses = Vec::new();
    for i in 0..24u32 {
        pulses.push(p(true, 3_000 + rng.next() % 30_000));
        pulses.push(p(false, 60_000 + (i % 5) * 90_000 + rng.next() % 9_000));
        // A few stray edges in the pause, as an AM receiver gives on an FSK signal.
        for _ in 0..4 {
            pulses.push(p(true, 60 + rng.next() % 400));
            pulses.push(p(false, 60 + rng.next() % 200));
        }
        pulses.push(p(true, 80));
        pulses.push(p(false, 70_000));
    }
    let r = report(&pulses, am());
    assert_eq!(r.headline(), "carrier without on/off data");
    assert!(r.repeat.is_none_or(|rep| rep.every_us == 0), "{:?}", r.repeat);
    let all = joined(&r, 76);
    assert!(!all.contains("have the same shape"), "{all}");
    assert!(all.contains("TE           none"), "{all}");
    assert!(!all.contains("Symbol rate") && !all.contains("baud"), "{all}");
    assert!(all.contains("WAVEFORM   the capture from its start"), "{all}");
    assert!(all.contains("RAW   the capture from its start in us"), "{all}");
}

/// Frames that keep their distance repeat with a pace; pressed again later, they repeat without.
#[test]
fn a_repeat_has_a_pace_only_when_it_is_kept() {
    let mut rng = Rng(0x77);
    let bits = rng.bits(40);
    let frames: Vec<_> = (0..6).map(|_| pwm_frame(400, &bits, 3, &mut rng)).collect();
    let steady = report(&capture(&frames, 20_000), am());
    let rep = steady.repeat.expect("repeat");
    // A preamble and a frame each time.
    assert_eq!((rep.rows, rep.times), (2, 6), "{:#?}", steady.rows);
    assert!(rep.every_us > 20_000, "{rep:?}");
    assert!(steady.frames_line().unwrap().contains(", every "), "{:?}", steady.frames_line());

    // The same frames with pauses of every length between them.
    let mut pulses = Vec::new();
    for (i, frame) in frames.iter().enumerate() {
        pulses.extend(frame.iter().copied());
        pulses.push(p(true, 400));
        pulses.push(p(false, 20_000 + 170_000 * (i as u32 * 7 % 5)));
    }
    let uneven = report(&pulses, am());
    assert_eq!(uneven.repeat.map(|r| r.every_us), Some(0), "{:#?}", uneven.rows);
    assert!(!uneven.frames_line().unwrap().contains("every"), "{:?}", uneven.frames_line());
    assert!(joined(&uneven, 76).contains("at no steady pace"));
}
