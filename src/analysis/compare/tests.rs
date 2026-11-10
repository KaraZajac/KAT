//! Tests of the comparison, on made-up frames.

use super::*;

/// Small deterministic PRNG (xorshift32) for reproducible bits.
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
        (0..n).map(|_| (self.next() >> 16) & 1 == 1).collect()
    }
}

/// `width` bits of `value`, the high bit first.
fn msb(value: u64, width: usize) -> Vec<bool> {
    (0..width).rev().map(|i| (value >> i) & 1 == 1).collect()
}

/// `width` bits of `value`, the low bit first.
fn lsb(value: u64, width: usize) -> Vec<bool> {
    (0..width).map(|i| (value >> i) & 1 == 1).collect()
}

/// Frame number `i` (named "#01" on) of `parts` one after another.
fn frame(i: usize, group: Option<&str>, parts: &[Vec<bool>]) -> Frame {
    Frame { name: format!("#{:02}", i + 1), group: group.map(String::from), bits: parts.concat() }
}

fn compared(frames: Vec<Frame>) -> Comparison {
    compare(frames, "made-up frames".to_string(), Grouping::Button, Vec::new())
}

/// The letter of every column's class, as under the rows of bits.
fn letters(c: &Comparison) -> String {
    c.classes.iter().map(|class| class.letter()).collect()
}

fn text(lines: &[Line]) -> Vec<String> {
    lines.iter().map(|l| l.iter().map(|s| s.text.as_str()).collect()).collect()
}

fn report(c: &Comparison, width: usize) -> String {
    text(&c.render(width)).join("\n")
}

#[test]
fn the_chance_of_so_many_hits_is_the_binomial_tail() {
    let tails = log2_tails(4, 1.0 / 7.0);
    assert_eq!(tails[0], 0.0);
    // Four of four: (1/7)^4. Three or more: that plus 4 * (1/7)^3 * (6/7).
    assert!((tails[4] - (1.0f64 / 2401.0).log2()).abs() < 1e-9, "{tails:?}");
    assert!((tails[3] - (25.0f64 / 2401.0).log2()).abs() < 1e-9, "{tails:?}");
    assert!(tails.windows(2).all(|w| w[0] >= w[1]), "{tails:?}");
    // A chance far below what a float holds still has a logarithm.
    let small = log2_tail(63, 63, 1.0 / 16_777_215.0);
    assert!((small + 63.0 * 24.0).abs() < 0.01, "{small}");
    assert_eq!(log2_tail(10, 0, 0.5), 0.0);
    assert!((log2_tail(10, 10, 0.5) + 10.0).abs() < 1e-9);
}

/// A counter among fixed bits: the columns a step reached are the counter, whatever its width in
/// the frame, and its high bits stay with the fixed ones.
#[test]
fn a_counter_among_fixed_bits_is_found() {
    let frames = (0..8).map(|i| frame(i, None, &[msb(0xB444, 16), msb(0x3E + i as u64, 8), msb(0xA5, 8)])).collect();
    let c = compared(frames);
    // 0x3E to 0x45: bit 7 never changes.
    assert_eq!(letters(&c), format!("{}.CCCCCCC{}", ".".repeat(16), ".".repeat(8)));
    assert_eq!(
        c.counters,
        [Counter { start: 17, len: 7, low_first: false, down: false, steps: (1, 1), changes: 7, fits: 7, sure: true }]
    );
    assert_eq!(c.counters[0].value(&c.frames[0]), 0x3E);
    assert_eq!(c.counters[0].value(&c.frames[7]), 0x45);
    let report = report(&c, 80);
    assert!(report.contains("Counter      bits 17-23 count up by 1"), "{report}");
    assert!(report.contains("counts up by 1: 0x3E to 0x45"), "{report}");
    assert!(report.contains("A counter's high bits do not change in a few presses"), "{report}");
}

#[test]
fn a_counter_sent_low_bit_first_is_found() {
    let frames = (0..8).map(|i| frame(i, None, &[msb(0xB444, 16), lsb(0x3E + i as u64, 8), msb(0xA5, 8)])).collect();
    let c = compared(frames);
    assert_eq!(letters(&c), format!("{}CCCCCCC.{}", ".".repeat(16), ".".repeat(8)));
    let counter = &c.counters[0];
    assert!(counter.low_first && !counter.down && counter.sure, "{counter:?}");
    assert_eq!((counter.value(&c.frames[0]), counter.value(&c.frames[7])), (0x3E, 0x45));
    assert!(report(&c, 80).contains("counts up by 1, low bit first: 0x3E to 0x45"));
}

/// A counter that goes down (an inverted one does), and the same frames the other way round.
#[test]
fn a_counter_that_goes_down_is_found() {
    let down: Vec<Frame> = (0..8).map(|i| frame(i, None, &[msb(0x45 - i as u64, 8), msb(0xA5, 8)])).collect();
    let c = compared(down.clone());
    assert_eq!(letters(&c), ".CCCCCCC........");
    assert!(c.counters[0].down && c.counters[0].sure, "{:?}", c.counters);
    assert!(report(&c, 80).contains("counts down by 1: 0x45 to 0x3E"));

    let up = compared(down.into_iter().rev().collect());
    assert!(!up.counters[0].down, "{:?}", up.counters);
}

/// Beside a checksum that changes with every frame and far from an encrypted part, the counter is
/// the counter's columns and no more.
#[test]
fn a_counter_beside_random_bits_keeps_to_its_own_columns() {
    for seed in 1..=20 {
        let mut rng = Rng(seed * 7919);
        let frames = (0..10)
            .map(|i| frame(i, None, &[rng.bits(32), msb(0xABC, 12), msb(0x19 + i as u64, 8), rng.bits(4)]))
            .collect();
        let c = compared(frames);
        // 0x19 to 0x22: six bits change.
        assert_eq!(c.counters.len(), 1, "seed {seed}: {:?}", c.counters);
        let counter = &c.counters[0];
        assert_eq!((counter.start, counter.len, counter.low_first, counter.down), (46, 6, false, false), "seed {seed}");
        assert!(counter.sure && counter.steps == (1, 1), "seed {seed}: {counter:?}");
        let letters = letters(&c);
        assert_eq!(&letters[32..52], "..............CCCCCC", "seed {seed}");
        assert!(!letters.contains('B'), "seed {seed}: {letters}");
    }
}

/// Random bits are no counter: never a sure one, and seldom a likely one.
#[test]
fn random_bits_are_no_counter() {
    let (mut sure, mut likely) = (0, 0);
    let runs = 200;
    for seed in 1..=runs {
        let mut rng = Rng(seed * 2_654_435);
        let presses = 4 + (seed as usize % 9);
        let frames = (0..presses).map(|i| frame(i, None, &[msb(0x5A, 8), rng.bits(40)])).collect();
        let c = compared(frames);
        sure += c.counters.iter().filter(|counter| counter.sure).count();
        likely += usize::from(!c.counters.is_empty());
    }
    assert_eq!(sure, 0);
    assert!(likely <= runs as usize / 8, "{likely} of {runs}");
}

/// Five presses show a counter that may be one; a few more make it sure. With an encrypted part in
/// the frame, where three bits count by accident now and then, it takes one press more.
#[test]
fn more_presses_make_a_counter_sure() {
    let plain = |presses: usize| {
        compared((0..presses).map(|i| frame(i, None, &[msb(0xABC, 12), msb(0x11 + i as u64, 8)])).collect())
    };
    assert!(plain(3).counters.is_empty(), "{:?}", plain(3).counters);
    let five = plain(5);
    assert_eq!(letters(&five), format!("{}CCC", ".".repeat(17)));
    assert!(five.counters[0].sure, "{:?}", five.counters);

    let mut rng = Rng(77);
    let mut with_random = |presses: usize| {
        compared(
            (0..presses).map(|i| frame(i, None, &[rng.bits(32), msb(0xABC, 12), msb(0x11 + i as u64, 8)])).collect(),
        )
    };
    let five = with_random(5);
    assert_eq!(five.counters.len(), 1, "{:?}", five.counters);
    assert_eq!((five.counters[0].start, five.counters[0].len, five.counters[0].sure), (49, 3, false));
    assert!(report(&five, 80).contains("bits 49-51 may count up by 1"), "{}", report(&five, 80));
    let seven = with_random(7);
    assert!(seven.counters[0].sure && seven.counters[0].start == 49, "{:?}", seven.counters);
}

/// Presses in between that were not captured: steps of more than one, and a jump.
#[test]
fn a_counter_with_missed_presses_is_found() {
    let mut value = 0x120u64;
    let steps = [1, 2, 1, 3, 1, 1, 2, 1, 1];
    let mut frames = vec![frame(0, None, &[msb(0x77, 8), msb(value, 12)])];
    for (i, step) in steps.into_iter().enumerate() {
        value += step;
        frames.push(frame(i + 1, None, &[msb(0x77, 8), msb(value, 12)]));
    }
    let c = compared(frames);
    assert_eq!(c.counters.len(), 1, "{:?}", c.counters);
    assert_eq!(c.counters[0].steps, (1, 3));
    assert!(report(&c, 80).contains("counts up by 1 to 3: 0x0 to 0xD"), "{}", report(&c, 80));

    // Ten presses, and forty not captured after the fifth.
    let frames = (0..10u64)
        .map(|i| frame(i as usize, None, &[msb(0x77, 8), msb(0x100 + i + if i >= 5 { 40 } else { 0 }, 12)]))
        .collect();
    let c = compared(frames);
    assert_eq!(c.counters.len(), 1, "{:?}", c.counters);
    assert_eq!((c.counters[0].changes, c.counters[0].fits, c.counters[0].steps), (9, 8, (1, 1)));
    assert!(report(&c, 80).contains("counts up by 1, 1 jump"), "{}", report(&c, 80));
}

/// What the frames of each button tell about its bits is counted from their number, and the report
/// says what is missing in its terms.
#[test]
fn the_buttons_bits_need_frames_enough() {
    // Three frames of each of two buttons: a random column is the same within both once in 16 and
    // then differs between them once in 2. Among 6 columns that is 6 in 32.
    assert!((group_evidence(&[3, 3], 6) - (32f64 / 6.0).log2()).abs() < 1e-9);
    assert!((group_evidence(&[5, 5, 5], 35) - (4096f64 / (0.75 * 35.0)).log2()).abs() < 1e-9);
    assert_eq!(frames_for_groups(2, 6), 3);
    assert_eq!(frames_for_groups(2, 34), 5);
    assert_eq!(frames_for_groups(3, 34), 4);

    // A fixed code: each button's frame came twice, and no number of presses gives another.
    let buttons = [("Lock", 1u64), ("Unlock", 2), ("Trunk", 4)];
    let fixed: Vec<Frame> = (0..6)
        .map(|i| frame(i, Some(buttons[i / 2].0), &[msb(0xABCDE, 20), msb(buttons[i / 2].1, 4)]))
        .collect();
    let c = compared(fixed);
    assert_eq!(letters(&c), format!("{}.???", ".".repeat(20)));
    assert!(report(&c, 80).contains("Each button gives one frame here (a fixed code, or one press of each)"), "{}", report(&c, 80));

    // The button inside the encrypted part: frames enough, and no bit follows it.
    let mut rng = Rng(77);
    let hidden: Vec<Frame> =
        (0..10).map(|i| frame(i, Some(buttons[i / 5].0), &[rng.bits(32), msb(0x0C0E_D06, 28)])).collect();
    let c = compared(hidden);
    assert!(!letters(&c).contains('B'), "{}", letters(&c));
    assert!(report(&c, 80).contains("No bit follows the button: it may be among the bits with no pattern"), "{}", report(&c, 80));
}

/// Bits that are the same for every press of one button and differ between buttons follow the
/// button; the encrypted part does not.
#[test]
fn the_bits_that_follow_the_button_are_found() {
    let mut rng = Rng(4242);
    let buttons = [("Lock", 1u64), ("Unlock", 2), ("Trunk", 4)];
    let frames: Vec<Frame> = (0..15)
        .map(|i| {
            let (name, code) = buttons[i / 5];
            frame(i, Some(name), &[rng.bits(32), msb(0x0C0E_D06, 28), msb(code, 4)])
        })
        .collect();
    let c = compared(frames.clone());
    assert_eq!(&letters(&c)[32..], format!("{}.BBB", ".".repeat(28)));
    assert!(c.counters.is_empty(), "{:?}", c.counters);
    assert_eq!(c.classes[61], Class::Group { sure: true });
    assert!(letters(&c)[..32].chars().all(|l| l == '?'), "{}", letters(&c));
    let report80 = report(&c, 80);
    assert!(report80.contains("Button       bits 61-63 follow the button (Lock, Unlock, Trunk)"), "{report80}");
    assert!(report80.contains("follows the button: Lock 0x1, Unlock 0x2, Trunk 0x4"), "{report80}");
    assert!(report80.contains("B button"), "{report80}");
    assert!(report80.contains("32 bits with no pattern"), "{report80}");

    // Two presses of each button are too few to tell: nothing is claimed, and the report says so.
    let few: Vec<Frame> = [0, 1, 5, 6].into_iter().map(|i| frames[i].clone()).collect();
    let c = compared(few);
    assert!(!letters(&c).contains('B'), "{}", letters(&c));
    assert!(report(&c, 80).contains("Too few frames of each button to tell its bits"), "{}", report(&c, 80));
    assert!(report(&c, 80).contains("about 4 different ones of each would do."), "{}", report(&c, 80));
    // Five of each of two buttons: the bits are found, with less weight than from three buttons.
    let c = compared(frames[..10].to_vec());
    assert_eq!(c.classes[62], Class::Group { sure: false }, "{}", letters(&c));
    assert!(report(&c, 80).contains("bits 62-63 may follow the button (Lock, Unlock)"), "{}", report(&c, 80));

    // Groups made by hand are called groups.
    let by_hand = frames
        .iter()
        .enumerate()
        .map(|(i, f)| Frame { group: Some(["A", "B", "C"][i / 5].to_string()), ..f.clone() })
        .collect();
    let c = compare(by_hand, "made-up frames".to_string(), Grouping::Group, Vec::new());
    let report = report(&c, 80);
    assert!(report.contains("Group        bits 61-63 follow the group (A, B, C)"), "{report}");
    assert!(report.contains("follows the group: A 0x1, B 0x2, C 0x4") && report.contains("B group"), "{report}");
}

/// A button's bits right above a counter that uses all of its width: the counter does not take
/// them in (no step carries into them), and they follow the button.
#[test]
fn a_counter_does_not_take_in_the_buttons_bits() {
    let frames = (0..10)
        .map(|i| {
            let (name, code) = if i < 5 { ("Lock", 1) } else { ("Unlock", 2) };
            frame(i, Some(name), &[msb(0xE7, 8), msb(code, 4), msb(6 + i as u64, 4)])
        })
        .collect();
    let c = compared(frames);
    assert_eq!(letters(&c), "..........BBCCCC");
    assert_eq!((c.counters[0].start, c.counters[0].len, c.counters[0].sure), (12, 4, true));
    assert_eq!(c.classes[10], Class::Group { sure: true });
}

/// With one button only, or none known, the report says what would show the button's bits.
#[test]
fn the_report_says_what_the_frames_cannot_tell() {
    let mut rng = Rng(99);
    let mut frames: Vec<Frame> = (0..6).map(|i| frame(i, Some("Lock"), &[msb(0x3C, 8), rng.bits(16)])).collect();
    let c = compared(frames.clone());
    let one_button = report(&c, 80);
    assert!(one_button.contains("Every frame is of one button (Lock): add presses of another"), "{one_button}");
    assert!(one_button.contains("Bits with no pattern are encrypted or hashed"), "{one_button}");

    frames.iter_mut().for_each(|f| f.group = None);
    let unnamed = report(&compared(frames.clone()), 80);
    assert!(unnamed.contains("No button is known for these frames"), "{unnamed}");
    assert!(unnamed.contains(":compare 1-5 / 6-10"), "{unnamed}");

    // Two frames: what differs, and no more.
    let two = compared(frames[..2].to_vec());
    assert!(two.classes.iter().all(|c| matches!(c, Class::Fixed | Class::Varies)), "{}", letters(&two));
    assert!(report(&two, 80).contains("Two different frames show only which bits differ"), "{}", report(&two, 80));

    // The same frame every time.
    let same = compared((0..4).map(|i| frame(i, None, &[msb(0x1234, 16)])).collect());
    assert_eq!(letters(&same), ".".repeat(16));
    let same = report(&same, 80);
    assert!(same.contains("Fixed        16 of 16 bits"), "{same}");
    assert!(same.contains("Every frame is the same"), "{same}");
    assert!(same.contains("bits 0-15  fixed: 0x1234"), "{same}");

    // Nothing to compare.
    let none = compared(Vec::new());
    assert_eq!(none.bits, 0);
    assert!(report(&none, 80).contains("0 frames of 0 bits"), "{}", report(&none, 80));
}

/// A bit that flips with every frame, and one that flips once in sixteen frames.
#[test]
fn a_bit_that_changes_rarely_or_every_time_is_named() {
    let frames = (0..16u64)
        .map(|i| frame(i as usize, None, &[msb(i & 1, 1), msb(0x55, 7), msb(u64::from(i >= 9), 1), msb(0x2A, 7)]))
        .collect();
    let c = compared(frames);
    assert_eq!(letters(&c), "e.......r.......");
    let report = report(&c, 80);
    assert!(report.contains("1 bit that changes rarely, 1 bit that changes with every frame"), "{report}");
    assert!(report.contains("changes rarely: 0x0, 0x1"), "{report}");
    assert!(report.contains("r rarely  e every frame"), "{report}");
}

/// Frames of unequal length are compared over the shortest one's bits.
#[test]
fn frames_are_cut_to_the_shortest() {
    let mut frames: Vec<Frame> = (0..3).map(|i| frame(i, None, &[msb(0xF0 + i as u64, 8)])).collect();
    frames[1].bits.extend([true, false, true]);
    let c = compared(frames);
    assert_eq!(c.bits, 8);
    assert!(c.frames.iter().all(|f| f.bits.len() == 8));
}

#[test]
fn bits_read_as_hex_from_the_first_one() {
    assert_eq!(hex(&msb(0x1F, 5)), "0x1F");
    assert_eq!(hex(&msb(0x3, 2)), "0x3");
    assert_eq!(hex(&msb(0x0A5, 12)), "0x0A5");
    assert_eq!(hex(&[true]), "0x1");
    assert_eq!(hex(&[]), "0x");
    assert_eq!(range(7, 1), "bit 7");
    assert_eq!(range(8, 32), "bits 8-39");
}

/// The frames a test of the layout takes: an encrypted part, a serial, a button, a counter and a
/// checksum, with long names.
fn remote(presses: usize) -> Comparison {
    let mut rng = Rng(31337);
    let frames = (0..presses)
        .map(|i| {
            let (name, code) = [("Lock", 1u64), ("Unlock", 2), ("Trunk release", 4)][(i / 5) % 3];
            let parts = [rng.bits(32), msb(0x70C0_ED06, 32), msb(code, 4), msb(0x26 + i as u64, 16), rng.bits(4)];
            let mut f = frame(i, Some(name), &parts);
            if i % 4 == 3 {
                f.name = format!("#{:02}/2", i + 1);
            }
            f
        })
        .collect();
    compare(
        frames,
        "the bits as sent (Manchester)".to_string(),
        Grouping::Button,
        vec!["3 rows of other lengths left out: #02, #05, #11.".to_string()],
    )
}

/// No line is wider than the report at any width, and none is cut.
#[test]
fn every_line_fits_the_width() {
    let comparisons = [remote(15), remote(2), remote(1), compared(Vec::new())];
    for c in &comparisons {
        for width in MIN_WIDTH..=110 {
            for line in text(&c.render(width)) {
                assert!(line.chars().count() <= width, "width {width}: {line:?}");
                assert!(!line.contains('…') || line.contains("Trunk"), "width {width}: {line:?}");
            }
        }
    }
    // Narrower than the narrowest overlay: laid out for the narrowest.
    assert_eq!(text(&comparisons[0].render(20)), text(&comparisons[0].render(MIN_WIDTH)));
}

/// The rows of bits: blocks of 32 columns, of 64 where they fit, a letter under every column, and
/// one line for a block that never changes.
#[test]
fn the_rows_of_bits_line_up() {
    let c = remote(15);
    assert_eq!(c.bits, 88);
    let narrow = text(&c.render(MIN_WIDTH));
    let at = narrow.iter().position(|l| l.starts_with(" BITS")).unwrap();
    assert_eq!(narrow[at], " BITS   . fixed  C counter  B button  ? no pattern");
    assert_eq!(narrow[at + 1], " bits 0-31");
    // The label column is as wide as the longest label (cut at 14), then eight bits a group.
    let first = &narrow[at + 2];
    assert!(first.starts_with(" #01 Lock       "), "{first:?}");
    assert_eq!(first.chars().count(), 16 + 35, "{first:?}");
    assert!(narrow.iter().any(|l| l.starts_with(" #12/2 Trunk r… ")), "{narrow:#?}");
    // Under the fifteen frames, the letters.
    assert_eq!(narrow[at + 17], format!(" {:14} ???????? ???????? ???????? ????????", ""));
    // The serial's block never changes: one line.
    assert_eq!(narrow[at + 18], "");
    assert_eq!(narrow[at + 19], " bits 32-63  fixed: 0x70C0ED06");
    assert_eq!(narrow[at + 20], " bits 64-87");

    let wide = text(&c.render(100));
    let at = wide.iter().position(|l| l.starts_with(" BITS")).unwrap();
    assert_eq!(wide[at + 1], " bits 0-63");
    assert_eq!(wide[at + 2].chars().count(), 16 + 71, "{:?}", wide[at + 2]);
    let last = wide.last().unwrap();
    assert_eq!(last.trim_start(), ".BBB.... .......C CCCC????");

    // Every bit of every frame is there, in order.
    let shown: String = wide
        .iter()
        .filter(|l| l.starts_with(" #01 Lock"))
        .flat_map(|l| l[16..].chars().filter(|ch| *ch != ' ').collect::<Vec<_>>())
        .collect();
    let bits: String = c.frames[0].bits.iter().map(|b| if *b { '1' } else { '0' }).collect();
    assert_eq!(shown, bits);
}

/// The columns of one class are one span each, in the class's tone: the colours mark the fields.
#[test]
fn the_bits_carry_their_class_as_a_tone() {
    let c = remote(15);
    let lines = c.render(100);
    let row = lines.iter().rev().find(|l| l.first().is_some_and(|s| s.text.starts_with(" #15"))).unwrap();
    // The last block: the button's three bits... are in the first; here the counter and the tail.
    let tones: Vec<Tone> = row.iter().skip(1).map(|s| s.tone).collect();
    assert!(tones.contains(&Tone::Good) && tones.contains(&Tone::Plain), "{row:?}");
    assert!(row.windows(2).skip(1).all(|w| w[0].tone != w[1].tone), "{row:?}");
}

/// Rows of one frame read from a few bits earlier or later are lined up by what they hold: what
/// was caught of the preamble and what follows the frame differ from row to row.
#[test]
fn rows_read_from_another_bit_on_are_lined_up() {
    let mut rng = Rng(31337);
    let (lost, extra) = ([0, 3, 1, 5, 2, 0, 4, 1], [0, 1, 2, 0, 1, 3, 0, 2]);
    let rows: Vec<Vec<bool>> = (0..8)
        .map(|i| {
            let parts = [vec![false; 40], msb(0xF00D_5EED, 32), msb(0x19 + i as u64, 8), rng.bits(16), vec![false; extra[i]]];
            parts.concat()[lost[i]..].to_vec()
        })
        .collect();
    let lined_up = align(&rows.iter().map(Vec::as_slice).collect::<Vec<_>>(), 8);
    // From the bit the shortest preamble starts at to the last bit every row has.
    assert_eq!(lined_up.iter().map(Vec::len).collect::<Vec<_>>(), [91; 8]);
    for (i, row) in lined_up.iter().enumerate() {
        assert_eq!(row[..67], [vec![false; 35], msb(0xF00D_5EED, 32)].concat(), "row {i}");
        assert_eq!(row[67..75], msb(0x19 + i as u64, 8), "row {i}");
    }
    let c = compared(lined_up.into_iter().enumerate().map(|(i, bits)| frame(i, None, &[bits])).collect());
    assert_eq!((c.counters.len(), c.counters[0].start + c.counters[0].len), (1, 75), "{:?}", c.counters);

    // Rows that agree as they are stay as they are; a row too far off to move is cut like the rest.
    let a = rng.bits(48);
    let mut b = a.clone();
    b[40] = !b[40];
    assert_eq!(align(&[&a, &b], 8), [a.clone(), b.clone()]);
    assert_eq!(align(&[&a, &b[..44]], 8), [a[..44].to_vec(), b[..44].to_vec()]);
    assert!(align(&[], 8).is_empty());
    assert_eq!(align(&[&a], 8), [a]);
}
