//! Tests of the Compare overlay's side of the comparison: which frames the captures picked give,
//! and recordings in IMPORTS compared against what their decoders say.

use std::path::Path;

use super::*;
use crate::analysis::compare::Class;
use crate::capture::CaptureStatus;

/// A capture as a decoder leaves it, without its pulses.
fn decoded(id: u32, protocol: &str, serial: u32, button: u8, data: u64, bits: usize) -> Capture {
    let mut c = Capture::from_pairs(id, 433_920_000, Vec::new());
    c.protocol = Some(protocol.to_string());
    c.serial = Some(serial);
    c.button = Some(button);
    c.data = data;
    c.data_count_bit = bits;
    c.status = CaptureStatus::Decoded;
    c
}

fn unknown(id: u32, frequency: u32) -> Capture {
    Capture::from_pairs(id, frequency, Vec::new())
}

/// The captures as `c` picks them: each in the group of its button.
fn picks(captures: &[Capture]) -> Vec<Pick<'_>> {
    captures.iter().map(|capture| Pick { capture, group: button_of(capture) }).collect()
}

fn bits(text: &str) -> Vec<bool> {
    text.chars().filter(|c| !c.is_whitespace()).map(|c| c == '1').collect()
}

fn rows(id: u32, decoded: bool, rows: &[&str]) -> Rows {
    Rows { id, group: None, decoded, rows: rows.iter().map(|r| bits(r)).collect() }
}

fn names(frames: &[Frame]) -> Vec<&str> {
    frames.iter().map(|f| f.name.as_str()).collect()
}

/// A capture's button is the label its user gave it, else what its decoder read (the code where
/// KAT has no name for it); an unknown capture has none until it is labelled.
#[test]
fn a_button_is_its_label_or_what_the_decoder_read() {
    let mut c = decoded(1, "Acme V3", 0xA1B2C3, 0x02, 0x1234, 24);
    assert_eq!(button_of(&c).as_deref(), Some("Unlock"));
    c.button = Some(0x0B);
    assert_eq!(button_of(&c).as_deref(), Some("0xB"));
    c.command = Some("  ".to_string());
    assert_eq!(button_of(&c).as_deref(), Some("0xB"));
    c.command = Some(" Boot ".to_string());
    assert_eq!(button_of(&c).as_deref(), Some("Boot"));

    let mut u = unknown(2, 433_920_000);
    assert_eq!(button_of(&u), None);
    u.command = Some("Lock".to_string());
    assert_eq!(button_of(&u).as_deref(), Some("Lock"));
}

/// The decoded data's bits are those of the frame, the first sent first; of a frame longer than
/// the 64 bits KAT keeps, those 64.
#[test]
fn decoded_bits_are_the_frames_bits_in_order() {
    assert_eq!(decoded_bits(&decoded(1, "Acme V3", 1, 1, 0b1011_0010, 8)), bits("1011 0010"));
    assert_eq!(decoded_bits(&decoded(1, "Acme V3", 1, 1, 0b1011_0010, 6)), bits("11 0010"));
    let long = decoded_bits(&decoded(1, "Acme V3", 1, 1, 0x8000_0000_0000_0001, 68));
    assert_eq!((long.len(), long[0], long[1], long[63]), (64, true, false, true));
    assert!(decoded_bits(&unknown(2, 433_920_000)).is_empty());
}

/// The decoded data of the protocol and length most picks have is compared; the others are named
/// as left out.
#[test]
fn the_decoded_side_takes_the_frames_most_captures_have() {
    let mut list = vec![
        decoded(1, "Acme V3", 7, 0x01, 0xA1_0010, 24),
        decoded(2, "Acme V3", 7, 0x01, 0xA1_0011, 24),
        decoded(3, "Borg V0", 9, 0x02, 0xFFFF, 16),
        decoded(4, "Acme V3", 7, 0x02, 0xA2_0012, 24),
        unknown(5, 433_920_000),
    ];
    let side = decoded_side(&picks(&list), Grouping::Button, &["#09 not found.".to_string()]).unwrap();
    let c = &side.comparison;
    assert_eq!(side.label, "decoded data");
    assert_eq!(names(&c.frames), ["#01", "#02", "#04"]);
    assert_eq!(c.frames[2].group.as_deref(), Some("Unlock"));
    assert_eq!((c.bits, c.what.as_str()), (24, "the decoded data of Acme V3"));
    assert_eq!(c.notes, ["#09 not found.", "#03, #05 are left out: not Acme V3 frames of 24 bits."]);

    // As many of one kind as of the other: the kind named first.
    list.truncate(3);
    list.remove(1);
    let c = decoded_side(&picks(&list), Grouping::Button, &[]).unwrap().comparison;
    assert_eq!((names(&c.frames), c.notes), (vec!["#01"], vec!["#03 is left out: not Acme V3 frames of 24 bits.".to_string()]));

    // A frame longer than the data KAT keeps of it.
    let long = [decoded(1, "Acme V5", 7, 1, 1, 68), decoded(2, "Acme V5", 7, 1, 2, 68)];
    let c = decoded_side(&picks(&long), Grouping::Button, &[]).unwrap().comparison;
    assert_eq!((c.bits, c.notes.len()), (64, 1));
    assert!(c.notes[0].starts_with("A frame is 68 bits long: these are the first 64"), "{:?}", c.notes);

    assert!(decoded_side(&picks(&[unknown(1, 433_920_000)]), Grouping::Button, &[]).is_none());
}

/// A decoded capture's pulses start with the repeats of the press before: its own frame is its
/// last row. An unknown capture gives every different row it holds.
#[test]
fn a_capture_gives_its_own_frames_of_the_bits_as_sent() {
    let (press1, press2, press3) = ("1010 1100 0001", "1010 1100 0010", "1010 1100 0011");
    let decoded_ones = [rows(1, true, &[press1]), rows(2, true, &[press1, press1, press2]), rows(3, true, &[press2, press3])];
    let (frames, notes) = on_air_frames(&decoded_ones);
    assert_eq!(names(&frames), ["#01", "#02", "#03"]);
    assert_eq!(frames.iter().map(|f| f.bits.clone()).collect::<Vec<_>>(), [bits(press1), bits(press2), bits(press3)]);
    assert!(notes.is_empty(), "{notes:?}");

    // Unknown: the repeats of a row are one frame, another row is another.
    let unknown_ones = [rows(7, false, &[press1, press1, press2, press1]), rows(8, false, &[press3, press3])];
    let (frames, notes) = on_air_frames(&unknown_ones);
    assert_eq!(names(&frames), ["#07/1", "#07/2", "#08"]);
    assert_eq!(frames[1].bits, bits(press2));
    assert!(notes.is_empty(), "{notes:?}");

    assert_eq!(on_air_frames(&[]).0.len() + on_air_frames(&[rows(1, false, &[])]).0.len(), 0);
}

/// Rows of the length most captures have are compared (the longer on a tie); what is left out is
/// said, by capture where a whole capture is.
#[test]
fn rows_of_one_length_are_compared() {
    let (long1, long2, short) = ("1111 0000 1010 0101 0001", "1111 0000 1010 0101 0010", "1100 1");
    let list = [
        rows(1, false, &[short, long1]),
        rows(2, false, &[long2]),
        rows(3, false, &[short]),
        rows(4, false, &[]),
        rows(5, true, &[short, long1]),
        rows(6, true, &[long1, short]),
    ];
    let (frames, notes) = on_air_frames(&list);
    // #05 and #06 are decoded: only the last row of each is its own frame.
    assert_eq!(names(&frames), ["#01", "#02", "#05"]);
    assert_eq!(notes, ["#03, #04, #06 are left out: no row of that length (a: how a capture is read).", "Rows of other lengths are left out."]);

    // Two kinds in as many captures: the longer rows.
    let (frames, _) = on_air_frames(&[rows(1, false, &[short, long1]), rows(2, false, &[long2, short])]);
    assert_eq!(frames.iter().map(|f| f.bits.len()).collect::<Vec<_>>(), [20, 20]);
}

/// Rows a few bits apart in length are one kind: they are lined up by what they hold, and the
/// report says so.
#[test]
fn rows_of_ragged_lengths_are_lined_up() {
    let frame = |counter: &str| format!("0000 0000 0000 0000 1111 0101 1100 0011 1010 0110 {counter}");
    let list = [
        rows(1, true, &[&frame("0001")]),
        rows(2, true, &[&format!("000{}0", frame("0010"))]),
        rows(3, true, &[&frame("0011")[2..]]),
    ];
    let (frames, notes) = on_air_frames(&list);
    // From where the shortest preamble starts to the end of the frame.
    assert_eq!(frames.iter().map(|f| f.bits.clone()).collect::<Vec<_>>(), ["0001", "0010", "0011"].map(|c| bits(&frame(c)[2..])));
    assert_eq!(notes.len(), 1, "{notes:?}");
    assert!(notes[0].starts_with("The rows are 42 to 48 bits long: they are lined up by what they hold"), "{notes:?}");
}

/// `c` compares the captures of one remote that the list shows, in the order they came.
#[test]
fn the_remote_of_a_capture_is_its_protocol_and_serial() {
    let mut list = vec![
        decoded(1, "Acme V3", 7, 1, 1, 24),
        decoded(2, "Borg V0", 7, 1, 1, 24),
        decoded(3, "Acme V3", 7, 2, 2, 24),
        unknown(4, 433_920_000),
        unknown(5, 433_920_000),
        unknown(6, 315_000_000),
        decoded(7, "Acme V3", 8, 1, 3, 24),
    ];
    let all: Vec<usize> = (0..list.len()).collect();
    assert_eq!(same_remote(&list, &all, 0), [0, 2]);
    assert_eq!(same_remote(&list, &all, 1), [1]);
    assert_eq!(same_remote(&list, &all, 3), [3, 4]);
    assert_eq!(same_remote(&list, &all, 5), [5]);
    assert!(same_remote(&list, &all, 9).is_empty());
    // A sorted or filtered list: the captures it shows, still in the order they came.
    assert_eq!(same_remote(&list, &[6, 2, 0], 2), [0, 2]);
    assert_eq!(same_remote(&list, &[1, 2], 2), [2]);
    // Unknown signals heard through another demodulator are another remote.
    list[4].received_rf = Some(crate::capture::RfModulation::FM);
    assert_eq!(same_remote(&list, &all, 3), [3]);
}

/// `:compare` takes its captures in the order named, each once; a `/` between them makes groups.
#[test]
fn a_compare_spec_names_captures_and_groups() {
    let existing = [1, 2, 3, 4, 5];
    let plain = |ids: &[u32]| ids.iter().map(|id| (*id, None)).collect::<Vec<_>>();
    let grouped = |ids: &[(u32, &str)]| ids.iter().map(|(id, g)| (*id, Some(g.to_string()))).collect::<Vec<_>>();
    assert_eq!(parse_spec("1-3", &existing), Ok((plain(&[1, 2, 3]), None)));
    assert_eq!(parse_spec("3 1, 3 9", &existing), Ok((plain(&[3, 1]), Some("#09".to_string()))));
    assert_eq!(parse_spec("3,1 / 2 / 1 4", &existing), Ok((grouped(&[(3, "A"), (1, "A"), (2, "B"), (4, "C")]), None)));
    assert_eq!(parse_spec("1-2/4-8/9 5", &existing).unwrap().1, Some("#06-#08, #09".to_string()));
    assert_eq!(parse_spec("8 9", &existing), Err("Captures #08, #09 not found".to_string()));
    assert_eq!(parse_spec("1 / 9", &existing), Err("Capture #09 not found".to_string()));
    assert!(parse_spec("1-3 /", &existing).unwrap_err().starts_with("No capture ID given"));
    assert_eq!([group_name(0), group_name(25), group_name(26)], ["A", "Z", "G27"]);
}

/// A side with one frame is dropped when the other has frames to compare; the decoded data comes
/// first when both do.
#[test]
fn the_sides_are_those_with_frames_to_compare() {
    let mut protocols = ProtocolRegistry::new();
    let list = [decoded(1, "Acme V3", 7, 1, 0xA1_0010, 24), decoded(2, "Acme V3", 7, 2, 0xA2_0011, 24)];
    let sides = sides_of(&mut protocols, &picks(&list), Grouping::Button, &[]);
    assert_eq!(sides.iter().map(|s| (s.label, s.comparison.frames.len())).collect::<Vec<_>>(), [("decoded data", 2)]);
    // One capture alone: its one frame, for the command to say there is nothing to compare.
    let sides = sides_of(&mut protocols, &picks(&list[..1]), Grouping::Button, &[]);
    assert_eq!(sides.iter().map(|s| s.comparison.frames.len()).collect::<Vec<_>>(), [1]);
    assert!(sides_of(&mut protocols, &picks(&[unknown(1, 433_920_000)]), Grouping::Button, &[]).is_empty());
}

/// Tab shows the other side under its own title, and the header names the key while there is
/// another side.
#[test]
fn tab_shows_the_other_side() {
    let side = |what: &str, list: &[Capture]| {
        let frames = list.iter().map(|c| Frame { name: format!("#{:02}", c.id), group: button_of(c), bits: decoded_bits(c) });
        compare::compare(frames.collect(), what.to_string(), Grouping::Button, Vec::new())
    };
    let list = [decoded(1, "Acme V3", 7, 1, 0xA1_0010, 24), decoded(2, "Acme V3", 7, 2, 0xA2_0011, 24)];
    let text = |view: &CompareView| -> Vec<String> {
        view.lines(76).iter().map(|l| l.iter().map(|s| s.text.as_str()).collect()).collect()
    };
    let header = vec!["Acme V3  serial 0000007".to_string(), "Captures #01, #02".to_string()];
    let both = vec![("decoded data", side("the decoded data of Acme V3", &list)), ("bits as sent", side("the bits as sent", &list[..1]))];
    let mut view = CompareView::new("2 captures of Acme V3".to_string(), header.clone(), both);
    assert_eq!(view.title(), "Compare (decoded data): 2 captures of Acme V3");
    assert_eq!(text(&view)[0], " Acme V3  serial 0000007  Captures #01, #02  Tab: the bits as sent");
    assert!(text(&view).iter().any(|l| l.contains("2 frames of 24 bits: the decoded data of Acme V3")), "{:#?}", text(&view));

    assert!(view.show_other_side());
    assert_eq!(view.title(), "Compare (bits as sent): 2 captures of Acme V3");
    assert_eq!(text(&view)[0], " Acme V3  serial 0000007  Captures #01, #02  Tab: the decoded data");
    assert!(text(&view).iter().any(|l| l.contains("1 frame of 24 bits: the bits as sent")), "{:#?}", text(&view));
    assert!(view.show_other_side());
    assert_eq!(view.title(), "Compare (decoded data): 2 captures of Acme V3");

    let mut alone = CompareView::new("2 captures of Acme V3".to_string(), header, vec![("decoded data", side("the data", &list))]);
    assert!(!alone.show_other_side());
    assert_eq!(text(&alone)[0], " Acme V3  serial 0000007  Captures #01, #02");
}

/// The captures of a recording in IMPORTS, decoded as `:load` decodes them. None when the file
/// is not there.
fn imported(file: &str) -> Option<(ProtocolRegistry, Vec<Capture>)> {
    let path = Path::new("IMPORTS").join(file);
    if !path.exists() {
        eprintln!("Skip: {path:?} not found (run from crate root)");
        return None;
    }
    crate::protocols::keys::load_keystore_from_embedded();
    let mut protocols = ProtocolRegistry::new();
    let (mut captures, mut next_id) = (Vec::new(), 1);
    let mut importer = crate::app::files::Importer {
        protocols: &mut protocols,
        captures: &mut captures,
        next_id: &mut next_id,
        show_unknown: true,
        ignore_duplicates: false,
        unchecked: Vec::new(),
        import_dir: Path::new("IMPORTS"),
    };
    importer.import(&path);
    Some((protocols, captures))
}

/// The bits found to follow the button tell the frames apart exactly as the decoder's buttons do.
/// The comparison is told the buttons' names, never which bits hold them.
fn assert_the_button_bits_follow_the_decoder(side: &Side, captures: &[Capture]) {
    let c = &side.comparison;
    assert_eq!(c.frames.len(), captures.len(), "{}", side.label);
    let columns: Vec<usize> = (0..c.bits).filter(|i| matches!(c.classes[*i], Class::Group { .. })).collect();
    assert!(!columns.is_empty(), "{}: {:?}", side.label, c.classes);
    let code = |frame: &Frame| columns.iter().map(|i| frame.bits[*i]).collect::<Vec<_>>();
    for (a, of_a) in c.frames.iter().zip(captures) {
        for (b, of_b) in c.frames.iter().zip(captures) {
            assert_eq!(code(a) == code(b), of_a.button == of_b.button, "{}: {} and {}", side.label, a.name, b.name);
        }
    }
}

/// The one counter found counts from frame to frame as the decoder's counter does: the comparison
/// knows nothing of the protocol's layout.
fn assert_the_counter_follows_the_decoder(side: &Side, captures: &[Capture]) {
    let c = &side.comparison;
    assert_eq!(c.frames.len(), captures.len(), "{}", side.label);
    assert_eq!(c.counters.len(), 1, "{}: {:?}", side.label, c.counters);
    let counter = &c.counters[0];
    assert!(counter.sure && !counter.down, "{}: {counter:?}", side.label);
    let mask = (1u32 << counter.len) - 1;
    let decoded = |capture: &Capture| u32::from(capture.counter.expect("a decoded counter"));
    for (frame, capture) in c.frames.iter().zip(captures) {
        let counted = counter.value(frame).wrapping_sub(counter.value(&c.frames[0])) & mask;
        assert_eq!(counted, decoded(capture).wrapping_sub(decoded(&captures[0])) & mask, "{}: {}", side.label, frame.name);
    }
}

fn unexplained(c: &Comparison) -> usize {
    c.classes.iter().filter(|class| !matches!(class, Class::Fixed | Class::Counter(_) | Class::Group { .. })).count()
}

/// Kia V1, five presses of Lock and five of Unlock: in the decoded data and in the bits as sent,
/// the counter and the button are found where the decoder reads them, and all that is left beside
/// fixed bits is the 4-bit checksum.
#[test]
fn kia_v1_presses_show_their_counter_and_button() {
    let Some((mut protocols, captures)) = imported("KIA/KiaV1_5cl_5op.sub") else { return };
    assert_eq!(captures.len(), 10);
    let sides = sides_of(&mut protocols, &picks(&captures), Grouping::Button, &[]);
    assert_eq!(sides.iter().map(|s| (s.label, s.comparison.bits)).collect::<Vec<_>>(), [("decoded data", 57), ("bits as sent", 234)]);
    for side in &sides {
        assert_the_counter_follows_the_decoder(side, &captures);
        assert_the_button_bits_follow_the_decoder(side, &captures);
        assert_eq!(unexplained(&side.comparison), 4, "{}", side.label);
        assert!(side.comparison.notes.is_empty(), "{}: {:?}", side.label, side.comparison.notes);
    }
}

/// Kia V3 (KeeLoq), fifteen presses of two buttons: the 32 encrypted bits that come first show no
/// pattern and no counter, and the button's bits are found beside them. The frame is 68 bits long:
/// the decoded data holds its first 64.
#[test]
fn kia_v3_presses_show_their_button_beside_the_encrypted_part() {
    let _keystore = crate::protocols::keeloq_common::KEYSTORE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let Some((mut protocols, captures)) = imported("KIA/Kia3_5cl_5op_5tr.sub") else { return };
    assert_eq!(captures.len(), 15);
    let sides = sides_of(&mut protocols, &picks(&captures), Grouping::Button, &[]);
    assert_eq!(sides.iter().map(|s| (s.label, s.comparison.bits)).collect::<Vec<_>>(), [("decoded data", 64), ("bits as sent", 68)]);
    for side in &sides {
        let c = &side.comparison;
        assert!(c.counters.is_empty(), "{}: {:?}", side.label, c.counters);
        assert!(c.classes[..32].iter().all(|class| *class == Class::Varies), "{}: {:?}", side.label, &c.classes[..32]);
        assert_the_button_bits_follow_the_decoder(side, &captures);
    }
    assert!(sides[0].comparison.notes[0].starts_with("A frame is 68 bits long"), "{:?}", sides[0].comparison.notes);
    assert!(sides[1].comparison.notes.is_empty(), "{:?}", sides[1].comparison.notes);
}

/// Suzuki V0, fifteen presses of three buttons. Its rows come 375 to 381 bits long, the preamble
/// caught at another bit each time: lined up, they show the counter and the button as the decoded
/// data does.
#[test]
fn suzuki_v0_rows_of_ragged_lengths_show_their_counter_and_button() {
    let Some((mut protocols, captures)) = imported("SUZUKI/Suzuki_v0_kd.sub") else { return };
    assert_eq!(captures.len(), 15);
    let sides = sides_of(&mut protocols, &picks(&captures), Grouping::Button, &[]);
    assert_eq!(sides.iter().map(|s| s.label).collect::<Vec<_>>(), ["decoded data", "bits as sent"]);
    for side in &sides {
        assert_the_counter_follows_the_decoder(side, &captures);
        assert_the_button_bits_follow_the_decoder(side, &captures);
    }
    let on_air = &sides[1].comparison;
    assert!(on_air.notes.iter().any(|n| n.contains("lined up by what they hold")), "{:?}", on_air.notes);
    // Every frame's last 64 bits but its tail hold what the decoder read: the serial is in them.
    assert!(on_air.bits >= 370, "{}", on_air.bits);
}
