//! Native buttons of the gate / garage remotes, for the signal menu's "Send button…" and the button
//! column: the codes each protocol's encoder takes as `encode(decoded, button)` (its module's
//! **Encoder** doc) and their labels.
//!
//! A protocol's list is the remote's buttons the references name or switch between (labels from
//! `ProtocolInfo::buttons`, the references' custom-button D-pad sets); without such a set, every
//! code the encoder accepts (a 2-, 3- or 4-bit field: shown as "Btn 0xN"). Protocols whose encoder
//! ignores the button (fixed codes that re-send the capture: CAME, Nice FLO, Linear, Marantec,
//! Power Smart, Telcoma EDGE, …) and decode-only ones list nothing.

use std::sync::OnceLock;

use super::DecodedSignal;

/// The remote's buttons for `protocol` (a display name) when the encoder can switch to them, as
/// native codes in the order shown; empty when it cannot (see the module doc).
pub fn tx_buttons(protocol: &str, decoded: &DecodedSignal) -> Vec<u8> {
    let Some(info) = super::protocol_info(protocol) else { return Vec::new() };
    let captured = decoded.button;
    // Encoders that switch only between the buttons of a remote whose captured button is one of
    // them (the references' custom-button switch).
    let switch = |set: &[u8]| if captured.is_some_and(|b| set.contains(&b)) { set.to_vec() } else { Vec::new() };
    let range = |r: std::ops::RangeInclusive<u8>| r.collect::<Vec<u8>>();
    match info.decoder {
        // KeeLoq (barrier and alarm manufacturers; car keyfobs keep their Lock/Unlock menu): the
        // 4-bit function code, 1..F (BFT/Aprimatic/Dea_Mio: 0xF = programming mode).
        "KeeLoq" if super::is_keeloq_non_car(protocol) => range(0x1..=0xF),
        // Beninca ARC: any 8-bit code; the C's custom buttons cycle 0x02 / 0x04 / 0x00.
        "Beninca ARC" => vec![0x02, 0x04, 0x00],
        // KingGates Stylo4k: 4 bits; the C's custom buttons cycle 0xE / 0xD / 0xB / 0x7.
        "KingGates Stylo4k" => vec![0xE, 0xD, 0xB, 0x7],
        // Jarolift: 1 Learn, 2 Down, 4 Stop, 8 Up.
        "Jarolift" => vec![0x1, 0x2, 0x4, 0x8],
        // Faac SLH: any 4-bit code; 0x10 = the C's UP button (programming-mode frame).
        "Faac SLH" => range(0x0..=0xF).into_iter().chain([super::faac_slh::PROG_BUTTON]).collect(),
        // Security+ 2.0: any 8-bit code; the references' D-pad offers these.
        "Security+ 2.0" => vec![0x68, 0x80, 0x81, 0xE2, 0x78],
        // Somfy: the 4-bit ctrl code (labelled).
        "Somfy Telis" | "Somfy Keytis" => range(0x0..=0xF),
        // Security+ 1.0: the lowest base-3 digit of the fixed code (middle, left, right).
        "Security+ 1.0" => vec![0, 1, 2],
        "Alutech AT-4N" => vec![0xFF, 0x11, 0x22, 0x33, 0x44],
        "Nice FloR-S" => vec![0x1, 0x2, 0x4, 0x8, 0x3],
        "Ditec GOL4" => vec![0x1, 0x2, 0x4, 0x8, 0x0],
        "CAME Atomo" => range(1..=6),
        "Phoenix_V2" => switch(&[0x1, 0x2, 0x4, 0x8, 0x3]),
        "Roger" => switch(&[0x1, 0x2, 0x4, 0x8]),
        "Hay21" => switch(&[0x5A, 0xC3, 0x88]),
        "Prastel" => switch(&[1, 2, 3, 4]),
        // GangQi / Hollarm (alarm remotes): the 4-bit code (the references' button editor takes any
        // value); listed: the codes the C names — the remote's four buttons (its D-pad set) first,
        // then the settings-mode codes.
        "GangQi" => super::gangqi::DPAD.iter().copied().chain([0x5, 0x1, 0x2, 0x4, 0x6]).collect(),
        "Hollarm" => super::hollarm::DPAD.iter().copied().chain([0x9, 0xA, 0xB, 0xD]).collect(),
        // Dooya: Flipper-ARF's D-pad codes (Up_Long, Down_Long, Stop, P2).
        "Dooya" => vec![0x11, 0x33, 0x55, 0xCC],
        // Holtek: position << 4 | ON (0xE) / OFF (0xB), as the C displays them.
        "Holtek" => vec![0x1E, 0x1B, 0x2E, 0x2B, 0x3E, 0x3B, 0x4E, 0x4B],
        // Princeton: the remote's type — one of the 8-bit codes, else a 4-bit code.
        "Princeton" => match captured {
            Some(b) if [0x30, 0xC0, 0xF3, 0xFC].contains(&b) => vec![0x30, 0xC0, 0xF3, 0xFC],
            _ => range(0x0..=0xF),
        },
        "Ansonic" | "Clemsa" | "Doitrand" => range(0x0..=0x3),
        "MegaCode" => range(0x0..=0x7),
        "Hormann HSM" | "Nero Radio" | "Marantec24" | "CAME TWEE" | "GateTX" | "Holtek_HT12X" => range(0x0..=0xF),
        _ => Vec::new(),
    }
}

/// "Btn 0xN" (the label of a button the protocol does not name).
pub fn hex_label(code: u8) -> &'static str {
    static LABELS: OnceLock<Vec<String>> = OnceLock::new();
    &LABELS.get_or_init(|| (0..=255u16).map(|c| format!("Btn 0x{c:X}")).collect())[usize::from(code)]
}

/// The label of a native button code: the protocol's own name for it, else "Btn 0xN".
pub fn button_label(protocol: &str, code: u8) -> &'static str {
    super::native_button_name(protocol, code).unwrap_or_else(|| hex_label(code))
}

/// Whether `protocol` is a gate / garage remote: a `Category::Gate` protocol, a KeeLoq barrier
/// manufacturer, or a KeeLoq remote whose key KAT recovered (`BF_<serial>`, make unknown). Their
/// buttons are native codes, never KAT's car commands.
pub fn is_gate(protocol: &str) -> bool {
    super::keeloq_barriers::is_keeloq_barrier(protocol)
        || super::keeloq_barriers::is_keeloq_recovered(protocol)
        || super::protocol_info(protocol).is_some_and(|i| i.category == super::info::Category::Gate)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::add_manually::{fill, generate, Rng, TYPES};
    use crate::protocols::{FlipperKey, ProtocolRegistry};

    #[test]
    fn labels() {
        assert_eq!(hex_label(0x2), "Btn 0x2");
        assert_eq!(hex_label(0x68), "Btn 0x68");
        assert_eq!(button_label("Jarolift", 0x8), "Up");
        assert_eq!(button_label("Beninca ARC", 0x4), "Btn 0x4");
        assert_eq!(button_label("Princeton", 0xA), "Btn A");
        assert!(is_gate("Roger") && is_gate("KeeLoq (DoorHan)") && is_gate("KeeLoq (FAAC_RC,XT)"));
        assert!(is_gate("KeeLoq (BF_0B5A7E3)") && !is_gate("KeeLoq (Unknown)"));
        assert!(!is_gate("Kia V0") && !is_gate("KeeLoq (Star Line)") && !is_gate("Unknown"));
        // The button column: native names, "Btn 0xN" for gate remotes without them, KAT's car
        // commands for car protocols (unchanged).
        let name = |protocol: &str, code: u8| {
            let mut c = crate::capture::Capture::from_pairs(1, 433_920_000, Vec::new());
            c.protocol = Some(protocol.to_string());
            c.button = Some(code);
            c.button_name().to_string()
        };
        assert_eq!(name("Roger", 0x2), "Btn 0x2");
        assert_eq!(name("Security+ 2.0", 0x68), "Btn 0x68");
        assert_eq!(name("KeeLoq (Beninca)", 0x1), "Btn 0x1");
        assert_eq!(name("KeeLoq (DoorHan)", 0x2), "B1");
        assert_eq!(name("Somfy Telis", 0x4), "Down");
        assert_eq!(name("Kia V0", 0x2), "Unlock");
        assert_eq!(name("KeeLoq (Star Line)", 0x1), "Lock");
        assert_eq!(name("Unknown", 0x8), "Panic");
    }

    /// Every listed button of every "Add Manually" remote KAT creates (and of a few captured ones)
    /// is sent: the transmission decodes with that button where the decoder reports buttons.
    #[test]
    fn send_button_sends_each_listed_button() {
        crate::protocols::keys::load_keystore_from_embedded();
        let mut reg = ProtocolRegistry::new();
        let mut failures = Vec::new();
        let mut keys: Vec<FlipperKey> = Vec::new();
        for ty in TYPES {
            let mut rng = Rng::seeded(11);
            if let Ok(k) = generate(&fill(ty, &mut rng), &mut rng) {
                keys.push(k);
            }
        }
        for key in keys {
            let Some((name, d)) = reg.decode_key(&key) else { continue };
            let buttons = tx_buttons(&name, &d);
            if buttons.is_empty() {
                // Only fixed codes whose encoder ignores the button list nothing.
                if !matches!(key.protocol.as_str(), "CAME" | "Nice FLO" | "Linear" | "BETT" | "Revers_RB2" | "Marantec" | "Telcoma/Cardin EDGE" | "Roger") {
                    failures.push(format!("{name}: no buttons"));
                }
                continue;
            }
            for code in buttons {
                let Some(upload) = reg.get(&name).unwrap().encode(&d, code) else {
                    failures.push(format!("{name}: button {code:#x} does not encode"));
                    continue;
                };
                // The upload once more after a gap, adjacent equal levels merged.
                let mut stream: Vec<crate::radio::LevelDuration> = Vec::new();
                let gap = [crate::radio::LevelDuration::new(false, 20_000)];
                for p in gap.iter().chain(&upload).chain(&gap).chain(&upload).chain(&gap) {
                    match stream.last_mut() {
                        Some(last) if last.level == p.level => last.duration_us += p.duration_us,
                        _ => stream.push(*p),
                    }
                }
                let decoder = reg.get(&name).unwrap().name();
                let hits = reg.decode_with(decoder, &stream);
                let ok = match key.protocol.as_str() {
                    // Fields need a table the test environment does not have / no button field.
                    _ if hits.iter().any(|x| x.button.is_none()) && hits.iter().all(|x| x.button.is_none()) => !hits.is_empty(),
                    // Faac SLH: 0x10 sends a programming frame (no serial/button on air).
                    "Faac SLH" if code == super::super::faac_slh::PROG_BUTTON => hits.iter().any(|x| x.serial.is_none()),
                    // Marantec24 / Revers RB2: see add_manually's round-trip test (reference
                    // round-trip failures); the frame's button is checked on the key instead.
                    "Marantec24" => true,
                    // CAME TWEE: the C's serial includes the button bits.
                    "CAME TWEE" => hits.iter().any(|x| x.button == Some(code)),
                    // Princeton: a 4-bit key whose low byte reads 0x30/0xC0/0x03/0x0C decodes as
                    // the 8-bit type (the C's own ambiguity); the key's button nibble is sent.
                    "Princeton" => hits.iter().any(|x| x.button == Some(code) || (x.data & 0xF) as u8 == code),
                    _ => hits.iter().any(|x| x.button == Some(code) && x.serial == d.serial),
                };
                if !ok {
                    let got: Vec<String> = hits.iter().map(|x| format!("sn={:X?} btn={:X?}", x.serial, x.button)).collect();
                    failures.push(format!("{name}: button {code:#x} decodes as {got:?}"));
                }
            }
        }
        assert!(failures.is_empty(), "{} failure(s):\n{}", failures.len(), failures.join("\n"));
    }
}
