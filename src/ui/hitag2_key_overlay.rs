//! The overlay "Recover key (HiTag2)" opens when it finds a key (app/hitag2_bf.rs): the key in the
//! standard byte order and, for a Fiat remote, in the Fiat cipher's, with what takes each. The
//! status line names the key once and in one order, and a Fiat key read out of `hitag2_keys.txt`
//! is in the other: typed into qDoctor or doctor-firmware that way, it does not work.

use ratatui::{
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Clear, Paragraph},
    Frame,
};

use crate::app::{App, Hitag2KeyReport};
use crate::protocols::hitag2_hell::Hitag2Remote;
use crate::protocols::hitag2_keys;

use super::text;
use super::text_overlay::{overlay_block, overlay_rect};
use super::theme;

/// The overlay's width at most: its notes are sentences, hard to read across a wide terminal.
const MAX_WIDTH: u16 = 66;

/// Column where the key starts, after " Standard order   ".
const KEY_COL: usize = 18;

/// The remote's protocol, as the list names it.
fn protocol_name(remote: Hitag2Remote) -> &'static str {
    match remote {
        Hitag2Remote::FiatV1 => "Fiat V1",
        Hitag2Remote::RenaultV1 => "Renault V1",
        Hitag2Remote::FiatV2 => "Fiat V2",
        Hitag2Remote::Hitag2Rke => "Hitag2",
    }
}

/// What takes the key in the standard order, and (a Fiat remote) what takes it in the Fiat
/// cipher's: `docs/hitag2_hell.md`, "Which order is where". Fiat V2 and Hitag2 RKE have no
/// `Hitag2 Key` line in any reference's key files.
fn notes(remote: Hitag2Remote) -> (&'static str, Option<&'static str>) {
    match remote {
        Hitag2Remote::FiatV1 => (
            "As qDoctor shows it. doctor-firmware's key prompt and the Hitag2 Key line of a key file take it so.",
            Some("As the remote's line in hitag2_keys.txt has it, and as ProtoPirate and Flipper-ARF read the Hitag2 Key line."),
        ),
        Hitag2Remote::RenaultV1 => (
            "As qDoctor shows it. doctor-firmware's key prompt, the Hitag2 Key line of a key file and hitag2_keys.txt take it so.",
            None,
        ),
        Hitag2Remote::FiatV2 => (
            "The order qDoctor and doctor-firmware write a HITAG2 key in.",
            Some("As the remote's line in hitag2_keys.txt has it."),
        ),
        Hitag2Remote::Hitag2Rke => (
            "The order qDoctor and doctor-firmware write a HITAG2 key in, and the remote's line in hitag2_keys.txt.",
            None,
        ),
    }
}

/// What the key changes for the remote's captures.
fn effect(report: &Hitag2KeyReport) -> String {
    let n = report.captures;
    let captures = format!("{n} capture{}", crate::plural(n));
    if report.remote == Hitag2Remote::Hitag2Rke {
        format!("{captures} now carry the key (decode only: KAT does not transmit Hitag2 RKE).")
    } else {
        format!("{captures} can now send fresh codes.")
    }
}

/// The overlay's lines for a text `width` columns wide (each starts with a space).
fn lines(report: &Hitag2KeyReport, width: usize) -> Vec<Line<'static>> {
    let bold = Style::default().add_modifier(Modifier::BOLD);
    let key_style = Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD);
    let mut out = Vec::new();
    let mut push = |line: Line<'static>, indent: usize| out.extend(text::wrap_line(line, width, indent));
    let key_line = |name: &str, key: &[u8; 6]| {
        Line::from(vec![
            Span::styled(format!(" {name:<w$}", w = KEY_COL - 1), bold),
            Span::styled(hitag2_keys::hex(key, " "), key_style),
        ])
    };
    let note = |text: &str| Line::styled(format!("   {text}"), theme::TEXT);

    push(
        Line::from(vec![
            Span::styled(" Remote ", theme::TEXT),
            Span::styled(format!("{:08X}", report.uid), Style::default().fg(Color::Cyan)),
            Span::styled(format!(" ({}), key found in {}", protocol_name(report.remote), report.took), theme::TEXT),
        ]),
        1,
    );
    push(Line::default(), 0);

    let (standard_note, fiat_note) = notes(report.remote);
    push(key_line("Standard order", &report.key.standard), KEY_COL);
    push(note(standard_note), 3);
    if let (Some(fiat), Some(fiat_note)) = (report.key.fiat_order(), fiat_note) {
        push(Line::default(), 0);
        push(key_line("Fiat order", &fiat), KEY_COL);
        push(note(fiat_note), 3);
    }
    push(Line::default(), 0);

    // "saved in <file>" / "not saved (<why>), usable until KAT exits", as the status line has it.
    let mut saved = report.saved.clone();
    if let Some(first) = saved.get(..1).map(str::to_ascii_uppercase) {
        saved.replace_range(..1, &first);
    }
    push(Line::styled(format!(" {saved}"), theme::TEXT), 1);
    push(Line::styled(format!(" {} The key stays in their detail panel (HiTag2).", effect(report)), theme::TEXT), 1);
    out
}

/// Draw the overlay, no larger than its text; sets the scroll limit and the page step the keys
/// use (`App::overlay_key`), as the help does.
pub fn render(frame: &mut Frame, app: &App) {
    let Some(report) = app.hitag2_key_report() else {
        return;
    };
    let full = overlay_rect(frame.area());
    let width = full.width.min(MAX_WIDTH);
    // The lines start with a space; one column stays free at the right border too.
    let lines = lines(report, width.saturating_sub(3) as usize);
    let height = (lines.len().min(usize::from(u16::MAX) - 2) as u16 + 2).min(full.height);
    let area = ratatui::layout::Rect::new(
        frame.area().width.saturating_sub(width) / 2,
        frame.area().height.saturating_sub(height) / 2,
        width,
        height,
    );
    frame.render_widget(Clear, area);

    let rows = height.saturating_sub(2) as usize;
    let scroll_max = lines.len().saturating_sub(rows);
    let scroll = app.overlay_scroll.min(scroll_max);
    app.overlay_page_rows.set(rows);
    app.overlay_scroll_max.set(scroll_max);

    let block = overlay_block("HiTag2 key recovered", width, scroll, rows, lines.len());
    frame.render_widget(Paragraph::new(lines).block(block).scroll((scroll as u16, 0)), area);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::hitag2_keys::{KeyOrder, ShownKey};

    fn report(remote: Hitag2Remote, order: KeyOrder) -> Hitag2KeyReport {
        Hitag2KeyReport {
            uid: 0x2DF9_F620,
            remote,
            key: ShownKey { standard: [0xB3, 0xC1, 0x01, 0x2C, 0xF7, 0x7F], order, epoch: 0 },
            took: "69 s".to_string(),
            saved: "saved in /home/user/.config/KAT/hitag2_keys.txt".to_string(),
            captures: 3,
        }
    }

    fn text(report: &Hitag2KeyReport, width: usize) -> Vec<String> {
        lines(report, width).iter().map(|l| l.spans.iter().map(|s| s.content.as_ref()).collect()).collect()
    }

    /// A Fiat V1 key in both orders, each with what takes it: the Fiat order is the standard one
    /// with its first two bytes moved to the end.
    #[test]
    fn a_fiat_key_is_shown_in_both_orders() {
        let got = text(&report(Hitag2Remote::FiatV1, KeyOrder::FiatBcm), 63);
        assert_eq!(
            got,
            vec![
                " Remote 2DF9F620 (Fiat V1), key found in 69 s",
                "",
                " Standard order   B3 C1 01 2C F7 7F",
                "   As qDoctor shows it. doctor-firmware's key prompt and the",
                "   Hitag2 Key line of a key file take it so.",
                "",
                " Fiat order       01 2C F7 7F B3 C1",
                "   As the remote's line in hitag2_keys.txt has it, and as",
                "   ProtoPirate and Flipper-ARF read the Hitag2 Key line.",
                "",
                " Saved in /home/user/.config/KAT/hitag2_keys.txt",
                " 3 captures can now send fresh codes. The key stays in their",
                " detail panel (HiTag2).",
            ]
        );
    }

    /// Renault V1 and Hitag2 RKE have one order everywhere; Hitag2 RKE is not transmitted.
    #[test]
    fn a_standard_order_remote_has_one_key_line() {
        for remote in [Hitag2Remote::RenaultV1, Hitag2Remote::Hitag2Rke] {
            let got = text(&report(remote, KeyOrder::Standard), 63).join("\n");
            assert!(got.contains(" Standard order   B3 C1 01 2C F7 7F\n"), "{got}");
            assert!(!got.contains("Fiat order"), "{got}");
        }
        let rke = text(&report(Hitag2Remote::Hitag2Rke, KeyOrder::Standard), 63).join("\n");
        assert!(rke.contains("3 captures now carry the key (decode only"), "{rke}");
        let mut one = report(Hitag2Remote::RenaultV1, KeyOrder::Standard);
        one.captures = 1;
        one.saved = "not saved (read-only file system), usable until KAT exits".to_string();
        let got = text(&one, 63).join("\n");
        assert!(got.contains(" 1 capture can now send fresh codes."), "{got}");
        assert!(got.contains(" Not saved (read-only file system), usable until KAT"), "{got}");
    }

    /// No line is wider than the text width, down to the narrowest overlay.
    #[test]
    fn the_lines_fit_the_width() {
        for width in [30usize, 44, 55, 63] {
            for (remote, order) in [(Hitag2Remote::FiatV1, KeyOrder::FiatBcm), (Hitag2Remote::RenaultV1, KeyOrder::Standard)] {
                for line in text(&report(remote, order), width) {
                    assert!(text::width(&line) <= width, "{width}: {line}");
                }
            }
        }
    }
}
