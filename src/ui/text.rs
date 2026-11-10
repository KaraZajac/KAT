//! Fitting text into terminal cells: display width, cutting with "…", word wrap, and the window
//! of a list that keeps its selection in view.

use ratatui::{
    style::Style,
    text::{Line, Span},
};

/// Display width of `s` in terminal columns (wide characters count 2).
pub fn width(s: &str) -> usize {
    Span::raw(s).width()
}

fn char_width(c: char) -> usize {
    let mut buf = [0u8; 4];
    width(c.encode_utf8(&mut buf))
}

/// The longest prefix of `chars` that fits in `max` columns.
pub(super) fn take_width(chars: impl Iterator<Item = char>, max: usize) -> Vec<char> {
    let mut out = Vec::new();
    let mut used = 0;
    for c in chars {
        let w = char_width(c);
        if used + w > max {
            break;
        }
        used += w;
        out.push(c);
    }
    out
}

/// `s` in at most `max` columns, keeping its start: "abcdef" → "abc…".
pub fn truncate_end(s: &str, max: usize) -> String {
    if width(s) <= max {
        return s.to_string();
    }
    if max == 0 {
        return String::new();
    }
    let mut out: String = take_width(s.chars(), max - 1).into_iter().collect();
    out.push('…');
    out
}

/// `s` cut to `width` columns with "…" and padded with spaces to exactly that width (a table
/// column of text).
pub fn fit(s: &str, width: usize) -> String {
    let cut = truncate_end(s, width);
    let pad = width.saturating_sub(self::width(&cut));
    format!("{cut}{}", " ".repeat(pad))
}

/// `s` in at most `max` columns, keeping its end (the tail of a path or of typed text):
/// "abcdef" → "…def".
pub fn truncate_start(s: &str, max: usize) -> String {
    if width(s) <= max {
        return s.to_string();
    }
    if max == 0 {
        return String::new();
    }
    let tail = take_width(s.chars().rev(), max - 1);
    std::iter::once('…').chain(tail.into_iter().rev()).collect()
}

/// `s` in at most `max` columns, cut in the middle so both ends stay (file names, messages whose
/// end says what went wrong): "abcdefgh" → "abc…gh".
pub fn truncate_middle(s: &str, max: usize) -> String {
    if width(s) <= max {
        return s.to_string();
    }
    if max < 3 {
        return truncate_end(s, max);
    }
    let room = max - 1;
    let head = take_width(s.chars(), room - room / 2);
    let head_w: usize = head.iter().map(|&c| char_width(c)).sum();
    let tail = take_width(s.chars().rev(), room - head_w);
    head.into_iter().chain(std::iter::once('…')).chain(tail.into_iter().rev()).collect()
}

/// A styled line in at most `max` columns: spans past the edge are dropped and the one that
/// crosses it is cut, ending with "…".
pub fn truncate_line(line: Line<'_>, max: usize) -> Line<'static> {
    let style = line.style;
    let fits = line.width() <= max;
    let mut out: Vec<Span<'static>> = Vec::new();
    let mut used = 0;
    for span in line.spans {
        let w = span.width();
        // Keep a whole span while there is still room for the "…" a later cut adds.
        if fits || used + w < max {
            used += w;
            out.push(Span::styled(span.content.into_owned(), span.style));
            continue;
        }
        let room = (max - used).saturating_sub(1);
        let mut cut: String = take_width(span.content.chars(), room).into_iter().collect();
        if max > used {
            cut.push('…');
        }
        out.push(Span::styled(cut, span.style));
        break;
    }
    Line::from(out).style(style)
}

/// Width of the marker of a list item ("1. ", "- ", "* ", after any indentation): continuation
/// lines of the item are indented by it. 0 for other lines.
fn hanging_indent(line: &str) -> usize {
    let lead = line.chars().take_while(|c| *c == ' ').count();
    let rest = &line[lead..];
    let digits = rest.chars().take_while(char::is_ascii_digit).count();
    let marker = if digits > 0 && rest[digits..].starts_with(". ") {
        digits + 2
    } else if rest.starts_with("- ") || rest.starts_with("* ") {
        2
    } else {
        0
    };
    if marker == 0 {
        0
    } else {
        lead + marker
    }
}

/// Word-wrap plain text to `max` columns: every source line is wrapped on its own, breaking at
/// spaces (a word longer than a line is split) and indenting continuation lines of list items
/// under their text. The result has exactly the lines the text takes on screen.
pub fn wrap(text: &str, max: usize) -> Vec<String> {
    let mut out = Vec::new();
    for source in text.lines() {
        let indent = hanging_indent(source);
        for line in wrap_line(Line::raw(source), max, indent) {
            out.push(line.spans.iter().map(|s| s.content.as_ref()).collect());
        }
    }
    out
}

/// A message in at most `max_lines` lines of `max` columns: word-wrapped (a line break in it
/// counts as a space), and when it needs more lines than that, the last one takes all that is
/// left, cut in the middle, so the message keeps its start and its end (often the reason, or the
/// file).
pub fn wrap_within(message: &str, max: usize, max_lines: usize) -> Vec<String> {
    let flat: String = message.trim().chars().map(|c| if c.is_whitespace() { ' ' } else { c }).collect();
    let mut lines: Vec<String> = wrap_line(Line::raw(flat.as_str()), max, 0)
        .iter()
        .map(|line| line.spans.iter().map(|s| s.content.as_ref()).collect())
        .collect();
    let max_lines = max_lines.max(1);
    if lines.len() > max_lines {
        lines.truncate(max_lines - 1);
        // Where the lines kept end in the message: a break drops its spaces, so count the rest.
        let kept: usize = lines.iter().map(|l| l.chars().filter(|c| *c != ' ').count()).sum();
        let rest = flat.char_indices().filter(|(_, c)| *c != ' ').nth(kept).map_or("", |(i, _)| &flat[i..]);
        lines.push(truncate_middle(rest, max));
    }
    lines
}

/// Word-wrap a styled line to `max` columns. Breaks at spaces (the spaces of a break are
/// dropped), splits words longer than a line, starts continuation lines with `indent` spaces
/// (in the style of the line's first span) and keeps each character's style (the line's own
/// style, as `Line::styled` sets it, is carried into every span).
pub fn wrap_line(line: Line<'_>, max: usize, indent: usize) -> Vec<Line<'static>> {
    let cells: Vec<(char, Style)> = line
        .spans
        .iter()
        .flat_map(|s| {
            let style = line.style.patch(s.style);
            s.content.chars().map(move |c| (c, style))
        })
        .collect();
    if max == 0 || line.width() <= max {
        return vec![styled_line(&cells)];
    }
    let indent = indent.min(max / 2);
    let indent_style = cells.first().map_or_else(Style::default, |c| c.1);
    let mut lines: Vec<Vec<(char, Style)>> = Vec::new();
    let mut cur: Vec<(char, Style)> = Vec::new();
    let mut cur_w = 0;
    // True once the current line holds a word (before that, a word never moves to a new line).
    let mut has_word = false;
    let mut i = 0;
    while i < cells.len() {
        let spaces_start = i;
        while i < cells.len() && cells[i].0 == ' ' {
            i += 1;
        }
        let word_start = i;
        while i < cells.len() && cells[i].0 != ' ' {
            i += 1;
        }
        let (spaces, word) = (&cells[spaces_start..word_start], &cells[word_start..i]);
        if word.is_empty() {
            break; // trailing spaces
        }
        let word_w: usize = word.iter().map(|c| char_width(c.0)).sum();
        // A word too long for any line (a URL) is split anyway: it starts on this line when
        // there is room for some of it, rather than leaving a short line behind.
        let breaks = if word_w <= max - indent {
            cur_w + spaces.len() + word_w > max
        } else {
            cur_w + spaces.len() >= max
        };
        if has_word && breaks {
            lines.push(std::mem::take(&mut cur));
            cur.extend(std::iter::repeat_n((' ', indent_style), indent));
            cur_w = indent;
        } else {
            cur.extend_from_slice(spaces);
            cur_w += spaces.len();
        }
        for &(c, style) in word {
            let w = char_width(c);
            if cur_w + w > max && cur_w > indent {
                lines.push(std::mem::take(&mut cur));
                cur.extend(std::iter::repeat_n((' ', indent_style), indent));
                cur_w = indent;
            }
            cur.push((c, style));
            cur_w += w;
        }
        has_word = true;
    }
    lines.push(cur);
    lines.iter().map(|l| styled_line(l)).collect()
}

/// Characters back into spans, one per run of the same style.
fn styled_line(cells: &[(char, Style)]) -> Line<'static> {
    let mut spans: Vec<Span<'static>> = Vec::new();
    let mut text = String::new();
    let mut style = cells.first().map_or_else(Style::default, |c| c.1);
    for &(c, s) in cells {
        if s != style && !text.is_empty() {
            spans.push(Span::styled(std::mem::take(&mut text), style));
        }
        style = s;
        text.push(c);
    }
    if !text.is_empty() {
        spans.push(Span::styled(text, style));
    }
    Line::from(spans)
}

/// First item to draw of a `len`-item list that has room for `visible` items, so `selected` is in
/// view: `start` (the list's own scroll position) when it already is, else scrolled just enough.
pub fn keep_in_view(start: usize, selected: usize, visible: usize, len: usize) -> usize {
    if visible == 0 || len <= visible {
        return 0;
    }
    let start = start.min(len - visible);
    if selected < start {
        selected
    } else if selected >= start + visible {
        (selected + 1 - visible).min(len - visible)
    } else {
        start
    }
}

/// First item to draw of a `len`-item list with room for `visible`, keeping `selected` near the
/// middle: for lists that keep no scroll position of their own between frames.
pub fn centered_start(selected: usize, visible: usize, len: usize) -> usize {
    if visible == 0 || len <= visible {
        return 0;
    }
    selected.saturating_sub(visible / 2).min(len - visible)
}

/// " ↑ 2 more · ↓ 5 more " for a list that shows items `start..start + visible` of `len`
/// (None when every item is shown).
pub fn more_hint(start: usize, visible: usize, len: usize) -> Option<String> {
    more_of(start.min(len), len.saturating_sub(start + visible))
}

/// The same for a list with `above` items before the ones shown and `below` after them.
pub fn more_of(above: usize, below: usize) -> Option<String> {
    match (above, below) {
        (0, 0) => None,
        (0, b) => Some(format!(" ↓ {b} more ")),
        (a, 0) => Some(format!(" ↑ {a} more ")),
        (a, b) => Some(format!(" ↑ {a} · ↓ {b} more ")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::style::{Color, Style};

    #[test]
    fn truncation_keeps_the_requested_end() {
        assert_eq!(truncate_end("Unlock+Trunk", 6), "Unloc…");
        assert_eq!(truncate_end("Lock", 6), "Lock");
        assert_eq!(truncate_end("abc", 0), "");
        assert_eq!(fit("Came Atomo", 6), "Came …");
        assert_eq!(fit("Nice", 6), "Nice  ");
        assert_eq!(truncate_start("/home/kara/caps/kia.sub", 10), "…s/kia.sub");
        assert_eq!(truncate_middle("Kia3_5cl_5op_5tr.sub", 11), "Kia3_…r.sub");
        assert_eq!(truncate_middle("abcdefgh", 6), "abc…gh");
        // Non-ASCII text is cut by characters (never inside one) and by display width.
        assert_eq!(truncate_start("/дом/Кира/записи", 8), "…/записи");
        assert_eq!(truncate_end("車のキー", 5), "車の…");
        for max in 0..12 {
            assert!(width(&truncate_middle("Привет, мир!", max)) <= max);
            assert!(width(&truncate_start("日本語のファイル", max)) <= max);
        }
    }

    #[test]
    fn line_truncation_cuts_the_last_span() {
        let line = Line::from(vec![Span::raw("Key: "), Span::styled("0x1234567890", Style::new().fg(Color::Yellow))]);
        let cut = truncate_line(line.clone(), 10);
        assert_eq!(cut.width(), 10);
        assert_eq!(cut.spans[1].content, "0x12…");
        assert_eq!(cut.spans[1].style.fg, Some(Color::Yellow));
        assert_eq!(truncate_line(line, 40).width(), 17);
    }

    #[test]
    fn wrap_breaks_at_spaces_and_indents_list_items() {
        let text = "Redistribution and use in source\n\n1. Redistributions of source code must retain";
        let lines = wrap(text, 20);
        assert_eq!(
            lines,
            vec![
                "Redistribution and",
                "use in source",
                "",
                "1. Redistributions",
                "   of source code",
                "   must retain",
            ]
        );
        // A word longer than the line is split; no line is ever wider than the limit.
        assert_eq!(wrap("https://nvd.nist.gov/vuln", 10), vec!["https://nv", "d.nist.gov", "/vuln"]);
        for max in 1..30 {
            assert!(wrap(text, max).iter().all(|l| width(l) <= max), "width {max}");
        }
    }

    #[test]
    fn a_message_wraps_within_its_lines() {
        // It fits: one line, as it is.
        assert_eq!(wrap_within("Loaded 3 captures", 40, 3), ["Loaded 3 captures"]);
        assert_eq!(wrap_within("", 40, 3), [""]);
        // Longer: wrapped at spaces; a line break in the message is a space.
        assert_eq!(
            wrap_within("Error: could not read\nthe file: no such file", 24, 3),
            ["Error: could not read", "the file: no such file"]
        );
        // More than the lines there are: the last one has the rest, with the message's end.
        let long = "one two three four five six seven eight nine ten";
        assert_eq!(wrap_within(long, 10, 4), ["one two", "three four", "five six", "seven… ten"]);
        assert_eq!(wrap_within(long, 10, 3), ["one two", "three four", "five … ten"]);
        assert_eq!(wrap_within(long, 10, 2), ["one two", "three… ten"]);
        // One line (a bar that cannot grow): the cut in the middle alone.
        assert_eq!(wrap_within(long, 10, 1), [truncate_middle(long, 10)]);
        assert_eq!(wrap_within(long, 10, 0), [truncate_middle(long, 10)]);
        // A word longer than a line is split, and no line is wider than the bar.
        let path = "Error: /home/user/a/very/long/folder/name/with_a_file_in_it.sub: No such file or directory";
        for (max, lines) in [(20, 1), (20, 2), (20, 3), (30, 3), (58, 3), (78, 3), (200, 3)] {
            let wrapped = wrap_within(path, max, lines);
            assert!(wrapped.len() <= lines && wrapped.iter().all(|l| width(l) <= max), "{max} {lines}: {wrapped:?}");
            assert!(wrapped[0].starts_with("Error:"), "{wrapped:?}");
            assert!(wrapped.last().unwrap().ends_with("directory"), "{wrapped:?}");
        }
        // The path is a word of 57 columns: it has a line of its own at 58.
        assert_eq!(
            wrap_within(path, 58, 3),
            ["Error:", "/home/user/a/very/long/folder/name/with_a_file_in_it.sub:", "No such file or directory"]
        );
    }

    #[test]
    fn styled_wrap_keeps_styles() {
        let label = Style::new().fg(Color::DarkGray);
        let line = Line::from(vec![Span::styled(" Description: ", label), Span::raw("same RF signal for each request")]);
        let lines = wrap_line(line, 24, 1);
        let text: Vec<String> = lines.iter().map(|l| l.spans.iter().map(|s| s.content.as_ref()).collect()).collect();
        assert_eq!(text, vec![" Description: same RF", " signal for each request"]);
        assert_eq!(lines[0].spans[0].style, label);
        assert_eq!(lines[0].spans[1].style, Style::default());
        // The style of a whole line (Line::styled) reaches every wrapped piece.
        let banner = Style::new().fg(Color::Green);
        let lines = wrap_line(Line::styled("Encryption is broken — complete emulation", banner), 20, 1);
        assert!(lines.len() > 1 && lines.iter().all(|l| l.spans.iter().all(|s| s.style == banner)));
        // A URL longer than a line fills the label's line, then continues under it.
        let url = Line::from(vec![Span::styled(" Source: ", label), Span::raw("https://nvd.nist.gov/vuln/detail/CVE-2022-27254")]);
        let text: Vec<String> = wrap_line(url, 30, 1).iter().map(|l| l.spans.iter().map(|s| s.content.as_ref()).collect()).collect();
        assert_eq!(text, vec![" Source: https://nvd.nist.gov/", " vuln/detail/CVE-2022-27254"]);
    }

    #[test]
    fn list_window_follows_the_selection() {
        assert_eq!(keep_in_view(0, 3, 10, 5), 0);
        assert_eq!(keep_in_view(0, 12, 10, 30), 3);
        assert_eq!(keep_in_view(8, 5, 10, 30), 5);
        assert_eq!(keep_in_view(8, 12, 10, 30), 8);
        // A scroll position past the end (entries changed) is pulled back.
        assert_eq!(keep_in_view(50, 29, 10, 30), 20);
        assert_eq!(centered_start(1, 4, 9), 0);
        assert_eq!(centered_start(5, 4, 9), 3);
        assert_eq!(centered_start(8, 4, 9), 5);
        assert_eq!(centered_start(8, 4, 3), 0);
        assert_eq!(more_hint(0, 10, 10), None);
        assert_eq!(more_hint(0, 10, 12).as_deref(), Some(" ↓ 2 more "));
        assert_eq!(more_hint(3, 10, 13).as_deref(), Some(" ↑ 3 more "));
        assert_eq!(more_hint(3, 10, 20).as_deref(), Some(" ↑ 3 · ↓ 7 more "));
    }
}
