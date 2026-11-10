//! The command line's history: Up and Down bring back the commands run before.

/// Commands kept: the oldest goes when one more is run.
const KEPT: usize = 50;

/// The commands run from the command line this session, and where Up / Down are in them.
#[derive(Debug, Default)]
pub struct CommandHistory {
    /// Oldest first.
    entries: Vec<String>,
    /// The entry the command line shows while Up / Down browse.
    at: Option<usize>,
    /// What was typed when the browsing started: Down past the newest entry brings it back.
    draft: String,
}

impl CommandHistory {
    /// Remember a command that was run (not an empty line, and not the last one again).
    pub fn push(&mut self, command: &str) {
        let command = command.trim();
        self.at = None;
        self.draft.clear();
        if command.is_empty() || self.entries.last().is_some_and(|last| last == command) {
            return;
        }
        if self.entries.len() == KEPT {
            self.entries.remove(0);
        }
        self.entries.push(command.to_string());
    }

    /// Up: the command before the one shown (`line` is what the command line holds now). None at
    /// the oldest, and when nothing was run yet.
    pub fn older(&mut self, line: &str) -> Option<&str> {
        let at = match self.at {
            Some(0) => return None,
            Some(at) => at - 1,
            None => {
                self.draft = line.to_string();
                self.entries.len().checked_sub(1)?
            }
        };
        self.at = Some(at);
        Some(&self.entries[at])
    }

    /// Down: the command after the one shown, and after the newest what was typed before Up.
    /// None when Up was not pressed.
    pub fn newer(&mut self) -> Option<&str> {
        let at = self.at? + 1;
        if at < self.entries.len() {
            self.at = Some(at);
            Some(&self.entries[at])
        } else {
            self.at = None;
            Some(&self.draft)
        }
    }

    /// The command line was opened or closed, or its text edited: Up starts from the newest again.
    pub fn leave(&mut self) {
        self.at = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn up_and_down_walk_the_commands_and_come_back_to_the_draft() {
        let mut h = CommandHistory::default();
        assert_eq!(h.older("fr"), None, "nothing run yet");
        assert_eq!(h.newer(), None);
        for c in ["freq 433.92", " lock 3 ", "", "lock 3", "analyze 4"] {
            h.push(c);
        }
        assert_eq!(h.older("fr"), Some("analyze 4"));
        assert_eq!(h.older("analyze 4"), Some("lock 3"));
        assert_eq!(h.older("lock 3"), Some("freq 433.92"));
        assert_eq!(h.older("freq 433.92"), None, "the oldest stays");
        assert_eq!(h.newer(), Some("lock 3"));
        assert_eq!(h.newer(), Some("analyze 4"));
        assert_eq!(h.newer(), Some("fr"), "what was being typed");
        assert_eq!(h.newer(), None);
    }

    #[test]
    fn running_or_editing_starts_from_the_newest_again() {
        let mut h = CommandHistory::default();
        h.push("hop on");
        h.push("hop off");
        assert_eq!(h.older(""), Some("hop off"));
        assert_eq!(h.older(""), Some("hop on"));
        // Edited: the next Up takes the edited text as the draft.
        h.leave();
        assert_eq!(h.older("hop on 315"), Some("hop off"));
        assert_eq!(h.newer(), Some("hop on 315"));
        // Run again from the history: kept once more at the end, unless it is the last already.
        h.push("hop on");
        h.push("hop on");
        assert_eq!(h.older(""), Some("hop on"));
        assert_eq!(h.older(""), Some("hop off"));
        assert_eq!(h.older(""), Some("hop on"));
        assert_eq!(h.older(""), None);
    }

    #[test]
    fn only_the_latest_commands_are_kept() {
        let mut h = CommandHistory::default();
        for i in 0..KEPT + 5 {
            h.push(&format!("freq {i}"));
        }
        let mut seen = 0;
        let mut oldest = String::new();
        while let Some(c) = h.older("") {
            oldest = c.to_string();
            seen += 1;
        }
        assert_eq!((seen, oldest.as_str()), (KEPT, "freq 5"));
    }
}
