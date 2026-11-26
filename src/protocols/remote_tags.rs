//! Per-remote user tags, keyed by UID — a small store for facts KAT cannot read from the air.
//!
//! The first (and so far only) tag is **Nissan**: a 2010-era Nissan Qashqai (and its
//! Renault-Nissan Alliance siblings) sends a frame byte-identical to Fiat V1 on air, so KAT
//! decodes it as "Fiat V1" and cannot tell the two apart from a capture (same timing, frame and
//! XOR). What differs is the HITAG2 key schedule — the Alliance remote uses
//! [`super::hitag2::renault_authenticator`] (key into state\[0\]\[1\], no epoch) where Fiat uses
//! [`super::hitag2::fiat_bcm_authenticator`] (key into state\[4\]\[5\], 18-bit epoch) — and the
//! button map (0x8 = Lock, 0x2 = Unlock, not Fiat's Unlock / Trunk). Below counter 1024 the two
//! schedules are the same cipher up to a two-byte key rotation (KAT's [`super::hitag2_keys`]
//! KeyOrder bridges it), so the tag changes the make, the protocol name KAT shows, the button
//! labels and which schedule signs fresh codes; it does **not** change the bytes sent while the
//! counter stays below 1024. The flat-counter divergence above 1024 is an open item (no capture
//! past the boundary to validate it).
//!
//! The user applies a tag from the signal menu ("Tag as Nissan (Renault-Alliance)"); it is
//! remembered for that UID for the rest of the session and written to `<config>/remote_tags.txt`,
//! so the next capture of the same remote is tagged at once. File format mirrors
//! `hitag2_keys.txt`: a header, then one `UID TAG` line per remote (UID = 8 hex digits, TAG = a
//! lowercase word); the first line of a UID counts.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::RwLock;

/// The store's file name, in KAT's config folder.
pub const FILE_NAME: &str = "remote_tags.txt";

/// Header written when the file is created.
const HEADER: &str = "# KAT per-remote tags\n# UID TAG\n";

/// A user tag for a remote. Extensible; today only [`RemoteTag::Nissan`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RemoteTag {
    /// Renault-Nissan Alliance remote (e.g. Nissan Qashqai) decoded as Fiat V1: Renault key
    /// schedule (no epoch), Nissan button map.
    Nissan,
}

impl RemoteTag {
    /// The word written in `remote_tags.txt`.
    pub fn as_str(self) -> &'static str {
        match self {
            RemoteTag::Nissan => "nissan",
        }
    }

    /// The tag of a stored word, if known.
    pub fn from_str(word: &str) -> Option<Self> {
        match word {
            "nissan" => Some(RemoteTag::Nissan),
            _ => None,
        }
    }

    /// The protocol name KAT shows for a tagged capture (its engine name stays "Fiat V1").
    pub fn display_protocol(self) -> &'static str {
        match self {
            RemoteTag::Nissan => "Nissan V1",
        }
    }

    /// The make KAT shows for a tagged capture.
    pub fn make(self) -> &'static str {
        match self {
            RemoteTag::Nissan => "Nissan",
        }
    }

    /// The tag's own button label for a Fiat-V1 nibble, when it differs from Fiat's. `None` falls
    /// back to the generic / Fiat label. Only the two nibbles seen on a real Qashqai are mapped.
    pub fn button_name(self, code: u8) -> Option<&'static str> {
        match self {
            RemoteTag::Nissan => match code {
                0x8 => Some("Lock"),
                0x2 => Some("Unlock"),
                _ => None,
            },
        }
    }
}

/// The tags of this session: the file's at startup, then every tag set.
static DB: RwLock<Vec<(u32, RemoteTag)>> = RwLock::new(Vec::new());

/// Bumped on every change of [`DB`], so callers (e.g. the detail-panel memo) can notice.
static GENERATION: AtomicU64 = AtomicU64::new(0);

/// The config folder [`load`] was called with: where [`set_tag`] writes.
static CONFIG_DIR: RwLock<Option<PathBuf>> = RwLock::new(None);

/// The store's version: changes whenever a tag is loaded, set or cleared.
pub fn generation() -> u64 {
    GENERATION.load(Ordering::Relaxed)
}

fn uid_ok(uid: u32) -> bool {
    uid != 0 && uid != u32::MAX
}

/// A record line as (UID, tag). `None` for comments, blank lines, bad lines, unknown tag words
/// and the UIDs 00000000 / FFFFFFFF.
pub fn parse_line(line: &str) -> Option<(u32, RemoteTag)> {
    let mut fields = line.split_whitespace();
    let uid_field = fields.next()?;
    if uid_field.len() != 8 || !uid_field.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    let uid = u32::from_str_radix(uid_field, 16).ok()?;
    let tag = RemoteTag::from_str(fields.next()?)?;
    if fields.next().is_some() || !uid_ok(uid) {
        return None;
    }
    Some((uid, tag))
}

/// A record line (with its newline).
pub fn format_line(uid: u32, tag: RemoteTag) -> String {
    format!("{uid:08X} {}\n", tag.as_str())
}

/// The tag of remote `uid` in this session: its first line.
pub fn tag_of(uid: u32) -> Option<RemoteTag> {
    if !uid_ok(uid) {
        return None;
    }
    let db = DB.read().unwrap_or_else(|e| e.into_inner());
    db.iter().find(|(u, _)| *u == uid).map(|(_, t)| *t)
}

/// Whether a capture's remote (by serial) is tagged Nissan.
pub fn is_nissan(serial: Option<u32>) -> bool {
    serial.map(tag_of) == Some(Some(RemoteTag::Nissan))
}

/// Use `tag` for remote `uid` for the rest of the session, in place of any earlier one.
fn remember(uid: u32, tag: Option<RemoteTag>) {
    let mut db = DB.write().unwrap_or_else(|e| e.into_inner());
    db.retain(|(u, _)| *u != uid);
    if let Some(tag) = tag {
        db.push((uid, tag));
    }
    GENERATION.fetch_add(1, Ordering::Relaxed);
}

/// Read `<config dir>/remote_tags.txt` if it exists (once, at startup). Returns a status line for
/// the user when there was a file.
pub fn load(config_dir: &Path) -> Option<String> {
    *CONFIG_DIR.write().unwrap_or_else(|e| e.into_inner()) = Some(config_dir.to_path_buf());
    let path = config_dir.join(FILE_NAME);
    let text = std::fs::read_to_string(&path).ok()?;
    let mut tags = 0;
    {
        let mut db = DB.write().unwrap_or_else(|e| e.into_inner());
        for (uid, tag) in text.lines().filter_map(parse_line) {
            if !db.iter().any(|(u, _)| *u == uid) {
                tags += 1;
                db.push((uid, tag));
            }
        }
        GENERATION.fetch_add(1, Ordering::Relaxed);
    }
    Some(format!("Loaded {tags} remote tag{} from {}", crate::plural(tags), path.display()))
}

/// Set (or clear, with `None`) the tag of remote `uid` from now on, and write the file: the
/// remote's line is replaced or appended (cleared = dropped), a new file starts with the header.
/// The session keeps the change even when the file cannot be written. Returns the file path.
pub fn set_tag(config_dir: &Path, uid: u32, tag: Option<RemoteTag>) -> std::io::Result<PathBuf> {
    if !uid_ok(uid) {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            format!("UID {uid:08X} cannot be tagged"),
        ));
    }
    remember(uid, tag);
    let path = config_dir.join(FILE_NAME);
    let existing = match std::fs::read_to_string(&path) {
        Ok(text) => Some(text),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
        Err(e) => return Err(e),
    };
    let mut out = String::new();
    let mut written = false;
    match &existing {
        None => out.push_str(HEADER),
        Some(text) => {
            for line in text.lines() {
                if parse_line(line).is_some_and(|(u, _)| u == uid) {
                    if !written {
                        if let Some(tag) = tag {
                            out.push_str(&format_line(uid, tag));
                        }
                        written = true;
                    }
                    continue;
                }
                out.push_str(line);
                out.push('\n');
            }
        }
    }
    if !written {
        if let Some(tag) = tag {
            out.push_str(&format_line(uid, tag));
        }
    }
    std::fs::create_dir_all(config_dir)?;
    let tmp = config_dir.join(format!("{FILE_NAME}.tmp"));
    std::fs::write(&tmp, out)?;
    std::fs::rename(&tmp, &path)?;
    Ok(path)
}

/// Set the tag for the process only (no file), for tests.
#[cfg(test)]
pub(crate) fn remember_for_test(uid: u32, tag: Option<RemoteTag>) {
    remember(uid, tag);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lines_round_trip() {
        assert_eq!(format_line(0x1A2B_3C4D, RemoteTag::Nissan), "1A2B3C4D nissan\n");
        assert_eq!(parse_line("1A2B3C4D nissan"), Some((0x1A2B_3C4D, RemoteTag::Nissan)));
        assert_eq!(parse_line("  1a2b3c4d\tnissan "), Some((0x1A2B_3C4D, RemoteTag::Nissan)));
        for bad in [
            "# KAT per-remote tags",
            "",
            "1A2B3C4D",
            "1A2B3C4D fiat",
            "1A2B3C4D nissan extra",
            "1A2B3C nissan",
            "00000000 nissan",
            "FFFFFFFF nissan",
        ] {
            assert_eq!(parse_line(bad), None, "{bad:?}");
        }
    }

    #[test]
    fn set_and_clear_writes_the_file() {
        let dir = std::env::temp_dir().join(format!("kat-remote-tags-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let uid = 0x7A61_0001;
        let path = set_tag(&dir, uid, Some(RemoteTag::Nissan)).unwrap();
        assert_eq!(std::fs::read_to_string(&path).unwrap(), format!("{HEADER}7A610001 nissan\n"));
        assert_eq!(tag_of(uid), Some(RemoteTag::Nissan));
        assert!(is_nissan(Some(uid)));
        // Clearing drops the line, keeps other lines.
        std::fs::write(&path, format!("{HEADER}12345678 nissan\n7A610001 nissan\n")).unwrap();
        set_tag(&dir, uid, None).unwrap();
        assert_eq!(std::fs::read_to_string(&path).unwrap(), format!("{HEADER}12345678 nissan\n"));
        assert_eq!(tag_of(uid), None);
        assert!(set_tag(&dir, 0, Some(RemoteTag::Nissan)).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn button_and_make_labels() {
        assert_eq!(RemoteTag::Nissan.button_name(0x8), Some("Lock"));
        assert_eq!(RemoteTag::Nissan.button_name(0x2), Some("Unlock"));
        assert_eq!(RemoteTag::Nissan.button_name(0x4), None);
        assert_eq!(RemoteTag::Nissan.display_protocol(), "Nissan V1");
        assert_eq!(RemoteTag::Nissan.make(), "Nissan");
    }
}
