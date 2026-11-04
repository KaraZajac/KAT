---
layout: default
---

# Sessions and saved-fob matching

**Rust modules:** `src/session.rs` (file format, I/O), `src/saved_fobs.rs` (index, match rules),
`src/app/persistence.rs` and `src/app/fob_matching.rs` (app glue)
**Reference:** `REFERENCES/ProtoPirate/helpers/protopirate_saved_match.c` (saved-signal matching)

## Sessions

Captures used to live only in memory: quitting KAT lost every capture that had not been exported
one by one. KAT now autosaves the whole capture list and offers to restore it at the next start.

### What is saved

`~/.config/KAT/session.json` holds every capture with every field: protocol and decoded fields,
`data_extra`, the raw level/duration pairs, `received_rf`, status, the user metadata set with **i**
(year, make, model, region, command) and the source file of imports. It also holds
`next_capture_id`, so capture ids continue where the last session stopped.

```json
{
  "format": "kat-session",
  "version": 1,
  "kat_version": "1.5.0",
  "saved_at": "2026-11-04T10:31:07.123456789Z",
  "next_capture_id": 42,
  "captures": [ { "id": 1, "timestamp": "…", "frequency": 433920000, "protocol": "Ford V0", "…": "…",
                  "raw_pairs": [ { "level": true, "duration_us": 250 }, … ] } ]
}
```

Each capture is written by `Capture`'s own serde derive, so a field added to `Capture` later is saved
without a second schema. The file is compact JSON: a 375-pair capture is about 11 KB, and a
worst-case session (100 captures × 4096 pairs, 14 MB) loads in about 80 ms and writes in about 60 ms
(release build, desktop PC).

### When it is saved

- **Autosave:** about ten times a second KAT compares a fingerprint of the capture list with the last
  saved state. When something changed (new capture, import, delete, metadata, PSA key recovery …)
  it writes a snapshot, at most once per second.
- **On exit:** KAT saves once more on every exit path (`q`, `:q`, an error, a panic that unwinds) and
  waits for the write to finish.
- Writes run on a background thread, so the UI never waits for the disk. A write that fails shows
  "session autosave failed: …" in the status bar; the session in memory is unaffected.

Each write is **atomic** (temp file `session.json.tmp` in the same directory, fsync, rename over
`session.json`, directory fsync), so a crash or power loss leaves either the old or the new session,
never a torn file. The file is created **owner-only (0600)** because captures hold rolling codes.
An empty capture list removes `session.json` instead of writing an empty session.

### Restoring

If `session.json` holds captures, KAT asks at startup:

```
Restore 12 captures from your last session?
saved 2026-11-04 10:31 UTC
(y/n)
```

- **y** restores the captures with their ids. Captures that match a saved `.fob` get their ★ marker
  (see below) without a status message.
- **n** starts an empty session and keeps the old file as `session.json.bak` (one generation; the
  next decline replaces it). Rename it back to `session.json` to get it offered again.

Order of the startup prompts: the "no HackRF or RTL-SDR" warning first, then the restore prompt,
then the import prompt for files in the import folder. Restored captures keep their ids and imports
are numbered after them. Import files whose captures are already in the restored session (same
path relative to the import folder) are not offered again; if none are left, the import prompt
is skipped.

### Robustness

- **Older and newer files:** loading ignores unknown fields and takes defaults for missing optional
  fields. A capture that lacks `raw_pairs`, `crc_valid`, `data`, `data_count_bit`, `status`, `id` or
  `timestamp` gets a safe default (ids are renumbered past the highest one). A capture without a
  `frequency` is skipped, because a guessed frequency would transmit on the wrong band.
- **A broken capture** inside an otherwise readable file is skipped and counted. The rest is restored,
  and the original file is copied to `session.json.bak` before the next autosave rewrites it (also
  done for a file from a newer format version).
- **An unreadable file** (not JSON, not a KAT session) does not stop KAT: the status bar shows
  "Saved session unreadable (…); kept as session.json.bak" and startup continues.

### `max_captures`

The saved session keeps the **newest** `max_captures` captures of the list (config `[general]`,
default 100, `0` = no limit). The list in memory is not trimmed. When it grows past the limit the
status bar says so once ("The saved session keeps the newest 100 captures …"), so you can export
older captures you want to keep. A session file with more captures than the current limit restores
its newest `max_captures`, and the prompt then says "Restore the newest 100 of 130 captures".

### `:clear`

`:clear` asks "Delete all N captures and the saved session file? (y/n)". **y** empties the list and
deletes `session.json`, `session.json.bak` and any temp file. Exported `.fob` / `.sub` files are not
touched. `:delete all` only empties the list (the session follows on the next autosave, but a
`session.json.bak` stays).

### Configuration

```ini
[session]
; Autosave every capture (raw pairs and metadata included) to session.json in this
; directory and offer to restore them at the next start. The file holds rolling codes:
; it is written with owner-only permissions (0600). :clear deletes it.
session_autosave = true
```

The default is `true` (also when the key is missing, as in a `config.ini` written by KAT 1.4; the
key is accepted in `[general]` too). With `false`, KAT neither writes nor offers a session. It leaves
an existing `session.json` alone, and `:clear` still deletes it.

One session per config directory: two KAT instances running at the same time overwrite each other's
session (the last write wins).

## Saved-fob matching

When a decoded capture is one of the fobs you already saved as a `.fob` file, KAT says so. This mirrors
ProtoPirate's "Check Saved" (`protopirate_saved_match_signal`).

### Rules (ProtoPirate)

A capture matches a saved file when:

1. **Same protocol.** KAT compares canonical names as `ProtocolRegistry::get()` resolves them, so
   retired names and display variants match: `PSA2` = `PSA`, `Kia V3` / `Kia V4` = `Kia V3/V4`,
   `Land Rover V0` = `Honda V2`, `Porsche Touareg` = `Porsche Cayenne`, `Suzuki` = `Kia V0`,
   `Star Line [SL_A2-A4]` = `Star Line`, and the generic fallback's `Keeloq (X)` = `KeeLoq (X)`.
2. **Same identity.** If the capture has a serial, the saved file must have the same serial. Without a
   serial, the key (data hex, the `.fob` `key` field) must be equal.
3. **Counter within ±50** (`CNT_MATCH_MARGIN`), checked only when both sides have a counter.

Captures with no protocol (unknown signals) never match.

KAT differences:

- KAT stores 16-bit counters, so the difference is taken modulo 2^16 (`0xFFFE` → `0x0003` is `+5`).
  ProtoPirate keeps the full counter, so a counter that wraps in its low 16 bits must not break the match.
- ProtoPirate stops at the first match in directory order. KAT picks the match with the smallest
  counter difference (then path order), so the nearest saved press is named; a saved file without a
  counter ranks after every counter match.

### Which files

All `.fob` files (v1 and v2 layout) under the **export** and **import** directories, including
subfolders. `.sub` files hold no decoded fields and are not indexed. The index is built on a background
thread at startup and again after every `.fob` export, so a fob you just exported matches from then on.
Unreadable files and unknown-protocol exports are skipped.

### What you see

- **Capture list:** a ★ after the ID (`03★`).
- **Detail panel:**

  ```
  Saved:     ★ 2019_Hyundai_Elantra_NA_Lock_1A2B3C4D  (cnt +3)
             2019 Hyundai Elantra (NA), Lock — spare key
  ```

  It shows the saved file name, the counter difference (received − saved: `cnt +3` means the fob has
  been pressed three times since the saved capture), and the year/make/model/region/command and notes stored
  in the file.
- **Status bar** for a live capture: "Signal #12 matches saved fob 2019_Hyundai_Elantra_NA_Lock_1A2B3C4D
  (cnt +3)". Restored and imported captures are annotated silently.

A capture is matched again when its decoded fields change (e.g. a PSA "Recover key" adds the serial)
and when the index is rebuilt.
