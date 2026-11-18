//! The protocol facts the site's pages are built from: `docs/data/protocols.json`.
//!
//! kat.karazajac.io has a page per protocol (`docs/<module>.md`) with a box of facts beside it,
//! and tables of all protocols. Those facts are the ones KAT itself uses: [info::ALL] and what
//! each registered decoder says about its timing, bit count, frequencies, encoder and button names. This test
//! writes them out, so the site cannot say something else than the program. With them goes the
//! entry each display name in the two pinned capture lists belongs to (`KeeLoq (DoorHan)` is
//! KeeLoq's), as [info::lookup] resolves it, and the entry that reads each `Protocol:` name of the
//! key files in IMPORTS (`KIA/HYU V0` is Kia V0's), as loading a key file resolves it:
//!
//! ```text
//! KAT_SITE_DATA=docs/data/protocols.json cargo test --bin kat site_data -- --ignored
//! ```
//!
//! (`scripts/site_data.py` runs it and writes the list of captures beside it.) Run it on the tree
//! that is published.

use super::{flipper_key, info, ProtocolRegistry, RTL433_PORTS};
use std::collections::BTreeMap;
use std::path::Path;

/// The `Protocol:` names of the key files under `dir`, each with the entry whose decoder loads it.
fn key_names(reg: &ProtocolRegistry, dir: &Path, out: &mut BTreeMap<String, &'static str>) {
    let Ok(list) = std::fs::read_dir(dir) else { return };
    for path in list.flatten().map(|e| e.path()) {
        if path.is_dir() {
            key_names(reg, &path, out);
            continue;
        }
        let Ok(bytes) = std::fs::read(&path) else { continue };
        let head = String::from_utf8_lossy(&bytes[..bytes.len().min(600)]).into_owned();
        if !head.lines().next().is_some_and(|l| l.contains("Key File")) {
            continue;
        }
        let Some(name) = head.lines().find_map(|l| l.strip_prefix("Protocol:")).map(str::trim) else { continue };
        let registry = flipper_key::registry_name(name);
        if let Some(i) = reg.get(registry).and_then(|d| info::lookup(d.name())) {
            out.insert(name.to_string(), i.name);
        }
    }
}

/// One entry of [info::ALL] as the site wants it.
fn entry(reg: &ProtocolRegistry, order: &[&'static str], i: &info::ProtocolInfo) -> serde_json::Value {
    let dec = order.contains(&i.decoder).then(|| reg.get(i.decoder)).flatten();
    let timing = dec.map(|d| d.timing());
    // The protocol's own button names, code by code (not "Other", and not a combination of flags).
    let buttons: Vec<(u8, &'static str)> = (0..=255u8)
        .filter_map(|code| Some((code, i.button_name(i.name, code)?)))
        .filter(|(_, name)| !name.is_empty() && !matches!(*name, "Other" | "Unknown") && !name.contains(';'))
        .collect();
    serde_json::json!({
        "name": i.name,
        "decoder": i.decoder,
        "aliases": i.aliases,
        "prefix": i.name_prefix,
        "coding": format!("{:?}", i.encoding),
        "rf": format!("{:?}", i.rf),
        "encryption": i.encryption,
        "make": i.make,
        "category": format!("{:?}", i.category),
        "presets": i.presets.iter().map(|p| p.name).collect::<Vec<_>>(),
        "registered": dec.is_some(),
        "order": order.iter().position(|n| *n == i.decoder),
        "rtl433": RTL433_PORTS.contains(&i.decoder),
        "te_short": timing.as_ref().map(|t| t.te_short),
        "te_long": timing.as_ref().map(|t| t.te_long),
        "te_delta": timing.as_ref().map(|t| t.te_delta),
        "bits": timing.as_ref().map(|t| t.min_count_bit),
        "frequencies": dec.map(|d| d.supported_frequencies().to_vec()),
        "encoder": dec.map(|d| d.supports_encoding()),
        "buttons": buttons,
        "generic_buttons": i.buttons.is_none(),
    })
}

#[test]
#[ignore]
fn site_data() {
    let Some(path) = std::env::var_os("KAT_SITE_DATA") else {
        eprintln!("Skip: KAT_SITE_DATA not set (the file to write)");
        return;
    };
    let reg = ProtocolRegistry::new();
    let order = reg.list_protocols();
    let protocols: Vec<serde_json::Value> = info::ALL.iter().map(|i| entry(&reg, &order, i)).collect();
    // Display names in the pinned lists: a line is `file | frequency | [key |] name | …`.
    let mut display = BTreeMap::new();
    for list in ["testdata/imports_decodes.txt", "testdata/imports_more_decodes.txt"] {
        let Ok(text) = std::fs::read_to_string(list) else {
            eprintln!("Skip: {list} not found (run from crate root)");
            continue;
        };
        for line in text.lines() {
            let name = line.split(" | ").skip(1).find(|f| *f != "key" && !f.bytes().all(|b| b.is_ascii_digit()));
            if let Some((name, i)) = name.and_then(|n| Some((n, info::lookup(n)?))) {
                display.insert(name.to_string(), i.name);
            }
        }
    }
    let mut keys = BTreeMap::new();
    key_names(&reg, Path::new("IMPORTS"), &mut keys);
    let doc = serde_json::json!({
        "version": env!("CARGO_PKG_VERSION"),
        "decoders": order.len(),
        "protocols": protocols,
        "display_names": display,
        "key_names": keys,
    });
    std::fs::write(&path, serde_json::to_string_pretty(&doc).unwrap() + "\n").unwrap();
    eprintln!("site data: {} entries, {} registered decoders", info::ALL.len(), order.len());
}
