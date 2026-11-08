//! Test-only helpers shared by the FoxFW Honda decoders (V2&V3, V4, V6..V10): synthetic pulse
//! builders and a Flipper RAW `.sub` writer, so that the very pulses the unit tests feed KAT can be
//! given to the FoxFW host oracle (`KAT_DUMP_SYNTH=<dir> cargo test honda_ -- --nocapture`).

use super::{DecodedSignal, ProtocolDecoder};

/// One pulse: (level, duration in µs).
pub(super) type Pulse = (bool, u32);

/// Feed `pulses` to `dec`, collecting every frame it reports.
pub(super) fn decode(dec: &mut dyn ProtocolDecoder, pulses: &[Pulse]) -> Vec<DecodedSignal> {
    pulses.iter().filter_map(|&(l, d)| dec.feed(l, d)).collect()
}

/// Half-bit samples to pulses at `half` µs per sample: equal neighbours merge (1 or 2 samples).
pub(super) fn halves_to_pulses(halves: &[bool], half: u32) -> Vec<Pulse> {
    let mut out: Vec<Pulse> = Vec::new();
    for &h in halves {
        match out.last_mut() {
            Some(last) if last.0 == h => last.1 += half,
            _ => out.push((h, half)),
        }
    }
    out
}

/// `bytes` MSB-first as bits.
pub(super) fn bits_of(bytes: &[u8]) -> Vec<bool> {
    bytes.iter().flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1 == 1)).collect()
}

/// Write `pulses` as a Flipper RAW .sub file in `$KAT_DUMP_SYNTH` (nothing when unset).
pub(super) fn dump(name: &str, pulses: &[Pulse], preset: &str) {
    let Some(dir) = std::env::var_os("KAT_DUMP_SYNTH") else { return };
    let mut text = format!(
        "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 433920000\nPreset: {preset}\nProtocol: RAW\n"
    );
    for chunk in pulses.chunks(256) {
        text.push_str("RAW_Data:");
        for &(level, us) in chunk {
            text.push_str(&format!(" {}{}", if level { "" } else { "-" }, us));
        }
        text.push('\n');
    }
    let path = std::path::Path::new(&dir).join(format!("{name}.sub"));
    std::fs::write(&path, text).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
}

/// Each synthetic `.sub` in `$KAT_DUMP_SYNTH`, fed to its own decoder alone (the file name starts
/// with the module name), one line per frame: `ORACLE_CMP|file|protocol|data|bits|serial|button|
/// counter`. The same fields come from the FoxFW oracle's `decode --protocol <name>` (counter
/// masked to 16 bits): the comparison of KAT's ports against the C.
#[test]
#[ignore = "run with KAT_DUMP_SYNTH=<dir> after the other tests wrote the files"]
fn honda_oracle_compare() {
    use super::{honda_v10, honda_v2_v3, honda_v4, honda_v6, honda_v7, honda_v8, honda_v9};
    let Some(dir) = std::env::var_os("KAT_DUMP_SYNTH") else { return };
    let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().flatten().map(|e| e.path()).collect();
    files.sort();
    for path in files {
        let name = path.file_stem().unwrap().to_string_lossy().to_string();
        let mut dec: Box<dyn ProtocolDecoder> = match name.split('_').next().unwrap() {
            "hondav2v3" => Box::new(honda_v2_v3::HondaV2V3Decoder::new()),
            "honda" => match name.split('_').nth(1).unwrap() {
                "v4" => Box::new(honda_v4::HondaV4Decoder::new()),
                "v6" => Box::new(honda_v6::HondaV6Decoder::new()),
                "v7" => Box::new(honda_v7::HondaV7Decoder::new()),
                "v8" => Box::new(honda_v8::HondaV8Decoder::new()),
                "v9" => Box::new(honda_v9::HondaV9Decoder::new()),
                "v10" => Box::new(honda_v10::HondaV10Decoder::new()),
                _ => continue,
            },
            _ => continue,
        };
        let (_, pairs) = crate::export::flipper::import_sub_raw(&path).unwrap();
        for p in &pairs {
            if let Some(d) = dec.feed(p.level, p.duration_us) {
                println!(
                    "ORACLE_CMP|{name}|{}|{}|{}|{}|{}|{}",
                    dec.name(),
                    d.data,
                    d.data_count_bit,
                    d.serial.unwrap(),
                    d.button.unwrap(),
                    d.counter.unwrap()
                );
            }
        }
    }
}
