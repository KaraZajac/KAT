//! Oracle harnesses and diagnostics for the rtl_433 ports (all `#[ignore]`d): the KAT side of the
//! comparisons with rtl_433 run on the same pulses, a sweep of every port over KAT's captures and
//! a cost measurement.

use super::bitbuffer::BitBuffer;
use super::pulse_detect::{Ends, Event, Gaps};
use super::pulse_slicer::{Modulation, Slicer};
use super::test_util::{decode_all, package_wire, read_ook, recording_bursts, rtl433_tests};
use super::{DecodeFn, Device};
use crate::protocols::*;
use crate::radio::demodulator::LevelDuration;

/// How a port's decode is printed for the oracle scripts.
type Render = fn(&DecodedSignal) -> String;

/// A port: its constructor and its [Render].
type Port = (fn() -> Box<dyn ProtocolDecoder>, Render);

/// Every rtl_433 port, with rtl_433's JSON or fields where the port renders them.
fn ports() -> Vec<Port> {
    vec![
        (|| Box::new(alps_fwb1u545::AlpsFwb1u545Decoder::new()), alps_fwb1u545::rtl433_json),
        (|| Box::new(astrostart_2000::Astrostart2000Decoder::new()), astrostart_2000::rtl433_json),
        (|| Box::new(audiovox_pro_oe3b::AudiovoxProOe3bDecoder::new()), audiovox_pro_oe3b::rtl433_json),
        (|| Box::new(chrysler_car_remote::ChryslerCarRemoteDecoder::new()), chrysler_car_remote::rtl433_json),
        (|| Box::new(code_alarm_frdpc2002::CodeAlarmFrdpc2002Decoder::new()), code_alarm_frdpc2002::rtl433_json),
        (|| Box::new(compustar_1wg3r::Compustar1wg3rDecoder::new()), compustar_1wg3r::rtl433_json),
        (|| Box::new(siemens_5wy72xx::Siemens5wy72xxDecoder::new()), siemens_5wy72xx::rtl433_json),
        (|| Box::new(continental_kr5v2x::ContinentalKr5v2xDecoder::new()), continental_kr5v2x::rtl433_fields),
        (|| Box::new(honda_kr5v2x1x::HondaKr5v2x1xDecoder::new()), honda_kr5v2x1x::rtl433_fields),
        (|| Box::new(mic6sc2::Mic6sc2Decoder::new()), mic6sc2::rtl433_fields),
        (|| Box::new(opel_mokka::OpelMokkaDecoder::new()), opel_mokka::rtl433_fields),
        (|| Box::new(elero::EleroDecoder::new()), fields),
        (|| Box::new(somfy_iohc::SomfyIohcDecoder::new()), fields),
        (|| Box::new(rojaflex::RojaflexDecoder::new()), fields),
        (|| Box::new(microchip_hcs361::MicrochipHcs361Decoder::new()), keeloq_fields),
        (|| Box::new(microchip_hcs362::MicrochipHcs362Decoder::new()), keeloq_fields),
        (|| Box::new(markisol::MarkisolDecoder::new()), fields),
    ]
}

/// A decode's KAT fields, for the ports without an rtl_433 rendering.
fn fields(d: &DecodedSignal) -> String {
    format!(
        "{:X} | {:X} | {:?} | {}/{} {:X?} | {}",
        d.serial.unwrap_or(0),
        d.button.unwrap_or(0),
        d.counter,
        d.data,
        d.data_count_bit,
        d.extra,
        d.protocol_display_name.as_deref().unwrap_or("-")
    )
}

/// A Microchip KeeLoq chip's decode as rtl_433 prints it: id, button (S3 S2 S1 S0), encrypted.
fn keeloq_fields(d: &DecodedSignal) -> String {
    let (fix, hop) = keeloq::fix_hop(d.data);
    let btn = (fix >> 28) as u8;
    format!("{:07X} | {} | {hop:08X}", fix & 0x0FFF_FFFF, ((btn & 1) << 3) | (btn >> 1))
}

/// The port named `spec` (its protocol name, any case), with an `r_device` variant after a colon
/// for the Microchip chips (`Microchip-HCS361:<0..5>`, `Microchip-HCS362:pwm|mc`).
fn port(spec: &str) -> (Box<dyn ProtocolDecoder>, Render) {
    let (name, variant) = spec.split_once(':').unwrap_or((spec, ""));
    let (make, render) = ports().into_iter().find(|(make, _)| make().name().eq_ignore_ascii_case(name)).unwrap_or_else(|| panic!("no rtl_433 port {name}"));
    let dec: Box<dyn ProtocolDecoder> = match variant {
        "" => make(),
        v if name.eq_ignore_ascii_case(microchip_hcs361::NAME) => Box::new(microchip_hcs361::MicrochipHcs361Decoder::only(v.parse().unwrap())),
        v if name.eq_ignore_ascii_case(microchip_hcs362::NAME) => Box::new(microchip_hcs362::MicrochipHcs362Decoder::only(v == "mc")),
        v => panic!("{name} has no variant {v}"),
    };
    (dec, render)
}

/// The KAT side of an oracle comparison: `KAT_RTL433_REPLAY=<file> KAT_RTL433_PORT=<port>` feeds
/// every stream of the file — the packages of an rtl_433 pulse file (`.ook`, each between idle
/// LOWs), else one train per line (`pulse gap …` µs, from a HIGH) — to a fresh decoder (`<port>`:
/// see [port]) and writes `<file>.kat`: `<stream>\t<decode>` per decode, `<stream>\tEND` per stream
/// (the C harness's output format).
#[test]
#[ignore = "oracle harness"]
fn replay() {
    let (Ok(path), Ok(spec)) = (std::env::var("KAT_RTL433_REPLAY"), std::env::var("KAT_RTL433_PORT")) else {
        eprintln!("Skip: set KAT_RTL433_REPLAY=<file> and KAT_RTL433_PORT=<port>");
        return;
    };
    crate::protocols::keys::load_keystore_from_embedded();
    let text = std::fs::read_to_string(&path).unwrap();
    let streams: Vec<Vec<LevelDuration>> = if path.ends_with(".ook") {
        read_ook(&text).iter().map(|p| package_wire(p)).collect()
    } else {
        text.lines()
            .map(|l| l.split_whitespace().enumerate().map(|(k, x)| LevelDuration::new(k % 2 == 0, x.parse().unwrap())).collect())
            .collect()
    };
    let mut out = String::new();
    for (i, s) in streams.iter().enumerate() {
        let (mut dec, render) = port(&spec);
        for d in decode_all(dec.as_mut(), s) {
            out += &format!("{i}\t{}\n", render(&d));
        }
        out += &format!("{i}\tEND\n");
    }
    std::fs::write(format!("{path}.kat"), out).unwrap();
}

/// A `decode_fn` that prints every message's rows as rtl_433 codes.
struct Codes(Vec<String>);

impl DecodeFn for Codes {
    fn message(&mut self, bits: &mut BitBuffer) -> Option<DecodedSignal> {
        self.0.push((0..bits.num_rows()).map(|r| bits.code(r)).collect::<Vec<_>>().join(" "));
        None
    }
}

/// The slicers against rtl_433's: `KAT_RTL433_SLICE=<file> KAT_RTL433_DEVICE=<modulation>,<short>,
/// <long>,<gap>,<reset>,<tolerance>,<sync>` (modulation `pwm`, `ppm`, `zerobit`, `piwm` or `pcm`;
/// µs) slices every package of an rtl_433 pulse file as rtl_433 slices a package (`rtl_433 -r`)
/// and prints each message's rows as codes, `<package> {bits}hex …`, to diff with
/// `rtl_433 -s 1M -r <file> -R 0 -X 'n=x,m=OOK_PCM,s=…'` (the flex decoder's rows).
#[test]
#[ignore = "oracle harness"]
fn slice() {
    let (Ok(path), Ok(spec)) = (std::env::var("KAT_RTL433_SLICE"), std::env::var("KAT_RTL433_DEVICE")) else {
        eprintln!("Skip: set KAT_RTL433_SLICE=<file> and KAT_RTL433_DEVICE=<modulation>,<short>,<long>,<gap>,<reset>,<tolerance>,<sync>");
        return;
    };
    let f: Vec<&str> = spec.split(',').collect();
    let n = |i: usize| f.get(i).and_then(|v| v.parse().ok()).unwrap_or(0);
    let modulation = match f[0] {
        "pwm" => Modulation::OokPwm,
        "ppm" => Modulation::OokPpm,
        "zerobit" => Modulation::OokManchesterZerobit,
        "piwm" => Modulation::OokPiwmDc,
        "pcm" => Modulation::OokPcm,
        m => panic!("unknown modulation {m}"),
    };
    let device = Device { modulation, short_width: n(1), long_width: n(2), gap_limit: n(3), reset_limit: n(4), tolerance: n(5), sync_width: n(6), gaps: Gaps::Merged, ends: Ends::Ook, resync: false };
    for (i, pkg) in read_ook(&std::fs::read_to_string(path).unwrap()).iter().enumerate() {
        let mut slicer = Slicer::new(&device);
        let mut bits = BitBuffer::new();
        slicer.new_package(&mut bits);
        let mut codes = Codes(Vec::new());
        for (k, &(pulse, gap)) in pkg.iter().enumerate() {
            let last = k + 1 == pkg.len();
            if let Slicer::Pcm(pcm) = &mut slicer {
                pcm.event(Event::Pulse(pulse));
                if pcm.event(Event::Gap { gap, last }) {
                    pcm.slice(&mut bits, &mut codes, &mut None);
                }
                continue;
            }
            let mut flush = |slicer: &mut Slicer, bits: &mut BitBuffer| {
                codes.message(bits);
                bits.clear();
                slicer.after_message(bits);
            };
            if slicer.pulse(&mut bits, pulse) {
                flush(&mut slicer, &mut bits);
            }
            if slicer.gap(&mut bits, gap, last) {
                flush(&mut slicer, &mut bits);
            }
        }
        for c in codes.0 {
            println!("{i} {c}");
        }
    }
}

/// Diagnostic: every rtl_433 port ALONE (no first-match registry, both polarities) over every
/// IMPORTS capture, the Flipper unit-test captures and the rtl_433_tests recordings (the AM and
/// FM pulses of every burst); prints every decode.
#[test]
#[ignore = "diagnostic sweep, minutes in a debug build"]
fn standalone_sweep() {
    crate::protocols::keys::load_keystore_from_embedded();
    let mut decoders: Vec<Box<dyn ProtocolDecoder>> = ports().iter().map(|(make, _)| make()).collect();
    let mut files = Vec::new();
    let mut stack = vec![std::path::PathBuf::from("IMPORTS"), std::path::PathBuf::from("REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz")];
    stack.extend(rtl433_tests());
    while let Some(d) = stack.pop() {
        for e in std::fs::read_dir(&d).into_iter().flatten().flatten() {
            let p = e.path();
            if p.is_dir() {
                stack.push(p);
            } else if p.extension().is_some_and(|x| x == "sub" || x == "cu8") {
                files.push(p);
            }
        }
    }
    files.sort();
    for path in &files {
        let mut streams: Vec<(&str, Vec<LevelDuration>)> = Vec::new();
        if path.extension().is_some_and(|x| x == "sub") {
            let Ok((_, raw)) = crate::export::flipper::import_sub_raw(path) else { continue };
            streams.push(("raw", raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect()));
        } else {
            for (am, fm, _) in recording_bursts(path) {
                streams.extend(am.map(|p| ("am", p)));
                streams.extend(fm.map(|p| ("fm", p)));
            }
        }
        for (tag, s) in &streams {
            for invert in [false, true] {
                for dec in decoders.iter_mut() {
                    let wire: Vec<LevelDuration> = s.iter().map(|p| LevelDuration::new(p.level != invert, p.duration_us)).collect();
                    for d in decode_all(dec.as_mut(), &wire) {
                        let name = d.protocol_display_name.clone().unwrap_or_else(|| dec.name().to_string());
                        println!("HIT {} | {tag}{} | {name} | {}", path.display(), if invert { " inv" } else { "" }, fields(&d));
                    }
                }
            }
        }
    }
    println!("SWEEP {} files", files.len());
}

/// Diagnostic: the time each rtl_433 port takes alone over every registry encoder's upload (both
/// polarities), the input of the registry guard tests. Run in release.
#[test]
#[ignore = "diagnostic, run in release"]
fn decoder_cost() {
    use std::panic::{catch_unwind, AssertUnwindSafe};
    let reg = ProtocolRegistry::new();
    let mut wires = Vec::new();
    for name in reg.list_protocols() {
        let dec = reg.get(name).unwrap();
        if !dec.supports_encoding() {
            continue;
        }
        let src = DecodedSignal {
            serial: Some(0x0012_3456),
            button: Some(0x01),
            counter: Some(5),
            crc_valid: true,
            data: 0xA5C3_1E7B_92D4_6F08,
            data_count_bit: dec.timing().min_count_bit,
            encoder_capable: true,
            extra: Some(0x1234_5678_9ABC),
            protocol_display_name: None,
        };
        for btn in [0x01u8, 0x02, 0x04] {
            if let Ok(Some(w)) = catch_unwind(AssertUnwindSafe(|| dec.encode(&src, btn))) {
                wires.push(w);
            }
        }
    }
    let pulses: usize = wires.iter().map(Vec::len).sum();
    println!("{} uploads, {pulses} pulses", wires.len());
    for (make, _) in ports() {
        let mut dec = make();
        let t = std::time::Instant::now();
        let mut hits = 0;
        let mut slowest = (std::time::Duration::ZERO, 0usize, false);
        for (k, w) in wires.iter().enumerate() {
            for invert in [false, true] {
                let tw = std::time::Instant::now();
                let wire: Vec<LevelDuration> = w.iter().map(|p| LevelDuration::new(p.level != invert, p.duration_us)).collect();
                hits += decode_all(dec.as_mut(), &wire).len();
                if tw.elapsed() > slowest.0 {
                    slowest = (tw.elapsed(), k, invert);
                }
            }
        }
        println!("{}: {:?} ({hits} decodes; slowest wire {:?})", dec.name(), t.elapsed(), slowest);
    }
}
