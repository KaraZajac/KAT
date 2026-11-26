//! `kat --soapy-helper find | rx …`: the process that loads SoapySDR ([super]). It writes only
//! frames ([super::protocol]) or JSON lines to stdout; what it, SoapySDR and the radios' libraries
//! have to say goes to stderr, which KAT copies into kat.log.

use super::ffi::{Device, Range, Soapy, OVERFLOW, TIMEOUT};
use super::protocol::{self, Command, Listing, Ready, ERROR, IQ, READY, TUNE_FAILED};
use crate::radio::rx::lo_for;
use std::io::{self, BufRead, Write};
use std::sync::mpsc;

/// SoapySDR drivers KAT leaves out: the RTL-SDRs and HackRFs it drives itself, the modules that
/// wrap those again (osmosdr), sound cards (audio), and SoapySDR's test device.
const LEFT_OUT: &[&str] = &["rtlsdr", "hackrf", "osmosdr", "audio", "null"];

/// Samples per read.
const READ_SAMPLES: usize = 1 << 16;
/// Samples gathered into one frame at least (8 ms at 2 MS/s), unless [FRAME_TIME] passes first:
/// a radio may hand over a few samples per read, and a frame each would cost KAT a write, a read
/// and a pass of the pipeline for each.
const FRAME_SAMPLES: usize = 1 << 14;
const FRAME_TIME: std::time::Duration = std::time::Duration::from_millis(50);

/// The helper's entry point (`main` calls it for `--soapy-helper`); the process's exit code.
pub fn main(args: impl Iterator<Item = String>) -> i32 {
    let args: Vec<String> = args.collect();
    match args.first().map(String::as_str) {
        Some("find") => find(),
        Some("rx") => rx(&args[1..]),
        _ => {
            eprintln!("usage: kat {} find | rx <Hz> <dB|auto> <key=value>…", super::HELPER_FLAG);
            2
        }
    }
}

/// Whether KAT leaves out the radio these arguments name ([LEFT_OUT]).
pub fn left_out(args: &[(String, String)]) -> bool {
    args.iter().any(|(k, v)| k == "driver" && LEFT_OUT.contains(&v.as_str()))
}

fn find() -> i32 {
    let print = |l: &Listing| println!("{}", serde_json::to_string(l).unwrap_or_default());
    let soapy = match Soapy::load() {
        Ok(s) => s,
        Err(tried) => {
            print(&Listing::Missing(tried));
            return 0;
        }
    };
    soapy.log_to_stderr();
    eprintln!("SoapySDR {} (ABI {}) from {}", soapy.version, soapy.abi, soapy.path);
    match soapy.enumerate() {
        Ok(radios) => {
            for args in radios {
                if left_out(&args) {
                    eprintln!("left out: {}", describe(&args));
                } else {
                    print(&Listing::Radio(args));
                }
            }
            0
        }
        Err(e) => {
            print(&Listing::Error(e));
            1
        }
    }
}

fn describe(args: &[(String, String)]) -> String {
    args.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(", ")
}

/// KAT's 2 MS/s when the radio has it; else the lowest rate above it up to 10 MS/s (an Airspy
/// R2: 2.5 MS/s), else the highest below it from 250 kS/s.
pub fn choose_rate(ranges: &[Range]) -> f64 {
    const WANT: f64 = 2e6;
    if ranges.is_empty() || ranges.iter().any(|r| r.minimum <= WANT && WANT <= r.maximum) {
        return WANT;
    }
    let above = ranges.iter().map(|r| r.minimum).filter(|&m| m > WANT && m <= 10e6).fold(f64::NAN, f64::min);
    if !above.is_nan() {
        return above;
    }
    let below = ranges.iter().map(|r| r.maximum).filter(|&m| (250e3..WANT).contains(&m)).fold(f64::NAN, f64::max);
    if below.is_nan() {
        WANT
    } else {
        below
    }
}

/// Write an error frame; the exit code that goes with it.
fn fail(out: &mut impl Write, why: &str) -> i32 {
    eprintln!("{why}");
    let _ = protocol::write_frame(out, ERROR, &[why.as_bytes()]);
    1
}

fn rx(args: &[String]) -> i32 {
    let stdout = io::stdout();
    let mut out = io::BufWriter::with_capacity(1 << 20, stdout.lock());
    let Some((target, gain, device_args)) = protocol::parse_rx_args(args) else {
        return fail(&mut out, &format!("bad arguments: {args:?}"));
    };
    let soapy = match Soapy::load() {
        Ok(s) => s,
        Err(tried) => return fail(&mut out, &format!("SoapySDR is not installed ({tried})")),
    };
    soapy.log_to_stderr();
    eprintln!("SoapySDR {} (ABI {}); opening {}", soapy.version, soapy.abi, describe(&device_args));
    let mut dev = match soapy.make(&device_args) {
        Ok(d) => d,
        Err(e) => return fail(&mut out, &format!("cannot open the radio: {e}")),
    };
    match receive(&soapy, &mut dev, &mut out, target, gain) {
        Ok(()) => 0,
        Err(why) => fail(&mut out, &why),
    }
    // (the radio closes as `dev` drops)
}

fn receive(soapy: &Soapy, dev: &mut Device, out: &mut impl Write, target: u32, gain: Option<f64>) -> Result<(), String> {
    let ranges = dev.sample_rate_ranges();
    let wanted = choose_rate(&ranges);
    dev.set_sample_rate(wanted)?;
    let rate = match dev.sample_rate() {
        r if r.is_finite() && (1.0..=100e6).contains(&r) => r.round() as u32,
        _ => wanted as u32,
    };
    dev.set_frequency(f64::from(lo_for(target, rate)))?;
    dev.set_gain(gain)?;
    let range = dev.gain_range();
    dev.start_stream()?;
    let ready = Ready {
        rate,
        driver: dev.driver_key(),
        hardware: dev.hardware_key(),
        gain_min: range.minimum,
        gain_max: range.maximum,
    };
    eprintln!("streaming: {ready:?}");
    let ready = serde_json::to_vec(&ready).map_err(|e| e.to_string())?;
    protocol::write_frame(out, READY, &[&ready]).map_err(|e| e.to_string())?;

    // KAT's commands, from stdin; its end (KAT gone) stops the helper too.
    let (cmd_tx, commands) = mpsc::channel();
    std::thread::spawn(move || {
        for line in io::stdin().lock().lines().map_while(Result::ok) {
            if let Some(c) = Command::parse(&line) {
                if cmd_tx.send(c).is_err() {
                    return;
                }
            }
        }
        let _ = cmd_tx.send(Command::Quit);
    });

    let mut samples = vec![0f32; 2 * READ_SAMPLES];
    // The frame being gathered: the samples as bytes, and since when.
    let mut bytes = Vec::with_capacity(8 * (READ_SAMPLES + FRAME_SAMPLES));
    let mut since = std::time::Instant::now();
    let mut tag = target;
    // After a retune, samples already in the radio's buffers are from the old frequency: 20 ms
    // of them are dropped.
    let mut drop_left = 0usize;
    let mut overflows = 0u64;
    loop {
        while let Ok(cmd) = commands.try_recv() {
            match cmd {
                Command::Tune(t) => match dev.set_frequency(f64::from(lo_for(t, rate))) {
                    Ok(()) => {
                        // what was gathered is the old frequency's
                        if !bytes.is_empty() {
                            write_iq(out, tag, &mut bytes)?;
                            since = std::time::Instant::now();
                        }
                        tag = t;
                        drop_left = rate as usize / 50;
                    }
                    Err(e) => {
                        eprintln!("retune to {t} Hz failed: {e}");
                        protocol::write_frame(out, TUNE_FAILED, &[&t.to_le_bytes()]).map_err(|e| e.to_string())?;
                    }
                },
                Command::Gain(g) => {
                    if let Err(e) = dev.set_gain(g) {
                        eprintln!("{e}");
                    }
                }
                Command::Quit => return Ok(()),
            }
        }
        let n = dev.read(&mut samples, 100_000);
        let got: &[f32] = match n {
            n if n > 0 => take_read_samples(&samples, n as usize, &mut drop_left)?,
            0 | TIMEOUT => &[],
            OVERFLOW => {
                overflows += 1;
                if overflows.is_power_of_two() {
                    eprintln!("samples dropped (overflow) {overflows} time(s)");
                }
                // Fall through so the 50 ms heartbeat frame is still sent during sustained overflows.
                &[]
            }
            e => return Err(format!("reading the radio failed: {}", soapy.err_str(e))),
        };
        bytes.extend(got.iter().flat_map(|v| v.to_le_bytes()));
        // A frame once enough is gathered, or at least every 50 ms (an empty frame is the
        // heartbeat). A write that fails: KAT is gone.
        if bytes.len() >= 8 * FRAME_SAMPLES || since.elapsed() >= FRAME_TIME {
            if write_iq(out, tag, &mut bytes).is_err() {
                return Ok(());
            }
            since = std::time::Instant::now();
        }
    }
}

/// Slice the newly read complex samples (2 `f32`s per sample), dropping up to `*drop_left`
/// post-retune samples first. Returns `Err` if the C driver reported more samples than fit in
/// `samples`, preventing an out-of-bounds slice panic.
fn take_read_samples<'a>(samples: &'a [f32], count: usize, drop_left: &mut usize) -> Result<&'a [f32], String> {
    let max_samples = samples.len() / 2;
    if count > max_samples {
        return Err(format!("SoapySDR driver returned {count} samples for a {max_samples}-sample buffer"));
    }
    let skip = (*drop_left).min(count);
    *drop_left -= skip;
    Ok(&samples[2 * skip..2 * count])
}

/// Write the gathered samples as an IQ frame for `tag`, and start gathering again.
fn write_iq(out: &mut impl Write, tag: u32, bytes: &mut Vec<u8>) -> Result<(), String> {
    let r = protocol::write_frame(out, IQ, &[&tag.to_le_bytes(), bytes]).map_err(|e| e.to_string());
    bytes.clear();
    r
}

#[cfg(test)]
mod tests {
    use super::*;

    fn range(min: f64, max: f64) -> Range {
        Range { minimum: min, maximum: max, step: 0.0 }
    }

    /// 2 MS/s when the radio has it, else the nearest above (to 10 MS/s), else below.
    #[test]
    fn the_sample_rate_is_2_ms_s_or_the_nearest_the_radio_has() {
        assert_eq!(choose_rate(&[range(0.5e6, 10e6)]), 2e6, "a continuous range (SDRplay, LimeSDR)");
        assert_eq!(choose_rate(&[range(2.5e6, 2.5e6), range(10e6, 10e6)]), 2.5e6, "an Airspy R2");
        assert_eq!(choose_rate(&[range(3e6, 3e6), range(6e6, 6e6), range(10e6, 10e6)]), 3e6, "an Airspy Mini");
        assert_eq!(choose_rate(&[range(2.083e6, 61.44e6)]), 2.083e6, "a PlutoSDR");
        assert_eq!(choose_rate(&[range(0.25e6, 1.024e6), range(20e6, 20e6)]), 1.024e6);
        assert_eq!(choose_rate(&[range(0.1e6, 0.2e6)]), 2e6, "nothing usable: ask for 2 MS/s");
        assert_eq!(choose_rate(&[]), 2e6);
    }

    #[test]
    fn the_radios_kat_drives_itself_are_left_out() {
        let radio = |driver: &str| vec![("driver".to_string(), driver.to_string()), ("serial".into(), "1".into())];
        assert!(left_out(&radio("rtlsdr")) && left_out(&radio("hackrf")) && left_out(&radio("osmosdr")));
        assert!(!left_out(&radio("airspy")) && !left_out(&radio("sdrplay")) && !left_out(&radio("lime")));
        assert!(!left_out(&[("label".to_string(), "hackrf".to_string())]), "only the driver counts");
    }

    #[test]
    fn take_read_samples_handles_retune_drop_and_rejects_overflow() {
        let buf = [1.0f32, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]; // 4 complex samples
        let mut drop_left = 0;
        assert_eq!(take_read_samples(&buf, 3, &mut drop_left).unwrap(), &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);

        // Retune drop spanning more than one read.
        drop_left = 5;
        assert_eq!(take_read_samples(&buf, 3, &mut drop_left).unwrap(), &[] as &[f32]);
        assert_eq!(drop_left, 2);
        assert_eq!(take_read_samples(&buf, 4, &mut drop_left).unwrap(), &[5.0, 6.0, 7.0, 8.0]);
        assert_eq!(drop_left, 0);

        // Buggy C driver returning more samples than the buffer holds must fail cleanly, not panic.
        let err = take_read_samples(&buf, 5, &mut drop_left).unwrap_err();
        assert!(err.contains("5 samples") && err.contains("4-sample buffer"), "{err}");
    }
}
