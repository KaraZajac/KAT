//! The pipe between KAT and its SoapySDR helper (`kat --soapy-helper`, [super]).
//!
//! `find`: the helper prints one JSON line per radio ([Listing]) and exits.
//!
//! `rx <Hz> <dB|auto> <key=value>…`: the helper opens the radio the arguments name, writes frames
//! to its stdout — a tag byte, the payload's length (u32, little-endian), the payload — and takes
//! [Command]s on its stdin, one per line. First [READY] (or [ERROR]), then [IQ] frames; an IQ frame
//! with no samples is a heartbeat, so that KAT hears from it at least every 100 ms.

use serde::{Deserialize, Serialize};
use std::io::{self, Read, Write};

/// A line of `kat --soapy-helper find`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Listing {
    /// A radio: the arguments that open it (SoapySDR's: driver, serial, label …).
    Radio(Vec<(String, String)>),
    /// SoapySDR is not installed (the libraries tried).
    Missing(String),
    /// SoapySDR is installed but could not list its radios.
    Error(String),
}

/// The radio streams: [Ready] as JSON.
pub const READY: u8 = b'R';
/// Samples: the frequency they were received for (u32, little-endian, the `tune` value), then
/// complex f32 samples (I, Q, little-endian).
pub const IQ: u8 = b'I';
/// A retune failed: the frequency asked for (u32, little-endian); the radio stays where it was.
pub const TUNE_FAILED: u8 = b'F';
/// The helper cannot go on: why, in words. It exits after.
pub const ERROR: u8 = b'E';

/// Longest frame KAT reads (a frame of 2^20 samples is 8 MiB).
pub const MAX_FRAME: usize = 16 << 20;

/// What the helper says once the radio streams.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ready {
    /// Sample rate, S/s.
    pub rate: u32,
    /// SoapySDR's driver and hardware keys ("airspy", "AirSpy One").
    pub driver: String,
    pub hardware: String,
    /// The gain range the radio reports, dB.
    pub gain_min: f64,
    pub gain_max: f64,
}

/// Write one frame (the payload in parts) and flush it.
pub fn write_frame(w: &mut impl Write, tag: u8, payload: &[&[u8]]) -> io::Result<()> {
    let len: usize = payload.iter().map(|p| p.len()).sum();
    let len = u32::try_from(len).map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "frame too long"))?;
    w.write_all(&[tag])?;
    w.write_all(&len.to_le_bytes())?;
    for part in payload {
        w.write_all(part)?;
    }
    w.flush()
}

/// The next frame, or None at the end of the stream.
pub fn read_frame(r: &mut impl Read) -> io::Result<Option<(u8, Vec<u8>)>> {
    let mut head = [0u8; 5];
    match r.read_exact(&mut head) {
        Ok(()) => {}
        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e),
    }
    let len = u32::from_le_bytes([head[1], head[2], head[3], head[4]]) as usize;
    if len > MAX_FRAME {
        return Err(io::Error::new(io::ErrorKind::InvalidData, format!("frame of {len} bytes")));
    }
    let mut payload = vec![0u8; len];
    r.read_exact(&mut payload)?;
    Ok(Some((head[0], payload)))
}

/// A command to the receiving helper, one per line on its stdin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    /// Receive for this frequency (the helper tunes the radio below it, as KAT tunes every SDR).
    Tune(u32),
    /// A manual gain in dB, or None for the radio's automatic gain.
    Gain(Option<f64>),
    /// Stop, close the radio and exit.
    Quit,
}

impl Command {
    pub fn line(&self) -> String {
        match self {
            Command::Tune(hz) => format!("tune {hz}\n"),
            Command::Gain(Some(db)) => format!("gain {db}\n"),
            Command::Gain(None) => "gain auto\n".to_string(),
            Command::Quit => "quit\n".to_string(),
        }
    }

    pub fn parse(line: &str) -> Option<Self> {
        let mut words = line.split_whitespace();
        let cmd = match (words.next()?, words.next()) {
            ("tune", Some(hz)) => Command::Tune(hz.parse().ok()?),
            ("gain", Some(g)) => Command::Gain(parse_gain(g)?),
            ("quit", None) => Command::Quit,
            _ => return None,
        };
        words.next().is_none().then_some(cmd)
    }
}

/// `auto`, or a gain in dB.
pub fn parse_gain(word: &str) -> Option<Option<f64>> {
    if word.eq_ignore_ascii_case("auto") {
        return Some(None);
    }
    word.parse::<f64>().ok().filter(|g| g.is_finite()).map(Some)
}

/// The `rx` arguments for the helper's command line.
pub fn rx_args(target: u32, gain: Option<f64>, device: &[(String, String)]) -> Vec<String> {
    let mut args = vec!["rx".to_string(), target.to_string(), gain.map_or("auto".to_string(), |g| g.to_string())];
    args.extend(device.iter().map(|(k, v)| format!("{k}={v}")));
    args
}

/// What `rx_args` wrote: the frequency, the gain and the radio's arguments.
pub fn parse_rx_args(args: &[String]) -> Option<(u32, Option<f64>, Vec<(String, String)>)> {
    let target = args.first()?.parse().ok()?;
    let gain = parse_gain(args.get(1)?)?;
    let device = args[2..]
        .iter()
        .map(|kv| kv.split_once('=').map(|(k, v)| (k.to_string(), v.to_string())))
        .collect::<Option<Vec<_>>>()?;
    Some((target, gain, device))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frames_go_through() {
        let mut pipe = Vec::new();
        write_frame(&mut pipe, IQ, &[&433_920_000u32.to_le_bytes(), &[1, 2, 3, 4, 5, 6, 7, 8]]).unwrap();
        write_frame(&mut pipe, ERROR, &[b"no radio"]).unwrap();
        write_frame(&mut pipe, IQ, &[&433_920_000u32.to_le_bytes()]).unwrap();
        let mut r = pipe.as_slice();
        let (tag, p) = read_frame(&mut r).unwrap().unwrap();
        assert_eq!((tag, p.len(), &p[..4]), (IQ, 12, &433_920_000u32.to_le_bytes()[..]));
        assert_eq!(read_frame(&mut r).unwrap(), Some((ERROR, b"no radio".to_vec())));
        assert_eq!(read_frame(&mut r).unwrap().map(|f| f.1.len()), Some(4), "a heartbeat");
        assert_eq!(read_frame(&mut r).unwrap(), None);
        // a helper that died mid-frame
        let mut cut = &pipe[..7];
        assert!(read_frame(&mut cut).is_err());
        let mut huge = Vec::new();
        huge.push(IQ);
        huge.extend_from_slice(&u32::MAX.to_le_bytes());
        assert!(read_frame(&mut huge.as_slice()).is_err(), "a garbled length is refused, not allocated");
    }

    #[test]
    fn commands_and_arguments_go_through() {
        for c in [Command::Tune(868_350_000), Command::Gain(Some(20.5)), Command::Gain(None), Command::Quit] {
            assert_eq!(Command::parse(&c.line()), Some(c));
        }
        assert_eq!(Command::parse("tune"), None);
        assert_eq!(Command::parse("gain loud"), None);
        assert_eq!(Command::parse("quit now"), None);
        let device = vec![
            ("driver".to_string(), "airspy".to_string()),
            ("label".to_string(), "AirSpy One [a=b]".to_string()),
        ];
        let args = rx_args(315_000_000, Some(12.0), &device);
        assert_eq!(parse_rx_args(&args[1..]), Some((315_000_000, Some(12.0), device.clone())));
        assert_eq!(parse_rx_args(&rx_args(315_000_000, None, &device)[1..]).map(|a| a.1), Some(None));
        assert_eq!(parse_rx_args(&["315000000".to_string(), "auto".to_string(), "novalue".to_string()]), None);
    }

    #[test]
    fn listings_are_json_lines() {
        let radio = Listing::Radio(vec![("driver".into(), "lime".into()), ("serial".into(), "0009".into())]);
        let line = serde_json::to_string(&radio).unwrap();
        assert_eq!(line, r#"{"radio":[["driver","lime"],["serial","0009"]]}"#);
        assert_eq!(serde_json::from_str::<Listing>(&line).unwrap(), radio);
        let missing: Listing = serde_json::from_str(r#"{"missing":"libSoapySDR.so.0.8"}"#).unwrap();
        assert_eq!(missing, Listing::Missing("libSoapySDR.so.0.8".into()));
    }
}
