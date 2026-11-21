// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! The ported drivers against librtlsdr's. Each scenario in testdata/ was run through the C
//! drivers by tools/tuner_trace.c, against the same model of the tuner's registers as [MockBus]
//! here, and its output kept next to it (`<tuner>.trace`); the Rust drivers must make the same
//! I2C transactions, byte for byte, and fail where the C fails. Also: which tuner the probe finds.

use super::e4k::E4k;
use super::fc0012::Fc0012;
use super::fc0013::Fc0013;
use super::fc2580::Fc2580;
use super::{TunerBus, TunerOps};
use crate::error::{Result, RtlsdrError};
use crate::rtlsdr::{probe_tuner, TunerKind};
use std::cell::RefCell;
use std::collections::HashSet;
use std::fmt::Write;

/// A model of the devices on the RTL2832U's I2C bus: 256 registers per address, the register
/// pointer the last write set, and read overrides (`(stored & and) | or`) for status registers.
/// It writes each transaction as tools/tuner_trace.c does: `W <addr>: <bytes>`, `R <addr>:
/// <bytes>`, `G <pin> <0|1>`; also `O <pin>` (GPIO made an output) and `W <addr>: <bytes> nak`
/// for a device that does not answer.
struct MockBus {
    regs: RefCell<Vec<[u8; 256]>>,
    ptr: RefCell<[u8; 256]>,
    ovrs: Vec<(u8, u8, u8, u8)>,
    /// Only these addresses answer; None: all do.
    present: Option<HashSet<u8>>,
    trace: RefCell<String>,
}

impl MockBus {
    /// Every address answers (the C model).
    fn new() -> Self {
        MockBus {
            regs: RefCell::new(vec![[0; 256]; 256]),
            ptr: RefCell::new([0; 256]),
            ovrs: Vec::new(),
            present: None,
            trace: RefCell::new(String::new()),
        }
    }

    /// Only the devices given as (address, register, value) answer, with those registers set.
    fn with_devices(devices: &[(u8, u8, u8)]) -> Self {
        let bus = MockBus {
            present: Some(devices.iter().map(|d| d.0).collect()),
            ..Self::new()
        };
        for &(addr, reg, val) in devices {
            bus.set_reg(addr, reg, val);
        }
        bus
    }

    fn set_reg(&self, addr: u8, reg: u8, val: u8) {
        self.regs.borrow_mut()[usize::from(addr)][usize::from(reg)] = val;
    }

    fn set_ovr(&mut self, addr: u8, reg: u8, and: u8, or: u8) {
        self.ovrs.retain(|o| (o.0, o.1) != (addr, reg));
        self.ovrs.push((addr, reg, and, or));
    }

    fn answers(&self, addr: u8) -> bool {
        self.present.as_ref().map_or(true, |p| p.contains(&addr))
    }

    fn take_trace(&self) -> String {
        std::mem::take(&mut *self.trace.borrow_mut())
    }

    fn line(&self, line: String) {
        let mut t = self.trace.borrow_mut();
        t.push_str(&line);
        t.push('\n');
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!(" {b:02x}")).collect()
}

impl TunerBus for MockBus {
    fn write(&self, addr: u8, data: &[u8]) -> Result<usize> {
        if !self.answers(addr) {
            self.line(format!("W {addr:02x}:{} nak", hex(data)));
            return Err(RtlsdrError::Usb(rusb::Error::Pipe));
        }
        self.line(format!("W {addr:02x}:{}", hex(data)));
        if let Some((&first, rest)) = data.split_first() {
            self.ptr.borrow_mut()[usize::from(addr)] = first;
            let mut regs = self.regs.borrow_mut();
            for (i, &val) in rest.iter().enumerate() {
                regs[usize::from(addr)][usize::from(first.wrapping_add(i as u8))] = val;
            }
        }
        Ok(data.len())
    }

    fn read(&self, addr: u8, buf: &mut [u8]) -> Result<usize> {
        if !self.answers(addr) {
            self.line(format!("R {addr:02x}: nak"));
            return Err(RtlsdrError::Usb(rusb::Error::Pipe));
        }
        let start = self.ptr.borrow()[usize::from(addr)];
        for (i, b) in buf.iter_mut().enumerate() {
            let reg = start.wrapping_add(i as u8);
            let mut v = self.regs.borrow()[usize::from(addr)][usize::from(reg)];
            for &(a, r, and, or) in &self.ovrs {
                if (a, r) == (addr, reg) {
                    v = (v & and) | or;
                }
            }
            *b = v;
        }
        self.line(format!("R {addr:02x}:{}", hex(buf)));
        Ok(buf.len())
    }

    fn set_gpio_bit(&self, gpio: u8, on: bool) -> Result<()> {
        self.line(format!("G {gpio} {}", u8::from(on)));
        Ok(())
    }

    fn set_gpio_output(&self, gpio: u8) -> Result<()> {
        self.line(format!("O {gpio}"));
        Ok(())
    }
}

fn tuner_named(name: &str) -> Box<dyn TunerOps> {
    match name {
        "e4000" => Box::new(E4k::new()),
        "fc0012" => Box::new(Fc0012::new()),
        "fc0013" => Box::new(Fc0013::new()),
        "fc2580" => Box::new(Fc2580::new()),
        _ => panic!("unknown tuner {name}"),
    }
}

/// Run a scenario (the language of tools/tuner_trace.c) and return its trace.
fn run(scenario: &str) -> String {
    let mut out = String::new();
    let mut bus = MockBus::new();
    let mut tuner: Option<Box<dyn TunerOps>> = None;
    for raw in scenario.lines() {
        let line = raw.split('#').next().unwrap_or("").trim_end_matches(' ');
        let words: Vec<&str> = line.split_whitespace().collect();
        let Some((&op, args)) = words.split_first() else { continue };
        writeln!(out, "> {line}").unwrap();
        let hex = |i: usize| u8::from_str_radix(args[i], 16).unwrap();
        let num = || args.first().map_or(0, |a| a.parse::<i64>().unwrap());
        match op {
            "tuner" => {
                bus = MockBus::new();
                tuner = Some(tuner_named(args[0]));
            }
            "xtal" => tuner.as_mut().unwrap().set_xtal(num() as u32),
            "reg" => bus.set_reg(hex(0), hex(1), hex(2)),
            "ovr" => bus.set_ovr(hex(0), hex(1), hex(2), hex(3)),
            _ => {
                let t = tuner.as_mut().expect("a tuner line first");
                let r = match op {
                    "init" => t.init(&bus),
                    "exit" => t.exit(&bus),
                    "freq" => t.set_freq(&bus, num() as u32),
                    "bw" => t.set_bw(&bus, num() as u32),
                    "gain" => t.set_gain(&bus, num() as i32),
                    "gainmode" => t.set_gain_mode(&bus, num() != 0),
                    _ => panic!("unknown operation {op}"),
                };
                out.push_str(&bus.take_trace());
                writeln!(out, "= {}", if r.is_ok() { "ok" } else { "err" }).unwrap();
            }
        }
    }
    out
}

/// The Rust driver's trace for `scenario` is the C driver's, line for line.
fn same_as_librtlsdr(scenario: &str, librtlsdr: &str) {
    let ours = run(scenario);
    let (mut ours_lines, mut c_lines) = (ours.lines(), librtlsdr.lines());
    let mut context = String::new();
    for n in 1.. {
        match (ours_lines.next(), c_lines.next()) {
            (None, None) => break,
            (o, c) => {
                if let Some(l) = o.filter(|l| l.starts_with('>')) {
                    context = l.to_string();
                }
                assert_eq!(o, c, "line {n} (after `{context}`)");
            }
        }
    }
}

#[test]
fn e4000_makes_librtlsdrs_transactions() {
    same_as_librtlsdr(include_str!("testdata/e4000.scn"), include_str!("testdata/e4000.trace"));
}

#[test]
fn fc0012_makes_librtlsdrs_transactions() {
    same_as_librtlsdr(include_str!("testdata/fc0012.scn"), include_str!("testdata/fc0012.trace"));
}

#[test]
fn fc0013_makes_librtlsdrs_transactions() {
    same_as_librtlsdr(include_str!("testdata/fc0013.scn"), include_str!("testdata/fc0013.trace"));
}

#[test]
fn fc2580_makes_librtlsdrs_transactions() {
    same_as_librtlsdr(include_str!("testdata/fc2580.scn"), include_str!("testdata/fc2580.trace"));
}

/// The probe's I2C reads and GPIO changes for a dongle with these devices on its bus.
fn probe(devices: &[(u8, u8, u8)]) -> (Option<TunerKind>, String) {
    let bus = MockBus::with_devices(devices);
    let kind = probe_tuner(&bus);
    (kind, bus.take_trace())
}

const NOBODY_UPSTREAM: &str = "W 34: 00 nak\nW 74: 00 nak\n";

/// The R820T and R828D are found as before, with the same two reads at most.
#[test]
fn the_probe_finds_the_r82xx_first() {
    assert_eq!(probe(&[(0x34, 0x00, 0x69)]), (Some(TunerKind::R820T), "W 34: 00\nR 34: 69\n".into()));
    let (kind, trace) = probe(&[(0x74, 0x00, 0x69)]);
    assert_eq!((kind, trace.as_str()), (Some(TunerKind::R828D), "W 34: 00 nak\nW 74: 00\nR 74: 69\n"));
}

/// Then librtlsdr's order: E4000, FC0013, a pulse on GPIO 4 (it resets the tuner), FC2580
/// (compared on 7 bits), FC0012 (whose band filter pin, GPIO 6, becomes an output).
#[test]
fn the_probe_finds_the_ported_tuners_in_librtlsdrs_order() {
    let e4k = format!("{NOBODY_UPSTREAM}W c8: 02\nR c8: 40\n");
    assert_eq!(probe(&[(0xc8, 0x02, 0x40)]), (Some(TunerKind::E4000), e4k));

    let fc0013 = format!("{NOBODY_UPSTREAM}W c8: 02 nak\nW c6: 00\nR c6: a3\n");
    assert_eq!(probe(&[(0xc6, 0x00, 0xa3)]), (Some(TunerKind::Fc0013), fc0013));

    let reset = "O 4\nG 4 1\nG 4 0\n";
    let fc2580 = format!("{NOBODY_UPSTREAM}W c8: 02 nak\nW c6: 00 nak\n{reset}W ac: 01\nR ac: d6\n");
    assert_eq!(probe(&[(0xac, 0x01, 0xd6)]), (Some(TunerKind::Fc2580), fc2580));

    // The FC0012 answers at the FC0013's address with its own id, 0xa1.
    let fc0012 = format!(
        "{NOBODY_UPSTREAM}W c8: 02 nak\nW c6: 00\nR c6: a1\n{reset}W ac: 01 nak\nW c6: 00\nR c6: a1\nO 6\n"
    );
    assert_eq!(probe(&[(0xc6, 0x00, 0xa1)]), (Some(TunerKind::Fc0012), fc0012));

    let none = format!("{NOBODY_UPSTREAM}W c8: 02 nak\nW c6: 00 nak\n{reset}W ac: 01 nak\nW c6: 00 nak\n");
    assert_eq!(probe(&[]), (None, none));
}
