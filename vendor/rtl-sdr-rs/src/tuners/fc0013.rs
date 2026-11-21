// Fitipower FC0013 tuner driver: a Rust port of librtlsdr's tuner_fc0013.c and of the FC0013
// glue in librtlsdr.c (rtl-sdr 2.0.3), made for KAT. The I2C transactions are the C driver's,
// checked against it by the tests in trace.rs.
//
// Copyright (C) 2012 Hans-Frieder Vogt <hfvogt@gmx.net>
// partially based on driver code from Fitipower
// Copyright (C) 2010 Fitipower Integrated Technology Inc
//
// modified for use in librtlsdr
// Copyright (C) 2012 Steve Markgraf <steve@steve-m.de>
//
// Rust port: Copyright (C) 2026 KAT contributors
//
// This program is free software; you can redistribute it and/or modify it under the terms of the
// GNU General Public License as published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
// the GNU General Public License for more details.
//
// SPDX-License-Identifier: GPL-2.0-or-later

use super::{TunerBus, TunerInfo, TunerOps, FC0013_TUNER_ID};
use crate::error::{Result, RtlsdrError};
use log::warn;

const I2C_ADDR: u8 = 0xc6;

pub const FC0013_TUNER_INFO: TunerInfo = TunerInfo {
    id: FC0013_TUNER_ID,
    name: "Fitipower FC0013",
    i2c_addr: I2C_ADDR,
    check_addr: 0x00,
    check_val: 0xa3,
};

/// librtlsdr's FC0013 gain steps (`rtlsdr_get_tuner_gains`), tenths of a dB.
const GAINS: [i32; 23] = [
    -99, -73, -65, -63, -60, -58, -54, 58, 61, 63, 65, 67, 68, 70, 71, 179, 181, 182, 184, 186,
    188, 191, 197,
];

/// `fc0013_lna_gains`: gain (tenths of a dB) and the LNA bits of register 0x14 for it.
const LNA_GAINS: [(i32, u8); 24] = [
    (-99, 0x02),
    (-73, 0x03),
    (-65, 0x05),
    (-63, 0x04),
    (-63, 0x00),
    (-60, 0x07),
    (-58, 0x01),
    (-54, 0x06),
    (58, 0x0f),
    (61, 0x0e),
    (63, 0x0d),
    (65, 0x0c),
    (67, 0x0b),
    (68, 0x0a),
    (70, 0x09),
    (71, 0x08),
    (179, 0x17),
    (181, 0x16),
    (182, 0x15),
    (184, 0x14),
    (186, 0x13),
    (188, 0x12),
    (191, 0x11),
    (197, 0x10),
];

#[derive(Debug)]
pub struct Fc0013 {
    /// The tuner's clock (`rtlsdr_get_tuner_clock`): the RTL2832U's 28.8 MHz, corrected.
    xtal: u32,
}

impl Fc0013 {
    pub fn new() -> Self {
        Fc0013 { xtal: 28_800_000 }
    }
}

impl Default for Fc0013 {
    fn default() -> Self {
        Self::new()
    }
}

fn writereg(bus: &dyn TunerBus, reg: u8, val: u8) -> Result<()> {
    bus.write(I2C_ADDR, &[reg, val]).map(|_| ())
}

fn readreg(bus: &dyn TunerBus, reg: u8) -> Result<u8> {
    let mut data = [reg];
    bus.write(I2C_ADDR, &data)?;
    bus.read(I2C_ADDR, &mut data)?;
    Ok(data[0])
}

/// `fc0013_init`: the register defaults, written in order; the first failed write ends it.
fn init(bus: &dyn TunerBus) -> Result<()> {
    let mut reg: [u8; 22] = [
        0x00, // reg. 0x00: dummy
        0x09, // reg. 0x01
        0x16, // reg. 0x02
        0x00, // reg. 0x03
        0x00, // reg. 0x04
        0x17, // reg. 0x05
        0x02, // reg. 0x06: LPF bandwidth
        0x0a, // reg. 0x07: CHECK
        0xff, // reg. 0x08: AGC Clock divide by 256, AGC gain 1/256, Loop Bw 1/8
        0x6e, // reg. 0x09: Disable LoopThrough, Enable LoopThrough: 0x6f
        0xb8, // reg. 0x0a: Disable LO Test Buffer
        0x82, // reg. 0x0b: CHECK
        0xfc, // reg. 0x0c: depending on AGC Up-Down mode, may need 0xf8
        0x01, // reg. 0x0d: AGC Not Forcing & LNA Forcing, may need 0x02
        0x00, // reg. 0x0e
        0x00, // reg. 0x0f
        0x00, // reg. 0x10
        0x00, // reg. 0x11
        0x00, // reg. 0x12
        0x00, // reg. 0x13
        0x50, // reg. 0x14: DVB-t High Gain, UHF. Middle Gain: 0x48, Low Gain: 0x40
        0x01, // reg. 0x15
    ];
    // 28.8 MHz crystal
    reg[0x07] |= 0x20;
    // dual master
    reg[0x0c] |= 0x02;
    for (i, &val) in reg.iter().enumerate().skip(1) {
        writereg(bus, i as u8, val)?;
    }
    Ok(())
}

/// `fc0013_set_vhf_track`
fn set_vhf_track(bus: &dyn TunerBus, freq: u32) -> Result<()> {
    let tmp = readreg(bus, 0x1d)? & 0xe3;
    let track = if freq <= 177_500_000 {
        0x1c // VHF Track: 7
    } else if freq <= 184_500_000 {
        0x18 // VHF Track: 6
    } else if freq <= 191_500_000 {
        0x14 // VHF Track: 5
    } else if freq <= 198_500_000 {
        0x10 // VHF Track: 4
    } else if freq <= 205_500_000 {
        0x0c // VHF Track: 3
    } else if freq <= 219_500_000 {
        0x08 // VHF Track: 2
    } else if freq < 300_000_000 {
        0x04 // VHF Track: 1
    } else {
        0x1c // UHF and GPS
    };
    writereg(bus, 0x1d, tmp | track)
}

/// `fc0013_set_params`: tune to `freq` Hz with the given channel bandwidth.
fn set_params(bus: &dyn TunerBus, xtal: u32, freq: u32, bandwidth: u32) -> Result<()> {
    let mut reg = [0u8; 7];
    let xtal_freq_div_2 = xtal / 2;

    set_vhf_track(bus, freq)?;

    if freq < 300_000_000 {
        // enable VHF filter
        let tmp = readreg(bus, 0x07)?;
        writereg(bus, 0x07, tmp | 0x10)?;
        // disable UHF & disable GPS
        let tmp = readreg(bus, 0x14)?;
        writereg(bus, 0x14, tmp & 0x1f)?;
    } else {
        // disable VHF filter
        let tmp = readreg(bus, 0x07)?;
        writereg(bus, 0x07, tmp & 0xef)?;
        // enable UHF & disable GPS
        let tmp = readreg(bus, 0x14)?;
        writereg(bus, 0x14, (tmp & 0x1f) | 0x40)?;
    }

    // select frequency divider and the frequency of VCO
    let multi: u8 = if freq < 37_084_000 {
        // freq * 96 < 3560000000
        reg[5] = 0x82;
        reg[6] = 0x00;
        96
    } else if freq < 55_625_000 {
        reg[5] = 0x02;
        reg[6] = 0x02;
        64
    } else if freq < 74_167_000 {
        reg[5] = 0x42;
        reg[6] = 0x00;
        48
    } else if freq < 111_250_000 {
        reg[5] = 0x82;
        reg[6] = 0x02;
        32
    } else if freq < 148_334_000 {
        reg[5] = 0x22;
        reg[6] = 0x00;
        24
    } else if freq < 222_500_000 {
        reg[5] = 0x42;
        reg[6] = 0x02;
        16
    } else if freq < 296_667_000 {
        reg[5] = 0x12;
        reg[6] = 0x00;
        12
    } else if freq < 445_000_000 {
        reg[5] = 0x22;
        reg[6] = 0x02;
        8
    } else if freq < 593_334_000 {
        reg[5] = 0x0a;
        reg[6] = 0x00;
        6
    } else if freq < 950_000_000 {
        // freq * 4 < 3800000000
        reg[5] = 0x12;
        reg[6] = 0x02;
        4
    } else {
        reg[5] = 0x0a;
        reg[6] = 0x02;
        2
    };

    // (the C multiplies in 32 bits, then widens)
    let f_vco = u64::from(freq.wrapping_mul(u32::from(multi)));
    let vco_select = f_vco >= 3_060_000_000;
    if vco_select {
        reg[6] |= 0x08;
    }

    // From divided value (XDIV) determined the FA and FP value
    let mut xdiv = (f_vco / u64::from(xtal_freq_div_2)) as u16;
    if f_vco.wrapping_sub(u64::from(u32::from(xdiv).wrapping_mul(xtal_freq_div_2)))
        >= u64::from(xtal_freq_div_2 / 2)
    {
        xdiv = xdiv.wrapping_add(1);
    }

    let mut pm = (xdiv / 8) as u8;
    let mut am = (i32::from(xdiv) - 8 * i32::from(pm)) as u8;

    if am < 2 {
        am = am.wrapping_add(8);
        pm = pm.wrapping_sub(1);
    }

    if pm > 31 {
        reg[1] = (i32::from(am) + 8 * (i32::from(pm) - 31)) as u8;
        reg[2] = 31;
    } else {
        reg[1] = am;
        reg[2] = pm;
    }

    if reg[1] > 15 || reg[2] < 0x0b {
        warn!("[FC0013] no valid PLL combination found for {freq} Hz!");
        return Err(RtlsdrError::RtlsdrErr(format!(
            "FC0013: no valid PLL combination for {freq} Hz"
        )));
    }

    // fix clock out
    reg[6] |= 0x20;

    // From VCO frequency determines the XIN (fractional part of Delta Sigma PLL) and divided
    // value (XDIV)
    let mut xin = ((f_vco - (f_vco / u64::from(xtal_freq_div_2)) * u64::from(xtal_freq_div_2))
        / 1000) as u16;
    xin = ((u32::from(xin) << 15) / (xtal_freq_div_2 / 1000)) as u16;
    if xin >= 16384 {
        xin = xin.wrapping_add(32768);
    }

    reg[3] = (xin >> 8) as u8;
    reg[4] = (xin & 0xff) as u8;

    reg[6] &= 0x3f; // bits 6 and 7 describe the bandwidth
    match bandwidth {
        6_000_000 => reg[6] |= 0x80,
        7_000_000 => reg[6] |= 0x40,
        _ => {} // 8 MHz
    }

    // modified for Realtek demod
    reg[5] |= 0x07;

    for (i, &val) in reg.iter().enumerate().skip(1) {
        writereg(bus, i as u8, val)?;
    }

    let tmp = readreg(bus, 0x11)?;
    if multi == 64 {
        writereg(bus, 0x11, tmp | 0x04)?;
    } else {
        writereg(bus, 0x11, tmp & 0xfb)?;
    }

    // VCO Calibration
    writereg(bus, 0x0e, 0x80)?;
    writereg(bus, 0x0e, 0x00)?;

    // VCO Re-Calibration if needed
    writereg(bus, 0x0e, 0x00)?;

    let tmp = readreg(bus, 0x0e)? & 0x3f;

    // vco selection
    if vco_select {
        if tmp > 0x3c {
            reg[6] &= !0x08;
            writereg(bus, 0x06, reg[6])?;
            writereg(bus, 0x0e, 0x80)?;
            writereg(bus, 0x0e, 0x00)?;
        }
    } else if tmp < 0x02 {
        reg[6] |= 0x08;
        writereg(bus, 0x06, reg[6])?;
        writereg(bus, 0x0e, 0x80)?;
        writereg(bus, 0x0e, 0x00)?;
    }
    Ok(())
}

/// `fc0013_set_gain_mode`: errors do not stop it, as in the C.
fn set_gain_mode(bus: &dyn TunerBus, manual: bool) -> bool {
    let mut ok = true;
    let mut tmp = readreg(bus, 0x0d).unwrap_or_else(|_| {
        ok = false;
        0
    });
    if manual {
        tmp |= 1 << 3;
    } else {
        tmp &= !(1 << 3);
    }
    ok &= writereg(bus, 0x0d, tmp).is_ok();
    // set a fixed IF-gain for now
    ok &= writereg(bus, 0x13, 0x0a).is_ok();
    ok
}

/// `fc0013_set_lna_gain`: the first step at or above `gain`, else the last.
fn set_lna_gain(bus: &dyn TunerBus, gain: i32) -> bool {
    let mut ok = true;
    let mut tmp = readreg(bus, 0x14).unwrap_or_else(|_| {
        ok = false;
        0
    });
    // mask bits off
    tmp &= 0xe0;
    for (i, &(step, bits)) in LNA_GAINS.iter().enumerate() {
        if step >= gain || i + 1 == LNA_GAINS.len() {
            tmp |= bits;
            break;
        }
    }
    ok &= writereg(bus, 0x14, tmp).is_ok();
    ok
}

fn status(ok: bool, what: &str) -> Result<()> {
    if ok {
        Ok(())
    } else {
        Err(RtlsdrError::RtlsdrErr(format!("FC0013: {what} failed")))
    }
}

impl TunerOps for Fc0013 {
    fn info(&self) -> TunerInfo {
        FC0013_TUNER_INFO
    }
    fn gains(&self) -> &'static [i32] {
        &GAINS
    }
    fn xtal(&self) -> u32 {
        self.xtal
    }
    fn set_xtal(&mut self, xtal: u32) {
        self.xtal = xtal;
    }
    fn init(&mut self, bus: &dyn TunerBus) -> Result<()> {
        init(bus)
    }
    fn exit(&mut self, _bus: &dyn TunerBus) -> Result<()> {
        Ok(())
    }
    fn set_freq(&mut self, bus: &dyn TunerBus, freq: u32) -> Result<()> {
        set_params(bus, self.xtal, freq, 6_000_000)
    }
    fn set_bw(&mut self, _bus: &dyn TunerBus, _bw: u32) -> Result<()> {
        Ok(())
    }
    fn set_gain(&mut self, bus: &dyn TunerBus, gain: i32) -> Result<()> {
        status(set_lna_gain(bus, gain), "setting the gain")
    }
    fn set_gain_mode(&mut self, bus: &dyn TunerBus, manual: bool) -> Result<()> {
        status(set_gain_mode(bus, manual), "setting the gain mode")
    }
}
