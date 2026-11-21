// Fitipower FC0012 tuner driver: a Rust port of librtlsdr's tuner_fc0012.c and of the FC0012
// glue in librtlsdr.c (rtl-sdr 2.0.3), made for KAT. The I2C transactions are the C driver's,
// checked against it by the tests in trace.rs.
//
// Copyright (C) 2012 Hans-Frieder Vogt <hfvogt@gmx.net>
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

use super::{TunerBus, TunerInfo, TunerOps, FC0012_TUNER_ID};
use crate::error::{Result, RtlsdrError};
use log::warn;

const I2C_ADDR: u8 = 0xc6;

pub const FC0012_TUNER_INFO: TunerInfo = TunerInfo {
    id: FC0012_TUNER_ID,
    name: "Fitipower FC0012",
    i2c_addr: I2C_ADDR,
    check_addr: 0x00,
    check_val: 0xa1,
};

/// The RTL2832U GPIO pin that selects the FC0012's V-band or U-band filter.
pub const BAND_FILTER_GPIO: u8 = 6;

/// librtlsdr's FC0012 gain steps (`rtlsdr_get_tuner_gains`), tenths of a dB.
const GAINS: [i32; 5] = [-99, -40, 71, 179, 192];

#[derive(Debug)]
pub struct Fc0012 {
    /// The tuner's clock (`rtlsdr_get_tuner_clock`): the RTL2832U's 28.8 MHz, corrected.
    xtal: u32,
}

impl Fc0012 {
    pub fn new() -> Self {
        Fc0012 { xtal: 28_800_000 }
    }
}

impl Default for Fc0012 {
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

// Incomplete list of register settings (from the C driver):
//
// Name            Reg   Bits  Desc
// CHIP_ID         0x00  0-7   Chip ID (constant 0xA1)
// RF_A            0x01  0-3   Number of count-to-9 cycles in RF divider (suggested: 2..9)
// RF_M            0x02  0-7   Total number of cycles (to-8 and to-9) in RF divider
// RF_K_HIGH       0x03  0-6   Bits 8..14 of fractional divider
// RF_K_LOW        0x04  0-7   Bits 0..7 of fractional RF divider
// RF_OUTDIV_A     0x05  3-7   Power of two required?
// LNA_POWER_DOWN  0x06  0     Set to 1 to switch off low noise amp
// RF_OUTDIV_B     0x06  1     Set to select 3 instead of 2 for the RF output divider
// VCO_SPEED       0x06  3     Select tuning range of VCO: 0 = Low range (ca. 1.1 - 1.5GHz),
//                             1 = High range (ca. 1.4 - 1.8GHz)
// BANDWIDTH       0x06  6-7   Set bandwidth. 6MHz = 0x80, 7MHz=0x40, 8MHz=0x00
// XTAL_SPEED      0x07  5     Set to 1 for 28.8MHz Crystal input or 0 for 36MHz
// <agc params>    0x08  0-7
// EN_CAL_RSSI     0x09  4     Enable calibrate RSSI (Receive Signal Strength Indicator)
// LNA_FORCE       0x0d  0
// AGC_FORCE       0x0d  ?
// LNA_GAIN        0x13  3-4   Low noise amp gain
// LNA_COMPS       0x15  3     ?
// VCO_CALIB       0x0e  7     Set high then low to calibrate VCO (fast lock?)
// VCO_VOLTAGE     0x0e  0-6   Read Control voltage of VCO (big value -> low freq)

/// `fc0012_init`: the register defaults, written in order; the first failed write ends it.
fn init(bus: &dyn TunerBus) -> Result<()> {
    let mut reg: [u8; 22] = [
        0x00, // dummy reg. 0
        0x05, // reg. 0x01
        0x10, // reg. 0x02
        0x00, // reg. 0x03
        0x00, // reg. 0x04
        0x0f, // reg. 0x05: may also be 0x0a
        0x00, // reg. 0x06: divider 2, VCO slow
        0x00, // reg. 0x07: may also be 0x0f
        0xff, // reg. 0x08: AGC Clock divide by 256, AGC gain 1/256, Loop Bw 1/8
        0x6e, // reg. 0x09: Disable LoopThrough, Enable LoopThrough: 0x6f
        0xb8, // reg. 0x0a: Disable LO Test Buffer
        0x82, // reg. 0x0b: Output Clock is same as clock frequency, may also be 0x83
        0xfc, // reg. 0x0c: depending on AGC Up-Down mode, may need 0xf8
        0x02, // reg. 0x0d: AGC Not Forcing & LNA Forcing, 0x02 for DVB-T
        0x00, // reg. 0x0e
        0x00, // reg. 0x0f
        0x00, // reg. 0x10: may also be 0x0d
        0x00, // reg. 0x11
        0x1f, // reg. 0x12: Set to maximum gain
        0x00, // reg. 0x13: Set to Low Gain: 0x00, Low Gain: 0x00, High Gain: 0x10, enable IX2: 0x80
        0x00, // reg. 0x14
        0x04, // reg. 0x15: Enable LNA COMPS
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

/// `fc0012_set_params`: tune to `freq` Hz with the given channel bandwidth.
fn set_params(bus: &dyn TunerBus, xtal: u32, freq: u32, bandwidth: u32) -> Result<()> {
    let mut reg = [0u8; 7];
    let xtal_freq_div_2 = xtal / 2;

    // select frequency divider and the frequency of VCO
    let multi: u8 = if freq < 37_084_000 {
        // freq * 96 < 3560000000
        reg[5] = 0x82;
        reg[6] = 0x00;
        96
    } else if freq < 55_625_000 {
        reg[5] = 0x82;
        reg[6] = 0x02;
        64
    } else if freq < 74_167_000 {
        reg[5] = 0x42;
        reg[6] = 0x00;
        48
    } else if freq < 111_250_000 {
        reg[5] = 0x42;
        reg[6] = 0x02;
        32
    } else if freq < 148_334_000 {
        reg[5] = 0x22;
        reg[6] = 0x00;
        24
    } else if freq < 222_500_000 {
        reg[5] = 0x22;
        reg[6] = 0x02;
        16
    } else if freq < 296_667_000 {
        reg[5] = 0x12;
        reg[6] = 0x00;
        12
    } else if freq < 445_000_000 {
        reg[5] = 0x12;
        reg[6] = 0x02;
        8
    } else if freq < 593_334_000 {
        reg[5] = 0x0a;
        reg[6] = 0x00;
        6
    } else {
        reg[5] = 0x0a;
        reg[6] = 0x02;
        4
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
        warn!("[FC0012] no valid PLL combination found for {freq} Hz!");
        return Err(RtlsdrError::RtlsdrErr(format!(
            "FC0012: no valid PLL combination for {freq} Hz"
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

    reg[3] = (xin >> 8) as u8; // xin with 9 bit resolution
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

/// `fc0012_set_gain`: a failed read leaves the other bits 0, as in the C.
fn set_gain(bus: &dyn TunerBus, gain: i32) -> Result<()> {
    let mut tmp = readreg(bus, 0x13).unwrap_or(0);
    // mask bits off
    tmp &= 0xe0;
    tmp |= if gain < -40 {
        0x02 // -9.9 dB
    } else if gain < 71 {
        0x00 // -4.0 dB
    } else if gain < 179 {
        0x08 // 7.1 dB
    } else if gain < 192 {
        0x17 // 17.9 dB
    } else {
        0x10 // 19.2 dB
    };
    writereg(bus, 0x13, tmp)
}

impl TunerOps for Fc0012 {
    fn info(&self) -> TunerInfo {
        FC0012_TUNER_INFO
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
        // select V-band/U-band filter (librtlsdr ignores the result)
        let _ = bus.set_gpio_bit(BAND_FILTER_GPIO, freq > 300_000_000);
        set_params(bus, self.xtal, freq, 6_000_000)
    }
    fn set_bw(&mut self, _bus: &dyn TunerBus, _bw: u32) -> Result<()> {
        Ok(())
    }
    fn set_gain(&mut self, bus: &dyn TunerBus, gain: i32) -> Result<()> {
        set_gain(bus, gain)
    }
    fn set_gain_mode(&mut self, _bus: &dyn TunerBus, _manual: bool) -> Result<()> {
        Ok(())
    }
}
