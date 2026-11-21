// FCI FC2580 tuner driver: a Rust port of librtlsdr's tuner_fc2580.c ("taken from the kernel
// driver that can be found on http://linux.terratec.de/tv_en.html") and of the FC2580 glue in
// librtlsdr.c (rtl-sdr 2.0.3), made for KAT. The I2C transactions are the C driver's, checked
// against it by the tests in trace.rs.
//
// librtlsdr (and the FC2580 code in it): Copyright (C) 2012 Steve Markgraf <steve@steve-m.de>
// and the librtlsdr contributors; the FC2580 code comes from FCI's driver.
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

use super::{TunerBus, TunerInfo, TunerOps, FC2580_TUNER_ID};
use crate::error::{Result, RtlsdrError};

const I2C_ADDR: u8 = 0xac;

pub const FC2580_TUNER_INFO: TunerInfo = TunerInfo {
    id: FC2580_TUNER_ID,
    name: "FCI FC2580",
    i2c_addr: I2C_ADDR,
    check_addr: 0x01,
    // compared with the register's low 7 bits
    check_val: 0x56,
};

/// 16.384 MHz (at least on the Logilink VG0002A). librtlsdr uses this, not the RTL2832U's
/// clock or its correction.
const CRYSTAL_FREQ: u32 = 16_384_000;
/// 2.6 GHz, in kHz: the border frequency which determines whether Low VCO or High VCO is used.
const BORDER_FREQ: u32 = 2_600_000;
/// 0: use the internal XTAL oscillator, 1: an external clock input.
const USE_EXT_CLK: u8 = 0;
/// `FC2580_AGC_EXTERNAL`: voltage control mode.
const AGC_EXTERNAL: i32 = 2;

/// No gain control: librtlsdr lists one step, 0.
const GAINS: [i32; 1] = [0];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Band {
    Uhf,
    L,
    Vhf,
}

#[derive(Debug)]
pub struct Fc2580 {
    /// Kept for [TunerOps::xtal]; the driver works from [CRYSTAL_FREQ].
    xtal: u32,
}

impl Fc2580 {
    pub fn new() -> Self {
        Fc2580 { xtal: CRYSTAL_FREQ }
    }
}

impl Default for Fc2580 {
    fn default() -> Self {
        Self::new()
    }
}

/// `fc2580_i2c_write`: whether it succeeded (`FC2580_FCI_SUCCESS`).
fn i2c_write(bus: &dyn TunerBus, reg: u8, val: u8) -> bool {
    bus.write(I2C_ADDR, &[reg, val]).is_ok()
}

/// `fc2580_i2c_read`: the register, or None (the C then leaves its output as it was).
fn i2c_read(bus: &dyn TunerBus, reg: u8) -> Option<u8> {
    let mut data = [reg];
    bus.write(I2C_ADDR, &data).ok()?;
    bus.read(I2C_ADDR, &mut data).ok()?;
    Some(data[0])
}

fn crystal_freq_khz() -> u32 {
    (CRYSTAL_FREQ + 500) / 1000
}

/// `fc2580_Initialize`
fn initialize(bus: &dyn TunerBus) -> bool {
    set_init(bus, AGC_EXTERNAL, crystal_freq_khz())
}

/// `fc2580_SetRfFreqHz`
fn set_rf_freq_hz(bus: &dyn TunerBus, rf_freq_hz: u32) -> bool {
    let rf_freq_khz = ((u64::from(rf_freq_hz) + 500) / 1000) as u32;
    set_freq(bus, rf_freq_khz, crystal_freq_khz())
}

/// `fc2580_SetBandwidthMode`
fn set_bandwidth_mode(bus: &dyn TunerBus, bandwidth_mode: u8) -> bool {
    set_filter(bus, bandwidth_mode, crystal_freq_khz())
}

/// `fc2580_set_init`: the initial setting, for DVB-H or L-band TDMB. `ifagc_mode` 1 is the
/// internal AGC, 2 the voltage control mode.
fn set_init(bus: &dyn TunerBus, ifagc_mode: i32, freq_xtal: u32) -> bool {
    let mut ok = true;
    ok &= i2c_write(bus, 0x00, 0x00); // Confidential
    ok &= i2c_write(bus, 0x12, 0x86);
    ok &= i2c_write(bus, 0x14, 0x5C);
    ok &= i2c_write(bus, 0x16, 0x3C);
    ok &= i2c_write(bus, 0x1F, 0xD2);
    ok &= i2c_write(bus, 0x09, 0xD7);
    ok &= i2c_write(bus, 0x0B, 0xD5);
    ok &= i2c_write(bus, 0x0C, 0x32);
    ok &= i2c_write(bus, 0x0E, 0x43);
    ok &= i2c_write(bus, 0x21, 0x0A);
    ok &= i2c_write(bus, 0x22, 0x82);
    if ifagc_mode == 1 {
        ok &= i2c_write(bus, 0x45, 0x10); // internal AGC
        ok &= i2c_write(bus, 0x4C, 0x00); // HOLD_AGC polarity
    } else if ifagc_mode == 2 {
        ok &= i2c_write(bus, 0x45, 0x20); // Voltage Control Mode
        ok &= i2c_write(bus, 0x4C, 0x02); // HOLD_AGC polarity
    }
    ok &= i2c_write(bus, 0x3F, 0x88);
    ok &= i2c_write(bus, 0x02, 0x0E);
    ok &= i2c_write(bus, 0x58, 0x14);
    ok &= set_filter(bus, 8, freq_xtal); // BW = 7.8MHz
    ok
}

/// `fc2580_set_freq`: set the LO to `f_lo` kHz (`freq_xtal` in kHz too). The C computes in
/// unsigned 32-bit arithmetic.
fn set_freq(bus: &dyn TunerBus, f_lo: u32, freq_xtal: u32) -> bool {
    // number of preshift to prevent overflow in shifting f_diff to f_diff_shifted
    let pre_shift_bits: u32 = 4;
    let mut data_0x02: u8 = (USE_EXT_CLK << 5) | 0x0E;
    let band = if f_lo > 1_000_000 {
        Band::L
    } else if f_lo > 400_000 {
        Band::Uhf
    } else {
        Band::Vhf
    };
    let mut ok = true;

    let f_vco = match band {
        Band::Uhf => f_lo.wrapping_mul(4),
        Band::L => f_lo.wrapping_mul(2),
        Band::Vhf => f_lo.wrapping_mul(12),
    };
    let r_val: u32 = if f_vco >= (2 * 76u32).wrapping_mul(freq_xtal) {
        1
    } else if f_vco >= 76u32.wrapping_mul(freq_xtal) {
        2
    } else {
        4
    };
    let f_comp = freq_xtal / r_val;
    let n_val = (f_vco / 2) / f_comp;

    let f_diff = f_vco.wrapping_sub((2 * f_comp).wrapping_mul(n_val));
    let f_diff_shifted = f_diff << (20 - pre_shift_bits);
    let div = (2 * f_comp) >> pre_shift_bits;
    let mut k_val = f_diff_shifted / div;

    if f_diff_shifted.wrapping_sub(k_val.wrapping_mul(div)) >= (f_comp >> pre_shift_bits) {
        k_val = k_val.wrapping_add(1);
    }

    if f_vco >= BORDER_FREQ {
        // Select VCO Band
        data_0x02 |= 0x08; // 0x02[3] = 1;
    } else {
        data_0x02 &= 0xF7; // 0x02[3] = 0;
    }

    match band {
        Band::Uhf => {
            data_0x02 &= 0x3F;
            ok &= i2c_write(bus, 0x25, 0xF0);
            ok &= i2c_write(bus, 0x27, 0x77);
            ok &= i2c_write(bus, 0x28, 0x53);
            ok &= i2c_write(bus, 0x29, 0x60);
            ok &= i2c_write(bus, 0x30, 0x09);
            ok &= i2c_write(bus, 0x50, 0x8C);
            ok &= i2c_write(bus, 0x53, 0x50);

            if f_lo < 538_000 {
                ok &= i2c_write(bus, 0x5F, 0x13);
            } else {
                ok &= i2c_write(bus, 0x5F, 0x15);
            }

            if f_lo < 538_000 {
                ok &= i2c_write(bus, 0x61, 0x07);
                ok &= i2c_write(bus, 0x62, 0x06);
                ok &= i2c_write(bus, 0x67, 0x06);
                ok &= i2c_write(bus, 0x68, 0x08);
                ok &= i2c_write(bus, 0x69, 0x10);
                ok &= i2c_write(bus, 0x6A, 0x12);
            } else if f_lo < 794_000 {
                ok &= i2c_write(bus, 0x61, 0x03);
                ok &= i2c_write(bus, 0x62, 0x03);
                ok &= i2c_write(bus, 0x67, 0x03); // ACI improve
                ok &= i2c_write(bus, 0x68, 0x05); // ACI improve
                ok &= i2c_write(bus, 0x69, 0x0C);
                ok &= i2c_write(bus, 0x6A, 0x0E);
            } else {
                ok &= i2c_write(bus, 0x61, 0x07);
                ok &= i2c_write(bus, 0x62, 0x06);
                ok &= i2c_write(bus, 0x67, 0x07);
                ok &= i2c_write(bus, 0x68, 0x09);
                ok &= i2c_write(bus, 0x69, 0x10);
                ok &= i2c_write(bus, 0x6A, 0x12);
            }

            ok &= i2c_write(bus, 0x63, 0x15);

            ok &= i2c_write(bus, 0x6B, 0x0B);
            ok &= i2c_write(bus, 0x6C, 0x0C);
            ok &= i2c_write(bus, 0x6D, 0x78);
            ok &= i2c_write(bus, 0x6E, 0x32);
            ok &= i2c_write(bus, 0x6F, 0x14);
            ok &= set_filter(bus, 8, freq_xtal); // BW = 7.8MHz
        }
        Band::Vhf => {
            data_0x02 = (data_0x02 & 0x3F) | 0x80;
            ok &= i2c_write(bus, 0x27, 0x77);
            ok &= i2c_write(bus, 0x28, 0x33);
            ok &= i2c_write(bus, 0x29, 0x40);
            ok &= i2c_write(bus, 0x30, 0x09);
            ok &= i2c_write(bus, 0x50, 0x8C);
            ok &= i2c_write(bus, 0x53, 0x50);
            ok &= i2c_write(bus, 0x5F, 0x0F);
            ok &= i2c_write(bus, 0x61, 0x07);
            ok &= i2c_write(bus, 0x62, 0x00);
            ok &= i2c_write(bus, 0x63, 0x15);
            ok &= i2c_write(bus, 0x67, 0x03);
            ok &= i2c_write(bus, 0x68, 0x05);
            ok &= i2c_write(bus, 0x69, 0x10);
            ok &= i2c_write(bus, 0x6A, 0x12);
            ok &= i2c_write(bus, 0x6B, 0x08);
            ok &= i2c_write(bus, 0x6C, 0x0A);
            ok &= i2c_write(bus, 0x6D, 0x78);
            ok &= i2c_write(bus, 0x6E, 0x32);
            ok &= i2c_write(bus, 0x6F, 0x54);
            ok &= set_filter(bus, 7, freq_xtal); // BW = 6.8MHz
        }
        Band::L => {
            data_0x02 = (data_0x02 & 0x3F) | 0x40;
            ok &= i2c_write(bus, 0x2B, 0x70);
            ok &= i2c_write(bus, 0x2C, 0x37);
            ok &= i2c_write(bus, 0x2D, 0xE7);
            ok &= i2c_write(bus, 0x30, 0x09);
            ok &= i2c_write(bus, 0x44, 0x20);
            ok &= i2c_write(bus, 0x50, 0x8C);
            ok &= i2c_write(bus, 0x53, 0x50);
            ok &= i2c_write(bus, 0x5F, 0x0F);
            ok &= i2c_write(bus, 0x61, 0x0F);
            ok &= i2c_write(bus, 0x62, 0x00);
            ok &= i2c_write(bus, 0x63, 0x13);
            ok &= i2c_write(bus, 0x67, 0x00);
            ok &= i2c_write(bus, 0x68, 0x02);
            ok &= i2c_write(bus, 0x69, 0x0C);
            ok &= i2c_write(bus, 0x6A, 0x0E);
            ok &= i2c_write(bus, 0x6B, 0x08);
            ok &= i2c_write(bus, 0x6C, 0x0A);
            ok &= i2c_write(bus, 0x6D, 0xA0);
            ok &= i2c_write(bus, 0x6E, 0x50);
            ok &= i2c_write(bus, 0x6F, 0x14);
            ok &= set_filter(bus, 1, freq_xtal); // BW = 1.53MHz
        }
    }

    // A command about AGC clock's pre-divide ratio
    if freq_xtal >= 28000 {
        ok &= i2c_write(bus, 0x4B, 0x22);
    }

    // Commands about VCO Band and PLL setting.
    ok &= i2c_write(bus, 0x02, data_0x02);
    let r_bits: u8 = match r_val {
        1 => 0x00,
        2 => 0x10,
        _ => 0x20,
    };
    let data_0x18 = r_bits.wrapping_add((k_val >> 16) as u8);
    ok &= i2c_write(bus, 0x18, data_0x18); // Load 'R' value and high part of 'K' values
    ok &= i2c_write(bus, 0x1A, (k_val >> 8) as u8); // Load middle part of 'K' value
    ok &= i2c_write(bus, 0x1B, k_val as u8); // Load lower part of 'K' value
    ok &= i2c_write(bus, 0x1C, n_val as u8); // Load 'N' value

    // A command about UHF LNA Load Cap
    if band == Band::Uhf {
        ok &= i2c_write(bus, 0x2D, if f_lo <= 794_000 { 0x9F } else { 0x8F }); // LNA_OUT_CAP
    }

    ok
}

/// `fc2580_set_filter`: the channel filter's bandwidth: 1 is 1.53 MHz (TDMB), 6 is 6 MHz, 7 is
/// 6.8 MHz, 8 is 7.8 MHz. Then up to five tries at its calibration (the C's 5 ms waits are
/// no-ops: "USB latency is enough for now").
fn set_filter(bus: &dyn TunerBus, filter_bw: u8, freq_xtal: u32) -> bool {
    let mut cal_mon: u8 = 0;
    let mut ok = true;

    if filter_bw == 1 {
        ok &= i2c_write(bus, 0x36, 0x1C);
        ok &= i2c_write(bus, 0x37, (4151u32.wrapping_mul(freq_xtal) / 1_000_000) as u8);
        ok &= i2c_write(bus, 0x39, 0x00);
        ok &= i2c_write(bus, 0x2E, 0x09);
    }
    if filter_bw == 6 {
        ok &= i2c_write(bus, 0x36, 0x18);
        ok &= i2c_write(bus, 0x37, (4400u32.wrapping_mul(freq_xtal) / 1_000_000) as u8);
        ok &= i2c_write(bus, 0x39, 0x00);
        ok &= i2c_write(bus, 0x2E, 0x09);
    } else if filter_bw == 7 {
        ok &= i2c_write(bus, 0x36, 0x18);
        ok &= i2c_write(bus, 0x37, (3910u32.wrapping_mul(freq_xtal) / 1_000_000) as u8);
        ok &= i2c_write(bus, 0x39, 0x80);
        ok &= i2c_write(bus, 0x2E, 0x09);
    } else if filter_bw == 8 {
        ok &= i2c_write(bus, 0x36, 0x18);
        ok &= i2c_write(bus, 0x37, (3300u32.wrapping_mul(freq_xtal) / 1_000_000) as u8);
        ok &= i2c_write(bus, 0x39, 0x80);
        ok &= i2c_write(bus, 0x2E, 0x09);
    }
    for _ in 0..5 {
        match i2c_read(bus, 0x2F) {
            Some(v) => cal_mon = v,
            None => ok = false,
        }
        if (cal_mon & 0xC0) != 0xC0 {
            ok &= i2c_write(bus, 0x2E, 0x01);
            ok &= i2c_write(bus, 0x2E, 0x09);
        } else {
            break;
        }
    }

    ok &= i2c_write(bus, 0x2E, 0x01);
    ok
}

fn status(ok: bool, what: &str) -> Result<()> {
    if ok {
        Ok(())
    } else {
        Err(RtlsdrError::RtlsdrErr(format!("FC2580: {what} failed")))
    }
}

impl TunerOps for Fc2580 {
    fn info(&self) -> TunerInfo {
        FC2580_TUNER_INFO
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
        status(initialize(bus), "initialising")
    }
    fn exit(&mut self, _bus: &dyn TunerBus) -> Result<()> {
        Ok(())
    }
    fn set_freq(&mut self, bus: &dyn TunerBus, freq: u32) -> Result<()> {
        status(set_rf_freq_hz(bus, freq), "tuning")
    }
    fn set_bw(&mut self, bus: &dyn TunerBus, _bw: u32) -> Result<()> {
        status(set_bandwidth_mode(bus, 1), "setting the bandwidth")
    }
    fn set_gain(&mut self, _bus: &dyn TunerBus, _gain: i32) -> Result<()> {
        Ok(())
    }
    fn set_gain_mode(&mut self, _bus: &dyn TunerBus, _manual: bool) -> Result<()> {
        Ok(())
    }
}
