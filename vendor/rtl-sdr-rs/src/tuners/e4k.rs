// Elonics E4000 tuner driver: a Rust port of librtlsdr's tuner_e4k.c and of the E4000 glue in
// librtlsdr.c (rtl-sdr 2.0.3), made for KAT. The I2C transactions are the C driver's, checked
// against it by the tests in trace.rs. Only what librtlsdr uses is ported (not the DC offset
// table, the common mode or the enhanced LNA gain, which it leaves out).
//
// (C) 2011-2012 by Harald Welte <laforge@gnumonks.org>
// (C) 2012 by Sylvain Munaut <tnt@246tNt.com>
// (C) 2012 by Hoernchen <la@tfc-server.de>
// librtlsdr glue: Copyright (C) 2012 Steve Markgraf <steve@steve-m.de>
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

use super::{TunerBus, TunerInfo, TunerOps, E4000_TUNER_ID};
use crate::error::{Result, RtlsdrError};
use log::warn;

const I2C_ADDR: u8 = 0xc8;

pub const E4000_TUNER_INFO: TunerInfo = TunerInfo {
    id: E4000_TUNER_ID,
    name: "Elonics E4000",
    i2c_addr: I2C_ADDR,
    check_addr: 0x02,
    check_val: 0x40,
};

/// librtlsdr's E4000 gain steps (`rtlsdr_get_tuner_gains`), tenths of a dB.
const GAINS: [i32; 14] = [-10, 15, 40, 65, 90, 115, 140, 165, 190, 215, 240, 290, 340, 420];

const EINVAL: i32 = 22;

// Registers (enum e4k_reg)
const REG_MASTER1: u8 = 0x00;
const REG_CLK_INP: u8 = 0x05;
const REG_REF_CLK: u8 = 0x06;
const REG_SYNTH1: u8 = 0x07;
const REG_SYNTH3: u8 = 0x09;
const REG_SYNTH4: u8 = 0x0a;
const REG_SYNTH5: u8 = 0x0b;
const REG_SYNTH7: u8 = 0x0d;
const REG_FILT1: u8 = 0x10;
const REG_FILT2: u8 = 0x11;
const REG_FILT3: u8 = 0x12;
const REG_GAIN1: u8 = 0x14;
const REG_GAIN2: u8 = 0x15;
const REG_GAIN3: u8 = 0x16;
const REG_GAIN4: u8 = 0x17;
const REG_AGC1: u8 = 0x1a;
const REG_AGC4: u8 = 0x1d;
const REG_AGC5: u8 = 0x1e;
const REG_AGC6: u8 = 0x1f;
const REG_AGC7: u8 = 0x20;
const REG_AGC11: u8 = 0x24;
const REG_DC5: u8 = 0x2d;
const REG_DCTIME1: u8 = 0x70;
const REG_DCTIME2: u8 = 0x71;
const REG_BIAS: u8 = 0x78;
const REG_CLKOUT_PWDN: u8 = 0x7a;

const MASTER1_RESET: u8 = 1 << 0;
const MASTER1_NORM_STBY: u8 = 1 << 1;
const MASTER1_POR_DET: u8 = 1 << 2;
const FILT3_DISABLE: u8 = 1 << 5;
const AGC7_MIX_GAIN_AUTO: u8 = 1 << 0;
const AGC1_MOD_MASK: u8 = 0xF;
const AGC_MOD_SERIAL: u8 = 0x0;
const AGC_MOD_IF_SERIAL_LNA_AUTON: u8 = 0x9;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Band {
    Vhf2 = 0,
    Vhf3 = 1,
    Uhf = 2,
    L = 3,
}

#[derive(Debug, Clone, Copy)]
enum IfFilter {
    Mix = 0,
    Chan = 1,
    Rc = 2,
}

/// A field of a register: the register, the field's lowest bit and its width.
struct RegField {
    reg: u8,
    shift: u8,
    width: u8,
}

/// look-up table bit-width -> mask
const WIDTH2MASK: [u8; 9] = [0, 1, 3, 7, 0xf, 0x1f, 0x3f, 0x7f, 0xff];

const fn mhz(x: u32) -> u32 {
    x * 1000 * 1000
}
const fn khz(x: u32) -> u32 {
    x * 1000
}

const RF_FILT_CENTER_UHF: [u32; 16] = [
    mhz(360), mhz(380), mhz(405), mhz(425), mhz(450), mhz(475), mhz(505), mhz(540), mhz(575),
    mhz(615), mhz(670), mhz(720), mhz(760), mhz(840), mhz(890), mhz(970),
];

const RF_FILT_CENTER_L: [u32; 16] = [
    mhz(1300), mhz(1320), mhz(1360), mhz(1410), mhz(1445), mhz(1460), mhz(1490), mhz(1530),
    mhz(1560), mhz(1590), mhz(1640), mhz(1660), mhz(1680), mhz(1700), mhz(1720), mhz(1750),
];

/// Mixer Filter
const MIX_FILTER_BW: [u32; 16] = [
    khz(27000), khz(27000), khz(27000), khz(27000), khz(27000), khz(27000), khz(27000), khz(27000),
    khz(4600), khz(4200), khz(3800), khz(3400), khz(3300), khz(2700), khz(2300), khz(1900),
];

/// IF RC Filter
const IFRC_FILTER_BW: [u32; 16] = [
    khz(21400), khz(21000), khz(17600), khz(14700), khz(12400), khz(10600), khz(9000), khz(7700),
    khz(6400), khz(5300), khz(4400), khz(3400), khz(2600), khz(1800), khz(1200), khz(1000),
];

/// IF Channel Filter
const IFCH_FILTER_BW: [u32; 32] = [
    khz(5500), khz(5300), khz(5000), khz(4800), khz(4600), khz(4400), khz(4300), khz(4100),
    khz(3900), khz(3800), khz(3700), khz(3600), khz(3400), khz(3300), khz(3200), khz(3100),
    khz(3000), khz(2950), khz(2900), khz(2800), khz(2750), khz(2700), khz(2600), khz(2550),
    khz(2500), khz(2450), khz(2400), khz(2300), khz(2280), khz(2240), khz(2200), khz(2150),
];

fn if_filter_bw(filter: IfFilter) -> &'static [u32] {
    match filter {
        IfFilter::Mix => &MIX_FILTER_BW,
        IfFilter::Chan => &IFCH_FILTER_BW,
        IfFilter::Rc => &IFRC_FILTER_BW,
    }
}

/// Indexed by [IfFilter].
const IF_FILTER_FIELDS: [RegField; 3] = [
    RegField { reg: REG_FILT2, shift: 4, width: 4 },
    RegField { reg: REG_FILT3, shift: 0, width: 5 },
    RegField { reg: REG_FILT2, shift: 0, width: 4 },
];

// Frequency Control

const PLL_Y: u64 = 65536;

struct PllSettings {
    freq: u32,
    reg_synth7: u8,
    mult: u8,
}

const PLL_VARS: [PllSettings; 10] = [
    PllSettings { freq: khz(72400), reg_synth7: (1 << 3) | 7, mult: 48 },
    PllSettings { freq: khz(81200), reg_synth7: (1 << 3) | 6, mult: 40 },
    PllSettings { freq: khz(108300), reg_synth7: (1 << 3) | 5, mult: 32 },
    PllSettings { freq: khz(162500), reg_synth7: (1 << 3) | 4, mult: 24 },
    PllSettings { freq: khz(216600), reg_synth7: (1 << 3) | 3, mult: 16 },
    PllSettings { freq: khz(325000), reg_synth7: (1 << 3) | 2, mult: 12 },
    PllSettings { freq: khz(350000), reg_synth7: (1 << 3) | 1, mult: 8 },
    PllSettings { freq: khz(432000), reg_synth7: 3, mult: 8 },
    PllSettings { freq: khz(667000), reg_synth7: 2, mult: 6 },
    PllSettings { freq: khz(1200000), reg_synth7: 1, mult: 4 },
];

// Gain Control

const IF_STAGE1_GAIN: [i8; 2] = [-3, 6];
const IF_STAGE23_GAIN: [i8; 4] = [0, 3, 6, 9];
const IF_STAGE4_GAIN: [i8; 4] = [0, 1, 2, 2];
const IF_STAGE56_GAIN: [i8; 8] = [3, 6, 9, 12, 15, 15, 15, 15];

fn if_stage_gain(stage: u8) -> &'static [i8] {
    match stage {
        1 => &IF_STAGE1_GAIN,
        2 | 3 => &IF_STAGE23_GAIN,
        4 => &IF_STAGE4_GAIN,
        5 | 6 => &IF_STAGE56_GAIN,
        _ => &[],
    }
}

const IF_STAGE_GAIN_REGS: [RegField; 7] = [
    RegField { reg: 0, shift: 0, width: 0 },
    RegField { reg: REG_GAIN3, shift: 0, width: 1 },
    RegField { reg: REG_GAIN3, shift: 1, width: 2 },
    RegField { reg: REG_GAIN3, shift: 3, width: 2 },
    RegField { reg: REG_GAIN3, shift: 5, width: 2 },
    RegField { reg: REG_GAIN4, shift: 0, width: 3 },
    RegField { reg: REG_GAIN4, shift: 3, width: 3 },
];

/// LNA gain (tenths of a dB) and the GAIN1 value for it.
const LNAGAIN: [(i32, u8); 13] = [
    (-50, 0),
    (-25, 1),
    (0, 4),
    (25, 5),
    (50, 6),
    (75, 7),
    (100, 8),
    (125, 9),
    (150, 10),
    (175, 11),
    (200, 12),
    (250, 13),
    (300, 14),
];

/// `struct e4k_pll_params`
#[derive(Debug, Clone, Copy, Default)]
struct PllParams {
    fosc: u32,
    intended_flo: u32,
    flo: u32,
    x: u16,
    z: u8,
    r: u8,
    r_idx: u8,
    threephase: u8,
}

/// The E4000's state (`struct e4k_state`).
#[derive(Debug)]
pub struct E4k {
    band: Band,
    vco: PllParams,
    /// The tuner's clock: the RTL2832U's 28.8 MHz, corrected; [Self::init] takes it as fosc.
    xtal: u32,
}

impl E4k {
    pub fn new() -> Self {
        E4k { band: Band::Vhf2, vco: PllParams::default(), xtal: 28_800_000 }
    }
}

impl Default for E4k {
    fn default() -> Self {
        Self::new()
    }
}

fn unsigned_delta(a: u32, b: u32) -> u32 {
    if a > b {
        a - b
    } else {
        b - a
    }
}

// Register Access

/// `e4k_reg_write`: 0, or -1 unless both bytes went out.
fn reg_write(bus: &dyn TunerBus, reg: u8, val: u8) -> i32 {
    match bus.write(I2C_ADDR, &[reg, val]) {
        Ok(2) => 0,
        _ => -1,
    }
}

/// `e4k_reg_read`: the register, or -1.
fn reg_read(bus: &dyn TunerBus, reg: u8) -> i32 {
    let mut data = [reg];
    match bus.write(I2C_ADDR, &data) {
        Ok(n) if n >= 1 => {}
        _ => return -1,
    }
    match bus.read(I2C_ADDR, &mut data) {
        Ok(n) if n >= 1 => i32::from(data[0]),
        _ => -1,
    }
}

/// `e4k_reg_set_mask`: set or clear some (masked) bits inside a register; nothing is written
/// when the masked bits already equal `val` (a failed read counts as 0xff, as in the C).
fn reg_set_mask(bus: &dyn TunerBus, reg: u8, mask: u8, val: u8) -> i32 {
    let tmp = reg_read(bus, reg) as u8;
    if (tmp & mask) == val {
        return 0;
    }
    reg_write(bus, reg, (tmp & !mask) | (val & mask))
}

/// `e4k_field_write`
fn field_write(bus: &dyn TunerBus, field: &RegField, val: u8) -> i32 {
    let rc = reg_read(bus, field.reg);
    if rc < 0 {
        return rc;
    }
    let mask = (u32::from(WIDTH2MASK[usize::from(field.width)]) << field.shift) as u8;
    reg_set_mask(bus, field.reg, mask, (u32::from(val) << field.shift) as u8)
}

// Filter Control

fn closest_arr_idx(arr: &[u32], freq: u32) -> usize {
    let mut bi = 0;
    let mut best_delta = u32::MAX;
    // iterate over the array containing a list of the center frequencies, selecting the
    // closest one
    for (i, &f) in arr.iter().enumerate() {
        let delta = unsigned_delta(freq, f);
        if delta < best_delta {
            best_delta = delta;
            bi = i;
        }
    }
    bi
}

/// return 4-bit index as to which RF filter to select
fn choose_rf_filter(band: Band, freq: u32) -> u8 {
    match band {
        Band::Vhf2 | Band::Vhf3 => 0,
        Band::Uhf => closest_arr_idx(&RF_FILT_CENTER_UHF, freq) as u8,
        Band::L => closest_arr_idx(&RF_FILT_CENTER_L, freq) as u8,
    }
}

/// `e4k_if_filter_bw_set`: set the band-width of one of the IF filters.
fn if_filter_bw_set(bus: &dyn TunerBus, filter: IfFilter, bandwidth: u32) -> i32 {
    let bw_idx = closest_arr_idx(if_filter_bw(filter), bandwidth);
    field_write(bus, &IF_FILTER_FIELDS[filter as usize], bw_idx as u8)
}

/// `e4k_if_filter_chan_enable`
fn if_filter_chan_enable(bus: &dyn TunerBus, on: bool) -> i32 {
    reg_set_mask(bus, REG_FILT3, FILT3_DISABLE, if on { 0 } else { FILT3_DISABLE })
}

// Frequency Control

fn is_fosc_valid(fosc: u32) -> bool {
    if !(mhz(16)..=mhz(30)).contains(&fosc) {
        warn!("[E4K] Fosc {fosc} invalid");
        return false;
    }
    true
}

/// compute Fvco based on Fosc, Z and X: Fvco = Fosc * Z + (Fosc * X) / Y
fn compute_fvco(f_osc: u32, z: u8, x: u16) -> u64 {
    let fvco_z = u64::from(f_osc) * u64::from(z);
    let fvco_x = (u64::from(f_osc) * u64::from(x)) / PLL_Y;
    fvco_z + fvco_x
}

fn compute_flo(f_osc: u32, z: u8, x: u16, r: u8) -> u32 {
    let fvco = compute_fvco(f_osc, z, x);
    if fvco == 0 {
        return (-EINVAL) as u32;
    }
    (fvco / u64::from(r)) as u32
}

/// `e4k_compute_pll_params`: the PLL parameters for `intended_flo`; the frequency they give,
/// or 0 when fosc is out of range.
fn compute_pll_params(oscp: &mut PllParams, fosc: u32, intended_flo: u32) -> u32 {
    let mut r: u8 = 2;
    let mut three_phase_mixing = 0;
    oscp.r_idx = 0;

    if !is_fosc_valid(fosc) {
        return 0;
    }

    for pv in &PLL_VARS {
        if intended_flo < pv.freq {
            three_phase_mixing = u8::from(pv.reg_synth7 & 0x08 != 0);
            oscp.r_idx = pv.reg_synth7;
            r = pv.mult;
            break;
        }
    }

    // flo(max) = 1700MHz, R(max) = 48, we need 64bit!
    let intended_fvco = u64::from(intended_flo) * u64::from(r);

    // compute integral component of multiplier
    let z = intended_fvco / u64::from(fosc);

    // compute fractional part. this will not overflow, as fosc(max) = 30MHz and z(max) = 255
    let remainder = intended_fvco - u64::from(fosc) * z;
    // remainder(max) = 30MHz, E4K_PLL_Y = 65536 -> 64bit!
    let x = ((remainder * PLL_Y) / u64::from(fosc)) as u32;
    // x(max) as result of this computation is 65536

    let flo = compute_flo(fosc, z as u8, x as u16, r);

    oscp.fosc = fosc;
    oscp.flo = flo;
    oscp.intended_flo = intended_flo;
    oscp.r = r;
    oscp.threephase = three_phase_mixing;
    oscp.x = x as u16;
    oscp.z = z as u8;

    flo
}

impl E4k {
    /// `e4k_rf_filter_set`: select the RF filter for the band and frequency.
    fn rf_filter_set(&self, bus: &dyn TunerBus) -> i32 {
        let rc = choose_rf_filter(self.band, self.vco.flo);
        reg_set_mask(bus, REG_FILT1, 0xF, rc)
    }

    /// `e4k_band_set`
    fn band_set(&mut self, bus: &dyn TunerBus, band: Band) -> i32 {
        match band {
            Band::Vhf2 | Band::Vhf3 | Band::Uhf => {
                reg_write(bus, REG_BIAS, 3);
            }
            Band::L => {
                reg_write(bus, REG_BIAS, 0);
            }
        }

        // workaround: if we don't reset this register before writing to it, we get a gap
        // between 325-350 MHz
        reg_set_mask(bus, REG_SYNTH1, 0x06, 0);
        let rc = reg_set_mask(bus, REG_SYNTH1, 0x06, (band as u8) << 1);
        if rc >= 0 {
            self.band = band;
        }
        rc
    }

    /// `e4k_tune_params`: program the PLL, then the band and the RF filter for it.
    fn tune_params(&mut self, bus: &dyn TunerBus, p: &PllParams) -> u32 {
        // program R + 3phase/2phase
        reg_write(bus, REG_SYNTH7, p.r_idx);
        // program Z
        reg_write(bus, REG_SYNTH3, p.z);
        // program X
        reg_write(bus, REG_SYNTH4, (p.x & 0xff) as u8);
        reg_write(bus, REG_SYNTH5, (p.x >> 8) as u8);

        // we're in auto calibration mode, so there's no need to trigger it

        self.vco = *p;

        // set the band
        if self.vco.flo < mhz(140) {
            self.band_set(bus, Band::Vhf2);
        } else if self.vco.flo < mhz(350) {
            self.band_set(bus, Band::Vhf3);
        } else if self.vco.flo < mhz(1135) {
            self.band_set(bus, Band::Uhf);
        } else {
            self.band_set(bus, Band::L);
        }

        // select and set proper RF filter
        self.rf_filter_set(bus);

        self.vco.flo
    }

    /// `e4k_tune_freq`: 0, or -EINVAL when there are no PLL parameters, or -1 when the PLL does
    /// not lock (a failed read of SYNTH1 counts as locked, as in the C).
    fn tune_freq(&mut self, bus: &dyn TunerBus, freq: u32) -> i32 {
        let mut p = PllParams::default();

        // determine PLL parameters
        if compute_pll_params(&mut p, self.vco.fosc, freq) == 0 {
            return -EINVAL;
        }

        // actually tune to those parameters
        self.tune_params(bus, &p);

        // check PLL lock
        let rc = reg_read(bus, REG_SYNTH1) as u32;
        if rc & 0x01 == 0 {
            warn!("[E4K] PLL not locked for {freq} Hz!");
            return -1;
        }
        0
    }
}

// Gain Control

/// `e4k_set_lna_gain`: only the steps of [LNAGAIN].
fn set_lna_gain(bus: &dyn TunerBus, gain: i32) -> i32 {
    for &(g, val) in &LNAGAIN {
        if g == gain {
            reg_set_mask(bus, REG_GAIN1, 0xf, val);
            return gain;
        }
    }
    -EINVAL
}

/// `e4k_enable_manual_gain`
fn enable_manual_gain(bus: &dyn TunerBus, manual: bool) -> i32 {
    if manual {
        // Set LNA mode to manual
        reg_set_mask(bus, REG_AGC1, AGC1_MOD_MASK, AGC_MOD_SERIAL);

        // Set Mixer Gain Control to manual
        reg_set_mask(bus, REG_AGC7, AGC7_MIX_GAIN_AUTO, 0);
    } else {
        // Set LNA mode to auto
        reg_set_mask(bus, REG_AGC1, AGC1_MOD_MASK, AGC_MOD_IF_SERIAL_LNA_AUTON);
        // Set Mixer Gain Control to auto
        reg_set_mask(bus, REG_AGC7, AGC7_MIX_GAIN_AUTO, 1);

        reg_set_mask(bus, REG_AGC11, 0x7, 0);
    }
    0
}

fn find_stage_gain(stage: u8, val: i8) -> i32 {
    if usize::from(stage) >= IF_STAGE_GAIN_REGS.len() {
        return -EINVAL;
    }
    match if_stage_gain(stage).iter().position(|&g| g == val) {
        Some(i) => i as i32,
        None => -EINVAL,
    }
}

/// `e4k_if_gain_set`: set the gain of one of the IF gain stages (1..6), in dB.
fn if_gain_set(bus: &dyn TunerBus, stage: u8, value: i8) -> i32 {
    let rc = find_stage_gain(stage, value);
    if rc < 0 {
        return rc;
    }

    // compute the bit-mask for the given gain field
    let field = &IF_STAGE_GAIN_REGS[usize::from(stage)];
    let mask = (u32::from(WIDTH2MASK[usize::from(field.width)]) << field.shift) as u8;

    reg_set_mask(bus, field.reg, mask, (rc << field.shift) as u8)
}

/// `e4k_mixer_gain_set`: 4 or 12 dB.
fn mixer_gain_set(bus: &dyn TunerBus, value: i8) -> i32 {
    let bit = match value {
        4 => 0,
        12 => 1,
        _ => return -EINVAL,
    };
    reg_set_mask(bus, REG_GAIN2, 1, bit)
}

// Standby

/// `e4k_standby`
fn standby(bus: &dyn TunerBus, enable: bool) -> i32 {
    reg_set_mask(bus, REG_MASTER1, MASTER1_NORM_STBY, if enable { 0 } else { MASTER1_NORM_STBY });
    0
}

// Initialization

fn magic_init(bus: &dyn TunerBus) {
    reg_write(bus, 0x7e, 0x01);
    reg_write(bus, 0x7f, 0xfe);
    reg_write(bus, 0x82, 0x00);
    reg_write(bus, 0x86, 0x50); // polarity A
    reg_write(bus, 0x87, 0x20);
    reg_write(bus, 0x88, 0x01);
    reg_write(bus, 0x9f, 0x7f);
    reg_write(bus, 0xa0, 0x07);
}

/// `e4k_init`
fn init(bus: &dyn TunerBus) -> i32 {
    // make a dummy i2c read or write command, will not be ACKed!
    reg_read(bus, 0);

    // Make sure we reset everything and clear POR indicator
    reg_write(bus, REG_MASTER1, MASTER1_RESET | MASTER1_NORM_STBY | MASTER1_POR_DET);

    // Configure clock input
    reg_write(bus, REG_CLK_INP, 0x00);

    // Disable clock output
    reg_write(bus, REG_REF_CLK, 0x00);
    reg_write(bus, REG_CLKOUT_PWDN, 0x96);

    // Write some magic values into registers
    magic_init(bus);

    // Set LNA mode to manual
    reg_write(bus, REG_AGC4, 0x10); // High threshold
    reg_write(bus, REG_AGC5, 0x04); // Low threshold
    reg_write(bus, REG_AGC6, 0x1a); // LNA calib + loop rate

    reg_set_mask(bus, REG_AGC1, AGC1_MOD_MASK, AGC_MOD_SERIAL);

    // Set Mixer Gain Control to manual
    reg_set_mask(bus, REG_AGC7, AGC7_MIX_GAIN_AUTO, 0);

    // Use auto-gain as default
    enable_manual_gain(bus, false);

    // Select moderate gain levels
    if_gain_set(bus, 1, 6);
    if_gain_set(bus, 2, 0);
    if_gain_set(bus, 3, 0);
    if_gain_set(bus, 4, 0);
    if_gain_set(bus, 5, 9);
    if_gain_set(bus, 6, 9);

    // Set the most narrow filter we can possibly use
    if_filter_bw_set(bus, IfFilter::Mix, khz(1900));
    if_filter_bw_set(bus, IfFilter::Rc, khz(1000));
    if_filter_bw_set(bus, IfFilter::Chan, khz(2150));
    if_filter_chan_enable(bus, true);

    // Disable time variant DC correction and LUT
    reg_set_mask(bus, REG_DC5, 0x03, 0);
    reg_set_mask(bus, REG_DCTIME1, 0x03, 0);
    reg_set_mask(bus, REG_DCTIME2, 0x03, 0);

    0
}

fn status(rc: i32, what: &str) -> Result<()> {
    if rc == 0 {
        Ok(())
    } else {
        Err(RtlsdrError::RtlsdrErr(format!("E4000: {what} failed")))
    }
}

impl TunerOps for E4k {
    fn info(&self) -> TunerInfo {
        E4000_TUNER_INFO
    }
    fn gains(&self) -> &'static [i32] {
        &GAINS
    }
    fn xtal(&self) -> u32 {
        self.xtal
    }
    /// librtlsdr reads a new clock or correction straight into the PLL's fosc.
    fn set_xtal(&mut self, xtal: u32) {
        self.xtal = xtal;
        self.vco.fosc = xtal;
    }
    fn init(&mut self, bus: &dyn TunerBus) -> Result<()> {
        self.vco.fosc = self.xtal;
        status(init(bus), "initialising")
    }
    fn exit(&mut self, bus: &dyn TunerBus) -> Result<()> {
        status(standby(bus, true), "entering standby")
    }
    fn set_freq(&mut self, bus: &dyn TunerBus, freq: u32) -> Result<()> {
        match self.tune_freq(bus, freq) {
            0 => Ok(()),
            rc if rc == -EINVAL => Err(RtlsdrError::RtlsdrErr(format!(
                "E4000: no PLL settings for {freq} Hz"
            ))),
            _ => Err(RtlsdrError::RtlsdrErr(format!("E4000: PLL not locked at {freq} Hz"))),
        }
    }
    fn set_bw(&mut self, bus: &dyn TunerBus, bw: u32) -> Result<()> {
        let mut r = 0;
        r |= if_filter_bw_set(bus, IfFilter::Mix, bw);
        r |= if_filter_bw_set(bus, IfFilter::Rc, bw);
        r |= if_filter_bw_set(bus, IfFilter::Chan, bw);
        status(r, "setting the IF filters")
    }
    fn set_gain(&mut self, bus: &dyn TunerBus, gain: i32) -> Result<()> {
        let mixgain: i8 = if gain > 340 { 12 } else { 4 };
        if set_lna_gain(bus, 300.min(gain - i32::from(mixgain) * 10)) == -EINVAL {
            return status(-1, &format!("setting the gain to {gain}"));
        }
        if mixer_gain_set(bus, mixgain) == -EINVAL {
            return status(-1, &format!("setting the gain to {gain}"));
        }
        Ok(())
    }
    fn set_gain_mode(&mut self, bus: &dyn TunerBus, manual: bool) -> Result<()> {
        status(enable_manual_gain(bus, manual), "setting the gain mode")
    }
}
