// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

pub mod r82xx;
// The drivers ported from librtlsdr (GPL-2.0-or-later, see each file), and what puts them behind
// [Tuner].
#[cfg(feature = "gpl-tuners")]
pub mod e4k;
#[cfg(feature = "gpl-tuners")]
pub mod fc0012;
#[cfg(feature = "gpl-tuners")]
pub mod fc0013;
#[cfg(feature = "gpl-tuners")]
pub mod fc2580;
#[cfg(feature = "gpl-tuners")]
mod ported;
#[cfg(all(test, feature = "gpl-tuners"))]
mod trace;
#[cfg(feature = "gpl-tuners")]
pub(crate) use ported::{Ported, TunerOps};

use crate::device::{Device, BLOCK_SYS, GPD, GPO, GPOE};
use crate::error::Result;
use crate::TunerGain;

/// Ids of the tuners ported from librtlsdr ([TunerInfo::id]).
pub const E4000_TUNER_ID: &str = "e4000";
pub const FC0012_TUNER_ID: &str = "fc0012";
pub const FC0013_TUNER_ID: &str = "fc0013";
pub const FC2580_TUNER_ID: &str = "fc2580";

#[derive(Debug, Clone, Copy)]

pub struct TunerInfo {
    pub id: &'static str,
    pub name: &'static str,
    pub i2c_addr: u8,
    pub check_addr: u8,
    pub check_val: u8,
    // pub gains: Vec<i8>,
}

pub trait Tuner: std::fmt::Debug {
    fn init(&mut self, handle: &Device) -> Result<()>;
    fn get_info(&self) -> Result<TunerInfo>;
    fn get_gains(&self) -> Result<Vec<i32>>;
    fn read_gain(&self, handle: &Device) -> Result<i32>;
    fn set_gain(&mut self, handle: &Device, gain: TunerGain) -> Result<()>;
    fn set_freq(&mut self, handle: &Device, freq: u32) -> Result<()>;
    fn set_bandwidth(&mut self, handle: &Device, bw: u32, rate: u32) -> Result<()>;
    fn get_if_freq(&self) -> Result<u32>;
    fn get_xtal_freq(&self) -> Result<u32>;
    fn set_xtal_freq(&mut self, freq: u32) -> Result<()>;
    fn exit(&mut self, handle: &Device) -> Result<()>;
}

/// What a tuner driver needs of the RTL2832U: its I2C bus (through the repeater, which the caller
/// turns on) and its GPIO pins. [Device] is the real one; the tests use a model of the tuner.
pub trait TunerBus {
    /// Write `data` to the I2C device at `addr` (8-bit address); the number of bytes written.
    fn write(&self, addr: u8, data: &[u8]) -> Result<usize>;
    /// Read `buf.len()` bytes from the I2C device at `addr`; the number of bytes read.
    fn read(&self, addr: u8, buf: &mut [u8]) -> Result<usize>;
    /// Drive GPIO pin `gpio` high or low (`rtlsdr_set_gpio_bit`).
    #[cfg_attr(not(feature = "gpl-tuners"), allow(dead_code))]
    fn set_gpio_bit(&self, gpio: u8, on: bool) -> Result<()>;
    /// Make GPIO pin `gpio` an output (`rtlsdr_set_gpio_output`).
    #[cfg_attr(not(feature = "gpl-tuners"), allow(dead_code))]
    fn set_gpio_output(&self, gpio: u8) -> Result<()>;
}

impl TunerBus for Device {
    fn write(&self, addr: u8, data: &[u8]) -> Result<usize> {
        self.i2c_write(addr.into(), data)
    }

    fn read(&self, addr: u8, buf: &mut [u8]) -> Result<usize> {
        let len = buf.len() as u8;
        self.i2c_read(addr.into(), buf, len)
    }

    fn set_gpio_bit(&self, gpio: u8, on: bool) -> Result<()> {
        let bit = 1u16 << gpio;
        let r = self.read_reg(BLOCK_SYS, GPO, 1)?;
        self.write_reg(BLOCK_SYS, GPO, if on { r | bit } else { r & !bit }, 1)?;
        Ok(())
    }

    fn set_gpio_output(&self, gpio: u8) -> Result<()> {
        let bit = 1u16 << gpio;
        let r = self.read_reg(BLOCK_SYS, GPD, 1)?;
        self.write_reg(BLOCK_SYS, GPD, r & !bit, 1)?;
        let r = self.read_reg(BLOCK_SYS, GPOE, 1)?;
        self.write_reg(BLOCK_SYS, GPOE, r | bit, 1)?;
        Ok(())
    }
}

/// Read one register of the I2C device at `addr` (`rtlsdr_i2c_read_reg`); None when the device
/// does not answer.
pub(crate) fn i2c_read_reg(bus: &dyn TunerBus, addr: u8, reg: u8) -> Option<u8> {
    let mut data = [0u8];
    bus.write(addr, &[reg]).ok()?;
    bus.read(addr, &mut data).ok()?;
    Some(data[0])
}

#[derive(Debug)]
pub struct NoTuner {}
impl Tuner for NoTuner {
    fn init(&mut self, _handle: &Device) -> Result<()> {
        Ok(())
    }
    fn get_info(&self) -> Result<TunerInfo> {
        Ok(TunerInfo {
            id: "",
            name: "",
            i2c_addr: 0,
            check_addr: 0,
            check_val: 0,
        })
    }
    fn get_gains(&self) -> Result<Vec<i32>> {
        Ok(vec![])
    }
    fn read_gain(&self, _handle: &Device) -> Result<i32> {
        Ok(0)
    }
    fn set_gain(&mut self, _handle: &Device, _gain: TunerGain) -> Result<()> {
        Ok(())
    }
    fn set_freq(&mut self, _handle: &Device, _freq: u32) -> Result<()> {
        Ok(())
    }
    fn set_bandwidth(&mut self, _handle: &Device, _bw: u32, _rate: u32) -> Result<()> {
        Ok(())
    }
    fn get_xtal_freq(&self) -> Result<u32> {
        Ok(0)
    }
    fn set_xtal_freq(&mut self, _freq: u32) -> Result<()> {
        Ok(())
    }
    fn get_if_freq(&self) -> Result<u32> {
        Ok(0)
    }
    fn exit(&mut self, _handle: &Device) -> Result<()> {
        Ok(())
    }
}
