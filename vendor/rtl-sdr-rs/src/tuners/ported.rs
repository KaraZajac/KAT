// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! The drivers ported from librtlsdr (e4k, fc0012, fc0013, fc2580) behind rtl-sdr-rs's [Tuner].

use super::{Tuner, TunerBus, TunerInfo};
use crate::device::Device;
use crate::error::Result;
use crate::TunerGain;

/// A tuner driver ported from librtlsdr, at the level of librtlsdr's tuner interface
/// (`rtlsdr_tuner_iface_t`, `rtlsdr_get_tuner_gains`); [Ported] puts it behind [Tuner].
pub(crate) trait TunerOps: std::fmt::Debug {
    fn info(&self) -> TunerInfo;
    /// The gain steps, in tenths of a dB.
    fn gains(&self) -> &'static [i32];
    /// The tuner's clock in Hz, corrected by the PPM correction.
    fn xtal(&self) -> u32;
    fn set_xtal(&mut self, xtal: u32);
    fn init(&mut self, bus: &dyn TunerBus) -> Result<()>;
    fn exit(&mut self, bus: &dyn TunerBus) -> Result<()>;
    fn set_freq(&mut self, bus: &dyn TunerBus, freq: u32) -> Result<()>;
    fn set_bw(&mut self, bus: &dyn TunerBus, bw: u32) -> Result<()>;
    /// Set a gain in tenths of a dB, as librtlsdr's `set_gain` (the E4000 takes only its steps).
    fn set_gain(&mut self, bus: &dyn TunerBus, gain: i32) -> Result<()>;
    fn set_gain_mode(&mut self, bus: &dyn TunerBus, manual: bool) -> Result<()>;
}

/// A librtlsdr port behind [Tuner]. A manual gain is first rounded to the nearest step, as
/// librtlsdr's tools do (`nearest_gain`), since the E4000 refuses anything else. All four are
/// zero-IF tuners.
#[derive(Debug)]
pub(crate) struct Ported<T: TunerOps> {
    ops: T,
    /// The last manual gain set (librtlsdr's `dev->gain`).
    gain: i32,
}

impl<T: TunerOps> Ported<T> {
    pub(crate) fn new(ops: T) -> Self {
        Ported { ops, gain: 0 }
    }
}

/// The step in `gains` nearest to `target` (the first of two as near).
fn nearest_gain(gains: &[i32], target: i32) -> i32 {
    let dist = |g: i32| (i64::from(target) - i64::from(g)).abs();
    let mut nearest = gains.first().copied().unwrap_or(0);
    for &g in gains {
        if dist(g) < dist(nearest) {
            nearest = g;
        }
    }
    nearest
}

impl<T: TunerOps> Tuner for Ported<T> {
    fn init(&mut self, handle: &Device) -> Result<()> {
        self.ops.init(handle)
    }
    fn get_info(&self) -> Result<TunerInfo> {
        Ok(self.ops.info())
    }
    fn get_gains(&self) -> Result<Vec<i32>> {
        Ok(self.ops.gains().to_vec())
    }
    fn read_gain(&self, _handle: &Device) -> Result<i32> {
        Ok(self.gain)
    }
    fn set_gain(&mut self, handle: &Device, gain: TunerGain) -> Result<()> {
        match gain {
            TunerGain::Auto => self.ops.set_gain_mode(handle, false),
            TunerGain::Manual(target) => {
                self.ops.set_gain_mode(handle, true)?;
                let step = nearest_gain(self.ops.gains(), target);
                let r = self.ops.set_gain(handle, step);
                self.gain = if r.is_ok() { step } else { 0 };
                r
            }
        }
    }
    fn set_freq(&mut self, handle: &Device, freq: u32) -> Result<()> {
        self.ops.set_freq(handle, freq)
    }
    fn set_bandwidth(&mut self, handle: &Device, bw: u32, _rate: u32) -> Result<()> {
        self.ops.set_bw(handle, bw)
    }
    fn get_if_freq(&self) -> Result<u32> {
        Ok(0)
    }
    fn get_xtal_freq(&self) -> Result<u32> {
        Ok(self.ops.xtal())
    }
    fn set_xtal_freq(&mut self, freq: u32) -> Result<()> {
        self.ops.set_xtal(freq);
        Ok(())
    }
    fn exit(&mut self, handle: &Device) -> Result<()> {
        self.ops.exit(handle)
    }
}

#[cfg(test)]
mod tests {
    use super::nearest_gain;

    /// librtlsdr's tools round a requested gain to the nearest step (`nearest_gain`, the first of
    /// two as near); the E4000 refuses anything else.
    #[test]
    fn manual_gains_round_to_the_nearest_step() {
        let e4k = [-10, 15, 40, 65, 90, 115, 140, 165, 190, 215, 240, 290, 340, 420];
        assert_eq!(nearest_gain(&e4k, 0), -10);
        assert_eq!(nearest_gain(&e4k, 2), -10);
        assert_eq!(nearest_gain(&e4k, 3), 15);
        assert_eq!(nearest_gain(&e4k, 265), 240, "265 is as near to 240 as to 290: the first");
        assert_eq!(nearest_gain(&e4k, 496), 420);
        assert_eq!(nearest_gain(&e4k, -500), -10);
        assert_eq!(nearest_gain(&[0], 300), 0);
        assert_eq!(nearest_gain(&[], 300), 0);
    }
}
