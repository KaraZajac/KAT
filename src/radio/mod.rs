//! Radio subsystem: HackRF, RTL-SDR and Flipper Zero control.

pub mod demodulator;
pub mod dsp;
mod flipper;
mod hackrf;
pub mod hopper;
pub mod iq_file;
mod modulator;
pub mod pipeline;
/// CC1101 RF presets and the transmit modulation they imply (TX).
pub mod preset;
pub mod recorder;
/// Transmit waveform (OOK / 2-FSK on a digital offset) and the TX sample stream (TX).
pub mod tx;
mod rtlsdr;
pub mod rx;
#[cfg(test)]
mod sim;

pub use demodulator::LevelDuration;

/// What to do about a radio that is plugged in but may not be opened: Windows needs a WinUSB
/// driver for it (Zadig installs one), Linux the udev rules its package provides.
#[cfg(windows)]
pub(crate) const USB_ACCESS_HINT: &str = "install its WinUSB driver with Zadig";
#[cfg(not(windows))]
pub(crate) const USB_ACCESS_HINT: &str = "check its USB permissions / udev rules";
pub use flipper::FlipperController;
pub use hackrf::HackRfController;
pub use rtlsdr::RtlSdrController;

#[allow(unused_imports)]
pub use demodulator::Demodulator;
#[allow(unused_imports)]
pub use demodulator::FmDemodulator;
#[allow(unused_imports)]
pub use modulator::Modulator;
