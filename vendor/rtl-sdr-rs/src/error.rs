// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::{error, fmt, result};

/// A result of a function that may return a `Error`.
pub type Result<T> = result::Result<T, RtlsdrError>;

#[derive(Debug)]
pub enum RtlsdrError {
    Usb(rusb::Error),
    RtlsdrErr(String),
    /// No tuner this library has a driver for answered on the dongle's I2C bus (upstream panics:
    /// "Failed to find tuner, aborting").
    UnknownTuner,
}

impl RtlsdrError {
    /// Returns true when the underlying USB transfer timed out.
    pub fn is_timeout(&self) -> bool {
        matches!(self, RtlsdrError::Usb(rusb::Error::Timeout))
    }

    /// Construct a USB timeout error without requiring callers to depend on `rusb` directly.
    pub fn timeout() -> Self {
        RtlsdrError::Usb(rusb::Error::Timeout)
    }
}

impl fmt::Display for RtlsdrError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            RtlsdrError::Usb(err) => err.fmt(f),
            RtlsdrError::RtlsdrErr(err) => err.fmt(f),
            RtlsdrError::UnknownTuner => f.write_str("No supported tuner found"),
        }
    }
}

impl From<rusb::Error> for RtlsdrError {
    fn from(e: rusb::Error) -> Self {
        RtlsdrError::Usb(e)
    }
}

impl From<String> for RtlsdrError {
    fn from(e: String) -> Self {
        RtlsdrError::RtlsdrErr(e)
    }
}

impl error::Error for RtlsdrError {
    fn source(&self) -> Option<&(dyn error::Error + 'static)> {
        // A USB error's text is this error's own text (Display above), so it is no cause behind
        // it: a chain printed whole said it twice ("Failed to open RTL-SDR …: Pipe error: Pipe
        // error"). The `Usb` variant still holds it.
        None
    }
}
