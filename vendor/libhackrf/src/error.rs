use std::{
    error::Error,
    fmt::{self, Display},
};

pub type Result<T> = std::result::Result<T, HackrfError>;

#[derive(Debug)]
pub enum HackrfError {
    InvalidParam = -2,
    NotFound = -5,
    Busy = -6,
    NoMem = -11,
    Libusb = -1000,
    Thread = -1001,
    StreamingThreadErr = -1002,
    StreamingStopped = -1003,
    StreamingExitCalled = -1004,
    Other = -9999,
}

impl HackrfError {
    pub fn from_id(id: i32) -> Result<()> {
        Err(match id {
            0 | 1 => return Ok(()),
            -2 => HackrfError::InvalidParam,
            -5 => HackrfError::NotFound,
            -6 => HackrfError::Busy,
            -11 => HackrfError::NoMem,
            -1000 => HackrfError::Libusb,
            -1001 => HackrfError::Thread,
            -1002 => HackrfError::StreamingThreadErr,
            -1003 => HackrfError::StreamingStopped,
            -1004 => HackrfError::StreamingExitCalled,
            -9999 => HackrfError::Other,
            _ => HackrfError::Other,
        })
    }
}

impl Display for HackrfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_fmt(format_args!("{self:?}"))
    }
}

impl Error for HackrfError {}
