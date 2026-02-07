//! Common utilities for protocol implementations.

/// Decoded signal information
#[derive(Debug, Clone)]
pub struct DecodedSignal {
    /// Serial number / device ID
    pub serial: Option<u32>,
    /// Button code
    pub button: Option<u8>,
    /// Rolling counter
    pub counter: Option<u16>,
    /// CRC is valid
    pub crc_valid: bool,
    /// Raw data (up to 64 bits)
    pub data: u64,
    /// Number of bits in data
    pub data_count_bit: usize,
    /// Whether encoding is supported
    pub encoder_capable: bool,
}

impl DecodedSignal {
    #[allow(dead_code)]
    pub fn new(data: u64, bit_count: usize) -> Self {
        Self {
            serial: None,
            button: None,
            counter: None,
            crc_valid: false,
            data,
            data_count_bit: bit_count,
            encoder_capable: false,
        }
    }
}

/// CRC8 calculation with custom polynomial
/// 
/// # Arguments
/// * `data` - Data bytes to calculate CRC over
/// * `poly` - CRC polynomial
/// * `init` - Initial CRC value
pub fn crc8(data: &[u8], poly: u8, init: u8) -> u8 {
    let mut crc = init;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// CRC8 for Kia protocol (polynomial 0x7F, init 0x00)
pub fn crc8_kia(data: &[u8]) -> u8 {
    crc8(data, 0x7F, 0x00)
}

/// Add a bit to the decoder's data accumulator
#[inline]
pub fn add_bit(data: &mut u64, count: &mut usize, bit: bool) {
    *data = (*data << 1) | (bit as u64);
    *count += 1;
}

/// Button names for common keyfob buttons
#[allow(dead_code)]
pub fn get_button_name(btn: u8) -> &'static str {
    match btn {
        0x01 => "Lock",
        0x02 => "Unlock",
        0x03 => "Lock+Unlock",
        0x04 => "Trunk",
        0x08 => "Panic",
        _ => "Unknown",
    }
}

/// Button code constants
#[allow(dead_code)]
pub mod buttons {
    pub const LOCK: u8 = 0x01;
    pub const UNLOCK: u8 = 0x02;
    pub const TRUNK: u8 = 0x04;
    pub const PANIC: u8 = 0x08;
}
