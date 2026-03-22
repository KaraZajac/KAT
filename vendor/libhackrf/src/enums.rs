pub enum DeviceType {
    Jellybean = 0,
    Jawbreaker = 1,
    /// HackRF One prior to r9
    Hackrf1Og = 2,
    Rad1O = 3,
    Hackrf1R9 = 4,
    /// Tried detection but did not recognize board
    Unrecognized = 0xFE,
    /// detection not yet attempted
    Undetected = 0xFF,
}

impl DeviceType {
    pub fn from_id(id: u8) -> Self {
        match id {
            0 => DeviceType::Jellybean,
            1 => DeviceType::Jawbreaker,
            2 => DeviceType::Hackrf1Og,
            3 => DeviceType::Rad1O,
            4 => DeviceType::Hackrf1R9,
            0xFE => DeviceType::Unrecognized,
            0xFF => DeviceType::Undetected,
            _ => DeviceType::Undetected,
        }
    }
}
