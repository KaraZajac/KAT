//! The helpers of rtl_433's `src/bit_util.c` the ports call.

/// `reverse8`.
pub fn reverse8(x: u8) -> u8 {
    x.reverse_bits()
}

/// `xor_bytes`.
pub fn xor_bytes(message: &[u8]) -> u8 {
    message.iter().fold(0, |acc, &x| acc ^ x)
}

/// `crc16`: MSB-first CRC-16 over `message`.
pub fn crc16(message: &[u8], polynomial: u16, init: u16) -> u16 {
    let mut remainder = init;
    for &byte in message {
        remainder ^= u16::from(byte) << 8;
        for _ in 0..8 {
            remainder = if remainder & 0x8000 != 0 { (remainder << 1) ^ polynomial } else { remainder << 1 };
        }
    }
    remainder
}

/// `crc16lsb`: LSB-first (reflected) CRC-16 over `message`.
pub fn crc16lsb(message: &[u8], polynomial: u16, init: u16) -> u16 {
    let mut remainder = init;
    for &byte in message {
        remainder ^= u16::from(byte);
        for _ in 0..8 {
            remainder = if remainder & 1 != 0 { (remainder >> 1) ^ polynomial } else { remainder >> 1 };
        }
    }
    remainder
}

/// `ibm_whitening`: XOR with the IBM PN9 sequence (seed 0x1FF), in place.
pub fn ibm_whitening(buffer: &mut [u8]) {
    let mut key_msb: u8 = 0x01;
    let mut key_lsb: u8 = 0xFF;
    for b in buffer.iter_mut() {
        *b ^= key_lsb;
        for _ in 0..8 {
            let key_msb_previous = key_msb;
            key_msb = (key_lsb & 0x01) ^ ((key_lsb >> 5) & 0x01);
            key_lsb = (key_lsb >> 1) | ((key_msb_previous << 7) & 0x80);
        }
    }
}

/// `extract_bytes_uart_8n1`: UART 8N1 bytes (start 0, 8 data bits LSB first, stop 1) from
/// `num_bits` bits of `message` at `offset_bits`, until a framing error. Returns the number of
/// bytes written to `dst`.
pub fn extract_bytes_uart_8n1(message: &[u8], mut offset_bits: usize, mut num_bits: usize, dst: &mut [u8]) -> usize {
    let at = |i: usize| u32::from(message.get(i).copied().unwrap_or(0));
    let mut ret = 0;
    while num_bits >= 10 {
        let startb = at(offset_bits / 8) >> (7 - (offset_bits % 8));
        offset_bits += 1;
        let mut datab = at(offset_bits / 8);
        if !offset_bits.is_multiple_of(8) {
            datab = (at(offset_bits / 8) << 8) | at(offset_bits / 8 + 1);
            datab >>= 8 - (offset_bits % 8);
        }
        offset_bits += 8;
        let stopb = at(offset_bits / 8) >> (7 - (offset_bits % 8));
        offset_bits += 1;
        if startb & 1 != 0 {
            break; // start-bit error
        }
        if stopb & 1 != 1 {
            break; // stop-bit error
        }
        let Some(d) = dst.get_mut(ret) else { break };
        *d = reverse8((datab & 0xFF) as u8);
        ret += 1;
        num_bits -= 10;
    }
    ret
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn helpers_match_the_c_self_tests() {
        // `src/bit_util.c` self-tests.
        let mut buf = [0u8; 16];
        ibm_whitening(&mut buf);
        assert_eq!(buf, [0xff, 0xe1, 0x1d, 0x9a, 0xed, 0x85, 0x33, 0x24, 0xea, 0x7a, 0xd2, 0x39, 0x70, 0x97, 0x57, 0x0a]);
        let uart = [0x7f, 0xd9, 0x90];
        let mut out = [0u8; 6];
        assert_eq!(extract_bytes_uart_8n1(&uart, 0, 24, &mut out), 2);
        assert_eq!(&out[..2], &[0xff, 0x33]);
        let uart123 = [0x07, 0xfd, 0x99, 0x40, 0x48, 0x16, 0x04, 0x00];
        assert_eq!(extract_bytes_uart_8n1(&uart123, 4, 60, &mut out), 5);
        assert_eq!(&out[..5], &[0xff, 0x33, 0x01, 0x02, 0x03]);
        // CRC-16/CMS check value ("123456789").
        assert_eq!(crc16(b"123456789", 0x8005, 0xffff), 0xaee7);
        // CRC-16/KERMIT (reflected 0x1021): check 0x2189.
        assert_eq!(crc16lsb(b"123456789", 0x8408, 0x0000), 0x2189);
        assert_eq!((reverse8(0x01), xor_bytes(&[0x0F, 0xF0, 0x33])), (0x80, 0xCC));
    }
}
