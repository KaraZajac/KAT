//! rtl_433's `bitbuffer_t` and the `bitbuffer_*` functions the ports call (`src/bitbuffer.c`,
//! `include/bitbuffer.h`, default build): up to 50 rows of MSB-first bits in one block of 50 ×
//! 128 bytes. A row longer than 1024 bits spills into the storage of the next rows (`free_row`),
//! and past 50 rows a new row only clears the last row's length (new bits are OR'ed into its old
//! bytes), exactly as in the C.

/// `BITBUF_COLS`: bytes per row (1024 bits).
pub const BITBUF_COLS: usize = 128;
/// `BITBUF_ROWS`.
pub const BITBUF_ROWS: usize = 50;
const BITBUF_BYTES: usize = BITBUF_ROWS * BITBUF_COLS;

/// `bitrow_get_bit`: bit `bit` of an MSB-first byte string (0 past its end).
pub fn bitrow_get_bit(row: &[u8], bit: usize) -> bool {
    row.get(bit >> 3).is_some_and(|&b| b >> (7 - (bit & 7)) & 1 == 1)
}

/// rtl_433's `bitbuffer_t`.
#[derive(Clone)]
pub struct BitBuffer {
    /// Number of active rows.
    num_rows: u16,
    /// Index of the next free row slot (a spilled row takes more than one).
    free_row: u16,
    /// Active bits per row.
    bits_per_row: [u16; BITBUF_ROWS],
    /// Sync pulses before each row.
    syncs_before_row: [u16; BITBUF_ROWS],
    /// The rows, contiguous like the C's `bitarray_t` (a spilled row continues into the next slot).
    bb: Vec<u8>,
}

impl Default for BitBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for BitBuffer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let rows: Vec<String> = (0..self.num_rows()).map(|r| self.code(r)).collect();
        f.debug_struct("BitBuffer").field("rows", &rows).finish()
    }
}

impl BitBuffer {
    pub fn new() -> Self {
        Self {
            num_rows: 0,
            free_row: 0,
            bits_per_row: [0; BITBUF_ROWS],
            syncs_before_row: [0; BITBUF_ROWS],
            bb: vec![0; BITBUF_BYTES],
        }
    }

    /// `bitbuffer_clear`. Bits and counts are only ever written into the first `free_row` row
    /// slots, so only those are zeroed (the C clears the whole buffer: same result).
    pub fn clear(&mut self) {
        let used = usize::from(self.free_row).min(BITBUF_ROWS);
        self.bb[..used * BITBUF_COLS].fill(0);
        self.bits_per_row[..used].fill(0);
        self.syncs_before_row[..used].fill(0);
        self.num_rows = 0;
        self.free_row = 0;
    }

    /// `num_rows`.
    #[inline]
    pub fn num_rows(&self) -> usize {
        usize::from(self.num_rows)
    }

    /// `bits_per_row[row]` (0 past the last row slot).
    #[inline]
    pub fn bits_per_row(&self, row: usize) -> usize {
        self.bits_per_row.get(row).map_or(0, |&n| usize::from(n))
    }

    /// `syncs_before_row[row]` (0 past the last row slot).
    #[cfg(test)]
    pub fn syncs_before_row(&self, row: usize) -> usize {
        self.syncs_before_row.get(row).map_or(0, |&n| usize::from(n))
    }

    /// `bb[row]`: the row's bytes, followed by the storage it may have spilled into.
    pub fn row(&self, row: usize) -> &[u8] {
        &self.bb[(row * BITBUF_COLS).min(BITBUF_BYTES)..]
    }

    /// Byte `i` of row `row` (`bb[row][i]`, which runs on into the next rows like the C array);
    /// 0 past the buffer.
    pub fn byte(&self, row: usize, i: usize) -> u8 {
        self.bb.get(row * BITBUF_COLS + i).copied().unwrap_or(0)
    }

    /// The first `N` bytes of row `row`.
    pub fn bytes<const N: usize>(&self, row: usize) -> [u8; N] {
        std::array::from_fn(|i| self.byte(row, i))
    }

    /// Bit `bit` of row `row`.
    pub fn bit(&self, row: usize, bit: usize) -> bool {
        self.byte(row, bit >> 3) >> (7 - (bit & 7)) & 1 == 1
    }

    /// The row as rtl_433 prints a code: `{bits}hex` (what `rtl_433 -y` reads).
    pub fn code(&self, row: usize) -> String {
        let n = self.bits_per_row(row);
        let hex: String = (0..n.div_ceil(8)).map(|i| format!("{:02x}", self.byte(row, i))).collect();
        format!("{{{n}}}{hex}")
    }

    /// `bitbuffer_add_bit`.
    #[inline]
    pub fn add_bit(&mut self, bit: bool) {
        if self.num_rows == 0 {
            self.free_row = 1;
            self.num_rows = 1;
        }
        let row = usize::from(self.num_rows) - 1;
        let n = self.bits_per_row[row];
        if n == u16::MAX {
            return;
        }
        if n > 0 && usize::from(n).is_multiple_of(BITBUF_COLS * 8) {
            // Spill into the next row slot.
            if usize::from(self.free_row) < BITBUF_ROWS {
                self.free_row += 1;
            } else {
                return;
            }
        }
        if let Some(b) = self.bb.get_mut(row * BITBUF_COLS + usize::from(n / 8)) {
            *b |= u8::from(bit) << (7 - n % 8);
        }
        self.bits_per_row[row] += 1;
    }

    /// `n` × [Self::add_bit], stopping once the row takes no more bits (every further call would
    /// be a no-op: the row is at `UINT16_MAX` bits, or its storage is exhausted), so absurdly long
    /// pulses cost nothing.
    pub fn add_bits(&mut self, bit: bool, n: i32) {
        let mut n = n.max(0) as usize;
        while n > 0 {
            if self.num_rows == 0 {
                self.add_bit(bit);
                n -= 1;
                continue;
            }
            let row = usize::from(self.num_rows) - 1;
            let cur = usize::from(self.bits_per_row[row]);
            let at_spill = cur > 0 && cur.is_multiple_of(BITBUF_COLS * 8);
            if !cur.is_multiple_of(8) || n < 8 || at_spill {
                self.add_bit(bit);
                if usize::from(self.bits_per_row[row]) == cur {
                    return; // full: every further add is a no-op
                }
                n -= 1;
                continue;
            }
            // Byte aligned and inside a storage row: whole bytes up to the next spill boundary.
            let to_spill = BITBUF_COLS * 8 - cur % (BITBUF_COLS * 8);
            let room = usize::from(u16::MAX) - cur;
            let k = n.min(to_spill).min(room) / 8 * 8;
            if k == 0 {
                self.add_bit(bit);
                if usize::from(self.bits_per_row[row]) == cur {
                    return;
                }
                n -= 1;
                continue;
            }
            if bit {
                let start = row * BITBUF_COLS + cur / 8;
                for b in self.bb.iter_mut().skip(start).take(k / 8) {
                    *b = 0xFF;
                }
            }
            self.bits_per_row[row] += k as u16;
            n -= k;
        }
    }

    /// `bitbuffer_add_row`.
    pub fn add_row(&mut self) {
        if self.num_rows == 0 {
            self.free_row = 1;
            self.num_rows = 1;
        }
        if usize::from(self.free_row) < BITBUF_ROWS {
            self.free_row += 1;
            self.num_rows = self.free_row;
        } else {
            // "Clear last row to handle overflow somewhat gracefully": only its length, new bits
            // are OR'ed into the old bytes, as in the C.
            self.bits_per_row[usize::from(self.num_rows) - 1] = 0;
        }
    }

    /// `bitbuffer_add_sync`.
    pub fn add_sync(&mut self) {
        if self.num_rows == 0 {
            self.free_row = 1;
            self.num_rows = 1;
        }
        if self.bits_per_row[usize::from(self.num_rows) - 1] != 0 {
            self.add_row();
        }
        let row = usize::from(self.num_rows) - 1;
        self.syncs_before_row[row] = self.syncs_before_row[row].wrapping_add(1);
    }

    /// `bitbuffer_invert` (the unused bits of each row's last byte stay as they are: the C XORs
    /// them back, and a full last byte is left alone).
    pub fn invert(&mut self) {
        for row in 0..self.num_rows() {
            let n = self.bits_per_row(row);
            if n == 0 {
                continue;
            }
            let last_col = (n - 1) / 8;
            let last_bits = (n - 1) % 8 + 1;
            for col in 0..=last_col {
                if let Some(b) = self.bb.get_mut(row * BITBUF_COLS + col) {
                    *b = !*b;
                }
            }
            if let Some(b) = self.bb.get_mut(row * BITBUF_COLS + last_col) {
                *b ^= (0xFFu32 >> last_bits) as u8;
            }
        }
    }

    /// `bitbuffer_extract_bytes`: `len` bits of row `row` from bit `pos` into `out`, MSB first (the
    /// unused low bits of the last byte cleared). Bytes past the row are the following rows' (as
    /// in the C), past the buffer 0.
    pub fn extract_bytes(&self, row: usize, pos: usize, out: &mut [u8], len: usize) {
        if len == 0 {
            return;
        }
        let nbytes = len.div_ceil(8);
        if pos & 7 == 0 {
            for (i, o) in out.iter_mut().take(nbytes).enumerate() {
                *o = self.byte(row, pos / 8 + i);
            }
        } else {
            let shift = 8 - (pos & 7);
            let mut p = pos >> 3;
            let mut word = u16::from(self.byte(row, p));
            for o in out.iter_mut().take(nbytes) {
                word <<= 8;
                p += 1;
                word |= u16::from(self.byte(row, p));
                *o = (word >> shift) as u8;
            }
        }
        if len & 7 != 0 {
            if let Some(o) = out.get_mut((len - 1) / 8) {
                *o &= (0xff00u16 >> (len & 7)) as u8;
            }
        }
    }

    /// `bitbuffer_search`: the first bit position ≥ `start` in row `row` where the first
    /// `pattern_bits` bits of `pattern` occur, or the row's length when they do not (the C's naive
    /// search, kept).
    pub fn search(&self, row: usize, start: usize, pattern: &[u8], pattern_bits: usize) -> usize {
        let len = self.bits_per_row(row);
        let (mut ipos, mut ppos) = (start, 0);
        while ipos < len && ppos < pattern_bits {
            if self.bit(row, ipos) == bitrow_get_bit(pattern, ppos) {
                ppos += 1;
                ipos += 1;
                if ppos == pattern_bits {
                    return ipos - pattern_bits;
                }
            } else {
                ipos -= ppos;
                ipos += 1;
                ppos = 0;
            }
        }
        len
    }

    /// `bitbuffer_compare_rows`: equal rows (or, when both are at least `max_bits` long, equal
    /// first `max_bits` bits).
    pub fn compare_rows(&self, a: usize, b: usize, max_bits: usize) -> bool {
        let (la, lb) = (self.bits_per_row(a), self.bits_per_row(b));
        if max_bits == 0 || la < max_bits || lb < max_bits {
            la == lb && (0..la.div_ceil(8)).all(|i| self.byte(a, i) == self.byte(b, i))
        } else {
            let last = (max_bits - 1) / 8;
            let mask = (0xff00u32 >> (max_bits & 7)) as u8;
            (0..max_bits / 8).all(|i| self.byte(a, i) == self.byte(b, i)) && self.byte(a, last) & mask == self.byte(b, last) & mask
        }
    }

    /// `bitbuffer_count_repeats` (the row itself counts).
    pub fn count_repeats(&self, row: usize, max_bits: usize) -> usize {
        (0..self.num_rows()).filter(|&i| self.compare_rows(row, i, max_bits)).count()
    }

    /// `bitbuffer_find_repeated_row`: the first row of at least `min_bits` bits that occurs at
    /// least `min_repeats` times (counting itself).
    pub fn find_repeated_row(&self, min_repeats: usize, min_bits: usize) -> Option<usize> {
        (0..self.num_rows()).find(|&i| self.bits_per_row(i) >= min_bits && self.count_repeats(i, 0) >= min_repeats)
    }

    /// `bitbuffer_manchester_decode`: decode the bit pairs of row `row` from `start` into `out`
    /// (01 → 1, 10 → 0: the pair's second bit), at most `max` output bits (0 = no limit); stops at
    /// the first invalid pair (00 or 11). Returns the input position where decoding stopped.
    pub fn manchester_decode(&self, row: usize, start: usize, out: &mut BitBuffer, max: usize) -> usize {
        let mut len = self.bits_per_row(row);
        let mut ipos = start;
        if max != 0 && len > start + max * 2 {
            len = start + max * 2;
        }
        while ipos < len {
            let bit1 = self.bit(row, ipos);
            ipos += 1;
            let bit2 = self.bit(row, ipos);
            ipos += 1;
            if bit1 == bit2 {
                break;
            }
            out.add_bit(bit2);
        }
        ipos
    }

    /// `bitbuffer_set_width` (for [Self::parse]).
    #[cfg(test)]
    fn set_width(&mut self, width: usize) {
        if self.num_rows == 0 {
            self.num_rows = 1;
            self.free_row = 1;
        }
        let remaining_rows = BITBUF_ROWS - usize::from(self.num_rows) + 1;
        let width = width.min(remaining_rows * BITBUF_COLS * 8).min(usize::from(u16::MAX));
        let row = usize::from(self.num_rows) - 1;
        let cur = usize::from(self.bits_per_row[row]);
        if cur > width {
            let base = row * BITBUF_COLS;
            let clr_from = width.div_ceil(8);
            let clr_end = cur.div_ceil(8);
            for b in &mut self.bb[base + clr_from..(base + clr_end).min(BITBUF_BYTES)] {
                *b = 0;
            }
            if let Some(b) = self.bb.get_mut(base + width / 8) {
                *b &= (0xFF00u16 >> (width % 8)) as u8;
            }
        }
        self.bits_per_row[row] = width as u16;
        let extra_rows = if width == 0 { 0 } else { (width - 1) / (BITBUF_COLS * 8) };
        self.free_row = self.num_rows + extra_rows as u16;
    }

    /// `bitbuffer_parse`: rows from rtl_433's code notation, e.g. `{25}fb2dd58` or
    /// `{12}fff/{66}2066…` (what `rtl_433 -y` reads).
    #[cfg(test)]
    pub fn parse(code: &str) -> Self {
        let mut bits = Self::new();
        let mut width: Option<usize> = None;
        // The C keeps the last nibble for a character that is not a hex digit.
        let mut data = 0u8;
        let chars: Vec<char> = code.chars().collect();
        let mut i = 0;
        while i < chars.len() {
            let c = chars[i];
            if c == ' ' {
                i += 1;
                continue;
            }
            if c == '0' && matches!(chars.get(i + 1), Some('x') | Some('X')) {
                i += 2;
                continue;
            }
            if c == '{' {
                if let Some(w) = width {
                    bits.set_width(w);
                }
                if bits.num_rows > 0 {
                    bits.add_row();
                }
                let mut j = i + 1;
                let mut num = String::new();
                while j < chars.len() && chars[j].is_ascii_digit() {
                    num.push(chars[j]);
                    j += 1;
                }
                width = Some(num.parse::<usize>().unwrap_or(0).min(BITBUF_BYTES * 8));
                while j < chars.len() && chars[j] != '}' {
                    j += 1;
                }
                i = j + 1;
                continue;
            }
            if c == '/' {
                if let Some(w) = width.take() {
                    bits.set_width(w);
                }
                bits.add_row();
                i += 1;
                continue;
            }
            if let Some(v) = c.to_digit(16) {
                data = v as u8;
            }
            for shift in [3, 2, 1, 0] {
                bits.add_bit((data >> shift) & 1 == 1);
            }
            i += 1;
        }
        if let Some(w) = width {
            bits.set_width(w);
        }
        bits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn from_bytes(bytes: &[u8], n: usize) -> BitBuffer {
        let mut b = BitBuffer::new();
        for i in 0..n {
            b.add_bit(bitrow_get_bit(bytes, i));
        }
        b
    }

    /// Rows, syncs, search, extraction and inversion against hand-worked C results.
    #[test]
    fn rows_syncs_search_extract_and_invert_like_the_c() {
        let mut b = BitBuffer::new();
        for bit in [1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1] {
            b.add_bit(bit == 1);
        }
        assert_eq!((b.num_rows(), b.bits_per_row(0)), (1, 12));
        assert_eq!(b.code(0), "{12}b2f0");
        let mut out = [0u8; 2];
        b.extract_bytes(0, 3, &mut out, 9);
        // bits 3..12: 1 0 0 1 0 1 1 1 1 → 0x97 0x80
        assert_eq!(out, [0x97, 0x80]);
        assert_eq!(b.search(0, 0, &[0b1001_0000], 4), 3);
        assert_eq!(b.search(0, 0, &[0b0000_0000], 4), 12, "not found → row length");
        b.add_sync(); // row 0 has bits: a new row
        assert_eq!((b.num_rows(), b.syncs_before_row(1)), (2, 1));
        b.add_sync(); // empty row: only counted
        assert_eq!((b.num_rows(), b.syncs_before_row(1)), (2, 2));
        b.add_bit(true);
        assert_eq!(b.bits_per_row(1), 1);
        b.invert();
        assert_eq!((b.code(0), b.code(1)), ("{12}4d00".to_string(), "{1}00".to_string()), "the unused bits of the last byte stay 0");
        // A row of whole bytes: every byte inverted.
        let mut w = BitBuffer::parse("{16}a50f");
        w.invert();
        assert_eq!(&w.row(0)[..3], &[0x5A, 0xF0, 0x00]);
        // Extraction: aligned, unaligned, past the row (zero bytes, then the length mask).
        let b = from_bytes(&[0x12, 0x34, 0x56, 0x78, 0x9A], 40);
        let mut out = [0u8; 3];
        b.extract_bytes(0, 0, &mut out, 20);
        assert_eq!(out, [0x12, 0x34, 0x50]);
        b.extract_bytes(0, 4, &mut out, 24);
        assert_eq!(out, [0x23, 0x45, 0x67]);
        b.extract_bytes(0, 33, &mut out, 11);
        assert_eq!(out[..2], [0x34, 0x00]);
        // Search: first match, the row length when absent, a pattern straddling a retry.
        assert_eq!(b.search(0, 0, &[0x45, 0x60], 12), 12);
        assert_eq!(b.search(0, 13, &[0x45, 0x60], 12), 40);
        let b = from_bytes(&[0xF0, 0xF0, 0xF0, 0x60], 32);
        assert_eq!(b.search(0, 0, &[0xF0, 0xF0, 0x60], 20), 8);
    }

    #[test]
    fn compare_and_find_repeated_rows_like_the_c() {
        let mut c = BitBuffer::new();
        for v in [0xABu8, 0xCD, 0xE0] {
            for i in (0..8).rev() {
                c.add_bit(v >> i & 1 == 1);
            }
        }
        c.add_row();
        for v in [0xABu8, 0xCD, 0xFF] {
            for i in (0..8).rev() {
                c.add_bit(v >> i & 1 == 1);
            }
        }
        assert!(c.compare_rows(0, 1, 19));
        assert!(!c.compare_rows(0, 1, 20));
        assert!(!c.compare_rows(0, 1, 0));
        // find_repeated_row counts the row itself.
        let mut b = BitBuffer::new();
        b.add_bits(true, 40);
        b.add_row();
        b.add_bits(false, 50);
        assert_eq!(b.find_repeated_row(1, 48), Some(1));
        assert_eq!(b.find_repeated_row(2, 48), None);
        assert_eq!(b.find_repeated_row(1, 30), Some(0));
    }

    /// Row limits: 50 rows, then a new row clears the last one's length; a long row spills into
    /// the next rows' storage (and the next row starts behind it).
    #[test]
    fn row_overflow_and_spill_like_the_c() {
        let mut b = BitBuffer::new();
        for _ in 0..60 {
            b.add_bit(true);
            b.add_row();
        }
        assert_eq!(b.num_rows(), BITBUF_ROWS);
        assert_eq!(b.bits_per_row(BITBUF_ROWS - 1), 0);
        let mut s = BitBuffer::new();
        for i in 0..1030 {
            s.add_bit(i % 3 == 0);
        }
        assert_eq!((s.num_rows(), s.bits_per_row(0)), (1, 1030));
        let spilled = (1024..1030).fold(0u8, |acc, i| acc | u8::from(i % 3 == 0) << (7 - (i - 1024)));
        assert_eq!(s.row(1)[0], spilled, "spilled bits in row 1's bytes");
        assert_eq!(s.row(0)[128], spilled, "row 0 reads on into them");
        assert!(s.bit(0, 1026) == (1026 % 3 == 0));
        s.add_row();
        s.add_bit(true);
        assert_eq!((s.num_rows(), s.bits_per_row(1), s.bits_per_row(2)), (3, 0, 1), "the spilled slot counts as a row");
        s.clear();
        assert!(s.row(0)[..BITBUF_COLS * 3].iter().all(|&x| x == 0) && s.num_rows() == 0);
    }

    #[test]
    fn add_bits_equals_add_bit_and_stops_when_the_row_is_full() {
        let mut b = BitBuffer::new();
        b.add_bits(true, i32::MAX);
        assert_eq!(b.bits_per_row(0), BITBUF_ROWS * BITBUF_COLS * 8);
        let mut c = BitBuffer::new();
        for _ in 0..BITBUF_ROWS * BITBUF_COLS * 8 + 10 {
            c.add_bit(true);
        }
        assert_eq!((c.bits_per_row(0), c.free_row), (b.bits_per_row(0), b.free_row));
        assert_eq!(b.bb, c.bb);
        // Mixed runs across spill boundaries and rows: byte-wise == bit by bit.
        let mut x = BitBuffer::new();
        let mut y = BitBuffer::new();
        let mut seed = 12345u32;
        for i in 0..400 {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let n = (seed >> 16) % 3000;
            let bit = i % 2 == 1;
            x.add_bits(bit, n as i32);
            for _ in 0..n {
                y.add_bit(bit);
            }
            if seed.is_multiple_of(7) {
                x.add_row();
                y.add_row();
            }
        }
        assert_eq!((x.num_rows, x.free_row, x.bits_per_row), (y.num_rows, y.free_row, y.bits_per_row));
        assert_eq!(x.bb, y.bb);
    }

    #[test]
    fn parse_reads_rtl_433_codes() {
        let b = BitBuffer::parse("{12}fff{66}2066 6767 b352 e802 40");
        assert_eq!(b.num_rows(), 2);
        assert_eq!((b.bits_per_row(0), b.bits_per_row(1)), (12, 66));
        assert_eq!(&b.row(0)[..2], &[0xFF, 0xF0]);
        assert_eq!(&b.row(1)[..9], &[0x20, 0x66, 0x67, 0x67, 0xB3, 0x52, 0xE8, 0x02, 0x40]);
        let b = BitBuffer::parse("{3}ff");
        assert_eq!((b.bits_per_row(0), b.row(0)[0]), (3, 0xE0));
    }

    #[test]
    fn manchester_decode_stops_at_an_invalid_pair() {
        let b = BitBuffer::parse("{12}9a4");
        let mut out = BitBuffer::new();
        // 10 01 10 10 01 00 → 0 1 0 0 1, stop at 00.
        assert_eq!(b.manchester_decode(0, 0, &mut out, 0), 12);
        assert_eq!(out.bits_per_row(0), 5);
        assert_eq!(out.row(0)[0], 0b0100_1000);
    }
}
