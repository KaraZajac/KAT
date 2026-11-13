//! Renault V1 protocol decoder/encoder (PCF79xx HITAG2 RKE frame behind a 1500/1000 µs header).
//!
//! Aligned with the CURRENT ProtoPirate `REFERENCES/ProtoPirate/protocols/renault_v1.c` (added in
//! 1287d10 "move newer Renault variants into Renault V1"; later upstream commits only refactor it),
//! the primary reference for the decoder, the seed brute force and the seed-path encoder.
//! doctor-firmware's Renault V1 (`lib/subghz/protocols/hitag2.c` since 2026-11, 98d1fc10a, which
//! merged its `renault_v1.c`, `fiat_v1.c` and `fiat_v2.c`) is the same reader, run behind its older
//! narrow header windows (1400-2200 / 920-1150 µs) and behind ProtoPirate's, plus a key-less
//! REPLAY, ported here as the fallback TX path (see Encoder), and a rule for which frames are
//! Fiat V1's (see Decoder). Flipper-ARF ships ProtoPirate's file unchanged in
//! its bundled ProtoPirate app; ARF's own firmware `renault_v1.c` is a different decoder for the
//! same name (the old 82-bit type-table frames that ProtoPirate retired in 1287d10), ported
//! separately as "Renault V1 (ARF)" (`renault_v1_arf.rs`).
//!
//! RF: 315 / 433.92 / 868.35 MHz, AM or FM, Manchester with an adaptive TE (nominal 125 µs).
//!
//! ## Decoder (`subghz_protocol_decoder_renault_v1_feed`)
//! Reset → CheckSync on a LOW of 1150-2200 µs; a following HIGH of 800-1150 µs starts Data with
//! the Flipper Manchester machine in `Start1` and per-level TE estimates te_high = 120 /
//! te_low = 150 µs. In Data, pulses ≤ 49 µs are ignored and a pulse > 520 µs returns to Reset and
//! is re-examined there (so the next header is caught). A pulse longer than
//! `hitag2_data_threshold(te)` is a long event, otherwise short and `te` adapts
//! (`hitag2_adapt_te`: (7·te + d)/8 clamped to 70-185 µs). Events are mapped exactly as
//! renault_v1.c does it — `level ? ShortHigh : ShortLow` / `level ? LongHigh : LongLow` (it does
//! not use `pp_manchester_event`, whose mapping is the opposite) — and fed to the canonical
//! Flipper `manchester_advance` (transitions `[0x01, 0x91, 0x9B, 0xFB]`, events 0/2/4/6, entry
//! state `Start1`, reset → `Mid1`; same machine as `common.rs`). The accumulated words are inverted
//! (`~`), so on air bit 1 = HIGH then LOW.
//!
//! **Fiat V1's frames** (doctor-firmware `hitag2.c`, `adaptive_feed`). Fiat V1 type B sends the
//! same 13 bytes behind a 1230 µs gap and an 850 µs lead, a header the windows above take. A frame
//! is Fiat V1's, and this decoder leaves it to `fiat_v1.rs` (which runs the same reader,
//! [`AdaptiveReader`]), when the header's HIGH is at most 949 µs, the header is not the narrow one
//! (LOW of 1400 µs or more with a HIGH of 920 µs or more) and the frame passes Fiat V1's gates
//! (button 1, 2, 4 or 8, a UID that is not all zeros or all ones). Every other frame is Renault
//! V1's. The C's wide-header path also drops every frame whose UID has the hex digit 3 in the
//! second-lowest place (`((key >> 32) & 0xF0) != 0x30`, no reason given); KAT keeps those, as
//! ProtoPirate does.
//!
//! ## Frame (104 Manchester bits, MSB first)
//!   bits 0..16   header, must invert to 0x0001
//!   bits 16..80  "Key"   (64): uid(32) ‖ btn(4) cnt[9:6] ‖ cnt[5:0] hop[31:30] ‖ hop[29:6](24)
//!   bits 80..104 "Key_2" (24): hop[5:0] tail(2) ‖ XOR of the 10 preceding payload bytes
//! (`hitag2_unpack_frame`: serial, btn, cnt10, hop, tail). This is byte-for-byte the same 13-byte
//! frame as Fiat V1 (`00 01 uid btn/cnt … xor`); only the timing/sync differ.
//! Buttons (C names): 0 Sync, 1 Lock, 2 Unlock, 4 Trunk, 8 Panic.
//!
//! Gating: header == 0x0001, XOR byte, duplicate suppression (`hitag2_accept_frame`) — plus a
//! KAT-side UID sanity check (not 0x00000000 / 0xFFFFFFFF, as Fiat V1's C does) so degenerate
//! preamble-like runs cannot pass. The duplicate memory is cleared by `reset()` (the C keeps it
//! across resets; KAT's registry reuses decoders across captures, so a persistent memory would
//! hide re-decoded captures).
//!
//! ## Seed recovery (ProtoPirate "Hitag2 BF", run automatically)
//! Remotes whose HITAG2 key is derived from the serial (`hitag2_serial_permute`) hide only 18 IV
//! bits (counter bits 10..15 + a 12-bit seed). ProtoPirate recovers them with an on-device brute
//! force the user starts from the saved capture (`hitag2_bf_*`, 2^18 candidates, minutes on a
//! Flipper); on success it stores "Recovered: 01" + "Seed" (= the IV) and re-derives `Cnt` as the
//! full 16-bit counter (`hitag2_apply_check_remote`). KAT runs the identical search
//! ([`hitag2::renault_seed_recover`], same candidate order and first match; ≈10 ms release /
//! ≈65 ms debug for a full miss) on every accepted frame (cached per frame) and, on success,
//! reports `counter` = that 16-bit counter. A random (non-seed-scheme) frame passes by chance with
//! probability ≈ 2^-14, exactly as on the Flipper.
//!
//! ## DecodedSignal packing
//!   serial = uid, button = Renault code (matches KAT's 1/2/4/8 codes), counter = cnt10, or the
//!   16-bit counter when the seed was recovered, crc_valid = XOR ok (always true on emission),
//!   data_count_bit = 104, data = wire bits 0..63 (0x0001 header ‖ Key[63:16]), extra = wire
//!   bits 64..103 right-aligned (Key[15:0] ‖ Key_2), encoder_capable = true (see Encoder). The C's
//!   own "Key"/"Key_2"/88-bit view is `data << 16 | extra >> 24` / `extra & 0xFFFFFF`. Same layout
//!   as Fiat V1, so an identical frame packs identically. No recovery state is stored: `encode()`
//!   re-runs the (deterministic) brute force on the frame, so `.fob` files of any KAT version work.
//!
//! ## Encoder
//! `encode()` rebuilds the 11-byte payload from `data`/`extra` (must pass the decoder's gates) and:
//! 1. **Seed recovered** (ProtoPirate `subghz_protocol_encoder_renault_v1_deserialize` with
//!    "Recovered: 01", + the emulate plugin): a FRESH frame `hitag2_encrypt_frame(uid, cnt16 + 1,
//!    btn, seed)` — the counter is the recovered 16-bit counter + 1 (the plugin's
//!    `current_counter++`; `hitag2_build_iv` keeps 16 bits), the button is KAT's Lock/Unlock/
//!    Trunk/Panic = Renault 1/2/4/8 (plugin Up/OK/Left/Right; other codes keep the captured one).
//!    Upload = `renault_v1_encoder_get_upload`: 250 (HIGH te, LOW te) preamble pairs, 3 long frames
//!    [LOW 1500, HIGH 1000, 0x0001 (16 bits), Key (64), Key_2 (24)], 3 short frames [LOW 1500,
//!    HIGH 1000, 0x0001, Key[27:18] (10 bits), LOW 21500]; bit 1 = (HIGH te, LOW te), 0 = (LOW te,
//!    HIGH te), te = 125 µs, adjacent levels merged; C repeat 1. doctor-firmware builds the same
//!    frame (its D-pad Up/Down/Left/Right = 1/2/4/8, counter + 1).
//! 2. **Saved key** (doctor-firmware's UID key database, `hitag2_keys.txt`, where "Recover key
//!    (HiTag2)" saves the keys it finds; `hitag2_keys.rs`): with the epoch that reproduces the
//!    captured hop (`hitag2_hell_find_epoch`: the record's epoch and the next seven), a FRESH
//!    frame as `hitag2_encoder_hell_next_frame` builds it — the requested button (else the
//!    captured one), counter `epoch << 10 | cnt10` + 1, hop
//!    [`hitag2::renault_authenticator`], the captured tail bits — in the upload of 3. below.
//!    ProtoPirate's and doctor-firmware's user-typed "Hitag2 Key" is not read: KAT has no key
//!    entry.
//! 3. **No seed, no key** — doctor-firmware's encoder (`subghz_protocol_encoder_hitag2_deserialize`)
//!    has no failure path without a key or seed: it REPLAYS the captured Key/Key_2 byte-for-byte (the
//!    requested button is ignored) with its own upload `subghz_protocol_encoder_hitag2_get_upload`
//!    in the default "Kangoo" variant: 255 preamble pairs + one HIGH te, 3 long frames, 8 short
//!    frames each closed by LOW 20000 (doctor's "Old" variant — 2 long frames + a 115.5 ms gap —
//!    is chosen from a per-burst repeat count that KAT's per-frame registry cannot observe).
//!    ProtoPirate refuses to transmit here.
//!
//! ## Deviations from C
//! - Emission is delayed by exactly one pulse: the C fires at the mid-bit of the 104th bit, KAT
//!   returns the frame on the next pulse (the one carrying that bit's second half). Fiat V1 —
//!   same frame format, earlier in the registry — completes its window on exactly that pulse, so
//!   when both decode a frame Fiat V1 keeps first-match priority. That is for a frame Fiat V1's
//!   cells read behind a header that is Renault's by the rule above (a lead longer than 949 µs,
//!   or the narrow header); behind Fiat V1's own header this decoder reports nothing.
//! - UID sanity gate and dedup cleared on reset (above). The short 26-bit frames the C encoder
//!   appends are ignored by the C decoder too (they never reach 104 bits).
//! - The seed brute force runs at decode time instead of on user request (same result).
//!
//! ## Key files
//! ProtoPirate `subghz_protocol_decoder_renault_v1_serialize` / `_deserialize`: `Protocol: Renault
//! V1`, `Bit: 88`, `Key` (the C's 64-bit Key = `data << 16 | extra >> 24`), `Key_2` (8 bytes, the
//! 24-bit tail), `Recovered` (1 byte), `Seed` (4 bytes), then `Serial`, `Btn`, `Cnt`, `Hop`
//! (decimal). KAT writes what PP's file holds after its "Hitag2 BF": `Recovered: 01` + the
//! recovered IV as Seed + the 16-bit counter, or `Recovered: 02` (brute force missed) + Seed 0 +
//! the 10-bit counter. Loading needs Bit 88 and a frame that passes the decoder's gates, and
//! re-runs the (deterministic) seed search, so a PP file saved before its brute force loads with
//! the recovered counter too. Without a seed, KAT adds doctor-firmware's "Hitag2 Key" (standard
//! byte order) + "Hitag2 Epoch" when the remote's saved key verifies; on loading, those lines
//! (PP's user-typed key) are not read.
//! Flipper-ARF names its type-table protocol "Renault V1" too (KAT's "Renault V1 (ARF)",
//! `renault_v1_arf.rs`); its keys (`Bit: 82`, decimal `Key2`) are handed to that decoder.

use super::hitag2;
use super::hitag2_keys::{self, KeyOrder};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Renault V1",
    decoder: "Renault V1",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::Both,
    encryption: "HITAG2",
    make: "Renault",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650, &FM476],
};

/// Renault V1 buttons (2 is KAT's generic Unlock).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0 => "Sync",
        _ => return None,
    })
}

/// Nominal half-bit (encoder `HITAG2_TE_US`).
const TE_US: u32 = 125;
const HEADER_LOW_MIN_US: u32 = 1150;
const HEADER_LOW_MAX_US: u32 = 2200;
const HEADER_HIGH_MIN_US: u32 = 800;
const HEADER_HIGH_MAX_US: u32 = 1150;
const DATA_IGNORE_US: u32 = 49;
const DATA_RESET_US: u32 = 520;
const TE_HIGH_INIT_US: u16 = 120;
const TE_LOW_INIT_US: u16 = 150;

/// doctor-firmware's merged decoder (`hitag2.c`, `adaptive_feed`): behind a header whose HIGH is
/// at most this long, a frame that passes Fiat V1's gates is Fiat V1's (type B sends a 1230 µs
/// gap and an 850 µs lead; Renault's header is LOW 1500, HIGH 1000).
const FIAT_HEADER_HIGH_MAX_US: u32 = 949;
/// Its narrow header (doctor-firmware's Renault V1 windows before the merge), Renault's whatever
/// the frame holds.
const NARROW_HEADER_LOW_MIN_US: u32 = 1400;
const NARROW_HEADER_HIGH_MIN_US: u32 = 920;

const HEADER_BITS: u8 = 16;
const KEY_END_BITS: u8 = 80;
const LONG_FRAME_BITS: u8 = 104;
const WIRE_BITS: usize = LONG_FRAME_BITS as usize;

/// Encoder (`renault_v1_encoder_get_upload`): header LOW/HIGH, short-frame payload width.
const ENC_HEADER_LOW_US: u32 = 1500;
const ENC_HEADER_HIGH_US: u32 = 1000;
const ENC_SHORT_KEY_BITS: u32 = 10;

/// One upload layout: preamble (HIGH te, LOW te) pairs, optional trailing HIGH te, long and
/// short frame counts, LOW after each short frame.
struct UploadShape {
    preamble_pairs: usize,
    preamble_tail_high: bool,
    long_frames: usize,
    short_frames: usize,
    short_gap_us: u32,
}

/// ProtoPirate `renault_v1_encoder_get_upload` (HITAG2_PREAMBLE_PAIRS 250, HITAG2_LONG_FRAMES 3,
/// HITAG2_SHORT_FRAMES 3, HITAG2_SHORT_GAP_US 21500).
const PP_UPLOAD: UploadShape = UploadShape {
    preamble_pairs: 250,
    preamble_tail_high: false,
    long_frames: 3,
    short_frames: 3,
    short_gap_us: 21_500,
};

/// doctor-firmware `subghz_protocol_encoder_hitag2_get_upload`, default "Kangoo" variant
/// (HITAG2_PREAMBLE_PAIRS 255 + one HIGH te, HITAG2_KANGOO_LONG_FRAMES 3, HITAG2_SHORT_FRAMES 8,
/// HITAG2_SHORT_GAP_US 20000, no long-to-short gap).
const DOCTOR_KANGOO_UPLOAD: UploadShape = UploadShape {
    preamble_pairs: 255,
    preamble_tail_high: true,
    long_frames: 3,
    short_frames: 8,
    short_gap_us: 20_000,
};

/// Flipper `ManchesterEvent` values (`lib/toolbox/manchester_decoder.h`).
const EVENT_SHORT_LOW: u8 = 0;
const EVENT_SHORT_HIGH: u8 = 2;
const EVENT_LONG_LOW: u8 = 4;
const EVENT_LONG_HIGH: u8 = 6;

/// Flipper `ManchesterState` (canonical numbering, used as the transition-table index).
#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Start1 = 0,
    Mid1 = 1,
    Mid0 = 2,
    Start0 = 3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckSync,
    Data,
}

/// A frame read by [`AdaptiveReader`]: the header's two pulses and the 104 bits behind them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct AdaptiveFrame {
    /// The header's LOW and HIGH in µs.
    pub(super) header_low: u32,
    pub(super) header_high: u32,
    /// Bits 0..16, inverted as the C keeps them: 0x0001 in a frame.
    pub(super) header: u16,
    /// Bits 16..80 ("Key") and 80..104 ("Key_2").
    pub(super) key: u64,
    pub(super) key_2: u32,
}

impl AdaptiveFrame {
    /// The 13 wire bytes, header ‖ Key ‖ Key_2: the frame as Fiat V1 keeps it.
    pub(super) fn wire(&self) -> [u8; 13] {
        let mut raw = [0u8; 13];
        raw[..2].copy_from_slice(&self.header.to_be_bytes());
        raw[2..10].copy_from_slice(&self.key.to_be_bytes());
        raw[10..].copy_from_slice(&self.key_2.to_be_bytes()[1..]);
        raw
    }

    /// doctor-firmware `adaptive_feed`: the header is Fiat V1's when its HIGH is at most 949 µs
    /// and it is not the narrow Renault header (LOW of 1400 µs or more with a HIGH of 920 µs or
    /// more, which the C's narrow path reports as Renault V1 first).
    fn fiat_v1_header(&self) -> bool {
        self.header_high <= FIAT_HEADER_HIGH_MAX_US
            && !(self.header_low >= NARROW_HEADER_LOW_MIN_US
                && self.header_high >= NARROW_HEADER_HIGH_MIN_US)
    }

    /// The frame is Fiat V1's to report (`fiat_v1.rs` does), not Renault V1's: Fiat V1's header
    /// and a frame that passes Fiat V1's gates (`queue_frame(instance, HitagFiat1, 1, raw)`).
    pub(super) fn is_fiat_v1(&self) -> bool {
        self.fiat_v1_header() && super::fiat_v1::FiatV1Decoder::frame_valid(&self.wire())
    }
}

/// The header detection and the adaptive-TE Manchester reader (the C's `parser_step` machine).
/// Fiat V1 (`fiat_v1.rs`) runs one too, for the type-B frames its cells do not take.
pub(super) struct AdaptiveReader {
    step: DecoderStep,
    manchester_state: ManchesterState,
    header_low: u32,
    header_high: u32,
    te_high: u16,
    te_low: u16,
    decode_data: u64,
    decode_count_bit: u8,
    header: u16,
    key: u64,
}

impl AdaptiveReader {
    pub(super) fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: ManchesterState::Mid1,
            header_low: 0,
            header_high: 0,
            te_high: TE_HIGH_INIT_US,
            te_low: TE_LOW_INIT_US,
            decode_data: 0,
            decode_count_bit: 0,
            header: 0,
            key: 0,
        }
    }

    fn is_header_low(duration: u32) -> bool {
        (HEADER_LOW_MIN_US..=HEADER_LOW_MAX_US).contains(&duration)
    }

    /// `hitag2_data_threshold`: long/short boundary for the current TE estimate.
    fn data_threshold(te: u16) -> u32 {
        let triple = te as u32 * 3;
        if triple < 300 {
            150
        } else if triple >= 422 {
            210
        } else {
            triple / 2
        }
    }

    /// `hitag2_adapt_te`: (7·te + duration) / 8, clamped to 70..=185 µs.
    fn adapt_te(te: u16, duration: u32) -> u16 {
        let mixed = te as u32 * 7 + duration;
        if mixed < 560 {
            70
        } else if mixed >= 1488 {
            185
        } else {
            (mixed / 8) as u16
        }
    }

    /// Canonical Flipper `manchester_advance` (transitions[] in manchester_decoder.c).
    fn manchester_advance(&mut self, event: u8) -> Option<bool> {
        const TRANSITIONS: [u8; 4] = [0b0000_0001, 0b1001_0001, 0b1001_1011, 0b1111_1011];
        let state_idx = self.manchester_state as u8;
        let new_idx = (TRANSITIONS[state_idx as usize] >> event) & 0x3;
        if new_idx == state_idx {
            self.manchester_state = ManchesterState::Mid1;
            return None;
        }
        self.manchester_state = match new_idx {
            0 => ManchesterState::Start1,
            1 => ManchesterState::Mid1,
            2 => ManchesterState::Mid0,
            _ => ManchesterState::Start0,
        };
        match self.manchester_state {
            ManchesterState::Mid0 => Some(false),
            ManchesterState::Mid1 => Some(true),
            _ => None,
        }
    }

    /// One pulse through the C state machine (`while(true) switch(parser_step)`). Returns the
    /// frame the pulse completes: 104 bits, whatever they hold.
    pub(super) fn feed(&mut self, level: bool, duration: u32) -> Option<AdaptiveFrame> {
        loop {
            match self.step {
                DecoderStep::Reset => {
                    if !level && Self::is_header_low(duration) {
                        self.header_low = duration;
                        self.step = DecoderStep::CheckSync;
                    }
                    return None;
                }
                DecoderStep::CheckSync => {
                    if level {
                        if !(HEADER_HIGH_MIN_US..=HEADER_HIGH_MAX_US).contains(&duration)
                            || !Self::is_header_low(self.header_low)
                        {
                            self.step = DecoderStep::Reset;
                            return None;
                        }
                        self.header_high = duration;
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.manchester_state = ManchesterState::Start1;
                        self.te_high = TE_HIGH_INIT_US;
                        self.te_low = TE_LOW_INIT_US;
                        self.step = DecoderStep::Data;
                        return None;
                    }
                    self.step = DecoderStep::Reset;
                    if Self::is_header_low(duration) {
                        self.header_low = duration;
                        self.step = DecoderStep::CheckSync;
                    }
                    return None;
                }
                DecoderStep::Data => {
                    if duration <= DATA_IGNORE_US {
                        return None;
                    }
                    if duration > DATA_RESET_US {
                        self.step = DecoderStep::Reset;
                        continue;
                    }
                    let te = if level {
                        &mut self.te_high
                    } else {
                        &mut self.te_low
                    };
                    let event = if duration > Self::data_threshold(*te) {
                        if level {
                            EVENT_LONG_HIGH
                        } else {
                            EVENT_LONG_LOW
                        }
                    } else {
                        *te = Self::adapt_te(*te, duration);
                        if level {
                            EVENT_SHORT_HIGH
                        } else {
                            EVENT_SHORT_LOW
                        }
                    };
                    let bit = self.manchester_advance(event)?;
                    self.decode_data = (self.decode_data << 1) | bit as u64;
                    self.decode_count_bit += 1;

                    if self.decode_count_bit == HEADER_BITS {
                        self.header = !(self.decode_data as u16);
                        self.decode_data = 0;
                    } else if self.decode_count_bit == KEY_END_BITS {
                        self.key = !self.decode_data;
                        self.decode_data = 0;
                    } else if self.decode_count_bit == LONG_FRAME_BITS {
                        let key_2 = (!self.decode_data & 0xFF_FFFF) as u32;
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        return Some(AdaptiveFrame {
                            header_low: self.header_low,
                            header_high: self.header_high,
                            header: self.header,
                            key: self.key,
                            key_2,
                        });
                    }
                    return None;
                }
            }
        }
    }
}

pub struct RenaultV1Decoder {
    reader: AdaptiveReader,
    /// Last accepted (Key, Key_2) — `last_data` / `last_data_2`.
    last_frame: Option<(u64, u32)>,
    /// Accepted frame waiting for the next pulse (see "Deviations from C").
    pending: Option<DecodedSignal>,
    /// Last seed brute-force result (payload → recovered IV). Pure memo of a deterministic
    /// function; kept across `reset()` so a burst's repeated long frames search once.
    seed_cache: Option<([u8; 11], Option<[u8; 4]>)>,
}

impl RenaultV1Decoder {
    pub fn new() -> Self {
        Self {
            reader: AdaptiveReader::new(),
            last_frame: None,
            pending: None,
            seed_cache: None,
        }
    }

    /// `hitag2_pack_key_bytes`: Key (8 bytes) ‖ Key_2 (3 bytes).
    fn payload(key: u64, key_2: u32) -> [u8; 11] {
        let mut raw = [0u8; 11];
        raw[..8].copy_from_slice(&key.to_be_bytes());
        raw[8..].copy_from_slice(&key_2.to_be_bytes()[1..]);
        raw
    }

    /// `hitag2_frame_xor`: XOR of payload bytes 0..10.
    fn payload_xor(raw: &[u8; 11]) -> u8 {
        raw[..10].iter().fold(0, |acc, b| acc ^ b)
    }

    /// [`hitag2::renault_seed_recover`] through the one-entry memo.
    fn recover_seed(&mut self, raw: &[u8; 11]) -> Option<[u8; 4]> {
        if let Some((frame, iv)) = self.seed_cache {
            if frame == *raw {
                return iv;
            }
        }
        let iv = hitag2::renault_seed_recover(raw);
        self.seed_cache = Some((*raw, iv));
        iv
    }

    /// The decoder's frame gates on a payload: XOR byte and sane UID.
    fn payload_valid(raw: &[u8; 11]) -> bool {
        let uid = u32::from_be_bytes([raw[0], raw[1], raw[2], raw[3]]);
        Self::payload_xor(raw) == raw[10] && uid != 0 && uid != u32::MAX
    }

    /// 11-byte payload (Key ‖ Key_2) of a decoded frame: `data` = 0x0001 ‖ Key[63:16], `extra` =
    /// Key[15:0] ‖ Key_2. `None` unless it is a 104-bit frame passing the decoder's gates.
    fn payload_from_signal(decoded: &DecodedSignal) -> Option<[u8; 11]> {
        if decoded.data_count_bit != WIRE_BITS || decoded.data >> 48 != 1 {
            return None;
        }
        let extra = decoded.extra?;
        if extra >> 40 != 0 {
            return None;
        }
        let mut raw = [0u8; 11];
        raw[..6].copy_from_slice(&decoded.data.to_be_bytes()[2..8]);
        raw[6..].copy_from_slice(&extra.to_be_bytes()[3..8]);
        Self::payload_valid(&raw).then_some(raw)
    }

    /// `hitag2_accept_frame` (+ KAT UID sanity gate, + doctor-firmware's hand-over of Fiat V1's
    /// frames). Returns the frame to emit.
    fn accept_frame(&mut self, frame: &AdaptiveFrame) -> Option<DecodedSignal> {
        if frame.header != 1 {
            return None;
        }
        let (key, key_2) = (frame.key, frame.key_2);
        let raw = Self::payload(key, key_2);
        if Self::payload_xor(&raw) != raw[10] {
            return None;
        }
        if frame.is_fiat_v1() {
            return None;
        }
        if self.last_frame == Some((key, key_2)) {
            return None;
        }
        let uid = (key >> 32) as u32;
        if uid == 0 || uid == u32::MAX {
            return None;
        }
        self.last_frame = Some((key, key_2));
        let iv = self.recover_seed(&raw);
        Some(Self::frame_signal(key, key_2, iv))
    }

    /// The reported signal for an accepted frame (Key, Key_2) and its seed search result:
    /// `hitag2_unpack_frame`; with a recovered seed, `hitag2_apply_check_remote`'s 16-bit counter.
    fn frame_signal(key: u64, key_2: u32, iv: Option<[u8; 4]>) -> DecodedSignal {
        let raw = Self::payload(key, key_2);
        let button = raw[4] >> 4;
        let counter = match iv {
            Some(iv) => hitag2::renault_counter_from_seed(hitag2::renault_seed_from_iv(&iv)),
            None => Self::cnt10(&raw),
        };
        DecodedSignal {
            serial: Some((key >> 32) as u32),
            button: Some(button),
            counter: Some(counter),
            crc_valid: true,
            data: (1u64 << 48) | (key >> 16),
            data_count_bit: WIRE_BITS,
            // Seed recovered → fresh frames; otherwise doctor-firmware's replay (see Encoder).
            encoder_capable: true,
            extra: Some(((key & 0xFFFF) << 24) | key_2 as u64),
            protocol_display_name: None,
        }
    }

    /// `hitag2_frame_cnt10`: counter bits 9..0 of a payload.
    fn cnt10(raw: &[u8; 11]) -> u16 {
        (((raw[4] & 0x0F) as u16) << 6) | (raw[5] >> 2) as u16
    }

    /// `hitag2_frame_hop`: the 32-bit authenticator of a payload.
    fn hop(raw: &[u8; 11]) -> u32 {
        ((raw[5] as u32 & 3) << 30)
            | ((raw[6] as u32) << 22)
            | ((raw[7] as u32) << 14)
            | ((raw[8] as u32) << 6)
            | (raw[9] as u32 >> 2)
    }

    /// KAT generic button → Renault code (ProtoPirate emulate plugin: Up = Lock 1, OK = Unlock 2,
    /// Left = Trunk 4, Right = Panic 8 — the same values). `None` keeps the captured button.
    fn map_button(button: u8) -> Option<u8> {
        matches!(button, 0x01 | 0x02 | 0x04 | 0x08).then_some(button)
    }

    fn enc_add_level(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }

    /// `hitag2_encoder_add_bits`: MSB first, 1 → (HIGH te, LOW te), 0 → (LOW te, HIGH te).
    fn enc_add_bits(signal: &mut Vec<LevelDuration>, value: u64, bits: u32) {
        for i in (0..bits).rev() {
            let one = (value >> i) & 1 == 1;
            Self::enc_add_level(signal, one, TE_US);
            Self::enc_add_level(signal, !one, TE_US);
        }
    }

    /// Upload for an 11-byte payload in the given reference layout (adjacent levels merged, as
    /// on air; neither C repeats the upload).
    fn build_upload(raw: &[u8; 11], shape: &UploadShape) -> Vec<LevelDuration> {
        let key = u64::from_be_bytes(raw[..8].try_into().unwrap());
        let key_2 = ((raw[8] as u64) << 16) | ((raw[9] as u64) << 8) | raw[10] as u64;
        let short_key = (key >> 18) & 0x3FF;
        let mut s = Vec::with_capacity(shape.preamble_pairs * 2 + 1400);
        for _ in 0..shape.preamble_pairs {
            Self::enc_add_level(&mut s, true, TE_US);
            Self::enc_add_level(&mut s, false, TE_US);
        }
        if shape.preamble_tail_high {
            Self::enc_add_level(&mut s, true, TE_US);
        }
        for _ in 0..shape.long_frames {
            Self::enc_add_level(&mut s, false, ENC_HEADER_LOW_US);
            Self::enc_add_level(&mut s, true, ENC_HEADER_HIGH_US);
            Self::enc_add_bits(&mut s, 1, HEADER_BITS as u32);
            Self::enc_add_bits(&mut s, key, 64);
            Self::enc_add_bits(&mut s, key_2, 24);
        }
        for _ in 0..shape.short_frames {
            Self::enc_add_level(&mut s, false, ENC_HEADER_LOW_US);
            Self::enc_add_level(&mut s, true, ENC_HEADER_HIGH_US);
            Self::enc_add_bits(&mut s, 1, HEADER_BITS as u32);
            Self::enc_add_bits(&mut s, short_key, ENC_SHORT_KEY_BITS);
            Self::enc_add_level(&mut s, false, shape.short_gap_us);
        }
        s
    }

    /// The 11-byte payload `encode()` transmits and the upload layout it uses: a fresh
    /// seed-scheme frame (ProtoPirate), a fresh frame from the remote's saved key
    /// (doctor-firmware), or the captured frame (doctor-firmware replay).
    fn tx_payload(raw: &[u8; 11], button: u8) -> ([u8; 11], &'static UploadShape) {
        match hitag2::renault_seed_recover(raw) {
            Some(iv) => {
                let uid = u32::from_be_bytes([raw[0], raw[1], raw[2], raw[3]]);
                let seed = hitag2::renault_seed_from_iv(&iv);
                let cnt = hitag2::renault_counter_from_seed(seed) as u32 + 1;
                let btn = Self::map_button(button).unwrap_or(raw[4] >> 4);
                let (frame, _) = hitag2::renault_encrypt_frame(uid, cnt, btn, seed);
                (frame, &PP_UPLOAD)
            }
            None => match Self::saved_key_payload(raw, button) {
                Some(frame) => (frame, &DOCTOR_KANGOO_UPLOAD),
                None => (*raw, &DOCTOR_KANGOO_UPLOAD),
            },
        }
    }

    /// The key saved for the payload's remote (`hitag2_keys.txt`, e.g. by "Recover key
    /// (HiTag2)") and the epoch with which it reproduces the captured hop: doctor-firmware's
    /// `hitag2_hell_find_epoch` (the record's epoch and the next seven).
    fn saved_key(raw: &[u8; 11]) -> Option<([u8; 6], u32)> {
        let uid = u32::from_be_bytes([raw[0], raw[1], raw[2], raw[3]]);
        let (btn, cnt10, hop) = (raw[4] >> 4, u32::from(Self::cnt10(raw)), Self::hop(raw));
        hitag2_keys::keys_for(uid, KeyOrder::Standard).into_iter().find_map(|(key, start)| {
            (0..8u32)
                .map(|i| (start + i) & 0x3FFFF)
                .find(|&epoch| hitag2::renault_authenticator(uid, btn, (epoch << 10) | cnt10, &key) == hop)
                .map(|epoch| (key, epoch))
        })
    }

    /// doctor-firmware's key path (`hitag2_encoder_hell_next_frame`): a FRESH frame from the saved
    /// key — the requested button (else the captured one), the counter `epoch << 10 | cnt10` + 1
    /// (carrying into the epoch), its hop, the captured tail bits (`hitag2_pack_hell_frame`).
    fn saved_key_payload(raw: &[u8; 11], button: u8) -> Option<[u8; 11]> {
        let (key, epoch) = Self::saved_key(raw)?;
        let uid = u32::from_be_bytes([raw[0], raw[1], raw[2], raw[3]]);
        let btn = Self::map_button(button).unwrap_or(raw[4] >> 4);
        let full = ((epoch << 10) | u32::from(Self::cnt10(raw))) + 1;
        let hop = hitag2::renault_authenticator(uid, btn, full, &key);
        let cnt10 = full & 0x3FF;
        let mut frame = [0u8; 11];
        frame[..4].copy_from_slice(&uid.to_be_bytes());
        frame[4] = ((btn & 0x0F) << 4) | (cnt10 >> 6) as u8;
        frame[5] = (((cnt10 & 0x3F) << 2) as u8) | (hop >> 30) as u8;
        frame[6] = (hop >> 22) as u8;
        frame[7] = (hop >> 14) as u8;
        frame[8] = (hop >> 6) as u8;
        frame[9] = ((hop << 2) as u8) | (raw[9] & 0x03);
        frame[10] = Self::payload_xor(&frame);
        Some(frame)
    }

    /// Whether `encode()` sends fresh frames of this decode (seed recovered, or a saved key that
    /// verifies) instead of replaying it: "Recover key (HiTag2)" has nothing left to find.
    pub(super) fn has_key(decoded: &DecodedSignal) -> bool {
        Self::payload_from_signal(decoded)
            .is_some_and(|raw| hitag2::renault_seed_recover(&raw).is_some() || Self::saved_key(&raw).is_some())
    }
}

impl ProtocolDecoder for RenaultV1Decoder {
    fn name(&self) -> &'static str {
        "Renault V1"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_US,
            te_long: TE_US * 2,
            te_delta: 50,
            min_count_bit: WIRE_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000, 868_350_000]
    }

    fn reset(&mut self) {
        let seed_cache = self.seed_cache.take();
        *self = Self::new();
        self.seed_cache = seed_cache;
    }

    /// The input ended on the pulse that completed a frame: report it now (the one-pulse delay has
    /// no next pulse to wait for; the C reports on that last bit).
    fn flush(&mut self) -> Option<DecodedSignal> {
        self.pending.take()
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        // A frame completed on the previous pulse is reported now (one-pulse delay, see module
        // doc); this pulse still runs through the state machine.
        let ready = self.pending.take();
        if let Some(frame) = self.reader.feed(level, duration) {
            if let Some(signal) = self.accept_frame(&frame) {
                self.pending = Some(signal);
            }
        }
        ready
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let raw = Self::payload_from_signal(decoded)?;
        let (payload, shape) = Self::tx_payload(&raw, button);
        Some(Self::build_upload(&payload, shape))
    }

    /// Key files: ProtoPirate `subghz_protocol_decoder_renault_v1_deserialize`: Bit must be 88;
    /// Key (the C's 64-bit `generic.data`) and Key_2 (8 bytes, the 24-bit tail) rebuild the
    /// frame, which must pass the decoder's gates (XOR byte, sane UID; the C loads any key).
    /// The counter is the decoder's: the seed brute force runs as on air, so "Recovered" / "Seed"
    /// (which PP's brute force writes) and the Serial/Btn/Cnt/Hop display lines are not needed;
    /// "Hitag2 Key" is not read (KAT uses the saved keys, `hitag2_keys.rs`). A `Bit: 82` key is Flipper-ARF's
    /// "Renault V1" (KAT's "Renault V1 (ARF)") and loads through that decoder.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits == 82 {
            return super::renault_v1_arf::RenaultV1ArfDecoder::new().from_flipper_key(key);
        }
        if key.bits != 88 {
            return None;
        }
        let k = key.key_u64()?;
        let key_2 = (key.hex_field("Key_2")? & 0xFF_FFFF) as u32;
        let raw = Self::payload(k, key_2);
        if !Self::payload_valid(&raw) {
            return None;
        }
        Some(Self::frame_signal(k, key_2, hitag2::renault_seed_recover(&raw)))
    }

    /// Key files: ProtoPirate `subghz_protocol_decoder_renault_v1_serialize`: Protocol "Renault V1",
    /// Bit 88, Key, then Key_2 (8 bytes), Recovered (1 byte: 01 = seed recovered, 02 = brute force
    /// missed — KAT always runs it), Seed (4 bytes: the recovered IV, else 0), Serial, Btn, Cnt
    /// (16-bit with a recovered seed, else 10-bit) and Hop (decimal), as PP's file reads after its
    /// "Hitag2 BF"; then doctor-firmware's Hitag2 Key + Hitag2 Epoch when a saved key verifies.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let raw = Self::payload_from_signal(decoded)?;
        let k = u64::from_be_bytes(raw[..8].try_into().ok()?);
        let key_2 = u32::from_be_bytes([0, raw[8], raw[9], raw[10]]);
        let iv = hitag2::renault_seed_recover(&raw);
        let seed = iv.as_ref().map_or(0, hitag2::renault_seed_from_iv);
        let counter = match iv {
            Some(_) => hitag2::renault_counter_from_seed(seed),
            None => Self::cnt10(&raw),
        };
        let mut key = FlipperKey::for_protocol("Renault V1", 88, k);
        key.push_hex("Key_2", u64::from(key_2), 8);
        key.push_hex("Recovered", if iv.is_some() { 1 } else { 2 }, 1);
        key.push_hex("Seed", u64::from(seed), 4);
        key.push_u32("Serial", (k >> 32) as u32);
        key.push_u32("Btn", u32::from(raw[4] >> 4));
        key.push_u32("Cnt", counter.into());
        key.push_u32("Hop", Self::hop(&raw));
        // doctor-firmware's lines for a key it verified (`HITAG2_HELL_KEY_FIELD`, standard order).
        if iv.is_none() {
            if let Some((hitag2_key, epoch)) = Self::saved_key(&raw) {
                key.push_hex_bytes("Hitag2 Key", &hitag2_key);
                key.push_u32("Hitag2 Epoch", epoch);
            }
        }
        Some(key)
    }
}

impl Default for RenaultV1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    /// `hitag2_pack_auth_frame`: 11-byte payload from fields.
    fn pack(uid: u32, btn: u8, cnt10: u16, hop: u32, tail: u8) -> [u8; 11] {
        let cnt10 = cnt10 as u32;
        let mut raw = [0u8; 11];
        raw[..4].copy_from_slice(&uid.to_be_bytes());
        raw[4] = ((btn & 0x0F) << 4) | ((cnt10 >> 6) & 0x0F) as u8;
        raw[5] = (((cnt10 & 0x3F) << 2) as u8) | ((hop >> 30) & 3) as u8;
        raw[6] = (hop >> 22) as u8;
        raw[7] = (hop >> 14) as u8;
        raw[8] = (hop >> 6) as u8;
        raw[9] = (((hop << 2) & 0xFC) as u8) | (tail & 3);
        raw[10] = RenaultV1Decoder::payload_xor(&raw);
        raw
    }

    fn add(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }

    /// `hitag2_encoder_add_bits`: MSB first, 1 → (HIGH, LOW), 0 → (LOW, HIGH).
    fn add_bits(signal: &mut Vec<LevelDuration>, value: u64, bits: u32, te: u32) {
        for i in (0..bits).rev() {
            let one = (value >> i) & 1 == 1;
            add(signal, one, te);
            add(signal, !one, te);
        }
    }

    /// The C encoder's upload (`renault_v1_encoder_get_upload`: 250 preamble pairs, 3 long frames,
    /// 3 short frames) for a payload, at an arbitrary half-bit `te` (remotes faster/slower than
    /// the encoder's 125 µs), with adjacent levels merged as on air. At 125 µs it is the encoder's
    /// own upload (checked in `pp_upload_matches_c_reference`).
    fn c_upload(raw: &[u8; 11], te: u32) -> Vec<LevelDuration> {
        let key = u64::from_be_bytes(raw[..8].try_into().unwrap());
        let key_2 = ((raw[8] as u64) << 16) | ((raw[9] as u64) << 8) | raw[10] as u64;
        let mut s = Vec::new();
        for _ in 0..250 {
            add(&mut s, true, te);
            add(&mut s, false, te);
        }
        for _ in 0..3 {
            add(&mut s, false, 1500);
            add(&mut s, true, 1000);
            add_bits(&mut s, 1, 16, te);
            add_bits(&mut s, key, 64, te);
            add_bits(&mut s, key_2, 24, te);
        }
        for _ in 0..3 {
            add(&mut s, false, 1500);
            add(&mut s, true, 1000);
            add_bits(&mut s, 1, 16, te);
            add_bits(&mut s, (key >> 18) & 0x3FF, 10, te);
            add(&mut s, false, 21500);
        }
        s
    }

    fn decode_all(dec: &mut RenaultV1Decoder, signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        signal
            .iter()
            .filter_map(|ld| dec.feed(ld.level, ld.duration_us))
            .collect()
    }

    fn registry_names(signal: &[LevelDuration], freq: u32) -> Vec<String> {
        let mut reg = ProtocolRegistry::new();
        reg.process_signal_stream(signal, freq)
            .into_iter()
            .map(|r| r.0)
            .collect()
    }

    /// A synthetic frame in the C encoder's format decodes with all fields, the 104-bit packing,
    /// and (standalone, no reset) once per unique frame thanks to duplicate suppression. Its hop is
    /// not a seed-scheme hop (the C brute force reports NOT_FOUND), so the counter stays 10-bit.
    #[test]
    fn synthetic_burst_decodes() {
        let raw = pack(0x1A2B3C4D, 0x2, 0x155, 0x89ABCDEF, 2);
        assert_eq!(hitag2::renault_seed_recover(&raw), None);
        let mut dec = RenaultV1Decoder::new();
        let out = decode_all(&mut dec, &c_upload(&raw, TE_US));
        assert_eq!(out.len(), 1, "3 identical long frames → 1 decode (dedup)");
        let d = &out[0];
        assert_eq!(d.serial, Some(0x1A2B3C4D));
        assert_eq!(d.button, Some(0x2));
        assert_eq!(d.counter, Some(0x155));
        assert!(d.crc_valid);
        assert!(
            d.encoder_capable,
            "replay (doctor-firmware) is always possible"
        );
        assert_eq!(d.data_count_bit, 104);
        // data/extra = the 13-byte wire frame 00 01 ‖ payload.
        let mut wire = vec![0x00, 0x01];
        wire.extend_from_slice(&raw);
        assert_eq!(d.data, u64::from_be_bytes(wire[..8].try_into().unwrap()));
        assert_eq!(
            d.extra,
            Some(wire[8..].iter().fold(0u64, |a, &b| (a << 8) | b as u64))
        );
        // hop / tail recoverable from the packing (hitag2_unpack_frame).
        let extra = d.extra.unwrap();
        let hop = (((d.data & 0x3) as u32) << 30) | ((extra >> 10) & 0x3FFF_FFFF) as u32;
        assert_eq!(hop, 0x89ABCDEF);
        assert_eq!((extra >> 8) & 0x3, 2);
    }

    /// The adaptive TE tracks remotes faster or slower than the nominal 125 µs.
    #[test]
    fn adaptive_te_decodes_timing_spread() {
        for te in [100u32, 110, 140, 150] {
            let raw = pack(0x00C0FFEE, 0x1, 0x3FF, 0x01234567, 1);
            let mut dec = RenaultV1Decoder::new();
            let out = decode_all(&mut dec, &c_upload(&raw, te));
            assert_eq!(out.len(), 1, "te {te}");
            assert_eq!(out[0].serial, Some(0x00C0FFEE), "te {te}");
            assert_eq!(out[0].counter, Some(0x3FF), "te {te}");
        }
    }

    /// Gate: bad XOR, wrong header word or a degenerate UID never decode.
    #[test]
    fn corrupted_frames_rejected() {
        let good = pack(0x1A2B3C4D, 0x4, 0x001, 0x0F0F0F0F, 2);
        let mut bad_xor = good;
        bad_xor[10] ^= 0x01;
        let zero_uid = pack(0, 0x4, 0x001, 0x0F0F0F0F, 2);
        let ones_uid = pack(u32::MAX, 0x4, 0x001, 0x0F0F0F0F, 2);
        for (what, raw) in [
            ("xor", bad_xor),
            ("uid 0", zero_uid),
            ("uid ffffffff", ones_uid),
        ] {
            let mut dec = RenaultV1Decoder::new();
            assert!(
                decode_all(&mut dec, &c_upload(&raw, TE_US)).is_empty(),
                "{what} must not decode"
            );
        }
        // Header word 0x0003 instead of 0x0001.
        let key = u64::from_be_bytes(good[..8].try_into().unwrap());
        let key_2 = ((good[8] as u64) << 16) | ((good[9] as u64) << 8) | good[10] as u64;
        let mut s = Vec::new();
        add(&mut s, false, 1500);
        add(&mut s, true, 1000);
        add_bits(&mut s, 3, 16, TE_US);
        add_bits(&mut s, key, 64, TE_US);
        add_bits(&mut s, key_2, 24, TE_US);
        add(&mut s, false, 20_000);
        assert!(
            decode_all(&mut RenaultV1Decoder::new(), &s).is_empty(),
            "header 0x0003 must not decode"
        );
        // Sanity: the good frame decodes.
        assert_eq!(
            decode_all(&mut RenaultV1Decoder::new(), &c_upload(&good, TE_US)).len(),
            1
        );
    }

    /// Duplicate memory is cleared by reset() (unlike the C, see module doc).
    #[test]
    fn reset_clears_duplicate_memory() {
        let burst = c_upload(&pack(0x31415926, 0x8, 0x0AA, 0x27182818, 0), TE_US);
        let mut dec = RenaultV1Decoder::new();
        assert_eq!(decode_all(&mut dec, &burst).len(), 1);
        assert!(
            decode_all(&mut dec, &burst).is_empty(),
            "same frame again without reset: suppressed"
        );
        dec.reset();
        assert_eq!(decode_all(&mut dec, &burst).len(), 1);
    }

    /// Emission is one pulse after the 104th bit's mid-bit transition (the pulse carrying its
    /// second half), i.e. where Fiat V1's window completes.
    #[test]
    fn emission_is_delayed_one_pulse() {
        // Two hops chosen so the XOR byte ends in 1 and in 0.
        let frames = [
            pack(0x1A2B3C4D, 0x2, 0x155, 0x89ABCDEF, 2),
            pack(0x1A2B3C4D, 0x2, 0x155, 0x89ABCDEF, 3),
        ];
        assert_ne!(frames[0][10] & 1, frames[1][10] & 1);
        for raw in frames {
            let key = u64::from_be_bytes(raw[..8].try_into().unwrap());
            let key_2 = ((raw[8] as u64) << 16) | ((raw[9] as u64) << 8) | raw[10] as u64;
            let mut s = vec![
                LevelDuration::new(false, 1500),
                LevelDuration::new(true, 1000),
            ];
            add_bits(&mut s, 1, 16, TE_US);
            add_bits(&mut s, key, 64, TE_US);
            add_bits(&mut s, key_2, 24, TE_US);
            add(&mut s, false, 20_000);
            let mut dec = RenaultV1Decoder::new();
            let hits: Vec<usize> = s
                .iter()
                .enumerate()
                .filter_map(|(i, ld)| dec.feed(ld.level, ld.duration_us).map(|_| i))
                .collect();
            // Last bit 1 = (HIGH, LOW): its LOW half merges into the final gap pulse. Last bit 0 =
            // (LOW, HIGH): its HIGH half is the pulse before the gap. Either way the C would fire
            // one pulse earlier (on the first half).
            let second_half = if raw[10] & 1 == 1 {
                s.len() - 1
            } else {
                s.len() - 2
            };
            assert_eq!(hits, vec![second_half], "xor {:02X}", raw[10]);
            // The input ends on the pulse the C fires on: the frame is reported when it ends
            // ([ProtocolDecoder::flush]) instead of waiting for a pulse that never comes.
            let reported = {
                let mut dec = RenaultV1Decoder::new();
                s.iter().find_map(|ld| dec.feed(ld.level, ld.duration_us)).expect("frame")
            };
            let mut dec = RenaultV1Decoder::new();
            assert!(s[..second_half].iter().all(|ld| dec.feed(ld.level, ld.duration_us).is_none()));
            assert_eq!(dec.flush(), Some(reported), "xor {:02X}", raw[10]);
            assert!(dec.flush().is_none());
        }
    }

    /// A Renault-format burst at the nominal 125 µs decodes as Renault V1 through the registry
    /// (and not as Fiat V1, whose 100/200 µs window rejects the 250 µs longs).
    #[test]
    fn registry_reports_renault_burst() {
        let raw = pack(0x5EED1234, 0x1, 0x2C3, 0xA5A55A5A, 2);
        let mut signal = vec![LevelDuration::new(false, 30_000)];
        signal.extend(c_upload(&raw, TE_US));
        for freq in [433_920_000u32, 315_000_000, 868_350_000] {
            let names = registry_names(&signal, freq);
            assert!(names.iter().any(|n| n == "Renault V1"), "{freq}: {names:?}");
            assert!(names.iter().all(|n| n == "Renault V1"), "{freq}: {names:?}");
        }
    }

    /// Fiat V1 type-B bursts (850 µs lead, 100 µs Manchester, 1230 µs gap, 6 repeats) carry the
    /// same frame and pass Renault's header window from the 2nd repeat on. Fiat V1 must keep every
    /// one of them (both parities of the last bit, which decide whether Renault re-syncs).
    #[test]
    fn fiat_v1_type_b_bursts_stay_fiat() {
        for (hop, tail) in [
            (0x9DAB8C6Au32, 2u8),
            (0x9D03CFDA, 2),
            (0x13579BDF, 1),
            (0x2468ACE0, 3),
        ] {
            let payload = pack(0x12345678, 0x4, 0x101, hop, tail);
            let mut frame = vec![0x00u8, 0x01];
            frame.extend_from_slice(&payload);
            let mut upload = Vec::new();
            add(&mut upload, true, 850);
            for bit_index in 0..104 {
                let bit = (frame[bit_index >> 3] >> (7 - (bit_index & 7))) & 1 == 1;
                add(&mut upload, bit, 100);
                add(&mut upload, !bit, 100);
            }
            add(&mut upload, false, 1230);
            let mut signal = vec![LevelDuration::new(false, 20_000)];
            for _ in 0..6 {
                signal.extend(
                    upload
                        .iter()
                        .map(|ld| LevelDuration::new(ld.level, ld.duration_us)),
                );
            }
            let names = registry_names(&signal, 433_920_000);
            assert_eq!(names.len(), 6, "hop {hop:08X}: {names:?}");
            assert!(
                names.iter().all(|n| n == "Fiat V1"),
                "hop {hop:08X}: {names:?}"
            );
            // Standalone, Renault V1 leaves them alone: behind Fiat V1's header (a HIGH of 949 µs
            // or less) a frame that passes Fiat V1's gates is Fiat V1's.
            let mut dec = RenaultV1Decoder::new();
            assert!(decode_all(&mut dec, &signal).is_empty(), "hop {hop:08X}");
        }
    }

    /// doctor-firmware's merged decoder (`hitag2.c`, `adaptive_feed`): the header decides whose
    /// frame it is. The frames below have 125 µs half-bits, which keeps Fiat V1's cells out (a
    /// 250 µs long is outside 200 ± 50), so every name is the adaptive reader's.
    #[test]
    fn header_decides_between_fiat_v1_and_renault_v1() {
        let burst = |raw: &[u8; 11], low: u32, high: u32| {
            // The last bit is 0 = (LOW, HIGH), so the next copy's header LOW is `low` exactly.
            assert_eq!(raw[10] & 1, 0);
            let key = u64::from_be_bytes(raw[..8].try_into().unwrap());
            let key_2 = ((raw[8] as u64) << 16) | ((raw[9] as u64) << 8) | raw[10] as u64;
            let mut s = Vec::new();
            for _ in 0..3 {
                add(&mut s, false, low);
                add(&mut s, true, high);
                add_bits(&mut s, 1, 16, TE_US);
                add_bits(&mut s, key, 64, TE_US);
                add_bits(&mut s, key_2, 24, TE_US);
            }
            add(&mut s, false, 20_000);
            s
        };
        let decodes = |signal: &[LevelDuration]| {
            let mut reg = ProtocolRegistry::new();
            reg.process_signal_stream(signal, 433_920_000)
        };
        // Button 4 (Fiat V1's Lock, Renault's Trunk): a frame both decoders would take.
        let both = pack(0x12345678, 0x4, 0x101, 0x9DAB8C6A, 3);
        for (low, high, name) in [
            (1230, 850, "Fiat V1"), // Fiat V1 type B: a gap of 1230 µs, a lead of 850 µs
            (1150, 800, "Fiat V1"),
            (1399, 949, "Fiat V1"),
            (2200, 919, "Fiat V1"),
            (1230, 950, "Renault V1"),
            (1150, 1150, "Renault V1"),
            (1400, 920, "Renault V1"), // the narrow header: Renault's whatever the HIGH
            (1400, 949, "Renault V1"),
            (1500, 1000, "Renault V1"), // Renault's own header
        ] {
            let got = decodes(&burst(&both, low, high));
            let names: Vec<&str> = got.iter().map(|r| r.0.as_str()).collect();
            assert_eq!(names, vec![name; 3], "header {low}/{high}");
            for (_, d, _) in &got {
                assert_eq!((d.serial, d.button, d.counter), (Some(0x12345678), Some(0x4), Some(0x101)));
                assert_eq!(d.data, 0x0001_1234_5678_4406, "header {low}/{high}");
                // Fiat V1 keeps the TE variant above the 40 tail bits: type B.
                let variant = if name == "Fiat V1" { 1 } else { 0 };
                assert_eq!(d.extra.map(|e| e >> 40), Some(variant), "header {low}/{high}");
            }
        }
        // Behind Fiat V1's header, a frame Fiat V1's gates refuse stays Renault V1's: buttons 0
        // (Renault's Sync) and 3 are none of Fiat V1's 1, 2, 4 and 8.
        for button in [0x0u8, 0x3] {
            let renault = pack(0x12345678, button, 0x101, 0x9DAB8C6A, 3);
            let got = decodes(&burst(&renault, 1230, 850));
            let names: Vec<&str> = got.iter().map(|r| r.0.as_str()).collect();
            assert_eq!(names, vec!["Renault V1"; 3], "button {button}");
        }
    }

    fn hex11(s: &str) -> [u8; 11] {
        let mut out = [0u8; 11];
        for (i, b) in out.iter_mut().enumerate() {
            *b = u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap();
        }
        out
    }

    /// The DecodedSignal the decoder produces for an 11-byte payload.
    fn signal_for(raw: &[u8; 11]) -> DecodedSignal {
        let mut dec = RenaultV1Decoder::new();
        let out = decode_all(&mut dec, &c_upload(raw, TE_US));
        assert_eq!(out.len(), 1, "payload {raw:02X?} must decode once");
        out.into_iter().next().unwrap()
    }

    /// Payload carried by an upload: decode it standalone and unpack data/extra.
    fn payload_of(upload: &[LevelDuration]) -> Vec<[u8; 11]> {
        let mut dec = RenaultV1Decoder::new();
        decode_all(&mut dec, upload)
            .iter()
            .map(|d| RenaultV1Decoder::payload_from_signal(d).expect("decoded frames pass gates"))
            .collect()
    }

    fn pulses(signal: &[LevelDuration]) -> Vec<i64> {
        signal
            .iter()
            .map(|ld| {
                if ld.level {
                    ld.duration_us as i64
                } else {
                    -(ld.duration_us as i64)
                }
            })
            .collect()
    }

    /// Seed-scheme frames (from the C `hitag2_encrypt_frame`) are recovered at decode time: the
    /// counter becomes the full 16-bit counter `hitag2_apply_check_remote` derives from the seed
    /// (vectors: C `hitag2_brute_force_run` → IV).
    #[test]
    fn seed_frames_decode_with_recovered_counter() {
        // (frame, C-recovered IV, 16-bit counter)
        let vectors = [
            (
                "EFB82D6622345188720EAF",
                [0xD2, 0x08, 0xA0, 0xC1],
                0x008Du16,
            ),
            ("1234567848D122E6FC52FB", [0x44, 0x23, 0xE1, 0x01], 0x1234),
            ("C2064F3E2B1CDFDB258625", [0x72, 0xAC, 0x3C, 0x2B], 0xCAC7),
            ("378F852988DFF51255F203", [0x78, 0xA3, 0x70, 0x97], 0x0A37),
        ];
        for (frame_hex, iv, cnt16) in vectors {
            let raw = hex11(frame_hex);
            assert_eq!(hitag2::renault_seed_recover(&raw), Some(iv), "{frame_hex}");
            let d = signal_for(&raw);
            assert_eq!(d.counter, Some(cnt16), "{frame_hex}");
            assert_eq!(
                d.serial,
                Some(u32::from_be_bytes([raw[0], raw[1], raw[2], raw[3]]))
            );
            assert_eq!(d.button, Some(raw[4] >> 4));
            assert!(d.encoder_capable);
            assert_eq!(RenaultV1Decoder::payload_from_signal(&d), Some(raw));
        }
    }

    /// The encoder's ProtoPirate upload is `renault_v1_encoder_get_upload` (merged): pulse count,
    /// total duration and both ends against the compiled C for the TX of frame
    /// 1234567848D122E6FC52FB with KAT Lock (C: Renault btn 1, cnt 0x1235 →
    /// 1234567818D627D1A61E88), and equal to the test helper at 125 µs.
    #[test]
    fn pp_upload_matches_c_reference() {
        let raw = hex11("1234567848D122E6FC52FB");
        let up = RenaultV1Decoder::new()
            .encode(&signal_for(&raw), 0x01)
            .unwrap();
        let tx = hex11("1234567818D627D1A61E88");
        assert_eq!(pulses(&up), pulses(&c_upload(&tx, TE_US)));
        let p = pulses(&up);
        // From the C oracle (merged levels): the preamble's last LOW merges into the 1500 µs
        // header LOW, the last short frame's final LOW half into the 21.5 ms gap.
        assert_eq!(p.len(), 1136);
        assert_eq!(p.iter().map(|v| v.abs()).sum::<i64>(), 239_500);
        assert_eq!(&p[498..504], &[125, -1625, 1000, -125, 125, -125]);
        assert_eq!(&p[p.len() - 4..], &[250, -250, 250, -21_625]);
    }

    /// Seed path: a FRESH frame per button, exactly the C TX frame (ProtoPirate encoder
    /// deserialize after its BF patch + emulate press; doctor-firmware builds the same frame):
    /// counter + 1, Renault button = KAT 1/2/4/8, other codes keep the captured button.
    #[test]
    fn seed_path_tx_frames_match_c() {
        let raw = hex11("1234567848D122E6FC52FB");
        let sig = signal_for(&raw);
        let enc = RenaultV1Decoder::new();
        // (KAT button, C TX frame)
        let vectors = [
            (0x01u8, "1234567818D627D1A61E88"),
            (0x02, "1234567828D56697BE368C"),
            (0x04, "1234567848D722CDCDEE5B"),
            (0x08, "1234567888D726162696D7"),
            (0x10, "1234567848D722CDCDEE5B"),
        ];
        for (btn, tx_hex) in vectors {
            let up = enc.encode(&sig, btn).expect("seed recovered → encodable");
            let tx = hex11(tx_hex);
            let frames = payload_of(&up);
            assert_eq!(
                frames,
                vec![tx],
                "btn {btn:#X}: 3 identical long frames, deduped"
            );
            // The fresh frame is itself recoverable, one counter step later.
            let d = signal_for(&tx);
            assert_eq!(d.counter, Some(0x1235));
            assert_eq!(d.serial, Some(0x12345678));
        }
    }

    /// A key saved for the remote (`hitag2_keys.txt`, e.g. by "Recover key (HiTag2)") gives
    /// doctor-firmware's fresh frame instead of the replay: the requested button, counter + 1, its
    /// hop, the captured tail; the `.sub` carries doctor-firmware's Hitag2 Key / Epoch lines. A key
    /// saved in Fiat V1's byte order (the same remote decoded as Fiat V1) serves too.
    #[test]
    fn saved_key_signs_fresh_frames() {
        let key = [0x4B, 0x41, 0x54, 0x31, 0x2E, 0x38];
        // UIDs no other test uses: the key database is global.
        for (uid, stored) in [(0x5EED_0EA1u32, key), (0x5EED_0EA2, KeyOrder::FiatBcm.arrange(key))] {
            let hop = hitag2::renault_authenticator(uid, 2, 0x123, &key);
            let mut raw = [0u8; 11];
            raw[..4].copy_from_slice(&uid.to_be_bytes());
            raw[4] = (2 << 4) | (0x123u16 >> 6) as u8;
            raw[5] = (((0x123u16 & 0x3F) << 2) as u8) | (hop >> 30) as u8;
            raw[6] = (hop >> 22) as u8;
            raw[7] = (hop >> 14) as u8;
            raw[8] = (hop >> 6) as u8;
            raw[9] = ((hop << 2) as u8) | 0x01;
            raw[10] = RenaultV1Decoder::payload_xor(&raw);
            assert_eq!(hitag2::renault_seed_recover(&raw), None, "not a seed-scheme frame");
            let k = u64::from_be_bytes(raw[..8].try_into().unwrap());
            let sig = RenaultV1Decoder::frame_signal(k, u32::from_be_bytes([0, raw[8], raw[9], raw[10]]), None);
            assert_eq!(RenaultV1Decoder::payload_from_signal(&sig), Some(raw));
            assert!(!RenaultV1Decoder::has_key(&sig));
            assert_eq!(RenaultV1Decoder::tx_payload(&raw, 0x01).0, raw, "replay");
            hitag2_keys::tests::remember_for_test(uid, stored);
            assert!(RenaultV1Decoder::has_key(&sig));
            assert_eq!(RenaultV1Decoder::saved_key(&raw), Some((key, 0)));
            let (tx, _) = RenaultV1Decoder::tx_payload(&raw, 0x01);
            assert_eq!((&tx[..4], tx[4] >> 4, RenaultV1Decoder::cnt10(&tx)), (&uid.to_be_bytes()[..], 1, 0x124));
            assert_eq!(RenaultV1Decoder::hop(&tx), hitag2::renault_authenticator(uid, 1, 0x124, &key));
            assert_eq!((tx[9] & 3, tx[10]), (1, RenaultV1Decoder::payload_xor(&tx)));
            let text = RenaultV1Decoder::new().to_flipper_key(&sig).unwrap().to_text();
            assert!(text.contains("\nHitag2 Key: 4B 41 54 31 2E 38\nHitag2 Epoch: 0\n"), "{text}");
        }
    }

    /// No seed: doctor-firmware's key-less path replays the captured frame byte-for-byte (any
    /// button) with its "Kangoo" upload — 255 pairs + HIGH, 3 long frames, 8 short frames closed by
    /// 20 ms (pulse count / total / ends from the compiled doctor-firmware C).
    #[test]
    fn no_seed_replays_with_doctor_upload() {
        let raw = pack(0x5EED1234, 0x1, 0x2C3, 0xA5A55A5A, 2);
        assert_eq!(hitag2::renault_seed_recover(&raw), None);
        let sig = signal_for(&raw);
        let enc = RenaultV1Decoder::new();
        let lock = enc.encode(&sig, 0x01).unwrap();
        for btn in [0x02u8, 0x04, 0x08, 0x40] {
            assert_eq!(
                pulses(&enc.encode(&sig, btn).unwrap()),
                pulses(&lock),
                "btn {btn:#X}"
            );
        }
        assert_eq!(payload_of(&lock), vec![raw]);
        let p = pulses(&lock);
        assert_eq!(p.len(), 1364);
        assert_eq!(p.iter().map(|v| v.abs()).sum::<i64>(), 381_375);
        assert_eq!(&p[..4], &[125, -125, 125, -125]);
        assert_eq!(&p[508..513], &[125, -125, 125, -1500, 1000]);
        assert_eq!(*p.last().unwrap(), -20_125);
    }

    /// Frames that fail the decoder's gates (or are not 104-bit Renault frames) do not encode.
    #[test]
    fn invalid_signals_do_not_encode() {
        let raw = pack(0x1A2B3C4D, 0x2, 0x155, 0x89ABCDEF, 2);
        let good = signal_for(&raw);
        let enc = RenaultV1Decoder::new();
        assert!(enc.encode(&good, 0x01).is_some());
        let mut bad_xor = good.clone();
        bad_xor.extra = good.extra.map(|e| e ^ 1);
        let mut bad_header = good.clone();
        bad_header.data ^= 1 << 49;
        let mut bad_len = good.clone();
        bad_len.data_count_bit = 88;
        let mut no_extra = good.clone();
        no_extra.extra = None;
        let mut zero_uid = good.clone();
        zero_uid.data &= !0x0000_FFFF_FFFF_0000;
        for (what, sig) in [
            ("xor", bad_xor),
            ("header", bad_header),
            ("length", bad_len),
            ("extra", no_extra),
            ("uid", zero_uid),
        ] {
            assert!(enc.encode(&sig, 0x01).is_none(), "{what}");
        }
    }

    /// Encoder output decodes as Renault V1 only, at every supported frequency, through the full
    /// registry (seed path and doctor-firmware replay), with the fields the C would transmit.
    #[test]
    fn encoder_output_round_trips_through_registry() {
        let enc = RenaultV1Decoder::new();
        let cases = [
            (hex11("C2064F3E2B1CDFDB258625"), 0x02u8, Some(0xCAC8u16)),
            (pack(0x5EED1234, 0x1, 0x2C3, 0xA5A55A5A, 2), 0x04, None),
        ];
        for (raw, btn, fresh_counter) in cases {
            let up = enc.encode(&signal_for(&raw), btn).unwrap();
            let mut signal = vec![LevelDuration::new(false, 30_000)];
            signal.extend(up);
            for freq in [315_000_000u32, 433_920_000, 868_350_000] {
                let mut reg = ProtocolRegistry::new();
                let hits = reg.process_signal_stream(&signal, freq);
                assert_eq!(hits.len(), 3, "{freq}: one decode per long frame");
                for (name, d, _) in &hits {
                    assert_eq!(name, "Renault V1", "{freq}");
                    let uid = u32::from_be_bytes([raw[0], raw[1], raw[2], raw[3]]);
                    assert_eq!(d.serial, Some(uid));
                    match fresh_counter {
                        Some(cnt) => {
                            assert_eq!(d.counter, Some(cnt));
                            assert_eq!(d.button, Some(btn));
                        }
                        None => assert_eq!(RenaultV1Decoder::payload_from_signal(d), Some(raw)),
                    }
                }
            }
        }
    }

    /// Key files (ProtoPirate `subghz_protocol_decoder_renault_v1_serialize` / `_deserialize`):
    /// seed-scheme and no-seed frames round-trip with the C's lines as they read after PP's
    /// "Hitag2 BF"; a PP file saved before its brute force loads with the recovered counter; Bit
    /// ≠ 88 and frames failing the gates are refused; TX from a loaded key is the C's fresh frame.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = RenaultV1Decoder::new();
        let seed_frame = signal_for(&hex11("1234567848D122E6FC52FB"));
        assert!(assert_round_trip(&dec, &seed_frame).ends_with(
            "Protocol: Renault V1\nBit: 88\nKey: 12 34 56 78 48 D1 22 E6\nKey_2: 00 00 00 00 00 FC 52 FB\n\
             Recovered: 01\nSeed: 44 23 E1 01\nSerial: 305419896\nBtn: 4\nCnt: 4660\nHop: 1220132628\n"
        ));
        for frame in ["EFB82D6622345188720EAF", "C2064F3E2B1CDFDB258625", "378F852988DFF51255F203"] {
            assert_round_trip(&dec, &signal_for(&hex11(frame)));
        }
        let no_seed = signal_for(&pack(0x5EED1234, 0x1, 0x2C3, 0xA5A55A5A, 2));
        let text = assert_round_trip(&dec, &no_seed);
        assert!(text.contains("\nRecovered: 02\nSeed: 00 00 00 00\nSerial: 1592594996\nBtn: 1\nCnt: 707\n"), "{text}");

        // A file ProtoPirate saved straight after decoding (before its brute force): Recovered 00,
        // Seed 0, the 10-bit counter. KAT's decoder recovers the seed, so the counter is the 16-bit one.
        let pp = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                  Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Renault V1\nBit: 88\n\
                  Key: EF B8 2D 66 22 34 51 88\nKey_2: 00 00 00 00 00 72 0E AF\nRecovered: 00\n\
                  Seed: 00 00 00 00\nSerial: 4022873446\nBtn: 2\nCnt: 141\nHop: 0\n";
        let loaded = dec.from_flipper_key(&key(pp)).unwrap();
        assert_eq!(loaded, signal_for(&hex11("EFB82D6622345188720EAF")));
        assert_eq!((loaded.serial, loaded.button, loaded.counter), (Some(0xEFB8_2D66), Some(2), Some(0x008D)));
        assert!(dec.from_flipper_key(&key(&pp.replace("Bit: 88", "Bit: 104"))).is_none());
        assert!(dec.from_flipper_key(&key(&pp.replace("72 0E AF", "72 0E AE"))).is_none(), "XOR byte");
        assert!(dec.from_flipper_key(&key(&pp.replace("Key_2: 00 00 00 00 00 72 0E AF\n", ""))).is_none());
        // TX from the loaded key: a fresh seed-scheme frame, counter + 1.
        let up = dec.encode(&loaded, 0x01).unwrap();
        let tx = payload_of(&up);
        let d = signal_for(&tx[0]);
        assert_eq!((d.serial, d.button, d.counter), (Some(0xEFB8_2D66), Some(1), Some(0x008E)));
    }

    /// `KAT_KEY_VECTORS=<dir> cargo test renault_v1_oracle_vectors -- --ignored`: RAW uploads and
    /// KAT's key files for the C oracle (seed path and no-seed replay).
    #[test]
    #[ignore]
    fn renault_v1_oracle_vectors() {
        let Some(dir) = std::env::var_os("KAT_KEY_VECTORS") else { return };
        let dir = std::path::PathBuf::from(dir);
        let dec = RenaultV1Decoder::new();
        for (i, raw) in [hex11("1234567848D122E6FC52FB"), pack(0x5EED1234, 0x1, 0x2C3, 0xA5A55A5A, 2)].iter().enumerate() {
            let up = dec.encode(&signal_for(raw), 0x02).unwrap();
            let d = decode_all(&mut RenaultV1Decoder::new(), &up).remove(0);
            let key = dec.to_flipper_key(&d).unwrap();
            super::super::star_line::write_oracle_vector(&dir, &format!("renault_v1_{}", i + 1), 433_920_000, "FuriHalSubGhzPresetOok650Async", &up, &key);
        }
    }
}
