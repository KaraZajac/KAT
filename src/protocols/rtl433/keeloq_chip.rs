//! Microchip's KeeLoq code-hopping encoders (HCS361, HCS362) through KAT's KeeLoq machinery.

use super::frame_signal;
use crate::protocols::{info, keeloq, microchip_hcs361, microchip_hcs362, DecodedSignal};

/// A KeeLoq frame from one of Microchip's code-hopping encoders (HCS361 / HCS362) that passed its
/// chip's checks, through KAT's KeeLoq machinery: the first 64 bits are the same 32-bit hop +
/// 28-bit serial + 4-bit function code an HCS200 sends, so `keeloq.rs`'s identification (embedded
/// and user keystore, every learning type) names the manufacturer and decrypts the counter, or
/// reports "KeeLoq (Unknown)" with the serial and button from the fixed part. `model` is rtl_433's
/// model name; the decode is shown as `<model> [KeeLoq (<manufacturer>)]`. `crc_valid`: the chip's
/// CRC passed. Receive only: KAT's KeeLoq encoder sends the HCS200 line coding, not these chips'.
pub fn keeloq_chip_signal(model: &str, frame: &[u8], bit_count: usize) -> DecodedSignal {
    let mut d = frame_signal(frame, bit_count);
    if let Some(kl) = keeloq::KeeloqDecoder::parse_data(d.data) {
        d.serial = kl.serial;
        d.button = kl.button;
        d.counter = kl.counter;
        let kl_name = kl.protocol_display_name.unwrap_or_else(|| keeloq::UNKNOWN_NAME.to_string());
        d.protocol_display_name = Some(format!("{model} [{kl_name}]"));
    }
    d.crc_valid = true;
    d.encoder_capable = false;
    d
}

/// Whether a display name is a Microchip HCS361 / HCS362 capture's (`Microchip-HCS36x [KeeLoq (…)]`),
/// whose `data` is the KeeLoq code word an HCS200 sends (KeeLoq key recovery takes those frames).
pub fn is_keeloq_chip(protocol: &str) -> bool {
    matches!(info::strip_variant(protocol), microchip_hcs361::NAME | microchip_hcs362::NAME)
}

/// App integration point (KeeLoq key recovery, `app/keeloq_bf.rs`): a Microchip HCS361 / HCS362
/// capture — its display name (`Microchip-HCS36x [KeeLoq (…)]`), `data`, `data_extra` and
/// `data_count_bit` — identified again with the current keystore, e.g. after a recovered key was
/// added; `Frame::from_data(data)` already gives the search its fix / hop. None for anything else.
pub fn redecode_keeloq_chip(protocol: &str, data: u64, extra: Option<u64>, bit_count: usize) -> Option<DecodedSignal> {
    let model = info::strip_variant(protocol);
    let tail_bits = match (model, bit_count) {
        (microchip_hcs361::NAME, 67) => 3,
        (microchip_hcs362::NAME, 69) => 5,
        _ => return None,
    };
    let mut frame = data.to_be_bytes().to_vec();
    frame.push(((extra? << (8 - tail_bits)) & 0xFF) as u8);
    Some(keeloq_chip_signal(model, &frame, bit_count))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keeloq_chip_captures_redecode_to_the_same_signal() {
        crate::protocols::keys::load_keystore_from_embedded();
        let d = keeloq_chip_signal("Microchip-HCS361", &[0x1D, 0xB4, 0xF1, 0x4A, 0x84, 0x90, 0xB0, 0x04, 0xA0], 67);
        let name = d.protocol_display_name.clone().unwrap();
        assert_eq!(name, "Microchip-HCS361 [KeeLoq (Magic_3)]");
        assert_eq!(redecode_keeloq_chip(&name, d.data, d.extra, 67), Some(d.clone()));
        let e = keeloq_chip_signal("Microchip-HCS362", &[0x55, 0xA8, 0x22, 0x25, 0xB3, 0x52, 0xE8, 0x04, 0x58], 69);
        let name = e.protocol_display_name.clone().unwrap();
        assert_eq!(redecode_keeloq_chip(&name, e.data, e.extra, 69), Some(e));
        assert_eq!(redecode_keeloq_chip("KeeLoq (Unknown)", d.data, d.extra, 64), None);
        assert_eq!(redecode_keeloq_chip(&name, d.data, None, 69), None);
        assert!(is_keeloq_chip("Microchip-HCS362 [KeeLoq (Unknown)]") && !is_keeloq_chip("KeeLoq (Unknown)"));
    }
}
