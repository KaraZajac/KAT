//! Protocol metadata: the one place KAT keeps the per-protocol facts the UI, the exports and the
//! transmitter use — line coding, RF (AM/FM), encryption label, make, native button names and the
//! transmit presets.
//!
//! Each protocol module declares its facts as `pub(super) const INFO: ProtocolInfo` (plus one const
//! per display variant with different facts: Kia V0's "Suzuki V0" and "Honda V0"), next to the
//! decoder they describe. [lookup] resolves every name the app can see:
//! - registry names ("Kia V3/V4") and decoder display names: a " [variant]" suffix is dropped
//!   ("Star Line [Cenmax]", "Renault V1 (ARF) [0x0C]", "Porsche Cayenne [First]"), and a module may
//!   own a name family (`name_prefix`: KeeLoq's "KeeLoq (<manufacturer>)" and the generic
//!   fallback's "Keeloq (<key>)");
//! - per-module aliases: the references' display names and catalog aliases ("Kia V3", "KIA/HYU V4",
//!   "VW", "StarLine") and the names of retired or renamed protocols ("Land Rover V0", "PSA2",
//!   "Suzuki", "Porsche Touareg"), which keep old captures and `.fob` files working.
//!
//! [super::ProtocolRegistry::get] resolves names to decoders through the same aliases. Adding a
//! protocol takes its module (decoder + `INFO`), one registry line and one line in [ALL]; the
//! tests fail if a registered decoder has no metadata.

pub(super) use crate::capture::{ModulationType as Coding, RfModulation as Rf};
use crate::radio::preset::PresetDef;

/// What a protocol module needs to declare its `INFO`.
pub(super) mod prelude {
    pub(in crate::protocols) use super::{Category, Coding, ProtocolInfo, Rf};
    pub(in crate::protocols) use crate::radio::preset::{AM650, F2, F3, F4, FM20K, FM476, HONDA1};
}

/// Label for a protocol's native button code; gets the display name without its " [variant]"
/// suffix (KeeLoq's labels depend on the manufacturer in it). None = KAT's generic names
/// (0x01 Lock, 0x02 Unlock, 0x04 Trunk, 0x08 Panic).
pub type ButtonNames = fn(name: &str, code: u8) -> Option<&'static str>;

/// What a protocol's remotes operate; decides the transmit menu.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Category {
    /// Car keyfob or car alarm: KAT's Lock / Unlock / Trunk / Panic commands.
    Vehicle,
    /// Garage door, gate, barrier or shutter remote: replay and "send next code" with the
    /// captured button (KeeLoq decides per manufacturer, see `keeloq_barriers.rs`).
    Gate,
}

/// Static facts about one protocol, or one display variant of it.
#[derive(Debug)]
pub struct ProtocolInfo {
    /// Display name the facts describe (the registry name for a protocol's main entry).
    pub name: &'static str,
    /// Registry name of the decoder/encoder that handles it (`ProtocolDecoder::name()`).
    pub decoder: &'static str,
    /// Other names for the same facts: reference display names and aliases, retired names.
    pub aliases: &'static [&'static str],
    /// Display names starting with this (ASCII case-insensitive) belong to this entry.
    pub name_prefix: Option<&'static str>,
    /// Line coding (PWM / Manchester / differential Manchester).
    pub encoding: Coding,
    /// RF modulation the references flag the protocol with (AM, FM or both).
    pub rf: Rf,
    /// Encryption / code type label.
    pub encryption: &'static str,
    /// Vehicle (or alarm / gate) make suggested for the protocol.
    pub make: &'static str,
    /// Car keyfob or gate/garage remote (menus).
    pub category: Category,
    /// Native button labels, when the protocol does not report KAT's generic codes.
    pub buttons: Option<ButtonNames>,
    /// Transmit presets, primary first; the deviation of an FSK preset is the one KAT transmits
    /// with when the capture carries no preset of its own (`docs/transmit.md`).
    pub presets: &'static [&'static PresetDef],
}

impl ProtocolInfo {
    /// Native label for a button code (None = KAT's generic names apply).
    pub fn button_name(&self, display_name: &str, code: u8) -> Option<&'static str> {
        self.buttons.and_then(|f| f(strip_variant(display_name), code))
    }
}

/// Every protocol's metadata (main entries and display variants), in registry order, then the
/// modules KAT keeps but does not register.
pub static ALL: &[&ProtocolInfo] = &[
    &super::kia_v0::INFO,
    &super::kia_v0::SUZUKI_V0_INFO,
    &super::kia_v0::HONDA_V0_INFO,
    &super::kia_v1::INFO,
    &super::kia_v2::INFO,
    &super::kia_v3_v4::INFO,
    &super::kia_v5::INFO,
    &super::kia_v6::INFO,
    &super::kia_v7::INFO,
    &super::vag::INFO,
    &super::ford_v0::INFO,
    &super::ford_v1::INFO,
    &super::ford_v3::INFO,
    &super::ford_v2::INFO,
    &super::honda_v5::INFO,
    &super::honda_static::INFO,
    &super::honda_v1::INFO,
    &super::subaru::INFO,
    &super::fiat_v0::INFO,
    &super::fiat_v1::INFO,
    &super::fiat_marelli::INFO,
    &super::scher_khan::INFO,
    &super::star_line::INFO,
    &super::sheriff_cfm::INFO,
    &super::keeloq::INFO,
    &super::psa::INFO,
    &super::chrysler_v0::INFO,
    &super::honda_v2::INFO,
    &super::mazda_v0::INFO,
    &super::mazda_siemens::INFO,
    &super::mitsubishi_v0::INFO,
    &super::porsche_cayenne::INFO,
    &super::toyota::INFO,
    &super::bmw::INFO,
    &super::mazda_v2::INFO,
    &super::mercedes_v0::INFO,
    &super::mercedes_v1::INFO,
    &super::opel_v1::INFO,
    &super::renault_v1::INFO,
    &super::renault_v0::INFO,
    &super::renault_v1_arf::INFO,
    &super::fiat_v2::INFO,
    &super::gm::INFO,
    &super::datsun_lada::INFO,
    // Garage door / gate / barrier / shutter remotes, in registry order; each porting group adds
    // its entries directly below its own anchor line.
    // [gate-info: G5 KeeLoq family]
    &super::beninca_arc::INFO,
    &super::kinggates_stylo_4k::INFO,
    &super::jarolift::INFO,
    &super::faac_slh::INFO,
    // [gate-info: G4 Security+ / Somfy]
    &super::secplus_v2::INFO,
    &super::somfy_keytis::INFO,
    &super::somfy_telis::INFO,
    &super::secplus_v1::INFO,
    // [gate-info: G6 Nice FloR-S / CAME Atomo / Alutech / Ditec]
    &super::alutech_at_4n::INFO,
    &super::nice_flor_s::INFO,
    &super::ditec_gol4::INFO,
    &super::came_atomo::INFO,
    // [gate-info: G3 misc]
    &super::power_smart::INFO,
    &super::urmet::INFO,
    &super::revers_rb2::INFO,
    &super::telcoma_edge::INFO,
    &super::phoenix_v2::INFO,
    &super::dooya::INFO,
    &super::roger::INFO,
    &super::hay21::INFO,
    &super::allstar_firefly::INFO,
    // [gate-info: G2 static B]
    &super::marantec::INFO,
    &super::hormann::INFO,
    &super::nero_radio::INFO,
    &super::nero_sketch::INFO,
    &super::ido::INFO,
    &super::dickert_mahs::INFO,
    &super::doitrand::INFO,
    &super::mastercode::INFO,
    &super::marantec24::INFO,
    &super::clemsa::INFO,
    &super::bett::INFO,
    &super::elplast::INFO,
    &super::ansonic::INFO,
    // [gate-info: G1 static A]
    &super::holtek::INFO,
    &super::came_twee::INFO,
    &super::megacode::INFO,
    &super::chamberlain_code::INFO,
    &super::gate_tx::INFO,
    &super::holtek_ht12x::INFO,
    &super::came::INFO,
    &super::prastel::INFO,
    &super::nice_flo::INFO,
    &super::linear::INFO,
    &super::linear_delta3::INFO,
    &super::smc5326::INFO,
    &super::princeton::INFO,
    // Flipper firmware ports.
    &super::hormann_bisecur::INFO,
    &super::gangqi::INFO,
    &super::hollarm::INFO,
    &super::cardin_s508::INFO,
    &super::freezer::INFO,
    &super::faac_tm::INFO,
    // Home / sensor / key-finder remotes.
    &super::honeywell::INFO,
    &super::magellan::INFO,
    &super::x10::INFO,
    &super::honeywell_wdb::INFO,
    &super::legrand::INFO,
    &super::doorbell32::INFO,
    &super::intertechno_v3::INFO,
    &super::keyfinder::INFO,
    &super::treadmill37::INFO,
    &super::nord_ice::INFO,
    &super::feron::INFO,
    &super::keyfinder2::INFO,
    // FoxFW: decode-only car remotes.
    &super::vag_polo::INFO,
    &super::chevrolet_v0::INFO,
    // FoxFW decoders.
    &super::tesla_charge::INFO,
    &super::hyundai_kia_rio::INFO,
    &super::santa_fe::INFO,
    &super::hitag2_rke::INFO,
    &super::nidec_rke::INFO,
    // FoxFW decoders.
    &super::renault_v4::INFO,
    &super::ducato_v0::INFO,
    &super::renault_v2::INFO,
    &super::opel_v0::INFO,
    // FoxFW Honda decoders (decode only).
    &super::honda_v4::INFO,
    &super::honda_v7::INFO,
    &super::honda_v9::INFO,
    &super::honda_v2_v3::INFO,
    &super::honda_v10::INFO,

    // FoxFW decoders (decode-only).
    &super::mitsubishi_v1::INFO,
    &super::ldv_v0::INFO,
    &super::lexus_v0::INFO,
    &super::mazda_v1::INFO,
    &super::nissan_v0::INFO,
    &super::toyota_foxfw::INFO,
    // FoxFW decoders.
    &super::audi_v2::INFO,
    &super::bmw_v2::INFO,
    &super::bmw_v3::INFO,
    &super::audi_v3::INFO,
    &super::bmw_cas3::INFO,

    // rtl_433 ports: car remotes.
    &super::siemens_5wy72xx::INFO,
    &super::alps_fwb1u545::INFO,
    &super::code_alarm_frdpc2002::INFO,
    &super::astrostart_2000::INFO,
    &super::chrysler_car_remote::INFO,
    &super::compustar_1wg3r::INFO,
    &super::audiovox_pro_oe3b::INFO,
    &super::honda_kr5v2x1x::INFO,
    &super::continental_kr5v2x::INFO,
    &super::mic6sc2::INFO,
    &super::opel_mokka::INFO,

    // rtl_433 ports: KeeLoq chips and shutter remotes.
    &super::elero::INFO,
    &super::somfy_iohc::INFO,
    &super::rojaflex::INFO,
    &super::microchip_hcs362::INFO,
    &super::microchip_hcs361::INFO,
    &super::cardin_s466::INFO,

    // Kept, not registered (as in Flipper-ARF; Markisol as in rtl_433).
    &super::land_rover_rke::INFO,
    &super::audi::INFO,
    &super::audi_v1::INFO,
    &super::hyundai::INFO,
    &super::mazda_infinity::INFO,
    &super::fiat_multi::INFO,
    &super::renault_v3::INFO,
    &super::markisol::INFO,
    // Ported from FoxFW, not registered (honda_v6.rs, honda_v8.rs).
    &super::honda_v6::INFO,
    &super::honda_v8::INFO,
];

/// A display name without its " [variant]" suffix ("Star Line [Cenmax]" → "Star Line").
pub fn strip_variant(name: &str) -> &str {
    name.split(" [").next().unwrap_or(name)
}

/// The entry whose name or alias is `name` (ASCII case-insensitive; no suffix or family rules).
pub(super) fn by_name(name: &str) -> Option<&'static ProtocolInfo> {
    ALL.iter().copied().find(|i| {
        i.name.eq_ignore_ascii_case(name) || i.aliases.iter().any(|a| a.eq_ignore_ascii_case(name))
    })
}

/// Metadata for any protocol name the app shows: registry and display names (with or without a
/// " [variant]" suffix), name families ("KeeLoq (DoorHan)") and legacy aliases. None for unknown
/// protocols ("Unknown", retired protocols without a successor such as "BMW CAS4").
pub fn lookup(name: &str) -> Option<&'static ProtocolInfo> {
    let base = strip_variant(name);
    by_name(base).or_else(|| {
        ALL.iter().copied().find(|i| {
            i.name_prefix.is_some_and(|p| {
                base.len() >= p.len()
                    && base.is_char_boundary(p.len())
                    && base[..p.len()].eq_ignore_ascii_case(p)
            })
        })
    })
}

/// Decoders of protocols that carry no checksum or other check KAT can verify: their frames never
/// report `crc_valid`, so the interface shows "-" for them rather than "FAIL". Each sets
/// `crc_valid: false` and nothing else (directly or through the gate modules' shared helpers);
/// `no_checksum_protocols_never_report_a_valid_crc` holds them to it on the test captures.
pub(super) const NO_CHECKSUM: &[&str] = &[
    // Fixed-code gate, garage, barrier and shutter remotes.
    "Allstar Firefly", "Ansonic", "BETT", "CAME", "CAME TWEE", "Cham_Code", "Clemsa", "Dickert_MAHS",
    "Doitrand", "Dooya", "Elplast", "GateTX", "Holtek", "Holtek_HT12X", "Hormann HSM", "Linear",
    "LinearDelta3", "Marantec24", "Mastercode", "MegaCode", "Nero Radio", "Nero Sketch", "Nice FLO",
    "Princeton", "Revers_RB2", "Roger", "SMC5326", "Telcoma/Cardin EDGE", "Urmet", "Freezer", "Faac TM",
    // Home / key-finder remotes without a check (Magellan has a CRC-8, X10 complement bytes).
    "Legrand", "Doorbell32", "Intertechno_V3", "KeyFinder", "Feron", "KeyFinder2", "Treadmill37", "Nord ICE",
    // Rolling codes the references decode without a check.
    "CAME Atomo", "Cardin S508", "Ditec GOL4", "Hay21", "iDo 117/111", "Phoenix_V2", "Prastel",
    "Security+ 1.0",
    // Car protocols whose frames carry no checksum: structural gates decide emission.
    // (Hyundai, unregistered, verifies nothing either: list it here when it is registered.)
    "Fiat V0", "Fiat V2", "Ford V2", "Ford V3", "Mercedes V0", "Mitsubishi V0", "Subaru",
    // Key files only: the file never holds the frame's CRC byte (see fiat_marelli.rs).
    "MARELLI",
    // FoxFW decoders whose C has no integrity check (structural gates, button whitelists, framing bytes).
    "Honda V10", "Renault V2", "Opel V0", "LDV V0", "Mazda V1", "Mitsubishi V1", "Nissan V0",
    "Audi V3", "BMW V2", "BMW/Mini V3", "BMW CAS3", "Toyota (FoxFW)",
    // rtl_433 ports whose C has no integrity check.
    "Alps-FWB1U545", "Audiovox-PROOE3B", "CodeAlarm-FRDPC2002", "Opel-Mokka", "Cardin-S466",
    // FoxFW's structural-gate car remotes.
    "VAG Polo", "Chevrolet V0",
];

/// Whether a decode shown as `name`, with `bits` bits and `extra`, had a check at all.
type Checked = fn(&str, usize, Option<u64>) -> bool;

/// Decoders that verify only some of their frames: the others show "-", not "FAIL".
const PARTIAL_CHECK: &[(&str, Checked)] = &[
    // PRO1/PRO2 (57 bits) carry a CRC-8; the Static, Dynamic and unknown variants nothing.
    ("Scher-Khan", |_, bits, _| bits == 57),
    // The 2-bit CRC field follows the 64 key bits; a frame cut at 64 bits carries none.
    ("Kia V5", |_, bits, _| bits > 64),
    // Nice One's 72-bit tail has a CRC; a 52-bit Nice FloR-S frame has none.
    ("Nice FloR-S", |name, _, _| strip_variant(name) == "Nice One"),
    // An unidentified KeeLoq frame was decrypted by no key.
    ("KeeLoq", |name, _, _| !name.eq_ignore_ascii_case("KeeLoq (Unknown)")),
    // Fiat Multi: V3's hop linear checks hold for every reported 70-bit frame; V5 (97 bits) has none.
    ("Fiat Multi", |_, bits, _| bits == 70),
    // Faac SLH frames verify only with the remote's seed (kept in `extra`).
    ("Faac SLH", |_, _, extra| extra.is_some()),
    // HondaV2&V3: a V3 frame (120 bits) has a CRC, a V2 frame (148 bits) none.
    ("HondaV2&V3", |_, bits, _| bits == 120),
    // Lexus V0 verifies a frame only once its complement partner was seen (`extra` = 1).
    ("Lexus V0", |_, _, extra| extra == Some(1)),
    // Continental KR5V2X: a received frame (112 bits) has its XOR checksum; a FoxFW key file keeps
    // bytes 0..7 only (64 bits), without it.
    ("Continental-KR5V2X", |_, bits, _| bits == 112),
];

/// Whether `name`'s protocol has a checksum (or another check) KAT verifies, so that a frame
/// without `crc_valid` failed it. False for the protocols in [`NO_CHECKSUM`]; true for names
/// [`lookup`] doesn't know. [has_checksum_for] also knows the decoders that check some frames only.
pub fn has_checksum(name: &str) -> bool {
    lookup(name).is_none_or(|i| !NO_CHECKSUM.contains(&i.decoder))
}

/// [has_checksum] for one decode: shown as `name`, `bits` bits, with `extra` (the capture's
/// `data_extra`), so that a frame of a decoder that checks only some frames ([PARTIAL_CHECK]) is
/// reported "not checked" rather than "failed".
pub fn has_checksum_for(name: &str, bits: usize, extra: Option<u64>) -> bool {
    has_checksum(name)
        && lookup(name).is_none_or(|i| {
            PARTIAL_CHECK.iter().find(|(d, _)| *d == i.decoder).is_none_or(|(_, f)| f(name, bits, extra))
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::Capture;
    use crate::protocols::{native_button_name, ProtocolRegistry};
    use std::collections::HashMap;

    /// The 1.4 value of each field, from the fixture written by KAT 1.4's own functions
    /// (`Capture::modulation` / `rf_modulation` / `encryption_type`, `App::get_make_for_protocol`,
    /// `protocols::native_button_name`) for every name the app could see.
    struct Legacy {
        name: String,
        fields: [String; 5],
    }

    const FIELDS: [&str; 5] = ["coding", "rf", "encryption", "make", "buttons"];

    fn legacy() -> Option<Vec<Legacy>> {
        let text = std::fs::read_to_string("testdata/protocol_metadata_1.4.txt").ok()?;
        Some(
            text.lines()
                .filter(|l| !l.starts_with('#'))
                .map(|l| {
                    let f: Vec<&str> = l.split(" | ").collect();
                    assert_eq!(f.len(), 6, "{l}");
                    Legacy {
                        name: f[0].to_string(),
                        fields: [f[1], f[2], f[3], f[4], f[5]].map(str::to_string),
                    }
                })
                .collect(),
        )
    }

    /// Native button names for codes 0..=255 as run-length segments ("0=Other,1=Lock,...").
    fn buttons(name: &str) -> String {
        let label = |c: u16| if c < 256 { native_button_name(name, c as u8) } else { Some("\u{0}") };
        let mut segs = Vec::new();
        let mut start = 0u16;
        for code in 1u16..=256 {
            if label(code) != label(start) || code == 256 {
                let l = label(start).unwrap_or("-");
                segs.push(if start == code - 1 { format!("{start}={l}") } else { format!("{start}-{}={l}", code - 1) });
                start = code;
            }
        }
        segs.join(",")
    }

    fn current(name: &str) -> [String; 5] {
        let mut c = Capture::from_pairs(0, 433_920_000, Vec::new());
        c.protocol = (!name.is_empty()).then(|| name.to_string());
        [
            c.modulation().to_string(),
            c.rf_modulation().to_string(),
            c.encryption_type().to_string(),
            c.protocol_make().to_string(),
            buttons(c.protocol_name()),
        ]
    }

    /// The deliberate differences from KAT 1.4: (name, field, new value). Everything else must be
    /// unchanged for every name in the fixture.
    const INTENTIONAL: &[(&str, &str, &str)] = &[
        // Kia V2: AM in ProtoPirate's flags/catalog (and ARF/doctor), FM476 in ProtoPirate's help
        // text and README and in the two real Kia V2 key files in IMPORTS → AM/FM (see kia_v2.rs).
        ("Kia V2", "rf", "AM/FM"),
        // Honda V1 carries a 16-bit counter (honda_v1.c builds the key from serial, button and
        // counter); ProtoPirate labels it "Rolling Code". KAT 1.4 said "Fixed Code".
        ("Honda V1", "encryption", "Rolling Code"),
        // The bare registry name had no metadata (its captures are always "KeeLoq (<mf>)").
        ("KeeLoq", "coding", "PWM"),
        ("KeeLoq", "rf", "AM"),
        ("KeeLoq", "encryption", "KeeLoq"),
        ("KeeLoq", "make", "KeeLoq"),
        // ... and, as names are matched case-insensitively (like `get()`), "Keeloq" too.
        ("Keeloq", "coding", "PWM"),
        ("Keeloq", "rf", "AM"),
        ("Keeloq", "encryption", "KeeLoq"),
        ("Keeloq", "make", "KeeLoq"),
        // BMW CAS4 was retired (Flipper-ARF disabled bmw_cas4.c and replaced it with "BMW", a
        // different frame); KAT 1.4 still carried its arms. It is an unknown protocol now.
        ("BMW CAS4", "coding", "Unknown"),
        ("BMW CAS4", "rf", "—"),
        ("BMW CAS4", "encryption", "Unknown"),
        ("BMW CAS4", "make", "Unknown"),
        // Aliases `get()` resolved for TX but the metadata tables did not (or only partly):
        // they now have the facts of the protocol they resolve to.
        ("PSA OLD", "coding", "Manchester"),
        ("PSA OLD", "rf", "AM/FM"),
        ("PSA OLD", "encryption", "TEA/XOR"),
        ("PSA GROUP", "coding", "Manchester"),
        ("PSA GROUP", "rf", "AM/FM"),
        ("PSA GROUP", "encryption", "TEA/XOR"),
        ("Mitsu V0", "coding", "PWM"),
        ("Mitsu V0", "rf", "FM"),
        ("Mitsu V0", "encryption", "Rolling Code"),
        ("Mitsu V0", "make", "Mitsubishi"),
        ("Mitsu V0", "buttons", "0=Other,1=Lock,2=Unlock,3=Trunk,4=Panic,5-255=Other"),
        ("KIA/HYU V3", "coding", "PWM"),
        ("KIA/HYU V3", "rf", "FM"),
        ("KIA/HYU V3", "encryption", "KeeLoq"),
        ("KIA/HYU V3", "make", "Kia/Hyundai"),
        ("KIA/HYU V3", "buttons", "0=Other,1=Lock,2=Unlock,3=Trunk,4=Panic,5-7=Other,8=Horn,9-255=Other"),
        ("KIA/HYU V4", "coding", "PWM"),
        ("KIA/HYU V4", "rf", "FM"),
        ("KIA/HYU V4", "encryption", "KeeLoq"),
        ("KIA/HYU V4", "make", "Kia/Hyundai"),
        ("KIA/HYU V4", "buttons", "0=Other,1=Lock,2=Unlock,3=Trunk,4=Panic,5-7=Other,8=Horn,9-255=Other"),
        // ProtoPirate catalog aliases (VW → VAG, StarLine → Star Line), now also in `get()`.
        ("VW", "coding", "Manchester"),
        ("VW", "rf", "AM"),
        ("VW", "encryption", "AUT64/XTEA"),
        ("VW", "buttons", "0=Other,1=Unlock,2=Lock,3=Other,4=Trunk,5-15=Other,16-31=Unlock,32-47=Lock,48-63=Other,64-79=Trunk,80-255=Other"),
        ("StarLine", "coding", "PWM"),
        ("StarLine", "rf", "AM"),
        ("StarLine", "encryption", "KeeLoq"),
        ("StarLine", "make", "Star Line"),
        ("StarLine", "buttons", "0=Other,1=Lock,2=Unlock,3=Trunk,4=Panic,5-32=Other,33=Lock,34=Unlock,35=Trunk,36=Start,37=Stop,38=Extra,39-255=Other"),
        // Names are matched case-insensitively, as `get()` always did.
        ("land rover v0", "coding", "Diff. Manchester"),
        ("land rover v0", "rf", "FM"),
        ("land rover v0", "encryption", "Rolling Code"),
        ("land rover v0", "make", "Honda/Acura"),
        ("land rover v0", "buttons", "0-1=Other,2=Lock,3=Other,4=Unlock,5-255=Other"),
        ("LAND ROVER V0", "coding", "Diff. Manchester"),
        ("LAND ROVER V0", "rf", "FM"),
        ("LAND ROVER V0", "encryption", "Rolling Code"),
        ("LAND ROVER V0", "make", "Honda/Acura"),
        ("LAND ROVER V0", "buttons", "0-1=Other,2=Lock,3=Other,4=Unlock,5-255=Other"),
        ("SUZUKI", "coding", "PWM"),
        ("SUZUKI", "rf", "AM/FM"),
        ("SUZUKI", "encryption", "Rolling Code"),
        ("SUZUKI", "make", "Suzuki"),
        ("SUZUKI", "buttons", "0=Other,1=Panic,2=Trunk,3=Lock,4=Unlock,5-255=Other"),
        ("Mitsu v0", "coding", "PWM"),
        ("Mitsu v0", "rf", "FM"),
        ("Mitsu v0", "encryption", "Rolling Code"),
        ("Mitsu v0", "make", "Mitsubishi"),
        ("Mitsu v0", "buttons", "0=Other,1=Lock,2=Unlock,3=Trunk,4=Panic,5-255=Other"),
        ("Mitsubishi v0", "coding", "PWM"),
        ("Mitsubishi v0", "rf", "FM"),
        ("Mitsubishi v0", "encryption", "Rolling Code"),
        ("Mitsubishi v0", "make", "Mitsubishi"),
        ("Mitsubishi v0", "buttons", "0=Other,1=Lock,2=Unlock,3=Trunk,4=Panic,5-255=Other"),
        ("psa2", "coding", "Manchester"),
        ("psa2", "rf", "AM/FM"),
        ("psa2", "encryption", "TEA/XOR"),
        ("psa2", "make", "Peugeot/Citroen"),
        ("psa2", "buttons", "0=Lock,1=Unlock,2=Trunk,3-255=Other"),
        ("honda v2", "coding", "Diff. Manchester"),
        ("honda v2", "rf", "FM"),
        ("honda v2", "encryption", "Rolling Code"),
        ("honda v2", "make", "Honda/Acura"),
        ("honda v2", "buttons", "0-1=Other,2=Lock,3=Other,4=Unlock,5-255=Other"),
        ("kia v0", "coding", "PWM"),
        ("kia v0", "rf", "AM/FM"),
        ("kia v0", "encryption", "Rolling Code"),
        ("kia v0", "make", "Kia/Hyundai"),
        ("kia v0", "buttons", "0=Other,1=Lock,2=Unlock,3=Trunk,4=Panic,5-255=Other"),
        // v1.8.7 review: Kia V7's labels are the references' (`kia_v7.c`: 1 Lock, 2 Unlock, 3 and 8
        // Trunk) — KAT 1.4 labelled 8 "Panic" like ProtoPirate's emulate key; Fiat V1 is AM | FM in
        // Flipper-ARF and doctor-firmware (union); Hyundai (unregistered) names the C's three codes.
        ("Kia V7", "buttons", "0=Other,1=Lock,2=Unlock,3=Trunk,4-7=Other,8=Trunk,9-255=Other"),
        ("Fiat V1", "rf", "AM/FM"),
        // Flipper-ARF and FoxFW flag Subaru AM + FM (ProtoPirate AM; November 2026).
        ("Subaru", "rf", "AM/FM"),
        ("Hyundai", "buttons", "0=Other,1=Lock,2=Unlock,3-8=Other,9=Trunk,10-255=Other"),
        // Prefix arms that matched names no protocol has: a bare "Ford" / "Fiat" / "Kia".
        ("Ford", "coding", "Unknown"),
        ("Ford", "rf", "—"),
        ("Ford", "encryption", "Unknown"),
        ("Ford", "make", "Unknown"),
        ("Fiat", "encryption", "Unknown"),
        ("Fiat", "make", "Unknown"),
        ("Kia", "make", "Unknown"),
    ];

    /// Goal of the v1.5 move: the per-module metadata reproduces KAT 1.4's tables for every name
    /// the app can see (registry names, every display name in the golden file, the KeeLoq / Star
    /// Line / Scher-Khan / Renault ARF / Porsche variants, aliases and retired names), except for
    /// the listed, deliberate fixes.
    #[test]
    fn metadata_matches_kat_1_4_except_the_listed_fixes() {
        crate::protocols::keys::load_keystore_from_embedded();
        let Some(legacy) = legacy() else {
            eprintln!("Skip: testdata/protocol_metadata_1.4.txt not found (run from crate root)");
            return;
        };
        assert!(legacy.len() > 300, "fixture has {} names", legacy.len());
        let intentional: HashMap<(&str, &str), &str> =
            INTENTIONAL.iter().map(|&(n, f, v)| ((n, f), v)).collect();
        let mut used = std::collections::HashSet::new();
        let mut diffs = Vec::new();
        for l in &legacy {
            let now = current(&l.name);
            for (k, field) in FIELDS.iter().enumerate() {
                let key = (l.name.as_str(), *field);
                let want = match intentional.get(&key) {
                    Some(v) => {
                        used.insert(key);
                        assert_ne!(*v, l.fields[k], "{key:?} is listed but unchanged from 1.4");
                        v.to_string()
                    }
                    None => l.fields[k].clone(),
                };
                if now[k] != want {
                    diffs.push(format!("{:?} {field}: 1.4 {:?}, expected {want:?}, now {:?}", l.name, l.fields[k], now[k]));
                }
            }
        }
        assert!(diffs.is_empty(), "metadata changed:\n{}", diffs.join("\n"));
        let unused: Vec<_> = INTENTIONAL.iter().filter(|(n, f, _)| !used.contains(&(*n, *f))).collect();
        assert!(unused.is_empty(), "INTENTIONAL entries for names not in the fixture: {unused:?}");
    }

    /// Every registered decoder has its own metadata entry, and every entry's decoder exists
    /// (registered, or one of the modules KAT keeps unregistered).
    #[test]
    fn every_decoder_has_metadata() {
        let reg = ProtocolRegistry::new();
        for name in reg.list_protocols() {
            let info = lookup(name).unwrap_or_else(|| panic!("{name}: no ProtocolInfo"));
            assert_eq!((info.name, info.decoder), (name, name), "{name}");
        }
        let unregistered = [
            "Land Rover RKE", "Audi", "Hyundai", "Mazda Infinity", "Markisol", "Fiat Multi", "Renault V3",
            "Honda V6", "Honda V8", "Audi V1", "Audi V2",
        ];
        for info in ALL {
            assert!(
                reg.get(info.decoder).is_some() || unregistered.contains(&info.decoder),
                "{}: decoder {} unknown",
                info.name,
                info.decoder
            );
            assert!(!info.presets.is_empty(), "{}: no transmit preset", info.name);
        }
        // Names and aliases are unique across entries.
        let mut seen = HashMap::new();
        for info in ALL {
            for n in std::iter::once(&info.name).chain(info.aliases) {
                if let Some(other) = seen.insert(n.to_ascii_lowercase(), info.name) {
                    panic!("{n} belongs to both {other} and {}", info.name);
                }
            }
        }
    }

    /// Every protocol name the golden IMPORTS decodes produce has metadata.
    #[test]
    fn golden_display_names_have_metadata() {
        let Ok(golden) = std::fs::read_to_string("testdata/imports_decodes.txt") else {
            eprintln!("Skip: testdata/imports_decodes.txt not found (run from crate root)");
            return;
        };
        for line in golden.lines() {
            let f: Vec<&str> = line.split(" | ").collect();
            if f.len() >= 4 {
                assert!(lookup(f[2]).is_some(), "{}: no metadata", f[2]);
            }
        }
    }

    /// Metadata and decoder resolution agree: wherever `get()` finds a decoder, `lookup()` finds
    /// that decoder's entry (or one of its display variants). `get()` only differs where it must:
    /// the generic KeeLoq fallback's "Keeloq (<key>)" captures are Kia V3/V4- or Star Line-format
    /// frames that the KeeLoq encoder would re-send in the wrong format, so they stay untransmittable.
    #[test]
    fn lookup_agrees_with_registry_get() {
        let Some(legacy) = legacy() else {
            return;
        };
        let reg = ProtocolRegistry::new();
        for l in &legacy {
            let info = lookup(&l.name);
            match (reg.get(&l.name), info) {
                (Some(d), Some(i)) => assert_eq!(d.name(), i.decoder, "{}", l.name),
                (Some(d), None) => panic!("{}: get() → {} but no metadata", l.name, d.name()),
                (None, Some(i)) => assert!(
                    l.name.starts_with("Keeloq (")
                        || ["Land Rover RKE", "Audi", "Hyundai", "Mazda Infinity"].contains(&i.decoder),
                    "{}: metadata ({}) but get() finds no decoder",
                    l.name,
                    i.name
                ),
                (None, None) => {}
            }
        }
    }

    /// The transmit presets follow the RF flags: FM protocols transmit 2-FSK, AM protocols OOK,
    /// AM/FM protocols carry one of each.
    #[test]
    fn presets_match_the_rf_flags() {
        for info in ALL {
            let fsk: Vec<bool> = info.presets.iter().map(|p| p.modulation().is_fsk()).collect();
            match info.rf {
                Rf::FM => assert_eq!(fsk, [true], "{}", info.name),
                Rf::AM => assert_eq!(fsk, [false], "{}", info.name),
                Rf::Both => {
                    assert_eq!(fsk.len(), 2, "{}", info.name);
                    assert_ne!(fsk[0], fsk[1], "{}", info.name);
                }
                Rf::Unknown => panic!("{}: RF unknown", info.name),
            }
        }
    }

    /// `docs/transmit.md` documents every FM and AM/FM protocol with the presets KAT uses.
    #[test]
    fn transmit_doc_lists_every_fsk_protocol() {
        let Ok(doc) = std::fs::read_to_string("docs/transmit.md") else {
            eprintln!("Skip: docs/transmit.md not found (run from crate root)");
            return;
        };
        for info in ALL.iter().filter(|i| matches!(i.rf, Rf::FM | Rf::Both)) {
            let row = doc
                .lines()
                .find(|l| l.starts_with(&format!("| {} |", info.name)))
                .unwrap_or_else(|| panic!("docs/transmit.md: no row for {}", info.name));
            let presets: Vec<&str> = info.presets.iter().map(|p| p.name).collect();
            assert!(row.contains(&format!("| {} |", presets.join(" + "))), "{}: {row}", info.name);
        }
    }

    #[test]
    fn lookup_examples() {
        let name = |n: &str| lookup(n).map(|i| i.name);
        assert_eq!(name("Kia V3"), Some("Kia V3/V4"));
        assert_eq!(name("Suzuki"), Some("Suzuki V0"));
        assert_eq!(lookup("Suzuki V0").map(|i| (i.decoder, i.make)), Some(("Kia V0", "Suzuki")));
        assert_eq!(lookup("Honda V0").map(|i| (i.decoder, i.make)), Some(("Kia V0", "Honda/Acura")));
        assert_eq!(lookup("Mitsubishi V0").map(|i| (i.decoder, i.rf)), Some(("Mitsubishi V0", Rf::FM)));
        assert_eq!(name("Land Rover V0"), Some("Honda V2"));
        assert_eq!(name("PSA2"), Some("PSA"));
        assert_eq!(name("Porsche Touareg [Final]"), Some("Porsche Cayenne"));
        assert_eq!(name("Renault V1 (ARF) [Dynamic 0x2A]"), Some("Renault V1 (ARF)"));
        assert_eq!(name("Renault V1"), Some("Renault V1"));
        assert_eq!(name("KeeLoq (DoorHan)"), Some("KeeLoq"));
        assert_eq!(name("Keeloq (Star Line)"), Some("KeeLoq"));
        assert_eq!(name("Star Line [Faraon]"), Some("Star Line"));
        assert_eq!(name("BMW CAS4"), None);
        assert_eq!(name("Unknown"), None);
        assert_eq!(name("Ké"), None, "non-ASCII input");
        // Display variants pick their own TX facts through the decoder's entry.
        assert_eq!(lookup("Ford V1").unwrap().presets[0].name, "F4");
        assert_eq!(lookup("Honda Static").unwrap().presets[0].name, "Honda1");
        assert_eq!(lookup("Kia V4").unwrap().presets[0].name, "FM476");
        assert_eq!(lookup("PSA").unwrap().presets.iter().map(|p| p.name).collect::<Vec<_>>(), ["AM650", "F3"]);
    }
}
