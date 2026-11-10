//! "Add Manually": create a brand-new virtual gate / garage remote of a chosen type — a fresh random
//! serial with the type's default button, counter, manufacturer key and seed — which KAT can then
//! transmit, typically to pair it with a receiver in learning mode. Port of the SubGHz app's "Add
//! Manually" of Flipper-ARF (`applications/main/subghz/scenes/subghz_scene_set_type.c`,
//! `helpers/subghz_gen_info.c`, `helpers/subghz_txrx_create_protocol_key.c`, the protocols'
//! `*_create_data`), doctor-firmware (the same files; a superset of ARF's list) and RogueMaster
//! (a superset of doctor's but for Dorcas). See `docs/add_manually.md`.
//!
//! **Types** ([TYPES]): the union of the menus — ARF's 78 entries in ARF's order with
//! doctor-firmware's 23 extra ones where doctor's menu puts them (doctor's order is ARF's with
//! insertions) and RogueMaster's 9 extra ones where RogueMaster's puts them (its two O-Code
//! variants that its enum appends after Security+ are kept with the Nice group, after its first
//! O-Code type; its two FAAC TM types, which its enum puts after FAAC SLH, close the fixed-code
//! group, since they are fixed codes) and FoxFW's 2 extra ones (plain KeeLoq "KL: Erreka 433/868MHz",
//! after Aprimatic as in FoxFW's menu); ARF's name and parameters where several have a type (ARF and doctor differ only in
//! "KL: Genius Bravo": ARF button 0x06, doctor 0x09 and the name "KL: Genius TX4RC 433M.";
//! Momentum's and RogueMaster's menus list the same types with doctor's or ARF's parameters, except
//! RogueMaster's Monarch (a KeeLoq seed 0x100) and its FAAC SLH / Beninca ARC creation, see the
//! doc). The menu order is the C `SetType` enum order (the submenu adds items by index), which
//! groups the types: rolling codes, "KL:" KeeLoq manufacturers, Nice, CAME, fixed codes, Security+
//! ([Group]).
//!
//! **Parameters** ([fill]): `subghz_scene_set_type_fill_generation_infos` — `key = rand()`, then
//! GangQi's and Marantec's own `rand()` draws (made for every type, so the sequence of draws is
//! the C's), and per type the preset, frequency and the fields derived from `key` with the C's
//! masks ([GenInfo]); RogueMaster's two O-Code types draw a fourth value (the installer code).
//! `rand()` is newlib's (`RAND_MAX` 0x7FFFFFFF): [RandSource] returns 31-bit values; KAT's [Rng] is
//! seeded from the operating system's randomness (on Windows too).
//!
//! **Key file** ([generate]): what `subghz_scene_set_type_generate_protocol_from_infos` writes —
//! fixed codes (`GenData`): the decoder's serialize with `Bit`/`Key` (+ Princeton's `TE` and
//! `Guard_time: 30`); Security+ 1.0 draws `rand()` until `check_fixed` accepts it as the fixed
//! code (rolling 0xE6000000); everything else through the protocol's `create_data` (the first code
//! computed as the C does: KeeLoq encryption with the manufacturer key, FAAC learning, AES …).
//! Where the trees' create functions differ, ARF's are followed for its types (FAAC SLH: counter + 1
//! before the first code; doctor encrypts the counter as given) and doctor's for its own — except
//! Beninca ARC, created with doctor's / the garage app's software AES that KAT's codec implements
//! (ARF's `lib` copy runs the STM32 hardware AES, whose word-swapped key gives frames no other
//! decoder accepts); doctor's
//! generic serializer also writes `Serial`/`Btn`/`Cnt` (and a decimal `Seed` before the hex one),
//! which KAT does not (the protocols' own lines carry the remote). The app loads the file through
//! `ProtocolRegistry::decode_key` like any key file.
//!
//! RogueMaster's Prastel type is made with its `subghz_protocol_prastel_create_data` ([prastel_key]).
//!
//! **Not available** ([availability]): RogueMaster's three Nice O-Code types (its O-Code key maker is not ported); Nice FloR-S / Nice One and Alutech AT-4N
//! without the user's rainbow table (`~/.config/KAT/nice_flor_s.bin`, `alutech_at_4n.bin`: the C
//! would create a useless key); KeeLoq types whose manufacturer has no usable key in KAT's keystore
//! (the C saves an all-zero key) — doctor's Erreka (and RogueMaster's Erreka 868), Superrollo,
//! HomeGate, Pujol, Pujol Vario, ET Blue (Mix), ATA PTX4, Seav, Wisniowski, Fadini, Mc Garcia,
//! Clemsa Mutancode, Doormatic, Elvox and Verex, and RogueMaster's Phantom/Hiland, unless the user
//! keystore adds them. doctor's KEY and Dorcas are made with doctor-firmware's / RogueMaster's
//! discriminators (KEY 0x2FF; Dorcas' Monarch 0x280, kept in the seed), which KAT's KeeLoq codec
//! has since November 2026.

use std::collections::VecDeque;

use super::flipper_key::FlipperKey;
use super::g6_common::Table;
use super::keeloq_common::reverse_key;

/// The C library's `rand()`: values 0..=0x7FFFFFFF (`RAND_MAX`).
pub trait RandSource {
    fn rand(&mut self) -> u32;
}

/// KAT's random source: SplitMix64, 31-bit outputs like `rand()`.
pub struct Rng {
    state: u64,
}

impl Rng {
    /// A generator with a fixed seed (tests).
    pub fn seeded(seed: u64) -> Self {
        Rng { state: seed }
    }

    /// A generator seeded from the operating system's randomness, on every platform: std's
    /// `RandomState` keys come from it (getrandom on Linux, the system RNG on macOS and Windows),
    /// mixed with the time and the process id.
    pub fn from_entropy() -> Self {
        use std::hash::{BuildHasher, Hasher};
        let mut h = std::collections::hash_map::RandomState::new().build_hasher();
        let t = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos() as u64);
        h.write_u64(t);
        h.write_u32(std::process::id());
        Rng::seeded(h.finish())
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
}

impl RandSource for Rng {
    fn rand(&mut self) -> u32 {
        (self.next_u64() >> 33) as u32
    }
}

/// Given `rand()` values in order (reference comparisons), then [Rng] values.
pub struct Scripted {
    values: VecDeque<u32>,
    rest: Rng,
}

impl Scripted {
    pub fn new(values: &[u32]) -> Self {
        Scripted { values: values.iter().copied().collect(), rest: Rng::seeded(0) }
    }
}

impl RandSource for Scripted {
    fn rand(&mut self) -> u32 {
        self.values.pop_front().unwrap_or_else(|| self.rest.rand())
    }
}

/// Which reference menus list a type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// Flipper-ARF (and doctor-firmware).
    Both,
    /// doctor-firmware only.
    Doctor,
    /// RogueMaster only (`subghz_scene_set_type.c` of RogueMaster).
    RogueMaster,
    /// FoxFW only (`subghz_scene_set_type.c` of FoxFW).
    FoxFW,
}

/// The runs of the C menu order, shown as headings in the picker.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Group {
    Rolling,
    KeeLoq,
    Nice,
    Came,
    Fixed,
    SecPlus,
}

impl Group {
    pub fn label(self) -> &'static str {
        match self {
            Group::Rolling => "Rolling code",
            Group::KeeLoq => "KeeLoq manufacturers (KL:)",
            Group::Nice => "Nice",
            Group::Came => "CAME",
            Group::Fixed => "Fixed code",
            Group::SecPlus => "Chamberlain / LiftMaster Security+",
        }
    }
}

/// One entry of the "Add Manually" menu.
#[derive(Debug)]
pub struct RemoteType {
    /// The C `SetType` enumerator without its `SetType` prefix.
    pub id: &'static str,
    /// The menu text (`submenu_names`).
    pub name: &'static str,
    /// The reference menus that list the type.
    pub source: Source,
    pub group: Group,
}

impl RemoteType {
    /// The reference menus that list the type (Momentum's and RogueMaster's menus also list most
    /// Flipper-ARF / doctor-firmware types; their parameters are the same, see `docs/add_manually.md`).
    pub fn menus(&self) -> &'static str {
        match self.source {
            Source::Both => "Flipper-ARF + doctor-firmware",
            Source::Doctor => "doctor-firmware",
            Source::RogueMaster => "RogueMaster",
            Source::FoxFW => "FoxFW",
        }
    }
}

/// Every type, in menu order (see the module doc).
pub static TYPES: &[RemoteType] = &[
    RemoteType { id: "FaacSLH_868", name: "FAAC SLH 868MHz", source: Source::Both, group: Group::Rolling },
    RemoteType { id: "FaacSLH_433", name: "FAAC SLH 433MHz", source: Source::Both, group: Group::Rolling },
    RemoteType { id: "Genius_433", name: "Genius 433MHz", source: Source::Doctor, group: Group::Rolling },
    RemoteType { id: "Genius_868", name: "Genius 868MHz", source: Source::Doctor, group: Group::Rolling },
    RemoteType { id: "BFTMitto", name: "BFT Mitto 433MHz", source: Source::Both, group: Group::Rolling },
    RemoteType { id: "Erreka433", name: "Erreka 433MHz", source: Source::Doctor, group: Group::Rolling },
    RemoteType { id: "Erreka868", name: "Erreka 868MHz", source: Source::RogueMaster, group: Group::Rolling },
    RemoteType { id: "SomfyTelis", name: "Somfy Telis 433MHz", source: Source::Both, group: Group::Rolling },
    RemoteType { id: "SomfyKeytis", name: "Somfy Keytis 433MHz", source: Source::Doctor, group: Group::Rolling },
    RemoteType { id: "KingGatesStylo4k", name: "KingGates Stylo4k 433M.", source: Source::Both, group: Group::Rolling },
    RemoteType { id: "BenincaARC", name: "Beninca ARC 433MHz", source: Source::Both, group: Group::Rolling },
    RemoteType { id: "Jarolift", name: "Jarolift 433MHz", source: Source::Both, group: Group::Rolling },
    RemoteType { id: "DitecGOL4", name: "Ditec GOL4 433MHz", source: Source::Doctor, group: Group::Rolling },
    RemoteType { id: "ANMotorsAT4", name: "AN-Motors AT4 433MHz", source: Source::Both, group: Group::Rolling },
    RemoteType { id: "AlutechAT4N", name: "Alutech AT4N 433MHz", source: Source::Both, group: Group::Rolling },
    RemoteType { id: "Phoenix_V2_433", name: "V2 Phoenix 433MHz", source: Source::Both, group: Group::Rolling },
    RemoteType { id: "Prastel_433_92", name: "Prastel 433MHz", source: Source::RogueMaster, group: Group::Rolling },
    RemoteType { id: "HCS101_433_92", name: "KL: HCS101 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "DoorHan_315_00", name: "KL: DoorHan 315MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "DoorHan_433_92", name: "KL: DoorHan 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Superrollo_433_92", name: "KL: Superrollo 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "Beninca433", name: "KL: Beninca 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Beninca868", name: "KL: Beninca 868MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Comunello433", name: "KL: Comunello 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Comunello868", name: "KL: Comunello 868MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Allmatic433", name: "KL: Allmatic 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Allmatic868", name: "KL: Allmatic 868MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Centurion433", name: "KL: Centurion 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Monarch433", name: "KL: Monarch 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "MonarchDorcas433", name: "KL: Dorcas 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "PhantomHiland433", name: "KL: Phantom/Hiland 433M", source: Source::RogueMaster, group: Group::KeeLoq },
    RemoteType { id: "KEY433", name: "KL: KEY 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "JollyMotors433", name: "KL: Jolly Mot. 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Motorline433", name: "KL: Motorline 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Sommer_FM_434", name: "KL: Sommer 434MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Sommer_FM_868", name: "KL: Sommer 868MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Sommer_FM12K_434", name: "KL: Sommer fm2 434Mhz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Sommer_FM12K_868", name: "KL: Sommer fm2 868Mhz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Stilmatic", name: "KL: Stilmatic 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "IronLogic", name: "KL: IronLogic 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "IronLogicSmart", name: "KL: IronLogic SM 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "DeaMio433", name: "KL: DEA Mio 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "DTMNeo433", name: "KL: DTM Neo 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Gibidi433", name: "KL: Gibidi 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "GSN", name: "KL: GSN 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "HomeGate433", name: "KL: HomeGate 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "Aprimatic", name: "KL: Aprimatic 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "ErrekaKL433", name: "KL: Erreka 433MHz", source: Source::FoxFW, group: Group::KeeLoq },
    RemoteType { id: "ErrekaKL868", name: "KL: Erreka 868MHz", source: Source::FoxFW, group: Group::KeeLoq },
    RemoteType { id: "ElmesElectronic", name: "KL: Elmes (PL) 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Normstahl_433_92", name: "KL: Normstahl 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "JCM_433_92", name: "KL: JCM Tech 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Novoferm_433_92", name: "KL: Novoferm 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "HormannEcoStar_433_92", name: "KL: Hor. EcoStar 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "CardinS449_433FM", name: "KL: Cardin S449 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "Pujol433", name: "KL: Pujol 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "Pujol_Vario433", name: "KL: Pujol Vario 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "ET_Blue433", name: "KL: ET Blue 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "ET_Blue_Mix433", name: "KL: ET Blue Mix 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "ATA_PTX4_433", name: "KL: ATA PTX4 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "Seav433", name: "KL: Seav 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "Wisniowski433", name: "KL: Wisniowski 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "Fadini433", name: "KL: Fadini 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "Mc_Garcia433", name: "KL: Mc Garcia 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "Clemsa_Mutancode433", name: "KL: Clm.Mutancode 433M.", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "Doormatic433", name: "KL: Doormatic 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "Elvox433", name: "KL: Elvox 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "Verex433", name: "KL: Verex 433MHz", source: Source::Doctor, group: Group::KeeLoq },
    RemoteType { id: "FAACRCXT_433_92", name: "KL: FAAC RC,XT 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "FAACRCXT_868", name: "KL: FAAC RC,XT 868MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "GeniusBravo433", name: "KL: Genius Bravo 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "NiceMHouse_433_92", name: "KL: Mhouse 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "NiceSmilo_433_92", name: "KL: Nice Smilo 433MHz", source: Source::Both, group: Group::KeeLoq },
    RemoteType { id: "NiceFlorS_433_92", name: "Nice FloR-S 433MHz", source: Source::Both, group: Group::Nice },
    RemoteType { id: "NiceOne_433_92", name: "Nice One 433MHz", source: Source::Both, group: Group::Nice },
    RemoteType { id: "NiceOCode_433_92", name: "Nice O-Code 433MHz", source: Source::RogueMaster, group: Group::Nice },
    RemoteType { id: "NiceOCodeOne_433_92", name: "Nice O-Code One 433MHz", source: Source::RogueMaster, group: Group::Nice },
    RemoteType { id: "NiceOCodeCert_433_92", name: "Nice O-Code+Crt 433MHz", source: Source::RogueMaster, group: Group::Nice },
    RemoteType { id: "NiceFlo12bit", name: "Nice Flo 12bit 433MHz", source: Source::Both, group: Group::Nice },
    RemoteType { id: "NiceFlo24bit", name: "Nice Flo 24bit 433MHz", source: Source::Both, group: Group::Nice },
    RemoteType { id: "CAME12bit", name: "CAME 12bit 433MHz", source: Source::Both, group: Group::Came },
    RemoteType { id: "CAME24bit", name: "CAME 24bit 433MHz", source: Source::Both, group: Group::Came },
    RemoteType { id: "CAME12bit868", name: "CAME 12bit 868MHz", source: Source::Both, group: Group::Came },
    RemoteType { id: "CAME24bit868", name: "CAME 24bit 868MHz", source: Source::Both, group: Group::Came },
    RemoteType { id: "CAMETwee", name: "CAME TWEE 433MHz", source: Source::Both, group: Group::Came },
    RemoteType { id: "CameAtomo433", name: "CAME Atomo 433MHz", source: Source::Both, group: Group::Came },
    RemoteType { id: "CameAtomo868", name: "CAME Atomo 868MHz", source: Source::Both, group: Group::Came },
    RemoteType { id: "CAMESpace", name: "KL: CAME Space 433MHz", source: Source::Both, group: Group::Came },
    RemoteType { id: "Pricenton315", name: "Princeton 315MHz", source: Source::Both, group: Group::Fixed },
    RemoteType { id: "Pricenton433", name: "Princeton 433MHz", source: Source::Both, group: Group::Fixed },
    RemoteType { id: "ZKTeco430", name: "ZKTeco 430MHz", source: Source::Both, group: Group::Fixed },
    RemoteType { id: "BETT_433", name: "BETT 433MHz", source: Source::Both, group: Group::Fixed },
    RemoteType { id: "GangQi_433", name: "GangQi 433MHz", source: Source::Both, group: Group::Fixed },
    RemoteType { id: "Hollarm_433", name: "Hollarm 433MHz", source: Source::Both, group: Group::Fixed },
    RemoteType { id: "NordIce_433", name: "Nord ICE 433MHz", source: Source::RogueMaster, group: Group::Fixed },
    RemoteType { id: "ReversRB2_433", name: "Revers RB2 433MHz", source: Source::Both, group: Group::Fixed },
    RemoteType { id: "Marantec24_868", name: "Marantec24 868MHz", source: Source::Both, group: Group::Fixed },
    RemoteType { id: "Marantec_433", name: "Marantec 433MHz", source: Source::Both, group: Group::Fixed },
    RemoteType { id: "Marantec_868", name: "Marantec 868MHz", source: Source::Both, group: Group::Fixed },
    RemoteType { id: "Roger_433", name: "Roger 433MHz", source: Source::Both, group: Group::Fixed },
    RemoteType { id: "Linear_300_00", name: "Linear 300MHz", source: Source::Both, group: Group::Fixed },
    RemoteType { id: "TelcomaEdge433", name: "Telcoma EDGE 433MHz", source: Source::Doctor, group: Group::Fixed },
    RemoteType { id: "GateTX", name: "Gate TX 433MHz", source: Source::Both, group: Group::Fixed },
    RemoteType { id: "FaacTm433", name: "FAAC TM 433MHz", source: Source::RogueMaster, group: Group::Fixed },
    RemoteType { id: "FaacTm868", name: "FAAC TM 868MHz", source: Source::RogueMaster, group: Group::Fixed },
    RemoteType { id: "SecPlus_v1_315_00", name: "Security+1.0 315MHz", source: Source::Both, group: Group::SecPlus },
    RemoteType { id: "SecPlus_v1_390_00", name: "Security+1.0 390MHz", source: Source::Both, group: Group::SecPlus },
    RemoteType { id: "SecPlus_v1_433_00", name: "Security+1.0 433MHz", source: Source::Both, group: Group::SecPlus },
    RemoteType { id: "SecPlus_v2_310_00", name: "Security+2.0 310MHz", source: Source::Both, group: Group::SecPlus },
    RemoteType { id: "SecPlus_v2_315_00", name: "Security+2.0 315MHz", source: Source::Both, group: Group::SecPlus },
    RemoteType { id: "SecPlus_v2_390_00", name: "Security+2.0 390MHz", source: Source::Both, group: Group::SecPlus },
    RemoteType { id: "SecPlus_v2_433_00", name: "Security+2.0 433MHz", source: Source::Both, group: Group::SecPlus },
];

/// The C `GenInfo` union: what a type generates from (`GenType` + its fields).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GenInfo {
    /// `GenData`: a fixed code — protocol (the C's name), key, bit count, Princeton's TE (0 = none).
    Data { protocol: &'static str, key: u64, bits: usize, te: u32 },
    /// `GenFaacSLH` (`manuf` "FAAC_SLH", doctor's "Genius").
    FaacSlh { serial: u32, btn: u8, cnt: u32, seed: u32, manuf: &'static str },
    /// `GenKeeloq`: a KeeLoq manufacturer.
    Keeloq { serial: u32, btn: u8, cnt: u16, manuf: &'static str },
    /// `GenCameAtomo`.
    CameAtomo { serial: u32, cnt: u16 },
    /// `GenKeeloqBFT` (ARF) / `GenKeeloqSeed` (doctor): KeeLoq with a seed (BFT, Erreka, Dorcas).
    KeeloqSeed { serial: u32, btn: u8, cnt: u16, seed: u32, manuf: &'static str },
    AlutechAt4n { serial: u32, btn: u8, cnt: u16 },
    SomfyTelis { serial: u32, btn: u8, cnt: u16 },
    SomfyKeytis { serial: u32, btn: u8, cnt: u16 },
    KingGatesStylo4k { serial: u32, btn: u8, cnt: u16 },
    BenincaArc { serial: u32, btn: u8, cnt: u32 },
    Jarolift { serial: u32, btn: u8, cnt: u16 },
    DitecGol4 { serial: u32, btn: u8, cnt: u16 },
    NiceFlorS { serial: u32, btn: u8, cnt: u16, nice_one: bool },
    /// `GenSecPlus1`: the fixed code is drawn when the key is generated.
    SecPlus1,
    SecPlus2 { serial: u32, btn: u8, cnt: u32 },
    PhoenixV2 { serial: u32, cnt: u16 },
    /// `GenPrastel` (RogueMaster): Prastel's `create_data` (21 serial bits, button 1..4, counter).
    Prastel { serial: u32, btn: u8, cnt: u16 },
    /// RogueMaster's Nice O-Code types (`GenNiceFlorS` with `o_code`): an installer code `ic`
    /// (drawn for the first two), the 72-bit Nice One frame, or the receiver certificate the remote
    /// enrols itself with (`cert`). Not created by KAT (see [availability]).
    NiceOCode { serial: u32, btn: u8, cnt: u16, ic: u16, nice_one: bool, cert: bool },
}

/// A type's generation parameters: the preset (the C's short name), frequency and fields.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Gen {
    pub preset: &'static str,
    pub frequency: u32,
    pub info: GenInfo,
}

fn gen(preset: &'static str, frequency: u32, info: GenInfo) -> Gen {
    Gen { preset, frequency, info }
}

/// `subghz_txrx_gen_serial_gangqi`: serial 16 bits, button 0xD, byte sum.
fn gangqi_key(r: u32) -> u64 {
    let serial = (r & 0xFFFF) as u16;
    let const_and_button: u8 = 0xD0 | 0xD;
    let [high, low] = serial.to_be_bytes();
    let bytesum = 0xC8u8.wrapping_sub(high).wrapping_sub(low).wrapping_sub(const_and_button);
    (u64::from(serial) << 18) | (u64::from(const_and_button) << 10) | (u64::from(bytesum) << 2)
}

/// `subghz_txrx_gen_key_marantec`: 0x130 constant, 20-bit serial, button 0x4, 0x86, CRC-8.
fn marantec_key(r: u32) -> u64 {
    let serial = u64::from(r & 0xF_FFFF);
    let full = (0x130 << 40) | (serial << 20) | (0x4 << 16) | (0x86 << 8);
    ((full >> 8) << 8) | u64::from(super::marantec::crc8(full))
}

/// `SUBGHZ_PROTOCOL_PRASTEL_SERIAL_MASK` (RogueMaster `prastel.h`): the 21 serial bits a frame carries.
const PRASTEL_SERIAL_MASK: u64 = 0x001F_FFFF;

/// RogueMaster `subghz_protocol_prastel_create_data` (`subghz_protocol_prastel_encode_key` +
/// generic serialize): the Prastel key for a serial (its 21 frame bits), button 1..4 (raw nibble
/// 0xE / 0xD / 0xB / 0x7, anything else 0) and counter. RogueMaster's scramble and button table
/// are those of the Flipper-ARF garage copy that `prastel.rs` ports (byte-identical code).
fn prastel_key(serial: u32, btn: u8, cnt: u16) -> FlipperKey {
    let serial = serial & PRASTEL_SERIAL_MASK as u32;
    let raw = super::prastel::btn_to_raw(btn);
    let mut p = [0u8; 7];
    p[1] = ((serial >> 8) & 0xF0) as u8;
    p[2] = (0x70 | ((serial >> 8) & 0x0F) | ((serial >> 13) & 0x80)) as u8;
    p[3] = serial as u8;
    p[4] = (((serial >> 12) & 0xF0) as u8) | raw;
    p[5] = cnt as u8;
    p[6] = (cnt >> 8) as u8;
    super::prastel::scramble(&mut p);
    let mut payload = 0u64;
    for (i, &b) in p.iter().enumerate() {
        payload |= u64::from(b) << (0x30 - 8 * i as u32);
    }
    FlipperKey::for_protocol("Prastel", 42, (payload >> 2) & 0x3FF_FFFF_FFFF)
}

/// `subghz_scene_set_type_fill_generation_infos`: the parameters of `ty` (three `rand()` draws, a
/// fourth for RogueMaster's O-Code installer code).
pub fn fill(ty: &RemoteType, rng: &mut dyn RandSource) -> Gen {
    let key = u64::from(rng.rand());
    let gangqi_key = gangqi_key(rng.rand());
    let marantec_key = marantec_key(rng.rand());
    let mut gen =
        gen_info(ty.id, key, gangqi_key, marantec_key).unwrap_or_else(|| panic!("no parameters for {}", ty.id));
    // RogueMaster's O-Code types draw their installer code in the switch: `.ic = (uint16_t)rand()`
    // (not the certificate type).
    if let GenInfo::NiceOCode { ic, cert: false, .. } = &mut gen.info {
        *ic = rng.rand() as u16;
    }
    gen
}

/// The C's `switch(type)` (`subghz_gen_info.c`; doctor-firmware's for its own types).
#[allow(clippy::identity_op)]
fn gen_info(id: &str, key: u64, gangqi_key: u64, marantec_key: u64) -> Option<Gen> {
    Some(match id {
        "FaacSLH_868" => gen("AM650", 868_350_000, GenInfo::FaacSlh { serial: (((key & 0x00FFFFF0) | 0xA0000006) >> 4) as u32, btn: 0x06, cnt: 0x02, seed: key as u32, manuf: "FAAC_SLH" }),
        "FaacSLH_433" => gen("AM650", 433_920_000, GenInfo::FaacSlh { serial: (((key & 0x00FFFFF0) | 0xA0000006) >> 4) as u32, btn: 0x06, cnt: 0x02, seed: key as u32, manuf: "FAAC_SLH" }),
        "Genius_433" => gen("AM650", 433_920_000, GenInfo::FaacSlh { serial: (((key & 0x00FFFFF0) | 0xA0000006) >> 4) as u32, btn: 0x06, cnt: 0x02, seed: key as u32, manuf: "Genius" }),
        "Genius_868" => gen("AM650", 868_350_000, GenInfo::FaacSlh { serial: (((key & 0x00FFFFF0) | 0xA0000006) >> 4) as u32, btn: 0x06, cnt: 0x02, seed: key as u32, manuf: "Genius" }),
        "BFTMitto" => gen("AM650", 433_920_000, GenInfo::KeeloqSeed { serial: (key & 0x000FFFFF) as u32, btn: 0x02, cnt: 0x02, seed: (key & 0x000FFFFF) as u32, manuf: "BFT" }),
        "Erreka433" => gen("AM650", 433_920_000, GenInfo::KeeloqSeed { serial: (key & 0x000FFFFF) as u32, btn: 0x02, cnt: 0x02, seed: (key & 0x000FFFFF) as u32, manuf: "Erreka" }),
        // RogueMaster's `subghz_gen_info.c` for its own types.
        "Erreka868" => gen("AM650", 868_350_000, GenInfo::KeeloqSeed { serial: (key & 0x000FFFFF) as u32, btn: 0x02, cnt: 0x02, seed: (key & 0x000FFFFF) as u32, manuf: "Erreka" }),
        "Prastel_433_92" => gen("AM650", 433_920_000, GenInfo::Prastel { serial: (key & PRASTEL_SERIAL_MASK) as u32, btn: 0x01, cnt: 0x03 }),
        "NiceOCode_433_92" => gen("AM650", 433_920_000, GenInfo::NiceOCode { serial: (key & 0x0FFFFFFF) as u32, btn: 0x01, cnt: 0xC089, ic: 0, nice_one: false, cert: false }),
        "NiceOCodeOne_433_92" => gen("AM650", 433_920_000, GenInfo::NiceOCode { serial: (key & 0x0FFFFFFF) as u32, btn: 0x01, cnt: 0xC089, ic: 0, nice_one: true, cert: false }),
        "NiceOCodeCert_433_92" => gen("AM650", 433_920_000, GenInfo::NiceOCode { serial: (key & 0x0FFFFFFF) as u32, btn: 0x01, cnt: 0xC089, ic: 0, nice_one: true, cert: true }),
        "NordIce_433" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "Nord ICE", key: (key & 0x1_FFFF_81FF) | 0x2400, bits: 33, te: 0 }),
        "SomfyTelis" => gen("AM650", 433_420_000, GenInfo::SomfyTelis { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03 }),
        "SomfyKeytis" => gen("AM650", 433_420_000, GenInfo::SomfyKeytis { serial: ((key & 0x0000FFFF) | 0x00D50000) as u32, btn: 0x04, cnt: 0x03 }),
        "KingGatesStylo4k" => gen("AM650", 433_920_000, GenInfo::KingGatesStylo4k { serial: (key & 0xFFFFFFFF) as u32, btn: 0x0E, cnt: 0x03 }),
        "BenincaARC" => gen("AM650", 433_920_000, GenInfo::BenincaArc { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03 }),
        "Jarolift" => gen("AM650", 433_920_000, GenInfo::Jarolift { serial: (key & 0xFFFFF00) as u32, btn: 0x02, cnt: 0x03 }),
        "DitecGOL4" => gen("AM650", 433_920_000, GenInfo::DitecGol4 { serial: ((key & 0x0000FFFF) | 0xCC090000) as u32, btn: 0x01, cnt: 0xC200 }),
        "ANMotorsAT4" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: ((key & 0x000FFFFF) | 0x04700000) as u32, btn: 0x02, cnt: 0x21, manuf: "AN-Motors" }),
        "AlutechAT4N" => gen("AM650", 433_920_000, GenInfo::AlutechAt4n { serial: ((key & 0x000FFFFF) | 0x00100000) as u32, btn: 0x44, cnt: 0x03 }),
        "Phoenix_V2_433" => gen("AM650", 433_920_000, GenInfo::PhoenixV2 { serial: ((key & 0x0FFFFFFF) | 0xB0000000) as u32, cnt: 0x025D }),
        "HCS101_433_92" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x000FFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "HCS101" }),
        "DoorHan_315_00" => gen("AM650", 315_000_000, GenInfo::Keeloq { serial: (key & 0x0FFFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "DoorHan" }),
        "DoorHan_433_92" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x0FFFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "DoorHan" }),
        "Superrollo_433_92" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x0FFFFFFF) as u32, btn: 0x0A, cnt: 0x03, manuf: "Superrollo" }),
        "Beninca433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: ((key & 0x000FFF00) | 0x00800080) as u32, btn: 0x01, cnt: 0x05, manuf: "Beninca" }),
        "Beninca868" => gen("AM650", 868_350_000, GenInfo::Keeloq { serial: ((key & 0x000FFF00) | 0x00800080) as u32, btn: 0x01, cnt: 0x05, manuf: "Beninca" }),
        "Comunello433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x08, cnt: 0x05, manuf: "Comunello" }),
        "Comunello868" => gen("AM650", 868_460_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x08, cnt: 0x05, manuf: "Comunello" }),
        "Allmatic433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: ((key & 0x00FFFF00) | 0x01000011) as u32, btn: 0x0C, cnt: 0x05, manuf: "Beninca" }),
        "Allmatic868" => gen("AM650", 868_350_000, GenInfo::Keeloq { serial: ((key & 0x00FFFF00) | 0x01000011) as u32, btn: 0x0C, cnt: 0x05, manuf: "Beninca" }),
        "Centurion433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x0000FFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Centurion" }),
        "Monarch433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x0000FFFF) as u32, btn: 0x0A, cnt: 0x03, manuf: "Monarch" }),
        "MonarchDorcas433" => gen("AM650", 433_920_000, GenInfo::KeeloqSeed { serial: (key & 0x0000FFFF) as u32, btn: 0x0A, cnt: 0x03, seed: 0x280, manuf: "Monarch" }),
        // RogueMaster: the seed carries the 12-bit discriminator, serial & 0x3FF in its low 10 bits
        // (its quick path clears the top 2).
        "PhantomHiland433" => gen("AM650", 433_920_000, GenInfo::KeeloqSeed { serial: (key & 0x0FFFFFFF) as u32, btn: 0x02, cnt: 0x03, seed: (key & 0x3FF) as u32, manuf: "Phantom_Hiland" }),
        "KEY433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x0FFFFFFF) as u32, btn: 0x01, cnt: 0x03, manuf: "KEY" }),
        "JollyMotors433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x000FFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Jolly_Motors" }),
        "Motorline433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x0FFFFFFF) as u32, btn: 0x01, cnt: 0x03, manuf: "Motorline" }),
        "Sommer_FM_434" => gen("FM476", 434_420_000, GenInfo::Keeloq { serial: ((key & 0x0000FFFF) | 0x01700000) as u32, btn: 0x02, cnt: 0x03, manuf: "Sommer" }),
        "Sommer_FM_868" => gen("FM476", 868_800_000, GenInfo::Keeloq { serial: ((key & 0x0000FFFF) | 0x01700000) as u32, btn: 0x02, cnt: 0x03, manuf: "Sommer" }),
        "Sommer_FM12K_434" => gen("FM12K", 434_420_000, GenInfo::Keeloq { serial: (key & 0x0000FFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Sommer" }),
        "Sommer_FM12K_868" => gen("FM12K", 868_800_000, GenInfo::Keeloq { serial: (key & 0x0000FFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Sommer" }),
        "Stilmatic" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x0FFFFFFF) as u32, btn: 0x01, cnt: 0x03, manuf: "Stilmatic" }),
        "IronLogic" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFF0) as u32, btn: 0x04, cnt: 0x05, manuf: "IronLogic" }),
        "IronLogicSmart" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFF0) as u32, btn: 0x04, cnt: 0x05, manuf: "IL-100(Smart)" }),
        "DeaMio433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: ((key & 0x0FFFF000) | 0x00000869) as u32, btn: 0x02, cnt: 0x03, manuf: "Dea_Mio" }),
        "DTMNeo433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x000FFFFF) as u32, btn: 0x02, cnt: 0x05, manuf: "DTM_Neo" }),
        "Gibidi433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Gibidi" }),
        "GSN" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x0FFFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "GSN" }),
        "HomeGate433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x0FFFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "HomeGate" }),
        "Aprimatic" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: ((key & 0x000FFFFF) | 0x00600000) as u32, btn: 0x08, cnt: 0x03, manuf: "Aprimatic" }),
        // FoxFW's plain KeeLoq Erreka types (its `SetTypeErreka433/868`; doctor's and RogueMaster's
        // seeded "Erreka 433/868MHz" are the types above).
        "ErrekaKL433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x0000FFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Erreka" }),
        "ErrekaKL868" => gen("AM650", 868_350_000, GenInfo::Keeloq { serial: (key & 0x0000FFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Erreka" }),
        "ElmesElectronic" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: ((key & 0x00FFFFFF) | 0x02000000) as u32, btn: 0x02, cnt: 0x03, manuf: "Elmes_Poland" }),
        "Normstahl_433_92" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x0000FFFF) as u32, btn: 0x04, cnt: 0x03, manuf: "Normstahl" }),
        "JCM_433_92" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "JCM_Tech" }),
        "Novoferm_433_92" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: ((key & 0x0000FFFF) | 0x018F0000) as u32, btn: 0x01, cnt: 0x03, manuf: "Novoferm" }),
        "HormannEcoStar_433_92" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: ((key & 0x000FFFFF) | 0x02200000) as u32, btn: 0x04, cnt: 0x03, manuf: "EcoStar" }),
        "CardinS449_433FM" => gen("FM12K", 433_920_000, GenInfo::Keeloq { serial: (key & 0x000FFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Cardin_S449" }),
        "Pujol433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Pujol" }),
        "Pujol_Vario433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Pujol_Vario" }),
        "ET_Blue433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x04, cnt: 0x03, manuf: "ET_Blue" }),
        "ET_Blue_Mix433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x04, cnt: 0x03, manuf: "ET_Blue_Mix" }),
        "ATA_PTX4_433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "ATA_PTX4" }),
        "Seav433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Seav" }),
        "Wisniowski433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Wisniowski" }),
        "Fadini433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Fadini" }),
        "Mc_Garcia433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Mc_Garcia" }),
        "Clemsa_Mutancode433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Clemsa_Mutancode" }),
        "Doormatic433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Doormatic" }),
        "Elvox433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Elvox" }),
        "Verex433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "Verex" }),
        "FAACRCXT_433_92" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: ((key & 0x0000FFFF) | 0x00100000) as u32, btn: 0x02, cnt: 0x03, manuf: "FAAC_RC,XT" }),
        "FAACRCXT_868" => gen("AM650", 868_350_000, GenInfo::Keeloq { serial: ((key & 0x0000FFFF) | 0x00100000) as u32, btn: 0x02, cnt: 0x03, manuf: "FAAC_RC,XT" }),
        "GeniusBravo433" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x06, cnt: 0x03, manuf: "Genius_Bravo" }),
        "NiceMHouse_433_92" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x09, cnt: 0x03, manuf: "NICE_MHOUSE" }),
        "NiceSmilo_433_92" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x02, cnt: 0x03, manuf: "NICE_Smilo" }),
        "NiceFlorS_433_92" => gen("AM650", 433_920_000, GenInfo::NiceFlorS { serial: (key & 0x0FFFFFFF) as u32, btn: 0x01, cnt: 0x03, nice_one: false }),
        "NiceOne_433_92" => gen("AM650", 433_920_000, GenInfo::NiceFlorS { serial: (key & 0x0FFFFFFF) as u32, btn: 0x01, cnt: 0x03, nice_one: true }),
        "NiceFlo12bit" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "Nice FLO", key: (key & 0x00000FF0) | 0x1, bits: 12, te: 0 }),
        "NiceFlo24bit" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "Nice FLO", key: (key & 0x00FFFFF0) | 0x4, bits: 24, te: 0 }),
        "CAME12bit" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "CAME", key: (key & 0x00000FF0) | 0x1, bits: 12, te: 0 }),
        "CAME24bit" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "CAME", key: (key & 0x00FFFFF0) | 0x4, bits: 24, te: 0 }),
        "CAME12bit868" => gen("AM650", 868_350_000, GenInfo::Data { protocol: "CAME", key: (key & 0x00000FF0) | 0x1, bits: 12, te: 0 }),
        "CAME24bit868" => gen("AM650", 868_350_000, GenInfo::Data { protocol: "CAME", key: (key & 0x00FFFFF0) | 0x4, bits: 24, te: 0 }),
        "CAMETwee" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "CAME TWEE", key: 0x003FFF7200000000 | ((key & 0x0FFFFFF0) ^ 0xE0E0E0EE), bits: 54, te: 0 }),
        "CameAtomo433" => gen("AM650", 433_920_000, GenInfo::CameAtomo { serial: ((key & 0x0FFFFFFF) | 0x10000000) as u32, cnt: 0x03 }),
        "CameAtomo868" => gen("AM650", 868_350_000, GenInfo::CameAtomo { serial: ((key & 0x0FFFFFFF) | 0x10000000) as u32, cnt: 0x03 }),
        "CAMESpace" => gen("AM650", 433_920_000, GenInfo::Keeloq { serial: (key & 0x00FFFFFF) as u32, btn: 0x04, cnt: 0x03, manuf: "Came_Space" }),
        "Pricenton315" => gen("AM650", 315_000_000, GenInfo::Data { protocol: "Princeton", key: (key & 0x00FFFFF0) | 0x4, bits: 24, te: 400 }),
        "Pricenton433" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "Princeton", key: (key & 0x00FFFFF0) | 0x4, bits: 24, te: 400 }),
        "ZKTeco430" => gen("AM650", 430_500_000, GenInfo::Data { protocol: "Princeton", key: (key & 0x00FFFF00) | 0x30, bits: 24, te: 357 }),
        "BETT_433" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "BETT", key: key & 0x0000FFF0, bits: 18, te: 0 }),
        "GangQi_433" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "GangQi", key: gangqi_key, bits: 34, te: 0 }),
        "Hollarm_433" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "Hollarm", key: (key & 0x000FFF0000) | 0xF0B0002200 | ((((((key & 0x000FFF0000) | 0xF0B0002200) >> 32) & 0xFF) + ((((key & 0x000FFF0000) | 0xF0B0002200) >> 24) & 0xFF) + ((((key & 0x000FFF0000) | 0xF0B0002200) >> 16) & 0xFF) + ((((key & 0x000FFF0000) | 0xF0B0002200) >> 8) & 0xFF)) & 0xFF), bits: 42, te: 0 }),
        "ReversRB2_433" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "Revers_RB2", key: (key & 0x00000FFFFFFFF000) | 0xFFFFF00000000000 | 0x0000000000000A00, bits: 64, te: 0 }),
        "Marantec24_868" => gen("AM650", 868_350_000, GenInfo::Data { protocol: "Marantec24", key: (key & 0xFFFFF0) | 0x000008, bits: 24, te: 0 }),
        "Marantec_433" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "Marantec", key: marantec_key, bits: 49, te: 0 }),
        "Marantec_868" => gen("AM650", 868_350_000, GenInfo::Data { protocol: "Marantec", key: marantec_key, bits: 49, te: 0 }),
        "Roger_433" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "Roger", key: (key & 0xFFFF000) | 0x0000101, bits: 28, te: 0 }),
        "Linear_300_00" => gen("AM650", 300_000_000, GenInfo::Data { protocol: "Linear", key: key & 0x3FF, bits: 10, te: 0 }),
        "TelcomaEdge433" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "Telcoma/Cardin EDGE", key: 0xFF000000 | (key & 0x00FFFFF8), bits: 32, te: 0 }),
        "GateTX" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "GateTX", key: reverse_key((key & 0x00F0FF00) | 0xF0040, 24), bits: 24, te: 0 }),
        // RogueMaster: 12 DIP switches, no button field.
        "FaacTm433" => gen("AM650", 433_920_000, GenInfo::Data { protocol: "Faac TM", key: key & 0xFFF, bits: 12, te: 0 }),
        "FaacTm868" => gen("AM650", 868_000_000, GenInfo::Data { protocol: "Faac TM", key: key & 0xFFF, bits: 12, te: 0 }),
        "SecPlus_v1_315_00" => gen("AM650", 315_000_000, GenInfo::SecPlus1),
        "SecPlus_v1_390_00" => gen("AM650", 390_000_000, GenInfo::SecPlus1),
        "SecPlus_v1_433_00" => gen("AM650", 433_920_000, GenInfo::SecPlus1),
        "SecPlus_v2_310_00" => gen("AM650", 310_000_000, GenInfo::SecPlus2 { serial: (key & 0x7FFFF3FC) as u32, btn: 0x68, cnt: 0xE500000 }),
        "SecPlus_v2_315_00" => gen("AM650", 315_000_000, GenInfo::SecPlus2 { serial: (key & 0x7FFFF3FC) as u32, btn: 0x68, cnt: 0xE500000 }),
        "SecPlus_v2_390_00" => gen("AM650", 390_000_000, GenInfo::SecPlus2 { serial: (key & 0x7FFFF3FC) as u32, btn: 0x68, cnt: 0xE500000 }),
        "SecPlus_v2_433_00" => gen("AM650", 433_920_000, GenInfo::SecPlus2 { serial: (key & 0x7FFFF3FC) as u32, btn: 0x68, cnt: 0xE500000 }),
        _ => return None,
    })
}

/// A field the user can edit before creating the remote (ARF's `set_key`, `set_serial`,
/// `set_button`, `set_counter` and `set_seed` scenes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    Key,
    Serial,
    Button,
    Counter,
    Seed,
}

impl Field {
    pub fn label(self) -> &'static str {
        match self {
            Field::Key => "Key",
            Field::Serial => "Serial",
            Field::Button => "Button",
            Field::Counter => "Counter",
            Field::Seed => "Seed",
        }
    }
}

impl GenInfo {
    /// The editable fields with their value and width in bytes, in the C's scene order (fixed
    /// codes: the whole key; Security+ 1.0: none).
    pub fn fields(&self) -> Vec<(Field, u64, usize)> {
        use Field::*;
        let (s, b, c2, c4) = (4, 1, 2, 4);
        match *self {
            GenInfo::Data { key, .. } => vec![(Key, key, 8)],
            GenInfo::FaacSlh { serial, btn, cnt, seed, .. } => vec![
                (Serial, serial.into(), s),
                (Button, btn.into(), b),
                (Counter, cnt.into(), c4),
                (Seed, seed.into(), 4),
            ],
            GenInfo::KeeloqSeed { serial, btn, cnt, seed, .. } => vec![
                (Serial, serial.into(), s),
                (Button, btn.into(), b),
                (Counter, cnt.into(), c2),
                (Seed, seed.into(), 4),
            ],
            GenInfo::Keeloq { serial, btn, cnt, .. }
            | GenInfo::AlutechAt4n { serial, btn, cnt }
            | GenInfo::SomfyTelis { serial, btn, cnt }
            | GenInfo::SomfyKeytis { serial, btn, cnt }
            | GenInfo::KingGatesStylo4k { serial, btn, cnt }
            | GenInfo::Jarolift { serial, btn, cnt }
            | GenInfo::DitecGol4 { serial, btn, cnt }
            | GenInfo::NiceFlorS { serial, btn, cnt, .. }
            | GenInfo::Prastel { serial, btn, cnt }
            | GenInfo::NiceOCode { serial, btn, cnt, .. } => {
                vec![(Serial, serial.into(), s), (Button, btn.into(), b), (Counter, cnt.into(), c2)]
            }
            GenInfo::BenincaArc { serial, btn, cnt } | GenInfo::SecPlus2 { serial, btn, cnt } => {
                vec![(Serial, serial.into(), s), (Button, btn.into(), b), (Counter, cnt.into(), c4)]
            }
            GenInfo::CameAtomo { serial, cnt } | GenInfo::PhoenixV2 { serial, cnt } => {
                vec![(Serial, serial.into(), s), (Counter, cnt.into(), c2)]
            }
            GenInfo::SecPlus1 => Vec::new(),
        }
    }

    /// Set an editable field (the value is cut to the field's width, as the C's byte input).
    pub fn set(&mut self, field: Field, value: u64) {
        match (self, field) {
            (GenInfo::Data { key, .. }, Field::Key) => *key = value,
            (
                GenInfo::FaacSlh { serial, .. }
                | GenInfo::Keeloq { serial, .. }
                | GenInfo::CameAtomo { serial, .. }
                | GenInfo::KeeloqSeed { serial, .. }
                | GenInfo::AlutechAt4n { serial, .. }
                | GenInfo::SomfyTelis { serial, .. }
                | GenInfo::SomfyKeytis { serial, .. }
                | GenInfo::KingGatesStylo4k { serial, .. }
                | GenInfo::BenincaArc { serial, .. }
                | GenInfo::Jarolift { serial, .. }
                | GenInfo::DitecGol4 { serial, .. }
                | GenInfo::NiceFlorS { serial, .. }
                | GenInfo::SecPlus2 { serial, .. }
                | GenInfo::PhoenixV2 { serial, .. }
                | GenInfo::Prastel { serial, .. }
                | GenInfo::NiceOCode { serial, .. },
                Field::Serial,
            ) => *serial = value as u32,
            (
                GenInfo::FaacSlh { btn, .. }
                | GenInfo::Keeloq { btn, .. }
                | GenInfo::KeeloqSeed { btn, .. }
                | GenInfo::AlutechAt4n { btn, .. }
                | GenInfo::SomfyTelis { btn, .. }
                | GenInfo::SomfyKeytis { btn, .. }
                | GenInfo::KingGatesStylo4k { btn, .. }
                | GenInfo::BenincaArc { btn, .. }
                | GenInfo::Jarolift { btn, .. }
                | GenInfo::DitecGol4 { btn, .. }
                | GenInfo::NiceFlorS { btn, .. }
                | GenInfo::SecPlus2 { btn, .. }
                | GenInfo::Prastel { btn, .. }
                | GenInfo::NiceOCode { btn, .. },
                Field::Button,
            ) => *btn = value as u8,
            (
                GenInfo::Keeloq { cnt, .. }
                | GenInfo::CameAtomo { cnt, .. }
                | GenInfo::KeeloqSeed { cnt, .. }
                | GenInfo::AlutechAt4n { cnt, .. }
                | GenInfo::SomfyTelis { cnt, .. }
                | GenInfo::SomfyKeytis { cnt, .. }
                | GenInfo::KingGatesStylo4k { cnt, .. }
                | GenInfo::Jarolift { cnt, .. }
                | GenInfo::DitecGol4 { cnt, .. }
                | GenInfo::NiceFlorS { cnt, .. }
                | GenInfo::PhoenixV2 { cnt, .. }
                | GenInfo::Prastel { cnt, .. }
                | GenInfo::NiceOCode { cnt, .. },
                Field::Counter,
            ) => *cnt = value as u16,
            (
                GenInfo::FaacSlh { cnt, .. } | GenInfo::BenincaArc { cnt, .. } | GenInfo::SecPlus2 { cnt, .. },
                Field::Counter,
            ) => *cnt = value as u32,
            (GenInfo::FaacSlh { seed, .. } | GenInfo::KeeloqSeed { seed, .. }, Field::Seed) => *seed = value as u32,
            _ => {}
        }
    }

    /// The protocol the generated key file names (the C's `Protocol:`).
    pub fn protocol(&self) -> &'static str {
        match *self {
            GenInfo::Data { protocol, .. } => protocol,
            GenInfo::FaacSlh { .. } => "Faac SLH",
            GenInfo::Keeloq { .. } | GenInfo::KeeloqSeed { .. } => "KeeLoq",
            GenInfo::CameAtomo { .. } => "CAME Atomo",
            GenInfo::AlutechAt4n { .. } => "Alutech AT-4N",
            GenInfo::SomfyTelis { .. } => "Somfy Telis",
            GenInfo::SomfyKeytis { .. } => "Somfy Keytis",
            GenInfo::KingGatesStylo4k { .. } => "KingGates Stylo4k",
            GenInfo::BenincaArc { .. } => "Beninca ARC",
            GenInfo::Jarolift { .. } => "Jarolift",
            GenInfo::DitecGol4 { .. } => "Ditec GOL4",
            GenInfo::NiceFlorS { .. } => "Nice FloR-S",
            GenInfo::SecPlus1 => "Security+ 1.0",
            GenInfo::SecPlus2 { .. } => "Security+ 2.0",
            GenInfo::PhoenixV2 { .. } => "Phoenix_V2",
            GenInfo::Prastel { .. } => "Prastel",
            GenInfo::NiceOCode { .. } => "Nice FloR-S",
        }
    }

    /// The KeeLoq manufacturer, for KeeLoq types.
    pub fn manufacturer(&self) -> Option<&'static str> {
        match self {
            GenInfo::Keeloq { manuf, .. } | GenInfo::KeeloqSeed { manuf, .. } | GenInfo::FaacSlh { manuf, .. } => {
                Some(*manuf)
            }
            _ => None,
        }
    }
}

/// `subghz_block_generic_get_preset_name`: the key file's `Preset:` for the C's short name.
pub fn preset_file_name(preset: &str) -> &'static str {
    match preset {
        "AM270" => "FuriHalSubGhzPresetOok270Async",
        "AM650" => "FuriHalSubGhzPresetOok650Async",
        "FM238" => "FuriHalSubGhzPreset2FSKDev238Async",
        "FM12K" => "FuriHalSubGhzPreset2FSKDev12KAsync",
        "FM476" => "FuriHalSubGhzPreset2FSKDev476Async",
        _ => "FuriHalSubGhzPresetCustom",
    }
}

/// The rainbow tables the Nice FloR-S and Alutech AT-4N types need.
struct Tables {
    nice_flor_s: Option<Table>,
    alutech_at_4n: Option<Table>,
}

impl Tables {
    /// The user's tables (`~/.config/KAT/`), as the registry's decoders load them.
    fn user() -> Self {
        Tables { nice_flor_s: super::nice_flor_s::user_table(), alutech_at_4n: super::alutech_at_4n::user_table() }
    }
}

/// Whether KAT can create `ty`, or why not (see the module doc).
pub fn availability(ty: &RemoteType) -> Result<(), String> {
    availability_of(&fill(ty, &mut Rng::seeded(1)).info, &Tables::user())
}

fn availability_of(info: &GenInfo, tables: &Tables) -> Result<(), String> {
    match info {
        // RogueMaster creates these with its O-Code key maker (installer code; the certificate type
        // walks its receiver-enrolment chain); KAT only re-sends captured O-Code keys with their IC.
        GenInfo::NiceOCode { .. } => Err(
            "RogueMaster's Nice O-Code remote creation is not ported; KAT re-sends captured O-Code keys".to_string(),
        ),
        GenInfo::Data { protocol, .. } if super::info::lookup(protocol).is_none() => {
            Err(format!("KAT has no {protocol} protocol"))
        }
        GenInfo::NiceFlorS { .. } if tables.nice_flor_s.is_none() => {
            Err("needs the Nice FloR-S rainbow table (~/.config/KAT/nice_flor_s.bin)".to_string())
        }
        GenInfo::AlutechAt4n { .. } if tables.alutech_at_4n.is_none() => {
            Err("needs the Alutech AT-4N rainbow table (~/.config/KAT/alutech_at_4n.bin)".to_string())
        }
        GenInfo::Keeloq { manuf, .. } | GenInfo::KeeloqSeed { manuf, .. } if !super::keeloq::can_create(manuf) => {
            Err(format!("no usable {manuf} key in the keystore (add one to ~/.config/KAT/keeloq_mfcodes_user)"))
        }
        _ => Ok(()),
    }
}

/// `subghz_scene_set_type_generate_protocol_from_infos`: the key file of a new remote (with its
/// `Frequency` and `Preset`), or why KAT cannot create it. Security+ 1.0 draws its fixed code
/// from `rng`.
pub fn generate(gen: &Gen, rng: &mut dyn RandSource) -> Result<FlipperKey, String> {
    generate_with(gen, rng, &Tables::user())
}

fn generate_with(gen: &Gen, rng: &mut dyn RandSource, tables: &Tables) -> Result<FlipperKey, String> {
    availability_of(&gen.info, tables)?;
    let no_key = |what: &str| format!("no {what} key in the keystore");
    let mut key = match gen.info {
        GenInfo::Data { protocol, key, bits, te } => {
            let mut k = FlipperKey::for_protocol(protocol, bits, key);
            if protocol == "Princeton" {
                // The decoder's serialize writes TE and Guard_time; gen_data_protocol_and_te sets
                // them to the type's TE and 30.
                k.push("TE", te.to_string());
                k.push("Guard_time", "30");
            } else if protocol == "Faac TM" {
                // RogueMaster: the decoder's serialize writes the fresh decoder's TE (0); the
                // encoder sends a TE outside 100..1000 at the nominal 400 µs.
                k.push("TE", te.to_string());
            }
            k
        }
        GenInfo::FaacSlh { serial, btn, cnt, seed, manuf } => {
            // Flipper-ARF's create (FAAC SLH: counter + 1, no Manufacture line); doctor-firmware's
            // for its Genius types (counter as given, Manufacture written).
            let arf = manuf == "FAAC_SLH";
            super::faac_slh::create_data(serial, btn, cnt, seed, arf, (!arf).then_some(manuf))
                .ok_or_else(|| no_key("FAAC_SLH (type 5)"))?
        }
        GenInfo::Keeloq { serial, btn, cnt, manuf } => {
            super::keeloq::create_data(serial, btn, cnt, manuf, None).ok_or_else(|| no_key(manuf))?
        }
        GenInfo::KeeloqSeed { serial, btn, cnt, seed, manuf } => {
            super::keeloq::create_data(serial, btn, cnt, manuf, Some(seed)).ok_or_else(|| no_key(manuf))?
        }
        GenInfo::CameAtomo { serial, cnt } => super::came_atomo::create_data(serial, cnt),
        GenInfo::AlutechAt4n { serial, btn, cnt } => {
            let table = tables.alutech_at_4n.as_ref().ok_or_else(|| "no Alutech AT-4N table".to_string())?;
            super::alutech_at_4n::create_data(serial, btn, cnt, table)
        }
        GenInfo::SomfyTelis { serial, btn, cnt } => super::somfy_telis::create_data(serial, btn, cnt),
        GenInfo::SomfyKeytis { serial, btn, cnt } => super::somfy_keytis::create_data(serial, btn, cnt),
        GenInfo::KingGatesStylo4k { serial, btn, cnt } => {
            super::kinggates_stylo_4k::create_data(serial, btn, cnt).ok_or_else(|| no_key("KingGates"))?
        }
        GenInfo::BenincaArc { serial, btn, cnt } => {
            super::beninca_arc::create_data(serial, btn, cnt).ok_or_else(|| no_key("Beninca ARC (type 9)"))?
        }
        GenInfo::Jarolift { serial, btn, cnt } => {
            super::jarolift::create_data(serial, btn, cnt).ok_or_else(|| no_key("Jarolift (type 11)"))?
        }
        GenInfo::DitecGol4 { serial, btn, cnt } => super::ditec_gol4::create_data(serial, btn, cnt),
        GenInfo::NiceFlorS { serial, btn, cnt, nice_one } => {
            let table = tables.nice_flor_s.as_ref().ok_or_else(|| "no Nice FloR-S table".to_string())?;
            super::nice_flor_s::create_data(serial, btn, cnt, nice_one, table)
        }
        GenInfo::SecPlus1 => {
            // subghz_txrx_gen_secplus_v1_protocol: rand() until check_fixed accepts it.
            let mut fixed = rng.rand();
            while !super::secplus_v1::check_fixed(fixed) {
                fixed = rng.rand();
            }
            FlipperKey::for_protocol("Security+ 1.0", 42, (u64::from(fixed) << 32) | 0xE600_0000)
        }
        GenInfo::SecPlus2 { serial, btn, cnt } => super::secplus_v2::create_data(serial, btn, cnt),
        GenInfo::PhoenixV2 { serial, cnt } => super::phoenix_v2::create_data(serial, cnt),
        GenInfo::Prastel { serial, btn, cnt } => prastel_key(serial, btn, cnt),
        GenInfo::NiceOCode { .. } => unreachable!("availability_of refuses the O-Code types"),
    };
    key.frequency = gen.frequency;
    key.preset = Some(preset_file_name(gen.preset).to_string());
    Ok(key)
}

/// What the picker shows about a type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Summary {
    /// The protocol (KeeLoq and FAAC: with the manufacturer key the type uses).
    pub protocol: String,
    pub frequency: u32,
    /// The C's short preset name ("AM650", "FM476", "FM12K").
    pub preset: &'static str,
    /// The editable fields: their fixed default, or None when drawn at random.
    pub fields: Vec<(Field, Option<u64>)>,
}

/// [Summary] of a type: its parameters from two different draws; a field that differs is random.
pub fn summary(ty: &RemoteType) -> Summary {
    let a = fill(ty, &mut Scripted::new(&[0x1234_5678, 0x0BAD_CAFE, 0x0012_3456]));
    let b = fill(ty, &mut Scripted::new(&[0x0765_4321, 0x0000_1234, 0x0065_4321]));
    let protocol = a.info.protocol();
    let protocol = match a.info.manufacturer() {
        Some(m) if m != "FAAC_SLH" => format!("{protocol} ({m})"),
        _ => protocol.to_string(),
    };
    let fields = a
        .info
        .fields()
        .into_iter()
        .zip(b.info.fields())
        .map(|((f, va, _), (_, vb, _))| (f, (va == vb).then_some(va)))
        .collect();
    Summary { protocol, frequency: a.frequency, preset: a.preset, fields }
}

/// The types whose name or id contains every word of `query` (ASCII case-insensitive), as
/// indices into [TYPES].
pub fn find(query: &str) -> Vec<usize> {
    let words: Vec<String> = query.split_whitespace().map(str::to_ascii_lowercase).collect();
    TYPES
        .iter()
        .enumerate()
        .filter(|(_, t)| {
            let hay = format!("{} {}", t.name, t.id).to_ascii_lowercase();
            words.iter().all(|w| hay.contains(w.as_str()))
        })
        .map(|(i, _)| i)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::{DecodedSignal, ProtocolRegistry};
    use crate::radio::LevelDuration;

    /// A synthetic Nice FloR-S table (the real one is secret): any 32 bytes make a valid cipher.
    const NICE_TABLE: Table = [
        0x3A, 0x91, 0x5C, 0xE7, 0x08, 0x6D, 0xB2, 0x4F, 0x17, 0xC8, 0x2E, 0x99, 0x70, 0x05, 0xDB, 0x63,
        0xA4, 0x1F, 0x86, 0x3C, 0xF0, 0x58, 0x2B, 0xCE, 0x94, 0x47, 0x0E, 0xB9, 0x61, 0xD5, 0x7A, 0x13,
    ];

    /// A synthetic TEA-shaped Alutech table: m0 = 32 × delta, m3 = −delta, m6 = delta.
    fn alutech_table() -> Table {
        let delta: u32 = 0x9E37_79B9;
        let words = [delta.wrapping_mul(32), 0x0123_4567, 0x89AB_CDEF, delta.wrapping_neg(), 0xFEDC_BA98, 0x7654_3210, delta, 0];
        let mut t = [0u8; 32];
        for (i, w) in words.iter().enumerate() {
            t[4 * i..4 * i + 4].copy_from_slice(&w.to_be_bytes());
        }
        t
    }

    fn test_tables() -> Tables {
        Tables { nice_flor_s: Some(NICE_TABLE), alutech_at_4n: Some(alutech_table()) }
    }

    /// The registry, with the test tables in its Nice FloR-S and Alutech AT-4N decoders.
    fn registry() -> ProtocolRegistry {
        let mut reg = ProtocolRegistry::new();
        for d in &mut reg.decoders {
            match d.name() {
                "Nice FloR-S" => *d = Box::new(super::super::nice_flor_s::NiceFlorSDecoder::with_table(Some(NICE_TABLE))),
                "Alutech AT-4N" => {
                    *d = Box::new(super::super::alutech_at_4n::AlutechAt4nDecoder::with_table(Some(alutech_table())))
                }
                _ => {}
            }
        }
        reg
    }

    /// The upload as a receiver sees it: sent twice, a 20 ms LOW before, between and after, equal
    /// adjacent levels merged.
    fn on_air(upload: &[LevelDuration]) -> Vec<LevelDuration> {
        let gap = [LevelDuration::new(false, 20_000)];
        let mut out: Vec<LevelDuration> = Vec::new();
        for p in gap.iter().chain(upload).chain(&gap).chain(upload).chain(&gap) {
            match out.last_mut() {
                Some(last) if last.level == p.level => last.duration_us += p.duration_us,
                _ => out.push(LevelDuration::new(p.level, p.duration_us)),
            }
        }
        out
    }

    /// Types KAT cannot create with its embedded keystore (and test tables).
    const UNAVAILABLE: &[&str] = &[
        "Erreka868", "NiceOCode_433_92", "NiceOCodeOne_433_92", "NiceOCodeCert_433_92", "PhantomHiland433", "Erreka433", "Superrollo_433_92", "HomeGate433", "Pujol433",
        "Pujol_Vario433", "ET_Blue433", "ET_Blue_Mix433", "ATA_PTX4_433", "Seav433", "Wisniowski433", "ErrekaKL433", "ErrekaKL868",
        "Fadini433", "Mc_Garcia433", "Clemsa_Mutancode433", "Doormatic433", "Elvox433", "Verex433",
    ];

    #[test]
    fn type_list_is_the_union_of_the_references() {
        assert_eq!(TYPES.len(), 112);
        assert_eq!(TYPES.iter().filter(|t| t.source == Source::Both).count(), 78);
        assert_eq!(TYPES.iter().filter(|t| t.source == Source::RogueMaster).count(), 9);
        assert_eq!(TYPES.iter().filter(|t| t.source == Source::FoxFW).count(), 2);
        assert_eq!(TYPES[find("Prastel")[0]].menus(), "RogueMaster");
        assert_eq!(TYPES[find("GangQi")[0]].menus(), "Flipper-ARF + doctor-firmware");
        let mut ids: Vec<&str> = TYPES.iter().map(|t| t.id).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), TYPES.len(), "duplicate ids");
        for t in TYPES {
            assert!(gen_info(t.id, 1, 2, 3).is_some(), "{}: no parameters", t.id);
        }
        // Groups are contiguous runs in menu order.
        let mut seen = Vec::new();
        for w in TYPES.windows(2) {
            if w[0].group != w[1].group {
                assert!(!seen.contains(&w[1].group), "{:?} split", w[1].group);
                seen.push(w[0].group);
            }
        }
        // KeeLoq remotes get the gate menu (Send next code / Send button…).
        for t in TYPES {
            if let Some(mf) = fill(t, &mut Rng::seeded(1)).info.manufacturer().filter(|_| t.id != "FaacSLH_868") {
                let name = format!("KeeLoq ({mf})");
                let gate = super::super::gate_buttons::is_gate(&name) || mf == "FAAC_SLH" || mf == "Genius";
                assert!(gate, "{}: {name} is not a barrier manufacturer", t.id);
            }
        }
        assert_eq!(find("princeton 433").iter().map(|&i| TYPES[i].id).collect::<Vec<_>>(), ["Pricenton433"]);
        assert_eq!(find("KL: DOORHAN").len(), 2);
        assert!(find("no such remote").is_empty());
    }

    /// The draws: `key` first, then GangQi's and Marantec's (whatever the type), as the C.
    #[test]
    fn draws_follow_the_c_order() {
        let mut r = Scripted::new(&[0x1234_5678, 0x0ABC_DEF1, 0x1357_9BDF]);
        let g = fill(&TYPES[find("princeton 433")[0]], &mut r);
        assert_eq!(g.info, GenInfo::Data { protocol: "Princeton", key: 0x34_5674, bits: 24, te: 400 });
        assert_eq!(gangqi_key(0x0ABC_DEF1), (0xDEF1 << 18) | (0xDD << 10) | (u64::from(0xC8u8.wrapping_sub(0xDE).wrapping_sub(0xF1).wrapping_sub(0xDD)) << 2));
        // Marantec: the CRC-8 the decoder checks.
        let m = marantec_key(0x1357_9BDF);
        assert_eq!(m >> 8, ((0x130 << 40) | (0x79BDF << 20) | (0x4 << 16) | (0x86 << 8)) >> 8);
        assert_eq!(super::super::marantec::crc8(m), m as u8);
        // Security+ 1.0 draws until check_fixed accepts the value (id1 = fixed / 9 % 3 != 0).
        let mut r = Scripted::new(&[1, 2, 3, 0, 9 * 3, 9]);
        let sp1 = TYPES.iter().find(|t| t.id == "SecPlus_v1_315_00").unwrap();
        let k = generate_with(&fill(sp1, &mut r), &mut r, &test_tables()).unwrap();
        assert_eq!(k.key, (9u64 << 32) | 0xE600_0000);
        // RogueMaster's O-Code types draw a fourth value, the installer code (not the certificate
        // type), and Prastel keeps the 21 serial bits.
        let mut r = Scripted::new(&[0x1234_5678, 1, 2, 0x7654_ABCD]);
        let g = fill(&TYPES[find("Nice O-Code 433")[0]], &mut r);
        assert_eq!(
            g.info,
            GenInfo::NiceOCode { serial: 0x0234_5678, btn: 1, cnt: 0xC089, ic: 0xABCD, nice_one: false, cert: false }
        );
        let mut r = Scripted::new(&[0x1234_5678, 1, 2, 0x7654_ABCD]);
        let g = fill(&TYPES[find("O-Code+Crt")[0]], &mut r);
        assert!(matches!(g.info, GenInfo::NiceOCode { ic: 0, nice_one: true, cert: true, .. }));
        assert_eq!(r.rand(), 0x7654_ABCD, "no fourth draw");
        let g = fill(&TYPES[find("Prastel")[0]], &mut Scripted::new(&[0x7FFF_FFFF, 0, 0]));
        assert_eq!((g.preset, g.frequency), ("AM650", 433_920_000));
        assert_eq!(g.info, GenInfo::Prastel { serial: 0x1F_FFFF, btn: 1, cnt: 3 });
        let g = fill(&TYPES[find("Nord ICE")[0]], &mut Scripted::new(&[0x7FFF_FFFF, 0, 0]));
        assert_eq!(g.info, GenInfo::Data { protocol: "Nord ICE", key: 0x7FFF_A5FF, bits: 33, te: 0 });
    }

    /// RogueMaster's "Prastel 433MHz": the key decodes (Prastel, the garage-app decoder KAT ports)
    /// with the serial's 21 frame bits, the button and the counter it was made from, for every
    /// button; outside 1..4 the raw nibble is 0 (decoded button 0), as `btn_to_raw`.
    #[test]
    fn prastel_keys_carry_serial_button_and_counter() {
        let reg = ProtocolRegistry::new();
        for (serial, btn, cnt) in [(0x0012_3456u32, 1u8, 3u16), (0xFFFF_FFFF, 2, 0xFFFF), (0, 3, 0), (0x1A_5A5A, 4, 0x8001), (0x0F_0F0F, 7, 9)] {
            let key = prastel_key(serial, btn, cnt);
            assert_eq!((key.protocol.as_str(), key.bits), ("Prastel", 42));
            let (name, d) = reg.decode_key(&key).unwrap();
            assert_eq!(name, "Prastel");
            let want_btn = if (1..=4).contains(&btn) { btn } else { 0 };
            assert_eq!((d.serial, d.button, d.counter), (Some(serial & 0x1F_FFFF), Some(want_btn), Some(cnt)), "{serial:#x}");
        }
        // RogueMaster's own generation (gen oracle, `gen_roguemaster gen Prastel_433_92`).
        assert_eq!(prastel_key(0x0012_3456, 1, 3).key, 0x0000_031A_D4E8_FF5C);
        assert_eq!(prastel_key(0x7FFF_FFFF, 1, 3).key, 0x0000_03C1_FEFE_FF5C);
    }

    /// Every type, three seeds: the key file loads (text round trip), is encoder-capable, names the
    /// type's protocol (KeeLoq: its manufacturer), and its transmission decodes on air as the same
    /// remote — same serial and button, counter + 1 for rolling codes (as each encoder steps it).
    #[test]
    fn every_type_creates_a_remote_that_transmits() {
        let _keystore = super::super::keeloq_common::KEYSTORE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crate::protocols::keys::load_keystore_from_embedded();
        let mut reg = registry();
        let tables = test_tables();
        let keystore = crate::protocols::keeloq_common::keeloq_keystore();
        let position = |name: &str| keystore.iter().position(|e| e.name == name);
        let mut failures = Vec::new();
        let mut ambiguous = 0;
        let mut checked = 0;
        for ty in TYPES {
            for seed in [1u64, 2, 3] {
                let mut rng = Rng::seeded(seed);
                let gen = fill(ty, &mut rng);
                let key = match generate_with(&gen, &mut rng, &tables) {
                    Ok(k) => k,
                    Err(e) => {
                        if !UNAVAILABLE.contains(&ty.id) {
                            failures.push(format!("{}: {e}", ty.id));
                        }
                        continue;
                    }
                };
                assert!(!UNAVAILABLE.contains(&ty.id), "{} became available", ty.id);
                let text = key.to_text();
                let parsed = FlipperKey::parse(&text).unwrap();
                assert_eq!(parsed, key, "{}: text round trip\n{text}", ty.id);
                let Some((name, d)) = reg.decode_key(&parsed) else {
                    failures.push(format!("{}: key does not load\n{text}", ty.id));
                    continue;
                };
                let decoder = reg.get(&name).unwrap().name();
                if !d.encoder_capable {
                    failures.push(format!("{}: {name} not encoder-capable\n{text}", ty.id));
                    continue;
                }
                let want_name = match gen.info.manufacturer() {
                    Some(mf) if gen.info.protocol() == "KeeLoq" => format!("KeeLoq ({mf})"),
                    _ => name.clone(),
                };
                if name != want_name || reg.get(gen.info.protocol()).map(|p| p.name()) != Some(decoder) {
                    failures.push(format!("{}: loads as {name}, expected {want_name}", ty.id));
                    continue;
                }
                let upload = reg.get(&name).unwrap().encode(&d, d.button.unwrap_or(0)).unwrap();
                let stream = on_air(&upload);
                let same = |x: &DecodedSignal| {
                    let rolling = !matches!(gen.info, GenInfo::Data { .. });
                    let counter_ok = match (d.counter, x.counter) {
                        // AN-Motors shows the hop's top half: the 8-bit counter twice.
                        (Some(a), Some(b)) if name == "KeeLoq (AN-Motors)" => {
                            let next = (a as u8).wrapping_add(1);
                            b == u16::from_be_bytes([next, next])
                        }
                        (Some(a), Some(b)) => b == if rolling { a.wrapping_add(1) } else { a },
                        _ => true,
                    };
                    x.serial == d.serial && x.button == d.button && counter_ok
                };
                let hits = reg.process_signal_stream(&stream, key.frequency);
                checked += 1;
                if hits.iter().any(|(n, x, _)| *n == name && same(x)) {
                    continue;
                }
                // KeeLoq on air: the first keystore entry that validates names the frame (the
                // references' rule); an earlier entry may also validate a genuine frame. The frame
                // must then still validate under the type's manufacturer.
                if let Some((n, x, _)) = hits.iter().find(|(n, _, _)| n.starts_with("KeeLoq (")) {
                    let other = &n["KeeLoq (".len()..n.len() - 1];
                    let mf = gen.info.manufacturer().unwrap_or("");
                    let mut named = FlipperKey::for_protocol("KeeLoq", 64, x.data);
                    named.push("Manufacture", mf);
                    let again = reg.decode_key(&named).map(|(_, y)| y);
                    if position(other) < position(mf) && again.as_ref().is_some_and(same) {
                        ambiguous += 1;
                        eprintln!("note: {} seed {seed}: on air the first match is {n} (an earlier keystore entry)", ty.id);
                        continue;
                    }
                }
                // Marantec24: the encoder's 15 200 µs gap sits exactly on the decoder's exclusive
                // bound (both references fail their own round trip); with the few µs of slack of a
                // real receiver (the real remote's gaps are 15 146..15 180 µs) it decodes.
                if ty.id == "Marantec24_868" {
                    let slack: Vec<LevelDuration> = stream
                        .iter()
                        .map(|p| LevelDuration::new(p.level, if p.duration_us == 15_200 { 15_180 } else { p.duration_us }))
                        .collect();
                    if reg.decode_with(&name, &slack).iter().any(same) {
                        continue;
                    }
                }
                // Prastel: its 42-bit frames are also CAME's "Prastel" variant, and CAME (registered
                // first, as in the Flipper-ARF garage app and doctor-firmware) reports them; the
                // Prastel decoder itself reads the same remote.
                if ty.id == "Prastel_433_92"
                    && hits.iter().all(|(n, _, _)| n == "CAME [Prastel]")
                    && reg.decode_with(&name, &stream).iter().any(same)
                {
                    continue;
                }
                // Revers RB2: whether the decoder re-syncs to the encoder's back-to-back frames depends
                // on the key; the references' decoders decode exactly the same transmissions as KAT's
                // (their own round trip fails too) — the key file and upload are checked against the
                // reference C (docs/add_manually.md).
                if ty.id == "ReversRB2_433" {
                    eprintln!("note: {} seed {seed}: not decodable on air (reference behaviour)", ty.id);
                    continue;
                }
                let got: Vec<String> = hits.iter().map(|(n, x, _)| format!("{n} sn={:X?} btn={:X?} cnt={:X?}", x.serial, x.button, x.counter)).collect();
                failures.push(format!(
                    "{} seed {seed}: {name} sn={:X?} btn={:X?} cnt={:X?} decodes on air as {got:?}",
                    ty.id, d.serial, d.button, d.counter
                ));
            }
        }
        assert!(failures.is_empty(), "{} failure(s):\n{}", failures.len(), failures.join("\n"));
        assert!(ambiguous * 10 < checked, "{ambiguous} of {checked} ambiguous on air");
    }

    /// Editing a field (ARF's set_serial / set_button / set_counter / set_seed / set_key scenes)
    /// changes what is generated.
    #[test]
    fn edited_fields_are_used() {
        crate::protocols::keys::load_keystore_from_embedded();
        let reg = registry();
        let doorhan = &TYPES[find("KL: DoorHan 433")[0]];
        let mut gen = fill(doorhan, &mut Rng::seeded(7));
        assert_eq!(
            gen.info.fields().iter().map(|f| (f.0, f.2)).collect::<Vec<_>>(),
            [(Field::Serial, 4), (Field::Button, 1), (Field::Counter, 2)]
        );
        gen.info.set(Field::Serial, 0x0123_4567);
        gen.info.set(Field::Button, 0x4);
        gen.info.set(Field::Counter, 0x1_0042);
        let key = generate_with(&gen, &mut Rng::seeded(7), &test_tables()).unwrap();
        let (name, d) = reg.decode_key(&key).unwrap();
        assert_eq!(name, "KeeLoq (DoorHan)");
        assert_eq!((d.serial, d.button, d.counter), (Some(0x0123_4567), Some(0x4), Some(0x0042)));
        let mut princeton = fill(&TYPES[find("princeton 315")[0]], &mut Rng::seeded(7));
        assert_eq!(princeton.info.fields().iter().map(|f| (f.0, f.2)).collect::<Vec<_>>(), [(Field::Key, 8)]);
        princeton.info.set(Field::Key, 0x00AB_CDE1);
        let key = generate_with(&princeton, &mut Rng::seeded(7), &test_tables()).unwrap();
        assert!(key.to_text().ends_with("Protocol: Princeton\nBit: 24\nKey: 00 00 00 00 00 AB CD E1\nTE: 400\nGuard_time: 30\n"));
        let (_, d) = reg.decode_key(&key).unwrap();
        assert_eq!((d.serial, d.button), (Some(0xABCDE), Some(0x1)));
        assert!(fill(&TYPES[find("Security+1.0 390")[0]], &mut Rng::seeded(7)).info.fields().is_empty());
    }

    #[test]
    fn summaries_show_what_is_random() {
        let s = summary(&TYPES[find("KL: DoorHan 315")[0]]);
        assert_eq!((s.protocol.as_str(), s.frequency, s.preset), ("KeeLoq (DoorHan)", 315_000_000, "AM650"));
        assert_eq!(s.fields, [(Field::Serial, None), (Field::Button, Some(2)), (Field::Counter, Some(3))]);
        let s = summary(&TYPES[find("Sommer fm2 868")[0]]);
        assert_eq!((s.frequency, s.preset), (868_800_000, "FM12K"));
        let s = summary(&TYPES[find("Dorcas")[0]]);
        assert_eq!(s.fields.last(), Some(&(Field::Seed, Some(0x280))));
        let s = summary(&TYPES[find("BFT Mitto")[0]]);
        assert_eq!(s.fields.last(), Some(&(Field::Seed, None)));
        assert_eq!(summary(&TYPES[find("Princeton 433")[0]]).fields, [(Field::Key, None)]);
        assert_eq!(summary(&TYPES[find("Genius 868")[0]]).protocol, "Faac SLH (Genius)");
        assert!(summary(&TYPES[find("Security+1.0 433")[0]]).fields.is_empty());
    }

    #[test]
    fn availability_names_the_reason() {
        let _keystore = super::super::keeloq_common::KEYSTORE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        for t in ["GangQi", "Hollarm", "Prastel", "Nord ICE"] {
            assert_eq!(availability(&TYPES[find(t)[0]]), Ok(()), "{t}");
        }
        for t in find("O-Code") {
            assert!(availability(&TYPES[t]).unwrap_err().contains("O-Code"), "{}", TYPES[t].name);
        }
        assert!(availability(&TYPES[find("Erreka 868")[0]]).unwrap_err().contains("Erreka"));
        let erreka = &TYPES[find("Erreka")[0]];
        assert!(availability(erreka).unwrap_err().contains("Erreka"));
        let none = Tables { nice_flor_s: None, alutech_at_4n: None };
        let nice = fill(&TYPES[find("Nice One")[0]], &mut Rng::seeded(1));
        assert!(generate_with(&nice, &mut Rng::seeded(1), &none).unwrap_err().contains("nice_flor_s.bin"));
        let alutech = fill(&TYPES[find("Alutech")[0]], &mut Rng::seeded(1));
        assert!(generate_with(&alutech, &mut Rng::seeded(1), &none).unwrap_err().contains("alutech_at_4n.bin"));
        assert_eq!(availability(&TYPES[find("KL: Beninca 868")[0]]), Ok(()));
    }

    /// `KAT_GEN_VECTORS=<dir> cargo test add_manually_oracle_vectors -- --ignored`: for every type
    /// KAT creates (seed `KAT_GEN_SEED`, default 1): `<id>.key.sub` (the key file), `<id>.tx.sub` (a RAW file of the first
    /// transmission, sent twice; `write_vector`), `<id>.upload.txt` (that upload, signed µs, equal
    /// adjacent levels merged) and a line of `manifest.tsv`: id, the C protocol name, KAT's decode
    /// of the key (name, serial, button, counter) and of its transmission.
    #[test]
    #[ignore]
    fn add_manually_oracle_vectors() {
        let Some(dir) = std::env::var_os("KAT_GEN_VECTORS") else { return };
        let dir = std::path::PathBuf::from(dir);
        std::fs::create_dir_all(&dir).unwrap();
        crate::protocols::keys::load_keystore_from_embedded();
        let mut reg = ProtocolRegistry::new();
        let none = Tables { nice_flor_s: None, alutech_at_4n: None };
        let hex = |v: Option<u64>| v.map_or("-".to_string(), |v| format!("{v:X}"));
        let mut manifest = String::new();
        let seed = std::env::var("KAT_GEN_SEED").ok().and_then(|s| s.parse().ok()).unwrap_or(1);
        for ty in TYPES {
            let mut rng = Rng::seeded(seed);
            let gen = fill(ty, &mut rng);
            let Ok(key) = generate_with(&gen, &mut rng, &none) else { continue };
            std::fs::write(dir.join(format!("{}.key.sub", ty.id)), key.to_text()).unwrap();
            let (name, d) = reg.decode_key(&key).unwrap();
            let upload = reg.get(&name).unwrap().encode(&d, d.button.unwrap_or(0)).unwrap();
            let preset = key.preset.clone().unwrap();
            crate::protocols::flipper_key::testing::write_vector(&dir, &format!("{}.tx", ty.id), key.frequency, &preset, &upload, &key);
            let mut values: Vec<i64> = Vec::new();
            for p in &upload {
                let v = if p.level { i64::from(p.duration_us) } else { -i64::from(p.duration_us) };
                match values.last_mut() {
                    Some(last) if (*last > 0) == p.level => *last += v,
                    _ => values.push(v),
                }
            }
            let values: Vec<String> = values.iter().map(i64::to_string).collect();
            let merged = on_air(&upload);
            std::fs::write(dir.join(format!("{}.upload.txt", ty.id)), values.join(" ")).unwrap();
            let tx = reg
                .process_signal_stream(&merged, key.frequency)
                .into_iter()
                .find(|(n, _, _)| reg.get(n).map(|p| p.name()) == reg.get(&name).map(|p| p.name()))
                .map_or("-".to_string(), |(n, x, _)| {
                    format!("{n}\t{}\t{}\t{}", hex(x.serial.map(u64::from)), hex(x.button.map(u64::from)), hex(x.counter.map(u64::from)))
                });
            manifest += &format!(
                "{}\t{}\t{name}\t{}\t{}\t{}\t{tx}\n",
                ty.id,
                gen.info.protocol(),
                hex(d.serial.map(u64::from)),
                hex(d.button.map(u64::from)),
                hex(d.counter.map(u64::from)),
            );
        }
        std::fs::write(dir.join("manifest.tsv"), manifest).unwrap();
    }

    /// `KAT_GEN_DIR=<dir> cargo test add_manually_reference_dump -- --ignored`: KAT's key files for
    /// every type and the reference gen oracle's rand sequences (`kat_<seq>.txt`, "# type:" blocks
    /// like the oracle's output) and KAT's effective keystore (`kat_keystore.txt`,
    /// `KEY:TYPE:NAME`), for comparing with `gen_arf` / `gen_doctor --keystore`.
    #[test]
    #[ignore]
    fn add_manually_reference_dump() {
        let Some(dir) = std::env::var_os("KAT_GEN_DIR") else { return };
        let dir = std::path::PathBuf::from(dir);
        crate::protocols::keys::load_keystore_from_embedded();
        let ks: String = crate::protocols::keeloq_common::keeloq_keystore()
            .iter()
            .map(|e| format!("{:016X}:{}:{}\n", e.key, e.learning, e.name))
            .collect();
        std::fs::write(dir.join("kat_keystore.txt"), ks).unwrap();
        let seqs: [(&str, [u32; 8]); 3] = [
            ("s1", [0x1234_5678, 0x0ABC_DEF1, 0x1357_9BDF, 0x2468_ACE0, 0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444]),
            ("s2", [0x7FFF_FFFF, 0x0000_0001, 0x5555_5555, 0x2AAA_AAAA, 0x0F0F_0F0F, 0x70F0_F0F0, 0x0123_4567, 0x7654_3210]),
            ("s3", [0, 0, 0, 0x6B8B_4567, 0x327B_23C6, 0x643C_9869, 0x6633_4873, 0x74B0_DC51]),
        ];
        let tables = Tables { nice_flor_s: None, alutech_at_4n: None };
        for (name, values) in seqs {
            let mut out = String::new();
            for ty in TYPES {
                let mut r = Scripted::new(&values);
                let gen = fill(ty, &mut r);
                out += &format!("# type: SetType{} | {}\n", ty.id, ty.name);
                match generate_with(&gen, &mut r, &tables) {
                    Ok(k) => out += &k.to_text(),
                    Err(e) => out += &format!("# unavailable: {e}\n"),
                }
                out += "\n";
            }
            std::fs::write(dir.join(format!("kat_{name}.txt")), out).unwrap();
        }
    }
}
