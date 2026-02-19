//! KeeLoq barrier/gate/garage and alarm manufacturer names.
//! When a decode matches one of these, the signal action menu shows only Replay (and export/delete),
//! not TX Lock/Unlock/Trunk/Panic (those are for car keyfobs).

use std::collections::HashSet;
use std::sync::OnceLock;

fn normalize(s: &str) -> String {
    s.chars()
        .filter(|c| c.is_ascii_alphanumeric())
        .flat_map(|c| c.to_lowercase())
        .collect()
}

/// Barrier/gate/garage door manufacturers (normalized names).
/// Covers Flipper/ProtoPirate-style names and common variants (e.g. NICE_MHOUSE, Guard_RF-311A).
const BARRIER_NAMES: &[&str] = &[
    "airforce",
    "allmatic",
    "alutechat4n",
    "ansonic",
    "aprimatic",
    "beninca",
    "benincavaaes128",
    "bett",
    "bft",
    "cameatomo",
    "cameatomotop44rbn",
    "camespace",
    "camestatic",
    "cameweetwin",
    "chamberlaincode",
    "clemsa",
    "comunello",
    "deamio",
    "dickertmahs",
    "doitrand",
    "doorhan",
    "dooya",
    "dtmneo",
    "ecostar",
    "elmespoland",
    "elplast",
    "faacrctx",
    "faacslh",
    "faacslhspa",
    "feron",
    "gatetx",
    "gbidi",
    "geniusbravo",
    "geniusbravoecho",
    "gibidi",
    "gsn",
    "guardrf311a",
    "gangqi",
    "hay21",
    "hollarm",
    "holtek",
    "holtekht12x",
    "honeywell",
    "honeywellwdb",
    "hormannbisecur",
    "hormannhsm",
    "ido",
    "intertechnov3",
    "ironlogic",
    "jcmtech",
    "jollymotors",
    "kingatesstylo4k",
    "legrand",
    "linear",
    "lineardelta3",
    "magellan",
    "marantec",
    "marantec24",
    "mastercode",
    "megacode",
    "merlin",
    "monarch",
    "motorline",
    "mutancode",
    "mutancomutancode",
    "neroradio",
    "nerosketch",
    "niceflo",
    "niceflorsone",
    "nicemhouse",
    "nicesmilo",
    "novoferm",
    "pecninin",
    "pecnin",
    "phoenixv2",
    "powersmart",
    "prastel",
    "princeton",
    "reversrb2",
    "roger",
    "rosh",
    "rossi",
    "sea",
    "secplusv1v2",
    "smc5326",
    "somfykeytis",
    "somfytelis",
    "steelmate",
    "stilmatic",
];

/// Alarm (car alarm / aftermarket) manufacturers (normalized names).
/// Same menu behaviour as barriers: Replay + export + delete only.
const ALARM_NAMES: &[&str] = &[
    "a2a4",
    "sla2a4",
    "a6a9",
    "sla6a9",
    "alligator",
    "alligators275",
    "aps1100",
    "aps2550",
    "aps1100aps2550",
    "b6b9",
    "slb6b9dop",
    "cenmaxst5",
    "cenmaxst7",
    "cenmax",
    "cfm",
    "faraon",
    "harpoon",
    "jaguar",
    "kgd",
    "leopard",
    "mongoose",
    "panteraclk",
    "panteraxsjaguar",
    "partisanrx",
    "pro1",
    "pro2",
    "pandorapro2",
    "reff",
    "sheriff",
    "tomahawk9010",
    "tomahawkzx35",
    "tomahawktz9030",
    "tz9030",
    "starline",
    "zx730",
    "zx750",
    "zx755",
    "zx930",
    "zx940",
    "zx1070",
    "zx1090",
    "zx7307501055",
];

fn barrier_set() -> &'static HashSet<String> {
    static BARRIER_SET: OnceLock<HashSet<String>> = OnceLock::new();
    BARRIER_SET.get_or_init(|| BARRIER_NAMES.iter().map(|s| (*s).to_string()).collect())
}

fn alarm_set() -> &'static HashSet<String> {
    static ALARM_SET: OnceLock<HashSet<String>> = OnceLock::new();
    ALARM_SET.get_or_init(|| ALARM_NAMES.iter().map(|s| (*s).to_string()).collect())
}

/// Returns true if the protocol string is KeeLoq with a barrier/gate/garage door manufacturer.
/// Protocol is typically "KeeLoq (ManufacturerName)" e.g. "KeeLoq (BFT)" or "KeeLoq (NICE_MHOUSE)".
pub fn is_keeloq_barrier(protocol: &str) -> bool {
    let protocol = protocol.trim();
    let open = protocol.find(" (");
    let Some(open_idx) = open else { return false };
    if !protocol[..open_idx].eq_ignore_ascii_case("keeloq") {
        return false;
    }
    let rest = protocol[open_idx + 2..].trim_start();
    let close = rest.rfind(')');
    let Some(close_idx) = close else { return false };
    let inner = rest[..close_idx].trim();
    if inner.is_empty() {
        return false;
    }
    barrier_set().contains(&normalize(inner))
}

/// Returns true if the protocol string is KeeLoq with an alarm (car alarm / aftermarket) manufacturer.
/// Same menu behaviour as barriers: Replay + export + delete only, no TX Lock/Unlock/Trunk/Panic.
pub fn is_keeloq_alarm(protocol: &str) -> bool {
    let protocol = protocol.trim();
    let open = protocol.find(" (");
    let Some(open_idx) = open else { return false };
    if !protocol[..open_idx].eq_ignore_ascii_case("keeloq") {
        return false;
    }
    let rest = protocol[open_idx + 2..].trim_start();
    let close = rest.rfind(')');
    let Some(close_idx) = close else { return false };
    let inner = rest[..close_idx].trim();
    if inner.is_empty() {
        return false;
    }
    alarm_set().contains(&normalize(inner))
}

/// True if this KeeLoq protocol should hide car-style TX actions (barrier/gate or alarm).
pub fn is_keeloq_non_car(protocol: &str) -> bool {
    is_keeloq_barrier(protocol) || is_keeloq_alarm(protocol)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn barrier_detection() {
        assert!(is_keeloq_barrier("KeeLoq (BFT)"));
        assert!(is_keeloq_barrier("KeeLoq (DoorHan)"));
        assert!(is_keeloq_barrier("KeeLoq (NICE_MHOUSE)"));
        assert!(is_keeloq_barrier("KeeLoq (Guard_RF-311A)"));
        assert!(is_keeloq_barrier("KeeLoq (Stilmatic)"));
        assert!(is_keeloq_barrier("KeeLoq (Motorline)"));
        assert!(!is_keeloq_barrier("KeeLoq (KIAV5)"));
        assert!(!is_keeloq_barrier("KeeLoq (Star Line)"));
        assert!(!is_keeloq_barrier("Ford V0"));
        assert!(!is_keeloq_barrier("Unknown"));
    }

    #[test]
    fn alarm_detection() {
        assert!(is_keeloq_alarm("KeeLoq (Star Line)"));
        assert!(is_keeloq_alarm("KeeLoq (Pantera_CLK)"));
        assert!(is_keeloq_alarm("KeeLoq (Sheriff)"));
        assert!(is_keeloq_alarm("KeeLoq (Alligator_S-275)"));
        assert!(is_keeloq_alarm("KeeLoq (Harpoon)"));
        assert!(is_keeloq_alarm("KeeLoq (Partisan_RX)"));
        assert!(is_keeloq_alarm("KeeLoq (Cenmax_St-7)"));
        assert!(is_keeloq_alarm("KeeLoq (Reff)"));
        assert!(!is_keeloq_alarm("KeeLoq (KIAV5)"));
        assert!(!is_keeloq_alarm("Ford V0"));
    }
}
