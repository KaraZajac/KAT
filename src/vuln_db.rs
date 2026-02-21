//! Vulnerability database: CVE entries matched by year range, makes, models, region.
//! Used for "Vuln Found" column and the Vulnerability detail panel.
//!
//! Each row has a unique `id` so the same CVE can appear multiple times with different
//! year/make/model scope (e.g. per-model year ranges for CVE-2022-37418).

/// One vulnerability scope: a single row in the DB. Uniquely identified by `id`.
/// The same CVE can appear in multiple entries with different year/make/model bounds.
#[derive(Debug, Clone)]
pub struct VulnEntry {
    /// Unique id for this row (same CVE can have multiple rows with different scope). Used for tracking and future export/dedupe.
    #[allow(dead_code)]
    pub id: u32,
    pub cve: &'static str,
    /// Inclusive start year (e.g. "2018"). "ALL" = no lower bound.
    pub year_start: &'static str,
    /// Inclusive end year (e.g. "2021"). "ALL" = no upper bound.
    pub year_end: &'static str,
    /// Makes that are affected (e.g. ["Renault"]). ["ALL"] = any make.
    pub makes: &'static [&'static str],
    /// Models that are affected (e.g. ["ZOE"], ["Civic"]). ["ALL"] = any model.
    pub models: &'static [&'static str],
    pub region: &'static str,
    pub description: &'static str,
    /// Source URL (e.g. NVD detail page) for further reading.
    pub url: &'static str,
}

pub const VULN_DB: [VulnEntry; 16] = [
    VulnEntry {
        id: 1,
        cve: "CVE-2022-38766",
        year_start: "2020",
        year_end: "2022",
        makes: &["Renault"],
        models: &["ZOE"],
        region: "ALL",
        description: "The remote keyless system on Renault ZOE 2021 vehicles sends 433.92 MHz RF signals from the same Rolling Codes set for each door-open request, which allows for a replay attack.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-38766",
    },
    VulnEntry {
        id: 2,
        cve: "CVE-2022-27254",
        year_start: "2016",
        year_end: "2019",
        makes: &["Honda"],
        models: &["Civic"],
        region: "ALL",
        description: "The remote keyless system on Honda Civic 2018 vehicles sends the same RF signal for each door-open request, which allows for a replay attack, a related issue to CVE-2019-20626.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-27254",
    },
    // CVE-2022-37418 (RollBack): per make/model/year from research table (RKE RollBack variant)
    // Honda
    VulnEntry {
        id: 3,
        cve: "CVE-2022-37418",
        year_start: "2016",
        year_end: "2018",
        makes: &["Honda"],
        models: &["Fit (hybrid)", "Fit Hybrid"],
        region: "ALL",
        description: "RKE RollBack: unlock/resync after capturing two consecutive key fob signals. Attacker can unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-37418",
    },
    VulnEntry {
        id: 4,
        cve: "CVE-2022-37418",
        year_start: "2018",
        year_end: "2018",
        makes: &["Honda"],
        models: &["Fit"],
        region: "ALL",
        description: "RKE RollBack: unlock/resync after capturing two consecutive key fob signals. Attacker can unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-37418",
    },
    VulnEntry {
        id: 5,
        cve: "CVE-2022-37418",
        year_start: "2017",
        year_end: "2017",
        makes: &["Honda"],
        models: &["City"],
        region: "ALL",
        description: "RKE RollBack: unlock/resync after capturing two consecutive key fob signals. Attacker can unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-37418",
    },
    VulnEntry {
        id: 6,
        cve: "CVE-2022-37418",
        year_start: "2016",
        year_end: "2022",
        makes: &["Honda"],
        models: &["Vezel"],
        region: "ALL",
        description: "RKE RollBack: unlock/resync after capturing two consecutive key fob signals. Attacker can unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-37418",
    },
    // Hyundai (Elantra 2013-2015 only; 2012 and Avante marked NO in table)
    VulnEntry {
        id: 7,
        cve: "CVE-2022-37418",
        year_start: "2013",
        year_end: "2015",
        makes: &["Hyundai"],
        models: &["Elantra"],
        region: "ALL",
        description: "RKE RollBack: unlock/resync after capturing two consecutive key fob signals. Attacker can unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-37418",
    },
    // Kia Cerato/Forte K3 (two year ranges in table)
    VulnEntry {
        id: 8,
        cve: "CVE-2022-37418",
        year_start: "2016",
        year_end: "2018",
        makes: &["Kia"],
        models: &["Cerato", "Forte", "K3"],
        region: "ALL",
        description: "RKE RollBack: unlock/resync after capturing two consecutive key fob signals. Attacker can unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-37418",
    },
    VulnEntry {
        id: 9,
        cve: "CVE-2022-37418",
        year_start: "2012",
        year_end: "2018",
        makes: &["Kia"],
        models: &["Cerato", "Forte", "K3"],
        region: "ALL",
        description: "RKE RollBack: unlock/resync after capturing two consecutive key fob signals. Attacker can unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-37418",
    },
    // Mazda — CVE-2022-36945 (RollBack with three consecutive signals; through 2020)
    VulnEntry {
        id: 10,
        cve: "CVE-2022-36945",
        year_start: "2018",
        year_end: "2018",
        makes: &["Mazda"],
        models: &["3"],
        region: "ALL",
        description: "The RKE receiving unit on certain Mazda vehicles through 2020 allows remote attackers to perform unlock operations and force a resynchronization after capturing three consecutive valid key-fob signals over the radio, aka a RollBack attack. The attacker retains the ability to unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-36945",
    },
    VulnEntry {
        id: 11,
        cve: "CVE-2022-36945",
        year_start: "2018",
        year_end: "2018",
        makes: &["Mazda"],
        models: &["2 Sedan"],
        region: "ALL",
        description: "The RKE receiving unit on certain Mazda vehicles through 2020 allows remote attackers to perform unlock operations and force a resynchronization after capturing three consecutive valid key-fob signals over the radio, aka a RollBack attack. The attacker retains the ability to unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-36945",
    },
    VulnEntry {
        id: 12,
        cve: "CVE-2022-36945",
        year_start: "2020",
        year_end: "2020",
        makes: &["Mazda"],
        models: &["2 HB (facelift)", "2 HB"],
        region: "ALL",
        description: "The RKE receiving unit on certain Mazda vehicles through 2020 allows remote attackers to perform unlock operations and force a resynchronization after capturing three consecutive valid key-fob signals over the radio, aka a RollBack attack. The attacker retains the ability to unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-36945",
    },
    VulnEntry {
        id: 13,
        cve: "CVE-2022-36945",
        year_start: "2019",
        year_end: "2019",
        makes: &["Mazda"],
        models: &["Cx-3", "CX-3"],
        region: "ALL",
        description: "The RKE receiving unit on certain Mazda vehicles through 2020 allows remote attackers to perform unlock operations and force a resynchronization after capturing three consecutive valid key-fob signals over the radio, aka a RollBack attack. The attacker retains the ability to unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-36945",
    },
    VulnEntry {
        id: 14,
        cve: "CVE-2022-36945",
        year_start: "2018",
        year_end: "2018",
        makes: &["Mazda"],
        models: &["Cx-5", "CX-5"],
        region: "ALL",
        description: "The RKE receiving unit on certain Mazda vehicles through 2020 allows remote attackers to perform unlock operations and force a resynchronization after capturing three consecutive valid key-fob signals over the radio, aka a RollBack attack. The attacker retains the ability to unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-36945",
    },
    // Nissan (Latio, Sylphy; Teana and Wish marked NO in table — excluded)
    VulnEntry {
        id: 15,
        cve: "CVE-2022-37418",
        year_start: "2007",
        year_end: "2012",
        makes: &["Nissan"],
        models: &["Latio"],
        region: "ALL",
        description: "RKE RollBack: unlock/resync after capturing two consecutive key fob signals. Attacker can unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-37418",
    },
    VulnEntry {
        id: 16,
        cve: "CVE-2022-37418",
        year_start: "2012",
        year_end: "2019",
        makes: &["Nissan"],
        models: &["Sylphy"],
        region: "ALL",
        description: "RKE RollBack: unlock/resync after capturing two consecutive key fob signals. Attacker can unlock indefinitely.",
        url: "https://nvd.nist.gov/vuln/detail/CVE-2022-37418",
    },
];

/// Match a capture's year/make/model/region against the DB.
/// Year: parsed as number; must be in [year_start, year_end] when entry has bounds.
/// Make/Model: capture value must match one of the entry's list, or entry list contains "ALL".
/// Region: "ALL" in entry matches any; otherwise case-insensitive match.
pub fn match_vulns(
    year: Option<&str>,
    make: Option<&str>,
    model: Option<&str>,
    region: Option<&str>,
) -> Vec<&'static VulnEntry> {
    let y = year.unwrap_or("");
    let m = make.unwrap_or("");
    let mod_ = model.unwrap_or("");
    let r = region.unwrap_or("");

    let year_num: Option<u32> = y.trim().parse().ok();

    VULN_DB
        .iter()
        .filter(|e| {
            year_in_range(e.year_start, e.year_end, year_num)
                && list_matches(e.makes, m)
                && list_matches(e.models, mod_)
                && (e.region == "ALL" || eq_ignore_case(e.region, r))
        })
        .collect()
}

fn year_in_range(start: &str, end: &str, capture_year: Option<u32>) -> bool {
    let has_start = start != "ALL" && !start.trim().is_empty();
    let has_end = end != "ALL" && !end.trim().is_empty();
    if !has_start && !has_end {
        return true;
    }
    let Some(yr) = capture_year else {
        return false;
    };
    if has_start {
        let Ok(s) = start.trim().parse::<u32>() else {
            return false;
        };
        if yr < s {
            return false;
        }
    }
    if has_end {
        let Ok(e) = end.trim().parse::<u32>() else {
            return false;
        };
        if yr > e {
            return false;
        }
    }
    true
}

fn list_matches(list: &[&'static str], capture_value: &str) -> bool {
    if list.is_empty() {
        return false;
    }
    if list.iter().any(|s| *s == "ALL") {
        return true;
    }
    list.iter()
        .any(|s| eq_ignore_case(s, capture_value))
}

fn eq_ignore_case(a: &str, b: &str) -> bool {
    a.eq_ignore_ascii_case(b)
}
