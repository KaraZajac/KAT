//! Vulnerability database: CVE entries matched by Year, Make, Model, Region.
//! Used for "Vuln Found" column and the Vulnerability detail panel.

/// One CVE entry. Year/Make/Model/Region use "ALL" to match any value.
#[derive(Debug, Clone)]
pub struct VulnEntry {
    pub cve: &'static str,
    pub year: &'static str,
    pub make: &'static str,
    pub model: &'static str,
    pub region: &'static str,
    pub description: &'static str,
}

pub const VULN_DB: [VulnEntry; 2] = [
    VulnEntry {
        cve: "CVE-2022-38766",
        year: "2021",
        make: "Renault",
        model: "ALL",
        region: "ALL",
        description: "The remote keyless system on Renault ZOE 2021 vehicles sends 433.92 MHz RF signals from the same Rolling Codes set for each door-open request, which allows for a replay attack.",
    },
    VulnEntry {
        cve: "CVE-2022-27254",
        year: "2018",
        make: "Honda",
        model: "Civic",
        region: "ALL",
        description: "The remote keyless system on Honda Civic 2018 vehicles sends the same RF signal for each door-open request, which allows for a replay attack, a related issue to CVE-2019-20626.",
    },
];

/// Match a capture's year/make/model/region against the DB. "ALL" in an entry matches any value.
/// Empty/None from capture is treated as empty string (so "ALL" matches it).
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

    VULN_DB
        .iter()
        .filter(|e| {
            (e.year == "ALL" || eq_ignore_case(e.year, y))
                && (e.make == "ALL" || eq_ignore_case(e.make, m))
                && (e.model == "ALL" || eq_ignore_case(e.model, mod_))
                && (e.region == "ALL" || eq_ignore_case(e.region, r))
        })
        .collect()
}

fn eq_ignore_case(a: &str, b: &str) -> bool {
    a.eq_ignore_ascii_case(b)
}
