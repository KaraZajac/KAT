//! Vulnerability database: CVE entries matched by year range, makes, models, region.
//! Used for "Vuln Found" column and the Vulnerability detail panel.

/// One CVE entry. Year range is inclusive; "ALL" for start/end means no bound.
/// Makes and models are arrays; any match counts. Use ["ALL"] to match any make/model.
#[derive(Debug, Clone)]
pub struct VulnEntry {
    pub cve: &'static str,
    /// Inclusive start year (e.g. "2018"). "ALL" = no lower bound.
    pub year_start: &'static str,
    /// Inclusive end year (e.g. "2021"). "ALL" = no upper bound.
    pub year_end: &'static str,
    /// Makes that are affected (e.g. ["Renault"], ["Honda", "Acura"]). ["ALL"] = any make.
    pub makes: &'static [&'static str],
    /// Models that are affected (e.g. ["ZOE"], ["Civic", "Accord"]). ["ALL"] = any model.
    pub models: &'static [&'static str],
    pub region: &'static str,
    pub description: &'static str,
}

pub const VULN_DB: [VulnEntry; 2] = [
    VulnEntry {
        cve: "CVE-2022-38766",
        year_start: "2020",
        year_end: "2022",
        makes: &["Renault"],
        models: &["ZOE"],
        region: "ALL",
        description: "The remote keyless system on Renault ZOE 2021 vehicles sends 433.92 MHz RF signals from the same Rolling Codes set for each door-open request, which allows for a replay attack.",
    },
    VulnEntry {
        cve: "CVE-2022-27254",
        year_start: "2016",
        year_end: "2019",
        makes: &["Honda"],
        models: &["Civic"],
        region: "ALL",
        description: "The remote keyless system on Honda Civic 2018 vehicles sends the same RF signal for each door-open request, which allows for a replay attack, a related issue to CVE-2019-20626.",
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
