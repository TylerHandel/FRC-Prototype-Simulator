//! Onshape's REST API, as little of it as an import needs (D8): the document, its elements, the
//! assembly definition (the mates), the bill of materials, and a STEP translation.
//!
//! Education accounts get about 2,500 calls a year, so every call is counted (`Budget`), polls back
//! off and never come faster than every 30 s, and nothing here runs in tests or CI.
use crate::propose::BomRow;
use base64::Engine as _;
use std::collections::HashMap;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::time::Duration;

#[derive(Debug, Clone, PartialEq)]
pub struct DocRef { pub host: String, pub did: String, pub wvm: String, pub wvmid: String, pub eid: String }

/// An Onshape link: https://cad.onshape.com/documents/<did>/<w|v|m>/<id>/e/<eid>, any host
/// (a team's own enterprise subdomain works too; the API is asked at cad.onshape.com).
pub fn parse_url(url: &str) -> anyhow::Result<DocRef> {
    let u = url.trim();
    let parts: Vec<&str> = u.split(['/', '?', '#']).collect();
    let find = |k: &str| parts.iter().position(|p| *p == k).and_then(|i| parts.get(i + 1)).map(|s| s.to_string());
    let did = find("documents").ok_or_else(|| anyhow::anyhow!("That doesn't look like an Onshape link: it should contain /documents/..."))?;
    let (wvm, wvmid) = ["w", "v", "m"].iter().find_map(|k| find(k).map(|id| (k.to_string(), id))).unwrap_or(("w".into(), String::new()));
    let eid = find("e").ok_or_else(|| anyhow::anyhow!("The link needs to point at the assembly's tab (it should contain /e/...). Open the assembly in Onshape and copy the link from the address bar."))?;
    Ok(DocRef { host: "https://cad.onshape.com".into(), did, wvm, wvmid, eid })
}

/// Calls used this year, kept in a small file in the user's app data folder.
pub struct Budget { pub path: PathBuf }

impl Budget {
    fn year() -> u64 { std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| 1970 + d.as_secs() / 31_556_952).unwrap_or(0) }
    pub fn used(&self) -> u64 {
        let v: serde_json::Value = std::fs::read_to_string(&self.path).ok().and_then(|s| serde_json::from_str(&s).ok()).unwrap_or_default();
        v.get(Self::year().to_string()).and_then(|x| x.as_u64()).unwrap_or(0)
    }
    pub fn add(&self, n: u64) {
        let mut v: serde_json::Value = std::fs::read_to_string(&self.path).ok().and_then(|s| serde_json::from_str(&s).ok()).unwrap_or_else(|| serde_json::json!({}));
        let y = Self::year().to_string();
        let now = v.get(&y).and_then(|x| x.as_u64()).unwrap_or(0) + n;
        v[y] = serde_json::json!(now);
        if let Some(d) = self.path.parent() { let _ = std::fs::create_dir_all(d); }
        let _ = std::fs::write(&self.path, v.to_string());
    }
}

pub struct Client { auth: String, agent: ureq::Agent, pub budget: Budget, pub calls: u64 }

#[derive(Debug)]
pub enum Refused { NotAllowed, Other(String) }

impl Client {
    pub fn new(access: &str, secret: &str, budget: Budget) -> Self {
        let auth = format!("Basic {}", base64::engine::general_purpose::STANDARD.encode(format!("{access}:{secret}")));
        let cfg = ureq::Agent::config_builder()
            .timeout_global(Some(Duration::from_secs(1800)))
            .proxy(ureq::Proxy::try_from_env())
            .http_status_as_error(false)
            .build();
        Client { auth, agent: cfg.into(), budget, calls: 0 }
    }
    fn count(&mut self) { self.calls += 1; self.budget.add(1); }

    fn get_json(&mut self, url: &str) -> Result<serde_json::Value, Refused> {
        self.count();
        let resp = self.agent.get(url).header("Authorization", &self.auth).header("Accept", "application/json;charset=UTF-8; qs=0.09").call().map_err(|e| Refused::Other(e.to_string()))?;
        let status = resp.status().as_u16();
        let mut body = resp.into_body();
        let text = body.with_config().limit(1 << 30).read_to_string().map_err(|e| Refused::Other(e.to_string()))?;
        match status {
            200..=299 => serde_json::from_str(&text).map_err(|e| Refused::Other(e.to_string())),
            401 => Err(Refused::Other("Onshape didn't accept the API keys (401). Check them in the import screen.".into())),
            403 | 404 => Err(Refused::NotAllowed),
            429 => Err(Refused::Other("Onshape says too many requests (429): wait a minute and try again.".into())),
            s => Err(Refused::Other(format!("Onshape answered {s}: {}", text.chars().take(200).collect::<String>()))),
        }
    }

    pub fn document(&mut self, did: &str) -> anyhow::Result<serde_json::Value> {
        self.get_json(&format!("https://cad.onshape.com/api/v6/documents/{did}")).map_err(|e| match e {
            Refused::NotAllowed => anyhow::anyhow!("Onshape won't show this document to your account. If it's another team's, they need to make it public or share it with you."),
            Refused::Other(s) => anyhow::anyhow!(s),
        })
    }
    /// The document's assemblies: (element id, name)
    pub fn assemblies(&mut self, d: &DocRef) -> anyhow::Result<Vec<(String, String)>> {
        let v = self.get_json(&format!("https://cad.onshape.com/api/v6/documents/d/{}/{}/{}/elements", d.did, d.wvm, d.wvmid)).map_err(|e| anyhow::anyhow!("{e:?}"))?;
        Ok(v.as_array().into_iter().flatten().filter(|e| e["elementType"].as_str() == Some("ASSEMBLY")).map(|e| (e["id"].as_str().unwrap_or("").to_string(), e["name"].as_str().unwrap_or("").to_string())).collect())
    }
    /// The mates, if Onshape serves them for this document (it does for your own and for public ones).
    pub fn assembly(&mut self, d: &DocRef) -> Option<serde_json::Value> {
        self.get_json(&format!("https://cad.onshape.com/api/v6/assemblies/d/{}/{}/{}/e/{}?includeMateFeatures=true&includeMateConnectors=true&includeNonSolids=false", d.did, d.wvm, d.wvmid, d.eid)).ok()
    }
    /// The bill of materials: each part's material, mass and vendor number, by part name.
    pub fn bom(&mut self, d: &DocRef) -> HashMap<String, BomRow> {
        let Ok(v) = self.get_json(&format!("https://cad.onshape.com/api/v6/assemblies/d/{}/{}/{}/e/{}/bom?indented=false&multiLevel=true&generateIfAbsent=false&thumbnail=false", d.did, d.wvm, d.wvmid, d.eid)) else { return HashMap::new() };
        parse_bom(&v)
    }
    /// Asks Onshape for a STEP export of the assembly, waits for it, downloads it to `out`.
    pub fn step(&mut self, d: &DocRef, out: &Path, progress: &dyn Fn(&str)) -> anyhow::Result<()> {
        self.count();
        let body = serde_json::json!({ "formatName": "STEP", "storeInDocument": false, "flattenAssemblies": false }).to_string();
        let resp = self.agent.post(&format!("https://cad.onshape.com/api/v6/assemblies/d/{}/{}/{}/e/{}/translations", d.did, d.wvm, d.wvmid, d.eid))
            .header("Authorization", &self.auth).header("Content-Type", "application/json").header("Accept", "application/json").send(&body)?;
        let status = resp.status().as_u16();
        let v: serde_json::Value = serde_json::from_str(&resp.into_body().read_to_string()?)?;
        if status == 403 { anyhow::bail!("Onshape won't export this assembly for your account (the document doesn't allow exporting)."); }
        let id = v["id"].as_str().ok_or_else(|| anyhow::anyhow!("Onshape didn't start the export: {v}"))?.to_string();
        let mut wait = 30u64;
        let started = std::time::Instant::now();
        let result = loop {
            for s in (0..wait).rev() {
                progress(&format!("Onshape is making the STEP file ({} s so far, checking again in {s} s)", started.elapsed().as_secs()));
                std::thread::sleep(Duration::from_secs(1));
            }
            let st = self.get_json(&format!("https://cad.onshape.com/api/v6/translations/{id}")).map_err(|e| anyhow::anyhow!("{e:?}"))?;
            match st["requestState"].as_str() {
                Some("DONE") => break st,
                Some("FAILED") => anyhow::bail!("Onshape's export failed: {}", st["failureReason"].as_str().unwrap_or("no reason given")),
                _ => {}
            }
            if started.elapsed().as_secs() > 3600 { anyhow::bail!("Onshape's export took more than an hour"); }
            wait = (wait + 15).min(120);
        };
        let ext = result["resultExternalDataIds"][0].as_str().ok_or_else(|| anyhow::anyhow!("no file in Onshape's answer"))?;
        progress("Downloading the STEP file");
        self.count();
        let resp = self.agent.get(&format!("https://cad.onshape.com/api/v6/documents/d/{}/externaldata/{ext}", d.did)).header("Authorization", &self.auth).call()?;
        let mut body = resp.into_body();
        let mut r = body.with_config().limit(u64::MAX).reader();
        let mut f = std::fs::File::create(out)?;
        let mut buf = vec![0u8; 1 << 20];
        let mut total = 0usize;
        loop {
            let n = r.read(&mut buf)?;
            if n == 0 { break; }
            std::io::Write::write_all(&mut f, &buf[..n])?;
            total += n;
            if total % (8 << 20) < n { progress(&format!("Downloading the STEP file ({} MB)", total >> 20)); }
        }
        Ok(())
    }
}

pub fn parse_bom(v: &serde_json::Value) -> HashMap<String, BomRow> {
    let headers: HashMap<String, String> = v["headers"].as_array().into_iter().flatten().map(|h| (h["id"].as_str().unwrap_or("").to_string(), h["name"].as_str().unwrap_or("").to_string())).collect();
    let mut out = HashMap::new();
    for r in v["rows"].as_array().into_iter().flatten() {
        let get = |name: &str| r["headerIdToValue"].as_object().and_then(|o| o.iter().find(|(k, _)| headers.get(*k).map(String::as_str) == Some(name)).map(|(_, v)| v.clone()));
        let Some(name) = get("Name").and_then(|x| x.as_str().map(String::from)) else { continue };
        let material = get("Material").and_then(|m| m["displayName"].as_str().map(String::from));
        let mass = get("Mass").and_then(|m| m.as_str().map(String::from)).and_then(|s| {
            let mut it = s.split_whitespace();
            let v: f64 = it.next()?.parse().ok()?;
            Some(match it.next().unwrap_or("kg") { "lb" => v * library::LB, "g" => v / 1000.0, "oz" => v * library::LB / 16.0, _ => v })
        });
        let part_number = get("Part number").and_then(|x| x.as_str().map(String::from));
        out.insert(name.clone(), BomRow { name, material, mass, part_number });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn links() {
        let d = parse_url("https://2910.onshape.com/documents/f33ab032b00dc4b711aa86a6/w/951c0955d38b370eab02b587/e/415a66cb7efdf4aba6199086?configuration=default&renderMode=0").unwrap();
        assert_eq!((d.did.as_str(), d.wvm.as_str(), d.eid.as_str()), ("f33ab032b00dc4b711aa86a6", "w", "415a66cb7efdf4aba6199086"));
        assert!(parse_url("https://example.com/foo").is_err());
        assert!(parse_url("https://cad.onshape.com/documents/abc/w/def").is_err());
    }
    #[test]
    fn bom_rows() {
        let v = serde_json::json!({ "headers": [{"id": "a", "name": "Name"}, {"id": "b", "name": "Material"}, {"id": "c", "name": "Mass"}],
            "rows": [{ "headerIdToValue": { "a": "Kraken X60", "b": { "displayName": "Aluminum - 6061" }, "c": "1.2 lb" } }] });
        let b = parse_bom(&v);
        let r = &b["Kraken X60"];
        assert_eq!(r.material.as_deref(), Some("Aluminum - 6061"));
        assert!((r.mass.unwrap() - 1.2 * library::LB).abs() < 1e-9);
    }
}
