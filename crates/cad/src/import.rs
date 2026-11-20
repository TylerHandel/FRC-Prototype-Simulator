//! The import itself, start to finish, off the interface thread: fetch from Onshape (or take a STEP
//! file), run the sidecar, load the cadpack, read the mates, and propose (D8, D20).
use crate::cadpack::{self, Cadpack, Dir};
use crate::onshape::{self, Budget, Client, DocRef};
use crate::propose::{self, BomRow, Proposal};
use library::Library;
use model::Pose;
use std::collections::HashMap;
use std::io::BufRead;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

#[derive(Debug, Clone)]
pub enum Source {
    Onshape { url: String, access: String, secret: String, second_pose: bool },
    Step { path: PathBuf },
}

/// How to run the sidecar: the packaged program next to the app, or Python with OpenCascade.
#[derive(Debug, Clone)]
pub struct Sidecar { pub program: PathBuf, pub args: Vec<String> }

#[derive(Debug, Clone)]
pub enum Progress { Step(String), Fraction(f64, String) }

pub struct Imported {
    pub dir: PathBuf,
    pub cadpack: Cadpack,
    pub second: Option<Cadpack>,
    pub assembly: Option<serde_json::Value>,
    pub bom: HashMap<String, BomRow>,
    pub source: model::CadKind,
    pub notes: Vec<String>,
    pub calls: u64,
}

fn run_sidecar(sc: &Sidecar, step: &Path, out: &Path, progress: &dyn Fn(Progress)) -> anyhow::Result<()> {
    let mut cmd = Command::new(&sc.program);
    cmd.args(&sc.args).arg(step).arg(out).stdout(Stdio::piped()).stderr(Stdio::piped());
    #[cfg(windows)]
    { use std::os::windows::process::CommandExt; cmd.creation_flags(0x0800_0000); }
    let mut child = cmd.spawn().map_err(|e| anyhow::anyhow!("Couldn't start the CAD reader ({}): {e}", sc.program.display()))?;
    let stdout = child.stdout.take().unwrap();
    for line in std::io::BufReader::new(stdout).lines().map_while(Result::ok) {
        if let Some(rest) = line.strip_prefix("PROGRESS ") {
            let (f, what) = rest.split_once(' ').unwrap_or((rest, ""));
            progress(Progress::Fraction(f.parse().unwrap_or(0.0), what.to_string()));
        }
    }
    let out_ = child.wait_with_output()?;
    if !out_.status.success() {
        let err = String::from_utf8_lossy(&out_.stderr);
        anyhow::bail!("The CAD reader couldn't read this file: {}", err.lines().last().unwrap_or("no message"));
    }
    Ok(())
}

/// Fetches and reads, into `<project>/cad/`.
pub fn run(src: &Source, project: &Path, sidecar: &Sidecar, budget: PathBuf, progress: &dyn Fn(Progress)) -> anyhow::Result<Imported> {
    let dir = project.join("cad");
    std::fs::create_dir_all(&dir)?;
    let mut notes = vec![];
    match src {
        Source::Step { path } => {
            progress(Progress::Step("Reading the STEP file".into()));
            run_sidecar(sidecar, path, &dir.join("pack"), progress)?;
            let cp = Cadpack::load(&dir.join("pack"))?;
            Ok(Imported { dir, cadpack: cp, second: None, assembly: None, bom: HashMap::new(), source: model::CadKind::Step { path: path.display().to_string() }, notes, calls: 0 })
        }
        Source::Onshape { url, access, secret, second_pose } => {
            let mut d: DocRef = onshape::parse_url(url)?;
            let mut c = Client::new(access, secret, Budget { path: budget });
            progress(Progress::Step("Asking Onshape about the document".into()));
            let doc = c.document(&d.did)?;
            // the document's current workspace (a link's own can be out of date)
            if d.wvm == "w" || d.wvmid.is_empty() {
                if let Some(w) = doc["defaultWorkspace"]["id"].as_str() { d.wvm = "w".into(); d.wvmid = w.into(); }
            }
            progress(Progress::Step("Reading the mates".into()));
            let assembly = c.assembly(&d);
            if assembly.is_none() { notes.push("Onshape doesn't share this document's mates with your account (it does for your own documents and public ones), so joints will come from two poses or from clicking.".into()); }
            progress(Progress::Step("Reading the bill of materials".into()));
            let bom = c.bom(&d);
            let step = dir.join("source.step");
            c.step(&d, &step, &|s| progress(Progress::Step(s.to_string())))?;
            progress(Progress::Step("Reading the STEP file".into()));
            run_sidecar(sidecar, &step, &dir.join("pack"), progress)?;
            let cp = Cadpack::load(&dir.join("pack"))?;
            std::fs::write(dir.join("bom.json"), serde_json::to_string(&bom)?)?;
            if let Some(a) = &assembly { std::fs::write(dir.join("assembly.json"), a.to_string())?; }
            // a second pose, when the mates are refused and the document has another assembly
            let mut second = None;
            if assembly.is_none() && *second_pose {
                let others: Vec<(String, String)> = c.assemblies(&d)?.into_iter().filter(|(id, _)| *id != d.eid).collect();
                if let Some((eid, name)) = others.first() {
                    progress(Progress::Step(format!("Exporting '{name}' too, to compare poses")));
                    let d2 = DocRef { eid: eid.clone(), ..d.clone() };
                    let step2 = dir.join("second.step");
                    c.step(&d2, &step2, &|s| progress(Progress::Step(s.to_string())))?;
                    run_sidecar(sidecar, &step2, &dir.join("pack2"), progress)?;
                    second = Some(Cadpack::load(&dir.join("pack2"))?);
                    notes.push(format!("Joints compared against '{name}'"));
                } else {
                    notes.push("No second assembly to compare poses with: mark joints by clicking.".into());
                }
            }
            let calls = c.calls;
            notes.push(format!("{calls} Onshape API calls ({} this year)", c.budget.used()));
            Ok(Imported { dir, cadpack: cp, second, assembly, bom, source: model::CadKind::Onshape { url: url.clone(), document: d.did, workspace: d.wvmid, element: d.eid }, notes, calls })
        }
    }
}

/// Proposals for an import, with the CAD turned so `forward` and `up` are the robot's.
pub fn proposals(im: &Imported, lib: &Library, forward: Dir, up: Dir) -> (Pose, Proposal) {
    let to = cadpack::to_robot(&im.cadpack, forward, up);
    let (to, p) = proposals_at(im, lib, to);
    // centred on the drive wheels, when they were found (the robot's own centre, not its parts' box)
    if p.module_positions.len() >= 3 {
        let n = p.module_positions.len() as f64;
        let c = [p.module_positions.iter().map(|x| x[0]).sum::<f64>() / n, p.module_positions.iter().map(|x| x[1]).sum::<f64>() / n];
        if c[0].abs() > 0.002 || c[1].abs() > 0.002 {
            let shifted = Pose::new([to.pos[0] - c[0], to.pos[1] - c[1], to.pos[2]], to.quat);
            return proposals_at(im, lib, shifted);
        }
    }
    (to, p)
}

fn proposals_at(im: &Imported, lib: &Library, to: Pose) -> (Pose, Proposal) {
    let placed = propose::place(&im.cadpack, &to);
    let mut notes = im.notes.clone();
    let (links, joints, from) = if let Some(a) = &im.assembly {
        match propose::from_mates(a, &placed, &to) {
            Ok((l, j, n)) if l.len() > 1 || !j.is_empty() => { notes.extend(n); (l, j, "Onshape mates") }
            Ok((_, _, n)) => { notes.extend(n); notes.push("The mates left everything in one piece: using the part tree".into()); (propose::links_from_tree(&placed), vec![], "part tree") }
            Err(e) => { notes.push(format!("Couldn't read the mates: {e}")); (propose::links_from_tree(&placed), vec![], "part tree") }
        }
    } else if let Some(b) = &im.second {
        let placed_b = propose::place(b, &to);
        let (l, j, n) = propose::from_poses(&placed, &placed_b);
        notes.extend(n);
        if l.len() > 1 { (l, j, "two poses") } else { (propose::links_from_tree(&placed), vec![], "part tree") }
    } else {
        (propose::links_from_tree(&placed), vec![], "part tree")
    };
    (to, propose::propose(&im.cadpack, &to, lib, &im.bom, links, joints, notes, from))
}
