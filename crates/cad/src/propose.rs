//! Guess, then show the guess (interface rule 3): links and joints proposed from Onshape's mates,
//! from two poses of the same robot, or from the part tree (D20); rollers from part and BOM names
//! confirmed by the parts' cylinders; motors, swerve modules and masses from the BOM.
use crate::cadpack::{Cadpack, PartRow};
use library::Library;
use model::math::*;
use model::*;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LinkProp { pub name: String, pub parts: Vec<String>, pub source: String }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointProp {
    pub name: String,
    pub kind: JointKind,
    /// link indices into `Proposal::links`
    pub parent: usize,
    pub child: usize,
    pub anchor: V3,
    pub axis: V3,
    pub range: Option<[f64; 2]>,
    pub source: String,
    pub accepted: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RollerProp {
    pub part: String,
    pub name: String,
    pub link: usize,
    pub center: V3,
    pub axis: V3,
    pub diameter: f64,
    pub width: f64,
    pub wheel: String,
    /// rollers on one shaft share this
    pub shaft: usize,
    pub source: String,
    pub accepted: bool,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Proposal {
    pub links: Vec<LinkProp>,
    pub joints: Vec<JointProp>,
    pub rollers: Vec<RollerProp>,
    /// (part id, library motor id)
    pub motors: Vec<(String, String)>,
    pub module: Option<String>,
    pub module_positions: Vec<[f64; 2]>,
    /// part id -> (library material id, kg)
    pub masses: HashMap<String, (Option<String>, f64)>,
    pub notes: Vec<String>,
    pub links_from: String,
}

/// What the BOM says about a part, by name: (material, mass kg, vendor part number)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BomRow { pub name: String, pub material: Option<String>, pub mass: Option<f64>, pub part_number: Option<String> }

/// Parts in the robot frame.
pub struct Placed<'a> { pub row: &'a PartRow, pub pose: Pose, pub volume: f64, pub com: V3, pub bbox: [V3; 2], pub hardware: bool }

pub fn place<'a>(cp: &'a Cadpack, to_robot: &Pose) -> Vec<Placed<'a>> {
    cp.parts.iter().map(|p| {
        let pose = to_robot.mul(&p.pose());
        let s = &cp.shapes[p.shape];
        let com = pose.apply(s.com);
        let (lo, hi) = (s.bbox[0], s.bbox[1]);
        let mut a = [f64::INFINITY; 3];
        let mut b = [f64::NEG_INFINITY; 3];
        for i in 0..8 {
            let w = pose.apply([if i & 1 == 0 { lo[0] } else { hi[0] }, if i & 2 == 0 { lo[1] } else { hi[1] }, if i & 4 == 0 { lo[2] } else { hi[2] }]);
            for k in 0..3 { a[k] = a[k].min(w[k]); b[k] = b[k].max(w[k]); }
        }
        Placed { row: p, pose, volume: s.volume, com, bbox: [a, b], hardware: s.hardware }
    }).collect()
}

fn strip_instance(s: &str) -> String {
    let t = s.trim();
    match t.rfind(" <") { Some(i) if t.ends_with('>') => t[..i].to_string(), _ => t.to_string() }
}

/// Links from the part tree: each top-level sub-assembly is a link; the drivetrain's (or the
/// biggest) is the chassis. Always available, usually too coarse or too fine, a start.
pub fn links_from_tree(placed: &[Placed]) -> Vec<LinkProp> {
    let mut groups: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for p in placed {
        let key = p.row.path.get(1).filter(|_| p.row.path.len() > 2).cloned().unwrap_or_else(|| "(top level)".into());
        groups.entry(key).or_default().push(p.row.id.clone());
    }
    let chassis_key = groups.keys().find(|k| { let l = k.to_lowercase(); ["drive", "chassis", "base", "frame", "swerve"].iter().any(|w| l.contains(w)) }).cloned()
        .or_else(|| groups.iter().max_by_key(|(_, v)| v.len()).map(|(k, _)| k.clone())).unwrap_or_default();
    let mut links = vec![LinkProp { name: "Chassis".into(), parts: vec![], source: "part tree".into() }];
    for (k, v) in groups {
        if k == chassis_key || k == "(top level)" { links[0].parts.extend(v); } else { links.push(LinkProp { name: strip_instance(&k), parts: v, source: "part tree".into() }); }
    }
    links
}

/// Links and joints from Onshape's assembly definition (`GET .../assemblies/...?includeMateFeatures=true`).
/// Port of spikes/onshape_mates.py: Fastened mates, mate groups and sub-assemblies Onshape marks rigid
/// join parts into links; revolute, slider and cylindrical mates between two links are joints.
pub fn from_mates(asm: &serde_json::Value, placed: &[Placed], to_robot: &Pose) -> anyhow::Result<(Vec<LinkProp>, Vec<JointProp>, Vec<String>)> {
    let root = asm.get("rootAssembly").ok_or_else(|| anyhow::anyhow!("no rootAssembly: is this an assembly definition?"))?;
    let subs = asm.get("subAssemblies").and_then(|s| s.as_array()).cloned().unwrap_or_default();
    let key = |i: &serde_json::Value| format!("{}/{}/{}", i["documentId"].as_str().unwrap_or(""), i["elementId"].as_str().unwrap_or(""), i.get("fullConfiguration").and_then(|x| x.as_str()).unwrap_or(""));
    let sub_by_key: HashMap<String, &serde_json::Value> = subs.iter().map(|s| (key(s), s)).collect();
    let mut inst: HashMap<String, &serde_json::Value> = HashMap::new();
    for i in root["instances"].as_array().into_iter().flatten() { inst.insert(i["id"].as_str().unwrap_or("").into(), i); }
    for s in &subs { for i in s["instances"].as_array().into_iter().flatten() { inst.insert(i["id"].as_str().unwrap_or("").into(), i); } }
    let occs: Vec<(Vec<String>, Pose)> = root["occurrences"].as_array().into_iter().flatten().map(|o| {
        let path: Vec<String> = o["path"].as_array().into_iter().flatten().filter_map(|x| x.as_str().map(String::from)).collect();
        let t: Vec<f64> = o["transform"].as_array().into_iter().flatten().filter_map(|x| x.as_f64()).collect();
        (path, if t.len() >= 12 { Pose::from_rows(&t) } else { Pose::default() })
    }).collect();
    let is_part = |id: &str| inst.get(id).and_then(|i| i["type"].as_str()) == Some("Part");
    // occurrence path (ids) to the part's names path, as the STEP export spells it
    let names = |path: &[String]| -> String { path.iter().map(|id| inst.get(id).and_then(|i| i["name"].as_str()).unwrap_or("?").to_string()).collect::<Vec<_>>().join("/") };
    // cadpack parts by their names path below the root, both exact and without instance numbers
    let mut by_names: HashMap<String, Vec<&str>> = HashMap::new();
    let mut by_loose: HashMap<String, Vec<&str>> = HashMap::new();
    for p in placed {
        let tail = p.row.path.iter().skip(1).cloned().collect::<Vec<_>>();
        by_names.entry(tail.join("/")).or_default().push(&p.row.id);
        by_loose.entry(tail.iter().map(|s| strip_instance(s)).collect::<Vec<_>>().join("/")).or_default().push(&p.row.id);
    }
    let parts: Vec<&Vec<String>> = occs.iter().map(|(p, _)| p).filter(|p| p.last().is_some_and(|id| is_part(id))).collect();
    let occ_pose: HashMap<Vec<String>, Pose> = occs.iter().cloned().collect();
    let mut occ_to_part: HashMap<Vec<String>, String> = HashMap::new();
    let mut used: HashMap<String, usize> = HashMap::new();
    let mut unmatched_occ: Vec<&Vec<String>> = vec![];
    for p in &parts {
        let n = names(p);
        let loose = p.iter().map(|id| strip_instance(inst.get(id).and_then(|i| i["name"].as_str()).unwrap_or("?"))).collect::<Vec<_>>().join("/");
        let cands = by_names.get(&n).or_else(|| by_loose.get(&loose));
        match cands {
            Some(c) => {
                let k = used.entry(n.clone()).or_insert(0);
                if let Some(id) = c.get(*k).or(c.last()) { occ_to_part.insert((*p).clone(), id.to_string()); }
                *k += 1;
            }
            None => unmatched_occ.push(p),
        }
    }
    // names that differ (a STEP part named by its part number, Onshape's instance by its
    // description): the same part sits in the same place in both, so match by placement
    let taken: std::collections::HashSet<String> = occ_to_part.values().cloned().collect();
    let mut free: Vec<(&str, Pose)> = placed.iter().filter(|p| !taken.contains(&p.row.id)).map(|p| (p.row.id.as_str(), p.row.pose())).collect();
    let mut unmatched = 0;
    for p in unmatched_occ {
        let Some(t) = occ_pose.get(p) else { unmatched += 1; continue };
        let hit = free.iter().position(|(_, q)| len(sub(q.pos, t.pos)) < 2e-4 && quat_angle(q.quat, t.quat) < 2e-3);
        match hit { Some(i) => { occ_to_part.insert(p.clone(), free[i].0.to_string()); free.remove(i); } None => unmatched += 1 }
    }
    // union-find over occurrence paths
    let mut parent: HashMap<Vec<String>, Vec<String>> = HashMap::new();
    fn find(parent: &mut HashMap<Vec<String>, Vec<String>>, x: &Vec<String>) -> Vec<String> {
        let mut cur = x.clone();
        loop {
            let p = parent.entry(cur.clone()).or_insert_with(|| cur.clone()).clone();
            if p == cur { return cur; }
            cur = p;
        }
    }
    fn union(parent: &mut HashMap<Vec<String>, Vec<String>>, a: &Vec<String>, b: &Vec<String>) {
        let (ra, rb) = (find(parent, a), find(parent, b));
        if ra != rb { parent.insert(ra, rb); }
    }
    let leaves = |prefix: &[String]| -> Vec<Vec<String>> { parts.iter().filter(|p| p.len() >= prefix.len() && p[..prefix.len()] == *prefix).map(|p| (*p).clone()).collect() };
    let mut owners: Vec<(Vec<String>, &serde_json::Value)> = vec![(vec![], root)];
    for (path, _) in &occs {
        if let Some(i) = path.last().and_then(|id| inst.get(id)) {
            if i["type"].as_str() == Some("Assembly") { if let Some(s) = sub_by_key.get(&key(i)) { owners.push((path.clone(), s)); } }
        }
    }
    let mut moving: Vec<(String, String, V3, V3, Vec<String>, Vec<String>, Option<[f64; 2]>)> = vec![];
    for (prefix, asmv) in &owners {
        if !prefix.is_empty() && asmv["isRigid"].as_bool() == Some(true) {
            let g = leaves(prefix);
            for x in g.iter().skip(1) { union(&mut parent, &g[0], x); }
            continue;
        }
        for f in asmv["features"].as_array().into_iter().flatten() {
            if f["suppressed"].as_bool() == Some(true) { continue; }
            let fd = &f["featureData"];
            match f["featureType"].as_str() {
                Some("mateGroup") => {
                    let mut g = vec![];
                    for o in fd["occurrences"].as_array().into_iter().flatten() {
                        let mut p = prefix.clone();
                        p.extend(o["occurrence"].as_array().into_iter().flatten().filter_map(|x| x.as_str().map(String::from)));
                        g.extend(leaves(&p));
                    }
                    for x in g.iter().skip(1) { union(&mut parent, &g[0], x); }
                }
                Some("mate") => {
                    let ents = fd["matedEntities"].as_array().cloned().unwrap_or_default();
                    if ents.len() != 2 { continue; }
                    let side = |e: &serde_json::Value| -> Option<Vec<String>> {
                        let rel: Vec<String> = e["matedOccurrence"].as_array().into_iter().flatten().filter_map(|x| x.as_str().map(String::from)).collect();
                        if rel.is_empty() { let mut o = vec!["origin".to_string()]; o.extend(prefix.iter().cloned()); return Some(o); }
                        let mut p = prefix.clone(); p.extend(rel);
                        if parts.iter().any(|x| **x == p) { Some(p) } else { None }
                    };
                    let (Some(a), Some(b)) = (side(&ents[0]), side(&ents[1])) else { continue };
                    let kind = fd["mateType"].as_str().unwrap_or("");
                    if kind == "FASTENED" { union(&mut parent, &a, &b); continue; }
                    if !matches!(kind, "REVOLUTE" | "SLIDER" | "CYLINDRICAL") { continue; }
                    let cs = &ents[0]["matedCS"];
                    let v = |k: &str| -> V3 { let a: Vec<f64> = cs[k].as_array().into_iter().flatten().filter_map(|x| x.as_f64()).collect(); if a.len() == 3 { [a[0], a[1], a[2]] } else { [0.0, 0.0, 1.0] } };
                    let occ = { let mut p = prefix.clone(); p.extend(ents[0]["matedOccurrence"].as_array().into_iter().flatten().filter_map(|x| x.as_str().map(String::from))); p };
                    let t = occ_pose.get(&occ).copied().unwrap_or_default();
                    let lim = &fd["mateLimits"];
                    let range = match kind {
                        "REVOLUTE" => lim["limitAxialZMin"].as_f64().zip(lim["limitAxialZMax"].as_f64()).map(|(a, b)| [a, b]),
                        _ => lim["limitZMin"].as_f64().zip(lim["limitZMax"].as_f64()).map(|(a, b)| [a, b]),
                    };
                    moving.push((kind.to_string(), fd["name"].as_str().unwrap_or("mate").to_string(), to_robot.apply(t.apply(v("origin"))), to_robot.rotate(t.rotate(v("zAxis"))), a, b, range));
                }
                _ => {}
            }
        }
    }
    // links: the groups of parts
    let mut groups: BTreeMap<Vec<String>, Vec<Vec<String>>> = BTreeMap::new();
    for p in &parts { let r = find(&mut parent, p); groups.entry(r).or_default().push((*p).clone()); }
    let mut sorted: Vec<(Vec<String>, Vec<Vec<String>>)> = groups.into_iter().collect();
    sorted.sort_by_key(|(_, v)| std::cmp::Reverse(v.len()));
    let mut group_index: HashMap<Vec<String>, usize> = HashMap::new();
    let mut links: Vec<LinkProp> = vec![];
    // a group of one or two parts nobody mated joins the nearest bigger group in its sub-assembly (flagged)
    let mut loose = 0;
    for (root_key, members) in &sorted {
        let ids: Vec<String> = members.iter().filter_map(|m| occ_to_part.get(m).cloned()).collect();
        if members.len() <= 2 && !links.is_empty() {
            let sub = members[0].first().cloned();
            let host = links.iter().position(|l| l.parts.iter().any(|pid| placed.iter().any(|p| &p.row.id == pid && p.row.path.get(1) == sub.as_ref().and_then(|s| inst.get(s)).and_then(|i| i["name"].as_str()).map(String::from).as_ref()))).unwrap_or(0);
            links[host].parts.extend(ids);
            group_index.insert(root_key.clone(), host);
            loose += members.len();
            continue;
        }
        let name = members.iter().filter_map(|m| m.first().and_then(|id| inst.get(id)).and_then(|i| i["name"].as_str())).next().map(strip_instance).unwrap_or_else(|| format!("Link {}", links.len()));
        group_index.insert(root_key.clone(), links.len());
        links.push(LinkProp { name, parts: ids, source: "Onshape mates".into() });
    }
    if let Some(l) = links.first_mut() { l.name = "Chassis".into(); }
    let mut joints = vec![];
    let mut seen: HashMap<(usize, usize), usize> = HashMap::new();
    for (kind, name, anchor, axis, a, b, range) in moving {
        let ga = if a[0] == "origin" { 0 } else { *group_index.get(&find(&mut parent, &a)).unwrap_or(&0) };
        let gb = if b[0] == "origin" { 0 } else { *group_index.get(&find(&mut parent, &b)).unwrap_or(&0) };
        if ga == gb { continue; }
        // the parent is the bigger (or the chassis); mates between the same two links are one joint
        let (p, c) = if ga == 0 || (gb != 0 && links[ga].parts.len() >= links[gb].parts.len()) { (ga, gb) } else { (gb, ga) };
        if seen.contains_key(&(p, c)) { continue; }
        seen.insert((p, c), joints.len());
        let jk = if kind == "SLIDER" { JointKind::Slide } else { JointKind::Hinge };
        joints.push(JointProp { name: format!("{} ({})", links[c].name, name), kind: jk, parent: p, child: c, anchor, axis: norm(axis), range, source: format!("Onshape mate '{name}'"), accepted: true });
    }
    let mut notes = vec![];
    let (links, joints, n) = tidy(links, joints, placed);
    notes.extend(n);
    if unmatched > 0 { notes.push(format!("{unmatched} parts in the mates weren't found in the STEP")); }
    if loose > 0 { notes.push(format!("{loose} parts nobody mated joined the nearest link in their sub-assembly")); }
    Ok((links, joints, notes))
}

pub const STEERING: &str = "Swerve steering";

/// Joints that aren't mechanisms: a few parts turning on a shaft (a roller, a pulley, an idler)
/// rides with its parent, and four upright hinges in the drivetrain are its swerve modules'
/// steering, which the drivetrain model stands in for. Their links merge into their parents.
pub fn tidy(mut links: Vec<LinkProp>, joints: Vec<JointProp>, placed: &[Placed]) -> (Vec<LinkProp>, Vec<JointProp>, Vec<String>) {
    let solid = |l: &LinkProp| l.parts.iter().filter(|p| placed.iter().any(|x| &x.row.id == *p && !x.hardware)).count();
    if std::env::var("FPS_DEBUG_TIDY").is_ok() {
        for j in &joints { eprintln!("in: {} {:?} parent {} ({} solid) child {} ({} solid) axis {:?} at {:?}", j.name, j.kind, j.parent, solid(&links[j.parent]), j.child, solid(&links[j.child]), j.axis.map(|x| (x * 100.0).round() / 100.0), j.anchor.map(|x| (x * 1000.0).round() / 1000.0)); }
    }
    let mut merge_into: Vec<Option<usize>> = vec![None; links.len()];
    let mut notes = vec![];
    // swerve steering: upright hinges low in the robot, on the chassis
    let steering: Vec<usize> = joints.iter().enumerate().filter(|(_, j)| j.kind == JointKind::Hinge && j.axis[2].abs() > 0.95 && j.parent == 0 && j.anchor[2] < 0.25 && solid(&links[j.child]) < 80).map(|(i, _)| i).collect();
    let steering = if steering.len() >= 3 { steering } else { vec![] };
    // something spinning on its own axis (a roller, a pulley, an idler) stays close to it; an arm
    // reaches away from its pivot
    let reach = |l: &LinkProp, j: &JointProp| -> f64 {
        let mut r: f64 = 0.0;
        for p in placed.iter().filter(|x| !x.hardware && l.parts.contains(&x.row.id)) {
            for i in 0..8 {
                let c = [p.bbox[i & 1][0], p.bbox[(i >> 1) & 1][1], p.bbox[(i >> 2) & 1][2]];
                let d = sub(c, j.anchor);
                r = r.max(len(sub(d, scale(j.axis, dot(d, j.axis)))));
            }
        }
        r
    };
    for (i, j) in joints.iter().enumerate() {
        let spins = j.kind == JointKind::Hinge && reach(&links[j.child], j) < 0.075;
        if steering.contains(&i) || (spins && j.child != 0) { merge_into[j.child] = Some(j.parent); }
    }
    if !steering.is_empty() { notes.push(format!("{} upright hinges in the drivetrain taken as swerve steering", steering.len())); }
    let small = merge_into.iter().filter(|m| m.is_some()).count() - steering.len();
    if small > 0 { notes.push(format!("{small} small turning parts (rollers, pulleys) ride with what carries them")); }
    // follow merges to their end
    let root = |mut i: usize| { let mut n = 0; while let Some(p) = merge_into[i] { i = p; n += 1; if n > 64 { break; } } i };
    for i in 0..links.len() { if merge_into[i].is_some() { let r = root(i); let parts = std::mem::take(&mut links[i].parts); links[r].parts.extend(parts); } }
    let keep: Vec<usize> = (0..links.len()).filter(|i| merge_into[*i].is_none()).collect();
    let map = |i: usize| keep.iter().position(|k| *k == root(i)).unwrap_or(0);
    // steering axes are kept as markers (not joints) so the drivetrain gets the modules' places
    let marks: Vec<JointProp> = steering.iter().map(|&i| JointProp { name: STEERING.into(), parent: 0, child: 0, accepted: false, ..joints[i].clone() }).collect();
    let mut joints: Vec<JointProp> = joints.into_iter().enumerate().filter(|(i, j)| !steering.contains(i) && merge_into[j.child].is_none())
        .map(|(_, mut j)| { j.parent = map(j.parent); j.child = map(j.child); j }).filter(|j| j.parent != j.child).collect();
    let mut links: Vec<LinkProp> = keep.into_iter().map(|i| links[i].clone()).collect();
    let orphans = attach_orphans(&mut links, &mut joints, placed);
    if orphans > 0 { notes.push(format!("{orphans} groups of parts with no joint to the rest were fixed to the part they touch")); }
    joints.extend(marks);
    (links, joints, notes)
}

/// A link no joint reaches from the chassis (its mates to the rest were lost, or never made) is
/// fixed to the link it touches, one from its own sub-assembly first; its own joints come with it.
/// Returns how many were fixed.
pub fn attach_orphans(links: &mut Vec<LinkProp>, joints: &mut Vec<JointProp>, placed: &[Placed]) -> usize {
    let by_id: HashMap<&str, &Placed> = placed.iter().map(|p| (p.row.id.as_str(), p)).collect();
    let boxes = |l: &LinkProp| -> Vec<[V3; 2]> {
        let solid: Vec<[V3; 2]> = l.parts.iter().filter_map(|p| by_id.get(p.as_str())).filter(|p| !p.hardware).map(|p| p.bbox).collect();
        if solid.is_empty() { l.parts.iter().filter_map(|p| by_id.get(p.as_str())).map(|p| p.bbox).collect() } else { solid }
    };
    let sub_of = |l: &LinkProp| -> Option<String> {
        let mut n: HashMap<&str, usize> = HashMap::new();
        for p in l.parts.iter().filter_map(|p| by_id.get(p.as_str())) { if let Some(s) = p.row.path.get(1) { *n.entry(s.as_str()).or_default() += 1; } }
        n.into_iter().max_by_key(|x| x.1).map(|x| x.0.to_string())
    };
    let gap = |a: &[V3; 2], b: &[V3; 2]| -> f64 { (0..3).map(|k| (a[0][k] - b[1][k]).max(b[0][k] - a[1][k]).max(0.0)).map(|x| x * x).sum::<f64>().sqrt() };
    let mut fixed = 0;
    loop {
        let mut reached = vec![false; links.len()];
        if reached.is_empty() { return fixed; }
        reached[0] = true;
        let mut grew = true;
        while grew {
            grew = false;
            for j in joints.iter() { if reached[j.parent] && !reached[j.child] { reached[j.child] = true; grew = true; } }
        }
        let Some(o) = (0..links.len()).filter(|i| !reached[*i]).max_by_key(|i| links[*i].parts.len()) else { return fixed };
        let ob = boxes(&links[o]);
        let os = sub_of(&links[o]);
        let host = (0..links.len()).filter(|i| reached[*i]).map(|h| {
            let hb = boxes(&links[h]);
            let g = ob.iter().flat_map(|a| hb.iter().map(move |b| gap(a, b))).fold(f64::INFINITY, f64::min);
            let other = if os.is_some() && sub_of(&links[h]) != os { 0.05 } else { 0.0 };
            (h, g + other)
        }).min_by(|a, b| a.1.total_cmp(&b.1)).map_or(0, |x| x.0);
        let parts = std::mem::take(&mut links[o].parts);
        links[host].parts.extend(parts);
        links.remove(o);
        let re = |i: usize| if i == o { if host > o { host - 1 } else { host } } else if i > o { i - 1 } else { i };
        for j in joints.iter_mut() { j.parent = re(j.parent); j.child = re(j.child); }
        joints.retain(|j| j.parent != j.child);
        fixed += 1;
    }
}

/// Links and joints from two poses of the same robot (D20 2): parts that made the same rigid move
/// between them are a link; each link's move against its neighbour is a hinge or a slide. Port of
/// spikes/pose_diff.py, with the parent chosen as the link nearest the joint's axis.
pub fn from_poses(a: &[Placed], b: &[Placed]) -> (Vec<LinkProp>, Vec<JointProp>, Vec<String>) {
    // the second pose may hold the robot one level down (a "Scoring Positions" assembly with the
    // robot and a REEF in it): line it up on the first's top assembly, by name
    let root = a.first().and_then(|p| p.row.path.first()).map(|s| strip_instance(s)).unwrap_or_default();
    let mut bm: HashMap<String, Vec<Pose>> = HashMap::new();
    for p in b {
        let at = p.row.path.iter().position(|n| strip_instance(n) == root);
        let key = match at { Some(i) => p.row.path[i + 1..].join("/"), None if p.row.path.first().map(|s| strip_instance(s)) == Some(root.clone()) => strip_path(&p.row.path), None => continue };
        bm.entry(key).or_default().push(p.pose);
    }
    let mut am: HashMap<String, Vec<(&str, Pose)>> = HashMap::new();
    for p in a { am.entry(strip_path(&p.row.path)).or_default().push((&p.row.id, p.pose)); }
    let same = |x: &Pose, y: &Pose| len(sub(x.pos, y.pos)) < 3e-4 && quat_angle(x.quat, y.quat) < 2e-3;
    // moves of uniquely named parts first
    let mut groups: Vec<(Pose, Vec<String>)> = vec![];
    for (k, va) in &am {
        let Some(vb) = bm.get(k) else { continue };
        if va.len() != 1 || vb.len() != 1 { continue; }
        let mv = vb[0].mul(&va[0].1.inverse());
        match groups.iter_mut().find(|g| same(&g.0, &mv)) { Some(g) => g.1.push(va[0].0.to_string()), None => groups.push((mv, vec![va[0].0.to_string()])) }
    }
    groups.sort_by_key(|g| std::cmp::Reverse(g.1.len()));
    // repeats go to the group whose move lands them on a twin
    for (k, va) in &am {
        let Some(vb) = bm.get(k) else { continue };
        if va.len() == 1 && vb.len() == 1 { continue; }
        for (id, pa) in va {
            if let Some(g) = groups.iter_mut().find(|g| vb.iter().any(|t| len(sub(g.0.mul(pa).pos, t.pos)) < 5e-4)) { g.1.push(id.to_string()); }
        }
    }
    let big: Vec<(Pose, Vec<String>)> = groups.into_iter().filter(|g| g.1.len() >= 3).collect();
    if big.is_empty() { return (vec![], vec![], vec!["the two poses share no parts".into()]); }
    let base = big[0].0;
    let mut links: Vec<LinkProp> = big.iter().enumerate().map(|(i, g)| LinkProp { name: if i == 0 { "Chassis".into() } else { format!("Moving part {i}") }, parts: g.1.clone(), source: "two poses".into() }).collect();
    let bbox_of = |ids: &[String]| -> Vec<[V3; 2]> { a.iter().filter(|p| ids.contains(&p.row.id) && !p.hardware).map(|p| p.bbox).collect() };
    // how far each link moved between the poses (its parts' centre): a link rides on one that
    // moved less, so the chain builds outward from the base
    let moved: Vec<f64> = big.iter().map(|g| { let c = bbox_center(&bbox_of(&g.1)); len(sub(base.inverse().mul(&g.0).apply(c), c)) }).collect();
    let mut joints = vec![];
    for (ci, g) in big.iter().enumerate().skip(1) {
        // name it after its parts' most common sub-assembly, one level deeper where that tells
        // links apart (three stages of one arm)
        let mut c: HashMap<String, usize> = HashMap::new();
        for id in &g.1 {
            if let Some(p) = a.iter().find(|p| &p.row.id == id && !p.hardware) {
                let n = if p.row.path.len() > 3 { p.row.path[2].as_str() } else { p.row.path.get(1).map(String::as_str).unwrap_or("?") };
                *c.entry(strip_instance(n)).or_default() += 1;
            }
        }
        if let Some((n, _)) = c.into_iter().max_by_key(|x| x.1) { links[ci].name = n; }
        let mut best: Option<(f64, usize, JointKind, V3, V3, f64)> = None;
        for (pi, h) in big.iter().enumerate() {
            if pi == ci || (pi != 0 && moved[pi] >= moved[ci] - 1e-4) { continue; }
            let rel = h.0.inverse().mul(&g.0);
            let (kind, axis, point, amount) = screw(&rel);
            if kind == JointKind::Fixed { continue; }
            // `rel` (the child's move with the parent's undone) works in the first pose's frame, so
            // its axis point is already where the joint is in the first pose
            let anchor = if kind == JointKind::Hinge { point } else { [0.0; 3] };
            let near = bbox_of(&h.1).iter().map(|bb| dist_box(anchor, bb)).fold(f64::INFINITY, f64::min);
            let score = if kind == JointKind::Slide { amount * 0.01 } else { near } + if pi == 0 { 0.0 } else { 0.001 };
            if best.is_none_or(|b| score < b.0) { best = Some((score, pi, kind, axis, anchor, amount)); }
        }
        if let Some((_, pi, kind, axis, anchor, amount)) = best {
            let anchor = if kind == JointKind::Slide { bbox_center(&bbox_of(&g.1)) } else { anchor };
            let range = if amount >= 0.0 { [0.0, amount] } else { [amount, 0.0] };
            // a few millimetres or degrees is a configuration difference, not a mechanism
            let real = if kind == JointKind::Slide { amount.abs() > 0.012 } else { amount.abs() > 0.05 };
            joints.push(JointProp { name: format!("{} {}", links[ci].name, if kind == JointKind::Slide { "slide" } else { "pivot" }), kind, parent: pi, child: ci, anchor, axis: norm(axis), range: Some(range), source: "two poses".into(), accepted: real });
        }
    }
    // a link whose move was too small to be a mechanism goes back into the link it was on
    let mut merged = vec![false; links.len()];
    for j in joints.iter().filter(|j| !j.accepted) {
        let parts = std::mem::take(&mut links[j.child].parts);
        links[j.parent].parts.extend(parts);
        merged[j.child] = true;
    }
    if merged.iter().any(|m| *m) {
        let map: Vec<Option<usize>> = { let mut n = 0; merged.iter().map(|m| if *m { None } else { n += 1; Some(n - 1) }).collect() };
        joints = joints.into_iter().filter(|j| j.accepted).map(|mut j| { j.parent = map[j.parent].unwrap_or(0); j.child = map[j.child].unwrap_or(0); j }).collect();
        links = links.into_iter().enumerate().filter(|(i, _)| !merged[*i]).map(|(_, l)| l).collect();
    }
    let notes = vec!["Joints seen between the two poses only; a joint that didn't move between them isn't found, and the travel seen is a minimum".into()];
    (links, joints, notes)
}

/// A wheel family from a part's name.
fn family(name: &str) -> Option<&'static str> {
    let n = name.to_lowercase();
    [("compliant", "compliant"), ("flex wheel", "flex"), ("omni", "omni"), ("mecanum", "mecanum"), ("stealth", "stealth"), ("silicone", "silicone"),
     ("urethane", "urethane"), ("nitrile", "nitrile"), ("tread", "tread"), ("colson", "colson"), ("grip tape", "grip")]
        .iter().find(|(k, _)| n.contains(k)).map(|(_, f)| *f)
}

/// A durometer written in a part's name ("45A", "60 A").
fn durometer(name: &str) -> Option<f64> {
    let b = name.as_bytes();
    for i in 0..b.len().saturating_sub(2) {
        if b[i].is_ascii_digit() && b[i + 1].is_ascii_digit() && (i == 0 || !b[i - 1].is_ascii_digit()) {
            let rest = &name[i + 2..];
            let rest = rest.trim_start();
            if rest.starts_with('A') && rest[1..].chars().next().is_none_or(|c| !c.is_alphanumeric()) { return name[i..i + 2].parse().ok(); }
        }
    }
    None
}

fn strip_path(p: &[String]) -> String { p.iter().skip(1).cloned().collect::<Vec<_>>().join("/") }
fn quat_angle(a: Quat, b: Quat) -> f64 { let d = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]).abs().min(1.0); 2.0 * d.acos() }
fn dist_box(p: V3, b: &[V3; 2]) -> f64 { let d: V3 = std::array::from_fn(|k| (b[0][k] - p[k]).max(0.0).max(p[k] - b[1][k])); len(d) }
fn bbox_center(bs: &[[V3; 2]]) -> V3 {
    let mut lo = [f64::INFINITY; 3]; let mut hi = [f64::NEG_INFINITY; 3];
    for b in bs { for k in 0..3 { lo[k] = lo[k].min(b[0][k]); hi[k] = hi[k].max(b[1][k]); } }
    if lo[0].is_finite() { lerp(lo, hi, 0.5) } else { [0.0; 3] }
}

/// A relative move as a turn about a line or a straight shift: (kind, axis, a point on it, amount)
fn screw(rel: &Pose) -> (JointKind, V3, V3, f64) {
    let ang = quat_angle(rel.quat, QI);
    if ang < 1e-3 {
        let d = len(rel.pos);
        return if d < 3e-4 { (JointKind::Fixed, [0.0, 0.0, 1.0], [0.0; 3], 0.0) } else { (JointKind::Slide, scale(rel.pos, 1.0 / d), [0.0; 3], d) };
    }
    let s = (1.0 - rel.quat[0] * rel.quat[0]).sqrt().max(1e-12);
    let mut axis = [rel.quat[1] / s, rel.quat[2] / s, rel.quat[3] / s];
    let mut amount = 2.0 * rel.quat[0].clamp(-1.0, 1.0).acos();
    if amount > std::f64::consts::PI { amount -= std::f64::consts::TAU; }
    if axis[0] + axis[1] + axis[2] < 0.0 { axis = neg(axis); amount = -amount; }
    // the point on the axis nearest the origin solves (I - R) p = t across the axis; exactly,
    // p = (t + cot(angle/2) axis x t) / 2
    let t = sub(rel.pos, scale(axis, dot(rel.pos, axis)));
    let p = scale(add(t, scale(cross(axis, t), 1.0 / (amount / 2.0).tan())), 0.5);
    (JointKind::Hinge, axis, p, amount)
}

/// Rollers: parts whose name (or BOM row) is a library wheel, confirmed by a cylinder of about the
/// wheel's radius; their axis and width from the geometry. Coaxial rollers on one link share a shaft.
pub fn rollers(cp: &Cadpack, placed: &[Placed], links: &[LinkProp], lib: &Library, bom: &HashMap<String, BomRow>) -> (Vec<RollerProp>, Vec<String>) {
    let link_of: HashMap<&str, usize> = links.iter().enumerate().flat_map(|(i, l)| l.parts.iter().map(move |p| (p.as_str(), i))).collect();
    let mut out: Vec<RollerProp> = vec![];
    let mut notes = vec![];
    for p in placed {
        if p.hardware { continue; }
        let names = [p.row.name.clone(), bom.get(&p.row.name).map(|b| format!("{} {}", b.name, b.part_number.clone().unwrap_or_default())).unwrap_or_default()];
        let named = names.iter().find_map(|n| lib.match_wheel(n));
        let fam = names.iter().find_map(|n| family(n));
        if named.is_none() && fam.is_none() { continue; }
        let axes = cp.axes(p.row);
        // its biggest round face is its tread
        let Some((ax, r)) = axes.iter().filter_map(|a| a.r.iter().cloned().fold(None, |m: Option<f64>, x| Some(m.map_or(x, |m| m.max(x)))).map(|r| (a, r))).filter(|(_, r)| *r > 0.008 && *r < 0.11).max_by(|a, b| a.1.partial_cmp(&b.1).unwrap()) else {
            notes.push(format!("{}: named like a wheel but has no round face to roll on", p.row.name));
            continue;
        };
        // the library wheel nearest in size and hardness, of its family
        let d_in = 2.0 * r / library::IN;
        let duro = names.iter().find_map(|n| durometer(n));
        let w = match (named, fam) {
            (Some(w), _) if (w.diameter_in - d_in).abs() < 0.4 && duro.is_none_or(|d| (w.durometer_a - d).abs() < 6.0) => w,
            (_, Some(f)) => match lib.wheels.iter().filter(|w| w.name.to_lowercase().contains(f) || w.id.contains(f)).min_by(|a, b| {
                let sa = (a.diameter_in - d_in).abs() + duro.map_or(0.0, |d| 0.03 * (a.durometer_a - d).abs());
                let sb = (b.diameter_in - d_in).abs() + duro.map_or(0.0, |d| 0.03 * (b.durometer_a - d).abs());
                sa.partial_cmp(&sb).unwrap() }) { Some(w) => w, None => match named { Some(w) => w, None => continue } },
            (Some(w), None) => w,
            (None, None) => continue,
        };
        let to = |v: V3| v; // axes are in the CAD frame; placed poses are robot frame: re-derive below
        let _ = to;
        // the axis in the robot frame: through the part's own frame
        let local = &cp.shapes[p.row.shape].axes.iter().find(|a| a.r.contains(&r)).cloned().unwrap_or_else(|| ax.clone());
        let axis = norm(p.pose.rotate(local.d));
        let pt = p.pose.apply(local.p);
        let com = p.com;
        let center = add(pt, scale(axis, dot(sub(com, pt), axis)));
        // width: the part's extent along its axis
        let s = &cp.shapes[p.row.shape];
        let mut lo = f64::INFINITY; let mut hi = f64::NEG_INFINITY;
        for i in 0..8 {
            let c = [s.bbox[i & 1][0], s.bbox[(i >> 1) & 1][1], s.bbox[(i >> 2) & 1][2]];
            let t = dot(sub(c, local.p), local.d);
            lo = lo.min(t); hi = hi.max(t);
        }
        let width = (hi - lo).max(0.005);
        out.push(RollerProp { part: p.row.id.clone(), name: w.name.clone(), link: *link_of.get(p.row.id.as_str()).unwrap_or(&0), center, axis, diameter: 2.0 * r, width, wheel: w.id.clone(), shaft: 0, source: "name and cylinder".into(), accepted: true });
    }
    // shafts: same link, axes parallel and on one line
    let mut shaft = 0;
    for i in 0..out.len() {
        if let Some(j) = (0..i).find(|&j| out[j].link == out[i].link && dot(out[j].axis, out[i].axis).abs() > 0.999 && len(cross(sub(out[i].center, out[j].center), out[j].axis)) < 0.004) {
            out[i].shaft = out[j].shaft;
        } else { out[i].shaft = shaft; shaft += 1; }
    }
    (out, notes)
}

/// Drive wheels (treads on the chassis at floor level) become the drivetrain's module positions.
pub fn drivetrain(rollers: &mut Vec<RollerProp>, placed: &[Placed], lib: &Library) -> (Option<String>, Vec<[f64; 2]>) {
    let module = placed.iter().filter(|p| !p.hardware).find_map(|p| lib.match_module(&p.row.path.join(" "))).map(|m| m.id.clone());
    let drive: Vec<usize> = rollers.iter().enumerate().filter(|(_, r)| r.link == 0 && r.center[2] < 0.08 && r.diameter > 0.07 && r.axis[2].abs() < 0.3).map(|(i, _)| i).collect();
    let positions: Vec<[f64; 2]> = drive.iter().map(|&i| [rollers[i].center[0], rollers[i].center[1]]).collect();
    if positions.len() >= 3 { let keep: Vec<RollerProp> = rollers.iter().enumerate().filter(|(i, _)| !drive.contains(i)).map(|(_, r)| r.clone()).collect(); *rollers = keep; }
    (module, if positions.len() >= 3 { positions } else { vec![] })
}

/// Masses: the BOM's where it has one, else the part's volume times its material's density
/// (the BOM's material, then the name, then aluminium).
/// A part that only marks space (a clearance or keep-out box, a frame-perimeter envelope): it
/// weighs nothing.
pub const TOO_HEAVY: f64 = 10.0;

pub fn is_reference(name: &str) -> bool {
    let n = name.to_lowercase();
    ["clearance", "keep out", "keepout", "keep-out", "envelope", "reference", "extension limit", "frame perimeter", "bounding box"].iter().any(|k| n.contains(k))
}

/// Each part's material and mass: the BOM's mass when it has one, else volume times the density of
/// its material (aluminium when unknown). A solid can't hold more than its box, so a broken solid
/// whose volume comes out bigger is held to half its box.
pub fn masses(placed: &[Placed], lib: &Library, bom: &HashMap<String, BomRow>) -> HashMap<String, (Option<String>, f64)> {
    placed.iter().map(|p| {
        let row = bom.get(&p.row.name);
        let mat = row.and_then(|b| b.material.as_ref()).and_then(|m| lib.match_material(m)).or_else(|| lib.match_material(&p.row.name)).map(|m| m.id.clone());
        let density = mat.as_ref().and_then(|m| lib.material(m)).map_or(2700.0, |m| m.density);
        let d = sub(p.bbox[1], p.bbox[0]);
        let boxed = (d[0] * d[1] * d[2]).abs();
        let volume = if p.volume.is_finite() && p.volume > 0.0 && p.volume <= boxed * 1.01 { p.volume } else { 0.5 * boxed };
        // no single part of a robot weighs 10 kg without the BOM saying so: it's a reference body
        // (a camera's field of view, a clearance volume) that came along in the part
        // Onshape reports 0 for a part with no material: that's no answer, not a weightless part
        let mass = match row.and_then(|b| b.mass).filter(|m| *m > 0.0) {
            _ if is_reference(&p.row.name) => 0.0,
            Some(m) => m,
            None if volume * density > TOO_HEAVY => 0.0,
            None => volume * density,
        };
        (p.row.id.clone(), (mat, mass))
    }).collect()
}

/// Everything proposed for a cadpack, links from `links` (mates, poses or the tree).
pub fn propose(cp: &Cadpack, to_robot: &Pose, lib: &Library, bom: &HashMap<String, BomRow>, links: Vec<LinkProp>, joints: Vec<JointProp>, mut notes: Vec<String>, from: &str) -> Proposal {
    let placed = place(cp, to_robot);
    let (mut rollers, rn) = rollers(cp, &placed, &links, lib, bom);
    notes.extend(rn);
    let (module, mut module_positions) = drivetrain(&mut rollers, &placed, lib);
    // swerve steering axes from the mates give the modules' places when the treads didn't
    let steer: Vec<[f64; 2]> = joints.iter().filter(|j| j.name == STEERING).map(|j| [j.anchor[0], j.anchor[1]]).collect();
    let joints: Vec<JointProp> = joints.into_iter().filter(|j| j.name != STEERING).collect();
    if module_positions.is_empty() && steer.len() >= 3 { module_positions = steer; }
    // a motor is named like one and sized like one (a swerve module's parts often carry the motor's name)
    let motor_sized = |p: &Placed| { let d = sub(p.bbox[1], p.bbox[0]); let (lo, hi) = (d.iter().cloned().fold(f64::INFINITY, f64::min), d.iter().cloned().fold(0.0, f64::max)); p.volume > 3e-5 && lo > 0.025 && hi < 0.2 };
    let motors = placed.iter().filter(|p| !p.hardware && motor_sized(p)).filter_map(|p| lib.match_motor(&p.row.name).or_else(|| bom.get(&p.row.name).and_then(|b| lib.match_motor(&b.name))).map(|m| (p.row.id.clone(), m.id.clone()))).collect();
    let masses = masses(&placed, lib, bom);
    let ignored: Vec<&str> = placed.iter().filter(|p| !p.hardware && masses.get(&p.row.id).is_some_and(|m| m.1 == 0.0)).map(|p| p.row.name.as_str()).collect();
    if !ignored.is_empty() {
        let mut names: Vec<&str> = ignored.clone(); names.sort(); names.dedup();
        notes.push(format!("{} parts count as reference only (no mass, no collision): {}. Give one a mass in its inspector if it's real.", ignored.len(), names.iter().take(4).cloned().collect::<Vec<_>>().join(", ")));
    }
    Proposal { masses, links, joints, rollers, motors, module, module_positions, notes, links_from: from.into() }
}

/// Makes the robot from an accepted proposal (D20): parts, links with their masses and collision
/// boxes, joints with a guessed drive each, rollers on shafts with a drive each, mechanisms, and the
/// drivetrain. Keeps the frame's bumpers.
pub fn apply(r: &mut Robot, cp: &Cadpack, to_robot: &Pose, prop: &Proposal, lib: &Library) {
    let placed = place(cp, to_robot);
    let by_id: HashMap<&str, &Placed> = placed.iter().map(|p| (p.row.id.as_str(), p)).collect();
    r.parts = placed.iter().map(|p| Part { id: p.row.id.clone(), name: p.row.name.clone(), path: p.row.path.clone(), fingerprint: Fingerprint { volume: p.volume, com: p.com, bbox: p.bbox },
        material: prop.masses.get(&p.row.id).and_then(|m| m.0.clone()), mass: prop.masses.get(&p.row.id).map(|m| m.1), hardware: p.hardware }).collect();
    // start over from the chassis
    let bumpers: Vec<Shape> = r.link(CHASSIS).map(|l| l.shapes.iter().filter(|s| s.role == ShapeRole::Bumper).cloned().collect()).unwrap_or_default();
    r.links.retain(|l| l.id == CHASSIS);
    r.joints.clear(); r.drives.clear(); r.shafts.clear(); r.rollers.clear(); r.mechanisms.clear(); r.controls = Controls::default(); r.sensors.clear();
    let mut ids = vec![CHASSIS];
    for (i, lp) in prop.links.iter().enumerate() {
        let shapes = vec![];
        let mass: f64 = lp.parts.iter().filter_map(|p| prop.masses.get(p).map(|m| m.1)).sum();
        if i == 0 {
            let c = r.link_mut(CHASSIS).unwrap();
            c.parts = lp.parts.clone();
            c.shapes = shapes;
            c.shapes.extend(bumpers.clone());
            c.mass = Some(mass.max(5.0) + 6.0);
            c.collision = Collision::Simple;
        } else {
            let id = r.id();
            ids.push(id);
            r.links.push(Link { id, name: lp.name.clone(), shapes, parts: lp.parts.clone(), mass: Some(mass.max(0.05)), color: palette(i), collision: Collision::Detailed, material: None });
        }
    }
    let motor_default = prop.motors.iter().map(|m| m.1.clone()).fold(HashMap::new(), |mut h: HashMap<String, usize>, m| { *h.entry(m).or_default() += 1; h })
        .into_iter().max_by_key(|x| x.1).map(|x| x.0).unwrap_or_else(|| "kraken-x60-foc".into());
    let stall = lib.motor(&motor_default).map_or(7.0, |m| m.stall_torque) * 60.0 / lib.motor(&motor_default).map_or(366.0, |m| m.stall_current);
    for j in prop.joints.iter().filter(|j| j.accepted) {
        let (Some(&p), Some(&c)) = (ids.get(j.parent), ids.get(j.child)) else { continue };
        if r.joint_of(c).is_some() { continue; }
        let jid = r.id();
        r.joints.push(Joint { id: jid, name: j.name.clone(), kind: j.kind, parent: p, child: c, anchor: j.anchor, axis: j.axis, range: j.range, initial: 0.0, damping: 0.0, friction: 0.0, follows: None, spring: None, source: j.source.clone() });
        let did = r.id();
        // geared to hold what it carries with room to move it: the load from everything past the
        // joint (its mass, and for a pivot the farthest it reaches), against one motor at 60 A
        let (mass, reach) = carried(prop, &by_id, j);
        let (count, ratio, spool) = if j.kind == JointKind::Slide {
            let need = 2.0 * mass * 9.81 * 0.0222;
            (1, (need / (0.9 * stall)).clamp(5.0, 30.0).round(), 0.0222)
        } else {
            let need = 2.0 * mass * 9.81 * reach;
            let one = (need / (0.9 * stall)).clamp(20.0, 400.0);
            if one > 150.0 { (2, (one / 2.0).round(), 0.0) } else { (1, one.round(), 0.0) }
        };
        r.drives.push(Drive { id: did, name: format!("{} motor (guessed: check it)", j.name), motor: motor_default.clone(), count, ratio, efficiency: 0.9, stator_limit: 60.0, supply_limit: 60.0,
            target: DriveTarget::Joint { joint: jid, spool_radius: spool }, control: Control::Ideal, brake: true });
    }
    // rollers on shafts
    let mut shaft_ids: HashMap<(usize, usize), Id> = HashMap::new();
    for x in prop.rollers.iter().filter(|x| x.accepted) {
        let link = ids.get(x.link).copied().unwrap_or(CHASSIS);
        let sid = *shaft_ids.entry((x.link, x.shaft)).or_insert_with(|| {
            let sid = r.id();
            r.shafts.push(model::Shaft { id: sid, name: format!("{} shaft", x.name), link, inertia: 2e-4, drag: 2e-4 });
            let did = r.id();
            r.drives.push(Drive { id: did, name: format!("{} rollers (guessed: check it)", r.link(link).map_or("?".into(), |l| l.name.clone())), motor: motor_default.clone(), count: 1, ratio: 3.0, efficiency: 0.9,
                stator_limit: 40.0, supply_limit: 40.0, target: DriveTarget::Shaft { shaft: sid }, control: Control::Voltage, brake: true });
            sid
        });
        let rid = r.id();
        r.rollers.push(Roller { id: rid, name: x.name.clone(), shaft: sid, ratio: 1.0, center: x.center, axis: x.axis, width: x.width, wheel: x.wheel.clone(), diameter: Some(x.diameter), kind: RollerKind::Plain, part: Some(x.part.clone()) });
    }
    // mechanisms: each link riding on the chassis, with whatever rides on it
    let children = |l: Id, r: &Robot| -> Vec<Id> { let mut out = vec![l]; let mut i = 0; while i < out.len() { let p = out[i]; for j in r.joints.iter().filter(|j| j.parent == p) { out.push(j.child); } i += 1; } out };
    let tops: Vec<Id> = r.joints.iter().filter(|j| j.parent == CHASSIS).map(|j| j.child).collect();
    for t in tops {
        let links = children(t, r);
        let has_rollers = r.shafts.iter().any(|s| links.contains(&s.link));
        let id = r.id();
        let name = r.link(t).map_or("Mechanism".into(), |l| l.name.clone());
        r.mechanisms.push(Mechanism { id, name, kind: MechKind::Custom, fidelity: if has_rollers { Fidelity::Full } else { Fidelity::Concept }, links, template: None, pickup: None, holders: vec![], color: String::new() });
    }
    // rollers on links no joint moves (on the chassis): a fixed mechanism of their own, Full
    if r.shafts.iter().any(|s| s.link == CHASSIS) {
        let id = r.id();
        r.mechanisms.push(Mechanism { id, name: "Rollers on the chassis".into(), kind: MechKind::Custom, fidelity: Fidelity::Full, links: vec![], template: None, pickup: None, holders: vec![], color: String::new() });
    }
    if let Some(m) = &prop.module {
        let d = r.drivetrain.get_or_insert_with(Drivetrain::default);
        d.module = m.clone();
        if let Some(lm) = lib.module(m) { d.ratio = lm.drive_ratio; d.wheel = lm.wheel.clone(); }
    }
    if !prop.module_positions.is_empty() {
        let d = r.drivetrain.get_or_insert_with(Drivetrain::default);
        d.positions = prop.module_positions.clone();
        // the frame from the modules: their spread plus the usual inset
        let xs = prop.module_positions.iter().map(|p| p[0].abs()).fold(0.0, f64::max);
        let ys = prop.module_positions.iter().map(|p| p[1].abs()).fold(0.0, f64::max);
        r.frame.length = 2.0 * xs + 0.127;
        r.frame.width = 2.0 * ys + 0.127;
    }
    templates::assign_preset_buttons(r);
    for d in r.drives.clone() {
        if let DriveTarget::Shaft { .. } = d.target {
            if !r.controls.bindings.iter().any(|b| b.button == Button::RT) { r.controls.bindings.push(Binding { button: Button::RT, action: Action::Run { drive: d.id, value: 10.0, until: None } }); }
            else if !r.controls.bindings.iter().any(|b| b.button == Button::RB) { r.controls.bindings.push(Binding { button: Button::RB, action: Action::Run { drive: d.id, value: 10.0, until: None } }); }
        }
    }
    for j in r.joints.clone() {
        let Some(d) = r.drives.iter().find(|d| matches!(d.target, DriveTarget::Joint { joint, .. } if joint == j.id)).map(|d| d.id) else { continue };
        let pid = r.id();
        // "out" is the end of its travel farther from where the CAD has it
        let far = j.range.map_or(1.0, |[lo, hi]| if lo.abs() > hi.abs() { lo } else { hi });
        r.controls.presets.push(Preset { reef_gap: model::default_reef_gap(), id: pid, name: format!("{} out", j.name), targets: vec![PresetTarget { drive: d, value: far }] });
        let pid2 = r.id();
        r.controls.presets.push(Preset { reef_gap: model::default_reef_gap(), id: pid2, name: format!("{} home", j.name), targets: vec![PresetTarget { drive: d, value: 0.0 }] });
    }
    templates::assign_preset_buttons(r);
    fit_collision(r, cp, to_robot);
}

/// Collision shapes for every link made of CAD parts, from the parts' meshes (M5): `Box` one
/// box round the link, `Simple` the 24 biggest parts, `Detailed` every part (up to 200 shapes),
/// thin parts split into flat slabs. Rollers' own parts are left out (the roller contact squeezes
/// pieces against them), and so are fasteners and reference bodies. Bumpers and hand-made blocks
/// stay.
pub fn fit_collision(r: &mut Robot, cp: &Cadpack, to_robot: &Pose) {
    let placed = place(cp, to_robot);
    let by_id: HashMap<&str, &Placed> = placed.iter().map(|p| (p.row.id.as_str(), p)).collect();
    let rollers: std::collections::HashSet<String> = r.rollers.iter().filter_map(|x| x.part.clone()).collect();
    let weightless: std::collections::HashSet<String> = r.parts.iter().filter(|p| p.mass == Some(0.0)).map(|p| p.id.clone()).collect();
    for l in &mut r.links {
        if l.parts.is_empty() { continue; }
        let mut ps: Vec<&Placed> = l.parts.iter().filter_map(|p| by_id.get(p.as_str()).copied())
            .filter(|p| !p.hardware && p.volume > 0.0 && !rollers.contains(&p.row.id) && !weightless.contains(&p.row.id)).collect();
        ps.sort_by(|a, b| b.volume.total_cmp(&a.volume));
        let mut sets: Vec<(String, Vec<V3>)> = vec![];
        match l.collision {
            Collision::None => {}
            Collision::Box => {
                let pts: Vec<V3> = ps.iter().flat_map(|p| (0..8).map(move |i| [p.bbox[i & 1][0], p.bbox[(i >> 1) & 1][1], p.bbox[(i >> 2) & 1][2]])).collect();
                if !pts.is_empty() { sets.push((l.name.clone(), crate::hull::support(&pts))); }
            }
            level => {
                let (parts, cap) = if level == Collision::Simple { (24, 64) } else { (usize::MAX, 200) };
                for p in ps.iter().take(parts) {
                    let Some(m) = cp.mesh(p.row.shape) else { continue };
                    for h in crate::hull::part_hulls(&m, &p.pose, p.volume) { sets.push((p.row.name.clone(), h)); }
                    if sets.len() >= cap { break; }
                }
            }
        }
        let keep: Vec<Shape> = l.shapes.iter().filter(|s| s.role == ShapeRole::Bumper || !matches!(s.kind, ShapeKind::Hull { .. }) && s.role != ShapeRole::Ghost).cloned().collect();
        l.shapes = keep;
        for (name, pts) in sets {
            if pts.len() < 4 { continue; }
            let c = { let mut lo = [f64::INFINITY; 3]; let mut hi = [f64::NEG_INFINITY; 3]; for q in &pts { for k in 0..3 { lo[k] = lo[k].min(q[k]); hi[k] = hi[k].max(q[k]); } } lerp(lo, hi, 0.5) };
            let points = pts.iter().map(|q| { let d = sub(*q, c); [round4(d[0]), round4(d[1]), round4(d[2])] }).collect();
            l.shapes.push(Shape { name, kind: ShapeKind::Hull { points }, pose: Pose::at(c), color: None, role: ShapeRole::Body });
        }
    }
}

fn round4(x: f64) -> f64 { (x * 1e4).round() / 1e4 }

/// What a joint carries: the mass of its child and everything riding on it, and how far from
/// the joint's axis that reaches at most (half the farthest part, the load's middle when reaching
/// out; for a slide, 0).
fn carried(prop: &Proposal, by_id: &HashMap<&str, &Placed>, j: &JointProp) -> (f64, f64) {
    let mut links = vec![j.child];
    let mut i = 0;
    while i < links.len() {
        let l = links[i];
        for k in prop.joints.iter().filter(|k| k.accepted && k.parent == l) { if !links.contains(&k.child) { links.push(k.child); } }
        i += 1;
    }
    let mut mass = 0.0;
    let mut reach: f64 = 0.0;
    for l in links {
        for pid in &prop.links[l].parts {
            mass += prop.masses.get(pid).map_or(0.0, |m| m.1);
            if let Some(p) = by_id.get(pid.as_str()) {
                if p.hardware { continue; }
                for c in 0..8 {
                    let q = [p.bbox[c & 1][0], p.bbox[(c >> 1) & 1][1], p.bbox[(c >> 2) & 1][2]];
                    let d = sub(q, j.anchor);
                    reach = reach.max(len(sub(d, scale(j.axis, dot(d, j.axis)))));
                }
            }
        }
    }
    // a telescope past the pivot reaches farther than the CAD shows it
    let travel: f64 = prop.joints.iter().filter(|k| k.accepted && k.kind == JointKind::Slide && k.child != j.child && links_has(prop, j.child, k.child)).filter_map(|k| k.range).map(|[a, b]| (b - a).abs()).sum();
    (mass, if j.kind == JointKind::Slide { 0.0 } else { 0.5 * (reach + travel) })
}

/// whether link `l` rides on link `top` (or is it)
fn links_has(prop: &Proposal, top: usize, l: usize) -> bool {
    let mut x = l;
    for _ in 0..32 {
        if x == top { return true; }
        match prop.joints.iter().find(|k| k.accepted && k.child == x) { Some(k) => x = k.parent, None => return false }
    }
    false
}

fn palette(i: usize) -> String { ["#9aa3ad", "#4f8fd6", "#e2a03f", "#3fae6a", "#d65a8a", "#9b6fd0", "#5aa7b3", "#c94f4f"][i % 8].into() }


/// Re-import (D9): carries markup from the old robot onto a new cadpack. Parts are matched by id,
/// then by fingerprint; returns the parts that couldn't be found.
pub fn reattach(r: &mut Robot, new_parts: &[Part]) -> Vec<String> {
    let by_id: HashMap<&str, &Part> = new_parts.iter().map(|p| (p.id.as_str(), p)).collect();
    let mut rename: HashMap<String, Option<String>> = HashMap::new();
    for old in &r.parts {
        if by_id.contains_key(old.id.as_str()) { continue; }
        let f = &old.fingerprint;
        let m = new_parts.iter().filter(|n| !by_id.is_empty() && (n.fingerprint.volume / f.volume.max(1e-12) - 1.0).abs() < 0.01 && len(sub(n.fingerprint.com, f.com)) < 0.005).min_by(|a, b| len(sub(a.fingerprint.com, f.com)).partial_cmp(&len(sub(b.fingerprint.com, f.com))).unwrap());
        rename.insert(old.id.clone(), m.map(|m| m.id.clone()));
    }
    let mut lost = vec![];
    for l in &mut r.links {
        l.parts = l.parts.iter().filter_map(|p| match rename.get(p) { None => Some(p.clone()), Some(Some(n)) => Some(n.clone()), Some(None) => { lost.push(p.clone()); None } }).collect();
    }
    for x in &mut r.rollers {
        if let Some(p) = &x.part { if let Some(n) = rename.get(p) { x.part = n.clone(); } }
    }
    // new parts no link holds ride on the chassis
    let held: std::collections::HashSet<String> = r.links.iter().flat_map(|l| l.parts.iter().cloned()).collect();
    let fresh: Vec<String> = new_parts.iter().filter(|p| !held.contains(&p.id)).map(|p| p.id.clone()).collect();
    if let Some(c) = r.link_mut(CHASSIS) { c.parts.extend(fresh); }
    r.parts = new_parts.to_vec();
    lost.sort(); lost.dedup();
    lost
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cadpack::PartRow;

    #[test]
    fn screw_finds_the_pivot() {
        for (deg, pivot) in [(5.0, [0.3, -0.2, 0.7]), (80.0, [-1.0, 0.4, 0.2]), (170.0, [0.1, 0.1, 0.1])] {
            let q = axis_angle([0.0, 1.0, 0.0], f64::to_radians(deg));
            // a turn about the line through `pivot` along y
            let rel = Pose::new(sub(pivot, qrot(q, pivot)), q);
            let (k, axis, p, amount) = screw(&rel);
            assert_eq!(k, JointKind::Hinge);
            assert!((amount.abs() - f64::to_radians(deg)).abs() < 1e-9 && axis[1].abs() > 0.999);
            assert!((p[0] - pivot[0]).abs() < 1e-9 && (p[2] - pivot[2]).abs() < 1e-9, "{deg}: {p:?} vs {pivot:?}");
        }
    }

    fn row(id: &str, sub: &str) -> PartRow { PartRow { id: id.into(), name: id.into(), path: vec!["Robot".into(), sub.into(), id.into()], shape: 0, m: vec![], color: None } }
    fn placed<'a>(r: &'a PartRow, lo: V3, hi: V3) -> Placed<'a> {
        let d = sub(hi, lo);
        Placed { row: r, pose: Pose::default(), volume: d[0] * d[1] * d[2] * 0.5, com: lerp(lo, hi, 0.5), bbox: [lo, hi], hardware: false }
    }
    fn link(name: &str, parts: &[&str]) -> LinkProp { LinkProp { name: name.into(), parts: parts.iter().map(|s| s.to_string()).collect(), source: String::new() } }
    fn hinge(name: &str, parent: usize, child: usize, anchor: V3, axis: V3, range: Option<[f64; 2]>) -> JointProp {
        JointProp { name: name.into(), kind: JointKind::Hinge, parent, child, anchor, axis, range, source: String::new(), accepted: true }
    }

    #[test]
    fn tidy_tells_steering_rollers_arms_and_turrets_apart() {
        let rows: Vec<PartRow> = ["frame", "m0", "m1", "m2", "m3", "roller", "arm", "turret"].iter().map(|n| row(n, "Sub")).collect();
        let mut p = vec![placed(&rows[0], [-0.35, -0.35, 0.05], [0.35, 0.35, 0.1])];
        let corners = [[0.25, 0.25], [0.25, -0.25], [-0.25, 0.25], [-0.25, -0.25]];
        for (i, c) in corners.iter().enumerate() { p.push(placed(&rows[1 + i], [c[0] - 0.05, c[1] - 0.05, 0.0], [c[0] + 0.05, c[1] + 0.05, 0.1])); }
        p.push(placed(&rows[5], [0.3, -0.3, 0.2], [0.35, 0.3, 0.25]));
        p.push(placed(&rows[6], [0.0, -0.05, 0.3], [0.5, 0.05, 0.35]));
        p.push(placed(&rows[7], [-0.2, -0.2, 0.35], [0.2, 0.2, 0.5]));
        let links: Vec<LinkProp> = ["frame", "m0", "m1", "m2", "m3", "roller", "arm", "turret"].iter().map(|n| link(n, &[n])).collect();
        let mut joints: Vec<JointProp> = corners.iter().enumerate().map(|(i, c)| hinge("steer", 0, 1 + i, [c[0], c[1], 0.05], [0.0, 0.0, 1.0], None)).collect();
        joints.push(hinge("roller", 0, 5, [0.325, 0.0, 0.225], [0.0, 1.0, 0.0], None));
        joints.push(hinge("arm", 0, 6, [0.0, 0.0, 0.325], [0.0, 1.0, 0.0], Some([0.0, 1.5])));
        joints.push(hinge("turret", 0, 7, [0.0, 0.0, 0.35], [0.0, 0.0, 1.0], None));
        let (l, j, _) = tidy(links, joints, &p);
        let names: Vec<&str> = l.iter().map(|l| l.name.as_str()).collect();
        assert_eq!(names, ["frame", "arm", "turret"], "steering and the roller merge; the arm and the turret stay");
        assert_eq!(j.iter().filter(|j| j.name == STEERING).count(), 4);
        assert_eq!(j.iter().filter(|j| j.name != STEERING).count(), 2);
    }

    #[test]
    fn loose_groups_join_what_they_touch() {
        let rows = [row("frame", "Drive"), row("arm", "Arm"), row("bracket", "Arm"), row("far", "Drive")];
        let p = vec![placed(&rows[0], [-0.3, -0.3, 0.0], [0.3, 0.3, 0.1]), placed(&rows[1], [0.0, 0.0, 0.3], [0.5, 0.05, 0.35]),
            placed(&rows[2], [0.45, 0.0, 0.35], [0.5, 0.05, 0.4]), placed(&rows[3], [-0.3, -0.3, 0.1], [-0.2, -0.2, 0.2])];
        let mut links = vec![link("frame", &["frame"]), link("arm", &["arm"]), link("bracket", &["bracket"]), link("far", &["far"])];
        let mut joints = vec![hinge("arm", 0, 1, [0.0, 0.0, 0.3], [0.0, 1.0, 0.0], Some([0.0, 1.0]))];
        assert_eq!(attach_orphans(&mut links, &mut joints, &p), 2);
        assert_eq!(links.len(), 2);
        assert!(links[1].parts.contains(&"bracket".to_string()), "the bracket touching the arm rides on the arm");
        assert!(links[0].parts.contains(&"far".to_string()));
        assert_eq!((joints[0].parent, joints[0].child), (0, 1));
    }

    #[test]
    fn reference_bodies_and_broken_solids_weigh_nothing() {
        let lib = Library::builtin();
        let rows = [row("Clearance Box", "R"), row("USB Camera", "R"), row("Plate", "R")];
        let p = vec![placed(&rows[0], [0.0; 3], [0.5, 0.5, 0.5]), placed(&rows[1], [0.0; 3], [0.8, 0.6, 0.6]), placed(&rows[2], [0.0; 3], [0.3, 0.3, 0.005])];
        let m = masses(&p, &lib, &HashMap::new());
        assert_eq!(m["Clearance Box"].1, 0.0);
        assert_eq!(m["USB Camera"].1, 0.0);
        assert!((m["Plate"].1 - 0.3 * 0.3 * 0.005 * 0.5 * 2700.0).abs() < 1e-9);
    }
}
