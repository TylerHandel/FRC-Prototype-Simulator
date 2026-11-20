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
    let mut occ_to_part: HashMap<Vec<String>, String> = HashMap::new();
    let mut used: HashMap<String, usize> = HashMap::new();
    let mut unmatched = 0;
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
            None => unmatched += 1,
        }
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
    let occ_pose: HashMap<Vec<String>, Pose> = occs.iter().cloned().collect();
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
    if unmatched > 0 { notes.push(format!("{unmatched} parts in the mates weren't found in the STEP by name")); }
    if loose > 0 { notes.push(format!("{loose} parts nobody mated joined the nearest link in their sub-assembly")); }
    Ok((links, joints, notes))
}

/// Links and joints from two poses of the same robot (D20 2): parts that made the same rigid move
/// between them are a link; each link's move against its neighbour is a hinge or a slide. Port of
/// spikes/pose_diff.py, with the parent chosen as the link nearest the joint's axis.
pub fn from_poses(a: &[Placed], b: &[Placed]) -> (Vec<LinkProp>, Vec<JointProp>, Vec<String>) {
    let mut bm: HashMap<String, Vec<Pose>> = HashMap::new();
    for p in b { bm.entry(strip_path(&p.row.path)).or_default().push(p.pose); }
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
    let mut joints = vec![];
    for (ci, g) in big.iter().enumerate().skip(1) {
        // name it after its most common sub-assembly
        let mut c: HashMap<String, usize> = HashMap::new();
        for id in &g.1 { if let Some(p) = a.iter().find(|p| &p.row.id == id) { *c.entry(strip_instance(p.row.path.get(1).map(String::as_str).unwrap_or("?"))).or_default() += 1; } }
        if let Some((n, _)) = c.into_iter().max_by_key(|x| x.1) { links[ci].name = n; }
        let mut best: Option<(f64, usize, JointKind, V3, V3, f64)> = None;
        for (pi, h) in big.iter().enumerate() {
            if pi == ci { continue; }
            let rel = h.0.inverse().mul(&g.0);
            let (kind, axis, point, amount) = screw(&rel);
            if kind == JointKind::Fixed { continue; }
            // the parent's parts nearest the axis point (in the first pose)
            let anchor = if kind == JointKind::Hinge { h.0.inverse().apply(point) } else { [0.0; 3] };
            let near = bbox_of(&h.1).iter().map(|bb| dist_box(anchor, bb)).fold(f64::INFINITY, f64::min);
            let score = if kind == JointKind::Slide { amount * 0.01 } else { near } + if pi == 0 { 0.0 } else { 0.001 };
            if best.is_none_or(|b| score < b.0) { best = Some((score, pi, kind, axis, anchor, amount)); }
        }
        if let Some((_, pi, kind, axis, anchor, amount)) = best {
            let anchor = if kind == JointKind::Slide { bbox_center(&bbox_of(&g.1)) } else { anchor };
            let range = if amount >= 0.0 { [0.0, amount] } else { [amount, 0.0] };
            joints.push(JointProp { name: format!("{} {}", links[ci].name, if kind == JointKind::Slide { "slide" } else { "pivot" }), kind, parent: pi, child: ci, anchor, axis: norm(base.rotate(axis)), range: Some(range), source: "two poses".into(), accepted: true });
        }
    }
    let notes = vec!["Joints seen between the two poses only; a joint that didn't move between them isn't found, and the travel seen is a minimum".into()];
    (links, joints, notes)
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
    // a point on the axis: solve (I - R) p = t across the axis, by a few fixed-point steps
    let t = sub(rel.pos, scale(axis, dot(rel.pos, axis)));
    let mut p = [0.0; 3];
    for _ in 0..200 { let rp = rel.rotate(p); p = add(scale(add(rp, t), 0.5), scale(p, 0.5)); p = sub(p, scale(axis, dot(p, axis))); }
    let mut q = [0.0; 3];
    // the fixed-point above converges to the axis point for turns under about 180 degrees
    for _ in 0..400 { q = add(rel.rotate(q), t); q = sub(q, scale(axis, dot(q, axis))); q = scale(add(q, p), 0.5); }
    (JointKind::Hinge, axis, q, amount)
}

/// Rollers: parts whose name (or BOM row) is a library wheel, confirmed by a cylinder of about the
/// wheel's radius; their axis and width from the geometry. Coaxial rollers on one link share a shaft.
pub fn rollers(cp: &Cadpack, placed: &[Placed], links: &[LinkProp], lib: &Library, bom: &HashMap<String, BomRow>) -> (Vec<RollerProp>, Vec<String>) {
    let link_of: HashMap<&str, usize> = links.iter().enumerate().flat_map(|(i, l)| l.parts.iter().map(move |p| (p.as_str(), i))).collect();
    let mut out: Vec<RollerProp> = vec![];
    let mut notes = vec![];
    for p in placed {
        if p.hardware { continue; }
        let names = [p.row.name.clone(), p.row.path.join(" "), bom.get(&p.row.name).map(|b| format!("{} {}", b.name, b.part_number.clone().unwrap_or_default())).unwrap_or_default()];
        let Some(w) = names.iter().find_map(|n| lib.match_wheel(n)) else { continue };
        let axes = cp.axes(p.row);
        let want = w.radius();
        let Some((ax, r)) = axes.iter().filter_map(|a| a.r.iter().cloned().fold(None, |m: Option<f64>, x| Some(m.map_or(x, |m| m.max(x)))).map(|r| (a, r))).filter(|(_, r)| (r / want - 1.0).abs() < 0.25).max_by(|a, b| a.1.partial_cmp(&b.1).unwrap()) else {
            notes.push(format!("{}: named like {} but no cylinder of that size", p.row.name, w.name));
            continue;
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
pub fn masses(placed: &[Placed], lib: &Library, bom: &HashMap<String, BomRow>) -> HashMap<String, (Option<String>, f64)> {
    placed.iter().map(|p| {
        let row = bom.get(&p.row.name);
        let mat = row.and_then(|b| b.material.as_ref()).and_then(|m| lib.match_material(m)).or_else(|| lib.match_material(&p.row.name)).map(|m| m.id.clone());
        let density = mat.as_ref().and_then(|m| lib.material(m)).map_or(2700.0, |m| m.density);
        let mass = row.and_then(|b| b.mass).unwrap_or(p.volume * density);
        (p.row.id.clone(), (mat, mass))
    }).collect()
}

/// Everything proposed for a cadpack, links from `links` (mates, poses or the tree).
pub fn propose(cp: &Cadpack, to_robot: &Pose, lib: &Library, bom: &HashMap<String, BomRow>, links: Vec<LinkProp>, joints: Vec<JointProp>, mut notes: Vec<String>, from: &str) -> Proposal {
    let placed = place(cp, to_robot);
    let (mut rollers, rn) = rollers(cp, &placed, &links, lib, bom);
    notes.extend(rn);
    let (module, module_positions) = drivetrain(&mut rollers, &placed, lib);
    // a motor is named like one and sized like one (a swerve module's parts often carry the motor's name)
    let motor_sized = |p: &Placed| { let d = sub(p.bbox[1], p.bbox[0]); let (lo, hi) = (d.iter().cloned().fold(f64::INFINITY, f64::min), d.iter().cloned().fold(0.0, f64::max)); p.volume > 3e-5 && lo > 0.025 && hi < 0.2 };
    let motors = placed.iter().filter(|p| !p.hardware && motor_sized(p)).filter_map(|p| lib.match_motor(&p.row.name).or_else(|| bom.get(&p.row.name).and_then(|b| lib.match_motor(&b.name))).map(|m| (p.row.id.clone(), m.id.clone()))).collect();
    Proposal { masses: masses(&placed, lib, bom), links, joints, rollers, motors, module, module_positions, notes, links_from: from.into() }
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
        let shapes = collision_boxes(&lp.parts, &by_id);
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
            r.links.push(Link { id, name: lp.name.clone(), shapes, parts: lp.parts.clone(), mass: Some(mass.max(0.05)), color: palette(i), collision: Collision::Simple, material: None });
        }
    }
    let motor_default = prop.motors.iter().map(|m| m.1.clone()).fold(HashMap::new(), |mut h: HashMap<String, usize>, m| { *h.entry(m).or_default() += 1; h })
        .into_iter().max_by_key(|x| x.1).map(|x| x.0).unwrap_or_else(|| "kraken-x60-foc".into());
    for j in prop.joints.iter().filter(|j| j.accepted) {
        let (Some(&p), Some(&c)) = (ids.get(j.parent), ids.get(j.child)) else { continue };
        if r.joint_of(c).is_some() { continue; }
        let jid = r.id();
        r.joints.push(Joint { id: jid, name: j.name.clone(), kind: j.kind, parent: p, child: c, anchor: j.anchor, axis: j.axis, range: j.range, initial: 0.0, damping: 0.0, friction: 0.0, follows: None, spring: None, source: j.source.clone() });
        let did = r.id();
        let (ratio, spool) = if j.kind == JointKind::Slide { (9.0, 0.0222) } else { (60.0, 0.0) };
        r.drives.push(Drive { id: did, name: format!("{} motor (guessed: check it)", j.name), motor: motor_default.clone(), count: 1, ratio, efficiency: 0.9, stator_limit: 60.0, supply_limit: 60.0,
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
            if !r.controls.bindings.iter().any(|b| b.button == Button::RT) { r.controls.bindings.push(Binding { button: Button::RT, action: Action::Run { drive: d.id, value: 10.0 } }); }
            else if !r.controls.bindings.iter().any(|b| b.button == Button::RB) { r.controls.bindings.push(Binding { button: Button::RB, action: Action::Run { drive: d.id, value: 10.0 } }); }
        }
    }
    for j in r.joints.clone() {
        let Some(d) = r.drives.iter().find(|d| matches!(d.target, DriveTarget::Joint { joint, .. } if joint == j.id)).map(|d| d.id) else { continue };
        let pid = r.id();
        let hi = j.range.map_or(1.0, |x| x[1]);
        r.controls.presets.push(Preset { id: pid, name: format!("{} out", j.name), targets: vec![PresetTarget { drive: d, value: hi }] });
        let pid2 = r.id();
        r.controls.presets.push(Preset { id: pid2, name: format!("{} home", j.name), targets: vec![PresetTarget { drive: d, value: 0.0 }] });
    }
    templates::assign_preset_buttons(r);
}

fn palette(i: usize) -> String { ["#9aa3ad", "#4f8fd6", "#e2a03f", "#3fae6a", "#d65a8a", "#9b6fd0", "#5aa7b3", "#c94f4f"][i % 8].into() }

/// Collision for a link (D7 "Simple"): an oriented box for each of its biggest parts, at most 24.
pub fn collision_boxes(parts: &[String], by_id: &HashMap<&str, &Placed>) -> Vec<Shape> {
    let mut ps: Vec<&&Placed> = parts.iter().filter_map(|p| by_id.get(p.as_str())).filter(|p| !p.hardware && p.volume > 0.0).collect();
    ps.sort_by(|a, b| b.volume.partial_cmp(&a.volume).unwrap());
    ps.into_iter().take(24).filter_map(|p| {
        // the part's own bounding box, turned with it
        let s = &p.row;
        let _ = s;
        let lo = p.bbox[0]; let hi = p.bbox[1];
        let size = sub(hi, lo);
        if size.iter().any(|x| *x < 1e-4) { return None; }
        Some(Shape { name: p.row.name.clone(), kind: ShapeKind::Box { size }, pose: Pose::at(lerp(lo, hi, 0.5)), color: None, role: ShapeRole::Ghost })
    }).collect()
}

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
