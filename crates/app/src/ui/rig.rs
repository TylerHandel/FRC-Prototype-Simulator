//! Rig CAD (M3): bring the robot in from an Onshape link or a STEP file, check what the app
//! guessed, and use it. Three numbered steps, then the CAD drawn link by link: click a part to see
//! where it is, move it to another link, or make a roller or a joint from its round faces.
use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy_egui::egui::{self, RichText, Ui};
use cad::cadpack::{Cadpack, Dir};
use cad::import::{self, Imported, Progress, Sidecar, Source};
use cad::propose::Proposal;
use model::{Id, Pose};
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::mpsc::channel;

use super::widgets::*;
use crate::simrun::Sim;
use crate::state::{AppState, Selection};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Stage { #[default] Choose, Working, Review, Done }

pub enum Msg { Progress(f64, String), Done(Result<Box<Imported>, String>) }

pub struct ImportUi {
    pub stage: Stage,
    pub onshape: bool,
    pub url: String,
    pub access: String,
    pub secret: String,
    pub remember: bool,
    pub second_pose: bool,
    pub step_path: String,
    pub forward: Dir,
    pub up: Dir,
    pub busy: Option<crate::state::Rx<Msg>>,
    pub progress: String,
    pub fraction: f64,
    pub result: Option<Box<Imported>>,
    pub proposal: Option<(Pose, Proposal)>,
    pub error: Option<String>,
    pub hide_blocks: bool,
    pub reimport: bool,
    pub lost: Vec<String>,
}

impl Default for ImportUi {
    fn default() -> Self {
        let (access, secret) = saved_keys();
        ImportUi { stage: Stage::Choose, onshape: true, url: String::new(), remember: !access.is_empty(), access, secret, second_pose: true, step_path: String::new(),
            forward: Dir::PX, up: Dir::PZ, busy: None, progress: String::new(), fraction: 0.0, result: None, proposal: None, error: None, hide_blocks: true, reimport: false, lost: vec![] }
    }
}

fn keys_file() -> PathBuf { crate::settings::app_dir().join("onshape-keys.json") }

/// The user's Onshape API keys: from ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY, or the keys file in
/// their app data folder (never in a project, D8).
fn saved_keys() -> (String, String) {
    if let (Ok(a), Ok(s)) = (std::env::var("ONSHAPE_ACCESS_KEY"), std::env::var("ONSHAPE_SECRET_KEY")) { return (a, s); }
    let v: serde_json::Value = std::fs::read_to_string(keys_file()).ok().and_then(|s| serde_json::from_str(&s).ok()).unwrap_or_default();
    (v["access"].as_str().unwrap_or("").into(), v["secret"].as_str().unwrap_or("").into())
}

/// The CAD reader: the packaged one next to the app, or (developing) Python with OpenCascade.
pub fn sidecar() -> Sidecar {
    let exe_dir = std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.to_path_buf())).unwrap_or_default();
    let packaged = exe_dir.join("cadread").join(if cfg!(windows) { "cadread.exe" } else { "cadread" });
    if packaged.exists() { return Sidecar { program: packaged, args: vec![] }; }
    let script = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tools/cadread/cadread.py");
    let python = std::env::var("FPS_CADREAD_PYTHON").unwrap_or_else(|_| if cfg!(windows) { "python".into() } else { "python3".into() });
    Sidecar { program: python.into(), args: vec!["-I".into(), script.display().to_string()] }
}

/// The CAD's meshes for the 3D view, one per distinct part, and each part's place.
#[derive(Resource, Default)]
pub struct CadMeshes {
    pub rev: u64,
    pub key: String,
    pub meshes: HashMap<usize, Handle<Mesh>>,
    pub parts: HashMap<String, (usize, Pose, Handle<StandardMaterial>)>,
    pub preview: bool,
    pub show_hardware: bool,
    pub hardware: std::collections::HashSet<String>,
}

impl CadMeshes {
    /// The parts drawn on a link: (part id, mesh, material, place in the link's frame).
    pub fn for_link(&self, l: &model::Link, o: [f64; 3]) -> Vec<(String, Handle<Mesh>, Handle<StandardMaterial>, Transform)> {
        let ids: Vec<&String> = if self.preview && l.id == model::CHASSIS { self.parts.keys().collect() } else { l.parts.iter().collect() };
        ids.into_iter().filter(|p| self.show_hardware || !self.hardware.contains(*p)).filter_map(|pid| {
            let (shape, pose, mat) = self.parts.get(pid)?;
            let mesh = self.meshes.get(shape)?.clone();
            let local = Pose::new([pose.pos[0] - o[0], pose.pos[1] - o[1], pose.pos[2] - o[2]], pose.quat);
            Some((pid.clone(), mesh, mat.clone(), crate::view::to_bevy(&local)))
        }).collect()
    }
}

fn load_cad(commands: &mut Commands, cad: Option<ResMut<CadMeshes>>, st: &AppState, meshes: &mut Assets<Mesh>, mats: &mut Assets<StandardMaterial>) {
    // what to show: the import under review, or the project's own CAD
    let (cp, to, key, preview, colors): (Option<&Cadpack>, Pose, String, bool, HashMap<String, String>) = match (&st.import.result, &st.import.proposal) {
        (Some(im), Some((to, prop))) if st.import.stage == Stage::Review => {
            let colors = prop.links.iter().enumerate().flat_map(|(i, l)| { let c = LINK_COLORS[i % LINK_COLORS.len()].to_string(); l.parts.iter().map(move |p| (p.clone(), c.clone())) }).collect();
            (Some(&im.cadpack), *to, format!("review {:?} {:?} {}", st.import.forward, st.import.up, prop.links.len()), true, colors)
        }
        _ => (None, Pose::default(), String::new(), false, HashMap::new()),
    };
    let owned;
    let (cp, to, key, preview) = if cp.is_some() { (cp, to, key, preview) } else {
        match (&st.project().robot.cad, &st.dir) {
            (Some(src), Some(dir)) => {
                let k = format!("{} {}", dir.join(&src.cadpack).display(), src.imported);
                if cad.as_ref().is_some_and(|c| c.key == k) { return; }
                owned = Cadpack::load(&dir.join(&src.cadpack)).ok();
                (owned.as_ref(), src.to_robot, k, false)
            }
            _ => {
                if cad.as_ref().is_some_and(|c| !c.key.is_empty()) { commands.insert_resource(CadMeshes { rev: cad.map_or(1, |c| c.rev + 1), ..default() }); }
                return;
            }
        }
    };
    if cad.as_ref().is_some_and(|c| c.key == key) { return; }
    let Some(cp) = cp else { return };
    let mut out = CadMeshes { rev: cad.as_ref().map_or(1, |c| c.rev + 1), key, preview, ..default() };
    let mut mat_cache: HashMap<String, Handle<StandardMaterial>> = HashMap::new();
    for p in &cp.parts {
        let hw = cp.is_hardware(p);
        if hw { out.hardware.insert(p.id.clone()); continue; }
        if !out.meshes.contains_key(&p.shape) {
            let Some(m) = cp.mesh(p.shape) else { continue };
            let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
            mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, m.positions);
            mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, m.normals);
            mesh.insert_indices(Indices::U32(m.indices));
            out.meshes.insert(p.shape, meshes.add(mesh));
        }
        let color = colors.get(&p.id).cloned().or_else(|| p.color.clone()).unwrap_or_else(|| "#9aa3ad".into());
        let mat = mat_cache.entry(color.clone()).or_insert_with(|| mats.add(StandardMaterial { base_color: crate::view::hex(&color), perceptual_roughness: 0.55, metallic: 0.15, double_sided: true, cull_mode: None, ..default() })).clone();
        out.parts.insert(p.id.clone(), (p.shape, to.mul(&p.pose()), mat));
    }
    commands.insert_resource(out);
}

const LINK_COLORS: [&str; 10] = ["#9aa3ad", "#4f8fd6", "#e2a03f", "#3fae6a", "#d65a8a", "#9b6fd0", "#5aa7b3", "#c94f4f", "#c9c34f", "#6f6fd0"];

/// Checks on the import running in the background, and keeps the CAD's meshes current.
pub fn poll_import(mut commands: Commands, mut st: ResMut<AppState>, cad: Option<ResMut<CadMeshes>>, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    let mut done = None;
    let msgs: Vec<Msg> = st.import.busy.as_ref().and_then(|rx| rx.lock().ok().map(|r| r.try_iter().collect())).unwrap_or_default();
    {
        for m in msgs {
            match m {
                Msg::Progress(f, s) => { st.import.fraction = f; st.import.progress = s; }
                Msg::Done(r) => { done = Some(r); }
            }
        }
    }
    if let Some(r) = done {
        st.import.busy = None;
        match r {
            Ok(im) => {
                let p = import::proposals(&im, &st.lib, st.import.forward, st.import.up);
                st.import.proposal = Some(p);
                st.import.result = Some(im);
                st.import.stage = if st.import.reimport { finish_reimport(&mut st); Stage::Done } else { Stage::Review };
            }
            Err(e) => { st.import.error = Some(e); st.import.stage = Stage::Choose; }
        }
    }
    load_cad(&mut commands, cad, &st, &mut meshes, &mut mats);
}

fn start(st: &mut AppState) {
    let Some(dir) = st.dir.clone() else { st.import.error = Some("Save the project first (start one from Home).".into()); return };
    let src = if st.import.onshape {
        if st.import.access.trim().is_empty() || st.import.secret.trim().is_empty() { st.import.error = Some("Fill in both API keys (see 'How do I get API keys?').".into()); return; }
        if let Err(e) = cad::onshape::parse_url(&st.import.url) { st.import.error = Some(e.to_string()); return; }
        if st.import.remember {
            let _ = std::fs::create_dir_all(crate::settings::app_dir());
            let _ = std::fs::write(keys_file(), serde_json::json!({ "access": st.import.access.trim(), "secret": st.import.secret.trim() }).to_string());
        }
        Source::Onshape { url: st.import.url.trim().into(), access: st.import.access.trim().into(), secret: st.import.secret.trim().into(), second_pose: st.import.second_pose }
    } else {
        let p = PathBuf::from(st.import.step_path.trim().trim_matches('"'));
        if !p.exists() { st.import.error = Some(format!("No file at {}", p.display())); return; }
        Source::Step { path: p }
    };
    let (tx, rx) = channel();
    let budget = crate::settings::app_dir().join("onshape-calls.json");
    let sc = sidecar();
    std::thread::spawn(move || {
        let tx2 = tx.clone();
        let progress = move |p: Progress| { let _ = tx2.send(match p { Progress::Step(s) => Msg::Progress(-1.0, s), Progress::Fraction(f, s) => Msg::Progress(f, s) }); };
        let r = import::run(&src, &dir, &sc, budget, &progress).map(Box::new).map_err(|e| e.to_string());
        let _ = tx.send(Msg::Done(r));
    });
    st.import.busy = Some(std::sync::Mutex::new(rx));
    st.import.error = None;
    st.import.stage = Stage::Working;
    st.import.progress = "Starting".into();
    st.import.fraction = 0.0;
}

fn apply(st: &mut AppState) {
    let (Some(im), Some((to, prop))) = (st.import.result.take(), st.import.proposal.clone()) else { return };
    let lib = st.lib.clone();
    let src = model::CadSource { kind: im.source.clone(), cadpack: "cad/pack".into(), imported: now(), to_robot: to };
    st.doc.edit("Import CAD", |p| {
        cad::propose::apply(&mut p.robot, &im.cadpack, &to, &prop, &lib);
        p.robot.cad = Some(src);
    });
    st.import.stage = Stage::Done;
    st.sel = Selection::Robot;
    st.toast(format!("{} links, {} joints and {} rollers from the CAD", prop.links.len(), prop.joints.iter().filter(|j| j.accepted).count(), prop.rollers.iter().filter(|r| r.accepted).count()));
}

/// Re-import (D9): the new CAD's parts, the old markup carried across.
fn finish_reimport(st: &mut AppState) {
    let (Some(im), Some((to, _))) = (st.import.result.take(), st.import.proposal.clone()) else { return };
    let lib = st.lib.clone();
    let placed = cad::propose::place(&im.cadpack, &to);
    let masses = cad::propose::masses(&placed, &lib, &im.bom);
    let parts: Vec<model::Part> = placed.iter().map(|p| model::Part { id: p.row.id.clone(), name: p.row.name.clone(), path: p.row.path.clone(),
        fingerprint: model::Fingerprint { volume: p.volume, com: p.com, bbox: p.bbox }, material: masses.get(&p.row.id).and_then(|m| m.0.clone()), mass: masses.get(&p.row.id).map(|m| m.1), hardware: p.hardware }).collect();
    let mut lost = vec![];
    st.doc.edit("Re-import CAD", |p| {
        lost = cad::propose::reattach(&mut p.robot, &parts);
        if let Some(c) = &mut p.robot.cad { c.imported = now(); c.to_robot = to; }
    });
    st.toast(if lost.is_empty() { "Re-imported: every marked part was found".to_string() } else { format!("Re-imported: {} parts weren't found (listed in Rig)", lost.len()) });
    st.import.lost = lost;
    st.import.reimport = false;
}

fn now() -> String { format!("{}", std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)) }

pub fn left(ui: &mut Ui, st: &mut AppState, _sim: &Sim, _cad: Option<&mut CadMeshes>) {
    match st.import.stage {
        Stage::Choose | Stage::Working => choose(ui, st),
        Stage::Review => review(ui, st),
        Stage::Done => done(ui, st),
    }
}

fn choose(ui: &mut Ui, st: &mut AppState) {
    heading(ui, "1. Where is your CAD?");
    ui.horizontal(|ui| {
        ui.selectable_value(&mut st.import.onshape, true, "Onshape link");
        ui.selectable_value(&mut st.import.onshape, false, "STEP file");
    });
    let working = st.import.stage == Stage::Working;
    ui.add_enabled_ui(!working, |ui| {
        if st.import.onshape {
            hint(ui, "Open your robot's assembly in Onshape and copy the address from the browser.");
            ui.add(egui::TextEdit::singleline(&mut st.import.url).hint_text("https://cad.onshape.com/documents/.../e/...").desired_width(f32::INFINITY));
            ui.add_space(4.0);
            ui.label(RichText::new("Your Onshape API keys").strong());
            ui.horizontal(|ui| { ui.label("Access key"); ui.add(egui::TextEdit::singleline(&mut st.import.access).desired_width(200.0)); });
            ui.horizontal(|ui| { ui.label("Secret key"); ui.add(egui::TextEdit::singleline(&mut st.import.secret).password(true).desired_width(200.0)); });
            ui.checkbox(&mut st.import.remember, "Remember them on this computer").on_hover_text("Kept in your user's app data folder, never in a project file");
            egui::CollapsingHeader::new("How do I get API keys?").show(ui, |ui| {
                hint(ui, "1. Go to dev-portal.onshape.com and sign in with your Onshape account.");
                hint(ui, "2. Choose API keys, then Create new API key.");
                hint(ui, "3. Tick 'Read' (that's all this app needs) and create it.");
                hint(ui, "4. Copy the access key and the secret key here. The secret is shown only once.");
                hint(ui, "Each teammate uses their own keys. An import uses about 8 of your account's yearly API calls.");
            });
            ui.checkbox(&mut st.import.second_pose, "If Onshape won't share the mates, compare with another assembly in the document").on_hover_text("Another team's export-only document hides its mates. If it has a second assembly with the robot in another pose (say, 'Scoring Positions'), comparing the two finds the joints. About 3 more API calls.");
        } else {
            hint(ui, "Export the assembly from your CAD as STEP (in Onshape: right-click the assembly tab, Export, STEP). Then drop the file on this window, or paste its path.");
            ui.horizontal(|ui| {
                ui.add(egui::TextEdit::singleline(&mut st.import.step_path).hint_text("C:\\Users\\you\\Downloads\\robot.step").desired_width(220.0));
                #[cfg(any(windows, target_os = "macos"))]
                if ui.button("Browse...").clicked() {
                    if let Some(f) = rfd::FileDialog::new().add_filter("STEP", &["step", "stp", "STEP", "STP"]).pick_file() { st.import.step_path = f.display().to_string(); }
                }
            });
        }
        ui.add_space(6.0);
        let label = if st.import.reimport { "Re-import" } else { "Import" };
        if ui.add_sized([140.0, 30.0], egui::Button::new(RichText::new(label).strong())).clicked() { start(st); }
    });
    if let Some(e) = &st.import.error { ui.colored_label(BAD, e); }
    if working {
        heading(ui, "2. Bringing it in");
        if st.import.fraction >= 0.0 { ui.add(egui::ProgressBar::new(st.import.fraction as f32).show_percentage()); } else { ui.spinner(); }
        ui.label(&st.import.progress);
        hint(ui, "A whole robot takes a few minutes: Onshape makes the STEP file, then the app reads every part.");
    }
    // a STEP file dropped on the window
    let dropped: Vec<PathBuf> = ui.ctx().input(|i| i.raw.dropped_files.iter().map(|f| f.path().to_path_buf()).collect());
    if let Some(p) = dropped.into_iter().find(|p| p.extension().is_some_and(|e| e.eq_ignore_ascii_case("step") || e.eq_ignore_ascii_case("stp"))) {
        st.import.onshape = false;
        st.import.step_path = p.display().to_string();
    }
    if st.project().robot.cad.is_some() {
        ui.separator();
        if ui.button("Back to the rigged robot").clicked() { st.import.stage = Stage::Done; }
    }
}

fn review(ui: &mut Ui, st: &mut AppState) {
    heading(ui, "3. Check what it found");
    let Some((_, prop)) = st.import.proposal.clone() else { return };
    hint(ui, &format!("Links from {}. Each colour in the view is one link: parts that move together.", prop.links_from));
    let (mut f, mut u) = (st.import.forward, st.import.up);
    ui.horizontal(|ui| {
        ui.label("Forward is the CAD's");
        egui::ComboBox::from_id_salt("fwd").selected_text(f.label()).width(50.0).show_ui(ui, |ui| { for d in Dir::ALL { ui.selectable_value(&mut f, d, d.label()); } });
        ui.label("and up is");
        egui::ComboBox::from_id_salt("up").selected_text(u.label()).width(50.0).show_ui(ui, |ui| { for d in Dir::ALL { ui.selectable_value(&mut u, d, d.label()); } });
    });
    hint(ui, "Turn it until the robot stands on the floor with its intake (front) facing away from you.");
    if (f, u) != (st.import.forward, st.import.up) && f.v() != u.v() && f.v() != u.v().map(|x| -x) {
        st.import.forward = f; st.import.up = u;
        if let Some(im) = &st.import.result { st.import.proposal = Some(import::proposals(im, &st.lib, f, u)); }
    }
    let mut prop = prop;
    let mut changed = false;
    egui::CollapsingHeader::new(format!("Links ({})", prop.links.len())).default_open(true).show(ui, |ui| {
        for (i, l) in prop.links.iter_mut().enumerate() {
            ui.horizontal(|ui| {
                ui.colored_label(color32(LINK_COLORS[i % LINK_COLORS.len()]), "■");
                changed |= ui.text_edit_singleline(&mut l.name).changed();
                ui.label(RichText::new(format!("{} parts", l.parts.len())).color(MUTED));
            });
        }
    });
    egui::CollapsingHeader::new(format!("Joints ({})", prop.joints.len())).default_open(true).show(ui, |ui| {
        if prop.joints.is_empty() { hint(ui, "None found. After 'Use this', select a part's round face to make a joint, or set links by hand."); }
        for j in prop.joints.iter_mut() {
            ui.horizontal(|ui| {
                changed |= ui.checkbox(&mut j.accepted, "").changed();
                ui.label(format!("{} ({})", j.name, if j.kind == model::JointKind::Slide { "slide" } else { "hinge" })).on_hover_text(&j.source);
            });
        }
    });
    egui::CollapsingHeader::new(format!("Rollers ({})", prop.rollers.len())).default_open(true).show(ui, |ui| {
        if prop.rollers.is_empty() { hint(ui, "No wheels or rollers recognised by name. You can mark them after 'Use this'."); }
        let lib = st.lib.clone();
        for (i, r) in prop.rollers.iter_mut().enumerate() {
            ui.horizontal(|ui| {
                changed |= ui.checkbox(&mut r.accepted, "").changed();
                changed |= wheel_pick(ui, &format!("rp{i}"), &lib, &mut r.wheel, r.diameter / library::IN);
            });
        }
    });
    if let Some(m) = &prop.module { ui.label(format!("Swerve modules: {}", st.lib.module(m).map_or(m.clone(), |x| x.name.clone()))); }
    if !prop.module_positions.is_empty() { ui.label(format!("{} drive wheels found: the drivetrain uses their places", prop.module_positions.len())); }
    if !prop.motors.is_empty() { ui.label(format!("{} motors found ({})", prop.motors.len(), prop.motors.first().and_then(|m| st.lib.motor(&m.1)).map_or(String::new(), |m| m.name.clone()))); }
    for n in &prop.notes { hint(ui, n); }
    if changed { if let Some(p) = &mut st.import.proposal { p.1 = prop; } }
    ui.add_space(6.0);
    ui.horizontal(|ui| {
        if ui.add_sized([140.0, 30.0], egui::Button::new(RichText::new("Use this").strong())).clicked() { apply(st); }
        if ui.button("Start over").clicked() { st.import.stage = Stage::Choose; st.import.result = None; st.import.proposal = None; }
    });
    hint(ui, "Motors and gear ratios are guessed: check them after (they say 'guessed').");
}

fn done(ui: &mut Ui, st: &mut AppState) {
    heading(ui, "Your CAD robot");
    let r = st.project().robot.clone();
    if r.cad.is_none() { st.import.stage = Stage::Choose; return; }
    hint(ui, "Click a part in the view to see its link. Joints and rollers are on the right when selected.");
    ui.checkbox(&mut st.import.hide_blocks, "Hide the collision boxes");
    for l in &r.links {
        let sel = Selection::Link(l.id);
        let j = r.joint_of(l.id).map(|j| format!(" (on {})", j.name)).unwrap_or_default();
        if ui.selectable_label(st.sel == sel, format!("{}: {} parts{}", l.name, l.parts.len(), j)).clicked() { st.sel = sel; }
    }
    heading(ui, "Joints");
    for j in &r.joints {
        let sel = Selection::Joint(j.id);
        if ui.selectable_label(st.sel == sel, format!("{} ({:?})", j.name, j.kind)).clicked() { st.sel = sel; }
    }
    heading(ui, "Motors (check these)");
    for d in &r.drives {
        let sel = Selection::Drive(d.id);
        let c = if d.name.contains("guessed") { WARN } else { MUTED };
        if ui.selectable_label(st.sel == sel, RichText::new(&d.name).color(c)).clicked() { st.sel = sel; }
    }
    if !st.import.lost.is_empty() {
        heading(ui, "Not found after re-import");
        for p in st.import.lost.clone() { ui.label(RichText::new(p).color(WARN).size(11.5)); }
    }
    ui.add_space(8.0);
    ui.horizontal(|ui| {
        if ui.button("Re-import").on_hover_text("Bring the CAD in again after it changed: marked parts keep their links, joints and rollers").clicked() {
            if let Some(src) = &r.cad {
                match &src.kind {
                    model::CadKind::Onshape { url, .. } => { st.import.onshape = true; st.import.url = url.clone(); }
                    model::CadKind::Step { path } => { st.import.onshape = false; st.import.step_path = path.clone(); }
                }
            }
            st.import.reimport = true;
            st.import.stage = Stage::Choose;
        }
        if ui.button("Import different CAD").clicked() { st.import.reimport = false; st.import.stage = Stage::Choose; }
    });
}

/// A part was clicked in the view.
pub fn clicked_part(st: &mut AppState, id: &str, _hit: Option<[f64; 3]>) {
    if st.import.stage == Stage::Review { return; }
    st.sel = Selection::Part(id.to_string());
}

/// The inspector for a CAD part: where it is, and what to make of it.
pub fn part(ui: &mut Ui, st: &mut AppState, id: &str) {
    let r = st.project().robot.clone();
    let Some(p) = r.parts.iter().find(|p| p.id == id) else { ui.label(id); return };
    heading(ui, &p.name);
    hint(ui, &p.path.join(" / "));
    let mass = st.unit(library::units::Kind::Mass);
    ui.label(format!("{} ({}){}", library::units::format(p.mass.unwrap_or(0.0), mass), p.material.as_deref().and_then(|m| st.lib.material(m)).map_or("aluminium assumed", |m| m.name.as_str()), if p.hardware { ", hardware" } else { "" }));
    let link = r.links.iter().find(|l| l.parts.iter().any(|x| x == id));
    if let Some(l) = link {
        if ui.selectable_label(false, format!("In link: {}", l.name)).clicked() { st.sel = Selection::Link(l.id); return; }
    }
    // move it (and everything in its sub-assembly) to another link
    let mut to: Option<Id> = None;
    let mut whole = true;
    ui.checkbox(&mut whole, "with the rest of its sub-assembly");
    egui::ComboBox::from_id_salt("move_part").selected_text("Move to link...").show_ui(ui, |ui| {
        for l in &r.links { if ui.selectable_label(false, &l.name).clicked() { to = Some(l.id); } }
        if ui.selectable_label(false, "a new link").clicked() { to = Some(0); }
    });
    if let Some(target) = to {
        let group: Vec<String> = if whole && p.path.len() > 2 { let pre = &p.path[..p.path.len() - 1]; r.parts.iter().filter(|x| x.path.starts_with(pre)).map(|x| x.id.clone()).collect() } else { vec![id.to_string()] };
        let name = p.path.get(p.path.len().saturating_sub(2)).cloned().unwrap_or_else(|| p.name.clone());
        let cad = load_cadpack(st);
        st.doc.edit("Move parts", |pr| {
            let target = if target == 0 { let nid = pr.robot.id(); pr.robot.links.push(model::Link { id: nid, name, shapes: vec![], parts: vec![], mass: None, color: "#4f8fd6".into(), collision: model::Collision::Box, material: None }); nid } else { target };
            for l in &mut pr.robot.links { l.parts.retain(|x| !group.contains(x)); }
            if let Some(l) = pr.robot.link_mut(target) { l.parts.extend(group.iter().cloned()); }
            refit(&mut pr.robot);
            if let Some((cp, to)) = &cad { cad::propose::fit_collision(&mut pr.robot, cp, to); }
        });
    }
    // round faces: a roller or a joint axis
    let (cp, to_robot) = match (&r.cad, &st.dir) { (Some(c), Some(d)) => (Cadpack::load(&d.join(&c.cadpack)).ok(), c.to_robot), _ => (None, Pose::default()) };
    let Some(cp) = cp else { return };
    let Some(row) = cp.parts.iter().find(|x| x.id == id) else { return };
    let pose = to_robot.mul(&row.pose());
    let axes = cp.shapes[row.shape].axes.clone();
    if !axes.is_empty() { heading(ui, "Its round faces"); }
    let len = st.unit(library::units::Kind::Length);
    for (i, a) in axes.iter().enumerate().take(4) {
        let r_max = a.r.iter().cloned().fold(0.0, f64::max);
        let axis = model::math::norm(pose.rotate(a.d));
        let pt = pose.apply(a.p);
        ui.horizontal(|ui| {
            ui.label(format!("{} across", library::units::format(2.0 * r_max, len)));
            if ui.small_button("Roller here").on_hover_text("A roller on this axis, this size, on the part's link").clicked() {
                let link_id = link.map_or(model::CHASSIS, |l| l.id);
                let wheel = st.lib.match_wheel(&p.name).map(|w| w.id.clone()).unwrap_or_else(|| st.lib.wheels_like(2.0 * r_max / library::IN).first().map(|w| w.id.clone()).unwrap_or_default());
                let center = model::math::add(pt, model::math::scale(axis, model::math::dot(model::math::sub(p.fingerprint.com, pt), axis)));
                let width = (0..3).map(|k| (p.fingerprint.bbox[1][k] - p.fingerprint.bbox[0][k]) * axis[k].abs()).sum::<f64>().max(0.01);
                let pid = id.to_string();
                st.doc.edit("Roller from a part", |pr| {
                    let r = &mut pr.robot;
                    let sid = match r.shafts.iter().find(|s| s.link == link_id) { Some(s) => s.id, None => {
                        let sid = r.id();
                        r.shafts.push(model::Shaft { id: sid, name: "Rollers".into(), link: link_id, inertia: 2e-4, drag: 2e-4 });
                        let did = r.id();
                        r.drives.push(model::Drive { id: did, name: "Rollers (guessed: check it)".into(), motor: "kraken-x60-foc".into(), count: 1, ratio: 3.0, efficiency: 0.9, stator_limit: 40.0, supply_limit: 40.0, target: model::DriveTarget::Shaft { shaft: sid }, control: model::Control::Voltage, brake: true });
                        sid
                    } };
                    let rid = r.id();
                    r.rollers.push(model::Roller { id: rid, name: format!("Roller {i}"), shaft: sid, ratio: 1.0, center, axis, width, wheel, diameter: Some(2.0 * r_max), kind: model::RollerKind::Plain, part: Some(pid) });
                });
            }
            if let Some(l) = link {
                if l.id != model::CHASSIS && ui.small_button("Joint axis here").on_hover_text("This link turns about this axis on its parent").clicked() {
                    let lid = l.id;
                    let parent = r.joint_of(lid).map_or(model::CHASSIS, |j| j.parent);
                    st.doc.edit("Joint from a part", |pr| {
                        if let Some(j) = pr.robot.joints.iter_mut().find(|j| j.child == lid) { j.anchor = pt; j.axis = axis; j.kind = model::JointKind::Hinge; j.source = "clicked".into(); return; }
                        let jid = pr.robot.id();
                        pr.robot.joints.push(model::Joint { id: jid, name: format!("{} pivot", pr.robot.link(lid).map_or("?".into(), |l| l.name.clone())), kind: model::JointKind::Hinge, parent, child: lid, anchor: pt, axis, range: Some([-1.57, 1.57]), initial: 0.0, damping: 0.0, friction: 0.0, follows: None, spring: None, source: "clicked".into() });
                        let did = pr.robot.id();
                        pr.robot.drives.push(model::Drive { id: did, name: "Pivot motor (guessed: check it)".into(), motor: "kraken-x60-foc".into(), count: 1, ratio: 60.0, efficiency: 0.9, stator_limit: 60.0, supply_limit: 60.0, target: model::DriveTarget::Joint { joint: jid, spool_radius: 0.0 }, control: model::Control::Ideal, brake: true });
                    });
                }
            }
        });
    }
}

/// After parts move between links: masses again (collision is refitted from the meshes).
fn refit(r: &mut model::Robot) {
    let parts: HashMap<String, model::Part> = r.parts.iter().map(|p| (p.id.clone(), p.clone())).collect();
    for l in &mut r.links {
        if l.parts.is_empty() { continue; }
        let m: f64 = l.parts.iter().filter_map(|p| parts.get(p).and_then(|x| x.mass)).sum();
        l.mass = Some(m.max(0.05) + if l.id == model::CHASSIS { 6.0 } else { 0.0 });
    }
}

/// The project's cadpack and its placement, for refitting collision after an edit.
pub fn load_cadpack(st: &AppState) -> Option<(Cadpack, Pose)> {
    let (c, d) = (st.project().robot.cad.as_ref()?, st.dir.as_ref()?);
    Some((Cadpack::load(&d.join(&c.cadpack)).ok()?, c.to_robot))
}
