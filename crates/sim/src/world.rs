//! A running simulation: a project's robot compiled into a MuJoCo model (D3), on the Bench or the
//! field, stepped with its motors, battery, controllers, roller contact and concept pickups.
//!
//! Frames: the robot's links are bodies whose frames line up with the robot frame at the design
//! pose, each placed at its joint's anchor (the chassis at the robot's origin). So a roller or a
//! shape's place in its link's body frame is just its design position minus the body's origin.
use crate::contact::{self, Bristles, RollerState, RollerSurface, SliceContact};
use crate::control;
use crate::electrical::Bus;
use crate::engine::{Engine, Obj};
use crate::game::{GamePack, Match, StaticShape};
use crate::motor::Motor;
use crate::piece::PieceKind;
use crate::swerve;
use library::Library;
use model::math::*;
use model::*;
use std::collections::HashMap;
use std::fmt::Write as _;

const G: f64 = 9.81;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// the robot held still, pieces sent at it (M1)
    Bench,
    /// the robot free on the game's field
    Field,
    /// the robot free on bare carpet (Build)
    Carpet,
}

/// What to build besides the robot.
#[derive(Debug, Clone)]
pub struct Scene {
    pub mode: Mode,
    /// x, y, yaw (field or carpet)
    pub start: [f64; 3],
    /// put this wheel on every roller (Bench comparisons, D19)
    pub wheel_override: Option<String>,
    /// pieces besides the game's
    pub pieces: Vec<Spawn>,
    /// the game's own pieces and feeders (Field)
    pub game_pieces: bool,
}

impl Scene {
    pub fn bench() -> Self { Scene { mode: Mode::Bench, start: [0.0; 3], wheel_override: None, pieces: vec![], game_pieces: false } }
    pub fn carpet() -> Self { Scene { mode: Mode::Carpet, start: [0.0; 3], wheel_override: None, pieces: vec![], game_pieces: false } }
    pub fn field(game: &GamePack) -> Self {
        Scene { mode: Mode::Field, start: [game.start.pos[0], game.start.pos[1], game.start.yaw_deg.to_radians()], wheel_override: None, pieces: vec![], game_pieces: true }
    }
}

/// A piece to put in the world: where, which way, how fast.
#[derive(Debug, Clone)]
pub struct Spawn { pub piece: String, pub pos: V3, pub quat: Quat, pub lin: V3, pub ang: V3 }

/// What the driver and the operator are asking for.
pub use crate::piece::{BIT_CARPET, BIT_FIELD, BIT_HOLLOW, BIT_PIECE, BIT_ROBOT, BIT_SOFT, BIT_WHEEL};

/// a piece this close to a seat, nearly still for this long, is scored where it lies
const SETTLE_REACH: f64 = 0.12;
const SETTLE_TIME: f64 = 0.3;

#[derive(Debug, Clone, Default)]
pub struct Commands {
    /// x, y (m/s) and turn (rad/s); field-relative when `field_relative`
    pub drive: [f64; 3],
    pub field_relative: bool,
    /// drive id -> volts (rollers) or speed (velocity control), while held
    pub run: HashMap<Id, f64>,
    /// drive id -> position target (joints)
    pub targets: HashMap<Id, f64>,
    /// let go of what these mechanisms hold, this step
    pub release: Vec<Id>,
    /// drive id -> the sensor (by name) that stops its run
    pub until: HashMap<Id, String>,
}

#[derive(Debug, Clone)]
struct LinkRt { id: Id, body: usize, origin: V3 }

#[derive(Debug, Clone)]
struct JointRt { id: Id, jnt: usize, kind: JointKind, range: Option<[f64; 2]>, followers: Vec<(usize, f64)>, spring: Option<Spring> }

#[derive(Debug, Clone)]
enum Target { Joint { joint: usize, act: usize }, Shaft { shaft: usize } }

#[derive(Debug, Clone)]
struct DriveRt { id: Id, name: String, motor: Motor, target: Target, control: Control, state: control::State, out: crate::motor::MotorOut, inertia: f64, brake: bool }

#[derive(Debug, Clone)]
struct ShaftRt { id: Id, omega: f64, angle: f64, inertia: f64, drag: f64, torque: f64 }

#[derive(Debug, Clone)]
struct RollerRt { id: Id, shaft: usize, ratio: f64, link: usize, center: V3, axis: V3, surface: RollerSurface, full: bool, wheel: String, radius: f64 }

#[derive(Debug, Clone)]
struct HolderRt { link: usize, pose: Pose, release: Release, piece: Option<usize> }

#[derive(Debug, Clone)]
struct MechRt { id: Id, full: bool, pickup: Option<(usize, Pose, V3, Option<Id>, Vec<String>, f64)>, holders: Vec<HolderRt>, dwell: HashMap<usize, f64> }

#[derive(Debug, Clone)]
enum SensorKind { Beam { link: usize, from: V3, to: V3 }, Limit { jnt: usize, at: f64, band: f64 } }

#[derive(Debug, Clone)]
struct SensorRt { name: String, kind: SensorKind, on: bool }

/// A sensor's reading, and where a beam break's beam is (for drawing).
#[derive(Debug, Clone, serde::Serialize)]
pub struct SensorSnap { pub name: String, pub on: bool, pub beam: Option<[V3; 2]> }

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PieceWhere { Free, Held { mech: Id }, Scored, Parked }

#[derive(Debug, Clone)]
struct PieceRt { kind: usize, jnt: usize, body: usize, at: PieceWhere, shown: Pose, park: V3, soft_geom: Option<usize> }

#[derive(Debug, Clone)]
struct FeederRt { last: f64 }

/// A link's pose for drawing.
#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct LinkSnap { pub id: Id, pub pose: Pose }

#[derive(Debug, Clone, serde::Serialize)]
pub struct PieceSnap { pub kind: String, pub pose: Pose, pub held: bool, pub scored: bool }

#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct DriveSnap { pub id: Id, pub volts: f64, pub current: f64, pub supply: f64, pub position: f64, pub speed: f64, pub target: f64, pub limited: bool }

#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct ContactSnap { pub point: V3, pub normal: V3, pub force: f64, pub sliding: bool, pub roller: Id }

#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct RollerSnap { pub id: Id, pub center: V3, pub axis: V3, pub angle: f64, pub speed: f64 }

/// Everything a view needs to draw and plot one moment (D15).
#[derive(Debug, Clone, serde::Serialize)]
pub struct Snapshot {
    pub time: f64,
    pub links: Vec<LinkSnap>,
    pub pieces: Vec<PieceSnap>,
    pub rollers: Vec<RollerSnap>,
    pub drives: Vec<DriveSnap>,
    pub contacts: Vec<ContactSnap>,
    pub bus_voltage: f64,
    pub bus_current: f64,
    pub energy: f64,
    pub brownout: bool,
    pub breaker_heat: f64,
    pub com: V3,
    pub support: Vec<[f64; 2]>,
    /// how far the centre of mass is inside the wheels' footprint, m (negative: tipping)
    pub tip_margin: f64,
    pub speed: f64,
    pub score: f64,
    pub match_time: f64,
    pub phase: String,
    pub sensors: Vec<SensorSnap>,
}

pub struct World {
    pub engine: Engine,
    pub lib: Library,
    pub project: Project,
    pub scene: Scene,
    pub game: Option<GamePack>,
    pub dt: f64,
    pub time: f64,
    pub cmd: Commands,
    pub bus: Bus,
    pub warnings: Vec<String>,
    pub matched: Match,
    /// the model as MuJoCo got it (for debugging)
    pub mjcf: String,
    links: Vec<LinkRt>,
    joints: Vec<JointRt>,
    drives: Vec<DriveRt>,
    shafts: Vec<ShaftRt>,
    rollers: Vec<RollerRt>,
    mechs: Vec<MechRt>,
    kinds: Vec<PieceKind>,
    pieces: Vec<PieceRt>,
    feeders: Vec<FeederRt>,
    drive: Option<swerve::Drive>,
    root_body: usize,
    root_joint: Option<usize>,
    bristles: Bristles,
    contacts: Vec<SliceContact>,
    contact_rollers: Vec<Id>,
    robot_bodies: Vec<usize>,
    /// (geom, link) for the robot's solid shapes
    robot_geoms: Vec<(usize, usize)>,
    sensors: Vec<SensorRt>,
    /// how long each free piece has sat still by a seat
    settle: HashMap<usize, f64>,
}

fn f(v: f64) -> String { let s = format!("{v:.6}"); let s = s.trim_end_matches('0').trim_end_matches('.').to_string(); if s == "-0" || s.is_empty() { "0".into() } else { s } }
fn v3(v: V3) -> String { format!("{} {} {}", f(v[0]), f(v[1]), f(v[2])) }
fn q4(q: Quat) -> String { format!("{} {} {} {}", f(q[0]), f(q[1]), f(q[2]), f(q[3])) }
fn rgba(hex: &str, a: f64) -> String {
    let h = hex.trim_start_matches('#');
    let c = |i: usize| u8::from_str_radix(h.get(i..i + 2).unwrap_or("80"), 16).unwrap_or(128) as f64 / 255.0;
    format!("{} {} {} {}", f(c(0)), f(c(2)), f(c(4)), f(a))
}

/// mass properties of a link from its blocks (uniform density), scaled to `mass`: (com, inertia 3x3 about com)
fn link_inertia(shapes: &[Shape], mass: f64, fallback_com: V3) -> (V3, [f64; 9]) {
    let solid: Vec<&Shape> = shapes.iter().filter(|s| s.role != ShapeRole::Ghost).collect();
    let vol: f64 = solid.iter().map(|s| s.kind.volume()).sum();
    if solid.is_empty() || vol <= 0.0 {
        let i = 0.01 * mass;
        return (fallback_com, [i, 0.0, 0.0, 0.0, i, 0.0, 0.0, 0.0, i]);
    }
    let mut com = [0.0; 3];
    for s in &solid { com = add(com, scale(s.pose.pos, s.kind.volume() / vol)); }
    let mut it = [0.0; 9];
    for s in &solid {
        let m = mass * s.kind.volume() / vol;
        let d = match &s.kind {
            ShapeKind::Cylinder { radius, length } => { let a = m / 12.0 * (3.0 * radius * radius + length * length); [a, a, m / 2.0 * radius * radius] }
            ShapeKind::Sphere { radius } => { let a = 0.4 * m * radius * radius; [a, a, a] }
            // a box, or a hull taken as the box round it
            k => { let size = k.extents(); [m / 12.0 * (size[1].powi(2) + size[2].powi(2)), m / 12.0 * (size[0].powi(2) + size[2].powi(2)), m / 12.0 * (size[0].powi(2) + size[1].powi(2))] }
        };
        let r = qmat(s.pose.quat);
        // R diag(d) R^T, then the parallel-axis term
        for i in 0..3 { for j in 0..3 {
            let mut v = 0.0;
            for k in 0..3 { v += r[i * 3 + k] * d[k] * r[j * 3 + k]; }
            it[i * 3 + j] += v;
        } }
        let o = sub(s.pose.pos, com);
        let o2 = dot(o, o);
        for i in 0..3 { for j in 0..3 { it[i * 3 + j] += m * ((if i == j { o2 } else { 0.0 }) - o[i] * o[j]); } }
    }
    (com, it)
}

impl World {
    pub fn new(project: &Project, lib: &Library, scene: Scene) -> anyhow::Result<World> {
        let game = if scene.mode == Mode::Field { GamePack::builtin(&project.game) } else { None };
        let r = &project.robot;
        let issues = model::validate::issues(r, lib);
        if let Some(e) = issues.iter().find(|i| i.severity == model::validate::Severity::Error) {
            anyhow::bail!("{}", e.message);
        }
        let full_any = r.mechanisms.iter().any(|m| project.fidelity(m) == Fidelity::Full);
        let dt = match (scene.mode, full_any) {
            (Mode::Bench, _) => 0.0005,
            (_, true) => 0.001,
            (_, false) => 0.002,
        };
        let mut warnings = Vec::new();

        // ---- the MJCF ----
        let mut x = String::new();
        let sleep = if scene.mode == Mode::Field { "<flag sleep=\"enable\"/>" } else { "" };
        let _ = write!(x, "<mujoco model=\"robot\"><compiler angle=\"radian\" inertiafromgeom=\"auto\"/>\
            <option timestep=\"{dt}\" integrator=\"implicitfast\" cone=\"elliptic\" impratio=\"10\" gravity=\"0 0 -{G}\">{sleep}</option>\
            <size memory=\"256M\"/><visual><global offwidth=\"64\" offheight=\"64\"/></visual>");
        // CAD hulls are meshes: MuJoCo takes the convex hull of each point set
        let mut assets = String::new();
        for l in &r.links {
            for (si, s) in l.shapes.iter().enumerate() {
                let ShapeKind::Hull { points } = &s.kind else { continue };
                let v: Vec<String> = points.iter().map(|p| v3(*p)).collect();
                let _ = write!(assets, "<mesh name=\"h{}_{si}\" vertex=\"{}\"/>", l.id, v.join(" "));
            }
        }
        if !assets.is_empty() { let _ = write!(x, "<asset>{assets}</asset>"); }
        x.push_str("<worldbody>");
        let carpet_mu = game.as_ref().map_or(1.0, |g| g.field.carpet_friction);
        let _ = write!(x, "<geom name=\"carpet\" type=\"plane\" size=\"40 40 0.1\" friction=\"{carpet_mu} 0.005 0.0001\" contype=\"{BIT_CARPET}\" conaffinity=\"0\" rgba=\"0.2 0.2 0.22 1\"/>");
        if let Some(g) = &game {
            for (i, s) in g.statics.iter().enumerate() {
                if s.ghost { continue; }
                let q = euler_to_quat(s.euler_deg.map(f64::to_radians));
                let (t, size) = match &s.shape {
                    StaticShape::Box { size } => ("box", v3(scale(*size, 0.5))),
                    StaticShape::Cylinder { radius, length } => ("cylinder", format!("{} {}", f(*radius), f(length / 2.0))),
                };
                let _ = write!(x, "<geom name=\"s{i}\" type=\"{t}\" size=\"{size}\" pos=\"{}\" quat=\"{}\" friction=\"0.5 0.005 0.0001\" contype=\"{}\" conaffinity=\"0\" rgba=\"{}\"/>", v3(s.pos), q4(q), BIT_FIELD, rgba(&s.color, 1.0));
            }
        }

        // the robot: bodies nested along the joint tree
        let order = r.link_order();
        let origin_of = |id: Id| -> V3 { if id == CHASSIS { [0.0; 3] } else { r.joint_of(id).map_or([0.0; 3], |j| j.anchor) } };
        let mut children: HashMap<Id, Vec<Id>> = HashMap::new();
        for &l in order.iter().skip(1) {
            let p = r.parent_of(l).unwrap_or(CHASSIS);
            children.entry(p).or_default().push(l);
        }
        // drives on joints: armature (the rotors) per joint
        let mut armature: HashMap<Id, f64> = HashMap::new();
        let mut motors: HashMap<Id, Motor> = HashMap::new();
        for d in &r.drives {
            let Some(m) = lib.motor(&d.motor) else { continue };
            let g = match d.target {
                DriveTarget::Joint { joint, spool_radius } => {
                    let j = r.joint(joint).unwrap();
                    if j.kind == JointKind::Slide { d.ratio / spool_radius.max(1e-4) } else { d.ratio }
                }
                DriveTarget::Shaft { .. } => d.ratio,
            };
            let mo = Motor::new(m.constants(), d.count as f64, g, d.efficiency, d.stator_limit, d.supply_limit);
            if let DriveTarget::Joint { joint, .. } = d.target { *armature.entry(joint).or_default() += mo.reflected_inertia(); }
            motors.insert(d.id, mo);
        }
        let weighed = r.weighed_mass.map(|w| w / r.mass().max(1e-6)).unwrap_or(1.0);
        fn body(x: &mut String, r: &Robot, id: Id, parent_origin: V3, children: &HashMap<Id, Vec<Id>>, armature: &HashMap<Id, f64>, origin_of: &dyn Fn(Id) -> V3, scene: &Scene, weighed: f64) {
            let l = r.link(id).unwrap();
            let o = origin_of(id);
            let rel = sub(o, parent_origin);
            if id == CHASSIS {
                let q = axis_angle([0.0, 0.0, 1.0], scene.start[2]);
                let _ = write!(x, "<body name=\"l{id}\" pos=\"{} {} 0\" quat=\"{}\">", f(scene.start[0]), f(scene.start[1]), q4(q));
                if scene.mode != Mode::Bench { let _ = write!(x, "<freejoint name=\"root\"/>"); }
            } else {
                let _ = write!(x, "<body name=\"l{id}\" pos=\"{}\">", v3(rel));
                if let Some(j) = r.joint_of(id) {
                    let t = match j.kind { JointKind::Hinge => Some("hinge"), JointKind::Slide => Some("slide"), JointKind::Fixed => None };
                    if let Some(t) = t {
                        let range = j.range.map_or(String::new(), |[a, b]| format!(" limited=\"true\" range=\"{} {}\"", f(a), f(b)));
                        let arm = armature.get(&j.id).copied().unwrap_or(0.0) + 1e-4;
                        let _ = write!(x, "<joint name=\"j{}\" type=\"{t}\" axis=\"{}\" pos=\"0 0 0\"{range} damping=\"{}\" frictionloss=\"{}\" armature=\"{}\"/>", j.id, v3(j.axis), f(j.damping + 0.01), f(j.friction), f(arm));
                    }
                }
            }
            let mass = if id == CHASSIS { l.mass_or_estimate() } else { l.mass_or_estimate() } * weighed;
            let (mut com, it) = link_inertia(&l.shapes, mass, o);
            if id == CHASSIS { com[2] = r.frame.chassis_com_height; }
            let _ = write!(x, "<inertial pos=\"{}\" mass=\"{}\" fullinertia=\"{} {} {} {} {} {}\"/>", v3(sub(com, o)), f(mass.max(1e-3)),
                f(it[0].max(1e-6)), f(it[4].max(1e-6)), f(it[8].max(1e-6)), f(it[1]), f(it[2]), f(it[5]));
            for (si, s) in l.shapes.iter().enumerate() {
                let (t, size) = match &s.kind {
                    ShapeKind::Box { size } => ("box", format!("size=\"{}\"", v3(scale(*size, 0.5)))),
                    ShapeKind::Cylinder { radius, length } => ("cylinder", format!("size=\"{} {}\"", f(*radius), f(length / 2.0))),
                    ShapeKind::Sphere { radius } => ("sphere", format!("size=\"{}\"", f(*radius))),
                    ShapeKind::Hull { .. } => ("mesh", format!("mesh=\"h{id}_{si}\"")),
                };
                // CAD hulls touch pieces and the field but not the carpet: the drivetrain stands
                // the robot up, and a swerve module's tread in the CAD sits right on the floor
                let hull = matches!(s.kind, ShapeKind::Hull { .. });
                let (ct, ca, fr) = match (s.role, l.collision) {
                    (ShapeRole::Ghost, _) | (_, Collision::None) => (0, 0, 0.5),
                    (ShapeRole::Bumper, _) => (BIT_ROBOT, BIT_CARPET | BIT_PIECE | BIT_FIELD, 0.1),
                    _ if hull => (BIT_ROBOT, BIT_PIECE | BIT_FIELD, 0.5),
                    _ => (BIT_ROBOT, BIT_CARPET | BIT_PIECE | BIT_FIELD, 0.5),
                };
                let color = s.color.clone().unwrap_or_else(|| l.color.clone());
                let _ = write!(x, "<geom name=\"g{id}_{si}\" type=\"{t}\" {size} pos=\"{}\" quat=\"{}\" contype=\"{ct}\" conaffinity=\"{ca}\" friction=\"{fr} 0.005 0.0001\" rgba=\"{}\" mass=\"0\"/>",
                    v3(sub(s.pose.pos, o)), q4(s.pose.quat), rgba(&color, 1.0));
            }
            if id == CHASSIS && scene.mode != Mode::Bench {
                if let Some(dt) = &r.drivetrain {
                    let rad = library::Library::builtin().wheel(&dt.wheel).map_or(0.0508, |w| w.radius());
                    for (k, p) in dt.module_positions(&r.frame).iter().enumerate() {
                        let _ = write!(x, "<geom name=\"wheel{k}\" type=\"sphere\" size=\"{}\" pos=\"{} {} {}\" contype=\"{}\" conaffinity=\"{}\" condim=\"1\" priority=\"2\" friction=\"0 0 0\" mass=\"0\" rgba=\"0.1 0.1 0.1 1\"/>",
                            f(rad), f(p[0]), f(p[1]), f(rad), BIT_WHEEL, BIT_CARPET | BIT_FIELD);
                    }
                }
            }
            if let Some(cs) = children.get(&id) {
                for &c in cs { body(x, r, c, o, children, armature, origin_of, scene, weighed); }
            }
            let _ = write!(x, "</body>");
        }
        body(&mut x, r, CHASSIS, [0.0; 3], &children, &armature, &origin_of, &scene, weighed);

        // pieces
        let mut kinds: Vec<PieceKind> = Vec::new();
        let mut spawns: Vec<Spawn> = scene.pieces.clone();
        if let (Some(g), true) = (&game, scene.game_pieces) {
            for s in &g.pieces {
                let Some(lp) = lib.piece(&s.piece) else { continue };
                let k = PieceKind::from_library(lp);
                let (_, lay) = k.resting();
                let quat = if s.upright { axis_angle([0.0, 0.0, 1.0], s.yaw_deg.to_radians()) } else { qmul(axis_angle([0.0, 0.0, 1.0], s.yaw_deg.to_radians()), lay) };
                spawns.push(Spawn { piece: s.piece.clone(), pos: s.pos, quat, lin: [0.0; 3], ang: [0.0; 3] });
            }
            // pieces the feeders will hand out: two per feeder, parked until then
            for fd in &g.feeders { for _ in 0..2 { spawns.push(Spawn { piece: fd.piece.clone(), pos: [0.0, 0.0, -50.0], quat: QI, lin: [0.0; 3], ang: [0.0; 3] }); } }
        }
        let mut piece_kind_idx = Vec::new();
        for (i, s) in spawns.iter().enumerate() {
            let ki = match kinds.iter().position(|k| k.id == s.piece) {
                Some(k) => k,
                None => {
                    let Some(lp) = lib.piece(&s.piece) else { anyhow::bail!("no game piece '{}' in the library", s.piece) };
                    kinds.push(PieceKind::from_library(lp));
                    kinds.len() - 1
                }
            };
            piece_kind_idx.push(ki);
            let friction = match kinds[ki].surface.as_str() { "pvc" => 0.45, "playground-rubber" => 0.8, "foam" => 0.7, _ => 0.6 };
            x.push_str(&kinds[ki].mjcf(&format!("p{i}"), s.pos, s.quat, friction));
        }
        x.push_str("</worldbody><equality>");
        for j in &r.joints {
            if let Some(fl) = &j.follows {
                let _ = write!(x, "<joint joint1=\"j{}\" joint2=\"j{}\" polycoef=\"0 {} 0 0 0\" solref=\"0.005 1\"/>", j.id, fl.joint, f(fl.ratio));
            }
        }
        x.push_str("</equality><actuator>");
        for d in &r.drives {
            if let DriveTarget::Joint { joint, .. } = d.target {
                let Some(m) = motors.get(&d.id) else { continue };
                if r.joint(joint).is_some_and(|j| j.kind != JointKind::Fixed) {
                    let _ = write!(x, "<general name=\"d{}\" joint=\"j{joint}\" gainprm=\"{}\" biastype=\"affine\" biasprm=\"0 0 {}\" ctrllimited=\"true\" ctrlrange=\"-16 16\"/>", d.id, f(m.gain()), f(-m.slope()));
                }
            }
        }
        x.push_str("</actuator></mujoco>");
        // empty sections MuJoCo dislikes
        let x = x.replace("<equality></equality>", "").replace("<actuator></actuator>", "");

        let mut engine = Engine::from_xml(&x)?;
        let mjcf = x.clone();

        // ---- runtime tables ----
        let body_of = |e: &Engine, id: Id| e.id(Obj::Body, &format!("l{id}")).unwrap();
        let links: Vec<LinkRt> = order.iter().map(|&id| LinkRt { id, body: body_of(&engine, id), origin: origin_of(id) }).collect();
        let robot_bodies: Vec<usize> = links.iter().map(|l| l.body).collect();
        let sensors: Vec<SensorRt> = r.sensors.iter().filter_map(|x| match x {
            Sensor::BeamBreak { name, link, from, to } => {
                let o = origin_of(*link);
                Some(SensorRt { name: name.clone(), kind: SensorKind::Beam { link: links.iter().position(|l| l.id == *link).unwrap_or(0), from: sub(*from, o), to: sub(*to, o) }, on: false })
            }
            Sensor::LimitSwitch { name, joint, at, band, .. } => {
                let jnt = engine.id(Obj::Joint, &format!("j{joint}"))?;
                Some(SensorRt { name: name.clone(), kind: SensorKind::Limit { jnt, at: *at, band: *band }, on: false })
            }
        }).collect();
        // the robot's solid geoms, for soft pieces' contact
        let mut robot_geoms: Vec<(usize, usize)> = vec![];
        for (li_, lr) in links.iter().enumerate() {
            let Some(l) = r.link(lr.id) else { continue };
            if l.collision == Collision::None { continue; }
            for (si, s) in l.shapes.iter().enumerate() {
                if s.role == ShapeRole::Ghost { continue; }
                if let Some(g) = engine.id(Obj::Geom, &format!("g{}_{si}", l.id)) { robot_geoms.push((g, li_)); }
            }
        }
        let li = |id: Id| links.iter().position(|l| l.id == id).unwrap_or(0);
        let mut joints: Vec<JointRt> = Vec::new();
        for j in &r.joints {
            if j.kind == JointKind::Fixed { continue; }
            let Some(jnt) = engine.id(Obj::Joint, &format!("j{}", j.id)) else { continue };
            joints.push(JointRt { id: j.id, jnt, kind: j.kind, range: j.range, followers: vec![], spring: j.spring.clone() });
        }
        for j in &r.joints {
            if let Some(fl) = &j.follows {
                let Some(fj) = engine.id(Obj::Joint, &format!("j{}", j.id)) else { continue };
                if let Some(lead) = joints.iter_mut().find(|x| x.id == fl.joint) { lead.followers.push((fj, fl.ratio)); }
            }
        }
        // set the starting positions
        for j in &r.joints {
            if j.kind == JointKind::Fixed { continue; }
            if let Some(jnt) = engine.id(Obj::Joint, &format!("j{}", j.id)) {
                let lead = j.follows.as_ref().and_then(|fl| r.joint(fl.joint)).map(|l| l.initial * j.follows.as_ref().unwrap().ratio);
                engine.set_q(jnt, lead.unwrap_or(j.initial));
            }
        }
        let root_joint = engine.id(Obj::Joint, "root");
        engine.forward();
        let mdiag = engine.mass_diag();

        let mut shafts: Vec<ShaftRt> = r.shafts.iter().map(|s| ShaftRt { id: s.id, omega: 0.0, angle: 0.0, inertia: s.inertia, drag: s.drag, torque: 0.0 }).collect();
        let mut rollers = Vec::new();
        for x in &r.rollers {
            let Some(si) = shafts.iter().position(|s| s.id == x.shaft) else { continue };
            let link_id = r.shaft(x.shaft).map_or(CHASSIS, |s| s.link);
            let wheel_id = scene.wheel_override.clone().unwrap_or_else(|| x.wheel.clone());
            let wheel = lib.wheel(&wheel_id).or_else(|| lib.wheel("am-compliant-4in-60a")).unwrap();
            let radius = if scene.wheel_override.is_some() { wheel.radius() } else { x.radius(lib) };
            // the Bench is for contact: every roller touches there (D3)
            let full = scene.mode == Mode::Bench || r.mechanism_of_link(link_id).is_none_or(|m| project.fidelity(m) == Fidelity::Full);
            let surface_kind = match wheel.build { library::WheelBuild::Omni => RollerKind::Omni, library::WheelBuild::Mecanum => RollerKind::Mecanum, _ => x.kind };
            // the pieces' surface: the most common one this scene has
            let surf = kinds.first().map_or("pvc".to_string(), |k| k.surface.clone());
            let (mu_s, mu_k) = lib.friction(&wheel.tread, &surf);
            let surface = RollerSurface { radius, width: x.width, k_area: wheel.k_area(), mu_static: mu_s, mu_sliding: mu_k, kind: surface_kind, loss: 0.2, slices: RollerSurface::slices_for(x.width) };
            // the roller's own inertia: a wheel of about this mass
            let m_w = 0.12 * (radius / 0.0508).powi(2) * (x.width / 0.0254).max(0.5);
            shafts[si].inertia += 0.5 * m_w * radius * radius * x.ratio * x.ratio;
            let o = origin_of(link_id);
            rollers.push(RollerRt { id: x.id, shaft: si, ratio: x.ratio, link: li(link_id), center: sub(x.center, o), axis: norm(x.axis), surface, full, wheel: wheel_id, radius });
        }
        let mut drives = Vec::new();
        for d in &r.drives {
            let Some(m) = motors.get(&d.id).cloned() else { continue };
            let (target, inertia) = match d.target {
                DriveTarget::Joint { joint, .. } => {
                    let Some(jr) = joints.iter().find(|j| j.id == joint) else { continue };
                    let Some(act) = engine.id(Obj::Actuator, &format!("d{}", d.id)) else { continue };
                    let mut inertia = mdiag.get(engine.dof(jr.jnt)).copied().unwrap_or(1.0);
                    for (fj, k) in &jr.followers { inertia += mdiag.get(engine.dof(*fj)).copied().unwrap_or(0.0) * k * k; }
                    (Target::Joint { joint: joints.iter().position(|j| j.id == joint).unwrap(), act }, inertia)
                }
                DriveTarget::Shaft { shaft } => {
                    let Some(si) = shafts.iter().position(|s| s.id == shaft) else { continue };
                    shafts[si].inertia += m.reflected_inertia();
                    (Target::Shaft { shaft: si }, shafts[si].inertia)
                }
            };
            drives.push(DriveRt { id: d.id, name: d.name.clone(), motor: m, target, control: d.control.clone(), state: control::State::default(), out: Default::default(), inertia, brake: d.brake });
        }
        let mechs = r.mechanisms.iter().map(|m| {
            let full = scene.mode == Mode::Bench || project.fidelity(m) == Fidelity::Full;
            let pickup = m.pickup.as_ref().map(|p| {
                let o = origin_of(p.link);
                (li(p.link), Pose::at(sub(p.center, o)), p.size, p.drive, p.pieces.clone(), p.acquire_time)
            });
            let holders = m.holders.iter().map(|h| {
                let o = origin_of(h.link);
                HolderRt { link: li(h.link), pose: Pose::new(sub(h.pose.pos, o), h.pose.quat), release: h.release.clone(), piece: None }
            }).collect();
            MechRt { id: m.id, full, pickup, holders, dwell: HashMap::new() }
        }).collect();
        let pieces: Vec<PieceRt> = spawns.iter().enumerate().map(|(i, s)| {
            let jnt = engine.id(Obj::Joint, &format!("p{i}")).unwrap();
            let body = engine.id(Obj::Body, &format!("p{i}")).unwrap();
            let parked = s.pos[2] < -10.0;
            let geom = if kinds[piece_kind_idx[i]].soft() { (0..engine.ngeom()).find(|g| engine.geom_body(*g) == body) } else { None };
            PieceRt { kind: piece_kind_idx[i], jnt, body, at: if parked { PieceWhere::Parked } else { PieceWhere::Free }, shown: Pose::new(s.pos, s.quat), park: [200.0 + 3.0 * i as f64, 200.0, -20.0], soft_geom: geom }
        }).collect();
        for (i, s) in spawns.iter().enumerate() { engine.set_free(pieces[i].jnt, if pieces[i].at == PieceWhere::Parked { pieces[i].park } else { s.pos }, s.quat, s.lin, s.ang); }
        let drive = match (&r.drivetrain, scene.mode) {
            (Some(d), Mode::Field | Mode::Carpet) => {
                let m = lib.motor(&d.motor).ok_or_else(|| anyhow::anyhow!("no motor {}", d.motor))?;
                let wheel_r = lib.wheel(&d.wheel).map_or(0.0508, |w| w.radius());
                let mo = Motor::new(m.constants(), 1.0, d.ratio, 0.93, d.stator_limit, d.supply_limit);
                Some(swerve::Drive::new(d.kind, d.module_positions(&r.frame), mo, wheel_r, d.mu, d.max_speed, d.max_turn))
            }
            _ => None,
        };
        // the step must resolve the stiffest roller contact (D4)
        for x in &rollers {
            if !x.full { continue; }
            for k in &kinds {
                let d: f64 = 0.01;
                let w = x.surface.width;
                let kk = 1.5 * x.surface.k_area * w * (4.0 / 3.0) * (2.0 * x.surface.radius).sqrt() * d.sqrt();
                let wn = (kk / k.mass).sqrt();
                if wn * dt > 0.5 {
                    warnings.push(format!("{} on {} is stiff for this step size ({:.0} Hz would be safer)", x.wheel, k.name, wn / 0.5 / std::f64::consts::TAU * std::f64::consts::TAU));
                }
            }
        }
        engine.forward();
        let root_body = links[0].body;
        let feeders = game.as_ref().map_or(vec![], |g| g.feeders.iter().map(|_| FeederRt { last: -10.0 }).collect());
        let n_zones = game.as_ref().map_or(0, |g| g.zones.len());
        let mut cmd = Commands::default();
        for d in &r.drives {
            if let DriveTarget::Joint { joint, .. } = d.target {
                if let Some(j) = r.joint(joint) { cmd.targets.insert(d.id, j.initial); }
            }
        }
        Ok(World {
            engine, lib: lib.clone(), project: project.clone(), scene, game, dt, time: 0.0, cmd, bus: Bus::new(r.electrical.clone()),
            warnings, matched: Match { filled: vec![0; n_zones], ..Default::default() }, mjcf,
            links, joints, drives, shafts, rollers, mechs, kinds, pieces, feeders, drive, root_body, root_joint,
            bristles: Bristles::new(), contacts: vec![], contact_rollers: vec![], robot_bodies, robot_geoms, sensors, settle: HashMap::new(),
        })
    }

    fn link_pose(&self, li: usize) -> Pose {
        let b = self.links[li].body;
        Pose::new(self.engine.body_pos(b), self.engine.body_quat(b))
    }

    /// Copies what it can from an older world after an edit (D: edit while it runs): the robot's
    /// place, joint positions, roller speeds and the pieces.
    pub fn carry_from(&mut self, old: &World) {
        if let (Some(a), Some(b)) = (self.root_joint, old.root_joint) {
            let (p, q) = old.engine.free_pose(b);
            let (lv, av) = old.engine.free_vel(b);
            self.engine.set_free(a, p, q, lv, av);
        }
        for j in &self.joints {
            if let Some(o) = old.joints.iter().find(|x| x.id == j.id) {
                self.engine.set_q(j.jnt, old.engine.q(o.jnt));
                self.engine.set_qd(j.jnt, old.engine.qd(o.jnt));
                for (k, (fj, _)) in j.followers.iter().enumerate() {
                    if let Some((ofj, _)) = o.followers.get(k) { self.engine.set_q(*fj, old.engine.q(*ofj)); }
                }
            }
        }
        for s in &mut self.shafts { if let Some(o) = old.shafts.iter().find(|x| x.id == s.id) { s.omega = o.omega; s.angle = o.angle; } }
        if self.pieces.len() == old.pieces.len() {
            for (i, p) in self.pieces.iter_mut().enumerate() {
                let o = &old.pieces[i];
                let (pos, q) = old.engine.free_pose(o.jnt);
                let (lv, av) = old.engine.free_vel(o.jnt);
                self.engine.set_free(p.jnt, pos, q, lv, av);
                p.at = if matches!(o.at, PieceWhere::Held { .. }) { PieceWhere::Free } else { o.at };
                p.shown = o.shown;
            }
        }
        self.cmd = old.cmd.clone();
        self.time = old.time;
        self.matched = old.matched.clone();
        if self.matched.filled.len() != self.game.as_ref().map_or(0, |g| g.zones.len()) { self.matched.filled = vec![0; self.game.as_ref().map_or(0, |g| g.zones.len())]; }
        self.bus = Bus { cfg: self.bus.cfg.clone(), ..old.bus.clone() };
        self.engine.forward();
    }

    pub fn debug_drive(&self) -> String { self.drive.as_ref().map_or(String::new(), |d| format!("{:?}", d.modules.iter().map(|m| (m.down, m.slip, (m.w * 100.0).round() / 100.0, (m.angle*100.0).round()/100.0)).collect::<Vec<_>>())) }

    /// Changes the step size (tests, and the Bench's own rate).
    pub fn set_timestep(&mut self, dt: f64) { self.dt = dt; self.engine.set_timestep(dt); }

    pub fn drive_ids(&self) -> Vec<Id> { self.drives.iter().map(|d| d.id).collect() }

    /// Applies a gamepad button (D12): presets set targets, runs hold a value while pressed.
    pub fn button(&mut self, b: Button, down: bool) {
        let bindings = self.project.robot.controls.bindings.clone();
        for bind in bindings.iter().filter(|x| x.button == b) {
            match &bind.action {
                Action::Preset { preset } if down => {
                    if let Some(p) = self.project.robot.controls.presets.iter().find(|p| p.id == *preset) {
                        for t in &p.targets { self.cmd.targets.insert(t.drive, t.value); }
                    }
                }
                Action::Run { drive, value, until } => {
                    // a run that stops on a sensor doesn't start while the sensor already reads true
                    let tripped = until.as_ref().is_some_and(|u| self.sensor(u) == Some(true));
                    if down && !tripped { self.cmd.run.insert(*drive, *value); } else { self.cmd.run.remove(drive); }
                    match until { Some(u) if down => { self.cmd.until.insert(*drive, u.clone()); } _ => { self.cmd.until.remove(drive); } }
                    // running a drive backwards lets go of what its mechanism holds
                    if down && *value < 0.0 {
                        let mech = self.project.robot.mechanisms.iter().find(|m| m.pickup.as_ref().is_some_and(|p| p.drive == Some(*drive))).map(|m| m.id);
                        if let Some(m) = mech { self.cmd.release.push(m); }
                    }
                }
                Action::Release { mechanism } if down => self.cmd.release.push(*mechanism),
                _ => {}
            }
        }
    }

    pub fn step(&mut self) {
        let dt = self.dt;
        self.engine.begin_step();
        if !self.sensors.is_empty() { self.read_sensors(); }
        let bus_v = self.bus.voltage;
        let mut supply = 0.0;

        // ---- joint drives ----
        for i in 0..self.drives.len() {
            let Target::Joint { joint, act } = self.drives[i].target else { continue };
            let jr = &self.joints[joint];
            let mut bias = self.engine.bias(jr.jnt);
            for (fj, k) in &jr.followers { bias += self.engine.bias(*fj) * k; }
            let pl = control::Plant { kind: jr.kind, q: self.engine.q(jr.jnt), qd: self.engine.qd(jr.jnt), inertia: self.drives[i].inertia, bias, range: jr.range, bus: bus_v };
            let d = &mut self.drives[i];
            let volts = if let Some(v) = self.cmd.run.get(&d.id) {
                if matches!(d.control, Control::Velocity { .. }) {
                    control::joint_volts(&d.control, &mut d.state, *v, &pl, &d.motor, dt)
                } else { *v }
            } else {
                let target = self.cmd.targets.get(&d.id).copied().unwrap_or(pl.q);
                control::joint_volts(&d.control, &mut d.state, target, &pl, &d.motor, dt)
            };
            let volts = if self.bus.brownout || self.bus.tripped { 0.0 } else { volts.clamp(-bus_v, bus_v) };
            d.out = d.motor.torque(pl.qd, volts, bus_v);
            supply += d.out.supply;
            self.engine.set_ctrl(act, d.out.volts);
        }
        // springs on joints
        self.engine.clear_joint_forces();
        for j in &self.joints {
            if let Some(s) = &j.spring { let q = self.engine.q(j.jnt); self.engine.add_joint_force(j.jnt, s.preload + s.stiffness * q); }
        }

        // ---- concept pickups and held pieces ----
        self.concept();

        // ---- roller contact ----
        for s in &mut self.shafts { s.torque = 0.0; }
        self.contacts.clear();
        self.contact_rollers.clear();
        let mut out = Vec::new();
        for ri in 0..self.rollers.len() {
            let rl = &self.rollers[ri];
            if !rl.full { continue; }
            let lp = self.link_pose(rl.link);
            let c = lp.apply(rl.center);
            let a = lp.rotate(rl.axis);
            let b = self.links[rl.link].body;
            let (lv, av) = self.engine.velocity_at(b, c);
            let spin = self.shafts[rl.shaft].omega * rl.ratio;
            let rs = RollerState { center: c, axis: a, spin, lin: lv, ang: av };
            for (pi, p) in self.pieces.iter().enumerate() {
                if p.at != PieceWhere::Free { continue; }
                let (pos, q) = self.engine.free_pose(p.jnt);
                let (lin, _) = self.engine.velocity_at(p.body, pos);
                let (_, ang) = self.engine.velocity_at(p.body, pos);
                let ps = contact::PieceState { pos, mat: qmat(q), lin, ang };
                out.clear();
                contact::roller_piece(&rl.surface, &rs, &self.kinds[p.kind], &ps, dt, &mut self.bristles, ri as u32, pi as u32, &mut out);
                for sc in &out {
                    self.engine.push(p.body, sc.force, sc.point);
                    // the roller pushes back on its link, except its turn about its own axis,
                    // which the shaft and motor take
                    let fr = neg(sc.force);
                    let axial = dot(cross(sub(sc.point, c), fr), a);
                    self.engine.push(b, fr, sc.point);
                    self.engine.torque(b, scale(a, -axial));
                    self.shafts[rl.shaft].torque += axial * rl.ratio;
                    self.contacts.push(*sc);
                    self.contact_rollers.push(rl.id);
                }
            }
        }

        // ---- soft pieces against the robot's rigid shapes (M5) ----
        for pi in 0..self.pieces.len() {
            let Some(pg) = self.pieces[pi].soft_geom else { continue };
            if self.pieces[pi].at != PieceWhere::Free { continue; }
            let pb = self.pieces[pi].body;
            let kind = &self.kinds[self.pieces[pi].kind];
            let (mu_s, mu_k) = match kind.surface.as_str() { "playground-rubber" => (0.9, 0.7), "foam" => (0.8, 0.65), _ => (0.5, 0.4) };
            let (pc, pr) = (self.engine.geom_pos(pg), self.engine.geom_rbound(pg));
            for gi in 0..self.robot_geoms.len() {
                let (g, li_) = self.robot_geoms[gi];
                let key = (u32::MAX - g as u32, 0u16, pi as u32);
                if len(sub(self.engine.geom_pos(g), pc)) > pr + self.engine.geom_rbound(g) { self.bristles.remove(&key); continue; }
                let Some((d, _, on_robot)) = self.engine.geom_distance(pg, g, 0.001) else { self.bristles.remove(&key); continue };
                if d >= 0.0 { self.bristles.remove(&key); continue; }
                let away = sub(pc, on_robot);
                if len(away) < 1e-9 { continue; }
                let n = norm(away);
                let lb = self.links[li_].body;
                let (vr, _) = self.engine.velocity_at(lb, on_robot);
                let (vp, _) = self.engine.velocity_at(pb, on_robot);
                let b = self.bristles.entry(key).or_default();
                let Some(c) = contact::soft_rigid(&self.kinds[self.pieces[pi].kind], -d, n, on_robot, sub(vr, vp), mu_s, mu_k, dt, b) else { continue };
                self.engine.push(pb, c.force, c.point);
                self.engine.push(lb, neg(c.force), c.point);
                self.contacts.push(c);
                self.contact_rollers.push(0);
            }
        }

        // ---- roller shafts and their motors ----
        for i in 0..self.drives.len() {
            let Target::Shaft { shaft } = self.drives[i].target else { continue };
            let d = &mut self.drives[i];
            let s = &mut self.shafts[shaft];
            let value = self.cmd.run.get(&d.id).copied().unwrap_or(0.0);
            let volts = control::shaft_volts(&d.control, &mut d.state, value, s.omega, dt, &d.motor);
            let volts = if self.bus.brownout || self.bus.tripped { 0.0 } else { volts.clamp(-bus_v, bus_v) };
            let coast = value == 0.0 && !d.brake;
            d.out = if coast { Default::default() } else { d.motor.torque(s.omega, volts, bus_v) };
            supply += d.out.supply;
            let kd = if d.out.limited || coast { 0.0 } else { d.motor.slope() };
            let j = s.inertia.max(1e-7);
            s.omega = (j * s.omega + dt * (d.out.torque + kd * s.omega + s.torque)) / (j + dt * (kd + s.drag));
        }
        for s in &mut self.shafts {
            // shafts with no drive spin down on their own drag
            s.angle += s.omega * dt;
        }

        // ---- the drivetrain ----
        if let (Some(_), Some(root)) = (&self.drive, self.root_joint) {
            let (pos, q) = self.engine.free_pose(root);
            let yaw = quat_to_euler(q)[2];
            let mass = self.engine.subtree_mass(self.root_body);
            let com = self.engine.subtree_com(self.root_body);
            let (lin, ang) = self.engine.velocity_at(self.root_body, com);
            let mut iz = 0.0;
            for &b in &self.robot_bodies {
                let (inr, m) = self.engine.body_inertia(b);
                for k in 0..3 { iz += inr[k] * m[6 + k] * m[6 + k]; }
                let c = self.engine.body_com(b);
                iz += self.engine.body_mass(b) * ((c[0] - com[0]).powi(2) + (c[1] - com[1]).powi(2));
            }
            let pose = Pose::new(pos, q);
            let drv = self.drive.as_mut().unwrap();
            let down: Vec<bool> = drv.pts.iter().map(|p| pose.apply([p[0], p[1], 0.0])[2] < 0.012).collect();
            let mut c = self.cmd.drive;
            if self.cmd.field_relative {
                let (cy, sy) = (yaw.cos(), yaw.sin());
                c = [c[0] * cy + c[1] * sy, -c[0] * sy + c[1] * cy, c[2]];
            }
            let body = swerve::Body { yaw, vel: [lin[0], lin[1]], omega: ang[2], mass, iz: iz.max(0.1), com: [com[0], com[1]], origin: [pos[0], pos[1]] };
            let (pushes, isum) = if self.bus.brownout || self.bus.tripped { drv.step(body, [0.0; 3], &down, bus_v, dt) } else { drv.step(body, c, &down, bus_v, dt) };
            supply += isum;
            if std::env::var("DEBUG_DRIVE").is_ok() && (self.time * 1000.0) as i64 % 250 == 0 {
                let fx: f64 = pushes.iter().map(|p| p.force[0]).sum();
                eprintln!("t {:.3} vx {:.3} fx {:.1} mass {:.1} iz {:.2}", self.time, lin[0], fx, mass, iz);
            }
            for p in pushes {
                let at = pose.apply([p.at[0], p.at[1], 0.0]);
                self.engine.push(self.root_body, [p.force[0], p.force[1], 0.0], at);
            }
        }

        // ---- the engine ----
        self.engine.step();
        self.time += dt;
        self.bus.step(supply, dt);
        if !self.engine.finite() { self.warnings.push("The simulation blew up (a value went to infinity). Try a smaller step or softer settings.".into()); }

        // ---- the game ----
        if self.game.is_some() { self.game_step(); }
        self.cmd.release.clear();
    }

    /// Concept pickups (D3): a free piece in a running pickup zone is acquired after its dwell time
    /// and parked; held pieces are drawn at their holders and weigh on them.
    fn concept(&mut self) {
        let release: Vec<Id> = self.cmd.release.clone();
        for mi in 0..self.mechs.len() {
            if self.mechs[mi].full { continue; }
            let mid = self.mechs[mi].id;
            // let go
            if release.contains(&mid) {
                for hi in 0..self.mechs[mi].holders.len() {
                    let Some(pi) = self.mechs[mi].holders[hi].piece.take() else { continue };
                    let h = self.mechs[mi].holders[hi].clone();
                    let hp = self.link_pose(h.link).mul(&h.pose);
                    let k = &self.kinds[self.pieces[pi].kind];
                    let (_, lay) = k.resting();
                    let quat = qmul(hp.quat, lay);
                    let (lv, _) = self.engine.velocity_at(self.links[h.link].body, hp.pos);
                    let dir = hp.rotate([1.0, 0.0, 0.0]);
                    let speed = match h.release {
                        Release::Place { eject } => eject,
                        Release::Launch { shaft, efficiency } => {
                            let s = self.shafts.iter().find(|s| s.id == shaft);
                            let r = self.rollers.iter().find(|r| self.shafts[r.shaft].id == shaft).map_or(0.05, |r| r.radius);
                            s.map_or(0.0, |s| s.omega.abs() * r * efficiency)
                        }
                    };
                    let lin = add(lv, scale(dir, speed));
                    self.engine.set_free(self.pieces[pi].jnt, hp.pos, quat, lin, [0.0; 3]);
                    self.pieces[pi].at = PieceWhere::Free;
                    self.pieces[pi].shown = Pose::new(hp.pos, quat);
                    self.seat_if_close(pi);
                }
            }
            // pick up
            let Some((link, zpose, size, drive, filter, acquire)) = self.mechs[mi].pickup.clone() else { continue };
            let running = drive.is_none_or(|d| self.cmd.run.get(&d).is_some_and(|v| *v > 0.0));
            let free_holder = self.mechs[mi].holders.iter().position(|h| h.piece.is_none());
            let zp = self.link_pose(link).mul(&zpose);
            let inv = zp.inverse();
            let mut seen = vec![];
            for pi in 0..self.pieces.len() {
                if self.pieces[pi].at != PieceWhere::Free { continue; }
                let k = &self.kinds[self.pieces[pi].kind];
                if !filter.is_empty() && !filter.contains(&k.id) { continue; }
                let (pos, quat) = self.engine.free_pose(self.pieces[pi].jnt);
                let l = inv.apply(pos);
                let inside = (0..3).all(|i| l[i].abs() <= size[i] / 2.0 + 0.4 * k.bound().min(0.1));
                if inside && running && free_holder.is_some() {
                    seen.push(pi);
                    // the rollers have it: it moves with the mechanism while it's drawn in (a piece
                    // dropped through a funnel doesn't fall out the bottom)
                    let (lv, _) = self.engine.velocity_at(self.links[link].body, pos);
                    self.engine.set_free(self.pieces[pi].jnt, pos, quat, lv, [0.0; 3]);
                    let t = self.mechs[mi].dwell.entry(pi).or_insert(0.0);
                    *t += self.dt;
                    if *t >= acquire {
                        let h = free_holder.unwrap();
                        self.mechs[mi].holders[h].piece = Some(pi);
                        self.pieces[pi].at = PieceWhere::Held { mech: mid };
                        if self.game.is_some() { self.matched.log.push(crate::game::Event { time: self.matched.time, what: format!("picked up {}", k.name), points: 0.0 }); }
                        break;
                    }
                }
            }
            self.mechs[mi].dwell.retain(|k, _| seen.contains(k));
        }
        // held pieces: parked out of the way, drawn at the holder, weighing on it
        for mi in 0..self.mechs.len() {
            for hi in 0..self.mechs[mi].holders.len() {
                let h = self.mechs[mi].holders[hi].clone();
                let Some(pi) = h.piece else { continue };
                let hp = self.link_pose(h.link).mul(&h.pose);
                let k = &self.kinds[self.pieces[pi].kind];
                let (_, lay) = k.resting();
                self.pieces[pi].shown = Pose::new(hp.pos, qmul(hp.quat, lay));
                let m = k.mass;
                self.engine.push(self.links[h.link].body, [0.0, 0.0, -m * G], hp.pos);
            }
        }
        for p in &mut self.pieces {
            if matches!(p.at, PieceWhere::Held { .. } | PieceWhere::Scored | PieceWhere::Parked) {
                self.engine.set_free(p.jnt, p.park, QI, [0.0; 3], [0.0; 3]);
            }
        }
    }

    /// Concept's scoring assist: a piece let go close enough to a seat is seated and counted.
    fn seat_if_close(&mut self, pi: usize) {
        let Some(g) = &self.game else { return };
        let kid = self.kinds[self.pieces[pi].kind].id.clone();
        let pos = self.pieces[pi].shown.pos;
        let mut best: Option<(usize, f64)> = None;
        for (zi, z) in g.zones.iter().enumerate() {
            let Some(seat) = &z.seat else { continue };
            if z.piece != kid || self.matched.filled.get(zi).copied().unwrap_or(0) >= z.capacity { continue; }
            let d = len(sub(seat.pos, pos));
            if d <= seat.tolerance && best.is_none_or(|(_, b)| d < b) { best = Some((zi, d)); }
        }
        if let Some((zi, _)) = best {
            let z = g.zones[zi].clone();
            let seat = z.seat.clone().unwrap();
            self.pieces[pi].at = PieceWhere::Scored;
            self.pieces[pi].shown = Pose::new(seat.pos, euler_to_quat(seat.euler_deg.map(f64::to_radians)));
            self.score_zone(zi);
        }
    }

    fn score_zone(&mut self, zi: usize) {
        let Some(g) = &self.game else { return };
        let z = &g.zones[zi];
        let auto = self.matched.running && self.matched.time < g.timing.auto;
        let pts = if auto { z.auto_points } else { z.teleop_points };
        self.matched.filled[zi] += 1;
        if self.matched.running && self.matched.time >= g.timing.auto + g.timing.teleop { return; }
        self.matched.score += pts;
        self.matched.log.push(crate::game::Event { time: self.matched.time, what: format!("scored {}", z.name), points: pts });
    }

    fn game_step(&mut self) {
        let dt = self.dt;
        self.matched.time += dt;
        let g = self.game.clone().unwrap();
        // pieces at rest in a zone without a seat (the trough, the PROCESSOR, the NET) count once
        for pi in 0..self.pieces.len() {
            if self.pieces[pi].at != PieceWhere::Free { continue; }
            let (pos, _) = self.engine.free_pose(self.pieces[pi].jnt);
            let kid = self.kinds[self.pieces[pi].kind].id.clone();
            for (zi, z) in g.zones.iter().enumerate() {
                if z.seat.is_some() || z.piece != kid || self.matched.filled[zi] >= z.capacity { continue; }
                if z.contains(pos) {
                    self.pieces[pi].at = PieceWhere::Scored;
                    self.pieces[pi].shown = Pose::new(pos, self.engine.free_pose(self.pieces[pi].jnt).1);
                    self.score_zone(zi);
                    break;
                }
            }
        }
        // a piece that comes to rest on a branch (Full: nobody seats it) is scored where it lies
        let mut still = vec![];
        for pi in 0..self.pieces.len() {
            if self.pieces[pi].at != PieceWhere::Free { continue; }
            let (pos, quat) = self.engine.free_pose(self.pieces[pi].jnt);
            let (lin, ang) = self.engine.velocity_at(self.pieces[pi].body, pos);
            if len(lin) > 0.15 || len(ang) > 1.0 { continue; }
            let kid = &self.kinds[self.pieces[pi].kind].id;
            let near = g.zones.iter().enumerate().find(|(zi, z)| z.piece == *kid && self.matched.filled[*zi] < z.capacity
                && z.seat.as_ref().is_some_and(|s| len(sub(s.pos, pos)) < SETTLE_REACH));
            let Some((zi, _)) = near else { continue };
            still.push(pi);
            let t = self.settle.entry(pi).or_insert(0.0);
            *t += dt;
            if *t >= SETTLE_TIME {
                self.pieces[pi].at = PieceWhere::Scored;
                self.pieces[pi].shown = Pose::new(pos, quat);
                self.score_zone(zi);
            }
        }
        self.settle.retain(|k, _| still.contains(k));
        // feeders hand a piece to a robot standing by them
        if let Some(root) = self.root_joint {
            let (rp, _) = self.engine.free_pose(root);
            for (fi, fd) in g.feeders.iter().enumerate() {
                if self.time - self.feeders[fi].last < fd.interval { continue; }
                if len(sub([fd.pos[0], fd.pos[1], 0.0], [rp[0], rp[1], 0.0])) > fd.reach { continue; }
                let Some(pi) = self.pieces.iter().position(|p| p.at == PieceWhere::Parked && self.kinds[p.kind].id == fd.piece) else { continue };
                let k = &self.kinds[self.pieces[pi].kind];
                let (_, lay) = k.resting();
                let yaw = fd.dir[1].atan2(fd.dir[0]) + std::f64::consts::FRAC_PI_2;
                let quat = qmul(axis_angle([0.0, 0.0, 1.0], yaw), lay);
                self.engine.set_free(self.pieces[pi].jnt, fd.pos, quat, scale(norm(fd.dir), fd.speed), [0.0; 3]);
                self.pieces[pi].at = PieceWhere::Free;
                self.feeders[fi].last = self.time;
            }
        }
        // scored pieces that were seated stay; free pieces that leave the field come back to a feeder
        let (hx, hy) = (g.field.length / 2.0 + 0.5, g.field.width / 2.0 + 0.5);
        for p in &mut self.pieces {
            if p.at != PieceWhere::Free { continue; }
            let (pos, _) = self.engine.free_pose(p.jnt);
            if pos[0].abs() > hx || pos[1].abs() > hy || pos[2] < -1.0 { p.at = PieceWhere::Parked; }
        }
    }

    /// Starts a timed match from now.
    pub fn start_match(&mut self) {
        let n = self.matched.filled.len();
        self.matched = Match { running: true, time: 0.0, score: 0.0, filled: vec![0; n], log: vec![] };
    }

    /// The robot's centre of mass and how far it is inside the wheels' footprint.
    fn tipping(&self) -> (V3, Vec<[f64; 2]>, f64) {
        let com = self.engine.subtree_com(self.root_body);
        let Some(d) = &self.drive else { return (com, vec![], 1.0) };
        let lp = self.link_pose(0);
        let pts: Vec<[f64; 2]> = d.pts.iter().map(|p| { let w = lp.apply([p[0], p[1], 0.0]); [w[0], w[1]] }).collect();
        // convex footprint of the wheels (four corners, in order round the robot)
        let mut ord = pts.clone();
        let cx = ord.iter().map(|p| p[0]).sum::<f64>() / ord.len() as f64;
        let cy = ord.iter().map(|p| p[1]).sum::<f64>() / ord.len() as f64;
        ord.sort_by(|a, b| (a[1] - cy).atan2(a[0] - cx).partial_cmp(&(b[1] - cy).atan2(b[0] - cx)).unwrap());
        let mut margin = f64::INFINITY;
        for i in 0..ord.len() {
            let (a, b) = (ord[i], ord[(i + 1) % ord.len()]);
            let e = [b[0] - a[0], b[1] - a[1]];
            let n = [e[1], -e[0]];
            let l = (n[0] * n[0] + n[1] * n[1]).sqrt().max(1e-9);
            // inside is to the left of each edge going round counter-clockwise
            let d = -((com[0] - a[0]) * n[0] + (com[1] - a[1]) * n[1]) / l;
            margin = margin.min(d);
        }
        (com, ord, margin)
    }

    /// a sensor's reading by name
    pub fn sensor(&self, name: &str) -> Option<bool> { self.sensors.iter().find(|x| x.name == name).map(|x| x.on) }

    /// Reads every sensor: a beam break is broken while any piece (free or held) crosses its beam;
    /// a limit switch is on within its band. Then stops the runs waiting on one.
    fn read_sensors(&mut self) {
        for i in 0..self.sensors.len() {
            let on = match self.sensors[i].kind.clone() {
                SensorKind::Limit { jnt, at, band } => (self.engine.q(jnt) - at).abs() <= band,
                SensorKind::Beam { link, from, to } => {
                    let lp = self.link_pose(link);
                    let (a, b) = (lp.apply(from), lp.apply(to));
                    self.pieces.iter().any(|p| {
                        let pose = match p.at { PieceWhere::Free => { let (x, q) = self.engine.free_pose(p.jnt); Pose::new(x, q) }, PieceWhere::Held { .. } => p.shown, _ => return false };
                        let k = &self.kinds[p.kind];
                        let n = (len(sub(b, a)) / 0.005).ceil().clamp(2.0, 400.0) as usize;
                        (0..=n).any(|s| {
                            let pt = lerp(a, b, s as f64 / n as f64);
                            if len(sub(pt, pose.pos)) > k.bound() { return false; }
                            k.sdf_local(pose.inverse().apply(pt)) < 0.0
                        })
                    })
                }
            };
            self.sensors[i].on = on;
        }
        let stop: Vec<Id> = self.cmd.until.iter().filter(|(_, s)| self.sensor(s) == Some(true)).map(|(d, _)| *d).collect();
        for d in stop { self.cmd.run.remove(&d); self.cmd.until.remove(&d); }
    }

    pub fn snapshot(&self) -> Snapshot {
        let links = self.links.iter().enumerate().map(|(i, l)| LinkSnap { id: l.id, pose: self.link_pose(i) }).collect();
        let pieces = self.pieces.iter().map(|p| {
            let pose = match p.at {
                PieceWhere::Free => { let (pos, q) = self.engine.free_pose(p.jnt); Pose::new(pos, q) }
                _ => p.shown,
            };
            PieceSnap { kind: self.kinds[p.kind].id.clone(), pose, held: matches!(p.at, PieceWhere::Held { .. }), scored: p.at == PieceWhere::Scored }
        }).filter(|p| p.pose.pos[2] > -5.0).collect();
        let rollers = self.rollers.iter().map(|r| {
            let lp = self.link_pose(r.link);
            let s = &self.shafts[r.shaft];
            RollerSnap { id: r.id, center: lp.apply(r.center), axis: lp.rotate(r.axis), angle: s.angle * r.ratio, speed: s.omega * r.ratio }
        }).collect();
        let drives = self.drives.iter().map(|d| {
            let (position, speed, target) = match d.target {
                Target::Joint { joint, .. } => { let j = &self.joints[joint]; (self.engine.q(j.jnt), self.engine.qd(j.jnt), self.cmd.targets.get(&d.id).copied().unwrap_or(0.0)) }
                Target::Shaft { shaft } => (self.shafts[shaft].angle, self.shafts[shaft].omega, self.cmd.run.get(&d.id).copied().unwrap_or(0.0)),
            };
            DriveSnap { id: d.id, volts: d.out.volts, current: d.out.current, supply: d.out.supply, position, speed, target, limited: d.out.limited }
        }).collect();
        let contacts = self.contacts.iter().zip(&self.contact_rollers).map(|(c, r)| ContactSnap { point: c.point, normal: c.normal, force: c.normal_force, sliding: c.sliding, roller: *r }).collect();
        let (com, support, tip_margin) = self.tipping();
        let speed = self.root_joint.map_or(0.0, |j| { let (l, _) = self.engine.free_vel(j); (l[0] * l[0] + l[1] * l[1]).sqrt() });
        let phase = self.game.as_ref().map_or("Practice".to_string(), |g| self.matched.phase(&g.timing).to_string());
        Snapshot {
            time: self.time, links, pieces, rollers, drives, contacts, bus_voltage: self.bus.voltage, bus_current: self.bus.current,
            energy: self.bus.energy, brownout: self.bus.brownout, breaker_heat: self.bus.breaker_heat, com, support, tip_margin, speed,
            score: self.matched.score, match_time: self.matched.time, phase,
            sensors: self.sensors.iter().map(|x| SensorSnap { name: x.name.clone(), on: x.on, beam: match x.kind {
                SensorKind::Beam { link, from, to } => { let lp = self.link_pose(link); Some([lp.apply(from), lp.apply(to)]) }
                _ => None,
            } }).collect(),
        }
    }

    /// A drive's name, for reports.
    pub fn drive_name(&self, id: Id) -> String { self.drives.iter().find(|d| d.id == id).map_or(String::new(), |d| d.name.clone()) }

    /// Gains from the model for a joint drive (D12's button), at the current pose.
    pub fn gains_from_model(&self, drive: Id) -> Option<Control> {
        let d = self.drives.iter().find(|d| d.id == drive)?;
        let Target::Joint { joint, .. } = d.target else { return None };
        let jr = &self.joints[joint];
        let mut bias = self.engine.bias(jr.jnt);
        for (fj, k) in &jr.followers { bias += self.engine.bias(*fj) * k; }
        let pl = control::Plant { kind: jr.kind, q: self.engine.q(jr.jnt), qd: 0.0, inertia: d.inertia, bias, range: jr.range, bus: 12.0 };
        Some(control::gains_from_model(&pl, &d.motor))
    }

    /// each Concept holder: its mechanism, where it is, and whether it holds a piece
    pub fn holders(&self) -> Vec<(Id, V3, bool)> {
        self.mechs.iter().flat_map(|m| m.holders.iter().map(move |h| (m.id, h, m.full))).filter(|x| !x.2)
            .map(|(id, h, _)| (id, self.link_pose(h.link).mul(&h.pose).pos, h.piece.is_some())).collect()
    }

    /// each Concept pickup zone: its mechanism, its pose in the world and its size
    pub fn pickup_zones(&self) -> Vec<(Id, Pose, V3)> {
        self.mechs.iter().filter(|m| !m.full).filter_map(|m| m.pickup.as_ref().map(|p| (m.id, self.link_pose(p.0).mul(&p.1), p.2))).collect()
    }

    /// the chassis's pose in the world
    pub fn chassis_pose(&self) -> Pose { self.link_pose(self.links.iter().position(|l| l.id == CHASSIS).unwrap_or(0)) }

    /// pieces in the world, free or held (for the Bench's report)
    pub fn piece_positions(&self) -> Vec<(V3, bool)> {
        self.pieces.iter().map(|p| {
            let held = matches!(p.at, PieceWhere::Held { .. });
            let pos = if p.at == PieceWhere::Free { self.engine.free_pose(p.jnt).0 } else { p.shown.pos };
            (pos, held)
        }).collect()
    }

    /// the robot-frame x of the rearmost and frontmost rollers in contact mode (Bench)
    pub fn roller_x_range(&self) -> Option<(f64, f64)> {
        let xs: Vec<f64> = self.rollers.iter().filter(|r| r.full).map(|r| self.links[r.link].origin[0] + r.center[0]).collect();
        if xs.is_empty() { None } else { Some((xs.iter().cloned().fold(f64::INFINITY, f64::min), xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max))) }
    }

    pub fn roller_speed(&self, shaft: Id) -> f64 { self.shafts.iter().find(|s| s.id == shaft).map_or(0.0, |s| s.omega) }

    /// stator current of these drives together, A
    pub fn snapshot_drive_currents(&self, ids: &[Id]) -> f64 { self.drives.iter().filter(|d| ids.contains(&d.id)).map(|d| d.out.current).sum() }
    /// the squeeze on pieces this step: the rollers' normal forces together, N
    pub fn contact_force(&self) -> f64 { self.contacts.iter().zip(&self.contact_rollers).filter(|(_, r)| **r != 0).map(|(c, _)| c.normal_force).sum() }
    /// (slices touching, slices slipping) this step
    pub fn contact_counts(&self) -> (usize, usize) { (self.contacts.len(), self.contacts.iter().filter(|c| c.sliding).count()) }
    /// a piece's turn about the vertical (its long axis's heading, for a tube)
    pub fn piece_yaw(&self, i: usize) -> f64 {
        let Some(p) = self.pieces.get(i) else { return 0.0 };
        let q = if p.at == PieceWhere::Free { self.engine.free_pose(p.jnt).1 } else { p.shown.quat };
        let ax = qrot(q, [0.0, 0.0, 1.0]);
        let yaw = ax[0].atan2(ax[1]);
        // a tube lying across the robot (along y) has no turn; either end forward is the same
        let y = yaw.rem_euclid(std::f64::consts::PI);
        if y > std::f64::consts::FRAC_PI_2 { y - std::f64::consts::PI } else { y }
    }
}
