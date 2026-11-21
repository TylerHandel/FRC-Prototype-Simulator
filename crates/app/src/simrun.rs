//! The simulation on its own thread (PLAN.md D15). The interface sends it messages (load this
//! project, play, drive, press a button); it steps in real time (or slower or faster) and leaves
//! the latest snapshot and a recording where the interface can read them.
use bevy::prelude::*;
use library::Library;
use model::{Button, Id, Project};
use sim::world::{Scene, World};
use sim::Snapshot;
use std::collections::VecDeque;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::time::Instant;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SceneKind { Carpet, Field, Bench }

pub enum Msg {
    /// Build the world for this project; keep the state across if the scene is the same (an edit)
    Load { project: Box<Project>, kind: SceneKind, keep: bool },
    Play(bool),
    Speed(f64),
    StepOnce,
    Reset,
    Drive { v: [f64; 3], field_relative: bool },
    Button(Button, bool),
    Run(Id, Option<f64>),
    Target(Id, f64),
    StartMatch,
    Release(Id),
}

pub struct Shared {
    pub snap: Option<Snapshot>,
    /// a recording of the run, a sample every 10 ms of sim time (D16)
    pub history: VecDeque<Snapshot>,
    pub error: Option<String>,
    pub warnings: Vec<String>,
    pub playing: bool,
    pub speed: f64,
    /// sim seconds per wall second, over the last half second
    pub realtime: f64,
    pub kind: SceneKind,
    /// bumped each time a world is built
    pub generation: u64,
    pub bench_throws: u32,
    pub log: Vec<sim::game::Event>,
    pub drive_names: Vec<(Id, String)>,
    pub dt: f64,
}

#[derive(Resource, Clone)]
pub struct Sim { pub tx: Sender<Msg>, pub shared: Arc<Mutex<Shared>> }

impl Sim {
    pub fn send(&self, m: Msg) { let _ = self.tx.send(m); }
    pub fn snap(&self) -> Option<Snapshot> { self.shared.lock().ok().and_then(|s| s.snap.clone()) }
    pub fn playing(&self) -> bool { self.shared.lock().is_ok_and(|s| s.playing) }
}

pub struct SimPlugin;

impl Plugin for SimPlugin {
    fn build(&self, app: &mut App) {
        let (tx, rx) = channel();
        let shared = Arc::new(Mutex::new(Shared {
            snap: None, history: VecDeque::new(), error: None, warnings: vec![], playing: true, speed: 1.0, realtime: 1.0,
            kind: SceneKind::Carpet, generation: 0, bench_throws: 0, log: vec![], drive_names: vec![], dt: 0.002,
        }));
        let s2 = shared.clone();
        std::thread::Builder::new().name("simulation".into()).spawn(move || run(rx, s2)).expect("simulation thread");
        app.insert_resource(Sim { tx, shared });
    }
}

const HISTORY_EVERY: f64 = 0.01;
const HISTORY_MAX: usize = 30_000;

fn build(project: &Project, lib: &Library, kind: SceneKind, throw: u32) -> anyhow::Result<World> {
    match kind {
        SceneKind::Carpet => World::new(project, lib, Scene::carpet()),
        SceneKind::Field => {
            let g = sim::game::GamePack::builtin(&project.game).ok_or_else(|| anyhow::anyhow!("no game pack '{}'", project.game))?;
            World::new(project, lib, Scene::field(&g))
        }
        SceneKind::Bench => {
            let mut scene = Scene::bench();
            let probe = World::new(project, lib, scene.clone())?;
            let bumper = project.robot.frame.length / 2.0 + project.robot.frame.bumper_thickness;
            let front = probe.roller_x_range().map_or(bumper, |r| r.1).max(bumper);
            drop(probe);
            let mut rng = sim::bench::Rng::new(project.bench.seed + throw as u64);
            scene.pieces.push(sim::bench::spawn(project, lib, front, &mut rng)?);
            let mut w = World::new(project, lib, scene)?;
            for t in sim::bench::default_run(project) {
                match project.robot.drive(t.drive).map(|d| &d.target) {
                    Some(model::DriveTarget::Shaft { .. }) => { w.cmd.run.insert(t.drive, t.value); }
                    Some(model::DriveTarget::Joint { .. }) => { w.cmd.targets.insert(t.drive, t.value); }
                    None => {}
                }
            }
            Ok(w)
        }
    }
}

fn run(rx: Receiver<Msg>, shared: Arc<Mutex<Shared>>) {
    let lib = {
        let mut l = Library::builtin();
        let _ = l.load_overlay(&crate::settings::app_dir().join("library"));
        l
    };
    let mut world: Option<World> = None;
    let mut project: Option<Project> = None;
    let mut kind = SceneKind::Carpet;
    let mut playing = true;
    let mut speed = 1.0;
    let mut last_hist = -1.0;
    let mut wall = Instant::now();
    let mut sim_debt = 0.0f64;
    let mut rt_wall = Instant::now();
    let mut rt_sim = 0.0;
    let mut throw = 0u32;
    let mut step_once = false;
    let mut pending: Vec<Msg> = vec![];
    loop {
        // take every message waiting (or wait a little when there's nothing to run)
        let timeout = if world.is_some() && playing { std::time::Duration::from_millis(0) } else { std::time::Duration::from_millis(20) };
        match rx.recv_timeout(timeout) {
            Ok(m) => { pending.push(m); while let Ok(m) = rx.try_recv() { pending.push(m); } }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
            Err(_) => return,
        }
        for m in pending.drain(..) {
            match m {
                Msg::Load { project: p, kind: k, keep } => {
                    let same = keep && k == kind && world.is_some();
                    if k != kind { throw = 0; }
                    match build(&p, &lib, k, throw) {
                        Ok(mut w) => {
                            if same { if let Some(old) = &world { w.carry_from(old); } }
                            let mut s = shared.lock().unwrap();
                            s.error = None;
                            s.warnings = w.warnings.clone();
                            s.kind = k;
                            s.generation += 1;
                            s.drive_names = p.robot.drives.iter().map(|d| (d.id, d.name.clone())).collect();
                            s.dt = w.dt;
                            if !same { s.history.clear(); s.log.clear(); last_hist = -1.0; }
                            s.snap = Some(w.snapshot());
                            world = Some(w);
                            project = Some(*p);
                            kind = k;
                        }
                        Err(e) => { shared.lock().unwrap().error = Some(format!("{e}")); }
                    }
                    wall = Instant::now(); sim_debt = 0.0;
                }
                Msg::Play(b) => { playing = b; wall = Instant::now(); sim_debt = 0.0; }
                Msg::Speed(x) => { speed = x; }
                Msg::StepOnce => { step_once = true; }
                Msg::Reset => {
                    if let Some(p) = &project {
                        throw += 1;
                        if let Ok(w) = build(p, &lib, kind, throw) {
                            let mut s = shared.lock().unwrap();
                            s.history.clear(); s.log.clear(); s.generation += 1; s.snap = Some(w.snapshot());
                            world = Some(w); last_hist = -1.0;
                        }
                    }
                }
                Msg::Drive { v, field_relative } => if let Some(w) = &mut world { w.cmd.drive = v; w.cmd.field_relative = field_relative; },
                Msg::Button(b, down) => if let Some(w) = &mut world { w.button(b, down); },
                Msg::Run(d, v) => if let Some(w) = &mut world { match v { Some(v) => { w.cmd.run.insert(d, v); } None => { w.cmd.run.remove(&d); } } },
                Msg::Target(d, v) => if let Some(w) = &mut world { w.cmd.targets.insert(d, v); },
                Msg::StartMatch => if let Some(w) = &mut world { w.start_match(); shared.lock().unwrap().log.clear(); },
                Msg::Release(m) => if let Some(w) = &mut world { w.cmd.release.push(m); },
            }
        }
        let Some(w) = &mut world else { continue };
        // how much sim time is owed: wall time times speed, at most a tenth of a second at once
        let now = Instant::now();
        let dtw = now.duration_since(wall).as_secs_f64();
        wall = now;
        if playing { sim_debt = (sim_debt + dtw * speed).min(0.1); } else { sim_debt = 0.0; }
        let mut n = 0;
        let budget = Instant::now();
        while (sim_debt >= w.dt || step_once) && budget.elapsed().as_secs_f64() < 0.03 {
            w.step();
            sim_debt -= w.dt;
            step_once = false;
            n += 1;
            if w.time - last_hist >= HISTORY_EVERY {
                last_hist = w.time;
                let snap = w.snapshot();
                let mut s = shared.lock().unwrap();
                s.history.push_back(snap);
                if s.history.len() > HISTORY_MAX { s.history.pop_front(); }
            }
            if !w.engine.finite() { playing = false; shared.lock().unwrap().error = Some("The simulation blew up. Reset it (R), and look for a joint without limits or a very stiff wheel.".into()); break; }
        }
        // the Bench throws again when a throw is done
        if kind == SceneKind::Bench && w.time > project.as_ref().map_or(1.5, |p| p.bench.duration) + 0.4 {
            throw += 1;
            if let Some(p) = &project { if let Ok(nw) = build(p, &lib, kind, throw) { *w = nw; let mut s = shared.lock().unwrap(); s.bench_throws = throw; s.generation += 1; s.history.clear(); last_hist = -1.0; } }
        }
        if n > 0 {
            rt_sim += n as f64 * w.dt;
            let snap = w.snapshot();
            let mut s = shared.lock().unwrap();
            s.snap = Some(snap);
            s.playing = playing;
            s.speed = speed;
            s.log = w.matched.log.clone();
            let el = rt_wall.elapsed().as_secs_f64();
            if el > 0.5 { s.realtime = rt_sim / el; rt_wall = Instant::now(); rt_sim = 0.0; }
        } else {
            let mut s = shared.lock().unwrap();
            s.playing = playing;
            s.speed = speed;
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
}
