//! The Bench (PLAN.md M1): the robot held still, a piece sent at it as if the robot drove onto
//! it, and what happened: did it come in, how long, what current, how hard the squeeze, did the
//! rollers bog down. Wheel choices run side by side (D19) and sweeps run in parallel (D15).
use crate::world::{Scene, Spawn, World};
use library::Library;
use model::math::*;
use model::{Bench, DriveTarget, Id, Project};
use rayon::prelude::*;
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct Sample { pub t: f64, pub current: f64, pub squeeze: f64, pub roller_rpm: f64, pub piece_x: f64, pub bus: f64, pub sliding: f64 }

#[derive(Debug, Clone, Serialize)]
pub struct Trial {
    pub label: String,
    /// the piece came in past the rollers (or was picked up)
    pub acquired: bool,
    /// when, s after the start
    pub time_in: Option<f64>,
    /// pushed back out, or never reached
    pub rejected: bool,
    pub peak_current: f64,
    /// the rollers' mean current while touching the piece, A
    pub mean_current: f64,
    pub peak_squeeze: f64,
    /// the mean squeeze while touching, N
    pub mean_squeeze: f64,
    pub min_roller_rpm: f64,
    pub start_roller_rpm: f64,
    pub energy: f64,
    pub min_bus: f64,
    /// the share of contact time spent slipping
    pub slip: f64,
    /// the piece's turn about the vertical at the end, deg
    pub final_yaw: f64,
    pub samples: Vec<Sample>,
    pub warnings: Vec<String>,
}

/// What the Bench runs: from `bench.run` if set, else every roller drive at 10 V and joints held.
pub fn default_run(p: &Project) -> Vec<model::PresetTarget> {
    if !p.bench.run.is_empty() { return p.bench.run.clone(); }
    p.robot.drives.iter().filter(|d| matches!(d.target, DriveTarget::Shaft { .. })).map(|d| {
        let v = p.robot.controls.bindings.iter().find_map(|b| match b.action { model::Action::Run { drive, value, .. } if drive == d.id && value > 0.0 => Some(value), _ => None }).unwrap_or(10.0);
        model::PresetTarget { drive: d.id, value: v }
    }).collect()
}

/// A tiny repeatable random source (the same seed, the same throws).
pub struct Rng(u64);
impl Rng {
    pub fn new(seed: u64) -> Self { Rng(seed.wrapping_mul(0x9E3779B97F4A7C15) | 1) }
    pub fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13; self.0 ^= self.0 >> 7; self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    /// -1..1
    pub fn sym(&mut self) -> f64 { 2.0 * self.next() - 1.0 }
}

/// The piece's spawn for one throw.
pub fn spawn(p: &Project, lib: &Library, front_x: f64, rng: &mut Rng) -> anyhow::Result<Spawn> {
    let b: &Bench = &p.bench;
    let lp = lib.piece(&b.piece).ok_or_else(|| anyhow::anyhow!("no piece {}", b.piece))?;
    let k = crate::piece::PieceKind::from_library(lp);
    let (h, lay) = k.resting();
    let yaw = (b.yaw + b.yaw_spread * rng.sym()).to_radians();
    let y = b.offset + b.offset_spread * rng.sym();
    let reach = match k.shape { crate::piece::Shape::Sphere { r } => r, crate::piece::Shape::Tube { r, .. } | crate::piece::Shape::Cylinder { r, .. } => r, crate::piece::Shape::Box { half } => half[0] };
    let pos = [front_x + b.distance + reach, y, h + 0.001];
    let quat = qmul(axis_angle([0.0, 0.0, 1.0], yaw), lay);
    let lin = [-b.speed, 0.0, 0.0];
    // rolling toward the robot (a ball or a pipe on its side rolls, it doesn't slide)
    let ang_world = match k.shape { crate::piece::Shape::Box { .. } => [0.0; 3], _ => [0.0, -b.speed / reach, 0.0] };
    let ang_body = qrot(qconj(quat), ang_world);
    Ok(Spawn { piece: b.piece.clone(), pos, quat, lin, ang: ang_body })
}

/// One throw at the robot, with `wheel` on every roller if given.
pub fn trial(p: &Project, lib: &Library, wheel: Option<&str>, seed: u64) -> anyhow::Result<Trial> {
    let mut scene = Scene::bench();
    scene.wheel_override = wheel.map(|w| w.to_string());
    // where the front of the robot is: its frontmost roller, or its bumper
    let probe = World::new(p, lib, scene.clone())?;
    let bumper = p.robot.frame.length / 2.0 + p.robot.frame.bumper_thickness;
    let (rmin, rmax) = probe.roller_x_range().unwrap_or((bumper, bumper));
    drop(probe);
    let mut rng = Rng::new(seed);
    let sp = spawn(p, lib, rmax.max(bumper), &mut rng)?;
    let start_x = sp.pos[0];
    scene.pieces.push(sp);
    let mut w = World::new(p, lib, scene)?;
    let run = default_run(p);
    for t in &run {
        match p.robot.drive(t.drive).map(|d| &d.target) {
            Some(DriveTarget::Shaft { .. }) => { w.cmd.run.insert(t.drive, t.value); }
            Some(DriveTarget::Joint { .. }) => { w.cmd.targets.insert(t.drive, t.value); }
            None => {}
        }
    }
    let shafts: Vec<Id> = run.iter().filter_map(|t| match p.robot.drive(t.drive).map(|d| &d.target) { Some(DriveTarget::Shaft { shaft }) => Some(*shaft), _ => None }).collect();
    let roller_drives: Vec<Id> = run.iter().filter(|t| matches!(p.robot.drive(t.drive).map(|d| &d.target), Some(DriveTarget::Shaft { .. }))).map(|t| t.drive).collect();
    // spin up before the piece arrives: hold the piece still for a moment
    let label = wheel.map(|x| lib.wheel(x).map_or(x.to_string(), |w| w.name.clone())).unwrap_or_else(|| "As designed".into());
    let mut tr = Trial { label, acquired: false, time_in: None, rejected: false, peak_current: 0.0, mean_current: 0.0, peak_squeeze: 0.0, mean_squeeze: 0.0, min_roller_rpm: f64::INFINITY, start_roller_rpm: 0.0,
        energy: 0.0, min_bus: f64::INFINITY, slip: 0.0, final_yaw: 0.0, samples: vec![], warnings: w.warnings.clone() };
    let steps = (p.bench.duration / w.dt).ceil() as usize;
    let every = ((0.005 / w.dt).round() as usize).max(1);
    let (mut contact_steps, mut slip_steps) = (0usize, 0usize);
    // when the piece last went in past the rearmost roller (and stayed)
    let mut first_in: Option<f64> = None;
    let (mut sum_i, mut sum_f, mut n_touch) = (0.0, 0.0, 0usize);
    let rpm = |w: &World| shafts.first().map_or(0.0, |s| w.roller_speed(*s).abs() * 60.0 / std::f64::consts::TAU);
    for i in 0..steps {
        w.step();
        let snap_needed = i % every == 0 || i + 1 == steps;
        let current: f64 = w.snapshot_drive_currents(&roller_drives);
        let squeeze = w.contact_force();
        let (cn, sn) = w.contact_counts();
        if cn > 0 { contact_steps += 1; if sn * 2 > cn { slip_steps += 1; } }
        if squeeze > 0.0 {
            sum_i += current; sum_f += squeeze; n_touch += 1;
            tr.peak_current = tr.peak_current.max(current);
            tr.peak_squeeze = tr.peak_squeeze.max(squeeze);
            tr.min_roller_rpm = tr.min_roller_rpm.min(rpm(&w));
        }
        tr.min_bus = tr.min_bus.min(w.bus.voltage);
        // the rollers' speed just before the piece arrives
        if squeeze <= 0.0 && n_touch == 0 { tr.start_roller_rpm = rpm(&w); }
        let pieces = w.piece_positions();
        let (pos, held) = pieces.first().copied().unwrap_or(([0.0; 3], false));
        let inside = held || pos[0] < rmin - 0.01;
        if inside && first_in.is_none() { first_in = Some(w.time); }
        if !inside && pos[0] > rmin + 0.02 { first_in = None; }
        if snap_needed {
            tr.samples.push(Sample { t: w.time, current, squeeze, roller_rpm: rpm(&w), piece_x: pos[0], bus: w.bus.voltage, sliding: if cn > 0 { sn as f64 / cn as f64 } else { 0.0 } });
        }
        if !w.engine.finite() { tr.warnings.push("blew up".into()); break; }
    }
    tr.energy = w.bus.energy;
    if n_touch > 0 { tr.mean_current = sum_i / n_touch as f64; tr.mean_squeeze = sum_f / n_touch as f64; }
    if !tr.min_roller_rpm.is_finite() { tr.min_roller_rpm = tr.start_roller_rpm; }
    tr.slip = if contact_steps > 0 { slip_steps as f64 / contact_steps as f64 } else { 0.0 };
    // in: it ended up behind the rearmost roller (or held); out: it ended up back in front
    let last = w.piece_positions();
    if let Some((pos, held)) = last.first() {
        tr.acquired = *held || pos[0] < rmin - 0.01;
        tr.time_in = if tr.acquired { first_in } else { None };
        tr.rejected = !tr.acquired && (pos[0] > rmax + 0.02 || pos[0] > start_x - 0.05);
    }
    tr.final_yaw = w.piece_yaw(0).to_degrees();
    Ok(tr)
}

/// The same throw with each wheel (D19), side by side, in parallel.
pub fn compare(p: &Project, lib: &Library, wheels: &[String]) -> Vec<anyhow::Result<Trial>> {
    let seed = p.bench.seed;
    wheels.par_iter().map(|w| trial(p, lib, Some(w), seed)).collect()
}

/// A sweep axis: what changes between runs.
#[derive(Debug, Clone)]
pub enum Axis {
    Wheel(Vec<String>),
    Speed(Vec<f64>),
    StatorLimit(Vec<f64>),
    Yaw(Vec<f64>),
    Volts(Vec<f64>),
    /// a template number, by name (e.g. "height" on an intake): the mechanism and its values
    Template { mechanism: Id, field: String, values: Vec<f64> },
}

impl Axis {
    pub fn len(&self) -> usize {
        match self { Axis::Wheel(v) => v.len(), Axis::Speed(v) | Axis::StatorLimit(v) | Axis::Yaw(v) | Axis::Volts(v) => v.len(), Axis::Template { values, .. } => values.len() }
    }
    pub fn is_empty(&self) -> bool { self.len() == 0 }
    /// what it changes, for a heading
    pub fn name(&self) -> String {
        match self {
            Axis::Wheel(_) => "wheel".into(), Axis::Speed(_) => "speed".into(), Axis::StatorLimit(_) => "current limit".into(),
            Axis::Yaw(_) => "turn".into(), Axis::Volts(_) => "roller volts".into(), Axis::Template { field, .. } => field.clone(),
        }
    }
    pub fn label(&self, i: usize) -> String {
        match self {
            Axis::Wheel(v) => v[i].clone(),
            Axis::Speed(v) => format!("{:.2} m/s", v[i]),
            Axis::StatorLimit(v) => format!("{:.0} A", v[i]),
            Axis::Yaw(v) => format!("{:.0} deg", v[i]),
            Axis::Volts(v) => format!("{:.1} V", v[i]),
            Axis::Template { field, values, .. } => {
                let v = values[i];
                if field.contains("deg") { format!("{field} {v:.0} deg") }
                else if field.contains("ratio") { format!("{field} {v:.1}:1") }
                else if field.contains("mass") { format!("{field} {:.1} lb", v / library::LB) }
                else if matches!(field.as_str(), "motors" | "stages" | "rollers" | "wheels_per_roller") { format!("{field} {v:.0}") }
                else { format!("{field} {:.2} in", v / library::IN) }
            }
        }
    }
    fn apply(&self, i: usize, p: &mut Project, wheel: &mut Option<String>) {
        match self {
            Axis::Wheel(v) => *wheel = Some(v[i].clone()),
            Axis::Speed(v) => p.bench.speed = v[i],
            Axis::Yaw(v) => p.bench.yaw = v[i],
            Axis::StatorLimit(v) => for d in &mut p.robot.drives { if matches!(d.target, DriveTarget::Shaft { .. }) { d.stator_limit = v[i]; } },
            Axis::Volts(v) => { let mut run = default_run(p); for t in &mut run { if matches!(p.robot.drive(t.drive).map(|d| &d.target), Some(DriveTarget::Shaft { .. })) { t.value = v[i]; } } p.bench.run = run; }
            Axis::Template { mechanism, field, values } => {
                if let Some(t) = p.robot.mechanism(*mechanism).and_then(|m| m.template.clone()) {
                    let mut v = serde_json::to_value(&t).unwrap();
                    if let Some(obj) = v.pointer_mut("/params").and_then(|x| x.as_object_mut()).and_then(|o| o.values_mut().next()).and_then(|x| x.as_object_mut()) {
                        obj.insert(field.clone(), serde_json::json!(values[i]));
                    }
                    if let Ok(t2) = serde_json::from_value(v) { let _ = model::templates::rebuild(&mut p.robot, *mechanism, t2); }
                }
            }
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Cell {
    pub index: Vec<usize>, pub labels: Vec<String>, pub trials: usize, pub acquired: usize, pub mean_time: f64, pub mean_peak_current: f64, pub mean_peak_squeeze: f64,
    /// the rollers' mean current and squeeze while touching, averaged over the throws
    pub mean_current: f64, pub mean_squeeze: f64,
    /// energy from the battery per throw, J
    pub mean_energy: f64,
}

impl Cell {
    /// the share of throws taken in
    pub fn success(&self) -> f64 { if self.trials == 0 { 0.0 } else { self.acquired as f64 / self.trials as f64 } }
}

/// A sweep as CSV: one row per combination, with its labels and outcomes.
pub fn sweep_csv(axes: &[Axis], cells: &[Cell]) -> String {
    let mut s = String::new();
    for a in axes { s.push_str(&format!("{},", a.name())); }
    s.push_str("throws,taken in,success,mean time in (s),mean current (A),peak current (A),mean squeeze (N),peak squeeze (N),energy (J)\n");
    for c in cells {
        for l in &c.labels { s.push_str(&format!("\"{}\",", l.replace('"', "'"))); }
        s.push_str(&format!("{},{},{:.3},{:.3},{:.2},{:.2},{:.1},{:.1},{:.2}\n", c.trials, c.acquired, c.success(), c.mean_time, c.mean_current, c.mean_peak_current, c.mean_squeeze, c.mean_peak_squeeze, c.mean_energy));
    }
    s
}

/// Every combination of the axes, `throws` random throws each (yaw and offset spread from the
/// Bench), all in parallel.
pub fn sweep(p: &Project, lib: &Library, axes: &[Axis], throws: usize) -> Vec<Cell> {
    let mut combos: Vec<Vec<usize>> = vec![vec![]];
    for a in axes {
        combos = combos.into_iter().flat_map(|c| (0..a.len()).map(move |i| { let mut c = c.clone(); c.push(i); c })).collect();
    }
    let jobs: Vec<(Vec<usize>, usize)> = combos.iter().flat_map(|c| (0..throws.max(1)).map(move |t| (c.clone(), t))).collect();
    let results: Vec<(Vec<usize>, Option<Trial>)> = jobs.par_iter().map(|(c, t)| {
        let mut q = p.clone();
        let mut wheel = None;
        for (ai, a) in axes.iter().enumerate() { a.apply(c[ai], &mut q, &mut wheel); }
        (c.clone(), trial(&q, lib, wheel.as_deref(), p.bench.seed + *t as u64).ok())
    }).collect();
    combos.into_iter().map(|c| {
        let mine: Vec<&Trial> = results.iter().filter(|(k, _)| *k == c).filter_map(|(_, t)| t.as_ref()).collect();
        let n = mine.len().max(1) as f64;
        let ins: Vec<f64> = mine.iter().filter_map(|t| t.time_in).collect();
        Cell {
            labels: c.iter().enumerate().map(|(ai, i)| axes[ai].label(*i)).collect(),
            index: c, trials: mine.len(), acquired: mine.iter().filter(|t| t.acquired).count(),
            mean_time: if ins.is_empty() { f64::NAN } else { ins.iter().sum::<f64>() / ins.len() as f64 },
            mean_peak_current: mine.iter().map(|t| t.peak_current).sum::<f64>() / n,
            mean_peak_squeeze: mine.iter().map(|t| t.peak_squeeze).sum::<f64>() / n,
            mean_current: mine.iter().map(|t| t.mean_current).sum::<f64>() / n,
            mean_squeeze: mine.iter().map(|t| t.mean_squeeze).sum::<f64>() / n,
            mean_energy: mine.iter().map(|t| t.energy).sum::<f64>() / n,
        }
    }).collect()
}
