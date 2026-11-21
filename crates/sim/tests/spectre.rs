//! M5 gate on an imported arm robot (PLAN.md "The test robot": 2910's Spectre). Another team's CAD
//! can't be in the repository, and nor can anything taken from it, so these run only where
//! `FPS_SPECTRE` names a project made from it, and read every pose and place from that project:
//!
//!     fps propose spectre.cadpack --bom bom.json --second scoring.cadpack --forward -Y --save <dir>
//!     FPS_SPECTRE=<dir> cargo test -p sim --release --test spectre -- --ignored --nocapture
use library::Library;
use model::math::*;
use model::*;
use sim::world::{Scene, Spawn, World};

/// The imported robot, as the importer rigged it (its motor and gear guesses included).
fn robot() -> Option<Project> {
    let dir = std::env::var("FPS_SPECTRE").ok()?;
    Project::from_json(&std::fs::read_to_string(std::path::Path::new(&dir).join("robot.json")).ok()?).ok()
}

/// Every joint at the far end of its travel (the importer's "out" presets): the second pose.
fn reach_out(r: &Robot) -> Vec<(Id, f64)> {
    r.joints.iter().filter_map(|j| {
        let [lo, hi] = j.range?;
        let d = r.drives.iter().find(|d| matches!(d.target, DriveTarget::Joint { joint, .. } if joint == j.id))?;
        Some((d.id, if lo.abs() > hi.abs() { lo } else { hi }))
    }).collect()
}

/// The link with rollers farthest out along the joints: the end effector.
fn end_effector(r: &Robot) -> Id {
    let depth = |mut l: Id| { let mut n = 0; while let Some(j) = r.joint_of(l) { l = j.parent; n += 1; } n };
    r.shafts.iter().map(|s| s.link).max_by_key(|l| depth(*l)).expect("no rollers")
}

/// the end effector's wheels: a speed loop (rad/s at the shaft), in, held still, and out
const IN: f64 = 40.0;
const HOLD: f64 = 1e-6;
const OUT: f64 = -30.0;

fn wrap(a: f64) -> f64 { (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI }
fn yaw_of(q: Quat) -> f64 { let x = qrot(q, [1.0, 0.0, 0.0]); x[1].atan2(x[0]) }
fn mm(v: V3) -> [f64; 3] { v.map(|x| (x * 1000.0).round() / 1000.0) }

/// drives field-relative to (x, y, yaw), no faster than `fast` m/s, and stops; true when it got there
fn go(w: &mut World, to: [f64; 3], limit: f64, fast: f64) -> bool {
    w.cmd.field_relative = true;
    let t0 = w.time;
    while w.time - t0 < limit {
        let c = w.chassis_pose();
        let (ex, ey) = (to[0] - c.pos[0], to[1] - c.pos[1]);
        let d = ex.hypot(ey);
        let eyaw = wrap(to[2] - yaw_of(c.quat));
        if d < 0.01 && eyaw.abs() < 0.01 { w.cmd.drive = [0.0; 3]; for _ in 0..300 { w.step(); } return true; }
        let v = (2.0 * d).min(fast);
        w.cmd.drive = [if d > 1e-6 { ex / d * v } else { 0.0 }, if d > 1e-6 { ey / d * v } else { 0.0 }, (4.0 * eyaw).clamp(-3.0, 3.0)];
        w.step();
    }
    w.cmd.drive = [0.0; 3];
    false
}

/// M5 gate, CORAL: fed lengthwise into the end effector's mouth, pulled in by its own wheels
/// until a beam break trips, held on a speed loop, carried to the REEF, raised (clear of the
/// REEF, on a smooth profile), brought in over an L4 branch, pushed out, and scored where it
/// lands. No assist anywhere: the field counts a CORAL that comes to rest on a branch.
#[test]
#[ignore]
fn arm_robot_scores_coral_on_l4_by_physics() {
    let Some(mut p) = robot() else { eprintln!("FPS_SPECTRE not set"); return };
    let lib = Library::builtin();
    p.force_fidelity = Some(Fidelity::Full);
    let r = p.robot.clone();
    let ee = end_effector(&r);
    // each side's wheels, and which way that side turns to pull a piece in
    let sides: Vec<(Id, f64)> = r.shafts.iter().filter(|s| s.link == ee).map(|s| {
        let y = r.rollers.iter().find(|x| x.shaft == s.id).unwrap().center[1];
        (r.drives.iter().find(|d| matches!(d.target, DriveTarget::Shaft { shaft } if shaft == s.id)).unwrap().id, y.signum())
    }).collect();
    assert_eq!(sides.len(), 2, "wheels on both sides of the end effector");
    // the mouth: between the two sides' wheels, facing along their surface motion there
    let wheels: Vec<&Roller> = r.rollers.iter().filter(|x| r.shaft(x.shaft).is_some_and(|s| s.link == ee)).collect();
    let mid = scale(wheels.iter().fold([0.0; 3], |a, x| add(a, x.center)), 1.0 / wheels.len() as f64);
    let mouth = norm(cross(wheels[0].axis, [0.0, 1.0, 0.0]));
    let coral = sim::piece::PieceKind::from_library(lib.piece("coral-2025").unwrap());
    let sim::piece::Shape::Tube { half, .. } = coral.shape else { panic!() };
    // a beam break a CORAL's length in from the wheels, less a little: it trips once the CORAL's
    // inner end is well past the wheels
    let beam = sub(mid, scale(mouth, 2.0 * half - 0.08));
    p.robot.sensors.push(Sensor::BeamBreak { name: "Coral in".into(), link: ee, from: add(beam, [0.0, -0.03, 0.0]), to: add(beam, [0.0, 0.03, 0.0]) });
    for (d, _) in &sides { if let Some(x) = p.robot.drive_mut(*d) { x.control = Control::Velocity { kp: 0.3, ki: 6.0, ks: 0.0, kv: 0.0 }; } }
    let g = sim::game::GamePack::builtin("reefscape-2025").unwrap();
    let branch = std::env::var("FPS_BRANCH").unwrap_or_else(|_| "Blue L4 1l".into());
    let seat = g.zones.iter().find(|z| z.name == branch).unwrap().seat.clone().unwrap().pos;
    let mut scene = Scene::field(&g);
    scene.game_pieces = false;
    let start = [-1.5, seat[1], std::f64::consts::PI];
    scene.start = start;
    // the CORAL fed in lengthwise, its inner end just past the wheels, coming in at 0.5 m/s (as
    // from a station's chute)
    let rq = axis_angle([0.0, 0.0, 1.0], start[2]);
    let to_field = |v: V3| add([start[0], start[1], 0.0], qrot(rq, v));
    scene.pieces.push(Spawn { piece: "coral-2025".into(), pos: to_field(add(mid, scale(mouth, half - 0.05))), quat: qmul(rq, quat_between([0.0, 0.0, 1.0], mouth)), lin: qrot(rq, scale(mouth, -0.5)), ang: [0.0; 3] });
    let mut w = World::new(&p, &lib, scene).unwrap();

    // in, until the beam breaks
    for (d, side) in &sides { w.cmd.run.insert(*d, IN * side); w.cmd.until.insert(*d, "Coral in".into()); }
    let mut k = 0;
    while w.time < 2.0 && w.sensor("Coral in") != Some(true) {
        w.step();
        k += 1;
        if k % 200 == 0 && std::env::var("FPS_TRACE").is_ok() { println!("t {:.2} CORAL {:?} squeeze {:.0} N contacts {:?}", w.time, mm(w.piece_positions()[0].0), w.contact_force(), w.piece_contacts(0)); }
    }
    assert_eq!(w.sensor("Coral in"), Some(true), "pulled in to the beam");
    println!("pulled in by {:.2} s, squeeze {:.0} N", w.time, w.contact_force());
    for (d, side) in &sides { w.cmd.until.remove(d); w.cmd.run.insert(*d, HOLD * side); }
    for _ in 0..500 { w.step(); }

    // to the REEF, a robot's length back; raised there, smoothly (flung at full motor
    // acceleration, a CORAL held by the wheels' edges slips out)
    let back = [seat[0] + 1.2, seat[1], std::f64::consts::PI];
    assert!(go(&mut w, back, 8.0, 1.5), "reached the REEF at {:?}", w.chassis_pose().pos);
    let out = reach_out(&r);
    let t0 = w.time;
    while w.time - t0 < 4.0 {
        let f = ((w.time - t0) / 2.5).min(1.0);
        let f = f * f * (3.0 - 2.0 * f);
        for (d, v) in &out { w.cmd.targets.insert(*d, v * f); }
        w.step();
    }
    let held = w.piece_positions()[0].0;
    for d in w.snapshot().drives.iter().filter(|d| out.iter().any(|o| o.0 == d.id)) { println!("joint drive {} at {:.3} for {:.3}, {:.0} A", d.id, d.position, d.target, d.current); }
    assert!(held[2] > seat[2], "still holding it, raised: {:?}", mm(held));
    // in: until the CORAL is over the branch, or the bumper meets the REEF (the branch's seat is
    // 10 cm in from the REEF's face); pushing it out carries it the rest of the way
    let ahead = w.chassis_pose().pos[0] - held[0];
    let bumper = r.frame.length / 2.0 + r.frame.bumper_thickness;
    let stand = [seat[0] + ahead.max(0.1 + bumper + 0.015), seat[1], std::f64::consts::PI];
    let t0 = w.time;
    // the last half metre slowly, as a driver lines up: the CORAL is held only by the wheels
    let ok = go(&mut w, stand, 8.0, 0.4);
    println!("drove in for {:.2} s; CORAL at {:?}", w.time - t0, mm(w.piece_positions()[0].0));
    assert!(ok, "in to the REEF at {:?}", w.chassis_pose().pos);
    println!("over the branch: CORAL at {:?}, the seat at {:?}", mm(w.piece_positions()[0].0), mm(seat));
    for (d, side) in &sides { w.cmd.run.insert(*d, OUT * side); }
    let t0 = w.time;
    while w.time - t0 < 3.0 { w.step(); }
    println!("let go: CORAL at {:?}; {:?}", mm(w.piece_positions()[0].0), w.matched.log.iter().map(|e| (&e.what, e.points)).collect::<Vec<_>>());
    assert!(w.matched.log.iter().any(|e| e.what.starts_with("scored Blue L4")), "scored on L4 by itself");
}
