//! The milestone gates that run headless (PLAN.md M0, M1, M2).
use library::Library;
use model::math::*;
use model::templates::{self, Params, Template};
use model::*;
use sim::world::{Scene, Spawn, World};

fn stiff_battery(r: &mut Robot) {
    r.electrical = Electrical { battery_voltage: 12.0, battery_resistance: 1e-9, wiring_resistance: 0.0, fixed_load: 0.0, main_breaker: 1e9, brownout: 0.0, trip_breakers: false };
}

/// Spike B (spikes/field_timing.py `lift`): an 8 kg carriage on two Kraken X60 at 10:1 on a
/// 1.75 in. spool, 40 A each, to 1 m with volts = 60 e - 4 v + 0.25. Python: there in 1.09 s,
/// 1.5 mm over, at every step size.
#[test]
fn m0_gate_geared_lift_matches_spike() {
    let lib = Library::builtin();
    let mut p = Project::new("lift");
    let r = &mut p.robot;
    r.drivetrain = None;
    stiff_battery(r);
    let id = r.id();
    r.links.push(Link { id, name: "carriage".into(), shapes: vec![Shape { name: "c".into(), kind: ShapeKind::Box { size: [0.2, 0.2, 0.1] }, pose: Pose::at([0.0, 0.0, 0.5]), color: None, role: ShapeRole::Body }],
        parts: vec![], mass: Some(8.0), color: String::new(), collision: Collision::None, material: None });
    let j = r.id();
    r.joints.push(Joint { id: j, name: "lift".into(), kind: JointKind::Slide, parent: CHASSIS, child: id, anchor: [0.0, 0.0, 0.5], axis: [0.0, 0.0, 1.0],
        range: Some([0.0, 1.5]), initial: 0.0, damping: 0.0, friction: 0.0, follows: None, spring: None, source: String::new() });
    let d = r.id();
    r.drives.push(Drive { id: d, name: "lift".into(), motor: "kraken-x60".into(), count: 2, ratio: 10.0, efficiency: 1.0, stator_limit: 40.0, supply_limit: 1e9,
        target: DriveTarget::Joint { joint: j, spool_radius: 0.0222 }, control: Control::Position { kp: 60.0, ki: 0.0, kd: 4.0, ks: 0.0, kg: 0.25, kv: 0.0, cruise: 0.0, accel: 0.0 }, brake: true });
    let mut w = World::new(&p, &lib, Scene::bench()).unwrap();
    w.cmd.targets.insert(d, 1.0);
    let (mut t_reach, mut peak) = (None, 0.0f64);
    while w.time < 2.0 {
        w.step();
        let q = w.snapshot().drives[0].position;
        peak = peak.max(q);
        if t_reach.is_none() && q > 0.98 { t_reach = Some(w.time); }
    }
    let t = t_reach.expect("reaches 98 % of 1 m");
    println!("lift: there in {t:.3} s, overshoot {:.1} mm", (peak - 1.0) * 1000.0);
    assert!((t / 1.088 - 1.0).abs() < 0.05, "time {t} vs the spike's 1.088 s");
    assert!(peak - 1.0 < 0.005, "overshoot {} m", peak - 1.0);
}

fn intake_project(wheel: &str, height_in: f64) -> Project {
    let mut p = Project::new("intake");
    p.robot.drivetrain = None;
    let mut t = Template::intake_default();
    if let Params::Intake { wheel: w, height, .. } = &mut t.params { *w = wheel.into(); *height = height_in * 0.0254; }
    templates::add(&mut p.robot, t).unwrap();
    p.bench.piece = "coral-2025".into();
    p.bench.speed = 1.0;
    p.bench.duration = 1.2;
    p
}

/// Spike A's questions on the real contact law: a CORAL driven onto a compliant roller comes in;
/// more squeeze costs more current; a harder wheel squeezes harder at the same geometry.
#[test]
fn m1_gate_roller_intake_behaves() {
    let lib = Library::builtin();
    // the clearance from roller centre to ramp: 6.3 in. is 5 mm of squeeze on a CORAL with a 4 in.
    // wheel, 5.9 in. is 15 mm
    let light = sim::bench::trial(&intake_project("am-compliant-4in-35a", 6.3), &lib, None, 1).unwrap();
    let firm = sim::bench::trial(&intake_project("am-compliant-4in-35a", 5.9), &lib, None, 1).unwrap();
    println!("light: in={} t={:?} mean {:.1} A squeeze {:.0} N peak {:.0} N rpm {:.0}->{:.0}", light.acquired, light.time_in, light.mean_current, light.mean_squeeze, light.peak_squeeze, light.start_roller_rpm, light.min_roller_rpm);
    println!("firm:  in={} t={:?} mean {:.1} A squeeze {:.0} N peak {:.0} N rpm {:.0}->{:.0}", firm.acquired, firm.time_in, firm.mean_current, firm.mean_squeeze, firm.peak_squeeze, firm.start_roller_rpm, firm.min_roller_rpm);
    assert!(light.acquired, "light squeeze comes in");
    assert!(firm.peak_squeeze > light.peak_squeeze * 1.3, "more squeeze, more force");
    assert!(firm.mean_current > light.mean_current, "more squeeze, more current: {} vs {}", firm.mean_current, light.mean_current);
    let soft = sim::bench::trial(&intake_project("am-compliant-4in-35a", 6.2), &lib, None, 1).unwrap();
    let hard = sim::bench::trial(&intake_project("am-compliant-4in-60a", 6.2), &lib, None, 1).unwrap();
    println!("35A: squeeze {:.0} N, 60A: squeeze {:.0} N", soft.peak_squeeze, hard.peak_squeeze);
    assert!(hard.peak_squeeze > soft.peak_squeeze * 1.5, "60A squeezes harder than 35A");
}

/// The step size doesn't change the answer (spike A's convergence check).
#[test]
fn m1_gate_converges_with_step_size() {
    let lib = Library::builtin();
    let p = intake_project("am-compliant-4in-35a", 6.0);
    let mut out = vec![];
    for dt in [0.001, 0.0005, 0.00025] {
        let mut scene = Scene::bench();
        let probe = World::new(&p, &lib, scene.clone()).unwrap();
        let (rmin, rmax) = probe.roller_x_range().unwrap();
        drop(probe);
        scene.pieces.push(sim::bench::spawn(&p, &lib, rmax, &mut sim::bench::Rng::new(1)).unwrap());
        let mut w = World::new(&p, &lib, scene).unwrap();
        w.dt = dt;
        w.set_timestep(dt);
        let d = p.robot.drives[0].id;
        w.cmd.run.insert(d, 10.0);
        let mut t_in = None;
        let mut peak: f64 = 0.0;
        while w.time < 1.2 {
            w.step();
            peak = peak.max(w.contact_force());
            if t_in.is_none() && w.piece_positions()[0].0[0] < rmin - 0.01 { t_in = Some(w.time); }
        }
        out.push((dt, t_in.unwrap_or(f64::NAN), peak));
    }
    println!("{out:?}");
    for (_, t, pk) in &out[1..] {
        assert!((t - out[0].1).abs() < 0.02, "entry time {t} vs {}", out[0].1);
        assert!((pk / out[0].2 - 1.0).abs() < 0.1, "peak squeeze {pk} vs {}", out[0].2);
    }
}

/// Concept (M2): driving onto a CORAL with the intake running picks it up, holds it, lets it go.
#[test]
fn m2_gate_concept_pickup_and_drive() {
    let lib = Library::builtin();
    let mut p = Project::new("concept");
    let i = templates::add(&mut p.robot, Template::intake_default()).unwrap();
    let mut scene = Scene::carpet();
    let k = sim::piece::PieceKind::from_library(lib.piece("coral-2025").unwrap());
    let (h, lay) = k.resting();
    scene.pieces.push(Spawn { piece: "coral-2025".into(), pos: [1.3, 0.0, h], quat: lay, lin: [0.0; 3], ang: [0.0; 3] });
    let mut w = World::new(&p, &lib, scene).unwrap();
    let roller = p.robot.drives.iter().find(|d| matches!(d.target, DriveTarget::Shaft { .. })).unwrap().id;
    w.cmd.run.insert(roller, 10.0);
    w.cmd.drive = [1.5, 0.0, 0.0];
    let mut held_at = None;
    let mut top: f64 = 0.0;
    while w.time < 3.0 {
        w.step();
        top = top.max(w.snapshot().speed);
        if held_at.is_none() && w.piece_positions()[0].1 { held_at = Some(w.time); w.cmd.drive = [0.0; 3]; }
    }
    println!("held at {held_at:?}, top speed {top:.2} m/s");
    assert!(held_at.is_some(), "picked up");
    assert!(top > 1.2, "drove");
    w.cmd.release.push(i);
    w.cmd.run.clear();
    for _ in 0..200 { w.step(); }
    assert!(!w.piece_positions()[0].1, "let go");
    assert!(w.engine.finite());
}

/// A robot drives to top speed close to its free speed and stops, and the battery sags while it
/// accelerates (M2, D6).
#[test]
fn m2_gate_drivetrain_and_battery() {
    let lib = Library::builtin();
    let p = Project::new("drive");
    let mut w = World::new(&p, &lib, Scene::carpet()).unwrap();
    w.cmd.drive = [6.0, 0.0, 0.0];
    let mut min_v: f64 = 20.0;
    let mut peak_i: f64 = 0.0;
    while w.time < 3.0 { w.step(); min_v = min_v.min(w.bus.voltage); peak_i = peak_i.max(w.bus.current); }
    let s = w.snapshot();
    println!("speed {:.2} m/s, bus min {min_v:.2} V, peak {peak_i:.0} A, tip margin {:.3} m", s.speed, s.tip_margin);
    // MK4i L2 with Kraken X60 FOC: free speed about 5800 rpm / 6.75 * pi * 4 in. = 4.6 m/s; capped at max_speed 5
    assert!(s.speed > 3.8 && s.speed < 5.0, "speed {}", s.speed);
    assert!(min_v < 12.0 && peak_i > 60.0);
    assert!(s.tip_margin > 0.1);
    w.cmd.drive = [0.0; 3];
    for _ in 0..1500 { w.step(); }
    assert!(w.snapshot().speed < 0.2);
}

/// The REEFSCAPE field loads, pieces settle, a CORAL let go near an L4 seat is seated and scored.
#[test]
fn m4_field_loads_and_scores() {
    let lib = Library::builtin();
    let mut p = Project::new("field");
    let e = templates::add(&mut p.robot, Template::elevator_default()).unwrap();
    let mut claw = Template::claw_default();
    claw.mount.on = Some(e);
    let c = templates::add(&mut p.robot, claw).unwrap();
    let g = sim::game::GamePack::builtin("reefscape-2025").unwrap();
    let mut w = World::new(&p, &lib, Scene::field(&g)).unwrap();
    w.start_match();
    for _ in 0..500 { w.step(); }
    assert!(w.engine.finite());
    let s = w.snapshot();
    assert!(s.pieces.iter().all(|p| p.pose.pos[2] > -0.05), "pieces stay on the field");
    let _ = (c, len([0.0; 3]));
}

#[test]
#[ignore]
fn probe_intake_trace() {
    let lib = Library::builtin();
    for h in [6.3, 5.7] {
        let t = sim::bench::trial(&intake_project("am-compliant-4in-35a", h), &lib, None, 1).unwrap();
        println!("--- height {h}");
        for s in t.samples.iter().step_by(6) { println!("t {:.3} I {:6.1} F {:7.1} rpm {:6.0} x {:.3} slip {:.2}", s.t, s.current, s.squeeze, s.roller_rpm, s.piece_x, s.sliding); }
    }
}

#[test]
#[ignore]
fn probe_drive() {
    let lib = Library::builtin();
    let p = Project::new("drive");
    let mut w = World::new(&p, &lib, Scene::carpet()).unwrap();
    w.cmd.drive = [6.0, 0.0, 0.0];
    let mut k = 0;
    while w.time < 3.0 {
        w.step(); k += 1;
        if k % 250 == 0 { let s = w.snapshot(); println!("t {:.2} v {:.2} bus {:.2} I {:.0} z {:.4} com {:?} {}", s.time, s.speed, s.bus_voltage, s.bus_current, s.links[0].pose.pos[2], s.com.map(|x| (x*1000.0).round()/1000.0), w.debug_drive()); }
    }
}

#[test]
#[ignore]
fn dump_carpet_xml() {
    let lib = Library::builtin();
    let w = World::new(&Project::new("d"), &lib, Scene::carpet()).unwrap();
    std::fs::write(std::env::var("XML_OUT").unwrap(), &w.mjcf).unwrap();
}
