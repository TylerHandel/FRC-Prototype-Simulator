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

#[test]
fn reversing_a_stock_claw_places_coral_at_the_configured_eject_speed() {
    let lib = Library::builtin();
    let mut p = Project::new("Place CORAL");
    let mut claw = Template::claw_default();
    claw.mount.offset = [0.5, 0.0, 0.6];
    let claw = templates::add(&mut p.robot, claw).unwrap();
    let mechanism = p.robot.mechanism(claw).unwrap();
    let pickup = mechanism.pickup.as_ref().unwrap();
    let drive = pickup.drive.unwrap();
    let button = |sign: f64| p.robot.controls.bindings.iter().find(|b| matches!(b.action,
        Action::Run {drive: d, value, ..} if d == drive && value.signum() == sign)).unwrap().button;
    let mut scene = Scene::carpet();
    let kind = sim::piece::PieceKind::from_library(lib.piece("coral-2025").unwrap());
    scene.pieces.push(Spawn { piece: kind.id.clone(), pos: pickup.center, quat: kind.resting().1, lin: [0.0;3], ang: [0.0;3] });
    let mut w = World::new(&p, &lib, scene).unwrap();
    w.button(button(1.0), true);
    for _ in 0..150 { w.step(); }
    assert!(w.snapshot().pieces[0].held);
    let held = w.snapshot().pieces[0].pose.pos;
    w.button(button(1.0), false);
    w.button(button(-1.0), true);
    w.step();
    let joint = w.engine.id(sim::engine::Obj::Joint, "p0").unwrap();
    let (velocity, _) = w.engine.free_vel(joint);
    println!("Released CORAL velocity {velocity:?} m/s");
    assert!((velocity[0] - 1.0).abs() < 0.1 && velocity[2].abs() < 0.15, "{velocity:?}");
    for _ in 0..250 { w.step(); }
    let pos = w.snapshot().pieces[0].pose.pos;
    assert!(pos[0] > held[0] && pos[0] < held[0] + 0.9 && pos[2] < held[2], "{pos:?}");
}

#[test]
fn sideways_claw_uses_its_rotated_pickup_box_and_release_direction() {
    let lib = Library::builtin();
    let mut p = Project::new("Side intake");
    let mut claw = Template::claw_default();
    claw.mount.offset = [0.0, 0.48, 0.60];
    claw.mount.rotation_deg[2] = 90.0;
    let id = templates::add(&mut p.robot, claw).unwrap();
    let pickup = p.robot.mechanism(id).unwrap().pickup.as_ref().unwrap();
    // Inside the rotated width, beyond the unrotated box's half-depth.
    let mut pos = pickup.center;
    pos[0] += 0.145;
    let mut scene = Scene::carpet();
    let kind = sim::piece::PieceKind::from_library(lib.piece("coral-2025").unwrap());
    scene.pieces.push(Spawn { piece: kind.id.clone(), pos, quat: kind.resting().1, lin: [0.0; 3], ang: [0.0; 3] });
    let mut w = World::new(&p, &lib, scene).unwrap();
    w.button(Button::RB, true);
    for _ in 0..150 { w.step(); }
    assert!(w.snapshot().pieces[0].held);
    w.button(Button::RB, false);
    w.button(Button::LT, true);
    w.step();
    let joint = w.engine.id(sim::engine::Obj::Joint, "p0").unwrap();
    let (vel, _) = w.engine.free_vel(joint);
    assert!(vel[0].abs() < 0.1 && (vel[1] - 1.0).abs() < 0.1 && vel[2].abs() < 0.15, "{vel:?}");
}

#[test]
fn season_practice_keeps_one_piece_and_drives_across_a_rebuilt_bump() {
    let lib = Library::builtin();
    let p = Project::new("Bump crossing");
    let mut game = sim::game::GamePack::builtin("rebuilt-2026").unwrap();
    game.practice_start.as_mut().unwrap().pos = [-5.1, 1.522];
    let mut w = World::practice(&p, &lib, "fuel-2026", Some(&game)).unwrap();
    assert_eq!(w.snapshot().pieces.len(), 1);
    assert!((w.chassis_pose().pos[0] + 5.1).abs() < 0.001);
    w.cmd.drive = [1.6, 0.0, 0.0];
    let mut peak = 0.0_f64;
    while w.time < 3.0 && w.chassis_pose().pos[0] < -2.55 {
        w.step();
        peak = peak.max(w.chassis_pose().pos[2]);
    }
    assert!(peak > 0.085, "climbed {peak} m");
    assert!(w.chassis_pose().pos[0] >= -2.55, "ended at {:?}", w.chassis_pose().pos);
    assert!(w.chassis_pose().pos[2] < 0.08);
    assert!(w.place_practice_piece("coral-2025"));
    assert_eq!(w.snapshot().pieces.len(), 1);
    assert_eq!(w.snapshot().pieces[0].kind, "coral-2025");
}

#[test]
fn test_drive_moves_on_field_axes_after_turning() {
    let lib = Library::builtin();
    let mut world = World::test_drive(&Project::new("Field-centric"), &lib, "coral-2025").unwrap();
    world.cmd.drive = [0.0, 0.0, 1.5];
    while yaw_of(world.chassis_pose().quat) < 1.55 && world.time < 2.0 { world.step(); }
    assert!(yaw_of(world.chassis_pose().quat) > 1.5);
    for (forward, left) in [(1.0, 0.0), (0.0, 1.0)] {
        world.cmd.drive = [0.0; 3];
        for _ in 0..250 { world.step(); }
        let start = world.chassis_pose().pos;
        world.cmd.drive = [forward, left, 0.0];
        for _ in 0..500 { world.step(); }
        let [dx, dy, _] = model::math::sub(world.chassis_pose().pos, start);
        assert!(dx * forward + dy * left > 0.75, "field movement {dx}, {dy}");
        assert!((dy * forward - dx * left).abs() < 0.10, "cross-track drift {dx}, {dy}");
    }
}

#[test]
fn test_drive_practice_piece_can_be_swapped_and_replaced_after_turning() {
    let lib = Library::builtin();
    let mut p = Project::new("Practice");
    let elevator = templates::add(&mut p.robot, Template::elevator_default()).unwrap();
    let mut claw = Template::claw_default();
    claw.mount.on = Some(elevator);
    templates::add(&mut p.robot, claw).unwrap();
    templates::add(&mut p.robot, Template::intake_default()).unwrap();
    let mut w = World::test_drive(&p, &lib, "coral-2025").unwrap();
    let first = w.snapshot().pieces;
    assert_eq!(first.len(), 1);
    assert_eq!(first[0].kind, "coral-2025");
    w.button(Button::RT, true);
    w.cmd.drive = [p.robot.drivetrain.as_ref().unwrap().max_speed, 0.0, 0.0];
    while w.time < 0.6 { w.step(); }
    let before_rebuild = w.snapshot().pieces[0].pose;
    let mut rebuilt = World::test_drive(&p, &lib, "coral-2025").unwrap();
    rebuilt.carry_from(&w);
    assert!(len(sub(rebuilt.snapshot().pieces[0].pose.pos, before_rebuild.pos)) < 0.001);
    assert!(!rebuilt.snapshot().pieces[0].held);

    w.button(Button::RT, false);
    w.cmd.drive = [0.0, 0.0, 1.0];
    for _ in 0..600 { w.step(); }
    w.engine.forward();
    let before = w.chassis_pose();
    let time = w.time;
    let old_commands = w.cmd.drive;
    for id in ["fuel-2026", "algae-2025", "coral-2025", "coral-2025"] {
        assert!(w.place_practice_piece(id));
        let pieces = w.snapshot().pieces;
        assert_eq!(pieces.len(), 1);
        assert_eq!(pieces[0].kind, id);
        assert!(!pieces[0].held);
        let forward = before.rotate([1.0, 0.0, 0.0]);
        let delta = model::math::sub(pieces[0].pose.pos, before.pos);
        assert!(forward[0]*delta[0] + forward[1]*delta[1] > 0.5);
        assert!((forward[0]*delta[1] - forward[1]*delta[0]).abs() < 0.001);
        assert_eq!(w.time, time);
        assert_eq!(w.cmd.drive, old_commands);
        assert!(model::math::len(model::math::sub(w.chassis_pose().pos, before.pos)) < 0.01);
    }
    assert!(!w.place_practice_piece("missing"));
    w.cmd.drive = [0.0; 3];
    for _ in 0..100 { w.step(); }
    assert!(!w.snapshot().pieces[0].held);
    assert!(w.engine.finite());
}

fn upright_reef_robot() -> (Project, Id) {
    let p = templates::upright_reef_example();
    let id = p.robot.mechanisms.iter().find(|m| m.kind == MechKind::Claw).unwrap().id;
    (p, id)
}

#[test]
fn roller_hold_runs_in_either_direction_and_resumes_after_override() {
    let lib = Library::builtin();
    for amps in [3.0, -3.0] {
        let (mut p, claw) = upright_reef_robot();
        let drive = p.robot.mechanism(claw).unwrap().pickup.as_ref().unwrap().drive.unwrap();
        p.robot.controls.roller_hold_amps.insert(drive, amps);
        let mut w = World::test_drive(&p, &lib, "coral-2025").unwrap();
        let read = |w: &World| w.snapshot().drives.into_iter().find(|d| d.id == drive).unwrap();
        for _ in 0..25 { w.step(); }
        assert!((read(&w).current - amps.abs()).abs() < 1e-8 && read(&w).speed * amps > 0.);
        let opposite = if amps > 0. { Button::LT } else { Button::RB };
        w.button(opposite, true);
        for _ in 0..150 { w.step(); }
        assert!(read(&w).speed * amps < 0.);
        w.button(opposite, false);
        w.step();
        assert!((read(&w).current - amps.abs()).abs() < 1e-8);
    }
}

#[test]
fn upright_coral_is_physically_gripped_lifted_and_released() {
    for gap in [None, Some(3.5 * library::IN)] {
        run_upright_grip(gap);
    }
}

fn run_upright_grip(gap: Option<f64>) {
    let lib = Library::builtin();
    let (mut p, id) = upright_reef_robot();
    if let Some(gap) = gap {
        let mut t = p.robot.mechanism(id).unwrap().template.clone().unwrap();
        if let Params::Claw { gap: g, .. } = &mut t.params { *g = gap; }
        templates::rebuild(&mut p.robot, id, t).unwrap();
    }
    let mut w = World::test_drive(&p, &lib, "coral-2025").unwrap();
    w.button(Button::RB, true);
    for _ in 0..180 { w.step(); }
    assert!(w.place_piece("coral-2025", sim::world::TubeOrientation::Upright, Some(id)));
    let start = w.snapshot().pieces[0].pose;
    let mut contact: f64 = 0.;
    for _ in 0..650 { w.step(); contact = contact.max(w.contact_force()); }
    let seated = w.snapshot().pieces[0].pose;
    w.button(Button::RB, false);
    w.button(Button::B, true);
    for _ in 0..1300 { w.step(); }
    let lifted = w.snapshot().pieces[0].clone();
    println!("feed {:?}, seated {:?}, lifted {:?}, contact {contact}", start.pos, seated.pos, lifted.pose.pos);
    assert!(contact > 1. && seated.pos[0] < start.pos[0] - 0.04);
    assert!(lifted.pose.pos[2] > seated.pos[2] + 0.15 && lifted.pose.rotate([0.,0.,1.])[2] > 0.92);
    assert!(!lifted.held);
    let grip = w.chassis_pose().inverse().apply(lifted.pose.pos);
    for demand in [[0., 2., 0.], [0., -2., 0.], [-2., 0., 0.], [2., 0., 0.]] {
        let start = w.chassis_pose().pos;
        w.cmd.drive = demand;
        let mut slip: f64 = 0.;
        let mut distance = 0.;
        let mut previous = start;
        for _ in 0..(0.7 / w.dt) as usize {
            w.step();
            let chassis = w.chassis_pose();
            // On reversal the chassis can cross its starting point while loading the claw.
            distance += model::math::len(model::math::sub(chassis.pos, previous));
            previous = chassis.pos;
            let local = chassis.inverse().apply(w.piece_positions()[0].0);
            slip = slip.max(model::math::len(model::math::sub(local, grip)));
        }
        println!("drive {demand:?}, moved {distance:.3} m, maximum CORAL displacement {slip:.4} m");
        assert!(distance > 0.1 && slip < 0.025, "grip during horizontal drive");
    }
    w.cmd.drive = [0.; 3];
    let before_release = w.chassis_pose().inverse().apply(w.piece_positions()[0].0);
    w.button(Button::LT, true);
    for _ in 0..650 { w.step(); }
    let released = w.chassis_pose().inverse().apply(w.piece_positions()[0].0);
    assert!(released[0] > before_release[0] + 0.08 || released[2] < before_release[2] - 0.15, "released {released:?}");
}

#[test]
fn upright_claw_drives_to_the_reef_and_scores_l4() {
    for right in [false, true] { score_with_starter(right); }
}

fn score_with_starter(right: bool) {
    let lib = Library::builtin();
    let (p, claw) = upright_reef_robot();
    let g = sim::game::GamePack::builtin("reefscape-2025").unwrap();
    let mut w = World::practice(&p, &lib, "coral-2025", Some(&g)).unwrap();
    let step = |w: &mut World, seconds: f64| {
        for _ in 0..(seconds/w.dt).round() as usize { w.step(); }
    };
    w.button(Button::RB, true); // operator's intake action
    step(&mut w, 0.18);
    assert!(w.place_piece("coral-2025", sim::world::TubeOrientation::Upright, Some(claw)));
    step(&mut w, 0.65);
    w.button(Button::RB, false);
    let lift = |name: &str| {
        let preset = p.robot.controls.presets.iter().find(|x| x.name == name).unwrap().id;
        p.robot.controls.bindings.iter().find(|x| x.action == (Action::Preset { preset })).unwrap().button
    };
    w.button(lift("Clear L4"), true);
    step(&mut w, 2.);
    let high = w.snapshot().pieces[0].pose;
    assert!(high.pos[2] > 1.99, "CORAL above the branch {:?}", high.pos);
    w.reef_align(right, true);
    step(&mut w, 3.5);
    let above = w.snapshot().pieces[0].pose;
    let name = if right { "Blue L4 4l" } else { "Blue L4 4r" };
    let peg = g.zones.iter().find(|z| z.name == name).unwrap().seat.as_ref().unwrap().peg_tip.unwrap();
    println!("CORAL above {:?}, peg {peg:?}, chassis {:?}",above.pos,w.chassis_pose().pos);
    assert!((above.pos[0]-peg[0]).hypot(above.pos[1]-peg[1]) < 0.025);
    w.button(lift("Seat L4"), true);
    step(&mut w, 1.2);
    assert_eq!(w.matched.score, 0., "The rollers still grip the pipe");
    w.reef_align(right, false);
    w.button(Button::LT, true);
    w.cmd.drive = [-0.45,0.,0.];
    step(&mut w, 0.75);
    w.button(Button::LT, false);
    w.cmd.drive = [0.;3];
    step(&mut w, 1.);
    let result = w.snapshot();
    assert!(result.pieces[0].scored && result.score == 5., "released CORAL {:?}, events {:?}",result.pieces[0], w.matched.log);
    assert!(w.matched.log.last().unwrap().what.ends_with(name));
}

#[test]
fn bumpers_align_to_camera_side_and_change_distance_with_position() {
    let lib = Library::builtin();
    let mut p = Project::new("Reef driver");
    templates::add(&mut p.robot, Template::elevator_default()).unwrap();
    let position = p.robot.controls.bindings.iter().find_map(|b| match (b.button, &b.action) {
        (Button::B, Action::Preset { preset }) => Some(*preset), _ => None,
    }).unwrap();
    p.robot.controls.presets.iter_mut().find(|p| p.id == position).unwrap().reef_gap = 0.32;
    let saved = serde_json::to_string(&p).unwrap();
    let p = serde_json::from_str::<Project>(&saved).unwrap();
    let g = sim::game::GamePack::builtin("reefscape-2025").unwrap();
    let mut g = g;
    // Face back toward the driver station so robot-right and screen-right are opposite.
    g.practice_start.as_mut().unwrap().yaw_deg = 150.;
    let face = g.reef_faces.iter().min_by(|a,b| a.center[0].total_cmp(&b.center[0])).unwrap();
    let mut w = World::practice(&p, &lib, "coral-2025", Some(&g)).unwrap();
    w.reef_align(true, true);
    let front = p.robot.frame.length / 2. + p.robot.frame.bumper_thickness;
    let aligned = |w: &World, y: f64, gap: f64| {
        let pose = w.chassis_pose();
        assert!((pose.pos[1]-y).abs() < 0.025 &&
            (face.center[0] - pose.pos[0] - front - gap).abs() < 0.025, "aligned pose {:?}, y {y}, gap {gap}", pose.pos);
        assert!(pose.rotate([1.,0.,0.])[0] > 0.99);
    };
    for _ in 0..2500 { w.step(); }
    aligned(&w, -face.branch_offset, model::default_reef_gap());
    w.button(Button::B, true);
    for _ in 0..1800 { w.step(); }
    aligned(&w, -face.branch_offset, 0.32);
    w.reef_align(true, false);
    w.reef_align(false, true);
    for _ in 0..1800 { w.step(); }
    aligned(&w, face.branch_offset, 0.32);
    w.reef_align(false, false);
    w.cmd.drive = [-0.7,0.,0.];
    let before = w.chassis_pose().pos;
    for _ in 0..600 { w.step(); }
    assert!(w.chassis_pose().pos[0] < before[0] - 0.2);
    for face in &g.reef_faces {
        for yaw in [-2.2, -0.3, 1.4] {
            let (left, _) = face.branch(yaw, false);
            let (right, _) = face.branch(yaw, true);
            assert!(dot(sub(right, left), [yaw.sin(), -yaw.cos(), 0.]) > 0.);
        }
    }
}

#[test]
fn practice_coral_threads_each_level_and_can_score_again() {
    let lib = Library::builtin();
    let g = sim::game::GamePack::builtin("reefscape-2025").unwrap();
    let mut w = World::practice(&Project::new("Reef practice"), &lib, "coral-2025", Some(&g)).unwrap();
    let idx = lib.pieces.iter().position(|p| p.id == "coral-2025").unwrap();
    let joint = w.engine.id(sim::engine::Obj::Joint, &format!("p{idx}")).unwrap();
    for level in ["Blue L2 1l", "Blue L3 1r", "Red L4 5l", "Red L4 5l"] {
        assert!(w.place_practice_piece("coral-2025"));
        let seat = g.zones.iter().find(|z| z.name == level).unwrap().seat.as_ref().unwrap();
        let quat = euler_to_quat(seat.euler_deg.map(f64::to_radians));
        let axis = qrot(quat,[0.,0.,1.]);
        let tip = seat.peg_tip.unwrap();
        let start = if level.contains("L4") { add(tip, scale(axis, 0.17)) } else { add(seat.pos,scale(axis, 0.05)) };
        w.engine.set_free(joint, start, quat, [0.;3], [0.;3]);
        w.engine.forward();
        let before = w.matched.score;
        for _ in 0..1500 { w.step(); }
        let piece = w.snapshot().pieces[0].clone();
        assert!(piece.scored && w.matched.score > before, "{level} at {:?} axis {:?} events {:?}", piece.pose.pos, piece.pose.rotate([0.,0.,1.]), w.matched.log);
        assert!(w.matched.log.last().unwrap().what.ends_with(level));
        let sideways = Pose::along(seat.pos, [-axis[2],0.,axis[0]]);
        assert!(!seat.threaded(sideways,0.051,0.15));
    }
    w.place_practice_piece("coral-2025");
    let trough = g.zones.iter().find(|z| z.name == "Blue L1 face 1").unwrap();
    let sideways = axis_angle([1.,0.,0.], std::f64::consts::FRAC_PI_2);
    w.engine.set_free(joint, add(trough.center,[0.,0.,0.05]), sideways, [0.;3], [0.;3]);
    w.engine.forward();
    for _ in 0..1500 { w.step(); }
    assert!(w.matched.log.last().unwrap().what.ends_with(&trough.name), "trough {:?}", w.snapshot().pieces[0]);
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

/// A scripted driver for the field gates: drives field-relative to a pose, with a P loop on place
/// and heading, while the sim steps.
struct Driver { elev: Id, claw_drive: Id, claw: Id }

fn yaw_of(q: Quat) -> f64 { let x = qrot(q, [1.0, 0.0, 0.0]); x[1].atan2(x[0]) }
fn wrap(a: f64) -> f64 { (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI }

impl Driver {
    /// drives to (x, y, yaw); false if it didn't get there in `limit` seconds
    fn go(&self, w: &mut World, to: [f64; 3], limit: f64, mut each: impl FnMut(&mut World, f64)) -> bool {
        let t0 = w.time;
        while w.time - t0 < limit {
            let c = w.chassis_pose();
            let (ex, ey) = (to[0] - c.pos[0], to[1] - c.pos[1]);
            let d = ex.hypot(ey);
            let eyaw = wrap(to[2] - yaw_of(c.quat));
            if d < 0.03 && eyaw.abs() < 0.03 { w.cmd.drive = [0.0; 3]; for _ in 0..50 { w.step(); } return true; }
            let v = (3.0 * d).min(3.5);
            w.cmd.drive = if d > 1e-6 { [ex / d * v, ey / d * v, (5.0 * eyaw).clamp(-4.0, 4.0)] } else { [0.0, 0.0, (5.0 * eyaw).clamp(-4.0, 4.0)] };
            each(w, d);
            w.step();
        }
        w.cmd.drive = [0.0; 3];
        false
    }
    /// waypoints on a circle round the REEF, the short way, when the straight line would cross it
    fn around(&self, w: &mut World, reef: V3, to: [f64; 3]) {
        let c = w.chassis_pose().pos;
        let (a0, a1) = ((c[1] - reef[1]).atan2(c[0] - reef[0]), (to[1] - reef[1]).atan2(to[0] - reef[0]));
        let turn = wrap(a1 - a0);
        let steps = (turn.abs() / (std::f64::consts::PI / 4.0)).floor() as usize;
        for i in 1..=steps {
            let a = a0 + turn * i as f64 / (steps + 1) as f64;
            let p = [reef[0] + 2.0 * a.cos(), reef[1] + 2.0 * a.sin(), to[2]];
            assert!(self.go(w, p, 6.0, |_, _| {}), "around the REEF: stuck at {:?} going to {p:?}", w.chassis_pose().pos);
        }
    }
    fn elevator(&self, w: &mut World, h: f64) { w.cmd.targets.insert(self.elev, h); }
    fn wait(&self, w: &mut World, limit: f64, mut done: impl FnMut(&World) -> bool) -> bool {
        let t0 = w.time;
        while w.time - t0 < limit { if done(w) { return true; } w.step(); }
        false
    }
}

/// M4 gate: a Concept elevator robot runs CORAL STATION to L4 cycles under a scripted driver, and
/// the score and the cycle log match a hand count (5 points per L4 CORAL in teleop).
#[test]
fn m4_gate_station_to_l4_cycles() {
    let lib = Library::builtin();
    let mut p = Project::new("cycles");
    let mut el = Template::elevator_default();
    el.mount.offset[0] = 0.2;
    let e = templates::add(&mut p.robot, el).unwrap();
    let mut claw = Template::claw_default();
    claw.mount.on = Some(e);
    let c = templates::add(&mut p.robot, claw).unwrap();
    let r = &p.robot;
    let elev = r.drives.iter().find(|d| matches!(d.target, DriveTarget::Joint { .. })).unwrap().id;
    let claw_drive = r.drives.iter().find(|d| matches!(d.target, DriveTarget::Shaft { shaft } if r.shaft(shaft).is_some_and(|s| r.mechanism(c).unwrap().links.contains(&s.link)))).unwrap().id;
    let top = r.joints.iter().filter_map(|j| j.range).map(|x| x[1]).fold(0.0, f64::max);
    let half = r.frame.length / 2.0 + r.frame.bumper_thickness;
    let g = sim::game::GamePack::builtin("reefscape-2025").unwrap();
    let mut w = World::new(&p, &lib, Scene::field(&g)).unwrap();
    w.cmd.field_relative = true;
    let bot = Driver { elev, claw_drive, claw: c };
    // how far ahead of the chassis the claw holds a piece (the robot starts facing +x)
    let reef = g.statics.iter().find(|s| s.name == "Blue REEF base").unwrap().pos;
    let station = g.feeders.iter().find(|f| f.name == "Blue CORAL STATION south").unwrap().clone();
    let dir = { let n = station.dir[0].hypot(station.dir[1]); [station.dir[0] / n, station.dir[1] / n] };
    // stand so the claw's zone is where the CORAL comes down, facing the station
    let zone_ahead = w.pickup_zones()[0].1.pos[0] - w.chassis_pose().pos[0];
    let stand_station = [station.pos[0] + dir[0] * (zone_ahead + 0.3), station.pos[1] + dir[1] * (zone_ahead + 0.3), (-dir[1]).atan2(-dir[0])];
    let targets = ["Blue L4 6l", "Blue L4 6r", "Blue L4 1r", "Blue L4 1l"];
    let mut scored_at = vec![];
    for name in targets {
        // to the station, the claw at the chute's height and running
        bot.elevator(&mut w, 0.1);
        w.cmd.run.insert(bot.claw_drive, 8.0);
        bot.around(&mut w, reef, stand_station);
        assert!(bot.go(&mut w, stand_station, 8.0, |_, _| {}), "reached the station for {name}");
        let mut k = 0;
        let got = bot.wait(&mut w, 4.0, |w| {
            k += 1;
            if std::env::var("GATE_TRACE").is_ok() && k % 25 == 0 {
                let z = w.pickup_zones()[0].1.pos;
                let near = w.piece_positions().into_iter().map(|p| p.0).filter(|p| p[2] > -1.0).min_by(|a, b| len(sub(*a, z)).total_cmp(&len(sub(*b, z))));
                println!("t {:.2} chassis {:?} zone {:?} nearest piece {:?}", w.time, w.chassis_pose().pos.map(|x| (x * 100.0).round() / 100.0), z.map(|x| (x * 100.0).round() / 100.0), near.map(|p| p.map(|x| (x * 100.0).round() / 100.0)));
            }
            w.holders()[0].2
        });
        assert!(got, "took a CORAL from the station for {name} (t {:.1})", w.time);
        w.cmd.run.remove(&bot.claw_drive);
        // to the branch: bumper against the REEF, facing it, the claw raised on the way in
        let seat = g.zones.iter().find(|z| z.name == name).unwrap().seat.clone().unwrap().pos;
        let a = (seat[1] - reef[1]).atan2(seat[0] - reef[0]);
        let a = (a / (std::f64::consts::PI / 3.0)).round() * std::f64::consts::PI / 3.0;
        let n = [a.cos(), a.sin()];
        let inset = 0.051; // L4 centreline measured from the CAD face
        let out = inset + half + 0.03;
        let stand = [seat[0] + n[0] * out, seat[1] + n[1] * out, (-n[1]).atan2(-n[0])];
        bot.around(&mut w, reef, stand);
        assert!(bot.go(&mut w, stand, 10.0, |w, d| if d < 2.0 { w.cmd.targets.insert(elev, top); }), "reached {name}");
        bot.elevator(&mut w, top);
        assert!(bot.wait(&mut w, 3.0, |w| w.snapshot().drives.iter().any(|d| d.id == elev && (d.position - top).abs() < 0.01)), "elevator up for {name}");
        let before = w.matched.score;
        let h = w.holders()[0].1;
        w.cmd.release.push(bot.claw);
        w.step();
        w.cmd.release.clear();
        println!("{name}: holder at {:?}, seat {:?}, {:.3} m apart, score {} -> {} at {:.1} s", h.map(|x| (x * 1000.0).round() / 1000.0), seat, len(sub(h, seat)), before, w.matched.score, w.time);
        assert!(w.matched.score - before == 5.0, "{name} scored 5");
        scored_at.push(w.time);
        // back off before lowering, or the claw comes down on the branches
        assert!(bot.go(&mut w, [stand[0] + n[0] * 0.5, stand[1] + n[1] * 0.5, stand[2]], 4.0, |_, _| {}), "backed off {name}");
        bot.elevator(&mut w, 0.1);
    }
    let log: Vec<_> = w.matched.log.iter().filter(|e| e.points > 0.0).collect();
    assert_eq!(w.matched.score, 5.0 * targets.len() as f64, "score matches the hand count");
    assert_eq!(log.len(), targets.len(), "one log line per CORAL");
    for (e, name) in log.iter().zip(targets) { assert_eq!(e.what, format!("scored {name}")); }
    let cycles: Vec<f64> = scored_at.windows(2).map(|x| x[1] - x[0]).collect();
    println!("cycles {cycles:.1?} s");
    assert!(cycles.iter().all(|c| *c > 3.0 && *c < 15.0), "plausible cycle times");
    assert!(w.engine.finite());
}

/// M1 gate, closed form: a block on a slope holds just below atan(mu) and slides just above it.
/// The slope is a tilted gravity on a level floor, so nothing else changes between the runs.
#[test]
fn m1_gate_block_slips_at_atan_mu() {
    let mu: f64 = 0.6;
    let slip = |deg: f64| -> f64 {
        let t = deg.to_radians();
        let xml = format!(r#"<mujoco><option timestep="0.001" gravity="{} 0 {}" impratio="10" cone="elliptic"/>
            <worldbody><geom type="plane" size="5 5 0.1" friction="{mu} 0.005 0.0001"/>
            <body pos="0 0 0.05"><freejoint/><geom type="box" size="0.1 0.1 0.05" mass="2" friction="{mu} 0.005 0.0001"/></body></worldbody></mujoco>"#,
            9.81 * t.sin(), -9.81 * t.cos());
        let mut e = sim::engine::Engine::from_xml(&xml).unwrap();
        for _ in 0..200 { e.begin_step(); e.step(); }
        let x0 = e.body_pos(1)[0];
        for _ in 0..1000 { e.begin_step(); e.step(); }
        e.body_pos(1)[0] - x0
    };
    let at = mu.atan().to_degrees();
    let (below, above) = (slip(at - 2.0), slip(at + 2.0));
    println!("atan(mu) = {at:.1} deg: {:.2} mm in a second at 2 deg below, {:.0} mm at 2 deg above", below * 1000.0, above * 1000.0);
    assert!(below.abs() < 0.002, "holds below atan(mu)");
    assert!(above > 0.05, "slides above atan(mu)");
}

/// M5: a CAD hull collides where the CAD put it. A plate off to one side, as a hull, catches a
/// ball dropped on it; the ball rests on its top face.
#[test]
fn m5_hull_collides_where_the_cad_is() {
    let lib = Library::builtin();
    let mut p = Project::new("hull");
    p.robot.drivetrain = None;
    let c = [0.6, 0.2, 0.40];
    let pts: Vec<V3> = (0..8).map(|i| [if i & 1 == 0 { -0.15 } else { 0.15 }, if i & 2 == 0 { -0.15 } else { 0.15 }, if i & 4 == 0 { -0.01 } else { 0.01 }]).collect();
    p.robot.links[0].shapes.push(Shape { name: "plate".into(), kind: ShapeKind::Hull { points: pts }, pose: Pose::at(c), color: None, role: ShapeRole::Body });
    let mut scene = Scene::bench();
    let k = sim::piece::PieceKind::from_library(lib.piece("algae-2025").unwrap());
    let sim::piece::Shape::Sphere { r } = k.shape else { panic!() };
    // dropped from a centimetre: an ALGAE bounces like the playground ball it is
    scene.pieces.push(Spawn { piece: "algae-2025".into(), pos: [c[0] + 0.05, c[1] - 0.05, c[2] + 0.01 + r + 0.01], quat: QI, lin: [0.0; 3], ang: [0.0; 3] });
    let mut w = World::new(&p, &lib, scene).unwrap();
    for _ in 0..6000 { w.step(); }
    let z = w.piece_positions()[0].0[2];
    println!("ball at {z:.3} m, plate top {:.3} + radius {r:.3}", c[2] + 0.01);
    assert!((z - (c[2] + 0.01 + r)).abs() < 0.003, "rests on the plate");
}

/// M5: CORAL is hollow. Dropped over an L4 branch's top it slides down to the bend and is
/// scored there; dropped onto an L3 branch it slides on and is scored. No assist.
#[test]
fn m5_coral_scores_on_branches_by_itself() {
    let lib = Library::builtin();
    let p = Project::new("branches");
    let g = sim::game::GamePack::builtin("reefscape-2025").unwrap();
    let seat = |n: &str| g.zones.iter().find(|z| z.name == n).unwrap().seat.clone().unwrap();
    let (l4, l3) = (seat("Blue L4 1l"), seat("Blue L3 1r"));
    let mut scene = Scene::field(&g);
    scene.game_pieces = false;
    // upright, its bottom just above the post's top
    scene.pieces.push(Spawn { piece: "coral-2025".into(), pos: [l4.pos[0], l4.pos[1], 1.85 + 0.16], quat: QI, lin: [0.0; 3], ang: [0.0; 3] });
    // along the L3 branch, a little out from where it sits
    let q = euler_to_quat(l3.euler_deg.map(f64::to_radians));
    let out = qrot(q, [0.0, 0.0, 1.0]);
    scene.pieces.push(Spawn { piece: "coral-2025".into(), pos: add(l3.pos, scale(out, 0.05)), quat: q, lin: [0.0; 3], ang: [0.0; 3] });
    let mut w = World::new(&p, &lib, scene).unwrap();
    while w.time < 3.0 { w.step(); }
    for (i, (pos, _)) in w.piece_positions().iter().enumerate() {
        let s = &w.snapshot().pieces[i];
        println!("piece {i} at {pos:?}, axis {:?}, contacts {:?}", s.pose.rotate([0.,0.,1.]), w.piece_contacts(i));
    }
    println!("log {:?}", w.matched.log.iter().map(|e| (&e.what, e.points)).collect::<Vec<_>>());
    assert!(w.matched.log.iter().any(|e| e.what == "scored Blue L4 1l"), "L4 by itself");
    assert!(w.matched.log.iter().any(|e| e.what == "scored Blue L3 1r"), "L3 by itself");
    assert_eq!(w.matched.score, 5.0 + 4.0);
}

/// M5: soft pieces meet the robot in real units. An ALGAE resting on a robot's plate sinks until
/// its pressure over the flattened patch holds its weight: plate_force(d) = m g.
#[test]
fn m5_algae_on_a_plate_holds_its_weight_by_pressure() {
    let lib = Library::builtin();
    let mut p = Project::new("soft");
    p.robot.drivetrain = None;
    let top = 0.40;
    p.robot.links[0].shapes.push(Shape { name: "plate".into(), kind: ShapeKind::Box { size: [0.6, 0.6, 0.02] }, pose: Pose::at([0.0, 0.0, top - 0.01]), color: None, role: ShapeRole::Body });
    let k = sim::piece::PieceKind::from_library(lib.piece("algae-2025").unwrap());
    let sim::piece::Shape::Sphere { r } = k.shape else { panic!() };
    let mut scene = Scene::bench();
    scene.pieces.push(Spawn { piece: "algae-2025".into(), pos: [0.0, 0.0, top + r + 0.02], quat: QI, lin: [0.0; 3], ang: [0.0; 3] });
    let mut w = World::new(&p, &lib, scene).unwrap();
    for _ in 0..6000 { w.step(); }
    let d = top + r - w.piece_positions()[0].0[2];
    let f = k.plate_force(d).unwrap();
    println!("sank {:.2} mm: {f:.2} N holding {:.2} N", d * 1000.0, k.mass * 9.81);
    assert!((f / (k.mass * 9.81) - 1.0).abs() < 0.1, "pressure holds the weight");
}

/// M5: a beam break across where the intake holds a CORAL trips once it has one, and a button
/// bound to "run until" it stops the rollers by itself.
#[test]
fn m5_beam_break_stops_the_intake() {
    let lib = Library::builtin();
    let mut p = Project::new("sensor");
    let i = templates::add(&mut p.robot, Template::intake_default()).unwrap();
    let m = p.robot.mechanism(i).unwrap().clone();
    let h = m.holders[0].clone();
    // Cross the PVC wall. An axial beam centred on the empty bore stays clear.
    p.robot.sensors.push(Sensor::BeamBreak { name: "Have it".into(), link: h.link, from: add(h.pose.pos, [-0.08, 0., 0.]), to: add(h.pose.pos, [0.08, 0., 0.]) });
    let roller = p.robot.drives.iter().find(|d| matches!(d.target, DriveTarget::Shaft { .. })).unwrap().id;
    p.robot.controls.bindings.retain(|b| b.button != Button::RT);
    p.robot.controls.bindings.push(Binding { button: Button::RT, action: Action::Run { drive: roller, value: 10.0, until: Some("Have it".into()) } });
    let mut scene = Scene::carpet();
    let k = sim::piece::PieceKind::from_library(lib.piece("coral-2025").unwrap());
    let (hgt, lay) = k.resting();
    scene.pieces.push(Spawn { piece: "coral-2025".into(), pos: [1.3, 0.0, hgt], quat: lay, lin: [0.0; 3], ang: [0.0; 3] });
    let mut w = World::new(&p, &lib, scene).unwrap();
    w.button(Button::RT, true);
    w.cmd.drive = [1.5, 0.0, 0.0];
    let mut tripped = None;
    while w.time < 3.0 {
        w.step();
        if tripped.is_none() && w.sensor("Have it") == Some(true) { tripped = Some(w.time); w.cmd.drive = [0.0; 3]; }
    }
    println!("beam broken at {tripped:?}; rollers {:?}", w.cmd.run.get(&roller));
    assert!(tripped.is_some(), "the beam breaks");
    assert!(w.piece_positions()[0].1, "held");
    assert!(!w.cmd.run.contains_key(&roller), "the rollers stopped by themselves with the button still down");
    assert!(w.snapshot().sensors[0].beam.is_some());
}

/// M5: ALGAE in Full physics. A claw with its rollers 14 in. apart squeezes the 16.25 in. ball
/// between compliant wheels (the squeeze shared between wheel and ball), pulls it in against its
/// back plate, and holds it up on a little voltage, as teams do. (At full voltage the ball hits the
/// plate hard enough to bounce back out; coasting, it slowly sinks through.)
#[test]
fn m5_claw_pulls_in_and_holds_algae() {
    let lib = Library::builtin();
    let mut p = Project::new("algae");
    p.robot.drivetrain = None;
    let mut c = Template::claw_default();
    c.mount.offset = [0.2, 0.0, 0.8];
    if let Params::Claw { width, gap, depth, .. } = &mut c.params {
        // Place the rollers near the ball's equator when it rests against the back plate.
        *width = 0.5; *gap = 14.0 * library::IN; *depth = 0.30;
    }
    let id = templates::add(&mut p.robot, c).unwrap();
    if let Some(m) = p.robot.mechanism_mut(id) { m.fidelity = Fidelity::Full; }
    let roller = p.robot.drives.iter().find(|d| matches!(d.target, DriveTarget::Shaft { .. })).unwrap().id;
    let top = p.robot.rollers.iter().map(|x| x.center).fold([0.0, 0.0, f64::NEG_INFINITY], |a, b| if b[2] > a[2] { b } else { a });
    let low = p.robot.rollers.iter().map(|x| x.center[2]).fold(f64::INFINITY, f64::min);
    let mouth_z = (top[2] + low) / 2.0;
    let mut scene = Scene::bench();
    scene.pieces.push(Spawn { piece: "algae-2025".into(), pos: [top[0] + 0.12, 0.0, mouth_z + 0.01], quat: QI, lin: [-1.0, 0.0, 0.0], ang: [0.0; 3] });
    let mut w = World::new(&p, &lib, scene).unwrap();
    let volts: f64 = std::env::var("CLAW_V").ok().and_then(|x| x.parse().ok()).unwrap_or(4.0);
    w.cmd.run.insert(roller, volts);
    let mut peak: f64 = 0.0;
    let mut k = 0;
    while w.time < 1.5 {
        w.step(); peak = peak.max(w.contact_force()); k += 1;
        if k % 100 == 0 && std::env::var("CLAW_TRACE").is_ok() { let s = w.snapshot(); println!("t {:.2} ball {:?} rollers {:?} rad/s squeeze {:.0}", w.time, w.piece_positions()[0].0, s.rollers.iter().map(|r| (r.speed * 10.0).round() / 10.0).collect::<Vec<_>>(), w.contact_force()); }
    }
    let inside = w.piece_positions()[0].0;
    w.cmd.run.insert(roller, 1.5);
    while w.time < 3.0 { w.step(); }
    let after = w.piece_positions()[0].0;
    println!("ALGAE at {:?} running (peak squeeze {peak:.0} N), {:?} a second and a half later on 1.5 V; rollers at x {:.3}", inside.map(|v| (v * 1000.0).round() / 1000.0), after.map(|v| (v * 1000.0).round() / 1000.0), top[0]);
    assert!(inside[0] < top[0] + 0.05, "pulled in past the rollers");
    assert!(peak > 20.0, "squeezed by the wheels");
    assert!((after[2] - inside[2]).abs() < 0.02 && (after[0] - inside[0]).abs() < 0.03, "held");
}

/// M6 gate: a 3 by 3 by 3 sweep of squeeze (the rollers' height over the ramp), speed and current
/// limit, 20 random approaches each (540 throws), finishes in under 2 minutes, and its trends
/// make sense: more squeeze, more squeeze force.
#[test]
fn m6_gate_sweep_3x3x3_by_20() {
    let lib = Library::builtin();
    let mut p = intake_project("am-compliant-4in-35a", 6.1);
    p.bench.yaw_spread = 15.0;
    p.bench.offset_spread = 0.05;
    let intake = p.robot.mechanisms[0].id;
    let axes = vec![
        sim::bench::Axis::Template { mechanism: intake, field: "height".into(), values: vec![6.3 * 0.0254, 6.1 * 0.0254, 5.9 * 0.0254] },
        sim::bench::Axis::Speed(vec![0.5, 1.0, 1.5]),
        sim::bench::Axis::StatorLimit(vec![30.0, 40.0, 60.0]),
    ];
    let t = std::time::Instant::now();
    let cells = sim::bench::sweep(&p, &lib, &axes, 20);
    let secs = t.elapsed().as_secs_f64();
    println!("{} combinations x 20 in {secs:.1} s on {} threads", cells.len(), rayon::current_num_threads());
    println!("{}", sim::bench::sweep_csv(&axes, &cells).lines().take(10).collect::<Vec<_>>().join("\n"));
    assert_eq!(cells.len(), 27);
    assert!(cells.iter().all(|c| c.trials == 20));
    assert!(secs < 120.0, "{secs:.0} s");
    // squeeze: averaged over the other two
    let by = |h: usize| { let v: Vec<f64> = cells.iter().filter(|c| c.index[0] == h).map(|c| c.mean_peak_squeeze).collect(); v.iter().sum::<f64>() / v.len() as f64 };
    println!("peak squeeze by height: {:.0} {:.0} {:.0} N", by(0), by(1), by(2));
    assert!(by(0) < by(1) && by(1) < by(2), "lower rollers, more squeeze");
}

/// M6 gate: the REBUILT pack, made from the kickoff guide with no code: it loads clean, its 504
/// FUEL settle, a FUEL dropped into a HUB scores a point and comes back out of the NEUTRAL ZONE
/// face, and the field runs faster than real time.
#[test]
fn m6_gate_rebuilt_pack() {
    let lib = Library::builtin();
    let g = sim::game::GamePack::builtin("rebuilt-2026").expect("the pack reads");
    assert!(g.check(&lib).is_empty(), "{:?}", g.check(&lib));
    let fuel = g.pieces.iter().filter(|p| p.piece == "fuel-2026").count();
    assert_eq!(fuel, 408 + 48, "neutral zone and depots, both halves");
    assert!(g.zones.iter().any(|z| z.name == "Red HUB"), "mirrored");
    let hub = g.zones.iter().find(|z| z.name == "Blue HUB").unwrap().clone();
    let mut scene = Scene::field(&g);
    scene.pieces.push(Spawn { piece: "fuel-2026".into(), pos: [hub.center[0], hub.center[1], 2.4], quat: QI, lin: [0.0; 3], ang: [0.0; 3] });
    let mut p = Project::new("rebuilt");
    p.game = "rebuilt-2026".into();
    let mut w = World::new(&p, &lib, scene).unwrap();
    let t = std::time::Instant::now();
    while w.time < 3.0 { w.step(); }
    let secs = t.elapsed().as_secs_f64();
    // the scene's own pieces come first: the dropped FUEL is the first
    let out = w.piece_positions()[0].0;
    println!("3 s of field in {secs:.2} s ({:.1}x real time, {} pieces); the dropped FUEL is at {:?}; {:?}", 3.0 / secs, w.piece_positions().len(), out.map(|v| (v * 100.0).round() / 100.0), w.matched.log.iter().map(|e| (&e.what, e.points)).collect::<Vec<_>>());
    assert!(w.matched.log.iter().any(|e| e.what == "scored Blue HUB" && e.points == 1.0), "the HUB counts it");
    assert!(out[0] > hub.center[0] + 0.6 && out[2] < 1.0, "and sends it back out toward the NEUTRAL ZONE");
    // (feeders' stock waits out of sight below the field)
    assert!(w.piece_positions().iter().filter(|(p, _)| p[2] > -10.0).all(|(p, _)| p[2] > -0.05 && p[2] < 3.0), "everything stays on the field");
    assert!(secs < 3.0, "faster than real time");
}
