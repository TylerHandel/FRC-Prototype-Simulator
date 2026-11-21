//! The inspector: the numbers of whatever is selected. Every field starts filled (rule 2); a
//! change reaches the running simulation at once (rule 5) and is one undo step (rule 7).
use bevy_egui::egui::{self, RichText, Ui};
use library::units::Kind;
use model::templates::{Params, Rigging, Template};
use model::*;

use super::widgets::*;
use crate::simrun::{Msg, Sim};
use crate::state::{AppState, Selection};

pub fn show(ui: &mut Ui, st: &mut AppState, sim: &Sim) {
    match st.sel.clone() {
        Selection::Robot => robot(ui, st),
        Selection::Drivetrain => drivetrain(ui, st),
        Selection::Electrical => electrical(ui, st),
        Selection::Controls => controls(ui, st),
        Selection::Mechanism(id) => mechanism(ui, st, sim, id),
        Selection::Link(id) => link(ui, st, sim, id),
        Selection::Joint(id) => joint(ui, st, id),
        Selection::Drive(id) => drive(ui, st, sim, id),
        Selection::Roller(id) => roller(ui, st, id),
        Selection::Part(id) => super::rig::part(ui, st, &id),
    }
}

fn robot(ui: &mut Ui, st: &mut AppState) {
    heading(ui, "Robot");
    let mut name = st.project().name.clone();
    if row(ui, "Name", "", |ui| ui.text_edit_singleline(&mut name).lost_focus()) && name != st.project().name {
        st.doc.edit("Rename", |p| { p.name = name.clone(); p.robot.name = name; });
    }
    let len = st.unit(Kind::Length);
    let mass = st.unit(Kind::Mass);
    let mut f = st.project().robot.frame.clone();
    let mut ch = false;
    heading(ui, "Frame and bumpers");
    ch |= num_row(ui, "Frame length", "Front to back, without bumpers", &mut f.length, Kind::Length, len);
    ch |= num_row(ui, "Frame width", "Side to side, without bumpers", &mut f.width, Kind::Length, len);
    ch |= num_row(ui, "Bumper thickness", "", &mut f.bumper_thickness, Kind::Length, len);
    ch |= num_row(ui, "Bumper bottom", "Height of the bumpers' bottom edge off the carpet", &mut f.bumper_bottom, Kind::Length, len);
    ch |= num_row(ui, "Bumper height", "", &mut f.bumper_height, Kind::Length, len);
    ch |= num_row(ui, "Chassis mass", "Drivetrain, battery, electronics and bumpers: everything that isn't a mechanism", &mut f.chassis_mass, Kind::Mass, mass);
    ch |= num_row(ui, "Chassis CoM height", "How high the chassis's own centre of mass sits", &mut f.chassis_com_height, Kind::Length, len);
    if ch {
        st.doc.edit("~Frame", |p| {
            p.robot.frame = f.clone();
            if let Some(c) = p.robot.links.iter().position(|l| l.id == CHASSIS) {
                let parts = p.robot.links[c].parts.clone();
                let mut l = chassis_link(&f);
                l.parts = parts;
                if !p.robot.links[c].parts.is_empty() { l.shapes = p.robot.links[c].shapes.clone(); }
                p.robot.links[c] = l;
            }
            // mechanisms sit relative to the frame: rebuild them
            let ids: Vec<(Id, Template)> = p.robot.mechanisms.iter().filter_map(|m| m.template.clone().map(|t| (m.id, t))).collect();
            for (id, t) in ids { let _ = templates::rebuild(&mut p.robot, id, t); }
        });
    }
    heading(ui, "Weight");
    let total = st.project().robot.mass();
    hint(ui, &format!("Sum of the parts: {}", library::units::format(total, mass)));
    let mut weighed = st.project().robot.weighed_mass.unwrap_or(total);
    let mut use_w = st.project().robot.weighed_mass.is_some();
    if ui.checkbox(&mut use_w, "Scale every link to a weighed total").on_hover_text("If you've weighed the real robot: masses are scaled so they add up to it").changed() {
        st.doc.edit("Weighed total", |p| p.robot.weighed_mass = if use_w { Some(total) } else { None });
    }
    if use_w && num_row(ui, "Weighed total", "", &mut weighed, Kind::Mass, mass) { st.doc.edit("~Weighed", |p| p.robot.weighed_mass = Some(weighed)); }
    heading(ui, "Physics level");
    hint(ui, "Concept: pickup zones and holders, fast. Full: rollers really squeeze and slip. Each mechanism has its own; this forces all of them.");
    let mut force = st.project().force_fidelity;
    egui::ComboBox::from_id_salt("force").selected_text(match force { None => "Each mechanism its own", Some(Fidelity::Concept) => "Everything Concept", Some(Fidelity::Full) => "Everything Full" }).show_ui(ui, |ui| {
        ui.selectable_value(&mut force, None, "Each mechanism its own");
        ui.selectable_value(&mut force, Some(Fidelity::Concept), "Everything Concept");
        ui.selectable_value(&mut force, Some(Fidelity::Full), "Everything Full");
    });
    if force != st.project().force_fidelity { st.doc.edit("Physics level", |p| p.force_fidelity = force); }
    heading(ui, "Notes");
    let mut notes = st.project().notes.clone();
    if ui.add(egui::TextEdit::multiline(&mut notes).desired_rows(3)).lost_focus() && notes != st.project().notes { st.doc.edit("Notes", |p| p.notes = notes); }
}

fn drivetrain(ui: &mut Ui, st: &mut AppState) {
    heading(ui, "Drivetrain");
    let Some(mut d) = st.project().robot.drivetrain.clone() else {
        hint(ui, "This robot has no drivetrain (a Bench-only project).");
        if ui.button("Add a swerve drivetrain").clicked() { st.doc.edit("Add drivetrain", |p| p.robot.drivetrain = Some(Drivetrain::default())); }
        return;
    };
    let mut ch = false;
    row(ui, "Kind", "", |ui| {
        ch |= ui.radio_value(&mut d.kind, DriveKind::Swerve, "Swerve").changed();
        ch |= ui.radio_value(&mut d.kind, DriveKind::Tank, "Tank").changed();
    });
    if d.kind == DriveKind::Swerve {
        row(ui, "Module", "", |ui| {
            let name = st.lib.module(&d.module).map_or(d.module.clone(), |m| m.name.clone());
            egui::ComboBox::from_id_salt("module").selected_text(name).show_ui(ui, |ui| {
                for m in &st.lib.modules {
                    if ui.selectable_label(d.module == m.id, &m.name).clicked() { d.module = m.id.clone(); d.ratio = m.drive_ratio; d.wheel = m.wheel.clone(); ch = true; }
                }
            });
        });
    }
    row(ui, "Motor", "", |ui| ch |= motor_pick(ui, "dt_motor", &st.lib, &mut d.motor));
    ch |= num_row(ui, "Gear ratio", "Motor turns per wheel turn", &mut d.ratio, Kind::Ratio, st.unit(Kind::Ratio));
    row(ui, "Wheel", "", |ui| ch |= wheel_pick(ui, "dt_wheel", &st.lib, &mut d.wheel, 4.0));
    ch |= num_row(ui, "Stator limit", "Per motor", &mut d.stator_limit, Kind::Current, st.unit(Kind::Current));
    ch |= num_row(ui, "Supply limit", "Per motor", &mut d.supply_limit, Kind::Current, st.unit(Kind::Current));
    row(ui, "Carpet friction", "Wheel to carpet", |ui| ch |= ui.add(egui::DragValue::new(&mut d.mu).speed(0.01).range(0.3..=2.0)).changed());
    ch |= num_row(ui, "Top speed asked", "The most the driver can ask for", &mut d.max_speed, Kind::Speed, st.unit(Kind::Speed));
    let free = st.lib.motor(&d.motor).zip(st.lib.wheel(&d.wheel)).map(|(m, w)| m.free_rpm * library::RPM / d.ratio * w.radius()).unwrap_or(0.0);
    hint(ui, &format!("Free speed {}: real top speed is about 85 % of that.", library::units::format(free, st.unit(Kind::Speed))));
    if ch { st.doc.edit("~Drivetrain", |p| p.robot.drivetrain = Some(d)); }
    if ui.button("Remove the drivetrain").clicked() { st.doc.edit("Remove drivetrain", |p| p.robot.drivetrain = None); }
}

fn electrical(ui: &mut Ui, st: &mut AppState) {
    heading(ui, "Battery and wiring");
    let mut e = st.project().robot.electrical.clone();
    let v = st.unit(Kind::Voltage);
    let a = st.unit(Kind::Current);
    let mut ch = false;
    ch |= num_row(ui, "Battery voltage", "Open-circuit, charged", &mut e.battery_voltage, Kind::Voltage, v);
    row(ui, "Battery resistance", "A fresh battery is about 0.011 ohm; a tired one 0.02 or more", |ui| ch |= ui.add(egui::DragValue::new(&mut e.battery_resistance).speed(0.0005).range(0.003..=0.05).suffix(" ohm")).changed());
    row(ui, "Wiring resistance", "Main breaker, PDH and the main wires", |ui| ch |= ui.add(egui::DragValue::new(&mut e.wiring_resistance).speed(0.0005).range(0.0..=0.05).suffix(" ohm")).changed());
    ch |= num_row(ui, "Always-on load", "Controller, radio, cameras, LEDs", &mut e.fixed_load, Kind::Current, a);
    ch |= num_row(ui, "Main breaker", "", &mut e.main_breaker, Kind::Current, a);
    ch |= num_row(ui, "Brownout below", "roboRIO: 6.75 V. Systemcore's (2027) isn't published yet.", &mut e.brownout, Kind::Voltage, v);
    ch |= ui.checkbox(&mut e.trip_breakers, "Trip the main breaker when it overheats (it only warns otherwise)").changed();
    if ch { st.doc.edit("~Electrical", |p| p.robot.electrical = e); }
}

fn controls(ui: &mut Ui, st: &mut AppState) {
    heading(ui, "Presets");
    hint(ui, "A preset sends each of its drives to a target (an angle, a height). Bind it to a button below.");
    let r = st.project().robot.clone();
    let mut edit: Option<Box<dyn FnOnce(&mut Robot)>> = None;
    for p in &r.controls.presets {
        egui::CollapsingHeader::new(&p.name).id_salt(("preset", p.id)).show(ui, |ui| {
            for (ti, t) in p.targets.iter().enumerate() {
                let Some(d) = r.drive(t.drive) else { continue };
                let (kind, unit) = match d.target {
                    DriveTarget::Joint { joint, .. } if r.joint(joint).is_some_and(|j| j.kind == JointKind::Hinge) => (Kind::Angle, st.unit(Kind::Angle)),
                    DriveTarget::Joint { .. } => (Kind::Length, st.unit(Kind::Length)),
                    DriveTarget::Shaft { .. } => (Kind::Voltage, st.unit(Kind::Voltage)),
                };
                let mut v = t.value;
                if num_row(ui, &d.name, "", &mut v, kind, unit) {
                    let pid = p.id;
                    edit = Some(Box::new(move |r: &mut Robot| { if let Some(p) = r.controls.presets.iter_mut().find(|x| x.id == pid) { p.targets[ti].value = v; } }));
                }
            }
            if ui.small_button("Remove this preset").clicked() {
                let pid = p.id;
                edit = Some(Box::new(move |r: &mut Robot| { r.controls.presets.retain(|x| x.id != pid); r.controls.bindings.retain(|b| !matches!(b.action, Action::Preset { preset } if preset == pid)); }));
            }
        });
    }
    heading(ui, "Buttons");
    hint(ui, "Gamepad button (keyboard key): what it does.");
    egui::Grid::new("bindings").striped(true).show(ui, |ui| {
        for (bi, b) in r.controls.bindings.iter().enumerate() {
            ui.label(format!("{:?} ({})", b.button, b.button.key()));
            let what = match &b.action {
                Action::Preset { preset } => format!("go to {}", r.controls.presets.iter().find(|p| p.id == *preset).map_or("?".into(), |p| p.name.clone())),
                Action::Run { drive, value, until } => format!("run {} at {value}{}", r.drive(*drive).map_or("?".into(), |d| d.name.clone()), until.as_ref().map_or(String::new(), |u| format!(" until {u}"))),
                Action::Release { mechanism } => format!("let go ({})", r.mechanism(*mechanism).map_or("?".into(), |m| m.name.clone())),
            };
            ui.label(what);
            // a run can stop itself on a sensor
            if let Action::Run { until, .. } = &b.action {
                let mut u = until.clone();
                egui::ComboBox::from_id_salt(("until", bi)).selected_text(u.clone().map_or("no stop".into(), |x| format!("until {x}"))).width(110.0).show_ui(ui, |ui| {
                    ui.selectable_value(&mut u, None, "no stop");
                    for x in &r.sensors { ui.selectable_value(&mut u, Some(x.name().to_string()), format!("until {}", x.name())); }
                });
                if u != *until { edit = Some(Box::new(move |r: &mut Robot| if let Action::Run { until, .. } = &mut r.controls.bindings[bi].action { *until = u; })); }
            } else { ui.label(""); }
            let mut btn = b.button;
            egui::ComboBox::from_id_salt(("bind", bi)).selected_text("change").width(70.0).show_ui(ui, |ui| { for x in Button::ALL { ui.selectable_value(&mut btn, x, format!("{x:?}")); } });
            if btn != b.button { edit = Some(Box::new(move |r: &mut Robot| r.controls.bindings[bi].button = btn)); }
            if ui.small_button("x").clicked() { edit = Some(Box::new(move |r: &mut Robot| { r.controls.bindings.remove(bi); })); }
            ui.end_row();
        }
    });
    if let Some(f) = edit { st.doc.edit("Controls", |p| f(&mut p.robot)); }
}

fn template_fields(ui: &mut Ui, st: &AppState, t: &mut Template) -> bool {
    let len = st.unit(Kind::Length);
    let mass = st.unit(Kind::Mass);
    let ang = st.unit(Kind::Angle);
    let ratio = st.unit(Kind::Ratio);
    let lib = &st.lib;
    let mut ch = false;
    let count = |ui: &mut Ui, label: &str, v: &mut u32, lo: u32, hi: u32| row(ui, label, "", |ui| ui.add(egui::DragValue::new(v).range(lo..=hi)).changed());
    let deg = |v: &mut f64| *v * std::f64::consts::PI / 180.0;
    match &mut t.params {
        Params::Elevator { stages, travel, height, width, rigging, spool_diameter, motor, motors, ratio: r, carriage_mass, stage_mass } => {
            ch |= count(ui, "Moving stages", stages, 1, 4);
            ch |= num_row(ui, "Carriage travel", "How far the carriage goes, bottom to top", travel, Kind::Length, len);
            ch |= num_row(ui, "Fixed stage height", "", height, Kind::Length, len);
            ch |= num_row(ui, "Width", "", width, Kind::Length, len);
            row(ui, "Rigging", "Cascade: each stage moves with the one below. Continuous: the carriage moves first.", |ui| {
                ch |= ui.radio_value(rigging, Rigging::Cascade, "Cascade").changed();
                ch |= ui.radio_value(rigging, Rigging::Continuous, "Continuous").changed();
            });
            ch |= num_row(ui, "Spool diameter", "Or the pulley's pitch diameter", spool_diameter, Kind::Length, len);
            row(ui, "Motor", "", |ui| ch |= motor_pick(ui, "el_m", lib, motor));
            ch |= count(ui, "Motors", motors, 1, 4);
            ch |= num_row(ui, "Gear ratio", "Motor turns per spool turn", r, Kind::Ratio, ratio);
            ch |= num_row(ui, "Carriage mass", "With what it carries", carriage_mass, Kind::Mass, mass);
            ch |= num_row(ui, "Mass per stage", "", stage_mass, Kind::Mass, mass);
        }
        Params::Arm { length, width, mass: m, motor, motors, ratio: r, min_deg, max_deg, start_deg } | Params::Wrist { length, width, mass: m, motor, motors, ratio: r, min_deg, max_deg, start_deg } => {
            ch |= num_row(ui, "Length", "Pivot to tip", length, Kind::Length, len);
            ch |= num_row(ui, "Width", "", width, Kind::Length, len);
            ch |= num_row(ui, "Mass", "", m, Kind::Mass, mass);
            row(ui, "Motor", "", |ui| ch |= motor_pick(ui, "arm_m", lib, motor));
            ch |= count(ui, "Motors", motors, 1, 4);
            ch |= num_row(ui, "Gear ratio", "Motor turns per arm turn", r, Kind::Ratio, ratio);
            for (label, v) in [("Lowest angle", min_deg), ("Highest angle", max_deg), ("Starts at", start_deg)] {
                let mut rad = deg(v);
                if num_row(ui, label, "0 is level and forward; up is positive", &mut rad, Kind::Angle, ang) { *v = rad.to_degrees(); ch = true; }
            }
        }
        Params::Telescope { travel, length, mass: m, motor, motors, ratio: r, spool_diameter } => {
            ch |= num_row(ui, "Travel", "", travel, Kind::Length, len);
            ch |= num_row(ui, "Length", "", length, Kind::Length, len);
            ch |= num_row(ui, "Mass", "", m, Kind::Mass, mass);
            row(ui, "Motor", "", |ui| ch |= motor_pick(ui, "tel_m", lib, motor));
            ch |= count(ui, "Motors", motors, 1, 4);
            ch |= num_row(ui, "Gear ratio", "", r, Kind::Ratio, ratio);
            ch |= num_row(ui, "Spool diameter", "", spool_diameter, Kind::Length, len);
        }
        Params::Intake { width, height, ramp_deg, rollers, wheel, wheels_per_roller, motor, ratio: r, mass: m, deploy, deploy_ratio } => {
            ch |= num_row(ui, "Width", "", width, Kind::Length, len);
            ch |= num_row(ui, "Roller height", "From each roller's centre to the ramp: this sets the squeeze", height, Kind::Length, len);
            let mut rad = deg(ramp_deg);
            if num_row(ui, "Ramp angle", "How steeply the ramp climbs over the bumper", &mut rad, Kind::Angle, ang) { *ramp_deg = rad.to_degrees(); ch = true; }
            ch |= count(ui, "Rollers", rollers, 1, 2);
            row(ui, "Wheels", "What's on the rollers (D19: swap it here, or per roller)", |ui| ch |= wheel_pick(ui, "in_w", lib, wheel, 4.0));
            ch |= count(ui, "Wheels per roller", wheels_per_roller, 1, 8);
            row(ui, "Motor", "", |ui| ch |= motor_pick(ui, "in_m", lib, motor));
            ch |= num_row(ui, "Gear ratio", "Motor turns per roller turn", r, Kind::Ratio, ratio);
            ch |= num_row(ui, "Mass", "", m, Kind::Mass, mass);
            ch |= ui.checkbox(deploy, "Pivots out (a deploy motor)").changed();
            if *deploy { ch |= num_row(ui, "Deploy ratio", "", deploy_ratio, Kind::Ratio, ratio); }
            // the squeeze it puts on each piece
            if let Some(w) = lib.wheel(wheel) {
                for p in &lib.pieces {
                    let d = piece_diameter(p);
                    let sq = d + w.radius() - *height;
                    let c = if sq < 0.0 { BAD } else if sq < 0.003 { WARN } else { MUTED };
                    hint(ui, "");
                    ui.label(RichText::new(format!("Squeeze on {}: {}", p.name, library::units::format(sq, len))).color(c).size(12.0));
                }
            }
        }
        Params::Claw { width, gap, depth, wheel, motor, ratio: r, mass: m } => {
            ch |= num_row(ui, "Width", "", width, Kind::Length, len);
            ch |= num_row(ui, "Gap between rollers", "Smaller than the piece by the squeeze", gap, Kind::Length, len);
            ch |= num_row(ui, "Depth", "", depth, Kind::Length, len);
            row(ui, "Wheels", "", |ui| ch |= wheel_pick(ui, "cl_w", lib, wheel, 2.0));
            row(ui, "Motor", "", |ui| ch |= motor_pick(ui, "cl_m", lib, motor));
            ch |= num_row(ui, "Gear ratio", "", r, Kind::Ratio, ratio);
            ch |= num_row(ui, "Mass", "", m, Kind::Mass, mass);
            for p in &lib.pieces {
                let sq = piece_diameter(p) - *gap;
                ui.label(RichText::new(format!("Squeeze on {}: {}", p.name, library::units::format(sq, len))).color(if sq < 0.0 { BAD } else { MUTED }).size(12.0));
            }
        }
        Params::Shooter { wheel, width, motor, motors, ratio: r, hood_deg, mass: m, efficiency } => {
            row(ui, "Flywheel wheels", "", |ui| ch |= wheel_pick(ui, "sh_w", lib, wheel, 4.0));
            ch |= num_row(ui, "Width", "", width, Kind::Length, len);
            row(ui, "Motor", "", |ui| ch |= motor_pick(ui, "sh_m", lib, motor));
            ch |= count(ui, "Motors", motors, 1, 4);
            ch |= num_row(ui, "Gear ratio", "", r, Kind::Ratio, ratio);
            let mut rad = deg(hood_deg);
            if num_row(ui, "Hood angle", "Launch angle above level", &mut rad, Kind::Angle, ang) { *hood_deg = rad.to_degrees(); ch = true; }
            ch |= num_row(ui, "Mass", "", m, Kind::Mass, mass);
            row(ui, "Exit speed share", "Piece speed over the wheel's surface speed (Concept)", |ui| ch |= ui.add(egui::DragValue::new(efficiency).speed(0.01).range(0.1..=1.0)).changed());
        }
        Params::Turret { radius, motor, ratio: r, mass: m, range_deg } => {
            ch |= num_row(ui, "Radius", "", radius, Kind::Length, len);
            row(ui, "Motor", "", |ui| ch |= motor_pick(ui, "tu_m", lib, motor));
            ch |= num_row(ui, "Gear ratio", "", r, Kind::Ratio, ratio);
            ch |= num_row(ui, "Mass", "", m, Kind::Mass, mass);
            let mut rad = deg(range_deg);
            if num_row(ui, "Range", "", &mut rad, Kind::Angle, ang) { *range_deg = rad.to_degrees(); ch = true; }
        }
        Params::Climber { length, motor, motors, ratio: r, mass: m } => {
            ch |= num_row(ui, "Arm length", "", length, Kind::Length, len);
            row(ui, "Motor", "", |ui| ch |= motor_pick(ui, "cb_m", lib, motor));
            ch |= count(ui, "Motors", motors, 1, 4);
            ch |= num_row(ui, "Gear ratio", "", r, Kind::Ratio, ratio);
            ch |= num_row(ui, "Mass", "", m, Kind::Mass, mass);
        }
    }
    ch
}

pub fn piece_diameter(p: &library::Piece) -> f64 {
    use library::{PieceShape, IN};
    match p.shape { PieceShape::Sphere { d_in } => d_in * IN, PieceShape::Tube { outer_d_in, .. } => outer_d_in * IN, PieceShape::Cylinder { d_in, .. } => d_in * IN, PieceShape::Box { z_in, .. } => z_in * IN }
}

fn mechanism(ui: &mut Ui, st: &mut AppState, sim: &Sim, id: Id) {
    let Some(m) = st.project().robot.mechanism(id).cloned() else { st.sel = Selection::Robot; return };
    heading(ui, &m.name);
    let mut name = m.name.clone();
    if row(ui, "Name", "", |ui| ui.text_edit_singleline(&mut name).lost_focus()) && name != m.name {
        st.doc.edit("Rename", |p| if let Some(m) = p.robot.mechanism_mut(id) { m.name = name; });
    }
    let mut fid = m.fidelity;
    row(ui, "Physics", "Concept: a pickup zone takes pieces after a moment. Full: the rollers really squeeze, grip and slip.", |ui| {
        ui.radio_value(&mut fid, Fidelity::Concept, "Concept");
        ui.radio_value(&mut fid, Fidelity::Full, "Full");
    });
    if fid != m.fidelity { st.doc.edit("Physics level", |p| if let Some(m) = p.robot.mechanism_mut(id) { m.fidelity = fid; }); }
    if let Some(mut t) = m.template.clone() {
        // where it rides
        let carriers: Vec<(Option<Id>, String)> = std::iter::once((None, "the chassis".to_string()))
            .chain(st.project().robot.mechanisms.iter().filter(|x| x.id != id && x.template.is_some()).map(|x| (Some(x.id), x.name.clone()))).collect();
        let current = carriers.iter().find(|c| c.0 == t.mount.on).map_or("?".to_string(), |c| c.1.clone());
        let mut ch = false;
        row(ui, "Mounted on", "", |ui| {
            egui::ComboBox::from_id_salt("mount").selected_text(current).show_ui(ui, |ui| {
                for (cid, cname) in &carriers { if ui.selectable_label(t.mount.on == *cid, cname).clicked() { t.mount.on = *cid; ch = true; } }
            });
        });
        let len = st.unit(Kind::Length);
        for (i, axis) in ["Mount x (forward)", "Mount y (left)", "Mount z (up)"].iter().enumerate() {
            ch |= num_row(ui, axis, "From the chassis's centre on the floor, or from what it's mounted on", &mut t.mount.offset[i], Kind::Length, len);
        }
        ui.separator();
        ch |= template_fields(ui, st, &mut t);
        if ch { st.doc.edit(&format!("~{}", m.name), |p| { let _ = templates::rebuild(&mut p.robot, id, t); }); }
    }
    // its drives
    let r = st.project().robot.clone();
    let mine: Vec<&Drive> = r.drives.iter().filter(|d| match d.target {
        DriveTarget::Joint { joint, .. } => r.joint(joint).is_some_and(|j| m.links.contains(&j.child)),
        DriveTarget::Shaft { shaft } => r.shaft(shaft).is_some_and(|s| m.links.contains(&s.link)),
    }).collect();
    if !mine.is_empty() { heading(ui, "Motors and control"); }
    for d in mine {
        if ui.selectable_label(false, format!("{}: {} x{} at {:.2}:1 ({})", d.name, st.lib.motor(&d.motor).map_or(d.motor.clone(), |x| x.name.clone()), d.count, d.ratio, control_name(&d.control))).clicked() {
            st.sel = Selection::Drive(d.id);
        }
    }
    let rollers: Vec<&Roller> = r.rollers.iter().filter(|x| r.shaft(x.shaft).is_some_and(|s| m.links.contains(&s.link))).collect();
    if !rollers.is_empty() { heading(ui, "Rollers"); hint(ui, "Click one to swap its wheel."); }
    for x in rollers {
        if ui.selectable_label(false, format!("{}: {}", x.name, st.lib.wheel(&x.wheel).map_or(x.wheel.clone(), |w| w.name.clone()))).clicked() { st.sel = Selection::Roller(x.id); }
    }
    if !m.holders.is_empty() && sim.playing() && ui.button("Let go of what it holds").clicked() { sim.send(Msg::Release(id)); }
    ui.add_space(8.0);
    ui.horizontal(|ui| {
        if ui.button("Try it on the Bench").on_hover_text("Send pieces at this mechanism and measure").clicked() {
            st.doc.edit("Full physics", |p| if let Some(m) = p.robot.mechanism_mut(id) { m.fidelity = Fidelity::Full; });
            st.ws = crate::state::Workspace::Bench;
        }
        if ui.button(RichText::new("Remove").color(BAD)).on_hover_text("Delete").clicked() {
            st.doc.edit(&format!("Remove {}", m.name), |p| p.robot.remove_mechanism(id));
            st.sel = Selection::Robot;
        }
    });
}

fn control_name(c: &Control) -> &'static str {
    match c { Control::Voltage => "volts", Control::Velocity { .. } => "speed loop", Control::Position { .. } => "position PID", Control::Ideal => "ideal profile" }
}

fn drive(ui: &mut Ui, st: &mut AppState, sim: &Sim, id: Id) {
    let Some(mut d) = st.project().robot.drive(id).cloned() else { st.sel = Selection::Robot; return };
    heading(ui, &d.name);
    let mut ch = false;
    row(ui, "Motor", "", |ui| ch |= motor_pick(ui, "dr_m", &st.lib, &mut d.motor));
    let mut n = d.count;
    if row(ui, "How many", "", |ui| ui.add(egui::DragValue::new(&mut n).range(1..=6)).changed()) { d.count = n; ch = true; }
    ch |= num_row(ui, "Gear ratio", "Motor turns per output turn", &mut d.ratio, Kind::Ratio, st.unit(Kind::Ratio));
    row(ui, "Gearbox efficiency", "", |ui| ch |= ui.add(egui::DragValue::new(&mut d.efficiency).speed(0.01).range(0.5..=1.0)).changed());
    ch |= num_row(ui, "Stator limit", "Per motor", &mut d.stator_limit, Kind::Current, st.unit(Kind::Current));
    ch |= num_row(ui, "Supply limit", "Per motor", &mut d.supply_limit, Kind::Current, st.unit(Kind::Current));
    if let DriveTarget::Joint { joint, spool_radius } = &mut d.target {
        if st.project().robot.joint(*joint).is_some_and(|j| j.kind == JointKind::Slide) {
            let mut dia = *spool_radius * 2.0;
            if num_row(ui, "Spool diameter", "", &mut dia, Kind::Length, st.unit(Kind::Length)) { *spool_radius = dia / 2.0; ch = true; }
        }
    }
    ch |= ui.checkbox(&mut d.brake, "Brake when stopped (coast if not)").changed();
    heading(ui, "Control");
    let is_joint = matches!(d.target, DriveTarget::Joint { .. });
    egui::ComboBox::from_id_salt("ctl").selected_text(control_name(&d.control)).show_ui(ui, |ui| {
        if ui.selectable_label(matches!(d.control, Control::Voltage), "volts").clicked() { d.control = Control::Voltage; ch = true; }
        if ui.selectable_label(matches!(d.control, Control::Velocity { .. }), "speed loop").clicked() { d.control = Control::Velocity { kp: 0.05, ki: 0.0, ks: 0.1, kv: 0.0 }; ch = true; }
        if is_joint {
            if ui.selectable_label(matches!(d.control, Control::Ideal), "ideal profile (Concept)").on_hover_text("The fastest the motor allows, with perfect gravity and inertia compensation").clicked() { d.control = Control::Ideal; ch = true; }
            if ui.selectable_label(matches!(d.control, Control::Position { .. }), "position PID").clicked() { d.control = Control::Position { kp: 20.0, ki: 0.0, kd: 0.5, ks: 0.1, kg: 0.3, kv: 0.0, cruise: 0.0, accel: 0.0 }; ch = true; }
        }
    });
    match &mut d.control {
        Control::Position { kp, ki, kd, ks, kg, kv, cruise, accel } => {
            for (label, v, sp) in [("kP (V per unit)", kp, 0.5), ("kI", ki, 0.1), ("kD (V per unit/s)", kd, 0.05), ("kS (V)", ks, 0.01), ("kG (V)", kg, 0.01), ("kV (V per unit/s)", kv, 0.01), ("Cruise speed", cruise, 0.01), ("Acceleration", accel, 0.05)] {
                row(ui, label, "", |ui| ch |= ui.add(egui::DragValue::new(v).speed(sp)).changed());
            }
        }
        Control::Velocity { kp, ki, ks, kv } => {
            for (label, v, sp) in [("kP", kp, 0.005), ("kI", ki, 0.005), ("kS (V)", ks, 0.01), ("kV (0: from the motor)", kv, 0.001)] {
                row(ui, label, "", |ui| ch |= ui.add(egui::DragValue::new(v).speed(sp)).changed());
            }
        }
        _ => {}
    }
    if is_joint && ui.button("Gains from the model").on_hover_text("kS, kG, kV and kP/kD from the mechanism's own inertia and gravity, at its pose now").clicked() {
        // asks a fresh world for the plant at the current pose
        let lib = st.lib.clone();
        if let Ok(w) = sim::world::World::new(st.project(), &lib, sim::world::Scene::carpet()) {
            if let Some(c) = w.gains_from_model(id) { d.control = c; ch = true; }
        }
    }
    if ch { st.doc.edit(&format!("~{}", d.name), |p| if let Some(x) = p.robot.drive_mut(id) { *x = d.clone(); }); }
    try_drive(ui, st, sim, &d);
    // live numbers
    if let Some(s) = sim.snap().and_then(|s| s.drives.iter().find(|x| x.id == id).copied()) {
        heading(ui, "Now");
        ui.label(format!("{:.1} V, {:.1} A stator, {:.1} A from the battery{}", s.volts, s.current, s.supply, if s.limited { " (current-limited)" } else { "" }));
    }
}

/// Run one drive by hand while the sim plays: hold a button to run it, or pick where a joint goes.
fn try_drive(ui: &mut Ui, st: &AppState, sim: &Sim, d: &Drive) {
    heading(ui, "Try it");
    if !sim.playing() { hint(ui, "Press Play (Space) first."); }
    // what "run" means: volts, or half the output's free speed for a speed loop
    let run = match d.control {
        Control::Velocity { .. } => st.lib.motor(&d.motor).map_or(10.0, |m| m.free_rpm * std::f64::consts::TAU / 60.0 / d.ratio.max(1e-6) * 0.5),
        _ => 6.0,
    };
    let what = if matches!(d.control, Control::Velocity { .. }) { "half speed".to_string() } else { "6 V".to_string() };
    ui.horizontal(|ui| {
        for (label, v) in [(format!("Hold: back ({what})"), -run), (format!("Hold: forward ({what})"), run)] {
            let b = ui.add(egui::Button::new(label).sense(egui::Sense::drag()));
            if b.drag_started() { sim.send(Msg::Run(d.id, Some(v))); }
            if b.drag_stopped() { sim.send(Msg::Run(d.id, None)); }
        }
    });
    let DriveTarget::Joint { joint, .. } = d.target else { return };
    if matches!(d.control, Control::Voltage | Control::Velocity { .. }) { return; }
    let Some(j) = st.project().robot.joint(joint) else { return };
    let Some([lo, hi]) = j.range else { return };
    let id = ui.make_persistent_id(("goto", d.id));
    let mut v: f64 = ui.data_mut(|m| m.get_temp(id)).unwrap_or(j.initial);
    let slide = j.kind == JointKind::Slide;
    let shown = |x: f64| if slide { x } else { x.to_degrees() };
    let mut t = shown(v);
    let r = row(ui, "Go to", "Moves the joint with its controller", |ui| ui.add(egui::Slider::new(&mut t, shown(lo)..=shown(hi)).suffix(if slide { " m" } else { " deg" })));
    if r.changed() {
        v = if slide { t } else { t.to_radians() };
        ui.data_mut(|m| m.insert_temp(id, v));
        sim.send(Msg::Target(d.id, v));
    }
}

fn link(ui: &mut Ui, st: &mut AppState, sim: &Sim, id: Id) {
    let Some(l) = st.project().robot.link(id).cloned() else { st.sel = Selection::Robot; return };
    heading(ui, &l.name);
    let mut mass = l.mass_or_estimate();
    if num_row(ui, "Mass", if l.mass.is_some() { "Typed" } else { "Estimated from its shapes or parts" }, &mut mass, Kind::Mass, st.unit(Kind::Mass)) {
        st.doc.edit("~Link mass", |p| if let Some(l) = p.robot.link_mut(id) { l.mass = Some(mass); });
    }
    let mut col = l.collision;
    row(ui, "Collision", "Box: one box round it. Simple: its 24 biggest parts (or its own blocks). Detailed: every part, thin ones as flat slabs. None: passes through.", |ui| {
        egui::ComboBox::from_id_salt("col").selected_text(format!("{col:?}")).show_ui(ui, |ui| {
            for c in [Collision::Box, Collision::Simple, Collision::Detailed, Collision::None] { ui.selectable_value(&mut col, c, format!("{c:?}")); }
        });
    });
    if col != l.collision {
        let cad = if l.parts.is_empty() { None } else { super::rig::load_cadpack(st) };
        st.doc.edit("Collision", |p| {
            if let Some(l) = p.robot.link_mut(id) { l.collision = col; }
            if let Some((cp, to)) = &cad { cad::propose::fit_collision(&mut p.robot, cp, to); }
        });
    }
    hint(ui, &format!("{} shapes, {} CAD parts", l.shapes.len(), l.parts.len()));
    if let Some(j) = st.project().robot.joint_of(id) {
        if ui.selectable_label(false, format!("Moves on: {}", j.name)).clicked() { st.sel = Selection::Joint(j.id); }
    }
    sensors(ui, st, sim, id);
}

/// The beam breaks on a link: where each beam runs, whether it's broken now, add and remove.
fn sensors(ui: &mut Ui, st: &mut AppState, sim: &Sim, id: Id) {
    heading(ui, "Beam breaks");
    hint(ui, "A beam that reads true while a piece crosses it. Bind a button to run a motor until it trips (Controls).");
    let r = st.project().robot.clone();
    let snap = sim.snap();
    let len = st.unit(Kind::Length).clone();
    let mut edit: Option<(usize, Sensor)> = None;
    let mut remove = None;
    for (si, x) in r.sensors.iter().enumerate() {
        let Sensor::BeamBreak { name, link, from, to } = x else { continue };
        if *link != id { continue; }
        let on = snap.as_ref().and_then(|s| s.sensors.iter().find(|y| &y.name == name)).map(|y| y.on);
        egui::CollapsingHeader::new(format!("{name}{}", match on { Some(true) => "  (broken)", Some(false) => "  (clear)", None => "" })).id_salt(("sensor", si)).show(ui, |ui| {
            let (mut f, mut t, mut n) = (*from, *to, name.clone());
            let mut ch = row(ui, "Name", "Buttons find it by this name", |ui| ui.text_edit_singleline(&mut n).lost_focus()) && n != *name;
            for (k, axis) in ["x (forward)", "y (left)", "z (up)"].iter().enumerate() {
                ch |= num_row(ui, &format!("From {axis}"), "", &mut f[k], Kind::Length, &len);
            }
            for (k, axis) in ["x (forward)", "y (left)", "z (up)"].iter().enumerate() {
                ch |= num_row(ui, &format!("To {axis}"), "", &mut t[k], Kind::Length, &len);
            }
            if ch { edit = Some((si, Sensor::BeamBreak { name: n, link: id, from: f, to: t })); }
            if ui.small_button(RichText::new("Remove").color(BAD)).clicked() { remove = Some(si); }
        });
    }
    if ui.button("Add a beam break").on_hover_text("Across the link, side to side, through its middle. Move it with the numbers.").clicked() {
        // across the middle of the link's shapes (or its parts), left to right
        let l = r.link(id);
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        let mut grow = |p: [f64; 3]| for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); };
        if let Some(l) = l {
            for s in &l.shapes { let e = s.kind.extents(); for k in [-0.5, 0.5] { grow(model::math::add(s.pose.pos, [e[0] * k, e[1] * k, e[2] * k])); } }
            for p in r.parts.iter().filter(|p| l.parts.contains(&p.id) && !p.hardware) { grow(p.fingerprint.bbox[0]); grow(p.fingerprint.bbox[1]); }
        }
        let c = if lo[0].is_finite() { model::math::lerp(lo, hi, 0.5) } else { [0.0, 0.0, 0.3] };
        let w = if lo[1].is_finite() { (hi[1] - lo[1]) / 2.0 } else { 0.2 };
        let n = (1..).map(|i| format!("Beam break {i}")).find(|n| !r.sensors.iter().any(|x| x.name() == n)).unwrap();
        st.doc.edit("Add a beam break", |p| p.robot.sensors.push(Sensor::BeamBreak { name: n, link: id, from: [c[0], c[1] - w, c[2]], to: [c[0], c[1] + w, c[2]] }));
    }
    if let Some((si, x)) = edit {
        let old = r.sensors[si].name().to_string();
        st.doc.edit("~Beam break", |p| {
            let new = x.name().to_string();
            p.robot.sensors[si] = x;
            // buttons follow a renamed sensor
            for b in &mut p.robot.controls.bindings { if let Action::Run { until: Some(u), .. } = &mut b.action { if *u == old { *u = new.clone(); } } }
        });
    }
    if let Some(si) = remove { st.doc.edit("Remove a beam break", |p| { p.robot.sensors.remove(si); }); }
}

fn joint(ui: &mut Ui, st: &mut AppState, id: Id) {
    let Some(mut j) = st.project().robot.joint(id).cloned() else { st.sel = Selection::Robot; return };
    heading(ui, &j.name);
    if !j.source.is_empty() { hint(ui, &format!("From: {}", j.source)); }
    let (kind, unit) = match j.kind { JointKind::Hinge => (Kind::Angle, st.unit(Kind::Angle)), _ => (Kind::Length, st.unit(Kind::Length)) };
    let mut ch = false;
    row(ui, "Kind", "", |ui| {
        ch |= ui.radio_value(&mut j.kind, JointKind::Hinge, "Hinge").changed();
        ch |= ui.radio_value(&mut j.kind, JointKind::Slide, "Slide").changed();
        ch |= ui.radio_value(&mut j.kind, JointKind::Fixed, "Fixed").changed();
    });
    let mut limited = j.range.is_some();
    if ui.checkbox(&mut limited, "Travel limits").changed() { j.range = if limited { Some([0.0, if kind == Kind::Angle { 1.57 } else { 0.5 }]) } else { None }; ch = true; }
    if let Some(r) = &mut j.range {
        ch |= num_row(ui, "From", "", &mut r[0], kind, unit);
        ch |= num_row(ui, "To", "", &mut r[1], kind, unit);
    }
    ch |= num_row(ui, "Starts at", "", &mut j.initial, kind, unit);
    let len = st.unit(Kind::Length);
    for (i, a) in ["Axis point x", "Axis point y", "Axis point z"].iter().enumerate() { ch |= num_row(ui, a, "", &mut j.anchor[i], Kind::Length, len); }
    row(ui, "Axis direction", "", |ui| {
        for (i, a) in ["x", "y", "z"].iter().enumerate() { ch |= ui.add(egui::DragValue::new(&mut j.axis[i]).speed(0.01).range(-1.0..=1.0).prefix(*a)).changed(); }
    });
    row(ui, "Damping", "", |ui| ch |= ui.add(egui::DragValue::new(&mut j.damping).speed(0.01).range(0.0..=100.0)).changed());
    row(ui, "Friction", "", |ui| ch |= ui.add(egui::DragValue::new(&mut j.friction).speed(0.01).range(0.0..=100.0)).changed());
    let mut spring = j.spring.is_some();
    if ui.checkbox(&mut spring, "Spring or gas spring").changed() { j.spring = if spring { Some(Spring { preload: 0.0, stiffness: 0.0 }) } else { None }; ch = true; }
    if let Some(s) = &mut j.spring {
        row(ui, "Preload", "Force (N) or torque (N*m) at the start", |ui| ch |= ui.add(egui::DragValue::new(&mut s.preload).speed(0.1)).changed());
        row(ui, "Stiffness", "Per m or per rad of travel", |ui| ch |= ui.add(egui::DragValue::new(&mut s.stiffness).speed(0.1)).changed());
    }
    if ch {
        j.axis = model::math::norm(j.axis);
        st.doc.edit(&format!("~{}", j.name), |p| if let Some(x) = p.robot.joint_mut(id) { *x = j; });
    }
}

fn roller(ui: &mut Ui, st: &mut AppState, id: Id) {
    let Some(mut x) = st.project().robot.roller(id).cloned() else { st.sel = Selection::Robot; return };
    heading(ui, &x.name);
    hint(ui, "Swap the wheel here (D19). Its geometry (where it is, how wide) stays the design's.");
    let mut ch = false;
    let near = x.diameter.map_or(st.lib.wheel(&x.wheel).map_or(4.0, |w| w.diameter_in), |d| d / library::IN);
    row(ui, "Wheel", "", |ui| ch |= wheel_pick(ui, "rl_w", &st.lib, &mut x.wheel, near));
    let mut whole = false;
    if ui.button("Use this wheel on every roller of its shaft").clicked() { whole = true; }
    if let Some(w) = st.lib.wheel(&x.wheel) {
        let est = if w.measured && w.k_area.is_some() { "measured".to_string() } else { format!("estimated from {:.0}A", w.durometer_a) };
        hint(ui, &format!("{} | stiffness {:.2e} Pa/m ({est})", w.source, w.k_area()));
    }
    let len = st.unit(Kind::Length);
    let mut own = x.diameter.is_some();
    if ui.checkbox(&mut own, "Its own diameter (the CAD's), not the wheel's").changed() { x.diameter = if own { Some(x.radius(&st.lib) * 2.0) } else { None }; ch = true; }
    if let Some(d) = &mut x.diameter { ch |= num_row(ui, "Diameter", "", d, Kind::Length, len); }
    ch |= num_row(ui, "Width", "", &mut x.width, Kind::Length, len);
    row(ui, "Speed vs shaft", "Roller turns per shaft turn: negative turns it the other way", |ui| ch |= ui.add(egui::DragValue::new(&mut x.ratio).speed(0.01)).changed());
    if ch { st.doc.edit("~Roller", |p| if let Some(r) = p.robot.roller_mut(id) { *r = x.clone(); }); }
    if whole {
        let (shaft, wheel) = (x.shaft, x.wheel.clone());
        st.doc.edit("Wheel on the shaft", |p| for r in &mut p.robot.rollers { if r.shaft == shaft { r.wheel = wheel.clone(); } });
    }
}
