//! Configurable mechanism dimensions shared by the template editors.
use bevy_egui::egui::{self, RichText, Ui};
use library::units::Kind;
use model::templates::{Params, Rigging, Template};

use super::widgets::*;
use crate::state::AppState;

pub(super) fn template_fields(ui: &mut Ui, st: &AppState, t: &mut Template) -> bool {
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
        Params::Claw { width, gap, depth, open_plates, wheel, motor, ratio: r, mass: m } => {
            ch |= num_row(ui, "Width", "", width, Kind::Length, len);
            ch |= num_row(ui, "Gap between rollers", "Smaller than the piece by the squeeze", gap, Kind::Length, len);
            ch |= num_row(ui, "Depth", "", depth, Kind::Length, len);
            ch |= ui.checkbox(open_plates, "Open side plates for through-scoring").changed();
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

pub(super) fn piece_diameter(p: &library::Piece) -> f64 {
    use library::{PieceShape, IN};
    match p.shape { PieceShape::Sphere { d_in } => d_in * IN, PieceShape::Tube { outer_d_in, .. } => outer_d_in * IN, PieceShape::Cylinder { d_in, .. } => d_in * IN, PieceShape::Box { z_in, .. } => z_in * IN }
}
