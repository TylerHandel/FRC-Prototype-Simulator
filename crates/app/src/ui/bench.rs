//! The Bench (M1): the robot held still, pieces sent at it as if it drove onto them. Swap wheels,
//! compare up to four side by side with the same throw (D19), sweep a number (M6).
use bevy::prelude::*;
use bevy_egui::egui::{self, Color32, RichText, Ui};
use egui_plot::{Legend, Line, Plot};
use library::units::Kind;
use model::DriveTarget;

use super::widgets::*;
use crate::simrun::{Msg, Sim};
use crate::state::AppState;

const COLORS: [Color32; 4] = [Color32::from_rgb(240, 160, 50), Color32::from_rgb(80, 170, 240), Color32::from_rgb(120, 210, 110), Color32::from_rgb(220, 100, 200)];

pub fn left(ui: &mut Ui, st: &mut AppState, sim: &Sim) {
    heading(ui, "What comes at it");
    let mut b = st.project().bench.clone();
    let mut ch = false;
    row(ui, "Piece", "", |ui| ch |= piece_pick(ui, "b_piece", &st.lib, &mut b.piece));
    let len = st.unit(Kind::Length);
    let spd = st.unit(Kind::Speed);
    ch |= num_row(ui, "Driving onto it at", "How fast the robot drives onto the piece", &mut b.speed, Kind::Speed, spd);
    ch |= num_row(ui, "Starts this far out", "From the front of the intake", &mut b.distance, Kind::Length, len);
    let ang = st.unit(Kind::Angle);
    let mut yaw = b.yaw.to_radians();
    let mut yaw_s = b.yaw_spread.to_radians();
    if num_row(ui, "Turned by", "How the piece lies across the robot's path: 0 is square to it", &mut yaw, Kind::Angle, ang) { b.yaw = yaw.to_degrees(); ch = true; }
    if num_row(ui, "  give or take", "Random spread, a different one each throw", &mut yaw_s, Kind::Angle, ang) { b.yaw_spread = yaw_s.to_degrees(); ch = true; }
    ch |= num_row(ui, "Off centre by", "Sideways", &mut b.offset, Kind::Length, len);
    ch |= num_row(ui, "  give or take", "", &mut b.offset_spread, Kind::Length, len);
    ch |= num_row(ui, "Each throw lasts", "", &mut b.duration, Kind::Time, st.unit(Kind::Time));

    heading(ui, "What runs");
    let r = st.project().robot.clone();
    let mut run = sim::bench::default_run(st.project());
    let mut run_ch = false;
    for t in &mut run {
        let Some(d) = r.drive(t.drive) else { continue };
        if matches!(d.target, DriveTarget::Shaft { .. }) {
            run_ch |= num_row(ui, &d.name, "Volts while the piece comes in", &mut t.value, Kind::Voltage, st.unit(Kind::Voltage));
            let mut lim = d.stator_limit;
            if num_row(ui, "  stator limit", "", &mut lim, Kind::Current, st.unit(Kind::Current)) {
                let id = d.id;
                st.doc.edit("~Stator limit", |p| if let Some(x) = p.robot.drive_mut(id) { x.stator_limit = lim; });
            }
        }
    }
    if run_ch { b.run = run; ch = true; }
    if ch { st.doc.edit("~Bench", |p| p.bench = b.clone()); }

    heading(ui, "Wheels on the rollers");
    hint(ui, "Swap what's on each roller shaft. The 3D view shows it running.");
    let shafts: Vec<(model::Id, String, String, f64)> = r.shafts.iter().filter_map(|s| {
        let first = r.rollers.iter().find(|x| x.shaft == s.id)?;
        Some((s.id, s.name.clone(), first.wheel.clone(), st.lib.wheel(&first.wheel).map_or(4.0, |w| w.diameter_in)))
    }).collect();
    if shafts.is_empty() { hint(ui, "This robot has no rollers. Add an intake or a claw in Build."); }
    for (sid, name, wheel, d) in shafts {
        let mut w = wheel.clone();
        row(ui, &name, "", |ui| { if wheel_pick(ui, &format!("bw{sid}"), &st.lib, &mut w, d) { st.doc.edit("Swap wheels", |p| for x in &mut p.robot.rollers { if x.shaft == sid { x.wheel = w.clone(); } }); } });
    }

    heading(ui, "Compare wheels");
    hint(ui, "The same throw with each wheel on every roller, side by side.");
    let mut cmp = st.project().bench.compare.clone();
    if cmp.is_empty() {
        if let Some(first) = r.rollers.first() { cmp.push(first.wheel.clone()); }
    }
    let mut cmp_ch = false;
    let mut remove = None;
    for (i, w) in cmp.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            ui.colored_label(COLORS[i % 4], "■");
            cmp_ch |= wheel_pick(ui, &format!("cmp{i}"), &st.lib, w, 4.0);
            if ui.small_button("x").clicked() { remove = Some(i); }
        });
    }
    if let Some(i) = remove { cmp.remove(i); cmp_ch = true; }
    if cmp.len() < 4 && ui.button("+ Add a wheel").clicked() {
        let next = st.lib.wheels.iter().find(|w| !cmp.contains(&w.id)).map(|w| w.id.clone()).unwrap_or_default();
        cmp.push(next); cmp_ch = true;
    }
    if cmp_ch { st.doc.edit("Compare", |p| p.bench.compare = cmp.clone()); }
    let busy = st.bench.running.is_some();
    if ui.add_enabled(!busy && !cmp.is_empty(), egui::Button::new(RichText::new(if busy { "Running..." } else { "Run the comparison" }).strong())).clicked() {
        let (tx, rx) = std::sync::mpsc::channel();
        let (p, lib) = (st.project().clone(), st.lib.clone());
        std::thread::spawn(move || {
            let out = sim::bench::compare(&p, &lib, &cmp).into_iter().zip(cmp.iter())
                .map(|(r, w)| (lib.wheel(w).map_or(w.clone(), |x| x.name.clone()), r.map_err(|e| e.to_string()))).collect();
            let _ = tx.send(out);
        });
        st.bench.running = Some(std::sync::Mutex::new(rx));
    }

    heading(ui, "Sweep");
    hint(ui, "Many throws over a range of one number, in parallel. Uses the Bench's 'give or take' for each throw.");
    let axes = ["Driving speed (m/s)", "Roller volts", "Stator limit (A)", "Piece turned (deg)", "Roller height (in)"];
    egui::ComboBox::from_id_salt("sweep_axis").selected_text(axes[st.bench.sweep_axis.min(axes.len() - 1)]).show_ui(ui, |ui| {
        for (i, a) in axes.iter().enumerate() { ui.selectable_value(&mut st.bench.sweep_axis, i, *a); }
    });
    if st.bench.sweep_values.is_empty() { st.bench.sweep_values = ["0.5, 1, 1.5, 2", "6, 8, 10, 12", "20, 30, 40, 60", "0, 15, 30, 45", "5.8, 6.0, 6.2, 6.4"][st.bench.sweep_axis].into(); }
    ui.horizontal(|ui| { ui.label("Values"); ui.text_edit_singleline(&mut st.bench.sweep_values); });
    ui.horizontal(|ui| { ui.label("Throws each"); ui.add(egui::DragValue::new(&mut st.bench.throws).range(1..=50)); });
    let busy = st.bench.sweep_running.is_some();
    if ui.add_enabled(!busy, egui::Button::new(if busy { "Sweeping..." } else { "Run the sweep" })).clicked() {
        let vals: Vec<f64> = st.bench.sweep_values.split([',', ' ']).filter_map(|s| s.trim().parse().ok()).collect();
        let intake = st.project().robot.mechanisms.iter().find(|m| matches!(m.template.as_ref().map(|t| &t.params), Some(model::templates::Params::Intake { .. }))).map(|m| m.id);
        let axis = match st.bench.sweep_axis {
            0 => sim::bench::Axis::Speed(vals),
            1 => sim::bench::Axis::Volts(vals),
            2 => sim::bench::Axis::StatorLimit(vals),
            3 => sim::bench::Axis::Yaw(vals),
            _ => sim::bench::Axis::Template { mechanism: intake.unwrap_or(0), field: "height".into(), values: vals.iter().map(|v| v * library::IN).collect() },
        };
        let (tx, rx) = std::sync::mpsc::channel();
        let (p, lib, throws) = (st.project().clone(), st.lib.clone(), st.bench.throws);
        std::thread::spawn(move || { let _ = tx.send(sim::bench::sweep(&p, &lib, &[axis], throws)); });
        st.bench.sweep_running = Some(std::sync::Mutex::new(rx));
    }
    ui.add_space(6.0);
    if ui.button("Throw again in the 3D view").clicked() { sim.send(Msg::Reset); }
}

pub fn poll(mut st: ResMut<AppState>) {
    let got = st.bench.running.as_ref().and_then(|rx| rx.lock().ok().and_then(|r| r.try_recv().ok()));
    if let Some(r) = got { st.bench.results = r; st.bench.running = None; }
    let got = st.bench.sweep_running.as_ref().and_then(|rx| rx.lock().ok().and_then(|r| r.try_recv().ok()));
    if let Some(r) = got { st.bench.sweep = r; st.bench.sweep_running = None; }
}

pub fn right(ui: &mut Ui, st: &mut AppState, sim: &Sim) {
    heading(ui, "Results");
    if st.bench.results.is_empty() { hint(ui, "Run a comparison (left) to see each wheel's numbers here."); }
    for (i, (label, r)) in st.bench.results.iter().enumerate() {
        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.colored_label(COLORS[i % 4], RichText::new(label).strong());
            match r {
                Err(e) => { ui.colored_label(BAD, e); }
                Ok(t) => {
                    let verdict = if t.acquired { RichText::new(format!("In, at {:.2} s", t.time_in.unwrap_or(0.0))).color(GOOD) } else if t.rejected { RichText::new("Pushed away").color(BAD) } else { RichText::new("Stuck").color(WARN) };
                    ui.label(verdict.strong());
                    egui::Grid::new(("res", i)).num_columns(2).show(ui, |ui| {
                        ui.label("Current while touching"); ui.label(format!("{:.1} A mean, {:.1} A peak", t.mean_current, t.peak_current)); ui.end_row();
                        ui.label("Squeeze"); ui.label(format!("{:.0} N mean, {:.0} N peak", t.mean_squeeze, t.peak_squeeze)); ui.end_row();
                        ui.label("Roller speed"); ui.label(format!("{:.0} rpm, dips to {:.0}", t.start_roller_rpm, t.min_roller_rpm)); ui.end_row();
                        ui.label("Slipping"); ui.label(format!("{:.0}% of the time", t.slip * 100.0)); ui.end_row();
                        ui.label("Battery"); ui.label(format!("{:.0} J, lowest {:.2} V", t.energy, t.min_bus)); ui.end_row();
                    });
                    for w in &t.warnings { ui.colored_label(WARN, w); }
                }
            }
        });
    }
    if !st.bench.results.is_empty() {
        hint(ui, "Squeeze and current rest on wheel stiffness estimated from published durometer: trust the ranking between wheels more than the absolute numbers until a press test replaces them.");
    }
    if !st.bench.sweep.is_empty() {
        heading(ui, "Sweep");
        egui::Grid::new("sweep").striped(true).show(ui, |ui| {
            ui.label("Value"); ui.label("In"); ui.label("Time"); ui.label("Current"); ui.label("Squeeze"); ui.end_row();
            for c in &st.bench.sweep {
                ui.label(c.labels.join(", "));
                let frac = c.acquired as f64 / c.trials.max(1) as f64;
                ui.colored_label(if frac > 0.8 { GOOD } else if frac > 0.4 { WARN } else { BAD }, format!("{}/{}", c.acquired, c.trials));
                ui.label(if c.mean_time.is_finite() { format!("{:.2} s", c.mean_time) } else { "-".into() });
                ui.label(format!("{:.1} A", c.mean_peak_current));
                ui.label(format!("{:.0} N", c.mean_peak_squeeze));
                ui.end_row();
            }
        });
    }
    let throws = sim.shared.lock().map(|s| s.bench_throws).unwrap_or(0);
    ui.add_space(8.0);
    hint(ui, &format!("The 3D view throws a new piece every {:.1} s ({} so far).", st.project().bench.duration + 0.4, throws));
}

pub fn bottom(ui: &mut Ui, st: &mut AppState, sim: &Sim) {
    if st.bench.results.is_empty() {
        heading(ui, "Live");
        super::analyze::live_plots(ui, st, sim);
        return;
    }
    ui.columns(3, |cols| {
        for (ci, (title, f)) in [("Current (A)", 0usize), ("Squeeze (N)", 1), ("Roller speed (rpm)", 2)].into_iter().enumerate() {
            let ui = &mut cols[ci];
            ui.label(RichText::new(title).strong());
            let mut plot = Plot::new(("bench_plot", ci)).height((ui.available_height() - 4.0).max(140.0)).allow_scroll(false);
            if ci == 0 { plot = plot.legend(Legend::default().position(egui_plot::Corner::RightTop)); }
            plot.show(ui, |p| {
                for (i, (label, r)) in st.bench.results.iter().enumerate() {
                    let Ok(t) = r else { continue };
                    let pts: Vec<[f64; 2]> = t.samples.iter().map(|s| [s.t, match f { 0 => s.current, 1 => s.squeeze, _ => s.roller_rpm }]).collect();
                    p.line(Line::new(label.clone(), pts).color(COLORS[i % 4]));
                }
            });
        }
    });
}
