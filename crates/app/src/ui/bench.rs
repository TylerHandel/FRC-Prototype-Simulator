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

    sweep_controls(ui, st);
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
        ui.horizontal(|ui| {
            if ui.button("Save as CSV").on_hover_text("Into the project's runs folder").clicked() { save_sweep(st); }
            ui.checkbox(&mut st.bench.show_sweep, "Heat map below");
        });
        egui::ScrollArea::horizontal().id_salt("sweep_table").show(ui, |ui| {
            egui::Grid::new("sweep").striped(true).show(ui, |ui| {
                for a in &st.bench.sweep_axes { ui.label(RichText::new(a.name()).strong()); }
                for h in ["In", "Time", "Current", "Squeeze"] { ui.label(RichText::new(h).strong()); }
                ui.end_row();
                for c in &st.bench.sweep {
                    for l in &c.labels { ui.label(l); }
                    let frac = c.success();
                    ui.colored_label(if frac > 0.8 { GOOD } else if frac > 0.4 { WARN } else { BAD }, format!("{}/{}", c.acquired, c.trials));
                    ui.label(if c.mean_time.is_finite() { format!("{:.2} s", c.mean_time) } else { "-".into() });
                    ui.label(format!("{:.1} A", c.mean_current));
                    ui.label(format!("{:.0} N", c.mean_peak_squeeze));
                    ui.end_row();
                }
            });
        });
        hint(ui, "Current is the mean while touching; squeeze the peak. Each row averages its throws.");
    }
    let throws = sim.shared.lock().map(|s| s.bench_throws).unwrap_or(0);
    ui.add_space(8.0);
    hint(ui, &format!("The 3D view throws a new piece every {:.1} s ({} so far).", st.project().bench.duration + 0.4, throws));
}

pub fn bottom(ui: &mut Ui, st: &mut AppState, sim: &Sim) {
    if st.bench.show_sweep && !st.bench.sweep.is_empty() { heat_map(ui, st); return; }
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

/// What can be swept, with example values. "Wheel" sweeps the wheels picked under Compare.
pub const KINDS: [(&str, &str); 6] = [
    ("Driving speed (m/s)", "0.5, 1, 1.5, 2"),
    ("Roller volts", "6, 8, 10, 12"),
    ("Stator limit (A)", "20, 30, 40, 60"),
    ("Piece turned (deg)", "0, 15, 30, 45"),
    ("Roller height (in)", "5.8, 6.0, 6.2, 6.4"),
    ("Wheel (the ones under Compare)", ""),
];

const OUTCOMES: [&str; 5] = ["Taken in (share)", "Time in (s)", "Mean current (A)", "Peak squeeze (N)", "Energy (J)"];

fn outcome(c: &sim::bench::Cell, k: usize) -> f64 {
    match k { 0 => c.success(), 1 => c.mean_time, 2 => c.mean_current, 3 => c.mean_peak_squeeze, _ => c.mean_energy }
}

/// The sweep's setup: up to three things to vary, their values, and how many throws each.
fn sweep_controls(ui: &mut Ui, st: &mut AppState) {
    heading(ui, "Sweep");
    hint(ui, "Every combination of up to three things, many throws each, in parallel. Each throw uses the Bench's 'give or take'.");
    let mut remove = None;
    for (ri, (kind, values)) in st.bench.rows.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            let before = *kind;
            egui::ComboBox::from_id_salt(("sweep_kind", ri)).selected_text(KINDS[(*kind).min(KINDS.len() - 1)].0).width(190.0).show_ui(ui, |ui| {
                for (i, k) in KINDS.iter().enumerate() { ui.selectable_value(kind, i, k.0); }
            });
            if *kind != before { *values = KINDS[*kind].1.into(); }
            if ri > 0 && ui.small_button("x").on_hover_text("Stop varying this").clicked() { remove = Some(ri); }
        });
        if *kind != 5 { ui.horizontal(|ui| { ui.label("   Values"); ui.add(egui::TextEdit::singleline(values).desired_width(150.0)); }); }
    }
    if let Some(ri) = remove { st.bench.rows.remove(ri); }
    if st.bench.rows.len() < 3 && ui.small_button("+ vary something else too").clicked() {
        let k = (0..KINDS.len()).find(|k| !st.bench.rows.iter().any(|r| r.0 == *k)).unwrap_or(0);
        st.bench.rows.push((k, KINDS[k].1.into()));
    }
    ui.horizontal(|ui| { ui.label("Throws each"); ui.add(egui::DragValue::new(&mut st.bench.throws).range(1..=50)); });
    let combos: usize = st.bench.rows.iter().map(|(k, v)| if *k == 5 { st.project().bench.compare.len() } else { v.split([',', ' ']).filter(|x| x.trim().parse::<f64>().is_ok()).count() }).product();
    hint(ui, &format!("{} combinations, {} throws", combos, combos * st.bench.throws));
    let busy = st.bench.sweep_running.is_some();
    if ui.add_enabled(!busy && combos > 0, egui::Button::new(if busy { "Sweeping..." } else { "Run the sweep" })).clicked() {
        let intake = st.project().robot.mechanisms.iter().find(|m| matches!(m.template.as_ref().map(|t| &t.params), Some(model::templates::Params::Intake { .. }))).map(|m| m.id);
        let axes: Vec<sim::bench::Axis> = st.bench.rows.iter().map(|(k, v)| {
            let vals: Vec<f64> = v.split([',', ' ']).filter_map(|s| s.trim().parse().ok()).collect();
            match k {
                0 => sim::bench::Axis::Speed(vals),
                1 => sim::bench::Axis::Volts(vals),
                2 => sim::bench::Axis::StatorLimit(vals),
                3 => sim::bench::Axis::Yaw(vals),
                4 => sim::bench::Axis::Template { mechanism: intake.unwrap_or(0), field: "height".into(), values: vals.iter().map(|v| v * library::IN).collect() },
                _ => sim::bench::Axis::Wheel(st.project().bench.compare.clone()),
            }
        }).collect();
        st.bench.sweep_axes = axes.clone();
        st.bench.show_sweep = axes.len() >= 2;
        let (tx, rx) = std::sync::mpsc::channel();
        let (p, lib, throws) = (st.project().clone(), st.lib.clone(), st.bench.throws);
        std::thread::spawn(move || { let _ = tx.send(sim::bench::sweep(&p, &lib, &axes, throws)); });
        st.bench.sweep_running = Some(std::sync::Mutex::new(rx));
    }
}

fn save_sweep(st: &mut AppState) {
    let csv = sim::bench::sweep_csv(&st.bench.sweep_axes, &st.bench.sweep);
    let dir = st.dir.clone().unwrap_or_else(|| st.settings.projects_dir.clone()).join("runs");
    let _ = std::fs::create_dir_all(&dir);
    let name = format!("sweep-{}.csv", std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0));
    match std::fs::write(dir.join(&name), csv) {
        Ok(()) => st.toast(format!("Saved {}", dir.join(name).display())),
        Err(e) => st.toast(format!("Could not save: {e}")),
    }
}

/// Two of the sweep's things against each other, an outcome as colour (averaged over the third);
/// one thing alone is a row of cells.
fn heat_map(ui: &mut Ui, st: &mut AppState) {
    let axes = &st.bench.sweep_axes;
    ui.horizontal(|ui| {
        ui.label(RichText::new("Sweep").strong());
        egui::ComboBox::from_id_salt("outcome").selected_text(OUTCOMES[st.bench.outcome]).show_ui(ui, |ui| {
            for (i, o) in OUTCOMES.iter().enumerate() { ui.selectable_value(&mut st.bench.outcome, i, *o); }
        });
        if axes.len() > 2 { hint(ui, &format!("averaged over {}", axes[2].name())); }
        if ui.small_button("Back to the plots").clicked() { st.bench.show_sweep = false; }
    });
    let (nx, ny) = (axes.first().map_or(1, |a| a.len()), axes.get(1).map_or(1, |a| a.len()));
    let k = st.bench.outcome;
    // each (x, y) cell's mean over the third axis
    let mut grid = vec![vec![(0.0, 0usize); ny]; nx];
    for c in &st.bench.sweep {
        let v = outcome(c, k);
        if !v.is_finite() { continue; }
        let (x, y) = (c.index[0], c.index.get(1).copied().unwrap_or(0));
        grid[x][y].0 += v; grid[x][y].1 += 1;
    }
    let vals: Vec<f64> = grid.iter().flatten().filter(|c| c.1 > 0).map(|c| c.0 / c.1 as f64).collect();
    let (lo, hi) = (vals.iter().cloned().fold(f64::INFINITY, f64::min), vals.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
    let avail = ui.available_size();
    let label_w = 120.0;
    let cw = ((avail.x - label_w) / nx as f32).clamp(30.0, 160.0);
    let ch = ((avail.y - 40.0) / ny as f32).clamp(18.0, 60.0);
    let (rect, _) = ui.allocate_exact_size(egui::vec2(label_w + cw * nx as f32, ch * ny as f32 + 22.0), egui::Sense::hover());
    let paint = ui.painter_at(rect);
    let font = egui::FontId::proportional(12.0);
    for x in 0..nx {
        let lx = rect.left() + label_w + cw * (x as f32 + 0.5);
        paint.text(egui::pos2(lx, rect.bottom() - 10.0), egui::Align2::CENTER_CENTER, axes[0].label(x), font.clone(), MUTED);
        for y in 0..ny {
            let r = egui::Rect::from_min_size(egui::pos2(rect.left() + label_w + cw * x as f32, rect.top() + ch * (ny - 1 - y) as f32), egui::vec2(cw - 2.0, ch - 2.0));
            let (sum, n) = grid[x][y];
            if n == 0 { paint.rect_filled(r, 2.0, Color32::from_gray(40)); continue; }
            let v = sum / n as f64;
            let t = if hi > lo { ((v - lo) / (hi - lo)) as f32 } else { 0.5 };
            // dark blue (low) to orange (high)
            let col = Color32::from_rgb((40.0 + 200.0 * t) as u8, (60.0 + 100.0 * t) as u8, (140.0 - 100.0 * t) as u8);
            paint.rect_filled(r, 2.0, col);
            let text = match k { 0 => format!("{:.0}%", v * 100.0), 1 => format!("{v:.2}"), 3 | 4 => format!("{v:.0}"), _ => format!("{v:.1}") };
            paint.text(r.center(), egui::Align2::CENTER_CENTER, text, font.clone(), Color32::WHITE);
        }
    }
    if let Some(a) = axes.get(1) {
        for y in 0..ny {
            paint.text(egui::pos2(rect.left() + label_w - 6.0, rect.top() + ch * (ny as f32 - 0.5 - y as f32)), egui::Align2::RIGHT_CENTER, a.label(y), font.clone(), MUTED);
        }
    }
}
