//! Analyze: the run's recording as plots, the power budget, and CSV export (D16).
use bevy_egui::egui::{self, RichText, Ui};
use egui_plot::{Legend, Line, Plot};

use super::widgets::*;
use crate::simrun::Sim;
use crate::state::AppState;

/// every signal a run records, by name
fn signals(sim: &Sim) -> Vec<String> {
    let names = sim.shared.lock().map(|s| s.drive_names.clone()).unwrap_or_default();
    let mut v: Vec<String> = ["Battery voltage", "Battery current", "Energy used", "Speed", "Tip margin", "Score"].map(String::from).to_vec();
    for (_, n) in names { for k in ["current", "position", "speed", "volts"] { v.push(format!("{n} {k}")); } }
    v
}

fn value(s: &sim::Snapshot, name: &str, names: &[(model::Id, String)]) -> Option<f64> {
    Some(match name {
        "Battery voltage" => s.bus_voltage,
        "Battery current" => s.bus_current,
        "Energy used" => s.energy,
        "Speed" => s.speed,
        "Tip margin" => s.tip_margin,
        "Score" => s.score,
        _ => {
            let (id, rest) = names.iter().find_map(|(id, n)| name.strip_prefix(n.as_str()).map(|r| (*id, r.trim())))?;
            let d = s.drives.iter().find(|d| d.id == id)?;
            match rest { "current" => d.current, "position" => d.position, "speed" => d.speed, "volts" => d.volts, _ => return None }
        }
    })
}

pub fn left(ui: &mut Ui, st: &mut AppState, sim: &Sim) {
    heading(ui, "Signals");
    hint(ui, "Pick what to plot from the last run (up to 5 minutes is kept).");
    for s in signals(sim) {
        let mut on = st.plot_signals.contains(&s);
        if ui.checkbox(&mut on, &s).changed() {
            if on { st.plot_signals.push(s); } else { st.plot_signals.retain(|x| *x != s); }
        }
    }
}

pub fn right(ui: &mut Ui, st: &mut AppState, sim: &Sim) {
    heading(ui, "Power budget");
    let (hist, names) = sim.shared.lock().map(|s| (s.history.iter().cloned().collect::<Vec<_>>(), s.drive_names.clone())).unwrap_or_default();
    if hist.len() < 2 { hint(ui, "Run something first: drive, intake, score."); return; }
    let span = hist.last().unwrap().time - hist[0].time;
    egui::Grid::new("budget").striped(true).show(ui, |ui| {
        ui.label(RichText::new("Drive").strong()); ui.label(RichText::new("Peak A").strong()); ui.label(RichText::new("Mean A").strong()); ui.label(RichText::new("Battery Wh").strong()); ui.end_row();
        for (id, n) in &names {
            let cur: Vec<f64> = hist.iter().filter_map(|s| s.drives.iter().find(|d| d.id == *id).map(|d| d.current)).collect();
            let sup: f64 = hist.windows(2).filter_map(|w| w[1].drives.iter().find(|d| d.id == *id).map(|d| d.supply * w[1].bus_voltage * (w[1].time - w[0].time))).sum();
            if cur.is_empty() { continue; }
            ui.label(n);
            ui.label(format!("{:.0}", cur.iter().cloned().fold(0.0, f64::max)));
            ui.label(format!("{:.1}", cur.iter().sum::<f64>() / cur.len() as f64));
            ui.label(format!("{:.2}", sup / 3600.0));
            ui.end_row();
        }
    });
    let e = hist.last().unwrap().energy - hist[0].energy;
    let vmin = hist.iter().map(|s| s.bus_voltage).fold(f64::INFINITY, f64::min);
    let imax = hist.iter().map(|s| s.bus_current).fold(0.0, f64::max);
    ui.label(format!("Over {span:.1} s: {:.2} Wh from the battery, peak {imax:.0} A, lowest {vmin:.2} V", e / 3600.0));
    let match_wh = if span > 0.0 { e / span * 150.0 / 3600.0 } else { 0.0 };
    ui.label(format!("At this rate a 2:30 match takes about {match_wh:.1} Wh (a fresh FRC battery holds about 200 Wh, much less at high current)."));
    let brown = hist.iter().filter(|s| s.brownout).count();
    if brown > 0 { ui.colored_label(BAD, format!("Brownout for {:.2} s", brown as f64 * 0.01)); }
    ui.add_space(8.0);
    if ui.button("Export the run as CSV").clicked() { export_csv(st, sim); }
}

pub fn export_csv(st: &mut AppState, sim: &Sim) {
    let (hist, names) = sim.shared.lock().map(|s| (s.history.iter().cloned().collect::<Vec<_>>(), s.drive_names.clone())).unwrap_or_default();
    if hist.is_empty() { st.toast("Nothing recorded yet"); return; }
    let mut cols = vec!["time".to_string()];
    let sigs = signals(sim);
    cols.extend(sigs.iter().cloned());
    let mut out = cols.join(",") + "\n";
    for s in &hist {
        let mut row = vec![format!("{:.4}", s.time)];
        for name in &sigs { row.push(value(s, name, &names).map_or(String::new(), |v| format!("{v:.5}"))); }
        out += &(row.join(",") + "\n");
    }
    let dir = st.dir.clone().unwrap_or_else(|| st.settings.projects_dir.clone()).join("runs");
    let _ = std::fs::create_dir_all(&dir);
    let name = format!("run-{}.csv", std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0));
    match std::fs::write(dir.join(&name), out) {
        Ok(()) => st.toast(format!("Saved {}", dir.join(name).display())),
        Err(e) => st.toast(format!("Could not save: {e}")),
    }
}

pub fn bottom(ui: &mut Ui, st: &mut AppState, sim: &Sim) {
    let (hist, names) = sim.shared.lock().map(|s| (s.history.iter().cloned().collect::<Vec<_>>(), s.drive_names.clone())).unwrap_or_default();
    Plot::new("analyze").legend(Legend::default()).height(ui.available_height() - 4.0).show(ui, |p| {
        for name in &st.plot_signals {
            let pts: Vec<[f64; 2]> = hist.iter().filter_map(|s| value(s, name, &names).map(|v| [s.time, v])).collect();
            p.line(Line::new(name.clone(), pts));
        }
    });
}

/// Battery voltage and current over the last 10 s, small.
pub fn mini_plot(ui: &mut Ui, _st: &AppState, sim: &Sim, height: f32) {
    let hist: Vec<sim::Snapshot> = sim.shared.lock().map(|s| { let n = s.history.len(); s.history.iter().skip(n.saturating_sub(1000)).cloned().collect() }).unwrap_or_default();
    Plot::new("mini").height(height).legend(Legend::default()).allow_scroll(false).allow_drag(false).allow_zoom(false).show(ui, |p| {
        p.line(Line::new("Volts", hist.iter().map(|s| [s.time, s.bus_voltage]).collect::<Vec<_>>()));
        p.line(Line::new("Amps / 10", hist.iter().map(|s| [s.time, s.bus_current / 10.0]).collect::<Vec<_>>()));
    });
}

/// The rollers' current and the battery, live (the Bench before any comparison runs).
pub fn live_plots(ui: &mut Ui, _st: &AppState, sim: &Sim) {
    let (hist, names) = sim.shared.lock().map(|s| { let n = s.history.len(); (s.history.iter().skip(n.saturating_sub(400)).cloned().collect::<Vec<_>>(), s.drive_names.clone()) }).unwrap_or_default();
    Plot::new("live").height(ui.available_height() - 4.0).legend(Legend::default()).allow_scroll(false).show(ui, |p| {
        for (id, n) in &names {
            p.line(Line::new(format!("{n} A"), hist.iter().filter_map(|s| s.drives.iter().find(|d| d.id == *id).map(|d| [s.time, d.current])).collect::<Vec<_>>()));
        }
        p.line(Line::new("Battery V", hist.iter().map(|s| [s.time, s.bus_voltage]).collect::<Vec<_>>()));
    });
}
