//! Home: start a project from a starting point, or open one.
use bevy_egui::egui::{self, RichText, Ui};

use super::widgets::*;
use crate::state::AppState;

pub fn show(ui: &mut Ui, st: &mut AppState) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.add_space(16.0);
        ui.vertical_centered(|ui| {
            ui.heading(RichText::new("FRC Prototype Simulator").size(28.0).color(ACCENT));
            ui.label(RichText::new("Bring a robot as blocks or as CAD, and see whether it picks the piece up, how fast it cycles, what it draws from the battery, and whether it tips.").color(MUTED));
        });
        ui.add_space(16.0);
        ui.columns(2, |cols| {
            let ui = &mut cols[0];
            heading(ui, "Start a new robot");
            ui.horizontal(|ui| { ui.label("Name"); ui.text_edit_singleline(&mut st.new_name); });
            hint(ui, "Pick a starting point. Everything in it can be changed or removed.");
            for (title, about, p) in crate::projects::starters() {
                if card(ui, title, about) {
                    let mut p = p;
                    p.name = if st.new_name.trim().is_empty() { title.to_string() } else { st.new_name.trim().to_string() };
                    p.robot.name = p.name.clone();
                    st.start_new(p);
                }
            }
            ui.add_space(8.0);
            if card(ui, "From CAD", "Start empty and bring your robot in from an Onshape link or a STEP file (Rig CAD).") {
                let mut p = model::Project::new(if st.new_name.trim().is_empty() { "CAD robot" } else { st.new_name.trim() });
                p.robot.drivetrain = Some(model::Drivetrain::default());
                st.start_new(p);
                st.ws = crate::state::Workspace::Rig;
            }

            let ui = &mut cols[1];
            heading(ui, "Open");
            let recent = st.settings.recent.clone();
            if recent.is_empty() { hint(ui, "Projects you open show up here."); }
            for d in recent {
                let name = d.file_name().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
                let exists = d.join(crate::projects::FILE).exists();
                ui.add_enabled_ui(exists, |ui| {
                    if card(ui, &name, &d.display().to_string()) { st.open(d.clone()); }
                });
            }
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                #[cfg(any(windows, target_os = "macos"))]
                if ui.button("Open a project folder...").clicked() {
                    if let Some(d) = rfd::FileDialog::new().set_directory(&st.settings.projects_dir).pick_folder() { st.open(d); }
                }
                #[cfg(not(any(windows, target_os = "macos")))]
                {
                    let id = egui::Id::new("open_path");
                    let mut path: String = ui.data_mut(|d| d.get_temp(id)).unwrap_or_default();
                    ui.label("Folder");
                    ui.text_edit_singleline(&mut path);
                    if ui.button("Open").clicked() { st.open(path.clone().into()); }
                    ui.data_mut(|d| d.insert_temp(id, path));
                }
            });
            hint(ui, &format!("New projects are saved in {}", st.settings.projects_dir.display()));
            heading(ui, "Settings");
            ui.horizontal(|ui| {
                ui.label("Units");
                let mut imp = st.settings.imperial();
                if ui.radio_value(&mut imp, true, "inches and pounds").changed() || ui.radio_value(&mut imp, false, "millimetres and kilograms").changed() {
                    st.settings.units = if imp { "imperial".into() } else { "metric".into() };
                    st.settings.save();
                }
            });
            ui.checkbox(&mut st.settings.field_relative, "Drive field-relative on the Field (from the driver's point of view)");
            ui.add_space(8.0);
            hint(ui, &format!("Version {}", crate::update::current_version()));
        });
    });
}
