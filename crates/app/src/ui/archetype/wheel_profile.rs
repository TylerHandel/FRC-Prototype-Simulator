use bevy_egui::egui::{self, Color32, Pos2, Stroke, Ui, Vec2};
use library::profile::{RimCut, Section, WheelEdits, SECTORS};

pub(super) fn show(ui: &mut Ui, lib: &library::Library, wheel: &library::Wheel, edits: &mut WheelEdits) {
    ui.collapsing("Wheel profile and grip", |ui| {
        let section = Section::new(wheel, edits);
        let size = ui.available_width().min(180.);
        let (rect, response) = ui.allocate_exact_size(Vec2::splat(size), egui::Sense::click());
        let center = rect.center();
        let radius = size * 0.46;
        let vertex = |i: usize, r: f64| -> Pos2 {
            let a = std::f32::consts::TAU * i as f32 / SECTORS as f32;
            center + Vec2::new(a.cos(), -a.sin()) * (radius * r as f32)
        };
        for (i, bands) in section.bands.iter().enumerate() {
            for &[lo, hi] in bands {
                let face = vec![vertex(i,lo), vertex(i,hi), vertex(i+1,hi), vertex(i+1,lo)];
                ui.painter().add(egui::Shape::convex_polygon(face, Color32::from_rgb(74,165,133), Stroke::NONE));
            }
        }
        if let Some(sku) = section.cad { ui.small(format!("AndyMark {sku} · CAD center section")); }
        else { ui.small("Approximate profile; internal pockets unavailable."); }
        ui.label("Click the rim to cut a notch through the wheel width.");
        let click = response.clicked().then(|| response.interact_pointer_pos()).flatten()
            .filter(|p| (radius*0.45..=radius*1.05).contains(&p.distance(center)));
        if ui.button("Add rim notch").clicked() || click.is_some() {
            let angle_deg = click.map_or(0., |p| f64::from((center.y-p.y).atan2(p.x-center.x).to_degrees()).rem_euclid(360.));
            edits.cuts.push(RimCut { depth: wheel.radius()*0.25, width: wheel.radius()*0.4, angle_deg, repeat: 1 });
        }
        let mut remove = None;
        for (i, cut) in edits.cuts.iter_mut().enumerate() {
            ui.push_id(i, |ui| {
                ui.horizontal(|ui| {
                    ui.strong(format!("Notch {}", i+1));
                    if ui.small_button("Remove").clicked() { remove = Some(i); }
                });
                for (label, value, max) in [("Depth", &mut cut.depth, wheel.radius()*0.75), ("Width at rim", &mut cut.width, wheel.radius()*2.)] {
                    let mut inches = *value / library::IN;
                    ui.horizontal(|ui| {
                        ui.label(label);
                        if ui.add(egui::DragValue::new(&mut inches).range(0.01..=(max/library::IN)).speed(0.01).suffix(" in")).changed() {
                            *value = inches * library::IN;
                        }
                    });
                }
                ui.horizontal(|ui| {
                    ui.label("Angle");
                    ui.add(egui::DragValue::new(&mut cut.angle_deg).range(0.0..=360.).suffix("°"));
                    ui.label("Repeat");
                    ui.add(egui::DragValue::new(&mut cut.repeat).range(1..=32));
                });
            });
        }
        if let Some(i) = remove { edits.cuts.remove(i); }
        let reference = wheel.radius()*0.1;
        let estimate = library::stiffness::plate_force(section.stiffness(wheel, edits), wheel.radius(), wheel.width(), reference);
        ui.separator();
        ui.label(format!("Single wheel at {:.2} mm compression: {:.1} N", reference*1000., estimate));
        let mut calibrate = edits.press_force.is_some();
        if ui.checkbox(&mut calibrate, "Use a measured press force").changed() { edits.press_force = calibrate.then_some(estimate); }
        if let Some(force) = &mut edits.press_force {
            ui.add(egui::DragValue::new(force).range(0.1..=2000.).speed(0.5).suffix(" N"));
        }
        let (s,d) = lib.friction(&wheel.tread,"pvc");
        let mut custom = edits.pvc_friction.is_some();
        if ui.checkbox(&mut custom,"Tune CORAL friction").changed() { edits.pvc_friction = custom.then_some([s,d]); }
        if let Some(mu) = &mut edits.pvc_friction {
            ui.horizontal(|ui| {
                ui.label("Static");
                ui.add(egui::DragValue::new(&mut mu[0]).range(0.0..=4.).speed(0.02));
                ui.label("Sliding");
                let limit = mu[0];
                mu[1] = mu[1].min(limit);
                ui.add(egui::DragValue::new(&mut mu[1]).range(0.0..=limit).speed(0.02));
            });
        } else { ui.small(format!("Estimated CORAL friction · static {s:.2}, sliding {d:.2}")); }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::{Event, FullOutput, PointerButton, RawInput};
    fn locate(out: &FullOutput, text: &str) -> Pos2 {
        out.shapes.iter().find_map(|s| match &s.shape {
            egui::Shape::Text(t) if t.galley.text() == text => Some(t.visual_bounding_rect().center()),
            _ => None,
        }).unwrap_or_else(|| panic!("Missing {text}"))
    }

    #[test]
    fn add_and_remove_profile_cut() {
        let ctx = egui::Context::default();
        ctx.global_style_mut(|s| s.animation_time = 0.);
        let lib = library::Library::builtin();
        let wheel = lib.wheel("am-compliant-4in-35a").unwrap();
        let mut edits = WheelEdits::default();
        let frame = |edits: &mut WheelEdits, events| ctx.run_ui(RawInput {
            screen_rect: Some(egui::Rect::from_min_size(Pos2::ZERO, Vec2::new(600., 800.))),
            events, ..Default::default()
        }, |ui| { egui::CentralPanel::default().show(ui, |ui| show(ui, &lib, wheel, edits)); });
        let click = |edits: &mut WheelEdits, pos| {
            for pressed in [true,false] {
                frame(edits, vec![Event::PointerMoved(pos), Event::PointerButton { pos, pressed,
                    button: PointerButton::Primary, modifiers: egui::Modifiers::NONE }]);
            }
            frame(edits, vec![])
        };
        let closed = frame(&mut edits, vec![]);
        let open = click(&mut edits, locate(&closed,"Wheel profile and grip"));
        let added = click(&mut edits, locate(&open,"Add rim notch"));
        assert_eq!(edits.cuts.len(),1);
        let saved: WheelEdits = serde_json::from_str(&serde_json::to_string(&edits).unwrap()).unwrap();
        assert_eq!(saved, edits);
        click(&mut edits, locate(&added,"Remove"));
        assert!(edits.cuts.is_empty());
    }
}
