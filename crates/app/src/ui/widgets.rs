//! Small widgets every panel uses: number fields that take units and arithmetic (D18), pickers for
//! motors, wheels and pieces, and the look of a section.
use bevy_egui::egui::{self, Color32, RichText, Ui};
use library::units::{self, Kind, Unit};
use library::Library;

pub const ACCENT: Color32 = Color32::from_rgb(240, 160, 50);
pub const MUTED: Color32 = Color32::from_rgb(150, 156, 166);
pub const GOOD: Color32 = Color32::from_rgb(90, 200, 120);
pub const BAD: Color32 = Color32::from_rgb(230, 90, 80);
pub const WARN: Color32 = Color32::from_rgb(235, 190, 70);

/// A number field in display units. Typing `1.75in`, `30/11` or `2ft + 3in` works; the value is
/// SI. Returns true once the value is committed (Enter, or leaving the field) and changed.
pub fn num(ui: &mut Ui, id: impl std::hash::Hash + std::fmt::Debug, v: &mut f64, kind: Kind, unit: &Unit) -> bool {
    let id = ui.make_persistent_id(id);
    let mut text: String = ui.data_mut(|d| d.get_temp::<String>(id)).unwrap_or_else(|| units::format(*v, unit));
    let resp = ui.add(egui::TextEdit::singleline(&mut text).desired_width(96.0).id(id.with("edit")));
    let mut changed = false;
    if resp.has_focus() {
        ui.data_mut(|d| d.insert_temp(id, text.clone()));
    }
    let commit = resp.lost_focus();
    if commit {
        match units::parse(&text, kind, unit) {
            Ok(x) if (x - *v).abs() > 1e-12 * v.abs().max(1.0) => { *v = x; changed = true; }
            Ok(_) => {}
            Err(e) => { ui.ctx().data_mut(|d| d.insert_temp(egui::Id::new("num_error"), format!("{e}"))); }
        }
        ui.data_mut(|d| d.remove::<String>(id));
    } else if !resp.has_focus() {
        ui.data_mut(|d| d.remove::<String>(id));
    }
    resp.on_hover_text(format!("A {}. Type a number in {}, or with its unit (for example 1.75in, 30mm, 2ft + 3in, 30/11).", units::kind_name(kind), unit.label));
    changed
}

/// A labelled row: label on the left, widget on the right.
pub fn row<R>(ui: &mut Ui, label: &str, hint: &str, add: impl FnOnce(&mut Ui) -> R) -> R {
    ui.horizontal(|ui| {
        let l = ui.add_sized([150.0, 20.0], egui::Label::new(label).truncate());
        if !hint.is_empty() { l.on_hover_text(hint); }
        add(ui)
    }).inner
}

/// A number row: returns true when changed.
pub fn num_row(ui: &mut Ui, label: &str, hint: &str, v: &mut f64, kind: Kind, unit: &Unit) -> bool {
    // the field's id from its place in the panel, so two rows with the same label don't clash
    row(ui, label, hint, |ui| { let id = ui.next_auto_id(); num(ui, (id, label), v, kind, unit) })
}

pub fn heading(ui: &mut Ui, s: &str) {
    ui.add_space(6.0);
    ui.label(RichText::new(s).strong().size(15.0).color(ACCENT));
    ui.add_space(2.0);
}

pub fn hint(ui: &mut Ui, s: &str) { ui.label(RichText::new(s).color(MUTED).size(12.0)); }

pub fn motor_pick(ui: &mut Ui, id: &str, lib: &Library, v: &mut String) -> bool {
    let name = lib.motor(v).map_or(v.clone(), |m| m.name.clone());
    let mut changed = false;
    egui::ComboBox::from_id_salt(id).selected_text(name).width(180.0).show_ui(ui, |ui| {
        for m in &lib.motors {
            if ui.selectable_label(*v == m.id, &m.name).on_hover_text(format!("{} rpm free, {:.2} N*m and {:.0} A at stall. {}", m.free_rpm, m.stall_torque, m.stall_current, m.source)).clicked() {
                *v = m.id.clone(); changed = true;
            }
        }
    });
    changed
}

/// A wheel picker, nearest sizes first, each with its durometer and whether its stiffness is measured.
pub fn wheel_pick(ui: &mut Ui, id: &str, lib: &Library, v: &mut String, near_in: f64) -> bool {
    let name = lib.wheel(v).map_or(v.clone(), |w| w.name.clone());
    let mut changed = false;
    egui::ComboBox::from_id_salt(id).selected_text(name).width(260.0).height(400.0).show_ui(ui, |ui| {
        for w in lib.wheels_like(near_in) {
            let tag = if w.measured && w.k_area.is_some() { "measured" } else { "estimated" };
            let label = format!("{}  ({:.0}A, {})", w.name, w.durometer_a, tag);
            if ui.selectable_label(*v == w.id, label).on_hover_text(&w.source).clicked() { *v = w.id.clone(); changed = true; }
        }
    });
    changed
}

pub fn piece_pick(ui: &mut Ui, id: &str, lib: &Library, v: &mut String) -> bool {
    let name = lib.piece(v).map_or(v.clone(), |p| format!("{} ({})", p.name, p.game));
    let mut changed = false;
    egui::ComboBox::from_id_salt(id).selected_text(name).width(200.0).show_ui(ui, |ui| {
        for p in &lib.pieces {
            if ui.selectable_label(*v == p.id, format!("{} ({})", p.name, p.game)).on_hover_text(&p.source).clicked() { *v = p.id.clone(); changed = true; }
        }
    });
    changed
}

/// The app's look: dark, with an orange accent, roomier spacing than egui's default.
pub fn theme(ctx: &egui::Context) {
    let mut v = egui::Visuals::dark();
    v.selection.bg_fill = Color32::from_rgb(170, 105, 25);
    v.hyperlink_color = ACCENT;
    v.widgets.hovered.bg_stroke = egui::Stroke::new(1.0, ACCENT);
    v.panel_fill = Color32::from_rgb(28, 30, 35);
    v.window_fill = Color32::from_rgb(32, 34, 40);
    v.extreme_bg_color = Color32::from_rgb(20, 21, 25);
    ctx.set_visuals(v);
    ctx.global_style_mut(|s| {
        s.spacing.item_spacing = egui::vec2(8.0, 5.0);
        s.spacing.button_padding = egui::vec2(8.0, 4.0);
        s.text_styles.insert(egui::TextStyle::Body, egui::FontId::proportional(14.0));
        s.text_styles.insert(egui::TextStyle::Button, egui::FontId::proportional(14.0));
        s.text_styles.insert(egui::TextStyle::Heading, egui::FontId::proportional(20.0));
    });
}

/// A big friendly button with a line of explanation under it.
pub fn card(ui: &mut Ui, title: &str, about: &str) -> bool {
    let r = egui::Frame::group(ui.style()).inner_margin(8.0).show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.label(RichText::new(title).strong().size(15.0));
        ui.label(RichText::new(about).color(MUTED).size(12.5));
    }).response.interact(egui::Sense::click());
    if r.hovered() { ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand); }
    r.clicked()
}

/// "#rrggbb" to an egui colour
pub fn color32(hex: &str) -> Color32 {
    let h = hex.trim_start_matches('#');
    let c = |i: usize| u8::from_str_radix(h.get(i..i + 2).unwrap_or("80"), 16).unwrap_or(128);
    Color32::from_rgb(c(0), c(2), c(4))
}
