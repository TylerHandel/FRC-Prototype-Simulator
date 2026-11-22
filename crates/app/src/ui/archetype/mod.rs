//! The early design workspace. It edits native templates and uses the simulator only for Drive.
//! Import, rigging, and bench tests have their own workspaces.
mod controls;
mod wheel_profile;
mod drawing;
mod size_checks;

use bevy_egui::egui::{self, RichText, Ui};
use library::units::Kind;
use model::templates::{self, Params, Template};
use model::{CHASSIS, Id, MechKind, Mechanism, Robot};

use super::widgets::*;
use crate::simrun::{Msg, Sim};
use crate::state::{AppState, Selection};

#[derive(Clone, Copy, Default, PartialEq, Eq)]
enum View {
    #[default]
    Side,
    Top,
}

#[derive(Clone, Copy, Default, PartialEq, Eq)]
enum Task {
    #[default]
    Layout,
    Controls,
    Drive,
}

#[derive(Clone, Copy, Default, PartialEq, Eq)]
enum Preview {
    #[default]
    Start,
    Stowed,
    Deployed,
}

impl Preview {
    fn label(self) -> &'static str {
        match self {
            Self::Start => "Starting position",
            Self::Stowed => "Stowed preview",
            Self::Deployed => "Deployed preview",
        }
    }
}

#[derive(Default)]
pub struct Assembly {
    task: Task,
    view: View,
    free_placement: bool,
    preset: Option<Id>,
    operator_action: Option<model::Action>,
    preview: Preview,
    season: String,
    show_limits: bool,
    outline_hidden: bool,
    bumper_gap: f64,
    fit_piece: String,
    tube: sim::world::TubeOrientation,
    open_carpet: bool,
    overview: bool,
}

impl Assembly {
    pub fn open_controls(&mut self) {
        self.task = Task::Controls;
    }
    pub fn editing_layout(&self) -> bool {
        self.task == Task::Layout
    }
    pub fn driving(&self) -> bool {
        self.task == Task::Drive
    }
    pub fn open_drive(&mut self) {
        self.task = Task::Drive;
    }
    pub fn field_view(&self) -> bool { self.driving() && !self.open_carpet }
    pub fn overview(&self) -> bool { self.field_view() && self.overview }
    pub fn show_field_overview(&mut self) { self.overview = true; }
    pub fn show_size_checks(&mut self) {
        self.show_limits = true;
    }
}

fn sync_season(st: &mut AppState) {
    if st.assembly.season == st.project().game {
        return;
    }
    let piece = st.game(false).ok().and_then(|g| {
        g.pack
            .pieces
            .first()
            .map(|p| &p.piece)
            .or_else(|| g.pack.feeders.first().map(|p| &p.piece))
            .or_else(|| g.pack.zones.first().map(|z| &z.piece))
            .cloned()
    });
    st.assembly.fit_piece = piece
        .filter(|id| st.lib.piece(id).is_some())
        .or_else(|| st.lib.pieces.first().map(|p| p.id.clone()))
        .unwrap_or_default();
    let robot = &st.project().robot;
    let vertical = robot.rollers.iter().any(|r| r.axis[2].abs() > 0.95 && robot.shaft(r.shaft)
        .and_then(|s| robot.mechanism_of_link(s.link)).is_some_and(|m| m.kind == MechKind::Claw));
    st.assembly.tube = if vertical { sim::world::TubeOrientation::Upright } else { sim::world::TubeOrientation::Crosswise };
    st.assembly.season = st.project().game.clone();
    st.sim_rev = u64::MAX;
}

#[cfg(test)]
mod season_tests {
    use super::*;

    #[test]
    fn changing_season_resets_the_practice_piece_and_undo_restores_the_game() {
        let mut st = AppState::new(Default::default());
        st.doc.project.game = "reefscape-2025".into();
        sync_season(&mut st);
        assert_eq!(st.assembly.fit_piece, "coral-2025");
        st.assembly.fit_piece = "algae-2025".into();
        sync_season(&mut st);
        assert_eq!(st.assembly.fit_piece, "algae-2025");
        st.doc.edit("Change game", |p| p.game = "rebuilt-2026".into());
        test_drive_setup(&mut st);
        assert_eq!(st.assembly.fit_piece, "fuel-2026");
        st.doc.undo();
        test_drive_setup(&mut st);
        assert_eq!(st.assembly.fit_piece, "coral-2025");
    }
}

pub(super) fn test_drive_setup(st: &mut AppState) -> crate::simrun::SceneSetup {
    sync_season(st);
    if st.lib.piece(&st.assembly.fit_piece).is_none() {
        st.assembly.fit_piece = st.lib.pieces.first().map(|p| p.id.clone()).unwrap_or_default();
    }
    let field = if st.assembly.open_carpet { None } else { st.game(false).ok().map(|g| Box::new(g.pack.clone())) };
    crate::simrun::SceneSetup::TestDrive { pieces: st.lib.pieces.clone(), selected: st.assembly.fit_piece.clone(), tube: st.assembly.tube, field }
}

pub fn task_bar(root: &mut Ui, st: &mut AppState, sim: &Sim) {
    sync_season(st);
    egui::Panel::top("archetype_task").show(root, |ui| {
        ui.horizontal_wrapped(|ui| {
            for (task, label) in
                [(Task::Layout, "Layout"), (Task::Controls, "Controls"), (Task::Drive, "Test drive")]
            {
                if ui.selectable_value(&mut st.assembly.task, task, label).changed() {
                    st.doc.seal();
                    if task == Task::Drive {
                        sim.send(Msg::Play(true));
                    }
                }
            }
            ui.separator();
            if st.games.is_empty() {
                st.list_games();
            }
            let mut id = st.project().game.clone();
            let label = st.games.iter().find(|g| g.0 == id).map_or(id.as_str(), |g| g.1.as_str());
            egui::ComboBox::from_id_salt("archetype_season")
                .selected_text(label)
                .show_ui(ui, |ui| {
                    for (game, name, _) in &st.games {
                        ui.selectable_value(&mut id, game.clone(), name);
                    }
                    if ui.small_button("Reload game packs").clicked() {
                        st.list_games();
                        let _ = st.game(true);
                        st.assembly.season.clear();
                    }
                })
                .response
                .on_hover_text("Game for field practice, size checks and game pieces");
            if id != st.project().game {
                st.doc.edit("Change game", |p| p.game = id);
            }
            sync_season(st);
        });
    });
}

fn palette_name(kind: MechKind) -> &'static str {
    match kind {
        MechKind::Claw => "Roller claw",
        MechKind::Intake => "Ground intake",
        _ => kind.label(),
    }
}

fn mount_name(m: &Mechanism) -> String {
    let site = match m.kind {
        MechKind::Elevator => "carriage",
        MechKind::Arm | MechKind::Wrist | MechKind::Telescope | MechKind::Climber => "tip",
        MechKind::Turret => "deck",
        _ => "body",
    };
    format!("{} {site}", m.name)
}

fn suggested(r: &Robot, selected: &Selection, kind: MechKind) -> Option<Id> {
    let suitable = |m: &Mechanism| match kind {
        MechKind::Claw | MechKind::Wrist | MechKind::Telescope => {
            matches!(m.kind, MechKind::Elevator | MechKind::Arm | MechKind::Wrist | MechKind::Telescope)
        }
        MechKind::Shooter => m.kind == MechKind::Turret,
        _ => false,
    };
    if let Selection::Mechanism(id) = selected
        && r.mechanism(*id).is_some_and(suitable)
    {
        return Some(*id);
    }
    let mut mounts = r.mechanisms.iter().filter(|m| suitable(m));
    let first = mounts.next()?;
    mounts.next().is_none().then_some(first.id)
}

pub(super) fn add_suggested(st: &mut AppState, kind: MechKind) {
    let parent = suggested(&st.project().robot, &st.sel, kind);
    add(st, kind, parent);
}

fn add(st: &mut AppState, kind: MechKind, parent: Option<Id>) {
    let Some(mut t) = Template::default_for(kind) else { return };
    set_mount(&mut t, parent);
    add_template(st, t);
}

fn add_template(st: &mut AppState, t: Template) {
    let label = palette_name(t.params.kind());
    let result = st.doc.edit(&format!("Add {label}"), |p| {
        let id = templates::add(&mut p.robot, t)?;
        // Multiple instances must be distinguishable in the mount picker.
        let mut name = label.to_string();
        let mut n = 2;
        while p.robot.mechanisms.iter().any(|m| m.id != id && m.name == name) {
            name = format!("{label} {n}");
            n += 1;
        }
        p.robot.mechanism_mut(id).unwrap().name = name;
        Ok::<_, anyhow::Error>(id)
    });
    match result {
        Ok(id) => st.sel = Selection::Mechanism(id),
        Err(e) => st.toast(e.to_string()),
    }
}

fn set_mount(t: &mut Template, parent: Option<Id>) {
    t.mount.on = parent;
    t.mount.offset = if parent.is_some() {
        [0.0; 3]
    } else {
        match t.params.kind() {
            MechKind::Claw | MechKind::Wrist | MechKind::Telescope => [0.1, 0.0, 0.4],
            k => Template::default_for(k).unwrap().mount.offset,
        }
    };
}

fn edit_template(st: &mut AppState, id: Id, t: Template, what: &str) {
    let mut robot = st.project().robot.clone();
    if let Err(e) = templates::rebuild(&mut robot, id, t) {
        st.toast(e.to_string());
        return;
    }
    let m = robot.mechanism(id).unwrap();
    for link in m.links.iter().filter_map(|id| robot.link(*id)) {
        if link.shapes.iter().any(|s| s.kind.extents().iter().any(|n| !n.is_finite() || *n <= 0.0)) {
            st.toast(format!(
                "These dimensions leave no room for {}. Increase its width, height, or length.",
                link.name
            ));
            return;
        }
        if let Some([lo, hi]) = robot.joint_of(link.id).and_then(|j| j.range)
            && lo >= hi
        {
            st.toast("Give the mechanism positive travel between its limits.");
            return;
        }
    }
    st.doc.edit(what, |p| p.robot = robot);
}

pub fn palette(ui: &mut Ui, st: &mut AppState) {
    if st.assembly.task == Task::Controls {
        controls::list(ui, st);
        return;
    }
    heading(ui, "On this robot");
    if ui.selectable_label(st.sel == Selection::Robot, "Swerve chassis").clicked() {
        st.sel = Selection::Robot;
    }
    assembly_children(ui, &st.doc.project.robot, &mut st.sel, None, 1);
    if st.assembly.driving() {
        return;
    }
    ui.separator();
    heading(ui, "Add a mechanism");
    hint(ui, "Click to add or drag into the drawing.");
    let tile = |ui: &mut Ui, kind: MechKind, st: &mut AppState| {
        let parent = suggested(&st.project().robot, &st.sel, kind);
        let target = parent.and_then(|id| st.project().robot.mechanism(id)).map_or("chassis".into(), mount_name);
        let response = ui.add_sized(
            [ui.available_width(), 38.0],
            egui::Button::new(palette_name(kind)).sense(egui::Sense::click_and_drag()),
        );
        if response.rect.width() > 145.0 {
            let icon = egui::Rect::from_center_size(
                response.rect.left_center() + egui::vec2(23.0, 0.0),
                egui::vec2(31.0, 27.0),
            );
            drawing::thumbnail(ui, icon, kind, &st.lib);
        }
        response.dnd_set_drag_payload(kind);
        if response.on_hover_text(format!("Click to mount on {target}")).clicked() {
            add(st, kind, parent);
        }
    };
    for k in [MechKind::Elevator, MechKind::Claw, MechKind::Intake, MechKind::Arm] {
        tile(ui, k, st);
    }
    ui.collapsing("More mechanisms", |ui| {
        for k in [MechKind::Wrist, MechKind::Telescope, MechKind::Shooter, MechKind::Turret, MechKind::Climber] {
            tile(ui, k, st);
        }
    });
}

fn assembly_children(ui: &mut Ui, r: &Robot, selected: &mut Selection, parent: Option<Id>, depth: usize) {
    for m in r.mechanisms.iter().filter(|m| m.template.as_ref().and_then(|t| t.mount.on) == parent) {
        ui.horizontal(|ui| {
            ui.add_space(12.0 * depth as f32);
            if ui.selectable_label(*selected == Selection::Mechanism(m.id), &m.name).clicked() {
                *selected = Selection::Mechanism(m.id);
            }
        });
        assembly_children(ui, r, selected, Some(m.id), depth + 1);
    }
}

pub fn inspector(ui: &mut Ui, st: &mut AppState, sim: &Sim) {
    match st.assembly.task {
        Task::Drive => {
            controls::drive_card(ui, st, sim);
            return;
        }
        Task::Controls => {
            controls::inspector(ui, st);
            return;
        }
        Task::Layout => {}
    }
    let m = match st.sel {
        Selection::Mechanism(id) => st.project().robot.mechanism(id).cloned(),
        _ => None,
    };
    let Some(m) = m else {
        chassis(ui, st);
        return;
    };
    let Some(mut t) = m.template.clone() else {
        ui.label(&m.name);
        return;
    };
    heading(ui, &m.name);
    heading(ui, "Placement");
    let r = &st.project().robot;
    let previous = t.mount.on;
    egui::ComboBox::from_id_salt("mount")
        .selected_text(previous.and_then(|id| r.mechanism(id)).map_or("Chassis".into(), mount_name))
        .show_ui(ui, |ui| {
            ui.selectable_value(&mut t.mount.on, None, "Chassis");
            for carrier in &r.mechanisms {
                if templates::can_mount(r, m.id, Some(carrier.id)) {
                    ui.selectable_value(&mut t.mount.on, Some(carrier.id), mount_name(carrier));
                }
            }
        });
    if t.mount.on != previous {
        let on = t.mount.on;
        set_mount(&mut t, on);
    }
    ui.collapsing("Offset from mount", |ui| {
        let len = st.unit(Kind::Length);
        for (axis, value) in ["Forward", "Left", "Up"].iter().zip(&mut t.mount.offset) {
            num_row(ui, axis, "", value, Kind::Length, len);
        }
    });
    let mut rotation_edit = None;
    let mut finish_rotation = false;
    ui.push_id(("mount_rotation", m.id), |ui| {
        ui.label("Rotation from mount");
        for (axis, label, help) in [
            (2, "Yaw", "Turn the mechanism left or right. Positive turns left."),
            (1, "Pitch", "Tilt the mechanism up or down. Positive tilts the front down."),
            (0, "Roll", "Rotate around the forward axis. Positive raises the left side."),
        ] {
            let response = row(ui, label, help, |ui| {
                ui.add(egui::DragValue::new(&mut t.mount.rotation_deg[axis]).speed(1.0).suffix("°"))
            });
            if response.drag_started() { st.doc.seal(); }
            if response.changed() { rotation_edit = Some(format!("~Rotate {} {label}", m.id)); }
            finish_rotation |= response.drag_stopped() || response.lost_focus();
        }
        if t.mount.rotation_deg != [0.0; 3] && ui.small_button("Reset rotation").clicked() {
            t.mount.rotation_deg = [0.0; 3];
            rotation_edit = Some("Reset rotation".into());
        }
    });
    let len = st.unit(Kind::Length);
    heading(ui, "Dimensions");
    if matches!(t.params, Params::Claw { .. }) && st.project().game == "reefscape-2025"
        && ui.button("Set up for upright CORAL").on_hover_text("Roll 90°, set a 3.75 in depth and 4.25 in gap, and leave the pipe ends open for the reef branch.").clicked() {
        t.upright_coral();
    }
    let mut dimension = |label, value: &mut f64| {
        num_row(ui, label, "", value, Kind::Length, len);
    };
    match &mut t.params {
        Params::Elevator { height, width, travel, .. } => {
            dimension("Fixed stage height", height);
            dimension("Carriage travel", travel);
            dimension("Width", width);
        }
        Params::Intake { height, width, wheel, .. } => {
            dimension("Roller height", height);
            dimension("Width", width);
            wheel_pick(ui, "assembly_intake_wheel", &st.lib, wheel, 4.0);
        }
        Params::Claw { depth, width, gap, wheel, .. } => {
            dimension("Depth", depth);
            dimension("Width", width);
            dimension("Gap between rollers", gap);
            wheel_pick(ui, "assembly_claw_wheel", &st.lib, wheel, 2.0);
        }
        Params::Arm { length, width, .. } | Params::Wrist { length, width, .. } => {
            dimension("Length", length);
            dimension("Width", width);
        }
        Params::Telescope { length, travel, .. } => {
            dimension("Length", length);
            dimension("Travel", travel);
        }
        Params::Turret { radius, .. } => dimension("Radius", radius),
        Params::Shooter { width, wheel, .. } => {
            dimension("Width", width);
            wheel_pick(ui, "assembly_shooter_wheel", &st.lib, wheel, 4.0);
        }
        Params::Climber { length, .. } => dimension("Length", length),
    }
    if let Params::Intake { wheel, .. } | Params::Claw { wheel, .. } | Params::Shooter { wheel, .. } = &t.params
        && let Some(spec) = st.lib.wheel(wheel) {
        wheel_profile::show(ui, &st.lib, spec, &mut t.wheel_edits);
    }
    let opening = match &t.params {
        Params::Claw { gap, .. } => Some(*gap),
        Params::Intake { height, wheel, .. } => st.lib.wheel(wheel).map(|w| height - w.radius()),
        _ => None,
    };
    if let Some(opening) = opening {
        heading(ui, "Piece fit");
        if st.lib.piece(&st.assembly.fit_piece).is_none()
            && let Some(piece) = st.lib.pieces.first()
        {
            st.assembly.fit_piece = piece.id.clone();
        }
        piece_pick(ui, "fit_piece", &st.lib, &mut st.assembly.fit_piece);
        if let Some(piece) = st.lib.piece(&st.assembly.fit_piece) {
            let squeeze = super::template_fields::piece_diameter(piece) - opening;
            let distance = library::units::format(squeeze.abs(), len);
            ui.label(if squeeze >= 0.0 {
                format!("{distance} of squeeze")
            } else {
                format!("{distance} of free space")
            })
            .on_hover_text("Nominal fit from the piece size and the unloaded gap between contact surfaces.");
            if let Params::Claw { depth, wheel, gap, .. } = &mut t.params
                && let Some(w) = st.lib.wheel(wheel)
                && piece.id == "coral-2025" && squeeze > 0. {
                let pipe = super::template_fields::piece_diameter(piece) / 2.;
                let wheel_r = w.radius();
                let cosine = ((*gap / 2. + wheel_r) / (pipe + wheel_r)).clamp(0.001, 1.);
                let min_mu = cosine.acos().tan();
                let margin = 0.75 * *depth - (0.010 + pipe);
                ui.small(format!("Side-on feed needs about {min_mu:.2} friction to draw CORAL between round wheels"));
                if margin < 0. {
                    ui.label("The back plate stops CORAL before its center passes the roller axles. Increase claw depth to seat it past the pinch.");
                    let quarter_in = 0.25 * library::IN;
                    let seat_depth = ((0.010 + pipe + quarter_in) / (0.75 * quarter_in)).ceil() * quarter_in;
                    if ui.button(format!("Set depth to {}", library::units::format(seat_depth, len))).clicked() { *depth = seat_depth; }
                } else {
                    ui.small(format!("Seated center is {} behind the roller axles.", library::units::format(margin, len)));
                }
            }
        }
    }
    ui.collapsing("All mechanism settings", |ui| {
        super::template_fields::template_fields(ui, st, &mut t);
    });
    if Some(&t) != m.template.as_ref() {
        edit_template(st, m.id, t, rotation_edit.as_deref().unwrap_or("Edit mechanism"));
    }
    if finish_rotation { st.doc.seal(); }
    ui.separator();
    if ui.small_button(format!("Remove {}", m.name)).clicked() {
        st.doc.edit("Remove mechanism", |p| p.robot.remove_mechanism(m.id));
        st.sel = Selection::Robot;
    }
}

fn chassis(ui: &mut Ui, st: &mut AppState) {
    heading(ui, "Swerve chassis");
    let mut frame = st.project().robot.frame.clone();
    let len = st.unit(Kind::Length);
    num_row(ui, "Frame length", "", &mut frame.length, Kind::Length, len);
    num_row(ui, "Frame width", "", &mut frame.width, Kind::Length, len);
    num_row(ui, "Bumper thickness", "", &mut frame.bumper_thickness, Kind::Length, len);
    if frame != st.project().robot.frame {
        resize_chassis(st, frame, "Resize chassis");
    }
    let r = &st.project().robot;
    let unit = st.unit(Kind::Mass);
    let mut mass = r.link(CHASSIS).map_or(r.frame.chassis_mass, |link| link.mass_or_estimate()) * r.mass_scale();
    if num_row(ui, "Chassis mass", "Include the drivetrain, battery, electronics, bumpers and ballast.", &mut mass, Kind::Mass, unit) {
        if mass.is_finite() && mass > 0.0 {
            st.doc.edit("Change chassis mass", |p| set_chassis_mass(&mut p.robot, mass));
        } else {
            st.toast("Enter a chassis mass greater than zero.");
        }
    }
    let r = &st.project().robot;
    let total = r.weighed_mass.unwrap_or_else(|| r.mass());
    ui.label(format!("Total with mechanisms   {}", library::units::format(total, unit)));
}

fn set_chassis_mass(r: &mut Robot, mass: f64) {
    let calibration = r.mass_scale();
    let Some(link) = r.link_mut(CHASSIS) else { return };
    link.mass = Some(mass / calibration);
    r.frame.chassis_mass = mass / calibration;
    if r.weighed_mass.is_some() {
        r.weighed_mass = Some(r.mass() * calibration);
    }
}

#[cfg(test)]
#[test]
fn chassis_mass_survives_reopen_and_reaches_the_test_drive_body() {
    let mut st = AppState::new(Default::default());
    add_suggested(&mut st, MechKind::Elevator);
    for calibration in [None, Some(1.2)] {
        let mut p = st.project().clone();
        let nominal = p.robot.mass();
        p.robot.weighed_mass = calibration.map(|k| k * nominal);
        let other = (nominal - p.robot.frame.chassis_mass) * p.robot.mass_scale();
        set_chassis_mass(&mut p.robot, 52.0);
        let p = model::Project::from_json(&p.to_json()).unwrap();
        let w = sim::world::World::test_drive(&p, &st.lib, "coral-2025").unwrap();
        let body = w.engine.id(sim::engine::Obj::Body, &format!("l{CHASSIS}")).unwrap();
        assert!((w.engine.body_mass(body) - 52.0).abs() < 0.001);
        assert!((w.engine.subtree_mass(body) - 52.0 - other).abs() < 0.001);
    }
}

fn resize_chassis(st: &mut AppState, frame: model::Frame, what: &str) {
    if frame.length <= 0.1 || frame.width <= 0.1 || frame.bumper_thickness <= 0.0 {
        st.toast("Use a frame larger than 100 mm and a positive bumper thickness.");
        return;
    }
    st.doc.edit(what, |p| {
        let r = &mut p.robot;
        let link = model::chassis_link(&frame);
        if let Some(chassis) = r.link_mut(CHASSIS) {
            *chassis = link;
        }
        r.frame = frame;
        let roots: Vec<_> = r
            .mechanisms
            .iter()
            .filter_map(|m| Some((m.id, m.template.clone()?)))
            .filter(|(_, t)| t.mount.on.is_none())
            .collect();
        for (id, t) in roots {
            let _ = templates::rebuild(r, id, t);
        }
    });
}

pub fn center(root: &mut Ui, st: &mut AppState, sim: &Sim) -> f32 {
    if st.assembly.driving() {
        let top = root.available_rect_before_wrap().top();
        if let Ok(shared) = sim.shared.lock()
            && let Some(err) = &shared.error
        {
            egui::Area::new("assembly_sim_error".into())
                .fixed_pos(root.available_rect_before_wrap().left_top())
                .show(root.ctx(), |ui| {
                    ui.label(RichText::new(err).color(BAD));
                });
        }
        return top;
    }
    let bar = egui::Panel::top("assembly_views")
        .show(root, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.selectable_value(&mut st.assembly.view, View::Side, "Side");
                ui.selectable_value(&mut st.assembly.view, View::Top, "Top");
                let reference_name = st.game(false).ok()
                    .and_then(|g| g.pack.design_reference.as_ref()).map(|r| r.name.clone());
                if let Some(name) = reference_name {
                    ui.separator();
                    let mut show = !st.assembly.outline_hidden;
                    ui.checkbox(&mut show, name).on_hover_text("Compare the robot with the scoring structure at the same scale.");
                    st.assembly.outline_hidden = !show;
                    if show && st.assembly.task == Task::Layout {
                        ui.label("Bumper gap").on_hover_text("Distance from the front bumper to the field structure.");
                        let unit = st.unit(Kind::Length);
                        if num(ui, "bumper_gap", &mut st.assembly.bumper_gap, Kind::Length, unit) {
                            st.assembly.bumper_gap = st.assembly.bumper_gap.max(0.0);
                        }
                    }
                }
                if st.assembly.task == Task::Layout {
                    ui.separator();
                    let mut snap = !st.assembly.free_placement;
                    if ui.checkbox(&mut snap, "Snap to mounts").changed() {
                        st.assembly.free_placement = !snap;
                    }
                    let mut label = st.assembly.preview.label().to_owned();
                    if st.assembly.preview == Preview::Start
                        && let Some(p) = st.project().robot.starting_preset()
                    {
                        label.push_str(&format!(" · {}", p.name));
                    }
                    egui::ComboBox::from_id_salt("preview_pose")
                        .selected_text(label)
                        .show_ui(ui, |ui| {
                            for mode in [Preview::Start, Preview::Stowed, Preview::Deployed] {
                                ui.selectable_value(&mut st.assembly.preview, mode, mode.label());
                            }
                        })
                        .response
                        .on_hover_text("Preview the robot at its travel limits");
                }
            });
        })
        .response
        .rect;
    egui::CentralPanel::default().show(root, |ui| {
        if st.assembly.task == Task::Layout {
            hint(ui, "Drag a part to move it. Drag the gold dot to resize.");
        } else if let Some(p) = controls::selected(st) {
            ui.label(RichText::new(&p.name).strong());
            hint(ui, "Drag a blue handle to teach this position.");
        }
        drawing::show(ui, st);
    });
    bar.bottom()
}
