use std::collections::HashMap;

use bevy_egui::egui::{self, Align2, Color32, FontId, Pos2, Rect, Sense, Stroke, Ui, Vec2, pos2, vec2};
use model::math::{add, axis_angle, scale};
use model::{Id, JointKind, MechKind, Pose, Robot, ShapeKind, V3};

use super::{AppState, Params, Preview, Selection, View, edit_template, mount_name, templates};
use crate::ui::widgets::{ACCENT, MUTED};

fn project(view: View, p: V3) -> Pos2 {
    pos2(p[0] as f32, -p[if view == View::Side { 2 } else { 1 }] as f32)
}

/// Transform design-space geometry about each joint, propagating the carrier's motion to its children.
fn poses(r: &Robot, deployed: bool) -> HashMap<Id, Pose> {
    pose_map(r, if deployed { Preview::Deployed } else { Preview::Stowed }, None)
}

fn pose_map(r: &Robot, preview: Preview, preset: Option<&model::Preset>) -> HashMap<Id, Pose> {
    let use_setpoints = preview == Preview::Start || preset.is_some();
    let mut values =
        r.starting_preset().filter(|_| use_setpoints).map(|p| r.position_targets(p)).unwrap_or_default();
    if let Some(p) = preset {
        values.extend(r.position_targets(p));
    }
    let mut poses = HashMap::new();
    for id in r.link_order() {
        let Some(j) = r.joint_of(id) else {
            poses.insert(id, Pose::default());
            continue;
        };
        let parent = poses.get(&j.parent).copied().unwrap_or_default();
        let kind = r.mechanism_of_link(id).map(|m| m.kind);
        let [lo, hi] = j.range.unwrap_or([j.initial; 2]);
        let value = if use_setpoints {
            let v = j
                .follows
                .as_ref()
                .and_then(|f| values.get(&f.joint).map(|v| v * f.ratio))
                .or_else(|| values.get(&j.id).copied())
                .unwrap_or(j.initial);
            j.range.map_or(v, |[a, b]| v.clamp(a.min(b), a.max(b)))
        } else {
            match (j.kind, kind, preview == Preview::Deployed) {
                (JointKind::Slide, _, true) => hi,
                (JointKind::Hinge, Some(MechKind::Intake), false) => hi,
                (JointKind::Hinge, Some(MechKind::Climber), true) => {
                    (-90f64).to_radians().clamp(lo.min(hi), lo.max(hi))
                }
                (JointKind::Hinge, Some(MechKind::Arm | MechKind::Wrist), true) => {
                    0.0f64.clamp(lo.min(hi), lo.max(hi))
                }
                _ => j.initial,
            }
        };
        values.insert(j.id, value);
        let local = match j.kind {
            JointKind::Slide => Pose::at(scale(j.axis, value)),
            JointKind::Hinge => Pose::at(j.anchor)
                .mul(&Pose::new([0.0; 3], axis_angle(j.axis, value)))
                .mul(&Pose::at(scale(j.anchor, -1.0))),
            JointKind::Fixed => Pose::default(),
        };
        poses.insert(id, parent.mul(&local));
    }
    poses
}

fn cross(a: Pos2, b: Pos2, c: Pos2) -> f32 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

/// Clockwise outline in screen coordinates, including tilted wheels and guide plates.
fn outline(mut points: Vec<Pos2>) -> Vec<Pos2> {
    points.retain(|p| p.is_finite());
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    points.dedup();
    if points.len() < 3 {
        return points;
    }
    let mut hull = Vec::new();
    for seq in [false, true] {
        let start = hull.len();
        for i in 0..points.len() {
            let p = points[if seq { points.len() - 1 - i } else { i }];
            while hull.len() >= start + 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    hull
}

#[derive(Clone)]
struct Part {
    outline: Vec<Pos2>,
    color: Color32,
    owner: Option<Id>,
}

fn silhouette(shape: &ShapeKind, pose: Pose, view: View) -> Vec<Pos2> {
    let mut points = Vec::new();
    let mut point = |p| points.push(project(view, pose.apply(p)));
    match shape {
        ShapeKind::Box { size } => {
            for x in [-0.5, 0.5] {
                for y in [-0.5, 0.5] {
                    for z in [-0.5, 0.5] {
                        point([x * size[0], y * size[1], z * size[2]]);
                    }
                }
            }
        }
        ShapeKind::Cylinder { radius, length } => {
            for i in 0..24 {
                let (s, c) = (i as f64 * std::f64::consts::TAU / 24.0).sin_cos();
                for z in [-length / 2.0, length / 2.0] {
                    point([c * radius, s * radius, z]);
                }
            }
        }
        ShapeKind::Sphere { radius } => {
            let center = project(view, pose.pos);
            for i in 0..24 {
                points.push(center + Vec2::angled(i as f32 * std::f32::consts::TAU / 24.0) * *radius as f32);
            }
        }
        ShapeKind::Hull { points: vertices } => {
            for p in vertices {
                point(*p);
            }
        }
    }
    outline(points)
}

fn parts(r: &Robot, lib: &library::Library, poses: &HashMap<Id, Pose>, view: View) -> Vec<Part> {
    let mut parts = Vec::new();
    for l in &r.links {
        let Some(pose) = poses.get(&l.id) else { continue };
        let owner = r.mechanism_of_link(l.id).map(|m| m.id);
        for s in &l.shapes {
            let color = Color32::from_hex(s.color.as_deref().unwrap_or(&l.color)).unwrap_or(MUTED);
            parts.push(Part { outline: silhouette(&s.kind, pose.mul(&s.pose), view), color, owner });
        }
    }
    for x in &r.rollers {
        let Some(shaft) = r.shaft(x.shaft) else { continue };
        let Some(pose) = poses.get(&shaft.link) else { continue };
        let wheel = ShapeKind::Cylinder { radius: x.radius(lib), length: x.width };
        let color = lib.wheel(&x.wheel).and_then(|w| Color32::from_hex(&w.color).ok()).unwrap_or(MUTED);
        parts.push(Part {
            outline: silhouette(&wheel, pose.mul(&Pose::along(x.center, x.axis)), view),
            color,
            owner: r.mechanism_of_link(shaft.link).map(|m| m.id),
        });
    }
    parts
}

#[derive(Clone, Copy)]
struct Scale {
    origin: Pos2,
    pixels: f32,
}
impl Scale {
    fn screen(self, point: Pos2) -> Pos2 {
        self.origin + point.to_vec2() * self.pixels
    }
    fn fit(rect: Rect, parts: &[Part], paths: &[TravelPath], view: View, guides: Rect) -> Self {
        let mut bounds = guides.union(Rect::from_min_max(
            pos2(-0.55, if view == View::Side { -0.85 } else { -0.55 }),
            pos2(0.55, if view == View::Side { 0.03 } else { 0.55 }),
        ));
        for p in parts.iter().flat_map(|p| &p.outline) {
            bounds.extend_with(*p);
        }
        for p in paths.iter().flat_map(|path| &path.points) {
            bounds.extend_with(*p);
        }
        let px = ((rect.width() - 64.0) / bounds.width()).min((rect.height() - 72.0) / bounds.height()).max(1.0);
        Self { origin: rect.center() - bounds.center().to_vec2() * px, pixels: px }
    }
}

/// Render palette thumbnails from the same template geometry as the full-size drawing.
pub(super) fn thumbnail(ui: &Ui, rect: Rect, kind: MechKind, lib: &library::Library) {
    let key = ui.id().with(("mechanism_thumbnail", kind.label()));
    let icon = ui.data(|d| d.get_temp::<std::sync::Arc<Vec<Part>>>(key)).unwrap_or_else(|| {
        let mut robot = Robot::new("");
        let id = templates::add(&mut robot, templates::Template::default_for(kind).unwrap()).unwrap();
        let mut icon = parts(&robot, lib, &poses(&robot, false), View::Side);
        icon.retain(|p| p.owner == Some(id));
        let icon = std::sync::Arc::new(icon);
        ui.data_mut(|d| d.insert_temp(key, icon.clone()));
        icon
    });
    let bounds = Rect::from_points(&icon.iter().flat_map(|p| p.outline.iter().copied()).collect::<Vec<_>>());
    let pixels = (rect.width() / bounds.width()).min(rect.height() / bounds.height());
    if !pixels.is_finite() {
        return;
    }
    for part in icon.iter().filter(|p| p.outline.len() >= 3) {
        let points = part.outline.iter().map(|p| rect.center() + (*p - bounds.center()) * pixels).collect();
        ui.painter().add(egui::Shape::convex_polygon(points, part.color, Stroke::new(0.5, part.color)));
    }
}

pub(super) fn show(ui: &mut Ui, st: &mut AppState) {
    let teaching = st.assembly.task == super::Task::Controls;
    let preset = teaching.then(|| super::controls::selected(st)).flatten().cloned();
    let preview = if teaching { Preview::Start } else { st.assembly.preview };
    let starting = preview == Preview::Start
        && preset.as_ref().is_none_or(|p| Some(p.id) == st.project().robot.controls.start_position);
    let poses = pose_map(&st.project().robot, preview, preset.as_ref());
    let guides = super::size_checks::panel(ui, st, &poses, starting);
    let (rect, canvas) = ui.allocate_exact_size(ui.available_size(), Sense::click_and_drag());
    if rect.width() < 60.0 || rect.height() < 60.0 {
        return;
    }
    let paint = ui.painter_at(rect);
    let view = st.assembly.view;
    let r = &st.project().robot;
    let parts = parts(r, &st.lib, &poses, view);
    let frozen_id = ui.id().with("dimension_drag_scale");
    let paths = preset.as_ref().map(|p| travel_paths(r, p, &poses, view)).unwrap_or_default();
    let guide = guides.as_ref().map(|g| {
        let x = (r.frame.length / 2.0 + g.limits.extension) as f32;
        let y = (r.frame.width / 2.0 + g.limits.extension) as f32;
        if view == View::Top {
            Rect::from_min_max(pos2(-x, -y), pos2(x, y))
        } else {
            Rect::from_min_max(
                pos2(-x, -(g.floor + g.limits.height.unwrap_or(0.0)) as f32),
                pos2(x, -g.floor as f32),
            )
        }
    });
    let reference = st.game.as_ref().and_then(|(_, found)| found.as_ref().ok())
        .and_then(|found| found.pack.design_reference.as_ref())
        .filter(|_| !st.assembly.outline_hidden)
        .map(|reference| if view == View::Side { &reference.side } else { &reference.top });
    let gap = if st.assembly.task == super::Task::Controls {
        super::controls::selected(st).map_or(st.assembly.bumper_gap, |p| p.reef_gap)
    } else { st.assembly.bumper_gap };
    let bumper = (r.frame.length / 2.0 + r.frame.bumper_thickness + gap) as f32;
    let reference_point = |[x, y]: [f32; 2]| pos2(bumper + x, -y);
    let mut bounds = guide.unwrap_or(Rect::NOTHING);
    if let Some(reference) = reference {
        for point in reference.contours.iter().flatten().copied().chain(reference.labels.iter().map(|l| l.at)) {
            bounds.extend_with(reference_point(point));
        }
    }
    let fit = Scale::fit(rect, &parts, &paths, view, bounds);
    let scale = ui.data(|d| d.get_temp::<Scale>(frozen_id)).unwrap_or(fit);
    paint.rect_filled(rect, 3.0, Color32::from_rgb(22, 26, 32));
    if let Some(reference) = reference {
        let ink = Color32::from_rgb(128, 178, 193);
        let screen = |p| scale.screen(reference_point(p));
        for path in reference.contours.iter().filter(|path| path.len() >= 3) {
            paint.add(egui::Shape::closed_line(path.iter().copied().map(screen).collect(), Stroke::new(1.5, ink)));
        }
        for label in &reference.labels {
            let at = screen(label.at);
            let align = if let Some(tip) = label.tip {
                paint.line_segment([screen(tip), at - vec2(4.0, 0.0)], Stroke::new(0.8, ink.gamma_multiply(0.6)));
                Align2::LEFT_CENTER
            } else { Align2::CENTER_CENTER };
            paint.text(at, align, &label.text, FontId::proportional(14.0), ink);
        }
    }
    if let Some(bounds) = guide {
        let limits = Rect::from_min_max(scale.screen(bounds.min), scale.screen(bounds.max));
        let line = |a, b| {
            paint.extend(egui::Shape::dashed_line(
                &[a, b],
                Stroke::new(1.5, Color32::from_rgb(244, 182, 97)),
                5.0,
                4.0,
            ))
        };
        if view == View::Top {
            let g = &guides.as_ref().unwrap().limits;
            let radius = g.extension as f32 * scale.pixels;
            let frame = limits.shrink(radius);
            for (i, corner) in [frame.right_bottom(), frame.left_bottom(), frame.left_top(), frame.right_top()]
                .into_iter()
                .enumerate()
            {
                let start = i as f32 * std::f32::consts::FRAC_PI_2;
                let at = |step: usize| {
                    corner + Vec2::angled(start + step as f32 / 12.0 * std::f32::consts::FRAC_PI_2) * radius
                };
                if radius > 0.0 {
                    let path = if g.one_side { vec![at(0), corner, at(12)] } else { (0..=12).map(at).collect() };
                    paint.extend(egui::Shape::dashed_line(
                        &path,
                        Stroke::new(1.5, Color32::from_rgb(244, 182, 97)),
                        5.0,
                        4.0,
                    ));
                }
            }
            line(pos2(frame.left(), limits.top()), pos2(frame.right(), limits.top()));
            line(pos2(frame.left(), limits.bottom()), pos2(frame.right(), limits.bottom()));
            line(pos2(limits.left(), frame.top()), pos2(limits.left(), frame.bottom()));
            line(pos2(limits.right(), frame.top()), pos2(limits.right(), frame.bottom()));
        } else {
            let top = if guides.as_ref().is_some_and(|g| g.limits.height.is_some()) {
                line(limits.left_top(), limits.right_top());
                limits.top()
            } else {
                rect.top()
            };
            line(pos2(limits.left(), top), limits.left_bottom());
            line(pos2(limits.right(), top), limits.right_bottom());
        }
    }
    let font = FontId::proportional(13.0);
    if view == View::Side {
        let y = scale.origin.y;
        paint.line_segment([pos2(rect.left(), y), pos2(rect.right(), y)], Stroke::new(1.0, MUTED));
        if !teaching
            && let Selection::Mechanism(id) = st.sel
            && let Some(m) = r.mechanism(id)
        {
            let bottom = parts
                .iter()
                .filter(|p| p.owner == Some(id))
                .flat_map(|p| &p.outline)
                .map(|p| -f64::from(p.y))
                .fold(f64::INFINITY, f64::min);
            let height = bottom - r.frame.bumper_bottom - r.frame.bumper_height;
            if height.is_finite() {
                let distance = library::units::format(height.abs(), st.unit(library::units::Kind::Length));
                let side = if height >= 0.0 { "above" } else { "below" };
                paint.text(
                    rect.left_top() + vec2(12.0, 10.0),
                    Align2::LEFT_TOP,
                    format!("{} bottom   {distance} {side} bumper top", m.name),
                    font.clone(),
                    MUTED,
                );
            }
        }
    }
    let ruler = if st.settings.imperial() { library::IN * 6.0 } else { 0.1 };
    let left = rect.left_bottom() + vec2(12.0, -14.0);
    let right = left + vec2(scale.pixels * ruler as f32, 0.0);
    if right.x < rect.center().x {
        paint.line_segment([left, right], Stroke::new(1.0, MUTED));
        for end in [left, right] {
            paint.line_segment([end - vec2(0.0, 3.0), end + vec2(0.0, 3.0)], Stroke::new(1.0, MUTED));
        }
        paint.text(
            left - vec2(0.0, 4.0),
            Align2::LEFT_BOTTOM,
            if st.settings.imperial() { "6 in" } else { "100 mm" },
            font.clone(),
            MUTED,
        );
    }
    let nose = rect.right_bottom() - vec2(29.0, 15.0);
    paint.arrow(nose, vec2(16.0, 0.0), Stroke::new(1.0, MUTED));
    paint.text(nose - vec2(5.0, 0.0), Align2::RIGHT_CENTER, "Front", font.clone(), MUTED);
    let pointer = ui.input(|i| i.pointer.press_origin().or(i.pointer.latest_pos()));
    let mut picked = None;
    for p in &parts {
        if p.outline.len() < 3 {
            continue;
        }
        let points: Vec<_> = p.outline.iter().map(|p| scale.screen(*p)).collect();
        let selected = p.owner.is_some_and(|id| st.sel == Selection::Mechanism(id));
        if pointer.is_some_and(|cursor| {
            (0..points.len()).all(|i| cross(points[i], points[(i + 1) % points.len()], cursor) >= -0.01)
        }) {
            picked = Some(p.owner.map_or(Selection::Robot, Selection::Mechanism));
        }
        let edge = if selected { ACCENT } else { p.color.lerp_to_gamma(Color32::LIGHT_GRAY, 0.25) };
        paint.add(egui::Shape::convex_polygon(
            points,
            p.color.gamma_multiply(0.65),
            Stroke::new(if selected { 1.8 } else { 1.0 }, edge),
        ));
    }
    if canvas.clicked()
        && let Some(sel) = picked.clone()
    {
        st.sel = sel;
    }
    if teaching {
        if let Some(p) = preset {
            teach_handles(ui, st, p.id, &paths, scale, frozen_id);
        }
        if !ui.input(|i| i.pointer.primary_down()) {
            ui.data_mut(|d| d.remove::<Scale>(frozen_id));
        }
        return;
    }
    if placement(ui, st, &canvas, picked, &poses, scale, frozen_id) {
        return;
    }
    if let Selection::Mechanism(id) = st.sel {
        dimension_handle(ui, st, &poses, scale, frozen_id, id);
        if let Some(t) = st.project().robot.mechanism(id).and_then(|m| m.template.as_ref()) {
            let at = scale.screen(project(view, anchor_world(&st.project().robot, t, &poses)));
            paint.circle_stroke(at, 5.0, Stroke::new(1.8, BLUE));
        }
    } else if st.sel == Selection::Robot {
        let mut frame = st.project().robot.frame.clone();
        let (label, point, axis, value) = if view == View::Side {
            ("Frame length", [frame.length / 2.0, 0.0, 0.065], vec2(0.5, 0.0), &mut frame.length)
        } else {
            ("Frame width", [0.0, frame.width / 2.0, 0.065], vec2(0.0, -0.5), &mut frame.width)
        };
        let at = scale.screen(project(view, point));
        let delta = resize_dot(ui, st, scale, frozen_id, model::CHASSIS, label, *value, at, axis * scale.pixels);
        if delta != 0.0 {
            *value = (*value + delta).max(0.13);
            super::resize_chassis(st, frame, "~Resize chassis");
        }
    }
    if !ui.input(|i| i.pointer.primary_down()) {
        ui.data_mut(|d| d.remove::<Scale>(frozen_id));
    }
}

const BLUE: Color32 = Color32::from_rgb(100, 207, 241);

fn carrier_pose(r: &Robot, parent: Option<Id>, poses: &HashMap<Id, Pose>) -> Pose {
    parent.and_then(|id| r.mechanism(id)?.end_link()).and_then(|l| poses.get(&l)).copied().unwrap_or_default()
}

fn anchor_world(r: &Robot, t: &templates::Template, poses: &HashMap<Id, Pose>) -> V3 {
    carrier_pose(r, t.mount.on, poses).apply(templates::placement_anchor(r, t))
}

#[derive(Clone)]
struct Grab {
    id: Id,
    to_anchor: Vec2,
}

fn placement(
    ui: &mut Ui,
    st: &mut AppState,
    canvas: &egui::Response,
    picked: Option<Selection>,
    poses: &HashMap<Id, Pose>,
    scale: Scale,
    frozen: egui::Id,
) -> bool {
    let view = st.assembly.view;
    let rect = canvas.rect;
    let grab_id = ui.id().with("grab_part");
    let pointer = ui.input(|i| i.pointer.press_origin().or(i.pointer.latest_pos()));
    let anchor_grip = match st.sel {
        Selection::Mechanism(id) => {
            st.project().robot.mechanism(id).and_then(|m| m.template.as_ref()).and_then(|t| {
                let at = scale.screen(project(view, anchor_world(&st.project().robot, t, poses)));
                pointer.filter(|p| at.distance(*p) <= 8.0).map(|_| Selection::Mechanism(id))
            })
        }
        _ => None,
    };
    let picked = anchor_grip.or(picked);
    if canvas.hovered() && matches!(picked, Some(Selection::Mechanism(_))) {
        ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
    }
    if canvas.drag_started()
        && let Some(Selection::Mechanism(id)) = picked
        && let Some(t) = st.project().robot.mechanism(id).and_then(|m| m.template.as_ref())
        && let Some(origin) = ui.input(|i| i.pointer.press_origin())
    {
        let at = scale.screen(project(view, anchor_world(&st.project().robot, t, poses)));
        ui.data_mut(|d| {
            d.insert_temp(grab_id, Grab { id, to_anchor: at - origin });
            d.insert_temp(frozen, scale);
        });
        st.sel = Selection::Mechanism(id);
    }
    let grab = ui.data(|d| d.get_temp::<Grab>(grab_id));
    let payload = egui::DragAndDrop::payload::<MechKind>(ui.ctx());
    let r = &st.project().robot;
    let (child, mut t, offset) = if let Some(kind) = payload {
        let Some(t) = templates::Template::default_for(*kind) else { return false };
        (None, t, Vec2::ZERO)
    } else if let Some(g) = grab {
        let Some(t) = r.mechanism(g.id).and_then(|m| m.template.clone()) else { return false };
        (Some(g.id), t, g.to_anchor)
    } else {
        return false;
    };
    let released = ui.input(|i| i.pointer.primary_released());
    let cancel = ui.input(|i| i.key_pressed(egui::Key::Escape) || !i.pointer.primary_down() && !released);
    if released || cancel {
        ui.data_mut(|d| {
            d.remove::<Grab>(grab_id);
            d.remove::<Scale>(frozen);
        });
        egui::DragAndDrop::take_payload::<MechKind>(ui.ctx());
    }
    if cancel {
        return true;
    }
    let cursor = ui.input(|i| i.pointer.latest_pos());
    let inside = cursor.filter(|p| rect.contains(*p));
    let anchor = inside.map(|p| p + offset);
    let paint = ui.painter_at(rect);
    let snap = !st.assembly.free_placement && !ui.input(|i| i.modifiers.alt);
    let mut sites = vec![(None, [0.0, 0.0, 0.075], "Chassis".into())];
    for m in &r.mechanisms {
        if child.is_some_and(|id| !templates::can_mount(r, id, Some(m.id))) {
            continue;
        }
        if let Some(pose) = m.end_link().and_then(|l| poses.get(&l)) {
            sites.push((Some(m.id), pose.apply(templates::tip(r, m)), mount_name(m)));
        }
    }
    let nearest = if snap {
        sites
            .iter()
            .enumerate()
            .filter_map(|(i, (_, at, _))| {
                let site = scale.screen(project(view, *at));
                let d = site.distance(anchor?).min(site.distance(inside?));
                (d <= 26.0).then_some((i, d))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
    } else {
        None
    };
    let world = if let Some(i) = nearest {
        t.mount.on = sites[i].0;
        Some(sites[i].1)
    } else {
        anchor.map(|screen| {
            let mut p = anchor_world(r, &t, poses);
            let xy = (screen - scale.origin) / scale.pixels;
            p[0] = xy.x as f64;
            p[if view == View::Side { 2 } else { 1 }] = -xy.y as f64;
            p
        })
    };
    if let Some(world) = world {
        let wanted = carrier_pose(r, t.mount.on, poses).inverse().apply(world);
        let delta = model::math::sub(wanted, templates::placement_anchor(r, &t));
        t.mount.offset = add(t.mount.offset, templates::mount_frame(r, t.mount.on).inverse().rotate(delta));
        if released {
            if let Some(id) = child {
                super::edit_template(st, id, t, "Move mechanism");
            } else {
                super::add_template(st, t);
            }
            return true;
        }
        // Rebuild a preview copy so the outline includes the moving assembly and its children.
        let mut preview = r.clone();
        let id = match child {
            Some(id) => templates::rebuild(&mut preview, id, t.clone()).ok().map(|_| id),
            None => templates::add(&mut preview, t.clone()).ok(),
        };
        if let Some(id) = id {
            let projected = pose_map(&preview, st.assembly.preview, None);
            for part in parts(&preview, &st.lib, &projected, view) {
                if part.outline.len() < 3
                    || !part.owner.is_some_and(|owner| !templates::can_mount(&preview, id, Some(owner)))
                {
                    continue;
                }
                let points = part.outline.iter().map(|p| scale.screen(*p)).collect();
                paint.add(egui::Shape::convex_polygon(points, BLUE.gamma_multiply(0.25), Stroke::new(1.6, BLUE)));
            }
        }
        let at = scale.screen(project(view, world));
        paint.circle_stroke(at, 10.0, Stroke::new(2.0, BLUE));
        let status = nearest.map_or_else(
            || {
                format!(
                    "Custom position on {}",
                    t.mount.on.and_then(|p| r.mechanism(p)).map_or("chassis", |m| m.name.as_str())
                )
            },
            |i| format!("Snap to {}", sites[i].2),
        );
        paint.text(
            rect.center_bottom() - vec2(0.0, 12.0),
            Align2::CENTER_BOTTOM,
            status,
            FontId::proportional(15.0),
            Color32::WHITE,
        );
    }
    if snap {
        for (i, (_, point, name)) in sites.iter().enumerate() {
            let at = scale.screen(project(view, *point));
            paint.circle_filled(at, if Some(i) == nearest { 7.0 } else { 4.5 }, BLUE);
            paint.text(
                at + vec2(10.0, -7.0),
                Align2::LEFT_BOTTOM,
                name,
                FontId::proportional(13.0),
                Color32::WHITE,
            );
        }
    }
    true
}

/// Trace each handle along the real joint's travel. Picking a point on this path works for
/// hinged tools, nested carriers and cascade elevators in either orthographic view.
struct TravelPath {
    drive: Id,
    mechanism: Id,
    name: String,
    range: [f64; 2],
    value: f64,
    display_value: f64,
    unit_kind: library::units::Kind,
    handle: Pos2,
    points: Vec<Pos2>,
}

fn travel_paths(r: &Robot, preset: &model::Preset, poses: &HashMap<Id, Pose>, view: View) -> Vec<TravelPath> {
    let mut paths = Vec::new();
    for drive in &r.drives {
        let model::DriveTarget::Joint { joint, .. } = drive.target else { continue };
        let Some(j) = r.joint(joint) else { continue };
        let Some([lo, hi]) = j.range else { continue };
        if hi <= lo {
            continue;
        }
        let Some(m) = r.mechanism_of_link(j.child) else { continue };
        let parent = poses.get(&j.parent).copied().unwrap_or_default();
        let tip = match m.template.as_ref().map(|t| &t.params) {
            Some(Params::Turret { radius, .. }) => templates::geometry_pose(r, m.template.as_ref().unwrap()).apply([*radius, 0.0, 0.02]),
            Some(Params::Intake { .. }) => r
                .rollers
                .iter()
                .find(|roller| r.shaft(roller.shaft).is_some_and(|s| s.link == j.child))
                .map_or(j.anchor, |r| r.center),
            _ => templates::tip(r, m),
        };
        let factor = super::controls::travel_factor(r, j);
        let point = |v| {
            let p = match j.kind {
                JointKind::Slide => add(tip, model::math::scale(j.axis, v * factor)),
                JointKind::Hinge => add(
                    j.anchor,
                    Pose::new([0.0; 3], axis_angle(j.axis, v)).rotate(model::math::sub(tip, j.anchor)),
                ),
                JointKind::Fixed => tip,
            };
            project(view, parent.apply(p))
        };
        let v = super::controls::pose_value(r, Some(preset), drive.id, j).clamp(lo, hi);
        let at = point(v);
        let path: Vec<_> = (0..=64).map(|i| point(lo + (hi - lo) * i as f64 / 64.0)).collect();
        paths.push(TravelPath {
            drive: drive.id,
            mechanism: m.id,
            name: m.name.clone(),
            range: [lo, hi],
            value: v,
            display_value: v * factor,
            unit_kind: if j.kind == JointKind::Slide {
                library::units::Kind::Length
            } else {
                library::units::Kind::Angle
            },
            handle: at,
            points: path,
        });
    }
    paths
}

fn teach_handles(
    ui: &mut Ui,
    st: &mut AppState,
    preset: Id,
    paths: &[TravelPath],
    scale: Scale,
    frozen: egui::Id,
) {
    let paint = ui.painter();
    let mut edit = None;
    let mut seal = false;
    for travel in paths {
        let [lo, hi] = travel.range;
        let v = travel.value;
        let at = scale.screen(travel.handle);
        let path: Vec<_> = travel.points.iter().map(|p| scale.screen(*p)).collect();
        let bounds = Rect::from_points(&path);
        if bounds.width().max(bounds.height()) < 8.0 {
            continue;
        }
        paint.add(egui::Shape::line(path.clone(), Stroke::new(1.5, BLUE.gamma_multiply(0.60))));
        paint.circle_filled(at, 8.0, BLUE);
        paint.circle_stroke(at, 8.0, Stroke::new(1.5, Color32::WHITE));
        let resp = ui
            .interact(
                Rect::from_center_size(at, vec2(27.0, 27.0)),
                ui.id().with(("teach", preset, travel.drive)),
                Sense::drag(),
            )
            .on_hover_cursor(egui::CursorIcon::Grab)
            .on_hover_text(format!("Set {} position", travel.name));
        if resp.dragged() || resp.hovered() || st.sel == Selection::Mechanism(travel.mechanism) {
            let value = library::units::format(travel.display_value, st.unit(travel.unit_kind));
            let near_right = at.x > ui.clip_rect().center().x;
            let (dx, align) = if near_right { (-11.0, Align2::RIGHT_BOTTOM) } else { (11.0, Align2::LEFT_BOTTOM) };
            paint.text(
                at + vec2(dx, -7.0),
                align,
                format!("{}  {value}", travel.name),
                FontId::proportional(14.0),
                Color32::WHITE,
            );
        }
        if resp.drag_started() {
            seal = true;
            ui.data_mut(|d| d.insert_temp(frozen, scale));
        }
        if resp.dragged()
            && let Some(p) = resp.interact_pointer_pos()
        {
            let mut best = (f32::INFINITY, v);
            for (i, w) in path.windows(2).enumerate() {
                let d = w[1] - w[0];
                if d.length_sq() < 0.001 {
                    continue;
                }
                let f = ((p - w[0]).dot(d) / d.length_sq()).clamp(0.0, 1.0);
                let val = lo + (hi - lo) * (i as f64 + f as f64) / 64.0;
                let dist = (w[0] + d * f).distance_sq(p);
                if dist < best.0 - 0.001 || (dist - best.0).abs() < 0.001 && (val - v).abs() < (best.1 - v).abs() {
                    best = (dist, val);
                }
            }
            if best.0.is_finite() {
                edit = Some((travel.mechanism, travel.drive, best.1));
            }
        }
        if resp.drag_stopped() {
            seal = true;
        }
    }
    if seal {
        st.doc.seal();
    }
    if let Some((m, drive, v)) = edit {
        st.sel = Selection::Mechanism(m);
        super::controls::set_value(st, preset, drive, v);
    }
}

/// A visible endpoint and its movement for a unit change in the same template field as the inspector.
fn dimension_handle(
    ui: &mut Ui,
    st: &mut AppState,
    poses: &HashMap<Id, Pose>,
    scale: Scale,
    frozen_id: egui::Id,
    id: Id,
) {
    let r = &st.project().robot;
    let Some(m) = r.mechanism(id) else { return };
    let Some(mut t) = m.template.clone() else { return };
    let Some(pose) = m.links.first().and_then(|l| poses.get(l)) else { return };
    let placement = templates::geometry_pose(r, &t);
    let base = [0.0; 3];
    let view = st.assembly.view;
    let (label, endpoint, direction, min, value) = match (&mut t.params, view) {
        (Params::Elevator { height, .. }, View::Side) => {
            ("Stage height", add(base, [0.0, 0.0, *height]), [0.0, 0.0, 1.0], 0.15, height)
        }
        (Params::Claw { depth, .. }, View::Side) => {
            ("Depth", add(base, [*depth, 0.0, 0.0]), [1.0, 0.0, 0.0], 0.025, depth)
        }
        (Params::Intake { height, ramp_deg, .. }, View::Side) => {
            let Some(roller) =
                r.rollers.iter().find(|x| r.shaft(x.shaft).is_some_and(|s| m.links.contains(&s.link)))
            else {
                return;
            };
            let (s, c) = ramp_deg.clamp(10.0, 60.0).to_radians().sin_cos();
            ("Roller height", placement.inverse().apply(roller.center), [s, 0.0, c], 0.005, height)
        }
        (
            Params::Elevator { width, .. }
            | Params::Claw { width, .. }
            | Params::Arm { width, .. }
            | Params::Wrist { width, .. }
            | Params::Intake { width, .. }
            | Params::Shooter { width, .. },
            View::Top,
        ) => {
            let mut end = add(base, [0.0, *width / 2.0, 0.0]);
            if matches!(m.kind, MechKind::Intake) {
                if let Some(l) = m.links.first().and_then(|id| r.link(*id))
                    && let Some(s) = l.shapes.first()
                {
                    end[0] = placement.inverse().apply(s.pose.pos)[0];
                }
            }
            ("Width", end, [0.0, 0.5, 0.0], if m.kind == MechKind::Elevator { 0.16 } else { 0.035 }, width)
        }
        (Params::Arm { length, .. } | Params::Wrist { length, .. } | Params::Telescope { length, .. }, _) => {
            ("Length", add(base, [*length, 0.0, 0.0]), [1.0, 0.0, 0.0], 0.025, length)
        }
        (Params::Turret { radius, .. }, View::Top) => {
            ("Radius", add(base, [*radius, 0.0, 0.0]), [1.0, 0.0, 0.0], 0.025, radius)
        }
        (Params::Climber { length, .. }, View::Side) => {
            ("Length", add(base, [0.0, 0.0, *length]), [0.0, 0.0, 1.0], 0.025, length)
        }
        _ => return,
    };
    let placed = pose.mul(&placement);
    let at = scale.screen(project(view, placed.apply(endpoint)));
    let axis = project(view, placed.rotate(direction)).to_vec2() * scale.pixels;
    if axis.length() < 1.0 {
        return;
    }
    let delta = resize_dot(ui, st, scale, frozen_id, id, label, *value, at, axis);
    if delta != 0.0 {
        *value = (*value + delta).max(min);
        edit_template(st, id, t, &format!("~Resize {label}"));
    }
}

fn resize_dot(
    ui: &mut Ui,
    st: &mut AppState,
    scale: Scale,
    frozen_id: egui::Id,
    id: Id,
    label: &str,
    value: f64,
    at: Pos2,
    axis: Vec2,
) -> f64 {
    let resp = ui
        .interact(Rect::from_center_size(at, vec2(24.0, 24.0)), ui.id().with(("resize", id)), Sense::drag())
        .on_hover_text(format!("Drag to change {label}"))
        .on_hover_cursor(egui::CursorIcon::Grab);
    let paint = ui.painter();
    paint.circle_filled(at, 7.0, ACCENT);
    paint.circle_stroke(at, 7.0, Stroke::new(1.5, Color32::WHITE));
    let text = format!("{label}   {}", library::units::format(value, st.unit(library::units::Kind::Length)));
    let (align, dx) =
        if at.x > ui.clip_rect().center().x { (Align2::RIGHT_BOTTOM, -9.0) } else { (Align2::LEFT_BOTTOM, 9.0) };
    paint.text(at + vec2(dx, -6.0), align, text, FontId::proportional(14.0), Color32::WHITE);
    if resp.drag_started() {
        st.doc.seal();
        ui.data_mut(|d| d.insert_temp(frozen_id, scale));
    }
    if resp.drag_stopped() {
        st.doc.seal();
    }
    (resp.drag_delta().dot(axis) / axis.length_sq()) as f64
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::{Context, Event, FullOutput, PointerButton, RawInput};

    fn frame(ctx: &Context, st: &mut AppState, events: Vec<Event>) -> FullOutput {
        let input = RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(1000.0, 700.0))),
            events,
            ..Default::default()
        };
        ctx.run_ui(input, |ui| {
            egui::Panel::left("palette").exact_size(205.0).show(ui, |ui| super::super::palette(ui, st));
            egui::CentralPanel::default().show(ui, |ui| show(ui, st));
        })
    }

    fn label(output: &FullOutput, text: &str) -> Pos2 {
        output
            .shapes
            .iter()
            .find_map(|shape| match &shape.shape {
                egui::Shape::Text(t) if t.galley.text() == text => Some(t.visual_bounding_rect().center()),
                _ => None,
            })
            .unwrap_or_else(|| panic!("missing label {text}"))
    }

    fn button(pos: Pos2, pressed: bool) -> Event {
        Event::PointerButton { pos, button: PointerButton::Primary, pressed, modifiers: egui::Modifiers::NONE }
    }

    fn grab_and_move(ctx: &Context, st: &mut AppState, from: Pos2, to: Pos2) -> FullOutput {
        frame(ctx, st, vec![Event::PointerMoved(from), button(from, true)]);
        frame(ctx, st, vec![Event::PointerMoved(from + vec2(8.0, 0.0))]);
        frame(ctx, st, vec![Event::PointerMoved(to)])
    }

    #[test]
    fn reef_reference_tracks_view_gap_and_season_without_editing_the_robot() {
        let ctx = Context::default();
        let mut st = AppState::new(Default::default());
        let robot = st.project().robot.clone();
        frame(&ctx, &mut st, vec![]);
        let side = frame(&ctx, &mut st, vec![]);
        let low = label(&side, "L1");
        assert!(label(&side, "L4").y < label(&side, "L3").y && label(&side, "L2").y < low.y);
        st.assembly.bumper_gap = 0.20;
        let spaced = frame(&ctx, &mut st, vec![]);
        assert!(label(&spaced, "L1").x > low.x);
        st.assembly.view = View::Top;
        let has_reef = |out: &FullOutput| out.shapes.iter().any(|s| matches!(&s.shape, egui::Shape::Text(t) if t.galley.text() == "REEF"));
        assert!(has_reef(&frame(&ctx, &mut st, vec![])));
        st.assembly.outline_hidden = true;
        assert!(!has_reef(&frame(&ctx, &mut st, vec![])));
        st.assembly.outline_hidden = false;
        st.doc.project.game = "rebuilt-2026".into();
        assert!(!has_reef(&frame(&ctx, &mut st, vec![])));
        assert_eq!(st.project().robot, robot);
    }

    #[test]
    fn free_drop_move_body_cancel_and_snap_existing_claw() {
        let ctx = Context::default();
        let mut st = AppState::new(Default::default());
        super::super::add_suggested(&mut st, MechKind::Elevator);
        let Selection::Mechanism(elevator) = st.sel else { panic!() };
        frame(&ctx, &mut st, vec![]);
        let initial = frame(&ctx, &mut st, vec![]);
        let tile = label(&initial, "Roller claw");
        let at = pos2(780.0, 340.0);
        grab_and_move(&ctx, &mut st, tile, at);
        frame(&ctx, &mut st, vec![button(at, false)]);
        let Selection::Mechanism(claw) = st.sel else { panic!() };
        assert_ne!(claw, elevator);
        let template = st.project().robot.mechanism(claw).unwrap().template.clone().unwrap();
        assert_eq!(template.mount.on, None);
        assert!(template.mount.offset[2] > 0.1);
        let before = st.project().clone();
        let selected = frame(&ctx, &mut st, vec![]);
        let pivot = selected
            .shapes
            .iter()
            .find_map(|s| match &s.shape {
                egui::Shape::Circle(c) if c.stroke.color == BLUE && c.radius == 5.0 => Some(c.center),
                _ => None,
            })
            .unwrap();
        let grip = pivot + vec2(10.0, 4.0);
        let outside = pos2(30.0, 600.0);
        grab_and_move(&ctx, &mut st, grip, outside);
        frame(&ctx, &mut st, vec![button(outside, false)]);
        assert_eq!(st.project(), &before);
        let dragged = grab_and_move(&ctx, &mut st, grip, grip + vec2(40.0, 0.0));
        let site = dragged
            .shapes
            .iter()
            .find_map(|s| match &s.shape {
                egui::Shape::Text(t) if t.galley.text() == "Elevator carriage" => Some(t.pos + vec2(-10.0, 18.0)),
                _ => None,
            })
            .expect("dragging the body exposes the carriage");
        let drop = site + grip.to_vec2() - pivot.to_vec2();
        frame(&ctx, &mut st, vec![Event::PointerMoved(drop)]);
        frame(&ctx, &mut st, vec![button(drop, false)]);
        assert_eq!(
            st.project().robot.mechanism(claw).unwrap().template.as_ref().unwrap().mount.on,
            Some(elevator)
        );
        st.doc.undo();
        assert_eq!(st.project(), &before);
    }

    #[test]
    fn saved_start_drives_the_preview_and_the_physical_carriage() {
        let mut st = AppState::new(Default::default());
        super::super::add_suggested(&mut st, MechKind::Elevator);
        super::super::add_suggested(&mut st, MechKind::Claw);
        let Selection::Mechanism(claw) = st.sel else { panic!() };
        let start_id = st.project().robot.controls.presets.iter().find(|p| p.name == "Elevator mid").unwrap().id;
        let baseline = sim::world::World::test_drive(st.project(), &st.lib, "fuel-2026").unwrap().snapshot();
        st.assembly.open_controls();
        st.assembly.preset = Some(start_id);
        let ctx = Context::default();
        let editor = |st: &mut AppState, events| {
            let input = RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(500.0, 800.0))),
                events,
                ..Default::default()
            };
            ctx.run_ui(input, |ui| super::super::controls::inspector(ui, st))
        };
        editor(&mut st, vec![]);
        let output = editor(&mut st, vec![]);
        let checkbox = label(&output, "Use as starting position");
        editor(&mut st, vec![Event::PointerMoved(checkbox), button(checkbox, true)]);
        editor(&mut st, vec![button(checkbox, false)]);
        let saved = model::Project::from_json(&st.project().to_json()).unwrap();
        let robot = &saved.robot;
        let preset = robot.starting_preset().unwrap();
        assert_eq!(preset.id, start_id);
        let child = robot.mechanism(claw).unwrap().links[0];
        let drawn = pose_map(robot, Preview::Start, None)[&child].pos[2];
        assert!(drawn > 0.1);
        let actual = sim::world::World::test_drive(&saved, &st.lib, "fuel-2026").unwrap().snapshot();
        let z = |snap: &sim::world::Snapshot| snap.links.iter().find(|l| l.id == child).unwrap().pose.pos[2];
        assert!((z(&actual) - z(&baseline) - drawn).abs() < 1e-7);
        let first = &preset.targets[0];
        let drive = actual.drives.iter().find(|d| d.id == first.drive).unwrap();
        assert!((drive.position - first.value).abs() < 1e-8 && (drive.target - first.value).abs() < 1e-8);
        let partial = model::Preset { reef_gap: model::default_reef_gap(), id: 0, name: String::new(), targets: vec![] };
        assert_eq!(pose_map(robot, Preview::Start, Some(&partial))[&child].pos[2], drawn);
        let output = editor(&mut st, vec![]);
        let delete = label(&output, "Delete position");
        editor(&mut st, vec![Event::PointerMoved(delete), button(delete, true)]);
        editor(&mut st, vec![button(delete, false)]);
        assert!(st.project().robot.controls.start_position.is_none());
        st.doc.undo();
        assert_eq!(st.project(), &saved);
        st.doc.undo();
        assert!(st.project().robot.starting_preset().is_none());
    }

    #[test]
    fn teaching_carriage_updates_saved_control_and_moves_attached_claw() {
        let ctx = Context::default();
        let mut st = AppState::new(Default::default());
        super::super::add_suggested(&mut st, MechKind::Elevator);
        super::super::add_suggested(&mut st, MechKind::Claw);
        let Selection::Mechanism(claw) = st.sel else { panic!() };
        st.assembly.open_controls();
        let before = st.project().clone();
        frame(&ctx, &mut st, vec![]);
        let initial = frame(&ctx, &mut st, vec![]);
        let handle = initial
            .shapes
            .iter()
            .find_map(|s| match &s.shape {
                egui::Shape::Circle(c) if c.fill == BLUE && c.radius == 8.0 => Some(c.center),
                _ => None,
            })
            .unwrap();
        let higher = handle - vec2(0.0, 90.0);
        grab_and_move(&ctx, &mut st, handle, higher);
        frame(&ctx, &mut st, vec![button(higher, false)]);
        let reopened = model::Project::from_json(&st.project().to_json()).unwrap();
        let r = &reopened.robot;
        let pose = &r.controls.presets[0];
        let target = &pose.targets[0];
        assert!(target.value > 0.0);
        let claw_link = r.mechanism(claw).unwrap().links[0];
        let stages = 1 + r.joints.iter().filter(|j| j.follows.is_some()).count();
        let moved = pose_map(r, Preview::Start, Some(pose))[&claw_link].pos[2];
        assert!((moved - target.value * stages as f64).abs() < 1e-8);
        let mut world = sim::world::World::new(&reopened, &st.lib, sim::world::Scene::carpet()).unwrap();
        let bound =
            r.controls.bindings.iter().find(|b| b.action == (model::Action::Preset { preset: pose.id })).unwrap();
        world.button(bound.button, true);
        assert_eq!(world.cmd.targets.get(&target.drive), Some(&target.value));
        st.doc.undo();
        assert_eq!(st.project(), &before);
    }

    #[test]
    fn assemble_resize_and_reopen_from_the_palette() {
        let ctx = Context::default();
        let mut st = AppState::new(Default::default());
        st.doc.project.archetype = true;
        frame(&ctx, &mut st, vec![]);
        let initial = frame(&ctx, &mut st, vec![]);
        let elevator_button = label(&initial, MechKind::Elevator.label());
        frame(&ctx, &mut st, vec![Event::PointerMoved(elevator_button), button(elevator_button, true)]);
        frame(&ctx, &mut st, vec![button(elevator_button, false)]);
        let Selection::Mechanism(elevator) = st.sel else { panic!("elevator should be selected") };

        let selected = frame(&ctx, &mut st, vec![]);
        let handle = selected
            .shapes
            .iter()
            .find_map(|shape| match &shape.shape {
                egui::Shape::Circle(c) if c.fill == ACCENT && c.radius == 7.0 => Some(c.center),
                _ => None,
            })
            .expect("dimension handle");
        let before = st.doc.project.clone();
        frame(&ctx, &mut st, vec![Event::PointerMoved(handle), button(handle, true)]);
        let taller = handle - vec2(0.0, 35.0);
        frame(&ctx, &mut st, vec![Event::PointerMoved(taller)]);
        frame(&ctx, &mut st, vec![button(taller, false)]);
        assert_ne!(st.doc.project, before);
        st.doc.undo();
        assert_eq!(st.doc.project, before);
        st.doc.redo();

        let selected = frame(&ctx, &mut st, vec![]);
        let claw_button = label(&selected, "Roller claw");
        frame(&ctx, &mut st, vec![Event::PointerMoved(claw_button), button(claw_button, true)]);
        frame(&ctx, &mut st, vec![Event::PointerMoved(claw_button + vec2(35.0, 0.0))]);
        let dragging = frame(&ctx, &mut st, vec![]);
        let target = dragging
            .shapes
            .iter()
            .find_map(|shape| match &shape.shape {
                egui::Shape::Text(t) if t.galley.text() == "Elevator carriage" => Some(t.pos),
                _ => None,
            })
            .expect("visible carriage drop target");
        // Mount labels start beside and slightly above their snap point.
        let carriage = target + vec2(-10.0, 18.0);
        frame(&ctx, &mut st, vec![Event::PointerMoved(carriage)]);
        frame(&ctx, &mut st, vec![button(carriage, false)]);
        let Selection::Mechanism(claw) = st.sel else { panic!("claw should be selected") };
        assert_ne!(claw, elevator);
        let reopened = model::Project::from_json(&st.doc.project.to_json()).unwrap();
        assert_eq!(crate::state::Workspace::for_project(&reopened), crate::state::Workspace::Archetype);
        let r = &reopened.robot;
        let claw = r.mechanism(claw).unwrap();
        assert_eq!(claw.template.as_ref().unwrap().mount.on, Some(elevator));
        let carriage_travel = match &r.mechanism(elevator).unwrap().template.as_ref().unwrap().params {
            Params::Elevator { travel, .. } => *travel,
            _ => unreachable!(),
        };
        let link = claw.links[0];
        assert!((poses(r, true)[&link].pos[2] - poses(r, false)[&link].pos[2] - carriage_travel).abs() < 1e-8);
        let mut world = sim::world::World::new(&reopened, &st.lib, sim::world::Scene::carpet()).unwrap();
        world.cmd.drive = [1.0, 0.0, 0.0];
        for _ in 0..500 {
            world.step();
        }
        assert!(world.chassis_pose().pos[0] > 0.1);
    }
}
