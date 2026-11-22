//! The 3D view: the robot's links drawn from the project, posed from the simulation's snapshot;
//! rollers spinning; pieces; the field; overlays for contact, centre of mass, the wheels'
//! footprint and pickup zones (interface rule 9); an orbit camera; click a link to select it.
//!
//! The simulation is z-up and Bevy is y-up: everything simulated hangs under one `SimRoot` turned
//! -90 degrees about x, so poses go in as they come out of the simulation.
use bevy::picking::events::PointerClick;
use bevy::picking::mesh_picking::MeshPickingPlugin;
use bevy::prelude::*;
use model::{Id, ShapeKind, ShapeRole, CHASSIS};
use std::collections::HashMap;

use crate::simrun::Sim;
use crate::state::{AppState, Selection, Workspace};

pub struct ViewPlugin;

impl Plugin for ViewPlugin {
    fn build(&self, app: &mut App) {
        let registry = app.world().resource::<bevy::asset::io::embedded::EmbeddedAssetRegistry>();
        for (path, data) in [
            ("fields/reefscape-2025.glb", include_bytes!("../../../assets/games/reefscape-2025/field.glb").as_slice()),
            ("fields/rebuilt-2026.glb", include_bytes!("../../../assets/games/rebuilt-2026/field.glb").as_slice()),
        ] {
            registry.insert_asset(Default::default(), std::path::Path::new(path), data);
        }
        app.add_plugins(MeshPickingPlugin)
            .init_resource::<Orbit>()
            .init_resource::<Built>()
            .init_resource::<PointerOverUi>()
            .add_systems(Startup, setup)
            .add_systems(Update, (rebuild, field, pose, overlays, orbit).chain());
    }
}

#[derive(Component)]
pub struct MainCam;
#[derive(Component)]
pub struct SimRoot;
#[derive(Component)]
pub struct LinkVis(pub Id);
#[derive(Component)]
pub struct RollerVis(pub Id);

#[derive(Component)]
struct TreadMesh {
    rest: Vec<[f32; 3]>,
    samples: Vec<[f32; 3]>,
    weights: Vec<(usize, f32)>,
    radius: f32,
    compressed: bool,
}

impl TreadMesh {
    fn new(rest: Vec<[f32; 3]>, radius: f32, core: f32) -> Self {
        // One surface probe per angular column and axial station. Interior pocket
        // vertices share its displacement, tapering to zero at the mounted core.
        let mut columns = HashMap::new();
        let mut samples = Vec::<[f32; 3]>::new();
        let mut mapping = Vec::new();
        for &[x,y,z] in &rest {
            let angle = (z.atan2(x) * 10000.).round() as i32;
            let i = *columns.entry((angle, y.to_bits())).or_insert_with(|| {
                samples.push([x,y,z]); samples.len()-1
            });
            let r = x.hypot(z);
            if r > samples[i][0].hypot(samples[i][2]) { samples[i] = [x,y,z]; }
            mapping.push((i,r));
        }
        let weights = mapping.into_iter().map(|(i,r)| {
            let edge = samples[i][0].hypot(samples[i][2]);
            (i, ((r-core)/(edge-core).max(0.0001)).clamp(0.,1.))
        }).collect();
        Self { rest, samples, weights, radius, compressed: false }
    }
}

#[derive(Component)]
struct RollerStripe;
#[derive(Component)]
pub struct PieceVis(pub usize);
#[derive(Component)]
pub struct FieldVis;
#[derive(Component)]
pub struct CadVis;

/// Set by the interface each frame: the pointer is over a panel, so the camera leaves it alone.
#[derive(Resource, Default)]
pub struct PointerOverUi(pub bool);

#[derive(Resource)]
pub struct Orbit { pub target: Vec3, pub yaw: f32, pub pitch: f32, pub dist: f32 }

// Looking along field +X from the blue driver station puts the camera on the -X side of its target.
const BLUE_STATION_ORBIT_YAW: f32 = -std::f32::consts::FRAC_PI_2;

impl Default for Orbit { fn default() -> Self { Orbit { target: Vec3::new(0.0, 0.4, 0.0), yaw: 0.9, pitch: 0.45, dist: 3.2 } } }

#[derive(Resource, Default)]
struct Built { rev: Option<u64>, generation: u64, pieces: usize, materials: HashMap<String, Handle<StandardMaterial>>, piece_meshes: HashMap<String, Handle<Mesh>>, cad_rev: u64 }

pub fn to_bevy(p: &model::Pose) -> Transform {
    Transform { translation: Vec3::new(p.pos[0] as f32, p.pos[1] as f32, p.pos[2] as f32), rotation: Quat::from_xyzw(p.quat[1] as f32, p.quat[2] as f32, p.quat[3] as f32, p.quat[0] as f32), scale: Vec3::ONE }
}

/// "#rrggbb" to a color
pub fn hex(s: &str) -> Color {
    let h = s.trim_start_matches('#');
    let c = |i: usize| u8::from_str_radix(h.get(i..i + 2).unwrap_or("80"), 16).unwrap_or(128);
    Color::srgb_u8(c(0), c(2), c(4))
}

fn setup(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>, mut ambient: ResMut<GlobalAmbientLight>,
    mut egui_settings: ResMut<bevy_egui::EguiGlobalSettings>) {
    commands.spawn((Camera3d::default(), MainCam, Msaa::Sample4, Transform::from_xyz(2.5, 1.8, 2.5).looking_at(Vec3::new(0.0, 0.4, 0.0), Vec3::Y)));
    // the interface draws through a camera of its own over the whole window; the 3D camera's
    // viewport is then just the part the panels leave (bevy_egui's side_panel example)
    egui_settings.auto_create_primary_context = false;
    commands.spawn((
        bevy_egui::PrimaryEguiContext,
        Camera2d,
        bevy::camera::visibility::RenderLayers::none(),
        Camera {
            order: 1,
            output_mode: bevy::camera::CameraOutputMode::Write { blend_state: Some(bevy::render::render_resource::BlendState::ALPHA_BLENDING), clear_color: ClearColorConfig::None },
            clear_color: ClearColorConfig::Custom(Color::NONE),
            ..default()
        },
    ));
    commands.spawn((DirectionalLight { illuminance: 9000.0, shadow_maps_enabled: true, ..default() }, Transform::from_xyz(3.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y)));
    ambient.brightness = 450.0;
    commands.spawn((SimRoot, Transform::from_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)), Visibility::default()));
    // the carpet
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(40.0, 40.0))),
        MeshMaterial3d(mats.add(StandardMaterial { base_color: Color::srgb(0.24, 0.25, 0.28), perceptual_roughness: 0.95, ..default() })),
        Transform::from_xyz(0.0, -0.001, 0.0),
    ));
}

fn material(built: &mut Built, mats: &mut Assets<StandardMaterial>, color: &str, alpha: f32) -> Handle<StandardMaterial> {
    let key = format!("{color}/{alpha}");
    built.materials.entry(key).or_insert_with(|| {
        let mut c = hex(color);
        c.set_alpha(alpha);
        mats.add(StandardMaterial { base_color: c, perceptual_roughness: 0.6, metallic: 0.1, alpha_mode: if alpha < 1.0 { AlphaMode::Blend } else { AlphaMode::Opaque }, ..default() })
    }).clone()
}

fn shape_mesh(k: &ShapeKind) -> (Mesh, Quat) {
    match k {
        ShapeKind::Cylinder { radius, length } => (Cylinder::new(*radius as f32, *length as f32).into(), Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        ShapeKind::Sphere { radius } => (Sphere::new(*radius as f32).mesh().ico(3).unwrap(), Quat::IDENTITY),
        // a box, or a CAD hull drawn as the box round it (the CAD itself is drawn over it)
        k => { let size = k.extents(); (Cuboid::new(size[0] as f32, size[1] as f32, size[2] as f32).into(), Quat::IDENTITY) }
    }
}

/// a link's body origin: its joint's anchor (the simulator places bodies there)
pub fn origin(r: &model::Robot, id: Id) -> [f64; 3] { if id == CHASSIS { [0.0; 3] } else { r.joint_of(id).map_or([0.0; 3], |j| j.anchor) } }

#[allow(clippy::too_many_arguments)]
fn rebuild(
    mut commands: Commands, st: Res<AppState>, sim: Res<Sim>, mut built: ResMut<Built>,
    mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>,
    root: Single<Entity, With<SimRoot>>, old: Query<Entity, Or<(With<LinkVis>, With<RollerVis>, With<PieceVis>)>>,
    cad: Option<Res<crate::ui::rig::CadMeshes>>,
) {
    if st.ws == Workspace::Archetype && !st.assembly.driving() { return; }
    let generation = sim.shared.lock().map(|s| s.generation).unwrap_or(0);
    let snap = sim.snap();
    let npieces = snap.as_ref().map_or(0, |s| s.pieces.len());
    let cad_rev = cad.as_ref().map_or(0, |c| c.rev);
    if built.rev == Some(st.doc.revision) && built.generation == generation && built.pieces == npieces && built.cad_rev == cad_rev { return; }
    built.rev = Some(st.doc.revision);
    built.generation = generation;
    built.pieces = npieces;
    built.cad_rev = cad_rev;
    for e in &old { commands.entity(e).despawn(); }
    let r = &st.project().robot;
    let ghost = |id: Id| st.ws == Workspace::Rig && st.import.hide_blocks && r.link(id).is_some_and(|l| !l.parts.is_empty());
    for l in &r.links {
        let o = origin(r, l.id);
        let link_e = commands.spawn((LinkVis(l.id), Transform::from_xyz(o[0] as f32, o[1] as f32, o[2] as f32), Visibility::default())).id();
        commands.entity(*root).add_child(link_e);
        // CAD parts, when the robot came from CAD
        if let Some(c) = &cad {
            for (pid, mesh, mat, local) in c.for_link(l, o) {
                let e = commands.spawn((Mesh3d(mesh), MeshMaterial3d(mat), local, CadVis, PartPick(pid))).observe(pick_part).id();
                commands.entity(link_e).add_child(e);
            }
        }
        if ghost(l.id) { continue; }
        for s in &l.shapes {
            let (mesh, fix) = shape_mesh(&s.kind);
            let color = s.color.clone().unwrap_or_else(|| l.color.clone());
            let alpha = match s.role { ShapeRole::Ghost => 0.25, ShapeRole::Guide => 0.55, _ => if !l.parts.is_empty() { 0.35 } else { 1.0 } };
            let mat = material(&mut built, &mut mats, if color.is_empty() { "#9aa3ad" } else { &color }, alpha);
            let mut t = to_bevy(&model::Pose::new([s.pose.pos[0] - o[0], s.pose.pos[1] - o[1], s.pose.pos[2] - o[2]], s.pose.quat));
            t.rotation *= fix;
            let e = commands.spawn((Mesh3d(meshes.add(mesh)), MeshMaterial3d(mat), t, LinkPick(l.id))).observe(pick_link).id();
            commands.entity(link_e).add_child(e);
        }
    }
    for x in &r.rollers {
        let wheel = st.lib.wheel(&x.wheel);
        let rad = x.radius(&st.lib) as f32;
        let color = wheel.map_or("#555555".to_string(), |w| if w.color.is_empty() { "#555555".into() } else { w.color.clone() });
        let mat = material(&mut built, &mut mats, &color, 1.0);
        let edits = r.shaft(x.shaft).and_then(|s| r.mechanism_of_link(s.link))
            .and_then(|m| m.template.as_ref()).map(|t| t.wheel_edits.clone()).unwrap_or_default();
        let section = wheel.map(|w| library::profile::Section::new(w, &edits))
            .filter(|s| s.cad.is_some() || s.irregular);
        let mesh = section.as_ref().map_or_else(
            || Cylinder::new(rad, x.width as f32).mesh().resolution(48).segments(4).build(),
            |s| wheel_section_mesh(s, rad, x.width as f32));
        let rest = mesh.attribute(Mesh::ATTRIBUTE_POSITION).unwrap().as_float3().unwrap().to_vec();
        let core = section.as_ref().map_or(0., |s| match s.cad {
            Some("am-3480") => rad * (20.955 / 50.8),
            Some("am-3945" | "am-5740") => rad * (10. / 38.1),
            Some("am-5123") => rad * (10. / 63.5),
            _ => 0.00635,
        });
        let tread = TreadMesh::new(rest, rad, core);
        let e = commands.spawn((RollerVis(x.id), tread, Mesh3d(meshes.add(mesh)), MeshMaterial3d(mat), Transform::default(), RollerPick(x.id))).observe(pick_roller).id();
        // a stripe to see it turn
        if section.is_none() {
            let stripe = material(&mut built, &mut mats, "#ffffff", 1.0);
            let s = commands.spawn((RollerStripe, Mesh3d(meshes.add(Cuboid::new(0.006, x.width as f32 + 0.002, 0.004))), MeshMaterial3d(stripe), Transform::from_xyz(rad, 0.0, 0.0))).id();
            commands.entity(e).add_child(s);
        }
        commands.entity(*root).add_child(e);
    }
    if let Some(snap) = &snap {
        for (i, p) in snap.pieces.iter().enumerate() {
            let Some(lp) = st.lib.piece(&p.kind) else { continue };
            let mesh = built.piece_meshes.entry(p.kind.clone()).or_insert_with(|| meshes.add(piece_mesh(lp))).clone();
            let mat = material(&mut built, &mut mats, if lp.color.is_empty() { "#dddddd" } else { &lp.color }, 1.0);
            let e = commands.spawn((PieceVis(i), Mesh3d(mesh), MeshMaterial3d(mat), to_bevy(&p.pose))).id();
            commands.entity(*root).add_child(e);
        }
    }
}

fn wheel_section_mesh(section: &library::profile::Section, radius: f32, width: f32) -> Mesh {
    use bevy::{asset::RenderAssetUsages, mesh::PrimitiveTopology};
    let mut vertices = Vec::new();
    for (i, bands) in section.bands.iter().enumerate() {
        let vertex = |j: usize, r: f64, y: f32| {
            let angle = std::f32::consts::TAU * j as f32 / section.bands.len() as f32;
            [radius * r as f32 * angle.cos(), y * width * 0.5, -radius * r as f32 * angle.sin()]
        };
        for &[lo, hi] in bands {
            let ring = |y| [vertex(i,lo,y), vertex(i,hi,y), vertex(i+1,hi,y), vertex(i+1,lo,y)];
            let bottom = ring(-1.);
            let top = ring(1.);
            let mut quad = |a,b,c,d| vertices.extend_from_slice(&[a,b,c,a,c,d]);
            quad(top[0],top[1],top[2],top[3]);
            quad(bottom[3],bottom[2],bottom[1],bottom[0]);
            for j in 0..4 {
                let k = (j+1)%4;
                quad(bottom[j],bottom[k],top[k],top[j]);
            }
        }
    }
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vertices);
    mesh.compute_normals();
    mesh
}

pub fn piece_mesh(p: &library::Piece) -> Mesh {
    use library::{PieceShape, IN};
    match p.shape {
        PieceShape::Sphere { d_in } => Sphere::new((d_in * IN / 2.0) as f32).mesh().ico(4).unwrap(),
        PieceShape::Tube { outer_d_in, inner_d_in, length_in } => Extrusion::new(Annulus::new((inner_d_in * IN / 2.0) as f32, (outer_d_in * IN / 2.0) as f32), (length_in * IN) as f32).mesh().resolution(40).into(),
        PieceShape::Cylinder { d_in, length_in } => {
            let mut m: Mesh = Cylinder::new((d_in * IN / 2.0) as f32, (length_in * IN) as f32).into();
            m.rotate_by(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2));
            m
        }
        PieceShape::Box { x_in, y_in, z_in } => Cuboid::new((x_in * IN) as f32, (y_in * IN) as f32, (z_in * IN) as f32).into(),
    }
}

#[derive(Component)]
pub struct LinkPick(pub Id);
#[derive(Component)]
pub struct RollerPick(pub Id);
#[derive(Component)]
pub struct PartPick(pub String);

fn pick_link(ev: On<PointerClick>, q: Query<&LinkPick>, mut st: ResMut<AppState>) {
    if ev.button != PointerButton::Primary { return; }
    let Ok(p) = q.get(ev.entity) else { return };
    let id = p.0;
    st.sel = match st.project().robot.mechanism_of_link(id) { Some(m) if matches!(st.ws, Workspace::Build | Workspace::Archetype) => Selection::Mechanism(m.id), _ => Selection::Link(id) };
}
fn pick_roller(ev: On<PointerClick>, q: Query<&RollerPick>, mut st: ResMut<AppState>) {
    if ev.button != PointerButton::Primary { return; }
    if let Ok(p) = q.get(ev.entity) { st.sel = Selection::Roller(p.0); }
}
fn pick_part(ev: On<PointerClick>, q: Query<&PartPick>, mut st: ResMut<AppState>) {
    if ev.button != PointerButton::Primary { return; }
    if let Ok(p) = q.get(ev.entity) { crate::ui::rig::clicked_part(&mut st, &p.0, ev.hit.position.map(|v| [v.x as f64, v.y as f64, v.z as f64])); }
}

#[derive(Default)]
struct FieldDrawing { revision: Option<u64>, pending: Option<Handle<WorldAsset>> }

/// Show the pack's simple field immediately, then replace it when its CAD is ready.
fn field(mut commands: Commands, sim: Res<Sim>, mut st: ResMut<AppState>, mut built: ResMut<Built>, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>,
    root: Single<Entity, With<SimRoot>>, old: Query<Entity, With<FieldVis>>, assets: Res<AssetServer>, mut drawing: Local<FieldDrawing>) {
    let active = sim.shared.lock().ok().and_then(|s| s.field.clone());
    let pack = st.game.as_ref().and_then(|(id, result)| (Some(id) == active.as_ref()).then(|| result.as_ref().ok()).flatten());
    let revision = pack.map(|_| st.game_revision);
    if drawing.revision == revision {
        if let Some(handle) = &drawing.pending {
            if assets.is_loaded_with_dependencies(handle.id()) {
                for e in &old { commands.entity(e).despawn(); }
                commands.spawn((FieldVis, WorldAssetRoot(drawing.pending.take().unwrap())));
            } else if let Some(bevy::asset::LoadState::Failed(err)) = assets.get_load_state(handle.id()) {
                st.toast(format!("Could not load field CAD. Showing the game pack shapes. {err}"));
                drawing.pending = None;
            }
        }
        return;
    }
    drawing.revision = revision;
    drawing.pending = None;
    for e in &old { commands.entity(e).despawn(); }
    let Some(found) = pack else { return };
    let g = &found.pack;
    let path = match &found.dir {
        Some(dir) if dir.join("field.glb").is_file() => dir.file_name().map(|name| {
            let relative = std::path::Path::new(name).join("field.glb");
            bevy::asset::AssetPath::from(relative).with_source("gamepacks")
        }),
        None if matches!(g.id.as_str(), "reefscape-2025" | "rebuilt-2026") =>
            Some(bevy::asset::AssetPath::from(format!("embedded://fields/{}.glb", g.id))),
        _ => None,
    };
    if let Some(path) = path {
        if found.dir.is_some() { assets.reload(path.clone()); }
        drawing.pending = Some(assets.load(GltfAssetLabel::Scene(0).from_asset(path)));
    }
    // the carpet's markings: the field's outline
    let lines = material(&mut built, &mut mats, "#e8e8e8", 1.0);
    let carpet = material(&mut built, &mut mats, "#3a4048", 1.0);
    let c = commands.spawn((FieldVis, Mesh3d(meshes.add(Cuboid::new(g.field.length as f32, g.field.width as f32, 0.002))), MeshMaterial3d(carpet), Transform::from_xyz(0.0, 0.0, 0.0005))).id();
    commands.entity(*root).add_child(c);
    for s in &g.statics {
        let (mesh, fix) = match &s.shape {
            sim::game::StaticShape::Box { size } => (Mesh::from(Cuboid::new(size[0] as f32, size[1] as f32, size[2] as f32)), Quat::IDENTITY),
            sim::game::StaticShape::Cylinder { radius, length } => (Mesh::from(Cylinder::new(*radius as f32, *length as f32)), Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        };
        let q = model::math::euler_to_quat(s.euler_deg.map(f64::to_radians));
        let mut t = to_bevy(&model::Pose::new(s.pos, q));
        t.rotation *= fix;
        // markings without a colour are white lines; a coloured ghost (a BUMP the drivetrain drives
        // through) is drawn in its colour, see-through
        let m = if s.ghost && s.color.is_empty() { lines.clone() } else { material(&mut built, &mut mats, if s.color.is_empty() { "#777777" } else { &s.color }, if s.ghost { 0.5 } else { 1.0 }) };
        let e = commands.spawn((FieldVis, Mesh3d(meshes.add(mesh)), MeshMaterial3d(m), t)).id();
        commands.entity(*root).add_child(e);
    }
}

/// Poses everything from the latest snapshot.
fn pose(sim: Res<Sim>, st: Res<AppState>, mut links: Query<(&LinkVis, &mut Transform), (Without<RollerVis>, Without<PieceVis>)>,
    mut rollers: Query<(&RollerVis, &mut Transform, &Mesh3d, &mut TreadMesh, Option<&Children>), (Without<LinkVis>, Without<PieceVis>)>, mut pieces: Query<(&PieceVis, &mut Transform, &mut Visibility), (Without<LinkVis>, Without<RollerVis>)>,
    mut meshes: ResMut<Assets<Mesh>>, mut stripes: Query<&mut Visibility, (With<RollerStripe>, Without<PieceVis>)>,
    mut orbit: ResMut<Orbit>, mut last_view: Local<Option<(Workspace, bool, bool)>>) {
    let practice = st.ws == Workspace::Archetype && st.assembly.driving();
    let overview = practice && st.assembly.overview();
    let view = (st.ws, practice, overview);
    if *last_view != Some(view) {
        if overview {
            let length = st.game.as_ref().and_then(|g| g.1.as_ref().ok()).map_or(16.5, |g| g.pack.field.length);
            *orbit = Orbit { target: Vec3::ZERO, yaw: BLUE_STATION_ORBIT_YAW, pitch: 1.30, dist: length as f32 * 1.15 };
        } else if st.ws == Workspace::Field || practice {
            orbit.yaw = BLUE_STATION_ORBIT_YAW;
            orbit.pitch = if practice { 0.50 } else { 0.55 };
            orbit.dist = 4.5;
        } else if last_view.is_some_and(|(ws, field, _)| ws == Workspace::Field || field) {
            *orbit = Orbit::default();
        }
        *last_view = Some(view);
    }
    let Some(snap) = sim.snap() else { return };
    let lp: HashMap<Id, &model::Pose> = snap.links.iter().map(|l| (l.id, &l.pose)).collect();
    for (l, mut t) in &mut links {
        if let Some(p) = lp.get(&l.0) { *t = to_bevy(p); }
    }
    let mut loads = HashMap::<Id, HashMap<usize, (f64, f64)>>::new();
    for c in snap.contacts.iter().filter(|c| c.roller != 0 && c.force > 0.) {
        let load = loads.entry(c.roller).or_default().entry(c.piece).or_default();
        load.0 += c.force * c.tread_share;
        load.1 += c.force;
    }
    for (rv, mut t, mesh, mut tread, children) in &mut rollers {
        if let Some(rs) = snap.rollers.iter().find(|x| x.id == rv.0) {
            let axis = Vec3::new(rs.axis[0] as f32, rs.axis[1] as f32, rs.axis[2] as f32).normalize_or(Vec3::Y);
            let zero = Vec3::from_array(rs.zero.map(|x| x as f32));
            let rot = Quat::from_mat3(&Mat3::from_cols(zero, axis, zero.cross(axis))) * Quat::from_rotation_y(rs.angle as f32);
            *t = Transform { translation: Vec3::new(rs.center[0] as f32, rs.center[1] as f32, rs.center[2] as f32), rotation: rot, scale: Vec3::ONE };
            let mut touching = Vec::new();
            if let Some(contacts) = loads.get(&rv.0) {
                for piece in &snap.pieces {
                    let Some(&(weighted_share, load)) = contacts.get(&piece.index) else { continue };
                    let Some(spec) = st.lib.piece(&piece.kind) else { continue };
                    touching.push((sim::piece::PieceKind::from_library(spec), piece.pose.inverse(), weighted_share / load));
                }
            }
            if !touching.is_empty() || tread.compressed {
                let indent = |v: [f32; 3]| {
                    let rim = Vec3::from_array(v);
                    let hub = Vec3::Y * rim.y;
                    let radial = rim - hub;
                    let to_world = |p: Vec3| (t.translation + rot * p).as_dvec3().to_array();
                    let mut depth = 0.0_f64;
                    for (piece, inverse, share) in &touching {
                        let c = inverse.apply(to_world(hub));
                        let rpos = inverse.apply(to_world(rim));
                        let overlap = match piece.shape {
                            sim::piece::Shape::Tube { r, r_in, half } => piece.tube_overlap(c, rpos, r, r_in, half),
                            _ => -piece.sdf_local(rpos),
                        };
                        depth = depth.max(overlap * share);
                    }
                    (rim - radial.normalize_or_zero() * (depth as f32).min(radial.length())).to_array()
                };
                let shifts: Vec<_> = tread.samples.iter().map(|&v| Vec3::from_array(indent(v))-Vec3::from_array(v)).collect();
                let positions: Vec<_> = tread.rest.iter().zip(&tread.weights).map(|(&v, &(i, weight))| {
                    (Vec3::from_array(v) + shifts[i]*weight).to_array()
                }).collect();
                // Hide the rigid spin marker while its strip of tread passes under the pipe.
                let buried = indent([tread.radius, 0., 0.])[0] < tread.radius - 0.001;
                for child in children.into_iter().flat_map(|children| children.iter()) {
                    if let Ok(mut vis) = stripes.get_mut(child) { *vis = if buried { Visibility::Hidden } else { Visibility::Inherited }; }
                }
                if let Some(mut mesh) = meshes.get_mut(mesh) {
                    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
                    mesh.compute_normals();
                }
                tread.compressed = !touching.is_empty();
            }
        }
    }
    for (pv, mut t, mut vis) in &mut pieces {
        if let Some(p) = snap.pieces.get(pv.0) { *t = to_bevy(&p.pose); *vis = Visibility::Inherited; } else { *vis = Visibility::Hidden; }
    }
    if !overview && (st.view.follow || st.ws == Workspace::Field || practice) {
        if let Some(c) = lp.get(&CHASSIS) {
            let target = Vec3::new(c.pos[0] as f32, c.pos[2] as f32 + 0.4, -(c.pos[1] as f32));
            if practice {
                orbit.target = target + Vec3::new(0.42, 0.30, 0.0);
            } else { orbit.target = orbit.target.lerp(target, 0.15); }
        }
    }
}

/// Contacts, the centre of mass and the wheels' footprint, pickup zones, the selected joint's axis.
fn overlays(mut g: Gizmos, sim: Res<Sim>, st: Res<AppState>) {
    let Some(snap) = sim.snap() else { return };
    let to = |p: [f64; 3]| Vec3::new(p[0] as f32, p[2] as f32, -(p[1] as f32));
    if st.view.contacts {
        for c in &snap.contacts {
            let a = to(c.point);
            let n = Vec3::new(c.normal[0] as f32, c.normal[2] as f32, -(c.normal[1] as f32));
            let len = (c.force as f32 / 400.0).clamp(0.01, 0.25);
            g.line(a, a - n * len, if c.sliding { Color::srgb(1.0, 0.35, 0.2) } else { Color::srgb(0.2, 0.9, 0.4) });
        }
    }
    if st.view.com && !snap.support.is_empty() {
        let com = to(snap.com);
        let color = if snap.tip_margin > 0.05 { Color::srgb(0.3, 0.8, 1.0) } else { Color::srgb(1.0, 0.3, 0.2) };
        g.sphere(Isometry3d::from_translation(com), 0.03, color);
        let ground = Vec3::new(com.x, 0.005, com.z);
        g.line(com, ground, color);
        let pts: Vec<Vec3> = snap.support.iter().map(|p| Vec3::new(p[0] as f32, 0.005, -(p[1] as f32))).collect();
        for i in 0..pts.len() { g.line(pts[i], pts[(i + 1) % pts.len()], color); }
    }
    let r = &st.project().robot;
    let pose_of = |id: Id| snap.links.iter().find(|l| l.id == id).map(|l| l.pose);
    // beam breaks: green while clear, red while a piece crosses
    for x in &snap.sensors {
        if let Some([a, b]) = x.beam { g.line(to(a), to(b), if x.on { Color::srgb(0.95, 0.25, 0.2) } else { Color::srgb(0.3, 0.9, 0.4) }); }
    }
    if st.view.zones && matches!(st.ws, Workspace::Build | Workspace::Field) {
        for m in &r.mechanisms {
            if st.project().fidelity(m) == model::Fidelity::Full { continue; }
            let Some(p) = &m.pickup else { continue };
            let Some(lp) = pose_of(p.link) else { continue };
            let o = origin(r, p.link);
            let c = lp.apply([p.center[0] - o[0], p.center[1] - o[1], p.center[2] - o[2]]);
            let q = model::math::qmul(lp.quat, p.quat);
            let rot = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2) * Quat::from_xyzw(q[1] as f32, q[2] as f32, q[3] as f32, q[0] as f32);
            g.cube(Transform { translation: to(c), rotation: rot, scale: Vec3::new(p.size[0] as f32, p.size[1] as f32, p.size[2] as f32) }, Color::srgba(0.3, 1.0, 0.5, 0.8));
        }
    }
    if st.view.axes {
        let jid = match &st.sel { Selection::Joint(j) => Some(*j), Selection::Mechanism(m) => r.mechanism(*m).and_then(|m| m.links.iter().find_map(|l| r.joint_of(*l).filter(|j| j.kind != model::JointKind::Fixed).map(|j| j.id))), _ => None };
        if let Some(j) = jid.and_then(|j| r.joint(j)) {
            let parent = pose_of(j.parent).unwrap_or_default();
            let po = origin(r, j.parent);
            let a = parent.apply([j.anchor[0] - po[0], j.anchor[1] - po[1], j.anchor[2] - po[2]]);
            let d = parent.rotate(j.axis);
            let (a, d) = (to(a), Vec3::new(d[0] as f32, d[2] as f32, -(d[1] as f32)));
            g.line(a - d * 0.25, a + d * 0.25, Color::srgb(1.0, 0.85, 0.1));
            g.sphere(Isometry3d::from_translation(a), 0.015, Color::srgb(1.0, 0.85, 0.1));
        }
    }
}

/// Right-drag turns, middle-drag (or shift right-drag) pans, the wheel zooms.
fn orbit(st: Res<AppState>, mut orbit: ResMut<Orbit>, buttons: Res<ButtonInput<MouseButton>>, keys: Res<ButtonInput<KeyCode>>, motion: Res<bevy::input::mouse::AccumulatedMouseMotion>,
    scroll: Res<bevy::input::mouse::AccumulatedMouseScroll>, over: Res<PointerOverUi>, mut cam: Single<&mut Transform, With<MainCam>>) {
    let d = motion.delta;
    let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    let test_drive = st.ws == Workspace::Archetype && st.assembly.driving();
    if !over.0 {
        if !test_drive && buttons.pressed(MouseButton::Right) && !shift {
            orbit.yaw -= d.x * 0.006;
            orbit.pitch = (orbit.pitch + d.y * 0.006).clamp(-0.2, 1.45);
        }
        if !test_drive && (buttons.pressed(MouseButton::Middle) || (buttons.pressed(MouseButton::Right) && shift)) {
            let right = Vec3::new(orbit.yaw.cos(), 0.0, -orbit.yaw.sin());
            let fwd = Vec3::new(orbit.yaw.sin(), 0.0, orbit.yaw.cos());
            let k = orbit.dist * 0.0015;
            orbit.target += -right * d.x * k + fwd * d.y * k * 0.0 + Vec3::Y * d.y * k;
        }
        if scroll.delta.y != 0.0 { orbit.dist = (orbit.dist * (1.0 - scroll.delta.y * 0.1)).clamp(0.3, 40.0); }
    }
    let mut off = Vec3::new(orbit.yaw.sin() * orbit.pitch.cos(), orbit.pitch.sin(), orbit.yaw.cos() * orbit.pitch.cos()) * orbit.dist;
    if test_drive && st.assembly.field_view() && !st.assembly.overview() && let Some(game) = st.game.as_ref().and_then(|(_, g)| g.as_ref().ok()) {
        // Shorten only the horizontal boom, keeping the camera above the robot and on the
        // playing side of the glass. Scale both axes together to preserve the driver's heading.
        let field = &game.pack.field;
        let mut fraction = 1.0_f32;
        for (origin, delta, span) in [(orbit.target.x,off.x,field.length), (orbit.target.z,off.z,field.width)] {
            if delta.abs() > 1e-5 {
                let wall = delta.signum() * (span as f32 * 0.5 - 0.25);
                fraction = fraction.min(((wall-origin)/delta).clamp(0.,1.));
            }
        }
        off.x *= fraction;
        off.z *= fraction;
    }
    **cam = Transform::from_translation(orbit.target + off).looking_at(orbit.target, Vec3::Y);
}
