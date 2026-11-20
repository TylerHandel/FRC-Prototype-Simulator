//! The parts library: motors, wheels, materials, game pieces and swerve modules (assets/library).
//!
//! The JSON files are compiled in, so the app always has a library; a user's own additions are
//! loaded on top from their app data folder (`Library::load_overlay`). Every value carries where it
//! came from and whether it was measured.
pub mod units;
pub mod stiffness;

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

pub const IN: f64 = 0.0254;
pub const LB: f64 = 0.45359237;
pub const LBF: f64 = 4.4482216152605;
pub const PSI: f64 = 6894.757293168;
pub const RPM: f64 = std::f64::consts::TAU / 60.0;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Motor {
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub vendor: String,
    #[serde(default)]
    pub r#match: Vec<String>,
    pub free_rpm: f64,
    pub stall_torque: f64,
    pub stall_current: f64,
    pub free_current: f64,
    pub rotor_inertia: f64,
    #[serde(default)]
    pub source: String,
    #[serde(default)]
    pub measured: bool,
}

/// A motor's electrical constants at 12 V (from its curve, as rebuilt-sim-native drives.rs).
#[derive(Debug, Clone, Copy)]
pub struct MotorConstants {
    /// winding resistance, ohm
    pub r: f64,
    /// torque per amp, N*m/A
    pub kt: f64,
    /// speed per volt of back-EMF, rad/s/V
    pub kv: f64,
    /// free speed, rad/s
    pub free: f64,
    pub rotor_inertia: f64,
    pub free_current: f64,
    pub stall_current: f64,
}

impl Motor {
    pub fn constants(&self) -> MotorConstants {
        let v = 12.0;
        let r = v / self.stall_current;
        let free = self.free_rpm * RPM;
        MotorConstants {
            r,
            kt: self.stall_torque / self.stall_current,
            kv: free / (v - self.free_current * r),
            free,
            rotor_inertia: self.rotor_inertia,
            free_current: self.free_current,
            stall_current: self.stall_current,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum WheelBuild {
    /// a solid tread over a hub (urethane wheels, treads, Colson)
    Solid,
    /// spokes that bend (AndyMark compliant wheels)
    Spoked,
    /// a thin rim on curved spokes (WCP Flex)
    Flex,
    /// a tube over a rigid core
    Tube,
    /// no give of its own
    Rigid,
    Omni,
    Mecanum,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Wheel {
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub vendor: String,
    #[serde(default)]
    pub r#match: Vec<String>,
    pub diameter_in: f64,
    pub width_in: f64,
    pub durometer_a: f64,
    pub build: WheelBuild,
    #[serde(default)]
    pub tread_thickness_in: Option<f64>,
    pub tread: String,
    #[serde(default)]
    pub color: String,
    /// measured foundation stiffness, Pa per m of squeeze (from a press test); otherwise estimated
    #[serde(default)]
    pub k_area: Option<f64>,
    #[serde(default)]
    pub source: String,
    #[serde(default)]
    pub measured: bool,
}

impl Wheel {
    pub fn radius(&self) -> f64 { self.diameter_in * IN / 2.0 }
    pub fn width(&self) -> f64 { self.width_in * IN }
    /// foundation stiffness, Pa per m of squeeze: measured if a press test was entered, else from
    /// the durometer and build (`stiffness::estimate`)
    pub fn k_area(&self) -> f64 { self.k_area.unwrap_or_else(|| stiffness::estimate(self)) }
    pub fn stiffness_estimated(&self) -> bool { self.k_area.is_none() || !self.measured }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Material {
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub r#match: Vec<String>,
    pub density: f64,
    #[serde(default)]
    pub color: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Named { pub id: String, pub name: String }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrictionPair { pub tread: String, pub surface: String, pub r#static: f64, pub sliding: f64 }

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PieceShape {
    Sphere { d_in: f64 },
    Tube { outer_d_in: f64, inner_d_in: f64, length_in: f64 },
    Cylinder { d_in: f64, length_in: f64 },
    Box { x_in: f64, y_in: f64, z_in: f64 },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PieceLaw {
    /// no give of its own: whatever squeezes it gives instead
    Rigid,
    /// held up by air: contact pressure is the gauge pressure (one of `psi`, chosen by `choice`)
    Inflated { psi: Vec<f64>, choice: usize, skin_n_per_m: f64 },
    /// foam that stiffens as it is squeezed: plate force k0*xc*(e^(s/xc)-1) for a squeeze s
    Foam { k0_lb_per_in: f64, xc_in: f64 },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Piece {
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub game: String,
    pub shape: PieceShape,
    pub mass_kg: f64,
    pub surface: String,
    pub law: PieceLaw,
    #[serde(default)]
    pub color: String,
    #[serde(default)]
    pub source: String,
    #[serde(default)]
    pub measured: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Module {
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub r#match: Vec<String>,
    pub drive_ratio: f64,
    pub steer_ratio: f64,
    pub wheel: String,
}

#[derive(Debug, Clone, Default)]
pub struct Library {
    pub motors: Vec<Motor>,
    pub wheels: Vec<Wheel>,
    pub materials: Vec<Material>,
    pub treads: Vec<Named>,
    pub surfaces: Vec<Named>,
    pub friction: Vec<FrictionPair>,
    pub pieces: Vec<Piece>,
    pub modules: Vec<Module>,
}

#[derive(Deserialize)]
struct MotorsFile { motors: Vec<Motor> }
#[derive(Deserialize)]
struct WheelsFile { wheels: Vec<Wheel> }
#[derive(Deserialize)]
struct MaterialsFile { materials: Vec<Material>, treads: Vec<Named>, surfaces: Vec<Named>, friction: Vec<FrictionPair> }
#[derive(Deserialize)]
struct PiecesFile { pieces: Vec<Piece> }
#[derive(Deserialize)]
struct ModulesFile { modules: Vec<Module> }

const MOTORS: &str = include_str!("../../../assets/library/motors.json");
const WHEELS: &str = include_str!("../../../assets/library/wheels.json");
const MATERIALS: &str = include_str!("../../../assets/library/materials.json");
const PIECES: &str = include_str!("../../../assets/library/pieces.json");
const MODULES: &str = include_str!("../../../assets/library/modules.json");

impl Library {
    /// The library as shipped.
    pub fn builtin() -> Self {
        let m: MaterialsFile = serde_json::from_str(MATERIALS).expect("materials.json");
        Library {
            motors: serde_json::from_str::<MotorsFile>(MOTORS).expect("motors.json").motors,
            wheels: serde_json::from_str::<WheelsFile>(WHEELS).expect("wheels.json").wheels,
            materials: m.materials,
            treads: m.treads,
            surfaces: m.surfaces,
            friction: m.friction,
            pieces: serde_json::from_str::<PiecesFile>(PIECES).expect("pieces.json").pieces,
            modules: serde_json::from_str::<ModulesFile>(MODULES).expect("modules.json").modules,
        }
    }

    /// Adds or replaces entries from a user's own library folder (files named as the built-in ones,
    /// holding only what they add or change). Missing files are fine.
    pub fn load_overlay(&mut self, dir: &std::path::Path) -> anyhow::Result<()> {
        let read = |name: &str| std::fs::read_to_string(dir.join(name)).ok();
        if let Some(s) = read("wheels.json") {
            for w in serde_json::from_str::<WheelsFile>(&s)?.wheels { upsert(&mut self.wheels, w, |w| &w.id); }
        }
        if let Some(s) = read("motors.json") {
            for m in serde_json::from_str::<MotorsFile>(&s)?.motors { upsert(&mut self.motors, m, |m| &m.id); }
        }
        if let Some(s) = read("pieces.json") {
            for p in serde_json::from_str::<PiecesFile>(&s)?.pieces { upsert(&mut self.pieces, p, |p| &p.id); }
        }
        Ok(())
    }

    /// Writes a user's wheels (those not in the built-in library, or changed) to their overlay.
    pub fn save_wheel_overlay(&self, dir: &std::path::Path) -> anyhow::Result<()> {
        let builtin = Library::builtin();
        let mine: Vec<&Wheel> = self.wheels.iter()
            .filter(|w| builtin.wheel(&w.id).map_or(true, |b| serde_json::to_string(b).ok() != serde_json::to_string(*w).ok()))
            .collect();
        std::fs::create_dir_all(dir)?;
        std::fs::write(dir.join("wheels.json"), serde_json::to_string_pretty(&serde_json::json!({ "wheels": mine }))?)?;
        Ok(())
    }

    pub fn motor(&self, id: &str) -> Option<&Motor> { self.motors.iter().find(|m| m.id == id) }
    pub fn wheel(&self, id: &str) -> Option<&Wheel> { self.wheels.iter().find(|w| w.id == id) }
    pub fn piece(&self, id: &str) -> Option<&Piece> { self.pieces.iter().find(|p| p.id == id) }
    pub fn module(&self, id: &str) -> Option<&Module> { self.modules.iter().find(|m| m.id == id) }
    pub fn material(&self, id: &str) -> Option<&Material> { self.materials.iter().find(|m| m.id == id) }

    /// Friction (static, sliding) of a tread against a surface; a middling estimate when no pair is listed.
    pub fn friction(&self, tread: &str, surface: &str) -> (f64, f64) {
        self.friction.iter().find(|f| f.tread == tread && f.surface == surface)
            .map_or((0.8, 0.65), |f| (f.r#static, f.sliding))
    }

    /// The library motor a BOM or part name means, if any (the longest pattern that matches wins).
    pub fn match_motor(&self, name: &str) -> Option<&Motor> { best_match(name, &self.motors, |m| &m.r#match) }
    /// The library wheel a BOM or part name means, if any.
    pub fn match_wheel(&self, name: &str) -> Option<&Wheel> { best_match(name, &self.wheels, |w| &w.r#match) }
    pub fn match_material(&self, name: &str) -> Option<&Material> { best_match(name, &self.materials, |m| &m.r#match) }
    pub fn match_module(&self, name: &str) -> Option<&Module> { best_match(name, &self.modules, |m| &m.r#match) }

    /// Wheels sorted for a picker: same diameter first, then by family and durometer.
    pub fn wheels_like(&self, diameter_in: f64) -> Vec<&Wheel> {
        let mut v: Vec<&Wheel> = self.wheels.iter().collect();
        v.sort_by(|a, b| {
            let da = (a.diameter_in - diameter_in).abs();
            let db = (b.diameter_in - diameter_in).abs();
            da.partial_cmp(&db).unwrap().then(a.name.cmp(&b.name))
        });
        v
    }
}

fn upsert<T>(v: &mut Vec<T>, item: T, id: impl Fn(&T) -> &String) {
    if let Some(i) = v.iter().position(|x| id(x) == id(&item)) { v[i] = item; } else { v.push(item); }
}

fn best_match<'a, T>(name: &str, items: &'a [T], pats: impl Fn(&T) -> &Vec<String>) -> Option<&'a T> {
    let lower = name.to_lowercase();
    let mut best: Option<(&T, usize)> = None;
    let mut cache: HashMap<&str, regex::Regex> = HashMap::new();
    for it in items {
        for p in pats(it) {
            let re = cache.entry(p.as_str()).or_insert_with(|| regex::Regex::new(p).unwrap_or_else(|_| regex::Regex::new("$^").unwrap()));
            if let Some(m) = re.find(&lower) {
                let score = m.as_str().len() * 1000 + p.len();
                if best.is_none_or(|(_, s)| score > s) { best = Some((it, score)); }
            }
        }
    }
    best.map(|(t, _)| t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builtin_loads() {
        let l = Library::builtin();
        assert!(l.motors.len() >= 8 && l.wheels.len() >= 20 && l.pieces.len() >= 3);
        for w in &l.wheels { assert!(w.k_area() > 0.0, "{}", w.id); }
    }

    #[test]
    fn matches_bom_names() {
        let l = Library::builtin();
        assert_eq!(l.match_motor("Kraken X60 powered by Talon FX (WCP-0940)").unwrap().id, "kraken-x60-foc");
        assert_eq!(l.match_motor("NEO 550 Brushless").unwrap().id, "neo-550");
        assert_eq!(l.match_motor("REV NEO Brushless Motor").unwrap().id, "neo");
        assert_eq!(l.match_wheel("4\" OD x 1\" Wide Straight Flex Wheel (1-1/4\" Round Stretch, 45A) (WCP-1362)").unwrap().id, "wcp-flex-4in-45a");
        assert_eq!(l.match_wheel("AM-3562 3/8\" Hex Bore 4\" Compliant Wheel").unwrap().id, "am-compliant-4in-50a");
        assert_eq!(l.match_wheel("1\" Solid Urethane Wheel (45A, 1\" Wide, 1/2\" Hex Bore)").unwrap().id, "solid-urethane-1in-45a");
        assert!(l.match_wheel("Socket head cap screw #10-32").is_none());
    }

    #[test]
    fn kraken_constants() {
        let c = Library::builtin().motor("kraken-x60").unwrap().constants();
        assert!((c.r - 12.0 / 366.0).abs() < 1e-12);
        assert!((c.free - 6000.0 * RPM).abs() < 1e-9);
    }
}
