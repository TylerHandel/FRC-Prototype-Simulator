//! A wheel's foundation stiffness from its durometer and build (PLAN.md D4), until a press test
//! replaces it.
//!
//! The model: the tread is a bed of springs. Pressed `d` into a flat plate, a wheel of radius `R`
//! and width `w` pushes back `F = k_A * w * (4/3) * sqrt(2R) * d^1.5` (the springs' depths summed
//! over the contact arc). `k_A`, in Pa per m of squeeze, is what this estimates:
//!
//! - Durometer to Young's modulus by Gent's relation, `E = 0.0981 (56 + 7.62336 S) / (0.137505 (254 - 2.54 S))` MPa.
//! - A solid tread of thickness `t`: `k_A = E / t`.
//! - Builds that bend rather than squash get a factor: spoked compliant wheels and flex wheels give
//!   far more than their rubber's modulus says. The factors are judgement, chosen so a 4 in. 35A
//!   compliant wheel takes about 90 N at 6 mm of squeeze. They are the least certain numbers in the
//!   library and the first a press test should replace.
use crate::{Wheel, WheelBuild, IN};

/// Young's modulus (Pa) from Shore A hardness, by Gent's relation.
pub fn modulus_from_shore_a(s: f64) -> f64 {
    let s = s.clamp(10.0, 95.0);
    0.0981 * (56.0 + 7.62336 * s) / (0.137505 * (254.0 - 2.54 * s)) * 1e6
}

/// The estimate the library uses when a wheel has no measured `k_area`.
pub fn estimate(w: &Wheel) -> f64 {
    let e = modulus_from_shore_a(w.durometer_a);
    let r = w.diameter_in * IN / 2.0;
    // the depth of rubber that gives: the stated tread, or a share of the radius
    let t = w.tread_thickness_in.map(|t| t * IN).unwrap_or(match w.build {
        WheelBuild::Spoked | WheelBuild::Flex => 0.3 * r,
        _ => 0.15 * r,
    }).max(1e-3);
    let factor = match w.build {
        WheelBuild::Solid => 1.0,
        WheelBuild::Tube => 0.6,
        WheelBuild::Spoked => 0.12,
        WheelBuild::Flex => 0.2,
        WheelBuild::Rigid => 20.0,
        WheelBuild::Omni | WheelBuild::Mecanum => 1.5,
    };
    factor * e / t
}

/// Force (N) a wheel takes pressed `d` (m) into a flat plate, by the foundation model.
pub fn plate_force(k_area: f64, radius: f64, width: f64, d: f64) -> f64 {
    if d <= 0.0 { return 0.0; }
    k_area * width * (4.0 / 3.0) * (2.0 * radius).sqrt() * d.powf(1.5)
}

/// `k_area` from a press test: force `f` (N) at squeeze `d` (m) for a wheel of radius and width.
pub fn k_from_press(f: f64, d: f64, radius: f64, width: f64) -> f64 {
    f / (width * (4.0 / 3.0) * (2.0 * radius).sqrt() * d.powf(1.5))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Library;

    #[test]
    fn gent_values() {
        // about 1.4 MPa at 35A and 3.6 MPa at 60A
        assert!((modulus_from_shore_a(35.0) / 1e6 - 1.40).abs() < 0.05);
        assert!((modulus_from_shore_a(60.0) / 1e6 - 3.60).abs() < 0.1);
    }

    #[test]
    fn harder_is_stiffer_and_press_round_trips() {
        let l = Library::builtin();
        let soft = l.wheel("am-compliant-4in-35a").unwrap();
        let hard = l.wheel("am-compliant-4in-60a").unwrap();
        assert!(hard.k_area() > 2.0 * soft.k_area());
        let f = plate_force(soft.k_area(), soft.radius(), soft.width(), 0.006);
        assert!(f > 40.0 && f < 200.0, "4 in. 35A at 6 mm: {f} N");
        let k = k_from_press(f, 0.006, soft.radius(), soft.width());
        assert!((k / soft.k_area() - 1.0).abs() < 1e-9);
    }
}
