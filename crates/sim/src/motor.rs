//! DC motors behind a gearbox, with stator and supply current limits. From rebuilt-sim-native
//! crates/sim/src/mech/drives.rs (Motor) at da6b231, without the JavaScript parity rules.
use library::MotorConstants;

#[derive(Debug, Clone)]
pub struct Motor {
    pub c: MotorConstants,
    pub count: f64,
    /// motor turns per output turn (or per output radian of a spool: see `compile`)
    pub ratio: f64,
    pub eff: f64,
    pub stator_limit: f64,
    pub supply_limit: f64,
}

/// What a motor did this step.
#[derive(Debug, Clone, Copy, Default)]
pub struct MotorOut {
    /// output torque (or force), after the gearbox
    pub torque: f64,
    /// stator current, all motors together, A
    pub current: f64,
    /// battery current, all motors together, A
    pub supply: f64,
    /// the volts actually applied (after limits)
    pub volts: f64,
    pub limited: bool,
}

impl Motor {
    pub fn new(c: MotorConstants, count: f64, ratio: f64, eff: f64, stator_limit: f64, supply_limit: f64) -> Self {
        Motor { c, count: count.max(1.0), ratio, eff, stator_limit, supply_limit }
    }
    /// Torque at output speed `w` with `volts` asked and the bus at `bus` volts. The supply limit
    /// is solved as in drives.rs (a quadratic in the current).
    pub fn torque(&self, w: f64, volts: f64, bus: f64) -> MotorOut {
        let v = volts.clamp(-bus, bus);
        let eb = w * self.ratio / self.c.kv;
        let i = (v - eb) / self.c.r;
        // published constants rarely have kt exactly 1/kv (the Kraken X60's are 1.4 % over), so
        // the battery pays for the work at whichever is bigger: no energy from nowhere
        let ew = w * self.ratio * self.c.kt.max(1.0 / self.c.kv);
        let e = i.signum() * ew;
        let isup = self.supply_limit * bus;
        let cap = if isup.is_finite() { (-e + (e * e + 4.0 * self.c.r * isup).sqrt()) / (2.0 * self.c.r) } else { f64::INFINITY };
        let cap = cap.min(self.stator_limit);
        let limited = i.abs() > cap;
        let ic = if limited { i.signum() * cap } else { i };
        let applied = eb + ic * self.c.r;
        let p = if ic * w > 0.0 { ic * ew + ic * ic * self.c.r } else { (ic * applied).abs() };
        let supply = if bus > 0.0 { p.abs() / bus } else { 0.0 };
        MotorOut { torque: self.count * self.c.kt * ic * self.ratio * self.eff, current: ic.abs() * self.count, supply: supply * self.count, volts: applied, limited }
    }
    /// output torque per volt, below the limits
    pub fn gain(&self) -> f64 { self.count * self.c.kt * self.ratio * self.eff / self.c.r }
    /// back-EMF damping at the output, torque per (rad/s)
    pub fn slope(&self) -> f64 { self.count * self.c.kt * self.ratio * self.ratio * self.eff / (self.c.kv * self.c.r) }
    /// rotor inertia seen at the output
    pub fn reflected_inertia(&self) -> f64 { self.count * self.c.rotor_inertia * self.ratio * self.ratio }
    /// free speed at the output, rad/s at 12 V
    pub fn free_speed(&self) -> f64 { self.c.free / self.ratio }
    /// the volts that hold the output at speed `w` with no load (back-EMF)
    pub fn back_emf(&self, w: f64) -> f64 { w * self.ratio / self.c.kv }
    /// the most torque it can give, current-limited, at stall
    pub fn max_torque(&self) -> f64 { self.count * self.c.kt * self.stator_limit.min(self.c.stall_current) * self.ratio * self.eff }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn curve_points() {
        let lib = library::Library::builtin();
        let m = Motor::new(lib.motor("kraken-x60").unwrap().constants(), 1.0, 1.0, 1.0, f64::INFINITY, f64::INFINITY);
        // stall: the published torque and current
        let s = m.torque(0.0, 12.0, 12.0);
        assert!((s.torque - 7.09).abs() < 1e-9 && (s.current - 366.0).abs() < 1e-6);
        // free: no torque but the free current's worth
        let f = m.torque(m.c.free, 12.0, 12.0);
        assert!(f.torque.abs() < 0.04 && (f.current - 2.0).abs() < 1e-6, "{f:?}");
        // a 40 A stator limit at stall
        let l = Motor { stator_limit: 40.0, ..m.clone() }.torque(0.0, 12.0, 12.0);
        assert!(l.limited && (l.current - 40.0).abs() < 1e-9);
        // a supply limit holds battery current
        let sup = Motor { supply_limit: 30.0, ..m.clone() }.torque(m.c.free * 0.5, 12.0, 12.0);
        assert!((sup.supply - 30.0).abs() < 1e-6, "{sup:?}");
    }
}

#[cfg(test)]
mod energy {
    use super::*;

    /// M1 gate, energy in against energy out: for every motor in the library, at any speed and
    /// voltage while it drives a load, the battery gives at least the work done plus the copper's
    /// heat, and no more than that plus what the motor's own losses (free current) can explain.
    #[test]
    fn power_in_covers_power_out() {
        let lib = library::Library::builtin();
        for m in &lib.motors {
            let c = m.constants();
            let mo = Motor::new(c.clone(), 2.0, 10.0, 1.0, f64::INFINITY, f64::INFINITY);
            for k in 0..=20 {
                let w = mo.free_speed() * k as f64 / 20.0;
                for v in [3.0, 6.0, 9.0, 12.0] {
                    let o = mo.torque(w, v, 12.0);
                    if o.torque <= 0.0 { continue; }
                    let p_in = o.supply * 12.0;
                    let p_out = o.torque * w;
                    let heat = (o.current / mo.count).powi(2) * c.r * mo.count;
                    assert!(p_in + 1e-9 >= p_out + heat, "{}: {p_in} W in, {p_out} W out + {heat} W heat at {w} rad/s, {v} V", m.id);
                    // what's left over is the gap between kt and 1/kv: the free-running losses
                    let spare = p_in - p_out - heat;
                    assert!(spare <= (o.current / mo.count) * mo.count * mo.back_emf(w) * 0.25 + 1e-6, "{}: {spare} W unaccounted", m.id);
                }
            }
        }
    }
}
