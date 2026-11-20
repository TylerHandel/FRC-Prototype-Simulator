//! Controls without code (PLAN.md D12): what volts each drive gets, from its controller block.
//!
//! - Voltage: the volts asked.
//! - Velocity: feedforward plus a proportional and integral term on speed.
//! - Position: PID with static, gravity and velocity feedforward, through a trapezoid profile when
//!   cruise and accel are set.
//! - Ideal: computed torque along the fastest profile the motor allows, using the model's own
//!   inertia and gravity (Concept: "what the mechanism can do", not "what our code will do").
use crate::motor::Motor;
use model::{Control, JointKind};

#[derive(Debug, Clone, Copy, Default)]
pub struct State {
    pub integral: f64,
    pub prof_pos: f64,
    pub prof_vel: f64,
    pub started: bool,
}

/// What the plant looks like to a joint's controller this step.
#[derive(Debug, Clone, Copy)]
pub struct Plant {
    pub kind: JointKind,
    /// position and speed at the output (rad or m)
    pub q: f64,
    pub qd: f64,
    /// effective inertia at the output (motor rotors included)
    pub inertia: f64,
    /// the force gravity and motion put on the joint (what the motor must supply to hold still)
    pub bias: f64,
    pub range: Option<[f64; 2]>,
    pub bus: f64,
}

/// One step of a trapezoid profile from (p, v) toward `goal`.
pub fn profile_step(p: f64, v: f64, goal: f64, vmax: f64, amax: f64, dt: f64) -> (f64, f64) {
    let e = goal - p;
    // the speed from which we can just stop at the goal
    let stop = (2.0 * amax * e.abs()).sqrt() * e.signum();
    let want = stop.clamp(-vmax, vmax);
    let dv = (want - v).clamp(-amax * dt, amax * dt);
    let v2 = v + dv;
    let mut p2 = p + 0.5 * (v + v2) * dt;
    // don't step past the goal
    if (goal - p2).signum() != e.signum() && e != 0.0 { p2 = goal; return (p2, 0.0); }
    (p2, v2)
}

/// Volts for a joint drive.
pub fn joint_volts(c: &Control, st: &mut State, target: f64, pl: &Plant, m: &Motor, dt: f64) -> f64 {
    let target = match pl.range { Some([lo, hi]) => target.clamp(lo, hi), None => target };
    let gain = m.gain().max(1e-9);
    match *c {
        Control::Voltage => target,
        Control::Velocity { kp, ki, ks, kv } => {
            st.integral = (st.integral + (target - pl.qd) * dt).clamp(-10.0, 10.0);
            kv * target + ks * target.signum() + kp * (target - pl.qd) + ki * st.integral
        }
        Control::Position { kp, ki, kd, ks, kg, kv, cruise, accel } => {
            if !st.started { st.prof_pos = pl.q; st.prof_vel = pl.qd; st.started = true; }
            let (sp, sv) = if cruise > 0.0 && accel > 0.0 {
                let (p, v) = profile_step(st.prof_pos, st.prof_vel, target, cruise, accel, dt);
                st.prof_pos = p; st.prof_vel = v;
                (p, v)
            } else { st.prof_pos = target; st.prof_vel = 0.0; (target, 0.0) };
            let e = sp - pl.q;
            st.integral = (st.integral + e * dt).clamp(-1.0, 1.0);
            let grav = match pl.kind { JointKind::Hinge => kg * pl.q.cos(), _ => kg };
            kp * e + ki * st.integral + kd * (sv - pl.qd) + ks * sv.signum() * (sv.abs() > 1e-4) as i32 as f64 + grav + kv * sv
        }
        Control::Ideal => {
            if !st.started { st.prof_pos = pl.q; st.prof_vel = pl.qd; st.started = true; }
            let bus = pl.bus.max(1.0);
            let vmax = 0.85 * m.free_speed() * bus / 12.0;
            // what the current limit can do against gravity, kept in hand for braking
            let amax = ((m.max_torque() - pl.bias.abs()).max(0.1 * m.max_torque()) / pl.inertia.max(1e-6)) * 0.7;
            let (p, v) = profile_step(st.prof_pos, st.prof_vel, target, vmax, amax, dt);
            let a = (v - st.prof_vel) / dt;
            st.prof_pos = p; st.prof_vel = v;
            // computed torque: inertia times the wanted acceleration plus gravity, with stiff feedback
            let wn = 40.0;
            let acc = a + 2.0 * wn * (v - pl.qd) + wn * wn * (p - pl.q);
            let force = pl.inertia * acc + pl.bias;
            (force + m.slope() * pl.qd) / gain
        }
    }
}

/// Volts for a roller shaft's drive (speed in rad/s at the shaft).
pub fn shaft_volts(c: &Control, st: &mut State, value: f64, omega: f64, dt: f64, m: &Motor) -> f64 {
    match *c {
        Control::Velocity { kp, ki, ks, kv } => {
            if value == 0.0 { st.integral = 0.0; return 0.0; }
            st.integral = (st.integral + (value - omega) * dt).clamp(-10.0, 10.0);
            let kv = if kv == 0.0 { m.back_emf(1.0) } else { kv };
            kv * value + ks * value.signum() + kp * (value - omega) + ki * st.integral
        }
        _ => value,
    }
}

/// Gains from the model (D12's button): kV and kG from the motor and the load, kP and kD for a
/// well-damped response at about 2.5 Hz, a profile at 80 % of free speed and half the stall-limited
/// acceleration.
pub fn gains_from_model(pl: &Plant, m: &Motor) -> Control {
    let gain = m.gain().max(1e-9);
    let kv = m.slope() / gain;
    let kg = match pl.kind { JointKind::Hinge => pl.bias.abs() / gain / pl.q.cos().abs().max(0.2), _ => pl.bias.abs() / gain };
    let wn = 15.0;
    let kp = pl.inertia * wn * wn / gain;
    let kd = (2.0 * 0.8 * wn * pl.inertia / gain - kv).max(0.0);
    let cruise = 0.8 * m.free_speed();
    let accel = 0.5 * (m.max_torque() - pl.bias.abs()).max(0.1 * m.max_torque()) / pl.inertia.max(1e-6);
    Control::Position { kp, ki: 0.0, kd, ks: 0.1, kg, kv, cruise, accel }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn profile_reaches_goal_without_overshoot() {
        let (mut p, mut v) = (0.0, 0.0);
        let mut peak: f64 = 0.0;
        for _ in 0..5000 { (p, v) = profile_step(p, v, 1.0, 2.0, 4.0, 0.001); peak = peak.max(p); }
        assert!((p - 1.0).abs() < 1e-9 && v.abs() < 1e-9 && peak <= 1.0 + 1e-9);
    }
}
