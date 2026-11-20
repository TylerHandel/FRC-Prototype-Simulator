//! A traction-limited drivetrain (PLAN.md M2): each wheel's contact solved against the whole
//! robot's planar mass and yaw inertia, within its motor's current-limited torque and the carpet's
//! friction circle, then applied to the chassis as forces at the wheels' contact points (so a high
//! centre of mass pitches under hard acceleration, D17).
//!
//! From rebuilt-sim-native crates/sim/src/drivetrain.rs (SwerveDrive::step) at da6b231, rewritten
//! for this engine and for tank drive (wheels that can't steer).
use crate::motor::Motor;
use model::DriveKind;

const STEER_ACCEL: f64 = 1500.0;
const STEER_ROLL: f64 = 0.015;
const STEER_FRICTION: f64 = 2.0;
const G: f64 = 9.81;

#[derive(Debug, Clone, Copy, Default)]
pub struct Module { pub angle: f64, pub rate: f64, pub w: f64, pub supply: f64, pub slip: bool, pub down: bool }

#[derive(Debug, Clone)]
pub struct Drive {
    pub kind: DriveKind,
    /// module positions, robot frame (x, y)
    pub pts: Vec<[f64; 2]>,
    pub motor: Motor,
    pub wheel_r: f64,
    pub mu: f64,
    pub steer_rate: f64,
    pub modules: Vec<Module>,
    pub max_speed: f64,
    pub max_turn: f64,
}

/// The robot's planar state, world frame (x, y, yaw), and its mass.
#[derive(Debug, Clone, Copy)]
pub struct Body { pub yaw: f64, pub vel: [f64; 2], pub omega: f64, pub mass: f64, pub iz: f64, pub com: [f64; 2], pub origin: [f64; 2] }

/// A wheel's push on the carpet this step: where (robot frame) and how hard (world, N).
#[derive(Debug, Clone, Copy)]
pub struct Push { pub at: [f64; 2], pub force: [f64; 2] }

fn wrap(a: f64) -> f64 { a.sin().atan2(a.cos()) }

struct K { u: [f64; 2], ru: f64, m: f64, lam: f64 }
struct Ct { i: usize, r: [f64; 2], k: [K; 2], want: f64, lo: f64, hi: f64, lim: f64, slip: bool }

impl Drive {
    pub fn new(kind: DriveKind, pts: Vec<[f64; 2]>, motor: Motor, wheel_r: f64, mu: f64, max_speed: f64, max_turn: f64) -> Self {
        let n = pts.len();
        let steer_rate = 0.85 * (6000.0 * std::f64::consts::TAU / 60.0) / (150.0 / 7.0);
        let free = motor.free_speed() * wheel_r;
        Drive { kind, pts, motor, wheel_r, mu, steer_rate, modules: vec![Module::default(); n], max_speed: max_speed.min(free), max_turn, }
    }

    /// One step: robot-frame speed (vx, vy) and turn wanted; which wheels touch; the bus voltage.
    /// Returns the wheels' pushes and the battery current.
    pub fn step(&mut self, b: Body, cmd: [f64; 3], down: &[bool], bus: f64, dt: f64) -> (Vec<Push>, f64) {
        let rw = self.wheel_r;
        let n = self.pts.len();
        let (c, s) = (b.yaw.cos(), b.yaw.sin());
        let to_world = |x: f64, y: f64| [x * c - y * s, x * s + y * c];
        let (rvx, rvy, omega) = (cmd[0].clamp(-self.max_speed, self.max_speed), cmd[1].clamp(-self.max_speed, self.max_speed), cmd[2].clamp(-self.max_turn, self.max_turn));
        // desaturate: no wheel asked past top speed
        let mut want_sp = vec![0.0; n];
        let mut worst: f64 = 1.0;
        for i in 0..n {
            let [px, py] = self.pts[i];
            let (ux, uy) = (rvx - omega * py, rvy + omega * px);
            worst = worst.max((ux * ux + uy * uy).sqrt() / self.max_speed.max(1e-6));
        }
        for i in 0..n {
            let [px, py] = self.pts[i];
            let (ux, uy) = ((rvx - omega * py) / worst, (rvy + omega * px) / worst);
            let md = &mut self.modules[i];
            match self.kind {
                DriveKind::Tank => { md.angle = 0.0; want_sp[i] = ux; }
                _ => {
                    let mut sp = (ux * ux + uy * uy).sqrt();
                    md.angle = wrap(md.angle);
                    let target = if sp > 0.02 { uy.atan2(ux) } else { md.angle };
                    let mut err = wrap(target - md.angle);
                    if err.abs() > std::f64::consts::FRAC_PI_2 { err = wrap(err + std::f64::consts::PI); sp = -sp; }
                    let goal = md.angle + err;
                    let stop_rate = (2.0 * STEER_ACCEL * err.abs()).sqrt() * err.signum();
                    let rate = stop_rate.clamp(-self.steer_rate, self.steer_rate);
                    md.rate += (rate - md.rate).clamp(-STEER_ACCEL * dt, STEER_ACCEL * dt);
                    let turn = md.rate * dt;
                    if turn.abs() >= err.abs() { md.angle = goal; md.rate = 0.0; } else { md.angle += turn; }
                    want_sp[i] = sp * (goal - md.angle).cos().max(0.0);
                }
            }
        }
        // the fastest a wheel can turn on this bus: a slipping wheel's modelled spin is kept below it,
        // or the motor would brake against a spin it could never have reached
        let w_free = 0.98 * self.motor.c.kv * bus.max(1.0) / self.motor.ratio;
        let n_down = down.iter().filter(|x| **x).count().max(1) as f64;
        let mut v = b.vel;
        let mut wz = b.omega;
        let mut cts: Vec<Ct> = Vec::new();
        for i in 0..n {
            self.modules[i].down = down[i];
            if !down[i] { self.modules[i].slip = false; self.modules[i].supply = 0.0; continue; }
            let [px, py] = self.pts[i];
            let pw = to_world(px, py);
            let r = [b.origin[0] + pw[0] - b.com[0], b.origin[1] + pw[1] - b.com[1]];
            let ang = self.modules[i].angle;
            let l = to_world(ang.cos(), ang.sin());
            let mk = |ux: f64, uy: f64| { let ru = r[0] * uy - r[1] * ux; K { u: [ux, uy], ru, m: 1.0 / (1.0 / b.mass + ru * ru / b.iz), lam: 0.0 } };
            let k = [mk(l[0], l[1]), mk(-l[1], l[0])];
            let w = self.modules[i].w;
            let fwd = self.motor.torque(w, bus, bus).torque * dt / rw;
            let back = self.motor.torque(w, -bus, bus).torque * dt / rw;
            let drag = ((STEER_ROLL * b.mass * G + STEER_FRICTION * (w * rw).abs()) / n as f64) * dt;
            cts.push(Ct { i, r, k, want: want_sp[i], lo: back - drag, hi: fwd + drag, lim: self.mu * b.mass * G * dt / n_down, slip: false });
        }
        for _ in 0..10 {
            for ct in &mut cts {
                let vpx = v[0] - wz * ct.r[1];
                let vpy = v[1] + wz * ct.r[0];
                let (a, bb) = (&ct.k[0], &ct.k[1]);
                let mut l1 = a.lam - (vpx * a.u[0] + vpy * a.u[1] - ct.want) * a.m;
                let mut l2 = bb.lam - (vpx * bb.u[0] + vpy * bb.u[1]) * bb.m;
                l1 = l1.clamp(ct.lo, ct.hi);
                let mag = (l1 * l1 + l2 * l2).sqrt();
                ct.slip = mag > ct.lim;
                if ct.slip { l1 *= ct.lim / mag; l2 *= ct.lim / mag; }
                for (kk, l) in ct.k.iter_mut().zip([l1, l2]) {
                    let dl = l - kk.lam;
                    kk.lam = l;
                    v = [v[0] + kk.u[0] * dl / b.mass, v[1] + kk.u[1] * dl / b.mass];
                    wz += kk.ru * dl / b.iz;
                }
            }
        }
        let mut pushes = Vec::new();
        let mut isum = 0.0;
        for ct in &cts {
            let a = &ct.k[0];
            let ground = (v[0] - wz * ct.r[1]) * a.u[0] + (v[1] + wz * ct.r[0]) * a.u[1];
            let tau = a.lam * rw / dt;
            let w = ((if ct.slip && ct.want.abs() > ground.abs() { ground + 0.5 * (ct.want - ground) } else { ground }) / rw).clamp(-w_free, w_free);
            // battery current for the torque it gave at the speed it turned
            let i_stator = tau / (self.motor.count * self.motor.c.kt * self.motor.ratio * self.motor.eff).max(1e-9);
            let eb = w * self.motor.ratio / self.motor.c.kv;
            let sup = (i_stator * (eb + i_stator / self.motor.count * self.motor.c.r) / bus.max(1.0)).abs();
            let md = &mut self.modules[ct.i];
            md.w = w; md.slip = ct.slip; md.supply = sup;
            isum += sup;
            let f = [(ct.k[0].u[0] * ct.k[0].lam + ct.k[1].u[0] * ct.k[1].lam) / dt, (ct.k[0].u[1] * ct.k[0].lam + ct.k[1].u[1] * ct.k[1].lam) / dt];
            pushes.push(Push { at: self.pts[ct.i], force: f });
        }
        (pushes, isum)
    }
}
