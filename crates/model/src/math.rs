//! Small vector and rotation helpers on plain arrays, so the schema stays readable JSON.
use serde::{Deserialize, Serialize};

pub type V3 = [f64; 3];
/// w, x, y, z
pub type Quat = [f64; 4];

pub const QI: Quat = [1.0, 0.0, 0.0, 0.0];

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Pose {
    pub pos: V3,
    #[serde(default = "qi")]
    pub quat: Quat,
}

pub(crate) fn qi() -> Quat { QI }

impl Default for Pose { fn default() -> Self { Pose { pos: [0.0; 3], quat: QI } } }

impl Pose {
    pub fn at(pos: V3) -> Self { Pose { pos, quat: QI } }
    pub fn new(pos: V3, quat: Quat) -> Self { Pose { pos, quat } }
    /// a pose turned so its z axis lies along `axis`
    pub fn along(pos: V3, axis: V3) -> Self { Pose { pos, quat: quat_from_to([0.0, 0.0, 1.0], norm(axis)) } }
    pub fn apply(&self, p: V3) -> V3 { add(qrot(self.quat, p), self.pos) }
    pub fn rotate(&self, v: V3) -> V3 { qrot(self.quat, v) }
    pub fn inverse(&self) -> Pose { let qi = qconj(self.quat); Pose { pos: neg(qrot(qi, self.pos)), quat: qi } }
    /// self then other (other expressed in self's frame)
    pub fn mul(&self, o: &Pose) -> Pose { Pose { pos: self.apply(o.pos), quat: qmul(self.quat, o.quat) } }
    pub fn euler_deg(&self) -> V3 { let e = quat_to_euler(self.quat); [e[0].to_degrees(), e[1].to_degrees(), e[2].to_degrees()] }
    pub fn set_euler_deg(&mut self, e: V3) { self.quat = euler_to_quat([e[0].to_radians(), e[1].to_radians(), e[2].to_radians()]); }
    /// 4x4 column-major
    pub fn matrix(&self) -> [f64; 16] {
        let m = qmat(self.quat);
        [m[0], m[3], m[6], 0.0, m[1], m[4], m[7], 0.0, m[2], m[5], m[8], 0.0, self.pos[0], self.pos[1], self.pos[2], 1.0]
    }
    /// from a 4x4 row-major matrix (Onshape, cadread)
    pub fn from_rows(m: &[f64]) -> Pose {
        let r = [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]];
        Pose { pos: [m[3], m[7], m[11]], quat: mat_to_quat(r) }
    }
}

pub fn add(a: V3, b: V3) -> V3 { [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
pub fn sub(a: V3, b: V3) -> V3 { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
pub fn scale(a: V3, s: f64) -> V3 { [a[0] * s, a[1] * s, a[2] * s] }
pub fn neg(a: V3) -> V3 { [-a[0], -a[1], -a[2]] }
pub fn dot(a: V3, b: V3) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
pub fn cross(a: V3, b: V3) -> V3 { [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]] }
pub fn len(a: V3) -> f64 { dot(a, a).sqrt() }
pub fn norm(a: V3) -> V3 { let l = len(a); if l < 1e-15 { [0.0, 0.0, 1.0] } else { scale(a, 1.0 / l) } }
pub fn lerp(a: V3, b: V3, t: f64) -> V3 { add(a, scale(sub(b, a), t)) }

pub fn qmul(a: Quat, b: Quat) -> Quat {
    [a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
     a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
     a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
     a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]]
}
pub fn qconj(q: Quat) -> Quat { [q[0], -q[1], -q[2], -q[3]] }
pub fn qnorm(q: Quat) -> Quat { let l = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt(); if l < 1e-15 { QI } else { [q[0] / l, q[1] / l, q[2] / l, q[3] / l] } }
pub fn qrot(q: Quat, v: V3) -> V3 {
    let u = [q[1], q[2], q[3]];
    let t = scale(cross(u, v), 2.0);
    add(add(v, scale(t, q[0])), cross(u, t))
}
pub fn axis_angle(axis: V3, a: f64) -> Quat {
    let n = norm(axis);
    let (s, c) = (a / 2.0).sin_cos();
    [c, n[0] * s, n[1] * s, n[2] * s]
}
/// the shortest rotation taking unit `a` to unit `b`
pub fn quat_from_to(a: V3, b: V3) -> Quat {
    let d = dot(a, b);
    if d > 1.0 - 1e-12 { return QI; }
    if d < -1.0 + 1e-12 {
        let mut ax = cross([1.0, 0.0, 0.0], a);
        if len(ax) < 1e-6 { ax = cross([0.0, 1.0, 0.0], a); }
        return axis_angle(ax, std::f64::consts::PI);
    }
    let c = cross(a, b);
    qnorm([1.0 + d, c[0], c[1], c[2]])
}
/// row-major 3x3
pub fn qmat(q: Quat) -> [f64; 9] {
    let [w, x, y, z] = q;
    [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
     2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
     2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)]
}
pub fn mat_to_quat(m: [f64; 9]) -> Quat {
    let tr = m[0] + m[4] + m[8];
    let q = if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        [0.25 * s, (m[7] - m[5]) / s, (m[2] - m[6]) / s, (m[3] - m[1]) / s]
    } else if m[0] > m[4] && m[0] > m[8] {
        let s = (1.0 + m[0] - m[4] - m[8]).sqrt() * 2.0;
        [(m[7] - m[5]) / s, 0.25 * s, (m[1] + m[3]) / s, (m[2] + m[6]) / s]
    } else if m[4] > m[8] {
        let s = (1.0 + m[4] - m[0] - m[8]).sqrt() * 2.0;
        [(m[2] - m[6]) / s, (m[1] + m[3]) / s, 0.25 * s, (m[5] + m[7]) / s]
    } else {
        let s = (1.0 + m[8] - m[0] - m[4]).sqrt() * 2.0;
        [(m[3] - m[1]) / s, (m[2] + m[6]) / s, (m[5] + m[7]) / s, 0.25 * s]
    };
    qnorm(q)
}
/// roll (x), pitch (y), yaw (z), applied in that order about fixed axes (so yaw last)
pub fn euler_to_quat(e: V3) -> Quat {
    qmul(axis_angle([0.0, 0.0, 1.0], e[2]), qmul(axis_angle([0.0, 1.0, 0.0], e[1]), axis_angle([1.0, 0.0, 0.0], e[0])))
}
pub fn quat_to_euler(q: Quat) -> V3 {
    let m = qmat(q);
    let pitch = (-m[6]).clamp(-1.0, 1.0).asin();
    let (roll, yaw) = if m[6].abs() < 0.999999 { (m[7].atan2(m[8]), m[3].atan2(m[0])) } else { (0.0, (-m[1]).atan2(m[4])) };
    [roll, pitch, yaw]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotations() {
        let q = axis_angle([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2);
        let v = qrot(q, [1.0, 0.0, 0.0]);
        assert!((v[1] - 1.0).abs() < 1e-12 && v[0].abs() < 1e-12);
        let e = [0.3, -0.4, 1.2];
        let back = quat_to_euler(euler_to_quat(e));
        for i in 0..3 { assert!((back[i] - e[i]).abs() < 1e-9); }
        let q2 = quat_from_to([0.0, 0.0, 1.0], norm([1.0, 1.0, 0.0]));
        let w = qrot(q2, [0.0, 0.0, 1.0]);
        assert!((w[0] - 0.5f64.sqrt()).abs() < 1e-9);
        let p = Pose::new([1.0, 2.0, 3.0], q);
        let r = p.mul(&p.inverse());
        assert!(len(r.pos) < 1e-12);
        let m = mat_to_quat(qmat(q2));
        assert!((dot([m[1], m[2], m[3]], [q2[1], q2[2], q2[3]]) + m[0] * q2[0]).abs() > 1.0 - 1e-9);
    }
}

/// The shortest turn taking direction `a` to direction `b`.
pub fn quat_between(a: V3, b: V3) -> Quat {
    let (a, b) = (norm(a), norm(b));
    let c = cross(a, b);
    let d = dot(a, b);
    if d < -0.999_999 {
        // opposite: half a turn about any line across `a`
        let o = if a[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
        return axis_angle(norm(cross(a, o)), std::f64::consts::PI);
    }
    let q = [1.0 + d, c[0], c[1], c[2]];
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}
