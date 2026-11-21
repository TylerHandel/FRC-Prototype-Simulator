//! MuJoCo, behind one door (PLAN.md D2): nothing outside this module names a MuJoCo type.
//!
//! The rest of the simulator builds an MJCF string (`compile`), and talks to the engine in plain
//! arrays: body poses and velocities, joint positions, forces applied at points.
use mujoco_rs::prelude::*;
use model::V3;

pub struct Engine {
    data: MjData<Box<MjModel>>,
    /// forces and torques applied this step, per body (cleared after each step)
    applied: Vec<[f64; 6]>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Obj { Body, Joint, Actuator, Geom, Equality }

impl Engine {
    pub fn from_xml(xml: &str) -> anyhow::Result<Self> {
        let model = MjModel::from_xml_string(xml).map_err(|e| anyhow::anyhow!("MuJoCo could not build the model: {e:?}"))?;
        let nbody = model.ffi().nbody as usize;
        let data = MjData::new(Box::new(model));
        Ok(Engine { data, applied: vec![[0.0; 6]; nbody] })
    }

    pub fn id(&self, kind: Obj, name: &str) -> Option<usize> {
        let k = match kind {
            Obj::Body => MjtObjBase::mjOBJ_BODY,
            Obj::Joint => MjtObjBase::mjOBJ_JOINT,
            Obj::Actuator => MjtObjBase::mjOBJ_ACTUATOR,
            Obj::Geom => MjtObjBase::mjOBJ_GEOM,
            Obj::Equality => MjtObjBase::mjOBJ_EQUALITY,
        };
        self.data.model().name_to_id(k, name)
    }

    pub fn timestep(&self) -> f64 { self.data.model().ffi().opt.timestep }
    pub fn set_timestep(&mut self, dt: f64) {
        // SAFETY: the step size is one of the model fields MuJoCo allows changing between steps;
        // nothing else holds a reference to the model while we do.
        unsafe { self.data.model_mut().opt_mut().timestep = dt; }
    }
    pub fn time(&self) -> f64 { self.data.ffi().time }
    pub fn nbody(&self) -> usize { self.applied.len() }
    pub fn contacts(&self) -> usize { self.data.ffi().ncon.max(0) as usize }

    /// Works out everything that depends on the current positions and velocities (body poses,
    /// velocities, gravity on each joint), so what the simulator reads before `step` is current.
    /// (After a plain mj_step those still describe the state before it.)
    pub fn begin_step(&mut self) { self.data.step1(); }

    /// Finishes the step begun by `begin_step`, with the forces applied since.
    pub fn step(&mut self) {
        {
            let x = self.data.xfrc_applied_mut();
            for (b, f) in self.applied.iter().enumerate() { x[b] = *f; }
        }
        self.data.step2();
        for f in &mut self.applied { *f = [0.0; 6]; }
    }

    /// Recomputes positions and velocities without stepping (after setting the state by hand).
    pub fn forward(&mut self) { self.data.forward(); }

    pub fn body_pos(&self, b: usize) -> V3 { self.data.xpos()[b] }
    /// row-major rotation of the body frame
    pub fn body_mat(&self, b: usize) -> [f64; 9] { self.data.xmat()[b] }
    pub fn body_quat(&self, b: usize) -> [f64; 4] { self.data.xquat()[b] }
    /// the body's centre of mass, world
    pub fn body_com(&self, b: usize) -> V3 { self.data.xipos()[b] }
    pub fn body_mass(&self, b: usize) -> f64 { self.data.model().body_mass()[b] }
    pub fn subtree_mass(&self, b: usize) -> f64 { self.data.model().body_subtreemass()[b] }
    pub fn subtree_com(&self, b: usize) -> V3 { self.data.subtree_com()[b] }
    /// principal inertia and its frame (row-major, world)
    pub fn body_inertia(&self, b: usize) -> (V3, [f64; 9]) { (self.data.model().body_inertia()[b], self.data.ximat()[b]) }

    /// (linear velocity at `p`, angular velocity), world
    pub fn velocity_at(&self, b: usize, p: V3) -> (V3, V3) {
        let c = self.data.cvel()[b];
        let root = self.data.model().body_rootid()[b] as usize;
        let o = self.data.subtree_com()[root];
        let w = [c[0], c[1], c[2]];
        let r = [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
        let wr = model::math::cross(w, r);
        ([c[3] + wr[0], c[4] + wr[1], c[5] + wr[2]], w)
    }

    /// Applies a force (N, world) at a point (world) on a body, for the next step.
    pub fn push(&mut self, b: usize, f: V3, at: V3) {
        let com = self.body_com(b);
        let t = model::math::cross(model::math::sub(at, com), f);
        let a = &mut self.applied[b];
        for i in 0..3 { a[i] += f[i]; a[i + 3] += t[i]; }
    }
    pub fn torque(&mut self, b: usize, t: V3) { let a = &mut self.applied[b]; for i in 0..3 { a[i + 3] += t[i]; } }

    pub fn set_ctrl(&mut self, a: usize, v: f64) { self.data.ctrl_mut()[a] = v; }
    pub fn actuator_force(&self, a: usize) -> f64 { self.data.actuator_force()[a] }

    fn qadr(&self, j: usize) -> usize { self.data.model().jnt_qposadr()[j] as usize }
    pub fn dof(&self, j: usize) -> usize { self.data.model().jnt_dofadr()[j] as usize }
    /// a hinge's or slide's position
    pub fn q(&self, j: usize) -> f64 { self.data.qpos()[self.qadr(j)] }
    pub fn qd(&self, j: usize) -> f64 { self.data.qvel()[self.dof(j)] }
    pub fn set_q(&mut self, j: usize, v: f64) { let a = self.qadr(j); self.data.qpos_mut()[a] = v; }
    pub fn set_qd(&mut self, j: usize, v: f64) { let a = self.dof(j); self.data.qvel_mut()[a] = v; }
    /// gravity and velocity forces on a joint (what holding it still has to cancel)
    pub fn bias(&self, j: usize) -> f64 { self.data.qfrc_bias()[self.dof(j)] }
    pub fn add_joint_force(&mut self, j: usize, f: f64) { let d = self.dof(j); self.data.qfrc_applied_mut()[d] += f; }
    pub fn clear_joint_forces(&mut self) { for x in self.data.qfrc_applied_mut() { *x = 0.0; } }

    /// a free joint's pose: position and quaternion (w x y z)
    pub fn free_pose(&self, j: usize) -> (V3, [f64; 4]) {
        let a = self.qadr(j);
        let q = &self.data.qpos()[a..a + 7];
        ([q[0], q[1], q[2]], [q[3], q[4], q[5], q[6]])
    }
    pub fn free_vel(&self, j: usize) -> (V3, V3) {
        let a = self.dof(j);
        let v = &self.data.qvel()[a..a + 6];
        // a free joint's qvel: linear (world), then angular (body frame)
        ([v[0], v[1], v[2]], [v[3], v[4], v[5]])
    }
    pub fn set_free(&mut self, j: usize, pos: V3, quat: [f64; 4], lin: V3, ang_body: V3) {
        let (a, d) = (self.qadr(j), self.dof(j));
        let qp = self.data.qpos_mut();
        qp[a..a + 3].copy_from_slice(&pos);
        qp[a + 3..a + 7].copy_from_slice(&quat);
        let qv = self.data.qvel_mut();
        qv[d..d + 3].copy_from_slice(&lin);
        qv[d + 3..d + 6].copy_from_slice(&ang_body);
    }

    /// the diagonal of the mass matrix for each dof (effective inertia a joint's motor sees)
    pub fn mass_diag(&mut self) -> Vec<f64> {
        let nv = self.data.model().ffi().nv as usize;
        let mut m = vec![0.0; nv * nv];
        self.data.forward();
        if self.data.full_m(&mut m).is_err() { return vec![1.0; nv]; }
        (0..nv).map(|i| m[i * nv + i]).collect()
    }

    pub fn set_equality(&mut self, e: usize, on: bool) { self.data.eq_active_mut()[e] = on; }

    /// MuJoCo's own contacts touching body `b`: (the other geom's name, its body, depth, normal force)
    pub fn contacts_of(&self, b: usize) -> Vec<(String, usize, f64, f64)> {
        let m = self.data.model();
        self.data.contact().iter().enumerate().filter_map(|(i, c)| {
            let (g1, g2) = (c.geom[0] as usize, c.geom[1] as usize);
            let (b1, b2) = (self.geom_body(g1), self.geom_body(g2));
            let other = if b1 == b { g2 } else if b2 == b { g1 } else { return None };
            let name = m.id_to_name(MjtObjBase::mjOBJ_GEOM, other).unwrap_or("?").to_string();
            Some((name, self.geom_body(other), -c.dist, self.data.contact_force(i)[0]))
        }).collect()
    }

    pub fn ngeom(&self) -> usize { self.data.model().ffi().ngeom as usize }
    pub fn geom_body(&self, g: usize) -> usize { self.data.model().geom_bodyid()[g] as usize }
    pub fn geom_pos(&self, g: usize) -> V3 { self.data.geom_xpos()[g] }
    /// the radius of a sphere round the geom, about its position
    pub fn geom_rbound(&self, g: usize) -> f64 { self.data.model().geom_rbound()[g] }
    /// the signed distance between two geoms (negative: overlapping by that much), and the
    /// nearest point on each; `None` when farther apart than `max`
    pub fn geom_distance(&mut self, g1: usize, g2: usize, max: f64) -> Option<(f64, V3, V3)> {
        let mut ft = [0.0; 6];
        let d = self.data.try_geom_distance(g1, g2, max, Some(&mut ft)).ok()?;
        if d >= max { return None; }
        Some((d, [ft[0], ft[1], ft[2]], [ft[3], ft[4], ft[5]]))
    }

    /// whether every joint position is finite (a blown-up model is not)
    pub fn finite(&self) -> bool { self.data.qpos().iter().all(|x| x.is_finite()) }
}

#[cfg(test)]
mod probe {
    use super::*;
    #[test]
    #[ignore]
    fn mesh_distance() {
        let xml = r#"<mujoco><asset><mesh name="m" vertex="-0.15 -0.15 -0.01 0.15 -0.15 -0.01 -0.15 0.15 -0.01 0.15 0.15 -0.01 -0.15 -0.15 0.01 0.15 -0.15 0.01 -0.15 0.15 0.01 0.15 0.15 0.01"/></asset>
            <worldbody><geom name="h" type="mesh" mesh="m" pos="0 0 0.4"/><geom name="b" type="box" size="0.15 0.15 0.01" pos="1 0 0.4"/>
            <body name="s" pos="0 0 1"><freejoint name="s"/><geom name="s" type="sphere" size="0.2"/></body>
            <body name="t" pos="1 0 1"><freejoint name="t"/><geom name="t" type="sphere" size="0.2"/></body></worldbody></mujoco>"#;
        let mut e = Engine::from_xml(xml).unwrap();
        let (h, b, s, t) = (e.id(Obj::Geom, "h").unwrap(), e.id(Obj::Geom, "b").unwrap(), e.id(Obj::Geom, "s").unwrap(), e.id(Obj::Geom, "t").unwrap());
        for z in [0.65, 0.62, 0.61, 0.605, 0.60, 0.59] {
            let js = e.id(Obj::Joint, "s").unwrap(); let jt = e.id(Obj::Joint, "t").unwrap();
            e.set_free(js, [0.0, 0.0, z], [1.0, 0.0, 0.0, 0.0], [0.0; 3], [0.0; 3]);
            e.set_free(jt, [1.0, 0.0, z], [1.0, 0.0, 0.0, 0.0], [0.0; 3], [0.0; 3]);
            e.forward();
            println!("z {z}: mesh {:?}  box {:?}", e.geom_distance(s, h, 0.1), e.geom_distance(t, b, 0.1));
        }
    }
}
