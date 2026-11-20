"""Spike 1: a compliant roller pulling a CORAL (PVC pipe) in against the carpet.

Checks the plan's contact split (PLAN.md D3/D4) before anything is built on it:
  - MuJoCo owns the rigid bodies and the rigid contacts (pipe on carpet).
  - The roller-to-piece contact is ours: an elastic foundation (a bed of springs) on the
    roller, sliced along its width, each slice with a stick/slip bristle friction.
  - The roller is not a MuJoCo body. It is a shaft with a DC motor behind a gearbox,
    integrated here, with a stator current limit.

The question it answers: does that model give the engineering answers a prototype gives
(too little squeeze slips, more squeeze costs current, a low current limit stalls), and is it
stable at 1-2 kHz?

    python spikes/roller_intake.py
"""
import math
import time

import mujoco
import numpy as np

IN = 0.0254

# CORAL, 2025: 4 in. Schedule 40 PVC, 11 7/8 in. long. Mass is a placeholder (see PLAN.md, game pack).
PIPE_R = 4.5 * IN / 2
PIPE_L = 11.875 * IN
PIPE_M = 0.60

ROLLER_R = 1.0 * IN          # a 2 in. roller
ROLLER_W = 0.36              # wider than the pipe is long
SLICES = 18

# Kraken X60 (trapezoidal), from the published curve
V_NOM = 12.0
STALL_T = 7.09
STALL_I = 366.0
FREE_I = 2.0
FREE_W = 6000 * 2 * math.pi / 60
R_OHM = V_NOM / STALL_I
KT = STALL_T / STALL_I
KV = FREE_W / (V_NOM - FREE_I * R_OHM)   # rad/s per volt
ROTOR_J = 7.0e-5                          # placeholder, to be sourced

XML = f"""
<mujoco>
  <option timestep="0.0005" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <worldbody>
    <geom name="carpet" type="plane" size="5 5 0.1" friction="0.45 0.005 0.0001"/>
    <body name="pipe" pos="-0.20 0 {PIPE_R}">
      <freejoint/>
      <geom name="pipe" type="cylinder" size="{PIPE_R} {PIPE_L / 2}" euler="90 0 0"
            mass="{PIPE_M}" friction="0.45 0.005 0.0001"/>
    </body>
  </worldbody>
</mujoco>
"""


def run(squeeze_mm, mu, limit_a, k_area=1.0e5, loss=0.25, gear=3.0, dt=0.0005,
        approach=1.0, yaw_deg=0.0, t_end=1.2):
    """One pass. Returns a dict of what an engineer would read off the bench."""
    model = mujoco.MjModel.from_xml_string(XML)
    model.opt.timestep = dt
    data = mujoco.MjData(model)
    pipe = model.body("pipe").id

    # the pipe arrives at `approach` m/s (the robot driving onto it), turned by yaw_deg
    half = math.radians(yaw_deg) / 2
    data.qpos[3:7] = [math.cos(half), 0, 0, math.sin(half)]
    data.qvel[0] = approach

    gap = 2 * PIPE_R - squeeze_mm * 1e-3
    axis_z = gap + ROLLER_R
    ys = (np.arange(SLICES) + 0.5) / SLICES * ROLLER_W - ROLLER_W / 2
    width = ROLLER_W / SLICES
    centers = np.stack([np.zeros(SLICES), ys, np.full(SLICES, axis_z)], axis=1)
    bristle = np.zeros((SLICES, 3))       # each slice's tangential stretch
    k_t = 0.6 * k_area                    # tangential foundation stiffness, per m of width

    shaft_j = ROTOR_J * gear * gear + 2.0e-4      # rotor reflected to the roller + the roller itself
    w = 0.0                                        # roller speed, rad/s (positive pulls +x at its bottom)
    out = dict(peak_a=0.0, peak_n=0.0, min_w=1e9, energy_j=0.0, passed=False, t_pass=None,
               max_yaw=0.0)
    spun_up = False

    steps = int(t_end / dt)
    for i in range(steps):
        t = i * dt
        pos = data.xpos[pipe].copy()
        rot = data.xmat[pipe].reshape(3, 3)
        axis = rot[:, 1]                           # the pipe's axis in the world
        v = data.cvel[pipe][3:].copy()
        om = data.cvel[pipe][:3].copy()
        # cvel is at the subtree's center of mass, which for a free single body is its own
        force = np.zeros(3)
        torque = np.zeros(3)
        shaft_t = 0.0
        normal_total = 0.0

        for s in range(SLICES):
            c = centers[s]
            rel = c - pos
            along = float(rel @ axis)
            if abs(along) > PIPE_L / 2:
                bristle[s] = 0.0
                continue
            radial = rel - along * axis
            d = float(np.linalg.norm(radial))
            depth = ROLLER_R + PIPE_R - d
            if depth <= 0.0 or d < 1e-9:
                bristle[s] = 0.0
                continue
            n = radial / d                         # from the pipe toward the roller
            p = pos + along * axis + n * (PIPE_R - depth / 2)   # the middle of the overlap
            # velocities at the contact: the pipe's, and the roller's surface
            v_pipe = v + np.cross(om, p - pos)
            r_roller = p - c
            v_roller = np.cross(np.array([0.0, -w, 0.0]), r_roller)   # -y spin moves the bottom +x
            v_rel = v_roller - v_pipe
            closing = -float(v_rel @ n)            # > 0 while the squeeze is growing
            # normal: a bed of springs, with hysteresis as a loss on the way in
            fn = k_area * width * depth * (1.0 + loss * np.tanh(closing / 0.05))
            fn = max(fn, 0.0)
            # tangential: a bristle that stretches with slip and lets go at mu * fn
            v_t = v_rel - (v_rel @ n) * n
            bristle[s] += v_t * dt
            bristle[s] -= (bristle[s] @ n) * n
            ft = k_t * width * bristle[s]
            mag = float(np.linalg.norm(ft))
            if mag > mu * fn and mag > 0.0:
                ft *= mu * fn / mag
                bristle[s] = ft / (k_t * width)
            f_on_pipe = -n * fn + ft               # the roller pushes the pipe away and drags it along
            force += f_on_pipe
            torque += np.cross(p - pos, f_on_pipe)
            shaft_t += float(np.cross(r_roller, -f_on_pipe) @ np.array([0.0, -1.0, 0.0]))
            normal_total += fn

        # the motor: back-EMF, a stator current limit, through the gearbox
        w_motor = w * gear
        amps = (V_NOM - w_motor / KV) / R_OHM
        amps = max(-limit_a, min(limit_a, amps))
        motor_t = KT * amps * gear * 0.9           # 90 % gearbox efficiency
        # implicit in the back-EMF term, so a big gear ratio stays stable at this step
        damp = KT * gear * gear * 0.9 / (KV * R_OHM) if abs(amps) < limit_a else 0.0
        w = (w + dt * (motor_t + shaft_t + damp * w) / shaft_j) / (1.0 + dt * damp / shaft_j)

        data.xfrc_applied[pipe, :3] = force
        data.xfrc_applied[pipe, 3:] = torque
        mujoco.mj_step(model, data)

        if not spun_up and w * gear > 0.9 * FREE_W:
            spun_up = True
        if normal_total > 0.0:
            out["peak_a"] = max(out["peak_a"], abs(amps))
            out["peak_n"] = max(out["peak_n"], normal_total)
            out["min_w"] = min(out["min_w"], w)
            out["energy_j"] += abs(amps) * V_NOM * min(1.0, abs(w_motor) / FREE_W + abs(amps) * R_OHM / V_NOM) * dt
        yaw = math.degrees(math.atan2(axis[0], axis[1]))
        out["max_yaw"] = max(out["max_yaw"], abs(yaw))
        if not out["passed"] and data.xpos[pipe][0] > ROLLER_R + PIPE_R:
            out["passed"] = True
            out["t_pass"] = t
        if not np.all(np.isfinite(data.qpos)):
            out["blew_up"] = True
            break
    out["final_x"] = float(data.xpos[pipe][0])
    out["final_yaw"] = math.degrees(math.atan2(data.xmat[pipe].reshape(3, 3)[0, 1],
                                               data.xmat[pipe].reshape(3, 3)[1, 1]))
    if out["min_w"] > 1e8:
        out["min_w"] = float("nan")
    return out


def row(label, r):
    passed = f"in at {r['t_pass']:.2f} s" if r["passed"] else f"NOT in (x={r['final_x']:+.2f} m)"
    print(f"  {label:<34} {passed:<22} peak {r['peak_a']:5.1f} A  squeeze force {r['peak_n']:6.0f} N"
          f"  slowest roller {r['min_w'] * 60 / (2 * math.pi):6.0f} rpm"
          + ("  BLEW UP" if r.get("blew_up") else ""))


if __name__ == "__main__":
    print("Squeeze sweep (roller-on-PVC friction 1.0, 40 A stator limit):")
    for sq in (1, 3, 6, 10, 15, 22, 30):
        row(f"{sq:>2} mm squeeze", run(sq, 1.0, 40))
    print("\nSlick roller (friction 0.25), same sweep:")
    for sq in (1, 3, 6, 10, 15, 22):
        row(f"{sq:>2} mm squeeze", run(sq, 0.25, 40))
    print("\nCurrent limit at 15 mm squeeze:")
    for lim in (10, 20, 40, 80):
        row(f"{lim:>2} A stator limit", run(15, 1.0, lim))
    print("\nPipe arriving turned (10 mm squeeze): does it square up on the roller?")
    for yaw in (0, 15, 30, 45):
        r = run(10, 1.0, 40, yaw_deg=yaw)
        row(f"arrives at {yaw:>2} deg", r)
        print(f"      leaves at {r['final_yaw']:+.1f} deg")
    print("\nStep size (10 mm squeeze): the same answer at each, or not?")
    for dt in (0.002, 0.001, 0.0005, 0.00025):
        t0 = time.perf_counter()
        r = run(10, 1.0, 40, dt=dt)
        wall = time.perf_counter() - t0
        row(f"{1 / dt:>5.0f} Hz", r)
