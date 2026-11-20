"""Spike 2: what a whole field costs in MuJoCo, and whether a geared lift is stable.

Two questions for PLAN.md's engine decision (D2):

  A. Six robots (swerve on real wheel contacts, a two-stage elevator, an arm and a wrist each)
     with 60 CORAL and 18 ALGAE on a REEFSCAPE-sized field: how many ms of one core per
     simulated second, at 1 kHz? With CORAL as one cylinder, and as a hollow ring of boxes.

  B. A carriage on two Kraken X60 at 10:1 (28 kg of reflected rotor on an 8 kg carriage):
     is it stable at 1 kHz with the motor as an affine actuator and the rotor as joint
     armature? This is the case a maximal-coordinate engine can't state at all.

    python spikes/field_timing.py
"""
import math
import time

import mujoco
import numpy as np

IN = 0.0254
PIPE_R = 4.5 * IN / 2
PIPE_L = 11.875 * IN
ALGAE_R = 16.25 * IN / 2

# Kraken X60 (trapezoidal)
R_OHM = 12.0 / 366.0
KT = 7.09 / 366.0
KV = (6000 * 2 * math.pi / 60) / (12.0 - 2.0 * R_OHM)
ROTOR_J = 7.0e-5      # placeholder


def motor(name, joint, gear, count=1, linear_r=None):
    """A DC motor as MuJoCo sees it: force = gain * volts + bias * velocity (the back-EMF),
    which the implicit integrator takes implicitly. `gear` is motor turns per joint radian,
    or per metre when linear_r (the spool radius) is given."""
    g = gear / linear_r if linear_r else gear
    gain = count * KT * g / R_OHM
    bias = -count * KT * g * g / (KV * R_OHM)
    arm = count * ROTOR_J * g * g
    act = (f'<general name="{name}" joint="{joint}" ctrlrange="-12 12" gainprm="{gain}" '
           f'biastype="affine" biasprm="0 0 {bias}"/>')
    return act, arm


def robot(i, x, y):
    acts = []
    wheels = ""
    for k, (wx, wy) in enumerate(((0.27, 0.27), (0.27, -0.27), (-0.27, 0.27), (-0.27, -0.27))):
        a_s, arm_s = motor(f"r{i}s{k}", f"r{i}steer{k}", 150 / 7)
        a_d, arm_d = motor(f"r{i}d{k}", f"r{i}drive{k}", 6.12)
        acts += [a_s, a_d]
        wheels += f"""
      <body pos="{wx} {wy} -0.03">
        <joint name="r{i}steer{k}" axis="0 0 1" armature="{arm_s}" damping="0.05"/>
        <geom type="box" size="0.03 0.03 0.02" pos="0 0 0.06" mass="1.5" contype="0" conaffinity="0"/>
        <body>
          <joint name="r{i}drive{k}" axis="0 1 0" armature="{arm_d}"/>
          <geom type="cylinder" size="0.0508 0.019" euler="90 0 0" mass="0.3"
                friction="1.2 0.005 0.0001" condim="3"/>
        </body>
      </body>"""
    a_e, arm_e = motor(f"r{i}elev", f"r{i}elev1", 10, count=2, linear_r=0.0222)
    a_a, arm_a = motor(f"r{i}arm", f"r{i}arm", 60)
    a_w, arm_w = motor(f"r{i}wrist", f"r{i}wrist", 25)
    acts += [a_e, a_a, a_w]
    body = f"""
    <body name="r{i}" pos="{x} {y} 0.085">
      <freejoint/>
      <geom type="box" size="0.42 0.42 0.07" pos="0 0 0.04" mass="40" friction="0.3 0.005 0.0001"/>
      {wheels}
      <body pos="0 0 0.15">
        <joint name="r{i}elev1" type="slide" axis="0 0 1" range="0 0.7" armature="{arm_e}" damping="5"/>
        <geom type="box" size="0.03 0.25 0.4" pos="0 0 0.4" mass="3"/>
        <body pos="0 0 0.1">
          <joint name="r{i}elev2" type="slide" axis="0 0 1" range="0 0.7" damping="5"/>
          <geom type="box" size="0.05 0.2 0.08" pos="0.06 0 0.3" mass="4"/>
          <body pos="0.1 0 0.3">
            <joint name="r{i}arm" axis="0 1 0" range="-2 2" armature="{arm_a}" damping="0.2"/>
            <geom type="capsule" fromto="0 0 0 0.45 0 0" size="0.025" mass="2.5"/>
            <body pos="0.45 0 0">
              <joint name="r{i}wrist" axis="0 1 0" range="-2 2" armature="{arm_w}" damping="0.1"/>
              <geom type="box" size="0.1 0.15 0.05" pos="0.1 0 0" mass="2"/>
            </body>
          </body>
        </body>
      </body>
    </body>"""
    # a continuous elevator: the second stage rides at the first's travel
    eq = f'<joint joint1="r{i}elev2" joint2="r{i}elev1" polycoef="0 1 0 0 0"/>'
    return body, acts, eq


def coral(j, x, y, z, hollow):
    if not hollow:
        geoms = f'<geom type="cylinder" size="{PIPE_R} {PIPE_L / 2}" mass="0.6" friction="0.45 0.005 0.0001"/>'
    else:
        n = 12
        wall = 0.006
        rm = PIPE_R - wall / 2
        half_w = rm * math.tan(math.pi / n) + 0.001
        geoms = ""
        for k in range(n):
            a = 2 * math.pi * k / n
            geoms += (f'<geom type="box" size="{wall / 2} {half_w} {PIPE_L / 2}" '
                      f'pos="{rm * math.cos(a)} {rm * math.sin(a)} 0" euler="0 0 {math.degrees(a)}" '
                      f'mass="{0.6 / n}" friction="0.45 0.005 0.0001"/>')
    return f'<body pos="{x} {y} {z}" euler="90 {37 * j % 180} 0"><freejoint/>{geoms}</body>'


def field(hollow, robots=6, corals=60, algae=18, dt=0.001):
    rng = np.random.default_rng(7)
    bodies, acts, eqs = "", [], ""
    for i in range(robots):
        b, a, e = robot(i, -6 + 2.4 * i, -2.5 if i % 2 else 2.5)
        bodies += b
        acts += a
        eqs += e
    pieces = ""
    for j in range(corals):
        pieces += coral(j, rng.uniform(-7, 7), rng.uniform(-1.5, 1.5), PIPE_R + 0.002 + 0.13 * (j % 3), hollow)
    for j in range(algae):
        pieces += (f'<body pos="{rng.uniform(-7, 7)} {rng.uniform(-3.5, 3.5)} {ALGAE_R + 0.01}"><freejoint/>'
                   f'<geom type="sphere" size="{ALGAE_R}" mass="0.7" friction="0.7 0.005 0.0001" '
                   f'solref="0.02 0.4"/></body>')
    reef = ""
    for k in range(6):
        a = math.pi / 3 * k
        reef += (f'<geom type="box" size="0.08 0.8 0.6" pos="{4.2 + 0.83 * math.cos(a)} {0.83 * math.sin(a)} 0.6" '
                 f'euler="0 0 {math.degrees(a)}"/>')
        reef += (f'<geom type="box" size="0.08 0.8 0.6" pos="{-4.2 + 0.83 * math.cos(a)} {0.83 * math.sin(a)} 0.6" '
                 f'euler="0 0 {math.degrees(a)}"/>')
    xml = f"""
<mujoco>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <size memory="512M"/>
  <worldbody>
    <geom type="plane" size="9 4.5 0.1" friction="1 0.005 0.0001"/>
    <geom type="box" size="8.8 0.02 0.25" pos="0 4.1 0.25"/>
    <geom type="box" size="8.8 0.02 0.25" pos="0 -4.1 0.25"/>
    <geom type="box" size="0.02 4.1 1" pos="8.8 0 1"/>
    <geom type="box" size="0.02 4.1 1" pos="-8.8 0 1"/>
    {reef}
    {bodies}
    {pieces}
  </worldbody>
  <equality>{eqs}</equality>
  <actuator>{''.join(acts)}</actuator>
</mujoco>"""
    return mujoco.MjModel.from_xml_string(xml)


def time_field(hollow, dt=0.001, seconds=4.0):
    model = field(hollow, dt=dt)
    data = mujoco.MjData(model)
    steps = int(seconds / dt)
    n_act = model.nu
    con = 0
    t0 = time.perf_counter()
    for s in range(steps):
        t = s * dt
        # every robot drives in a slow weave, lifts and swings its arm: all of it moving at once
        for a in range(n_act):
            kind = a % 11
            if kind in (1, 3, 5, 7):            # drive motors
                data.ctrl[a] = 6.0 * math.sin(0.8 * t + a)
            elif kind in (0, 2, 4, 6):          # steer motors
                data.ctrl[a] = 1.5 * math.sin(1.3 * t + a)
            elif kind == 8:                     # elevator
                data.ctrl[a] = 4.0 + 5.0 * math.sin(1.1 * t + a)
            else:
                data.ctrl[a] = 2.0 * math.sin(0.9 * t + a)
        mujoco.mj_step(model, data)
        con += data.ncon
    wall = time.perf_counter() - t0
    ok = bool(np.all(np.isfinite(data.qpos)))
    return dict(ms_per_s=1000 * wall / seconds, contacts=con / steps, dof=model.nv,
                geoms=model.ngeom, ok=ok)


def lift(dt, integrator="implicitfast"):
    act, arm = motor("lift", "lift", 10, count=2, linear_r=0.0222)
    xml = f"""
<mujoco>
  <option timestep="{dt}" integrator="{integrator}"/>
  <worldbody>
    <body>
      <joint name="lift" type="slide" axis="0 0 1" range="0 1.5" armature="{arm}"/>
      <geom type="box" size="0.1 0.1 0.05" mass="8"/>
    </body>
  </worldbody>
  <actuator>{act}</actuator>
</mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    goal, t_reach, peak = 1.0, None, 0.0
    g = 10 / 0.0222
    for s in range(int(2.0 / dt)):
        err = goal - data.qpos[0]
        volts = max(-12.0, min(12.0, 60.0 * err - 4.0 * data.qvel[0] + 0.25))
        # a 40 A stator limit per motor, by clamping the volts the limit allows at this speed
        back = data.qvel[0] * g / KV
        volts = max(back - 40 * R_OHM, min(back + 40 * R_OHM, volts))
        data.ctrl[0] = volts
        mujoco.mj_step(model, data)
        peak = max(peak, data.qpos[0])
        if t_reach is None and data.qpos[0] > 0.98 * goal:
            t_reach = (s + 1) * dt
        if not np.isfinite(data.qpos[0]):
            return dict(ok=False, arm=arm)
    return dict(ok=True, t_reach=t_reach, overshoot_mm=1000 * (peak - goal),
                settle_mm=1000 * (data.qpos[0] - goal), arm=arm)


if __name__ == "__main__":
    print(f"MuJoCo {mujoco.__version__}")
    print("\nA. Six robots, 60 CORAL, 18 ALGAE, everything moving, one core:")
    for hollow, label in ((False, "CORAL as one cylinder"), (True, "CORAL as a hollow ring of 12 boxes")):
        for dt in (0.002, 0.001):
            r = time_field(hollow, dt)
            print(f"  {label:<36} {1 / dt:>5.0f} Hz: {r['ms_per_s']:7.0f} ms per simulated s "
                  f"({r['dof']} dof, {r['geoms']} geoms, {r['contacts']:.0f} contacts a step)"
                  + ("" if r["ok"] else "  BLEW UP"))
    print("\nB. 8 kg carriage, two Kraken X60 at 10:1 on a 1.75 in. spool, 40 A each, to 1 m:")
    for integ in ("implicitfast", "Euler"):
        for dt in (0.005, 0.002, 0.001):
            r = lift(dt, integ)
            if r["ok"]:
                print(f"  {integ:<13} {1 / dt:>5.0f} Hz: there in {r['t_reach']:.3f} s, overshoot "
                      f"{r['overshoot_mm']:+.1f} mm, rests {r['settle_mm']:+.2f} mm off "
                      f"(reflected rotor {r['arm']:.0f} kg)")
            else:
                print(f"  {integ:<13} {1 / dt:>5.0f} Hz: BLEW UP")
