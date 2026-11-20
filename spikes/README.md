# Spikes

Throwaway measurements taken while planning (2026-11-20). `PLAN.md` quotes their results under
"What was measured". The physics ones become Rust tests at M0 and the import ones become `crates/cad`
at M3, after which this folder can go.

```bash
python3 -m venv .venv && .venv/bin/pip install mujoco==3.17.0 numpy
```

```bash
.venv/bin/python spikes/roller_intake.py
```

```bash
.venv/bin/python spikes/field_timing.py
```

| Script | Question |
|---|---|
| `roller_intake.py` | Does an elastic-foundation roller contact on top of MuJoCo give sensible intake answers, and at what step rate has it converged? |
| `field_timing.py` | What does a six-robot REEFSCAPE-sized field cost per simulated second, and is a geared lift with a large reflected rotor stable? |
| `onshape_mates.py` | Do an Onshape assembly's mates give the links and joints? Takes the API's assembly definition, saved as JSON. |
| `step_tree.py`, `pose_diff.py` | With no mates, do two poses of the same robot give them? Takes two STEP exports. Needs `pip install cadquery-ocp`. |

The CAD these two were run on (the owner's 2026 robot, 2910's 2025 robot) is not in the repository
and must not be added.

Masses, stiffnesses, friction and the rotor inertia in these scripts are placeholders, chosen to be
plausible. They are not library values.
