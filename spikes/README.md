# Spikes

Throwaway measurements taken while planning (2026-11-20). `PLAN.md` quotes their results under
"What was measured". They become Rust tests at M0, after which this folder can go.

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

Masses, stiffnesses, friction and the rotor inertia in these scripts are placeholders, chosen to be
plausible. They are not library values.
