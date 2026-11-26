# Simscape model

`simulink/welding_robot.slx` is **generated** by `matlab/build_simscape_model.m`
from the same parameters as the MATLAB `rigidBodyTree`: geometry, joint axes,
and mass properties from `init_robot_params.m` and `link_mass_properties.m`.

`run('matlab/run_simulation.m')` rebuilds the model automatically when it is
missing or older than those files. It then simulates the model and compares
its torques with MATLAB's `inverseDynamics`.

**Don't edit parameter values in the `.slx` by hand.** Change
`init_robot_params.m` and the model is rebuilt on the next run. To make
lasting structural changes (for example, CAD File Solids for visuals), change
`build_simscape_model.m` so the change survives a rebuild. Only one person
edits these files at a time, because Git cannot merge `.slx` files.

Requires **Simulink, Simscape, and Simscape Multibody** (license feature
`SimMechanics`).

## What's in the model

| Area | Blocks |
|---|---|
| Inputs (left) | From Workspace `joint_motion_ts` (21 columns: q, qd, qdd for 7 joints) → Demux → Goto tags `q1…q7`, `qd1…qd7`, `qdd1…qdd7` |
| Environment | World Frame, Mechanism Configuration (gravity), Solver Configuration |
| Each joint (one column per joint) | From tags → Simulink-PS Converter (*Provide signals*, 2 derivatives) → Revolute Joint (motion **provided by input**, torque **automatically computed**, torque sensing on) → PS-Simulink → Goto `tauK` |
| Each link | Inertia block (custom mass, CoM, inertia about the CoM) and massless cylinders for the animation |
| Outputs (right) | Transform Sensor World → torch tip → `tcp_sim`; From `tau1…tau7` → Mux → `tau_sim` |

### Frame alignment

A Simscape Revolute Joint always turns about its own **Z** axis, so each
joint sits between two Rigid Transforms. The first applies the joint offset
and turns Z onto the joint axis; the second undoes that rotation, so every
link frame matches the rigidBodyTree body frame.

| Axis | Before the joint | After the joint |
|---|---|---|
| Z (joints 1, 4, 7) | none | none |
| Y (joints 2, 3, 5) | +X, −90° | +X, +90° |
| X (joint 6) | +Y, +90° | +Y, −90° |

## Inputs and outputs

`run_simscape_dynamics.m` sets these through a `Simulink.SimulationInput`, so
nothing has to be in the base workspace:

| Variable | Meaning |
|---|---|
| `joint_motion_ts` | from `generate_welding_ik` |
| `sim_stop_time` | `time_s(end)` |
| `sim_dt` | output sample time and max solver step (0.01 s) |
| `tau_sim` (out) | 7 joint torques [N·m] |
| `tcp_sim` (out) | torch-tip position [m] |

## Verification (current status)

On the placeholder mass properties, Simscape and MATLAB `inverseDynamics`
agree to about 1e-11 N·m on every joint. The Simscape torch tip stays within
0.012 mm of the planned path; that small error comes from smoothing the IK
output. `compare_torques.m` checks both on every run.

To watch the animation, open the model in MATLAB and run it after
`run_simulation`. Mechanics Explorer opens automatically. It needs the
desktop, so you won't see it in `matlab -batch`.
