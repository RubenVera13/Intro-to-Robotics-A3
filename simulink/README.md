# Simscape model (Week 1 skeleton)

Target file: `simulink/welding_robot.slx`. Only one person edits the `.slx`
at a time, because Git cannot merge binary model files. Announce in the team
chat before you edit it, and commit as soon as you're done.

Requires **Simulink, Simscape, and Simscape Multibody**. Save the model in
the release the whole team agreed on.

## Inputs from MATLAB

Run `run('matlab/run_simulation.m')` first. It leaves these in the base
workspace (and saves them to `results/joint_motion.mat`):

| Variable | Contents |
|---|---|
| `joint_motion_ts` | timeseries, 21 columns: `q(1:7)`, `qd(1:7)`, `qdd(1:7)` in rad, rad/s, rad/s² |
| `time_s` | sample times [s], 0.01 s spacing |
| `joint_labels` | Waist, Shoulder, UpperElbow, ArmTwist, WristInboard, WristOutboard, ToolInterface |

Set the model stop time to `time_s(end)`.

## Top-level blocks

1. **World Frame**, **Mechanism Configuration** (gravity `[0 0 -9.81]`),
   **Solver Configuration**.
2. **From Workspace** → `joint_motion_ts` → **Demux** (21 outputs).
3. For each joint *k*: one **Simulink-PS Converter** set to
   *Input filtering and derivatives → Provide signals*. Wire
   `q(k)`, `qd(k)`, `qdd(k)` (Demux outputs *k*, *k*+7, *k*+14) to its
   input, first-derivative, and second-derivative ports. Its output goes
   to the joint's motion port.
4. For each joint: a **PS-Simulink Converter** on the joint's torque
   sensing port → **Mux** → **To Workspace** (`tau_sim`, timeseries).

## Kinematic chain: must match `build_welding_robot.m`

A Simscape **Revolute Joint** always turns about its own frame's **Z** axis.
In the rigidBodyTree, joints 2, 3, and 5 turn about **Y** and joint 6 turns
about **X**. So each joint sits between an "align" and an "un-align"
Rigid Transform:

```
link(k-1) frame ──[RT_k: translate origin_k, rotate R_align]──
             [Revolute Joint k]──[RT_k': rotate R_align⁻¹]── link(k) frame
```

| k | Joint | Axis | Translation `origin_k` [m] | R_align | R_align⁻¹ |
|---|---|---|---|---|---|
| 1 | j1_waist | Z | [0 0 0.30] | none | none |
| 2 | j2_shoulder | Y | [0 0 0] | +X, −90° | +X, +90° |
| 3 | j3_upper_elbow | Y | [0 0 0.35] | +X, −90° | +X, +90° |
| 4 | j4_arm_twist | Z | [0 0 0.35] | none | none |
| 5 | j5_wrist_inboard | Y | [0 0 0] | +X, −90° | +X, +90° |
| 6 | j6_wrist_outboard | X | [0 0 0] | +Y, +90° | +Y, −90° |
| 7 | j7_tool_interface | Z | [0 0 0] | none | none |

Then link 7 → **Rigid Transform** [0 0 0.15] → `weld_tip` frame.

Why these rotations: Rx(−90°) maps the joint's Z onto the parent's +Y, and
Ry(+90°) maps it onto +X. Un-aligning after the joint keeps every link frame
oriented like the rigidBodyTree body frame. The translations are copied
from `params.robot.joint_origins_xyz_m`. Read them from `init_robot_params`
instead of typing numbers, so the CAD update only has to change one file.

Each Revolute Joint:

- *Actuation → Motion*: **Provided by Input**
- *Actuation → Torque*: **Automatically Computed**
- *Sensing*: **Actuator Torque** on

## Week 1 bodies (placeholders)

Until the CAD mass properties arrive, attach a **Cylindrical Solid** to each
link frame (radius ≈ 0.04 m, length = distance to the next joint, offset by
half its length along Z). Short links (2, 5, 6) get a ~0.06 m cylinder or
sphere. Torques from these placeholder densities are **not** results.

## Week 1 done-check

- The model runs end to end with the exported motion.
- Add a **Transform Sensor** between World and `weld_tip`. Its position must
  match `project_result.position_m` to about 1 mm. If it doesn't, an axis
  alignment or a translation in the table above is wrong.
