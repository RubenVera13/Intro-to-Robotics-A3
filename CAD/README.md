# CAD and mass properties

Owner: Member 1 (mechanical design).

## What goes in this folder

- Native CAD source (or a link to the shared CAD document).
- `step/` with one STEP file per link (`link1_waist.step`, …, `torch.step`),
  which the Simscape **File Solid** blocks will use for visuals.
- The completed tables below. They feed `matlab/init_robot_params.m`.

## Frames

Each link's frame must match the rigidBodyTree body frame. At zero joint
angles every link frame is parallel to the base frame (Z up). Its origin is
at that link's joint, and `joint_origins_xyz_m(k+1, :)` is the offset to the
next joint. If the CAD needs different offsets or axes, change
`init_robot_params.m` and the table in `simulink/README.md` in the same
commit.

## Geometry and limits (Week 1 freeze)

| k | Joint | Axis | Origin offset from parent [m] | Lower limit [deg] | Upper limit [deg] |
|---|---|---|---|---|---|
| 1 | j1_waist | Z | [0 0 0.30] | | |
| 2 | j2_shoulder | Y | [0 0 0] | | |
| 3 | j3_upper_elbow | Y | [0 0 0.35] | | |
| 4 | j4_arm_twist | Z | [0 0 0.35] | | |
| 5 | j5_wrist_inboard | Y | [0 0 0] | | |
| 6 | j6_wrist_outboard | X | [0 0 0] | | |
| 7 | j7_tool_interface | Z | [0 0 0] | | |
| tip | weld_tip (fixed) | | [0 0 0.15] | | |

The offsets shown are the current provisional values. The current path needs
up to about 92% of the shoulder-to-wrist reach (see
`tests/run_core_tests.m`). Lengthening the upper arm or forearm, or moving
the workpiece closer, keeps the arm away from full stretch, where joint
speeds and torques rise sharply.

## Mass properties (Week 1–2)

Express every value in the **link frame** defined above. Give the inertia
about the link's **center of mass**, in kg·m². Include motors and the torch
in the links that carry them.

| Body | Mass [kg] | CoM [x y z] m | Ixx | Iyy | Izz | Iyz | Ixz | Ixy | Material / notes |
|---|---|---|---|---|---|---|---|---|---|
| link1_waist | | | | | | | | | |
| link2_shoulder | | | | | | | | | |
| link3_elbow | | | | | | | | | |
| link4_twist | | | | | | | | | |
| link5_wrist_in | | | | | | | | | |
| link6_wrist_out | | | | | | | | | |
| link7_tool | | | | | | | | | |
| torch | | | | | | | | | |

Combine the torch with link 7 in one row (mass, CoM and inertia of the
pair), because `params.dynamics.cad` has one row per link.

### Entering the values

In `matlab/init_robot_params.m`:

```matlab
params.dynamics.source = 'cad';
params.dynamics.cad.mass_kg = [ ... ];            % 7x1
params.dynamics.cad.com_m = [ ... ];              % 7x3, link frame
params.dynamics.cad.inertia_com_kg_m2 = [ ... ];  % 7x6 [Ixx Iyy Izz Iyz Ixz Ixy] about the CoM
```

Products of inertia are **tensor elements** (Ixy = −∫xy dm). Some CAD tools
report the integral without the minus sign; check which convention yours
uses. Enter inertia about the **center of mass**. `link_mass_properties.m`
applies the parallel-axis shift that `rigidBody` needs, while Simscape takes
the CoM values directly. Then run `run('matlab/run_simulation.m')`. The
Simscape model rebuilds itself, and `compare_torques` confirms that both
models still agree.

Until then, the code uses placeholder parts (aluminium tubes, motor masses
of 0.4–2 kg, a 1 kg torch; 13.6 kg in total). Torques from those prove the
pipeline works but are **not** report results.
