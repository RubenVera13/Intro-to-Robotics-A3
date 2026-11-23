# 7-DOF Welding-Path Robot Simulation

**Course:** Introduction to Robotics, NTUST, Fall 2026 (final project)  
**Team:** Emmanuel Acosta, Ivan Arce, Mauricio Olmedo, Rafael Pereira, Ruben Vera

## Project goal

Build a seven-axis robot model, trace plus, circle, and star paths above a flat workpiece, and use Simscape Multibody inverse dynamics to estimate the required joint torques.

This is an academic robot simulation. The current model represents the torch path and rigid-body kinematics; it does not simulate an arc, heat transfer, weld deposition, or a specific commercial robot.

## Current status

The repository currently contains a **kinematic prototype** that generates the three paths, solves inverse kinematics, exports joint-position time series, and plots the TCP and joint trajectories. The Simscape model, CAD geometry, validated mass properties, and torque results are still to be added.

The initial kinematic model uses seven single-axis revolute joints:

| Axis | Joint | Local axis |
|---:|---|---|
| 1 | Waist | Z |
| 2 | Shoulder | Y |
| 3 | Upper elbow | Y |
| 4 | Arm twist | Z |
| 5 | Wrist inboard | Y |
| 6 | Wrist outboard | X |
| 7 | Tool interface roll | Z |

The torch tip is a fixed frame 0.15 m beyond the seventh joint. The dimensions and ±π joint bounds in the prototype are provisional values, not CAD measurements or mechanical stops. The wrist joints are modeled as individual revolute axes, not Simscape Universal Joint blocks.

## Run the kinematic prototype

### Requirements

- MATLAB R2024a or later
- Robotics System Toolbox

From the repository root, run:

```matlab
run('matlab/run_simulation.m')
```

The script leaves a `project_result` structure in the MATLAB workspace. It contains the robot, path, joint positions, Simulink-ready joint time series, per-sample IK statuses, and position/orientation error measurements. The script also opens path and joint-angle plots.

The current entry point does **not** run Simscape or calculate torques. Simulink, Simscape, and Simscape Multibody will be required once that stage is integrated.

## Model and parameter files

- `matlab/init_robot_params.m` — shared kinematic, workpiece, path, and IK parameters.
- `matlab/build_welding_robot.m` — creates the seven-revolute `rigidBodyTree` from those parameters.
- `matlab/generate_welding_ik.m` — generates the path, solves IK, records per-sample errors, and plots results.
- `matlab/run_simulation.m` — current project entry point.

All lengths use metres, angles use radians, and time uses seconds. Before making physical torque claims, replace provisional dimensions and joint bounds with the agreed CAD model and add mass, center-of-mass, and inertia data for each link.

## Verification still needed

- Confirm the seven-axis definition and frame conventions against the team CAD/DH model.
- Check every IK sample for position error, orientation error, solver status, and joint-limit violations.
- Add robot and torch collision geometry before claiming link-level collision avoidance; the current lift only raises the TCP above the workpiece plane.
- Add the Simscape model and compare its inverse-dynamics torques with an independent analytical or MATLAB dynamics check.
- Record the MATLAB release and toolbox versions used for the final reproducibility run.

## Team roles

- **Member 1** — Mechanical design, CAD, and URDF/model parameters
- **Member 2** — Simscape assembly and environment
- **Member 3** — Trajectory generation and inverse kinematics
- **Member 4** — Inverse dynamics and torque analysis
- **Member 5** — Verification, visualization, and documentation

## References

- Course material: *Introduction to Robotics (M4609301)* — Week 01 Handout
- MathWorks: [Computing Actuator Torques Using Inverse Dynamics](https://www.mathworks.com/help/sm/ug/computing-actuator-torques-using-inverse-dynamics.html)
- Carbone, G., & Laribi, M. A. (Eds.). *Robot Design: From Theory to Service Applications*. Springer.
