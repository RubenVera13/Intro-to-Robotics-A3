# 7-DOF Industrial Welding Robot — Dynamic Modeling & Inverse Dynamics Simulation
**Course:** Introduction to Robotics, NTUST, Fall 2026 (final project)
**Team:** Emmanuel Acosta, Ivan Arce,  Mauricio Olmedo, Rafael Pereira, Ruben Vera
## Project Overview
This repository contains the design, kinematic/dynamic modeling, and simulation files for a **7-degree-of-freedom (7-DOF) robotic manipulator** equipped with a welding torch. Developed as the final team project for **Introduction to Robotics (Course No. M4609301)**[cite: 1, 20, 32], this project uses **MATLAB**, **Simulink**, and **Simscape Multibody** to simulate automated multi-shape welding trajectories and compute joint actuator torque profiles via inverse dynamics[cite: 21, 29, 39].

### Key Objectives
* **Multi-Shape Path Tracing:** Guide the welding torch tip smoothly across three distinct geometric patterns—a **plus sign**, a **circle**, and a **star sign**—on a flat workpiece surface.
* **Collision-Free Transitions:** Implement vertical lift-off and re-engagement trajectories when transitioning between welding shapes to avoid surface contact.
* **Inverse Dynamics Analysis:** Compute the time-varying joint actuator torques required to execute the prescribed welding paths under gravitational and inertial loads.

---

## System Architecture

### Manipulator Configuration (7-DOF)
The serial manipulator provides kinematic redundancy for flexible tool orientation and smooth path execution:
1. **Waist Revolute Joint** (Base rotation)
2. **Shoulder Joint**[cite: 39]
3. **Upper Elbow Joint**[cite: 39]
4. **Arm Twist Joint**[cite: 39]
5. **Wrist Universal Joint (Inboard)**[cite: 39]
6. **Wrist Universal Joint (Outboard)**[cite: 39]
7. **Tool / End-Effector Interface** (Welding Torch)[cite: 39]

### Simscape Multibody & Control Blocks
* **Environment & Constraints:** `Floor` reference frame, `Workpiece` geometry positioned via `Workpiece XForm`, `Solver Configuration` (`f(x) = 0`), and `Mechanism Configuration`[cite: 39].
* **Trajectory Controller:** Generates motion actuation signals for the torch tip across the plus, circle, and star paths, including lift segments between shapes[cite: 39].
* **Torque Instrumentation:** Simulink scopes monitor actuator torque outputs (`Nm`) for the Waist Revolute, Shoulder, Upper Elbow, Arm Twist, and Wrist Universal (Inboard/Outboard) joints[cite: 39].

---

## Repository Structure

```text
7-DOF-Welding-Robot/
├── README.md                  # Project documentation, setup instructions, and analysis
├── cad_or_urdf/               # Link geometry files, CAD exports, or URDF definitions
├── matlab/
│   ├── trajectory_gen.m       # Waypoint and trajectory generator (plus, circle, star + lift)
│   ├── inverse_dynamics.m     # Torque post-processing and peak load evaluation
│   └── run_simulation.m       # Master script to initialize parameters and run Simulink
├── simulink/
│   └── welding_robot_7dof.slx # Main Simscape Multibody & Trajectory Controller model
└── results/
    ├── joint_torques.png      # Simulated actuator torque profiles (Nm) vs. time
    └── torch_trajectory.png   # 3D end-effector path showing trace and lift phases

