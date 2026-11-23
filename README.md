# 7-DOF Industrial Welding Robot — Dynamic Modeling & Inverse Dynamics Simulation

**Course:** Introduction to Robotics, NTUST, Fall 2026 (final project)
**Team:** Emmanuel Acosta, Ivan Arce,  Mauricio Olmedo, Rafael Pereira, Ruben Vera

## Project Overview
This repository contains the design, kinematic/dynamic modeling, and simulation files for a **7-degree-of-freedom (7-DOF) robotic manipulator** equipped with a welding torch. Developed as the final team project for **Introduction to Robotics (Course No. M4609301)**, this project uses **MATLAB**, **Simulink**, and **Simscape Multibody** to simulate automated multi-shape welding trajectories and compute joint actuator torque profiles via inverse dynamics.

### Key Objectives
* **Multi-Shape Path Tracing:** Guide the welding torch tip smoothly across three distinct geometric patterns—a **plus sign**, a **circle**, and a **star sign**—on a flat workpiece surface.
* **Collision-Free Transitions:** Implement vertical lift-off and re-engagement trajectories when transitioning between welding shapes to avoid surface contact.
* **Inverse Dynamics Analysis:** Compute the time-varying joint actuator torques required to execute the prescribed welding paths under gravitational and inertial loads.

---

## System Architecture

### Manipulator Configuration (7-DOF)
The serial manipulator provides kinematic redundancy for flexible tool orientation and smooth path execution:
1. **Waist Revolute Joint** (Base rotation)
2. **Shoulder Joint**
3. **Upper Elbow Joint**
4. **Arm Twist Joint**
5. **Wrist Universal Joint (Inboard)**
6. **Wrist Universal Joint (Outboard)**
7. **Tool / End-Effector Interface** (Welding Torch)

### Simscape Multibody & Control Blocks
* **Environment & Constraints:** `Floor` reference frame, `Workpiece` geometry positioned via `Workpiece XForm`, `Solver Configuration` (`f(x) = 0`), and `Mechanism Configuration`.
* **Trajectory Controller:** Generates motion actuation signals for the torch tip across the plus, circle, and star paths, including lift segments between shapes.
* **Torque Instrumentation:** Simulink scopes monitor actuator torque outputs (`Nm`) for the Waist Revolute, Shoulder, Upper Elbow, Arm Twist, and Wrist Universal (Inboard/Outboard) joints.

## Getting Started

### Prerequisites
* **MATLAB** (R2024a or later recommended)
* **Simulink**
* **Simscape™** & **Simscape Multibody™**
* **Robotics System Toolbox™**

### Running the Simulation
1. Clone this repository:
   ```bash
   git clone https://github.com/<your-username>/7-DOF-Welding-Robot.git
   cd 7-DOF-Welding-Robot
   ```
2. Open MATLAB and navigate to the project root folder.
3. Run the initialization and simulation script:
   ```matlab
   run('matlab/run_simulation.m')
   ```
4. Inspect the 3D animation in **Mechanics Explorer** and open the `Actuator Torques` scope in `simulink/welding_robot_7dof.slx` to view joint torque curves.

---

## Simulation Results & Analysis
* **Path Accuracy:** Verify that the welding torch accurately traces the plus, circle, and star geometries on the workpiece plane and cleanly clears the surface during transitions.
* **Actuator Torque Profiles (`Nm`):** Evaluate peak and continuous torque demands across all joints—particularly during sharp directional changes (corners of the plus and star) and vertical lift accelerations.

---

## Team Members
* **Member 1** – Mechanical Design & CAD / URDF Modeling
* **Member 2** – Simscape Multibody Assembly & Environment Setup
* **Member 3** – Trajectory Generation (Plus, Circle, Star & Lift Logic)
* **Member 4** – Inverse Dynamics & Actuator Torque Analysis
* **Member 5** – Simulation Verification, Data Visualization & Documentation
---

## References
* Course Material: *Introduction to Robotics (M4609301)* — Week 01 Handout
* MathWorks Documentation: [Computing Actuator Torques Using Inverse Dynamics](https://www.mathworks.com/help/sm/ug/computing-actuator-torques-using-inverse-dynamics.html)
* Carbone, G., & Laribi, M. A. (Eds.). *Robot Design: From Theory to Service Applications*. Springer.
