%% run_simulation.m — Master Execution Pipeline
clear; clc; close all;

% 1. Load robot dimensions, masses, and workpiece coordinates
run('init_robot_params.m');

% 2. Generate Cartesian trajectories (Plus -> Lift -> Circle -> Lift -> Star)
[traj_pose, time_vec] = generate_welding_trajectories(wp_pos, dt);

% 3. Solve 7-DOF Inverse Kinematics for continuous joint trajectories q(t)
[q_traj, qd_traj, qdd_traj] = solve_7dof_ik(robot_rigidBodyTree, traj_pose, time_vec);

% 4. Run Simscape Multibody inverse dynamics simulation & plot torques
simOut = sim('simulink/welding_robot_7dof.slx', 'StopTime', num2str(time_vec(end)));
plot_results(simOut, traj_pose);
