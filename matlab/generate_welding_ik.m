%% generate_welding_ik.m
% Generates smooth Cartesian trajectories (Plus -> Lift -> Circle -> Lift -> Star)
% and continuous 7-DOF inverse kinematics joint trajectories q(t).
% Requires: Robotics System Toolbox

clear; clc; close all;

%% 1. Build 7-DOF Manipulator Model (Replace dimensions/URDF with Team CAD)
robot = build7DOFWeldingRobot();

%% 2. Workpiece & Trajectory Parameters
dt      = 0.01;                  % Sample time [s] (100 Hz)
wp_pos  = [0.55, 0.00, 0.25];    % Workpiece top surface center [x, y, z] in [m]
z_weld  = wp_pos(3);             % Contact welding height [m]
z_lift  = z_weld + 0.05;         % 50 mm vertical clearance height [m]

% Shape centers on the workpiece plane
c_plus   = [wp_pos(1) - 0.08, wp_pos(2) - 0.14];
c_circle = [wp_pos(1) + 0.06, wp_pos(2) + 0.00];
c_star   = [wp_pos(1) - 0.08, wp_pos(2) + 0.14];

% --- Waypoint Definitions ---
% A. Plus Sign (2 separate strokes with a Z-lift to avoid double-welding center)
L_plus = 0.045; % Half-length of plus arms [m]
plus_h = [c_plus(1)-L_plus, c_plus(2), z_weld;
          c_plus(1)+L_plus, c_plus(2), z_weld];
plus_v = [c_plus(1), c_plus(2)-L_plus, z_weld;
          c_plus(1), c_plus(2)+L_plus, z_weld];

% B. Circle Parameters
R_circ = 0.045; % Circle radius [m]
p_circ_start = [c_circle(1) + R_circ, c_circle(2), z_weld];

% C. Star Sign (10 perimeter segments: alternating outer & inner vertices)
R_outer = 0.055;
R_inner = R_outer * 0.382; % Classic regular pentagram inner-to-outer ratio
star_pts = zeros(11, 3);
for k = 0:4
    ang_out = pi/2 + k * (2*pi/5);
    ang_in  = ang_out + pi/5;
    star_pts(2*k + 1, :) = [c_star(1) + R_outer*cos(ang_out), c_star(2) + R_outer*sin(ang_out), z_weld];
    star_pts(2*k + 2, :) = [c_star(1) + R_inner*cos(ang_in),  c_star(2) + R_inner*sin(ang_in),  z_weld];
end
star_pts(11, :) = star_pts(1, :); % Close the star perimeter

%% 3. Generate C2-Smooth Cartesian Trajectory Segments
% Using quinticpolytraj ensures velocity and acceleration start and end at 0
% at every corner, preventing infinite torque spikes in inverse dynamics.
pos_traj = [];
time_vec = [];
t_curr   = 0;

% Helper to append piecewise-linear quintic segments
appendLinear = @(pts, seg_dur) deal(); % Pre-declared for clarity

% 1) Approach & weld Plus horizontal stroke
wpts_1 = [plus_h(1,1), plus_h(1,2), z_lift;
          plus_h(1,:);
          plus_h(2,:);
          plus_h(2,1), plus_h(2,2), z_lift];
[pos_traj, time_vec, t_curr] = addLinearSegments(pos_traj, time_vec, t_curr, wpts_1, [0.8, 1.5, 0.8], dt);

% 2) Transition & weld Plus vertical stroke
wpts_2 = [pos_traj(end,:);
          plus_v(1,1), plus_v(1,2), z_lift;
          plus_v(1,:);
          plus_v(2,:);
          plus_v(2,1), plus_v(2,2), z_lift];
[pos_traj, time_vec, t_curr] = addLinearSegments(pos_traj, time_vec, t_curr, wpts_2, [0.8, 0.8, 1.5, 0.8], dt);

% 3) Transition to Circle start & plunge
wpts_3 = [pos_traj(end,:);
          p_circ_start(1), p_circ_start(2), z_lift;
          p_circ_start];
[pos_traj, time_vec, t_curr] = addLinearSegments(pos_traj, time_vec, t_curr, wpts_3, [1.2, 0.8], dt);

% 4) Continuous smooth Circle trace (quintic angular profile)
T_circ = 3.5;
t_seg  = (dt:dt:T_circ)';
[theta, ~, ~] = quinticpolytraj([0, 2*pi], [0, T_circ], t_seg);
circ_xyz = [c_circle(1) + R_circ*cos(theta)', ...
            c_circle(2) + R_circ*sin(theta)', ...
            repmat(z_weld, length(t_seg), 1)];
pos_traj = [pos_traj; circ_xyz];
time_vec = [time_vec; t_curr + t_seg];
t_curr   = time_vec(end);

% 5) Lift from Circle, transit to Star, plunge, trace 10-edge Star, and retract
wpts_4 = [pos_traj(end,:);
          p_circ_start(1), p_circ_start(2), z_lift;
          star_pts(1,1),   star_pts(1,2),   z_lift;
          star_pts;
          star_pts(1,1),   star_pts(1,2),   z_lift];
durs_4 = [0.8, 1.2, 0.8, repmat(0.7, 1, 10), 0.8];
[pos_traj, time_vec, ~] = addLinearSegments(pos_traj, time_vec, t_curr, wpts_4, durs_4, dt);

%% 4. Define Torch Orientation (15-deg Travel Tilt to Avoid Wrist Gimbal Lock)
tilt = deg2rad(15);
z_tool = [ sin(tilt); 0; -cos(tilt)]; % Torch nozzle points down into workpiece
y_tool = [ 0;         1;  0        ];
x_tool = cross(y_tool, z_tool);
R_torch = [x_tool, y_tool, z_tool];

%% 5. Solve Sequential 7-DOF Inverse Kinematics
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverAlgorithm = 'LevenbergMarquardt';

% Weights: [roll, pitch, yaw, x, y, z]
% Relaxing torch roll (0.1) lets the 7-DOF redundancy resolve smoothly
weights = [0.1, 0.5, 0.5, 1.0, 1.0, 1.0];

num_steps = length(time_vec);
q_traj    = zeros(num_steps, 7);

% Bent-elbow initial configuration (away from workspace boundary singularities)
q_guess = [0.0, -0.35, 0.85, 0.0, 0.75, 0.0, 0.0];

fprintf('Solving 7-DOF Inverse Kinematics for %d waypoints...\n', num_steps);
for k = 1:num_steps
    T_target = [R_torch, pos_traj(k,:)'; 0 0 0 1];
    [q_sol, sol_info] = ik('weld_tip', T_target, weights, q_guess);
    q_traj(k, :) = q_sol;
    q_guess      = q_sol; % Warm-start next step for configuration continuity
end
fprintf('IK complete. Final position error: %.3e m\n', sol_info.PoseErrorNorm);

%% 6. Export Timeseries for Simscape 'From Workspace' Blocks
% Connect each q_ts(i) to the corresponding Simulink-PS Converter (Unit: rad)
joint_names = {'Waist', 'Shoulder', 'UpperElbow', 'ArmTwist', ...
               'WristInboard', 'WristOutboard', 'ToolInterface'};
for j = 1:7
    q_ts.(joint_names{j}) = timeseries(q_traj(:, j), time_vec, 'Name', joint_names{j});
end

%% 7. Visual Verification Plots
figure('Name', '7-DOF Welding Trajectory & IK Verification', 'Position', [100 100 1100 450]);

% Subplot 1: 3D Cartesian Path (Weld Contact vs. Lift Clearance)
subplot(1, 2, 1);
is_welding = abs(pos_traj(:,3) - z_weld) < 1e-4;
plot3(pos_traj(:,1), pos_traj(:,2), pos_traj(:,3), '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.2); hold on;
weld_pts = pos_traj; weld_pts(~is_welding, :) = NaN;
plot3(weld_pts(:,1), weld_pts(:,2), weld_pts(:,3), 'r-', 'LineWidth', 2.5);
grid on; axis equal; view(35, 30);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Torch Tip Path (Red = Weld, Dashed = Lift)');
legend('Lift / Transit', 'Active Weld Path', 'Location', 'best');

% Subplot 2: Joint Angle Trajectories q(t)
subplot(1, 2, 2);
plot(time_vec, q_traj, 'LineWidth', 1.4);
grid on;
xlabel('Time [s]'); ylabel('Joint Angle [rad]');
title('7-DOF Joint Trajectories q(t)');
legend(joint_names, 'Location', 'bestoutside');

%% --- Local Helper Functions ---
function [pos_out, t_out, t_end] = addLinearSegments(pos_in, t_in, t_start, wpts, durations, dt)
    pos_out = pos_in;
    t_out   = t_in;
    t_end   = t_start;
    for s = 1:size(wpts, 1) - 1
        t_seg = (dt:dt:durations(s))';
        if isempty(pos_out) && s == 1
            t_seg = (0:dt:durations(s))';
        end
        [p_seg, ~, ~] = quinticpolytraj([wpts(s,:)', wpts(s+1,:)'], [0, durations(s)], t_seg);
        pos_out = [pos_out; p_seg'];
        t_out   = [t_out; t_end + t_seg];
        t_end   = t_out(end);
    end
end

function robot = build7DOFWeldingRobot()
    % Constructs a 7-DOF serial manipulator matching the project architecture
    robot = rigidBodyTree('DataFormat', 'row');
    
    % Link lengths [m] — adjust to match your CAD / DH table
    L_base  = 0.30;
    L_upper = 0.35;
    L_fore  = 0.35;
    L_torch = 0.15;

    % 1. Waist Revolute Joint (Z-axis)
    b1 = rigidBody('link1_waist');
    j1 = rigidBodyJoint('j1_waist', 'revolute');
    j1.JointAxis = [0 0 1];
    setFixedTransform(j1, trvec2tform([0 0 L_base]));
    b1.Joint = j1; addBody(robot, b1, 'base');

    % 2. Shoulder Joint (Y-axis)
    b2 = rigidBody('link2_shoulder');
    j2 = rigidBodyJoint('j2_shoulder', 'revolute');
    j2.JointAxis = [0 1 0];
    b2.Joint = j2; addBody(robot, b2, 'link1_waist');

    % 3. Upper Elbow Joint (Y-axis)
    b3 = rigidBody('link3_elbow');
    j3 = rigidBodyJoint('j3_upper_elbow', 'revolute');
    j3.JointAxis = [0 1 0];
    setFixedTransform(j3, trvec2tform([0 0 L_upper]));
    b3.Joint = j3; addBody(robot, b3, 'link2_shoulder');

    % 4. Arm Twist Joint (Z-axis along forearm)
    b4 = rigidBody('link4_twist');
    j4 = rigidBodyJoint('j4_arm_twist', 'revolute');
    j4.JointAxis = [0 0 1];
    setFixedTransform(j4, trvec2tform([0 0 L_fore]));
    b4.Joint = j4; addBody(robot, b4, 'link3_elbow');

    % 5. Wrist Universal Joint - Inboard (Y-axis)
    b5 = rigidBody('link5_wrist_in');
    j5 = rigidBodyJoint('j5_wrist_inboard', 'revolute');
    j5.JointAxis = [0 1 0];
    b5.Joint = j5; addBody(robot, b5, 'link4_twist');

    % 6. Wrist Universal Joint - Outboard (X-axis)
    b6 = rigidBody('link6_wrist_out');
    j6 = rigidBodyJoint('j6_wrist_outboard', 'revolute');
    j6.JointAxis = [1 0 0];
    b6.Joint = j6; addBody(robot, b6, 'link5_wrist_in');

    % 7. Tool / End-Effector Interface (Z-axis roll)
    b7 = rigidBody('link7_tool');
    j7 = rigidBodyJoint('j7_tool_interface', 'revolute');
    j7.JointAxis = [0 0 1];
    b7.Joint = j7; addBody(robot, b7, 'link6_wrist_out');

    % Fixed Welding Torch Tip Frame
    tip = rigidBody('weld_tip');
    j_tip = rigidBodyJoint('j_weld_tip', 'fixed');
    setFixedTransform(j_tip, trvec2tform([0 0 L_torch]));
    tip.Joint = j_tip; addBody(robot, tip, 'link7_tool');
end
