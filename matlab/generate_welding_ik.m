function result = generate_welding_ik()
%GENERATE_WELDING_IK Generate the path and solve the provisional 7R IK.
% Requires Robotics System Toolbox. This is a kinematic prototype only;
% it does not compute link dynamics or actuator torques.

matlab_dir = fileparts(mfilename('fullpath'));
addpath(matlab_dir);

params = init_robot_params();
robot = build_welding_robot(params);
dt = params.trajectory.sample_time_s;
wp_pos = params.environment.workpiece_center_m;
z_weld = wp_pos(3);
z_lift = z_weld + params.trajectory.clearance_m;

%% Build plus, circle, and star waypoints
c_plus = wp_pos(1:2) + params.trajectory.plus_center_offset_xy_m;
c_circle = wp_pos(1:2) + params.trajectory.circle_center_offset_xy_m;
c_star = wp_pos(1:2) + params.trajectory.star_center_offset_xy_m;

plus_half_length = params.trajectory.plus_half_length_m;
plus_h = [c_plus(1)-plus_half_length, c_plus(2), z_weld; ...
          c_plus(1)+plus_half_length, c_plus(2), z_weld];
plus_v = [c_plus(1), c_plus(2)-plus_half_length, z_weld; ...
          c_plus(1), c_plus(2)+plus_half_length, z_weld];

circle_radius = params.trajectory.circle_radius_m;
p_circle_start = [c_circle(1)+circle_radius, c_circle(2), z_weld];

star_outer_radius = params.trajectory.star_outer_radius_m;
star_inner_radius = star_outer_radius * params.trajectory.star_inner_ratio;
star_pts = zeros(11, 3);
for k = 0:4
    angle_outer = pi/2 + k*(2*pi/5);
    angle_inner = angle_outer + pi/5;
    star_pts(2*k+1, :) = [c_star(1)+star_outer_radius*cos(angle_outer), ...
        c_star(2)+star_outer_radius*sin(angle_outer), z_weld];
    star_pts(2*k+2, :) = [c_star(1)+star_inner_radius*cos(angle_inner), ...
        c_star(2)+star_inner_radius*sin(angle_inner), z_weld];
end
star_pts(11, :) = star_pts(1, :);

%% Build C2 quintic path segments
% Each segment ends at zero velocity and acceleration. This makes corners
% gentle but pauses the TCP at every waypoint, including every star vertex.
pos_traj = zeros(0, 3);
time_vec = zeros(0, 1);
t_curr = 0;
stroke_duration = params.trajectory.stroke_duration_s;
lift_duration = params.trajectory.lift_duration_s;
transit_duration = params.trajectory.transit_duration_s;

% Approach and horizontal plus stroke.
wpts_1 = [plus_h(1,1), plus_h(1,2), z_lift; ...
          plus_h(1,:); ...
          plus_h(2,:); ...
          plus_h(2,1), plus_h(2,2), z_lift];
[pos_traj, time_vec, t_curr] = addLinearSegments(pos_traj, time_vec, ...
    t_curr, wpts_1, [lift_duration, stroke_duration, lift_duration], dt);

% Lift between the two plus strokes. The strokes intentionally overlap at
% the center of the plus, as both full-length lines pass through that point.
wpts_2 = [pos_traj(end,:); ...
          plus_v(1,1), plus_v(1,2), z_lift; ...
          plus_v(1,:); ...
          plus_v(2,:); ...
          plus_v(2,1), plus_v(2,2), z_lift];
[pos_traj, time_vec, t_curr] = addLinearSegments(pos_traj, time_vec, ...
    t_curr, wpts_2, [lift_duration, lift_duration, stroke_duration, ...
    lift_duration], dt);

% Lift and transit to the circle, then plunge.
wpts_3 = [pos_traj(end,:); ...
          p_circle_start(1), p_circle_start(2), z_lift; ...
          p_circle_start];
[pos_traj, time_vec, t_curr] = addLinearSegments(pos_traj, time_vec, ...
    t_curr, wpts_3, [transit_duration, lift_duration], dt);

% Circle with a quintic angular profile.
circle_duration = params.trajectory.circle_duration_s;
t_circle = (dt:dt:circle_duration)';
[theta, ~, ~] = quinticpolytraj([0, 2*pi], [0, circle_duration], t_circle);
circle_xyz = [c_circle(1) + circle_radius*cos(theta)', ...
              c_circle(2) + circle_radius*sin(theta)', ...
              repmat(z_weld, numel(t_circle), 1)];
pos_traj = [pos_traj; circle_xyz];
time_vec = [time_vec; t_curr + t_circle];
t_curr = time_vec(end);

% Retract from circle, transit to star, trace its perimeter, and retract.
star_edge_duration = params.trajectory.star_edge_duration_s;
wpts_4 = [pos_traj(end,:); ...
          p_circle_start(1), p_circle_start(2), z_lift; ...
          star_pts(1,1), star_pts(1,2), z_lift; ...
          star_pts; ...
          star_pts(1,1), star_pts(1,2), z_lift];
durations_4 = [lift_duration, transit_duration, lift_duration, ...
    repmat(star_edge_duration, 1, 10), lift_duration];
[pos_traj, time_vec, ~] = addLinearSegments(pos_traj, time_vec, ...
    t_curr, wpts_4, durations_4, dt);

%% Solve sequential inverse kinematics
tilt = params.ik.torch_tilt_rad;
z_tool = [sin(tilt); 0; -cos(tilt)];
y_tool = [0; 1; 0];
x_tool = cross(y_tool, z_tool);
R_torch = [x_tool, y_tool, z_tool];

ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverAlgorithm = 'LevenbergMarquardt';
weights = params.ik.weights;

num_steps = numel(time_vec);
q_traj = zeros(num_steps, numel(params.robot.joint_names));
pose_error_norm = zeros(num_steps, 1);
position_error_m = zeros(num_steps, 1);
orientation_error_rad = zeros(num_steps, 1);
ik_status = strings(num_steps, 1);
ik_exit_flag = zeros(num_steps, 1);
q_guess = params.ik.initial_guess_rad;

fprintf('Solving IK for %d path samples...\n', num_steps);
for k = 1:num_steps
    target = [R_torch, pos_traj(k,:)'; 0 0 0 1];
    [q_sol, solution_info] = ik('weld_tip', target, weights, q_guess);
    q_traj(k, :) = q_sol;
    q_guess = q_sol;

    actual = getTransform(robot, q_sol, 'weld_tip');
    position_error_m(k) = norm(actual(1:3, 4) - pos_traj(k,:)');
    relative_rotation = R_torch' * actual(1:3, 1:3);
    cosine_angle = (trace(relative_rotation) - 1)/2;
    orientation_error_rad(k) = acos(max(-1, min(1, cosine_angle)));
    pose_error_norm(k) = solution_info.PoseErrorNorm;
    ik_status(k) = string(solution_info.Status);
    ik_exit_flag(k) = solution_info.ExitFlag;
end

if any(ik_status ~= "success")
    warning('IK returned a non-success status for %d of %d samples.', ...
        nnz(ik_status ~= "success"), num_steps);
end

max_joint_step_rad = max(abs(diff(q_traj, 1, 1)), [], 1);
fprintf('Maximum position error: %.3f mm\n', 1e3*max(position_error_m));
fprintf('Maximum orientation error: %.3f deg\n', ...
    rad2deg(max(orientation_error_rad)));
fprintf('Maximum sample-to-sample joint step: %.3f deg\n', ...
    rad2deg(max(max_joint_step_rad)));

%% Export data for downstream Simulink integration
q_ts = struct();
for j = 1:numel(params.robot.joint_labels)
    label = params.robot.joint_labels{j};
    q_ts.(label) = timeseries(q_traj(:, j), time_vec, 'Name', label);
end

%% Plot the tool path and joint angles
figure('Name', '7R Welding-Path Kinematic Prototype', ...
    'Position', [100 100 1100 450]);

subplot(1, 2, 1);
at_workpiece_height = abs(pos_traj(:, 3) - z_weld) < 1e-4;
plot3(pos_traj(:, 1), pos_traj(:, 2), pos_traj(:, 3), '--', ...
    'Color', [0.6 0.6 0.6], 'LineWidth', 1.2);
hold on;
contact_height_pts = pos_traj;
contact_height_pts(~at_workpiece_height, :) = NaN;
plot3(contact_height_pts(:, 1), contact_height_pts(:, 2), ...
    contact_height_pts(:, 3), 'r-', 'LineWidth', 2.5);
grid on;
axis equal;
view(35, 30);
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('TCP path (red = workpiece height)');
legend('Full TCP path', 'At workpiece height', 'Location', 'best');

subplot(1, 2, 2);
plot(time_vec, q_traj, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Joint angle [rad]');
title('Seven revolute joint trajectories');
legend(params.robot.joint_labels, 'Location', 'bestoutside');

result.params = params;
result.robot = robot;
result.position_m = pos_traj;
result.time_s = time_vec;
result.q_rad = q_traj;
result.q_ts = q_ts;
result.pose_error_norm = pose_error_norm;
result.position_error_m = position_error_m;
result.orientation_error_rad = orientation_error_rad;
result.ik_status = ik_status;
result.ik_exit_flag = ik_exit_flag;
result.max_joint_step_rad = max_joint_step_rad;
end

function [pos_out, time_out, t_end] = addLinearSegments( ...
    pos_in, time_in, t_start, waypoints, durations, dt)
segment_count = size(waypoints, 1) - 1;
if numel(durations) ~= segment_count
    error('Expected one duration for each segment between waypoints.');
end

pos_out = pos_in;
time_out = time_in;
t_end = t_start;
for segment = 1:segment_count
    t_segment = (dt:dt:durations(segment))';
    if isempty(pos_out) && segment == 1
        t_segment = (0:dt:durations(segment))';
    end
    [p_segment, ~, ~] = quinticpolytraj( ...
        waypoints(segment:segment+1, :)', ...
        [0, durations(segment)], t_segment);
    pos_out = [pos_out; p_segment']; %#ok<AGROW>
    time_out = [time_out; t_end + t_segment]; %#ok<AGROW>
    t_end = time_out(end);
end
end
