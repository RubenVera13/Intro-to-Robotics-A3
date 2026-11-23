function params = init_robot_params()
%INIT_ROBOT_PARAMS Shared parameters for the kinematic welding-robot prototype.
% All lengths are in metres and all angles are in radians.
%
% Joint limits and link dimensions below are provisional kinematic values.
% Replace them with the agreed CAD/DH model before reporting physical or
% actuator-level results. No mass or inertia data is defined yet.

params.model.data_format = 'row';

params.robot.joint_labels = {'Waist', 'Shoulder', 'UpperElbow', ...
    'ArmTwist', 'WristInboard', 'WristOutboard', 'ToolInterface'};
params.robot.body_names = {'link1_waist', 'link2_shoulder', ...
    'link3_elbow', 'link4_twist', 'link5_wrist_in', ...
    'link6_wrist_out', 'link7_tool'};
params.robot.joint_names = {'j1_waist', 'j2_shoulder', ...
    'j3_upper_elbow', 'j4_arm_twist', 'j5_wrist_inboard', ...
    'j6_wrist_outboard', 'j7_tool_interface'};
params.robot.parent_names = {'base', 'link1_waist', 'link2_shoulder', ...
    'link3_elbow', 'link4_twist', 'link5_wrist_in', 'link6_wrist_out'};

% Seven independently actuated revolute axes: Z-Y-Y-Z-Y-X-Z.
params.robot.joint_axes = [
    0 0 1
    0 1 0
    0 1 0
    0 0 1
    0 1 0
    1 0 0
    0 0 1];

% Transform from each parent body to its joint frame, expressed as xyz [m].
params.robot.joint_origins_xyz_m = [
    0 0 0.30
    0 0 0.00
    0 0 0.35
    0 0 0.35
    0 0 0.00
    0 0 0.00
    0 0 0.00];
params.robot.weld_tip_offset_xyz_m = [0 0 0.15];

% Provisional mathematical bounds only; replace with design-specific stops.
params.robot.joint_limits_rad = repmat([-pi pi], 7, 1);
params.robot.joint_limits_status = 'provisional +/- pi bounds';
params.robot.inertial_properties_status = ...
    'not defined; kinematic model only, no torque claims';

params.environment.gravity_m_s2 = [0 0 -9.81];
params.environment.workpiece_center_m = [0.55 0.00 0.25];

params.trajectory.sample_time_s = 0.01;
params.trajectory.clearance_m = 0.05;
params.trajectory.plus_center_offset_xy_m = [-0.08 -0.14];
params.trajectory.circle_center_offset_xy_m = [0.06 0.00];
params.trajectory.star_center_offset_xy_m = [-0.08 0.14];
params.trajectory.plus_half_length_m = 0.045;
params.trajectory.circle_radius_m = 0.045;
params.trajectory.star_outer_radius_m = 0.055;
params.trajectory.star_inner_ratio = 0.382;
params.trajectory.circle_duration_s = 3.5;
params.trajectory.stroke_duration_s = 1.5;
params.trajectory.lift_duration_s = 0.8;
params.trajectory.transit_duration_s = 1.2;
params.trajectory.star_edge_duration_s = 0.7;

params.ik.torch_tilt_rad = deg2rad(15);
% inverseKinematics order: orientation-error components, then xyz position.
params.ik.weights = [0.1 0.5 0.5 1.0 1.0 1.0];
params.ik.initial_guess_rad = [0.0 -0.35 0.85 0.0 0.75 0.0 0.0];
% Random restarts can jump to a different arm configuration mid-path, which
% shows up as torque spikes later. Keep them off for sequential tracking.
params.ik.allow_random_restart = false;
params.ik.max_iterations = 1500;
params.ik.solution_tolerance = 1e-6;   % MATLAB default

% Joint-motion export for Simscape (q, qd, qdd). The window is in samples;
% set it to 1 to disable smoothing and differentiate the raw IK output.
params.motion.smoothing_method = 'sgolay';
params.motion.smoothing_window_samples = 9;

% Pass/fail thresholds used by check_ik_result.m.
params.checks.max_position_error_m = 0.5e-3;
params.checks.max_orientation_error_rad = deg2rad(1.0);
params.checks.joint_limit_margin_rad = deg2rad(2.0);
% A larger step between consecutive samples (dt = sample_time_s) usually
% means the IK switched configuration branch.
params.checks.max_joint_step_rad = deg2rad(3.0);
end
