function run_core_tests()
%RUN_CORE_TESTS Tests for the helpers that need only core MATLAB.
% Run from the repository root:  run('tests/run_core_tests.m')
% These do not need Robotics System Toolbox or Simscape.

repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repo_root, 'matlab'));
params = init_robot_params();
tol = 1e-12;

%% forward_kinematics_tip: zero pose is a straight vertical arm
T = forward_kinematics_tip(zeros(1, 7), params);
expected_height = sum(params.robot.joint_origins_xyz_m(:, 3)) + ...
    params.robot.weld_tip_offset_xyz_m(3);
assert(norm(T(1:3, 4) - [0; 0; expected_height]) < tol, ...
    'Zero pose tip position is wrong.');
assert(norm(T(1:3, 1:3) - eye(3)) < tol, 'Zero pose orientation is wrong.');

%% forward_kinematics_tip: shoulder +90 deg about Y swings the arm to +X
q = zeros(1, 7);
q(2) = pi/2;
T = forward_kinematics_tip(q, params);
shoulder_z = sum(params.robot.joint_origins_xyz_m(1:2, 3));
arm_length = expected_height - shoulder_z;
assert(norm(T(1:3, 4) - [arm_length; 0; shoulder_z]) < 1e-12, ...
    'Shoulder rotation moves the tip to the wrong place.');

%% forward_kinematics_tip: vectorised over samples
T = forward_kinematics_tip(repmat(q, 3, 1), params);
assert(isequal(size(T), [4 4 3]), 'Expected one transform per sample.');

%% check_path_reachability: the workpiece region is reachable
tilt = params.ik.torch_tilt_rad;
z_tool = [sin(tilt); 0; -cos(tilt)];
y_tool = [0; 1; 0];
R_torch = [cross(y_tool, z_tool), y_tool, z_tool];
wp = params.environment.workpiece_center_m;
tr = params.trajectory;
centers = wp(1:2) + [tr.plus_center_offset_xy_m; ...
    tr.circle_center_offset_xy_m; tr.star_center_offset_xy_m];
half_size = max([tr.plus_half_length_m, tr.circle_radius_m, ...
    tr.star_outer_radius_m]);
corners = [];
for c = 1:size(centers, 1)
    for dx = [-1 1]
        for dy = [-1 1]
            for z = [wp(3), wp(3) + tr.clearance_m]
                corners(end+1, :) = [centers(c, :) + ...
                    half_size*[dx dy], z]; %#ok<AGROW>
            end
        end
    end
end
reach = check_path_reachability(corners, R_torch, params);
assert(reach.ok, 'Workpiece corners should be reachable.');
fprintf('Workpiece corners need up to %.0f%% of max reach.\n', ...
    100*reach.max_reach_ratio);

%% check_path_reachability: a far target is rejected
far = check_path_reachability([2.0 0 0.25], R_torch, params);
assert(~far.ok && far.unreachable, 'A 2 m target should be unreachable.');

%% compute_joint_derivatives: recovers analytic derivatives of a smooth q
dt = params.trajectory.sample_time_s;
t = (0:dt:4)';
w = 1.5;
q_true = repmat(0.4*sin(w*t), 1, 7);
noise = 1e-7*sin(977*t);          % deterministic, IK-sized noise
motion = compute_joint_derivatives(q_true + noise, dt, params);
inner = 20:numel(t)-20;           % ignore one-sided edge differences
qd_err = max(abs(motion.qd(inner, 1) - 0.4*w*cos(w*t(inner))));
qdd_err = max(abs(motion.qdd(inner, 1) + 0.4*w^2*sin(w*t(inner))));
q_err = max(abs(motion.q(:, 1) - q_true(:, 1)));
assert(q_err < 1e-5, 'Smoothing moved q by %.2e rad.', q_err);
assert(qd_err < 1e-3, 'qd error too large: %.2e rad/s.', qd_err);
assert(qdd_err < 1e-2, 'qdd error too large: %.2e rad/s^2.', qdd_err);

%% check_ik_result: clean synthetic result passes
result = synthetic_result(t, q_true, motion);
report = evalc_quiet(@() check_ik_result(result, params));
assert(report.passed, 'Clean synthetic result should pass.');

%% check_ik_result: a joint-limit violation and a branch jump are caught
bad = result;
bad.motion.q(100, 3) = pi;                  % beyond limit - margin
bad.q_rad(200:end, 5) = bad.q_rad(200:end, 5) + 0.5;  % sudden jump
bad.ik_status(10) = "best available";
report = evalc_quiet(@() check_ik_result(bad, params));
assert(~report.passed, 'Bad result should fail.');
failed = report.checks.Check(~report.checks.Passed);
assert(any(failed == "IK solver status"), 'Status failure missed.');
assert(any(failed == "Joints within limits (minus margin)"), ...
    'Limit violation missed.');
assert(any(failed == "Largest joint step between samples"), ...
    'Branch jump missed.');
assert(isequal(report.bad_limit_joints, 3), 'Wrong joint flagged.');

fprintf('All core tests passed.\n');
end

function result = synthetic_result(t, q, motion)
n = numel(t);
result.time_s = t;
result.q_rad = q;
result.motion = motion;
result.ik_status = repmat("success", n, 1);
result.position_error_m = 1e-6*ones(n, 1);
result.orientation_error_rad = 1e-6*ones(n, 1);
result.motion_position_error_m = 1e-6*ones(n, 1);
result.motion_orientation_error_rad = 1e-6*ones(n, 1);
end

function out = evalc_quiet(fn) %#ok<INUSD> used inside the evalc string
% Run fn while hiding its printed tables.
out = [];
evalc('out = fn();');
end
