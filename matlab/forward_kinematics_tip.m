function T_tip = forward_kinematics_tip(q, params)
%FORWARD_KINEMATICS_TIP Torch-tip pose from joint angles, core MATLAB only.
%   T_tip = FORWARD_KINEMATICS_TIP(q, params) returns a 4x4xN array of
%   base-to-weld_tip transforms for the N-by-7 joint matrix q [rad].
%
%   This mirrors build_welding_robot.m (fixed translation to each joint
%   frame, then a rotation about the joint axis) without needing Robotics
%   System Toolbox, so it doubles as an independent check of the
%   rigidBodyTree model and of smoothed joint trajectories.

origins = params.robot.joint_origins_xyz_m;
axes_ = params.robot.joint_axes;
tip = params.robot.weld_tip_offset_xyz_m(:);
num_joints = size(axes_, 1);

if size(q, 2) ~= num_joints
    error('forward_kinematics_tip:size', ...
        'Expected q to have %d columns, got %d.', num_joints, size(q, 2));
end

num_samples = size(q, 1);
T_tip = zeros(4, 4, num_samples);
for n = 1:num_samples
    T = eye(4);
    for k = 1:num_joints
        T_origin = [eye(3), origins(k, :)'; 0 0 0 1];
        T_joint = [axis_rotation(axes_(k, :), q(n, k)), zeros(3, 1); ...
                   0 0 0 1];
        T = T * T_origin * T_joint;
    end
    T_tip(:, :, n) = T * [eye(3), tip; 0 0 0 1];
end
end

function R = axis_rotation(axis_vec, angle)
% Rodrigues' formula for a rotation of angle [rad] about axis_vec.
u = axis_vec(:) / norm(axis_vec);
K = [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0];
R = eye(3) + sin(angle)*K + (1 - cos(angle))*(K*K);
end
