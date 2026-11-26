function torques = compute_joint_torques(robot, motion)
%COMPUTE_JOINT_TORQUES Joint torques from MATLAB rigid-body dynamics.
%   torques = COMPUTE_JOINT_TORQUES(robot, motion) evaluates
%   inverseDynamics for every sample of motion.q / qd / qdd (N-by-7) and
%   returns:
%     torques.total_N_m     N-by-7, required actuator torque
%     torques.gravity_N_m   N-by-7, torque to hold the same pose at rest
%     torques.dynamic_N_m   N-by-7, total minus gravity (inertial and
%                           velocity-dependent terms)
%   This is the independent check for the Simscape model.
%   Requires Robotics System Toolbox.

num_samples = size(motion.q, 1);
num_joints = size(motion.q, 2);
total = zeros(num_samples, num_joints);
gravity = zeros(num_samples, num_joints);
for k = 1:num_samples
    total(k, :) = inverseDynamics(robot, motion.q(k, :), ...
        motion.qd(k, :), motion.qdd(k, :));
    gravity(k, :) = gravityTorque(robot, motion.q(k, :));
end

torques.total_N_m = total;
torques.gravity_N_m = gravity;
torques.dynamic_N_m = total - gravity;
torques.peak_abs_N_m = max(abs(total), [], 1);
torques.rms_N_m = sqrt(mean(total.^2, 1));
end
