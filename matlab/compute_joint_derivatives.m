function motion = compute_joint_derivatives(q, dt, params)
%COMPUTE_JOINT_DERIVATIVES Smooth joint positions and differentiate twice.
%   motion = COMPUTE_JOINT_DERIVATIVES(q, dt, params) returns a struct with
%   q, qd, and qdd (N-by-7, rad, rad/s, rad/s^2) sampled every dt seconds.
%
%   Simscape inverse dynamics needs the joint position and its first two
%   derivatives. Differentiating raw IK output twice amplifies solver noise
%   by 1/dt^2, so the positions are first smoothed with a short
%   Savitzky-Golay window (core MATLAB smoothdata) and the exported q is
%   the smoothed one, keeping q, qd, and qdd mutually consistent.
%   Check the smoothed q against the path with forward_kinematics_tip.

window = params.motion.smoothing_window_samples;
if window > 1 && size(q, 1) >= window
    q_s = smoothdata(q, 1, params.motion.smoothing_method, window);
else
    q_s = q;
end

motion.q = q_s;
motion.qd = gradient_columns(q_s, dt);
motion.qdd = gradient_columns(motion.qd, dt);
motion.smoothing_window_samples = window;
end

function dx = gradient_columns(x, dt)
% Central differences inside, one-sided at the ends, per column.
dx = zeros(size(x));
for j = 1:size(x, 2)
    dx(:, j) = gradient(x(:, j), dt);
end
end
