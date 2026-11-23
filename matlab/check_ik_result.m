function report = check_ik_result(result, params)
%CHECK_IK_RESULT Pass/fail verification of the IK and exported motion.
%   report = CHECK_IK_RESULT(result, params) checks the struct returned by
%   generate_welding_ik against the thresholds in params.checks and prints
%   a summary. Core MATLAB only.
%
%   report.checks is a table with one row per check; report.passed is true
%   only if every check passed. report.peak_qd_rad_s and
%   report.peak_qdd_rad_s give per-joint peaks for actuator discussion.

c = params.checks;
num_samples = numel(result.time_s);
limits = params.robot.joint_limits_rad;
margin = c.joint_limit_margin_rad;

names = strings(0, 1);
values = strings(0, 1);
limits_txt = strings(0, 1);
passed = false(0, 1);

% 1. Solver status.
num_failed = nnz(result.ik_status ~= "success");
add('IK solver status', sprintf('%d of %d failed', num_failed, ...
    num_samples), '0 failed', num_failed == 0);

% 2-3. Pose error of the raw IK solution.
max_pos = max(result.position_error_m);
add('IK position error', sprintf('%.4f mm', 1e3*max_pos), ...
    sprintf('<= %.3f mm', 1e3*c.max_position_error_m), ...
    max_pos <= c.max_position_error_m);
max_ori = max(result.orientation_error_rad);
add('IK orientation error', sprintf('%.4f deg', rad2deg(max_ori)), ...
    sprintf('<= %.2f deg', rad2deg(c.max_orientation_error_rad)), ...
    max_ori <= c.max_orientation_error_rad);

% 4-5. Pose error of the smoothed motion that Simscape will receive.
max_pos_m = max(result.motion_position_error_m);
add('Exported motion position error', sprintf('%.4f mm', 1e3*max_pos_m), ...
    sprintf('<= %.3f mm', 1e3*c.max_position_error_m), ...
    max_pos_m <= c.max_position_error_m);
max_ori_m = max(result.motion_orientation_error_rad);
add('Exported motion orientation error', ...
    sprintf('%.4f deg', rad2deg(max_ori_m)), ...
    sprintf('<= %.2f deg', rad2deg(c.max_orientation_error_rad)), ...
    max_ori_m <= c.max_orientation_error_rad);

% 6. Joint limits, with a safety margin.
q = result.motion.q;
low = q < limits(:, 1)' + margin;
high = q > limits(:, 2)' - margin;
bad_joints = find(any(low | high, 1));
if isempty(bad_joints)
    limit_value = 'none';
else
    limit_value = strjoin(params.robot.joint_labels(bad_joints), ', ');
end
add('Joints within limits (minus margin)', limit_value, ...
    sprintf('margin %.1f deg', rad2deg(margin)), isempty(bad_joints));

% 7. Configuration jumps between consecutive raw IK samples.
step = max(abs(diff(result.q_rad, 1, 1)), [], 1);
[max_step, worst_joint] = max(step);
add('Largest joint step between samples', sprintf('%.3f deg (%s)', ...
    rad2deg(max_step), params.robot.joint_labels{worst_joint}), ...
    sprintf('<= %.1f deg', rad2deg(c.max_joint_step_rad)), ...
    max_step <= c.max_joint_step_rad);

report.checks = table(names, values, limits_txt, passed, ...
    'VariableNames', {'Check', 'Value', 'Threshold', 'Passed'});
report.passed = all(passed);
report.bad_limit_joints = bad_joints;
report.peak_qd_rad_s = max(abs(result.motion.qd), [], 1);
report.peak_qdd_rad_s = max(abs(result.motion.qdd), [], 1);

fprintf('\n=== IK verification ===\n');
disp(report.checks);
peaks = table(params.robot.joint_labels(:), ...
    rad2deg(report.peak_qd_rad_s(:)), rad2deg(report.peak_qdd_rad_s(:)), ...
    'VariableNames', {'Joint', 'PeakVel_deg_s', 'PeakAcc_deg_s2'});
disp(peaks);
if report.passed
    fprintf('All IK checks passed.\n');
else
    fprintf('%d of %d IK checks FAILED.\n', nnz(~passed), numel(passed));
end

    function add(name, value, threshold, ok)
        names(end+1, 1) = string(name);
        values(end+1, 1) = string(value);
        limits_txt(end+1, 1) = string(threshold);
        passed(end+1, 1) = ok;
    end
end
