function report = compare_torques(torques, sim_result, params)
%COMPARE_TORQUES Check Simscape torques against MATLAB inverseDynamics.
%   report = COMPARE_TORQUES(torques, sim_result, params) compares
%   torques.total_N_m (compute_joint_torques) with sim_result.tau_N_m
%   (run_simscape_dynamics) joint by joint, and checks that the Simscape
%   torch tip follows the planned path. Prints a table; report.passed is
%   true only if every joint and the TCP check pass. Core MATLAB only.

c = params.checks;
tau_m = torques.total_N_m;
tau_s = sim_result.tau_N_m;
labels = params.robot.joint_labels(:);

peak = max(abs(tau_m), [], 1)';
max_diff = max(abs(tau_s - tau_m), [], 1)';
rms_diff = sqrt(mean((tau_s - tau_m).^2, 1))';
tolerance = max(c.torque_match_abs_N_m, c.torque_match_rel * peak);
passed = max_diff <= tolerance;

report.joints = table(labels, peak, max(abs(tau_s), [], 1)', max_diff, ...
    rms_diff, tolerance, passed, 'VariableNames', {'Joint', ...
    'PeakMATLAB_N_m', 'PeakSimscape_N_m', 'MaxDiff_N_m', 'RMSDiff_N_m', ...
    'Tolerance_N_m', 'Passed'});
report.max_tcp_error_m = max(sim_result.tcp_error_m);
report.tcp_passed = report.max_tcp_error_m <= c.simscape_tcp_error_m;
report.passed = all(passed) && report.tcp_passed;

fprintf('\n=== Simscape vs MATLAB inverse dynamics ===\n');
disp(report.joints);
fprintf('Simscape TCP max deviation from path: %.4f mm (limit %.2f mm)\n', ...
    1e3*report.max_tcp_error_m, 1e3*c.simscape_tcp_error_m);
if report.passed
    fprintf('Simscape and MATLAB dynamics agree.\n');
else
    fprintf('Simscape and MATLAB dynamics DISAGREE; see table above.\n');
end
end
