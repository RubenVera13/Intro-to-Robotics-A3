%% run_simulation.m — Project entry point
% 1. Path generation and IK, with verification (check_ik_result).
% 2. Joint torques from MATLAB inverseDynamics (compute_joint_torques).
% 3. If Simscape Multibody is available: build/refresh the Simscape model,
%    simulate it with the same motion, and compare torques (compare_torques).
% Results are saved in results/. Mass properties are placeholders until the
% CAD values are entered, so the torques are not report results yet.

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
results_dir = fullfile(script_dir, '..', 'results');
simulink_dir = fullfile(script_dir, '..', 'simulink');

project_result = generate_welding_ik();
params = project_result.params;
ik_report = check_ik_result(project_result, params);

% Variables the Simscape model reads (also handy in the workspace).
joint_motion_ts = project_result.joint_motion_ts;
motion = project_result.motion;
time_s = project_result.time_s;
joint_labels = params.robot.joint_labels;

%% Inverse dynamics in MATLAB
mass_props = link_mass_properties(params);
fprintf('\nMass properties: %s, total %.2f kg\n', mass_props.source, ...
    mass_props.total_mass_kg);
torques = compute_joint_torques(project_result.robot, motion);

%% Simscape inverse dynamics
sim_result = [];
dynamics_report = [];
has_simscape = params.simscape.enabled && ...
    exist('sm_lib', 'file') == 4 && license('test', 'SimMechanics');
if has_simscape
    model_file = fullfile(simulink_dir, [params.simscape.model_name '.slx']);
    if simscape_model_is_stale(model_file, script_dir)
        model_file = build_simscape_model(params, simulink_dir);
    end
    sim_result = run_simscape_dynamics(project_result, model_file);
    dynamics_report = compare_torques(torques, sim_result, params);
else
    fprintf(['\nSimscape Multibody is not available (or disabled): ' ...
        'skipped the Simscape run.\n']);
end

%% Torque plots
figure('Name', 'Joint torques', 'Position', [140 80 1100 700]);
for j = 1:numel(joint_labels)
    subplot(4, 2, j);
    plot(time_s, torques.total_N_m(:, j), 'LineWidth', 1.3);
    hold on;
    plot(time_s, torques.gravity_N_m(:, j), '--', 'LineWidth', 1.0);
    if ~isempty(sim_result)
        plot(time_s, sim_result.tau_N_m(:, j), ':', 'LineWidth', 1.6);
    end
    grid on;
    title(joint_labels{j});
    ylabel('N\cdotm');
    if j >= 6
        xlabel('Time [s]');
    end
end
if isempty(sim_result)
    legend_entries = {'Total (MATLAB)', 'Gravity only'};
else
    legend_entries = {'Total (MATLAB)', 'Gravity only', 'Total (Simscape)'};
end
legend(legend_entries, 'Position', [0.6 0.1 0.25 0.1]);
sgtitle(sprintf('Joint torques (mass properties: %s)', mass_props.source));

%% Save results
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
save(fullfile(results_dir, 'joint_motion.mat'), 'joint_motion_ts', ...
    'motion', 'time_s', 'joint_labels', 'ik_report');
motion_table = array2table([time_s, motion.q, motion.qd, motion.qdd], ...
    'VariableNames', [{'time_s'}, ...
    strcat(joint_labels, '_q_rad'), ...
    strcat(joint_labels, '_qd_rad_s'), ...
    strcat(joint_labels, '_qdd_rad_s2')]);
writetable(motion_table, fullfile(results_dir, 'joint_motion.csv'));

save(fullfile(results_dir, 'joint_torques.mat'), 'time_s', 'torques', ...
    'sim_result', 'dynamics_report', 'mass_props', 'joint_labels');
torque_table = array2table([time_s, torques.total_N_m, ...
    torques.gravity_N_m], 'VariableNames', [{'time_s'}, ...
    strcat(joint_labels, '_tau_N_m'), ...
    strcat(joint_labels, '_tau_gravity_N_m')]);
writetable(torque_table, fullfile(results_dir, 'joint_torques.csv'));

%% Summary
fprintf('\nRun completed: %d samples, %.2f s of motion.\n', ...
    numel(time_s), time_s(end));
fprintf('Peak |torque| per joint [N*m]: %s\n', ...
    mat2str(round(torques.peak_abs_N_m, 2)));
fprintf('Saved results/joint_motion.* and results/joint_torques.*\n');
if ~ik_report.passed
    warning('run_simulation:ikChecksFailed', ...
        'Some IK checks failed; see the verification table above.');
end
if ~isempty(dynamics_report) && ~dynamics_report.passed
    warning('run_simulation:dynamicsMismatch', ...
        'Simscape and MATLAB torques disagree; see the comparison table.');
end
if strcmp(mass_props.source, 'placeholder')
    fprintf(['Note: torques use placeholder mass properties. Enter the ' ...
        'CAD values in init_robot_params.m before reporting them.\n']);
end

function stale = simscape_model_is_stale(model_file, matlab_dir)
% Rebuild when the model is missing or older than the files it is
% generated from.
if ~isfile(model_file)
    stale = true;
    return
end
sources = {'init_robot_params.m', 'link_mass_properties.m', ...
    'build_simscape_model.m'};
model_info = dir(model_file);
stale = false;
for i = 1:numel(sources)
    info = dir(fullfile(matlab_dir, sources{i}));
    stale = stale || info.datenum > model_info.datenum;
end
end
