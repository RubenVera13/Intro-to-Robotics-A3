%% run_simulation.m — Current project entry point
% Runs the path generation and IK, verifies the result, and saves the joint
% motion (q, qd, qdd) for the Simscape model. The Simscape model and the
% inverse-dynamics stage have not been added yet.

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
results_dir = fullfile(script_dir, '..', 'results');

project_result = generate_welding_ik();
ik_report = check_ik_result(project_result, project_result.params);

% Variables the Simscape model reads from the base workspace.
joint_motion_ts = project_result.joint_motion_ts;
motion = project_result.motion;
time_s = project_result.time_s;
joint_labels = project_result.params.robot.joint_labels;

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

fprintf('\nKinematic prototype completed.\n');
fprintf('Samples: %d | duration: %.2f s\n', numel(time_s), time_s(end));
fprintf('Joint motion saved to results/joint_motion.mat and .csv\n');
if ~ik_report.passed
    warning('run_simulation:checksFailed', ...
        'Some IK checks failed; see the verification table above.');
end
fprintf(['Simscape inverse dynamics has not run: ' ...
    'the model file is not in this repository yet.\n']);
