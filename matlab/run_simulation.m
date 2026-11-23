%% run_simulation.m — Current project entry point
% Runs the kinematic path and IK prototype. The Simscape model and inverse-
% dynamics stage have not been added yet.

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);

project_result = generate_welding_ik();

fprintf('\nKinematic prototype completed.\n');
fprintf('Samples: %d | duration: %.2f s\n', ...
    numel(project_result.time_s), project_result.time_s(end));
fprintf(['Simscape inverse dynamics has not run: ' ...
    'the model file is not in this repository yet.\n']);
