function sim_result = run_simscape_dynamics(result, model_file)
%RUN_SIMSCAPE_DYNAMICS Simulate the Simscape model with the exported motion.
%   sim_result = RUN_SIMSCAPE_DYNAMICS(result, model_file) runs the model
%   built by build_simscape_model.m, driven by result.joint_motion_ts from
%   generate_welding_ik, and returns the logged signals resampled onto
%   result.time_s:
%     sim_result.tau_N_m          N-by-7 joint torques
%     sim_result.tcp_position_m   N-by-3 torch-tip position
%     sim_result.tcp_error_m      N-by-1 distance from the planned path

[~, mdl] = fileparts(model_file);
load_system(model_file);
cleanup = onCleanup(@() close_system(mdl, 0));

dt = result.params.trajectory.sample_time_s;
in = Simulink.SimulationInput(mdl);
in = in.setVariable('joint_motion_ts', result.joint_motion_ts);
in = in.setVariable('sim_stop_time', result.time_s(end));
in = in.setVariable('sim_dt', dt);

fprintf('Running Simscape inverse dynamics (%s)...\n', mdl);
out = sim(in);
if ~isempty(out.ErrorMessage)
    error('run_simscape_dynamics:sim', '%s', out.ErrorMessage);
end

t = result.time_s;
sim_result.tau_N_m = resample_to(out.tau_sim, t);
sim_result.tcp_position_m = resample_to(out.tcp_sim, t);
sim_result.tcp_error_m = vecnorm( ...
    sim_result.tcp_position_m - result.position_m, 2, 2);
sim_result.time_s = t;
end

function y = resample_to(ts, t)
data = squeeze(ts.Data);
if size(data, 1) ~= numel(ts.Time)
    data = data';
end
y = interp1(ts.Time, data, t, 'linear', 'extrap');
end
