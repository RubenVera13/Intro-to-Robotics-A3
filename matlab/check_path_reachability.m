function reach = check_path_reachability(position_m, R_torch, params)
%CHECK_PATH_REACHABILITY Quick geometric reach test before solving IK.
%   reach = CHECK_PATH_REACHABILITY(position_m, R_torch, params) checks
%   whether the wrist center needed for each N-by-3 torch-tip target can be
%   reached by the shoulder-elbow pair. Core MATLAB only.
%
%   Assumptions (true for the current Z-Y-Y-Z-Y-X-Z layout; revisit if the
%   CAD changes the geometry):
%     * the shoulder (joint 2) sits on the waist axis, at the sum of the
%       first two joint origins;
%     * joints 5-7 meet at one point (spherical wrist center at joint 5);
%     * the wrist-center-to-tip offset lies along the torch (tool Z) axis.
%   Passing this test is necessary, not sufficient: joint limits,
%   orientation, and collisions are checked after IK.

origins = params.robot.joint_origins_xyz_m;
shoulder = sum(origins(1:2, :), 1);
upper_arm = norm(origins(3, :));
forearm = norm(sum(origins(4:5, :), 1));
wrist_to_tip = norm(sum(origins(6:7, :), 1) + ...
    params.robot.weld_tip_offset_xyz_m);

tool_z = R_torch(:, 3)';
wrist_center = position_m - wrist_to_tip * tool_z;
distance = vecnorm(wrist_center - shoulder, 2, 2);

reach.max_reach_m = upper_arm + forearm;
reach.min_reach_m = abs(upper_arm - forearm);
reach.distance_m = distance;
reach.reach_ratio = distance / reach.max_reach_m;
reach.unreachable = distance > reach.max_reach_m | ...
    distance < reach.min_reach_m;
reach.ok = ~any(reach.unreachable);
reach.max_reach_ratio = max(reach.reach_ratio);
end
