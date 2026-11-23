function robot = build_welding_robot(params)
%BUILD_WELDING_ROBOT Build the provisional seven-revolute rigidBodyTree.
% The dimensions, axes, and provisional limits come from init_robot_params.

robot = rigidBodyTree('DataFormat', params.model.data_format);

for k = 1:numel(params.robot.joint_names)
    body = rigidBody(params.robot.body_names{k});
    joint = rigidBodyJoint(params.robot.joint_names{k}, 'revolute');
    joint.JointAxis = params.robot.joint_axes(k, :);
    joint.PositionLimits = params.robot.joint_limits_rad(k, :);
    setFixedTransform(joint, trvec2tform( ...
        params.robot.joint_origins_xyz_m(k, :)));
    body.Joint = joint;
    addBody(robot, body, params.robot.parent_names{k});
end

tip = rigidBody('weld_tip');
tip_joint = rigidBodyJoint('j_weld_tip', 'fixed');
setFixedTransform(tip_joint, trvec2tform( ...
    params.robot.weld_tip_offset_xyz_m));
tip.Joint = tip_joint;
addBody(robot, tip, params.robot.body_names{end});
end
