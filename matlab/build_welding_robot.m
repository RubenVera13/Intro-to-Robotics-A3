function robot = build_welding_robot(params)
%BUILD_WELDING_ROBOT Build the seven-revolute rigidBodyTree.
% Dimensions, axes, and limits come from init_robot_params; mass
% properties come from link_mass_properties (placeholder until CAD).

robot = rigidBodyTree('DataFormat', params.model.data_format);
robot.Gravity = params.environment.gravity_m_s2;
mp = link_mass_properties(params);

for k = 1:numel(params.robot.joint_names)
    body = rigidBody(params.robot.body_names{k});
    joint = rigidBodyJoint(params.robot.joint_names{k}, 'revolute');
    joint.JointAxis = params.robot.joint_axes(k, :);
    joint.PositionLimits = params.robot.joint_limits_rad(k, :);
    setFixedTransform(joint, trvec2tform( ...
        params.robot.joint_origins_xyz_m(k, :)));
    body.Joint = joint;
    body.Mass = mp.mass_kg(k);
    body.CenterOfMass = mp.com_m(k, :);
    % rigidBody.Inertia is about the body-frame origin.
    body.Inertia = mp.inertia_origin(k, :);
    addBody(robot, body, params.robot.parent_names{k});
end

% The weld tip is a massless frame; the torch mass is part of link 7.
tip = rigidBody('weld_tip');
tip.Mass = 0;
tip.Inertia = zeros(1, 6);
tip_joint = rigidBodyJoint('j_weld_tip', 'fixed');
setFixedTransform(tip_joint, trvec2tform( ...
    params.robot.weld_tip_offset_xyz_m));
tip.Joint = tip_joint;
addBody(robot, tip, params.robot.body_names{end});
end
