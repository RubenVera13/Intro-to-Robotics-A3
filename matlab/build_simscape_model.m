function model_file = build_simscape_model(params, model_dir)
%BUILD_SIMSCAPE_MODEL Generate the Simscape Multibody inverse-dynamics model.
%   model_file = BUILD_SIMSCAPE_MODEL(params, model_dir) builds
%   <model_dir>/<params.simscape.model_name>.slx from the same parameters
%   as build_welding_robot.m (geometry, joint axes, mass properties).
%
%   Each joint follows the joint_motion_ts input (q, qd, qdd) and computes
%   the torque needed for that motion. The model logs:
%     tau_sim  timeseries, 7 joint torques [N*m]
%     tcp_sim  timeseries, torch-tip position in the world frame [m]
%   It reads joint_motion_ts, sim_stop_time, and sim_dt from the
%   simulation input (see run_simscape_dynamics.m).
%
%   The model is generated: change init_robot_params.m and rebuild instead
%   of editing parameter values by hand. Requires Simulink, Simscape, and
%   Simscape Multibody.

if nargin < 2
    model_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'simulink');
end
mdl = params.simscape.model_name;
model_file = fullfile(model_dir, [mdl '.slx']);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
load_system('sm_lib');
load_system('nesl_utility');
new_system(mdl);
cleanup = onCleanup(@() close_system(mdl, 0));

mp = link_mass_properties(params);
origins = params.robot.joint_origins_xyz_m;
axes_ = params.robot.joint_axes;
labels = params.robot.joint_labels;
num_joints = size(axes_, 1);

%% Environment
world = addb('sm_lib/Frames and Transforms/World Frame', 'World', 330, 320);
mech = addb('sm_lib/Utilities/Mechanism Configuration', ...
    'Mechanism Configuration', 330, 420);
set_param(mech, 'UniformGravity', 'Constant', 'GravityVector', ...
    mat2str(params.environment.gravity_m_s2), 'GravityVectorUnits', 'm/s^2');
solver = addb('nesl_utility/Solver Configuration', ...
    'Solver Configuration', 330, 510);
phys(port(world, 'RConn', 1), any_port(mech));
phys(port(world, 'RConn', 1), any_port(solver));

%% Joint-motion input: [q(1:7), qd(1:7), qdd(1:7)] -> Goto tags
% Tags (q1, qd1, qdd1, ...) keep the diagram readable: each joint picks up
% its own signals with From blocks instead of long wires.
spacing = 340;                          % horizontal pitch per joint
x_end = 260 + spacing*(num_joints + 1); % right-hand column (outputs)
tag_pitch = 28;
from_ws = addb('simulink/Sources/From Workspace', 'joint_motion', ...
    20, 290, 80, 40);
set_param(from_ws, 'VariableName', 'joint_motion_ts', ...
    'Interpolate', 'on', 'OutputAfterFinalValue', 'Holding final value');
demux = addb('simulink/Signal Routing/Demux', 'Demux', 140, 20, 10, ...
    tag_pitch*3*num_joints);
set_param(demux, 'Outputs', num2str(3*num_joints));
add_line(mdl, 'joint_motion/1', 'Demux/1');
for i = 1:3*num_joints
    goto = addb('simulink/Signal Routing/Goto', ...
        sprintf('goto_%s', motion_tag(i, num_joints)), ...
        190, 20 + tag_pitch*(i - 1) + 2, 60, 20);
    set_param(goto, 'GotoTag', motion_tag(i, num_joints));
    add_line(mdl, sprintf('Demux/%d', i), ...
        sprintf('goto_%s/1', motion_tag(i, num_joints)));
end

%% Torque output: From tags -> Mux -> To Workspace
tau_mux = addb('simulink/Signal Routing/Mux', 'tau_mux', ...
    x_end + 100, 480, 10, 40*num_joints);
set_param(tau_mux, 'Inputs', num2str(num_joints));
tau_out = addb('simulink/Sinks/To Workspace', 'tau_out', ...
    x_end + 150, 480 + 20*num_joints - 15, 80, 30);
set_param(tau_out, 'VariableName', 'tau_sim', 'SaveFormat', 'Timeseries', ...
    'SampleTime', 'sim_dt');
add_line(mdl, 'tau_mux/1', 'tau_out/1');

%% Kinematic chain
prev = port(world, 'RConn', 1);
for k = 1:num_joints
    x = 260 + spacing*k;
    [align_axis, align_deg] = joint_alignment(axes_(k, :));

    % Parent frame -> joint frame: translation, then turn Z onto the axis.
    rt_in = addb('sm_lib/Frames and Transforms/Rigid Transform', ...
        sprintf('RT%d_to_joint', k), x - 150, 320);
    set_translation(rt_in, origins(k, :));
    set_rotation(rt_in, align_axis, align_deg);
    phys(prev, port(rt_in, 'LConn', 1));

    joint = addb('sm_lib/Joints/Revolute Joint', ...
        sprintf('J%d_%s', k, labels{k}), x - 50, 315);
    set_param(joint, 'MotionActuationMode', 'InputMotion', ...
        'TorqueActuationMode', 'ComputedTorque', 'SenseTorqueForce', 'on');
    phys(port(rt_in, 'RConn', 1), port(joint, 'LConn', 1));

    % Undo the alignment so the link frame matches the rigidBodyTree.
    if isempty(align_axis)
        link_frame = port(joint, 'RConn', 1);
    else
        rt_out = addb('sm_lib/Frames and Transforms/Rigid Transform', ...
            sprintf('RT%d_to_link', k), x + 50, 320);
        set_rotation(rt_out, align_axis, -align_deg);
        phys(port(joint, 'RConn', 1), port(rt_out, 'LConn', 1));
        link_frame = port(rt_out, 'RConn', 1);
    end

    % Motion input with user-provided first and second derivatives.
    conv = addb('nesl_utility/Simulink-PS Converter', ...
        sprintf('q%d_in', k), x - 90, 175, 40, 60);
    set_param(conv, 'Unit', 'rad', 'FilteringAndDerivatives', 'provide', ...
        'UdotUserProvided', '2');
    for d = 0:2
        tag = motion_tag(k + d*num_joints, num_joints);
        from = addb('simulink/Signal Routing/From', ['from_' tag], ...
            x - 190, 172 + 22*d, 60, 18);
        set_param(from, 'GotoTag', tag);
        add_line(mdl, sprintf('from_%s/1', tag), ...
            sprintf('q%d_in/%d', k, d + 1), 'autorouting', 'smart');
    end
    phys(port(conv, 'RConn', 1), port(joint, 'LConn', 2));

    % Torque sensing -> Goto tag tauK.
    tau_conv = addb('nesl_utility/PS-Simulink Converter', ...
        sprintf('tau%d_out', k), x - 10, 410);
    set_param(tau_conv, 'Unit', 'N*m');
    phys(port(joint, 'RConn', 2), port(tau_conv, 'LConn', 1));
    tau_tag = sprintf('tau%d', k);
    goto = addb('simulink/Signal Routing/Goto', ['goto_' tau_tag], ...
        x + 60, 420, 60, 20);
    set_param(goto, 'GotoTag', tau_tag);
    add_line(mdl, sprintf('tau%d_out/1', k), ['goto_' tau_tag '/1']);
    from = addb('simulink/Signal Routing/From', ['from_' tau_tag], ...
        x_end, 490 + 40*(k - 1), 60, 20);
    set_param(from, 'GotoTag', tau_tag);
    add_line(mdl, ['from_' tau_tag '/1'], sprintf('tau_mux/%d', k));

    % Mass properties (about the CoM, in the link frame).
    inertia = addb('sm_lib/Body Elements/Inertia', ...
        sprintf('Link%d_inertia', k), x + 50, 500);
    set_param(inertia, 'InertiaType', 'Custom', ...
        'Mass', num2str(mp.mass_kg(k), 12), 'MassUnits', 'kg', ...
        'CenterOfMass', mat2str(mp.com_m(k, :), 12), ...
        'CenterOfMassUnits', 'm', ...
        'MomentsOfInertia', mat2str(mp.inertia_com(k, 1:3), 12), ...
        'ProductsOfInertia', mat2str(mp.inertia_com(k, 4:6), 12), ...
        'MomentsOfInertiaUnits', 'kg*m^2', ...
        'ProductsOfInertiaUnits', 'kg*m^2');
    phys(link_frame, any_port(inertia));

    % Massless cylinders, for the Mechanics Explorer animation only.
    vis = mp.visuals{k};
    for v = 1:numel(vis)
        add_visual(link_frame, vis(v), sprintf('Link%d_vis%d', k, v), ...
            x + 50, 610 + 100*(v - 1), k);
    end

    prev = link_frame;
end

%% Torch tip and its position sensor
rt_tip = addb('sm_lib/Frames and Transforms/Rigid Transform', ...
    'RT_weld_tip', x_end - 140, 320);
set_translation(rt_tip, params.robot.weld_tip_offset_xyz_m);
phys(prev, port(rt_tip, 'LConn', 1));

sensor = addb('sm_lib/Frames and Transforms/Transform Sensor', ...
    'TCP_sensor', x_end - 40, 200, 50, 70);
set_param(sensor, 'SenseX', 'on', 'SenseY', 'on', 'SenseZ', 'on');
phys(port(world, 'RConn', 1), port(sensor, 'LConn', 1));
phys(port(rt_tip, 'RConn', 1), port(sensor, 'RConn', 1));
tcp_mux = addb('simulink/Signal Routing/Mux', 'tcp_mux', ...
    x_end + 150, 190, 10, 100);
set_param(tcp_mux, 'Inputs', '3');
for a = 1:3
    c = addb('nesl_utility/PS-Simulink Converter', ...
        sprintf('tcp%d_out', a), x_end + 60, 190 + 35*(a - 1), 40, 25);
    set_param(c, 'Unit', 'm');
    phys(port(sensor, 'RConn', a + 1), port(c, 'LConn', 1));
    add_line(mdl, sprintf('tcp%d_out/1', a), sprintf('tcp_mux/%d', a), ...
        'autorouting', 'smart');
end
tcp_out = addb('simulink/Sinks/To Workspace', 'tcp_out', ...
    x_end + 200, 225, 80, 30);
set_param(tcp_out, 'VariableName', 'tcp_sim', 'SaveFormat', 'Timeseries', ...
    'SampleTime', 'sim_dt');
add_line(mdl, 'tcp_mux/1', 'tcp_out/1');

%% Tidy: Goto/From blocks already show their [tag]
tag_blocks = [find_system(mdl, 'BlockType', 'Goto'); ...
    find_system(mdl, 'BlockType', 'From')];
for i = 1:numel(tag_blocks)
    set_param(tag_blocks{i}, 'ShowName', 'off');
end

%% Solver settings and save
set_param(mdl, 'StopTime', 'sim_stop_time', 'Solver', 'ode23t', ...
    'MaxStep', 'sim_dt', 'RelTol', '1e-6', 'AbsTol', '1e-8');
if ~exist(model_dir, 'dir')
    mkdir(model_dir);
end
save_system(mdl, model_file);
fprintf('Saved Simscape model: %s\n', model_file);

%% Helpers (nested: they share mdl)
    function h = addb(lib, name, x0, y0, w, hgt)
        if nargin < 5, w = 40; end
        if nargin < 6, hgt = 40; end
        h = add_block(lib, [mdl '/' name], ...
            'Position', [x0, y0, x0 + w, y0 + hgt]);
    end

    function add_visual(frame, vis, name, x0, y0, k)
        if norm(cross(vis.axis, [0 0 1])) > 1e-9
            warning('build_simscape_model:visualAxis', ['%s is not along ' ...
                'the link Z axis; drawing it along Z.'], name);
        end
        rt = addb('sm_lib/Frames and Transforms/Rigid Transform', ...
            [name '_pos'], x0 - 110, y0);
        set_translation(rt, vis.center_m);
        phys(frame, port(rt, 'LConn', 1));
        solid = addb('sm_lib/Body Elements/Cylindrical Solid', name, x0, y0);
        colors = lines(7);
        set_param(solid, 'CylinderRadius', num2str(vis.outer_radius_m), ...
            'CylinderRadiusUnits', 'm', ...
            'CylinderLength', num2str(vis.length_m), ...
            'CylinderLengthUnits', 'm', ...
            'InertiaType', 'PointMass', 'Mass', '0', 'MassUnits', 'kg', ...
            'GraphicDiffuseColor', mat2str(colors(k, :), 4));
        phys(port(rt, 'RConn', 1), any_port(solid));
    end

    function phys(src, dst)
        add_line(mdl, src, dst, 'autorouting', 'smart');
    end
end

function tag = motion_tag(i, num_joints)
% Demux output i -> 'q3', 'qd3', 'qdd3', ...
kinds = {'q', 'qd', 'qdd'};
tag = sprintf('%s%d', kinds{ceil(i/num_joints)}, mod(i - 1, num_joints) + 1);
end

function h = port(block, side, index)
ph = get_param(block, 'PortHandles');
h = ph.(side)(index);
end

function h = any_port(block)
ph = get_param(block, 'PortHandles');
h = [ph.LConn, ph.RConn];
h = h(1);
end

function set_translation(block, xyz)
if norm(xyz) == 0
    set_param(block, 'TranslationMethod', 'None');
else
    set_param(block, 'TranslationMethod', 'Cartesian', ...
        'TranslationCartesianOffset', mat2str(xyz, 12), ...
        'TranslationCartesianOffsetUnits', 'm');
end
end

function set_rotation(block, axis_name, angle_deg)
if isempty(axis_name)
    set_param(block, 'RotationMethod', 'None');
else
    set_param(block, 'RotationMethod', 'StandardAxis', ...
        'RotationStandardAxis', axis_name, ...
        'RotationAngle', num2str(angle_deg), 'RotationAngleUnits', 'deg');
end
end

function [axis_name, angle_deg] = joint_alignment(joint_axis)
% Rotation that maps a frame's Z onto joint_axis. Simscape revolute joints
% turn about Z; rotating the joint frame this way makes them turn about
% joint_axis instead. Only the six signed coordinate axes are supported.
a = round(joint_axis(:)' / norm(joint_axis));
if isequal(a, [0 0 1])
    axis_name = ''; angle_deg = 0;
elseif isequal(a, [0 0 -1])
    axis_name = '+X'; angle_deg = 180;
elseif isequal(a, [0 1 0])
    axis_name = '+X'; angle_deg = -90;    % Rx(-90) maps Z to +Y
elseif isequal(a, [0 -1 0])
    axis_name = '+X'; angle_deg = 90;
elseif isequal(a, [1 0 0])
    axis_name = '+Y'; angle_deg = 90;     % Ry(+90) maps Z to +X
elseif isequal(a, [-1 0 0])
    axis_name = '+Y'; angle_deg = -90;
else
    error('build_simscape_model:axis', ...
        'Joint axis %s is not a coordinate axis.', mat2str(joint_axis));
end
end
