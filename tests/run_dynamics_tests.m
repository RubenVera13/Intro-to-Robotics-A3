function run_dynamics_tests()
%RUN_DYNAMICS_TESTS Hand-calculation checks of the dynamics model.
% Run from the repository root:  run('tests/run_dynamics_tests.m')
% Needs Robotics System Toolbox (link_mass_properties checks run without).

repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repo_root, 'matlab'));
params = init_robot_params();
mp = link_mass_properties(params);
g = -params.environment.gravity_m_s2(3);

%% link_mass_properties: a solid cylinder matches the textbook formulas
p = params;
p.dynamics.motor_mass_kg = zeros(1, 7);
p.dynamics.wall_thickness_m = 1;          % ri clamps to 0 -> solid
p.dynamics.torch_mass_kg = 0;
mp_solid = link_mass_properties(p);
r = p.dynamics.link_outer_radius_m(2);
L = norm(p.robot.joint_origins_xyz_m(3, :));
m = p.dynamics.density_kg_m3 * pi * r^2 * L;
assert(abs(mp_solid.mass_kg(2) - m) < 1e-12, 'Cylinder mass wrong.');
assert(norm(mp_solid.com_m(2, :) - [0 0 L/2]) < 1e-12, 'CoM wrong.');
I_expected = [m*(3*r^2 + L^2)/12, m*(3*r^2 + L^2)/12, m*r^2/2, 0 0 0];
assert(norm(mp_solid.inertia_com(2, :) - I_expected) < 1e-12, ...
    'Cylinder inertia about CoM wrong.');
I_origin = I_expected + [m*(L/2)^2, m*(L/2)^2, 0, 0 0 0];
assert(norm(mp_solid.inertia_origin(2, :) - I_origin) < 1e-12, ...
    'Parallel-axis shift wrong.');

%% link_mass_properties: CAD values override the placeholders
p = params;
p.dynamics.source = 'cad';
p.dynamics.cad.mass_kg = (1:7)';
p.dynamics.cad.com_m = [zeros(7, 2), 0.01*(1:7)'];
p.dynamics.cad.inertia_com_kg_m2 = repmat([0.1 0.2 0.3 0 0 0], 7, 1);
mp_cad = link_mass_properties(p);
assert(isequal(mp_cad.mass_kg, (1:7)'), 'CAD masses not used.');
assert(norm(mp_cad.inertia_com(3, :) - [0.1 0.2 0.3 0 0 0]) < 1e-12, ...
    'CAD inertia not used.');

if isempty(ver('robotics'))
    fprintf('Robotics System Toolbox missing: skipped torque tests.\n');
    return
end
robot = build_welding_robot(params);

%% Static, arm horizontal: shoulder torque = sum(m * g * x_com)
q = zeros(1, 7);
q(2) = pi/2;                         % arm points along +X
expected = 0;
for k = 2:7
    T = getTransform(robot, q, params.robot.body_names{k});
    com_base = T * [mp.com_m(k, :)'; 1];
    expected = expected + mp.mass_kg(k) * g * com_base(1);
end
tau = inverseDynamics(robot, q);
% Gravity pulls the arm down (positive rotation about +Y), so the
% actuator must push back with a negative torque.
assert(abs(tau(2) + expected) < 1e-9, ...
    'Static shoulder torque %.6f, expected %.6f.', tau(2), -expected);
fprintf('Static horizontal shoulder torque: %.3f N*m\n', tau(2));

%% Waist acceleration, arm horizontal: tau1 = alpha * I_about_waist_axis
alpha = 2.0;
qdd = zeros(1, 7);
qdd(1) = alpha;
tau_dyn = inverseDynamics(robot, q, zeros(1, 7), qdd) - ...
    inverseDynamics(robot, q);
% Link 1 spins about its own Z. For links 2-7, rotating the arm +90 deg
% about Y puts each link's X axis vertical, so their CoM inertia about the
% vertical is Ixx, plus m*d^2 for the horizontal distance d to the waist.
I_waist = mp.inertia_com(1, 3);
for k = 2:7
    T = getTransform(robot, q, params.robot.body_names{k});
    com_base = T * [mp.com_m(k, :)'; 1];
    d = norm(com_base(1:2));
    I_waist = I_waist + mp.inertia_com(k, 1) + mp.mass_kg(k) * d^2;
end
assert(abs(tau_dyn(1) - alpha*I_waist) < 1e-9, ...
    'Waist torque %.6f, expected %.6f.', tau_dyn(1), alpha*I_waist);
fprintf('Waist inertia with arm horizontal: %.4f kg*m^2\n', I_waist);

fprintf('All dynamics tests passed.\n');
end
