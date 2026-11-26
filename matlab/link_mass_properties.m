function mp = link_mass_properties(params)
%LINK_MASS_PROPERTIES Mass, center of mass, and inertia of each link.
%   mp = LINK_MASS_PROPERTIES(params) returns, for the 7 links:
%     mp.mass_kg            7x1
%     mp.com_m              7x3, in the link frame
%     mp.inertia_com        7x6, [Ixx Iyy Izz Iyz Ixz Ixy] about the CoM
%     mp.inertia_origin     7x6, same order, about the link-frame origin
%                           (the convention rigidBody.Inertia uses)
%     mp.visuals            7x1 cell; each a struct array of tubes/cylinders
%                           (center_m, axis, length_m, outer_radius_m) for
%                           drawing only
%   Products of inertia are tensor elements (e.g. Ixy = -integral of x*y dm).
%   Core MATLAB only.
%
%   With params.dynamics.source = 'placeholder' the links are built from
%   simple parts (see init_robot_params.m). With 'cad' the values in
%   params.dynamics.cad are used and the placeholder parts only serve as
%   visuals.

d = params.dynamics;
origins = params.robot.joint_origins_xyz_m;
num_links = size(origins, 1);
tip = params.robot.weld_tip_offset_xyz_m;

mp.mass_kg = zeros(num_links, 1);
mp.com_m = zeros(num_links, 3);
mp.inertia_com = zeros(num_links, 6);
mp.inertia_origin = zeros(num_links, 6);
mp.visuals = cell(num_links, 1);

for k = 1:num_links
    parts = struct('mass', {}, 'com', {}, 'I', {});
    visuals = struct('center_m', {}, 'axis', {}, 'length_m', {}, ...
        'outer_radius_m', {});

    % Structural tube from this joint to the next joint (or a hub).
    if k < num_links
        segment = origins(k+1, :);
    else
        segment = [0 0 0];
    end
    ro = d.link_outer_radius_m(k);
    ri = max(ro - d.wall_thickness_m, 0);
    if norm(segment) >= d.hub_length_m
        len = norm(segment);
        u = segment / len;
        center = segment / 2;
    else
        len = d.hub_length_m;
        u = [0 0 1];
        center = [0 0 0];
    end
    m_tube = d.density_kg_m3 * pi * (ro^2 - ri^2) * len;
    % (parts/visuals hold 2-3 entries, so growing them is fine.)
    parts(end+1) = part(m_tube, center, ...
        tube_inertia(m_tube, ro, ri, len, u)); %#ok<AGROW>
    visuals(end+1) = visual(center, u, len, ro); %#ok<AGROW>

    % Motor at this link's joint (point mass).
    parts(end+1) = part(d.motor_mass_kg(k), [0 0 0], zeros(3)); %#ok<AGROW>

    % Torch on the last link, from the link origin to the weld tip.
    if k == num_links && d.torch_mass_kg > 0
        len_t = norm(tip);
        u_t = tip / len_t;
        parts(end+1) = part(d.torch_mass_kg, tip/2, ...
            tube_inertia(d.torch_mass_kg, d.torch_radius_m, 0, len_t, ...
            u_t)); %#ok<AGROW>
        visuals(end+1) = visual(tip/2, u_t, len_t, ...
            d.torch_radius_m); %#ok<AGROW>
    end

    [m, c, I_com] = combine(parts);
    if strcmpi(d.source, 'cad')
        if isempty(d.cad.mass_kg)
            error('link_mass_properties:noCad', ['params.dynamics.source ' ...
                'is ''cad'' but params.dynamics.cad is empty.']);
        end
        m = d.cad.mass_kg(k);
        c = d.cad.com_m(k, :);
        I_com = vec_to_tensor(d.cad.inertia_com_kg_m2(k, :));
    end

    mp.mass_kg(k) = m;
    mp.com_m(k, :) = c;
    mp.inertia_com(k, :) = tensor_to_vec(I_com);
    mp.inertia_origin(k, :) = tensor_to_vec(I_com + shift(m, c));
    mp.visuals{k} = visuals;
end
mp.total_mass_kg = sum(mp.mass_kg);
mp.source = d.source;
end

function p = part(mass, com, I)
p = struct('mass', mass, 'com', com, 'I', I);
end

function v = visual(center, axis_vec, len, radius)
v = struct('center_m', center, 'axis', axis_vec, 'length_m', len, ...
    'outer_radius_m', radius);
end

function I = tube_inertia(m, ro, ri, len, u)
% Thick-walled tube (ri = 0 gives a solid cylinder) about its own CoM,
% with its axis along unit vector u.
r2 = ro^2 + ri^2;
I_axial = m * r2 / 2;
I_trans = m * (3*r2 + len^2) / 12;
u = u(:);
I = I_trans * (eye(3) - u*u') + I_axial * (u*u');
end

function [m, c, I] = combine(parts)
m = sum([parts.mass]);
c = zeros(1, 3);
for i = 1:numel(parts)
    c = c + parts(i).mass * parts(i).com;
end
c = c / m;
I = zeros(3);
for i = 1:numel(parts)
    I = I + parts(i).I + shift(parts(i).mass, parts(i).com - c);
end
end

function S = shift(m, r)
% Parallel-axis term for a point mass m at offset r.
r = r(:);
S = m * ((r'*r)*eye(3) - (r*r'));
end

function v = tensor_to_vec(I)
v = [I(1,1) I(2,2) I(3,3) I(2,3) I(1,3) I(1,2)];
end

function I = vec_to_tensor(v)
I = [v(1) v(6) v(5); v(6) v(2) v(4); v(5) v(4) v(3)];
end
