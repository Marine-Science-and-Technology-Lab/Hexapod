function test_kinematics_regression()
% test_kinematics_regression - Run the vectorized kinematics solver and
% the legacy scalar one on the same synthetic pose trajectory, assert
% that every output field agrees within 1e-10 tolerance, and report the
% relative speedup.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root);

% ---- build a synthetic hex_obj / hex_setup / hex_path ---------------
[hex_obj, hex_setup] = makeSyntheticRig();
hex_path             = makeSyntheticPath(1000);

% Run vectorized
tic;
hp_vec = SimulateMotionProfile_kinematics(hex_obj, hex_setup, hex_path);
t_vec = toc;

% Run legacy
tic;
hp_leg = SimulateMotionProfile_kinematics_legacy(hex_obj, hex_setup, hex_path);
t_leg = toc;

fprintf('Vectorized: %.3f s    Legacy: %.3f s    Speedup: %.1fx\n', ...
    t_vec, t_leg, t_leg / max(t_vec, eps));

% Compare every field in hex_path that both versions produce.
fields = {'pose_t', 'pose_t_relative', 'pose_dt', 'pose_ddt', ...
          'axis_t', 'axis_dt', 'axis_ddt', 'axis_cts', ...
          'lhat', 'plati', ...
          'joint_separation', 'collisioncheck'};

tol = 1e-10;
for k = 1:length(fields)
    f = fields{k};
    a = getfieldDeep(hp_vec, f);
    b = getfieldDeep(hp_leg, f);
    err = maxAbsErr(a, b);
    assert(err < tol, ...
        'Field %s diverged: max-abs error = %g (tol %g)', f, err, tol);
    fprintf('  %-20s  max-abs err = %8.2e\n', f, err);
end

% Joint angle agreement (nested structs)
for name = {'angle_theta', 'angle_phi'}
    err = maxAbsErr(hp_vec.joint_AB.(name{1}), hp_leg.joint_AB.(name{1}));
    assert(err < tol, 'joint_AB.%s diverged: %g', name{1}, err);
    fprintf('  joint_AB.%-12s max-abs err = %8.2e\n', name{1}, err);
end
for name = {'angle_alpha', 'angle_psi'}
    err = maxAbsErr(hp_vec.joint_CD.(name{1}), hp_leg.joint_CD.(name{1}));
    assert(err < tol, 'joint_CD.%s diverged: %g', name{1}, err);
    fprintf('  joint_CD.%-12s max-abs err = %8.2e\n', name{1}, err);
end

fprintf('PASS  test_kinematics_regression  (%d samples, all fields agree)\n', ...
    size(hex_path.pose_t, 2));
end


function val = getfieldDeep(s, f)
% Flatten a potentially struct-valued field to a numeric array for
% comparison.
val = s.(f);
if isstruct(val)
    names = fieldnames(val);
    parts = cell(1, length(names));
    for k = 1:length(names)
        parts{k} = val.(names{k})(:);
    end
    val = vertcat(parts{:});
end
end


function err = maxAbsErr(a, b)
a = a(:); b = b(:);
if length(a) ~= length(b)
    err = inf;
    return
end
mask = ~(isnan(a) & isnan(b));       % treat matching NaNs as equal
err = max(abs(a(mask) - b(mask)), [], 'omitnan');
if isempty(err); err = 0; end
end


function [hex_obj, hex_setup] = makeSyntheticRig()
% Minimal synthetic rig mimicking the shape of a real hexapod setup.
% Exact values don't need to be physically meaningful - they just have
% to exercise every code path in both implementations identically.

hex_obj.pose  = zeros(6,1);
% Default frame configuration for regression: datum at a synthetic
% "home" point, POI offset by a translation in the platform body frame.
% No rotations in this test rig - exercises translation-only path.
hex_obj.Home_platform          = [0; 0; 0.5];
hex_obj.T_world_datum_platform = struct('R', eye(3), 't', hex_obj.Home_platform);
hex_obj.T_platform_POI         = struct('R', eye(3), 't', [0; 0; 0.1]);

th = linspace(0, 2*pi, 7); th(end) = [];   % 6 angles
R_base = 0.30;   % base radius
R_plat = 0.15;   % platform radius

base      = [R_base*cos(th); R_base*sin(th); zeros(1,6)];
plat      = [R_plat*cos(th); R_plat*sin(th); zeros(1,6)];
hex_obj.base      = base;
hex_obj.plat0     = plat;
hex_obj.base_link = base + [0;0;0.02];
hex_obj.plat_link_0 = plat + [0;0;-0.02];
hex_obj.z   = 0.2;
hex_obj.L0  = 0.4;
hex_obj.dL  = 0.2;

hex_setup.YokeA.Uhat = 0:60:300;  % 6 angles in degrees

% Encoder model matching LengthToEncoder.m expectations
hex_setup.Actuators.DatumLength_Individual = 0.4 * ones(6,1);
hex_setup.Actuators.CountsPerM             = 1e6;

% The joint-separation interpolants are called with degree inputs and
% return a scalar separation per (theta, phi) pair. A constant fn
% exercises the call pattern without needing real data.
hex_setup.Joint_Interp.SCAT_AB = @(theta, phi) 10 * ones(size(theta));
hex_setup.Joint_Interp.SCAT_CD = @(alpha, psi) 10 * ones(size(alpha));
hex_setup.collisionthreshold = 0.5;
end


function hex_path = makeSyntheticPath(N)
% Small-amplitude smooth trajectory exercising all 6 DOFs.
T  = linspace(0, 1, N);
dt = T(2) - T(1);

pose_t = zeros(6, N);
pose_t(1,:) = 0.02  * sin(2*pi*1*T);         % x
pose_t(2,:) = 0.015 * sin(2*pi*0.7*T + 0.3); % y
pose_t(3,:) = 0.01  * cos(2*pi*0.5*T);       % z
pose_t(4,:) = 0.05  * sin(2*pi*0.6*T);       % roll
pose_t(5,:) = 0.04  * cos(2*pi*0.8*T);       % pitch
pose_t(6,:) = 0.03  * sin(2*pi*1.1*T);       % yaw

hex_path.T      = T;
hex_path.dt     = dt;
hex_path.pose_t = pose_t;
end
