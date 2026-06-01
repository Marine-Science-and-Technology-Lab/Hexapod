function test_ForwardKinematics()
% test_ForwardKinematics - Round-trip X_plat -> lengths -> FK -> X_plat'
% using real hexapod geometry.
%
%   The FK solver works in platform-pose space (plat_CM + Euler) and
%   depends only on the fixed mechanical geometry - r_rel / Home are
%   applied separately at display time. This test exercises the
%   platform-space round-trip independently.
%
%   A second block verifies the display-frame transform used by
%   updateDigitalShadow: platform pose -> end-effector pose should be
%   the exact inverse of what InverseKinematics_hexapod does to get
%   platform pose from its hex_obj.pose input.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root);

[hex_obj, hex_setup] = InitializeHexapodObject();

jsi = load(fullfile(root, 'JointSep_Interpolants.mat'));
hex_setup.Joint_Interp = jsi.JointSep_Interpolants;

% Test runs against the default frame configuration (datum at Home,
% POI coincident with platform body). All rotations identity.

% Hexapod mechanical geometry: plat_link_0 and base_link are the only
% fields ForwardKinematics_hexapod actually touches.

tol_pose  = 1e-8;
tol_resid = 1e-9;

% ---- Block 1: cold-start round-trips ------------------------------
%
% Build a set of platform poses, compute the link lengths each one
% would produce, then verify the FK solver recovers the same poses
% from those lengths starting from a zero seed.

% "home" platform pose: plat_CM = -r_rel + Home mapped from display
% zero. Use the actual Home_platform value from hex_obj so the seed
% at zero can converge.
X_plat_home = [hex_obj.Home_platform; 0; 0; 0];

testPoses = { ...
    X_plat_home,                                                         'home'; ...
    X_plat_home + [0.01; 0; 0; 0; 0; 0],                                 '+X 10mm'; ...
    X_plat_home + [0; 0.01; 0; 0; 0; 0],                                 '+Y 10mm'; ...
    X_plat_home + [0; 0; -0.02; 0; 0; 0],                                '-Z 20mm'; ...
    X_plat_home + [0; 0; 0; deg2rad(3); 0; 0],                           '+roll 3deg'; ...
    X_plat_home + [0; 0; 0; 0; deg2rad(3); 0],                           '+pitch 3deg'; ...
    X_plat_home + [0; 0; 0; 0; 0; deg2rad(5)],                           '+yaw 5deg'; ...
    X_plat_home + [0.005; -0.003; -0.01; deg2rad(2); deg2rad(1.5); deg2rad(2)],  'combined small'; ...
    X_plat_home + [0.02; 0.015; -0.02; deg2rad(4); deg2rad(-3); deg2rad(6)],     'combined larger'; ...
};

nPass = 0;
for row = 1:size(testPoses, 1)
    X_true = testPoses{row, 1};
    label  = testPoses{row, 2};

    L_cmd = lengthsFromPlatformPose(X_true, hex_obj);

    [X_solved, converged, iters, residual] = ForwardKinematics_hexapod( ...
        hex_obj, L_cmd, X_plat_home);   % seed at home, not zero

    err = max(abs(X_solved - X_true));

    passed = converged && err < tol_pose && residual < tol_resid;
    if passed
        nPass = nPass + 1;
        status = 'PASS';
    else
        status = 'FAIL';
    end
    fprintf('  %s  %-18s  err=%.2e  iters=%d  residual=%.2e  converged=%d\n', ...
        status, label, err, iters, residual, converged);

    assert(passed, 'Round-trip failed for "%s" (err=%g, residual=%g)', ...
        label, err, residual);
end

% ---- Block 2: warm-start sequence ---------------------------------
%
% Consecutive small pose steps using the last solve as the next seed.
% Real-time shadow use case: should converge in 1-2 iterations each.

fprintf('\nWarm-start sequence (each uses previous solved pose as seed):\n');
X_seed_plat = X_plat_home;
N = 20;
total_iters = 0;
for n = 1:N
    X_true_plat = X_plat_home + ...
        [0.001*n; 0; -0.0005*n; deg2rad(0.2*n); 0; 0];
    L_cmd = lengthsFromPlatformPose(X_true_plat, hex_obj);
    [X_solved_plat, conv, iters] = ForwardKinematics_hexapod( ...
        hex_obj, L_cmd, X_seed_plat);
    assert(conv && max(abs(X_solved_plat - X_true_plat)) < tol_pose, ...
        'warm-start step %d diverged', n);
    X_seed_plat = X_solved_plat;
    total_iters = total_iters + iters;
end
avg_iters = total_iters / N;
fprintf('  %d consecutive poses, average %.1f Newton iterations\n', N, avg_iters);
assert(avg_iters < 3, 'warm-started solver should average <3 iterations/step');

% ---- Block 3: display-frame transform round-trip -----------------
%
% updateDigitalShadow applies:
%     r        = plat_CM + R * r_rel
%     disp_xyz = r - Home
% This must be the exact inverse of what IK does when converting the
% app's "pose" input (display-frame) into plat_CM. Check that
% display_xyz composed with Euler recovers the IK's hex_obj.pose.

fprintf('\nDisplay-frame transform consistency:\n');
N_disp = 5;
for n = 1:N_disp
    % Pick an arbitrary display-frame pose.
    disp_pose_in = 0.01 * [1.2; -0.8; -1.5; deg2rad(100)*0.03; deg2rad(100)*0.02; deg2rad(100)*0.05] * n;

    % Run IK to get the hex_obj geometry at that pose (which computes
    % plat_CM internally).
    hex_obj.pose = disp_pose_in;
    hex_obj_ik   = InverseKinematics_hexapod(hex_obj, hex_setup);
    L_cmd        = hex_obj_ik.axisPos;

    % Platform pose corresponds to pose_platform set by IK.
    X_plat_true  = hex_obj_ik.pose_platform;

    % FK solve from lengths.
    [X_plat_solved, conv] = ForwardKinematics_hexapod( ...
        hex_obj, L_cmd, X_plat_home);
    assert(conv, 'display-frame FK failed to converge');
    err_plat = max(abs(X_plat_solved - X_plat_true));

    % Apply the display transform and compare to the original input.
    plat_CM  = X_plat_solved(1:3);
    E        = X_plat_solved(4:6);
    R        = E2R(E);
    % Reverse transform chain (matches EffectorFromPlatformPose).
    T_world_plat_row = struct('R', R, 't', plat_CM);
    T_world_POI_row  = composeTransform(T_world_plat_row, hex_obj.T_platform_POI);
    T_world_datum_row = composeTransform(hex_obj.T_world_datum_platform, hex_obj.T_platform_POI);
    T_datum_POI_row  = composeTransform(invertTransform(T_world_datum_row), T_world_POI_row);
    disp_pose_row    = transformToPose(T_datum_POI_row);
    disp_xyz         = disp_pose_row(1:3);
    disp_pose_out = [disp_xyz; E];
    err_disp = max(abs(disp_pose_out - disp_pose_in));

    fprintf('  step %d: plat-err=%.2e  disp-err=%.2e\n', n, err_plat, err_disp);
    assert(err_disp < tol_pose, 'display-frame round-trip failed (err=%g)', err_disp);
end

fprintf('\nPASS  test_ForwardKinematics (%d/%d cold + warm-start + display-frame)\n', ...
    nPass, size(testPoses, 1));
end


function q = lengthsFromPlatformPose(X_plat, hex_obj)
% Closed-form computation of link lengths for a given platform pose,
% matching the math inside ForwardKinematics_hexapod's private helper.
% Lets this test generate ground-truth lengths without rerunning the
% full InverseKinematics_hexapod (which would also compute U-joint
% angles, joint separations, etc. - all unused here).
plat_CM = X_plat(1:3);
E       = X_plat(4:6);
R       = E2R(E);
l_W     = R * hex_obj.plat_link_0 + plat_CM;
link    = l_W - hex_obj.base_link;
q       = vecnorm(link, 2, 1)';
end
