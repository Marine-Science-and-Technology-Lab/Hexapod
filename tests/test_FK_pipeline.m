function test_FK_pipeline()
% test_FK_pipeline - End-to-end pipeline test for the digital shadow:
%
%   commanded pose -> IK (link lengths) -> LengthToEncoder (counts)
%     -> round() -> EncoderToLength (lengths back) -> FK -> solved pose
%
%   If this round-trip fails for pure-Z translation (or small pure
%   rotations), the bug is in the FK solver or one of the adjacent
%   helpers, not in the emulator or UI wiring.
%
%   Runs a focused set of cases and prints the Euler difference in
%   degrees for visual inspection, plus asserts a tolerance.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root);

[hex_obj, hex_setup] = InitializeHexapodObject();
jsi = load(fullfile(root, 'JointSep_Interpolants.mat'));
hex_setup.Joint_Interp = jsi.JointSep_Interpolants;
% Runs against default frame configuration: T_world_datum_platform at Home,
% T_platform_POI identity (both rotations identity, POI coincident with
% platform body origin, datum coincident with Home_platform).

cases = { ...
    [0; 0; 0; 0; 0; 0],               'home (no motion)'; ...
    [0; 0; -0.05; 0; 0; 0],           '-Z 50mm (pure translation)'; ...
    [0; 0; -0.02; 0; 0; 0],           '-Z 20mm'; ...
    [0.01; 0; 0; 0; 0; 0],            '+X 10mm'; ...
    [0; 0.01; 0; 0; 0; 0],            '+Y 10mm'; ...
    [0; 0; 0; deg2rad(3); 0; 0],      '+roll 3deg'; ...
    [0; 0; 0; 0; deg2rad(3); 0],      '+pitch 3deg'; ...
    [0; 0; 0; 0; 0; deg2rad(3)],      '+yaw 3deg'; ...
};

tol_deg = 0.01;   % pose Euler tolerance after full integer-count round-trip
max_euler_err = 0;

fprintf('\n%-30s  %-24s  %s\n', 'Case', 'FK Euler (deg)', 'max deg err');
fprintf('%s\n', repmat('-', 1, 80));

for row = 1:size(cases, 1)
    cmd_pose = cases{row, 1};
    label    = cases{row, 2};

    % Step 1: IK from commanded pose
    hex_obj.pose = cmd_pose;
    hex_obj_ik = InverseKinematics_hexapod(hex_obj, hex_setup);
    L_true = hex_obj_ik.axisPos;

    % Step 2: encode lengths -> counts (floating point)
    [counts_float, ~] = LengthToEncoder(hex_setup, L_true);

    % Step 3: round to integer counts (what the emulator / Galil sees)
    counts_int = round(counts_float);

    % Step 4: decode back to lengths (what updateDigitalShadow does)
    L_recovered = EncoderToLength(hex_setup, counts_int(:)');

    % Step 5: seed FK from commanded pose (same as updateDigitalShadow)
    T_DQ_cmd  = poseToTransform(cmd_pose);
    T_WD_cmd  = composeTransform(hex_obj.T_world_datum_platform, hex_obj.T_platform_POI);
    T_WQ_cmd  = composeTransform(T_WD_cmd, T_DQ_cmd);
    T_WP_cmd  = composeTransform(T_WQ_cmd, invertTransform(hex_obj.T_platform_POI));
    seed_plat = transformToPose(T_WP_cmd);

    % Step 6: solve FK
    [pose_plat, conv] = ForwardKinematics_hexapod(hex_obj, L_recovered(:), seed_plat);

    % Step 7: express as Euler degrees for comparison
    euler_solved_deg = rad2deg(pose_plat(4:6))';
    euler_cmd_deg    = rad2deg(cmd_pose(4:6))';
    euler_err_deg    = abs(euler_solved_deg - euler_cmd_deg);
    max_err          = max(euler_err_deg);
    max_euler_err    = max(max_euler_err, max_err);

    fprintf('  %-28s  [%+7.4f %+7.4f %+7.4f]  %8.4f  %s\n', ...
        label, euler_solved_deg, max_err, ternary(conv,'conv','NOCONV'));
end

fprintf('\nmax Euler error across all cases: %.4f deg\n', max_euler_err);

if max_euler_err > tol_deg
    fprintf('\nFAIL  test_FK_pipeline: Euler error %.4f deg exceeds tol %.4f deg\n', ...
        max_euler_err, tol_deg);
    error('test_FK_pipeline failed');
else
    fprintf('PASS  test_FK_pipeline (max %.4f deg error within %.4f deg tol)\n', ...
        max_euler_err, tol_deg);
end
end


function s = ternary(cond, a, b)
if cond; s = a; else; s = b; end
end
