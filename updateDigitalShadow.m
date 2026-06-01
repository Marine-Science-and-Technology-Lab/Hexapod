function updateDigitalShadow(app)
% updateDigitalShadow - Timer-driven "digital shadow" update.
%
%   Polls the controller (real or emulator) for encoder counts, solves
%   forward kinematics to recover the platform pose, applies the
%   display-frame transforms, and writes the HexControl DRO panel.
%
%   Takes the app handle directly (rather than individual fields) so
%   that user-mutable display-frame transforms - hex_obj.T_platform_POI
%   and hex_obj.T_world_datum_platform - are picked up on every tick
%   without requiring re-Initialize. The FK solve itself uses only the
%   fixed mechanical geometry (plat_link_0, base_link) and is unaffected
%   by those changes.
%
%   Widgets updated (ActuatorPositions indices, per startupFcn wiring):
%      1..6    Ax1..Ax6 linear gauges     (actuator lengths, meters)
%      7..12   AxNumeric_1..AxNumeric_6   (raw encoder counts)
%     13..15   DRO_X, DRO_Y, DRO_Z        (end-effector position, meters)
%     16..18   DRO_Rx, DRO_Ry, DRO_Rz     (platform orientation, degrees)
%     19..24   AB1..AB6                   (joint separation AB)
%     25..30   CD1..CD6                   (joint separation CD)
%
%   Performance notes:
%     * Multi-rate dispatch (option B). Fast tier (every tick) runs
%       encoder + FK + pose DROs. Slow tier (every JOINT_SEP_DIVISOR
%       ticks) runs the joint-separation IK pass + its widgets.
%     * Change-detection on every widget write (option A). Values that
%       didn't move by more than a per-widget epsilon are not re-written.
%       UIFigure widget writes are expensive (~5-20 ms each through the
%       JavaScript bridge); skipping no-op writes saves real time when
%       the platform is slow-moving or stationary.
%     * FK skip-if-stationary (option F). If encoder counts are
%       unchanged from the previous tick, the cached last solution is
%       reused rather than re-running Newton.
%     * Graceful failure: on FK non-convergence or controller disconnect
%       the function silently returns. Timer stays alive.
%
%   See also: ForwardKinematics_hexapod, InverseKinematics_hexapod,
%             EncoderToLength, Docs/UPGRADE_PLAN.md §5.3

persistent tick_count last_counts last_pose_plat last_written
if isempty(tick_count);      tick_count      = 0;              end
if isempty(last_counts);     last_counts     = nan(1,6);       end
if isempty(last_pose_plat);  last_pose_plat  = nan(6,1);       end
if isempty(last_written);    last_written    = nan(1,30);      end
tick_count = tick_count + 1;

ACTIVE_SLOTS       = [1 2 3 5 6 7];    % Galil axes A B C E F G (skipping D)
N_ACT              = 6;
JOINT_SEP_DIVISOR  = 4;                % slow-tier: joint-sep runs once per N fast ticks

% Per-widget change-detection thresholds. If the new value differs from
% what we last wrote by at most this much, skip the widget write.
EPS_COUNT      = 0.5;         % encoder counts (integer)
EPS_LENGTH     = 1e-6;        % actuator length (m); 1 um
EPS_POS_M      = 1e-5;        % POI position (m); 10 um
EPS_ANGLE_DEG  = 0.01;        % POI Euler (deg)
EPS_JOINT_MM   = 1e-3;        % joint separation (mm in display); 1 um worth

g                 = app.g;
hex_obj           = app.hex_obj;
hex_setup         = app.hex_setup;
ActuatorPositions = app.ActuatorPositions;

try

    % ============================================================
    %  FAST TIER - every tick
    % ============================================================

    % 1. Poll encoder counts
    try
        resp = g.GCommand('TP');
    catch
        return   % controller busy/disconnected; skip this tick
    end
    if ~isfield(resp, 'string') || isempty(resp.string)
        return
    end
    parts = strsplit(strtrim(resp.string), ',');
    vals  = str2double(parts);
    if numel(vals) < max(ACTIVE_SLOTS) || any(isnan(vals(ACTIVE_SLOTS)))
        return
    end
    counts = vals(ACTIVE_SLOTS);                    % 1 x 6
    L_meas = EncoderToLength(hex_setup, counts);    % 1 x 6

    % 2. Push counts + lengths (change-detected)
    last_written = writeRange(ActuatorPositions, 1,  L_meas, EPS_LENGTH, last_written);
    last_written = writeRange(ActuatorPositions, 7,  counts, EPS_COUNT,  last_written);

    % 3. Solve FK (with skip-if-stationary)
    if isequaln(counts, last_counts) && all(isfinite(last_pose_plat))
        % Encoders haven't changed -> last solution is still correct.
        pose_plat = last_pose_plat;
        converged = true;
    else
        % Seed from current commanded pose (datum-space -> platform space).
        T_WD              = composeTransform(hex_obj.T_world_datum_platform, hex_obj.T_platform_POI);
        T_datum_POI_cmd   = poseToTransform(hex_obj.pose);
        T_world_POI_cmd   = composeTransform(T_WD, T_datum_POI_cmd);
        T_world_plat_cmd  = composeTransform(T_world_POI_cmd, invertTransform(hex_obj.T_platform_POI));
        seed_plat         = transformToPose(T_world_plat_cmd);

        [pose_plat, converged] = ForwardKinematics_hexapod( ...
            hex_obj, L_meas(:), seed_plat);

        if converged && all(isfinite(pose_plat))
            last_pose_plat = pose_plat;
            last_counts    = counts;
        end
    end

    if ~converged || any(~isfinite(pose_plat))
        return  % leave pose / joint-sep widgets at their prior values
    end

    % 4. Transform platform world pose -> POI display-frame pose
    T_WD            = composeTransform(hex_obj.T_world_datum_platform, hex_obj.T_platform_POI);
    T_world_plat_fk = poseToTransform(pose_plat);
    T_world_POI_fk  = composeTransform(T_world_plat_fk, hex_obj.T_platform_POI);
    T_datum_POI_fk  = composeTransform(invertTransform(T_WD), T_world_POI_fk);
    disp_pose       = transformToPose(T_datum_POI_fk);
    disp_xyz        = disp_pose(1:3);
    disp_E_deg      = rad2deg(disp_pose(4:6));

    % 5. Push pose DROs (change-detected)
    last_written = writeRange(ActuatorPositions, 13, disp_xyz,   EPS_POS_M,     last_written);
    last_written = writeRange(ActuatorPositions, 16, disp_E_deg, EPS_ANGLE_DEG, last_written);

    % ============================================================
    %  SLOW TIER - joint separation, every JOINT_SEP_DIVISOR-th tick
    % ============================================================
    if mod(tick_count, JOINT_SEP_DIVISOR) ~= 0
        return
    end

    % Run IK on the solved pose so the joint-separation interpolants
    % (and the rest of the derived hex_obj fields) are populated.
    hex_obj.pose = disp_pose;
    hex_obj = InverseKinematics_hexapod(hex_obj, hex_setup);

    last_written = writeRange(ActuatorPositions, 19, hex_obj.joint_separation.AB, EPS_JOINT_MM, last_written);
    last_written = writeRange(ActuatorPositions, 25, hex_obj.joint_separation.CD, EPS_JOINT_MM, last_written);

catch ME
    warning('updateDigitalShadow:tick', ...
        'Digital-shadow tick failed: %s', ME.message);
end
end


function last_written = writeRange(ActuatorPositions, start_idx, values, epsilon, last_written)
% Push a row of values into consecutive ActuatorPositions widgets, one
% widget per value starting at start_idx. Skips any widget whose new
% value is within epsilon of its last-written value (change detection).
for k = 1:numel(values)
    slot = start_idx + k - 1;
    v    = values(k);
    last = last_written(slot);
    if isnan(last) || abs(v - last) > epsilon
        ActuatorPositions{slot}.Value = v;
        last_written(slot) = v;
    end
end
end
