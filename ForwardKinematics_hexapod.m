function [pose_plat, converged, iters, residual] = ForwardKinematics_hexapod( ...
    hex_obj, L_measured, X_seed_plat, opts)
% ForwardKinematics_hexapod - Solve platform pose from measured actuator
% lengths using Newton-Raphson with a numerical Jacobian.
%
%   [pose_plat, converged, iters, residual] = ...
%       ForwardKinematics_hexapod(hex_obj, L_measured, X_seed_plat, opts)
%
%   The solver works entirely in platform-pose coordinates:
%       pose_plat = [plat_CM_x; plat_CM_y; plat_CM_z; roll; pitch; yaw]
%   where plat_CM is the platform center-of-mass position in the base
%   frame (meters) and (roll, pitch, yaw) are platform orientation Euler
%   angles (radians, ZYX body-fixed, matching E2R.m).
%
%   Only the mechanical geometry (hex_obj.plat_link_0 and hex_obj.base_link)
%   is used. The user-settable frame transforms (hex_obj.T_platform_POI
%   and hex_obj.T_world_datum_platform) are DISPLAY-FRAME transforms that
%   the caller applies after this function returns. Changing them has no
%   effect on the FK solve - they only move the point we report pose
%   relative to.
%
%   Algorithm:
%     f(X) = link_lengths(X) - L_measured
%     Newton step:  X <- X - J(X) \ f(X)
%     Jacobian via central differences.
%
%   Inputs
%     hex_obj      - hexapod struct (needs .plat_link_0 (3x6) and
%                    .base_link (3x6); other fields ignored).
%     L_measured   - 6x1 measured actuator lengths, meters.
%     X_seed_plat  - 6x1 initial guess in platform-pose space. For
%                    real-time tracking, pass the last solved pose so
%                    Newton typically converges in 1-2 iterations.
%     opts         - optional struct of knobs:
%                      .tol     (default 1e-6 m) L_inf residual stop
%                      .maxiter (default 8)
%                      .delta   (default 1e-5)   finite-difference step
%
%   Outputs
%     pose_plat    - 6x1 solved platform pose (returns the last Newton
%                    iterate whether or not it converged).
%     converged    - true if residual < tol within maxiter.
%     iters        - Newton steps actually taken (0 if the seed already
%                    satisfies tol; capped at maxiter).
%     residual     - final L_inf residual in meters.
%
%   See also: InverseKinematics_hexapod, EncoderToLength,
%             updateDigitalShadow, Docs/UPGRADE_PLAN.md §5.3

if nargin < 4 || isempty(opts); opts = struct(); end
if ~isfield(opts, 'tol');     opts.tol     = 1e-10; end
if ~isfield(opts, 'maxiter'); opts.maxiter = 8;     end
if ~isfield(opts, 'delta');   opts.delta   = 1e-5;  end

X = X_seed_plat(:);
L_measured = L_measured(:);

plat_link = hex_obj.plat_link_0;
base_link = hex_obj.base_link;

converged = false;
residual  = inf;
iters     = 0;

% Loop structure counts Newton STEPS (not residual-check passes). A seed
% that already satisfies tol reports iters=0; every subsequent pass that
% actually takes a step increments iters.
while true
    L_X = computeLengths(X, plat_link, base_link);
    f   = L_X - L_measured;
    residual = max(abs(f));

    if residual < opts.tol
        converged = true;
        break
    end
    if iters >= opts.maxiter
        break
    end

    % Numerical Jacobian via central differences (6 DOFs, 12 evaluations).
    J = zeros(6, 6);
    d = opts.delta;
    for j = 1:6
        Xp = X; Xp(j) = Xp(j) + d;
        Xm = X; Xm(j) = Xm(j) - d;
        Lp = computeLengths(Xp, plat_link, base_link);
        Lm = computeLengths(Xm, plat_link, base_link);
        J(:, j) = (Lp - Lm) / (2 * d);
    end

    % Newton step; fall out if the Jacobian is singular.
    lastwarn('');
    dX = -(J \ f);
    [~, warnId] = lastwarn;
    if strcmp(warnId, 'MATLAB:singularMatrix') || ...
            strcmp(warnId, 'MATLAB:nearlySingularMatrix') || ...
            any(~isfinite(dX))
        break
    end

    X = X + dX;
    iters = iters + 1;
end

pose_plat = X;
end


function q = computeLengths(X_plat, plat_link, base_link)
% Link lengths for a given platform pose. X_plat = [plat_CM; E]. Uses
% only the fixed mechanical geometry - no CG offset, no Home offset.
plat_CM = X_plat(1:3);
E       = X_plat(4:6);
R       = E2R(E);
l_W     = R * plat_link + plat_CM;   % 3 x 6 (implicit expansion of plat_CM)
link    = l_W - base_link;            % 3 x 6
q       = vecnorm(link, 2, 1)';       % 6 x 1
end
