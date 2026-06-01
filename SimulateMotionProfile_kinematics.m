function [hex_path] = SimulateMotionProfile_kinematics(hex_obj, hex_setup, hex_path)
% SimulateMotionProfile_kinematics - Compute link lengths, platform-frame
% joint positions, and U-joint (AB / CD) yoke angles across a commanded
% pose trajectory.
%
% Vectorized implementation: a single pass over all N timesteps using
% batched 3x3xN rotation matrices and pagemtimes (R2020b+). Output struct
% layout matches SimulateMotionProfile_kinematics_legacy.m byte-for-byte
% within numerical tolerance. See Merged/Docs/UPGRADE_PLAN.md §2.1.
%
% Performance: the nine 3x3xN / 3x6xN scratch tensors are cached in a
% persistent struct keyed on N. Consecutive calls at the same trajectory
% length reuse the allocations, avoiding MATLAB's zero-fill and heap
% churn. Very large trajectories (> MAX_CACHE_N samples) bypass the
% cache to avoid pinning many megabytes of memory indefinitely.

persistent scratch
MAX_CACHE_N = 20000;

Time = hex_path.T;
dt   = hex_path.dt;

% Build the absolute pose trajectory from the relative one.
CurrentPose = hex_obj.pose;
hex_path.pose_t_relative = hex_path.pose_t;
hex_path.pose_t = hex_path.pose_t_relative + CurrentPose;   % implicit expansion

% Per-timestep POI pose in datum frame. These *are* the pose commands
% that came out of the trajectory planner.
r_DQ = hex_path.pose_t(1:3, :);   % 3 x N  POI translation in datum frame
E_DQ = hex_path.pose_t(4:6, :);   % 3 x N  POI Euler angles in datum frame (ZYX)

base_link = hex_obj.base_link;   % 3 x 6
plat      = hex_obj.plat0;       % 3 x 6
plat_link = hex_obj.plat_link_0; % 3 x 6

% U-joint reference unit vectors (one per joint; constant across time)
ujoint_angle = hex_setup.YokeA.Uhat;
u_hat = [cosd(ujoint_angle(:).'); sind(ujoint_angle(:).'); zeros(1, 6)];  % 3 x 6
w_hat = [0; 0; -1];

% --- Batch POI rotation matrices R_DQ from the commanded Euler -------
N  = size(r_DQ, 2);
cE = cos(E_DQ); sE = sin(E_DQ);   % each is 3 x N

% Acquire scratch tensors from the cache if N matches, otherwise
% allocate fresh. The cache itself is bypassed for very long
% trajectories to keep peak memory bounded.
use_cache = (N <= MAX_CACHE_N);
if use_cache && ~isempty(scratch) && scratch.N == N
    R_DQ = scratch.R_DQ;
    v_AB = scratch.v_AB;
    c    = scratch.c;
    v_CD = scratch.v_CD;
    r_CD = scratch.r_CD;
else
    R_DQ = zeros(3, 3, N);
    v_AB = zeros(3, 6, N);
    c    = zeros(3, 6, N);
    v_CD = zeros(3, 6, N);
    r_CD = zeros(3, 6, N);
end

R_DQ(1,1,:) = cE(3,:) .* cE(2,:);
R_DQ(1,2,:) = cE(3,:) .* sE(2,:) .* sE(1,:) - sE(3,:) .* cE(1,:);
R_DQ(1,3,:) = cE(3,:) .* sE(2,:) .* cE(1,:) + sE(3,:) .* sE(1,:);
R_DQ(2,1,:) = sE(3,:) .* cE(2,:);
R_DQ(2,2,:) = sE(3,:) .* sE(2,:) .* sE(1,:) + cE(3,:) .* cE(1,:);
R_DQ(2,3,:) = sE(3,:) .* sE(2,:) .* cE(1,:) - cE(3,:) .* sE(1,:);
R_DQ(3,1,:) = -sE(2,:);
R_DQ(3,2,:) = cE(2,:) .* sE(1,:);
R_DQ(3,3,:) = cE(2,:) .* cE(1,:);

% --- Frame chain: T_world_plat = T_WD * T_DQ * invert(T_PQ) ----------
% Constants:
T_WD = composeTransform(hex_obj.T_world_datum_platform, hex_obj.T_platform_POI);  % T_world_datum
T_PQ_R_inv = hex_obj.T_platform_POI.R';
T_PQ_t_neg = -T_PQ_R_inv * hex_obj.T_platform_POI.t;                               % 3 x 1

% Per-timestep T_world_POI: R = T_WD.R * R_DQ, t = T_WD.R * r_DQ + T_WD.t
R_WQ = pagemtimes(T_WD.R, R_DQ);                                                   % 3 x 3 x N
t_WQ = T_WD.R * r_DQ + T_WD.t;                                                     % 3 x N

% Per-timestep T_world_plat: R = R_WQ * T_PQ.R', t = R_WQ * (-T_PQ.R' * T_PQ.t) + t_WQ
R = pagemtimes(R_WQ, T_PQ_R_inv);                                                  % 3 x 3 x N (platform orientation)
plat_CM = reshape(pagemtimes(R_WQ, T_PQ_t_neg), 3, N) + t_WQ;                      % 3 x N (platform CM world)

% Derivatives - computed on the POI trajectory (same semantics as the
% pre-frame-refactor code, which used POI translation and platform/POI
% Euler interchangeably because they were assumed identical).
r_dt   = gradient(t_WQ)  ./ dt;
E_dt   = gradient(E_DQ)  ./ dt;
r_ddt  = gradient(r_dt)  ./ dt;
E_ddt  = gradient(E_dt)  ./ dt;

% z_dir per timestep (platform local +Z resolved in world)
z_dir = reshape(R(:, 3, :), 3, N);                 % 3 x N

plat_CM3 = reshape(plat_CM, 3, 1, N);              % for broadcast to 3 x 6 x N

% --- Joint positions resolved in world frame ---------------------------
p_W = pagemtimes(R, plat)      + plat_CM3;          % 3 x 6 x N
l_W = pagemtimes(R, plat_link) + plat_CM3;          % 3 x 6 x N
link = l_W - base_link;                             % 3 x 6 x N (base_link broadcasts over N)

q     = vecnorm(link, 2, 1);                        % 1 x 6 x N
l_hat = link ./ q;                                  % 3 x 6 x N

hex_path.lhat  = l_hat;
hex_path.plati = p_W;

linkl = reshape(q, 6, N);                           % 6 x N

% --- U-joint kinematics (AB and CD) ------------------------------------
% u_hat is 3 x 6, constant in time; broadcast over the 3rd dim.
U = u_hat;                                          % 3 x 6 (broadcasts)

% v_AB = cross(u_hat, l_hat), componentwise (avoids repmat; reuses the
% cached v_AB scratch tensor from above).
v_AB(1, :, :) = U(2, :) .* l_hat(3, :, :) - U(3, :) .* l_hat(2, :, :);
v_AB(2, :, :) = U(3, :) .* l_hat(1, :, :) - U(1, :) .* l_hat(3, :, :);
v_AB(3, :, :) = U(1, :) .* l_hat(2, :, :) - U(2, :) .* l_hat(1, :, :);

% c = cross(u_hat, v_AB)
c(1, :, :) = U(2, :) .* v_AB(3, :, :) - U(3, :) .* v_AB(2, :, :);
c(2, :, :) = U(3, :) .* v_AB(1, :, :) - U(1, :) .* v_AB(3, :, :);
c(3, :, :) = U(1, :) .* v_AB(2, :, :) - U(2, :) .* v_AB(1, :, :);

% v_star = v_AB - (v_AB . w_hat) * w_hat
vAB_dot_w  = sum(v_AB .* w_hat, 1);                 % 1 x 6 x N
v_star_AB  = v_AB - vAB_dot_w .* w_hat;

% angle_theta = acos((v_AB . v_star_AB) / (|v_AB| * |v_star_AB|))
vAB_dot_star = sum(v_AB .* v_star_AB, 1);
vAB_norm     = vecnorm(v_AB, 2, 1);
vstar_norm   = vecnorm(v_star_AB, 2, 1);
angle_theta  = reshape(acos(vAB_dot_star ./ (vAB_norm .* vstar_norm)), 6, N);

% angle_phi = acos((-l_hat . c) / (|l_hat| * |c|))
l_dot_c  = sum(l_hat .* c, 1);
l_norm   = vecnorm(l_hat, 2, 1);
c_norm   = vecnorm(c, 2, 1);
angle_phi = reshape(acos(-l_dot_c ./ (l_norm .* c_norm)), 6, N);

% CD joint: q_temp = R * u_hat(:,i) for each time step
Q = pagemtimes(R, u_hat);                           % 3 x 6 x N

% v_CD = cross(Q, l_hat) (reuses cached v_CD scratch tensor)
v_CD(1, :, :) = Q(2, :, :) .* l_hat(3, :, :) - Q(3, :, :) .* l_hat(2, :, :);
v_CD(2, :, :) = Q(3, :, :) .* l_hat(1, :, :) - Q(1, :, :) .* l_hat(3, :, :);
v_CD(3, :, :) = Q(1, :, :) .* l_hat(2, :, :) - Q(2, :, :) .* l_hat(1, :, :);

% r_CD = cross(Q, v_CD)
r_CD(1, :, :) = Q(2, :, :) .* v_CD(3, :, :) - Q(3, :, :) .* v_CD(2, :, :);
r_CD(2, :, :) = Q(3, :, :) .* v_CD(1, :, :) - Q(1, :, :) .* v_CD(3, :, :);
r_CD(3, :, :) = Q(1, :, :) .* v_CD(2, :, :) - Q(2, :, :) .* v_CD(1, :, :);

% v_star_CD = v_CD - (v_CD . z_dir) * z_dir, with z_dir broadcast over joints
z_dir3     = reshape(z_dir, 3, 1, N);
vCD_dot_z  = sum(v_CD .* z_dir3, 1);
v_star_CD  = v_CD - vCD_dot_z .* z_dir3;

vCD_dot_star = sum(v_CD .* v_star_CD, 1);
vCD_norm     = vecnorm(v_CD, 2, 1);
vstar2_norm  = vecnorm(v_star_CD, 2, 1);
angle_psi    = reshape(acos(vCD_dot_star ./ (vCD_norm .* vstar2_norm)), 6, N);

l_dot_r   = sum(l_hat .* r_CD, 1);
rCD_norm  = vecnorm(r_CD, 2, 1);
angle_alpha = reshape(acos(-l_dot_r ./ (l_norm .* rCD_norm)), 6, N);

% --- Assemble output ---------------------------------------------------
joint_AB = struct('angle_theta', angle_theta, 'angle_phi',   angle_phi);
joint_CD = struct('angle_alpha', angle_alpha, 'angle_psi',   angle_psi);
hex_path.joint_AB = joint_AB;
hex_path.joint_CD = joint_CD;

linkv   = gradient(linkl) / dt;
linkacc = gradient(linkv) / dt;

hex_path.pose_dt  = [r_dt;  E_dt];
hex_path.pose_ddt = [r_ddt; E_ddt];
hex_path.axis_t   = linkl;
hex_path.axis_dt  = linkv;
hex_path.axis_ddt = linkacc;

[hex_path.axis_cts, ~] = LengthToEncoder(hex_setup, linkl);

hex_path.joint_separation.AB = hex_setup.Joint_Interp.SCAT_AB( ...
    rad2deg(hex_path.joint_AB.angle_theta), rad2deg(hex_path.joint_AB.angle_phi));
hex_path.joint_separation.CD = hex_setup.Joint_Interp.SCAT_CD( ...
    rad2deg(hex_path.joint_CD.angle_alpha), rad2deg(hex_path.joint_CD.angle_psi));

hex_path.collisioncheck = max([hex_path.joint_separation.AB' hex_path.joint_separation.CD'] ...
    <= hex_setup.collisionthreshold);

% Persist scratch tensors for the next call at this same N.
if use_cache
    scratch = struct( ...
        'N',    N, ...
        'R_DQ', R_DQ, ...
        'v_AB', v_AB, ...
        'c',    c,    ...
        'v_CD', v_CD, ...
        'r_CD', r_CD);
end

end
