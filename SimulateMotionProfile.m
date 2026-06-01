function [fig_out]=SimulateMotionProfile(hex_obj,hex_setup,hex_path)


Time=hex_path.T;
dt=hex_path.dt;

r_DQ = hex_path.pose_t(1:3,:);
E_DQ = hex_path.pose_t(4:6,:);

% Constant part of the frame chain
T_WD       = composeTransform(hex_obj.T_world_datum_platform, hex_obj.T_platform_POI);
T_PQ_R_inv = hex_obj.T_platform_POI.R';
T_PQ_t_neg = -T_PQ_R_inv * hex_obj.T_platform_POI.t;

N = size(r_DQ, 2);

% POI world trajectory (for derivatives, matching pre-refactor semantics)
t_WQ = T_WD.R * r_DQ + T_WD.t;

        r_dt  = gradient(t_WQ) ./ dt;
        E_dt  = gradient(E_DQ) ./ dt;
        r_ddt = gradient(r_dt) ./ dt;
        E_ddt = gradient(E_dt) ./ dt;

 base = hex_obj.base; % need the locations of the base joints in world frame
    plat = hex_obj.plat0; % need the locations of the platform joints in platform frame (assume when E=0, the world and platform frames are aligned)
    z_min = hex_obj.z; % need minimum vertical distance for visualization purposes
    L0 = hex_obj.L0; % need minimum link length
    dL = hex_obj.dL; % need link stroke length


for j = 1:N

    % Per-timestep T_world_plat via the frame chain.
    T_DQ_j = struct('R', E2R(E_DQ(:,j)), 't', r_DQ(:,j));
    T_WQ_j = composeTransform(T_WD, T_DQ_j);
    T_WP_j = composeTransform(T_WQ_j, struct('R', T_PQ_R_inv, 't', T_PQ_t_neg));
    R       = T_WP_j.R;
    plat_CM = T_WP_j.t;

    link = zeros(3,6); % each column is a vector describing a link
    p_W = zeros(3,6); % platform link joints resolved in a world frame
    q = zeros(6,1); % link lengths; [m]
    l_hat = zeros(3,6); % unit vectors describing longitudinal axis of links (base to platform) resolved in world frame

    for i = 1:6
        p_W(:,i) = plat_CM + R*plat(:,i);
        link(:,i) = p_W(:,i) - base(:,i);
        q(i) = sqrt(link(:,i)'*link(:,i));
        l_hat(:,i) = link(:,i)./q(i);
    end

    if sum(q >= L0) ~= 6 || sum(q <= L0+dL) ~= 6
        check(j) = 1;
    end
    linkl(:,j)=q(:);

end

linkv(:,j)=gradient(linkl)/dt;
linkacc(:,j)=gradient(linkv)/dt;


hex_path.pose_dt=[r_dt;E_dt]; %Rate of change of platform pose
hex_path.pose_ddt=[r_ddt;E_ddt]; %Platform accelerations in world frame
hex_path.axis_t=linkl;
hex_path.axis_dt=linkv;
hex_path.axis_ddt=linkacc;