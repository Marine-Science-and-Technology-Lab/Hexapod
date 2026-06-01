
function [hex_path]=SimulateMotionProfile_kinematics_legacy(hex_obj,hex_setup,hex_path)
% Legacy scalar implementation retained only so the vectorized rewrite
% can be regression-tested against it. New callers should use
% SimulateMotionProfile_kinematics.m.


Time=hex_path.T;
dt=hex_path.dt;

% By default, the motion is assumed to be relative to the current pose. If
% absolute motion is specified (e.g. in the point-to-point dialog), the
% current pose is subtracted from each entry in the pose_t array when the
% hex_path structure is generated.
CurrentPose=hex_obj.pose;  
hex_path.pose_t_relative=hex_path.pose_t; % Create the relative pose array (this is the array generated from the motion-planning functions)
hex_path.pose_t=hex_path.pose_t_relative+repmat(CurrentPose,1,length(Time)); %Generate a new absolute (relative to home position) pose array from the relative by adding the starting pose vector to each timestep.

r_DQ = hex_path.pose_t(1:3,:);
E_DQ = hex_path.pose_t(4:6,:);

% Constant frame-chain pieces
T_WD       = composeTransform(hex_obj.T_world_datum_platform, hex_obj.T_platform_POI);
T_PQ_R_inv = hex_obj.T_platform_POI.R';
T_PQ_t_neg = -T_PQ_R_inv * hex_obj.T_platform_POI.t;

% POI world position trajectory (for derivatives - matches the pre-
% refactor semantic where "r" was POI world position).
t_WQ = T_WD.R * r_DQ + T_WD.t;

        r_dt  = gradient(t_WQ) ./ dt;
        E_dt  = gradient(E_DQ) ./ dt;
        r_ddt = gradient(r_dt) ./ dt;
        E_ddt = gradient(E_dt) ./ dt;

  base = hex_obj.base; % need the locations of the base joints in world frame
 base_link=hex_obj.base_link; % Attachment points of base U-joints. Includes vertical offset equal to base_Zlink parameter (set in initialization function)
      
 plat = hex_obj.plat0; % need the locations of the platform joints in platform frame (assume when E=0, the world and platform frames are aligned)
    plat_link=hex_obj.plat_link_0; % Attachment points of platform-side Ujoints in platform coordinates. Includes local plat_Zlink offset for yoke height.
   
    
    z_min = hex_obj.z; % need minimum vertical distance for visualization purposes
    L0 = hex_obj.L0; % need minimum link length
    dL = hex_obj.dL; % need link stroke length

    % U-joint kinematics
    ujoint_angle = hex_setup.YokeA.Uhat;
    % Convert angles to unit vector
    u_hat = [cosd(ujoint_angle(1)) cosd(ujoint_angle(2)) cosd(ujoint_angle(3)) cosd(ujoint_angle(4)) cosd(ujoint_angle(5)) cosd(ujoint_angle(6));
    sind(ujoint_angle(1)) sind(ujoint_angle(2)) sind(ujoint_angle(3)) sind(ujoint_angle(4)) sind(ujoint_angle(5)) sind(ujoint_angle(6));
    0 0 0 0 0 0];

    % w_hat for the AB joint
    w_hat = [0;0;-1];
   
    clearvars plati lhat joint_AB joint_CD;

N_timesteps = size(r_DQ, 2);
for j = 1:N_timesteps

    % Per-timestep T_world_plat via the frame chain.
    T_DQ_j = struct('R', E2R(E_DQ(:,j)), 't', r_DQ(:,j));
    T_WQ_j = composeTransform(T_WD, T_DQ_j);
    T_WP_j = composeTransform(T_WQ_j, struct('R', T_PQ_R_inv, 't', T_PQ_t_neg));
    R       = T_WP_j.R;
    plat_CM = T_WP_j.t;
    z_dir   = R(:,3);

    link = zeros(3,6); % each column is a vector describing a link
    p_W = zeros(3,6); % platform link joints resolved in a world frame
    l_W = zeros(3,6); % platform linkage joints resolved in a world frame
   
    q = zeros(6,1); % link lengths; [m]
    l_hat = zeros(3,6); % unit vectors describing longitudinal axis of links (base to platform) resolved in world frame

    for i = 1:6
        p_W(:,i) = plat_CM + R*plat(:,i); %Platform vertices.
        l_W(:,i)= plat_CM+ R*(plat_link(:,i)); %Platform yoke centers
        link(:,i) = l_W(:,i) - (base_link(:,i));
        q(i) = sqrt(link(:,i)'*link(:,i));
        l_hat(:,i) = link(:,i)./q(i);
        hex_path.lhat(:,i,j)=l_hat(:,i);

        % Calculating U-joint Kinematics for joint AB and joint CD
         % Computing AB joint Kinematics
        u_temp = u_hat(:,i);
        l_temp =l_hat(:,i);
        v_temp_AB = cross(u_temp,l_temp);
        c_temp = cross(u_temp,v_temp_AB);
        v_star_temp = v_temp_AB - (v_temp_AB.'*w_hat)*w_hat;
        angle_theta(i) = acos((v_temp_AB).'*v_star_temp/(norm(v_temp_AB)*norm(v_star_temp)));
        angle_phi(i) = acos((-1*l_temp).'*c_temp/(norm(-1*l_temp)*norm(c_temp)));
        % v(:,i) = v_temp;
        % c(:,i) = c_temp;
        % v_star(:,i) = v_star_temp;
    
        % Computing CD joint kinematics
        q_temp = R*u_temp;
        v_temp_CD = cross(q_temp,l_temp);
        r_temp = cross(q_temp,v_temp_CD);
        v_star_temp2 = v_temp_CD - (v_temp_CD.'*z_dir)*z_dir;
        angle_psi(i) = acos((v_temp_CD).'*v_star_temp2/(norm(v_temp_CD)*norm(v_star_temp2)));
        angle_alpha(i) = acos((-1*l_temp).'*r_temp/(norm(-1*l_temp)*norm(r_temp)));
        %v_star2(:,i) = v_star_temp2;
        % q(:,i) = q_temp;
        % r(:,i) = r_temp;
    end

    hex_path.plati(:,:,j)=p_W;
    hex_obj.plat_link_i=l_W;

joint_AB.angle_theta(:,j) = angle_theta;
    joint_AB.angle_phi(:,j) =angle_phi;
   joint_CD.angle_alpha(:,j) = angle_alpha;
   joint_CD.angle_psi(:,j) = angle_psi;

%     if sum(q >= L0) ~= 6 || sum(q <= L0+dL) ~= 6
%         check(j) = 1;
%     end
    linkl(:,j)=q(:);

    
end

hex_path.joint_AB=joint_AB; hex_path.joint_CD=joint_CD;

linkv=gradient(linkl)/dt;
linkacc=gradient(linkv)/dt;
hex_obj.link_vector=link;

hex_path.pose_dt=[r_dt;E_dt]; %Rate of change of platform pose
hex_path.pose_ddt=[r_ddt;E_ddt]; %Platform accelerations in world frame
hex_path.axis_t=linkl;
hex_path.axis_dt=linkv;
hex_path.axis_ddt=linkacc;

[hex_path.axis_cts null]=LengthToEncoder(hex_setup,linkl);

hex_path.joint_separation.AB=hex_setup.Joint_Interp.SCAT_AB(rad2deg(hex_path.joint_AB.angle_theta),rad2deg(hex_path.joint_AB.angle_phi));
hex_path.joint_separation.CD=hex_setup.Joint_Interp.SCAT_CD(rad2deg(hex_path.joint_CD.angle_alpha),rad2deg(hex_path.joint_CD.angle_psi));

hex_path.collisioncheck=max([hex_path.joint_separation.AB' hex_path.joint_separation.CD']<=hex_setup.collisionthreshold);

% [fname fpath]=uiputfile()
% save([fpath fname],'hex_path','-mat')