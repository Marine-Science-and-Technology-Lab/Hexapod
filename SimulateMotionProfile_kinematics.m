
function [hex_path]=SimulateMotionProfile_kinematics(hex_obj,hex_setup,hex_path)


Time=hex_path.T;
dt=hex_path.dt;


CurrentPose=hex_obj.pose;  
hex_path.pose_t_relative=hex_path.pose_t;
hex_path.pose_t=hex_path.pose_t_relative+repmat(CurrentPose,1,length(Time));

r0=hex_path.pose_t(1:3,:); E=(hex_path.pose_t(4:6,:)); % Timeseries of end effector pose, measured from home position;
r=r0+repmat(hex_obj.Home,1,size(r0,2));
r_rel=hex_obj.r_rel;

        r_dt = gradient(r)./dt;
        E_dt = gradient(E)./dt;
        r_ddt = gradient(r_dt)./dt;
        E_ddt = gradient(E_dt)./dt;

  base = hex_obj.base; % need the locations of the base joints in world frame
 base_link=hex_obj.base_link; % Attachment points of base U-joints. Includes vertical offset equal to base_Zlink parameter (set in initialization function)
      
 plat = hex_obj.plat0; % need the locations of the platform joints in platform frame (assume when E=0, the world and platform frames are aligned)
    plat_link=hex_obj.plat_link_0; % Attachment points of platform-side Ujoints in platform coordinates. Includes local plat_Zlink offset for yoke height.
   
    
    z_min = hex_obj.z; % need minimum vertical distance for visualization purposes
    L0 = hex_obj.L0; % need minimum link length
    dL = hex_obj.dL; % need link stroke length

    % U-joint kinematics
    ujoint_angle = [-70 70 50 -170 170 -50];
    % Convert angles to unit vector
    u_hat = [cosd(ujoint_angle(1)) cosd(ujoint_angle(2)) cosd(ujoint_angle(3)) cosd(ujoint_angle(4)) cosd(ujoint_angle(5)) cosd(ujoint_angle(6));
    sind(ujoint_angle(1)) sind(ujoint_angle(2)) sind(ujoint_angle(3)) sind(ujoint_angle(4)) sind(ujoint_angle(5)) sind(ujoint_angle(6));
    0 0 0 0 0 0];

    % w_hat for the AB joint
    w_hat = [0;0;-1];
   
    clearvars plati lhat joint_AB joint_CD;

for j = 1:size(r,2)

     R = E2R(E(:,j)); % convert Euler angles to rotation matrix
    %hex_path.rmatrix(:,:,j)=R;
    z_dir = R(:,3);
    plat_CM = r(:,j) - R*r_rel; % platform CM position resolved in world frame; [m]

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