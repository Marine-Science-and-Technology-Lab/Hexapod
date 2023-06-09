function [fig_out]=SimulateMotionProfile(hex_obj,hex_setup,hex_path)


Time=hex_path.T;
dt=hex_path.dt;

r0=hex_path.pose_t(1:3,:); E=hex_path.pose_t(4:6,:);
r=r0+repmat(hex_obj.Home,1,size(r0,2));
r_rel=hex_obj.r_rel;

        r_dt = gradient(r)./dt;
        E_dt = gradient(E)./dt;
        r_ddt = gradient(r_dt)./dt;
        E_ddt = gradient(E_dt)./dt;

 base = hex_obj.base; % need the locations of the base joints in world frame
    plat = hex_obj.plat0; % need the locations of the platform joints in platform frame (assume when E=0, the world and platform frames are aligned)
    z_min = hex_obj.z; % need minimum vertical distance for visualization purposes
    L0 = hex_obj.L0; % need minimum link length
    dL = hex_obj.dL; % need link stroke length
   

for j = 1:size(r,2)

     R = E2R(E(:,j)); % convert Euler angles to rotation matrix
    plat_CM = r(:,j) - R*r_rel; % platform CM position resolved in world frame; [m]

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