function hex_obj = InverseKinematics_hexapod(hex_obj,hex_setup)
    
  
    pose=hex_obj.pose;
    r0 = pose(1:3); % assuming position is first three elements of pose (x,y,z)
    r=r0+hex_obj.Home;
    E = pose(4:6); % assuming Euler angles are next three elements of pose (phi,theta,psi)
    
    r_rel = hex_obj.r_rel; % need the relative position of the point of interest in platform frame
    base = hex_obj.base; % need the locations of the base joints in world frame
    plat = hex_obj.plat0; % need the locations of the platform joints in platform frame (assume when E=0, the world and platform frames are aligned)
    base_link=hex_obj.base_link;
    plat_link=hex_obj.plat_link_0;
    
    z_min = hex_obj.z; % need minimum vertical distance for visualization purposes
    MinLength = hex_setup.Actuators.MinLength     ; % need minimum link length
    MaxLength= hex_setup.Actuators.MaxLength      ; % Max actuator link length
%     dL = hex_obj.dL; % need link stroke length
    % Inverse Kinematics
    R = E2R(E); % convert Euler angles to rotation matrix
    plat_CM = r - R*r_rel; % platform CM position resolved in world frame; [m]
    hex_obj.pose_platform=[plat_CM; E];
    link = zeros(3,6); % each column is a vector describing a link
    p_W = zeros(3,6); % platform vertices resolved in a world frame
    l_W = zeros(3,6); % platform linkage joints resolved in a world frame
    q = zeros(6,1); % link lengths; [m]
    l_hat = zeros(3,6); % unit vectors describing longitudinal axis of links (base to platform) resolved in world frame

    ex=[1 0 0]';ey=[0 1 0]'; ez=[0 0 1]';
    px=R*ex;py=R*ey; pz=R*ez;

    for i = 1:6
        p_W(:,i) = plat_CM + R*(plat(:,i)); %Platform vertices
        l_W(:,i)= plat_CM+ R*(plat_link(:,i)); %Platform Ujoint Locations
        link(:,i) = l_W(:,i) - (base_link(:,i));
        q(i) = sqrt(link(:,i)'*link(:,i));
        l_hat(:,i) = link(:,i)./q(i);
        check(i) = q(i) < MinLength || q(i) > MaxLength;
    end
    
    hex_obj.plat_link_i=l_W;
    hex_obj.axisPos=q;
    hex_obj.axisCt=LengthToEncoder(hex_setup,q);
    
    hex_obj.plati=p_W;
hex_obj.ex=ex; hex_obj.ey=ey; hex_obj.ez=ez;
hex_obj.px=px; hex_obj.py=py; hex_obj.pz=pz;

 

 

end