function [hex_obj]=Platform2ModelCM(hex_obj,p_pose)

r0 = p_pose(1:3); % assuming position is first three elements of pose (x,y,z)
    r=r0
    E = p_pose(4:6) % assuming Euler angles are next three elements of pose (phi,theta,psi)
    
    r_rel = hex_obj.r_rel; % need the relative position of the point of interest in platform frame
    
    % Inverse Kinematics
    R = E2R(E); % convert Euler angles to rotation matrix
    
   model_CM=r+R*r_rel
   
   
   hex_obj.pose=[model_CM; E];
   
   
end