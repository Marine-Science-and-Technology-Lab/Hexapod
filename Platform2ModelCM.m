function [hex_obj]=Platform2ModelCM(hex_obj,p_pose)
% Given a platform pose p_pose = [plat_CM; E] in world frame, compute
% the POI (end-effector) world pose and store it as hex_obj.pose. Under
% the three-frame model this is just one side of EffectorFromPlatformPose;
% kept as a separate function for the callers that pass in an arbitrary
% p_pose rather than reading from hex_obj.pose_platform.

    r_plat = p_pose(1:3);
    E      = p_pose(4:6);
    R      = E2R(E);

    % POI world position = plat_CM + R * POI translation (in platform body).
    model_CM = r_plat + R * hex_obj.T_platform_POI.t;

    hex_obj.pose = [model_CM; E];
end