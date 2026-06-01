function hex_obj = EffectorFromPlatformPose(hex_obj)
% EffectorFromPlatformPose - Given the platform's world pose, compute the
% display-frame pose of the POI (end-effector reference point) and write
% it into hex_obj.pose.
%
% Input:  hex_obj.pose_platform = [plat_CM_world; platform_Euler_world]
% Output: hex_obj.pose          = [T_datum_POI as 6-vector]
%
% Transform chain:
%   T_world_plat  = (pose_platform)
%   T_world_POI   = T_world_plat * T_platform_POI
%   T_datum_POI   = invert(T_world_datum) * T_world_POI
% where T_world_datum = T_world_datum_platform * T_platform_POI.

T_world_plat  = poseToTransform(hex_obj.pose_platform);
T_world_POI   = composeTransform(T_world_plat, hex_obj.T_platform_POI);
T_world_datum = composeTransform(hex_obj.T_world_datum_platform, hex_obj.T_platform_POI);
T_datum_POI   = composeTransform(invertTransform(T_world_datum), T_world_POI);
hex_obj.pose  = transformToPose(T_datum_POI);
end
