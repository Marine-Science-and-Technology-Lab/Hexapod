function T = poseToTransform(pose)
% poseToTransform - Convert a 6-vector pose to a rigid transform struct.
%
%   T = poseToTransform(pose)
%
%   Inputs
%     pose - 6x1 vector [x; y; z; roll; pitch; yaw]. Euler angles in
%            radians, ZYX body-fixed (matching E2R).
%
%   Outputs
%     T - struct with fields
%           .R  3x3 rotation matrix
%           .t  3x1 translation vector
%
%   Inverse: transformToPose.

T.t = pose(1:3);
T.R = E2R(pose(4:6));
end
