function pose = transformToPose(T)
% transformToPose - Convert a rigid transform struct to a 6-vector pose.
%
%   pose = transformToPose(T)
%
%   Inputs
%     T - struct with fields .R (3x3 rotation) and .t (3x1 translation)
%
%   Outputs
%     pose - 6x1 vector [x; y; z; roll; pitch; yaw]. Euler angles in
%            radians, ZYX body-fixed (matching E2R / R2E).
%
%   Inverse: poseToTransform.

pose = zeros(6, 1);
pose(1:3) = T.t;
pose(4:6) = R2E(T.R);
end
