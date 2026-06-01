function E = R2E(R)
% R2E - Extract ZYX body-fixed Euler angles from a rotation matrix.
%
%   E = R2E(R)
%
%   Inverse of E2R. Given a rotation matrix R such that R = E2R([roll;
%   pitch; yaw]), returns the 3x1 Euler-angle vector [roll; pitch; yaw]
%   in radians.
%
%   Extraction uses the standard atan2-based formulas:
%       pitch = asin(-R(3,1))
%       roll  = atan2(R(3,2), R(3,3))
%       yaw   = atan2(R(2,1), R(1,1))
%   These are singular when pitch approaches +/- pi/2 (gimbal lock). The
%   hexapod hardware limits pitch to +/- 40 degrees, well clear of the
%   singularity, so no guard is implemented.

E = zeros(3, 1);
E(2) = asin(-R(3,1));
E(1) = atan2(R(3,2), R(3,3));
E(3) = atan2(R(2,1), R(1,1));
end
