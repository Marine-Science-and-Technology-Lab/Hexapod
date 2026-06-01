function hex_path = ReturnToDatumPath(hex_obj, hex_path)
% ReturnToDatumPath - Generate a smooth 2 s trajectory from the current
% display-frame pose (hex_obj.pose) back to the datum-frame origin
% [0;0;0;0;0;0]. Uses squashfunction S-curves per DOF so the velocity
% is zero at both endpoints.
%
% Populates hex_path.T and hex_path.pose_t; caller then runs
% SimulateMotionProfile_kinematics / StreamContourData_to_Galil as with
% any other planned motion.
%
% IMPORTANT: the trajectory is produced in RELATIVE form (starting at
% [0;0;0;0;0;0] and ending at -current_pose), matching the convention
% used by JogFromCurrentPosition. SimulateMotionProfile_kinematics adds
% hex_obj.pose to every sample, yielding an absolute trajectory that
% runs from current_pose to [0;0;0;0;0;0] — which is what we want.
% Producing an absolute trajectory here (current -> 0) would double up
% with that addition and cause the platform to drift per press.

Tdur = 2;
dt   = hex_path.dt;
ttemp = 0:dt:Tdur;

delta = -hex_obj.pose(:);     % relative move needed to reach the datum origin

pose_t = zeros(6, length(ttemp));
for j = 1:6
    pose_t(j, :) = squashfunction(ttemp, 0, delta(j));
end

hex_path.T      = ttemp;
hex_path.pose_t = pose_t;
end
