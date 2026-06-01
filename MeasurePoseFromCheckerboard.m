function [hex_obj, status] = MeasurePoseFromCheckerboard(hex_obj, axesHandle)
% MeasurePoseFromCheckerboard - UI-friendly wrapper around GrabCheckerboard.
%
%   [hex_obj, status] = MeasurePoseFromCheckerboard(hex_obj, axesHandle)
%
%   Prompts the user to pick a Basler PNG, detects the checkerboard
%   corners, solves for platform pose, renders the captured image into
%   axesHandle, and writes the solved pose into hex_obj.pose.
%
%   Inputs
%     hex_obj    - hexapod object (must have .Cam and .Home fields)
%     axesHandle - target UI axes for the captured image (optional;
%                  pass [] to skip rendering)
%
%   Outputs
%     hex_obj    - updated hex_obj with .pose populated on success
%     status     - struct with fields .ok (logical), .msg (char),
%                  .pose (6x1 or [])
%
%   Intended to be called from the HexControl Camera tab's
%   "Measure Pose" button callback.

status = struct('ok', false, 'msg', '', 'pose', []);

try
    [P_Pose, img, hex_obj] = GrabCheckerboard(hex_obj);
catch ME
    status.msg = sprintf('Checkerboard grab failed: %s', ME.message);
    return
end

if isempty(P_Pose)
    status.msg = 'No checkerboard detected (or file selection cancelled).';
    return
end

hex_obj.pose = P_Pose;

if nargin >= 2 && ~isempty(axesHandle) && isgraphics(axesHandle)
    imshow(img, 'Parent', axesHandle);
end

status.ok = true;
status.pose = P_Pose;
status.msg = sprintf(['Pose measured: ' ...
    'x=%.3f m, y=%.3f m, z=%.3f m, ' ...
    'roll=%.2f deg, pitch=%.2f deg, yaw=%.2f deg'], ...
    P_Pose(1), P_Pose(2), P_Pose(3), ...
    rad2deg(P_Pose(4)), rad2deg(P_Pose(5)), rad2deg(P_Pose(6)));
end
