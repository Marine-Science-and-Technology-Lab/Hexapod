function [P_Pose,img,hex_obj]=GrabCheckerboard(hex_obj)
% GrabCheckerboard - Prompt for a PNG, detect checkerboard corners, and
% solve for platform pose using the loaded Basler camera parameters.
%
% Returns P_Pose = [] if the user cancels the file picker or the
% checkerboard corners cannot be detected in the selected image.

[fname, fpath] = uigetfile('*.png');
if isequal(fname, 0)
    P_Pose = [];
    img = [];
    return
end

load('Basler_Params.mat');

img = imread(fullfile(fpath, fname));

refrectY = [-62.5 62.5 62.5 -62.5 -62.5]';
refrectX = [-100 -100 100 100 -100]';
refrectZ = [0 0 0 0 0]';

try
    [imagePoints, boardSize] = detectCheckerboardPoints(img);
    if isempty(imagePoints) || any(boardSize < 2)
        P_Pose = [];
        return
    end
    squareSize = 25;  % in units of 'millimeters'
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    [rotationMatrix, translationVector] = extrinsics(imagePoints, worldPoints, cameraParams);

    [XYZ] = rotationMatrix*[refrectX, refrectY, refrectZ]';
    X = XYZ(1,:) + translationVector(1);
    Y = XYZ(2,:) - translationVector(2);
    Z = XYZ(3,:) - translationVector(3); %#ok<NASGU>

    rpy = rad2deg(rotm2eul(rotationMatrix));
catch
    P_Pose = [];
    return
end

Trans_camplatform = -translationVector'/1000;
% Express the camera-derived world position as a display-frame pose
% (datum-relative). Uses only the translation component of T_world_datum;
% rotated datums would need the full frame chain here - left as a
% follow-up since the camera path isn't exercised yet.
T_world_datum = composeTransform(hex_obj.T_world_datum_platform, hex_obj.T_platform_POI);
P_Pose = [Trans_camplatform + hex_obj.Cam - T_world_datum.t; fliplr(deg2rad(rpy))'];
