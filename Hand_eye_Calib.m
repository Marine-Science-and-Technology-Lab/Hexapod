function [t_estimate,r_estimate] = Hand_eye_Calib(image,path_to_frames)
    path_to_calib = 'Cam-Track/Cam_Data/'; % Change this path as needed
    path_to_save = 'C:/Users/juruiz/Downloads/CharAruco detect/'; % asked by user

    % Make sure your Python script is in MATLAB's current folder or add its folder to the Python search path
    if count(py.sys.path, '') == 0
        insert(py.sys.path, int32(0), '');
    end
    %% Result from OpenCV detect
    if image==1
        result = py.Image_Pose_Charuco.detect_pose(path_to_calib,path_to_frames)
    else
        %result = py.Video_Pose_Charuco.detect_video(path_to_calib, path_to_frames, path_to_save)
    end


    % A = cell(result)
    % A = cellfun(@double,A,'UniformOutput',false)
    % A =A';
    % Charuco_pose = cell2mat(A);

    try
        res = double(result);
        Charuco_pose = res';
    catch
        res = cell(result)
        A = cellfun(@double,res,'UniformOutput',false)
        A =A';
        Charuco_pose = cell2mat(A);
    end

    
    %% Setting up Hand-eye calib

    %Defining some variables 
    t_estimate = zeros(3,size(Charuco_pose,2))
    r_estimate = zeros(3,size(Charuco_pose,2))

    handEye = load('Cam-Track/Cam_Data/HandEyeCalib.mat');
    R_cam2Base = handEye.R_cam2Base;
    t_cam2Base = (handEye.t_cam2Base);
    H_cam2Base =[R_cam2Base t_cam2Base;0 0 0 1];

    %Char2Plat
    H_Charuco2Plat = load("Cam-Track/Cam_Data/Charuco2Plat.mat");
    H_Charuco2Plat = H_Charuco2Plat.H_Charuco2Plat;

    for i = 1:size(Charuco_pose,2)

        % Transformation matrix char/cam at i: Known 
        R_cam2Charuco_i = eul2rotm(Charuco_pose(6:-1:4,i)') %'ZYX'
        t_cam2Charuco_i = Charuco_pose(1:3,i)
        H_cam2Charuco_i = [R_cam2Charuco_i t_cam2Charuco_i;0 0 0 1]
        % Transformation matrix cam/char
        H_Charuco2Cam_i = [R_cam2Charuco_i' -R_cam2Charuco_i'*t_cam2Charuco_i;0 0 0 1];
        
    
        % Estimating Base2Plat p/b
        % p/b = p/char x char/cam x cam/base
        H_base2cam = [R_cam2Base' -R_cam2Base'*t_cam2Base;0 0 0 1]
        H_Estimate_i = H_Charuco2Plat*H_cam2Charuco_i*H_base2cam
        t_estimate(:,i) = H_Estimate_i(1:3,4)
        r_estimate(:,i) = rotm2eul(H_Estimate_i(1:3,1:3),"XYZ")
      
    end
    

end

