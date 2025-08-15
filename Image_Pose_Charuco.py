import os
import numpy as np
import cv2


ARUCO_DICT = cv2.aruco.DICT_6X6_250
SQUARES_VERTICALLY = 7
SQUARES_HORIZONTALLY = 9
SQUARE_LENGTH = 0.03
MARKER_LENGTH = 0.022

def detect_pose(path_to_calibration, path_to_video):
    # Import camera intrisic parameters
    camera_matrix = np.load(os.path.join(path_to_calibration, 'camera_matrix.npy'))
    dist_coeffs = np.load(os.path.join(path_to_calibration, 'dist_coeffs.npy'))

    # List to hold pose data
    x_position = []
    y_position = []
    z_position = []
    x_rotation = []
    y_rotation = []
    z_rotation = []

    frame = cv2.imread(path_to_video)

    undistorted_image = cv2.undistort(frame, camera_matrix, dist_coeffs)

    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH, MARKER_LENGTH,
                                       dictionary)
    params = cv2.aruco.DetectorParameters()

    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(undistorted_image, dictionary, parameters=params)


    # If at least one marker is detected
    if len(marker_corners) > 0:
        # Interpolate CharUco corners
        count = 1
        charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(marker_corners,
                                                                                               marker_ids,
                                                                                               undistorted_image, board)

        # If enough corners are found, estimate the pose
        if charuco_retval:
            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(charuco_corners, charuco_ids, board,
                                                                        camera_matrix,
                                                                        dist_coeffs, None, None)


            x_position.append(tvec[0][0])
            y_position.append(tvec[1][0])
            z_position.append(tvec[2][0])

            x_rotation.append(rvec[0][0])
            y_rotation.append(rvec[1][0])
            z_rotation.append(rvec[2][0])

            #if retval:
                #cv2.drawFrameAxes(undistorted_image, camera_matrix, dist_coeffs, rvec, tvec, length=0.1,
                                      #thickness=15)


     #print("Frame # pose not detected count: ", missCheck)
    pose_path = np.array([x_position, y_position, z_position, x_rotation, y_rotation, z_rotation]).T
    return pose_path