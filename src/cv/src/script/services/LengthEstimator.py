#!/usr/bin/env python3
import numpy as np
import cv2 as cv
import rospkg
from utils.Configurator import Configurator

class LengthEstimator:
    def __init__(self):
        rospack = rospkg.RosPack()
        workspace_path = rospack.get_path('gui')
        self.left_dist = workspace_path + f'/../../calibrationMatricies/calibration_data2/distL.npy' # Distortion matrix for left
        self.right_dist = workspace_path + f'/../../calibrationMatricies/calibration_data2/distR.npy' # Distortion matrix for right
        self.left_matrix = workspace_path + f'/../../calibrationMatricies/calibration_data2/mtxL.npy' # Intrinsic matrix for left
        self.right_matrix = workspace_path + f'/../../calibrationMatricies/calibration_data2/mtxR.npy' # Intrinsic matrix for right
        self.left_P = workspace_path + f'/../../calibrationMatricies/calibration_data2/P1.npy' # Projection matrix for left
        self.right_P = workspace_path + f'/../../calibrationMatricies/calibration_data2/P2.npy' # Projection matrix for right 
        self.left_R = workspace_path + f'/../../calibrationMatricies/calibration_data2/R1.npy' # Rectification transforms for left 
        self.right_R = workspace_path + f'/../../calibrationMatricies/calibration_data2/R2.npy' # Rectification transforms for right
        self.Q_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/Q.npy' # Disparity-to-depth map matrix
        self.R_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/R.npy' # Rotation matrix from left to right
        self.T_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/T.npy' # Translation matrix from left to right
        self.cameras_configs = Configurator().fetchData(Configurator.CAMERAS)
        self.stereo_camera_configs = self.cameras_configs['stereo']
        self.baseline = self.stereo_camera_configs['baseline']
        self.focal_length = self.stereo_camera_configs['focal_length']
        self.__loadMatricies()

    def __loadMatricies(self):
        self.left_dist = np.load(self.left_dist)
        self.right_dist = np.load(self.right_dist)
        self.left_matrix = np.load(self.left_matrix)
        self.right_matrix = np.load(self.right_matrix)
        self.left_P = np.load(self.left_P)
        self.right_P = np.load(self.right_P)
        self.left_R = np.load(self.left_R)
        self.right_R = np.load(self.right_R)
        self.Q = np.load(self.Q_path)
        self.R = np.load(self.R_path)
        self.T = np.load(self.T_path)

    def __undistortPoint(self, point, camera_matrix, dist_coeffs):
        """
        Undistort a point using the camera calibration parameters
        """
        point = np.array([point], dtype=np.float32)
        undistorted = cv.undistortPoints(point, camera_matrix, dist_coeffs, P=camera_matrix)
        return undistorted[0][0]

    def __reconstruct3D(self, point_left, point_right):
        """
        Reconstruct the 3D coordinate from matching points in the left and right images.
        Applies undistortion and rectification before calculating disparity.
        :param point_left: (x, y) from the left image.
        :param point_right: (x, y) from the right image.
        :return: 3D point as a NumPy array [X, Y, Z] or None if disparity is zero.
        """
        # Undistort points
        left_undistorted = self.__undistortPoint(point_left, self.left_matrix, self.left_dist)
        right_undistorted = self.__undistortPoint(point_right, self.right_matrix, self.right_dist)

        # Calculate disparity
        disparity = abs(left_undistorted[0] - right_undistorted[0])
        if disparity == 0:
            return None

        # Create homogeneous coordinates
        point_4d = np.array([left_undistorted[0], left_undistorted[1], disparity, 1.0])
        
        # Use Q matrix to convert to 3D coordinates
        point_3d = np.dot(self.Q, point_4d)
        point_3d = point_3d[:3] / point_3d[3]  # Convert from homogeneous coordinates
        
        return point_3d

    def estimateLength(self, right_img_right_point: tuple, right_img_left_point: tuple, left_img_right_point: tuple, left_img_left_point: tuple):
        """
        Perform interactive endpoint selection and estimate the pipe length.
        """
        first_3d_point = self.__reconstruct3D(left_img_right_point, right_img_right_point)
        second_3d_point = self.__reconstruct3D(left_img_left_point, right_img_left_point)
        if first_3d_point is None or second_3d_point is None:
            print("Error: Disparity is zero for one or both endpoints; cannot compute depth.")
            return None

        length = np.linalg.norm(second_3d_point - first_3d_point)
        print("Estimated pipe length: {:.2f} cm".format(length))
        return length
        