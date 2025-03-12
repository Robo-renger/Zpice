#!/usr/bin/env python3
import numpy as np
from utils.Configurator import Configurator

class LengthEstimator:
    def __init__(self):
        self.stereo_camera_configs = Configurator().fetchData(Configurator.STEREO_CAMERA_PARAMS)
        self.baseline = self.stereo_camera_configs['BASELINE']
        self.focal_length = self.stereo_camera_configs['FOCALLENGTH']
        self.c_x = self.stereo_camera_configs['WIDTH']
        self.c_y = self.stereo_camera_configs['HEIGHT']

    def __reconstruct3D(self, point_left, point_right):
        """
        Reconstruct the 3D coordinate from matching points in the left and right images.
        Assumes that the images are rectified.
        :param point_left: (x, y) from the left image.
        :param point_right: (x, y) from the right image.
        :return: 3D point as a NumPy array [X, Y, Z] or None if disparity is zero.
        """
        disparity = abs(point_left[0] - point_right[0])
        if disparity == 0:
            return None  
        Z = (self.focal_length * self.baseline) / disparity  # Depth in cm.
        X = ((point_left[0] - self.c_x) * Z) / self.focal_length
        Y = ((point_left[1] - self.c_y) * Z) / self.focal_length
        return np.array([X, Y, Z])


    def estimateLength(self, right_img_right_point: tuple, right_img_left_point: tuple, left_img_right_point: tuple, left_img_left_point: tuple):
        """
        Perform interactive endpoint selection and estimate the pipe length.
        """
        first_3d_point = self.__reconstruct3D(left_img_right_point, right_img_right_point)
        second_3d_point = self.__reconstruct3D(left_img_left_point, right_img_left_point)
        if first_3d_point is None or second_3d_point is None:
            print("Error: Disparity is zero for one or both endpoints; cannot compute depth.")
            return

        length = np.linalg.norm(second_3d_point - first_3d_point)
        print("Estimated pipe length: {:.2f} cm".format(length))
        return length
        