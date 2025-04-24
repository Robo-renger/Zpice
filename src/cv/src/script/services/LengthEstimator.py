#!/usr/bin/env python3
import cv2
import numpy as np
import math

class LengthEstimator:
    def __init__(self):
        
        self.ref_distances = []
        self.ref_points_left = []
        self.ref_points_right = []
        self.unknown_points_left = []
        self.unknown_points_right = []
        self.unknown_distances = []
        # self.reference_cm = 0

    def euclidean_distance(self, pt1, pt2):
        return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)

    def calculate_average(self, distances):
        return sum(distances) / len(distances)

    def finalize_measurements(self, reference_cm):
        print(f"reference_cm: {reference_cm}")
        if len(self.ref_distances) == 0 or len(self.unknown_distances) == 0:
            raise ValueError("No distances to calculate")
        if reference_cm == 0:
            raise ValueError("Reference centimeter value cannot be zero")
        
        ref_avg = self.calculate_average(self.ref_distances)
        if ref_avg == 0:
            raise ValueError("Reference average distance cannot be zero")
        
        unknown_avg = self.calculate_average(self.unknown_distances)
        pixels_per_cm = ref_avg / reference_cm
        real_world_size = unknown_avg / pixels_per_cm

        print(f'\nAverage reference distance: {ref_avg:.2f}px')
        print(f'Average unknown distance: {unknown_avg:.2f}px')
        print(f"pixels per cm = {pixels_per_cm}")
        print(f'Estimated size of unknown object: {real_world_size:.2f} cm')

        return real_world_size

    def estimateLength(self, ref_points_left, ref_points_right, unknown_points_left, unknown_points_right, reference_cm: int = 30):
            if len(ref_points_left) != len(unknown_points_left) and len(ref_points_right) != len(unknown_points_right):
                raise ValueError(f"Lists have different number of points")  
            for left_point, right_point in zip(ref_points_left, ref_points_right):
                self.ref_distances.append(self.euclidean_distance(left_point, right_point))
            for left_point, right_point in zip(unknown_points_left, unknown_points_right):
                self.unknown_distances.append(self.euclidean_distance(left_point, right_point))
            
            return self.finalize_measurements(reference_cm)
