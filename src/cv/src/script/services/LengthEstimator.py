#!/usr/bin/env python3
import cv2
import numpy as np
import math

class LengthEstimator:
    def __init__(self, ref_points_list: list = [], unknown_points_list: list = [], test_mode: int = 1, image_path: str = "", reference_cm: int = 30):
        
        self.test_mode = test_mode
        self.ref_distances = []
        self.unknown_distances = []
        self.reference_cm = reference_cm
        self.test_mode = test_mode
        
        ### unit test parameters        
        if self.test_mode == 0: 
            self.image = cv2.imread(image_path)
            if self.image is None:
                raise ValueError(f"Image at path '{image_path}' could not be loaded.")
            self.points = []
            self.window_name = 'Measurement Tool'
            
        ### integration test parameters
        elif self.test_mode == 1:
            if len(ref_points_list) != len(unknown_points_list):
                raise ValueError(f"Lists have different number of points")           
            self.ref_points = ref_points_list
            self.unknown_points = unknown_points_list

    def euclidean_distance(self, pt1, pt2):
        return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)

    def draw_point(self, pt):
        cv2.circle(self.image, pt, 5, (0, 255, 0), -1)
        cv2.putText(self.image, f'{pt[0]},{pt[1]}', (pt[0]+5, pt[1]-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    def draw_line_with_distance(self, pt1, pt2, color=(255, 0, 0)):
        cv2.line(self.image, pt1, pt2, color, 2)
        dist = self.euclidean_distance(pt1, pt2)
        mid = ((pt1[0] + pt2[0]) // 2, (pt1[1] + pt2[1]) // 2)
        cv2.putText(self.image, f'{dist:.2f}px', mid,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        return dist

    def calculate_average(self, distances):
        return sum(distances) / len(distances)

    def finalize_measurements(self):
        ref_avg = self.calculate_average(self.ref_distances)
        unknown_avg = self.calculate_average(self.unknown_distances)
        pixels_per_cm = ref_avg / self.reference_cm
        real_world_size = unknown_avg / pixels_per_cm

        print(f'\nAverage reference distance: {ref_avg:.2f}px')
        print(f'Average unknown distance: {unknown_avg:.2f}px')
        print(f"pixels per cm = {pixels_per_cm}")
        print(f'Estimated size of unknown object: {real_world_size:.2f} cm')

        if self.test_mode == 0: 
            cv2.putText(self.image, f'Est: {real_world_size:.2f} cm',
                        (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
        elif self.test_mode == 1:
            return real_world_size

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((x, y))
            self.draw_point((x, y))
            num_points = len(self.points)

            if num_points % 2 == 0:
                pt1 = self.points[-2]
                pt2 = self.points[-1]
                dist = self.draw_line_with_distance(pt1, pt2)

                if num_points <= 8:
                    self.ref_distances.append(dist)
                    
                elif 8 <= num_points <= 16:
                    self.unknown_distances.append(dist)
                # After all 16 points
                if num_points == 16:
                    self.finalize_measurements()

            cv2.imshow(self.window_name, self.image)

    def estimateLength(self):
        # unit test
        if self.test_mode == 0:
            cv2.imshow(self.window_name, self.image)
            cv2.setMouseCallback(self.window_name, self.mouse_callback)
            print("Click 8 points for the reference object (4 lines), then 8 for the unknown object.")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
        # service integration test
        elif self.test_mode == 1:
            for point in self.ref_points:
                self.ref_distances.append(self.euclidean_distance(point[0],point[1]))
            for point in self.unknown_points:
                self.unknown_distances.append(self.euclidean_distance(point[0],point[1]))
            
            return self.finalize_measurements()

            
                

# if __name__ == '__main__':
#     # Suppose your reference object is exactly 10 cm
#     reference_real_world_cm = 30.0
    
#     ## [[(x1,y1),(x2,y2)], [(x1`,y1`),(x2`,y2`)], [(x1``,y1``),(x2``,y2``)], etc...] where each 1-D array
#     ## represent a line for which a distance is calculated. afterwards, all distances are averaged.
#     ref_points_list = [[(100,200),(100,300)],[(100,200),(100,400)], [(100,200),(100,500)], [(100,200),(100,600)]] 
#     unknown_points_list = [[(100,200),(100,300)],[(100,200),(100,300)], [(100,200),(100,300)], [(100,200),(100,300)]]
    
#     test_mode = 1 ### unit test mode = 0, integration test with service = 1
    
#     estimator = LengthEstimator('test.jpg', reference_real_world_cm, ref_points_list, unknown_points_list, test_mode)
#     length = estimator.estimateLength() 
