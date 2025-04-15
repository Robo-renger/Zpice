#!/usr/bin/env python3
import rospy
from cv.srv import setPipePoints
import ast

def test_length_estimation():
    rospy.init_node('test_length_estimation')
    rospy.wait_for_service('set_pipe_points')
    
    try:
        # Sample points for testing
        # Format: (x, y) coordinates
        right_cam_right_point = "(100, 200)"  # Right point in right camera
        right_cam_left_point = "(300, 200)"   # Left point in right camera
        left_cam_right_point = "(120, 200)"   # Right point in left camera
        left_cam_left_point = "(320, 200)"    # Left point in left camera
        
        # Create service proxy
        set_pipe_points = rospy.ServiceProxy('set_pipe_points', setPipePoints)
        
        # Call the service
        response = set_pipe_points(
            right_cam_right_point,
            right_cam_left_point,
            left_cam_right_point,
            left_cam_left_point
        )
        
        print(f"Estimated length: {response.pipe_length} cm")
        
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    test_length_estimation() 