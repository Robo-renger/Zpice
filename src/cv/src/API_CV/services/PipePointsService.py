#!/usr/bin/env python3
import rospy
from cv.srv import setPipePointsResponse
from services.LengthEstimator import LengthEstimator
import ast

class PipePointsService:
    def __init__(self):
        self.length_estimator = LengthEstimator()

    def handleSetPipePoints(self, req):
        rospy.loginfo("Recieved Request to estimate the length of the pipe")
        pipe_length = self.length_estimator.estimateLength(
            ast.literal_eval(req.right_cam_right_point),
            ast.literal_eval(req.right_cam_left_point),
            ast.literal_eval(req.left_cam_right_point),
            ast.literal_eval(req.left_cam_left_point)
        )
        return setPipePointsResponse(pipe_length)