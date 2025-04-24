#!/usr/bin/env python3
import rospy
from cv.srv import setPipePointsResponse
from services.LengthEstimator import LengthEstimator
import ast

class PipePointsService:
    def __init__(self):
        self.length_estimator = LengthEstimator()

    def handleSetPipePoints(self, req):
        rospy.loginfo(f"Recieved Request to estimate the length of the pipe: {ast.literal_eval(req.reference_points_left)}, {ast.literal_eval(req.target_points_left)}, {ast.literal_eval(req.reference_points_right)}, {ast.literal_eval(req.target_points_right)}, true length = {req.ref_true_length}")
        pipe_length = self.length_estimator.estimateLength(
            ast.literal_eval(req.reference_points_left),
            ast.literal_eval(req.reference_points_right),    # Fixed order
            ast.literal_eval(req.target_points_left),        # Fixed order
            ast.literal_eval(req.target_points_right),
            float(req.ref_true_length))
        return setPipePointsResponse(pipe_length)