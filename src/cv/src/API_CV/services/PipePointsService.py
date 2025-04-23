#!/usr/bin/env python3
import rospy
from cv.srv import setPipePointsResponse
from services.LengthEstimator import LengthEstimator
import ast

class PipePointsService:
    def __init__(self):
        pass

    def handleSetPipePoints(self, req):
        rospy.loginfo(f"Recieved Request to estimate the length of the pipe: {req.reference_points}, {req.target_points}")
        rospy.loginfo(f"ref_points: {ast.literal_eval(req.reference_points)}\n target_points: {ast.literal_eval(req.target_points)}")
        self.length_estimator = LengthEstimator(
            ast.literal_eval(req.reference_points),
            ast.literal_eval(req.target_points), test_mode=1, reference_cm=req.ref_true_length)
        pipe_length = self.length_estimator.estimateLength()
        return setPipePointsResponse(pipe_length)