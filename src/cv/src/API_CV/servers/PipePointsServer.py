#!/usr/bin/env python3
import rospy
from cv.srv import setPipePoints
from API_CV.services.PipePointsService import PipePointsService

class PipePointsServer:
    def __init__(self):
        rospy.init_node('pipe_server')
        self.service = PipePointsService()

    def PipePointsServiceServer(self):
        s1 = rospy.Service('setPipePoints', setPipePoints, self.service.handleSetPipePoints)
        rospy.loginfo("Set pipe points server Ready")
        rospy.spin()

if __name__ == "__main__":
    PipePointsServer().PipePointsServiceServer()
