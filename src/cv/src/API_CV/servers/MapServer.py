#!/usr/bin/env python3
import rospy
from cv.srv import setMap
from API_CV.services.MapService import MapService

class MapServer:
    def __init__(self):
        rospy.init_node('map_server')
        self.service = MapService()
    
    def mapServiceServer(self):
        s2 = rospy.Service('setMapService', setMap, self.service.handleSetMap)
        rospy.loginfo("Set map service server Ready")
        rospy.spin()

if __name__ == "__main__":
    MapServer().mapServiceServer()