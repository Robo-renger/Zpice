#!/usr/bin/env python3
import rospy
from control.srv import GetStream
from API.services.StreamService import StreamService

class StreamServer:
    def __init__(self):
        rospy.init_node('get_stream_service')
        self.service = StreamService()

    def streamServer(self):
        rospy.Service('getStreamService', GetStream, self.service.handleGetStream)
        rospy.loginfo("Stream Service Server Ready")
        rospy.spin()
if __name__ == "__main__":
    StreamServer().streamServer()