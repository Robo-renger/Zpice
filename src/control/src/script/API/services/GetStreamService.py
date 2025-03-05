#!/usr/bin/env python3

import rospy
import json
from control.srv import GetStream, GetConfigResponse
from utils.Configurator import Configurator
from utils.LayoutManager import LayoutManager

class GetStreamService:
    def __init__(self):
        rospy.init_node('get_stream_service')
        self.camera_config = Configurator().fetchData("cameras")
        self.camera_layout = LayoutManager().fetchLayout("cameras")

    def handleGetStream(self, request):
        camera_streams = {**self.camera_layout, **self.camera_config}
        return GetConfigResponse(camera_streams)

    def run(self):
        rospy.Service('getStreamService', GetStream, self.handleGetStream)
        rospy.spin()

if __name__ == "__main__":
    try:
        GetStreamService().run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")
    except rospy.ROSInterruptException as e:
        rospy.logwarn(f"Get stream failed: {e}")
