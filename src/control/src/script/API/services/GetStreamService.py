#!/usr/bin/env python3

import rospy
from control.srv import GetStream, GetStreamResponse
from utils.Configurator import Configurator
from utils.LayoutManager import LayoutManager
from utils.JsonFileHandler import JSONFileHandler

class GetStreamService:
    def __init__(self):
        rospy.init_node('get_stream_service')
        self.camera_config = Configurator().fetchData("cameras")
        self.camera_layout = LayoutManager().fetchLayout("cameras")
        self.stream = JSONFileHandler().fetchData("stream")

    def handleGetStream(self, request):
        JSONFileHandler().setData("stream", self.camera_layout)
        JSONFileHandler().setData("stream", self.__handleConfig())
        return GetStreamResponse(str(self.stream))
    
    def __handleConfig(self) -> dict:
        updated_data = {
            view: {
                camera: {**camera_data, **self.camera_config.get(camera, {})}
                for camera, camera_data in cameras.items()
            }
            for view, cameras in self.stream.items()
        }
        return updated_data

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
