#!/usr/bin/env python3
import os 
import rospy
import subprocess
from multiprocessing import Process
from utils.Configurator import Configurator
from std_srvs.srv import SetBool, SetBoolResponse

class StereoSplitter:
    def __init__(self):
        rospy.init_node('stereo_splitter_node', anonymous=False)
        # self.split_service = rospy.Service('split_stereo', SetBool, self.handle_split_request)
        self.configurator = Configurator()
        self.cameraDetails = self.configurator.fetchData(Configurator.CAMERAS)['stereo']
        self.index = self.cameraDetails['index']
        self.width = self.cameraDetails['width']
        self.height = self.cameraDetails['height']
        self.split_process = None

    def startSplitPipeline(self):
        """Starts the GStreamer pipeline in a separate process"""
        command = f"""
            gst-launch-1.0 -v v4l2src device={self.index} ! \
              video/x-raw,width={int(self.width)},height={int(self.height)} ! videoconvert ! tee name=t \
              t. ! queue ! videocrop right={int(self.width / 2)} ! videoconvert ! v4l2sink device=/dev/video17 \
              t. ! queue ! videocrop left={int(self.width / 2)} ! videoconvert ! v4l2sink device=/dev/video18
            """
        subprocess.run(command, shell=True, executable="/bin/bash")

    # def handle_split_request(self, req):
    #     """Handle incoming service requests"""
    #     response = SetBoolResponse()
        
    #     if req.data:  # If request is to start splitting
    #         try:
    #             if not self.split_process or not self.split_process.is_alive():
    #                 self.split_process = Process(target=self.startSplitPipeline)
    #                 self.split_process.daemon = True
    #                 self.split_process.start()
    #             response.success = True
    #             response.message = "Stereo split started successfully"
    #         except Exception as e:
    #             response.success = False
    #             response.message = f"Failed to start stereo split: {str(e)}"
    #     else:  # If request is to stop splitting
    #         if self.split_process:
    #             self.split_process.terminate()
    #             self.split_process.join()
    #         response.success = True
    #         response.message = "Stereo split stopped"
            
    #     return response

    # def run(self):
    #     """Main run loop that handles service requests"""
    #     rospy.loginfo("Starting stereo splitter node...")
    #     rospy.spin()

    # def __del__(self):
    #     """Cleanup when the node is destroyed"""
    #     if self.split_process:
    #         self.split_process.terminate()
    #         self.split_process.join()

if __name__ == "__main__":
    try:
        node = StereoSplitter()
        node.startSplitPipeline()
    except Exception as e:
        rospy.logerr(f"Error in stereo splitter node: {e}")
    
