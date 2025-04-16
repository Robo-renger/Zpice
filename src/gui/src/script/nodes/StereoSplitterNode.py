#!/usr/bin/env python3
import os 
import rospy
import subprocess
from utils.Configurator import Configurator

class StereoSplitter:
    def __init__(self):
        rospy.init_node('stereo_splitter_node', anonymous=False)
        self.configurator = Configurator()
        self.cameraDetails = self.configurator.fetchData(Configurator.CAMERAS)['stereo']
        self.index = self.cameraDetails['index']
        self.width = self.cameraDetails['width']
        self.height = self.cameraDetails['height']

    def run(self):
        rospy.loginfo("Running spitting pipeline...")
        command = f"""
            gst-launch-1.0 -v v4l2src device={self.index} ! \
              video/x-raw,width={int(self.width)},height={int(self.height)} ! videoconvert ! tee name=t \
              t. ! queue ! videocrop right={int(self.width / 2)} ! videoconvert ! v4l2sink device=/dev/video17 \
              t. ! queue ! videocrop left={int(self.width / 2)} ! videoconvert ! v4l2sink device=/dev/video18
            """
        subprocess.run(command, shell=True, executable="/bin/bash")

if __name__ == "__main__":
    try:
        node = StereoSplitter()
        node.run()
    except Exception as e:
        rospy.logerr(f"Error while splitting... {e}")
    
