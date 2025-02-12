#!/usr/bin/env python3

import subprocess
from utils.EnvParams import EnvParams
import rospy

class GUIStreamingNode:
    def __init__(self):
        rospy.init_node('gui_streamer', anonymous=False)

    
    def stream(self):
        command = f'mjpg_streamer -o "output_http.so -p 8080 -w {EnvParams().WEB_INDEX_LOCATION}"'
        subprocess.run(command, shell=True)
        rospy.spin()
    
if __name__ == "__main__":
    guiStreamer = GUIStreamingNode()
    guiStreamer.stream()