#!/usr/bin/env python3
import os 
import rospy
import subprocess
from multiprocessing import Process
from utils.Configurator import Configurator
from gui.msg import splitState

class StereoSplitter:
    def __init__(self):
        rospy.init_node('stereo_splitter_node', anonymous=False)
        self.splitStatePub = rospy.Publisher('split_state', splitState, queue_size=10)
        self.configurator = Configurator()
        self.cameraDetails = self.configurator.fetchData(Configurator.CAMERAS)['stereo']
        self.index = self.cameraDetails['index']
        self.width = self.cameraDetails['width']
        self.height = self.cameraDetails['height']
        self.msg = splitState()
        self.msg.splitted = False
        self.split_process = None
        self.splitStatePub.publish(self.msg)

    def start_split_pipeline(self):
        """Starts the GStreamer pipeline in a separate process"""
        command = f"""
            gst-launch-1.0 -v v4l2src device={self.index} ! \
              video/x-raw,width={int(self.width)},height={int(self.height)} ! videoconvert ! tee name=t \
              t. ! queue ! videocrop right={int(self.width / 2)} ! videoconvert ! v4l2sink device=/dev/video17 \
              t. ! queue ! videocrop left={int(self.width / 2)} ! videoconvert ! v4l2sink device=/dev/video18
            """
        subprocess.run(command, shell=True, executable="/bin/bash")

    def run(self):
        """Main run loop that handles publishing"""
        rospy.loginfo("Starting stereo splitter node...")
        # Start the pipeline in a separate process
        self.split_process = Process(target=self.start_split_pipeline)
        self.split_process.daemon = True
        self.split_process.start()
        
        # Set split state to true
        self.msg.splitted = True
        
        # Keep publishing the state
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.splitStatePub.publish(self.msg)
            rate.sleep()

    def __del__(self):
        """Cleanup when the node is destroyed"""
        if self.split_process:
            self.split_process.terminate()
            self.split_process.join()

if __name__ == "__main__":
    try:
        node = StereoSplitter()
        node.run()
    except Exception as e:
        rospy.logerr(f"Error while splitting... {e}")
    
