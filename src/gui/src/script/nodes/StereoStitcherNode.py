#!/usr/bin/env python3
import rospy
from utils.Configurator import Configurator
from services.StereoStitcher import StereoStitcher
from gui.msg import splitState

class StereoStitcherNode:
    def __init__(self):
        rospy.init_node('stereo_stitcher_node', anonymous=False)
        self.configurator = Configurator()
        self.cameraDetails = self.__getCameraSteamDetails()
        self.stitcher = StereoStitcher(self.cameraDetails)
        self.split_state = False
        
    def splitStatecallback(self, msg):
        self.split_state = msg.splitted
        if self.split_state:
            self.stitcher.stitch()


    def __getCameraSteamDetails(self):
        return self.configurator.fetchData(Configurator.CAMERAS)

    def run(self):
        while not rospy.is_shutdown():
            rospy.Subscriber('split_state', splitState, self.splitstatecallback)
            rospy.spin()

if __name__ == "__main__":
    try:
        node = StereoStitcherNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"Error while stitching... {e}")
    
