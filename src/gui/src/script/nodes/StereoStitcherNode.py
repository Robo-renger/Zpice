#!/usr/bin/env python3
import rospy
from utils.Configurator import Configurator
from services.StereoStitcher import StereoStitcher

class StereoStitcher:
    def __init__(self):
        rospy.init_node('stereo_stitcher_node', anonymous=False)
        self.configurator = Configurator()
        self.cameraDetails = self.__getCameraSteamDetails()
        self.stitcher = StereoStitcher(self.cameraDetails)

    def __getCameraSteamDetails(self):
        return self.configurator.fetchData(Configurator.CAMERAS)

    def run(self):
        self.stitcher.stitch()

if __name__ == "__main__":
    try:
        node = StereoStitcher()
        node.run()
    except Exception as e:
        rospy.logerr(f"Error while stitching... {e}")
    
