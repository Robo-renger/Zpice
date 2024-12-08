#!/usr/bin/env python3

import rospy
import signal
import sys
from utils.Configurator import Configurator
from services.CameraStreamer import CameraStreamer
from services.GUIPresistence import GUIPresistence

class CameraStreamerNode:
    def __init__(self):
        rospy.init_node('cameras_streamer', anonymous=False)
        self.configurator = Configurator()
        self.camerasDetails = self.__getCameraSteamDetails()
        self.cameraStreamers = []

    def __getCameraSteamDetails(self):
        return self.configurator.fetchData(Configurator.CAMERAS)

    def runStreams(self):
        for camera, details in self.camerasDetails.items():
            html_content = GUIPresistence("/home/ziad/zpice_ws/src/gui/src/index.html").getGUI()
            cameraStreamer = CameraStreamer("test.mp4",html_content)
            self.cameraStreamers.append(cameraStreamer)
            cameraStreamer.setFPS(details['fps'])
            cameraStreamer.setFrameSize(details['width'], details['height'])
            cameraStreamer.stream(details['port'])

    def stopAllStreams(self):
        for cameraStreamer in self.cameraStreamers:
            cameraStreamer.closeStream()
        rospy.logwarn("All streams terminated")

    def main(self):
        self.runStreams()
        rospy.spin()

def signal_handler(sig, frame):
    print("\nCtrl+C detected. Stopping streaming...")
    cameraStreamerNode.stopAllStreams()
    sys.exit(0)

if __name__ == '__main__':
    cameraStreamerNode = CameraStreamerNode()
    signal.signal(signal.SIGINT, signal_handler)
    cameraStreamerNode.main()
