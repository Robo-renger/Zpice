#!/usr/bin/env python3

import rospy
import signal
import sys
from utils.Configurator import Configurator
from services.CameraStreamer import CameraStreamer
from services.GUIPresistence import GUIPresistence
import subprocess
from utils.EnvParams import EnvParams

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
            # html_content = GUIPresistence(EnvParams().WEB_INDEX_LOCATION+"/index.html").getGUI()
            html_content = GUIPresistence("/home/ziad/zpice_ws/src/gui/src/index.html").getGUI()
            cameraStreamer = CameraStreamer(details['index'],html_content)
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
        # command = f'mjpg-streamer -o "output_http.so -p 8080 -w /home/mypi/Zpice/src/gui/src"'
        command = f'mjpg-streamer -o "output_http.so -p 8080 -w {EnvParams().WEB_INDEX_LOCATION}"'
        subprocess.run(command, shell=True)
        rospy.spin()

def signal_handler(sig, frame):
    print("\nCtrl+C detected. Stopping streaming...")
    cameraStreamerNode.stopAllStreams()
    sys.exit(0)

if __name__ == '__main__':
    cameraStreamerNode = CameraStreamerNode()
    signal.signal(signal.SIGINT, signal_handler)
    cameraStreamerNode.main()