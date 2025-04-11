#!/usr/bin/env python3

import rospy
import signal
import sys
from utils.Configurator import Configurator
from services.CameraStreamer import CameraStreamer
from services.GUIPresistence import GUIPresistence
from services.Camera import Camera
from services.FishEyeCamera import FishEyeCamera
from services.StereoCamera import StereoCamera
import time
class CameraStreamerNode:
    def __init__(self):
        rospy.init_node('cameras_streamer', anonymous=False)
        self.configurator = Configurator()
        self.camerasDetails = self.__getCameraSteamDetails()
        self.cameraStreamers = []
        self.cameras = []

    def __getCameraSteamDetails(self):
        return self.configurator.fetchData(Configurator.CAMERAS)
    
    def __getCameraCaptures(self) -> None:
        for camera_data, details in self.camerasDetails.items():
            if details['type'] == 'NORMAL':
                self.cameras.append(Camera(details))
            elif details['type'] == 'FISHEYE':
                self.cameras.append(FishEyeCamera(details))
            elif details['type'] == 'STEREO':
                camera = StereoCamera(details)              
                self.cameras.append(StereoCamera(details))
            else:
                raise Exception(f"Unsupported camera type. Couldn't find {details['type']}")
            
    def runStreams(self):
        self.__getCameraCaptures()
        for camera in self.cameras:
            cameraStreamer = CameraStreamer(camera)
            self.cameraStreamers.append(cameraStreamer)
            cameraStreamer.stream()

    def stopAllStreams(self):
        for cameraStreamer in self.cameraStreamers:
            cameraStreamer.releaseCapture()
        rospy.logwarn("All streams terminated")

    def main(self):
        self.runStreams()
        time.sleep(5)
        # self.cameraStreamers[0].releaseCapture()
        # rospy.logwarn("Camera with index 0 has been terminated")
        # time.sleep(2)
        # for camera, details in self.camerasDetails.items():
        #     cameraStreamer = CameraStreamer(details['index'],details['port'])
        #     self.cameraStreamers.append(cameraStreamer)
        #     cameraStreamer.setFPS(details['fps'])
        #     cameraStreamer.setFrameSize(details['width'], details['height'])
        #     cameraStreamer.stream()
        #     break
        # rospy.logwarn("Camera with index 0 is now ready!")
        rospy.spin()

def signal_handler(sig, frame):
    print("\nCtrl+C detected. Stopping streaming...")
    cameraStreamerNode.stopAllStreams()
    sys.exit(0)

if __name__ == '__main__':
    cameraStreamerNode = CameraStreamerNode()
    signal.signal(signal.SIGINT, signal_handler)
    cameraStreamerNode.main()