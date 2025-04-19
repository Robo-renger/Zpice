#!/usr/bin/env python3

import rospy
import signal
import sys
from utils.Configurator import Configurator
from services.CameraStreamer import CameraStreamer
from services.StereoCameraStreamer import StereoCameraStreamer
from services.GUIPresistence import GUIPresistence
from services.Camera import Camera
from services.FishEyeCamera import FishEyeCamera
from services.StereoCamera import StereoCamera
from services.StereoStitcher import StereoStitcher
import time
from services.RightStereoCamera import RightStereoCamera
from services.LeftStereoCamera import LeftStereoCamera
from services.VisualizeDepthMap import VisualizeDepthMap

class CameraStreamerNode:
    def __init__(self):
        rospy.init_node('cameras_streamer', anonymous=False)
        self.configurator = Configurator()
        self.camerasDetails = self.__getCameraSteamDetails()
        self.cameraStreamers = []
        self.stereoCameraStreamers = []
        self.cameras = []
        self.stereo_cameras = []
        

    def __getCameraSteamDetails(self):
        return self.configurator.fetchData(Configurator.CAMERAS)
    
    def __getCameraCaptures(self) -> None:
        for camera_data, details in self.camerasDetails.items():
            if details['type'] == 'NORMAL':
                self.cameras.append(Camera(details))
            elif details['type'] == 'FISHEYE':
                self.cameras.append(FishEyeCamera(details))
            elif details['type'] == 'STITCHED':             
                pass
                # self.cameras.append(StereoStitcher(self.camerasDetails))
            elif details['type'] == 'STEREO':
                # self.stereo_cameras.append(StereoCamera(details))
                # self.cameras.append(LeftStereoCamera(details))
                # self.cameras.append(RightStereoCamera(details))
                # self.cameras.append(VisualizeDepthMap(details))
                pass
            else:
                raise Exception(f"Unsupported camera type. Couldn't find {details['type']}")
            
    def startStreaming(self):
        self.__getCameraCaptures()
        for camera in self.cameras:
            cameraStreamer = CameraStreamer(camera)
            self.cameraStreamers.append(cameraStreamer)
            cameraStreamer.stream()
        # for stereo_camera in self.stereo_cameras:
        #     stereoCameraStreamer = StereoCameraStreamer(stereo_camera)
        #     self.stereoCameraStreamers.append(stereoCameraStreamer)
        #     stereoCameraStreamer.stream()

    def stopAllStreams(self):
        for cameraStreamer in self.cameraStreamers:
            cameraStreamer.releaseCapture()
        # for stereoCameraStreamer in self.stereoCameraStreamers:
        #     stereoCameraStreamer.closeStream()
        rospy.logwarn("All streams terminated")

    def main(self):
        self.startStreaming()
        time.sleep(5)
        rospy.spin()

def signal_handler(sig, frame):
    print("\nCtrl+C detected. Stopping streaming...")
    cameraStreamerNode.stopAllStreams()
    sys.exit(0)

if __name__ == '__main__':
    cameraStreamerNode = CameraStreamerNode()
    signal.signal(signal.SIGINT, signal_handler)
    cameraStreamerNode.main()