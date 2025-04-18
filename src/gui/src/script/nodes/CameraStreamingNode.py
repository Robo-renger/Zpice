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
from std_srvs.srv import SetBool, SetBoolRequest

class CameraStreamerNode:
    def __init__(self):
        rospy.init_node('cameras_streamer', anonymous=False)
        self.configurator = Configurator()
        self.camerasDetails = self.__getCameraSteamDetails()
        self.cameraStreamers = []
        self.cameras = []
        self.stereo_cameras = []
        
        # Wait for the stereo split service to be available
        rospy.loginfo("Waiting for split_stereo service...")
        rospy.wait_for_service('split_stereo')
        self.split_stereo_client = rospy.ServiceProxy('split_stereo', SetBool)

    def __getCameraSteamDetails(self):
        return self.configurator.fetchData(Configurator.CAMERAS)
    
    def __getCameraCaptures(self) -> None:
        for camera_data, details in self.camerasDetails.items():
            if details['type'] == 'NORMAL':
                self.cameras.append(Camera(details))
            elif details['type'] == 'FISHEYE':
                self.cameras.append(FishEyeCamera(details))
            elif details['type'] == 'STITCHED':             
                self.cameras.append(StereoStitcher(self.camerasDetails))
            elif details['type'] == 'STEREO':
                pass
            else:
                raise Exception(f"Unsupported camera type. Couldn't find {details['type']}")
            
    def startStreaming(self):
        try:
            # Call the stereo split service
            req = SetBoolRequest(data=True)
            response = self.split_stereo_client(req)
            
            if response.success:
                rospy.loginfo("Stereo split successful. Starting camera streams...")
                self.__getCameraCaptures()
                for camera in self.cameras:
                    cameraStreamer = CameraStreamer(camera)
                    self.cameraStreamers.append(cameraStreamer)
                    cameraStreamer.stream()
            else:
                rospy.logerr(f"Failed to start stereo split: {response.message}")
                
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def stopAllStreams(self):
        try:
            # Stop the stereo split
            req = SetBoolRequest(data=False)
            self.split_stereo_client(req)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to stop stereo split service: {e}")
            
        # Stop all camera streams
        for cameraStreamer in self.cameraStreamers:
            cameraStreamer.releaseCapture()
        rospy.logwarn("All streams terminated")

    def main(self):
        self.startStreaming()
        rospy.spin()

def signal_handler(sig, frame):
    print("\nCtrl+C detected. Stopping streaming...")
    cameraStreamerNode.stopAllStreams()
    sys.exit(0)

if __name__ == '__main__':
    cameraStreamerNode = CameraStreamerNode()
    signal.signal(signal.SIGINT, signal_handler)
    cameraStreamerNode.main()