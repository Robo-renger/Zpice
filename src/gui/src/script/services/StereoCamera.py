#!/usr/bin/env python3
from zope.interface import implementer
from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity
from interface.ICamera import ICamera
from utils.EnvParams import EnvParams
import cv2 as cv
import numpy as np 

@implementer(ICamera)
class StereoCamera:
    """Responsible for handling camera attributes and initializing cameras"""
    def __init__(self, cameraDetails: dict) -> None:
        """Initialize the camera
        @param cameraDetails: all details of the camera found in the config"""
        self.address = EnvParams().WEB_DOMAIN
        self.camera_details = cameraDetails
        self.cameraIndex = cameraDetails['index']
        self.width = cameraDetails['width']
        self.height = cameraDetails['height']
        self.FPS = cameraDetails['fps']
        self.compression = cameraDetails['format']
        self.capture = None
        self.standalone_camera_details = {}

    def setupCamera(self) -> cv.VideoCapture:
        pass

    def setupStereoCamera(self, camera_details: dict) -> cv.VideoCapture:
        """Sets up the camera capture based on the selected format.""" 
        self.standalone_camera_details = camera_details   
        if self.compression == "H264":
            return self._setupH264()
        elif self.compression == "MJPG":
            return self._setupMJPG()
        else:
            raise ValueError("Unsupported format. Choose either 'MJPG' or 'H264'.")
        
    def _setupMJPG(self):
        """Sets up MJPEG streaming using OpenCV."""
        print("Initializing camera with MJPEG format...")
        self.capture = cv.VideoCapture(self.standalone_camera_details['index'])
        fourcc = cv.VideoWriter_fourcc(*'MJPG')
        self.capture.set(cv.CAP_PROP_FOURCC, fourcc)
        self.__setCVAttrs()
        return self.capture

    def _setupH264(self):
        print("eshta")
        """Sets up H.264 streaming using GStreamer."""
        print("Initializing camera with H.264 format...")
        gst_pipeline = (
            f"v4l2src device={self.standalone_camera_details['index']} ! "
            f"image/jpeg, width={self.standalone_camera_details['width']}, height={self.standalone_camera_details['height']}, framerate={self.standalone_camera_details['fps']}/1 ! "
            "jpegparse ! jpegdec ! videoconvert ! "
            "x264enc speed-preset=ultrafast tune=zerolatency bitrate=2000 ! "
            "rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self.address} port={self.standalone_camera_details['port']}"
        )
        self.capture = cv.VideoCapture(gst_pipeline, cv.CAP_GSTREAMER)
        self.__setCVAttrs()
        return self.capture
        
    def __setCVAttrs(self) -> None:
        self.capture.set(cv.CAP_PROP_BUFFERSIZE, 4)
        self.capture.set(cv.CAP_PROP_FRAME_WIDTH, self.standalone_camera_details['width'])
        self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, self.standalone_camera_details['height'])
        self.capture.set(cv.CAP_PROP_FPS, self.standalone_camera_details['fps'])

    def _setCalibration(self) -> None:
        pass

    def _stitch(self):
        pass 

    def _splitStreams(self):
        pass

    def getCameras(self) -> list:
        return [self.camera_details['left_cam'], self.camera_details['right_cam'], self.camera_details['stitched']]
 