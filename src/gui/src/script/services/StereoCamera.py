#!/usr/bin/env python3
from zope.interface import implementer
from multiprocessing import Process
from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity
from interface.ICamera import ICamera
from utils.EnvParams import EnvParams
from services.StereoStitcher import StereoStitcher
import cv2 as cv
import numpy as np
import subprocess 

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
        self.focal_length = cameraDetails['focal_length']
        self.baseline = cameraDetails['baseline']
        self.port = cameraDetails['port']
        self.left_port = cameraDetails['left_stereo']['port']
        self.right_port = cameraDetails['right_stereo']['port']
        self.capture = None
        self.frame = None
        self.frame_left = None
        self.frame_right = None

 
    
    def setupCamera(self) -> cv.VideoCapture:
        """Sets up the camera capture based on the selected format."""   
        if self.compression == "H264":
            return self._setupH264()
        elif self.compression == "MJPG":
            return self._setupMJPG()
        else:
            raise ValueError("Unsupported format. Choose either 'MJPG' or 'H264'.")     
        
    def _setupMJPG(self):
        """Sets up MJPEG streaming using OpenCV."""
        print("Initializing camera with MJPEG format...")
        self.capture = cv.VideoCapture(self.cameraIndex)
        fourcc = cv.VideoWriter_fourcc(*'MJPG')
        self.capture.set(cv.CAP_PROP_FOURCC, fourcc)
        self.__setCVAttrs()
        return self.capture

    def _setupH264(self):
        print("eshta")
        """Sets up H.264 streaming using GStreamer."""
        print("Initializing camera with H.264 format...")
        gst_pipeline = (
            f"v4l2src device={self.cameraIndex} ! "
            f"image/jpeg, width={self.width}, height={self.height}, framerate={self.FPS}/1 ! "
            "jpegparse ! jpegdec ! videoconvert ! "
            "x264enc speed-preset=ultrafast tune=zerolatency bitrate=2000 ! "
            "rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self.address} port={self.port}"
        )
        self.capture = cv.VideoCapture(gst_pipeline, cv.CAP_GSTREAMER)
        self.__setCVAttrs()
        return self.capture
        
    def __setCVAttrs(self) -> None:
        self.capture.set(cv.CAP_PROP_BUFFERSIZE, 4)
        self.capture.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv.CAP_PROP_FPS, self.FPS)

    def getPort(self):
        return self.port
    
    def getLeftPort(self):
        return self.left_port
    
    def getRightPort(self):
        return self.right_port
    
    def left_frame_gen(self):
        while True:
            ret, frame = self.capture.read()
            if not ret:
                continue
            frame_left = frame[:, int(self.width / 2):]
            yield frame_left
    
    def right_frame_gen(self):
        while True:
            ret, frame = self.capture.read()
            if not ret:
                continue
            frame_right = frame[:, :int(self.width / 2)]
            yield frame_right
    
    def getFrame(self):
        if self.capture is not None:
            self.frame = self.capture.read()
            return self.frame
        else:
            return None

    def read(self):
        ret, frame = self.capture.read()
        if ret:
            self.frame = frame
            self.frame_left = frame[:, :int(self.width / 2)]
            self.frame_right = frame[:, int(self.width / 2):]
        return ret, frame

    def isOpened(self):
        return self.capture.isOpened()

    def release(self):
        if self.capture is not None:
            self.capture.release()


    def get(self, prop_id):
        return self.capture.get(prop_id)
    