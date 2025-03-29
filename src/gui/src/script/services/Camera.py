#!/usr/bin/env python3
from zope.interface import implementer
from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity
from interface.ICamera import ICamera
from services.StereoStitcher import StereoStitcher
import cv2 as cv
import numpy as np 

@implementer(ICamera)
class Camera:
    """Responsible for handling camera attributes and initializing cameras"""
    def __init__(self, cameraIndex, format: str  = 'MJPG', type: str = 'Monocular') -> None:
        """Initialize the camera
        @param cameraIndex: index at which camera is accessed
        @param format: type of format camera is compatible with (MJPG or GStreamer)"""
        self.cameraIndex = cameraIndex
        self.width = 1280
        self.height = 720
        self.FPS = 60
        self.format_type = format
        self.camera_type = type
        self.capture = None

    def setFrameSize(self, width: int, height: int) -> None:
        """Sets the resolution of the camera"""
        self.width = width
        self.height = height

    def setFPS(self, FPS: int) -> None:
        """Sets FPS of the camera"""
        self.FPS = FPS

    def setup_camera(self, address: str, port: int):
        """Sets up the camera capture based on the selected format."""
        if self.camera_type == "Monocular":
            pass
        elif self.camera_type == "Stereo":
            self._setup_stereo()
        else:
            raise TypeError("Unsupported camera type. Choose either 'Monocular' or 'Stereo'.")
        
        if self.format_type == "H264":
            return self._setup_h264(address, port)
        elif self.format_type == "MJPG":
            return self._setup_mjpg()
        else:
            raise ValueError("Unsupported format. Choose either 'MJPG' or 'H264'.")
        
    def _setup_mjpg(self):
        """Sets up MJPEG streaming using OpenCV."""
        print("Initializing camera with MJPEG format...")
        self.capture = cv.VideoCapture(self.cameraIndex)
        fourcc = cv.VideoWriter_fourcc(*'MJPG')
        self.capture.set(cv.CAP_PROP_FOURCC, fourcc)
        self.__setCVAttrs()
        return self.capture

    def _setup_h264(self, address: str, port: int):
        print("eshta")
        """Sets up H.264 streaming using GStreamer."""
        print("Initializing camera with H.264 format...")
        gst_pipeline = (
            f"v4l2src device={self.cameraIndex} ! "
            f"image/jpeg, width={self.width}, height={self.height}, framerate={self.FPS}/1 ! "
            "jpegparse ! jpegdec ! videoconvert ! "
            "x264enc speed-preset=ultrafast tune=zerolatency bitrate=2000 ! "
            "rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={address} port={port}"
        )

        self.capture = cv.VideoCapture(gst_pipeline, cv.CAP_GSTREAMER)
        self.__setCVAttrs()
        return self.capture
    
    def _setup_stereo(self):
        """Take the video device of the stereo, split it into two video devices, pass them to the sticher finally return the output device """
        self.cameraIndex = StereoStitcher(self.cameraIndex).setup()
    
    def __setCVAttrs(self) -> None:
        self.capture.set(cv.CAP_PROP_BUFFERSIZE, 4)
        self.capture.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv.CAP_PROP_FPS, self.FPS)
