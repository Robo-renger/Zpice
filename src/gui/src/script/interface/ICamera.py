#!/usr/bin/env python3
from zope.interface import Interface
import cv2 as cv

class ICamera(Interface):
    """Responsible for handling camera attributes and initializing cameras"""
    def __init__(self, cameraDetails: dict) -> None:
        """Initialize the camera
        @param cameraDetails: all details of the camera found in the config
        """
   
    def setupCamera(self) -> cv.VideoCapture:
        """Sets up the camera capture based on the selected format.
        @returns object of cv2.VideoCapture"""

    def _setup_mjpg(self) -> cv.VideoCapture:
        """Sets up MJPEG streaming using OpenCV.
        @returns object of cv2.VideoCapture"""

    def _setup_h264(self) -> cv.VideoCapture:
        """Sets up H.264 streaming using GStreamer.
        @returns object of cv2.VideoCapture"""

    def __setCVAttrs(self) -> None:
        """Sets CV attributes"""
