#!/usr/bin/env python3
from zope.interface import Interface
import cv2 as cv

class ICamera(Interface):
    """Responsible for handling camera attributes and initializing cameras"""
    def __init__(self, cameraIndex, format: str) -> None:
        """Initialize the camera
        @param cameraIndex: index at which camera is accessed
        @param format: type of format camera is compatible with (MJPG or GStreamer)"""

    def setFrameSize(self, width: int, height: int) -> None:
        """Sets the resolution of the camera"""

    def setFPS(self, FPS: int) -> None:
        """Sets FPS of the camera"""

    def setup_camera(self, address: str, port: int) -> cv.VideoCapture:
        """Sets up the camera capture based on the selected format.
        @param address: The address Gstreamer will be using to output
        @param port: The port Gstreamer will be using to output
        @returns object of cv2.VideoCapture"""

    def _setup_mjpg(self) -> cv.VideoCapture:
        """Sets up MJPEG streaming using OpenCV.
        @returns object of cv2.VideoCapture"""

    def _setup_h264(self, address: str, port: int) -> cv.VideoCapture:
        """Sets up H.264 streaming using GStreamer.
        @param address: The address Gstreamer will be using to output
        @param port: The port Gstreamer will be using to output
        @returns object of cv2.VideoCapture"""

    def __setCVAttrs(self) -> None:
        """Sets CV attributes"""
