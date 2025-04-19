#!/usr/bin/env python3
from zope.interface import implementer
from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity
from interface.ICamera import ICamera
from utils.EnvParams import EnvParams
import cv2 as cv
import numpy as np 
import rospkg

@implementer(ICamera)
class RightStereoCamera:
    """Responsible for handling camera attributes and initializing cameras"""
    def __init__(self, cameraDetails: dict) -> None:
        """Initialize the camera
        @param cameraDetails: all details of the camera found in the config"""
        rospack = rospkg.RosPack()
        workspace_path = rospack.get_path('gui')
        self.address = EnvParams().WEB_DOMAIN
        self.cameraIndex = cameraDetails['index']
        self.width = cameraDetails['width']
        self.height = cameraDetails['height']
        self.FPS = cameraDetails['fps']
        self.compression = cameraDetails['format']
        self.port = cameraDetails['right_stereo']['port']
        self.mtx_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/mtxR.npy'
        self.dist_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/distR.npy'
        self.P1_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/P2.npy'
        self.Q_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/Q.npy'
        self.R_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/R.npy'
        self.R1_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/R2.npy'
        self.T_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/T.npy'
        self.capture = None
        self.frame = None
        self.mtx = None
        self.dist = None
        self.P2 = None
        self.Q = None
        self.R = None
        self.R2 = None
        self.T = None
        self.loadCalibrationData()
        self.mapR1, self.mapR2 = cv.initUndistortRectifyMap(self.mtx, self.dist, self.R2, self.P2, (640, 480), cv.CV_16SC2)
        

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
    
    def getFrame(self):
        if self.capture is not None:
            self.frame = self.capture.read()
            self.frame = self.frame[:, int(self.width/2):]
            self.frame = cv.remap(self.frame, self.mapR1, self.mapR2, cv.INTER_LINEAR)

    def read(self):
        ret, frame = self.capture.read()
        if ret:
            frame = frame[:, int(self.width/2):]
            frame = cv.remap(frame, self.mapR1, self.mapR2, cv.INTER_LINEAR)
        return ret, frame

    def isOpened(self):
        return self.capture.isOpened()

    def release(self):
        self.capture.release()

    def get(self, prop_id):
        return self.capture.get(prop_id)

    def loadCalibrationData(self):
        self.mtx = np.load(self.mtx_path)
        self.dist = np.load(self.dist_path)
        self.P1 = np.load(self.P1_path)
        self.Q = np.load(self.Q_path)
        self.R = np.load(self.R_path)
        
 
 