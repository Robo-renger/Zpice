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
class VisualizeDepthMap:
    """Responsible for handling camera attributes and initializing cameras"""
    def __init__(self, cameraDetails: dict) -> None:
        """Initialize the camera
        @param cameraDetails: all details of the camera found in the config"""
        rospack = rospkg.RosPack()
        workspace_path = rospack.get_path('gui')
        self.mtx_path_left = workspace_path + f'/../../calibrationMatricies/calibration_data2/mtxL.npy'
        self.dist_path_left = workspace_path + f'/../../calibrationMatricies/calibration_data2/distL.npy'
        self.P1_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/P1.npy'
        self.R1_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/R1.npy'
        self.mtx_path_right = workspace_path + f'/../../calibrationMatricies/calibration_data2/mtxR.npy'
        self.dist_path_right = workspace_path + f'/../../calibrationMatricies/calibration_data2/distR.npy'
        self.P2_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/P2.npy'
        self.R2_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/R2.npy'
        self.Q_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/Q.npy'
        self.R_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/R.npy'
        self.T_path = workspace_path + f'/../../calibrationMatricies/calibration_data2/T.npy'
        self.address = EnvParams().WEB_DOMAIN
        self.cameraIndex = cameraDetails['index']
        self.width = cameraDetails['width']
        self.height = cameraDetails['height']
        self.FPS = cameraDetails['fps']
        self.compression = cameraDetails['format']
        self.port = cameraDetails['left_stereo']['port']
        self.mtxL = None
        self.distL = None
        self.mtxR = None
        self.distR = None
        self.P1 = None
        self.R1 = None
        self.P2 = None
        self.R2 = None
        self.T = None
        self.Q = None 
        self.R = None
        self.capture = None
        self.frame = None
        self.loadCalibrationData()
        self.mapL1, self.mapL2 = cv.initUndistortRectifyMap(self.mtxL, self.distL, self.R1, self.P1, (640, 480), cv.CV_16SC2)
        self.mapR1, self.mapR2 = cv.initUndistortRectifyMap(self.mtxR, self.distR, self.R2, self.P2, (640, 480), cv.CV_16SC2)
        self.stereo = cv.StereoBM.create(numDisparities=32, blockSize=15) 

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
        self.capture.set(cv.CAP_PROP_FRAME_WIDTH, int(self.width / 2))
        self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv.CAP_PROP_FPS, self.FPS)

    def getPort(self):
        return self.port
    
    def getFrame(self):
        if self.capture is not None:
            frame = self.capture.read()
            frame_left = frame[:, :int(self.width/2)]
            frame_right = frame[:, int(self.width/2):]
            rectified_left = cv.remap(frame_left, self.mapL1, self.mapL2, cv.INTER_LINEAR)
            rectified_right = cv.remap(frame_right, self.mapR1, self.mapR2, cv.INTER_LINEAR)
            grayL = cv.cvtColor(rectified_left, cv.COLOR_BGR2GRAY)
            grayR = cv.cvtColor(rectified_right, cv.COLOR_BGR2GRAY)
            disparity = self.stereo.compute(grayL, grayR)
            depth_map = cv.normalize(disparity, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
            self.frame = depth_map
        
    def read(self):
        ret, frame = self.capture.read()
        if ret:
            frame = self.capture.read()
            frame_left = frame[:, :int(self.width/2)]
            frame_right = frame[:, int(self.width/2):]
            rectified_left = cv.remap(frame_left, self.mapL1, self.mapL2, cv.INTER_LINEAR)
            rectified_right = cv.remap(frame_right, self.mapR1, self.mapR2, cv.INTER_LINEAR)
            grayL = cv.cvtColor(rectified_left, cv.COLOR_BGR2GRAY)
            grayR = cv.cvtColor(rectified_right, cv.COLOR_BGR2GRAY)
            disparity = self.stereo.compute(grayL, grayR)
            depth_map = cv.normalize(disparity, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
            return ret, frame
    
    def isOpened(self):
        return self.capture.isOpened()

    def release(self):
        self.capture.release()

    def get(self, prop_id):
        return self.capture.get(prop_id)

    def loadCalibrationData(self):
        self.mtxL = np.load(self.mtx_path_left)
        self.distL = np.load(self.dist_path_left)
        self.mtxR = np.load(self.mtx_path_right)
        self.distR = np.load(self.dist_path_right)
        self.P1 = np.load(self.P1_path)
        self.P2 = np.load(self.P2_path)
        self.R1 = np.load(self.R1_path)
        self.R2 = np.load(self.R2_path)
        self.Q = np.load(self.Q_path)
        self.R = np.load(self.R_path)
        self.T = np.load(self.T_path)

    def computeDepthMap(self):
        pass

        
 
 