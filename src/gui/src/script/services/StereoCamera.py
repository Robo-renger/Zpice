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
        self.capture = None
        self.stitch_process = None
        self.split_process = None
        self.standalone_camera_details = {}
        self.stereo_stitcher = StereoStitcher(cameraDetails)
        # self.__splitStereo()
        self.__stitch()

    def __splitStereo(self):
        self.split_process = Process(target=self.__runSplitter)
        self.split_process.daemon = True
        self.split_process.start()

    def __runSplitter(self):
        self.cameraIndex = "/dev/video4"
        command = f"""
            gst-launch-1.0 -v v4l2src device={self.cameraIndex} ! \
                video/x-raw,width={int(self.width)},height={int(self.height)} ! videoconvert ! tee name=t \
                t. ! queue ! videocrop right={int(self.width / 2)} ! videoconvert ! v4l2sink device=/dev/video17 \
                t. ! queue ! videocrop left={int(self.width / 2)} ! videoconvert ! v4l2sink device=/dev/video18
            """
        subprocess.run(command, shell=True, executable="/bin/bash")

    def __stitch(self):
        print("anaaaaaaaa b stitchhhhh")
        self.stitch_process = Process(target=self.__runStitcher)
        self.stitch_process.daemon = True
        self.stitch_process.start()

    def __runStitcher(self):
        try:
            self.stereo_stitcher.stitch()
        except Exception as e:
            print(f"Error occurred: {e}")
        finally:
            self.stitch_process.terminate()
            self.stitch_process.join()
    
    
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

    def getPort(self):
        return self.port
    
    def getFrame(self):
        if self.capture is not None:
            self.frame = self.capture.read()

    def read(self):
        ret, frame = self.capture.read()
        if ret:
            frame = cv.undistort(frame, self.mtx, self.dist, None, self.newCameraMtx)
        return ret, frame

    def isOpened(self):
        return self.capture.isOpened()

    def release(self):
        self.capture.release()
        self.stitch_process.terminate()
        self.stitch_process.join()
        self.split_process.terminate()
        self.split_process.join()

    def get(self, prop_id):
        return self.capture.get(prop_id)
    
    def getCameras(self) -> list:
        return [self.camera_details['left_cam'], self.camera_details['right_cam'], self.camera_details['stitched']]
 