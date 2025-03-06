#!/usr/bin/env python3

from multiprocessing import Process
import cv2
from utils.EnvParams import EnvParams
import pyshine as ps

class CameraStreamer:
    def __init__(self, cameraIndex, port,format = "MJPG") -> None:
        self.address = EnvParams().WEB_DOMAIN
        self.cameraIndex = cameraIndex
        self.width = 1280
        self.height = 720
        self.FPS = 60
        self.port = port
        self.process = None
        self.capture = None
        self.server = None
        self.format_type = format
        
        
    def setFrameSize(self, width=1280, height=720):
        self.width = width
        self.height = height

    def setFPS(self, FPS):
        self.FPS = FPS
    def __setCVAttrs(self, capture):
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 4)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.FPS)
        
    def __setup_camera(self):
        """Sets up the camera capture based on the selected format."""
        if self.format_type == "H264":
            return self._setup_h264()
        elif self.format_type == "MJPG":
            return self._setup_mjpg()
        else:
            raise ValueError("Unsupported format. Choose either 'MJPG' or 'H264'.")
        
    def _setup_mjpg(self):
        """Sets up MJPEG streaming using OpenCV."""
        print("Initializing camera with MJPEG format...")
        self.capture = cv2.VideoCapture(self.cameraIndex)
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.capture.set(cv2.CAP_PROP_FOURCC, fourcc)
        return self.capture

    def _setup_h264(self):
        print("eshta")
        """Sets up H.264 streaming using GStreamer."""
        print("Initializing camera with H.264 format...")
        gst_pipeline = (
            "v4l2src device=/dev/right_camera ! "
            "image/jpeg, width=1280, height=720, framerate=30/1 ! "
            "jpegparse ! jpegdec ! videoconvert ! "
            "x264enc speed-preset=ultrafast tune=zerolatency bitrate=2000 ! "
            "rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self.address} port={self.port}"
        )


        self.capture = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        return self.capture
        
    def __run(self):
        try:
            StreamProps = ps.StreamProps
            StreamProps.set_Page(StreamProps, "")
            address = (self.address, self.port)
    
            self.capture = self.__setup_camera()
    
            if not self.capture.isOpened():
                raise Exception("Failed to open camera with GStreamer pipeline.")
    
            StreamProps.set_Mode(StreamProps, 'cv2')
            self.__setCVAttrs(self.capture)
            StreamProps.set_Capture(StreamProps, self.capture)
            StreamProps.set_Quality(StreamProps, 90)
    
            width = self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = self.capture.get(cv2.CAP_PROP_FPS)
            fourcc_code = int(self.capture.get(cv2.CAP_PROP_FOURCC))
            format_used = "".join([chr((fourcc_code >> 8 * i) & 0xFF) for i in range(4)])
    
            print(f"Resolution: {int(width)}x{int(height)}, FPS: {int(fps)}, Format: {format_used}")
    
            self.server = ps.Streamer(address, StreamProps)
            self.server.serve_forever()

        except Exception as e:
            pass
            print(f"Error occurred: {e}")
        finally:
            if self.capture is not None:
                self.capture.release()
            if self.server is not None:
                self.server.socket.close()

    def stream(self):
        self.process = Process(target=self.__run)
        self.process.daemon = True
        self.process.start()

    def closeStream(self):
        if self.capture is not None:
            self.capture.release()
        if self.server is not None:
            self.server.socket.close()
        if self.process is not None:
            self.process.terminate()
            self.process.join()