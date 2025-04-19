#!/usr/bin/env python3

from multiprocessing import Process
import cv2
from utils.EnvParams import EnvParams
from interface.ICamera import ICamera
import pyshine as ps
import os
import sys


sys.stderr = open(os.devnull, 'w')
sys.stdout = open(os.devnull, 'w')
class CameraStreamer:
    def __init__(self, camera: ICamera) -> None:
        self.address = EnvParams().WEB_DOMAIN
        self.camera = camera
        self.port = self.camera.getPort()
        self.capture = None
        self.process = None
        self.server = None
        
    def __run(self):
        try:
            StreamProps = ps.StreamProps
            StreamProps.set_Page(StreamProps, "")
            self.capture = self.camera.setupCamera()
            address = (self.address, self.port)
            if not self.capture.isOpened():
                raise Exception("Failed to open camera with GStreamer pipeline.")
    
            StreamProps.set_Mode(StreamProps, 'cv2')
            StreamProps.set_Capture(StreamProps, self.camera)
            StreamProps.set_Quality(StreamProps, 90)
            print(f"Port: {self.port}")
    
            # width = self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)
            # height = self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
            # fps = self.capture.get(cv2.CAP_PROP_FPS)
            # fourcc_code = int(self.capture.get(cv2.CAP_PROP_FOURCC))
            # format_used = "".join([chr((fourcc_code >> 8 * i) & 0xFF) for i in range(4)])
    
            # print(f"Resolution: {int(width)}x{int(height)}, FPS: {int(fps)}, Format: {format_used}")
    
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