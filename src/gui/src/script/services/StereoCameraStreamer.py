#!/usr/bin/env python3

from multiprocessing import Process
import cv2
from utils.EnvParams import EnvParams
from interface.ICamera import ICamera
import pyshine as ps
import os
import sys

# sys.stderr = open(os.devnull, 'w')
# sys.stdout = open(os.devnull, 'w')
class StereoCameraStreamer:
    def __init__(self, camera: ICamera) -> None:
        self.address = EnvParams().WEB_DOMAIN
        self.camera = camera
        self.left_port = self.camera.getLeftPort()
        self.right_port = self.camera.getRightPort()
        self.capture = None
        self.left_process = None
        self.right_process = None
        self.server = None
        
    def __run(self, port, capture):
        try:
            StreamProps = ps.StreamProps
            StreamProps.set_Page(StreamProps, "")
            # self.capture = self.camera.setupCamera()
            address = (self.address, port)
            # if not self.capture.isOpened():
            #     raise Exception("Failed to open camera with GStreamer pipeline.")
    
            StreamProps.set_Mode(StreamProps, 'cv2')
            StreamProps.set_Quality(StreamProps, 90)
            if port == self.left_port:  
                print("anaaaaa henaaaaaa")
                self.server = ps.Streamer(address, frame_generator=self.camera.left_frame_gen)
            else:
                self.server = ps.Streamer(address, frame_generator=self.camera.right_frame_gen)
            print(f"Port: {port}")
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
        self.capture = self.camera.setupCamera()
        if self.capture is None:
            raise Exception("Failed to open camera with GStreamer pipeline.")
        else:
            self.left_process = Process(target=self.__run, args=(self.left_port,self.capture))
            self.right_process = Process(target=self.__run, args=(self.right_port,self.capture))
            self.left_process.daemon = True
            self.right_process.daemon = True
            self.left_process.start()
            self.right_process.start()

    def closeStream(self):
        if self.capture is not None:
            self.capture.release()
        if self.server is not None:
            self.server.socket.close()
        if self.process is not None:
            self.process.terminate()
            self.process.join()