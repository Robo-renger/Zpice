#!/usr/bin/env python3

from multiprocessing import Process
import cv2
import pyshine as ps

class CameraStreamer:
    def __init__(self, cameraIndex: str,html_content) -> None:
        self.address = "192.168.1.233"  # Fetch from a config file/dynamically
        self.cameraIndex = cameraIndex
        self.width = 1280
        self.height = 720
        self.FPS = 60
        self.process = None
        self.capture = None
        self.server = None
        self.html_content = html_content
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
    def __run(self, port):
        try:
            StreamProps = ps.StreamProps
            StreamProps.set_Page(StreamProps, self.html_content)
            address = (self.address, port)
            self.capture = cv2.VideoCapture(self.cameraIndex, cv2.CAP_V4L2)
            # cap = cv2.VideoCapture("/dev/rapoo_camera", cv2.CAP_V4L2)
            StreamProps.set_Mode(StreamProps, 'cv2')
            self.__setCVAttrs(self.capture)
            StreamProps.set_Capture(StreamProps, self.capture)
            StreamProps.set_Quality(StreamProps, 90)
            self.server = ps.Streamer(address, StreamProps)
            self.server.serve_forever()
        except Exception as e:
            print(f"Error occurred: {e}")
        finally:
            if self.capture is not None:
                self.capture.release()
            if self.server is not None:
                self.server.socket.close()

    def stream(self, port: int):
        self.process = Process(target=self.__run, args=(port,))
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
