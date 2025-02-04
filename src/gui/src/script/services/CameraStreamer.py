#!/usr/bin/env python3

from multiprocessing import Process
import cv2
import pyshine as ps

class CameraStreamer:
    def __init__(self, cameraIndex,html_content) -> None:
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
            self.capture = cv2.VideoCapture(self.cameraIndex)
            use_format = 'MJPG'  # Change to 'YUYV' if you want YUYV format
            fourcc = cv2.VideoWriter_fourcc(*use_format)
            self.capture.set(cv2.CAP_PROP_FOURCC, fourcc)
            StreamProps.set_Mode(StreamProps, 'cv2')
            self.__setCVAttrs(self.capture)
            StreamProps.set_Capture(StreamProps, self.capture)
            StreamProps.set_Quality(StreamProps, 90)
            width = self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = self.capture.get(cv2.CAP_PROP_FPS)
            fourcc_code = int(self.capture.get(cv2.CAP_PROP_FOURCC))
            format_used = "".join([chr((fourcc_code >> 8 * i) & 0xFF) for i in range(4)])  # Decode FourCC code

            print(f"Resolution: {int(width)}x{int(height)}, FPS: {int(fps)}, Format: {format_used}")
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