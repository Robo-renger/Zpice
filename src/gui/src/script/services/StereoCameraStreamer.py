#!/usr/bin/env python3

from multiprocessing import Process
import cv2
from utils.EnvParams import EnvParams
from interface.ICamera import ICamera
import pyshine as ps

class StereoCameraStreamer:
    def __init__(self, camera: ICamera) -> None:
        self.address = EnvParams().WEB_DOMAIN
        self.camera = camera
        self.processes = []
        self.captures = []
        self.servers = []
        self.capture = None
        self.server = None
        self.process = None
        
    def __run(self):
        try:
            StreamProps = ps.StreamProps
            StreamProps.set_Page(StreamProps, "")
            capture = self.camera.setupStereoCamera(self.camera_details)
            port = self.camera_details['port']
            self.captures.append(capture)
            server = None
            address = (self.address, port)
            if not capture.isOpened():
                raise Exception("Failed to open camera with GStreamer pipeline.")
    
            StreamProps.set_Mode(StreamProps, 'cv2')
            StreamProps.set_Capture(StreamProps, capture)
            StreamProps.set_Quality(StreamProps, 90)
    
            width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = capture.get(cv2.CAP_PROP_FPS)
            fourcc_code = int(capture.get(cv2.CAP_PROP_FOURCC))
            format_used = "".join([chr((fourcc_code >> 8 * i) & 0xFF) for i in range(4)])
    
            print(f"Resolution: {int(width)}x{int(height)}, FPS: {int(fps)}, Format: {format_used}")
    
            server = ps.Streamer(address, StreamProps)
            self.servers.append(server)
            server.serve_forever()

        except Exception as e:
            print(f"Error occurred: {e}")
        finally:
            if capture is not None:
                capture.release()
            if server is not None:
                server.socket.close()

    def stream(self):
        for camera_details in self.camera.getCameras():
            self.camera_details = camera_details
            process = Process(target=self.__run)
            process.daemon = True
            process.start()
            self.processes.append(process)

    def closeStream(self):
        for capture in self.captures:
            if capture is not None:
                capture.release()
        for server in self.servers:
            if server is not None:
                server.socket.close()
        for process in self.processes:
            if process is not None:
                process.terminate()
                process.join()
