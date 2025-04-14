#!/usr/bin/env python3
import cv2 as cv
import rospkg

class StereoCalibrator:
    def __init__(self, cameraDetails: dict, chessboard_size: tuple):
        rospack = rospkg.RosPack()
        workspace_path = rospack.get_path('gui')
        self.frame_count = 0
        self.right_images_path = workspace_path + f'/../../calibrationImages/Stereo/right/'
        self.left_images_path = workspace_path + f'/../../calibrationImages/Stereo/left/'
        self.camera_details = cameraDetails
        self.chessboard_size = chessboard_size
        self.index = self.camera_details['index']
        self.width = self.camera_details['width']
        self.height = self.camera_details['height']
        self.capture = None

    def __initalizeCamera(self):
        try:
            self.capture = cv.VideoCapture(self.index)
            if not self.capture.isOpened():
                 raise Exception
            self.capture.set(cv.CAP_PROP_FRAME_WIDTH, self.width)  # Set width for stereo image
            self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        except Exception as e:
            print(f"Cannot initialize camera: {e}")
            exit()

    def captureCalibrationImages(self):
        try:
            self.__initalizeCamera()
            while True:
                ret, frame = self.capture.read()
                if not ret:
                    print("Failed to grab frame")
                    raise Exception
                left_frame = frame[:, :int(self.width / 2)].copy()
                right_frame = frame[:, int(self.width / 2):].copy()

                grayL = cv.cvtColor(left_frame, cv.COLOR_BGR2GRAY)
                grayR = cv.cvtColor(right_frame, cv.COLOR_BGR2GRAY)

                # Find chessboard corners
                retL, cornersL = cv.findChessboardCorners(grayL, self.chessboard_size, None)
                retR, cornersR = cv.findChessboardCorners(grayR, self.chessboard_size, None)

                previewL = left_frame.copy()
                previewR = right_frame.copy()
                if retL:
                    cv.drawChessboardCorners(previewL, self.chessboard_size, cornersL, retL)
                if retR:
                    cv.drawChessboardCorners(previewR, self.chessboard_size, cornersR, retR)           

                cv.imshow('Left', previewL)
                cv.imshow('Right', previewR)

                key = cv.waitKey(1)
                if key == ord('s'):
                    left_path = self.left_images_path + f"left_{self.frame_count}.png"
                    right_path = self.right_images_path + f"right_{self.frame_count}.png"
                    cv.imwrite(left_path, left_frame)
                    cv.imwrite(right_path, right_frame)
                    self.frame_count += 1
                    print(f"Saved: {left_path},\n {right_path}\n {self.frame_count}")
                elif key == ord('q'):
                    break
            self.capture.release()
            cv.destroyAllWindows()

        except Exception as e:
            print(f"Error occured capturing calibration images: {e}")
            exit()

    def calibrate(self):
        pass
