#!/usr/bin/env python3
import cv2 as cv
import rospkg
import numpy as np
import glob

class StereoCalibrator:
    def __init__(self, cameraDetails: dict, chessboard_size: tuple):
        rospack = rospkg.RosPack()
        workspace_path = rospack.get_path('gui')
        self.frame_count = 0
        self.right_images_path = workspace_path + f'/../../calibrationImages/Stereo/right/'
        self.left_images_path = workspace_path + f'/../../calibrationImages/Stereo/left/'
        self.left_dist = workspace_path + f'/../../calibrationMatricies/calibration_data/distL.npy' # Distortion matrix for left
        self.right_dist = workspace_path + f'/../../calibrationMatricies/calibration_data/distR.npy' # Distortion matrix for right
        self.left_matrix = workspace_path + f'/../../calibrationMatricies/calibration_data/mtxL.npy' # Intrinsic matrix for left
        self.right_matrix = workspace_path + f'/../../calibrationMatricies/calibration_data/mtxR.npy' # Intrinsic matrix for right
        self.left_P = workspace_path + f'/../../calibrationMatricies/calibration_data/P1.npy' # Projection matrix for left
        self.right_P = workspace_path + f'/../../calibrationMatricies/calibration_data/P2.npy' # Projection matrix for right 
        self.left_R = workspace_path + f'/../../calibrationMatricies/calibration_data/R1.npy' # Rectification transforms for left 
        self.right_R = workspace_path + f'/../../calibrationMatricies/calibration_data/R2.npy' # Rectification transforms for right
        self.Q_path = workspace_path + f'/../../calibrationMatricies/calibration_data/Q.npy' # Disparity-to-depth map matrix
        self.R_path = workspace_path + f'/../../calibrationMatricies/calibration_data/R.npy' # Rotation matrix from left to right
        self.T_path = workspace_path + f'/../../calibrationMatricies/calibration_data/T.npy' # Translation matrix from left to right
        self.camera_details = cameraDetails
        self.chessboard_size = chessboard_size
        self.index = self.camera_details['index']
        self.width = self.camera_details['width']
        self.height = self.camera_details['height']
        self.termination_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32) # Create 3D object points for the chessboard pattern
        self.objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        self.num_valid_calibration_images = 0
        self.capture = None
        self.objpoints = []
        self.imgpoints_left = []
        self.imgpoints_right = []
        
        # Calibration matrices
        self.mtx_left = None
        self.dist_left = None
        self.mtx_right = None
        self.dist_right = None
        self.R1 = None
        self.R2 = None
        self.P1 = None
        self.P2 = None
        self.Q = None

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
        try:
            left_images = sorted(glob.glob(f"{self.left_images_path}*.png"))
            right_images = sorted(glob.glob(f"{self.right_images_path}*.png"))
            for left_img, right_img in zip(left_images, right_images):
                imgL = cv.imread(left_img)
                imgR = cv.imread(right_img)
                grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
                grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

                # Find chessboard corners
                retL, cornersL = cv.findChessboardCorners(grayL, self.chessboard_size, None)
                retR, cornersR = cv.findChessboardCorners(grayR, self.chessboard_size, None)

                if retL and retR:
                    self.num_valid_calibration_images += 1
                    self.objpoints.append(self.objp)
                    cornersL = cv.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), self.termination_criteria)
                    cornersR = cv.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), self.termination_criteria)
                    self.imgpoints_left.append(cornersL)
                    self.imgpoints_right.append(cornersR)

                    # Draw and display detected corners
                    cv.drawChessboardCorners(imgL, self.chessboard_size, cornersL, retL)
                    cv.drawChessboardCorners(imgR, self.chessboard_size, cornersR, retR)
                    cv.imshow('Left Chessboard', imgL) # to be deleted --> not necessary just for testing locally
                    cv.imshow('Right Chessboard', imgR)# to be deleted --> not necessary just for testing locally
                else:
                    print(f"Failed to detect corners in {left_img} or {right_img}")
                    cv.imshow('Left Image (No Corners)', imgL)# to be deleted --> not necessary just for testing locally
                    cv.imshow('Right Image (No Corners)', imgR)# to be deleted --> not necessary just for testing locally

                cv.waitKey(500)  # Delay to view images # to be deleted --> not necessary just for testing locally
            print(f"Number of valid images used for calibration: {self.num_valid_calibration_images}")

            # Calibrate each camera separately
            retL, mtxL, distL, rvecsL, tvecsL = cv.calibrateCamera(self.objpoints, self.imgpoints_left, grayL.shape[::-1], None, None)
            retR, mtxR, distR, rvecsR, tvecsR = cv.calibrateCamera(self.objpoints, self.imgpoints_right, grayR.shape[::-1], None, None)

            # Stereo calibration # R --> Rotation matrix from left to right, T --> Translation matrix from left to right
            retS, mtxL, distL, mtxR, distR, R, T, E, F = cv.stereoCalibrate(
                self.objpoints, self.imgpoints_left, self.imgpoints_right, mtxL, distL, mtxR, distR, grayL.shape[::-1],
                criteria=self.termination_criteria, flags=cv.CALIB_FIX_INTRINSIC
            )

            # Compute rectification transforms for future image correction
            R1, R2, P1, P2, Q, roiL, roiR = cv.stereoRectify(mtxL, distL, mtxR, distR, grayL.shape[::-1], R, T)

            self.__saveMatricies(mtxL, distL, mtxR, distR, R1, R2, P1, P2, Q, R, T)
        except Exception as e:
            print(f"Error occured calibrating images: {e}")
            exit()
        
    def __saveMatricies(self, mtxL, distL, mtxR, distR, R1, R2, P1, P2, Q, R, T):
        try:
            np.save(self.left_matrix, mtxL)
            np.save(self.left_dist, distL)
            np.save(self.right_matrix, mtxR)
            np.save(self.right_dist, distR)
            np.save(self.left_R, R1)
            np.save(self.right_R, R2)
            np.save(self.left_P, P1)
            np.save(self.right_P, P2)
            np.save(self.Q_path, Q)
            np.save(self.R_path, R)
            np.save(self.T_path, T)

            print("Calibration completed and data saved.")
            cv.destroyAllWindows()
        except Exception as e:
            print(f"Error occured saving calibration matricies: {e}")

    def __loadCalibrationMatrices(self):
        """Load calibration matrices from saved files"""
        try:
            self.mtx_left = np.load(self.left_matrix)
            self.dist_left = np.load(self.left_dist)
            self.mtx_right = np.load(self.right_matrix)
            self.dist_right = np.load(self.right_dist)
            self.R1 = np.load(self.left_R)
            self.R2 = np.load(self.right_R)
            self.P1 = np.load(self.left_P)
            self.P2 = np.load(self.right_P)
            self.Q = np.load(self.Q_path)
            return True
        except Exception as e:
            print(f"Error loading calibration matrices: {e}")
            return False

    def testCalibration(self):
        """Test stereo camera calibration with live feed"""
        if not self.__loadCalibrationMatrices():
            print("Failed to load calibration matrices. Please run calibration first.")
            return

        cap = cv.VideoCapture(self.index)
        cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)

        if not cap.isOpened():
            print("Error: Could not open camera.")
            return

        try:
            # Initialize undistortion maps
            mapL1, mapL2 = cv.initUndistortRectifyMap(
                self.mtx_left, self.dist_left, self.R1, self.P1, 
                (int(self.width/2), self.height), cv.CV_16SC2
            )
            mapR1, mapR2 = cv.initUndistortRectifyMap(
                self.mtx_right, self.dist_right, self.R2, self.P2, 
                (int(self.width/2), self.height), cv.CV_16SC2
            )

            frame_count = 0

            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to grab frame")
                    break
                
                height, width = frame.shape[:2]
                half_width = width // 2
                left_frame = frame[:, :half_width]
                right_frame = frame[:, half_width:]
            
                # Undistort and rectify images
                rectified_left = cv.remap(left_frame, mapL1, mapL2, cv.INTER_LINEAR)
                rectified_right = cv.remap(right_frame, mapR1, mapR2, cv.INTER_LINEAR)
            
                # Display results
                cv.imshow("Rectified Left", rectified_left)
                cv.imshow("Rectified Right", rectified_right)
            
                key = cv.waitKey(1) & 0xFF
                if key == ord('s'):
                    left_path = f"calibration_data/rectified_left_{frame_count}.png"
                    right_path = f"calibration_data/rectified_right_{frame_count}.png"
                    cv.imwrite(left_path, rectified_left)
                    cv.imwrite(right_path, rectified_right)
                    print(f"Saved: {left_path}, {right_path}")
                    frame_count += 1
                
                if key == ord('q'):
                    break
                
        except Exception as e:
            print(f"Error during calibration testing: {e}")
        finally:
            cap.release()
            cv.destroyAllWindows()
