import cv2 as cv
import numpy as np
import os
import rospkg


class StereoStitcher:
    def __init__(self, cameraDetails: dict):
        rospack = rospkg.RosPack()
        workspace_path = rospack.get_path('gui')
        self.homography_file = workspace_path + f'/../../calibrationMatricies/Stereo/stitched/homography.npy'
        self.camera_details = cameraDetails
        self.left_video_path = cameraDetails['left_stereo']['index']
        self.right_video_path = cameraDetails['right_stereo']['index']
        self.output_index = cameraDetails['stitched']['index']
        self.output_stream = cv.VideoWriter(self.output_index, cv.VideoWriter_fourcc(*'MJPG'), cameraDetails['stitched']['fps'], (cameraDetails['stitched']['width'], cameraDetails['stitched']['height']))
        self.H = None
        self.cap_left = None
        self.cap_right = None
        self.frame_width = None
        self.frame_height = None
        self.output_size = None

    def compute_homography(self, ref_left, ref_right):
        gray_left = cv.cvtColor(ref_left, cv.COLOR_BGR2GRAY)
        gray_right = cv.cvtColor(ref_right, cv.COLOR_BGR2GRAY)
        
        orb = cv.ORB_create()
        kp1, des1 = orb.detectAndCompute(gray_left, None)
        kp2, des2 = orb.detectAndCompute(gray_right, None)
        
        bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)
        
        pts_left = np.float32([kp1[m.queryIdx].pt for m in matches[:50]]).reshape(-1, 1, 2)
        pts_right = np.float32([kp2[m.trainIdx].pt for m in matches[:50]]).reshape(-1, 1, 2)
        
        H, _ = cv.findHomography(pts_right, pts_left, cv.RANSAC)
        
        np.save(self.homography_file, H)
        print("Homography matrix saved.")
        
        return H

    def __setup_sticher(self):
        self.cap_left = cv.VideoCapture(self.left_video_path)
        self.cap_right = cv.VideoCapture(self.right_video_path)

        ret1, ref_left = self.cap_left.read()
        ret2, ref_right = self.cap_right.read()
        
        if not ret1 or not ret2:
            print("Error: Could not capture reference frames")
            exit()

        self.frame_height, self.frame_width = ref_left.shape[:2]

        if os.path.exists(self.homography_file):
            self.H = np.load(self.homography_file)
            print("Loaded precomputed homography matrix.")
        else:
            self.H = self.compute_homography(ref_left, ref_right)

        corners = np.array([[0, 0], [self.frame_width, 0], [0, self.frame_height], [self.frame_width, self.frame_height]], dtype=np.float32)
        transformed_corners = cv.perspectiveTransform(corners.reshape(-1, 1, 2), self.H).reshape(-1, 2)
        min_x = int(min(transformed_corners[:, 0]))
        output_width = self.frame_width + abs(min_x)
        self.output_size = (output_width, self.frame_height)

    def __stitch(self):
        print("anaaaaa heaaaaaaa")
        while True:
            ret_left, frame_left = self.cap_left.read()
            ret_right, frame_right = self.cap_right.read()
            
            if not ret_left or not ret_right:
                print("Error: Could not capture frames")
                exit("Error: Could not capture frames EXITING....")
            aligned = cv.warpPerspective(frame_right, self.H, self.output_size)
            aligned[0:self.frame_height, 0:self.frame_width] = frame_left
            self.output_stream.write(aligned)

    def cleanup(self):
        self.cap_left.release()
        self.cap_right.release()
        cv.destroyAllWindows()

    def stitch(self):
        self.__setup_sticher()
        self.__stitch()




