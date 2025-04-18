import cv2 as cv
import numpy as np
import os
import rospkg
import rospy
from interface.ICamera import ICamera
from zope.interface import implementer

@implementer(ICamera)
class StereoStitcher:
    def __init__(self, cameraDetails: dict):
        rospack = rospkg.RosPack()
        workspace_path = rospack.get_path('gui')
        self.homography_file = workspace_path + f'/../../calibrationMatricies/Stereo/stitched/homography.npy'
        self.camera_details = cameraDetails
        self.left_video_path = "/dev/video17"  # Fixed path for left camera
        self.right_video_path = "/dev/video18"  # Fixed path for right camera
        self.stitched_port = cameraDetails['stitched']['port']
   # Initialize to None - will be set up in setupCamera()
        self.cap_left = None
        self.cap_right = None
        self.H = None
        self.frame_width = None
        self.frame_height = None
        self.output_size = None
        self.aligned = None

    def __compute_homography(self, ref_left, ref_right):
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
        rospy.loginfo("Homography matrix saved.")
        
        return H

    def setupCamera(self):
        """Initialize camera captures and compute homography"""
        rospy.loginfo("Setting up stereo cameras...")
        
        # Initialize captures if not already done
        if self.cap_left is None:
            self.cap_left = cv.VideoCapture(self.left_video_path)
        if self.cap_right is None:
            self.cap_right = cv.VideoCapture(self.right_video_path)

        if not self.cap_left.isOpened() or not self.cap_right.isOpened():
            rospy.logerr("Failed to open one or both cameras")
            return False

        ret1, ref_left = self.cap_left.read()
        ret2, ref_right = self.cap_right.read()

        if not ret1 or not ret2:
            rospy.logerr("Error: Could not capture reference frames")
            return False

        self.frame_height, self.frame_width = ref_left.shape[:2]

        if os.path.exists(self.homography_file):
            self.H = np.load(self.homography_file)
            rospy.loginfo("Loaded precomputed homography matrix.")
        else:
            self.H = self.__compute_homography(ref_left, ref_right)

        corners = np.array([[0, 0], [self.frame_width, 0], [0, self.frame_height], [self.frame_width, self.frame_height]], dtype=np.float32)
        transformed_corners = cv.perspectiveTransform(corners.reshape(-1, 1, 2), self.H).reshape(-1, 2)
        min_x = int(min(transformed_corners[:, 0]))
        output_width = self.frame_width + abs(min_x)
        self.output_size = (output_width, self.frame_height)
        return self.cap_left

    def getPort(self):
        return self.stitched_port

    def getFrame(self):
        return self.aligned

    def read(self):
        """Read and stitch frames from both cameras"""
        if not self.isOpened():
            rospy.logerr("Cameras are not properly initialized")
            return False, None

        ret_left, frame_left = self.cap_left.read()
        ret_right, frame_right = self.cap_right.read()

        if not ret_left or not ret_right:
            rospy.logerr("Failed to capture frames from one or both cameras")
            return False, None

        try:
            self.aligned = cv.warpPerspective(frame_right, self.H, self.output_size)
            self.aligned[0:self.frame_height, 0:self.frame_width] = frame_left
            return True, self.aligned
        except Exception as e:
            rospy.logerr(f"Error during frame stitching: {str(e)}")
            return False, None

    def isOpened(self):
        return (self.cap_left is not None and self.cap_right is not None and 
                self.cap_left.isOpened() and self.cap_right.isOpened())

    def release(self):
        if self.cap_left is not None:
            self.cap_left.release()
            self.cap_left = None
        if self.cap_right is not None:
            self.cap_right.release()
            self.cap_right = None

    def get(self, prop_id):
        if self.cap_left is not None:
            return self.cap_left.get(prop_id)
        return 0

    def _setup_mjpg(self) -> cv.VideoCapture:
        """Sets up MJPEG streaming using OpenCV.
        @returns object of cv2.VideoCapture"""
        pass

    def _setup_h264(self) -> cv.VideoCapture:
        """Sets up H.264 streaming using GStreamer.
        @returns object of cv2.VideoCapture"""
        pass

    def __setCVAttrs(self) -> None:
        """Sets CV attributes"""
        pass

    




