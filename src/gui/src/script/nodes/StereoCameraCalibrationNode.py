#!/usr/bin/env python3

import rospy
from utils.Configurator import Configurator
from services.StereoCalibrator import StereoCalibrator

class StereoCalibrationNode:
    def __init__(self, chessboard_size: tuple):
        rospy.init_node('stereo_calibration', anonymous=False)
        self.configurator = Configurator()
        self.stereo_details = self.__getCameraSteamDetails()['stereo']
        self.chessboard_size = chessboard_size
        self.calibrator = StereoCalibrator(self.stereo_details, self.chessboard_size)

    def __getCameraSteamDetails(self):
        return self.configurator.fetchData(Configurator.CAMERAS)
    
    def run(self):
        # self.calibrator.captureCalibrationImages()
        # self.calibrator.calibrate() ## Could be done outside of the raspberry
        self.calibrator.testCalibration()

if __name__ == "__main__":
    try:
        node = StereoCalibrationNode((9, 7))
        node.run()
    except Exception as e:
        rospy.logerr(f"Error occured {e}")
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")


# # Get current directory
# path = os.getcwd()

# # Create folders if they don't exist
# os.makedirs("left", exist_ok=True)
# os.makedirs("right", exist_ok=True)

# # Initialize camera
# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640 * 2)  # Set width for stereo image
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Set height

# if not cap.isOpened():
#     print("Error: Could not open camera.")
#     exit()

# frame_count = 0  # Counter for naming images
# chessboard_size = (9, 7)  # Define chessboard pattern size

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("Failed to grab frame")
#         continue

#     # Get frame dimensions
#     height, width, _ = frame.shape
#     half_width = width // 2

#     # Split the frame into left and right halves
#     left_frame = frame[:, :half_width].copy()
#     right_frame = frame[:, half_width:].copy()

#     # Convert to grayscale for corner detection
#     grayL = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
#     grayR = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)

#     # Find chessboard corners
#     retL, cornersL = cv2.findChessboardCorners(grayL, chessboard_size, None)
#     retR, cornersR = cv2.findChessboardCorners(grayR, chessboard_size, None)

#     # Draw detected corners on preview windows
#     previewL = left_frame.copy()
#     previewR = right_frame.copy()
#     if retL:
#         cv2.drawChessboardCorners(previewL, chessboard_size, cornersL, retL)
#     if retR:
#         cv2.drawChessboardCorners(previewR, chessboard_size, cornersR, retR)

#     # Show the frames with corners
#     cv2.imshow('Left', previewL)
#     cv2.imshow('Right', previewR)

#     key = cv2.waitKey(1) & 0xFF

#     # Press 's' to save original images (without drawn corners)
#     if key == ord('s'):
#         left_path = f"left/left_{frame_count}.png"
#         right_path = f"right/right_{frame_count}.png"
#         cv2.imwrite(left_path, left_frame)
#         cv2.imwrite(right_path, right_frame)
#         print(f"Saved: {left_path}, {right_path}")
#         frame_count += 1

#     # Press 'q' to exit
#     if key == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()
