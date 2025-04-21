#!/usr/bin/env python3
import rospy
import cv2 as cv 
from utils.EnvParams import EnvParams
from utils.Configurator import Configurator
from gui.srv import screenshotsResponse
class ScreenshotsService:
    def __init__(self):
        self.address = EnvParams().WEB_DOMAIN
        self.configurator = Configurator()
        self.camerasDetails = self.__getCameraSteamDetails()
        self.left_url = f"http://{self.address}:{self.camerasDetails['left_stereo']['port']}/stream.mjpg"
        self.right_url = f"http://{self.address}:{self.camerasDetails['right_stereo']['port']}/stream.mjpg"
        self.index = 0
        

    def __getCameraSteamDetails(self):
        return self.configurator.fetchData(Configurator.CAMERAS)
    
    def handleGetScreenshots(self, req):
        rospy.loginfo("Recived Request to capture screenshots")
        try:
            right_output_path = f"/var/www/html/calibrationScreenshots/right/right{self.index}.jpg"
            left_output_path = f"/var/www/html/calibrationScreenshots/left/left{self.index}.jpg"
            left_cap = cv.VideoCapture(self.left_url)
            right_cap = cv.VideoCapture(self.right_url)
            left_ret, left_frame = left_cap.read()
            right_ret, right_frame = right_cap.read()
            cv.imwrite(left_output_path, left_frame)
            cv.imwrite(right_output_path, right_frame)
            left_cap.release()
            right_cap.release()
            self.index += 1
            return screenshotsResponse("True")

        except Exception as e:
            rospy.logerr(f"Failed to set map data: {str(e)}")
            return screenshotsResponse("False")


    
    

