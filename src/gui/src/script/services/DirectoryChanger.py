#!/usr/bin/env python3

import os
import rospy
import cv2

def capGetter():
    # Change the current working directory to /dev
    try:
        os.chdir('/dev')
        rospy.loginfo(f"Successfully changed working directory to {os.getcwd()}")
        try:
            cap = cv2.VideoCapture("/dev/rapoo_camera", cv2.CAP_V4L2)
            if cap.isOpened():
                try:
                    cap.read()
                    print("Camera opened successfully")
                except Exception as e:
                    print(f"5ra: {e}")

            else:
                print("Failed to open camera")
            cap.release()
        except Exception as e:
            print(f"Path Error: {e}")
    except Exception as e:
        rospy.logerr(f"Failed to change directory to /dev: {e}")

    # Your main node logic here
    rospy.loginfo("Node is running...")
    return cap

capGetter()