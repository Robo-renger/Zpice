#!/usr/bin/env python3

import cv2
import os 
original_path = os.getcwd()
print(original_path)
os.chdir('/dev')
print("Changed directory to:", os.getcwd())
video0_path = os.path.join(os.getcwd(), 'video0')
cap = cv2.VideoCapture(video0_path, cv2.CAP_V4L2)
if cap.isOpened():
    try:
        cap.read()
        print("Camera opened successfully")
    except Exception as e:
        print(f"5ra: {e}")

else:
    print("Failed to open camera")
cap.release()

