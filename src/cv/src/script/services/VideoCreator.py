#!/usr/bin/env python3
import cv2 as cv

class VideoCreator:
    def __init__(self,img_list):
        self.img_list = img_list
        self.__fourrc = cv.VideoWriter_fourcc(*'mp4v')
        try:
            self.frame_size = [self.img_list[0].shape[1], self.img_list[0].shape[0]]
        except IndexError:
            raise ValueError("Error: Image list is empty or images are not properly loaded")

    def create_video(self, filename):
        try:
            vid = cv.VideoWriter(filename, self.__fourrc, 1, self.frame_size)

            for img in self.img_list:
                for i in range(3):
                    vid.write(img)
            vid.release()
        except Exception as e:
            raise ValueError(f"Error creating video: {str(e)}")