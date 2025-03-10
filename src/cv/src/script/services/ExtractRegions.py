#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import pickle

class ColorDetector:
    def __init__(self, img_path):
        self.img = cv.imread(img_path)
        self.hsv = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
        self.lower_blue=np.array([110,175,50])
        self.upper_blue = np.array([130, 255, 255])

        # Define two red ranges
        self.lower_red1 = np.array([0, 120, 100])
        self.upper_red1 = np.array([10, 255, 255])

        self.lower_red2 = np.array([170, 120, 100])
        self.upper_red2 = np.array([180, 255, 255])

        self.lower_orange = np.array([10, 100, 100])
        self.upper_orange = np.array([25, 255, 255])

        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([35, 255, 255])

        self.lower_purple = np.array([125, 50, 50])
        self.upper_purple = np.array([160, 255, 255])

    def create_mask(self, lower_bound, upper_bound):
        return cv.inRange(self.hsv, lower_bound, upper_bound)
    
class ImageHandler:
    def __init__(self, real_img_path):
        self.real_img = cv.imread(real_img_path)

    def show(self, window_name, image):
        cv.imshow(window_name, image)

class ContourHandler:
    def __init__(self):
        self.contours_list = []

    def find_contours(self, mask, color):
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        self.contours_list.append((contours, color))

    def save_contours(self, filename):
        with open(filename, 'wb') as f:
            pickle.dump(self.contours_list, f)

    def load_contours(self, filename):
        with open(filename, 'rb') as f:
            self.contours_list = pickle.load(f)

    def draw_contours(self, img,contours, color):
       
        cv.drawContours(img, contours, -1, color, thickness=-1)

class RegionLoader:
    def __init__(self,contour_handler):
        self.contour_handler = contour_handler

    def load_regions(self,filename):
        self.contour_handler.load_contours(filename)
    
    def get_region(self,region_num):
        return self.contour_handler.contours_list[region_num-1]


if __name__ == "__main__":
    color_detector = ColorDetector("coloredMap.png")
    contour_handler = ContourHandler()
    # Load the image where contours will be drawn
    image_handler = ImageHandler("Map.webp")
    region_loader = RegionLoader(contour_handler)
    

    # Create masks

    mask_blue=color_detector.create_mask(color_detector.lower_blue,color_detector.upper_blue)
    mask_red1 = color_detector.create_mask(color_detector.lower_red1, color_detector.upper_red1)
    mask_red2 = color_detector.create_mask(color_detector.lower_red2, color_detector.upper_red2)
    mask_orange = color_detector.create_mask(color_detector.lower_orange, color_detector.upper_orange)
    mask_yellow = color_detector.create_mask(color_detector.lower_yellow, color_detector.upper_yellow)
    mask_purple = color_detector.create_mask(color_detector.lower_purple, color_detector.upper_purple)

    # Combine masks for accurate red color 
    mask_red = mask_red1 | mask_red2

    # Find contours
    contour_handler.find_contours(mask_red, (0, 0, 255))
    contour_handler.find_contours(mask_yellow, (0, 255, 255))
    contour_handler.find_contours(mask_orange, (0, 165, 255))
    contour_handler.find_contours(mask_blue, (255, 0, 0))
    contour_handler.find_contours(mask_purple, (255, 0, 255))

    # Save contours
    contour_handler.save_contours("contours_all.pkl")

    # Load contours
    region_loader.load_regions("contours_all.pkl")

    # Get region
    region1 = region_loader.get_region(1)
    region2 = region_loader.get_region(2)
    region3 = region_loader.get_region(3)
    region4 = region_loader.get_region(4)
    region5 = region_loader.get_region(5)

    # Draw contours
    # Draw contours for region 1
    copy1=image_handler.real_img.copy()
    contour_handler.draw_contours(copy1, region1[0], region1[1])
    image_handler.show("Region 1", copy1)   
    # Draw contours for region 2
    copy2=image_handler.real_img.copy()
    contour_handler.draw_contours(copy2, region2[0], region2[1])
    image_handler.show("Region 2", copy2)
    # Draw contours for region 3
    copy3=image_handler.real_img.copy()
    contour_handler.draw_contours(copy3, region3[0], region3[1])
    image_handler.show("Region 3", copy3)
    # Draw contours for region 4
    copy4=image_handler.real_img.copy()
    contour_handler.draw_contours(copy4, region4[0], region4[1])
    image_handler.show("Region 4",copy4)
    # Draw contours for region 5
    copy5=image_handler.real_img.copy()
    contour_handler.draw_contours(copy5, region5[0], region5[1])
    image_handler.show("Region 5", copy5) 


    cv.waitKey(0)
    cv.destroyAllWindows()