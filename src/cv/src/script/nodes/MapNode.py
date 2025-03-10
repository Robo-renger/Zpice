#!/usr/bin/env python3
import rospy
from services.ImageHandler import ImageHandler
from services.RegionLoader import RegionLoader
from services.VideoCreator import VideoCreator
from services.DataReader import DataReader

class MapNode:
    MAP_DIR="../data/Map.webp"
    CONTOURS_DIR="../data/contours_all.pkl"
    DATA_DIR="../data/data.csv"
    OUTPUT_VID_DIR="../output/output.mp4"

    def __init__(self):
        rospy.init_node('map_node')

    def run(self):
        try:
            image_handler = ImageHandler(MapNode.MAP_DIR, RegionLoader(MapNode.CONTOURS_DIR), DataReader(MapNode.DATA_DIR))
            video_creator = VideoCreator(image_handler.draw_images())
            video_creator.create_video(MapNode.OUTPUT_VID_DIR)
        except FileNotFoundError as e:
            print(f"File not found: {str(e)}")
        except ValueError as e:
            print(f"Value error: {str(e)}")
        except Exception as e:
            print(f"Unexpected error in map node: {str(e)}")
    
if __name__=="__main__":
    try:
        node = MapNode()
        node.run()
    except rospy.ROSException as e:
        rospy.logerr(f"Error in map node: {e}")
    
