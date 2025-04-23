#!/usr/bin/env python3
from services.ImageHandler import ImageHandler
from services.RegionLoader import RegionLoader
from services.VideoCreator import VideoCreator
from services.DataReader import DataReader
from utils.EnvParams import EnvParams
from pathlib import Path
import rospkg

class Map:
    def __init__(self):
        rospack = rospkg.RosPack()
        workspace_path = rospack.get_path('cv')
        self.base_dir = Path(__file__).resolve().parent.parent
        self.DATA_FILE = workspace_path + f'/../../mapData/map_data.json' 
        self.CONTOURS_FILE = self.base_dir / "data" / "contours_all.pkl"
        self.MAP_FILE = self.base_dir / "data" / "Map.webp"
        self.OUTPUT_VID_DIR="/var/www/html/mapMission/output.mp4"

    def run(self):
        try:
            image_handler = ImageHandler(self.MAP_FILE, RegionLoader(self.CONTOURS_FILE), DataReader(self.DATA_FILE))
            video_creator = VideoCreator(image_handler.draw_images())
            video_creator.create_video(self.OUTPUT_VID_DIR)
            return f"http://{EnvParams().WEB_DOMAIN}/mapMission/output.mp4"
        except FileNotFoundError as e:
            print(f"File not found: {str(e)}")
        except ValueError as e:
            print(f"Value error: {str(e)}")
        except Exception as e:
            print(f"Unexpected error in map node: {str(e)}")


    

