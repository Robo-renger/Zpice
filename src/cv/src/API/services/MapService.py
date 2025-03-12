#!/usr/bin/env python3
import rospy
from services.Map import Map
from cv.srv import setMapResponse
from utils.JsonFileHandler import JSONFileHandler
import json

class MapService:
    def __init__(self):
        self.map = Map()
        self.json_handler = JSONFileHandler()

    def handleSetMap(self, req):
        rospy.loginfo("Recived Request to set map data")
        try:
            new_data = json.loads(req.mapData)
            self.json_handler.setData("map_data", new_data)
            video_path = self.map.run()
            return setMapResponse(video_path)

        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to parse mapdata JSON: {str(e)}")
            return setMapResponse("")

        except Exception as e:
            rospy.logerr(f"Failed to set map data: {str(e)}")
            return setMapResponse("")


    
    

