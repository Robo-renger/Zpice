#!/usr/bin/env python3
import rospy
from control.srv import  GetLayoutResponse, SetLayoutResponse
from utils.LayoutManager import LayoutManager
import json
class LayoutService:
    def __init__(self):
        self.LayoutManager = LayoutManager()
        self.LayoutNames = self.LayoutManager.getLayoutsNames()
        pass

    def handleGetLayout(self, req):
        rospy.loginfo(f"Received GetLayout request: {req.layoutName}")

        if req.layoutName not in self.LayoutNames:
            return GetLayoutResponse(f"{req.layoutName} is not available. Available files are: {', '.join(self.LayoutNames)}")

        Layout_data = str(self.LayoutManager.fetchLayout(req.layoutName))
        return GetLayoutResponse(Layout_data)


    def handleSetLayout(self, req):
        rospy.loginfo(f"Received SetLayout request for: {req.layoutName}")

        try:
            # Parse LayoutObject from JSON string
            new_data = json.loads(req.layoutObject)

            # # Update the JSON file using the setLayout() method from LayoutManager
            self.LayoutManager.setLayout(req.layoutName, new_data)
            return SetLayoutResponse(True)

        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to parse LayoutObject JSON: {str(e)}")
            return SetLayoutResponse(False)

        except Exception as e:
            rospy.logerr(f"Failed to set Layout: {str(e)}")
            return SetLayoutResponse(False)