#!/usr/bin/env python3
import rospy
from control.srv import GetLogsResponse
from pathlib import Path
import os
import json

class LogsService:
    def __init__(self):
        base_dir = Path(__file__).resolve().parent.parent.parent  
        self.logs_dir = base_dir / "logs"

    def handleGetLogs(self, req):
        file_name = f"{req.logName}.json"
        file_path = os.path.join(self.logs_dir, file_name)
        rospy.loginfo(file_path)
        try:
            with open(file_path, 'r') as file:
                logs_data = json.load(file)
            rospy.loginfo(f"Successfully loaded logs from file: {req.logName}")
            return GetLogsResponse(str(logs_data))
        except Exception as e:
            rospy.logerr(f"Failed to load logs from {file_path}: {str(e)}")
            return GetLogsResponse("")
        except json.JSONDecodeError:
            return GetLogsResponse("")
