#!/usr/bin/env python3
import rospy
from control.srv import  GetConfigResponse, SetConfigResponse
from utils.Configurator import Configurator
import json
class ConfigService:
    def __init__(self):
        self.configurator = Configurator()
        self.configNames = self.configurator.getConfigsNames()
        pass

    def handleGetConfig(self, req):
        rospy.loginfo(f"Received GetConfig request: {req.configName}")

        if req.configName not in self.configNames:
            return GetConfigResponse(f"{req.configName} is not available. Available files are: {', '.join(self.configNames)}")

        config_data = json.dumps(self.configurator.fetchData(req.configName))
        return GetConfigResponse(config_data)


    def handleSetConfig(self, req):
        rospy.loginfo(f"Received SetConfig request for: {req.configName}")

        try:
            # Parse configObject from JSON string
            new_data = json.loads(req.configObject)

            # Update the YAML file using the setConfig() method from Configurator
            self.configurator.setConfig(req.configName, new_data)
            return SetConfigResponse(True)

        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to parse configObject JSON: {str(e)}")
            return SetConfigResponse(False)

        except Exception as e:
            rospy.logerr(f"Failed to set config: {str(e)}")
            return SetConfigResponse(False)