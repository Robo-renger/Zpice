#!/usr/bin/env python3
import rospy
from control.srv import GetConfig, SetConfig
from API.services.ConfigService import ConfigService


class ConfigServer:
    
    def __init__(self):
        rospy.init_node('configs_server')
        self.service = ConfigService()
        
    def config_service_server(self):
        s1 = rospy.Service('getConfigService', GetConfig, self.service.handleGetConfig)
        s2 = rospy.Service('setConfigService', SetConfig, self.service.handleSetConfig)
        rospy.loginfo("Config Service Server Ready")
        rospy.spin()

if __name__ == "__main__":
    ConfigServer().config_service_server()