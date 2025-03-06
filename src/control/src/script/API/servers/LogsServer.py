#!/usr/bin/env python3
import rospy
from control.srv import GetLogs
from API.services.LogsService import LogsService

class LogsServer:
    
    def __init__(self):
        rospy.init_node('logs_server')
        self.service = LogsService()     
        
    def LogsServiceServer(self):
        s1 = rospy.Service('getLogsService', GetLogs, self.service.handleGetLogs)
        rospy.loginfo("Logs Service Server Ready")
        rospy.spin()
        
if __name__ == "__main__":
    LogsServer().LogsServiceServer()