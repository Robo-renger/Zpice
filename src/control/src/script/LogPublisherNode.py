#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class LogPublisherNode:
    def __init__(self):
        self.log_pub = rospy.Publisher('/logs', String, queue_size=10)

    def publish(self, severity: str, message: str, component_name: str) -> None:
        """
        Publishes a log to the /logs topic.
        """
        log_msg = f"[{severity}]: {message} => {component_name}"
        self.log_pub.publish(log_msg)
        # rospy.loginfo(f"Published log: {log_msg}")
