#!/usr/bin/env python3
import rospy
from control.msg import Joystick
from services.Joystick import CJoystick
import signal
class JoystickNode:
    def __init__(self):
        rospy.init_node('joystick_node', anonymous=True)
        self.joystick = CJoystick()
        signal.signal(signal.SIGINT, self.handle_exit)
        signal.signal(signal.SIGTERM, self.handle_exit)

    def handle_exit(self, signum, frame):
        rospy.loginfo(f"Received termination signal {signum}. Cleaning up shared memory.")
        self.joystick.cleanup()
        rospy.signal_shutdown("Node terminated.")  # Stop rospy.spin()
        
    def callback(self, data):
        # Update shared memory with the joystick data
        self.joystick.updateData(data)

    def run(self):
        rospy.Subscriber("/joystick", Joystick, self.callback)
        rospy.spin()

if __name__ == "__main__":
    node = JoystickNode()
    try:
        node.run()
    finally:
        node.joystick.cleanup()  # Explicit cleanup if needed
