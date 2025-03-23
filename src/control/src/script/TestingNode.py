#!/usr/bin/env python3

import rospy

class TestingNode:
    def __init__(self):
        rospy.init_node('testing_node', anonymous=False)

    def run(self):
        pass



if __name__ == "__main__":
    try:
        testingNode = TestingNode()
        while not rospy.is_shutdown():
            testingNode.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")
