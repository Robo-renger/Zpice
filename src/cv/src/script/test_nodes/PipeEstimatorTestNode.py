#!/usr/bin/env python3
import rospy
from services.LengthEstimator import LengthEstimator

class PipeEstimatorTestNode:
    def __init__(self):
        rospy.init_node('pipe_estimator_test_node')
        self.estimator = LengthEstimator()

    def run(self):
        length = self.estimator.estimateLength(
            (754, 79),
            (427, 58),
            (945, 91),
            (633, 64)
        )
        rospy.loginfo(f"length {length}")

if __name__ == "__main__":
    try:
        test_node = PipeEstimatorTestNode()
        test_node.run()
    except KeyboardInterrupt:
        rospy.logwarn("Exiting...")
    except rospy.ROSException as e:
        rospy.logerr(f"Erros in pipe estimator test node: {e}")
