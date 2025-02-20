#!/usr/bin/env python3
import rospy
from services.Depth import DepthSensor
from exceptions.SensorInitializationError import SensorInitializationError
from exceptions.SensorReadError import SensorReadError
from control.msg import Depth
import time

class DepthNode:
    def __init__(self):
        rospy.init_node("depth_node", anonymous=False)
        self.sensor = self.initialize_sensor_with_retry()
        self.pub = rospy.Publisher('depth', Depth, queue_size=10)
        self.msg = Depth()

    def initialize_sensor_with_retry(self) -> DepthSensor:
        """
        Attempts to initialize the depth sensor with retries.

        Returns:
            DepthSensor: The initialized sensor object.
        Raises:
            SensorInitializationError: If the sensor fails to initialize after all retries.
        """
        max_attempts = 5
        for attempt in range(max_attempts):
            try:
                sensor = DepthSensor(3)
                return sensor
            except SensorInitializationError as e:
                if attempt < max_attempts - 1:
                    time.sleep(1)
                else:
                    raise SensorInitializationError("Depth sensor initialization failed after all retries.")

    def run(self):
        try:
            self.sensor.readData()
            self.msg.pressure = self.sensor.getPressure()
            self.msg.depth = self.sensor.getDepth()
            self.pub.publish(self.msg)
            # rospy.logerr(self.msg.pressure)
        except SensorReadError as e:
            rospy.logerr(f"{e} Skipping this cycle...")
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    try:
        node = DepthNode()
        while not rospy.is_shutdown():
            node.run()
    except SensorInitializationError as e:
        rospy.logerr(f"Sensor initialization failed. {e}")
    except KeyboardInterrupt:
        print("Exiting...")