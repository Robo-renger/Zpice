#!/usr/bin/env python3
import rospy
from zope.interface import implementer
from control.msg import Joystick
from interface.JoystickMock import IJoystickMock

@implementer(IJoystickMock)
class ManualJoystickMock():
    MIN = -1
    MAX = 1

    def __init__(self):
        self.pub = rospy.Publisher("/joystick", Joystick, queue_size=10)
        self.data = Joystick()
        self.axes_history = {'x': 0.0, 'y': 0.0, 'z': 0.0,
                             'yaw': 0.0, 'pitch': 0.0}

    def publish(self):
        self.pub.publish(self.data)

    def set_axis(self, axis: str, step: float = 0.05, dir: int = 1):
        """
        Set the value of the axis
        """
        try:
            axis_name = f"{axis}_axis"
            if (hasattr(self.data, axis_name)):
                prev_value = self.axes_history[axis]
                bounded_value = self._keepInBounds(prev_value + step * dir)
                setattr(self.data, axis_name, bounded_value)
            else:
                raise ValueError(f"{axis_name} not found")
            self.axes_history[axis] = bounded_value
        except Exception as e:
            rospy.logerr(e)
    
    def set_button(self, button: str, state: bool):
        """
        Set the state of the button
        """
        try:
            button_name = f"button{button}"
            if (hasattr(self.data, button_name)):
                setattr(self.data, button_name, state)
            else:
                raise ValueError(f"{button_name} not found")
        except Exception as e:
            rospy.logerr(e)

    def trigger_button(self, button_id):
        """
        Handle button press/release sequence with proper timing.
        """
        self.set_button(button_id, True)
        self.publish()
        rospy.sleep(0.1)
        self.set_button(button_id, False)
        self.publish()

    def _keepInBounds(self, value: float) -> float:
        """
        Clamp the value within the allowable range.
        """ 
        return max(ManualJoystickMock.MIN, min(value, ManualJoystickMock.MAX))