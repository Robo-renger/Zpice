#!/usr/bin/env python3
import rospy
from utils.Configurator import Configurator
from utils.LayoutManager import LayoutManager
from control.srv import SwitchJoystickModeResponse
from API.clients.ActiveControllerClient import ActiveController

class ControllerModeService:
    def __init__(self):
        self.configurator = Configurator()
        self.joystick_layout = LayoutManager().fetchLayout("controller")
        self.current_mode_index = 0
        self.active_controller = ActiveController().getController()
        self.num_modes = len(self.joystick_layout[self.active_controller]["modes"])

    def switchMode(self, req):
        rospy.loginfo(f"active controller: {self.active_controller}")
        self.current_mode_index = (self.current_mode_index + 1) % self.num_modes
        new_mode = self.joystick_layout[self.active_controller]["modes"][self.current_mode_index]
        self.configurator.setConfig("joystick_buttons", new_mode)
        return SwitchJoystickModeResponse(str(self.current_mode_index))

   