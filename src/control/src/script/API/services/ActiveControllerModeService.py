#!/usr/bin/env python3
from services.Joystick import CJoystick
from utils.Configurator import Configurator
from utils.LayoutManager import LayoutManager
from control.srv import  SwitchJoystickModeResponse, GetActiveControllerResponse

class ActiveControllerModeService:
    def __init__(self):
        self.joystick = CJoystick()
        self.configurator = Configurator()
        self.joystick_layout = LayoutManager().fetchLayout("controller")
        self.current_mode_index = 0
        self.type = "ps4" # Default controller type
        self.num_modes = len(self.joystick_layout[self.type]["modes"])
        
    def getType(self, req):
        self.type = req.activeController
        self.num_modes = len(self.joystick_layout[self.type]["modes"])
        return GetActiveControllerResponse("BONOOOO")

    def switchMode(self, req):
        self.current_mode_index = (self.current_mode_index + 1) % self.num_modes
        new_mode = self.joystick_layout[self.type]["modes"][self.current_mode_index]
        self.configurator.setConfig("joystick_buttons", new_mode)
        return SwitchJoystickModeResponse(str(self.current_mode_index))

   