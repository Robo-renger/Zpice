#!/usr/bin/env python3
import yaml
import rospkg
class Configurator():
    CAMERAS = "cameras"
    BUTTONS = "joystick_buttons"
    KEYBOARD_AXES = "keyboard_axes"
    KEYBOARD_BUTTONS = "keyboard_buttons"

    def __init__(self):
        self.__configFile = ''
    def __raiseTypeError(self,data_type):
        consts = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("_")]
        raise TypeError(f"Config file of type {data_type} doesn't exist, only {', '.join(consts)} are allowed.")

    def __getYamlFile(self,data_type):
        rospack = rospkg.RosPack()
        workspace_path = rospack.get_path('control') # CHANGE: when deploying or actual ws  
        if data_type == Configurator.CAMERAS:
            self.__configFile = workspace_path + f'/../../config/{Configurator.CAMERAS}.yaml'
        elif data_type == Configurator.BUTTONS:
            self.__configFile = workspace_path + f'/../../config/{Configurator.BUTTONS}.yaml'
        elif data_type == Configurator.KEYBOARD_AXES:
            self.__configFile = workspace_path + f'/../../config/{Configurator.KEYBOARD_AXES}.yaml'
        elif data_type == Configurator.KEYBOARD_BUTTONS:
            self.__configFile = workspace_path + f'/../../config/{Configurator.KEYBOARD_BUTTONS}.yaml'
        else: 
            self.__raiseTypeError(data_type)
    def fetchData(self,data_type):
        try:
            self.__getYamlFile(data_type)
            with open(self.__configFile, 'r') as file:
                data = yaml.safe_load(file)
                return data
        except FileNotFoundError:
            print(f"Error: The file '{self.__configFile}' was not found.")
        except yaml.YAMLError:
            print("Error: Failed to parse the YAML file.")
        except TypeError as e:
            print(e)
