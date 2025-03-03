#!/usr/bin/env python3
import yaml
import rospkg
class Configurator():
    CAMERAS = "cameras"
    BUTTONS = "joystick_buttons"
    KEYBOARD_AXES = "keyboard_axes"
    KEYBOARD_BUTTONS = "keyboard_buttons"
    CHANGEABLE_MODULES = "changeable_modules"
    PINS = "hardware_pins"
    def __init__(self):
        self.__configFile = ''
    def __raiseTypeError(self,data_type):
        consts = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("_")]
        raise TypeError(f"Config file of type {data_type} doesn't exist, only {', '.join(consts)} are allowed.")
    
    def getConfigsNames(self):
        consts = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("_")]
        return [getattr(self, attr) for attr in consts]

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
        elif data_type == Configurator.CHANGEABLE_MODULES:
            self.__configFile = workspace_path + f'/../../config/{Configurator.CHANGEABLE_MODULES}.yaml'
        elif data_type == Configurator.PINS:
            self.__configFile = workspace_path + f'/../../config/{Configurator.PINS}.yaml'
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
    def setConfig(self, data_type, new_data):
        """
        Update the YAML configuration file with new_data.
        Only updates the keys provided in new_data and keeps other keys intact.

        :param data_type: Type of configuration (e.g., "cameras", "joystick_buttons")
        :param new_data: Dictionary containing the new key-value pairs to update.
        """
        try:
            self.__getYamlFile(data_type)

            # Load existing data
            try:
                with open(self.__configFile, 'r') as file:
                    existing_data = yaml.safe_load(file) or {}  # Handle empty file case
            except FileNotFoundError:
                existing_data = {}
                print(f"Warning: {self.__configFile} not found. A new file will be created.")

            # Update existing data with new_data (merge dictionaries)
            updated_data = {**existing_data, **new_data}

            # Write back to file
            with open(self.__configFile, 'w') as file:
                yaml.safe_dump(updated_data, file, default_flow_style=False)

            print(f"Configuration for '{data_type}' updated successfully.")

        except yaml.YAMLError as e:
            print(f"Error: Failed to write YAML data. Details: {e}")
        except TypeError as e:
            print(e)