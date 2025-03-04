#!/usr/bin/env python3
import rospkg
import json

class LayoutManager:
    CAMERAS = "cameras"
    CONTROLLER = "controller"   
    def __init__(self):
        self.__layoutFile = ''

    def __raiseTypeError(self,data_type):
        consts = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("_")]
        raise TypeError(f"Layout file of type {data_type} doesn't exist, only {', '.join(consts)} are allowed.")
    
    def getLayoutsNames(self):
        consts = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("_")]
        return [getattr(self, attr) for attr in consts]

    def __getLayoutFile(self, layout_name):
        rospack = rospkg.RosPack()
        workspace_path = rospack.get_path('control')
        if layout_name == LayoutManager.CAMERAS:
            self.__layoutFile = workspace_path + f'/../../layouts/{LayoutManager.CAMERAS}.json'
        elif layout_name == LayoutManager.CONTROLLER:
            self.__layoutFile = workspace_path + f'/../../layouts/{LayoutManager.CONTROLLER}.json'
        else: 
            self.__raiseTypeError(layout_name)
    
    def fetchLayout(self, layout_name):
        try:
            self.__getLayoutFile(layout_name)
            with open(self.__layoutFile, 'r') as file:
                data = json.load(file)
                return data
        except FileNotFoundError:
            print(f"Error: The file '{self.__layoutFile}' was not found.")
        except json.JSONDecodeError:
            print("Error: Failed to parse the JSON file.")
        except TypeError as e:
            print(e)


    def setLayout(self, layout_name, new_layout):
        """
        Update the JSON layout file with new_layout.
        Only updates the keys provided in new_layout and keeps other keys intact.

        :param layout_name: Type of configuration (e.g., "cameras", "controller")
        :param new_layout: Dictionary containing the new key-value pairs to update.
        """
        
        try:
            self.__getLayoutFile(layout_name)
            # Load existing data
            try:
                with open(self.__layoutFile, 'r') as file:
                    pass
                    existing_data = json.load(file) or {}  # Handle empty file case
            except FileNotFoundError:
                existing_data = {}
                print(f"Warning: {self.__layoutFile} not found. A new file will be created.")

            # Update existing data with new_layout (merge dictionaries)
            updated_data = {**existing_data, **new_layout}

            # Write back to file
            with open(self.__layoutFile, 'w') as file:
                pass
                json.dump(updated_data, file, indent=4)

            print(f"Layout for '{layout_name}' updated successfully.")

        except json.JSONDecodeError as e:
            print(f"Error: Failed to write JSON data. Details: {e}")
        except TypeError as e:
            print(e)

    def __updateYamlConfig(self, layout_name, new_layout):
        """Update the yaml file with the required data
        UNDER CONSTRUCTION!!!!"""
        pass

    def __convertToYaml(self, data):
        """Convert the JSON data to YAML format
        UNDER CONSTRUCTION!!!!"""
        pass

    def __convertToJson(self, data):
        """Convert the YAML data to JSON format
        UNDER CONSTRUCTION!!!!"""