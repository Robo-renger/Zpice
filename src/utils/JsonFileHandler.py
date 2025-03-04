#!/usr/bin/env python3
import rospkg
import json

class JSONFileHandler:
    CAMERAS = "cameras"
    CONTROLLER = "controller"  
    DEPTHFIXATION = "depth_fixation"
    ANGLEFIXATION = "angle_fixation" 
    def __init__(self):
        self.__jsonFile = ''

    def __raiseTypeError(self,data_type):
        consts = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("_")]
        raise TypeError(f"Layout file of type {data_type} doesn't exist, only {', '.join(consts)} are allowed.")
    
    def getFilesNames(self):
        consts = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("_")]
        return [getattr(self, attr) for attr in consts]

    def __getJSONFile(self, file_name):
        rospack = rospkg.RosPack()
        workspace_path = rospack.get_path('control')
        if file_name == JSONFileHandler.CAMERAS:
            self.__jsonFile = workspace_path + f'/../../layouts/{JSONFileHandler.CAMERAS}.json'
        elif file_name == JSONFileHandler.CONTROLLER:
            self.__jsonFile = workspace_path + f'/../../layouts/{JSONFileHandler.CONTROLLER}.json'
        elif file_name == JSONFileHandler.DEPTHFIXATION:
            self.__jsonFile = workspace_path + f'/../../fixations/{JSONFileHandler.DEPTHFIXATION}.json'
        elif file_name == JSONFileHandler.ANGLEFIXATION:
            self.__jsonFile = workspace_path + f'/../../fixations/{JSONFileHandler.ANGLEFIXATION}.json'
        else: 
            self.__raiseTypeError(file_name)
    
    def fetchData(self, file_name):
        try:
            self.__getJSONFile(file_name)
            with open(self.__jsonFile, 'r') as file:
                data = json.load(file)
                return data
        except FileNotFoundError:
            print(f"Error: The file '{self.__jsonFile}' was not found.")
        except json.JSONDecodeError:
            print("Error: Failed to parse the JSON file.")
        except TypeError as e:
            print(e)


    def setData(self, file_name, new_data):
        """
        Update the JSON layout file with new_layout.
        Only updates the keys provided in new_layout and keeps other keys intact.

        :param file_name: Type of configuration (e.g., "cameras", "controller")
        :param new_layout: Dictionary containing the new key-value pairs to update.
        """
        
        try:
            self.__getJSONFile(file_name)
            # Load existing data
            try:
                with open(self.__jsonFile, 'r') as file:
                    pass
                    existing_data = json.load(file) or {}  # Handle empty file case
            except FileNotFoundError:
                existing_data = {}
                print(f"Warning: {self.__jsonFile} not found. A new file will be created.")

            # Update existing data with new_layout (merge dictionaries)
            updated_data = {**existing_data, **new_data}

            # Write back to file
            with open(self.__jsonFile, 'w') as file:
                pass
                json.dump(updated_data, file, indent=4)

            print(f"Layout for '{file_name}' updated successfully.")

        except json.JSONDecodeError as e:
            print(f"Error: Failed to write JSON data. Details: {e}")
        except TypeError as e:
            print(e)

    def __updateYamlConfig(self, file_name, new_layout):
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