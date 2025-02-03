import json
import os
import subprocess
from entities.Log import Log

class JsonFileHandler:
    def __init__(self, file_path: str = "logs/logs.json"):
        self.file_path = file_path

    def create(self):
        """Creates the JSON file if it doesn't already exist."""
        if not os.path.exists(self.file_path):
            with open(self.file_path, "w") as file:
                json.dump([], file)  # Initialize with an empty list

    def writeToFile(self, log: Log):
        """Appends an object to the JSON file."""
        self.create() 
        with open(self.file_path, "r+") as file:
            try:
                logs = json.load(file)  
            except json.JSONDecodeError:
                logs = []  

            logs.append(log.toDictionary())
            file.seek(0)  # Move to the beginning of the file
            json.dump(logs, file, indent=4)  # Write updated logs back to the file

    def readFromFile(self) -> list:
        """Reads and returns all logs from the JSON file."""
        self.create()
        with open(self.file_path, "r") as file:
            try:
                logs = json.load(file)  
            except json.JSONDecodeError:
                logs = []
        return logs

    def downloadFile(self, remote_host: str, remote_username: str, remote_password: str, remote_path: str, local_path: str = "downloaded_logs.json"):
        """
        Downloads the JSON file from a remote system to the local machine using SCP.
        Args:
            remote_host (str): The remote host (e.g., IP address or hostname).
            remote_username (str): The username for the remote system.
            remote_password (str): The password for the remote system.
            remote_path (str): The path to the JSON file on the remote system.
            local_path (str): The local path where the file will be saved.
        """
        try:
            # Construct the SCP command
            scp_command = [
                "scp",
                f"{remote_username}@{remote_host}:{remote_path}",
                local_path
            ]

            # Run the SCP command
            subprocess.run(scp_command, check=True)
            print(f"File downloaded to: {local_path}")
        except subprocess.CalledProcessError as e:
            print(f"Error downloading file: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")