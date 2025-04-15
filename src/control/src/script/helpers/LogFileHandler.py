#!/usr/bin/env python3
import json
from pathlib import Path
from datetime import date
# import paramiko
from DTOs.Log import Log

class LogFileHandler:
    def __init__(self, file_path: str = None):
        # Get the absolute path to the logs directory relative to the script's location
        base_dir = Path(__file__).resolve().parent.parent  # Moves up to `src/control/src`
        self.log_dir = base_dir / "logs"
        self.log_dir.mkdir(parents=True, exist_ok=True)  # Ensure logs directory exists
        
        self.file_path = file_path if file_path else self.log_dir / f"{date.today().isoformat()}.json"

    def create(self):
        """Creates the JSON file if it doesn't already exist."""
        if not self.file_path.exists():
            with open(self.file_path, "w") as file:
                json.dump([], file)  # Initialize with an empty list

    def writeToFile(self, log: Log):
        """Appends an object to the JSON file."""
        self.create() 
        self.pruneLogs()
        with open(self.file_path, "r+") as file:
            try:
                logs = json.load(file)  
            except json.JSONDecodeError:
                logs = []  

            log_dict = log.toDictionary()
            if logs and "id" in logs[0]:
                new_id = max(item["id"] for item in logs) + 1
            else:
                new_id = len(logs)
            log_dict["id"] = new_id
            logs.append(log_dict)
            
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
    
    def pruneLogs(self):
        """Keep the last 7 logs and delete the rest"""
        log_files = sorted(self.log_dir.glob("*.json"))
        if len(log_files) > 7:
            files_to_delete = log_files[:len(log_files) - 7]
            for file in files_to_delete:
                try:
                    file.unlink()  
                except Exception as e:
                    print(f"Error deleting {file.name}: {e}")

    

    # def downloadFile(self, remote_host: str, remote_username: str, remote_password: str, remote_path: str, local_path: str = "downloaded_logs.json"):
        # try:
        #     remote_path = str(remote_path)
        #     local_path = str(local_path)

        #     print(f"Connecting to {remote_username}@{remote_host}...")
        #     # Create an SSH client
        #     ssh = paramiko.SSHClient()
        #     ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        #     # Connect to the remote system
        #     ssh.connect(remote_host, username=remote_username, password=remote_password)
        #     print("SSH connection established.")

        #     # Create an SFTP client
        #     sftp = ssh.open_sftp()
        #     print("SFTP session opened.")

        #     # Verify the remote file exists and is not empty
        #     remote_file_size = sftp.stat(remote_path).st_size
        #     print(f"Remote file size: {remote_file_size} bytes")

        #     if remote_file_size == 0:
        #         print("Warning: Remote file is empty.")
        #     else:
        #         # Download the file
        #         print(f"Downloading {remote_path} to {local_path}...")
        #         sftp.get(remote_path, local_path)
        #         print(f"File downloaded to: {local_path}")

        #     # Close the SFTP and SSH connections
        #     sftp.close()
        #     ssh.close()
        #     print("SFTP and SSH connections closed.")
        # except Exception as e:
        #     print(f"Error downloading file: {e}")