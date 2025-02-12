#!/usr/bin/env python3

from multiprocessing.shared_memory import SharedMemory
import pickle
import threading
from utils.Configurator import Configurator
import atexit
import signal
import time
from zope.interface import implementer
from interface.iLoggable import iLoggable
from DTOs.Log import Log
from DTOs.LogSeverity import LogSeverity
from helpers.JsonFileHandler import JsonFileHandler
from script.LogPublisherNode import LogPublisherNode

@implementer(iLoggable)
class CJoystick:
    _instance = None
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:  # Ensure a single instance
            cls._instance = super(CJoystick, cls).__new__(cls)
        return cls._instance  # Always return the same instance

    def __init__(self, is_writer=False,shared_memory_name="joystick_data", buffer_size=1024):
        if not hasattr(self, "_initialized"):  # Ensure __init__ runs only once
            self._initialized = True
            self.shared_memory_name = shared_memory_name
            self.buffer_size = buffer_size
            self.is_writer = is_writer  # Distinguish between writer and reader
            self.lock = threading.Lock()
            self.json_file_handler = JsonFileHandler()
            self.log_publisher = LogPublisherNode()

            try:
                # Try to attach to an existing shared memory block
                self.shared_memory = SharedMemory(name=self.shared_memory_name)
                self.logToFile(LogSeverity.INFO, "Shared memory attached successfully.", "CJoystick")
                self.logToGUI(LogSeverity.INFO, "Shared memory attached successfully.", "CJoystick")
            except FileNotFoundError:
                # If not found, create a new shared memory block
                self.shared_memory = SharedMemory(name=self.shared_memory_name, create=True, size=self.buffer_size)
                self.is_writer = True  # This instance will act as the writer
                self.logToFile(LogSeverity.INFO, "Shared memory created successfully.", "CJoystick")
                self.logToGUI(LogSeverity.INFO, "Shared memory created successfully.", "CJoystick")

            atexit.register(self.cleanup)
            signal.signal(signal.SIGINT, self._signal_cleanup)
            signal.signal(signal.SIGTERM, self._signal_cleanup)
            self.__constructConstants()

    @classmethod
    def __constructConstants(cls):
        """
        Dynamically create class constants from the configuration file.
        This ensures that button mappings are automatically assigned as class attributes.

        Example Config:
        {
            "button_0": "FLASH",
            "button_x": "LEFTGRIPPER_OPEN",
            "button_l1": "LEFTGRIPPER_CLOSE",
            "button_4": "RIGHTGRIPPER_OPEN"
        }
        """
        joystickButtons = Configurator().fetchData(Configurator.BUTTONS)
        if joystickButtons:
            for button_key, button_name in joystickButtons.items():
                if button_key.startswith("button"):  # Ensure valid button key format
                    try:
                        button_number = button_key.replace("button", "")
                        print("ana ahooo: ")
                        print(button_number)
                        setattr(cls, button_name, button_number)
                        setattr(cls, f"_{button_number}", button_number)  # Add support for _X button notation
                    except ValueError:
                        continue  # Skip invalid keys


    def _serialize_data(self, data):
        """
        Serialize data using pickle.
        """
        return pickle.dumps(data)

    def _deserialize_data(self, buffer):
        """
        Deserialize data using pickle.
        """
        return pickle.loads(buffer.rstrip(b"\x00"))  # Remove padding

    def updateData(self, buttons_data, axis_data):
        """
        Write the joystick data (buttons and axes) to shared memory (writer).
        
        Parameters:
            buttons_data (object): Button data (can be any serializable object like a dictionary).
            axis_data (list): List containing joystick axis values [left_x, left_y, right_x, right_y].
        """
        if not self.is_writer:
            # self.logToFile(LogSeverity.ERROR, "Only the writer instance can update data.", "CJoystick")
            # self.logToGUI(LogSeverity.ERROR, "Only the writer instance can update data.", "CJoystick")
            raise PermissionError("Only the writer instance can update data.")

        try:
            with self.lock:
                data_with_timestamp = {
                    "timestamp": time.time(),
                    "buttons": buttons_data,
                    "axes": axis_data
                }
                serialized_data = self._serialize_data(data_with_timestamp)

                if len(serialized_data) > self.buffer_size:
                    raise ValueError("Data exceeds shared memory buffer size.")

                self.shared_memory.buf[:len(serialized_data)] = serialized_data
                self.shared_memory.buf[len(serialized_data):] = b"\x00" * (self.buffer_size - len(serialized_data))
        except Exception as e:
            self.logToFile(LogSeverity.ERROR, f"Shared memory update failed: {e}", "CJoystick")
            self.logToGUI(LogSeverity.ERROR, f"Shared memory update failed: {e}", "CJoystick")
            self.is_writer = False  # Fallback to prevent invalid writes


    def __getData(self):
        """
        Read the joystick data from shared memory (reader).
        
        Returns:
            object: The deserialized joystick data, or None if invalid or unavailable.
        """
        retries = 5
        while retries > 0:
            try:
                if not hasattr(self, "shared_memory") or self.shared_memory is None:
                    self.shared_memory = SharedMemory(name=self.shared_memory_name)

                with self.lock:
                    data_with_timestamp = self._deserialize_data(bytes(self.shared_memory.buf))
                    return data_with_timestamp
            except FileNotFoundError:
                self.logToFile(LogSeverity.ERROR, "Shared memory not found. Retrying...", "CJoystick")
                self.logToGUI(LogSeverity.ERROR, "Shared memory not found. Retrying...", "CJoystick")
                print("Shared memory not found. Retrying...")
                time.sleep(1)
                retries -= 1
            except (pickle.UnpicklingError, EOFError, KeyError):
                self.logToFile(LogSeverity.ERROR, "Failed to deserialize data.", "CJoystick")
                self.logToGUI(LogSeverity.ERROR, "Failed to deserialize data.", "CJoystick")
                return None

        print("Failed to connect to shared memory after retries.")
        return None

    def isClicked(self, button_name):
        """
        Check whether a specific button is clicked.

        Parameters:
            button_name (str): The name of the button constant (e.g., LEFTGRIPPER_OPEN, _1).

        Returns:
            bool: True if the button is clicked, False otherwise.

        Raises:
            ValueError: If the button name is not defined.
        """
        data = self.__getData()
        if data is None or "buttons" not in data:
            return False  # No data available

        # Handle numbered buttons prefixed with "_"
        if button_name.startswith("_"):
            try:
                button_number = int(button_name[1:])  # Extract the number
            except ValueError:
                self.logToFile(LogSeverity.ERROR, f"Invalid button number format: '{button_name}'", "CJoystick")
                self.logToGUI(LogSeverity.ERROR, f"Invalid button number format: '{button_name}'", "CJoystick")
                raise ValueError(f"Invalid button number format: '{button_name}'")
        else:
            # Get button number from class attributes (e.g., LEFTGRIPPER_OPEN → 1)
            button_number = getattr(self, button_name, None)
        if button_number is None:
            self.logToFile(LogSeverity.ERROR, f"Button name '{button_name}' is not defined.", "CJoystick")
            self.logToGUI(LogSeverity.ERROR, f"Button name '{button_name}' is not defined.", "CJoystick")
            raise ValueError(f"Button name '{button_name}' is not defined.")
        return data["buttons"].get(f"button{button_number}", False)


    def getAxis(self):
        """
        Returns an array of joystick axis values.
        
        Returns:
            dictionary: Dictionary of joystick axis values {left_x_axis: 0.0, left_y_axis: 0.0, right_x_axis: 0.0, right_y_axis: 0.0}.
        """
        data = self.__getData()
        if data is None or "axes" not in data:
            return {"left_x_axis": 0.0, "left_y_axis": 0.0, "right_x_axis": 0.0, "right_y_axis": 0.0}  # Default values if no data available
        return data["axes"]

    def _signal_cleanup(self, signum, frame):
        """
        Perform cleanup when a termination signal is received.
        """
        print(f"Received termination signal {signum}. Cleaning up shared memory.")
        self.cleanup()

    def cleanup(self):
        """
        Clean up shared memory (only the writer should unlink it).
        """
        if self.is_writer:
            self.shared_memory.unlink()
        self.shared_memory.close()
    
    
    def logToFile(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.json_file_handler.writeToFile(log)
        return log
    
    def logToGUI(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.log_publisher.publish(logSeverity.value, msg, component_name) 
        return log