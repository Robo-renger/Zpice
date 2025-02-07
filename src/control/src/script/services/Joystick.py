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
from nodes.LogPublisherNode import LogPublisherNode

@implementer(iLoggable)
class CJoystick:
    _instance = None
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(CJoystick, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self, shared_memory_name="joystick_data", buffer_size=1024):
        if not hasattr(self, "_initialized"):  # Ensure __init__ runs only once
            self._initialized = True
            self.shared_memory_name = shared_memory_name
            self.buffer_size = buffer_size
            self.is_writer = False  # Distinguish between writer and reader
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
        """
        joystickButtons = Configurator().fetchData(Configurator.BUTTONS)
        if joystickButtons:
            for button, name in joystickButtons.items():
                button_number = int(button.replace("button", ""))
                setattr(cls, name, button_number)

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
            self.logToFile(LogSeverity.ERROR, "Only the writer instance can update data.", "CJoystick")
            self.logToGUI(LogSeverity.ERROR, "Only the writer instance can update data.", "CJoystick")
            raise PermissionError("Only the writer instance can update data.")

        with self.lock:
            data_with_timestamp = {
                "timestamp": time.time(),
                "buttons": buttons_data,
                "axes": axis_data
            }
            serialized_data = self._serialize_data(data_with_timestamp)
            if len(serialized_data) > self.buffer_size:
                self.logToFile(LogSeverity.ERROR, "Data exceeds shared memory buffer size.", "CJoystick")
                self.logToGUI(LogSeverity.ERROR, "Data exceeds shared memory buffer size.", "CJoystick")
                raise ValueError("Data exceeds shared memory buffer size.")
            self.shared_memory.buf[:len(serialized_data)] = serialized_data
            self.shared_memory.buf[len(serialized_data):] = b"\x00" * (self.buffer_size - len(serialized_data))

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
            button_name (str): The name of the button constant (e.g., LEFTGRIPPER).
        
        Returns:
            bool: True if the button is clicked, False otherwise.

        Raises:
            ValueError: If the button name is not defined
        """
        data = self.__getData()
        if data is None or "buttons" not in data:
            return False  # No data available

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
            list: List of joystick axis values [left_x, left_y, right_x, right_y].
        """
        data = self.__getData()
        if data is None or "axes" not in data:
            return [0, 0, 0, 0]  # Default values if no data available
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
        try:
            if self.is_writer:
                self.shared_memory.unlink()
            self.shared_memory.close()
        except Exception as e:
            self.logToFile(LogSeverity.ERROR, f"Error during cleanup: {e}", "CJoystick")
            self.logToGUI(LogSeverity.ERROR, f"Error during cleanup: {e}", "CJoystick")
            print(f"Error during cleanup: {e}")
    
    
    
    def logToFile(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.json_file_handler.writeToFile(log)
        return log
    
    def logToGUI(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.log_publisher.publish(logSeverity.value, msg, component_name) 
        return log