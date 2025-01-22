#!/usr/bin/env python3

from multiprocessing.shared_memory import SharedMemory
import pickle
import threading
from utils.Configurator import Configurator
import atexit
import signal
import time

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

            try:
                # Try to attach to an existing shared memory block
                self.shared_memory = SharedMemory(name=self.shared_memory_name)
            except FileNotFoundError:
                # If not found, create a new shared memory block
                self.shared_memory = SharedMemory(name=self.shared_memory_name, create=True, size=self.buffer_size)
                self.is_writer = True  # This instance will act as the writer

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

    def updateData(self, data):
        """
        Write the entire joystick data to shared memory (writer).
        
        Parameters:
            data (object): Joystick data (can be any serializable Python object, e.g., dictionary, object).
        """
        if not self.is_writer:
            raise PermissionError("Only the writer instance can update data.")

        with self.lock:
            data_with_timestamp = {
                "timestamp": time.time(),
                "data": data
            }
            serialized_data = self._serialize_data(data_with_timestamp)
            if len(serialized_data) > self.buffer_size:
                raise ValueError("Data exceeds shared memory buffer size.")
            self.shared_memory.buf[:len(serialized_data)] = serialized_data
            self.shared_memory.buf[len(serialized_data):] = b"\x00" * (self.buffer_size - len(serialized_data))

    def __getData(self):
        """
        Read the entire joystick data from shared memory (reader).
        Reconnect to shared memory if it becomes unavailable.
        
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
                    return data_with_timestamp["data"]
            except FileNotFoundError:
                print("Shared memory not found. Retrying...")
                time.sleep(1)
                retries -= 1
            except (pickle.UnpicklingError, EOFError, KeyError):
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
        """
        data = self.__getData()
        if data is None:
            return False  # No data available

        button_number = getattr(self, button_name, None)
        if button_number is None:
            raise ValueError(f"Button name '{button_name}' is not defined.")

        return getattr(data, f"button{button_number}", False)

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
            print(f"Error during cleanup: {e}")
