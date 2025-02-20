#!/usr/bin/env python3

from smbus2 import SMBus
from zope.interface import implementer
from interface.PWMDriver import PWMDriver

@implementer(PWMDriver)
class STM32:
    def __init__(self, i2c_address, bus_number=1):
        """
        Initializes the STM32 I2C communication class.

        :param i2c_address: I2C address of the STM32 device
        :param bus_number: I2C bus number (default is 1 for Raspberry Pi)
        """
        self.i2c_address = i2c_address
        self.bus_number = bus_number
        self.bus = SMBus(bus_number)
        self.channels = {}  # Stores {channel: microseconds} values

    def __sendMessage(self, message):
        """
        Sends a formatted message to the STM32 over I2C.

        :param message: A string message like "0-1500"
        """
        if not isinstance(message, str):
            raise ValueError("Message must be a string")

        # Convert the message into a list of bytes
        byte_data = [ord(char) for char in message]

        # Send the message as a block of data
        self.bus.write_i2c_block_data(self.i2c_address, 0x00, byte_data)  # 0x00 is the register
        print(f"Sent: {message}")

    def PWMWrite(self, channel, microseconds):
        """
        Sends a PWM signal to a specific channel and updates the channel dictionary.

        :param channel: PWM channel number
        :param microseconds: Pulse width in microseconds
        """
        self.channels[channel] = microseconds  # Update the channel dictionary
        # formatted_message = f"{channel:02d}-{microseconds}"
        self.__sendMessage(f"{channel}-{microseconds}")

    def stop_all(self):
        """
        Stops all PWM signals by sending '0' microseconds to all channels.
        """
        for channel in self.channels.keys():
            self.__sendMessage(f"{channel}-0")
            self.channels[channel] = 0  # Update dictionary to reflect stopped channels
        print("All channels stopped.")

    def close(self):
        """Closes the I2C bus connection."""
        self.bus.close()

# # Example Usage --> in Node
# if __name__ == "__main__":
#     stm32 = STM32(i2c_address=0x20)  # Replace with your STM32's I2C address
#     stm32.PWMWrite(1, 1500)  # Set channel 1 to 1500 microseconds
#     stm32.PWMWrite(2, 1200)  # Set channel 2 to 1200 microseconds
    
#     print(stm32.channels)  # Check the stored values
    
#     stm32.stop_all()  # Stop all channels
#     print(stm32.channels)  # Check that all values are now 0

#     stm32.close()
