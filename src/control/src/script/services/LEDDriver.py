#!/usr/bin/env python3

import neopixel 
import board

class LEDDriver:
    def __init__(self, pin=board.D18, num_leds=16, brightness=1):
        # print("Rowan BTNAWARRRRR")
        """
        Initialize the WS2812 LED driver.

        :param pin: GPIO pin connected to the WS2812 data input (default is GPIO 27).
        :param num_leds: Number of LEDs on the strip/board (default is 16).
        :param brightness: Brightness level (0 to 255, default is 255).
        """
        self.num_leds = num_leds
        self.brightness = brightness

        self.strip = neopixel.NeoPixel(pin, num_leds, brightness=brightness)
        # print("alo")
        self.strip.fill((255, 255, 255))
        self.strip.show()

    def setColor(self, led_index, color):
        """
        Set the color of a specific LED.

        :param led_index: Index of the LED (0 to num_leds - 1).
        :param color: Tuple of (R, G, B) values (0-255).
        """
        if 0 <= led_index < self.num_leds:
            self.strip[led_index]
            self.strip.show()
        else:
            raise ValueError("LED index out of range")

    def setAllColors(self, color):
        """
        Set the color of all LEDs.

        :param color: Tuple of (R, G, B) values (0-255).
        """
        self.strip.fill(color)
        self.strip.show()

    def clear(self):
        """
        Turn off all LEDs.
        """
        self.strip.fill((0, 0, 0))
        self.strip.show()

    def setBrightness(self, brightness):
        """
        Set the brightness of all LEDs.

        :param brightness: Brightness level (0 to 255).
        """
        self.strip.brightness = brightness