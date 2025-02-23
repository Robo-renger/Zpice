#!/usr/bin/env python3
import time
import board
import neopixel
import threading

class LEDDriver:
    def __init__(self, pin=board.D18, num_leds=16, brightness=1):
        """
        Initialize the WS2812 LED driver.

        :param pin: GPIO pin connected to the WS2812 data input (default is GPIO 27).
        :param num_leds: Number of LEDs on the strip/board (default is 16).
        :param brightness: Brightness level (0 to 255, default is 255).
        """
        self.num_leds = num_leds
        self.brightness = brightness

        self.strip = neopixel.NeoPixel(pin, num_leds, brightness=brightness)
        self.strip.begin()

    def setColor(self, led_index, color):
        """
        Set the color of a specific LED.

        :param led_index: Index of the LED (0 to num_leds - 1).
        :param color: Tuple of (R, G, B) values (0-255).
        """
        
        if 0 <= led_index < self.num_leds:
            self.strip[led_index].fill(color)
            self.strip.show()
        else:
            raise ValueError("LED index out of range")

    def setAllColors(self, color):
        """
        Set the color of all LEDs.

        :param color: Tuple of (R, G, B) values (0-255).
        """
        for i in range(self.num_leds):
            self.strip.setPixelColor(i, color)
        self.strip.show()

    def clear(self):
        """
        Turn off all LEDs.
        """
        for i in range(self.num_leds):
            self.strip.setPixelColor(i, (0, 0, 0))
        self.strip.show()

    def setBrightness(self, brightness):
        """
        Set the brightness of all LEDs.

        :param brightness: Brightness level (0 to 255).
        """
        self.strip.setBrightness(brightness)
        self.strip.show()

    def blink(self, blink_times=5, on_time=0.5, off_time=0.5):
        """
        Blink all LEDs with a specified color.
        
        :param color: Tuple of (R, G, B) values (0-255).
        :param blink_times: Number of times to blink.
        :param on_time: Time in seconds LEDs stay on.
        :param off_time: Time in seconds LEDs stay off.
        """
        for _ in range(blink_times):
            self.setBrightness(1)
            time.sleep(on_time)
            self.setBrightness(0)
            time.sleep(off_time)

    def wheel(self, pos):
        """
        Generate an RGB color for a given position on a color wheel.
        
        The function accepts a value 'pos' in the range 0-255 and divides it into
        three segments (each ~85 steps long). In each segment, it scales the value by 3
        to cover the full 0-255 intensity range for one color channel.
        
        - For pos between 0 and 84:
            Red increases from 0 to 252, Green decreases from 255 to 3, Blue is 0.
        - For pos between 85 and 169:
            Red decreases from 255 to 3, Blue increases from 0 to 252, Green is 0.
        - For pos between 170 and 255:
            Blue decreases from 255 to 3, Green increases from 0 to 252, Red is 0.
        
        :param pos: Position on the color wheel (0-255).
        :return: A tuple (R, G, B) representing the color.
        """
        if pos < 0 or pos > 255:
            return (0, 0, 0)
        if pos < 85:
            return (int(pos * 3), int(255 - pos * 3), 0)
        elif pos < 170:
            pos -= 85
            return (int(255 - pos * 3), 0, int(pos * 3))
        else:
            pos -= 170
            return (0, int(pos * 3), int(255 - pos * 3))
        
    def rainbowCycle(self, wait=0.1, iterations=1):
        """
        Animate a rainbow cycle across the LED strip.
        
        This method generates colors for each LED based on
        its index and a shifting offset (j). This creates a flowing rainbow effect.
        
        :param wait: Delay between updates (in seconds).
        :param iterations: Number of full cycles through the color wheel.
        """
        for j in range(256 * iterations):
            with self.lock:
                for i in range(self.num_leds):
                    # Calculate color index by combining the LED position and the current offset
                    color_index = (int(i * 256 / self.num_leds) + j) & 255
                    color = self.wheel(color_index)
                    self.strip.setPixelColor(i, color)
                self.strip.show()
            time.sleep(wait)

    