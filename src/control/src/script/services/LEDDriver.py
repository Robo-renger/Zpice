#!/usr/bin/env python3

import neopixel 
import board
import time 

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
            self.strip[led_index] = color
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
        
        :param pos: Position on the color wheel (0-255).
        :return: A tuple (R, G, B) representing the color.
        """
        if pos < 0 or pos > 255:
            return (0, 0, 0)
        elif pos < 85:
            return (int(pos * 3), int(255 - pos * 3), 0)
        elif pos < 170:
            pos -= 85
            return (int(255 - pos * 3), 0, int(pos * 3))
        else:
            pos -= 170
            return (0, int(pos * 3), int(255 - pos * 3))
            
            
    def rainbowCycle(self, wait=0.1):
        """
        Animate a rainbow cycle across the LED strip.

        :param wait: Delay between updates (in seconds).
        """
        for j in range(255):
            for i in range(self.num_leds):
                pixel_index = (i * 256 // self.num_leds) + j
                self.strip[i] = self.wheel(pixel_index & 255)
            self.strip.show()
            time.sleep(wait)