#!/usr/bin/env python3
import webcolors

class NeoPixelMock:
    def __init__(self, num_leds=16, brightness=1):
        """
        Initialize the WS2812 LED driver.

        :param pin: GPIO pin connected to the WS2812 data input.
        :param num_leds: Number of LEDs on the strip/board.
        :param brightness: Brightness level (0 to 255).
        """
        self.num_leds = num_leds
        self.brightness = brightness
        self.color = (0, 0, 0)  
        self.strip = [(0, 0, 0)] * num_leds  

    def __getitem__(self, led_index):
        """Stub to access an LED by index"""
        if 0 <= led_index < self.num_leds:
            return self.strip[led_index]
        raise IndexError("LED index out of range")

    def __setitem__(self, led_index, color):
        """Stub to set an LED color by index"""
        if 0 <= led_index < self.num_leds:
            self.strip[led_index] = color
        else:
            raise IndexError("LED index out of range")

    def fill(self, color: tuple):
        """Sets the color for all LEDs"""
        print(f"Color is set to: {color}")
        self.strip = [color] * self.num_leds

    def show(self):
        """Shows the color of the LEDs"""
        for i in range(self.num_leds):
            try:
                color_rgb = self.strip[i][::-1]
                color_hex = '#{:02X}{:02X}{:02X}'.format(*color_rgb)
                color_name = webcolors.hex_to_name(color_hex)
            except ValueError:
                requested_color = webcolors.hex_to_rgb(color_hex)
                color_name = self.__closest_color(requested_color)
        print(f"Color name: {color_name}")

    def __closest_color(self, requested_color: tuple) -> str:
        min_distance = float('inf')
        closest_name = None
        for hex_code, name in webcolors.CSS3_HEX_TO_NAMES.items():
            r, g, b = webcolors.hex_to_rgb(hex_code)
            distance = (requested_color[0] - r) ** 2 + (requested_color[1] - g) ** 2 + (requested_color[2] - b) ** 2
            if distance < min_distance:
                min_distance = distance
                closest_name = name
        return closest_name
