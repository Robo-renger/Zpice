#!/usr/bin/env python3
from mock.LEDDriverMock import NeoPixelMock

class NeoPixelMockTest:
    def __init__(self):
        self.lib = NeoPixelMock()

    def setColorTest(self, color: tuple, led_index: int):
        self.lib[led_index]
        self.lib.fill(color)
        print(self.lib[led_index])

    def showTest(self, color: tuple):
        self.lib.fill(color)
        self.lib.show()


if __name__ == "__main__":
    try:
        led_driver = NeoPixelMockTest()
        # led_driver.setColorTest((255, 0, 0), 5)
        led_driver.showTest((255, 0, 255))
    except Exception as e:
        print(f"Error: {e}")