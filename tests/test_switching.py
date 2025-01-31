import unittest
from unittest.mock import MagicMock
from Switching import Switching

class TestSwitching(unittest.TestCase):
    def setUp(self):
        self.mock_gpio = MagicMock()
        self.switching = Switching(pin=17, gpio_interface=self.mock_gpio)

    def test_initial_state(self):
        self.mock_gpio.setmode.assert_called_once_with(self.mock_gpio.BCM)
        self.mock_gpio.setup.assert_called_once_with(17, self.mock_gpio.OUT)
        self.mock_gpio.output.assert_called_once_with(17, self.mock_gpio.LOW)
        self.assertFalse(self.switching.is_open)

    def test_open(self):
        self.switching.open()
        self.mock_gpio.output.assert_called_with(17, self.mock_gpio.HIGH)
        self.assertTrue(self.switching.is_open)

    def test_close(self):
        self.switching.close()
        self.mock_gpio.output.assert_called_with(17, self.mock_gpio.LOW)
        self.assertFalse(self.switching.is_open)

    def test_toggle(self):
        self.switching.toggle()
        self.mock_gpio.output.assert_called_with(17, self.mock_gpio.HIGH)
        self.assertTrue(self.switching.is_open)
        self.switching.toggle()
        self.mock_gpio.output.assert_called_with(17, self.mock_gpio.LOW)
        self.assertFalse(self.switching.is_open)

if __name__ == '__main__':
    unittest.main()
