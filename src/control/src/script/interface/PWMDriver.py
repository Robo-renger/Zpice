from zope.interface import Interface, implementer

class PWMDriver(Interface):
    def microsecondsToDutycycle(microseconds: int) -> int:
        """
        Converts a given pulse width in microseconds to a duty cycle value.

        :param microseconds: The pulse width in microseconds, typically required by the actuator.
        :return: The calculated duty cycle as an integer (0-65535).
        """

    def PWMWrite(channel: int, microseconds: int) -> None:
        """
        Sets the PWM signal on a specific channel based on the pulse width in microseconds.

        :param channel: The channel to set (0-15).
        :param microseconds: The pulse width in microseconds for the PWM signal.
        :raises ValueError: If the channel is out of range (0-15).
        """

    def stopAll() -> None:
        """
        Stops PWM output on all channels by setting their duty cycles to zero.
        """

    def close() -> None:
        """
        Releases any resources associated with the PWM driver, such as I2C connections.
        """
