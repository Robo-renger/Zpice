from zope.interface import Interface

class iSwitching(Interface):
    
    """
    Interface for switching devices.
    Will implement gpio logic.
    """
    
    def open(self) -> None:
        """
        Open the device.
        """
    def close(self) -> None:
        """
        Close the device.
        """
    def toggle(self) -> None:
        """
        Toggle the device.
        """
    def is_open(self) -> bool:
        """
        Return True if the device is open.
        """
    def is_closed(self) -> bool:
        """
        Return True if the device is closed.
        """