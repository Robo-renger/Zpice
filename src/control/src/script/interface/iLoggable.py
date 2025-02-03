from zope.interface import Interface
from entities.Log import Log

class iLoggable(Interface):
    def logToFile() -> Log:
        """Logs the message to a file and returns a Log object."""

    def logToGUI() -> Log:
        """Logs the message to the GUI and returns a Log object."""