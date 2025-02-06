from zope.interface import Interface
from DTOs.Log import Log
from DTOs.LogSeverity import LogSeverity

class iLoggable(Interface):
    def logToFile(logSeverity: LogSeverity, message: str, component_name: str) -> Log:
        """Logs the message to a file and returns a Log object."""

    def logToGUI(logSeverity: LogSeverity, message: str, component_name: str) -> Log:
        """Logs the message to the GUI and returns a Log object."""