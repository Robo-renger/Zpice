from zope.interface import Interface
from entities.Log import Log
from entities.LogSeverity import LogSeverity

class iLoggable(Interface):
    def logToFile(logSeverity: LogSeverity.value, msg: str, component_name: str) -> Log:
        """Logs the message to a file and returns a Log object."""

    def logToGUI() -> Log:
        """Logs the message to the GUI and returns a Log object."""