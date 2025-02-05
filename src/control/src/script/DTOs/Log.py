from LogSeverity import LogSeverity
from datetime import datetime

class Log:
    def __init__(self, severity: LogSeverity, message: str, component_name: str):
        self.severity = severity
        self.message = message
        self.component_name = component_name
        self.timestamp = datetime.now()

    def toDictionary(self):
        """Converts the Log object to a dictionary for JSON serialization."""
        return {
            "severity": self.severity.value,
            "message": self.message,
            "component_name": self.component_name,
            "timestamp": self.timestamp.isoformat()
        }

    def __str__(self):
        return f"[{self.timestamp}] [{self.severity.value}] [{self.component_name}] {self.message}"