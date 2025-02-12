#!/usr/bin/env python3
from utils.Configurator import Configurator
from zope.interface import implementer
from interface.iLoggable import iLoggable
from DTOs.Log import Log
from DTOs.LogSeverity import LogSeverity
from script.LogPublisherNode import LogPublisherNode
from helpers.JsonFileHandler import JsonFileHandler
from services.STM32 import STM32


@implementer(iLoggable)
class PWMFactory:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(PWMFactory, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, "_initialized"):  
            self.log_publisher = LogPublisherNode()
            self.json_file_handler = JsonFileHandler()
            self._initialized = True
            self.__PWMDriver = None
            for key, val in Configurator().fetchData(Configurator.CHANGEABLE_MODULES).items():
                if key == "PWM_DRIVER":
                    self.__PWMDriverType = val
            
    def getPWMDriver(self):
        if self.__PWMDriverType == "PCA":
            from services.PCADriver import PCA
            self.__PWMDriverType = PCA().getInst()
        elif self.__PWMDriverType == "STM32":
            #DO SMT32 REUIRED IMPORTS
            self.__PWMDriver = STM32(i2c_address=0x08)
            pass
        else:
            raise Exception(f"Unsupported PWMDriver module. Couldn't find {self.__PWMDriverType}")    
        return self.__PWMDriver
    
    def __logToFile(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.json_file_handler.writeToFile(log)
        return log
    
    def __logToGUI(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.log_publisher.publish(logSeverity.value, msg, component_name) 
        return log