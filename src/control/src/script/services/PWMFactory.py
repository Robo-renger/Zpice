#!/usr/bin/env python3
from utils.Configurator import Configurator
from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity
<<<<<<< HEAD
from services.STM32 import STM32
=======
>>>>>>> Logs

class PWMFactory:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(PWMFactory, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, "_initialized"):  
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
            Logger.logToFile(LogSeverity.ERROR, f"Unsupported PWMDriver module. couldn't find {self.__PWMDriverType}.", "PWMFactory")
            Logger.logToGUI(LogSeverity.ERROR, f"Unsupported PWMDriver module. couldn't find {self.__PWMDriverType}.", "PWMFactory")
            raise Exception(f"Unsupported PWMDriver module. Couldn't find {self.__PWMDriverType}")    
        return self.__PWMDriver