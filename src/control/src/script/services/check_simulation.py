#!/usr/bin/env python3

            
def check_simulation_mode():
    try:
        import RPi.GPIO as GPIO 
        print("RPi.GPIO module available. Running in hardware mode.")
        return False # Hardware mode
    except (RuntimeError, ImportError):
        print("RPi.GPIO module not available. Running in simulation mode.")
        return True # Simulation mode
        
