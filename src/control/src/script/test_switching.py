#!/usr/bin/env python3

import time
from Switching import Switching

def test_switching():

    pin = 17  
    
   
    switching = Switching(pin)
    
    
    print("Opening the switch")
    switching.open()
    assert switching.is_open == True, "The switch should be open"
    time.sleep(2)  
    
    
    print("Closing the switch")
    switching.close()
    assert switching.is_open == False, "The switch should be closed"
    time.sleep(2)  
    
   
    print("Toggling the switch")
    switching.toggle()
    assert switching.is_open == True, "The switch should be open after toggle"
    time.sleep(2)
    
    switching.toggle()
    assert switching.is_open == False, "The switch should be closed after second toggle"
    time.sleep(2)
    
    print("All tests passed successfully!")

if __name__ == "__main__":
    test_switching()
