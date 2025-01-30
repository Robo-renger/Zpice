# Servo 
This is a walkthrough the system of the servo

# 1. Servo360 
The 360 servo (mostly the one currently in the ROV) is a contious servo, meaning it works like the DC motor moving either forward or backward without any feedback, where it takes a microseconds ranging from (1000 - 2000us) and 1500us is the stop.

Controling this kind of servos is done through applying the smallest speed possible **assuming 1505us** for a direction and **1495us** for the other direction, then a small delay to indicate the motion period, then stop it from moving. **Values are based on test and trail.**

## 1.1 Scripts Clarification

1) `Servo360Interface`: Has the interface of the servo where every method is declared.

2) `Servo360`: Has the concrete implementation of interface.

3) `Servo360Test`: Has unit testing class for testing whether the values being sent by the servo class are identical to the ones the pwm driver.

4) `PCAMock`: Mock class used to test the servo without the need for actual hardware.

5) `Servo360HardwareTest`: This Script will be used to test the actual hardware it self, providing a structred menu for changing direction, stopping and also the delay values to be used.

# 2. Servo180
The 180 servo or the regular one which takes pulses or steps and then moving in angles, when the provided pulse increases, the servo increases its angle and vice versa. 

To operate it takes microseconds ranging from (1000 - 2000us) where:

1) us = 2000 --> Servo goes to angle 180
2) us = 1500 --> Servo goes to angle 90 
3) us = 1000 --> Servo goes to angle 0 

## 2.1 Scripts Clarification

1) `Servo180Interface`: Has the interface of the servo where every method is declared.

2) `Servo180`: Has the concrete implementation of interface.

3) `Servo180Test`: Has unit testing class for testing whether the values being sent by the servo class are identical to the ones the pwm driver and whether the pulse gets updated or not.

4) `PCAMock`: Mock class used to test the servo without the need for actual hardware.

5) `Servo180HardwareTest`: This Script will be used to test the actual hardware it self, asking for the step the servo needs to take