# JoystickMock

Mocking the joystick is the ability to achieve its functionalities without having the joystick itself through software.

The mock is split into two main ones:
1) **Manual**: simulating the actual joystick itself by keyboard presses.
2) **Autonomous**: randomizing the values of the axes and button presses which is used to test every possible scenario when testing any component.

## 1) Manual 

This method simulates the following:
1) Axes increment and decrement 
2) Button presses 

### 1.1) Scripts Clarification
1) `JoystickMock`: has the interface of the mock where every method is declared.
2) `Keyboard`: singelton class responsible for getting the keyboard pressed keys 
3) `ManualJoystick`: has the concrete implementation of the interface and responsible for publishing the updated key values to **/joystick** topic which is used by all other components used by joystick.
4) `ManualJoystickNode`: this node is responsible for fetching **Buttons** and **axes** configuration using **configurator** class and update the **keys**.

## 2) Autonomous 

This method randomizes the values of the axes and creates random presses.

### 2.1) Scripts Clarification
1) `JoystickMock`: has the interface of the mock where every method is declared.
3) `AutonomousJoystick`: has the concrete implementation of the interface and responsible for publishing the updated key values to **/joystick** topic which is used by all other components used by joystick.
4) `AutonomousJoystickNode`: this node is responsible for randomizing the values and publishing them.


## 3) Testing:

1) For manual mode run `ManualJoystickNode` and for autonomous mode run `AutonomousJoystickNode`
2) echo `rostopic echo /joystick`
3) for manual mode check `keyboard_axes` and `keyboard_buttons` for the configuration. 
