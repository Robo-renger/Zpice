#!/usr/bin/env python3

import rospy
from control.msg import IMU, Depth
from std_msgs.msg import String, Float32MultiArray, Float32
from services.Joystick import CJoystick
from services.Navigation import Navigation
from services.Vectorizer import Vectorizer
from services.PIDController import PIDController
from utils.Configurator import Configurator
import time

class NavigationNode:
    def __init__(self):
        self.joystick = CJoystick()
        self.x = 0
        self.y = 0
        self.z = 0
        self.pitch = 0
        self.yaw = 0
        self.last_reset_time = 0  
        
        self.pub = rospy.Publisher("Direction", String, queue_size=10)

        self.PID_configs = Configurator().fetchData(Configurator.PID_PARAMS)        
        self.pid_yaw   = PIDController(self.PID_configs['yaw_KP'], self.PID_configs['yaw_KI'], self.PID_configs['yaw_KD'])
        self.pid_pitch = PIDController(self.PID_configs['pitch_KP'], self.PID_configs['pitch_KI'], self.PID_configs['pitch_KD'])
        self.pid_pitch_live = PIDController(self.PID_configs['live_pitch_KP'], self.PID_configs['live_pitch_KI'], self.PID_configs['live_pitch_KD'])
        self.pid_heave = PIDController(self.PID_configs['heave_KP'], self.PID_configs['heave_KI'], self.PID_configs['heave_KD'])
        self.pid_heave_live = PIDController(self.PID_configs['live_heave_KP'], self.PID_configs['live_heave_KI'], self.PID_configs['live_heave_KD'])
        self.pid_yaw.isHeading = True
        
        self.imu_data = {
            'pitch': None,
            'yaw' : None
        }
        self.depth = None

        self.activePID = True
        self.fix_heading = False
        self.fix_tilting = False
        self.fix_heave = False
        self.is_rotating = False
        
        # self.kp_factor = 0.5
        # self.ki_factor = 0.0
        # self.kd_factor = 2.5
        # self.kp_factor = 100.0
        # self.ki_factor = 0.0
        # self.kd_factor = 1.0
        
        rospy.Subscriber("IMU", IMU, self._imuCallback)
        rospy.Subscriber("depth", Depth, self._depthCallback)
        rospy.Subscriber("constants", Float32MultiArray, self.constantsCallback)
        rospy.Subscriber("factor", Float32MultiArray, self.factorCallback)
        rospy.Subscriber("setpoint", Float32, self.setpointCallback)
        # rospy.Subscriber("set_target", setTarget, self._setTargetCallback)
        
    def reload(self):
        self.PID_configs = Configurator().fetchData(Configurator.PID_PARAMS)
        self.pid_yaw   = PIDController(self.PID_configs['yaw_KP'], self.PID_configs['yaw_KI'], self.PID_configs['yaw_KD'])
        self.pid_pitch = PIDController(self.PID_configs['pitch_KP'], self.PID_configs['pitch_KI'], self.PID_configs['pitch_KD'])
        self.pid_pitch_live = PIDController(self.PID_configs['live_pitch_KP'], self.PID_configs['live_pitch_KI'], self.PID_configs['live_pitch_KD'])
        self.pid_heave = PIDController(self.PID_configs['heave_KP'], self.PID_configs['heave_KI'], self.PID_configs['heave_KD'])
        self.pid_heave_live = PIDController(self.PID_configs['live_heave_KP'], self.PID_configs['live_heave_KI'], self.PID_configs['live_heave_KD'])
        
    def _imuCallback(self, msg: IMU):
        self.imu_data['pitch'] = msg.pitch
        self.imu_data['yaw'] = msg.yaw

    def _depthCallback(self, msg: Depth):
        self.depth = msg.depth

    def _setTargetCallback(self, msg):
        if msg.type == "heading":
            self.fix_heading = True
            if msg.reached:
                self.fix_heading = False        
            if not msg.reached:
                self.pid_yaw.updateSetpoint(msg.target)

        elif msg.type == "tilting":
            self.fix_tilting = True
            if msg.reached:
                self.fix_tilting = False        
            if not msg.reached:
                self.pid_pitch.updateSetpoint(msg.target)

        elif msg.type == "heave":
            self.fix_heave = True
            if msg.reached:
                self.fix_heave = False
            if not msg.reached:
                self.pid_heave.updateSetpoint(msg.target)

        elif msg.type == "rotate":
            self.is_rotating = True
            if msg.reached:
                self.is_rotating = False
            if not msg.reached:
                self.pid_yaw.updateSetpoint(self.depth)

    def constantsCallback(self, msg):
        rospy.logwarn(msg.data)
        self.pid_yaw.updateConstants(msg.data[0], msg.data[1], msg.data[2])
        self.pid_pitch_live.updateConstants(msg.data[3], msg.data[4], msg.data[5])
        self.pid_heave_live.updateConstants(msg.data[6], msg.data[7], msg.data[8])

    def setpointCallback(self, msg):
        if -180 <= msg.data <= 180:
            rospy.logwarn(msg.data)
            self.pid_yaw.updateSetpoint(msg.data)
            # self.pid_pitch.updateSetpoint(self.imu_data['pitch'])
            # self.pid_pitch_live.updateSetpoint(self.imu_data['pitch'])
            # self.pid_heave.updateSetpoint(self.depth)
            # self.pid_heave_live.updateSetpoint(self.depth)
    
    def factorCallback(self, msg):
        if msg.data is not None:
            rospy.logwarn(msg.data)
            self.kp_factor = msg.data[0]
            self.ki_factor = msg.data[1]
            self.kd_factor = msg.data[2]
            
    def _resetFlags(self):
        self.fix_heading = False
        self.fix_tilting = False
        self.fix_heave = False
        self.is_rotating = False

    def handleJoystickInput(self):
        current_time = time.time()
        axis_values = self.joystick.getAxis()
        self.x = -1 * axis_values.get('left_x_axis', 0)
        self.y = axis_values.get('left_y_axis', 0)
        self.pitch = -1 * axis_values.get('right_y_axis', 0)
        self.yaw = -1 * axis_values.get('right_x_axis', 0)

        if self.joystick.isPressed("HEAVE_DOWN") and self.joystick.isPressed("HEAVE_UP"):
            self.z = 0.0
            self.last_reset_time = current_time 
        elif current_time - self.last_reset_time > 0.2:  
            if self.joystick.isPressed("HEAVE_UP"):
                self.z = min(self.z + 0.07, 1)
            elif self.joystick.isPressed("HEAVE_DOWN"):
                self.z = max(self.z - 0.07, -1)

    def stabilizeAtRest(self):
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        pitch_output = self.pid_pitch.stabilize(self.imu_data['pitch'])
        heave_output = self.pid_heave.stabilize(self.depth)
        Navigation.navigate(0, 0, pitch_output, heave_output, yaw_output)

    def stabilizeHorizontal(self):
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        pitch_output = self.pid_pitch_live.stabilize(self.imu_data['pitch'])
        heave_output = self.pid_heave_live.stabilize(self.depth)
        Navigation.navigate(self.x, self.y, pitch_output, heave_output, yaw_output)

    def stabilizeVertical(self):
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        pitch_output = self.pid_pitch_live.stabilize(self.imu_data['pitch'])
        Navigation.navigate(self.x, self.y, pitch_output, self.z, yaw_output)

    def setHeading(self):
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        Navigation.navigate(self.x, self.y, -self.pitch, self.z, yaw_output)
    
    def setTilting(self):
        pitch_output = self.pid_pitch.stabilize(self.imu_data['pitch'])
        Navigation.navigate(self.x, self.y, pitch_output, self.z, self.yaw)

    def setDepth(self):
        heave_output = self.pid_heave.stabilize(self.depth)
        Navigation.navigate(self.x, self.y, -self.pitch, heave_output, self.yaw)

    def rotate(self):
        heave_output = self.pid_heave.stabilize(self.depth)
        Navigation.navigate(self.x, self.y, 0.5, heave_output, 0.5)
    
    def extractDir(self):
        dir = None
        if self.z > 0.3:
            dir = "Down"
        elif self.z < -0.3:
            dir = "Up"
        elif self.pitch > 0.3:
            dir = "PitchDown"
        elif self.pitch < -0.3:
            dir = "PitchUp"
        elif self.yaw > 0.3:
            dir = "YawRight"
        elif self.yaw < -0.3:
            dir = "YawLeft"
        elif self.x > 0.3:
            dir = "Left"
        elif self.x < -0.3:
            dir = "Right"
        elif self.y > 0.3:
            dir = "Backward"
        elif self.y < -0.3:
            dir = "Forward"
        else:
            dir = "Rest"
        
        if dir:
            self.pub.publish(dir)  # Publish to the ROS topic

    def _isRest(self):
        return (abs(self.x) <= 0.05 and abs(self.y) <= 0.05 and 
                abs(self.z) <= 0.05 and
                abs(self.pitch) <= 0.05 and abs(self.yaw) <= 0.05)
    
    def _isRestYawAxis(self):
        return (abs(self.yaw) <= 0.05)
    
    def _isYawControllerReady(self):
        return (self.imu_data['yaw'] is not None and self.pid_yaw.setpoint is not None)
    
    def _isRestPitchAxis(self):
        return (abs(self.pitch) <= 0.05)
    
    def _isPitchControllerReady(self):
        return (self.imu_data['pitch'] is not None and self.pid_pitch.setpoint is not None and self.pid_pitch_live.setpoint is not None)
    
    def _isRestHeaveAxis(self):
        return (abs(self.z) <= 0.05)
    
    def _isHeaveControllerReady(self):
        return (self.depth is not None and self.pid_heave.setpoint is not None and self.pid_heave_live.setpoint is not None)

    def _isRestAxes(self):
        return (self._isRestYawAxis() and self._isRestPitchAxis() and self._isRestHeaveAxis())
    
    def _isReadyControllers(self):
        return (self._isYawControllerReady() and self._isPitchControllerReady() and self._isHeaveControllerReady())

    def _isMovingHorizontally(self):
        return (abs(self.x) > 0.05 or abs(self.y) > 0.05)

    def _isMovingVertically(self):
        return (abs(self.z) > 0.05)
    
    def _isFixing(self):
        return (self.fix_heading or self.fix_tilting or self.fix_heave or self.is_rotating)

    def navigate(self):    
        self.handleJoystickInput()
        
        if not self._isRestYawAxis() and not self._isFixing():
            self.pid_yaw.updateSetpoint(self.imu_data['yaw'])

        if self._isRestPitchAxis() and not self._isFixing():
            self.pid_pitch.updateSetpoint(-3.76)
            self.pid_pitch_live.updateSetpoint(-3.76)
        else:
            if not self._isFixing():
                self.pid_pitch.updateSetpoint(self.imu_data['pitch'])
                self.pid_pitch_live.updateSetpoint(self.imu_data['pitch'])
        
        if not self._isRestHeaveAxis() and not self._isFixing():
            self.pid_heave.updateSetpoint(self.depth)
            self.pid_heave_live.updateSetpoint(self.depth)
        
        if self._isRest() and not self._isFixing():
            if self._isReadyControllers():
                rospy.logwarn("ROV at rest: Stabilizing Heading, Tilting and Depth")
                self.stabilizeAtRest()
        
        elif self.activePID and self._isMovingHorizontally() and self._isRestAxes() and not self._isFixing():
            if self._isReadyControllers():
                rospy.loginfo("ROV moving Horizontally: Stabilizing Heading, Tilting and Depth")
                self.stabilizeHorizontal()

        elif self.activePID and self._isMovingVertically() and (self._isRestYawAxis() and self._isRestPitchAxis()) and not self._isFixing():
            if self._isYawControllerReady() and self._isPitchControllerReady():
                rospy.loginfo("ROV moving Vertically: Stabilizing Heading and Tilting")
                self.stabilizeVertical()

        elif self.is_rotating:
            if self._isHeaveControllerReady():
                # rospy.logwarn("ROV rotating: Stabilizing Depth")
                self.rotate()
        
        elif self.fix_heading:
            if self._isYawControllerReady():
                # rospy.loginfo("Setting Heading")
                self.setHeading()

        elif self.fix_tilting:
            if self._isPitchControllerReady():
                # rospy.loginfo("Setting Tilting")
                self.setTilting()

        elif self.fix_heave:
            if self._isHeaveControllerReady():
                # rospy.loginfo("Setting Depth")
                self.setDepth()

        else:
            # rospy.logerr("ROV in manual control: Using joystick input")
            Navigation.navigate(self.x, self.y, -self.pitch, self.z, self.yaw * 0.65)

        self.extractDir()
        # rospy.logerr(f"YAW: {self.imu_data['yaw']}")

if __name__ == "__main__":
    Vectorizer.yaw_only = False
    rospy.init_node("navigation_node")
    try:
        node = NavigationNode()
        while not rospy.is_shutdown():
            node.navigate()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Navigation Node...")
    finally:
        Navigation.stopAll()
        rospy.logwarn("Stopped all motors")