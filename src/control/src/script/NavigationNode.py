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
        
        
        #Speeds for vertical motors, when full up thrust is required (scaling up)
        self.upThrustVerticalValueMax = 1900 
        self.upThrustVerticalValueMin = 1100
        
        #Speeds for horizontal motors, when full up thrust is required (scaling doww)
        self.upThrustHorizontalValueMax = 1650 
        self.upThrustHorizontalValueMin = 1100
        
        self.pub = rospy.Publisher("Direction", String, queue_size=10)

        self.PID_configs = Configurator().fetchData(Configurator.PID_PARAMS)        
        self.pid_yaw   = PIDController(self.PID_configs['yaw_KP'], self.PID_configs['yaw_KI'], self.PID_configs['yaw_KD'])
        self.pid_pitch = PIDController(self.PID_configs['pitch_KP'], self.PID_configs['pitch_KI'], self.PID_configs['pitch_KD'])
        self.pid_heave = PIDController(self.PID_configs['heave_KP'], self.PID_configs['heave_KI'], self.PID_configs['heave_KD'])
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
        self.kp_factor = 0.5
        self.ki_factor = 0.0
        self.kd_factor = 2.5
        
        rospy.Subscriber("IMU", IMU, self._imuCallback)
        rospy.Subscriber("depth", Depth, self._depthCallback)
        rospy.Subscriber("constants", Float32MultiArray, self.constantsCallback)
        rospy.Subscriber("setpoint", Float32, self.setpointCallback)
        rospy.Subscriber("factor", Float32MultiArray, self.factorCallback)
        # rospy.Subscriber("set", String, self._setFixationCallback)
        
    def reload(self):
        self.PID_configs = Configurator().fetchData(Configurator.PID_PARAMS)
        self.pid_yaw   = PIDController(self.PID_configs['yaw_KP'], self.PID_configs['yaw_KI'], self.PID_configs['yaw_KD'])
        self.pid_pitch = PIDController(self.PID_configs['pitch_KP'], self.PID_configs['pitch_KI'], self.PID_configs['pitch_KD'])
        self.pid_heave = PIDController(self.PID_configs['heave_KP'], self.PID_configs['heave_KI'], self.PID_configs['heave_KD'])
        
    def _imuCallback(self, msg: IMU):
        self.imu_data['pitch'] = msg.pitch
        self.imu_data['yaw'] = msg.yaw

    def _depthCallback(self, msg: Depth):
        self.depth = msg.depth

    def _setFixationCallback(self, msg: String):
        if msg.data == "heading":
            self.fix_heading = True
            self.fix_tilting = False
            self.fix_heave = False
            self.pid_yaw.updateSetpoint(self.imu_data['yaw'])
        elif msg.data == "heave":
            self.fix_heave = True
            self.fix_tilting = False
            self.fix_heading = False
            self.pid_heave.updateSetpoint(self.depth)
        elif msg.data == "both":
            self.fix_heading = True
            self.fix_heave = True
            self.fix_tilting = True
            self.pid_yaw.updateSetpoint(self.imu_data['yaw'])
            self.pid_heave.updateSetpoint(self.imu_data['pitch'])
            self.pid_heave.updateSetpoint(self.depth)
        else:
            self._resetFlags()

    def constantsCallback(self, msg):
        rospy.logwarn(msg.data)
        self.pid_yaw.updateConstants(msg.data[0], msg.data[1], msg.data[2])
        self.pid_pitch.updateConstants(msg.data[3], msg.data[4], msg.data[5])

    def setpointCallback(self, msg):
        if -180 <= msg.data <= 180:
            rospy.logwarn(msg.data)
            self.pid_yaw.updateSetpoint(msg.data)
            # self.pid_pitch.updateSetpoint(self.imu_data['pitch'])
            # self.pid_heave.updateSetpoint(self.depth)
    
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

        # if abs(self.pitch) >= 0.5:
        #     self.yaw = 0
        # else:
        #     self.pitch = 0

        # if (self.x != 0 or self.y != 0 or self.z != 0 or self.pitch != 0 or self.yaw != 0) and not (self.fix_heading or self.fix_heave):
        #     self.pid_yaw.updateSetpoint(self.imu_data['yaw'])
        #     self.pid_pitch.updateSetpoint(self.imu_data['pitch'])

    def stabilizeAtRest(self):
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        # self.pid_pitch.updateConstants(self.pid_pitch.kp, self.pid_pitch.ki, self.pid_pitch.kd)
        # pitch_output = self.pid_pitch.stabilize(self.imu_data['pitch'])
        Navigation.navigate(0, 0, 0, 0, yaw_output)

    def stabilizeHorizontal(self):
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        self.pid_pitch.updateConstants(self.pid_pitch.kp, self.pid_pitch.ki, self.pid_pitch.kd)
        pitch_output = self.pid_pitch.stabilize(self.imu_data['pitch'])
        Navigation.navigate(self.x, self.y, -pitch_output, 0, yaw_output)

    def stabilizeVertical(self):
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        self.pid_pitch.updateActiveConstants(self.kp_factor, self.ki_factor, self.kd_factor)
        pitch_output = self.pid_pitch.stabilize(self.imu_data['pitch'])
        Navigation.navigate(0, 0, -pitch_output, self.z, yaw_output)

    def stabilizeLively(self):
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        self.pid_pitch.updateActiveConstants(self.kp_factor, self.ki_factor, self.kd_factor)
        pitch_output = self.pid_pitch.stabilize(self.imu_data['pitch'])
        Navigation.navigate(self.x, self.y, -pitch_output, self.z, yaw_output)

    def fixHeading(self):
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        Navigation.navigate(0, 0, self.pitch, self.z, yaw_output)
    
    def fixTilting(self):
        pitch_output = self.pid_pitch.stabilize(self.imu_data['pitch'])
        Navigation.navigate(self.x, self.y, -pitch_output, 0, self.yaw)

    def fixHeave(self):
        depth_output = self.pid_heave.stabilize(self.depth)
        Navigation.navigate(self.x, self.y, 0, depth_output, self.yaw)
    
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
        if abs(self.x) <= 0.06 and abs(self.y) <= 0.06 and abs(self.z) <= 0.06 and abs(self.pitch) <= 0.06 and abs(self.yaw) <= 0.06:
            return True
        return False

    def navigate(self):     
           
        Vectorizer.yaw_only = False
        
        self.handleJoystickInput()

        if abs(self.z) == 1.0 :
            Navigation.setThrusterSpeed("front",self.upThrustVerticalValueMin,self.upThrustVerticalValueMax)
            Navigation.setThrusterSpeed("back",self.upThrustVerticalValueMin,self.upThrustVerticalValueMax)
            Navigation.setThrusterSpeed("front_right",self.upThrustHorizontalValueMin,self.upThrustHorizontalValueMax)
            Navigation.setThrusterSpeed("front_left",self.upThrustHorizontalValueMin,self.upThrustHorizontalValueMax)
            Navigation.setThrusterSpeed("back_right",self.upThrustHorizontalValueMin,self.upThrustHorizontalValueMax)
            Navigation.setThrusterSpeed("back_left",self.upThrustHorizontalValueMin,self.upThrustHorizontalValueMax) 
        else:
            Navigation.setDefaultSpeed()
            
        if abs(self.yaw) > 0.05:
            # rospy.logerr("UPDATING YAW SETPOINT")
            self.fix_heading = False
            self.pid_yaw.updateSetpoint(self.imu_data['yaw'])

        if abs(self.pitch) > 0.05:
            # rospy.logerr("UPDATING PITCH SETPOINT")
            self.pid_pitch.updateSetpoint(self.imu_data['pitch'])
        
        if abs(self.z) > 0.05:
            # rospy.logerr("UPDATING HEAVE SETPOINT")
            self.fix_heave = False
            self.pid_heave.updateSetpoint(self.depth)
        
        if self._isRest(): #and not (self.fix_heading or self.fix_heave or self.fix_tilting):
            if self.imu_data['yaw'] is not None and self.pid_yaw.setpoint is not None:
                rospy.logwarn("ROV at rest: Stabilizing Heading")
                self.stabilizeAtRest()
        
        elif self.activePID and (abs(self.x) > 0.05 or abs(self.y)) and (abs(self.yaw) <= 0.06 or abs(self.pitch) <= 0.06) and abs(self.z) <= 0.06:
            if self.imu_data['pitch'] is not None and self.imu_data['yaw'] is not None and self.pid_yaw.setpoint is not None and self.pid_pitch.setpoint is not None:
                rospy.loginfo("ROV moving Horizontally: Stabilizing Heading and Tilting using active PID")
                self.stabilizeHorizontal()

        elif self.activePID and (abs(self.z) > 0.05) and (abs(self.yaw) <= 0.06 or abs(self.pitch) <= 0.06) and (abs(self.x) <= 0.06 and abs(self.y) <= 0.06):
            if self.imu_data['pitch'] is not None and self.pid_pitch.setpoint is not None:
                rospy.loginfo("ROV moving Vertically: Stabilizing Heading and Tilting using active PID")
                self.stabilizeVertical()

        elif self.activePID and (abs(self.z) > 0.05 and (abs(self.x) > 0.05 or abs(self.y) > 0.5)) and (abs(self.yaw) <= 0.06 or abs(self.pitch) <= 0.06):
            if self.imu_data['pitch'] is not None and self.imu_data['yaw'] is not None and self.pid_yaw.setpoint is not None and self.pid_pitch.setpoint is not None:
                rospy.loginfo("ROV moving Vertically and Horizontally: Stabilizing Heading and Tilting using active PID")
                self.stabilizeLively()

        elif self.fix_heading and (self.z != 0 or self.pitch != 0):
            if self.imu_data['yaw'] is not None and self.pid_yaw.setpoint is not None:
                # rospy.loginfo("ROV moving: Stabilizing heading")
                self.fixHeading()

        elif self.fix_tilting and (self.x != 0 or self.y != 0):
            if self.imu_data['pitch'] is not None and self.pid_pitch.setpoint is not None:
                # rospy.loginfo("ROV moving: Stabilizing tilting")
                self.fixTilting()
        
        elif self.fix_heave and (self.x != 0 or self.y != 0):
            if self.depth is not None and self.pid_heave.setpoint is not None:
                # rospy.loginfo("ROV moving: Stabilizing depth")
                self.fixHeave()

        else:
            # rospy.logerr("ROV in manual control: Using joystick input")
            rospy.logwarn(self.yaw)
            rospy.logerr(self.pitch)
            Navigation.navigate(self.x, self.y, self.pitch, self.z, self.yaw * 0.5)

        self.extractDir()
        # print(f"YAW: {self.imu_data['yaw']}")


if __name__ == "__main__":
    rospy.init_node("navigation_node")
    try:
        # Navigation.stopAll()
        # time.sleep(9)
        node = NavigationNode()
        while not rospy.is_shutdown():
            node.navigate()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Navigation Node...")
    finally:
        rospy.logwarn("FINALLY: Exiting Navigation Node...")
        Navigation.stopAll()
        # time.sleep(3)
        rospy.logwarn("Stopped all motors")
        