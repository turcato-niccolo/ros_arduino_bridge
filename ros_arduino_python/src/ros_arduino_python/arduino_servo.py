#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 26 14:44:50 2022

@author: Niccolo' Turcato (niccolo.turcato@studenti.unipd.it)
"""
import rospy
import math
import numpy

from ros_arduino_msgs.srv import DigitalWrite
from ros_arduino_msgs.srv import DigitalSetDirection
from ros_arduino_msgs.srv import SetServoSpeed, ServoAttach, ServoWrite, ServoRead, ServoDetach

from sensor_msgs.msg import JointState
from ros_arduino_msgs.msg import Digital
from std_msgs.msg import Float64



"""
    This version uses topics
"""

#class ArduinoGripper():
#    """
#        Set the joint name of the gripper and the opening and closing values of the servo 
#        (in radiants, [0, pi] or [-p/2, pi/2])
#    """
#    def __init__(self, joint_name, open_val=math.pi, close_val=0):
#        self.joint_name = joint_name
#        self.open_val = open_val
#        self.close_val = close_val
#        
#        self.joint_pub = rospy.Publisher('/' + joint_name + '/command', Float64, queue_size=5)        
#        self.joint_state = JointState()
#        
#        # Subscribe to the /joint_states topic so we know where the servo is positioned
#        rospy.Subscriber('/joint_states', JointState, self.update_joint_state)
#    
#    def open(self):
#       self.joint_pub.publish(self.open_val)   
#       
#    def close(self):
#        self.joint_pub.publish(self.close_val)    
#
#    def update_joint_state(self, msg):
#        self.joint_state = msg
#        
#    def get_joint_position(self):
#        index = self.joint_state.name.index(self.joint_name)
#        return self.joint_state.position[index]
#
#    def set_pos(self, val):
#        self.joint_pub.publish(val)


"""
This version uses services
"""

class ArduinoGripper():
    """
        Set the pin of the gripper's servo and the opening and closing values of the servo 
        (in radiants, [0, pi] or [-p/2, pi/2])
    """
    def __init__(self, pin, open_val=45, close_val=0):
        self.pin = int(pin)
        self.open_val = open_val
        self.close_val = close_val
        
        self.attach_servo = rospy.ServiceProxy('arduino/servo_attach', ServoAttach)
        self.detach_servo = rospy.ServiceProxy('arduino/servo_attach', ServoDetach)

        self.attach_servo(self.pin)
        
        self.servo_write = rospy.ServiceProxy('arduino/servo_write', ServoWrite)
        self.servo_read = rospy.ServiceProxy('arduino/servo_read', ServoRead)
    
    def open(self):
       """
       service call /arduino/servo_write ID open_val
       """
       self.servo_write(self.pin, self.open_val)
       
    def close(self):
        """
        service call /arduino/servo_write ID close_val
        """
        self.servo_write(self.pin, self.close_val)


    def get_joint_position(self):
        return self.servo_read(self.pin).position
    
    def on_shutdown(self):
        rospy.loginfo('Detaching servo')
        self.detach_servo(self.pin)
        rospy.sleep(2)

    def set_pos(self, val):
        """
            rosservice call /arduino/servo_write ID val
        """
        self.servo_write(self.pin, val)

