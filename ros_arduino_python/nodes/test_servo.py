#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 26 15:47:07 2022

@author: Niccolo' Turcato (niccolo.turcato@studenti.unipd.it)
"""

import rospy
import sys

from ros_arduino_python.arduino_servo import ArduinoGripper

global gripper

def shutdown():
    if not gripper is None:
        gripper.on_shutdown()
    

if __name__ == '__main__':
    node_name = "test_servo" # Set a name for the node
    rospy.init_node(node_name) # Initialize the node
    rospy.on_shutdown(shutdown)
    
    delay = 2
    if len(sys.argv) == 2:
        gripper = ArduinoGripper(sys.argv[1])
    else:
        gripper = ArduinoGripper(sys.argv[1], sys.argv[2], sys.argv[3])
        

    #while not rospy.is_shutdown():      
    rospy.loginfo('Open gripper')
    gripper.open()
          
    rospy.sleep(delay)
      
    rospy.loginfo('Close gripper')
    gripper.close()
        
    rospy.sleep(delay)
