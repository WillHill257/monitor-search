#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib
import rospy
import numpy as np
import sys
import time
# Import the messages we're interested in sending and receiving
# for sending commands to the drone
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Empty  # for land/takeoff/emergency


# Some Constants
COMMAND_PERIOD = 200  # ms


class BasicBotController(object):

    def __init__(self):

        # Holds the current drone status
        self.status = -1
        self.navdata = None

        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        # Setup regular publishing of control packets
        self.command = Twist()
        self.commandTimer = rospy.Timer(
            rospy.Duration(COMMAND_PERIOD / 1000.0), self.SendCommand)

    def SetCommand(self, x,y,z):

        # Called by the main program to set the current command
        self.command.linear.x = x
        self.command.linear.y = y
        self.command.linear.z = z

    def SetRotate(self, z=0):
        # Called by the main program to set the current command
        self.command.angular.z = z

    def SendCommand(self, event):
        self.pubCommand.publish(self.command)
