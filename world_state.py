#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  # for sending commands to the drone
# for receiving navdata feedback
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Some Constants
COMMAND_PERIOD = 100  # ms


class WorldController(object):

    def __init__(self):

        self.stateService = rospy.ServiceProxy('/gazebo/get_model_state',
                                               GetModelState)

    def GetModelState(self):
        try:
            model_coordinates = self.stateService('mobile_base', '')
            return model_coordinates.pose
        except:
            print "Error getting pos"

    def GetTurtleRotation(self, state):
        orien = state.orientation
        orientation_list = [orien.x, orien.y, orien.z, orien.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw