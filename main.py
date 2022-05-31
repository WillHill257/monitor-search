#! /usr/bin/env python

from turtle import position
import rospy
from world_state import WorldController
from controller import BasicBotController
from PID import PID
import datetime
import numpy as np


def talker(waypoints):

    node = rospy.init_node("talker", anonymous=True)

    pid_x = PID(0.2, 0.1, 0.5)
    pid_rot = PID(2, 0.1, 0.15)

    controller = BasicBotController()

    world = WorldController()

    start = datetime.datetime.now()

    waypoint_index = 0

    while not rospy.is_shutdown() and waypoint_index < len(waypoints):

        end = datetime.datetime.now()
        d = end - start
        milli = int(d.total_seconds() * 1000)

        if milli > 100:

            state = world.GetModelState()
            position = state.position
            pos_list = np.array([position.x, position.y])

            setpoint = np.array(waypoints[waypoint_index])

            heading = setpoint - pos_list
            heading_ang = np.arctan2(heading[1], heading[0])
            if heading_ang < 0:
                heading_ang = 2 * np.pi + heading_ang

            yaw = world.GetTurtleRotation(state)

            if yaw < 0:
                yaw = 2 * np.pi + yaw

            if yaw - heading_ang > np.pi:
                heading_ang += 2*np.pi

            if heading_ang - yaw > np.pi:
                yaw += 2*np.pi


            control_rot = pid_rot.u(yaw, heading_ang)

            print "Heading: " + str(heading_ang)
            print "Yaw: " + str(yaw)
            print "Control: " + str(control_rot)
            print ''

            controller.SetRotate(control_rot)

            if np.abs(control_rot) < 0.5:
                np_pos = np.array([position.x, position.y])

                control_x = pid_x.u(np_pos, setpoint)

                controller.SetCommand(control_x, 0, 0)
            else:
                controller.SetCommand(0, 0, 0)

            start = datetime.datetime.now()

            if np.linalg.norm(pos_list - np.array(setpoint)) < 0.2:
                waypoint_index += 1


if __name__ == '__main__':
    try:
        waypoints = [(0, 0), (-5, 2), (-5, 3), (0, 3)]
        talker(waypoints)
    except rospy.ROSInterruptException:
        quit()
