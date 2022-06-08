#! /usr/bin/env python
"""
- create the PRM
- poll the getmap service for map data
- pass that data to a relevant function of the PRM
"""

#from skimage.io import imread
#from skimage.util import img_as_float
from PRM import PRM
from graph import Node
import numpy as np
from math import cos, sin, pi
# from scipy.ndimage import affine_transform
import matplotlib.pyplot as plt

# from turtle import position
import rospy
from world_state import WorldController
from controller import BasicBotController
from PID import PID
import datetime

import threading

# detector
from detector import Detector


def move(waypoints, controller, pid_x, pid_rot, world):
    waypoint_index = 0
    start = datetime.datetime.now()
    print "Moving..."

    while waypoint_index < len(waypoints):

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
                heading_ang += 2 * np.pi

            if heading_ang - yaw > np.pi:
                yaw += 2 * np.pi

            control_rot = pid_rot.u(yaw, heading_ang)

            # print "Heading: " + str(heading_ang)
            # print "Yaw: " + str(yaw)
            # print "Control: " + str(control_rot)
            # print ''

            controller.SetRotate(control_rot)

            drift_threshold = 0.5

            if np.abs(control_rot) < drift_threshold:
                np_pos = np.array([position.x, position.y])

                control_x = pid_x.u(np_pos, setpoint)

                controller.SetCommand(control_x, 0, 0)
            else:
                controller.SetCommand(0, 0, 0)

            start = datetime.datetime.now()

            if np.linalg.norm(pos_list - np.array(setpoint)) < 0.2:
                waypoint_index += 1

    controller.SetCommand(0, 0, 0)
    controller.SetRotate(0)


def generate_map():
    print "Generating PRM..."
    # read in the PGM map, and convert all elements to [0, 1]
    mapImage = plt.imread("./map3_edited.pgm") / 255.0

    # this is in picture coordinate
    prm = PRM(50, 0.25, 0.82)
    prm.updateMap(mapImage)
    prm.expandRoadmap(100)
    return prm


def get_transformed_path(prm):
    goal = list(map(float, raw_input("Enter goal position: ").split(" ")))

    state = world.GetModelState()
    position = state.position
    start = np.array([position.x, -position.y])
    print "starting at: " + str(start[0]) + ", " + str(start[1])
    # p = np.array([0, 0])  # world origin
    start = np.matmul(prm.WtP, np.concatenate((start, np.array([1]))))
    start = start.astype(np.int32)

    goal = np.array([goal[0], -goal[1]])
    goal = np.matmul(prm.WtP, np.concatenate((goal, np.array([1]))))
    goal = goal.astype(np.int32)

    prm.setStart(Node(start[0], start[1]))
    prm.setTarget(Node(goal[0], goal[1]))

    # prm.visualise()
    path = prm.findPath()
    prm.visualise(path)

    transformed_path = []
    for i in range(len(path)):
        point = np.matmul(prm.PtW, np.array([path[i].x, path[i].y, 1]))
        transformed_path.append((point[0], -point[1]))

    return transformed_path


if __name__ == "__main__":

    node = rospy.init_node("talker", anonymous=True)

    world = WorldController()

    prm = generate_map()

    transformed_path = get_transformed_path(prm)

    pid_x = PID(0.5, 0.1, 0.5)
    pid_rot = PID(2, 0.1, 0.15)

    controller = BasicBotController()

    move_thread = threading.Thread(target=move,
                                   args=(transformed_path, controller, pid_x,
                                         pid_rot, world))

    try:

        move_thread.start()
        detector = Detector()
        while not rospy.is_shutdown():

            found = False
            if not detector.image_received:
                continue

            detector.get_threshold()
            found = detector.is_present()

            detector.pub.publish('Yes' if found else 'No')

            if not move_thread.is_alive():
                transformed_path = get_transformed_path(prm)
                move_thread = threading.Thread(target=move,
                                   args=(transformed_path, controller, pid_x,
                                         pid_rot, world))
                move_thread.start()

    except rospy.ROSInterruptException:
        quit()

    finally:
        move_thread.join()
