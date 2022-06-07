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



if __name__ == "__main__":

    # read in the PGM map, and convert all elements to [0, 1]
    mapImage = plt.imread("./map3.pgm") / 255.0
    # rotate the map a little
    a = pi/40
    R = np.array([
        [cos(a), sin(a), 0],
        [-sin(a), cos(a), 0],
        [0, 0, 1]
    ])

    # mapImage = affine_transform(mapImage, np.linalg.inv(R))

    # mapImage = affine_transform(mapImage, np.linalg.inv(T))

    # calculating from image pixel, so distances are based on number of pixels
    trueRobotRadius = 0.3  # metres
    mapResolution = 0.05  # metres/block

    # transform from world to picture
    T = np.array([
        [1/mapResolution, 0, 320],
        [0, 1/mapResolution, 330],
        [0, 0, 1]
    ])

    WtP = np.matmul(T, R)

    p = np.array([0, 0])  # world origin
    p = np.matmul(WtP, np.concatenate((p, np.array([1]))))
    p = p.astype(np.int32)

    prm = PRM(40, 0.25, 0.82, 2*int(np.ceil(trueRobotRadius / mapResolution)))
    prm.updateMap(mapImage)
    prm.expandRoadmap(100)
    prm.setStart(Node(p[0], p[1]))
    prm.setTarget(Node(100, 150))
    path = prm.findPath()

    prm.visualise(path)

    # t = threading.Thread(target=prm.visualise, args=(path))
    # t.start()

    PtW = np.linalg.inv(WtP)
    transformed_path = []
    for i in range(len(path)):
        point = np.matmul(PtW, np.array([path[i].x, path[i].y, 1]))
        transformed_path.append((point[0], -point[1]))

    # print(path)

    try:
        talker(transformed_path)
    except rospy.ROSInterruptException:
        # t.join()
        quit()

