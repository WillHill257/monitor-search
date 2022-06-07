"""
- create the PRM
- poll the getmap service for map data
- pass that data to a relevant function of the PRM
"""

from skimage.io import imread
from skimage.util import img_as_float
from PRM import PRM
from graph import Node
import numpy as np
from math import cos, sin, pi
from scipy.ndimage import affine_transform


if __name__ == "__main__":

    # read in the PGM map, and convert all elements to [0, 1]
    mapImage = img_as_float(imread("./map3.pgm"))
    # rotate the map a little
    a = pi/40
    R = np.array([
        [cos(a), sin(a), 0],
        [-sin(a), cos(a), 0],
        [0, 0, 1]
    ])

    mapImage = affine_transform(mapImage, np.linalg.inv(R))

    # mapImage = affine_transform(mapImage, np.linalg.inv(T))

    # calculating from image pixel, so distances are based on number of pixels
    trueRobotRadius = 0.2  # metres
    mapResolution = 0.05  # metres/block

    # transform from world to picture
    T = np.array([
        [1, 0, 300],
        [0, 1, 355],
        [0, 0, 1]
    ])

    WtP = T@R

    p = np.array([0, 0])  # world origin
    p = WtP @ np.concatenate((p, np.array([1])))
    p = p.astype(np.int32)

    prm = PRM(40, 0.25, 0.82, 2*int(np.ceil(trueRobotRadius / mapResolution)))
    prm.updateMap(mapImage)
    prm.expandRoadmap(100)
    prm.setStart(Node(p[0], p[1]))
    prm.setTarget(Node(100, 150))
    path = prm.findPath()
    prm.visualise(path)

    PtW = np.linalg.inv(WtP)
    print(path)
    for i in range(len(path)):
        point = PtW @ np.array([path[i][0], path[i][1], 1])
        path[i] = (point[0], -point[1])

    print(path)
