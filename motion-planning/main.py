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

if __name__ == "__main__":

    # read in the PGM map, and convert all elements to [0, 1]
    mapImage = img_as_float(imread("~/Downloads/map10k.pgm"))

    # calculating from image pixel, so distances are based on number of pixels
    trueRobotRadius = 0.4  # metres
    mapResolution = 0.1  # metres/block
    prm = PRM(100, 0.25, 0.9, int(np.ceil(trueRobotRadius / mapResolution)))
    prm.updateMap(mapImage)
    prm.expandRoadmap(100)
    prm.setStart(Node(109, 186))
    prm.setTarget(Node(302, 368))
    # path = prm.findPath()
    prm.visualise(None)
