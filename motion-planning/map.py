"""
- This is a representation of the world
- Includes transformation functions so that our PRM view is constant
"""

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import numpy as np
from skimage.morphology import disk, binary_dilation


class Map:
    def __init__(self, step_size, occupancy_threshold, robot_radius) -> None:
        # define the map, which is a 1D array of the specified height and width
        # map is ROW-MAJOR
        self.data = []
        self.height = 0
        self.width = 0
        self.origin = None

        # visibility ray casting step size
        self.step_size = step_size
        # the threshold probability for a occupancy cell to be considered as free
        self.threshold = occupancy_threshold
        # the radius of the robot itself
        self.robot_radius = robot_radius

    # return the element given a 2D index
    def at(self, x: int, y: int):
        return self.map[x + y * self.width]

    # return the element given a 1D index - OVERLOADED
    def at(self, index: int):
        return self.map[index]

    # updates the map with occupancy grid
    def updateMap(self, occupancyGrid) -> None:

        # convert the occupancy grid into a binary image
        # this is useful for dilation purposes
        self.data = np.array(occupancyGrid.data)
        self.data = np.abs(self.data)
        self.data = self.data > self.threshold

        # dilate the binary grid to account for size of robot
        SE = disk(self.robot_radius)
        self.data = binary_dilation(self.data, SE)

        self.height = occupancyGrid.info.height
        self.width = occupancyGrid.info.width
        self.origin = occupancyGrid.info.origin

    # checks if a given coord is free, i.e. no obstacles and known
    def isFree(self, x, y):
        # TODO: tune this
        return self.at(x, y) == 0

    def isVisible(self, node1, node2):
        dist = node1.distance(node2)

        # use a ray to check for a intersection between the nodes
        n1 = np.array([node1.x, node1.y])
        n2 = np.array([node2.x, node2.y])

        # ray march from the first point to the second
        ro = n1
        rd = n2 - n1
        rd = rd/np.sum(rd)  # normalise ray direction

        dist_travelled = 0
        pos = ro

        # ray marching loop
        while(dist_travelled < dist):
            pos += rd * self.step_size

            if(not self.isFree(int(pos[0]), int(pos[1]))):
                return False

            dist_travelled += self.step_size

        return True
