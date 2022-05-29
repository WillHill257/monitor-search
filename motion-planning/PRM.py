"""
- this is the main PRM driver code
- should not be run in isolation, should be imported wherever needed
"""

from map import Map
from graph import *
import random
from typing import List


class PRM:
    # initialise
    def __init__(self, adjacency_radius, step_size, occupancy_threshold, robot_radius) -> None:
        self.worldmap = Map(step_size, occupancy_threshold, robot_radius)
        self.roadmap = Graph()
        self.start = None
        self.target = None

        # radius for adjacencies of nodes
        self.radius = adjacency_radius
        self.ROADMAP_MAX_ADDITIONAL_SAMPLES = 100
        self.ROADMAP_ADDITIONAL_SAMPLE_SIZE = 100

    # updates the map of the world/environment

    def updateMap(self, occupancyGrid):
        self.worldmap.update(occupancyGrid)

    # sample a random point with in the bounds of the world map
    def samplePoint(self):
        x = random.randint(0, self.worldmap.width-1)
        y = random.randint(0, self.worldmap.height-1)

        return (x, y)

    # check if 2 nodes are adjancent and valid neighbours
    def performNodeAdjacencyAddition(self, node1, node2):
        # check for radius
        if(node1.distance(node2) > self.radius):
            return

        # check if a node is visible to another
        if(not self.worldmap.isVisible(node1, node2)):
            return

        # add nodes as neighbours of each other
        node1.addAdjacency(node2)
        node2.addAdjacency(node1)

    # expand with roadmap with "sampleCount" number of random samples
    def expandRoadmap(self, sampleCount):

        for _ in range(sampleCount):
            x, y = self.samplePoint()

            if(self.worldmap.isFree(x, y)):
                new_node = Node(x, y)
                self.roadmap.addNode(new_node)
                for node in self.roadmap.nodes:
                    if(node != new_node):
                        self.performNodeAdjacencyAddition(node, new_node)

    # set the start coordinate
    def setStart(self, start: Node) -> None:
        # remove the current start from the graph
        if self.start != None:
            self.graph.removeNode(self.start)

        # set the start node
        self.start = start

        # check the start node for valid adjacencies
        # loop through the graphs nodes
        for node in self.graph.nodes:
            # check if valid and adjcent node
            self.performNodeAdjacencyAddition(self.start, node)

        # add the start to the graph
        self.graph.addNode(self.start)

    # set the target coordinate
    def setTarget(self, target: Node) -> None:
        # remove the current target from the graph
        if self.target != None:
            self.graph.removeNode(self.target)

        # set the target node
        self.target = target

        # check the target node for valid adjacencies
        # loop through the graphs nodes
        for node in self.graph.nodes:
            # check if valid and adjacent node
            self.performNodeAdjacencyAddition(self.target, node)

        # add the target to the graph
        self.graph.addNode(self.target)

    # find a path
    def findPath(self) -> List[str]:
        # try and find a path
        path_found, path = self.graph.astar(self.start, self.target)

        # if a path doesn't exist, sample more points and try again
        # do for a maximum number of times before returning no path
        count = 0
        while (not path_found) and (
            count < self.ROADMAP_MAX_ADDITIONAL_SAMPLES
        ):

            # expand the roadmap
            self.expandRoadmap(self.ROADMAP_ADDITIONAL_SAMPLE_SIZE)

            # check for a path
            path = self.graph.astar(self.start, self.target)

            count += 1

        return path
