"""
- Will has a node class representing a node and its adjacent nodes
- Will has a graph class representing all the nodes and graph-level algorithms like astar
"""

from queue import PriorityQueue
from typing import List, Tuple


class Node:
    def __init__(self, x: float, y: float) -> None:
        # takes in the x and y position of the cell represented by this node
        self.x = x
        self.y = y

        # the adjacent nodes
        self.adjacencies = []

    # add adajcency
    def addAdjacency(self, node: object) -> None:
        # add the other node to our adjacency list
        self.adjacencies.append(node)

    # remove adjacency
    def removeAdjacency(self, node: object) -> None:
        self.adjacencies.remove(node)

    # determine the straight line distance to another node
    def distance(self, node: object) -> float:
        # use euclidean distance
        return ((self.x - node.x) ** 2 + (self.y - node.y) ** 2) ** 0.5

    # convert the coordinates to a string
    def toString(self) -> str:
        return str(self.x) + ";" + str(self.y)

    # overload the equality operator
    def __eq__(self, __o: object) -> bool:
        # compare the class type and coordinate valuess
        return isinstance(__o, self.__class__) and __o.x == self.x and __o.y == self.y

    # overload the not-equal operator
    def __ne__(self, __o: object) -> bool:
        return not self.__eq__(__o)

    # overload the hash
    def __hash__(self) -> int:
        # hash is just the coordinates represented by this node
        return hash(self.toString())


# node for use with astar
class AStarNode:
    def __init__(self, node: Node) -> None:
        # save the node
        self.node = node

        # add the costs
        self.g = 0
        self.h = 0

        # add a parent
        self.parent = None

    # return the cost of this node
    def cost(self):
        return self.g + self.h

    # overload the equality operator
    def __eq__(self, __o: object) -> bool:
        # compare the class type and coordinate valuess
        return self.node == __o.node

    # overload the not-equal operator
    def __ne__(self, __o: object) -> bool:
        return not self.__eq__(__o)

    # overload the hash
    def __hash__(self) -> int:
        # hash is just the coordinates represented by this node
        return self.node.__hash__()


class Graph:
    def __init__(self) -> None:
        # create a set of nodes, which will just be coordinates
        # set because possibility of duplicates when sampling
        self.nodes = set()

    # add a node
    def addNode(self, node: Node) -> None:
        self.nodes.add(node)

    # remove a node
    def removeNode(self, node: Node) -> None:
        # loop through the node's adjacency list
        for adjacency in node.adjacencies:
            # remove this node from the adjacent nodes
            adjacency.removeAdjacency(node)

        # remove the node from the set
        self.nodes.remove(node)

    # return the heuristic distance to the target
    def heuristic(self, target: Node, point: Node) -> float:
        # use the straight line distance
        return target.distance(point)

    # perform astar from the start to the target nodes
    # will return a boolean of if a path exists and the list of waypoints (if applicable)
    def astar(self, start: Node, target: Node) -> Tuple[bool, List[str]]:
        # confirm that start and target aren't the same node
        if start == target:
            return (True, [])

        # define an open list as a priority queue
        # add elements as a tuple: (priority, item)
        frontier = PriorityQueue(maxsize=0)  # no max size

        # define the explored set
        explored = set()

        # create the start node and add it to the frontier
        startNode = AStarNode(start)
        targetNode = AStarNode(target)
        frontier.put((startNode.cost(), startNode))

        # iterate until the frontier is empty or the target is found
        found = False
        while not frontier.empty():
            # get the next (cheapest) node to visit
            currentNode = frontier.get()

            # lazy deletion
            # check if this node has been visited/seen before
            if currentNode in explored:
                # skip any further processing for this node
                continue

            # check if this node is the target
            if currentNode == targetNode:
                # found the target
                found = True

                # set the parent
                targetNode.parent = currentNode.parent
                break

            # mark this node as visisted
            explored.add(currentNode)

            # loop through the adjacent nodes
            for neighbour in currentNode.node.adjacencies:
                # create an AStarNode object
                neighbourNode = AStarNode(neighbour)

                # if we have seen this node before, skip it
                if neighbourNode in explored:
                    continue

                # set its known cost to the parent's known cost + distance to this node
                neighbourNode.g = currentNode.g + neighbourNode.node.distance(
                    currentNode.node
                )

                # set the heuristic cost
                neighbourNode.h = neighbourNode.node.distance(targetNode.node)

                # set the parent node to the current node
                neighbourNode.parent = currentNode

                # add to the frontier
                # if this node has never been seen before, then great
                # if it has, and this one is cheaper, it will be popped first and lazy deletion will catch the remainder
                # if it has, and this one is more expensive, it will be popped afterwards and lazy deletion will stop it
                frontier.put((neighbourNode.cost(), neighbourNode))

        # have finished the search, backtrack to get the path
        if not found:
            return (False, [])

        # do the backtrack
        path = []
        currentNode = targetNode
        while currentNode != None:
            # add the current coords to the path
            path.append(currentNode.node.toString())

            # traverse to the parent
            currentNode = currentNode.parent

        # return the reversed list so that the first point is at the beginning
        return (True, path[::-1])
