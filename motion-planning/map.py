"""
- This is a representation of the world
- Includes transformation functions so that our PRM view is constant
"""


class OccupancyGrid:
    def __init__(self) -> None:
        # define the map, which is a 1D array of the specified height and width
        # map is ROW-MAJOR
        self.map = None
        self.height = 0
        self.width = 0

    # return the element given a 2D index
    def at(self, x: int, y: int):
        return self.map[x + y * self.width]

    # return the element given a 1D index - OVERLOADED
    def at(self, index: int):
        return self.map[index]
