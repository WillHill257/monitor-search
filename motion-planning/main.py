"""
- create the PRM
- poll the getmap service for map data
- pass that data to a relevant function of the PRM
"""
from PRM import PRM

if __name__ == "__main__":
    # calculating from image pixel, so distances are based on number of pixels
    prm = PRM(20, 1, 0.2, 5)

    pass
