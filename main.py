"""Sample for finding magnetic field for a specific lat and long and altitude."""
from math import *
import networkx as nx
from datetime import date
from geomag import WorldMagneticModel

# GLOBALS #
RADIUS_OF_EARTH = 6378.1 # in kilometers
# barings #
EAST = radians(98.75389)
WEST = radians(279.0128)
NORTH = radians(8.89667)
SOUTH = radians(188.94)

def findCoord(lat, lon, baring, distance):
    """
        Find a specific coordinate based of a start coordinate.

        @param lat:     start point latitude in radians
        @param lon:     start point longitude in radians
        @param baring:  direction of point to be found in radians
        @param distance: distance from start point to the desired point in kilometers
        @return latitude and longitude of the new point
    """
    nLat = asin(sin(lat) * cos(distance / RADIUS_OF_EARTH) + cos(lat) * sin(distance / RADIUS_OF_EARTH) * cos(baring)) # in radians
    nLon = lon + atan2(sin(baring) * sin(distance / RADIUS_OF_EARTH) * cos(lat), cos(distance / RADIUS_OF_EARTH) - sin(lat) * sin(nLat))
    nLon = fmod((nLon + 540), 360) - 180 # in radians
    return [degrees(nLat), degrees(nLon)] # return point

def findGridStart(lat, lon, distance):
    """
        Find top left point of grid

        @param lat: start latitude.
        @param lon: start longitude.
        @param distance: radius of grid.
    """
    TL = findCoord(radians(lat), radians(lon),WEST, distance)
    TL = findCoord(radians(TL[0]), radians(TL[1]),NORTH, distance)
    return TL

def GenerateGrid(sLat, sLon, radius, distance):
    """
        Generate grid of latitude and longitude points for finding magnetic field

        @param sLat:    start latitude, center point of grid
        @param sLon:    start longitude, center point of grid
        @param radius:  radius of grid in kilometers
        2param distance:    distance between each point in radius, used to find how many points will be generated
    """
    start = findGridStart(sLat, sLon, radius) # find grid top left point
    points = int((radius / distance) * 2) # find the amount of points from left to right of grid
    Grid = nx.grid_2d_graph(points,points) # generate a grid of nodes
    nx.get_node_attributes(Grid, "data")

def main():
    """Main Function."""
    sLat = 26.30176
    sLon = -98.1635
    alt = 29 # in meters
    unit = 'm' # meter, ft = feet, k = kilometer
    GenerateGrid(sLat, sLon, 1.0, 0.01)
    # Find top left point of grid
    # Find bottom right point of grid

if __name__ == '__main__':
    WMM = WorldMagneticModel('WMM.COF') # Read the data file
    main()