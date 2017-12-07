"""Sample for finding magnetic field for a specific lat and long and altitude."""
from math import *
import networkx as nx
from networkx.utils import pairwise
from datetime import date
from geomag import WorldMagneticModel


# GLOBALS #
WMM = WorldMagneticModel('WMM.COF') # Read the data file
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

def GenerateGrid(sLat, sLon, alt, radius, distance, UNIT='m'):
    """
        Generate grid of latitude and longitude points for finding magnetic field

        @param sLat:    start latitude, center point of grid
        @param sLon:    start longitude, center point of grid
        @param radius:  radius of grid in kilometers
        2param distance:    distance between each point in radius, used to find how many points will be generated
    """
    start = findGridStart(sLat, sLon, radius) # find grid top left point
    rows = int(radius / distance) * 2
    cols = int(radius / distance) * 2 # find the amount of points from left to right of grid
    Grid = nx.empty_graph(0,None) # Create empty graph
    # Rather than se the grid_2d_graph, lets make our own
    data = WMM.calc_mag_field(start[0], start[1], alt, unit=UNIT) # get the first point
    point = start # set lat lon for pointer
    for i in range(0,rows):
        for j in range(0,cols):
            Grid.add_node((i,j), latitude=point[0],longitude=point[1], toal_intensity=data.total_intensity, Bx=data.Bx, By=data.By, Bz=data.Bz, H=data.Bh, I=data.dip, D=data.dip, GV=data.grid_variation)
            point = findCoord(radians(point[0]), radians(point[1]), EAST, distance) # lets shift this east to find points in row
        start = findCoord(radians(start[0]), radians(start[1]), SOUTH, distance) # shift start of row down to find next row
        point = start # update pointer
    # Now we can go add edges
    Grid.add_edges_from(((i, j), (pi, j))
                     for pi, i in pairwise(list(range(rows))) for j in range(cols))
    Grid.add_edges_from(((i, j), (i, pj))
                     for i in range(rows) for pj, j in pairwise(list(range(cols))))
    # Will ignore periodic and directed as I don't care too much about it
    Grid.add_edges_from((v,u) for u,v in Grid.edges())
    # Lets print the graph and see
    Grid.nodes(data=True)
    return Grid

def Export(grid):
    with open("output.txt","w") as o:
        for n in grid.nodes():
            o.write(str(grid.node[n]["latitude"]) + "," + str(grid.node[n]["longitude"]) + "\n")

def main():
    """Main Function."""
    sLat = 26.30176
    sLon = -98.1635
    alt = 29 # in meters
    unit = 'm' # meter, ft = feet, k = kilometer
    G = GenerateGrid(sLat, sLon, alt, 1, 0.01, 'm')
    path = nx.shortest_path(G,(1,90), (95,2))


if __name__ == '__main__':
    main()