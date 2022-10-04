#! /usr/bin/python3
from math import *

SEUIL = 100 #number of points taken into account during the research of closest points ON EACH SIDE of the cureent marker.

def closest_points_from_marker(p1, marker, x2, y2, seuil):
    min_dist = -1
    i_marker = marker
    d = max(marker-seuil, 0)
    f = min(seuil+marker, len(p1))
    for i in range(d, f, 1):
        dist = sqrt(( p1[i][0]-x2 )**2 + ( p1[i][1]-y2 )**2)
        if dist<min_dist or min_dist==-1:
            min_dist = dist
            i_marker = i
    return i_marker


def compare_paths(p1, p2, seuil=SEUIL):
    '''
    Paths are lists of (x,y) coordinates that may have different lengths.
    p1 will be the reference path.

    ALGO:
    define one cursor for each path to choose the points to use
    Mark the beginning of p1
    area = 0
    For each points in p2:
        find the closest point in p1 after the mark
        increase area by distance between points (abs)
        mark the point in p1
    '''
    #Define the step size for each paths
    k1, k2 = 1.0, 1.0
    l1, l2 = len(p1), len(p2)
    numberOfPoints = min(l1,l2)
    print("Planned path length:", l1, "| Taken path length:", l2)
    if l1>l2:
        k1 = l1/l2
    else:
        k2 = l2/l1
    
    #print("k1: ",k1, "| k2: ", k2)
    
    # Mark where is the robot in the path p1
    marker = 0

    #Area is the indicator of the paths' difference
    area = 0

    #i is the current position in p2
    i = 0

    #k is the current number of the current point
    k = 1

    traveled_distance = 0
    old_x=0
    old_y=0
    started=False

    while i < l2:
        
        x2 = p2[i][0]
        y2 = p2[i][1]
        marker = closest_points_from_marker(p1, marker, x2, y2, seuil)
        x1 = p1[marker][0]
        y1 = p1[marker][1]
        area += sqrt(( x1-x2 )**2 + ( y1-y2 )**2)
        #print("CURRENT AREA:",area)
        #print("----------------")

        if not started:
            old_x=x2
            old_y=y2
            started=True

        traveled_distance = traveled_distance + sqrt((old_x-x2)**2 + (old_y-y2)**2)
        old_x = x2
        old_y = y2

        marker = ceil(k*k1)
        i = ceil(k*k2)
        k += 1
    
    return area*traveled_distance/numberOfPoints, traveled_distance



if __name__ == '__main__':
    # Tests of the function
    p1 = [[cos(0),sin(0)],[cos(1),sin(1)],[cos(2),sin(2)],[cos(3),sin(3)],[cos(4),sin(4)],[cos(5),sin(5)],[cos(6),sin(6)],[cos(7),sin(7)],[cos(8),sin(8)],[cos(9),sin(9)],[cos(10),sin(10)]]
    p2 = [[cos(0),sin(0)],[cos(2),sin(2)],[cos(4),sin(4)],[cos(6),sin(6)],[cos(8),sin(8)],[cos(10),sin(10)]]

    p3 = [[0,i] for i in range(100)]
    p4 = [[1,i+1] for i in range(100)]

    area, traveled_distance = compare_paths(p3,p4)
    #print("Area:", area, "| Travelled distance:", traveled_distance)