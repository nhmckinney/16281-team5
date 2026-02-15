#main.py

import math
import numpy
import matplotlib.pyplot as plt
import time
from construct_map import generatePath
#import board
#import adafruit_bh1750

#from motorgo import BrakeMode, ControlMode, Plink



def main():
    isEasy = False
    resolution = 10
    path = generatePath(isEasy, resolution)


    i = 0
    prevDiff = (0,0)
    differences = list()
    distances = list()
    distance = 0
    while i < len(path) - 1:
        currCoord = path[i]
        nextCoord = path[i+1]
        newDiff = nextCoord[0] - currCoord[0], nextCoord[1] - currCoord[1]
        if newDiff != prevDiff:
            differences.append(newDiff)
            distances.append(distance/resolution)
            distance = 0
            prevDiff = newDiff
        distance += 1

        i += 1
    distances.append(distance/resolution)
    distances.pop(0)
    
    print(differences)
    print(distances)







main()