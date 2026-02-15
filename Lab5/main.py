#main.py

import math
import numpy
import matplotlib.pyplot as plt
import time
from construct_map import generatePath
#import board
#import adafruit_bh1750

#from motorgo import BrakeMode, ControlMode, Plink


from path import wavefront


def main():
    isEasy = False
    resolution = 10
    path = generatePath(isEasy, resolution)


    i = 0
    prevDiff = (0,0)
    differences = list()
    while i < len(path) - 1:
        currCoord = path[i]
        nextCoord = path[i+1]
        newDiff = nextCoord[0] - currCoord[0], nextCoord[1] - currCoord[1]
        if newDiff != prevDiff:
            differences.append(newDiff)
            prevDiff = newDiff

        i += 1

    print(differences)







main()