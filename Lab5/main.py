#main.py

import math
import numpy
import matplotlib.pyplot as plt
import time
from construct_map import generatePath
import board
import adafruit_bh1750
from odo import odometry

from motorgo import BrakeMode, ControlMode, Plink


def main():

    def moveinline(plink,dx,dy,dist,prevEncLeft,prevEncRight,angleDict):
        P = 0.2
        D = 0.1
        prevError = 0
        changeTime = 0.05
        radius = 2.4/2
        baseDist = 4.4
        x,y,theta = 0,0,0
        startx,starty,starttheta,right,left = odometry(plink,radius,baseDist,
                    prevEncLeft, prevEncRight, x, y, theta)
        targettheta = angleDict[(dy,dx)]
        #print(f"start angle: {starttheta}")
        currx, curry, currtheta = startx, starty, starttheta
        #print(f"target angle: {targettheta}")
        print(f"currtheta at start of move: {currtheta:.3f} rad")
        print(f"targettheta: {targettheta:.3f} rad")
        print(f"error before: {targettheta - currtheta:.3f} rad")
        while True:
            currx,curry,currtheta,currR,currL = odometry(plink, radius,baseDist, 
                    prevEncLeft, prevEncRight, x, y, currtheta)
            distance = math.sqrt((currx-startx)**2 + (curry-starty)**2)

            #DEBUG PRINT STATEMENTS ===================================================
            print(f"distance gone: {distance:.2f}")
            print(f"distance to go: {dist:.2f}")
            print(f"current angle w.r.t. x axis being 0: {currtheta:.2f}")
            #DEBUG PRINT STATEMENTS ===================================================
            if distance >= dist:
                break

            error = targettheta - currtheta
            dererror = (error-prevError)/changeTime
            prevError = error
            correct = P*error + D*dererror
            minspeed = 0.55
            leftspeed = minspeed - correct
            rightspeed = minspeed + correct
            print("moving")
            left_motor.power_command = leftspeed
            right_motor.power_command = -rightspeed
            time.sleep(changeTime)


    def turn(dx,dy):
        print(currAngle)
        pass

    def stopMotors():
        left_motor.power_command = 0.0
        right_motor.power_command = 0.0

    #====================================INPUT THESE ON TEST DAY ====================================
    start = (5,5) #input these on demo day
    goal = (50,60)
    #====================================INPUT THESE ON TEST DAY ====================================

    plink = Plink()
    left_motor = plink.channel1
    right_motor = plink.channel3
    plink.connect()

    imu = plink.imu

    left_motor.control_mode = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER

    time.sleep(1)

    isEasy = False
    resolution = 10
    path = generatePath(isEasy, resolution, start, goal)

    i = 0
    prevDiff = (0,0)
    differences = list()
    distances = list()
    distance = 0
    while i < len(path) - 1:
        currCoord = path[i]
        nextCoord = path[i+1]
        newDiff = nextCoord[0] - currCoord[0], nextCoord[1] - currCoord[1]

        stepLength = math.sqrt((nextCoord[0] - currCoord[0])**2 + (nextCoord[1] - currCoord[1])**2)
        if newDiff != prevDiff:
            differences.append(newDiff)
            distances.append(distance/resolution)
            distance = 0
            prevDiff = newDiff
        distance += stepLength

        i += 1
    distances.append(distance/resolution)
    distances.pop(0)

    print(f"differences: {differences}")
    print(f"distances: {distances}")

    angleDictionary = {
        (0,1):0,
        (1,1):(math.pi/4),
        (1,0):(math.pi/2),
        (1,-1):((3*math.pi)/4),
        (0,-1):(math.pi),
        (-1,-1):((5*math.pi)/4),
        (-1,0):((3*math.pi)/2),
        (-1,1):((7*math.pi)/4)
    }

    currAngle = angleDictionary[differences[0]]
    prevEncLeft = -plink.channel1.position
    prevEncRight = plink.channel3.position

    for i in range(len(differences)):
        dy,dx = differences[i]
        moveinline(plink,dx,dy,distances[i],prevEncLeft,prevEncRight,
                        angleDictionary)
        stopMotors()
        time.sleep(1)
        prevEncLeft = -plink.channel1.position
        prevEncRight = plink.channel3.position

        #if i < len(differences) - 1:
            #newDx, newDy = differences[i+1]
            #turn(angleDictionary[newDx,newDy],'nothing')

        time.sleep(1)

main()
