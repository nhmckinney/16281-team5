#main.py

import math
import numpy
import matplotlib.pyplot as plt
import time
from construct_map import generatePath
import board
import adafruit_bh1750

from motorgo import BrakeMode, ControlMode, Plink

def odometry(plink,radius,baseDist, prevEncLeft, prevEncRight, x, y, theta,
                left_motor,right_motor):
    currentLeft = -plink.channel3.position
    currentRight = plink.channel1.position
    distChangeL = (currentLeft - prevEncLeft) * radius
    distChangeR = (currentRight - prevEncRight) * radius
    prevEncLeft = currentLeft
    prevEncRight = currentRight

    changeLeft = distChangeL
    changeRight = distChangeR
    changeForward = (changeLeft + changeRight) / 2
    changeRot = (changeRight - changeLeft) / baseDist

    x += changeForward * math.cos(theta + changeRot/2)
    #print(x)
    y += changeForward * math.sin(theta + changeRot/2)
    #print(y)
    theta += changeRot
    # INSIDE THE WHILE LOOP
    #print(f"L_pos: {currentLeft:.2f} | R_pos: {currentRight:.2f} | dF: {changeForward:.4f} | dR: {changeRot:.4f}")
    #print("DONE")
    left_motor.power_command = 0
    right_motor.power_command = 0

    return x,y,theta,currentLeft,currentRight

#modified from lab 4
def calcAngle(accel,angvel,grav,changetime,currAngle):
    #inputs are the IMU outputs as numpy arrays
    #should return angle which is what is used in the PID control loop
    #there was a piazza post saying we should average some of the theta
    #calculations using various sensors
    angvx,angvy,angvz = angvel[0],angvel[1],angvel[2]
    angleacc = math.atan2(accel[1],accel[0])
    #weighted average below that can get changed based on tuning
    k1 = 0.9
    angle = k1 * (currAngle + angvz *changetime) + (1-k1) * angleacc
    return angle

def moveinline(plink,left_motor,right_motor,dx,dy,dist,startangle,imu):
    P = 0.4
    D = 0.2
    preverror = 0
    changetime = 0.05
    radius = 2.25
    baseDist = 4.4
    prevEncLeft,prevEncRight = 0,0
    x,y,theta = 0,0,0
    startx,starty,starttheta,right,left = odometry(plink,radius,baseDist,
                prevEncLeft, prevEncRight, x, y, theta,left_motor,right_motor)
    targettheta = math.atan2(dy,dx)
    print(startangle)
    currtheta = starttheta
    print(targettheta)
    while True:
        currx,curry,fadetheta,currR,currL = odometry(plink,radius,baseDist, 
                prevEncLeft, prevEncRight, x, y, theta,left_motor,right_motor)
        distance = math.sqrt((currx-startx)**2 + (curry-starty)**2)
        currtheta = calcAngle(imu.accel,imu.gyro,imu.gravity_vector,changetime,
                                currtheta)
        print(f"distance gone: {distance:.2f}")
        print(f"distance to go: {dist:.2f}")
        print(f"current angle w.r.t. x axis being 0: {currtheta:.2f}")
        if distance >= dist:
            break

        error = targettheta - currtheta
        dererror = (error-preverror)/changetime
        preverror = error
        correct = P*error + D*dererror
        minspeed = 0.2
        leftspeed = minspeed - correct
        rightspeed = -(minspeed + correct)
        print("moving")
        left_motor.power_command = leftspeed
        right_motor.power_command = rightspeed
        time.sleep(changetime)

def main():
    #====================================INPUT THESE ON TEST DAY ====================================
    start = (1,1) #input these on demo day
    goal = (50,70)
    #====================================INPUT THESE ON TEST DAY ====================================

    plink = Plink()
    left_motor = plink.channel3
    right_motor = plink.channel1
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

    startangledict = {
        (0,1):0,
        (1,1):(math.pi/4),
        (1,0):(math.pi/2),
        (1,-1):((3*math.pi)/4),
        (0,-1):(math.pi),
        (-1,-1):((5*math.pi)/4),
        (-1,0):((3*math.pi)/2),
        (-1,1):((7*math.pi)/4)
    }

    startdirs = (1,0)
    stdy = startdirs[0]
    stdx = startdirs[1]

    for i in range(len(differences)):
        print(f"pair {i}")
        dy,dx = differences[i]
        moveinline(plink,left_motor,right_motor,dx,dy,distances[i],
                    startangledict[(stdy,stdx)],imu)

main()
