# spin_motors.py
# Before running this script, ensure that the MotorGo Plink is
# connected to the Raspberry Pi and that it has been flashed with the
# MotorGo firmware.

import time
import math
import board
import adafruit_bh1750

from motorgo import BrakeMode, ControlMode, Plink

def odometry(plink,radius,baseDist, prevEncLeft, prevEncRight, x, y, theta):
        currentLeft = -plink.channel1.position
        currentRight = plink.channel3.position
        distChangeL = (currentLeft - prevEncLeft) * radius
        distChangeR = (currentRight - prevEncRight) * radius

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
        #left_motor.power_command = 0
        #right_motor.power_command = 0

        return x,y,theta,currentLeft,currentRight

def main():

    plink = Plink()
    left_motor = plink.channel1
    right_motor = plink.channel3
    plink.connect()

    imu = plink.imu

    left_motor.control_mode = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER

    time.sleep(3)

    wheelRadius = 2.4/2
    distWheelBases = 4.4
    x = 0
    y = 0
    theta = 0
    tStart = time.time()
    prevEncLeft = -plink.channel1.position
    prevEncRight = plink.channel3.position

    print(f"start vals: {x}, {y}, {theta}")

    while time.time() - tStart < 6:
        x,y,theta,currentLeft,currentRight = odometry(plink, wheelRadius, 
                            distWheelBases, prevEncLeft, prevEncRight,
                                    x,y,theta)
        prevEncLeft = currentLeft
        prevEncRight = currentRight
        time.sleep(0.02)
        
    print("FINAL POSITION")
    print(f"x = {x:.2f} inches")
    print(f"y = {y:.2f} inches")
    print(f"theta = {math.degrees(theta)%360} deg")
    time.sleep(0.02)

if __name__ == "__main__":
    main()
