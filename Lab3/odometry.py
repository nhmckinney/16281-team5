# spin_motors.py
# Before running this script, ensure that the MotorGo Plink is
# connected to the Raspberry Pi and that it has been flashed with the
# MotorGo firmware.

import time
import math
import board
import adafruit_bh1750

from motorgo import BrakeMode, ControlMode, Plink

def runPowerPair(plink,left_motor,right_motor,left_power,right_power,
                        x,y,theta,radius,baseDist):
    leftScale = 1
    rightScale = 0.75
    prevEncLeft = -plink.channel3.position
    print(prevEncLeft)
    prevEncRight = plink.channel1.position
    print(prevEncRight)
    left_motor.power_command = left_power * leftScale
    right_motor.power_command = -right_power * rightScale
    tStart = time.time()
        
    while time.time() - tStart < 3:
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
        print(f"L_pos: {currentLeft:.2f} | R_pos: {currentRight:.2f} | dF: {changeForward:.4f} | dR: {changeRot:.4f}")
        time.sleep(0.02)
    print("DONE")
    left_motor.power_command = 0
    right_motor.power_command = 0
    time.sleep(0.5)

    return x,y,theta

def main():

    plink = Plink()
    left_motor = plink.channel3
    right_motor = plink.channel1
    plink.connect()

    imu = plink.imu

    left_motor.control_mode = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER

    i2c = board.I2C()
    sensor = adafruit_bh1750.BH1750(i2c)

    time.sleep(3)

    wheelRadius = 2.4/2
    distWheelBases = 6.0625
    x = 0
    y = 0
    theta = 0

    powerPairs = [
        (-0.9,-0.72),
        (0.63, -0.4),
        (-0.55, 0.87)
    ]

    for lpower,rpower in powerPairs:
        x,y,theta = runPowerPair(plink,left_motor,right_motor,lpower,rpower,
                     x,y,theta,wheelRadius,distWheelBases)
        
    print("FINAL POSITION")
    print(f"x = {x:.2f} inches")
    print(f"y = {y:.2f} inches")
    print(f"theta = {math.degrees(theta)%360} deg")

if __name__ == "__main__":
    main()