#16281 Team 5 Lab 4

import time,math

import board

from motorgo import ControlMode, Plink


def main():
    plink = Plink()
    plink.connect()

    # The Plink object has an IMU object, corresponding to the 4 motor channels
    # You can save a reference as a local variable for convenience (as below) or
    # access them directly from the Plink object
    imu = plink.imu
    right_motor = plink.channel1
    left_motor = plink.channel3

    left_motor.control_mode = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER
        

    
    BASE_SPEED = 0.18
    KP = 0.175  
    KD = 0.225

    # The IMU object provides the raw IMU data:
    # - 3-axis accelerometer data in m/s^2
    # - 3-axis gyroscope data in rad/s
    # - 3-axis magnetometer data in uT

    while True:


        print("----")
        print(f"Acceleration: {imu.accel}")
        print(f"Angular Velocity: {imu.gyro}")
        #print(f"Magnetic Field: {imu.mag}")
        print(f"Gravity Vector: {imu.gravity_vector}")
        print("----")


        #last_error = 0 
        #error = CURRENT - DESIRED
        #derivative = error - last_error
        #correction = ((error * KP) + (derivative * KD))
        #last_error = error

        #l_cmd = BASE_SPEED + correction
        #r_cmd = -(BASE_SPEED - correction)
        #left_motor.power_command = 0.5 #l_cmd
        #right_motor.power_command = 0.5 #r_cmd

        # Delay as long as you need, communications continue in the background
        time.sleep(1)



def stop_motors():
        left_motor.power_command = 0.0
        right_motor.power_command = 0.0

if __name__ == "__main__":
    main()


