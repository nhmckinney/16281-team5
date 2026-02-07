#16281 Team 5 Lab 4

import time,math

import board

from motorgo import ControlMode, Plink

def calcAngle(accel,angvel,grav,changetime,currAngle):
    #for nathan and dho explanation
    #inputs are the IMU outputs as numpy arrays
    #should return angle which is what is used in the PID control loop
    #there was a piazza post saying we should average some of the theta
    #calculations using various sensors
    accx,accy,accz = accel[0],accel[1],accel[2]
    angvx,angvy,angvz = angvel[0],angvel[1],angvel[2]
    gravx,gravy,gravz = grav[0],grav[1],grav[2]
    anglegrav = math.atan2(gravx,gravz)
    angleacc = math.atan2(accx,accz)
    # weighted average depending on which is more accurate/useful, can play 
    # around with k between 0 and 1
    k1 = 0.5
    angle = k1*anglegrav + (1-k1)*angleacc
    anggyro = currAngle + angvy*changetime
    #another weighted average below that can get changed based on tuning
    # k2 should probably be a lot closer to 1 (like 0.9-1) because what matters
    # way more is the gyro I heard another group talking about how they did the
    # same as the calculation before and used k2 = 0.98. I tried using k2 = 0.98
    # and the numbers started getting weird so idk might have to play around or 
    # try to understand more how it works.
    k2 = 0.5
    angle = k2*anggyro + (1-k2)*angle
    return angle, angvy

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
    
    power = 0.3
    P = 0.175  
    D = 0.225
    tchange = 0.01 #time between while iterations (time.sleep(tchange))

    currAngle = 0 #initializing current angle, as robot should start straight up
    # The IMU object provides the raw IMU data:
    # - 3-axis accelerometer data in m/s^2
    # - 3-axis gyroscope data in rad/s
    # - 3-axis magnetometer data in uT

    while True:
        #1: read sensors (already done automatically)
        #2: calculate change in time for angle calculation
        # This step is only necessary if the "time.sleep(x)" isn't accurate
        # enough and errors start accumulating. We can use the x in this 
        #case as the time change since it is the (approximate) time between
        #iterations of the while loop
        
        #3: calc angle
        currAngle, gy = calcAngle(imu.accel,imu.gyro,imu.gravity_vector,
                                tchange,currAngle)
        print(f"angle = {currAngle}")
        #gy is the gyroscope measurement for the rad/s, is used for the D control

        #4: Use PID to control motors based on angle

        left_motor.power_command = 0
        right_motor.power_command = 0

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
        time.sleep(tchange)



def stop_motors():
        left_motor.power_command = 0.0
        right_motor.power_command = 0.0

if __name__ == "__main__":
    main()


