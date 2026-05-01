#16281 Team 5 Lab 4

import time,math

import board

from motorgo import ControlMode, Plink

def calcAngle(accel,angvel,grav,changetime,currAngle):
    #inputs are the IMU outputs as numpy arrays
    #should return angle which is what is used in the PID control loop
    #there was a piazza post saying we should average some of the theta
    #calculations using various sensors
    angvx,angvy,angvz = angvel[0],angvel[1],angvel[2]
    gravx,gravy,gravz = grav[0],grav[1],grav[2]
    anglegrav = math.atan2(gravy,gravz)
    #weighted average below that can get changed based on tuning
    k1 = 0.4
    angle = k1 * (currAngle + angvy*changetime) + (1-k1) * anglegrav
    return angle

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
    
    P = 15.5    #FINAL P = 14
    # P = 14
     #15 w/ half used battery
    #10 with full battery
    D = 0.35   #FINAL D = 0.35
    # D = 0.4 or 0.5 
      #0.3 w/ half used battery
    # D = 0.35 with full
    #WORKS: P = 15 D = 0.4
    I = 0.0125
    #FINAL I = 0.0125
    #I = 0.0125
    tchange = 0.01 #time between while iterations (time.sleep(tchange))

    currAngle = 0 #initializing current angle, as robot should start straight up
    last_error = 0
    ierror = 0
    # The IMU object provides the raw IMU data:
    # - 3-axis accelerometer data in m/s^2
    # - 3-axis gyroscope data in rad/s
    # - 3-axis magnetometer data in uT

    while True:
        currAngle = calcAngle(imu.accel,imu.gyro,imu.gravity_vector,
                                tchange,currAngle)
        print(f"angle: {currAngle:.1f}")
        if currAngle > 0.5: ierror = 0
        #Use PID to control motors based on angle
        error = currAngle
        dererror = (error - last_error) / tchange
        last_error = error
        output = P*error + D*dererror + I*ierror

        if output > 1:
            output = 1
        elif output < -1:
            output = -1
        else:
            ierror += currAngle *tchange

        print(f"ierror: {ierror:.2f}")

        left_motor.power_command = -output
        right_motor.power_command = output #motor is reversed
        print(f"output: {output:.2f}")

        #print("----")
        #print(f"Acceleration: {imu.accel}")
        #print(f"Angular Velocity: {imu.gyro}")
        #print(f"Magnetic Field: {imu.mag}")
        #print(f"Gravity Vector: {imu.gravity_vector}")
        #print("----")

        # Delay as long as you need
        time.sleep(tchange)



def stop_motors():
        left_motor.power_command = 0.0
        right_motor.power_command = 0.0

if __name__ == "__main__":
    main()
