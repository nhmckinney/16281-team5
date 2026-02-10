#16281 Team 5 Lab 4

import time, math
import board
from motorgo import ControlMode, Plink

def calcAngle(accel, angvel, grav, changetime, currAngle):
    accx, accy, accz = accel[0], accel[1], accel[2]
    angvx, angvy, angvz = angvel[0], angvel[1], angvel[2]
    gravx, gravy, gravz = grav[0], grav[1], grav[2]
    
    anglegrav = math.atan2(gravy, gravz)
    
    # Complementary Filter
    k1 = 0.4
    angle = k1 * (currAngle + angvy * changetime) + (1 - k1) * anglegrav
    return angle, angvy

def main():
    plink = Plink()
    plink.connect()

    # Setup Motors
    imu = plink.imu
    right_motor = plink.channel1
    left_motor = plink.channel3

    left_motor.control_mode = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER
    
    # --- PID GAINS ---
    # P and D for maintaining vertical balance (Angle Control)
    P = 10.5       
    D = 0.5      
    
    # I (Integral) for maintaining position (Distance Control)
    # Start VERY small. If it oscillates back and forth wildly, lower this.
    # If it drifts away and never comes back, increase slightly.
    Ki = 0.05 

    tchange = 0.01 

    currAngle = 0 
    last_error = 0
    
    # --- ENCODER INITIALIZATION ---
    # Store the starting positions to treat the current spot as "0"
    # Note: Using polarity from odometry.py (Left is negative, Right is positive)
    start_pos_left = -plink.channel3.position
    start_pos_right = plink.channel1.position

    print("Calibrating... Stand the robot up!")
    time.sleep(1)

    while True:
        # 1: Calculate Angle
        currAngle, gy = calcAngle(imu.accel, imu.gyro, imu.gravity_vector,
                                tchange, currAngle)
        
        # 2: Calculate Distance (The Integral Term)
        # Read current raw positions
        curr_pos_left = -plink.channel3.position
        curr_pos_right = plink.channel1.position
        
        # Calculate displacement from start
        delta_left = curr_pos_left - start_pos_left
        delta_right = curr_pos_right - start_pos_right
        
        # Average distance (in encoder ticks/radians)
        # We don't necessarily need to convert to inches if we just tune Ki
        avg_pos = (delta_left + delta_right) / 2.0

        # 3: PID Control
        # Error is the deviation from upright (0 degrees)
        angle_error = currAngle
        
        # Derivative is the rate of change of error
        dererror = (angle_error - last_error) / tchange
        last_error = angle_error
        
        # Total Output = Angle Correction + Damping + Position Correction
        # Ki * avg_pos acts as the "Integral" term because Position is the integral of Velocity
        output = (P * angle_error) + (D * dererror) + (Ki * avg_pos)

        # 4: Saturation (Clamp output to +/- 1.0)
        if output > 1:
            output = 1
        elif output < -1:
            output = -1

        # 5: Send Command
        # Note: Directions preserved from original balance.py
        left_motor.power_command = -output
        right_motor.power_command = output 
        
        # Debugging: Print Angle and Position to see if it's drifting
        # print(f"Ang: {currAngle:.2f} | Pos: {avg_pos:.2f} | Out: {output:.2f}")

        time.sleep(tchange)

def stop_motors():
        # Helper, though not called in main loop currently
        plink = Plink()
        plink.channel3.power_command = 0.0
        plink.channel1.power_command = 0.0

if __name__ == "__main__":
    main()