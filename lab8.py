import time
import math
import board
import adafruit_bh1750
from motorgo import BrakeMode, ControlMode, Plink

def walk_to_target(left_motor1, right_motor1,
    right_motor2, left_motor2, power, goalDistance, time_per_inch):
    """
    Commands the walking robot to move forward for a calculated amount of time.
    """
    # Calculate how many seconds to run the motors
    run_time = goalDistance * time_per_inch
    
    print(f"Goal: {goalDistance} inches.")
    print(f"Time per inch: {time_per_inch} sec/in.")
    print(f"Calculated run time: {run_time:.2f} seconds. Starting walk...")
    
    # Scale factors if one leg assembly drags more than the other
    leftScale = 1
    rightScale = 1
    
    # Start the motors
    left_motor1.power_command = -(power * leftScale)
    right_motor1.power_command = power * rightScale 
    left_motor2.power_command = -(power * leftScale)
    right_motor2.power_command = power * rightScale 
    
    # Let the motors run for the calculated time
    time.sleep(run_time)
        
    # Stop the motors once the time is up
    print("Time is up! Braking...")
    left_motor1.power_command = 0
    right_motor1.power_command = 0
    left_motor2.power_command = 0
    right_motor2.power_command = 0
    time.sleep(0.5) # Give it a moment to settle
    
    return goalDistance

def main():
    # Initialize hardware
    plink = Plink()
    left_motor1 = plink.channel2
    left_motor2 = plink.channel3
    right_motor1 = plink.channel1
    right_motor2 = plink.channel4
    plink.connect()

    left_motor1.control_mode = ControlMode.POWER
    right_motor1.control_mode = ControlMode.POWER
    left_motor2.control_mode = ControlMode.POWER
    right_motor2.control_mode = ControlMode.POWER

    # i2c = board.I2C()
    # sensor = adafruit_bh1750.BH1750(i2c)

    time.sleep(3)

    # --- WALKING CALIBRATION VARIABLES ---
    # You need to physically measure this! Run your robot at your chosen 
    # 'walk_power' for exactly 5 seconds, measure the distance in inches, 
    # and divide 5 by that distance to get seconds per inch.
    time_per_inch = 40/57 # seconds required to travel 1 inch (Example value)
    
    # The target distance you want to test
    goalDistance = 100.0 # inches
    
    # Motor power for walking
    # WARNING: If you change this power later, your time_per_inch will change!
    walk_power = 1.0

    # Execute the walk
    walk_to_target(
        left_motor1, 
        right_motor1,
        right_motor2,
        left_motor2, 
        power=walk_power, 
        goalDistance=goalDistance, 
        time_per_inch=time_per_inch
    )
        
    print("\n--- RUN COMPLETE ---")
    print("Robot stopped.")

if __name__ == "__main__":
    main()
