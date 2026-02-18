import math
import numpy
import matplotlib.pyplot as plt
import time
from construct_map import generatePath
import board
import adafruit_bh1750
from odo import odometry
from motorgo import BrakeMode, ControlMode, Plink

def normalize_angle(angle):
    """Bounds angle between -pi and pi to find the shortest turn."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle

def main():
    # Helper parameters
    radius = 2.4 / 2
    baseDist = 4.4

    def turn_in_place(plink, target_theta, current_theta, prevEncLeft, prevEncRight):
        # Optimization: Don't turn if we are already there
        if abs(normalize_angle(target_theta - current_theta)) < 0.05:
            return current_theta, prevEncLeft, prevEncRight
        
        print(f"--- Turning to {target_theta:.2f} (Curr: {current_theta:.2f}) ---")
 
        P = 4.0
        D = 0.2

        prevError = 0
        changeTime = 0.05
        
        error = normalize_angle(target_theta - current_theta)
        
        # Timeout safety (optional but recommended)
        start_turn = time.time()
        
        while abs(error) > 0.03: 
            # Update Odometry
            # Note: We pass 0,0 for X,Y because we only care about Theta changes here
            currx, curry, current_theta, currR, currL = odometry(plink, radius, baseDist, 
                    prevEncLeft, prevEncRight, 0, 0, current_theta)
            
            error = normalize_angle(target_theta - current_theta)

            if abs(error) < 0.02: 
                stopMotors()
                break 

            dererror = (error - prevError) / changeTime
            prevError = error

            correction = P * error + D * dererror
            
            # Clamp correction
            correction = max(min(correction, 0.6), -0.6)

            # Minimum power threshold
            min_power = 0.45 
            if 0 < abs(correction) < min_power:
                correction = min_power if correction > 0 else -min_power
    
            # Update Motors
            left_motor.power_command = -correction
            right_motor.power_command = -correction

            # Update Encoders for next loop
            prevEncLeft = -plink.channel1.position
            prevEncRight = plink.channel3.position
            time.sleep(changeTime)
            
        stopMotors()
        # Return the updated theta so the main loop knows where we actually ended up
        return current_theta, prevEncLeft, prevEncRight

    '''def turn_in_place(plink, target_theta, current_theta, prevEncLeft, prevEncRight):
        if target_theta == current_theta:
            return current_theta, prevEncLeft, prevEncRight
        
        print(f"--- Turning to {target_theta:.2f} ---")
        imu = plink.imu
        P = 2
        D = 0.1

        prevError = 0
        prev_time = time.monotonic()


        gyro_angle_accumulated = 0.0
    
    # Calculate how much we need to turn (Relative Target)
        turn_needed = normalize_angle(target_theta - current_theta)
            
        
        error = normalize_angle(target_theta - current_theta)
        
        while abs(error) > 0.03: 
            currx, curry, current_theta, currR, currL = odometry(plink, radius, baseDist, 
                    prevEncLeft, prevEncRight, 0, 0, current_theta)
            
            current_time = time.monotonic()
            dt = current_time - prev_time

            prev_time = current_time
            
            gyro_rate = imu.gyro[2]

            gyro_angle_accumulated += gyro_rate * dt
        
            # 3. CALCULATE ERROR
            # error = Target - Current
            error = turn_needed - gyro_angle_accumulated
            
            prevError = error

            correction = P * error + D * gyro_rate
            
        
            correction = max(min(correction, 0.6), -0.6)

    # 3. Apply a minimum power threshold so the robot doesn't stall
            min_power = 0.45 
            if 0 < abs(correction) < min_power:
                correction = min_power if correction > 0 else -min_power
    
    # 4. Pure Differential Drive: One positive, one negative
    # Depending on your motor wiring, you might need to swap these signs
            left_motor.power_command = -correction
            right_motor.power_command = -correction

            prevEncLeft = -plink.channel1.position
            prevEncRight = plink.channel3.position
            time.sleep(0.05)
            
        stopMotors()
    
        return current_theta, prevEncLeft, prevEncRight'''
    
    '''def turn_in_place(plink, target_theta, current_theta, prevEncLeft, prevEncRight):
        if target_theta == current_theta:
            return current_theta, prevEncLeft, prevEncRight
        
        print(f"--- Turning to {target_theta:.2f} ---")
        imu = plink.imu
        
        # PID Constants
        P = 2.0  # Reduced P to stop violent shaking
        D = 0.05 # Reduced D
        
        prev_time = time.monotonic()
        
        # Initialize Accumulator
        gyro_angle_accumulated = 0.0
        
        # Calculate Turn Needed (Relative)
        turn_needed = normalize_angle(target_theta - current_theta)
        
        while True:
            # 1. FIX TIME
            current_time = time.monotonic()
            dt = current_time - prev_time
            prev_time = current_time # <--- THIS LINE IS CRITICAL
            
            # 2. Get Gyro Data
            gyro_rate = imu.gyro[2] 
            
            # 3. Integrate
            gyro_angle_accumulated += gyro_rate * dt
            
            # 4. Calculate Error
            error = turn_needed - gyro_angle_accumulated
            
            # DEBUG PRINT: If "Curr" jumps up wildly, your dt is wrong
            # print(f"Err: {error:.2f} | Rate: {gyro_rate:.2f}")

            # 5. Exit Condition
            if abs(error) < 0.05:
                stopMotors()
                break

            # 6. Calculate Correction
            correction = (P * error) - (D * gyro_rate)
            
            # 7. Clamp Max Speed
            correction = max(min(correction, 0.5), -0.5)

            # 8. Min Speed Threshold (With a deadband!)
            # Only apply min power if we are NOT super close.
            # This prevents shaking at the very end.
            if abs(error) > 0.1: 
                min_power = 0.35 
                if 0 < abs(correction) < min_power:
                    correction = min_power if correction > 0 else -min_power
            
            # 9. Drive Motors
            # Note: If it spins the WRONG way, swap these signs to (+)
            left_motor.power_command = -correction
            right_motor.power_command = -correction
            
            # 10. Update Encoders (Record keeping only)
            prevEncLeft = -plink.channel1.position
            prevEncRight = plink.channel3.position
            
            time.sleep(0.01)
            
        stopMotors()
        return target_theta, prevEncLeft, prevEncRight'''

    def moveinline(plink, dist, target_theta, current_theta, prevEncLeft, prevEncRight):
        print(f"--- Moving Distance {dist:.2f} ---")
        P = 6
        D = 0.1
        prevError = 0
        changeTime = 0.05
        
        # Note: We do NOT reset x, y, theta to 0 here. 
        # We use the current_theta passed in.
        # We track local X/Y for distance, but keep Global Theta for heading.
        start_x, start_y = 0, 0 
        curr_x, curr_y = 0, 0
        
        while True:
            # We pass 0,0 for x,y to odometry to track relative distance for this segment
            # But we pass current_theta to track global orientation
            curr_x, curr_y, current_theta, currR, currL = odometry(plink, radius, baseDist, 
                    prevEncLeft, prevEncRight, curr_x, curr_y, current_theta)
            
            distance_traveled = math.sqrt(curr_x**2 + curr_y**2)

            if distance_traveled >= dist:
                stopMotors()
                break

            # Calculate Heading Error
            error = normalize_angle(target_theta - current_theta)
            dererror = (error - prevError) / changeTime
            prevError = error
            
            correct = P * error + D * dererror
            
            minspeed = 0.5
            leftspeed = minspeed - correct
            rightspeed = minspeed + correct
            
            left_motor.power_command = leftspeed
            right_motor.power_command = -rightspeed
            
            # Update Encoders
            prevEncLeft = -plink.channel1.position
            prevEncRight = plink.channel3.position
            time.sleep(changeTime)
            
        return prevEncLeft, prevEncRight

    def stopMotors():
        left_motor.power_command = 0.0
        right_motor.power_command = 0.0

    # ================= SETUP =================
    start = (5, 49) 
    goal = (65, 5)
    startDirX,startDirY = (1,0)
    # ================= SETUP =================
    
    
    # Initialize sensors
    ''' prevEncLeft = -plink.channel1.position
    prevEncRight = plink.channel3.position'''
    
    # Path Planning
    isEasy = False
    resolution = 10
    path = generatePath(isEasy, resolution, start, goal)

    directions = list()
    distances = list()
    prevDiff = (0,0)
    distance = 0
    i = 0

    # Process Path
    while i < len(path) - 1:
        currCoord = path[i]
        nextCoord = path[i+1]
        newDiff = (nextCoord[0] - currCoord[0], nextCoord[1] - currCoord[1])
        stepLength = math.sqrt((nextCoord[0] - currCoord[0])**2 + (nextCoord[1] - currCoord[1])**2)
        
        if newDiff != prevDiff and i > 0:
            directions.append(prevDiff) # Append the COMPLETED segment direction
            distances.append(distance / resolution)
            distance = 0
        
        prevDiff = newDiff
        distance += stepLength
        i += 1
        
    directions.append(prevDiff)
    distances.append(distance / resolution)

    print(f"Directions: {directions}")
    print(f"Distances: {distances}")

    # Angle Mapping
    angleDictionary = {
        (1, 0): 0,
        (1, 1): (math.pi / 4),
        (0, 1): (math.pi / 2),
        (-1, 1): ((3 * math.pi) / 4),
        (-1, 0): (math.pi),
        (-1, -1): ((5 * math.pi) / 4),
        (0, -1): ((3 * math.pi) / 2),
        (1, -1): ((7 * math.pi) / 4)
    }

    plink = Plink()
    left_motor = plink.channel1
    right_motor = plink.channel3
    plink.connect()

    time.sleep(3)
    

    global_theta = angleDictionary[(startDirX,startDirY)]

    prevEncLeft = -plink.channel1.position
    prevEncRight = plink.channel3.position
    imu = plink.imu

    # ================= EXECUTION LOOP =================
    for i in range(len(directions)):
        # 1. Get Target Info
        d_tuple = directions[i] # This is (dx, dy)
        dist = distances[i]
        target_angle = angleDictionary[d_tuple]

        print(f"Step {i}: Goal Angle {target_angle:.2f}, Curr Angle {global_theta:.2f}")

        # 2. TURN IN PLACE
        # We rotate until we face the target angle BEFORE driving
        global_theta, prevEncLeft, prevEncRight = turn_in_place(
            plink, target_angle, global_theta, prevEncLeft, prevEncRight
        )
        
        time.sleep(1)
        
        # 3. DRIVE STRAIGHT (with minor corrections)
        prevEncLeft, prevEncRight = moveinline(plink, dist, 0, 0, prevEncLeft, prevEncRight)

        time.sleep(1)

main()