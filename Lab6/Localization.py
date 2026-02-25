import time
import board
import adafruit_bh1750
import adafruit_vl53l4cx # CORRECTED: Based on the README.pdf
from motorgo import ControlMode, Plink

# ==========================================
# 1. HARDWARE INITIALIZATION
# ==========================================

# --- Plink and Motor Setup ---
# Used by: follow_black_line_slightly(), read_encoder_change(), stop_motors(), and main()
plink = Plink()
plink.connect()
right_motor = plink.channel3
left_motor = plink.channel1
left_motor.control_mode = ControlMode.POWER
right_motor.control_mode = ControlMode.POWER

# --- I2C Bus Setup ---
# Used by: Hardware initialization for both sensors below
i2c = board.I2C()

# --- Light Sensor Setup ---
# Used by: follow_black_line_slightly()
light_sensor = adafruit_bh1750.BH1750(i2c)

# --- Time-of-Flight Sensor Setup ---
# Used by: read_ToF_sensor()
tof_sensor = adafruit_vl53l4cx.VL53L4CX(i2c)
tof_sensor.start_ranging()


# ==========================================
# 2. ALGORITHM & CONTROL CONSTANTS
# ==========================================

# --- Bayes Filter Map & Goal ---
TOTAL_SECTORS = 16
# The Map: 1 bit equals cardboard, 0 bit equals empty space. Update for your specific trial.
COURSE_MAP = [1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0] # Insert map
GOAL_SECTOR = 11 # Replace with actual goal sector

# --- Probabilities & Tuning Constants ---
belief_array = [1.0 / TOTAL_SECTORS] * TOTAL_SECTORS
SENSOR_MATCH = 0.85      # Trust the ToF sensor 85% when reality matches the map
SENSOR_MISMATCH = 0.15   # 15% chance of a false positive/negative from the sensor
MOVE_EXACT = 0.985          # 90% chance odometry perfectly tracked one sector
MOVE_SHORT = 0.00625      # 5% chance we fell short 
MOVE_LONG = 0.00625        # 5% chance we went too far 

# --- Line Following Constants ---
# Used by: follow_black_line_slightly()
THRESHOLD = 8
BASE_SPEED = 0.385
KP = 0.065
KD = 0.03

# --- Odometry Constants & Variables ---
# Used by: main()
# Remember: Measure this physically by pushing your robot one sector distance!
RADIANS_PER_SECTOR = 88.5/16  # Measure thru testing
current_radians = 0.0

# --- Memory Variables ---
# Used by: follow_black_line_slightly()
last_error = 0 

# Used by: read_encoder_change() (Note: it is initialized at the start of main())
last_encoder_radians = 0 

#for debugging
#actual_sector = 0


# ==========================================
# 3. HELPER FUNCTIONS
# ==========================================

def follow_black_line_slightly():
    global last_error
    
    # Read the current light level in lux
    current_lux = light_sensor.lux
    
    # Calculate PD correction based on hardcoded threshold
    error = current_lux - THRESHOLD
    derivative = error - last_error
    correction = ((error * KP) + (derivative * KD))
    #print(error)
    # Save the current error for the next time the loop runs
    last_error = error

    # Calculate raw motor speeds
    raw_l_cmd = (BASE_SPEED + correction)
    raw_r_cmd = (BASE_SPEED - correction)
    #print(f"correction {correction}")
    #print(f"left: {raw_l_cmd}")
    #print(f"right: {raw_r_cmd}")
    
    # CLAMP THE SPEEDS: Force them to stay between -1.0 and 1.0
    clamped_l_cmd = max(-1.0, min(1.0, raw_l_cmd))
    clamped_r_cmd = max(-1.0, min(1.0, raw_r_cmd))
    
    # Send safe commands to motors
    left_motor.power_command = clamped_l_cmd
    right_motor.power_command = -clamped_r_cmd

def read_encoder_change():
    global last_encoder_radians
    
    # Read current encoder positions
    # Applying the negative sign to the left motor based on your hardware setup
    current_left = -left_motor.position 
    current_right = right_motor.position 
    
    # Calculate the average distance traveled by the center of the robot
    current_average_radians = (current_left + current_right) / 2.0
    
    # Find how many radians we traveled since the very last loop
    change_in_radians = current_average_radians - last_encoder_radians
    
    # Save the current position for the next loop to compare against
    last_encoder_radians = current_average_radians
    
    return change_in_radians

def stop_motors():
    left_motor.power_command = 0.0
    right_motor.power_command = 0.0

def read_ToF_sensor():
    # CORRECTED: Logic adapted from the MotorGo Python API Reference README
    # Wait until the sensor has a fresh reading to ensure we don't accidentally pull empty data
    while not tof_sensor.data_ready:
        pass
        

    tof_sensor.clear_interrupt()
    distance_cm = tof_sensor.distance

    print(distance_cm)
    
    if 12.0 < distance_cm < 45.0:
        print(f"box")
        print(f"distance: {distance_cm}")
        return 1
    
    return 0


# ==========================================
# 4. THE MAIN CONTINUOUS LOOP
# ==========================================

def main():

    #while True:
     #   read_ToF_sensor()


    global current_radians, belief_array, last_encoder_radians
    
    # Initialize the encoder memory before starting the loop so math doesn't spike
    last_encoder_radians = (-left_motor.position + right_motor.position) / 2.0
    
    print("Starting Localization Run...")

    while True:
        # Put sensor reading code in a try-catch block to prevent crashes from glitches
        try: 
            # A. Move and Measure
            follow_black_line_slightly()
            
            current_radians += read_encoder_change()
            #print(f"current_radians: {current_radians}")

            # B. Did we enter a new sector?
            if current_radians >= RADIANS_PER_SECTOR:
                current_radians = 0.0  # Reset the ruler for the next sector
                #actual_sector +=1
                #print(f"Actual sector: {actual_sector}")
                
                # ---------------------------------------------------------
                # C. Math Step 1: The Motion Update (Shift & Smear)
                # ---------------------------------------------------------
                new_belief_array = [0.0] * TOTAL_SECTORS
                
                for i in range(TOTAL_SECTORS):
                    prev_sector = (i - 1) % TOTAL_SECTORS
                    two_sectors_back = (i - 2) % TOTAL_SECTORS
                    
                    prob_from_exact = belief_array[prev_sector] * MOVE_EXACT
                    prob_from_short = belief_array[i] * MOVE_SHORT
                    prob_from_long = belief_array[two_sectors_back] * MOVE_LONG
                    
                    new_belief_array[i] = prob_from_exact + prob_from_short + prob_from_long
                    
                belief_array = new_belief_array 
                
                # ---------------------------------------------------------
                # D. Math Step 2: The Observation Update (Reality Check)
                # ---------------------------------------------------------
                sensor_reading = read_ToF_sensor() 
                
                for i in range(TOTAL_SECTORS):
                    expected_reading = COURSE_MAP[i]
                    
                    if sensor_reading == expected_reading:
                        belief_array[i] *= SENSOR_MATCH
                    else:
                        belief_array[i] *= SENSOR_MISMATCH
                
                # ---------------------------------------------------------
                # E. Math Step 3: Normalization
                # ---------------------------------------------------------
                total_probability = sum(belief_array)
                for i in range(TOTAL_SECTORS):
                    belief_array[i] /= total_probability
                    #print(f"belief_array[{i}]: {belief_array[i]}")
                    
                # ---------------------------------------------------------
                # F. Report and Check Goal
                # ---------------------------------------------------------
                highest_percentage = max(belief_array)
                best_sector_number = belief_array.index(highest_percentage)
                
                # Keep terminal output clean
                print(f"Highest prob location: Sector {best_sector_number} | Confidence: {highest_percentage:.2%}")
                

                # If highly confident we are in the target destination, stop
                if best_sector_number == GOAL_SECTOR and highest_percentage > 0.85:
                    stop_motors()
                    print(f"Localization complete. Goal {GOAL_SECTOR} Reached!")
                    break  
                    
        except KeyboardInterrupt:
            # Allows you to safely Ctrl+C to exit the program while testing
            stop_motors()
            print("\nStopped by User.")
            break
            
        except Exception as e:
            print(f"Sensor glitched: {e}, trying again...")
            pass
            
        # Sleep statement prevents sensor/I2C overload
        time.sleep(0.01)

if __name__ == "__main__":
    main()
