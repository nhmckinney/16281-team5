##Tentative code for now, need to update

import time # Required for the sleep statement to prevent sensor overload

# ==========================================
# 1. THE SETUP PHASE (Before the Robot Moves)
# ==========================================

TOTAL_SECTORS = 16

#USER INPUT REQUIRED: Map of carboard boxes
COURSE_MAP = {} 

#USER INPUT REQUIRED: Target destination
GOAL_SECTOR = 5 

#Initial Guess: Start by assuming we could be anywhere. 
#Create list of 16 sectors, each set to 1/16
belief_array = [1.0 / TOTAL_SECTORS] * TOTAL_SECTORS

#USER TUNING REQUIRED: Tuneable Uncertainty Constants
SENSOR_MATCH = 0.85      # Trust the ToF sensor 85% when reality matches the map
SENSOR_MISMATCH = 0.15   # 15% chance of a false positive/negative from the sensor

#USER TUNING REQUIRED: More unxertainty constants—
MOVE_EXACT = 0.80        # 80% chance odometry perfectly tracked one sector
MOVE_SHORT = 0.10        # 10% chance we hit a stretch in the circle and fell short (still in current sector)
MOVE_LONG = 0.10         # 10% chance we hit a dent in the circle and went too far (skipped a sector)

#For checking if we've changed sectors
#USER INPUT REQUIRED: the average encoder ticks needed to travel one sector (# ticks for full circle divided by 16)
#Necessary because circle is deformed. Every sector is different # ticks so best approach is to average all of them.
TICKS_PER_SECTOR = 1500  #CHANGE THIS
current_ticks = 0

# ==========================================
# Functions we need to write:
    # If reading through code for the first time, go to the main loop first to 
    # understand what each function does and is for.
# ==========================================
def follow_black_line_slightly():
    # Use your BH1750 Light Sensor to adjust steering and drive forward slightly
    pass

def read_encoder_change():
    # Read the MT6701 magnetic encoders and return the difference since last checked
    return 10 # Placeholder

def read_ToF_sensor():
    # Read VL53L4CD ToF sensor. Return 1 if distance < 2 meters, else return 0
    return 1 # Placeholder

def stop_motors():
    # Send 0 velocity to motor driver
    pass


# ==========================================
# 2. THE MAIN CONTINUOUS LOOP
# ==========================================

# The while True loop allows your robot to continue running forever
while True:
    
    # The Safety Net: Put sensor code in a try-catch block 
    # so sudden errors or glitches don't crash the robot
    try: 
        
        # A. Move and Measure
        follow_black_line_slightly()
        current_ticks += read_encoder_change()
        
        # B. Did we enter a new sector?
        if current_ticks >= TICKS_PER_SECTOR:
            current_ticks = 0  # Reset the ruler for the next sector
            
            # ---------------------------------------------------------
            # C. Math Step 1: The Motion Update (Shift & Smear)
            # ---------------------------------------------------------
            # Create a blank scratchpad to avoid the "Chain Reaction" overwriting problem
            new_belief_array = [0.0] * TOTAL_SECTORS
            
            for i in range(TOTAL_SECTORS):
                # We use modulo (%) to ensure the math wraps around the circular track (15 + 1 = 0)
                prev_sector = (i - 1) % TOTAL_SECTORS
                two_sectors_back = (i - 2) % TOTAL_SECTORS
                
                # Smear the probabilities based on our movement trust
                prob_from_exact = belief_array[prev_sector] * MOVE_EXACT
                prob_from_short = belief_array[i] * MOVE_SHORT
                prob_from_long = belief_array[two_sectors_back] * MOVE_LONG
                
                new_belief_array[i] = prob_from_exact + prob_from_short + prob_from_long
                
            # Overwrite old guesses with the newly shifted data
            belief_array = new_belief_array 
            
            # ---------------------------------------------------------
            # D. Math Step 2: The Observation Update (Reality Check)
            # ---------------------------------------------------------
            # Returns 1 if block, 0 if empty
            sensor_reading = read_ToF_sensor() 
            
            for i in range(TOTAL_SECTORS):
                expected_reading = COURSE_MAP[i]
                
                if sensor_reading == expected_reading:
                    # Sensor matches the map -> Boost probability
                    belief_array[i] *= SENSOR_MATCH
                else:
                    # Sensor contradicts the map -> Penalize probability
                    belief_array[i] *= SENSOR_MISMATCH
            
            # ---------------------------------------------------------
            # E. Math Step 3: Balancing the Odds (Normalization)
            # ---------------------------------------------------------
            # Recalculate the list so the total probability equals 1.0 (100%) again
            total_probability = sum(belief_array)
            
            for i in range(TOTAL_SECTORS):
                belief_array[i] /= total_probability
                
            # ---------------------------------------------------------
            # F. Report and Check Goal
            # ---------------------------------------------------------
            # Find the highest percentage and which sector it belongs to
            highest_percentage = max(belief_array)
            best_sector_number = belief_array.index(highest_percentage)
            
            # Keep terminal clean by printing only the highest probability location
            # Formats the float as a clean percentage (e.g., 87.50%)
            print(f"Highest probability location: Sector {best_sector_number} | Confidence: {highest_percentage:.2%}")
            
            # If we are highly confident we are in the target destination, stop
            if best_sector_number == GOAL_SECTOR and highest_percentage > 0.85:
                stop_motors()
                print("Localization complete. Goal Reached!")
                break  # Exits the while True loop entirely
                
    except Exception as e:
        # If the I2C bus glitches, catch it here and just try again next loop
        print(f"Sensor glitched: {e}, trying again...")
        
    # Safety Net: Add a sleep statement so reading sensors non-stop doesn't cause errors
    time.sleep(0.01) 
