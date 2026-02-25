import time
import board
import adafruit_bh1750
import adafruit_vl53l4cx
from motorgo import ControlMode, Plink

#INIT
plink = Plink()
plink.connect()
right_motor = plink.channel3
left_motor = plink.channel1
left_motor.control_mode = ControlMode.POWER
right_motor.control_mode = ControlMode.POWER

i2c = board.I2C()

light_sensor = adafruit_bh1750.BH1750(i2c)

tof_sensor = adafruit_vl53l4cx.VL53L4CX(i2c)
tof_sensor.start_ranging()


#CONSTANTS
TOTAL_SECTORS = 16
COURSE_MAP = [1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0] # Insert map
GOAL_SECTOR = 11

belief_array = [1.0 / TOTAL_SECTORS] * TOTAL_SECTORS
SENSOR_MATCH = 0.85 #trust tof sensor 85% when input matches map
SENSOR_MISMATCH = 0.15 #15% chance of false positive/negative from  sensor
MOVE_EXACT = 0.985 # Smearing weights
MOVE_SHORT = 0.00625 # smear
MOVE_LONG = 0.00625  # smear 

THRESHOLD = 8
BASE_SPEED = 0.385
KP = 0.065
KD = 0.03

RADIANS_PER_SECTOR = 88.5/16
current_radians = 0.0

last_error = 0 

last_encoder_radians = 0 


#HELPER FUNCS
def follow_black_line_slightly():
    global last_error
    
    current_lux = light_sensor.lux
    
    error = current_lux - THRESHOLD
    derivative = error - last_error
    correction = ((error * KP) + (derivative * KD))
    #print(error)
    last_error = error

    raw_l_cmd = (BASE_SPEED + correction)
    raw_r_cmd = (BASE_SPEED - correction)
    #print(f"correction {correction}")
    #print(f"left: {raw_l_cmd}")
    #print(f"right: {raw_r_cmd}")
    
    # speeds must stay b/w -1/1
    clamped_l_cmd = max(-1.0, min(1.0, raw_l_cmd))
    clamped_r_cmd = max(-1.0, min(1.0, raw_r_cmd))
    
    left_motor.power_command = clamped_l_cmd
    right_motor.power_command = -clamped_r_cmd

def read_encoder_change():
    global last_encoder_radians
    
    current_left = -left_motor.position 
    current_right = right_motor.position 
    
    current_average_radians = (current_left + current_right) / 2.0
    
    change_in_radians = current_average_radians - last_encoder_radians
    
    last_encoder_radians = current_average_radians
    
    return change_in_radians

def stop_motors():
    left_motor.power_command = 0.0
    right_motor.power_command = 0.0

def read_ToF_sensor():
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


def main():

    # Debugging:
    #while True:
     #   read_ToF_sensor()


    global current_radians, belief_array, last_encoder_radians
    
    last_encoder_radians = (-left_motor.position + right_motor.position) / 2.0
    
    print("Starting Localization Run...")

    while True:
        try: 
            follow_black_line_slightly()
            
            current_radians += read_encoder_change()

            if current_radians >= RADIANS_PER_SECTOR:
                current_radians = 0.0
                
                # shift +smear
                new_belief_array = [0.0] * TOTAL_SECTORS
                for i in range(TOTAL_SECTORS):
                    prev_sector = (i - 1) % TOTAL_SECTORS
                    two_sectors_back = (i - 2) % TOTAL_SECTORS
                    
                    prob_from_exact = belief_array[prev_sector] * MOVE_EXACT
                    prob_from_short = belief_array[i] * MOVE_SHORT
                    prob_from_long = belief_array[two_sectors_back] * MOVE_LONG
                    
                    new_belief_array[i] = prob_from_exact + prob_from_short + prob_from_long   
                belief_array = new_belief_array 
                
                #compare map to tof
                sensor_reading = read_ToF_sensor() 
                
                for i in range(TOTAL_SECTORS):
                    expected_reading = COURSE_MAP[i]
                    
                    if sensor_reading == expected_reading:
                        belief_array[i] *= SENSOR_MATCH
                    else:
                        belief_array[i] *= SENSOR_MISMATCH
                
                #normalize
                total_probability = sum(belief_array)
                for i in range(TOTAL_SECTORS):
                    belief_array[i] /= total_probability
                    
                #print and check if goal
                highest_percentage = max(belief_array)
                best_sector_number = belief_array.index(highest_percentage)
                
                print(f"Highest prob location: Sector {best_sector_number} | Confidence: {highest_percentage:.2%}")
                

                if best_sector_number == GOAL_SECTOR and highest_percentage > 0.85:
                    stop_motors()
                    print(f"Localization complete. Goal {GOAL_SECTOR} Reached!")
                    break  
                    
        except KeyboardInterrupt:
            stop_motors()
            print("\nStopped by User.")
            break
            
        except Exception as e:
            print(f"Sensor glitched: {e}, trying again...")
            pass
            
        #prevent sensor/I2C overload
        time.sleep(0.01)

if __name__ == "__main__":
    main()
