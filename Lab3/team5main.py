import time
import board
import adafruit_bh1750
from motorgo import ControlMode, Plink

def main():
    plink = Plink()
    plink.connect()
    right_motor = plink.channel1
    left_motor = plink.channel3

    left_motor.control_mode = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER
    
    i2c = board.I2C()
    sensor = adafruit_bh1750.BH1750(i2c)

    #working nums = 0.18,0.18,0.175,0.225 2-3 late seconds
    BASE_SPEED = 0.18
    BASE_BASE_SPEED = 0.18
    KP = 0.175  
    KD = 0.225

    i = 0


    #works = [0.15,0.175,0.225] 

    def stop_motors():
        left_motor.power_command = 0.0
        right_motor.power_command = 0.0

    def calibrate():
        print("--- CALIBRATION ---")
        print("Wave sensor over Black and White...")
        time.sleep(1)
        min_lux, max_lux = 5000, 0
        start_time = time.time()
        while time.time() - start_time < 4:
            val = sensor.lux
            if val < min_lux: min_lux = val
            if val > max_lux: max_lux = val
            time.sleep(0.01)
        
        threshold = (min_lux + max_lux) / 2
        print(f"Threshold set to: {threshold:.0f}")
        return threshold

    try:
        #threshold = calibrate()
        #time.sleep(10)
        threshold = 8
        last_error = 0 
        BIAS = -0.01


        while True:
            current_lux = sensor.lux
            error = current_lux - threshold
            derivative = error - last_error
            correction = ((error * KP) + (derivative * KD)) * 1.0
            last_error = error

            '''if abs(error) > 8.5:
                print(i) 
                i += 1
                BIAS = -0.05
            else:
                BASE_SPEED = BASE_BASE_SPEED'''
            l_cmd = BASE_SPEED - BIAS + correction
            r_cmd = -(BASE_SPEED + BIAS - correction)
            left_motor.power_command = l_cmd*1.5
            print(l_cmd, r_cmd)
            right_motor.power_command = r_cmd*1.5


    except KeyboardInterrupt:
        stop_motors()
        print("\nStopped.")
    except Exception as e:
        stop_motors()
        print(f"\nError: {e}")

if __name__ == "__main__":
    main()

