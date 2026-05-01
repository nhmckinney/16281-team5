import math
import numpy as np
import time
from collections import deque
from motorgo import BrakeMode, ControlMode, Plink
import matplotlib.pyplot as plt

# ==========================================
# 1. CONSTANTS & CONFIGURATION
# ==========================================
L1 = 4 + (1/8)  # Length of link 1 in inches
L2 = 4 + (3/8)  # Length of link 2 in inches

THETA1_MIN = 0     
THETA1_MAX = 180   
THETA2_MIN = -180  
THETA2_MAX = 180   

RESOLUTION_DEG = 5 # Map resolution

# Target Points for Demo
START_POINT = (6.25, 0)
POINT_A = (1, 1)   
POINT_B = (-1, 5)   
POINT_C = (-4.5, 2)  

# ==========================================
# 2. PHYSICAL OBSTACLES (From Image)
# ==========================================
class Line:
    def __init__(self, p1, p2, boundType):
        self.p1 = p1
        self.p2 = p2
        self.boundType = boundType
    
    def inLine(self, x, y):
        inLine = True
        if (self.p1[0] == self.p2[0]):
            y_on_line = 0
            x_on_line = self.p1[0]
        elif (self.p1[1] == self.p2[1]):
            y_on_line = self.p1[1]
            x_on_line = 0
        else:
            y_on_line = self.p1[1] + (self.p2[1] - self.p1[1]) * (x - self.p1[0]) / (self.p2[0] - self.p1[0])
            x_on_line = self.p1[0] + (self.p2[0] - self.p1[0]) * (y - self.p1[1]) / (self.p2[1] - self.p1[1])

        for char in self.boundType:
            if char == 'T': inLine = inLine and (y < y_on_line)
            if char == 'B': inLine = inLine and (y > y_on_line)
            if char == 'R': inLine = inLine and (x < x_on_line)
            if char == 'L': inLine = inLine and (x > x_on_line)
        return inLine

class Obstacle:
    def __init__(self, l1, l2, l3, l4):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
    
    def clash(self, x, y):
        inBounds = True
        inBounds = inBounds and self.l1.inLine(x, y)
        inBounds = inBounds and self.l2.inLine(x, y)
        inBounds = inBounds and self.l3.inLine(x, y)
        inBounds = inBounds and self.l4.inLine(x, y)
        return inBounds

def construct_obstacles():
    """Builds the exact blocks from the playing field image."""
    obstacles = [
        Obstacle(
            Line((-4, 4), (-2, 4), "B"), Line((-2, 4), (-2, 6), "R"),
            Line((-2, 6), (-4, 6), "T"), Line((-4, 6), (-4, 4), "L")
        ),
        Obstacle(
            Line((1, 5), (3, 5), "B"), Line((3, 5), (3, 7), "R"),
            Line((3, 7), (1, 7), "T"), Line((1, 7), (1, 5), "L")
        )
    ]
    return obstacles 

# ==========================================
# 3. KINEMATICS & COLLISION
# ==========================================
def forward_kinematics(theta1_deg, theta2_deg):
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    x_elbow = L1 * math.cos(t1)
    y_elbow = L1 * math.sin(t1)
    x_ee = x_elbow + L2 * math.cos(t1 + t2)
    y_ee = y_elbow + L2 * math.sin(t1 + t2)
    return ((x_elbow, y_elbow), (x_ee, y_ee))

def inverse_kinematics(x, y):
    solutions = []
    d_sq = x**2 + y**2
    cos_theta2 = (d_sq - L1**2 - L2**2) / (2 * L1 * L2)
    if cos_theta2 < -1.0 or cos_theta2 > 1.0:
        return solutions 
        
    theta2_rad_1 = math.atan2(math.sqrt(1 - cos_theta2**2), cos_theta2)
    theta2_rad_2 = math.atan2(-math.sqrt(1 - cos_theta2**2), cos_theta2)
    
    for t2 in [theta2_rad_1, theta2_rad_2]:
        k1 = L1 + L2 * math.cos(t2)
        k2 = L2 * math.sin(t2)
        t1 = math.atan2(y, x) - math.atan2(k2, k1)
        
        theta1_deg = math.degrees(t1)
        theta2_deg = math.degrees(t2)
        
        if theta2_deg > 180: theta2_deg -= 360
        if theta2_deg < -180: theta2_deg += 360
            
        if (THETA1_MIN <= theta1_deg <= THETA1_MAX) and (THETA2_MIN <= theta2_deg <= THETA2_MAX):
            solutions.append((theta1_deg, theta2_deg))
    return solutions

def is_link_in_collision(x0, y0, x1, y1, obstacles, steps=10):
    for i in range(steps + 1):
        t = i / steps
        sample_x = x0 + t * (x1 - x0)
        sample_y = y0 + t * (y1 - y0)
        for obs in obstacles:
            if obs.clash(sample_x, sample_y):
                return True 
    return False

def is_configuration_valid(theta1, theta2, obstacles):
    (x_elbow, y_elbow), (x_ee, y_ee) = forward_kinematics(theta1, theta2)
    if y_elbow < -6.0 or y_ee < -6.0: return False
    if is_link_in_collision(0, 0, x_elbow, y_elbow, obstacles): return False
    if is_link_in_collision(x_elbow, y_elbow, x_ee, y_ee, obstacles): return False
    return True

# ==========================================
# 4. MAP GENERATION & PATH PLANNING
# ==========================================
def generate_c_space(obstacles):
    t1_bins = int((THETA1_MAX - THETA1_MIN) / RESOLUTION_DEG) + 1
    t2_bins = int((THETA2_MAX - THETA2_MIN) / RESOLUTION_DEG) + 1
    c_space = np.zeros((t1_bins, t2_bins), dtype=int)
    for i in range(t1_bins):
        theta1 = THETA1_MIN + (i * RESOLUTION_DEG)
        for j in range(t2_bins):
            theta2 = THETA2_MIN + (j * RESOLUTION_DEG)
            if not is_configuration_valid(theta1, theta2, obstacles):
                c_space[i][j] = 1 
    return c_space

def pad_c_space(c_space_grid, pad_size=1):
    padded_grid = np.copy(c_space_grid)
    width, height = c_space_grid.shape
    for x in range(width):
        for y in range(height):
            if c_space_grid[x][y] == 1:
                for dx in range(-pad_size, pad_size + 1):
                    for dy in range(-pad_size, pad_size + 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            padded_grid[nx][ny] = 1
    return padded_grid

def angle_to_index(theta1, theta2):
    t1_idx = int((theta1 - THETA1_MIN) / RESOLUTION_DEG)
    t2_idx = int((theta2 - THETA2_MIN) / RESOLUTION_DEG)
    return (t1_idx, t2_idx)

def index_to_angle(t1_idx, t2_idx):
    theta1 = THETA1_MIN + (t1_idx * RESOLUTION_DEG)
    theta2 = THETA2_MIN + (t2_idx * RESOLUTION_DEG)
    return (theta1, theta2)

def wavefront_c_space(c_space_grid, start_idx, goal_idx):
    mapWidth = len(c_space_grid)
    mapHeight = len(c_space_grid[0])
    distances = np.full((mapWidth, mapHeight), 99999, dtype=int)
    seen = set()

    for x in range(mapWidth):
        for y in range(mapHeight):
            if c_space_grid[x][y] == 1:
                distances[x][y] = -1

    q = deque()
    q.append(goal_idx)
    seen.add(goal_idx)
    
    if 0 <= goal_idx[0] < mapWidth and 0 <= goal_idx[1] < mapHeight:
        if distances[goal_idx[0]][goal_idx[1]] == -1:
            print("ERROR: Goal is inside an obstacle!")
            return []
        distances[goal_idx[0]][goal_idx[1]] = 0
    else:
        return []

    dirs = [(0, 1), (1, 0), (-1, 0), (0, -1), (-1, -1), (1, 1), (-1, 1), (1, -1)]

    while q:
        x, y = q.popleft()
        currValue = distances[x][y]
        for dr, dc in dirs:
            newX, newY = x + dc, y + dr
            if 0 <= newX < mapWidth and 0 <= newY < mapHeight:
                if distances[newX][newY] == -1: 
                    continue
                if (newX, newY) not in seen:
                    seen.add((newX, newY))
                    distances[newX][newY] = currValue + 1
                    q.append((newX, newY))

    if distances[start_idx[0]][start_idx[1]] == 99999:
        return []

    x, y = start_idx
    returnPath = [(x, y)]

    while (x, y) != goal_idx:
        currentDist = distances[x][y]
        bestDist = currentDist
        bestMove = None
        for dr, dc in dirs:
            newX, newY = x + dc, y + dr
            if 0 <= newX < mapWidth and 0 <= newY < mapHeight:
                neighborDist = distances[newX][newY]
                if neighborDist != -1 and neighborDist < bestDist:
                    bestDist = neighborDist
                    bestMove = (newX, newY)
        if bestMove:
            x, y = bestMove
            returnPath.append((x, y))
        else:
            break
    return returnPath

# ==========================================
# 5. HARDWARE CONTROL 
# ==========================================
def setup_plink():
    plink = Plink()
    motor_base = plink.channel3  
    motor_elbow = plink.channel1 
    plink.connect()
    motor_base.control_mode = ControlMode.POWER
    motor_elbow.control_mode = ControlMode.POWER
    return plink, motor_base, motor_elbow


    

def move_to_angles(motor_base, motor_elbow, target_t1_deg, target_t2_deg, arm_state, tolerance_deg=2.0):
    print(f"Moving to: Theta1={target_t1_deg:.1f}°, Theta2={target_t2_deg:.1f}°")
    
    target_t1_deg -= 10
    #the bottom always overshoots since its heavy

    GEAR_RATIO_1 = (1/0.12)
    GEAR_RATIO_2 = 5
    
    Kp1 = 0.08
    Kd1 = 0.02 
    Kp2 = 0.03
    Kd2 = 0.01
    
    MIN_POWER_1 = 0.3
    MIN_POWER_2 = 0.3
    
    OFFSET_T1 = 44.24493414476984
    OFFSET_T2 = -85.38157389251214

    # 1. Load absolute state to prevent "Amnesia"
    prev_enc_t1 = arm_state['prev_t1']
    prev_enc_t2 = arm_state['prev_t2']
    accumulated_rad_t1 = arm_state['accum_t1']
    accumulated_rad_t2 = arm_state['accum_t2']
    
    prev_error_t1 = 0.0
    prev_error_t2 = 0.0

    settled_cycles = 0
    REQUIRED_SETTLE_CYCLES = 20 # Must stay on target for 20 loops (0.1 seconds)

    while True:
        cur_enc_t1 = motor_base.position 
        cur_enc_t2 = motor_elbow.position 
        
        delta_t1 = cur_enc_t1 - prev_enc_t1
        delta_t2 = cur_enc_t2 - prev_enc_t2
        
        # Handle Encoder Wrap
        if delta_t1 > math.pi: delta_t1 -= 2 * math.pi
        elif delta_t1 < -math.pi: delta_t1 += 2 * math.pi
        if delta_t2 > math.pi: delta_t2 -= 2 * math.pi
        elif delta_t2 < -math.pi: delta_t2 += 2 * math.pi
            
        accumulated_rad_t1 += delta_t1
        accumulated_rad_t2 += delta_t2
        
        prev_enc_t1 = cur_enc_t1
        prev_enc_t2 = cur_enc_t2
        
        # Convert true accumulated position to degrees
        current_t1_deg = (math.degrees(accumulated_rad_t1) / GEAR_RATIO_1) + OFFSET_T1
        current_t2_deg = (math.degrees(accumulated_rad_t2) / GEAR_RATIO_2) + OFFSET_T2
        print(f"curr1 {current_t1_deg}")
        print(f"curr2 {current_t2_deg}")
        
        error_t1 = target_t1_deg - current_t1_deg
        error_t2 = target_t2_deg - current_t2_deg
            
        # Only break if it proves it is physically stable
        if abs(error_t1) <= tolerance_deg and abs(error_t2) <= tolerance_deg:
            settled_cycles+=1
        else: settled_cycles = 0
        if settled_cycles >= REQUIRED_SETTLE_CYCLES:
            break

        # 2. Corrected Break Condition (Must wait for BOTH to finish)
            
        d_err_t1 = error_t1 - prev_error_t1
        d_err_t2 = error_t2 - prev_error_t2
        
        raw_power_t1 = (error_t1 * Kp1) + (d_err_t1 * Kd1)
        raw_power_t2 = (error_t2 * Kp2) + (d_err_t2 * Kd2)
        
        prev_error_t1 = error_t1
        prev_error_t2 = error_t2
        
        # 3. Independent Motor Braking
        # If a motor is within tolerance, kill its power so it doesn't jitter
        if abs(error_t1) > tolerance_deg:
            power_t1 = raw_power_t1 + (MIN_POWER_1 * (1 if raw_power_t1 > 0 else -1))
            power_t1 = max(min(power_t1, 0.4), -0.4)
        else:
            power_t1 = raw_power_t1

        if abs(error_t2) > tolerance_deg:
            power_t2 = raw_power_t2 + (MIN_POWER_2 * (1 if raw_power_t2 > 0 else -1))
            power_t2 = max(min(power_t2, 0.4), -0.4)
        else:
            power_t2 = raw_power_t2
        
        motor_base.power_command = -power_t1
        motor_elbow.power_command = -power_t2
        time.sleep(0.005)
        
    motor_base.power_command = 0
    motor_elbow.power_command = 0
    
    # Save absolute state for the next waypoint
    arm_state['prev_t1'] = prev_enc_t1
    arm_state['prev_t2'] = prev_enc_t2
    arm_state['accum_t1'] = accumulated_rad_t1
    arm_state['accum_t2'] = accumulated_rad_t2
        

def visualize_plan(c_space_grid, path_indices, start_angle, target_angle):
    """
    Plots the padded C-Space, overlays the path, and saves it to a PNG file.
    """
    print("\n>>> GENERATING PATH VISUALIZATION... <<<")
    
    # Configure the plot extents to match real-world degrees
    extent = [THETA2_MIN, THETA2_MAX, THETA1_MIN, THETA1_MAX]
    
    plt.figure(figsize=(10, 6))
    
    # Plot the padded C-Space background
    plt.imshow(c_space_grid, cmap='binary', origin='lower', extent=extent, aspect='auto')
    
    # Extract the physical angles from the path indices
    path_t1 = []
    path_t2 = []
    for idx in path_indices:
        t1_deg, t2_deg = index_to_angle(idx[0], idx[1])
        path_t1.append(t1_deg)
        path_t2.append(t2_deg)
        
    # Overlay the path line
    plt.plot(path_t2, path_t1, color='dodgerblue', linewidth=3, label='Wavefront Path')
    
    # Add clear markers for the Start and Goal
    plt.scatter([start_angle[1]], [start_angle[0]], color='limegreen', s=150, edgecolors='black', label='Start', zorder=5)
    plt.scatter([target_angle[1]], [target_angle[0]], color='crimson', s=150, edgecolors='black', label='Goal', zorder=5)
    
    plt.title(f"Pre-Flight Check: {start_angle} to {target_angle}")
    plt.ylabel("Theta 1 (Base Angle) in Degrees")
    plt.xlabel("Theta 2 (Elbow Angle) in Degrees")
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    
    # Save the figure to a file instead of trying to display it
    filename = "current_path_plan.png"
    plt.savefig(filename, dpi=300, bbox_inches="tight")
    print(f"✅ Success! Map saved to: {filename}")
    
    # Close the plot to free up memory
    plt.close()
    
    # Force the script to pause so you have time to open the image file
    input("Check the image file. Press ENTER when you are ready to execute the movement...")








# ==========================================
# 6. MAIN EXECUTION SEQUENCE
# ==========================================
if __name__ == "__main__":

    

    print("Connecting to Plink...")
    plink_board, base_motor, elbow_motor = setup_plink()

    # ---------------------------------------------------------
    # 2. INITIALIZE ARM STATE HERE
    # This takes a snapshot of the true physical starting positions
    # before any movement math happens.
    arm_state = {
        'prev_t1': 0,#base_motor.position,
        'prev_t2': 0,#elbow_motor.position,
        'accum_t1': 0.0,
        'accum_t2': 0.0
    }
    # ---------------------------------------------------------

    print("\nMapping Physical World...")
    obstacles = construct_obstacles()
    
    print("Generating C-Space Grid (Please wait)...")
    raw_c_space = generate_c_space(obstacles)
    
    print("Padding C-Space for Hardware Safety...")
    # 2 cells of padding = 10 degrees of buffer space from the blocks
    safe_c_space = pad_c_space(raw_c_space, pad_size=1) 
    
    # Define Demo Sequence
    waypoints_xy = [START_POINT, POINT_A, POINT_B, POINT_C]
    waypoints_angles = []
    
    print("\nCalculating Inverse Kinematics for Demo Points...")
    for x, y in waypoints_xy:
        solutions = inverse_kinematics(x, y)
        if not solutions:
            print(f"CRITICAL ERROR: Point ({x}, {y}) is completely unreachable! Exiting.")
            exit()
        
        # Select the first valid solution (Elbow Up)
        # We ensure it exists in our safe, padded C-Space
        valid_solution_found = False
        for sol in solutions:
            idx = angle_to_index(sol[0], sol[1])
            if safe_c_space[idx[0]][idx[1]] == 0:
                waypoints_angles.append(sol)
                valid_solution_found = True
                break
                
        if not valid_solution_found:
             print(f"CRITICAL ERROR: Point ({x}, {y}) is unreachable without colliding. Check padding/limits.")
             exit()

    print("\n*** ALL SYSTEMS GO. STARTING DEMO SEQUENCE ***")
    
    for i in range(len(waypoints_angles) - 1):
        start_angle = waypoints_angles[i]
        target_angle = waypoints_angles[i+1]
        
        print(f"\n--- Planning path from {waypoints_xy[i]} to {waypoints_xy[i+1]} ---")
        
        start_idx = angle_to_index(start_angle[0], start_angle[1])
        goal_idx = angle_to_index(target_angle[0], target_angle[1])
        
        path_indices = wavefront_c_space(safe_c_space, start_idx, goal_idx)
        
        if not path_indices:
            print("WAVEFRONT FAILED: No safe path found between these points!")
            break
            
        print(f"Path found! Executing {len(path_indices)} angular steps...")

        visualize_plan(safe_c_space, path_indices, start_angle, target_angle)
        
        for step_num, idx in enumerate(path_indices):
            t1_target, t2_target = index_to_angle(idx[0], idx[1])
            print(f"t1 target {t1_target} & t2 target {t2_target}")
            # Loose tolerance while tracing path, strict tolerance at the final point
            is_final_step = (step_num == len(path_indices) - 1)
            tolerance = 2.0 if is_final_step else 4.0 
            
            # ... inside your path_indices loop ...
            move_to_angles(base_motor, elbow_motor, t1_target, t2_target, arm_state, tolerance)
            
        print("Reached Waypoint. Holding for 3 seconds...")
        time.sleep(3) 
        
    print("\nDemo Complete!")




































from collections import deque

def wavefront(res, start, goal, obstaclesSet):
    # Adjust dimensions based on resolution
    # Note: Ensure these match your array indexing (Width vs Height)
    mapWidth = int(72 * res)
    mapHeight = int(54 * res)

    print("start:", start)
    print("goal:", goal)
    
    # Ensure start/goal are integers
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))

    # --- 1. SETUP ---
    # Initialize distances with a value larger than any possible path (infinity)
    # Using 99999 to be safe (300 might be too low if the map is high res)
    distances = [[9999] * mapHeight for _ in range(mapWidth)]
    seen = set()

    # Mark obstacles (Optional for BFS, but good for visualization/debugging)
    # We won't use this value for logic, we rely on the set
    for x, y in obstaclesSet:
        if 0 <= x < mapWidth and 0 <= y < mapHeight:
            distances[x][y] = -1 

    # --- 2. WAVEFRONT BFS (Goal -> Start) ---
    q = deque()
    q.append(goal)
    seen.add(goal)
    
    # Set goal distance to 0
    if 0 <= goal[0] < mapWidth and 0 <= goal[1] < mapHeight:
        distances[goal[0]][goal[1]] = 0
    else:
        print("Goal is out of bounds")
        return []

    dirs = [(0, 1), (1, 0), (-1, 0), (0, -1), (-1, -1), (1, 1), (-1, 1), (1, -1)]

    while q:
        x, y = q.popleft()
        currValue = distances[x][y]
        
        for dr, dc in dirs:
            newX, newY = x + dc, y + dr
            
            # Check Bounds
            if 0 <= newX < mapWidth and 0 <= newY < mapHeight:
                # Check Obstacles and Visited
                if (newX, newY) in obstaclesSet:
                    continue
                if (newX, newY) not in seen:
                    seen.add((newX, newY))
                    distances[newX][newY] = currValue + 1
                    q.append((newX, newY))

    # --- 3. PATH RECONSTRUCTION (Start -> Goal) ---
    
    # CHECK: Did the wave actually reach the start?
    if distances[start[0]][start[1]] == 99999:
        print("ERROR: No path exists from Start to Goal.")
        return []

    x, y = start
    returnPath = list()
    returnPath.append((x, y))

    # Gradient Descent
    while (x, y) != goal:
        currentDist = distances[x][y]
        bestDist = currentDist
        bestMove = None
        
        # Look for the neighbor with the lowest distance value
        for dr, dc in dirs:
            newX, newY = x + dc, y + dr
            
            if 0 <= newX < mapWidth and 0 <= newY < mapHeight:
                neighborDist = distances[newX][newY]
                
                # We look for a strictly lower value
                # We also ensure we don't walk into an obstacle (-1) or unvisited area (99999)
                if neighborDist != -1 and neighborDist < bestDist:
                    bestDist = neighborDist
                    bestMove = (newX, newY)

        if bestMove:
            x, y = bestMove
            returnPath.append((x, y))
        else:
            # If we are here, we are stuck in a local minimum or loop
            print(f"Stuck at {x}, {y} with distance {currentDist}")
            break

    return returnPath






























"""
File:           construct_map.py
Date:           2/12/2025
Description:    lab5 code for contructing a map from obstacle coordinates 
Author:         Joshua Samulak (jsamulak@andrew.cmu.edu)

"""

import math
import numpy
import matplotlib.pyplot as plt
import time


from path import wavefront

"""
#! IT IS HIGHLY RECOMMENDED YOU EDIT THIS FILE TO IMPLEMENT A CONFIGURATION SPACE
#! HOW YOU DO THAT IS FULLY UP TO YOU! :D

example code is at the bottom of the file
"""

class Line:
    # line defined by p1, p2; each point = (X, Y)
    # boundType:
    # ** information of what type of boundary is it? See valid inputs below
    # "T", "B", "R", "L" - only a Top, Bottom, Right, or Left line respectively
    # "TL" or "LT" - Top and Left line
    # "TR" or "RT" - Top and Right line
    # "BL" or "LB" - Bottom and Left line
    # "BR" or "RB" - Bottom and Right line
    def __init__(self, p1, p2, boundType):
        self.p1 = p1
        self.p2 = p2
        self.boundType = boundType
        pass
    
    # This function, given an (x, y) coordinate,
    # the line checks to see if, relative to itself, the said coordinate is
    # Above itself (in the case of it having boundType "B", ie bottom line)
    # Below itself (in the case of it having boundType "T", ie Top line)
    # Right of itself (in the case of it having boundType "L", ie Left line)
    # or Left of itself (in the case of it having boundType "R", ie Right line)
    def inLine(self, x, y):
        inLine = True
        y_on_line = 0 # gets changed
        x_on_line = 0 # gets changed

        # in the case of a vertical line, correct code shouldn't input "T" or "B"
        # so we don't care about "y_on_line"
        if (self.p1[0] == self.p2[0]):
            y_on_line = 0
            x_on_line = self.p1[0]
        # in the case of a horzontal line, correct code shouldn't input "R" or "L"
        # so we don't care about "x_on_line"
        elif (self.p1[1] == self.p2[1]):
            y_on_line = self.p1[1]
            x_on_line = 0
        # if not horizontal or vertical, then we do math to get relative xs and ys
        else:
            y_on_line = self.p1[1] + (self.p2[1] - self.p1[1]) * (x - self.p1[0]) / (self.p2[0] - self.p1[0])
            x_on_line = self.p1[0] + (self.p2[0] - self.p1[0]) * (y - self.p1[1]) / (self.p2[1] - self.p1[1])

        # using x_on_line and y_on_line, we can now easily check if the point is "within" the line
        for char in self.boundType:
            if char == 'T':
                inLine = inLine and (y < y_on_line)
            if char == 'B':
                inLine = inLine and (y > y_on_line)
            if char == 'R':
                inLine = inLine and (x < x_on_line)
            if char == 'L':
                inLine = inLine and (x > x_on_line)
            # print(char)
            # print(inLine)
        
        return inLine

class Obstacle:
    # all objects are of object type "Line"
    def __init__(self, l1, l2, l3, l4):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        pass
    
    # returns true if x,y is in the obstacle, false otherwise
    def clash(self, x, y):

        inBounds = True
        inBounds = inBounds and self.l1.inLine(x, y)
        inBounds = inBounds and self.l2.inLine(x, y)
        inBounds = inBounds and self.l3.inLine(x, y)
        inBounds = inBounds and self.l4.inLine(x, y)

        return inBounds

def construct_obstacles(isEasy=True):
    """
    Constructs the physical obstacles based on the Lab 9 field diagram.
    The origin (0,0) is at the center base of the robot.
    """
    
    obstacles = [
        # Left Obstacle (Red block from X=-4 to -2, Y=4 to 6)
        Obstacle(
            Line((-4, 4), (-2, 4), "B"),  # Bottom line: Left to Right
            Line((-2, 4), (-2, 6), "R"),  # Right line: Bottom to Top
            Line((-2, 6), (-4, 6), "T"),  # Top line: Right to Left
            Line((-4, 6), (-4, 4), "L")   # Left line: Top to Bottom
        ),
        
        # Right Obstacle (Red block from X=1 to 3, Y=5 to 7)
        Obstacle(
            Line((1, 5), (3, 5), "B"),    # Bottom line: Left to Right
            Line((3, 5), (3, 7), "R"),    # Right line: Bottom to Top
            Line((3, 7), (1, 7), "T"),    # Top line: Right to Left
            Line((1, 7), (1, 5), "L")     # Left line: Top to Bottom
        )
    ]

    return obstacles

# given a list of obstacles and a coordinate point,
# check if it is within the bounds of any obstacle
def check_obstacles(obstacles, x, y):
    for ob in obstacles:
        if ob.clash(x, y):
            return True
    return False

def drawPath(path,img):
    for x,y in path:
        for dr, dc in [(1,0),(0,1),(-1,0),(0,-1),(1,1),(-1,1),(-1,-1),(1,-1)]:
            newX, newY = x + dc, y + dr
            if 0 <= newX < len(img[0]) and 0 <= newY < len(img):
                img[newY][newX] = [255,255,255]
    return img

def construct_map(isEasy, resolution):

    obstacles = construct_obstacles(isEasy)
    obstaclesSet = set()

    # Discritize points and run through them:

    # @TODO Vary RESOLUTION as desired if
    # The larger it is, the more accurate your map will be,
    # but the longer it will take

    # RESOLUTION is the number of points you want per inch
    # (anything larger than 20 takes a while)
    RESOLUTION = resolution
    # WIDTH and HEIGHT in inches of the course
    MAP_WIDTH = 72
    MAP_HEIGHT = 54

    x_disc = MAP_WIDTH * RESOLUTION
    y_disc = MAP_HEIGHT * RESOLUTION
    
    # Discritized Image
    # numpy does y first, then x
    img = numpy.zeros((y_disc, x_disc, 3), dtype=numpy.uint8)

    for col in range(x_disc):
        for row in range(y_disc):
            hit = check_obstacles(obstacles, col/RESOLUTION, row/RESOLUTION)
            if hit: obstaclesSet.add((col,row))
            img[row][col] = [hit * 255, 0, 0]

    return img, obstaclesSet


# NOTE: Values in img are indexed (Y, X, color)
# this was to work with the plt functions,
# that for some reason start with y values, then x values (blame plt, not me)

# Example plt code using img result from construct_map:

def generatePath(isEasy, resolution, start, goal):
    startTime = time.time()
    img, obstaclesSet = construct_map(isEasy, resolution)
    elapsedTime = time.time() - startTime
    print(f"map constructed in {elapsedTime:.1f} seconds")


    #MAKES BUFFER
    bufferSize = 4.25 #this is the closest (in inches) that the robot will get to the obstacles
    #if it's running too close to the obstalces INCREASE THIS VALUE
    bufferSet = set()

    height = len(img)
    width = len(img[0])

# 1. Add all border pixels to a set
# This covers Top, Bottom, Left, and Right
    borderPixels = set()
    for r in range(width):
        borderPixels.add((r, 0))           # Left Edge
        borderPixels.add((r, height - 1))   # Right Edge
    for c in range(height):
        borderPixels.add((0, c))           # Top Edge
        borderPixels.add((width - 1, c))  # Bottom Edge

# 2. Combine obstacles and borders into one list to process
# We convert to a list so we can iterate over it easily
    allTargets = set(obstaclesSet).union(borderPixels)
    allTargets = list(allTargets)

    # 3. Apply the buffer expansion to EVERYTHING (Obstacles + Borders)
    for x,y in allTargets:
        # Ensure the hard obstacle/border itself is colored (optional, but good for vis)
        img[y][x] = [0, 0, 0] 

        for i in range(int(bufferSize * resolution)):
            for dr, dc in [(1,0),(0,1),(-1,0),(0,-1),(1,1),(-1,1),(-1,-1),(1,-1)]:
                newX, newY = x + dc*i, y + dr*i
                
                # CRITICAL: Check bounds so we don't crash when expanding off the map
                if 0 <= newX < width and 0 <= newY < height:
                    # If it's not a hard obstacle and not already buffered
                    if (newX, newY) not in obstaclesSet and (newX, newY) not in bufferSet:
                        bufferSet.add((newX, newY))
                        img[newY][newX] = [0, 0, 255] # Color the buffer blue


    for x,y in bufferSet:
        obstaclesSet.add((x,y))


    #EXHAUSTIVE TEST
    #TESTS EVERY POSSIBLE START AND GOAL
    '''for startX in range(54):
        for startY in range(72):
            for goalX in range(54):
                for goalY in range(72):
                    startTime = time.time()
                    start = start[0] * resolution, start[1] * resolution 
                    goal = goal[0] * resolution, goal[1] * resolution
                    if goal in obstaclesSet: 
                        continue
                    else:
                        path = wavefront(resolution,start,goal,obstaclesSet)
                        img = drawPath(path,img)

                        #plt.imshow(img, cmap=plt.cm.gray, origin='lower')
                        #plt.show()

                        elapsedTime = time.time() - startTime
                        print(f"done in {elapsedTime:.1f} seconds")'''

    start = start[0] * resolution, start[1] * resolution #EXPAND THE COORDINATES FROM INCHES TO PIXELS
    goal = goal[0] * resolution, goal[1] * resolution #EXPAND THE COORDINATES FROM INCHES TO PIXELS
    
    if goal in obstaclesSet: 
        print('Goal is in an obstacle. Aborting execution')
        return
    else:
        path = wavefront(resolution,start,goal,obstaclesSet)
        img = drawPath(path,img)

        plt.imshow(img, cmap=plt.cm.gray, origin='lower')
        #plt.show()

        plt.figure(figsize=(8,6))
        plt.imshow(img, origin='lower')
        #plt.axis("off")
        plt.savefig("map.png", dpi=300, bbox_inches="tight")
        #plt.close()
        print("saved map")
        elapsedTime = time.time() - startTime
        print(f"path created in {elapsedTime:.1f} seconds")
        return path































import math
import numpy as np
import matplotlib.pyplot as plt
import time

# Import the obstacle setup from your existing Lab 5 code
from construct_map import construct_obstacles

# --- ARM CONSTANTS ---
L1 = 4.0  # Length of link 1 in inches [cite: 20]
L2 = 4.5  # Length of link 2 in inches [cite: 23]

# Joint angle limits (in degrees)
THETA1_MIN = 0     # [cite: 18]
THETA1_MAX = 180   # [cite: 18]
THETA2_MIN = -180  # [cite: 25]
THETA2_MAX = 180   # [cite: 25]

RESOLUTION_DEG = 5 # 5-degree increments as specified in the lab 

# --- KINEMATICS & COLLISION ---

def forward_kinematics(theta1_deg, theta2_deg):
    """Calculates physical (x, y) coordinates of the elbow and end-effector."""
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    
    x_elbow = L1 * math.cos(t1)
    y_elbow = L1 * math.sin(t1)
    
    x_ee = x_elbow + L2 * math.cos(t1 + t2)
    y_ee = y_elbow + L2 * math.sin(t1 + t2)
    
    return ((x_elbow, y_elbow), (x_ee, y_ee))

def is_link_in_collision(x0, y0, x1, y1, obstacles, steps=15):
    """Samples points along a link to see if it hits any obstacle."""
    for i in range(steps + 1):
        t = i / steps
        sample_x = x0 + t * (x1 - x0)
        sample_y = y0 + t * (y1 - y0)
        
        # Check against every obstacle using your existing clash method
        for obs in obstacles:
            if obs.clash(sample_x, sample_y):
                return True 
    return False

def is_configuration_valid(theta1, theta2, obstacles):
    """Returns True if the arm is safe, False if it hits something."""
    (x_elbow, y_elbow), (x_ee, y_ee) = forward_kinematics(theta1, theta2)
    
    # 1. Table Clearance Check (Max 6 inches below origin) [cite: 49, 51]
    if y_elbow < -6.0 or y_ee < -6.0: 
        return False
        
    # 2. Link 1 Collision Check (Origin to Elbow)
    if is_link_in_collision(0, 0, x_elbow, y_elbow, obstacles): 
        return False
        
    # 3. Link 2 Collision Check (Elbow to End-Effector)
    if is_link_in_collision(x_elbow, y_elbow, x_ee, y_ee, obstacles): 
        return False
        
    return True

# --- C-SPACE GENERATION ---

def generate_c_space(obstacles):
    """
    Sweeps through all allowed angles and builds the C-space grid.
    Returns: A 2D numpy array (0 = free space, 1 = obstacle)
    """
    print("Generating C-Space... this might take a few seconds.")
    start_time = time.time()
    
    # Calculate grid dimensions based on limits and resolution
    t1_bins = int((THETA1_MAX - THETA1_MIN) / RESOLUTION_DEG) + 1
    t2_bins = int((THETA2_MAX - THETA2_MIN) / RESOLUTION_DEG) + 1
    
    # Initialize grid with zeros (assume free space)
    c_space = np.zeros((t1_bins, t2_bins), dtype=int)
    
    # Sweep through Theta 1
    for i in range(t1_bins):
        theta1 = THETA1_MIN + (i * RESOLUTION_DEG)
        
        # Sweep through Theta 2
        for j in range(t2_bins):
            theta2 = THETA2_MIN + (j * RESOLUTION_DEG)
            
            # If the configuration is a collision, mark it as a 1
            if not is_configuration_valid(theta1, theta2, obstacles):
                c_space[i][j] = 1 
                
    elapsed_time = time.time() - start_time
    print(f"C-Space generated in {elapsed_time:.2f} seconds.")
    
    return c_space, t1_bins, t2_bins

# --- EXECUTION & VISUALIZATION ---

if __name__ == "__main__":
    # 1. Load the obstacles using your existing function
    # Note: Pass 'True' for easy obstacles or 'False' for hard obstacles based on your construct_map.py
    obstacles = construct_obstacles(isEasy=True) 
    
    # 2. Generate the Map
    c_space_grid, t1_bins, t2_bins = generate_c_space(obstacles)
    
    # 3. Visualize the Map
    print("Plotting C-Space...")
    
    # Configure the plot extents so the axes show actual degrees, not array indices
    extent = [THETA2_MIN, THETA2_MAX, THETA1_MIN, THETA1_MAX]
    
    plt.figure(figsize=(10, 6))
    
    # We transpose the grid (.T) to put Theta 1 on the Y axis and Theta 2 on the X axis, 
    # which is a standard way to view a 2-DOF arm's C-space.
    plt.imshow(c_space_grid, cmap='binary', origin='lower', extent=extent, aspect='auto')
    
    plt.title("Robot Arm Configuration Space (C-Space)")
    plt.ylabel("Theta 1 (Base Angle) in Degrees")
    plt.xlabel("Theta 2 (Elbow Angle) in Degrees")
    
    # Add a colorbar for clarity (White = 0/Free, Black = 1/Obstacle)
    cbar = plt.colorbar(ticks=[0, 1])
    cbar.ax.set_yticklabels(['Free Space', 'Obstacle'])
    
    plt.grid(True, which='both', color='gray', linestyle='--', linewidth=0.5, alpha=0.5)
    
    plt.savefig("c_space_map.png", dpi=300, bbox_inches="tight")
    print("Saved C-Space image to 'c_space_map.png'")
    
    plt.show()
