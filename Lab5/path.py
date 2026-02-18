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
