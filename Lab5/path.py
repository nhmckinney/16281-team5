#generate a path

from collections import deque

def wavefront(res,start,goal,obstaclesSet):

    mapWidth = 72 * res
    mapHeight = 54 * res

    print("start:", start)
    print("goal:", goal)
    print("mapHeight:", mapHeight)
    print("mapWidth:", mapWidth)

    x1, y1 = start
    x2, y2 = goal

    start = x1, y1
    goal = x2, y2
    
    seen = set()
    distances = [[0] * mapWidth for _ in range(mapHeight)]
    for x,y in obstaclesSet:
        distances[x][y] = 300

    L = list()
    q = deque(L)
    q.append(goal)
    seen.add(goal)

    dirs = [(0,1),(1,0),(-1,0),(0,-1),(-1,-1),(1,1),(-1,1),(1,-1)]
    #EIGHT POINT CONNECTIVITY

    #wavefront bfs
    while q:
        x,y = q.popleft()
        currValue = distances[x][y]
        for dr, dc in dirs:
            newX, newY = x + dr, y + dc
            if 0 > newX or newX >= mapHeight or 0 > newY or newY >= mapWidth:
                continue
            if (newX, newY) in obstaclesSet: #checks if it's an obstacle
                continue
            elif (newX, newY) not in seen:
                seen.add((newX,newY))
                distances[newX][newY] = currValue + 1
                q.append((newX,newY))

    #at this point we have found a path to the end.
    #now start at the beginning and repeatedly choose the lowest 
    # valued neighbor until you get to the end

    x,y = start
    returnPath = list()
    returnPath.append((x,y))

    largestNeighbor = distances[x][y]
    while (x,y) != goal:
        largestNeighbor = distances[x][y]
        for dr, dc in dirs:
            newX, newY = x + dr, y + dc
            if 0 > newX or newX >= mapHeight or 0 > newY or newY >= mapWidth:
                continue
            if distances[newX][newY] < largestNeighbor:
                largestNeighbor = distances[newX][newY]
                bestX = newX
                bestY = newY
        x, y = bestX, bestY
        returnPath.append((x,y))

    return returnPath
