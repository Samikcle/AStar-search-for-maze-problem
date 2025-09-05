import itertools

# coordinate representation col (q), row (r) = [q,r]

mapSize = [9,5] 

entry = [0,0]

goals = [[4,1],[3,4],[7,3],[9,3]]

obstacle = [[0,3],[2,2],[3,3],[4,2],[4,4],[6,3],[6,4],[7,4],[8,1]]

eventSpace = [[1,1,2],[3,1,4],[2,4,2],[5,3,3],[6,1,3],[8,2,1],[1,3,5],[4,0,5],[5,5,6],[7,2,6]]

class Node:
    def __init__(self, coordinate=None, goalRemaining=goals, parent=None, cost=1, gCost=0, hCost=0, eventActivated=[]):
        self.coordinate = coordinate
        self.goalRemaining = goalRemaining
        self.parent = parent
        self.cost = cost
        self.gCost = gCost
        self.hCost = hCost
        self.fCost = gCost + hCost
        self.eventActivated = eventActivated
        
def convertToCube(coordinate):
    q = coordinate[0]
    r = coordinate[1] - (coordinate[0] + (coordinate[0]&1)) // 2
    s = -q-r
    return [q,r,s]

def getDistance(start, end):
    startCube = convertToCube(start)
    endCube = convertToCube(end)
    return max(abs(startCube[0]-endCube[0]), abs(startCube[1]-endCube[1]), abs(startCube[2]-endCube[2]))

def heuristic(coordinate, goalsToCheck, cost, eventSpace, eventActivated):
    eventType, event = checkEvent(coordinate, eventSpace, eventActivated)
    goalRemaining = goalsToCheck[:]
    permutations = list(itertools.permutations(goalRemaining))
    
    shortestDistance = 1000
    for perm in permutations:
        distance = getDistance(coordinate, perm[0]) 
        
        for i in range(len(perm)-1):
            distance += getDistance(perm[i], perm[i + 1]) 
            
        if distance < shortestDistance:
            shortestDistance = distance
    
    match eventType:
        case 1:
            return (cost + (((shortestDistance-1)*cost)*2))
        case 2:
            return (cost + (((shortestDistance-1)*cost)/2))
        case 4:
            return 1000
    return shortestDistance*cost

def getNeighbour(coordinate, obstacle, mapSize):
    neighbours = []
    if coordinate[0] % 2 == 0:
        direction = [[+1, +1], [+1, 0], [0, -1], [-1, 0], [-1, +1], [0, +1]]
    else:
        direction = [[+1, 0], [+1, -1], [0, -1], [-1, -1], [-1, 0], [0, +1]]
        
    for q, r in direction:
        neighbour = [coordinate[0]+q,coordinate[1]+r]
        if not ((neighbour in obstacle) or (neighbour[0] > mapSize[0]) or (neighbour[0] < 0) or (neighbour[1] > mapSize[1]) or (neighbour[1] < 0)):
            neighbours.append(neighbour)
    return neighbours

def runEvent(node, eventSpace):
    eventType, event = checkEvent(node.coordinate, eventSpace, node.eventActivated)
    match eventType:
        case 1:
            node.eventActivated.append(event)
            node.cost *= 2
        case 2:
            node.eventActivated.append(event)
            node.cost /= 2
        case 3:
            node.eventActivated.append(event)
            backNode = Node(node.parent.parent.coordinate, node.goalRemaining[:], node, node.cost, node.gCost, heuristic(node.parent.parent.coordinate, node.goalRemaining, node.cost, eventSpace, node.eventActivated[:]), node.eventActivated[:])
            return backNode
    return node

def checkEvent(coordinate, eventSpace, eventActivated):
    for event in eventSpace:
        if event[0] == coordinate[0] and event[1] == coordinate[1] and event[2] == 4:
            return 4, None
        if coordinate[0] == event[0] and coordinate[1] == event[1]:
            if not (event in eventActivated):
                if event[2] == 3:
                    return 3, event
                elif not (event[2] in (e[2] for e in eventActivated)):
                    match event[2]:
                        case 1 | 2:
                            return 1, event
                        case 5 | 6:
                            return 2, event
    return 0, None

def checkGoal(node):
    if node.coordinate in node.goalRemaining:
        node.goalRemaining.remove(node.coordinate)
    return node

def appendAndSort(node, frontier):
    duplicated = False
    removed = False
    for i, f in enumerate(frontier):
        if node.coordinate == f.coordinate and node.goalRemaining == f.goalRemaining and node.eventActivated == f.eventActivated:
            duplicated = True
            if node.fCost < f.fCost:
                del frontier[i]
                removed = True
                break
    if (not duplicated) or removed:
        location = len(frontier)
        for i, f in enumerate(frontier):
            if node.fCost < f.fCost:
                location = i
                break
        frontier.insert(location, node)
    return frontier

def aStar(entry, goals, mapSize, obstacle, eventSpace):
    frontier = []
    explored = []
    foundGoal = False
    solutionPath = []
    goalNode = Node()
    frontier.append(Node(entry, goals[:], None, 1, 0, heuristic(entry,goals,1,eventSpace,[]),[]))

    while not foundGoal:
        current = frontier[0]
        current = runEvent(current, eventSpace[:])
        explored.append(current)
        del frontier[0]
        
        if current.goalRemaining == []:
            foundGoal = True
            goalNode = current
            break
        
        neighbours = getNeighbour(current.coordinate, obstacle, mapSize)
        
        for neighbour in neighbours:
            childHcost = heuristic(neighbour, current.goalRemaining, current.cost, eventSpace, current.eventActivated[:])
            childNode = Node(neighbour, current.goalRemaining[:], current, current.cost, current.gCost + (1*current.cost), childHcost, current.eventActivated[:])
            childNode = checkGoal(childNode)
            
            if not any(childNode.coordinate == e.coordinate and childNode.goalRemaining == e.goalRemaining and childNode.eventActivated == e.eventActivated for e in explored):
                frontier = appendAndSort(childNode, frontier)
                
        print("Explored: ", [(e.coordinate, e.goalRemaining, e.fCost) for e in explored])
        print("Frontier: ", [(f.coordinate, f.goalRemaining, f.fCost) for f in frontier])
        print("")
        
    traceNode = goalNode
    solutionPath = [goalNode]
    totalCost = goalNode.fCost
        
    while traceNode.parent is not None:
        solutionPath.insert(0, traceNode.parent)
        traceNode = traceNode.parent
        
    return solutionPath, totalCost
            
solutionPath, cost = aStar(entry, goals, mapSize, obstacle, eventSpace)

print("Solution Path:")
print()
print(f"Step | Coordinate | G Cost | H Cost | F Cost | Cost to move | {'Goals Remaining':^27} | {'Event Activated':^20}")
print("----------------------------------------------------------------------------------------------------------------")
step = 0
for node in solutionPath:
    goalRemaining = ",".join(map(str,node.goalRemaining))
    eventActivated = ",".join(str(coord[:-1]) for coord in node.eventActivated)
    print(f"{step:>3}. | {str(node.coordinate):>10} | {node.gCost:>6.2f} | {node.hCost:>6.2f} | {node.fCost:>6.2f} | {node.cost:>12.2f} | {goalRemaining:<27} | {eventActivated:<20}")

    step += 1

print()
print("Total Cost: ", cost)

