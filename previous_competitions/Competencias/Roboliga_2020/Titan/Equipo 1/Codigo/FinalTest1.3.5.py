from controller import Robot
import numpy as np
import cv2 as cv
import math
import struct
from random import choice

# Show map variable. Turn True to see the visual representation of the map done by the robot
showMap = True

# Constants
timeStep = 32 // 2
max_velocity = 6.28
robotDiameter = 0.071
tileSize = 0.12

# Robot instantiation and enabling
robot = Robot()
# Wheels
wheel_left = robot.getMotor("left wheel motor")
wheel_right = robot.getMotor("right wheel motor")
# Cameras
centreCamera = robot.getCamera("camera_centre")
centreCamera.enable(timeStep)
rightCamera = robot.getCamera("camera_right")
rightCamera.enable(timeStep)
leftCamera = robot.getCamera("camera_left")
leftCamera.enable(timeStep)
# Colour sensor
colour_camera = robot.getCamera("colour_sensor")
colour_camera.enable(timeStep)
# Emitter
emitter = robot.getEmitter("emitter")
# Gps
gps = robot.getGPS("gps")
gps.enable(timeStep)
# Heat sensors
left_heat_sensor = robot.getLightSensor("left_heat_sensor")
right_heat_sensor = robot.getLightSensor("right_heat_sensor")
left_heat_sensor.enable(timeStep)
right_heat_sensor.enable(timeStep)
# Gyro
gyro = robot.getGyro("gyro")
gyro.enable(timeStep)
# Distance sensors
sensors = []
sensors.append(robot.getDistanceSensor("ps0"))
sensors[0].enable(timeStep)
sensors.append(robot.getDistanceSensor("ps1"))
sensors[1].enable(timeStep)
sensors.append(robot.getDistanceSensor("ps2"))
sensors[2].enable(timeStep)
sensors.append(robot.getDistanceSensor("ps3"))
sensors[3].enable(timeStep)
sensors.append(robot.getDistanceSensor("ps4"))
sensors[4].enable(timeStep)
sensors.append(robot.getDistanceSensor("ps5"))
sensors[5].enable(timeStep)
sensors.append(robot.getDistanceSensor("ps6"))
sensors[6].enable(timeStep)
sensors.append(robot.getDistanceSensor("ps7"))
sensors[7].enable(timeStep)
#        [left wheel speed, right wheel speed]
speeds = [0.0, 0.0]
wheel_left.setPosition(float("inf"))
wheel_right.setPosition(float("inf"))

# Initiate global position and rotation
globalRot = 0
globalPos = [0, 0]


# Node grid class for mapping
class NodeGrid:
    # Creates grid and prepares it
    def __init__(self, x, y):
        self.grid = np.zeros((x, y), dtype=np.uint8)
        self.center = (self.grid.shape[0] // 2, self.grid.shape[1] // 2)

        for i in range(0, len(self.grid)):
            if i % 2 != 0:
                for j in range(0, len(self.grid[i])):
                    if j % 2 != 0:
                        self.grid[i][j] = 255

        # Empty walls
        for i in range(0, len(self.grid)):
            if (i + 1) % 2 != 0:
                for j in range(0, len(self.grid[i])):
                    if (j + 0) % 2 != 0:
                        self.grid[i][j] = 20
        for i in range(0, len(self.grid)):
            if (i + 0) % 2 != 0:
                for j in range(0, len(self.grid[i])):
                    if (j + 1) % 2 != 0:
                        self.grid[i][j] = 20

    # Prints the grid in the console
    def printMap(self):
        print(self.grid)

    # Returns the grid
    def getMap(self):
        return self.grid

    # Changes the value of a given node in the grid
    def changeValue(self, pos, val):
        # print(pos)
        try:
            finalIndex = [pos[0] + self.center[0], pos[1] + self.center[1]]
            self.grid[int(finalIndex[0]), int(finalIndex[1])] = val
            if finalIndex[0] < 0 or finalIndex[1] < 0:
                return "undefined"
        except IndexError:
            return "undefined"

    # Gets the value of a given node of the grid
    def getValue(self, pos):
        try:
            finalIndex = [pos[0] + self.center[0], pos[1] + self.center[1]]

            if finalIndex[0] < 0 or finalIndex[1] < 0:
                return "undefined"
            else:
                return self.grid[int(finalIndex[0]), int(finalIndex[1])]
        except IndexError:
            return "undefined"


# Node class for A* pathfinding (Not to be confused with the node grid)
class Node():

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


# A Star algorithm
# Returns a list of tuples as a path from the given start to the given end in the given maze
def astar(maze, start, end):
    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0
    # Initialize open and closed list
    open_list = []
    closed_list = []
    # Add the start node
    open_list.append(start_node)
    # Loop until end
    while len(open_list) > 0:
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)
        # If found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path
        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Adjacent squares
            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (
                    len(maze[len(maze) - 1]) - 1) or node_position[1] < 0:
                continue
            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] == 255:
                continue
            # Create new node
            new_node = Node(current_node, node_position)
            # Append
            children.append(new_node)
        # Loop through children
        for child in children:
            continueLoop = False
            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continueLoop = True
                    break
            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                        (child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h
            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continueLoop = True
                    break
            if continueLoop:
                continue
            # Add the child to the open list
            open_list.append(child)


# Breath First Search algorithm
# Returns the tiles with the color given in objectives in order and with the distance of each one
def bfs(grid, node, objectives, victimColor):
    visited = []
    queue = []
    found = []
    victims = []
    node = [node[0], node[1], 0]
    visited.append(node)
    queue.append(node)
    while queue:
        coords = queue.pop(0)
        x = coords[0]
        y = coords[1]
        dist = coords[2]
        if grid[x][y] in objectives:
            found.append(coords)
        elif grid[x][y] == victimColor:
            victims.append(coords)
        for newPosition in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            neighbour = [x + newPosition[0], y + newPosition[1], dist + 1]
            inList = False
            for node in visited:
                if node[0] == neighbour[0] and node[1] == neighbour[1]:
                    inList = True
                    break
            if inList:
                continue
            if neighbour[0] > (len(grid) - 1) or neighbour[0] < 0 or neighbour[1] > (len(grid[len(grid) - 1]) - 1) or \
                    neighbour[1] < 0:
                continue
            # Make sure walkable terrain
            # I used 240 instead of 255  because of some discrepancies from 255
            if grid[neighbour[0]][neighbour[1]] > 240:
                continue
            visited.append(neighbour)
            queue.append(neighbour)
    return found, victims

# Sends a message to the game controller
messageSent = False
def sendMessage(v1, v2, victimType):
    message = struct.pack('i i c', v1, v2, victimType.encode())
    emitter.send(message)


# Sents a message of the game controller that a victim (of a certain type) has been detected
def sendVictimMessage(victimType='N'):
    global messageSent
    position = gps.getValues()
    if not messageSent:
        # robot type, position x cm, position z cm, victim type
        # The victim type is hardcoded as "H", but this should be changed to different victims for your program
        # Harmed = "H"
        # Stable = "S"
        # Unharmed = "U"
        # Heated (Temperature) = "T"
        sendMessage(int(position[0] * 100), int(position[2] * 100), victimType)
        messageSent = True


# Turns to given global angle
def turnToAngle(x):
    global globalRot
    diff = globalRot - x
    moveDiff = max(globalRot, x) - min(globalRot, x)
    speedFract = min(mapVals(moveDiff, 0, 90, 0.2, 1), 0.7)
    if -1 < diff < 1:
        stop()
        return True
    elif 0 < diff < 180:
        move(speedFract, speedFract * -1)
    elif diff < -180:
        move(speedFract, speedFract * -1)
    elif 0 > diff:
        move(speedFract * -1, speedFract)
    else:
        move(speedFract * -1, speedFract)
    return False


# Converts a number from a range of value to another
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# Gets the distance to given coordinates
def getDistance(position):
    return math.sqrt((position[0] ** 2) + (position[1] ** 2))


# Stops both wheels
def stop():
    speeds[0] = 0
    speeds[1] = 0


# Changes the wheel velocity given a ratio of the max velocity (Enter values from 0 to 1,  1 = max speed )
def move(fraction1, fraction2):
    speeds[0] = fraction1 * max_velocity
    speeds[1] = fraction2 * max_velocity


# Moves the robot to given global coordinates
turned = False
def moveToCoords(targetPos):
    global turned
    global globalPos
    global globalRot
    # level of precision. It can be modified, but if it is too small, the robot will never get to the position.
    errorMargin = 0.01
    diffX = targetPos[0] - globalPos[0]
    diffY = targetPos[1] - globalPos[1]
    dist = math.sqrt(diffX ** 2 + diffY ** 2)
    if errorMargin * -1 < dist < errorMargin:
        stop()
        turned = False
        return True
    else:
        rad = math.atan2(diffX, diffY)
        ang = rad * 180 / math.pi
        ang += 180
        ang = normalizeAngle(ang)
        if not turned:
            turned = turnToAngle(ang)
        if turned:
            ratio = min(mapVals(dist, 0, 0.1, 0.1, 1), 0.8)
            ratio = max(ratio, 0.2)
            diff = globalRot - ang
            if diff > 180:
                diff = 360 - diff
                diff * -1
            if -1 < diff < 1:
                move(ratio, ratio)
            else:
                turnToAngle(ang)


# Returns the robot global rotation by measuring the changes in its position
oldPos = [0, 0]
def getRotationByPos():
    global oldPos
    global gps
    newPos = [gps.getValues()[0] * 100 // 1, gps.getValues()[2] * 100 // 1]
    if newPos[0] == oldPos[0] and newPos[1] == oldPos[1]:
        return -1
    else:
        coords = [newPos[0] - oldPos[0], newPos[1] - oldPos[1]]
        rads = math.atan2(coords[0], coords[1])
        degs = rads * 180 / math.pi
        degs = normalizeAngle(degs)
        oldPos = newPos
        return degs


# Retuns the robot global rotation measuring the gyro
oldRotTime = robot.getTime()
def getRotationByVelocity(globalRotation):
    global oldRotTime
    result = globalRotation
    newRotTime = robot.getTime()
    timeElapsed = newRotTime - oldRotTime  # Time passed in time step
    radsInTimestep = (gyro.getValues())[0] * timeElapsed
    degsInTimestep = radsInTimestep * 180 / math.pi
    result += degsInTimestep
    result = normalizeAngle(result)
    oldRotTime = newRotTime
    return result


# Corrects the given angle to be in a range from 0 to 360
def normalizeAngle(ang):
    ang = ang % 360
    if ang < 0:
        ang += 360
    return ang


# Updates the wheel speeds
def updateWheels():
    wheel_left.setVelocity(speeds[0])
    wheel_right.setVelocity(speeds[1])


# Gets x, y coordinates from a given angle and distance
def getCoords(angle, distance):
    # print(distance)
    rad = angle * math.pi / 180
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return [x, y]


# Gets the tile of the position in the actual maze
def getDetectTile(position, offsets):
    global tileSize
    node = [int((position[0] + offsets[0]) // tileSize), int((position[1] + offsets[1]) // tileSize)]
    return node

# gets the tile of the position in the node grid
def getTile(robotGlobalPos, offsets):
    global tileSize
    node = [(robotGlobalPos[1] + offsets[1]) // tileSize * 2, (robotGlobalPos[0] + offsets[0]) // tileSize * 2]
    return node


# Gets the global postion of all the sensor detections
def getSensorGlobalDetection(robotRot, robotPos):
    finalGlobalPoses = []
    sensorVals = [sensors[0].getValue(), sensors[1].getValue(), sensors[2].getValue(), sensors[3].getValue(),
                  sensors[4].getValue(), sensors[5].getValue(), sensors[6].getValue(), sensors[7].getValue()]
    index = 0
    offset = 0.01
    for val in sensorVals:
        if val < 0.4:
            dist = mapVals(val, 0.0, 8, 0, tileSize * 2.4 * 10)

            dist += robotDiameter / 2
            dist += offset
            if index == 0:
                sensorAngle = 73 + 90
            elif index == 1:
                sensorAngle = 44 + 90
            elif index == 2:
                sensorAngle = 0 + 90
            elif index == 3:
                sensorAngle = 298 + 90
            elif index == 4:
                sensorAngle = 241 + 90
            elif index == 5:
                sensorAngle = 180 + 90
            elif index == 6:
                sensorAngle = 136 + 90
            elif index == 7:
                sensorAngle = 107 + 90
            else:
                print("somethin weird goin on")
            sensorAngle = normalizeAngle(sensorAngle)
            globalDetectionAngle = robotRot + sensorAngle
            globalDetectionAngle = normalizeAngle(globalDetectionAngle)
            globalDetectionAngle += 180
            globalDetectionAngle = normalizeAngle(globalDetectionAngle)
            pos = getCoords(globalDetectionAngle, dist)
            pos[0] -= robotPos[0]
            pos[1] -= robotPos[1]
            finalGlobalPoses.append(pos)
        index += 1
    return finalGlobalPoses


# Gets the global position of the detection of the given distance sensor
def get1SensorGlobalDetecion(index, robotRot, robotPos):
    sensorVal = sensors[index].getValue()
    offset = 0.01
    if sensorVal < 0.8:
        dist = mapVals(sensorVal, 0.00, 0.8, 0, tileSize * 2.4 - 0.01)
        dist += robotDiameter / 2
        dist += offset
        if index == 0:
            sensorAngle = 73 + 90
        elif index == 1:
            sensorAngle = 44 + 90
        elif index == 2:
            sensorAngle = 0 + 90
        elif index == 3:
            sensorAngle = 298 + 90
        elif index == 4:
            sensorAngle = 241 + 90
        elif index == 5:
            sensorAngle = 180 + 90
        elif index == 6:
            sensorAngle = 136 + 90
        elif index == 7:
            sensorAngle = 107 + 90
        else:
            print("somethin weird goin on")
        sensorAngle = normalizeAngle(sensorAngle)
        globalDetectionAngle = robotRot + sensorAngle
        globalDetectionAngle = normalizeAngle(globalDetectionAngle)
        globalDetectionAngle += 180
        globalDetectionAngle = normalizeAngle(globalDetectionAngle)
        pos = getCoords(globalDetectionAngle, dist)
        pos[0] -= robotPos[0]
        pos[1] -= robotPos[1]
    else:
        pos = None
    return pos


# Gets the walls and obstacles given the global positions
def getWalls(positions):
    global tileSize
    global masterOffset
    finalTiles = []
    finalOrientations = []
    sideClearance = 0.036
    centreClearance = 0.04
    thicknessClearance = 0.004
    for pos in positions:
        posTile = getDetectTile(pos, [0.06 - masterOffset[1], 0.06 - masterOffset[1]])
        tilePos = [posTile[0] * tileSize, posTile[1] * tileSize]
        posInTile = [pos[0] + tilePos[0] * -1 + (0.06 - masterOffset[0]), pos[1] + tilePos[1] * -1 + (0.06 - masterOffset[1])]
        if sideClearance < posInTile[1] < 0.12 - sideClearance and posInTile[0] > tileSize - thicknessClearance:
            orientation = "Left"
            finalTiles.append(posTile)
            finalOrientations.append(orientation)
        elif sideClearance < posInTile[1] < 0.12 - sideClearance and posInTile[0] < thicknessClearance:
            orientation = "Right"
            finalTiles.append(posTile)
            finalOrientations.append(orientation)
        elif sideClearance < posInTile[0] < 0.12 - sideClearance and posInTile[1] > tileSize - thicknessClearance:
            orientation = "Up"
            finalTiles.append(posTile)
            finalOrientations.append(orientation)
        elif sideClearance < posInTile[0] < 0.12 - sideClearance and posInTile[1] < thicknessClearance:
            orientation = "Down"
            finalTiles.append(posTile)
            finalOrientations.append(orientation)
        elif centreClearance < posInTile[0] < 0.12 - centreClearance and centreClearance < posInTile[
            1] < 0.12 - centreClearance:
            orientation = "centre"
            finalTiles.append(posTile)
            finalOrientations.append(orientation)
    return finalTiles, finalOrientations


# Makes the robot follow a series of global position
def followPath(path):
    global movementsDone
    global recalculate
    global robotTile
    global nodeGrid
    global tileSize
    if movementsDone < len(path):
        coords = [path[movementsDone][0], path[movementsDone][1]]
        if moveToCoords(coords):
            movementsDone += 1
            return False, True
        else:
            return False, False
    else:
        return True, True


# Gets the A* path and process it to meake it more usefull
def getAstar(start, end):
    global movementsDone
    global masterOffset
    if nodeGrid.getValue([end[0] - nodeGrid.center[0], end[1] - nodeGrid.center[1]]) != 200:
        nodeGrid.changeValue([end[0] - nodeGrid.center[0], end[1] - nodeGrid.center[1]], 100)
    path = astar(maze, start, (end[0], end[1]))
    finalPath = []
    movementsDone = 2
    for pos in path:
        finalPos = [(pos[1] - nodeGrid.center[0]) // 2, (pos[0] - nodeGrid.center[1]) // 2]
        finalPos = [(finalPos[0] * tileSize) - masterOffset[0], (finalPos[1] * tileSize) - masterOffset[1]]
        finalPath.append(finalPos)
    return finalPath


# Gets the type of tile with the color sensor data
def typeOfFloor():
    colour = colour_camera.getImage()
    r = colour[0]
    g = colour[1]
    b = colour[2]
    floorType = "undefined"
    if r == 252 and g == 252:
        floorType = "normal"
    elif (57 < r < 61 and 57 < g < 61) or (r == 111 and g == 111):
        floorType = "trap"
    elif 144 > r > 140 and 225 > g > 220 and b == 246:
        floorType = "swamp"
    elif r == 255 and g == 255 and b == 255:
        floorType = "checkpoint"
    return floorType


# Returns the position of a victim in the image
def getVictimImagePos(image_data, camera):
    img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
    img[:, :, 2] = np.zeros([img.shape[0], img.shape[1]])
    # Convert from BGR to Gray scale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Apply threshold
    thresh = cv.threshold(gray, 140, 255, cv.THRESH_BINARY)[1]
    # Apply Canny
    canny = cv.Canny(thresh, 0, 100)
    # Gets the lines in the image
    rho = 1
    theta = np.pi / 180
    threshold = 20
    min_line_length = 2
    max_line_gap = 10
    lines = cv.HoughLinesP(canny, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)
    if lines is not None:
        lineCount = len(lines)
    else:
        lineCount = 0
    contours, h = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    finalPoses = []
    for c in contours:
        # mGets the center of the contours
        M = cv.moments(c)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            finalPoses.append([cx, cy])
    return finalPoses, lineCount


# Does a 360 degree turn given the maximum velocity it can reach during that from 0 to 1
did180 = False
putObjective = False
objective = 0
def do360(maxratio):
    global globalRot
    global did180
    global objective
    global putObjective
    if not putObjective:
        objective = globalRot + 180
        objective = normalizeAngle(objective)
        putObjective = True
    diff = globalRot - objective
    moveDiff = max(globalRot, objective) - min(globalRot, objective)
    speedFract = min(mapVals(moveDiff, 0, 90, 0.2, 1), maxratio)
    if -1 < diff < 1:
        stop()
        # print("end")
        if did180:
            did180 = False
            return True
        else:
            did180 = True
            putObjective = False
    else:
        move(speedFract, speedFract * -1)
    return False


# Returns the victim tile and if it is in the centre of the camera given its position in the image, a clearance and the
# sensor used to detect distance
def getVictimTile(victimPoses, clearance, sensor=None):
    global globalPos
    global globalRot
    global masterOffset
    if len(victimPoses):
        victimTiles =[]
        for i in victimPoses:
            X = i[0]
            if (128 // 2) - clearance < X < (128 // 2) + clearance:
                if sensor is None:
                    return victimTiles, True
                else:
                    detect = get1SensorGlobalDetecion(sensor, globalRot, globalPos)
                    if detect is not None:
                        victimCoords = detect
                        victimCoords = [victimCoords[0] * -1, victimCoords[1] * -1]
                        victimTile = getTile(victimCoords, [tileSize / 2 + masterOffset[0], tileSize / 2 + masterOffset[1]])
                        victimTiles.append(victimTile)
        if len(victimTiles):
            return victimTiles, True
        else:
            return victimTiles, False
    else:
        return [], False

phrases = {
    "victim": ["Found a victim!", "Yup, victim right here", "Yess, victim", "I Think I see a victim here", "Good, victim!", "I really hope this is a victim"],
    "trap": ["Ups, go back!", "Nope, trap here", "Uff, almost fell", "Not this time trap"],
    "obstacle": ["Obstacle round there", "Found u obstacle", "Oh, obstacle, I see you there", "i´ll have to go around that obstacle"],
    "exit": ["I think I´ve seen everything, I better exit", ]
}
def say(situation):

    print("Robot0: " + "- " + choice(phrases[situation]) + " - ")


# Instantiates node grid
nodeGrid = NodeGrid(40, 40)

# Main loop variables
masterOffset = [0, 0]

oldNode = [0, 0]
end = [0, 0]
posNumber = 1
movementsDone = 1
turn = 0
finalPath = []
victims = []
notMoving = True
inPossibleTiles = False
did360 = False
ending = False
moved = True
heatAlreadyDetected = False

# Detecting victim variables
movedForwardAfterVictim = False
MFAVFirstTime = True
waiting = False

# At start variables
movedForward = False
movedBack = False
moving = False
firstTime = True

# All modes variables
overWrite = False
mode = "onStart"
startOfDelay = 0
stopDelay = 0

while robot.step(timeStep) != -1:
    globalPos = gps.getValues()
    globalPos = [globalPos[0], globalPos[2]]
    globalRot = getRotationByVelocity(globalRot)
    robotTile = getTile(globalPos, [tileSize / 2 + masterOffset[0], tileSize / 2 + masterOffset[1]])
    if nodeGrid.getValue(robotTile) not in (200, 255, 70, 120, 30):
        nodeGrid.changeValue(robotTile, 50)

    if showMap:
        cv.imshow("map", cv.resize(nodeGrid.getMap(), (800, 800), interpolation=cv.INTER_AREA))
    colour = typeOfFloor()
    colourSensorPos = getCoords(globalRot, robotDiameter / 2 + 0.01)
    colourSensorPos = [(colourSensorPos[0] * -1 + globalPos[0]), (colourSensorPos[1] * -1 + globalPos[1])]
    colourTile = getTile(colourSensorPos, [tileSize / 2 + masterOffset[0], tileSize / 2 + masterOffset[1]])


    if mode == "onStart":
        startTile = robotTile
        if firstTime:
            print("----- Program started ----")
            print("Robot position in tile: ")
            programStartTime = robot.getTime()
            masterOffset = [round(globalPos[0] * 100) / 100, round(globalPos[1] * 100) / 100]
            masterOffset = [masterOffset[0] % tileSize, masterOffset[1] % tileSize]
            robotTile = getTile(globalPos, [tileSize / 2 + masterOffset[0], tileSize / 2 + masterOffset[1]])
            firstTime = False
            print(masterOffset)
        if not movedForward:
            if not moving:
                FMStarting = robot.getTime()
                moving = True
            else:
                FMTimeElapsed = robot.getTime() - FMStarting
                move(0.8, 0.8)
                updateWheels()
                if FMTimeElapsed < 0.5:
                    val = getRotationByPos()
                    if val != -1:
                        globalRot = val
                if FMTimeElapsed > 1:
                    stop()
                    updateWheels()
                    moving = False
                    movedForward = True
        else:
            if not moving:
                FMStarting = robot.getTime()
                moving = True
            else:
                FMTimeElapsed = robot.getTime() - FMStarting
                move(-0.8, -0.8)
                updateWheels()
                if FMTimeElapsed > 1:
                    stop()
                    updateWheels()
                    globalRot += 180
                    globalRot = normalizeAngle(globalRot)
                    mode = "mainLoop"


    elif mode == "mainLoop":
        globalDetects = getSensorGlobalDetection(globalRot, globalPos)
        maze = nodeGrid.getMap()
        end = robotTile

        leftImage = leftCamera.getImage()
        victimPosesInLeftImage, leftLineCount = getVictimImagePos(leftImage, leftCamera)
        rightImage = rightCamera.getImage()
        victimPosesInRightImage, rightLineCount = getVictimImagePos(rightImage, rightCamera)
        centreImage = centreCamera.getImage()
        victimPosesInCentreImage, centreLineCount = getVictimImagePos(centreImage, centreCamera)

        if 4 in (leftLineCount, rightLineCount, centreLineCount) or 5 in (leftLineCount, rightLineCount, centreLineCount)\
                or 6 in (leftLineCount, rightLineCount, centreLineCount):
            victimInSight = "U"
        elif 8 in (leftLineCount, rightLineCount, centreLineCount) or 9 in (leftLineCount, rightLineCount, centreLineCount):
            victimInSight = "H"
        elif leftLineCount != 0 or rightLineCount != 0 or centreLineCount != 0:
            victimInSight = "S"
        else:
            victimInSight = "N"

        if len(victims) and victims[0][2] < 20:
            end = victims[0]

        _, centreDetected = getVictimTile(victimPosesInCentreImage, 20)

        rightVictimTiles, rightDetected = getVictimTile(victimPosesInLeftImage, 2, 0)
        leftVictimTiles, leftDetected = getVictimTile(victimPosesInLeftImage, 2, 7)
        if leftDetected:
            leftVictimTile = leftVictimTiles[0]
            if nodeGrid.getValue(leftVictimTile) == 0:
                stop()
                stopDelay = 0.5
                nodeGrid.changeValue(leftVictimTile, 200)

        if rightDetected:
            rightVictimTile = rightVictimTiles[0]
            if nodeGrid.getValue(rightVictimTile) == 0:
                stop()
                stopDelay = 0.5
                nodeGrid.changeValue(rightVictimTile, 200)

        if (leftDetected or rightDetected) and nodeGrid.getValue(robotTile) == 120:
            did360 = True
        if nodeGrid.getValue(robotTile) != 120:
            if (leftDetected or leftLineCount != 0) and sensors[7].getValue() < 0.1:
                if not overWrite:
                    mode = "detectingVictim"
            if (rightDetected or rightLineCount != 0) and sensors[0].getValue() < 0.1:
                if not overWrite:
                    mode = "detectingVictim"
            if (centreDetected or centreLineCount != 0) and sensors[0].getValue() < 0.15 and sensors[7].getValue() < 0.15:
                if not overWrite:
                    mode = "detectingVictim"
        if left_heat_sensor.getValue() > 35 or right_heat_sensor.getValue() > 35:
            if not heatAlreadyDetected:
                say("victim")
                heatAlreadyDetected = True
            stop()
            stopDelay = 3.2
            sendVictimMessage('T')
            messageSent = False

        else:
            heatAlreadyDetected = False

        start = (int(robotTile[0] + nodeGrid.center[0]), int(robotTile[1] + nodeGrid.center[1]))
        end = (int(end[0] + nodeGrid.center[0]), int(end[1] + nodeGrid.center[1]))
        wallTiles, wallDirections = getWalls(globalDetects)
        for wallTile, wallDirection in zip(wallTiles, wallDirections):
            finalWallTile = [wallTile[1] * -2, wallTile[0] * -2]
            if wallDirection == "Right":
                finalWallTile[1] += 1
            elif wallDirection == "Left":
                finalWallTile[1] -= 1
            elif wallDirection == "Up":
                finalWallTile[0] -= 1
            elif wallDirection == "Down":
                finalWallTile[0] += 1
            if nodeGrid.getValue(finalWallTile) not in (255, 30):
                if wallDirection == "centre":
                    for pos in ([0,1], [1,0], [0,-1], [-1,0]):
                        nodeGrid.changeValue([finalWallTile[0] + pos[0], finalWallTile[1] + pos[1]], 30)
                    say("obstacle")
                nodeGrid.changeValue([finalWallTile[0], finalWallTile[1]], 255)
                if did360:
                    possibleEnds, victims = bfs(maze, start, [0, 100], 200)
                    if robot.getTime() - programStartTime > 7.5 * 60:
                        say("exit")
                        end = startTile
                        end = (int(end[0] + nodeGrid.center[0]), int(end[1] + nodeGrid.center[1]))
                        ending = True
                    elif len(victims) and victims[0][2] < 20:
                        end = victims[0]
                    elif end not in possibleEnds:
                        if len(possibleEnds):
                            end = possibleEnds[0]
                        else:
                            say("exit")
                            end = startTile
                            end = (int(end[0] + nodeGrid.center[0]), int(end[1] + nodeGrid.center[1]))
                            ending = True
                    finalPath = getAstar(start, end)

        if stopDelay:
            if not waiting:
                waiting = True
                startOfDelay = robot.getTime()
            else:
                actualTime = robot.getTime()
                if actualTime - startOfDelay > stopDelay:
                    stopDelay = 0
                    waiting = False

        elif did360:
            done, moved = followPath(finalPath)

            if moved and nodeGrid.getValue(robotTile) != 120:
                nodeGrid.changeValue(robotTile, 30)

            if done:
                if nodeGrid.getValue(robotTile) == 200:
                    if leftDetected and sensors[7].getValue() < 0.1:
                        if not overWrite:
                            mode = "detectingVictim"
                    if rightDetected and sensors[0].getValue() < 0.1:
                        if not overWrite:
                            mode = "detectingVictim"
                    if (centreDetected or centreLineCount != 0) and sensors[0].getValue() < 0.15 and sensors[
                        7].getValue() < 0.15:
                        if not overWrite:
                            mode = "detectingVictim"

                    elif do360(0.3):
                        nodeGrid.changeValue(robotTile, 50)
                else:
                    possibleEnds, victims = bfs(maze, start, [0, 100], 200)
                    if robot.getTime() - programStartTime > 7.5 * 60:
                        say("exit")
                        end = startTile
                        end = (int(end[0] + nodeGrid.center[0]), int(end[1] + nodeGrid.center[1]))
                        ending = True
                    elif len(victims) and victims[0][2] < 20:
                        end = victims[0]
                    elif end not in possibleEnds:
                        if len(possibleEnds):
                            end = possibleEnds[0]
                        else:
                            say("exit")
                            end = startTile
                            end = (int(end[0] + nodeGrid.center[0]), int(end[1] + nodeGrid.center[1]))
                            ending = True
                    finalPath = getAstar(start, end)
            oldNode = robotTile
            if colour == "trap":
                overWrite = True
                if nodeGrid.getValue(colourTile) != 255:
                    say("trap")
                    nodeGrid.changeValue(colourTile, 255)
                    possibleEnds, victims = bfs(maze, start, [0, 100], 200)
                    if len(victims) and victims[0][2] < 20:

                        end = victims[0]
                    elif len(possibleEnds):
                        end = possibleEnds[0]
                    else:
                        say("exit")
                        end = startTile
                        end = (int(end[0] + nodeGrid.center[0]), int(end[1] + nodeGrid.center[1]))
                        ending = True
                    finalPath = getAstar(start, end)
            else:
                overWrite = False
        else:
            did360 = do360(0.3)
        if ending and robotTile == startTile:
            sendMessage(0, 0, 'E')
        updateWheels()

    elif mode == "detectingVictim":
        if colour == "trap":
            say("trap")
            MFAVMoveBackST = robot.getTime()
            MFAVMoveBack = True
            mode = "mainLoop"
        if MFAVFirstTime:
            say("victim")
            MFAVStartTime = robot.getTime()
            MFAVFirstTime = False
            MFAVMoveBack = False
        else:
            if stopDelay:
                if not waiting:
                    waiting = True
                    startOfDelay = robot.getTime()
                else:
                    actualTime = robot.getTime()
                    if actualTime - startOfDelay > stopDelay:
                        stopDelay = 0
                        waiting = False

            elif robot.getTime() - MFAVStartTime > 1.5 and not MFAVMoveBack:
                stop()
                stopDelay = 3
                sendVictimMessage(victimInSight)
                messageSent = False
                nodeGrid.changeValue(robotTile, 120)
                MFAVMoveBack = True
                MFAVMoveBackST = robot.getTime()
            elif MFAVMoveBack:
                if robot.getTime() - MFAVMoveBackST < 1.5 + 3:
                    move(-0.2, -0.2)
                else:
                    stop()
                    MFAVFirstTime = True
                    mode = "mainLoop"
            else:
                move(0.4, 0.4)


    updateWheels()
    cv.waitKey(1)
