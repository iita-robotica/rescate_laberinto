from controller import Robot
import numpy as np
import cv2 as cv
import math
import struct

tileSize = 0.12

timeStep = 32 // 2
max_velocity = 6.28

startTime = 0
duration = 0

globalRotation = 0



robot = Robot()

wheel_left = robot.getMotor("left wheel motor")
wheel_right = robot.getMotor("right wheel motor")

centreCamera = robot.getCamera("camera_centre")
centreCamera.enable(timeStep)
rightCamera = robot.getCamera("camera_right")
rightCamera.enable(timeStep)
leftCamera = robot.getCamera("camera_left")
leftCamera.enable(timeStep)

colour_camera = robot.getCamera("colour_sensor")
colour_camera.enable(timeStep)

emitter = robot.getEmitter("emitter")

gps = robot.getGPS("gps")
gps.enable(timeStep)

left_heat_sensor = robot.getLightSensor("left_heat_sensor")
right_heat_sensor = robot.getLightSensor("right_heat_sensor")

left_heat_sensor.enable(timeStep)
right_heat_sensor.enable(timeStep)

gyro = robot.getGyro("gyro")
gyro.enable(timeStep)

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

program_start = robot.getTime()

robotDiameter = 0.071

globalRot = 0
globalPos = [0, 0]


class NodeGrid:
    def __init__(self, x, y):
        self.grid = np.zeros((x, y), dtype=np.uint8)
        self.center = (self.grid.shape[0] // 2, self.grid.shape[1] // 2)

        for i in range(0, len(self.grid)):
            if i % 2 != 0:
                for j in range(0, len(self.grid[i])):
                    if j % 2 != 0:
                        self.grid[i][j] = 255

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

    def printMap(self):
        print(self.grid)

    def getMap(self):
        return self.grid

    def changeValue(self, pos, val):
        # print(pos)
        try:
            finalIndex = [pos[0] + self.center[0], pos[1] + self.center[1]]
            self.grid[int(finalIndex[0]), int(finalIndex[1])] = val
            if finalIndex[0] < 0 or finalIndex[1] < 0:
                return "undefined"
        except IndexError:
            return "undefined"

    def getValue(self, pos):
        try:
            finalIndex = [pos[0] + self.center[0], pos[1] + self.center[1]]

            if finalIndex[0] < 0 or finalIndex[1] < 0:
                return "undefined"
            else:
                return self.grid[int(finalIndex[0]), int(finalIndex[1])]
        except IndexError:
            return "undefined"


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # print("initating A star")

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # print("Initiating While loop")

    # Loop until you find the end
    while len(open_list) > 0:
        # print("start of while loop")
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

        # Found the goal
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


def bfs(grid, node, objectives, victimColor):
    visited = []  # List to keep track of visited nodes.
    queue = []  # Initialize a queue
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
            if grid[neighbour[0]][neighbour[1]] > 240:
                continue
            visited.append(neighbour)
            queue.append(neighbour)
    return found, victims


messageSent = False


# Sends a message to the game controller
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


# Gira al angulo especificado
def turnToAngle(x):
    global globalRot

    diff = globalRot - x

    moveDiff = max(globalRot, x) - min(globalRot, x)

    # print(diff)
    speedFract = min(mapVals(moveDiff, 0, 90, 0.2, 1), 0.7)
    if -1 < diff < 1:
        stop()
        # print("end")
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


# convierte un rango de valores a otro
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# Obtiene la distancia a unas coordenadas (Intruducir array con dos valores. por ej. [-0.12, 1,45])
def getDistance(position):
    return math.sqrt((position[0] ** 2) + (position[1] ** 2))


# para las dos ruedas
def stop():
    # set left wheel speed
    speeds[0] = 0
    # set right wheel speed
    speeds[1] = 0


# cambia la velocidad de las ruedas teniendo en cuenta un fraccion de la velocidad. (Poner valores de 0 a 1, donde 1 es
# igual a la velocidad maxima )
def move(fraction1, fraction2):
    # set left wheel speed
    speeds[0] = fraction1 * max_velocity
    # set right wheel speed
    speeds[1] = fraction2 * max_velocity


# Poner esta funcion antes de hacer una secuencia de movimientos
def startSequence():
    global moveOrderNumber
    global movesDone
    moveOrderNumber = 0
    movesDone = 0


# Mueve el robot a las coordenadas especificadas
turned = False


def moveToCoords(targetPos):
    global turned
    global globalPos
    global globalRot
    # Nivel de presicion. Se puede hacer mas chico o mas grande, pero puede ser que si es demasiado chico el robot nunca
    # llegue a su destino. Segun mis pruevas funciona bastante bien con este valor
    errorMargin = 0.01
    diffX = targetPos[0] - globalPos[0]
    diffY = targetPos[1] - globalPos[1]
    dist = math.sqrt(diffX ** 2 + diffY ** 2)
    if errorMargin * -1 < dist < errorMargin:
        print("done")
        stop()
        turned = False
        return True
    else:
        rad = math.atan2(diffX, diffY)
        ang = rad * 180 / math.pi
        ang += 180
        ang = normalizeAngle(ang)
        # print(ang)
        if not turned:
            turned = turnToAngle(ang)
        if turned:
            # print(dist)
            ratio = min(mapVals(dist, 0, 0.1, 0.1, 1), 0.8)
            ratio = max(ratio, 0.2)
            diff = globalRot - ang
            if diff > 180:
                diff = 360 - diff
                diff * -1
            if -1 < diff < 1:
                move(ratio, ratio)
            else:
                # print("turning to angle")
                turnToAngle(ang)

oldPos = [0, 0]
#Returns the robot global rotation
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


# Devuelve la rotacion global del robot teniendo en cuenta el gyroscopo
oldRotTime = robot.getTime()


def getRotationByVelocity(globalRotation):
    global oldRotTime
    result = globalRotation
    newRotTime = robot.getTime()
    timeElapsed = newRotTime - oldRotTime  # tiempo que paso en cada timeStep
    radsInTimestep = (gyro.getValues())[0] * timeElapsed
    degsInTimestep = radsInTimestep * 180 / math.pi
    # print("rads: " + str(radsInTimestep) + " | degs: " + str(degsInTimestep))
    result += degsInTimestep
    # Si se pasa de 360 grados se ajusta la rotacion empezando desde 0 grados
    result = result % 360

    # Si es mas bajo que 0 grados, le resta ese valor a 360
    if result < 0:
        result += 360
    # print("global rotation: " + str(globalRotation))
    # print("tiempo: " + str(timeElapsed))
    # Actualiza el tiempo de rotacion
    oldRotTime = newRotTime
    return result


# Corrige el angulo introducido por si se pasa de 360 o es mas bajo que 0
def normalizeAngle(ang):
    ang = ang % 360
    # Si es mas bajo que 0 grados, le resta ese valor a 360
    if ang < 0:
        ang += 360
    return ang


def updateWheels():
    wheel_left.setVelocity(speeds[0])
    wheel_right.setVelocity(speeds[1])


def getCoords(angle, distance):
    # print(distance)
    rad = angle * math.pi / 180
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return [x, y]


def getDetectTile(position, offsets):
    global tileSize
    node = [int((position[0] + offsets[0]) // tileSize), int((position[1] + offsets[1]) // tileSize)]
    return node


def getTile(robotGlobalPos, offsets):
    global tileSize
    node = [(robotGlobalPos[1] + offsets[1]) // tileSize * 2, (robotGlobalPos[0] + offsets[0]) // tileSize * 2]
    return node


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


def getWalls(positions):
    global tileSize
    finalTiles = []
    finalOrientations = []
    orientation = ""
    sideClearance = 0.036
    centreClearance = 0.036
    thicknessClearance = 0.004

    for pos in positions:
        posTile = getDetectTile(pos, [0, 0])
        tilePos = [posTile[0] * tileSize, posTile[1] * tileSize]
        posInTile = [pos[0] + tilePos[0] * -1 + 0, pos[1] + tilePos[1] * -1 + 0]

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


def followPath(path):
    global movementsDone
    global recalculate
    global robotTile
    global nodeGrid
    global tileSize

    if movementsDone < len(path):
        coords = [path[movementsDone][0], path[movementsDone][1]]
        startSequence()
        if moveToCoords(coords):
            movementsDone += 1
            return False, True
        else:
            return False, False
    else:
        return True, True


def getAstar(start, end):
    global movementsDone
    if nodeGrid.getValue([end[0] - nodeGrid.center[0], end[1] - nodeGrid.center[1]]) != 200:
        nodeGrid.changeValue([end[0] - nodeGrid.center[0], end[1] - nodeGrid.center[1]], 100)
    path = astar(maze, start, (end[0], end[1]))
    finalPath = []
    movementsDone = 2
    for pos in path:
        finalPos = [(pos[1] - nodeGrid.center[0]) // 2, (pos[0] - nodeGrid.center[1]) // 2]
        finalPos = [(finalPos[0] * tileSize) - 0.06, (finalPos[1] * tileSize) - 0.06]
        finalPath.append(finalPos)
    return finalPath


def typeOfFloor():
    colour = colour_camera.getImage()
    r = colour[0]
    g = colour[1]
    b = colour[2]
    floorType = "undefined"  # 0=normal 1=swamp 2=trap 3=exit
    # print(str(r) + " - " + str(g) + " - " + str(b))
    if r == 252 and g == 252:
        floorType = "normal"
    elif (57 < r < 61 and 57 < g < 61) or (r == 111 and g == 111):
        floorType = "trap"
    elif 144 > r > 140 and 225 > g > 220 and b == 246:
        floorType = "swamp"
    elif r == 255 and g == 255 and b == 255:
        floorType = "checkpoint"
    return floorType


def doCalibration():
    global masterOffset


def getVictimImagePos(image_data, camera):
    img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
    img[:, :, 2] = np.zeros([img.shape[0], img.shape[1]])
    # print(img.shape)
    cv.imshow("ventana", img)

    # convert from BGR to HSV color space
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # apply threshold
    thresh = cv.threshold(gray, 140, 255, cv.THRESH_BINARY)[1]

    # draw all contours in green and accepted ones in red
    contours, h = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    finalPoses = []

    for c in contours:
        M = cv.moments(c)

        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            finalPoses.append([cx, cy])
    return finalPoses


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

    # print(diff)
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



def getVictimTile(victimPoses, clearance, sensor=None):
    global globalPos
    global globalRot

    if len(victimPoses):
        victimTiles =[]
        for i in victimPoses:
            X = i[0]

            if (128 // 2) - clearance < X < (128 // 2) + clearance:
                #print("victim in centre")

                if sensor is None:
                    return victimTiles, True
                else:

                    detect = get1SensorGlobalDetecion(sensor, globalRot, globalPos)
                    if detect is not None:
                        victimCoords = detect
                        victimCoords = [victimCoords[0] * -1, victimCoords[1] * -1]
                        victimTile = getTile(victimCoords, [tileSize, tileSize])
                        #print(nodeGrid.getValue(victimTile))
                        victimTiles.append(victimTile)
        if len(victimTiles):
            return victimTiles, True
        else:
            return victimTiles, False
    else:
        return [], False

nodeGrid = NodeGrid(40, 40)

masterOffset = [0, 0]
oldPath = []
oldNode = [0, 0]
end = [0, 0]
posNumber = 1
movementsDone = 1
path = []
finalPath = []
finalNewPath = []
notMoving = True
inPossibleTiles = False
did360 = False
turn = 0
stopDelay = 0
waiting = False
startOfDelay = 0
victims = []
onStart = True
ending = False
overWrite = False
moved = True

# AT START
movedForward = False
movedBack = False
moving = False


while robot.step(timeStep) != -1:



    globalPos = gps.getValues()
    globalPos = [globalPos[0], globalPos[2]]
    globalRot = getRotationByVelocity(globalRot)
    robotTile = getTile(globalPos, [tileSize, tileSize])

    if nodeGrid.getValue(robotTile) not in (200, 255, 70, 120):
        nodeGrid.changeValue(robotTile, 50)
    cv.imshow("map", cv.resize(nodeGrid.getMap(), (400, 400), interpolation=cv.INTER_AREA))
    if onStart:
        print("----- Program started ----")
        startTile = robotTile
        #stop()
        #updateWheels()
        #print("Robot position in tile: ")
        #print(globalPos[0] % tileSize, globalPos[1] % tileSize)
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
                    print(globalRot)
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
                    onStart = False


        #onStart = False


    else:
        globalDetects = getSensorGlobalDetection(globalRot, globalPos)
        maze = nodeGrid.getMap()

        end = robotTile

        colour = typeOfFloor()
        colourSensorPos = getCoords(globalRot, robotDiameter / 2 + 0.01)
        colourSensorPos = [(colourSensorPos[0] * -1 + globalPos[0]), (colourSensorPos[1] * -1 + globalPos[1])]
        # print(colourSensorPos)
        colourTile = getTile(colourSensorPos, [tileSize, tileSize])

        leftImage = leftCamera.getImage()
        victimPosesInLeftImage = getVictimImagePos(leftImage, leftCamera)

        rightImage = rightCamera.getImage()
        victimPosesInRightImage = getVictimImagePos(rightImage, rightCamera)

        centreImage = centreCamera.getImage()
        victimPosesInCentreImage = getVictimImagePos(centreImage, centreCamera)

        if len(victims):
            end = victims[0]

        _, centreDetected = getVictimTile(victimPosesInCentreImage, 2)

        if centreDetected and sensors[0].getValue() < 0.12 and sensors[7].getValue() < 0.12 and nodeGrid.getValue(robotTile) != 120:
            stop()
            stopDelay = 3.2
            sendVictimMessage("H")
            messageSent = False
            print("---- Victim documented ----")
            nodeGrid.changeValue(robotTile, 120)


        leftVictimTiles, leftDetected = getVictimTile(victimPosesInLeftImage, 2, 7)
        if leftDetected:
            leftVictimTile = leftVictimTiles[0]
            if nodeGrid.getValue(leftVictimTile) == 0:
                stop()
                stopDelay = 0.5
                nodeGrid.changeValue(leftVictimTile, 200)

        rightVictimTiles, rightDetected = getVictimTile(victimPosesInLeftImage, 2, 0)
        if rightDetected:
            rightVictimTile = rightVictimTiles[0]
            if nodeGrid.getValue(rightVictimTile) == 0:
                stop()
                stopDelay = 0.5
                nodeGrid.changeValue(rightVictimTile, 200)

        if (leftDetected or rightDetected) and nodeGrid.getValue(robotTile) == 120:
            did360 = True
            #did180 = False

        if nodeGrid.getValue(robotTile) != 120:
            # print(sensors[7].getValue())

            if leftDetected and sensors[7].getValue() < 0.1:
                stop()
                stopDelay = 3.2
                sendVictimMessage("H")
                messageSent = False
                print("---- Victim documented ----")
                nodeGrid.changeValue(robotTile, 120)
            if rightDetected and sensors[0].getValue() < 0.1:
                stop()
                stopDelay = 3.2
                print("---- Victim documented ----")
                sendVictimMessage("H")
                messageSent = False
                nodeGrid.changeValue(robotTile, 120)

        if left_heat_sensor.getValue() > 32 or right_heat_sensor.getValue() > 32:
            print("heat detecting")
            stop()
            stopDelay = 3.2
            sendVictimMessage('T')
            messageSent = False

        # print(colour)

        start = (int(robotTile[0] + nodeGrid.center[0]), int(robotTile[1] + nodeGrid.center[1]))
        end = (int(end[0] + nodeGrid.center[0]), int(end[1] + nodeGrid.center[1]))
        wallTiles, wallDirections = getWalls(globalDetects)
        # possibleWalls = []
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
            if (nodeGrid.getValue(finalWallTile)) != 255:
                nodeGrid.changeValue([finalWallTile[0], finalWallTile[1]], 255)
                if did360:
                    possibleEnds, victims = bfs(maze, start, [0, 100], 200)
                    if robot.getTime() > 7.5 * 60:
                        end = startTile
                        end = (int(end[0] + nodeGrid.center[0]), int(end[1] + nodeGrid.center[1]))
                        ending = True
                    elif len(victims) and victims[0][2] < 25:
                            end = victims[0]
                    elif end not in possibleEnds:
                        if len(possibleEnds):
                            end = possibleEnds[0]
                        else:
                            end = startTile
                            end = (int(end[0] + nodeGrid.center[0]), int(end[1] + nodeGrid.center[1]))
                            ending = True
                            print("ending")
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


            if done:

                if left_heat_sensor.getValue() > 32 or right_heat_sensor.getValue() > 32:
                    print("heat detecting")
                    stop()
                    stopDelay = 3.2
                    sendVictimMessage('T')
                    messageSent = False

                if centreDetected and sensors[0].getValue() < 0.12 and sensors[7].getValue() < 0.12:
                    stop()
                    stopDelay = 3.2
                    sendVictimMessage("H")
                    messageSent = False
                    print("---- Victim documented ----")
                    nodeGrid.changeValue(robotTile, 120)
                # print(done)
                if nodeGrid.getValue(robotTile) == 200:
                    #print(sensors[7].getValue())

                    if leftDetected and sensors[7].getValue() < 0.1:
                        stop()
                        stopDelay = 3.2
                        sendVictimMessage("H")
                        messageSent = False
                        print("---- Victim documented ----")
                        nodeGrid.changeValue(robotTile, 120)
                    if rightDetected and sensors[0].getValue() < 0.1:
                        stop()
                        stopDelay = 3.2
                        print("---- Victim documented ----")
                        sendVictimMessage("H")
                        messageSent = False
                        nodeGrid.changeValue(robotTile, 120)


                    elif do360(0.3):

                        nodeGrid.changeValue(robotTile, 50)


                else:

                    possibleEnds, victims = bfs(maze, start, [0, 100], 200)
                    if robot.getTime() > 7.5 * 60:
                        end = (int(end[0] + nodeGrid.center[0]), int(end[1] + nodeGrid.center[1]))
                        end = startTile
                        ending = True

                    elif len(victims) and victims[0][2] < 25:
                        end = victims[0]
                    elif end not in possibleEnds:

                        if len(possibleEnds):
                            end = possibleEnds[0]
                        else:
                            end = startTile
                            end = (int(end[0] + nodeGrid.center[0]), int(end[1] + nodeGrid.center[1]))
                            ending = True
                    finalPath = getAstar(start, end)
            oldNode = robotTile
            oldPath = path

            if colour == "trap":
                nodeGrid.changeValue(colourTile, 255)
                possibleEnds, victims = bfs(maze, start, [0, 100], 200)

                overWrite = True

                if len(victims) and victims[0][2] < 25:
                    end = victims[0]
                elif len(possibleEnds):
                    end = possibleEnds[0]
                else:
                    end = startTile
                    end = (int(end[0] + nodeGrid.center[0]), int(end[1] + nodeGrid.center[1]))
                    ending = True
                finalPath = getAstar(start, end)

        else:
            did360 = do360(0.3)


        if ending and robotTile == startTile:
            sendMessage(0, 0, 'E')


        # do360(0.3)
        updateWheels()

        if cv.waitKey(1) == 'q':
            cv.destroyAllWindows()