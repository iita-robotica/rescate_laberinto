from controller import Robot
import math
import struct
import random
import numpy as np
import cv2 as cv

#### HOLA 

globalRot = 0
globalPos = [0, 0]
#tamano de nodo y de baldosa
nodeSize = 0.03

# Colores de baldosa
trap_colour = b'\n\n\n\xff'
swamp_colour = b'\x12\x1b \xff'
exit_colour = b'\x10\xb8\x10\xff'

#Define el timestep
timeStep = 32 * 1

#Velocidad maxima
max_velocity = 6.28

#Rotacion global
globalRotation = 0

messageSent = False

# Instanciacion de robot
robot = Robot()

# Ruedas
wheel_left = robot.getMotor("left wheel motor")
wheel_right = robot.getMotor("right wheel motor")

# Camara
colour_camera = robot.getCamera("colour_sensor")
colour_camera.enable(timeStep)

# emitter
emitter = robot.getEmitter("emitter")

# gps
gps = robot.getGPS("gps")
gps.enable(timeStep)

#Sensores de calor
left_heat_sensor = robot.getLightSensor("left_heat_sensor")
right_heat_sensor = robot.getLightSensor("right_heat_sensor")

left_heat_sensor.enable(timeStep)
right_heat_sensor.enable(timeStep)

# gyro
gyro = robot.getGyro("gyro")
gyro.enable(timeStep)

# Sensores de distancia
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

# Tiempo de inicio del programa

program_start = robot.getTime()

# Diametro del robot
robotDiameter = 0.071

moveOrderNumber = 0

movesDone = 0

def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def sendMessage(v1, v2, v3):
    message = struct.pack('i i i c', v1, v2, v3)
    emitter.send(message)
    print("Message sent")

def sendVictimMessage():
    global messageSent
    position = gps.getValues()

    if not messageSent:
        # robot type, position x cm, position z cm, victim type
        sendMessage(int(position[0] * 100), int(position[2] * 100), b'H')
        messageSent = True

    else:
        messageSent = False

def getDistance(position):
    return math.sqrt((position[0] ** 2) + (position[1] ** 2))

def getClosestVictim(victims):
    shortestDistance = 999
    closestVictim = []

    for victim in victims:
        dist = getObjectDistance(victim[0])
        if dist < shortestDistance:
            shortestDistance = dist
            closestVictim = victim

    return closestVictim


def stopAtVictim():
    global messageSent
    # get all the victims the camera can see
    victims = getVisibleVictims()

    foundVictim = False

    if len(victims) != 0:
        closest_victim = getClosestVictim(victims)
        turnToVictim(closest_victim)

    # if we are near a victim, stop and send a message to the supervisor
    for victim in victims:
        if nearObject(victim[0]):
            stop()
            sendVictimMessage()
            foundVictim = True

    if not foundVictim:
        messageSent = False


def avoidTiles():
    global duration, startTime
    colour = colour_camera.getImage()

    if colour == trap_colour or colour == swamp_colour:
        moveBackwards()
        startTime = robot.getTime()
        duration = 5


def turnRightToVictim():
    # set left wheel speed
    speeds[0] = 1 * max_velocity
    # set right wheel speed
    speeds[1] = 0.9 * max_velocity


def turnLeftToVictim():
    # set left wheel speed
    speeds[0] = 0.9 * max_velocity
    # set right wheel speed
    speeds[1] = 1 * max_velocity



def stop():
    # set left wheel speed
    speeds[0] = 0
    # set right wheel speed
    speeds[1] = 0


def move(fraction1, fraction2):
    # set left wheel speed
    speeds[0] = fraction1 * max_velocity
    # set right wheel speed
    speeds[1] = fraction2 * max_velocity


# Devuelve la rotacion global del robot teniendo en cuenta el cambio en su posicion
oldPos = [0, 0]

def getRotationByMovement():
    global oldPos
    if robot.step(timeStep * 5) != -1:
        newPos = [gps.getValues()[0] * 100 // 1, gps.getValues()[2] * 100 // 1]

        if newPos[0] == oldPos[0] and newPos[1] == oldPos[1]:
            return -1
        else:
            coords = [newPos[0] - oldPos[0], newPos[1] - oldPos[1]]
            rads = math.atan2(coords[0], coords[1])
            degs = rads * 180 / math.pi
            if degs < 0:
                degs = 180 - degs
            oldPos = newPos
            return degs

# Devuelve la velocidad angular con el gyro
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

def turnToAngle(x):
    global globalRot
    global moveOrderNumber
    global movesDone

    moveOrderNumber += 1

    if moveOrderNumber == movesDone + 1:
        diff = globalRot - x

        moveDiff = max(globalRot, x) - min(globalRot, x)

        print(diff)
        speedFract = min(mapVals(moveDiff, 0, 90, 0.2, 1), 1)
        if -1 < diff < 1:
            stop()
            movesDone += 1
            print("end")
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

def loopSequence():
    global movesDone
    movesDone = 0

def startSequence():
    global moveOrderNumber
    moveOrderNumber = 0


moveFlag = False
startPos = [0.0, 0.0]
def moveDistance(dist):
    global globalPos
    global moveFlag
    global startPos
    global moveOrderNumber
    global movesDone

    moveOrderNumber += 1

    if not moveFlag and moveOrderNumber == movesDone + 1:
        startPos = globalPos
        moveFlag = True
        # print("flag baja")
    elif moveFlag:
        diff = [max(globalPos[0], startPos[0]) - min(globalPos[0], startPos[0]), max(globalPos[1], startPos[1]) - min(globalPos[1], startPos[1])]
        diffDist = getDistance(diff)
        #print(diffDist)
        #print(dist)
        ratio = mapVals(diffDist, dist, 0, 0.4, 1)
        if (dist - 0.01) < diffDist < (dist + 0.01):
            stop()
            moveFlag = False
            movesDone += 1
            # print("listo!!")
        else:
            move(ratio, ratio)




def generateWall(center, size, lenght, orientation):
    finalList = []
    if orientation == "horizontal":
        node = []
        for i in range(- ((size -1) //2), (size +1) // 2):
            for j in range(lenght[0], lenght[1]):
                node.append(center + i)
                node.append(j)
                finalList.append(node)
                node = []

    elif orientation == "vertical":
        node = []
        for i in range(- ((size -1) //2), (size +1) // 2):
            for j in range(lenght[0], lenght[1]):
                node.append(j)
                node.append(center + i)

                finalList.append(node)
                node = []
    return finalList

# Returns all the nodes in a wall
def getEntireWall(globalPointPos):
    global nodeSize
    global robotDiameter

    robotNodeDiameter = (robotDiameter // nodeSize) + 1
    tileNodePos = [globalPointPos[0] % 0.3 // nodeSize, globalPointPos[1] % 0.3 // nodeSize]
    globalTileNodePos = [globalPointPos[0] // 0.3 * 10, globalPointPos[1] // 0.3 * 10]
    relativeNodePoints = []
    globalNodePoints = []

    direction = "None"

    if tileNodePos[0] == 9.0 and tileNodePos[1] > 3.0 and tileNodePos[1] < 8.0:
        direction = "up"
    elif tileNodePos[0] == 0.0 and tileNodePos[1] > 3.0 and tileNodePos[1] < 8.0:
        direction = "down"
    elif tileNodePos[1] == 0.0 and tileNodePos[0] > 3.0 and tileNodePos[0] < 8.0:
        direction = "left"
    elif tileNodePos[1] == 9.0 and tileNodePos[0] > 3.0 and tileNodePos[0] < 8.0:
        direction = "right"

    if direction == "up":
        relativeNodePoints = generateWall(10, 3, [-2, 12], "horizontal")
    elif direction == "down":
        relativeNodePoints = generateWall(0, 3, [-2, 12], "horizontal")
    elif direction == "left":
        relativeNodePoints = generateWall(0, 3, [-2, 12], "vertical")
    elif direction == "right":
        relativeNodePoints = generateWall(10, 3, [-2, 12], "vertical")

    if len(relativeNodePoints) > 0:
        for point in relativeNodePoints:
            globalNodePoints.append([point[0] + globalTileNodePos[0], point[1] + globalTileNodePos[1]])

    return globalNodePoints

def updateWheels():
    wheel_left.setVelocity(speeds[0])
    wheel_right.setVelocity(speeds[1])


while robot.step(timeStep) != -1:
    globalRot = getRotationByVelocity(globalRot)
    globalPos = gps.getValues()
    globalPos = [globalPos[0], globalPos[2]]

    # Empieza la sequencia
    startSequence()

    # funciones de rotacion y movimiento
    moveDistance(0.12)
    turnToAngle(270)
    moveDistance(0.12)
    turnToAngle(180)
    moveDistance(0.12)
    turnToAngle(90)
    moveDistance(0.12)
    if turnToAngle(0):
        # Empieza la sequencia de nuevo
        loopSequence()

    updateWheels()
