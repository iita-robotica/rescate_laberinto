from controller import Robot
import math
import numpy as np
import cv2 as cv


# Defines time step
#Define el time step
timeStep = 32 * 1
# Maximum velocity
#Velocidad maxima
max_velocity = 6.28
#Global rotation
#Rotacion global
globalRotation = 0
# robot instanciation
# Instanciacion de robot
robot = Robot()
# wheels
# Ruedas
wheel_left = robot.getMotor("left wheel motor")
wheel_right = robot.getMotor("right wheel motor")
# Colour camera (Colour sensor)
# Camara de color
colour_camera = robot.getCamera("colour_sensor")
colour_camera.enable(timeStep)
# emitter
emitter = robot.getEmitter("emitter")
# gps
gps = robot.getGPS("gps")
gps.enable(timeStep)
# gyro
gyro = robot.getGyro("gyro")
gyro.enable(timeStep)
# Distance sensors
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
# Program start time
# Tiempo de inicio del programa
program_start = robot.getTime()
# Robot diameter
# Diametro del robot
robotDiameter = 0.071
# Tile size
tileSize = 0.12
# Node size
# tamano de nodo
nodeSize = 0.03
# Global rotation
globalRot = 0
#Global position
globalPos = [0, 0]

# Node grid class
# Clase de grilla de nodos
class NodeGrid:
    def __init__(self, x, y):
        self.grid = np.zeros((x, y), dtype=np.uint8)
        self.center = (self.grid.shape[0] // 2, self.grid.shape[1] // 2)

    def printMap(self):
        print(self.grid)

    def getMap(self):
        return self.grid

    def changeValue(self, pos, val):
        print(pos)
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


# Turns to specified angle
# Gira al angulo especificado
def turnToAngle(x):
    global globalRot
    diff = globalRot - x
    moveDiff = max(globalRot, x) - min(globalRot, x)
    #print(diff)
    speedFract = min(mapVals(moveDiff, 0, 90, 0.2, 1), 1)
    if -1 < diff < 1:
        stop()
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

# Converts a range of value to another
# convierte un rango de valores a otro
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Gets distance to coordinates
# Obtiene la distancia a unas coordenadas (Intruducir array con dos valores. por ej. [-0.12, 1,45])
def getDistance(position):
    return math.sqrt((position[0] ** 2) + (position[1] ** 2))

# Stops both wheels
# para las dos ruedas
def stop():
    # set left wheel speed
    speeds[0] = 0
    # set right wheel speed
    speeds[1] = 0


# Changes wheel speeds from a fraction of the max speed.
# cambia la velocidad de las ruedas teniendo en cuenta un fraccion de la velocidad. (Poner valores de 0 a 1, donde 1 es
# igual a la velocidad maxima )
def move(fraction1, fraction2):
    # set left wheel speed
    speeds[0] = fraction1 * max_velocity
    # set right wheel speed
    speeds[1] = fraction2 * max_velocity


# Gets the robot global rotation with the gyro values. Input: Robot global rotation
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

# Corrects the inputted angle
# Corrige el angulo introducido por si se pasa de 360 o es mas bajo que 0
def normalizeAngle(ang):
    ang = ang % 360
    # Si es mas bajo que 0 grados, le resta ese valor a 360
    if ang < 0:
        ang += 360
    return ang

# Updates the wheel speed (Put at the end of loop)
def updateWheels():
    wheel_left.setVelocity(speeds[0])
    wheel_right.setVelocity(speeds[1])

# Gets coordinates from an angle and a distance
def getCoords(angle, distance):
    #print(distance)
    rad = angle * math.pi / 180
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return [x, y]

# Gets the global position of the sensor detections. Input: Global robot rotation and the global robot position
def getSensorGlobalDetection(robotRot, robotPos):
    finalGlobalPoses = []
    sensorVals = [sensors[0].getValue(), sensors[1].getValue(), sensors[2].getValue(), sensors[3].getValue(),
                  sensors[4].getValue(), sensors[5].getValue(), sensors[6].getValue(), sensors[7].getValue()]
    index = 0
    offset = 0.01
    for val in sensorVals:
        if val < 0.8:
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

# Instantiates the NodeGrid to a map for demonstration purposes
demoMap = NodeGrid(200, 200)
# Size of the node for the demoMap
demoNodeSize = 0.012

#Main loop
while robot.step(timeStep) != -1:
    # Gets the robot global position
    globalPos = gps.getValues()
    globalPos = [globalPos[0], globalPos[2]]
    # Gets the robot global rotation
    globalRot = getRotationByVelocity(globalRot)
    # Stops the robot
    stop()
    # Gets the sensor detection global positions
    globalDetects = getSensorGlobalDetection(globalRot, globalPos)
    # print("GlobalRot = " + str(globalRot))
    # Turns 180 degrees
    turnToAngle(180)
    # For each detection it changes the corresponding node in the grid to white
    for detection in globalDetects:
        detectNode = [detection[0] // demoNodeSize, detection[1] // demoNodeSize]
        demoMap.changeValue([detectNode[1] * -1, detectNode[0] * -1], 255)
    # Shows the map
    cv.imshow("map", cv.resize(demoMap.getMap(), (600, 600), interpolation=cv.INTER_AREA))
    # Updates the wheel speeds
    updateWheels()
    cv.waitKey(1)
