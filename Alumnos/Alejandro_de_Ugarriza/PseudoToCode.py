from controller import Robot
import numpy as np
import struct
import math

timeStep = 32 // 2

# Corrects the given angle to be in a range from 0 to 360
def normalizeAngle(ang):
    ang = ang % 360
    if ang < 0:
        ang += 360
    return ang

# Converts a number from a range of value to another
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Gets x, y coordinates from a given angle and distance
def getCoords(angle, distance):
    rad = angle * math.pi / 180
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return [x, y]

# Gets the distance to given coordinates
def getDistance(position):
    return math.sqrt((position[0] ** 2) + (position[1] ** 2))

class DistanceSensor:
    def __init__(self, sensor, sensorAngle, robotDiameter, timeStep):
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.offset = -1
        self.robotDiameter = robotDiameter
        self.angle = sensorAngle
    def getDistance(self):
        val = self.sensor.getValue()
        dist = mapVals(val, 0.0, 0.8, 0, 12 * 2.4)
        dist += self.robotDiameter / 2
        dist += self.offset
        return dist
    def getAngle(self, globalRotation):
        return normalizeAngle(self.angle + globalRotation + 270)
    def getGlobalDetection(self, globalRotation, robotPos):
        pos = getCoords(self.getAngle(globalRotation), self.getDistance())
        pos[0] -= robotPos[0]
        pos[1] -= robotPos[1]
        return pos


class Gyroscope:
    def __init__(self, gyro, timeStep):
        self.sensor = gyro
        self.sensor.enable(timeStep)
        self.rotation = 0
        self.oldTime = 0.0
    def update(self, time):
        timeElapsed = time - self.oldTime  # Time passed in time step
        radsInTimestep = (self.sensor.getValues())[0] * timeElapsed
        degsInTimestep = radsInTimestep * 180 / math.pi
        self.rotation += degsInTimestep
        self.rotation = normalizeAngle(self.rotation)
        self.oldTime = time
    def getRotation(self):
        return self.rotation

class HeatSensor:
    def __init__(self, sensor, timeStep):
        self.sensor = sensor
        self.sensor.enable(timeStep)
    def isClose(self):
        return self.sensor.getValue() > 35

class ColourSensor:
    def __init__(self, sensor, timeStep):
        self.sensor = sensor
        self.sensor.enable(timeStep)

    def getTileType(self):
        colour = self.sensor.getImage()
        r = colour[0]
        g = colour[1]
        b = colour[2]
        tileType = "undefined"
        if r == 252 and g == 252:
            tileType = "normal"
        elif (57 < r < 61 and 57 < g < 61) or (r == 111 and g == 111):
            tileType = "trap"
        elif 144 > r > 140 and 225 > g > 220 and b == 246:
            tileType = "swamp"
        elif r == 255 and g == 255 and b == 255:
            tileType = "checkpoint"
        return tileType

class Gps:
    def __init__(self, gps, timeStep):
        self.gps = gps
        self.gps.enable(timeStep)
    def getPosition(self):
        vals = self.gps.getValues()
        return [int(vals[0] * 100), int(vals[2] * 100)]

class Camera:
    def __init__(self, camera, timeStep):
        self.camera = camera
        self.camera.enable(timeStep)
        
    def getImg(self):
        imageData = self.camera.getImage()
        return np.array(np.frombuffer(imageData, np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4)))

class Emitter:
    def __init__(self, emmitter):
        self.emitter = emmitter
    
    def sendMessage(self,pos, identifier):
        message = struct.pack('i i c', pos[0], pos[1] * 100, identifier.encode())
        self.emitter.send(message)


# Clase de capa de obstarccion
# Abstraction layer class
class AbstractionLayer:
    def __init__(self,timeStep, initialState=""):

        self.robot = Robot()
        # For self.robot
        # Constants
        self.timeStep = timeStep
        self.maxVelocity = 6.28
        self.robotDiameter = 0.071
        # Variables
        # Wheels
        self.leftWheel = self.robot.getMotor("left wheel motor")
        self.rightWheel = self.robot.getMotor("right wheel motor")
        #Cameras
        self.centreCamera = Camera(self.robot.getCamera("camera_centre"), self.timeStep)
        self.rightCamera = Camera(self.robot.getCamera("camera_right"), self.timeStep)
        self.leftCamera = Camera(self.robot.getCamera("camera_left"), self.timeStep)
        #Colour sensor
        self.colourSensor = ColourSensor(self.robot.getCamera("colour_sensor"), self.timeStep)
        #Emitter
        self.emitter = Emitter(self.robot.getEmitter("emitter"))
        #Gps
        self.gps = Gps(self.robot.getGPS("gps"), self.timeStep)
        #Gyro
        self.gyro = Gyroscope(self.robot.getGyro("gyro"), self.timeStep)
        #Heat sensors
        self.heatLeft = HeatSensor(self.robot.getLightSensor("left_heat_sensor"), self.timeStep)
        self.heatRight = HeatSensor(self.robot.getLightSensor("right_heat_sensor"), self.timeStep)


        # Variables for abstraction layer
        self.state = initialState
        self.startTime = self.actualTime = self.robot.getTime()
        self.lineIdentifier = -1
        self.linePointer = 0
        self.delayStart = 0.0
        self.delayFirstTime = True
        self.globalPos = [0, 0]
        self.globalRot = 0
        
    def step(self):
        return self.robot.step(timeStep) != -1
    
    # Poner antes de empezar una sequencia o de usar una funcion sequencial
    # Put before starting a sequence or using a sequencial function
    def startSequence(self):
        self.lineIdentifier = -1
    
    # Para la sequencia por la cantidad de segundos que uno le ponga
    # Stops a sequence for the given amount of seconds 
    def delay(self, delay):
        self.lineIdentifier += 1
        if self.lineIdentifier == self.linePointer:
            if self.delayFirstTime:
                self.delayStart = self.robot.getTime
                self.delayFirstTime = False
            else:
                if self.actualTime - self.delayStart >= delay:
                    self.delayFirstTime = True
                    self.linePointer += 1
                    return True
        

    # Hace un print en sequencia
    # Prints something in sequence
    def seqPrint(self, text):
        self.lineIdentifier += 1
        if self.lineIdentifier == self.linePointer:
            print(text)
            self.linePointer += 1
            return True
        

    # Cambia el estado
    # Changes the state
    def changeState(self, newState):
        self.state = newState
        self.linePointer = 0

    # Poner al inicio del loop principal
    # Put at the start of the main loop
    def update(self):
        self.actualTime = self.robot.getTime()
        self.gyro.update(self.actualTime)
        self.globalRot = self.gyro.getRotation()
        self.globalPos = self.gps.getPosition()

# Instanciacion de capa de abstracción
# Abstraction layer instantiation
r = AbstractionLayer("start")

#MAIN PROGRAM
while r.step():
    r.update()
    print(r.globalPos)
    