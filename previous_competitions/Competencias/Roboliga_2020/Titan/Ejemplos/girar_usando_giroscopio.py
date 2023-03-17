# IMPORTAMOS LIBRERIA
from controller import Robot
from time import sleep
import math   #para pasar de rads a grados
# SETUP DE MAPA
timeStep = 32           
max_velocity = 6.28

globalRotation = 0



# SETUP DE RUEDAS Y COMUNICACION       
robot = Robot()
wheel_left = robot.getMotor("left wheel motor")   
wheel_right = robot.getMotor("right wheel motor") 
gyro = robot.getGyro("gyro")
gyro.enable(timeStep)
oldRotTime = robot.getTime()

#////////////////SPEED//////////////////////////////
speeds = [max_velocity, max_velocity]

#///////////////////////////WHEELS/////////////////////
wheel_left.setVelocity(max_velocity)
wheel_right.setVelocity(max_velocity)

wheel_left.setPosition(float("inf"))   # giro libre (infinito)
wheel_right.setPosition(float("inf"))  # giro libre (infinito)

#///////////////FUNCIONES DE MOVIMIENTO/////////////////////////
def stop():
    speeds[0] = 0
    speeds[1] = 0

def turn_right():
    speeds[0] = 0.1 * max_velocity
    speeds[1] = -0.1 * max_velocity

def turn_left():
    speeds[0] = -0.1 * max_velocity
    speeds[1] = 0.1 * max_velocity

#///////////////GIRO////////////////////////////
# esta funicon gira el robot en x grados

def getRotationByVelocity(x):
    global globalRotation          # variable global para que puedamos acceder desde cualquier lado
    global oldRotTime

    newRotTime = robot.getTime()
    timeElapsed = newRotTime - oldRotTime           # tiempo que paso en cada timeStep
    radsInTimestep = (gyro.getValues())[0] * timeElapsed # 
    degsInTimestep = radsInTimestep * 180 / math.pi

    print("rads: " + str(radsInTimestep) + " | degs: " + str(degsInTimestep))

    if -1 < (globalRotation - x) < 1:
        stop()
        return True
    else:
        turn_left()

    globalRotation += degsInTimestep

    # Si se pasa de 360 grados se ajusta la rotacion empezando desde 0 grados
    globalRotation = globalRotation % 360

    # Si es mas bajo que 0 grados, le resta ese valor a 360
    if globalRotation < 0:
        globalRotation += 360

    # print("global rotation: " + str(globalRotation))
    # print("tiempo: " + str(timeElapsed))
    #Actualiza el tiempo de rotacion
    oldRotTime = newRotTime

    return False

while robot.step(timeStep) != -1:


    if getRotationByVelocity(90) == True:
        print("Llegue")

    wheel_left.setVelocity(speeds[0])
    wheel_right.setVelocity(speeds[1])


