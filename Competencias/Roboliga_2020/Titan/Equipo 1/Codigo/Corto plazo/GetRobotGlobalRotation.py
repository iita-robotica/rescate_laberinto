from controller import Robot
import math

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
# gyro
gyro = robot.getGyro("gyro")
gyro.enable(timeStep)
#        [left wheel speed, right wheel speed]
speeds = [0.0, 0.0]
wheel_left.setPosition(float("inf"))
wheel_right.setPosition(float("inf"))
# Program start time
# Tiempo de inicio del programa
program_start = robot.getTime()
# Global rotation
globalRot = 0

# Converts a range of value to another
# convierte un rango de valores a otro
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Changes wheel speeds from a fraction of the max speed.
# cambia la velocidad de las ruedas teniendo en cuenta un fraccion de la velocidad. (Poner valores de 0 a 1, donde 1 es
# igual a la velocidad maxima )
def move(fraction1, fraction2):
    # set left wheel speed
    speeds[0] = fraction1 * max_velocity
    # set right wheel speed
    speeds[1] = fraction2 * max_velocity

# Updates the wheel speed (Put at the end of loop)
def updateWheels():
    wheel_left.setVelocity(speeds[0])
    wheel_right.setVelocity(speeds[1])

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

#Main loop
while robot.step(timeStep) != -1:
    # Gets the robot global rotation
    globalRot = getRotationByVelocity(globalRot)
    # Turns in place
    move(0.5, -0.5)
    # Prints the global rotation
    print(globalRot)
    # Updates the wheels speeds
    updateWheels()
