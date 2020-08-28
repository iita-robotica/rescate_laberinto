from controller import Robot
import math

"""
Con este codigo se pueden crear sequencias de movimientos por coordenadas. Al especificar una coordenada el robot girara
y se movera par llegar a ella, con presicion ajustable (Linea 170)

"""

# -- VARIABLES PARA EL CONTROLADOR --

#Define el timestep (Puede ser 32)
timeStep = 16

#Velocidad maxima
max_velocity = 6.28


# Instanciacion de robot
robot = Robot()

# Ruedas
wheel_left = robot.getMotor("left wheel motor")
wheel_right = robot.getMotor("right wheel motor")

# gps
gps = robot.getGPS("gps")
gps.enable(timeStep)

# gyro
gyro = robot.getGyro("gyro")
gyro.enable(timeStep)

#        [Velocidad de rueda izquierda, Velocidad de rueda derecha]
speeds = [0.0, 0.0]

wheel_left.setPosition(float("inf"))
wheel_right.setPosition(float("inf"))

# Tiempo de inicio del programa

program_start = robot.getTime()

# -- VARIABLES NECESARIAS PARA EL MOVIMIENTO POR COORDENADAS --
moveOrderNumber = 0
movesDone = 0
globalRot = 0
globalPos = [0, 0]

#tamano de nodo y de baldosa


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


# Poner esto en un if si se quiere repetir la sequencia indefinidamente
def loopSequence():
    global movesDone
    movesDone = 0

# Poner esta funcion antes de hacer una secuencia de movimientos
def startSequence():
    global moveOrderNumber
    moveOrderNumber = 0


# Mueve el robot a las coordenadas especificadas
turned = False
def moveToCoords(targetPos):
    global turned
    global globalPos
    global moveOrderNumber
    global movesDone
    global globalRot

    moveOrderNumber += 1

    # Nivel de presicion. Se puede hacer mas chico o mas grande, pero puede ser que si es demasiado chico el robot nunca
    # llegue a su destino. Segun mis pruevas funciona bastante bien con este valor
    errorMargin = 0.01

    diffX = targetPos[0] - globalPos[0]
    diffY = targetPos[1] - globalPos[1]
    dist = math.sqrt(diffX ** 2 + diffY ** 2)

    if moveOrderNumber == movesDone + 1:
        if errorMargin * -1 < dist < errorMargin:
            print("done")
            stop()
            turned = False
            movesDone += 1
            return True
        else:
            rad = math.atan2(diffX, diffY)
            ang = rad * 180 / math.pi
            ang += 180
            ang = normalizeAngle(ang)

            #print(ang)
            if not turned:
                turned = turnToAngle(ang)
            if turned:

                #print(dist)
                ratio = min(mapVals(dist, 0, 0.1, 0.1, 1), 1)
                ratio = max(ratio, 0.2)
                diff = globalRot - ang
                if diff > 180:
                    diff = 360 - diff
                    diff * -1
                if -1 < diff < 1:
                    move(ratio, ratio)
                else:
                    turnToAngle(ang)

# Hace que las ruedas giren con las velocidades especificadas
def updateWheels():
    wheel_left.setVelocity(speeds[0])
    wheel_right.setVelocity(speeds[1])


while robot.step(timeStep) != -1:

    # Actualiza la rotcion y posicion global
    globalRot = getRotationByVelocity(globalRot)
    globalPos = gps.getValues()
    globalPos = [globalPos[0], globalPos[2]]

    # -- CODIGO DE EJEMPLO --

    # Comienza la sequencia de movimiento
    startSequence()

    # Hace un circuito de tres coordenadas
    moveToCoords([-0.110, 0.240])
    moveToCoords([-0.120, 0.119])
    if moveToCoords([-0.240, 0.170]):
        # Cuando el ultimo movimiento termina, repite la sequencia
        loopSequence()

    # Actualiza las ruedas
    updateWheels()

# NOTA: Al correr este codigo suelen salir errores de: WARNING: The current physics step could not be computed
# correctly. Your world may be too complex. If this problem persists, try simplifying your bounding object(s), reducing
# the number of joints, or reducing WorldInfo.basicTimeStep.

# Ademas el robot se mueve medio raro. Esto ocurre por algunos problemas relacionados con el simulador, y que el codigo
# tiene que compensar. No se por que ocurren, pero sospecho que lo hacen al girar a velocidades bajas. Pregunte en el
# discord de webots, pero me han podido ofrecer una solucion.
# Entodo caso, el codigo es confiable, y siempre llega a la destinacion. Pudo moverse a los 3 puntos consistentemente a
# lo largo de los 8 minutos.
