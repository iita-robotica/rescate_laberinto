from controller import Robot
from controller import LED
from controller import Accelerometer

timeStep = 32
# Maxima velocidad del robot
max_velocity = 6.28
x = 0
y = 0

# Inicializo el robot
robot = Robot()

# Inicializo los motores de ambas ruedas
wheel_left = robot.getMotor("left wheel motor")
wheel_right = robot.getMotor("right wheel motor")

# Inicializo el gps
gps = robot.getGPS("gps")
gps.enable(timeStep)

# Actualizar la posicion del robot en las variables x,y
def posicion():
    global x, y, posinicial
    # Leo los valores del gps
    pos = gps.getValues()
    # Traduzco los valores del gps al plano con respecto a la posicion inicial
    x = (pos[2] - posinicial[2]) * (-1) * (7 / 2.1) + 4
    y = (pos[0] - posinicial[0] )* (-1) * (7 / 2.1) + 0.2
    

# Flag que me permite identificar la posicion inicial
primeravez = True


while robot.step(timeStep) != -1:
    if  primeravez: 
         posinicial = gps.getValues()
         print(posinicial)
         primeravez = False
    else:
        wheel_left.setVelocity(0)
        wheel_right.setVelocity(0)
        posicion()
        # Obtener valore
        print(x , y)
        print(round(x), round(y))
   
