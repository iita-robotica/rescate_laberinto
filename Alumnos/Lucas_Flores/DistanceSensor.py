from controller import Robot

"""
timeStep = 32
max_Velocity = 6.28

robot = Robot()

wheel_left = robot.getMotor("left wheel motor")
wheel_right = robot.getMotor("right wheel motor")

speeds = [max_Velocity,max_Velocity]

side_sensors = []

side_sensors.append(robot.getDistanceSensor("ps0"))
side_sensors[0].enable(timeStep)
side_sensors.append(robot.getDistanceSensor("ps1"))
side_sensors[1].enable(timeStep)

wheel_left.setPosition(float("inf"))
wheel_right.setPosition(float("inf"))

while robot.step(timeStep) !=  -1:
    for i in range(2):
        if side_sensors[i].getValue() > 15:
            speeds = [0,0]
            return True
"""
#*************************************************************************************
class Hole_Detector:
    def __init__(self, side_sensors):

        self.side_sensors = []
        self.side_sensors.append(robot.getDistanceSensor("ps0"))#Lado Izquierdo
        self.side_sensors[0].enable(timeStep)
        self.side_sensors.append(robot.getDistanceSensor("ps1"))#Lado Derecho
        self.side_sensors[1].enable(timeStep)

        self.limit = 15

    def Hole_Detector():

        for i in range(2):
            if self.side_sensors[i].getValue() > self.limit:
                hole_location = True
            else:
                hole_location = False
        return hole_location
#hola