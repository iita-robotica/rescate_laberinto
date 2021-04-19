from controller import Robot
import sys
import numpy as np
import cv2 as cv
import math
#REMEMBER TO COPY-PASTE THIS FUNCTIONS ON TO FINAL CODE
sys.path.append(r"C:\\Users\\ANA\\Desktop\\Webots - Erebus\\rescate_laberinto\\Competencias\\Robocup_2021\\Equipo\\FinalCode")
from UtilityFunctions import *
from StateMachines import *
from RobotLayer import RobotLayer

timeStep = 16 * 2 


robot = RobotLayer(timeStep)
seqMg = SequenceManager()
seqPrint = seqMg.makeSimpleSeqEvent(print)

# While the simulation is running
while robot.doLoop():
    # Update the robot
    robot.update()

    seqMg.startSequence()
    seqPrint("Hello")
    seqMg.resetSequence()
