from controller import Robot
import sys
import numpy as np
import cv2 as cv
#REMEMBER TO COPY-PASTE THIS FUNCTIONS ON TO FINAL CODE
sys.path.append(r"/home/ale/rescate_laberinto/Competencias/Roboliga_2021/FinalCode")
from AbstractionLayer import AbstractionLayer
from StateMachines import StateManager
timeStep = 16 * 2 


stMg = StateManager("init")
r = AbstractionLayer()


# While the simulation is running
while r.doLoop():
    # Update the robot
    r.update()
    print("rotation: " + str(r.rotation))
    print("position: " + str(r.position))
    print("State:", stMg.state)
    
    

    if not stMg.checkState("init"):
        if r.isEnded():
            r.seqResetSequence()
            stMg.changeState("end")

        elif r.isVictims():
            r.seqResetSequence()
            print("THERE ARE VICTIMS")
            stMg.changeState("victim")
        
        elif r.analyst.stoppedMoving and not stMg.checkState("stoppedMoving"):
            r.seqResetSequence()
            print("ESTADO STOPPED MOVING")
            stMg.changeState("stoppedMoving")


    if stMg.checkState("init"):
        if r.calibrate():
            stMg.changeState("followBest")
            #stMg.changeState("stop")
    
    if stMg.checkState("stop"):
        r.seqMg.startSequence()
        r.seqMoveWheels(0, 0)

    if stMg.checkState("stoppedMoving"):
        r.seqMg.startSequence()
        r.seqMoveWheels(-0.5, -0.5)
        r.seqDelaySec(0.5)
        r.seqMoveWheels(0, 0)
        r.seqDelaySec(0.2)
        r.analyst.calculatePath = True
        if r.seqResetSequence():
            stMg.changeState("followBest")


    if stMg.checkState("moveForward"):
        r.seqMg.startSequence()
        r.seqMoveWheels(0.5, -0.5)
        r.seqDelaySec(0.1)
        r.seqMoveWheels(-0.5, 0.5)
        r.seqDelaySec(0.1)
        r.seqResetSequence()

    
    if stMg.checkState("followBest"):
        r.seqMg.startSequence()
        bestPos = r.getBestPos()
        if bestPos is not None:
            r.seqMoveToCoords(bestPos)
        r.seqResetSequence()

        if r.isTrap:
            r.seqResetSequence()
            stMg.changeState("hole")
        
    
    if stMg.checkState("hole"):
        r.seqMg.startSequence()
        r.seqMoveWheels(-0.5, -0.5)
        r.seqDelaySec(0.5)
        r.seqMoveWheels(0, 0)
        if r.seqMg.simpleSeqEvent(): r.recalculatePath()
        if r.seqResetSequence():
            stMg.changeState("followBest")

    if stMg.checkState("victim"):
        print("Victim mode!!")
        r.seqMg.startSequence()
        r.seqMoveWheels(0, 0)
        r.seqPrint("stopping")
        r.seqDelaySec(3.2)
        r.seqPrint("reporting")
        if r.seqMg.simpleSeqEvent(): r.reportVictims()
        r.seqPrint("Victim reported")
        if r.seqResetSequence():
            stMg.changeState("followBest")
    
    if stMg.checkState("end"):
        r.seqMg.startSequence()
        if r.seqMg.simpleSeqEvent(): r.endGame()
        r.seqMoveWheels(0, 0)
        
    print("--------------------------------------------------------------------")