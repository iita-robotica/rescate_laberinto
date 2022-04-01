absuloute_dir = r'/home/ale/rescate_laberinto/Competencias/Robocup_2022/Refactored Code'
import sys
import cv2 as cv

sys.path.append(absuloute_dir)
import fixture_detection, camera_processing, mapping, pathfinding, point_cloud_processor, robot, state_machines, utils

# World constants
TIME_STEP = 32
TILE_SIZE = 0.06
TIME_IN_ROUND = (8 * 60)

# Components
robot = robot.RobotLayer(TIME_STEP)

# Stores, changes and compare states
stateManager = state_machines.StateManager("init")

# Resets flags that need to be in a certain value when changing sequence, for example when changing state
def resetSequenceFlags():
    robot.delayFirstTime = True

# Sequence manager
seq = state_machines.SequenceManager(resetFunction=resetSequenceFlags)

# sequential functions used frequently
seqPrint = seq.makeSimpleEvent(print)
seqDelaySec = seq.makeComplexEvent(robot.delaySec)
seqMoveWheels = seq.makeSimpleEvent(robot.moveWheels)
seqRotateToDegs = seq.makeComplexEvent(robot.rotateToDegs)
seqMoveToCoords = seq.makeComplexEvent(robot.moveToCoords)
seqResetSequenceFlags = seq.makeSimpleEvent(resetSequenceFlags)

def isHole():
    # TODO
    return False

# Calculates offsets in the robot position, in case it doesn't start perfectly centerd
def calibratePositionOffsets():
    actualTile = [robot.position[0] // TILE_SIZE, robot.position[1] // TILE_SIZE]
    robot.positionOffsets = [
        round((actualTile[0] * TILE_SIZE) - robot.position[0]) + TILE_SIZE // 2,
        round((actualTile[1] * TILE_SIZE) - robot.position[1]) + TILE_SIZE // 2]
    robot.positionOffsets = [robot.positionOffsets[0] % TILE_SIZE, robot.positionOffsets[1] % TILE_SIZE]
    print("positionOffsets: ", robot.positionOffsets)

def seqCalibrateRobotRotation():
    # Calibrates the robot rotation using the gps
    if seq.simpleEvent():
        robot.autoDecideRotation = False
    seqDelaySec(0.5)
    if seq.simpleEvent(): robot.rotationSensor = "gps"
    seqMoveWheels(1, 1)
    seqDelaySec(0.2)
    if seq.simpleEvent(): robot.rotationSensor= "gyro"
    seqDelaySec(0.2)
    seqMoveWheels(0, 0)
    seqMoveWheels(-1, -1)
    seqDelaySec(0.4)
    seqMoveWheels(0, 0)
    if seq.simpleEvent():
        robot.autoDecideRotation = True

doWallMapping = False
doFloorMapping = False

mi_grilla = mapping.Grid((10, 10))

# Each timeStep
while robot.doLoop():
    # Updates robot position and rotation, sensor positions, etc.
    robot.update()
    print("state: ", stateManager.state)

    # Runs once when starting the game
    if stateManager.checkState("init"):
        seq.startSequence()

        seqDelaySec(0.5)

        # Calculates offsets in the robot position, in case it doesn't start perfectly centerd
        seq.simpleEvent(calibratePositionOffsets)
            
        # Informs the mapping and pathfinding components of the starting position of the robot
        # seq.simpleEvent(pathfinding.registerStart())
        # seq.simpleEvent(mapping.registerStart())
        
        # Calibrates the rotation of the robot using the gps
        seqCalibrateRobotRotation()

        # Starts mapping walls
        if seq.simpleEvent():
            doWallMapping = True
            doFloorMapping = True

        # Changes state and resets the sequence
        seq.simpleEvent(stateManager.changeState, "explore")
        seq.seqResetSequence()
    
    elif stateManager.checkState("stop"):
        robot.moveWheels(0, 0)

    elif stateManager.checkState("save_img"):
        img = robot.centerCamera.getImg()
        img1 = camera_processing.flatten_image(img)
        cv.imwrite("/home/ale/rescate_laberinto/Competencias/Robocup_2022/Refactored Code/img1.png", img1)
        #stateManager.changeState("stop")

    # Explores and maps the maze
    elif stateManager.checkState("explore"):
        robot.autoDecideRotation = False
        robot.rotationSensor = "gyro"
        nube_de_puntos = robot.getDetectionPointCloud()
        #print("nube de puntos: ", nube_de_puntos)
        nueva_nube_de_puntos = point_cloud_processor.processPointCloud(nube_de_puntos, robot.position)
        #print("nueva nube de puntos: ", nueva_nube_de_puntos)
        for pos in nueva_nube_de_puntos:
            round_pos = [round(pos[0] * 1000), round(pos[1] * 1000)]
            mi_grilla.add_point(round_pos)
        mi_grilla.print_grid()

        # lidar es robot.getLidar()
        # Grilla es mapping.getGrilla(lidar)
        # mejor posicion es predictor de laberinto.getMejorPosición(grilla)
        # moviemientos es pathfinding.getCamino(mejor posición)
        # robot.sequir camino(moviemientos)
        # repetir

        img = robot.centerCamera.getImg()
        img1 = camera_processing.flatten_image(img)
        cv.imshow("camara", img1)

        cv.waitKey(1)

        # If it encountered a hole
        if isHole():
            # Changes state and resets the sequence
            seq.simpleEvent(stateManager.changeState, "hole")
            seq.seqResetSequence()
        robot.moveWheels(0, 0)

    # What to do if it encounters a hole
    elif stateManager.checkState("hole"):
        # TODO
        pass
        # reportar obstáculo a mapping
        # moverse para atras
        # volver a "explore"
        
    # Reports a victim
    elif stateManager.checkState("report_victim"):
        seq.startSequence()
        seqDelaySec(3)
        #Classifies and reports the vicitim
        if seq.simpleEvent():
            victims = []
            for cam in (robot.leftCamera, robot.rightCamera):
                image = cam.getImg()
                vics = fixture_detection.detectVictims(image)
                victims += fixture_detection.getCloseVictims(vics)
            if len(victims) > 0:
                letter = fixture_detection.classifyFixture(victims[0])
            robot.comunicator.sendVictim(robot.position, letter)
        # TODO Reportar victima a mapping
        seq.simpleEvent(stateManager.changeState, "explore")
    
    elif stateManager.checkState("teleported"):
        seq.startSequence()
        # parar mapping
        doWallMapping = False
        seqCalibrateRobotRotation()
        # Changes state and resets the sequence
        seq.simpleEvent(stateManager.changeState, "explore")
        seq.seqResetSequence()
