import cv2 as cv
import numpy as np
import time
import copy

from data_processing import camera_processor, data_extractor, fixture_detection, point_cloud_processor
from data_structures import lidar_persistent_grid, expandable_node_grid
import utilities, state_machines, robot, mapping

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
    seqMoveWheels(-1, -1)
    seqDelaySec(0.1)
    if seq.simpleEvent(): robot.rotationSensor = "gps"
    seqMoveWheels(1, 1)
    seqDelaySec(0.1)
    if seq.simpleEvent(): robot.rotationSensor= "gyro"
    seqDelaySec(0.1)
    seqMoveWheels(0, 0)
    seqMoveWheels(-1, -1)
    seqDelaySec(0.1)
    seqMoveWheels(0, 0)
    if seq.simpleEvent():
        robot.autoDecideRotation = True

initial_position = robot.position

def seqMoveToRelativeCoords(x, y):
    global initial_position
    if seq.simpleEvent():
        initial_position = [round(p / TILE_SIZE) * TILE_SIZE for p in robot.position]
    seqMoveToCoords((initial_position[0] + x, initial_position[1] + y))

def seqMoveToRelativeTile(x, y):
    seqMoveToRelativeCoords(x * TILE_SIZE, y * TILE_SIZE)

mapper = mapping.Mapper(TILE_SIZE)

doWallMapping = False
doFloorMapping = False


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
        # Informs the mapping components of the starting position of the robot
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
        img1 = camera_processor.flatten_image(img)
        cv.imwrite("/home/ale/rescate_laberinto/Competencias/Robocup_2022/Refactored Code/img1.png", img1)
        #stateManager.changeState("stop")
    
    elif stateManager.checkState("measure"):
        seq.startSequence()
        if seq.simpleEvent():
            initial_position = robot.position
        seqMoveToCoords((initial_position[0] + 0.12, initial_position[1] + 0.12))
        seqMoveWheels(0, 0)
        seqRotateToDegs(90)
        seqMoveWheels(0, 0)
        if seq.simpleEvent():
            start_time = time.time()
        seqRotateToDegs(270)
        if seq.simpleEvent():
            print("time taken: ", time.time() - start_time)
        seqMoveWheels(0, 0)

    # Explores and maps the maze
    elif stateManager.checkState("explore"):
        seq.startSequence()
        #seqMoveWheels(0.5, -0.5)
        #seqRotateToDegs(270)
        seqMoveToRelativeTile(2, 0)
        seqMoveToRelativeTile(0, 6)
        seqMoveToRelativeTile(4, 0)
        seqMoveToRelativeTile(0, -4)
        seqMoveToRelativeTile(-2, 0)
        seqMoveToRelativeTile(0, -2)
        seqMoveToRelativeTile(-4, 0)
        
        seqMoveWheels(0, 0)
        seqRotateToDegs(0)
        seqMoveWheels(0, 0)
        #seq.seqResetSequence()
        
        
        #robot.autoDecideRotation = False
        #robot.rotationSensor = "gyro"
        
        mapper.update(robot.getDetectionPointCloud(), robot.position)


        """
        images = [robot.rightCamera.getImg(), robot.centerCamera.getImg(), robot.leftCamera.getImg()]
        
        lidar_point_cloud = robot.getDetectionPointCloud()
        data_extractor.get_floor_colors(images, lidar_point_cloud, robot.rotation, robot.position)
        """

        """
        nube_de_puntos = robot.getDetectionPointCloud()
        #print("nube de puntos: ", nube_de_puntos)
        nueva_nube_de_puntos = point_cloud_processor.processPointCloud(nube_de_puntos, robot.position)
        #print("nueva nube de puntos: ", nueva_nube_de_puntos)
        for pos in nueva_nube_de_puntos:
            round_pos = [round(pos[0] * 1000), round(pos[1]  * 1000)]
            mi_grilla.add_point(round_pos)
        mi_grilla.print_grid()
        """
        
        # point_cloud = robot.getLidarPointCloud()
        # images = robot.getCameraImages()
        # grilla.update(point_cloud, images, robot.position, robot.rotation)

        # grilla = mapping.getGrilla()
        # mejor_moviemiento = agent.getAction(grilla)
        # coordenadas = robot.getCoordenadas(mejor_movimiento)
        # robot.moveToCoords(coordenadas)
        # repetir

        #imgs = (robot.rightCamera.getImg(), robot.centerCamera.getImg(), robot.leftCamera.getImg())
        #data_extractor.get_floor_colors(imgs, robot.getDetectionPointCloud(), robot.rotation, robot.position)
        
        print("rotation:", robot.rotation)

        

        
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
