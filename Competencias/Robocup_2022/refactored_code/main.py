import cv2 as cv
import numpy as np
import time
import copy

from data_processing import fixture_detection
import utilities, state_machines, robot, mapping

# World constants
TIME_STEP = 32
TILE_SIZE = 0.06
TIME_IN_ROUND = (8 * 60)


# Components
#Robot
robot = robot.RobotLayer(TIME_STEP)

# Stores, changes and compare states
stateManager = state_machines.StateManager("init")

# Sequence manager
# Resets flags that need to be in a certain value when changing sequence, for example when changing state
def resetSequenceFlags():
    robot.delay_first_time = True
seq = state_machines.SequenceManager(resetFunction=resetSequenceFlags)

# Mapper
mapper = mapping.Mapper(TILE_SIZE)


# Variables
doWallMapping = False
doFloorMapping = False


# Functions
# Sequential functions used frequently
seqPrint = seq.makeSimpleEvent(print)
seqDelaySec = seq.makeComplexEvent(robot.delay_sec)
seqMoveWheels = seq.makeSimpleEvent(robot.move_wheels)
seqRotateToDegs = seq.makeComplexEvent(robot.rotate_to_degs)
seqMoveToCoords = seq.makeComplexEvent(robot.move_to_coords)
seqResetSequenceFlags = seq.makeSimpleEvent(resetSequenceFlags)

# Calculates offsets in the robot position, in case it doesn't start perfectly centerd
def calibratePositionOffsets():
    actualTile = [robot.position[0] // TILE_SIZE, robot.position[1] // TILE_SIZE]
    robot.position_offsets = [
        round((actualTile[0] * TILE_SIZE) - robot.position[0]) + TILE_SIZE // 2,
        round((actualTile[1] * TILE_SIZE) - robot.position[1]) + TILE_SIZE // 2]
    robot.position_offsets = [robot.position_offsets[0] % TILE_SIZE, robot.position_offsets[1] % TILE_SIZE]
    print("positionOffsets: ", robot.position_offsets)

def seqCalibrateRobotRotation():
    # Calibrates the robot rotation using the gps
    if seq.simpleEvent():
        robot.auto_decide_rotation = False
    seqMoveWheels(-1, -1)
    seqDelaySec(0.1)
    if seq.simpleEvent(): robot.rotation_sensor = "gps"
    seqMoveWheels(1, 1)
    seqDelaySec(0.1)
    if seq.simpleEvent(): robot.rotation_sensor= "gyro"
    seqDelaySec(0.1)
    seqMoveWheels(0, 0)
    seqMoveWheels(-1, -1)
    seqDelaySec(0.1)
    seqMoveWheels(0, 0)
    if seq.simpleEvent():
        robot.auto_decide_rotation = True

initial_position = robot.position

def seqMoveToRelativeCoords(x, y):
    global initial_position
    if seq.simpleEvent():
        initial_position = [round(p / TILE_SIZE) * TILE_SIZE for p in robot.position]
    seqMoveToCoords((initial_position[0] + x, initial_position[1] + y))

def seqMoveToRelativeTile(x, y):
    seqMoveToRelativeCoords(x * TILE_SIZE, y * TILE_SIZE)

# Each timeStep
while robot.do_loop():
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
        robot.move_wheels(0, 0)
    
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
        #seqRotateToDegs(90)
        #seq.seqResetSequence()
        

        lidar_point_cloud = robot.get_detection_point_cloud()
        images = robot.get_camera_images()

        mapper.update(lidar_point_cloud, images, robot.position, robot.rotation)  
        #data_extractor.get_floor_colors(images, lidar_point_cloud, robot.rotation, robot.position)

        grid = mapper.get_node_grid()
        # mejor_moviemiento = agent.getAction(grid)
        # coordenadas = robot.getCoordenadas(mejor_movimiento)
        # robot.moveToCoords(coordenadas)
        # repetir
        
        print("rotation:", robot.rotation)

    # Reports a victim
    elif stateManager.checkState("report_victim"):
        seq.startSequence()
        seqDelaySec(3)
        #Classifies and reports the vicitim
        if seq.simpleEvent():
            victims = []
            for cam in (robot.left_camera, robot.right_camera):
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
