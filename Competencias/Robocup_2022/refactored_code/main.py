import cv2 as cv
import numpy as np
import time
import copy

from data_processing import fixture_detection
import utilities, state_machines, robot, mapping

from agents.closest_position_agent.closest_position_agent import ClosestPositionAgent

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

closest_position_agent = ClosestPositionAgent()


# Variables
do_mapping = False


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

    # Loads data to mapping
    if do_mapping:
        lidar_point_cloud = robot.get_detection_point_cloud()
        images = robot.get_camera_images()
        #utilities.save_image(images[1], "camera_image_center.png")
        mapper.update(lidar_point_cloud, images, robot.position, robot.rotation, current_time=robot.time)

    else:
        mapper.update(robot_position=robot.position, robot_rotation=robot.rotation, current_time=robot.time)


    # Updates state machine
    if stateManager.checkState("init"):
        pass

    print("state: ", stateManager.state)

    # Runs once when starting the game
    if stateManager.checkState("init"):
        seq.startSequence()
        seqDelaySec(0.5)
        # Calculates offsets in the robot position, in case it doesn't start perfectly centerd
        seq.simpleEvent(calibratePositionOffsets)
        # Informs the mapping components of the starting position of the robot
        seq.simpleEvent(mapper.register_start)
        # Calibrates the rotation of the robot using the gps
        seqCalibrateRobotRotation()
        # Starts mapping walls
        if seq.simpleEvent():
            do_mapping = True
        # Changes state and resets the sequence
        seq.simpleEvent(stateManager.changeState, "explore")
        seq.seqResetSequence()
    

    # Explores and maps the maze
    elif stateManager.checkState("explore"):
        seq.startSequence()
        #seqMoveWheels(0.5, -0.5)
        #seqRotateToDegs(270)
        """
        seqMoveToRelativeTile(2, 0)
        seqMoveToRelativeTile(0, 6)
        seqMoveToRelativeTile(4, 0)
        seqMoveToRelativeTile(0, -4)
        seqMoveToRelativeTile(-2, 0)
        seqMoveToRelativeTile(0, -2)
        seqMoveToRelativeTile(-4, 0)
        """
        
        #seqMoveWheels(-0.4, 0.4)
        #seqRotateToDegs(0)
        #seq.seqResetSequence()
        
        #data_extractor.get_floor_colors(images, lidar_point_cloud, robot.rotation, robot.position)

        grid = mapper.get_node_grid()
        move = closest_position_agent.get_action(grid)
        seqMoveToRelativeTile(move[0], move[1])
        seq.seqResetSequence()
        # mejor_moviemiento = agent.get_action(grid)
        # coordenadas = getCoordenadas(mejor_movimiento)
        # robot.moveToCoords(coordenadas)
        # repetir
        
        print("rotation:", robot.rotation)

    # Reports a victim
    elif stateManager.checkState("report_victim"):
        seq.startSequence()
        victims = mapper.get_node_grid().get_node(robot.position[0], robot.position[1]).victims
        # If there is an unreported vicitm in the tile, reports the vicitim
        for victim in victims:
            if victim["reported"] == False:
                seqDelaySec(3)
                if seq.simpleEvent():
                    robot.comunicator.sendVictim(robot.position, victim["letter"])
                    victim["reported"] = True
                if seq.simpleEvent():
                    break
        seq.simpleEvent(stateManager.changeState, "explore")
    
    elif stateManager.checkState("teleported"):
        seq.startSequence()
        # parar mapping
        do_mapping = False
        seqCalibrateRobotRotation()
        # Changes state and resets the sequence
        seq.simpleEvent(stateManager.changeState, "explore")
        seq.seqResetSequence()
