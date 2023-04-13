import cv2 as cv
import numpy as np
import time
import copy
import math

from data_processing.fixture_detection.fixture_detection import FixtureDetector
import state_machines, robot, mapping.mapping as mapping
from algorithms.expandable_node_grid.bfs import bfs

from agents.granular_navigation_agent.granular_navigation_agent import GranularNavigationAgent

from data_structures.vectors import Position2D

from flags import SHOW_DEBUG, PRINT_MAP_AT_END

window_n = 0


# World constants
TIME_STEP = 32
TILE_SIZE = 0.06
TIME_IN_ROUND = 8 * 60


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


# Fixture detection
fixture_detector = FixtureDetector()

# Mapper
mapper = mapping.Mapper(TILE_SIZE)

# Agents
navigation_agent = GranularNavigationAgent(mapper)

# Variables
do_mapping = False
do_victim_reporting = False


# Functions
# Sequential functions used frequently
seqPrint = seq.makeSimpleEvent(print)
seqDelaySec = seq.makeComplexEvent(robot.delay_sec)
seqMoveWheels = seq.makeSimpleEvent(robot.move_wheels)
seqRotateToDegs = seq.makeComplexEvent(robot.rotate_to_angle)
seqMoveToCoords = seq.makeComplexEvent(robot.move_to_coords)
seqResetSequenceFlags = seq.makeSimpleEvent(resetSequenceFlags)

# Calculates offsets in the robot position, in case it doesn't start perfectly centerd
def calibratePositionOffsets():
    actualTile = [robot.position[0] // TILE_SIZE, robot.position[1] // TILE_SIZE]
    robot.position_offsets = [
        round((actualTile[0] * TILE_SIZE) - robot.position[0]) + TILE_SIZE // 2,
        round((actualTile[1] * TILE_SIZE) - robot.position[1]) + TILE_SIZE // 2]
    robot.position_offsets = Position2D(robot.position_offsets[0] % TILE_SIZE, robot.position_offsets[1] % TILE_SIZE)
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
    return seqMoveToCoords((initial_position[0] + x, initial_position[1] + y))

def is_complete(grid, robot_node):
        possible_nodes = bfs(grid, robot_node, 500)
        if len(possible_nodes) == 0:
            return True
        return False

# Each timeStep
while robot.do_loop():
    # Updates robot position and rotation, sensor positions, etc.
    robot.update()

    
    if do_mapping:
        # Floor and lidar mapping
        mapper.update(robot.get_point_cloud(), 
                      robot.get_out_of_bounds_point_cloud(), 
                      robot.get_camera_images(), 
                      robot.position, 
                      robot.rotation)

        # Victim detection
        images = robot.get_camera_images()
        if images is not None:
            for index, image in enumerate(images):
                angle = (index - 1) * 90

                rot_img = np.rot90(image, -1)

                victims = fixture_detector.find_fixtures(rot_img)
                if len(victims) > 0:
                    letter = fixture_detector.classify_fixture(victims[0])
                    if letter is not None:
                        mapper.load_fixture_to_tile(letter, angle, robot.rotation.degrees)
                        
                    break
    
    else:
        # Only position and rotation
        mapper.update(robot_position=robot.position, 
                      robot_rotation=robot.rotation)
    
    #fixture_detector.tune_filter(robot.get_camera_images()[1])

    # Updates state machine
    if not stateManager.checkState("init"):
        if SHOW_DEBUG:
            print("stuck_counter: ", robot.stuck_counter)
        if robot.is_stuck():
            if SHOW_DEBUG:
                print("FRONT BLOCKED")
            mapper.block_front_vortex(robot.rotation.degrees)
            if not stateManager.checkState("stuck"):
                seq.resetSequence()
                stateManager.changeState("stuck")
    
    
        if mapper.get_fixture_from_tile().exists and not mapper.get_fixture_from_tile().reported and do_victim_reporting:
            if not stateManager.checkState("report_victim"):
                seq.resetSequence()
                stateManager.changeState("report_victim")

        elif robot.comunicator.remainingTime < 30:
            if not stateManager.checkState("go_back") and not stateManager.checkState("end"):
                seq.resetSequence()
                stateManager.changeState("go_back")

    if SHOW_DEBUG:
        print("state: ", stateManager.state)

    # Runs once when starting the game
    if stateManager.checkState("init"):
        seq.startSequence()
        seqDelaySec(0.5)
        # Calculates offsets in the robot position, in case it doesn't start perfectly centerd
        seq.simpleEvent(calibratePositionOffsets)
        # Informs the mapping components of the starting position of the robot
        seq.simpleEvent(mapper.register_start, robot.position)
        # Calibrates the rotation of the robot using the gps
        seqCalibrateRobotRotation()
        # Starts mapping walls
        if seq.simpleEvent():
            do_mapping = True
            do_victim_reporting = True
        if seq.simpleEvent():
            #do_mapping = False
            #do_victim_reporting = False
            pass
        # Changes state and resets the sequence
        seq.simpleEvent(stateManager.changeState, "explore")
        seq.seqResetSequence()

    elif stateManager.checkState("stop"):
        seq.startSequence()
        seqMoveWheels(0, 0)

    # Explores and maps the maze
    elif stateManager.checkState("explore"):
        seq.startSequence()

        #grid = mapper.get_node_grid()
        navigation_agent.update()
        #move = closest_position_agent.get_action(grid)
        node = mapper.robot_node

        if seqMoveToCoords(navigation_agent.get_target_position()):
            mapper.set_robot_node(robot.position)
        
        """
        if seqMoveToRelativeTile(move[0], move[1]):
            mapper.set_robot_node(robot.position)
            if mapper.node_grid.get_node(mapper.robot_node).is_start:
                if is_complete(mapper.node_grid, mapper.robot_node):
                    seq.resetSequence()
                    stateManager.changeState("end")
        """

        seq.seqResetSequence()

        if SHOW_DEBUG:
            print("rotation:", robot.rotation.degrees)
            print("position:", robot.position)

    # Reports a victim
    elif stateManager.checkState("report_victim"):
        seq.startSequence()
        seqMoveWheels(0, 0)
        if seq.simpleEvent() and SHOW_DEBUG:
            print("STOPPED")
        seqDelaySec(3)
        if seq.simpleEvent():
            fixture = mapper.get_fixture_from_tile()
            robot.comunicator.sendVictim(robot.position, fixture.type)
            fixture.reported = True
            mapper.load_fixture_to_wall(letter, fixture.detection_angle)
        seq.simpleEvent(stateManager.changeState, "explore")
        seq.seqResetSequence()
    
    elif stateManager.checkState("teleported"):
        seq.startSequence()
        # parar mapping
        do_mapping = False
        seqCalibrateRobotRotation()
        # Changes state and resets the sequence
        seq.simpleEvent(stateManager.changeState, "explore")
        seq.seqResetSequence()
    
    elif stateManager.checkState("end"):
        if PRINT_MAP_AT_END:
            mapper.node_grid.print_grid()
        robot.comunicator.sendMap(mapper.get_grid_for_bonus())
        robot.comunicator.sendEndOfPlay()
    
    elif stateManager.checkState("stuck"):
        if SHOW_DEBUG:
            print("IS IN STUCK")
        seq.startSequence()
        if seq.simpleEvent():
            robot.auto_decide_rotation = False
            robot.rotation_sensor = "gyro"
        seqMoveWheels(-0.5, -0.5)
        seqDelaySec(0.2)
        seqMoveWheels(0, 0)
        if seq.simpleEvent():
            mapper.block_front_vortex(robot.rotation.degrees)
            robot.auto_decide_rotation = True
        seq.simpleEvent(stateManager.changeState, "explore")
        seq.seqResetSequence()
    
    """
    elif stateManager.checkState("go_back"):
        if SHOW_DEBUG:
            print("IS IN GO BACK")
        seq.startSequence()
        grid = mapper.get_node_grid()
        move = go_back_agent.get_action(grid)
        if SHOW_DEBUG:
            print("move: ", move)
        if move is None:
            stateManager.changeState("end")
            seq.seqResetSequence()
        else:
            node = mapper.robot_node

            if seqMoveToRelativeTile(move[0], move[1]):
                mapper.set_robot_node(robot.position)
            seq.seqResetSequence()

            if SHOW_DEBUG:
                print("rotation:", robot.rotation)
                print("position:", robot.position)
    """
    if SHOW_DEBUG:
        print("robot time:", robot.comunicator.remainingTime)
    robot.comunicator.update()
    window_n = 0
        


