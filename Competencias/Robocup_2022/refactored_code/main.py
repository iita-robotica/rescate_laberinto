import cv2 as cv
import numpy as np
import time
import copy
import math

from data_processing import fixture_detection
import utilities, state_machines, robot, mapping
from algorithms.expandable_node_grid.bfs import bfs

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
do_victim_reporting = False


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
    return seqMoveToCoords((initial_position[0] + x, initial_position[1] + y))

def seqMoveToRelativeTile(x, y):
    node = mapper.robot_node
    tile = [node[0] // 2 + x, node[1] // 2 + y]
    return seqMoveToCoords(correct_position([tile[0] * TILE_SIZE, tile[1] * TILE_SIZE]))

def is_complete(grid, robot_node):
        possible_nodes = bfs(grid, robot_node, 500)
        if len(possible_nodes) == 0:
            return True
        return False

def robot_fits(robot_node, show_debug=False):
    robot_diameter_in_nodes = int(math.ceil(robot.diameter * mapper.lidar_grid.multiplier))
    robot_radious_in_nodes = robot_diameter_in_nodes // 2
    min_x = int(robot_node[0] - robot_radious_in_nodes)
    max_x = int(robot_node[0] + robot_radious_in_nodes+ 1)
    min_y = int(robot_node[1] - robot_radious_in_nodes)
    max_y = int(robot_node[1] + robot_radious_in_nodes + 1)
    min_x = max(min_x, 0)
    max_x = min(max_x, mapper.lidar_grid.grid.shape[0])
    min_y = max(min_y, 0)
    max_y = min(max_y, mapper.lidar_grid.grid.shape[1])
    #print("square: ", min_x, max_x, min_y, max_y)
    square = mapper.lidar_grid.get_bool_array()[min_y:max_y, min_x:max_x]

    square1 = copy.deepcopy(square).astype(np.uint8)
    square1 = square1 * 255

    if show_debug:
        try:
            cv.imshow("square", square1.astype(np.uint8))
        except:
            pass

    return np.count_nonzero(square)

def correct_position(robot_position):
    print("INITIAL POSITION: ", robot_position)
    max_correction = 2
    exageration_factor = 1.5
    robot_node = [round(p * mapper.lidar_grid.multiplier) for p in robot_position]
    robot_node = [robot_node[0] + mapper.lidar_grid.offsets[0], robot_node[1] + mapper.lidar_grid.offsets[1]]
    
    robot_fits(robot_node, show_debug=True)

    best_node = {"pos":robot_node, "dist":0, "amount":robot_fits(robot_node)}
    for x in range(-max_correction, max_correction + 1):
        for y in range(-max_correction, max_correction + 1):
            possible_pos = [robot_node[0] + (x * exageration_factor), robot_node[1] + (y * exageration_factor)]
            distance = math.sqrt(abs(x) ** (2) + abs(y) ** 2)
            amount_of_nodes = robot_fits(possible_pos)
            if amount_of_nodes < best_node["amount"]:
                best_node["pos"] = [p - 0.5 for p in possible_pos]
                best_node["dist"] = distance
                best_node["amount"] = amount_of_nodes
            elif amount_of_nodes == best_node["amount"]:
                if distance < best_node["dist"]:
                    best_node["pos"] = [p - 0.5 for p in possible_pos]
                    best_node["dist"] = distance
                    best_node["amount"] = amount_of_nodes

    final_pos = [(p - o) / mapper.lidar_grid.multiplier for p, o in zip(best_node["pos"], mapper.lidar_grid.offsets)]
    print("CORRECTED POSITION: ", final_pos)
    return final_pos

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
    
    if do_mapping:
        images = robot.get_camera_images()
        for index, image in enumerate(images):
            angle = (index - 1) * 90

            rot_img = np.rot90(image, -1)

            victims = fixture_detection.find_victims(rot_img)
            if len(victims) > 0:
                letter = fixture_detection.classify_fixture(victims[0])
                if letter is not None:
                    mapper.load_fixture(letter, angle, robot.rotation)
                    
                break
    
    if is_complete(mapper.node_grid, mapper.robot_node) and mapper.node_grid.get_node(mapper.robot_node).is_start:
        seq.resetSequence()
        stateManager.changeState("end")
    
    fixture_detection.tune_filter(robot.get_camera_images()[1])

    # Updates state machine
    if not stateManager.checkState("init"):
        if robot.is_stuck():
            mapper.block_front_vortex(robot.rotation)
    
    
    if mapper.get_fixture().exists and not mapper.get_fixture().reported and do_victim_reporting:
        if not stateManager.checkState("report_victim"):
            seq.resetSequence()
            stateManager.changeState("report_victim")

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

        grid = mapper.get_node_grid()
        move = closest_position_agent.get_action(grid)
        print("move: ", move)
        node = mapper.robot_node

        if seqMoveToRelativeTile(move[0], move[1]):
            mapper.set_robot_node(robot.position)
        seq.seqResetSequence()

        print("rotation:", robot.rotation)
        print("position:", robot.position)

    # Reports a victim
    elif stateManager.checkState("report_victim"):
        seq.startSequence()
        seqMoveWheels(0, 0)
        if seq.simpleEvent():
            print("STOPPED")
        seqDelaySec(1.2)
        if seq.simpleEvent():
            fixture = mapper.get_fixture()
            robot.comunicator.sendVictim(robot.position, fixture.type)
            fixture.reported = True
            mapper.load_wall_fixture(letter, fixture.detection_angle)
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
        robot.comunicator.sendMap(mapper.get_grid_for_bonus())
        robot.comunicator.sendEndOfPlay()
