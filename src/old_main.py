import numpy as np
import cv2 as cv

from data_structures.vectors import Position2D

from flow_control import sequencer
from flow_control.delay import DelayManager

from executor.stuck_detector import StuckDetector

from robot.robot import Robot

from mapping.mapper import Mapper

from agents.granular_navigation_agent.granular_navigation_agent import GranularNavigationAgent

from fixture_detection.fixture_clasification import FixtureDetector

from flags import SHOW_DEBUG

# World constants
time_step = 32
TILE_SIZE = 0.06
TIME_IN_ROUND = 8 * 60

# Components
#Robot
robot = Robot(time_step)

# Stores, changes and compare states
state_machine = sequencer.StateManager("init")

# Sequence manager
# Resets flags that need to be in a certain value when changing sequence, for example when changing state
def reset_sequence_flags():
    robot.delay_first_time = True
seq = sequencer.Sequencer(reset_function=reset_sequence_flags)

delay_manager = DelayManager()
stuck_detector = StuckDetector()

# Fixture detection
fixture_detector = FixtureDetector()

# Mapper
mapper = Mapper(TILE_SIZE, robot.diameter)

# Agents
navigation_agent = GranularNavigationAgent(mapper)

# Variables
do_mapping = False
do_victim_reporting = False


# Functions
# Sequential functions used frequently
seq_print = seq.make_simple_event(print)
seq_delay_seconds = seq.make_complex_event(delay_manager.delay_seconds)
seq_move_wheels = seq.make_simple_event(robot.move_wheels)
seq_rotate_to_angle = seq.make_complex_event(robot.rotate_to_angle)
seq_move_to_coords = seq.make_complex_event(robot.move_to_coords)
seq_reset_sequence_flags = seq.make_simple_event(reset_sequence_flags)


def calibrate_position_offsets():
    """
    Calculates offsets in the robot position, in case it doesn't start perfectly centerd
    """
    actualTile = robot.position // TILE_SIZE

    robot.position_offsets = (actualTile * TILE_SIZE - robot.position).apply_to_all(round) + TILE_SIZE // 2

    robot.position_offsets = robot.position_offsets % TILE_SIZE
    print("positionOffsets: ", robot.position_offsets)

def seq_calibrate_robot_rotation():
    """
    Calibrates the robot rotation using the gps
    """
    if seq.simple_event():
        robot.auto_decide_orientation_sensor = False
    seq_move_wheels(-1, -1)
    seq_delay_seconds(0.1)
    if seq.simple_event(): robot.orientation_sensor = robot.GPS
    seq_move_wheels(1, 1)
    seq_delay_seconds(0.1)
    if seq.simple_event(): robot.orientation_sensor = robot.GYROSCOPE
    seq_delay_seconds(0.1)
    seq_move_wheels(0, 0)
    seq_move_wheels(-1, -1)
    seq_delay_seconds(0.1)
    seq_move_wheels(0, 0)
    if seq.simple_event():
        robot.auto_decide_orientation_sensor = True

# Each timeStep
while robot.do_loop():
    # Updates robot position and rotation, sensor positions, etc.
    robot.update()

    delay_manager.update(robot.time)
    stuck_detector.update(robot.position,
                          robot.previous_position,
                          robot.drive_base.get_wheel_direction())
    
    if do_mapping:
        # Floor and lidar mapping
        mapper.update(robot.get_point_cloud(), 
                      robot.get_out_of_bounds_point_cloud(), 
                      robot.get_camera_images(), 
                      robot.position, 
                      robot.orientation)

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
                        mapper.load_fixture_to_tile(letter, angle, robot.orientation.degrees)
                        
                    break
    
    else:
        # Only position and rotation
        mapper.update(robot_position=robot.position, 
                      robot_orientation=robot.orientation)
    
    #fixture_detector.tune_filter(robot.get_camera_images()[1])

    # Updates state machine
    if not state_machine.check_state("init"):
        if SHOW_DEBUG:
            print("stuck_counter: ", stuck_detector.stuck_counter)
        if stuck_detector.is_stuck():
            if SHOW_DEBUG:
                print("FRONT BLOCKED")
            mapper.block_front_vortex(robot.orientation)
            if not state_machine.check_state("stuck"):
                seq.reset_sequence()
                state_machine.change_state("stuck")
    
        """
        if mapper.get_fixture_from_tile().exists and not mapper.get_fixture_from_tile().reported and do_victim_reporting:
            if not stateManager.checkState("report_victim"):
                seq.resetSequence()
                stateManager.changeState("report_victim")
        """

        if robot.comunicator.remaining_time < 30:
            if not state_machine.check_state("go_back") and not state_machine.check_state("end"):
                seq.reset_sequence()
                state_machine.change_state("go_back")

    if SHOW_DEBUG:
        print("state: ", state_machine.state)

    # Runs once when starting the game
    if state_machine.check_state("init"):
        seq.start_sequence()
        seq_delay_seconds(0.5)
        # Calculates offsets in the robot position, in case it doesn't start perfectly centerd
        seq.simple_event(calibrate_position_offsets)
        # Informs the mapping components of the starting position of the robot
        seq.simple_event(mapper.register_start, robot.position)
        # Calibrates the rotation of the robot using the gps
        seq_calibrate_robot_rotation()
        # Starts mapping walls
        if seq.simple_event():
            do_mapping = True
            do_victim_reporting = True
        if seq.simple_event():
            #do_mapping = False
            #do_victim_reporting = False
            pass
        # Changes state and resets the sequence
        seq.simple_event(state_machine.change_state, "explore")
        seq.seq_reset_sequence()

    elif state_machine.check_state("stop"):
        seq.start_sequence()
        seq_move_wheels(0, 0)

    # Explores and maps the maze
    elif state_machine.check_state("explore"):
        seq.start_sequence()

        navigation_agent.update()

        if seq_move_to_coords(navigation_agent.get_target_position()):
            mapper.set_robot_node(robot.position)

        seq.seq_reset_sequence()

        if SHOW_DEBUG:
            print("rotation:", robot.orientation)
            print("position:", robot.position)

    # Reports a victim
    elif state_machine.check_state("report_victim"):
        seq.start_sequence()
        seq_move_wheels(0, 0)
        if seq.simple_event() and SHOW_DEBUG:
            print("STOPPED")
        seq_delay_seconds(3)
        if seq.simple_event():
            #robot.comunicator.sendVictim(robot.position, fixture.type)
            #fixture.reported = True
            pass
        seq.simple_event(state_machine.change_state, "explore")
        seq.seq_reset_sequence()
    
    elif state_machine.check_state("teleported"):
        seq.start_sequence()
        # parar mapping
        do_mapping = False
        seq_calibrate_robot_rotation()
        # Changes state and resets the sequence
        seq.simple_event(state_machine.change_state, "explore")
        seq.seq_reset_sequence()
    
    elif state_machine.check_state("end"):
        robot.comunicator.send_map(mapper.get_grid_for_bonus())
        robot.comunicator.send_end_of_play()
    
    elif state_machine.check_state("stuck"):
        if SHOW_DEBUG:
            print("IS IN STUCK")
        seq.start_sequence()
        if seq.simple_event():
            robot.auto_decide_orientation_sensor = False
            robot.orientation_sensor = robot.GYROSCOPE
        seq_move_wheels(-0.5, -0.5)
        seq_delay_seconds(0.2)
        seq_move_wheels(0, 0)
        if seq.simple_event():
            mapper.block_front_vortex(robot.orientation)
            robot.auto_decide_orientation_sensor = True
        seq.simple_event(state_machine.change_state, "explore")
        seq.seq_reset_sequence()
    
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
        print("robot time:", robot.comunicator.remaining_time)
    robot.comunicator.update()
        


