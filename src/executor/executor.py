from flow_control import state_machines
from flow_control.delay import DelayManager

from executor.stuck_detector import StuckDetector

from robot.robot import Robot

from mapping.mapper import Mapper

from agents.granular_navigation_agent.granular_navigation_agent import Agent

from fixture_detection.fixture_detection import FixtureDetector

from flags import SHOW_DEBUG

class Executor:
    def __init__(self, agent: Agent, mapper: Mapper, robot: Robot) -> None:
        self.agent = agent
        self.mapper = mapper
        self.robot = robot

        self.delay_manager = DelayManager()
        self.stuck_detector = StuckDetector()

        self.state_machine = state_machines.StateManager("init")
        self.sequencer = state_machines.SequenceManager(reset_function=self.delay_manager.reset_delay)

        self.fixture_detector = FixtureDetector()
        
        # Flags
        self.mapping_enabled = False
        self.victim_reporting_enabled = False

        # Sequential functions used frequently
        self.seq_print =           self.sequencer.make_simple_event( print)
        self.seq_move_wheels =     self.sequencer.make_simple_event( self.robot.move_wheels)

        self.seq_rotate_to_angle = self.sequencer.make_complex_event(self.robot.rotate_to_angle)
        self.seq_move_to_coords =  self.sequencer.make_complex_event(self.robot.move_to_coords)
        self.seq_delay_seconds =   self.sequencer.make_complex_event(self.delay_manager.delay_seconds)

    def update(self):
        while self.robot.do_loop():
            # Updates robot position and rotation, sensor positions, etc.
            self.robot.update()

            self.delay_manager.update(self.robot.time)
            self.stuck_detector.update(self.robot.position,
                                       self.robot.previous_position,
                                       self.robot.drive_base.get_wheel_direction())
            
            self.do_mapping()
            
            
                
    def do_mapping(self):
        if self.mapping_enabled:
                # Floor and lidar mapping
                self.mapper.update(self.robot.get_point_cloud(), 
                                   self.robot.get_out_of_bounds_point_cloud(), 
                                   self.robot.get_camera_images(), 
                                   self.robot.position, 
                                   self.robot.orientation)
        else:
            # Only position and rotation
            self.mapper.update(robot_position=self.robot.position, 
                                robot_orientation=self.robot.orientation)


    def seq_calibrate_robot_rotation(self):
        """
        Calibrates the robot rotation using the gps.
        """
        if self.sequencer.simple_event():
            self.robot.auto_decide_orientation_sensor = False
        self.seq_move_wheels(-1, -1)
        self.seq_delay_seconds(0.1)
        if self.sequencer.simple_event(): 
            self.robot.orientation_sensor = self.robot.GPS
        self.seq_move_wheels(1, 1)
        self.seq_delay_seconds(0.1)
        if self.sequencer.simple_event(): 
            self.robot.orientation_sensor = self.robot.GYROSCOPE
        self.seq_delay_seconds(0.1)
        self.seq_move_wheels(0, 0)
        self.seq_move_wheels(-1, -1)
        self.seq_delay_seconds(0.1)
        self.seq_move_wheels(0, 0)
        if self.sequencer.simple_event():
            self.robot.auto_decide_orientation_sensor = True
            





    


