absuloute_dir = r'/home/ale/rescate_laberinto/Competencias/Robocup_2022/Refactored Code'
import sys

sys.path.append(absuloute_dir)
import fixture_detection, mapping, pathfinding, robot, state_machines, utils

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
    pass
    #robot.delayFirstTime = True

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

# Each timeStep
while robot.doLoop():
    # Updates robot position and rotation, sensor positions, etc.
    robot.update()
    print("state: ", stateManager.state)

    # Runs once when starting the game
    if stateManager.checkState("init"):
        seq.startSequence()

        if seqDelaySec(0.5):
            print("done1")

        # Calculates offsets in the robot position, in case it doesn't start perfectly centerd
        if seq.simpleEvent(calibratePositionOffsets()):
            print("done2")
            
        # Informs the mapping and pathfinding components of the starting position of the robot
        #seq.simpleEvent(pathfinding.registerStart())
        #seq.simpleEvent(mapping.registerStart())
            
        # Calibrates the robot rotation using the gps
        seqDelaySec(0.5)
        if seq.simpleEvent(): robot.rotationDetectionType = "gps"
        seqMoveWheels(1, 1)
        seqDelaySec(0.2)
        if seq.simpleEvent(): robot.rotationDetectionType = "gyroscope"
        seqDelaySec(0.2)
        seqMoveWheels(0, 0)
        seqMoveWheels(-1, -1)
        seqDelaySec(0.4)
        seqMoveWheels(0, 0)

        # Starts mapping walls
        if seq.simpleEvent():
            mapping.doWallMapping = True

        # Changes state and resets the sequence
        seq.simpleEvent(stateManager.changeState("explore"))
        seq.seqResetSequence()
    
    # Explores and maps the maze
    elif stateManager.checkState("explore"):
        robot.moveWheels(0, 0)

    # What to do if it encounters a hole
    elif stateManager.checkState("hole"):
        pass

    # Reports a victim
    elif stateManager.checkState("report_victim"):
        pass
