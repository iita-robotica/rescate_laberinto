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
    
    # Explores and maps the maze
    elif stateManager.checkState("explore"):
        robot.autoDecideRotation = False
        robot.rotationSensor = "gyro"
        nube_de_puntos = robot.getDetectionPointCloud()
        #print("nube de puntos: ", nube_de_puntos)
        nueva_nube_de_puntos = []
        for pos in nube_de_puntos:
            nueva_nube_de_puntos.append([((pos[0] * 1) + robot.position[0]) * 1000, ((pos[1] * 1) + robot.position[1]) * 1000])
        #print("nueva nube de puntos: ", nueva_nube_de_puntos)
        for pos in nueva_nube_de_puntos:
            round_pos = [round(pos[0]), round(pos[1])]
            mi_grilla.add_point(round_pos)
        mi_grilla.print_grid()

        # lidar es robot.getLidar()
        # Grilla es mapping.getGrilla(lidar)
        # mejor posicion es predictor de laberinto.getMejorPosición(grilla)
        # moviemientos es pathfinding.getCamino(mejor posición)
        # robot.sequir camino(moviemientos)
        # repetir

        # If it encountered a hole
        if isHole():
            # Changes state and resets the sequence
            seq.simpleEvent(stateManager.changeState, "hole")
            seq.seqResetSequence()
        robot.moveWheels(0, 0)

    # What to do if it encounters a hole
    elif stateManager.checkState("hole"):
        pass
        # reportar obstáculo a mapping
        # moverse para atras
        # volver a "explore"
        
    # Reports a victim
    elif stateManager.checkState("report_victim"):
        pass
        # Espera 3 segundos
        # manda reporte a robot
        # documenta vícitma
        # vuelve a explore
    
    elif stateManager.checkState("teleported"):
        seq.startSequence()
        # parar mapping
        doWallMapping = False
        seqCalibrateRobotRotation()
        # Changes state and resets the sequence
        seq.simpleEvent(stateManager.changeState, "explore")
        seq.seqResetSequence()
