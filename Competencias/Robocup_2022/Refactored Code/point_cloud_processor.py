
def processPointCloud(pc, robotPos):
    "For each point in the poincloud, sum its position with the position of the robot"
    newPC = []
    for point in pc:
        newPC.append([point[0] + robotPos[0], point[1] + robotPos[1]])
    return newPC

