import vrep
import numpy as np
import time

# wheel diameter [m]
WHEEL_DIAMETER = 0.1

# odometry calculation tolerance
TOLERANCE = 0.009

# intervall for movement
dt = 0.001


# vector to matrix
def to_matrix(l, n):
    return [l[i:i + n] for i in range(0, len(l), n)]


# transforms vector to matrix
def transform(vector):
    vector.pop(0)
    vector.pop(0)
    matrix = to_matrix(vector, 4)
    return matrix


def setVelocity(clientID, wheels, velocities):
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheels[i], velocities[i], vrep.simx_opmode_oneshot)


def wheelVel(forwBackVel, leftRightVel, rotVel):
    return [-forwBackVel - leftRightVel + rotVel,
            -forwBackVel + leftRightVel + rotVel,
            -forwBackVel - leftRightVel - rotVel,
            -forwBackVel + leftRightVel - rotVel]


# reset speed to 0
def resetWheels(clientID, wheels):
    # set wheel velocity to 0
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheels[i], 0, vrep.simx_opmode_oneshot)


# moves robot or rotates
def moveRobot(forwBackVel, leftRightVel, velocities, destination, dt):
    position = [0, 0, 0]

    R = WHEEL_DIAMETER / 2
    lx = 0.30046 / 2
    lz = 0.471 / 2

    invL = 1 / (lx + lz)

    while (np.absolute(position[0] - destination[0]) > TOLERANCE
           or np.absolute(position[1] - destination[1]) > TOLERANCE
           or np.absolute(position[2] - destination[2]) > TOLERANCE):
        time.sleep(dt)

        deltaTheta = R / 4 * (-invL * velocities[0] + invL * velocities[3] - invL * velocities[1] + invL * velocities[2])

        position[0] += R * forwBackVel * dt * np.cos(position[2])
        position[1] += R * leftRightVel * dt * np.sin(position[2])
        position[2] += deltaTheta * dt


# calculates the absolute point of the relative sensor data
def calculateAbsolutePoint(clientID, sensor, relativeDistance, rotationAngle):
    res, sensor_pos = vrep.simxGetObjectPosition(clientID, sensor, -1, vrep.simx_opmode_buffer)
    point = [0, 0]
    if abs(rotationAngle) <= np.pi / 2 and rotationAngle > 0:
        point[0] = np.sin(rotationAngle) * relativeDistance
        point[1] = -np.cos(rotationAngle) * relativeDistance
    elif abs(rotationAngle) > np.pi / 2 and rotationAngle > 0:
        rotationAngle = np.pi - abs(rotationAngle)
        point[0] = np.sin(rotationAngle) * relativeDistance
        point[1] = np.cos(rotationAngle) * relativeDistance
    elif abs(rotationAngle) <= np.pi / 2 and rotationAngle < 0:
        point[0] = np.sin(rotationAngle) * relativeDistance
        point[1] = -np.cos(rotationAngle) * relativeDistance
    elif abs(rotationAngle) > np.pi / 2 and rotationAngle < 0:
        rotationAngle = np.pi - abs(rotationAngle)
        point[0] = np.sin(-rotationAngle) * relativeDistance
        point[1] = np.cos(rotationAngle) * relativeDistance
    point = [sensor_pos[0] + point[0], sensor_pos[1] + point[1]]
    return point


# checks if given sensor data is on the goal
def isLaserOnGoal(clientID, goal_pos, base, sensor, sensorData, number):
    res, base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_buffer)
    res, base_orient = vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_buffer)
    distanceGoalYoubot = np.sqrt(np.square(goal_pos[0] - base_pos[0]) + np.square(goal_pos[1] - base_pos[1]))
    laserDistance = sensorData[3]
    relativeRotation = number*(np.pi/180)
    rotationAngle = base_orient[2] + relativeRotation
    if base_orient[2] < 0 and abs(rotationAngle) > np.pi:
        rotationAngle = np.pi + (np.pi + rotationAngle)
    elif base_orient[2] > 0 and abs(rotationAngle) > np.pi:
        rotationAngle = -(np.pi + (np.pi - rotationAngle))

    point = calculateAbsolutePoint(clientID, sensor, laserDistance, rotationAngle)

    distanceGoalPoint = np.sqrt(np.square(goal_pos[0] - point[0]) + np.square(goal_pos[1] - point[1]))
    distancePointYoubot = np.sqrt(np.square(point[0] - base_pos[0]) + np.square(point[1] - base_pos[1]))
    if abs(distancePointYoubot - (distanceGoalYoubot + distanceGoalPoint)) < 0.01:
        return True
    else:
        return False


# rotates the robot until it faces the goal direction
def findGoalDirection(clientID, wheelJoints, goal_pos, base, sensor, direction):
    print("Orient toward goal")
    res, base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_buffer)
    distanceGoalYoubot = np.sqrt(np.square(goal_pos[0] - base_pos[0]) + np.square(goal_pos[1] - base_pos[1]))
    found = False
    while not found:
        res, aux, auxD = vrep.simxReadVisionSensor(clientID, sensor, vrep.simx_opmode_buffer)
        sensorDataLeft = transform(auxD[1])
        if direction == -1:
            frontLaserLeftDistance = sensorDataLeft[0][3]
        else:
            frontLaserLeftDistance = sensorDataLeft[len(sensorDataLeft)-1][3]
        res, base_orient = vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_buffer)

        point = calculateAbsolutePoint(clientID, sensor, frontLaserLeftDistance, base_orient[2])

        distanceGoalPoint = np.sqrt(np.square(goal_pos[0] - point[0]) + np.square(goal_pos[1] - point[1]))
        distancePointYoubot = np.sqrt(np.square(point[0] - base_pos[0]) + np.square(point[1] - base_pos[1]))
        if abs(distanceGoalYoubot - (distanceGoalPoint + distancePointYoubot)) < 0.01\
                or abs(distancePointYoubot - (distanceGoalYoubot + distanceGoalPoint)) < 0.01:
            found = True

        velocities = wheelVel(0, 0, direction*5)
        vrep.simxPauseCommunication(clientID, True)
        setVelocity(clientID, wheelJoints, velocities)
        vrep.simxPauseCommunication(clientID, False)
        moveRobot(0, 0, velocities, [0, 0, direction*-2 * np.pi / 720], dt)


def moveAlongWall(clientID, wheelJoints, hokuyo, wallSide):
    print("Move along obstacle")
    turn = True
    while turn:
        velocities = wheelVel(0, 0, wallSide * 5)
        vrep.simxPauseCommunication(clientID, True)
        setVelocity(clientID, wheelJoints, velocities)
        vrep.simxPauseCommunication(clientID, False)
        moveRobot(0, 0, velocities, [0, 0, wallSide * -0.01], dt)
        if wallSide == 1:
            res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo, vrep.simx_opmode_buffer)
            sensorData = transform(auxD[1])
            nintyDegreeDistance = sensorData[85][3]

        else:
            res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo, vrep.simx_opmode_buffer)
            sensorData = transform(auxD[1])
            nintyDegreeDistance = sensorData[256][3]

        smallestDistance = 5.0
        for i in sensorData:
            if i[3] < smallestDistance:
                smallestDistance = i[3]
        if np.absolute(smallestDistance - nintyDegreeDistance) < 0.01:
            turn = False

    onWall = True
    nintyDegreeDistance = 5.0
    corner = False
    while onWall:
        velocities = wheelVel(-10, 0, 0)
        vrep.simxPauseCommunication(clientID, True)
        setVelocity(clientID, wheelJoints, velocities)
        vrep.simxPauseCommunication(clientID, False)
        moveRobot(-10, 0, velocities, [-0.01, 0, 0], dt)

        if wallSide == 1:
            res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo, vrep.simx_opmode_buffer)
            sensorData = transform(auxD[1])
            length = len(sensorData)
            nintyDegreeDistanceBefore = nintyDegreeDistance
            nintyDegreeDistance = sensorData[85][3]
            frontDistance = sensorData[length-1][3]

        else:
            res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo, vrep.simx_opmode_buffer)
            sensorData = transform(auxD[1])
            nintyDegreeDistanceBefore = nintyDegreeDistance
            nintyDegreeDistance = sensorData[256][3]
            frontDistance = sensorData[0][3]

        if (nintyDegreeDistanceBefore < 0.8 and nintyDegreeDistance > 1.5) or nintyDegreeDistance > 2.0:
            print("Leave obstacle")
            velocities = wheelVel(-10, 0, 0)
            vrep.simxPauseCommunication(clientID, True)
            setVelocity(clientID, wheelJoints, velocities)
            vrep.simxPauseCommunication(clientID, False)
            moveRobot(-10, 0, velocities, [-1, 0, 0], dt)
            onWall = False
        elif frontDistance < 0.8:
            corner = True
            velocities = wheelVel(0, 0, wallSide * 5)
            vrep.simxPauseCommunication(clientID, True)
            setVelocity(clientID, wheelJoints, velocities)
            vrep.simxPauseCommunication(clientID, False)
            moveRobot(0, 0, velocities, [0, 0, wallSide * -0.1], dt)
        elif nintyDegreeDistance < 0.4:
            corner = False
            velocities = wheelVel(0, 0, wallSide * 5)
            vrep.simxPauseCommunication(clientID, True)
            setVelocity(clientID, wheelJoints, velocities)
            vrep.simxPauseCommunication(clientID, False)
            moveRobot(0, 0, velocities, [0, 0, wallSide * -0.01], dt)
        elif not corner and nintyDegreeDistance > 0.6:
            velocities = wheelVel(0, 0, wallSide * -5)
            vrep.simxPauseCommunication(clientID, True)
            setVelocity(clientID, wheelJoints, velocities)
            vrep.simxPauseCommunication(clientID, False)
            moveRobot(0, 0, velocities, [0, 0, wallSide * 0.01], dt)


def main():
    print ('Program started')
    vrep.simxFinish(-1)  # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 2000, 5)

    if clientID != -1:
        print ('Connected to remote API server')

        # Start the simulation:
        vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

        # initialize robot
        # Retrieve wheel joint handles:
        wheelJoints = np.empty(4, dtype=int);
        wheelJoints.fill(-1)  # front left, rear left, rear right, front right
        res, wheelJoints[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
        res, wheelJoints[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
        res, wheelJoints[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)
        res, wheelJoints[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)

        # set Signal for Sensor
        vrep.simxSetIntegerSignal(clientID, 'handle_xy_sensor', 2, vrep.simx_opmode_oneshot)
        vrep.simxSetIntegerSignal(clientID, 'displaylasers', 1, vrep.simx_opmode_oneshot)

        # get object handles
        res, base = vrep.simxGetObjectHandle(clientID, 'youBot_center', vrep.simx_opmode_oneshot_wait)
        res, hokuyoRight = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor1', vrep.simx_opmode_oneshot_wait)
        res, hokuyoLeft = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor2', vrep.simx_opmode_oneshot_wait)
        res, goal = vrep.simxGetObjectHandle(clientID, 'Goal', vrep.simx_opmode_oneshot_wait)

        # first read of object properties
        vrep.simxReadVisionSensor(clientID, hokuyoRight, vrep.simx_opmode_streaming)
        vrep.simxReadVisionSensor(clientID, hokuyoLeft, vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(clientID, hokuyoRight, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(clientID, hokuyoLeft, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectOrientation(clientID, hokuyoRight, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectOrientation(clientID, hokuyoLeft, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(clientID, goal, -1, vrep.simx_opmode_streaming)

        # wait until all data has been read
        vrep.simxGetPingTime(clientID)

        # get goal and robot positions
        res, goal_pos = vrep.simxGetObjectPosition(clientID, goal, -1, vrep.simx_opmode_buffer)
        res, base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_buffer)

        # find goal direction
        findGoalDirection(clientID, wheelJoints, goal_pos, base, hokuyoLeft, 1)

        # move until goal has been reached
        checkLaser = True
        signal = hokuyoLeft
        rotation = -1
        ignoreGoal = 0
        # executes as long as not on the goal
        print("Drive toward nearest obstacle in direction of the goal")
        while not (abs(goal_pos[0] - base_pos[0]) < 1.0 and abs(goal_pos[1] - base_pos[1]) < 1.0):
            space = True
            goalDirectionSignal = hokuyoLeft
            goalRotation = -1
            while space:
                # moves the robot straight ahead
                velocities = wheelVel(-10, 0, 0)
                vrep.simxPauseCommunication(clientID, True)
                setVelocity(clientID, wheelJoints, velocities)
                vrep.simxPauseCommunication(clientID, False)
                moveRobot(-10, 0, velocities, [-0.01, 0, 0], dt)

                res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyoRight, vrep.simx_opmode_buffer)
                sensorDataRight = transform(auxD[1])
                res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyoLeft, vrep.simx_opmode_buffer)
                sensorDataLeft = transform(auxD[1])
                length1 = len(sensorDataRight)
                length2 = len(sensorDataLeft)

                res, base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_buffer)

                # if the robot is on the goal it breaks the loop and therefor ends the movement
                if abs(goal_pos[0] - base_pos[0]) < 1.5 and abs(goal_pos[1] - base_pos[1]) < 1.5:
                    break

                # if the robot sensed it's lasers on the goal for 1000times it changes direction to goal
                if ignoreGoal > 1000 and checkLaser:
                    checkLaser = False
                    findGoalDirection(clientID, wheelJoints, goal_pos, base, goalDirectionSignal, goalRotation)
                    break

                smallestDistanceRight = 5.0
                smallestDistanceLeft = 5.0
                sumDistanceRight = 5.0
                sumDistanceLeft = 5.0
                allowedDistanceRight = 1.0
                allowedDistanceLeft = 1.0
                # for the sensors on both sides it checks if one of them is on the goal and calculates minimal
                # distance to closest object
                for i in range(0, length1):
                    if checkLaser:
                        number = -120.0/length1*(length1 - i)
                        if isLaserOnGoal(clientID, goal_pos, base, hokuyoRight, sensorDataRight[i], number):
                            ignoreGoal = ignoreGoal + 1
                            goalDirectionSignal = hokuyoLeft
                            goalRotation = -1
                    sumDistanceRight = sumDistanceRight + sensorDataRight[i][3]
                    if sensorDataRight[i][3] < smallestDistanceRight:
                        smallestDistanceRight = sensorDataRight[i][3]
                        allowedDistanceRight = np.power(0.3 / (length1 - i), 1.0 / 6)

                for i in range(0, length2):
                    if checkLaser:
                        number = 120.0/length2*i
                        if isLaserOnGoal(clientID, goal_pos, base, hokuyoLeft, sensorDataLeft[i], number):
                            ignoreGoal = ignoreGoal + 1
                            goalDirectionSignal = hokuyoRight
                            goalRotation = 1
                    sumDistanceLeft = sumDistanceLeft + sensorDataLeft[i][3]
                    if sensorDataLeft[i][3] < smallestDistanceLeft:
                        smallestDistanceLeft = sensorDataLeft[i][3]
                        allowedDistanceLeft = np.power(0.3 / (i + 1), 1.0 / 6)
                if smallestDistanceLeft < smallestDistanceRight and smallestDistanceLeft < allowedDistanceLeft:
                    moveAlongWall(clientID, wheelJoints, hokuyoLeft, -1)
                    findGoalDirection(clientID, wheelJoints, goal_pos, base, hokuyoLeft, 1)
                    print("Drive toward nearest obstacle in direction of the goal")
                elif smallestDistanceRight < smallestDistanceLeft and smallestDistanceRight < allowedDistanceRight:
                    moveAlongWall(clientID, wheelJoints, hokuyoRight, 1)
                    findGoalDirection(clientID, wheelJoints, goal_pos, base, hokuyoRight, -1)
                    print("Drive toward nearest obstacle in direction of the goal")

            if checkLaser:
                findGoalDirection(clientID, wheelJoints, goal_pos, base, signal, rotation)

        findGoalDirection(clientID, wheelJoints, goal_pos, base, signal, rotation)
        velocities = wheelVel(-10, 0, 0)
        vrep.simxPauseCommunication(clientID, True)
        setVelocity(clientID, wheelJoints, velocities)
        vrep.simxPauseCommunication(clientID, False)
        moveRobot(-10, 0, velocities, [-1, 0, 0], dt)

        print("Hurray!!!")
        velocities = wheelVel(0, 0, -5)
        vrep.simxPauseCommunication(clientID, True)
        setVelocity(clientID, wheelJoints, velocities)
        vrep.simxPauseCommunication(clientID, False)
        moveRobot(0, 0, velocities, [0, 0,  np.pi], dt)

        # stop wheels
        vrep.simxPauseCommunication(clientID, True)
        resetWheels(clientID, wheelJoints)
        vrep.simxPauseCommunication(clientID, False)

        # Stop simulation:
        vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')


if __name__ == "__main__": main()
