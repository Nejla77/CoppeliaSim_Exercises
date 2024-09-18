import vrep
import numpy as np
import time
import math


# variables
diameterWheel = 0.1
circumferenceWheel = 3.14 * diameterWheel
speed = 5
# for odometry
xPos = 0.0
yPos = 0.0
angle = 0.0



# given function to check position
def checkPose(clientID):
    res,base=vrep.simxGetObjectHandle(clientID,'youBot_center',vrep.simx_opmode_oneshot_wait)
    base_pos=vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_streaming)
    base_orient= vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_streaming)
    vrep.simxGetPingTime(clientID) # make sure that all streaming data has reached the client at least once
    return base_pos, base_orient


# negative Numbers direct to left, poitive numbers to right
def wheelVel(forwBackVel, leftRightVel, rotVel):
    frontLeft = forwBackVel - leftRightVel - rotVel
    rearLeft = forwBackVel - leftRightVel - rotVel
    rearRight = forwBackVel + leftRightVel + rotVel
    frontRight = forwBackVel + leftRightVel + rotVel
    return np.array([frontLeft, rearLeft, rearRight, frontRight])


def straightRobot(clientID, wheelJoints, distanceMeter):
    # variables
    global speed
    global diameterWheel
    global circumferenceWheel

    # compute calculations
    if distanceMeter < 0:
        velocity = -1*speed
    else:
        velocity = speed
    distanceMeter = abs(distanceMeter)
    sleepTime = (distanceMeter / ((diameterWheel/2) * abs(velocity)))

    wheelVelocities = wheelVel(velocity, 0, 0)

    print("start straight")
    # apply calculations
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],wheelVelocities[i],vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)

    print("moving for: " + str(sleepTime))
    time.sleep(sleepTime)

    # set wheel velocity to 0
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)
    print("end straight")

    return sleepTime


# just used this magic function because it worked by far the best
# angle in radian
def rotateRobot(clientID, wheelJoints, angle):
    # variables
    global speed
    global diameterWheel
    global circumferenceWheel
    rotation = (1/(2*3.1416)) * angle

    # compute calculations
    # speed 5 and time 5 is a half rotation
    if rotation < 0:
        velocity = -1*speed
    else:
        velocity = speed
    rotation = abs(rotation)
    sleepTime = 5 * 2 * rotation

    wheelVelocities = wheelVel(0, 0, velocity)

    print("start rotation")
    # apply calculations
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],wheelVelocities[i],vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)

    print("moving for: " + str(sleepTime))
    time.sleep(sleepTime)

    # set wheel velocity to 0
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)
    print("end rotation")

    # time, xVelocity, yVelocity, angle
    return sleepTime; #, 0, 5, angle

# angle in rad
def moveRobot(clientID, wheelJoints, angle, distance):
    rotateRobot(clientID, wheelJoints, angle)
    straightRobot(clientID, wheelJoints, distance)


def odometry(angleR, distanceR):
    global xPos
    global yPos
    global angle

    angle -= angleR
    angle = angle % (2*3.1416)
    xVec = math.sin(angle) * distanceR
    xPos += xVec
    yVec = math.cos(angle) * distanceR
    yPos -= yVec

    return xPos, yPos, angle

def printPosition(clientID):
    global xPos
    global yPos 
    global angle
    pos, orient = checkPose(clientID)
    print("global [PosX, PosY, AngZ]: "+str(np.round(pos[1][0], 5))+", "+str(np.round(pos[1][1], 5))+", "+str(np.round(orient[1][2], 5)))
    print("local  [PosX, PosY, AngZ]: " + str(np.round(xPos, 5)) + ", " + str(np.round(yPos, 5)) + ", " + str(np.round(angle, 5)))



def main():
    global xPos
    global yPos
    global angle
    
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True, 3000,5)
    if clientID!=-1:
        print ('Connected to remote API server')

        emptyBuff = bytearray()

        # Start the simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # initiaize robot
        # Retrieve wheel joint handles:
        wheelJoints=np.empty(4, dtype=int); wheelJoints.fill(-1) #front left, rear left, rear right, front right
        res,wheelJoints[0]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_oneshot_wait)
        res,wheelJoints[1]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_oneshot_wait)
        res,wheelJoints[2]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_oneshot_wait)
        res,wheelJoints[3]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_oneshot_wait)

        # set wheel velocity to 0
        for i in range(0, 4):
            vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)
###############################################################################################################################
        pos, orient = checkPose(clientID)
        
        pos, orient = checkPose(clientID)
        xPos=pos[1][0]
        yPos=pos[1][1]
        angle=orient[1][2]

        angleR = -3.1416/4
        distanceR = -1
        moveRobot(clientID, wheelJoints, angleR, distanceR)
        odometry(angleR, distanceR)
        printPosition(clientID)
        angleR = 0 #-3.14/2
        distanceR = -1
        moveRobot(clientID, wheelJoints, angleR, distanceR)
        odometry(angleR, distanceR)
        printPosition(clientID)
        angleR = 3.1416/4
        distanceR = -1
        moveRobot(clientID, wheelJoints, angleR, distanceR)
        odometry(angleR, distanceR)
        printPosition(clientID)

###############################################################################################################################
        # Stop simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')

if __name__ == "__main__": main()
