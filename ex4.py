import vrep
import numpy as np
import time
import math
#import matplotlib.pyplot as plt


# Robot Geometry
diameter         = 0.1                 # Diameter of the wheels in m
perimeter        = math.pi * diameter  # Perimeter of the wheels in m
wheelDistanceVer = 0.471               # Vertical distance between the wheels
wheelDistanceHor = 0.30046             # Horizontal distance between the wheels


# robots global position
xPos = 0.0
yPos = 0.0
angle = 0.0
Obstacles=[]    # contains position of all detected obstacles


# update robots position
def odometry(angleR, distanceR):
    global xPos
    global yPos
    global angle

    angle += angleR*3.1415*2
    angle = angle % (2*3.1416)
    xVec = math.sin(angle) * distanceR
    xPos += xVec
    yVec = math.cos(angle) * distanceR
    yPos += yVec

    return xPos, yPos, angle

# given function to check position
def checkPose(clientID):
    global xPos
    global yPos
    global angle
    
    res,base=vrep.simxGetObjectHandle(clientID,'youBot_center',vrep.simx_opmode_oneshot_wait)
    base_pos=vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_streaming)
    base_orient= vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_streaming)
    vrep.simxGetPingTime(clientID) # make sure that all streaming data has reached the client at least once
    xPos=base_pos[1][0]
    yPos=base_pos[1][1]
    angle=-base_orient[1][2]
    return base_pos, base_orient



def wheelVel(forwBackVel, leftRightVel, rotVel):
    # set individual wheel velocities
    fl = + forwBackVel - leftRightVel - rotVel
    rl = + forwBackVel + leftRightVel - rotVel
    rr = + forwBackVel - leftRightVel + rotVel
    fr = + forwBackVel + leftRightVel + rotVel
    return np.array([fl,rl,rr,fr])


def moveForward(distance, speed, clientID, wheelJoints):
    correctionFactor = 1.045

    # set wheel velocity to 0
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)
    
    speed=abs(speed)
    if distance<0:
        speed=-speed

    # calculate velocity for each wheel
    wheelVelocities = wheelVel(speed, 0, 0)
    speed=abs(speed)
    vrep.simxPauseCommunication(clientID, True)

    # set wheel velocity to calculated values
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],wheelVelocities[i],vrep.simx_opmode_oneshot)

    vrep.simxPauseCommunication(clientID, False)

    # keep the velocity for the required amount of time
    time.sleep((abs(distance) / perimeter) * 2 * math.pi / speed * correctionFactor)

    # set wheel velocity to 0
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)

def turnRight(degree, speed, clientID, wheelJoints):
    # set wheel velocity to 0
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)
    
    # calculate velocity for each wheel
    wheelVelocities = wheelVel(0, 0, speed)
    vrep.simxPauseCommunication(clientID, True)
    
    # set wheel velocity to calculated values
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],wheelVelocities[i],vrep.simx_opmode_oneshot)
        
    vrep.simxPauseCommunication(clientID, False)
    
    # keep the velocity for the required amount of time
    time.sleep((1/perimeter) * (wheelDistanceVer + wheelDistanceHor) * math.pi * (degree * math.pi / 180) / abs(speed))

    # set wheel velocity to 0
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)

# isolates distance from sensor data
def getDistance(auxD,n):
    return auxD[1][4*n+5]

# calculates distances to obsticles around the robot
def getLeftFrontRightDistances(auxD1, auxD2):
    lastValue = int(len(auxD1[1])/4) - 1     # 342 value packages
    frontDistance = getDistance(auxD1, lastValue)
    rightDistance = getDistance(auxD1, int(lastValue * (30/120)))
    leftDistance = getDistance(auxD2, int(lastValue * (90/120)))
    return leftDistance, frontDistance, rightDistance

# prints distances to obsticles around the robot
def printDistances(left, front, right):
    print("Distance Left: " + str(left))
    print("Distance Front: " + str(front))
    print("Distance Right: " + str(right) + "\n")

# detects an obstacle in front of the robot
def getPoint(auxD1):
    global xPos
    global yPos
    global angle

    global Obstacles
    lastValue = int(len(auxD1[1])/4) - 1
    distance=getDistance(auxD1, int(lastValue))
    if distance<5.0:
        fullangle=angle+3.1415/2
        Obstacles.append((distance*math.cos(fullangle)+xPos, distance*math.sin(fullangle)+yPos))

        
def main():
    # connect to coppeliasim
    global Obstacles
    print ('Program started')
    vrep.simxFinish(-1) 
    clientID=vrep.simxStart('127.0.0.1', 19997, True, True, 3000, 5)
    if clientID != -1:
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
        time.sleep(1)


        # initialize sensors
        res = vrep.simxSetIntegerSignal(clientID, 'handle_xy_sensor', 2, vrep.simx_opmode_oneshot);
        vrep.simxSetIntegerSignal(clientID, 'displaylasers', 1, vrep.simx_opmode_oneshot);
        res1, hokuyo1 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor1', vrep.simx_opmode_oneshot_wait)
        res2, hokuyo2 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor2', vrep.simx_opmode_oneshot_wait)

        print("\ninit sensors\n")
        res1, aux1, auxD1 = vrep.simxReadVisionSensor(clientID, hokuyo1, vrep.simx_opmode_streaming)
        res2, aux2, auxD2 = vrep.simxReadVisionSensor(clientID, hokuyo2, vrep.simx_opmode_streaming)
        time.sleep(1)

        print("\nread sensors\n")
        res1, aux1, auxD1 = vrep.simxReadVisionSensor(clientID, hokuyo1, vrep.simx_opmode_buffer)
        res2, aux2, auxD2 = vrep.simxReadVisionSensor(clientID, hokuyo2, vrep.simx_opmode_buffer)
        time.sleep(1)


        # locates free space around the robot
        lastValue = int(len(auxD1[1])/4) - 1    
        middleValue = int(lastValue // 2)

        left, front, right = getLeftFrontRightDistances(auxD1, auxD2)
        printDistances(left, front, right)

        pos, orient = checkPose(clientID)
        print(str(pos))
        print(str(orient))
        
        # move backwards 1 meter
        moveForward(-1, 5, clientID, wheelJoints)
        odometry(0, -1)

        pos, orient = checkPose(clientID)
        print(str(pos))
        print(str(orient))

        print("\nread sensors\n")
        res1, aux1, auxD1 = vrep.simxReadVisionSensor(clientID, hokuyo1, vrep.simx_opmode_buffer)
        res2, aux2, auxD2 = vrep.simxReadVisionSensor(clientID, hokuyo2, vrep.simx_opmode_buffer)
        time.sleep(1)
        
        left, front, right = getLeftFrontRightDistances(auxD1, auxD2)
        printDistances(left, front, right)
        # rotate 3.6 deg 100 times and read data from sensor about the
        # obstacle in front of the robot
        for i in range(0, 100):
            turnRight(3.6, 5, clientID, wheelJoints)
            odometry(0.01, 0)

            print("\nread sensors\n")
            res1, aux1, auxD1 = vrep.simxReadVisionSensor(clientID, hokuyo1, vrep.simx_opmode_buffer)
            res2, aux2, auxD2 = vrep.simxReadVisionSensor(clientID, hokuyo2, vrep.simx_opmode_buffer)

            getPoint(auxD1)
            left, front, right = getLeftFrontRightDistances(auxD1, auxD2)
            printDistances(left, front, right)



        pos, orient = checkPose(clientID)

        print(str(pos))
        print(str(orient))
        x=[]
        y=[]
        for a, b in Obstacles:
            x.append(-a)
            y.append(b)

        print(Obstacles)

        #plotting points on a graph
        #plt.plot(x, y, 'ko')
        #plt.plot(-xPos, yPos, 'r*')
        #plt.show()





###############################################################################################################################
        # Stop simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')

if __name__ == "__main__": main()
