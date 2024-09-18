import vrep
import numpy as np
import time
import math

# Robot Geometry
diameter         = 0.1                 # Diameter of the wheels in m
perimeter        = math.pi * diameter  # Perimeter of the wheels in m
wheelDistanceVer = 0.471               # Vertical distance between the wheels
wheelDistanceHor = 0.30046             # Horizontal distance between the wheels


def moveForward(distance, speed, clientID, wheelJoints):
    correctionFactor = 1.045

    # set wheel velocity to 0
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)

    # calculate velocity for each wheel
    wheelVelocities = wheelVel(speed, 0, 0)
    vrep.simxPauseCommunication(clientID, True)

    # set wheel velocity to calculated values
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],wheelVelocities[i],vrep.simx_opmode_oneshot)

    vrep.simxPauseCommunication(clientID, False)

    # keep the velocity for the required amount of time
    time.sleep((distance / perimeter) * 2 * math.pi / speed * correctionFactor)

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


def wheelVel(forwBackVel, leftRightVel, rotVel):
    # set individual wheel velocities
    fl = + forwBackVel - leftRightVel - rotVel
    rl = + forwBackVel + leftRightVel - rotVel
    rr = + forwBackVel - leftRightVel + rotVel
    fr = + forwBackVel + leftRightVel + rotVel
    return np.array([fl,rl,rr,fr])


def main():
    # connect to coppeliasim
    print ('Program started')
    vrep.simxFinish(-1) 
    clientID=vrep.simxStart('127.0.0.1',19997,True,True, 2000,5)
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
        armJointsHandle = [0] * 5
        
        #--------------------------------------------------arm------------------------------------------------------------------
        # Retrieve arm joint handles
        for i in range(0, 5):
            res,armJointsHandle[i] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint%d' % i, vrep.simx_opmode_oneshot_wait)
        
        # Move the third joint of the arm four times
        for i in range(0, 4):
            res = vrep.simxSetJointTargetPosition(clientID, armJointsHandle[2], i*math.pi/8, vrep.simx_opmode_oneshot)
            time.sleep(1)
        #-----------------------------------------------------------------------------------------------------------------------
        
        # set wheel velocity to 0
        for i in range(0, 4):
            vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)

        for i in range(0,4):
            # moved forwards
            moveForward(1, 5, clientID, wheelJoints)
            # turn right
            turnRight(90, 5, clientID, wheelJoints)

        vrep.simxPauseCommunication(clientID, False)
        # Stop simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')

if __name__ == "__main__": main()
