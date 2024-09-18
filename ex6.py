import vrep
import time
import os
import math as m
import numpy
import copy

# robot state information
orientationVector = [0.0, -1.0]
startPos = [0.0, 0.0]
startOri = 0.0
gdeg = 0.0
relativePos = [0.0, 0.0]
wheels = [-1] * 4

clientID = -1
pathPlanner = None

# mathematical sgn function
def sgn(n):
    if (n < 0):
        return -1
    else:
        return 1

# keep degrees between 0 and 360 deg
def normalizeDegrees(deg):
    if(deg > 360):
        deg -= 360
    elif(deg < 0):
        deg = 360 + deg
    return deg


def getAngleToGoal(goal):
    global orientationVector
    # remember: vecA * vecB = |vecA| * |vecB| * cos(phi)
    # we have to normalize our goal vector!
    distanceToGoal = m.sqrt(goal[0] ** 2 + goal[1] ** 2)
    normgoal = [goal[0], goal[1]]
    normgoal[0] /= distanceToGoal
    normgoal[1] /= distanceToGoal
    
    # now, make the dot product and grab the arc cos
    cosAngle = orientationVector[0] * normgoal[0] + orientationVector[1] * normgoal[1]
    if(abs(cosAngle) > 1.0):
        print("invalid angle:" + str(cosAngle))
        return 0, 0
    rads = m.acos(cosAngle)
    direction = orientationVector[1]*goal[0] - orientationVector[0]*goal[1]
    return rads * (180 / m.pi), direction

def turnTowardsGoal(vecToGoal):
    global orientationVector
    # remember: vecA * vecB = |vecA| * |vecB| * cos(phi)
    # we have to normalize our vecToGoal vector!
    distanceToGoal = m.sqrt(vecToGoal[0] ** 2 + vecToGoal[1] ** 2)
    normVecToGoal = [vecToGoal[0], vecToGoal[1]]
    normVecToGoal[0] /= distanceToGoal
    normVecToGoal[1] /= distanceToGoal
    
    # now, make the dot product and grab the arc cos
    cosAngle = orientationVector[0] * normVecToGoal[0] + orientationVector[1] * normVecToGoal[1]
    if(abs(cosAngle) > 1.0):
        print("invalid angle:" + str(cosAngle))
        return
    rads = m.acos(cosAngle)
    
    # rads to degrees
    deg = rads * 180/m.pi
    
    # decide whether to turn left or right
    direction = orientationVector[1]*vecToGoal[0] - orientationVector[0]*vecToGoal[1]
    print(direction)
    if(direction < 0.0):
        turnUntilPhysical(-2,deg)
    else:
        turnUntilPhysical(2,deg)
    
    setWheelVels(0,0,0)


def normalizeVector(vec):
    lengthOfVector = m.sqrt(vec[0] ** 2 + vec[1] ** 2)
    normVec = vec
    if(lengthOfVector > 0):
        normVec[0] /= lengthOfVector
        normVec[1] /= lengthOfVector
        return normVec
    else:
        return vec

# turning robot
def turnUntilPhysical(v, deg):
    global orientationVector
    global gdeg
    #V0[va] = vx + vy - ((L1 + L2)/2)*v_a/R
    angular_vel = ((300.46+471)/2)*v/50
    timeRequired = abs(deg/angular_vel) * 1.04 # python factor
    setWheelVels(0, 0, v)
    time.sleep(timeRequired)
    if(v >= 0.0):
        gdeg += deg
    else:
        gdeg -= deg
    gdeg = normalizeDegrees(gdeg)
    if(v <= 0):
        or_x = orientationVector[0] * m.cos(deg*m.pi/180.0) - orientationVector[1] * m.sin(deg*m.pi/180.0)
        or_y = orientationVector[1] * m.cos(deg*m.pi/180.0) + orientationVector[0] * m.sin(deg*m.pi/180.0)
    else:
        or_x = orientationVector[0] * m.cos(-deg*m.pi/180.0) - orientationVector[1] * m.sin(-deg*m.pi/180.0)
        or_y = orientationVector[1] * m.cos(-deg*m.pi/180.0) + orientationVector[0] * m.sin(-deg*m.pi/180.0)
    orientationVector[0] = or_x
    orientationVector[1] = or_y
    orientationVector = normalizeVector(orientationVector)

# main function for planner
def plannerGo():
    init()
    # print all planners that coppeliasim can use
    printAvailablePlanners()
    plannerInput = input("Enter number of path planner you wish to use: ")
    # get calculated path from coppeliasim
    path = obtainPathWithPlannerID(int(plannerInput))
    followPath(path, 1)
    print("planner finished")
    time.sleep(3)

def printAvailablePlanners():
    print("""
        Available Path Planners
        
        1 sim_ompl_algorithm_BiTRRT
        2 sim_ompl_algorithm_BITstar    (unavailable on osx)
        3 sim_ompl_algorithm_BKPIECE1
        4 sim_ompl_algorithm_CForest    (crashes)
        5 sim_ompl_algorithm_EST
        6 sim_ompl_algorithm_FMT        (unavailable on osx)
        7 sim_ompl_algorithm_KPIECE1
        8 sim_ompl_algorithm_LazyPRM
        9 sim_ompl_algorithm_LazyPRMstar
        10 sim_ompl_algorithm_LazyRRT
        11 sim_ompl_algorithm_LBKPIECE1
        12 sim_ompl_algorithm_LBTRRT
        13 sim_ompl_algorithm_PDST
        14 sim_ompl_algorithm_PRM
        15 sim_ompl_algorithm_PRMstar
        16 sim_ompl_algorithm_pRRT
        17 sim_ompl_algorithm_pSBL
        18 sim_ompl_algorithm_RRT
        19 sim_ompl_algorithm_RRTConnect
        20 sim_ompl_algorithm_RRTstar
        21 sim_ompl_algorithm_SBL
        22 sim_ompl_algorithm_SPARS
        23 sim_ompl_algorithm_SPARStwo
        24 sim_ompl_algorithm_STRIDE
        25 sim_ompl_algorithm_TRRT
        """)

def chunks(l, n):
    return [l[i:i + n] for i in range(0, len(l), n)]

def init():
    global pathPlanner
    global clientID
    res,pathPlanner=vrep.simxGetObjectHandle(clientID,'end',vrep.simx_opmode_oneshot_wait)

def obtainPathWithPlannerID(plannerID):
    global clientID
    emptyBuff = ""
    res,retInts,path,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'end',vrep.sim_scripttype_childscript,'findPath',[plannerID],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    print(path)
    return path


def veclen(vec):
    return m.sqrt(vec[0] ** 2 + vec[1] ** 2)

def deEulerize(euler):
    global orientationVector
    # alpha < 0.0 -> Y < 0.0 -> beta - 1.0
    # alpha > 0.0 -> Y > 1.0 -> 1.0 - beta
    # beta = X
    x = euler[1] / (m.pi / 2) * -1.0
    y = 0.0
    if(euler[0] > 0):
        y = 1.0 - abs(euler[1] / (m.pi / 2))
    else:
        y = abs(euler[1] / (m.pi / 2)) - 1.0
    #print(euler)
    orientationVector = normalizeVector([x, y])

def followPath(path, divider):
    global orientationVector
    global relativePos
    pth = chunks(path, 3)

    # set first path point as current relative position of the youbot
    relativePos[0] = path[0]
    relativePos[1] = path[1]
    print(relativePos)
    bla = getObjectOrientation("youBot")
    print(bla)
    print(m.sin(bla[0]), m.cos(bla[0]), m.sin(bla[1]), m.cos(bla[1]))
    
    # initial turn for test
    turnTowardsGoal([path[3] - relativePos[0], path[4] - relativePos[1]])
    
    # iterate through all points to drive to the goal
    index = 0
    path = list(pth[1::20])
    path.append(pth[len(pth)-1]) # force last point to be processed for better goal results
    for p in path[1:]:
        print("point reached, looking for next")
        print(orientationVector)
        print(relativePos)
        print(p)
        pnt = [p[0] - relativePos[0], p[1] - relativePos[1]]
        relativePos = getObjectPosition("youBot")[0:2]
        ori = getObjectOrientation("youBot")
        deEulerize(ori)
        degs, dire = getAngleToGoal(pnt)
        vlen = veclen([p[0] - relativePos[0], p[1] - relativePos[1]])
        if(vlen < 0.4):
            continue
        
        if(degs > 90):
            setWheelVels(0,0,0)
            turnTowardsGoal([p[0] - relativePos[0], p[1] - relativePos[1]])
    
        while (vlen > 0.3):
            relativePos = getObjectPosition("youBot")[0:2]
            ori = getObjectOrientation("youBot")
            deEulerize(ori)
            #g.orientationVector = [m.cos(ori[1]), m.cos(ori[0])]
            pnt = [p[0] - relativePos[0], p[1] - relativePos[1]]
            vlen = veclen(pnt)
            degs, dire = getAngleToGoal(pnt)
            setWheelVels(-8, 0, ((degs) / 180 * m.pi) * sgn(dire) * 10*(vlen+0.5)**2)
            if(degs > 45):
                break
    
        index+=1
    setWheelVels(0,0,0)

# get orientation from orientationVector
def oriPhi():
    global orientationVector
    # 0+, 1+ = 0 <= phi <= 90
    # 0-, 1+ = 90 < phi <= 180
    # 0-, 1- = 180 < phi <= 270
    # 0+, 1- = 270 < phi <= 360/0
    phi = 0
    if(orientationVector[1] >= 0):
        phi = m.pi
        if(orientationVector[0] >= 0):
            phi = m.asin(orientationVector[0])
        else:
            phi = m.pi / 2 + abs(m.asin(orientationVector[0]))
    else:
        phi = m.pi * 2
        if(orientationVector[0] >= 0):
            phi = m.pi * 2 - abs(m.asin(orientationVector[0]))
        else:
            phi = m.pi + abs(m.asin(orientationVector[0]))
    print(phi)
    return phi

# main function for completing the task
def sim():
    # determine the robots state
    global startPos
    global startOri
    ori = getObjectOrientation("youBot")
    deEulerize(ori)
    startPos = getObjectPosition("youBot")
    startOri = oriPhi()
    
    #start pathplanner 
    plannerGo()

# used to find the global position of youbot
def getObjectPosition(oname, fromname = None):
    global clientID
    res, t = vrep.simxGetObjectHandle(clientID, oname, vrep.simx_opmode_oneshot_wait)
    res, pos = vrep.simxGetObjectPosition(clientID, t, -1, vrep.simx_opmode_oneshot_wait)
    if (fromname):
        res, f = vrep.simxGetObjectHandle(clientID, fromname, vrep.simx_opmode_oneshot_wait)
        res, posf = vrep.simxGetObjectPosition(clientID, f, -1, vrep.simx_opmode_oneshot_wait)
        pos = [pos[0] - posf[0], pos[1] - posf[1], pos[2] - posf[2]]
    return pos

# used to find global orientation of youbot
def getObjectOrientation(oname, fromname = None):
    global clientID
    res, t = vrep.simxGetObjectHandle(clientID, oname, vrep.simx_opmode_oneshot_wait)
    res, ori = vrep.simxGetObjectOrientation(clientID, t, -1, vrep.simx_opmode_oneshot_wait)
    if (fromname):
        res, f = vrep.simxGetObjectHandle(clientID, fromname, vrep.simx_opmode_oneshot_wait)
        res, orif = vrep.simxGetObjectOrientation(clientID, f, -1, vrep.simx_opmode_oneshot_wait)
        ori = [ori[0] - orif[0], ori[1] - orif[1], ori[2] - orif[2]]
    return ori

# functions for setting wheel velicities
def calcWheelVels(fbv, lrv, rv):
    return [-fbv-lrv-rv, -fbv+lrv-rv, -fbv-lrv+rv, -fbv+lrv+rv]

def setWheelVels(fbv, lrv, rv):
    vels = calcWheelVels(fbv, lrv, rv)
    ret = 0
    for i in range(0, 4):
        ret = setWheelVel(i, vels[i])
    return ret

def setWheelVel(w, v):
    global clientID
    global wheels
    return vrep.simxSetJointTargetVelocity(clientID, wheels[w], v, vrep.simx_opmode_oneshot)

def start():
    global clientID
    global wheels
    # send any random signal to wake vrep up
    vrep.simxReadStringStream(clientID, "test", vrep.simx_opmode_streaming);

    print("waiting for response ...")

    # check simulation status
    res, state = vrep.simxGetInMessageInfo(clientID, 17);
    while (res == -1):
        res, state = vrep.simxGetInMessageInfo(clientID, 17);

    # if simulation already runs, stop it
    if (state == 5 or state == 7):
        print("simulation is already running. stopping simulation ...")
        vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
        time.sleep(5.5)

    # start simulation
    print("starting simulation ...")
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

    time.sleep(1)
    
    # retrieve wheel joint handles (front left, rear left, rear right, front right):
    res, wheels[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
    res, wheels[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
    res, wheels[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)
    res, wheels[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)

    # set wheel velocity to 0
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheels[i], 0, vrep.simx_opmode_oneshot)

def quit():
    global clientID
    global wheels   
    # set wheel velocity to 0
    for i in range(0, 3):
        vrep.simxSetJointTargetVelocity(clientID, wheels[i], 0, vrep.simx_opmode_oneshot)

    # stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

def main():
    global clientID
    # close all connections from vrep
    vrep.simxFinish(-1)

    print("connecting ...")

    # connect to vrep
    clientID = vrep.simxStart("127.0.0.1", 19997, True, True, 2000, 5)

    if clientID != -1:

        print("connected!")

        start()
        
        sim()

        quit()

        # disconnect from vrep
        vrep.simxFinish(clientID)

        print("disconnected!")

    else:

        print("failed to connect!")

if __name__ == "__main__": main()
