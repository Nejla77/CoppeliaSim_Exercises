import vrep
import time
import os
import math as m
import numpy as np
import copy
from PIL import Image
import cv2
import array

# robot state information
orientationVector = [0.0, -1.0]
startPos = [0.0, 0.0]
startOri = 0.0
gdeg = 0.0
relativePos = [0.0, 0.0]
wheels = [-1] * 4

clientID = -1
imaging=None

#sensor data
sensor1 = -1
sensor2 = -1

sDataWidth1 = -1
sDataHeight1 = -1
sDataWidth2 = -1
sDataHeight2 = -1
sData1 = None
sData2 = None


def grabObjectAndPutOnBot(armJointsHandle, count):
    global clientID
    #define grasp joints
    # ...
    graspJoints = [(-20+(9-count)*5)*m.pi/180, 10*m.pi/180, 160*m.pi/180, -41*m.pi/180, 0*m.pi/180];
    #move arm
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 5):
        res = vrep.simxSetJointTargetPosition(clientID, armJointsHandle[i], graspJoints[i],vrep.simx_opmode_oneshot);
    vrep.simxPauseCommunication(clientID, False)
    time.sleep(2)

    graspJoints = [-90*m.pi/180, 10*m.pi/180, 140*m.pi/180, -41*m.pi/180, 0*m.pi/180];
    
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 5):
        res = vrep.simxSetJointTargetPosition(clientID, armJointsHandle[i], graspJoints[i],vrep.simx_opmode_oneshot);
    vrep.simxPauseCommunication(clientID, False)

    time.sleep(3)
    
    graspJoints = [-90*m.pi/180, 70*m.pi/180, 87*m.pi/180, -41*m.pi/180, 0*m.pi/180];
    
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 5):
        res = vrep.simxSetJointTargetPosition(clientID, armJointsHandle[i], graspJoints[i],vrep.simx_opmode_oneshot);
    vrep.simxPauseCommunication(clientID, False)

    time.sleep(3)
    
    res = vrep.simxSetIntegerSignal(clientID, 'gripper_open',0,vrep.simx_opmode_oneshot_wait)
    time.sleep(2)
    
    graspJoints = [-90*m.pi/180, 19.6*m.pi/180, 113*m.pi/180, -41*m.pi/180, 0*m.pi/180];
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 5):
        res = vrep.simxSetJointTargetPosition(clientID, armJointsHandle[i], graspJoints[i],vrep.simx_opmode_oneshot);
    vrep.simxPauseCommunication(clientID, False)
    time.sleep(2)

    graspJoints = [(-20+(8-count)*5)*m.pi/180, 19.6*m.pi/180, 113*m.pi/180, -41*m.pi/180, 0*m.pi/180];
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 5):
        res = vrep.simxSetJointTargetPosition(clientID, armJointsHandle[i], graspJoints[i],vrep.simx_opmode_oneshot);
    vrep.simxPauseCommunication(clientID, False)
    time.sleep(2)
    
    graspJoints = [(-20+(8-count)*5)*m.pi/180, 30*m.pi/180, 113*m.pi/180, -41*m.pi/180, 0*m.pi/180];
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 5):
        res = vrep.simxSetJointTargetPosition(clientID, armJointsHandle[i], graspJoints[i],vrep.simx_opmode_oneshot);
    vrep.simxPauseCommunication(clientID, False)
    time.sleep(1)

    res = vrep.simxSetIntegerSignal(clientID, 'gripper_open',1,vrep.simx_opmode_oneshot_wait)
    time.sleep(2)


def updateStats():
    global state
    global relativePos
    state = state
    os.system('clear')
    stext = ""
    if(state == 1):
        stext = "On Line, Turn towards goal"
    elif(state == 2):
        stext = "Approach goal until reached or obstacle"
    elif(state == 3):
        stext = "Obstacle found, choose avoidance path"
    elif(state == 4):
        stext = "Obstacle avoidance, LEFT side"
    elif(state == 5):
        stext = "Obstacle avoidance, RIGHT side"
    print("POS: " + str(relativePos))


# state machine for obstacle avoidance, for object discovery and grabbing
def stateMachineIMG(detector2, armJointsHandle):
    # for better understanding (and probably easier development)
    # reimplemented everything as a state machine
    #
    # STATES:
    # 0 = finished              -> nil
    # 2 = straight until blob
    #     visible / obstacle    -> 3, 6, 0
    #     or all grabbed
    # 3 = obstacle avoidance
    #     decision state (l)    -> 4
    # 4 = obstacle avoidance
    #     LEFT, until blob
    #     visible or            -> 6
    #     obstacle surrounded
    # 6 = grab object           -> 2
    global state
    global relativePos
    global orientationVector
    
    count = 0
    state = 2
    while(state != 0):
        if(state == 2):
            updateStats()
            print("forward, forward, forward!!")
            ret, blobs = forwardUntilObstacleOrBlob(-3.0, 0.5, detector2)
            if(ret == 100):
                print("there's a blob!")
                print("approaching blob, this may take some time")
                while(abs(465 - blobs[0][0]) >= 7 or abs(470 - blobs[0][1]) >= 7):
                    blobs = approach(blobs, detector2)
                setWheelVels(0,0,0)
                grabObjectAndPutOnBot(armJointsHandle, count)
                count += 1
                if(count < 8):
                    state = 2
                else:
                    state = 0
            else:
                state = 3
                lastLinePoint = [relativePos[0], relativePos[1]]
        elif(state == 3):
            selectedPoint = None
            selectedInvChoice = None
            thresh = 0.5
            
            readSensorData()
            rightAvg = getAverageOfSensorValues(getSensorValuesRightRange(0, 342-80))
            leftAvg = getAverageOfSensorValues(getSensorValuesLeftRange(80, 342))
            print(rightAvg)
            print(leftAvg)
            print("chose right")
            turnUntilCenterSensorsClear(2.0, 1.5)
            goStraightTimed(-5, 30)
            lockLeft = True
            starter = True
            orientationVector = [0.0, -1.0]
            relativePos = [0.0, 0.0]
            state = 4
        elif(state == 4):
            updateStats()
            if(starter == False and abs(relativePos[0]) + abs(relativePos[1]) < 0.2):
                print("We've been here, let's skip that one")
                state = 2
            else:
                if(abs(relativePos[0]) + abs(relativePos[1]) > 1.5):
                    starter = False
                res, blobs = forwardUntilLeftIsClearOrObstacleBlob(-3.0, 0.4, 1.0, 0.5, 0.6, detector2)
                print("mode: " + str(res))
                if(res == 0):
                    turnUntilCenterSensorsClear(1, 2.0)
                    goStraightTimed(-3,50)
                elif(res == 1): # near correction
                    turnUntilPhysical(1, 6)
                    goStraightTimed(-3,50)
                elif(res == 2): # far correction
                    turnUntilPhysical(-1, 6)
                    goStraightTimed(-3,60)
                elif(res == 3): # sharp left
                    turnUntilPhysical(-1, 10)
                    goStraightTimed(-3,50)
                    lockLeft = False
                elif(res == 5):
                    print("there's a blob")
                    print("approaching blob, this may take some time")
                    while(abs(465 - blobs[0][0]) >= 7 or abs(470 - blobs[0][1]) >= 7):
                        blobs = approach(blobs, detector2)
                    setWheelVels(0,0,0)
                    grabObjectAndPutOnBot(armJointsHandle, count)
                    count += 1
                    if(count < 8):
                        state = 2
                    else:
                        state = 0
                elif(res == 6):
                    turnUntilPhysical(-1, 50)
        else:
            break
    if(state == 0):
        print("Should have grabbed all objects!")
        print("================================")
        setWheelVels(0,0,0)
        input("press enter to quit")

# functions for reading sensor data
def getMinimumOfSensorValues(values):
    return getMinimumOfValues(values)

def getMinimumOfValues(values):
    mini = 100
    for i in range(0, len(values)):
        if(values[i] != 0.0):
            mini = min(mini, values[i])
    return mini

def getMinimumOfAllSensors():
    global sData1
    global sData2
    res = 5.0
    index = 0
    for i in range(0,342):
        val = getSensorDistanceAtIndex(sData1, 341-i)
        if(val < res):
            res = val
            index=341-i+342
        val = getSensorDistanceAtIndex(sData2, i)
        if(val < res):
            res = val
            index=i
    return [res,i]

def getMinimumOfCenterSensors(width):
    global sData1
    global sData2
    res = 5.0
    index = 0
    width = width//2
    for i in range(0,width):
        val = getSensorDistanceAtIndex(sData1, 341-i)
        if(val < res):
            res = val
            index=341-i
        val = getSensorDistanceAtIndex(sData2, i)
        if(val < res):
            res = val
            index=i
        #res = min(res, getSensorDistanceAtIndex(sData1, 341-i))
        #res = min(res, getSensorDistanceAtIndex(sData2, i))
    return [res,i]


def getAverageOfSensorValues(values):
    sum = 0
    for i in range(0, len(values)):
        sum += min(values[i], 1.5)
    sum /= len(values)
    return sum

def getAverageSensorCenterValues(width):
    global sData1
    global sData2
    sum = 0
    width = width//2
    for i in range(0,width):
        sum += getSensorDistanceAtIndex(sData1, 341-i)
        sum += getSensorDistanceAtIndex(sData2, i)
    return sum/((i+1)*2)

def getSensorValuesLeft(width):
    global sData2
    # 200 - 250
    cnt = 0
    arr = [0.0]*width
    for i in range((225-width//2), (225+width//2)):
        arr[cnt] = getSensorDistanceAtIndex(sData2, i);
        cnt += 1
    return arr

def getSensorValuesRight(width):
    global sData1
    # 200 - 250
    cnt = 0
    arr = [0.0]*width
    for i in range(117-width//2, 117+width//2):
        arr[cnt] = getSensorDistanceAtIndex(sData1, i);
        cnt += 1
    return arr

def getSensorValuesLeftRange(f, t):
    global sData2
    # 200 - 250
    readSensorData()
    cnt = 0
    arr = [0.0]*(t - f)
    for i in range(f, t):
        arr[cnt] = getSensorDistanceAtIndex(sData2, i);
        cnt += 1
    return arr

def getSensorValuesRightRange(f, t):
    global sData1
    # 200 - 250
    readSensorData()
    cnt = 0
    arr = [0.0]*(t - f)
    for i in range(f, t):
        arr[cnt] = getSensorDistanceAtIndex(sData1, i);
        cnt += 1
    return arr

def getSensorDistanceAtIndex(data, index):
    return data[2+(4*index)+3]


def readSensorData():
    global clientID
    global sensor1
    global sensor2
    global sDataWidth1
    global sDataWidth2
    global sDataHeight1
    global sDataHeight2
    global sData1
    global sData2
    
    res, aux1, auxD1 = vrep.simxReadVisionSensor(clientID, sensor1, vrep.simx_opmode_buffer)
    res, aux2, auxD2 = vrep.simxReadVisionSensor(clientID, sensor2, vrep.simx_opmode_buffer)

    # youbot has 2 sensors on board
    # the first one does measurements on the right side, the second on the left side
    # the center sensor indices will be (from right to left):
    #
    #       sData1          sData2
    # .. 339  340  341 |   0   1   2  ..
    # <R ------------- C ------------ L>
    #
    # the actual data sits at auxD[1]
    # the sensor data comes as 1370 values, where the first two represent the matrix dimensions
    # the following values are 4-tuples each. (x, y, z, min(5, distanceToSensor))
    # this means, if an object's distance is more than 5m, the sensor will return 5.0
    sDataWidth1 = auxD1[1][0]
    sDataHeight1 = auxD1[1][1]

    sDataWidth2 = auxD2[1][0]
    sDataHeight2 = auxD2[1][1]

    sData1 = auxD1[1]
    sData2 = auxD2[1]


    #res, aux, auxD = vrep.simxReadVisionSensor(clientID, sensor2, vrep.simx_opmode_streaming)
    #res, aux, auxD = vrep.simxReadVisionSensor(clientID, sensor2, vrep.simx_opmode_buffer)


# functions for computer vision
def initimg(clientID):
    res = vrep.simxSetFloatSignal(clientID, 'rgbd_sensor_scan_angle', 90 * (m.pi/180), vrep.simx_opmode_oneshot_wait)
    # turn on camera
    res = vrep.simxSetIntegerSignal(clientID, 'handle_rgb_sensor', 2, vrep.simx_opmode_oneshot_wait);
    # get camera object-handle
    res, youBotCam = vrep.simxGetObjectHandle(clientID, 'rgbSensor', vrep.simx_opmode_oneshot_wait)
    # get first image
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_streaming)
    cv2.namedWindow("display")
    time.sleep(0.1)
    return youBotCam

def updateImage(img):
    cv2.imshow("display", img)
    cv2.waitKey(25)

def getImage(clientID, youBotCam):
    start = time.time()
    err, res, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_buffer)
    if err == vrep.simx_return_ok:
        data1 = array.array('b', image)
        image_buffer = Image.frombytes("RGB", (res[0],res[1]), bytes(data1), "raw", "RGB", 0, 1)
        img = np.asarray(image_buffer)
        rimg = cv2.cvtColor(cv2.flip(img, 0), cv2.COLOR_BGR2RGB)
        return rimg
    return None

def extractBlobsOfColor(imageBGR, detector, color, KeypointColor):
    if(color == "y"):
        img = yellowBlobExtract(imageBGR)
    if(color == "o"):
        img = orangeBlobExtract(imageBGR)
    if(color == "r"):
        img = redBlobExtract(imageBGR)
    if(color == "g"):
        img = greenBlobExtract(imageBGR)
    if(color == "b"):
        img = blueBlobExtract(imageBGR)
    keypoints = detector.detect(img)
    for i in range(0, len(keypoints)):
        KeypointColor[keypoints[i].pt] = color
    return img

def getBottomPointFromKeypoint(kpoint, acc):
    x = int(kpoint[0])
    y = int(kpoint[1])
    while (y < 511 and x < 511):
        if(acc[y,x] > 128):
            y += 1
        else: break
    return (x,y)

def extractBlobs(imageBGR, detector):
    # first, extract all blobs and save their keypoints and colors
    KeypointColor = {}
    y = extractBlobsOfColor(imageBGR, detector, "y", KeypointColor)
    o = extractBlobsOfColor(imageBGR, detector, "o", KeypointColor)
    r = extractBlobsOfColor(imageBGR, detector, "r", KeypointColor)
    g = extractBlobsOfColor(imageBGR, detector, "g", KeypointColor)
    b = extractBlobsOfColor(imageBGR, detector, "b", KeypointColor)
    
    # then, merge images together in order to detect two-colored blobs
    acc = cv2.add(y, o)
    acc = cv2.add(acc, r)
    acc = cv2.add(acc, g)
    acc = cv2.add(acc, b)
    
    # detect blobs again and save keypoints
    keypoints = detector.detect(acc)
    
    # now, determine which blobs got merged
    points = []
    singles = []
    merged = []
    lost = []
    for i in range(0, len(keypoints)):
        if(keypoints[i].pt not in KeypointColor):
            # we have a merged blob, save it
            merged.append(keypoints[i].pt)
        else:
            singles.append(keypoints[i].pt)
        points.append(keypoints[i].pt)
    keys = list(KeypointColor.keys())
    for i in range(0, len(keys)):
        if(keys[i] not in points):
            # lost blob, got merged
            lost.append(keys[i])

    KeypointColorFinal = {}
    font = cv2.FONT_HERSHEY_SIMPLEX

    # final step is to determine the merged blob's colors
    for i in range(0, len(merged)):
        # get two nearest lost blobs.. they were merged
        offsets = []
        for j in range(0, len(lost)):
            offsets.append([abs(lost[j][0] - merged[i][0]) + abs(lost[j][1] - merged[i][1]), j])
        index1 = min((e for e in offsets), key = itemgetter(0))[1]
        del offsets[index1]
        if(len(offsets) < 1):
            continue
        index2 = min((e for e in offsets), key = itemgetter(0))[1]
        blob1 = lost[index1]
        blob2 = lost[index2]
        bottom = getBottomPointFromKeypoint(merged[i], acc)
        KeypointColorFinal[bottom] = KeypointColor[blob1] + ", " + KeypointColor[blob2]
        cv2.putText(imageBGR, KeypointColorFinal[bottom], bottom, font, 0.5,(0,0,0),1,cv2.LINE_AA)
        cv2.drawMarker(imageBGR, bottom, (128, 128, 128))

    # add single blobs to dict
    for i in range(0, len(singles)):
        bottom = getBottomPointFromKeypoint(singles[i], acc)
        KeypointColorFinal[bottom] = KeypointColor[singles[i]]
        cv2.putText(imageBGR, KeypointColorFinal[bottom], bottom, font, 0.5,(0,0,0),1,cv2.LINE_AA)
        cv2.drawMarker(imageBGR, bottom, (128, 128, 128))
    return imageBGR, KeypointColorFinal

def yellowBlobExtract(imageBGR):
    #RGB STYLE
    lower = np.array([0, 210, 210])
    upper = np.array([140, 255, 255])
    return cv2.inRange(imageBGR, lower, upper)

def orangeBlobExtract(imageBGR):
    #RGB STYLE
    lower = np.array([0, 140, 210])
    upper = np.array([110, 200, 255])
    return cv2.inRange(imageBGR, lower, upper)

def redBlobExtract(imageBGR):
    #RGB STYLE
    lower = np.array([0, 0, 170])
    upper = np.array([80, 80, 255])
    return cv2.inRange(imageBGR, lower, upper)

def blueBlobExtract(imageBGR):
    #RGB STYLE
    lower = np.array([170, 0, 0])
    upper = np.array([255, 150, 80])
    return cv2.inRange(imageBGR, lower, upper)

# this returns a binarized COLOR_U8 GRAY image
def greenBlobExtract(imageBGR):
    #RGB STYLE
    lower = np.array([0, 170, 0])
    upper = np.array([80, 255, 80])
    return cv2.inRange(imageBGR, lower, upper)



def goStraightTimed(vel, dst):
    global relativePos
    global orientationVector
    # vel is the angular velocity !! in rad/s!!
    # dst is the desired distance to be driven !! in mm !!
    # one rad = 50mm
    timeRequired = (dst/50.0) / abs(vel)
    setWheelVels(vel, 0, 0)
    time.sleep(timeRequired)
    relativePos[0] += (orientationVector[0] * (dst/1000.0))
    relativePos[1] += (orientationVector[1] * (dst/1000.0))



def forwardUntilObstacleOrBlob(vel, thresh, detector2):
    global clientID
    global imaging
    objDistance = 5.0
    goalDistance = 100
    while objDistance >= thresh:
        goStraightTimed(vel, 20)
        readSensorData()
        val = getMinimumOfAllSensors()
        objDistance = getMinimumOfCenterSensors(150)[0]
        objAverage = getAverageSensorCenterValues(150)
        ratio = objAverage / objDistance
        
        test = getImage(clientID, imaging)
        outImage, KeypointColor = extractBlobs(test, detector2)
        blobs = list(KeypointColor.keys())
        if(len(blobs) > 0):
            if(blobs[0][1] > 300):
                return 100, blobs

    return 0, None

def forwardUntilLeftIsClearOrObstacleBlob(vel, forw_thresh, leftClearThresh, leftNearThresh, leftFarThresh, detector2):
    global clientID
    global imaging
    leftValues = getSensorValuesLeft(50)
    leftIsClear = False
    objDistance = 5.0
    leftAverage = 0.51
    leftMinimum = 0.51
    onLine = False
    while(not leftIsClear and objDistance >= forw_thresh and leftMinimum > leftNearThresh and leftMinimum < leftFarThresh and not onLine):
        readSensorData()
        leftValues = getSensorValuesLeftRange(5, 341)
        leftAverage = getAverageOfSensorValues(leftValues)
        leftMinimum = getMinimumOfSensorValues(leftValues)
        print("Minimum distance: " + str(getMinimumOfValues(getSensorValuesLeftRange(0, 341))))
        objDistance = getMinimumOfCenterSensors(150)[0]
        goStraightTimed(vel, 20)
        test = getImage(clientID, imaging)
        outImage, KeypointColor = extractBlobs(test, detector2)
        blobs = list(KeypointColor.keys())
        if(len(blobs) > 0):
            if(blobs[0][1] > 300):
                return 5, blobs
    print(leftMinimum)
    setWheelVels(0,0,0)

    if(objDistance < forw_thresh):
        return 0, None
    if(leftIsClear):
        return 6, None
    leftmostdist = getAverageOfSensorValues(getSensorValuesLeftRange(290,320)) # for sharp turns

    if(leftMinimum < leftNearThresh): # near correction
        return 1, None
    if(leftmostdist > leftFarThresh + 0.1): # check for sharp turn (+0.2 for bigger tolerance)
        return 3, None
    if(leftMinimum > leftFarThresh): # far correction
        if(getMinimumOfValues(getSensorValuesLeftRange(0, 341)) <= leftNearThresh):
            while(getMinimumOfValues(getSensorValuesLeftRange(0, 341)) <= leftNearThresh):
                goStraightTimed(vel, 20)
            return 3, None
        else:
            return 2, None
    return 4, None


def turnUntilCenterSensorsClear(v, thresh):
    
    centerVal = 0.0
    while (centerVal < thresh):
        turnUntilPhysical(v, 2)
        readSensorData()
        centerVal = getMinimumOfCenterSensors(100)[0]

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

# used to get image, setup detectors and show the image
def imagingSetup():
    global clientID
    global imaging
    imaging = initimg(clientID)
    if(imaging is None):
        print("could not get cam handle")
        return

    test = getImage(clientID, imaging)
    updateImage(test)
    

    # params are parameters for blob detection
    # this determines which blobs will be recognized
    params = cv2.SimpleBlobDetector_Params()
    params.minDistBetweenBlobs = 1
    params.filterByColor = True
    params.blobColor = 255
    params.filterByArea = True
    params.minArea = 1
    params.maxArea = 50000
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
    params.maxInertiaRatio = 1
    
    # detectors are used to find the blobs in an image
    detector = cv2.SimpleBlobDetector_create(params)
    
    keypointColor={}
    image=extractBlobsOfColor(test, detector, "r", keypointColor)
    detector2 = cv2.SimpleBlobDetector_create(params)
    image2=extractBlobsOfColor(test, detector2, "b", keypointColor)

    return detector, detector2

# setup sensors
def startRangeSensors():
    global clientID
    global sensor1
    global sensor2
    res = vrep.simxSetIntegerSignal(clientID, 'handle_xy_sensor', 2, vrep.simx_opmode_oneshot);
    vrep.simxSetIntegerSignal(clientID, 'displaylasers', 1, vrep.simx_opmode_oneshot);
    res, sensor1 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor1', vrep.simx_opmode_oneshot_wait)
    res, sensor2 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor2', vrep.simx_opmode_oneshot_wait)
    res, aux, auxD = vrep.simxReadVisionSensor(clientID, sensor1, vrep.simx_opmode_streaming)
    res, aux, auxD = vrep.simxReadVisionSensor(clientID, sensor2, vrep.simx_opmode_streaming)

# main function for completing the task
def sim():
    # determine the robots state
    global startPos
    global startOri
    startRangeSensors()
    ori = getObjectOrientation("youBot")
    startPos = getObjectPosition("youBot")
    startOri = oriPhi()
    detector, detector2 = imagingSetup()
    
    armJointsHandle=np.empty(5, dtype=int); armJointsHandle.fill(-1)
    for i in range(0, 5):
        res,armJointsHandle[i]=vrep.simxGetObjectHandle(clientID,'youBotArmJoint%d'%i,vrep.simx_opmode_oneshot_wait)

    stateMachineIMG(detector2, armJointsHandle)

# approach detected blob
def approach(blobs, detector2):
    global clientID
    global imaging
    xspeed = -2
    yspeed = 0

    while(True):
        setWheelVels(xspeed/4, yspeed/4, 0)
        
        if(blobs[0][0] < 453):
            xspeed = -2
        
        if(blobs[0][1] < 460):
            yspeed = -1
        
        if(blobs[0][1] < 350):
            yspeed = -3
        
        if(blobs[0][1] < 280):
            yspeed = -5
        
        if(blobs[0][1] > 475):
            yspeed = 1

        if(blobs[0][0] - 465 > 7):
            xspeed = 1

        if(465 - blobs[0][0] > 7):
            xspeed = -1

        if(blobs[0][0] < 50):
            xspeed = -5

        if(blobs[0][0] < 200 and blobs[0][1] > 350):
            xspeed = -3

        if(abs(465 - blobs[0][0]) < 7 and abs(470 - blobs[0][1]) < 7):
            print("reached.")
            print(blobs)
            return blobs
        
        oldblob = blobs[0]
        test = getImage(clientID, imaging)
        outImage, KeypointColor = extractBlobs(test, detector2)
        blobs = list(KeypointColor.keys())

        while(len(blobs) < 1 or blobs is None):
            if(oldblob[0] < 30):
                xspeed = -2
            elif(oldblob[0] > 480):
                xspeed = 2
            setWheelVels(xspeed, -1, 0)
            test = getImage(clientID, imaging)
            outImage, KeypointColor = extractBlobs(test, detector2)
            blobs = list(KeypointColor.keys())

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
