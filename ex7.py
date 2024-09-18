import vrep
import time
import os
import math as m
import numpy as np
import cv2
import array
from PIL import Image

# robot state information
orientationVector = [0.0, -1.0]
startPos = [0.0, 0.0]
startOri = 0.0
clientID = -1
wheels = [-1] * 4

imaging=None

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

# display new image
def updateImage(img):
    cv2.imshow("display", img)
    cv2.waitKey(25)

#get image from youbot
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

# gets blobs of certain color from imageBGR
def extractBlobsOfColor(imageBGR, detector, color, KeypointColor):
    if(color == "r"):
        img = redBlobExtract(imageBGR)
    if(color == "b"):
        img = blueBlobExtract(imageBGR)
    
    # can be used for drawing a circle/square around blobs
    keypoints = detector.detect(img)
    for i in range(0, len(keypoints)):
        KeypointColor[keypoints[i].pt] = color
    return img

# functions for extracting red and blue blobs
# inRange functions scans each pixel and determines which are between upper and lower RGB values
# only blobs of certain area will be recognized
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

    cv2.imshow("red", image)
    cv2.imshow("blue", image2)
    cv2.waitKey(0)
    return detector, detector2

# main function for completing the task
def sim():
    # determine the robots state (optional since youbot doesn't move)
    global startPos
    global startOri
    global clientID
    ori = getObjectOrientation("youBot")
    startPos = getObjectPosition("youBot")
    startOri = oriPhi()
    detector, detector2 = imagingSetup()


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
#startRangeSensors()
    
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
