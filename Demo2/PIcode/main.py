from time import sleep
import time
import numpy as np
import cv2
from cv2 import aruco
import board
import busio
from smbus2 import SMBus
from multiprocessing import Process, Pipe
import json
        
#Data packet Format
# Angle,       
def serialHandler(serialConn):
    #Initializes Serial Connection
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout =1)
    prePosition = None
    
    while True:
        #Recieves move commands from main process
        messageObj = serialConn.recv()
        leftWheel = messageObj[0]
        rightWheel = messageObj[1]
        toSend = f"[{leftWheel},{rightWheel}]"
        if prePosition != messageObj:
            ser.write(toSend.encode(encoding='ascii'))
            prePosition = messageObj
        
        
if __name__ == '__main__':
   

    serialConn,mainSerialConn = Pipe()
    serial_process = Process(target = serialHandler, args=(serialConn,))
    serial_process.start()
    #For 4:3
    #CAMERA_HFOV = 57.15431399
    #For 16:9
    #CAMERA_HFOV = 61.37272481
    #Compensate lost degrees
    CAMERA_HFOV = 51.5
    ARUCO_DICT = cv2.aruco.DICT_6X6_250
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    #MARKER_LENGTH = 0.017145 # marker length in pixels
    # Sets up pipes for interprocess communication
    #lcdConn,mainLCDConn = Pipe()
    #Defines LCD iC2 Process
    #lcd_process = Process(target = lcdHandler, args=(lcdConn,))
    #Starts Processes
    #lcd_process.start()
    json_file_path = './calibration1.json'
    with open(json_file_path, 'r') as file:
        json_data = json.load(file)
    mtx = np.array(json_data['mtx'])
    dst = np.array(json_data['dist'])
    
    
    # initialize the camera
    camera = cv2.VideoCapture(0)
    #Arcuo setup
    w=640
    h=480
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dst, (w,h), 1, (w,h))
    #newW = roi[2]-roi[0]
    #CAMERA_HFOV = (CAMERA_HFOV/w)*newW
    
    #aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    #Let camera warmup
    sleep(1) 
    prevAngle = 370
    
    prevTime = time.time()
    
    
    
    #Main control loop
    #Takes image frame from video capture and processes it
    while camera.isOpened():
#########################################################################################
        #Determine if the arrow is pointing left(green) or right(red)
        
        
        
        ############################################################################
        angle = None
        feet_dist = None
        turnTo = 2
        ret, image1 = camera.read()
        image = cv2.cvtColor(image1,cv2.COLOR_BGR2GRAY)
        #h, w = image.shape[:2]
        
        image = cv2.undistort(image, mtx, dst, None, newcameramtx)
        smallImage = None
        #x, y, w, h = roi
        #image = image[y:y+h, x:x+w]
        corners,ids,rejected = detector.detectMarkers(image)
        #h,w = image.shape[:2]
        #If Marker is detected, perform advanced image processing
        if ids is not None and not np.where(ids==0)[0].size ==0 :
            ids = ids.flatten()
            #try:
            index = np.where(ids == 0)[0][0]
            centerX = ((corners[index][0][0][0] + corners[index][0][1][0] + corners[index][0][2][0] + corners[index][0][3][0])*.25)
            #centerY = ((corners[index][0][0][1] + corners[index][0][1][1] + corners[index][0][2][1] + corners[index][0][3][1])*.25)
            cX = mtx[0][2]
            #cY = mtx[1][2]
            
            #Determine distance
            ############################################################################
            #print(corners[index][0][1][0]-corners[index][0][0][0])           
            #print(corners[index][0][2][0]-corners[index][0][3][0])
            #1-0 tiles is 109 - 110,
            #2-0 tiles is 54-55                   - 1/2
            #3-0 tiles is 36 -37 3-1 is           -  2/3
            #4-0 tiles is 27-28        - 3/4
            #5-0 tiles is 21 22 5-1 is 22, 5-2 is 21 - 4/5
            #distance in feet:
            width = corners[index][0][1][0]-corners[index][0][0][0]
             #print(width)
            feet_dist = 110 / (width)
            #print(feet_dist)
            ############################################################################
            if (feet_dist<1.7):
                height = corners[index][0][2][1] - corners[index][0][1][1]
                #For width of mask:
                #Lower bound
                lowerXBound = int(corners[index][0][0][0] - round(1.5*width))
                lowerXBound = lowerXBound*(lowerXBound > 0)
                upperXBound = int(corners[index][0][1][0] + round(1.5*width))
                upperXBound = upperXBound*(upperXBound < 640) + 640*( upperXBound >= 640)
                lowerYBound= int(corners[index][0][1][1] - round(0.25*width))
                lowerYBound = lowerYBound*(lowerYBound > 0)
                upperYBound =  int(corners[index][0][2][1] + round(0.25*width))
                upperYBound = upperYBound*(upperYBound < 480) + 480*( upperYBound >= 480)
                
                print(f"lowerX {lowerXBound}")
                print(f"upperX {upperXBound}")
                print(f"lowerY {lowerYBound}")
                print(f"upperY {upperYBound}")
                
                #Height remains the same
                smallImage = image1[lowerYBound:upperYBound,lowerXBound:upperXBound]

                
                arrowDetectHSV = cv2.cvtColor(smallImage, cv2.COLOR_BGR2HSV)
                upperGreen = np.array([80, 180, 180])
                lowerGreen = np.array([45,80,80])
                upperRed = np.array([15, 255, 255])
                lowerRed = np.array([0,160,160])
                upperRed2 = np.array([179, 255, 255])
                lowerRed2 = np.array([170, 160, 160])
                
                #Display red or green (used to tune allowed array values)
                maskGreen = cv2.inRange(arrowDetectHSV, lowerGreen, upperGreen)
                maskRed1 = cv2.inRange(arrowDetectHSV, lowerRed, upperRed)
                maskRed2 = cv2.inRange(arrowDetectHSV, lowerRed2, upperRed2)
                maskRed=cv2.bitwise_or(maskRed1,maskRed2)
                
                #resultRed = 
                #resultGreen = 
                #0 is Red, 1 is Green
                resultMtxArray= [np.sum(cv2.bitwise_and(smallImage,smallImage, mask=maskRed)), np.sum(cv2.bitwise_and(smallImage,smallImage, mask=maskGreen))]
                #contours_red,_ = cv2.findContours(maskRed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                #contours_green,_ = cv2.findContours(maskGreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                #cv2.drawContours(resultGreen,contours_green,-1,(255,0,0),-1)
                
                
                whichOne = ( resultMtxArray[0] < resultMtxArray[1])
                print(whichOne)
                if (resultMtxArray[whichOne] > 100000):
                    if(whichOne):
                        turnTo = 0
                        #print("There is a green arrow showing")
                    else:
                        turnTo = 1
                        #print("There is a red arrow showing")
                        
            angle =  ((CAMERA_HFOV) * (cX-centerX))/w
            
            # Fix issue with negative angles being too large, start at like -0.5 angle
            if angle > 0.5:
                angle = angle + 0.7
            
            
            #if prevAngle == round(angle):
        #if smallImage is not None:
        #    cv2.imshow("wow",smallImage)
        
        
        mainSerialConn.send([angle,feet_dist,turnTo])
        #cv2.imshow("overlay",overlay)
        #cv2.imshow("undistort", image)
        #Exits loop after pressign 'q'
        if cv2.waitKey(10) == ord('q'):
            break
        
    #Releases camera and kills all windows
    camera.release()
    cv2.destroyAllWindows()
    
    
    #Closes pipes and joins sub-processes back to main process
    #mainLCDConn.close()
    #lcd_process.join() 
    #lcd.clear()
    
