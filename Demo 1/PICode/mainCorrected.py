from time import sleep
import time
import numpy as np
import cv2
from cv2 import aruco
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import busio
from smbus2 import SMBus
from multiprocessing import Process, Pipe
import json

#Calibrate camera
def angleFinder():
    
    
    #SQUARES_VERTICALLY = 7
    #SQUARES_HORIZONTALLY = 5
    #SQUARE_LENGTH = 0.034290# square side length in pixels
    #MARKER_LENGTH = 0.017145 # marker length in meters
    MARKER_LENGTH = 0.05
    #read json path data
    json_file_path = '/home/seedlab/calibration1.json'
    with open(json_file_path, 'r') as file:
        json_data = json.load(file)
    mtx = np.array(json_data['mtx'])
    dst = np.array(json_data['dist'])
    
    ##Image Specific
    #calculate distance between camera sensor and object
    image_path = '/home/seedlab/Pictures/calibrate/image8.png'
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    h, w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dst, (h,w), 1, (h,w))
    image = cv2.undistort(image, mtx, dst, None, newcameramtx)
    all_charuco_ids = []
    all_charuco_corners = []
    
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    #board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    corners,ids,rejected = detector.detectMarkers(image)
    #If ids are found run
    if ids is not None:
        ids = ids.flatten()
        index = np.where(ids == 0)[0][0]
        centerX = ((corners[index][0][0][0] + corners[index][0][1][0] + corners[index][0][2][0] + corners[index][0][3][0])*.25)
        centerY = ((corners[index][0][0][1] + corners[index][0][1][1] + corners[index][0][2][1] + corners[index][0][3][1])*.25)
        cX = mtx[0][2]
        cY = mtx[1][2]
        angle = np.arctan2((cX-centerX),(cY-centerY)) * 180 / np.pi
        print(angle)
        centXR = round(centerX)
        centYR = round(centerY)
        camXR = round(cX)
        camYR = round(cY)
        image = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)
        image = cv2.line(image,pt1=(centXR,centYR), pt2=(centXR,centYR), color = (0,0,255), thickness=10)
        image = cv2.line(image,pt1=(camXR,camYR), pt2=(camXR,camYR), color = (0,255,0), thickness=10)
        
        cv2.imshow("undistort", image)
        #topXlength = round((corners[index][0][1][0] - corners[index][0][0][0]))
        #bottomXlength = round((corners[index][0][2][0] - corners[index][0][3][0]))
        #avgXlength = round( (topXlength + bottomXlength)/2.0)
        #topLeftBottomRight = round((corners[index][0][2][0] - corners[index][0][0][0]))
        
        #Find size of square length of marker
        #tempTLBR = corners[index][0][2] - corners[index][0][0]
        #sum_sqTLBR = np.dot(tempTLBR.T,tempTLBR)
        #sqrtTLBR = np.sqrt(sum_sqTLBR)
        
        #tempTRBL = corners[index][0][3] - corners[index][0][1]
        #sum_sqTRBL = np.dot(tempTRBL.T,tempTRBL)
        #sqrtTRBL = np.sqrt(sum_sqTRBL)
        
        #avgLength = (sqrtTRBL + sqrtTLBR)/ (2*np.sqrt(2))
        
        #corners1 = corners[index]
        #rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners1 , MARKER_LENGTH, np.array(mtx), np.array(dst) )
        #Zx, Zy, Zz = tvec[0], tvec[1], tvec[2]
        #Zz = tvec[0][0][2]
        #Zx = tvec[0][0][0]
        #Zy = tvec[0][0][1]
        #fx,fy = mtx[0][0],mtx[1][1]
        #print(f'Zz = {Zz}\nfx = {fx}')
        #Find Z length
        #print( (avgLength*Zz)/fx)
        #print(avgLength)
        
        

#    marker_corners, marker_ids, rejectedCandidates, = detector.detectMarkers(image)
#    if marker_ids is not None and len(marker_ids) > 0:
#        ret, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids,image,board)
#        if charucoCorners is not None and charucoIds is not None and len(charucoCorners) >3:
#            all_charuco_corners.append(charucoCorners)
#            all_charuco_ids.append(charucoIds)
#        retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(np.array(all_charuco_corners)[0], np.array(all_charuco_ids)[0], board, np.array(mtx), np.array(dst), np.empty(1), np.empty(1))
#        
#        Zx, Zy, Zz = tvec[0][0], tvec[1][0], tvec[2][0]
#        fx, fy = mtx[0][0], mtx[1][1]
#        print( Zz / fx)
#        return(f'Zz = {Zz}\nfx = {fx}')
#    marker_corners, marker_ids, rejectedCandidates, = detector.detectMarkers(image)
    #if marker_ids is not None and len(marker_ids) > 0:
        
        #return(f'Zz = {Zz}\nfx = {fx}')
 
    


#LCD I2C Connection Handler
def lcdHandler(lcdConn):
    lcd_columns=16
    lcd_rows = 2
    i2c = board.I2C()
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    lcd.clear()
    lcd.color = [5,5,5]
    lcd.message="On"
    while True:
        #Recieves move commands from main process
        toPrint = lcdConn.recv()
        lcd.message = f"Marker detected\n Angle:{toPrint}"
        
if __name__ == '__main__':
    
    #For 4:3
    #CAMERA_HFOV = 57.15431399
    #For 16:9
    #CAMERA_HFOV = 61.37272481
    CAMERA_HFOV = 55
    ARUCO_DICT = cv2.aruco.DICT_6X6_250
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    MARKER_LENGTH = 0.017145 # marker length in pixels
    # Sets up pipes for interprocess communication
    lcdConn,mainLCDConn = Pipe()
    #Defines LCD iC2 Process
    lcd_process = Process(target = lcdHandler, args=(lcdConn,))
    #Starts Processes
    lcd_process.start()
    json_file_path = './calibration1.json'
    with open(json_file_path, 'r') as file:
        json_data = json.load(file)
    mtx = np.array(json_data['mtx'])
    dst = np.array(json_data['dist'])
    
    
    # initialize the camera
    camera = cv2.VideoCapture(0)
    #Arcuo setup

    #aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    #Let camera warmup
    sleep(1) 
    prevAngle = 370
    
    prevTime = time.time()
    
    
    
    #Main control loop
    #Takes image frame from video capture and processes it
    while camera.isOpened():
        ret, image = camera.read()
        image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        h, w = image.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dst, (w,h), 1, (w,h))
        image = cv2.undistort(image, mtx, dst, None, newcameramtx)
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
            centerY = ((corners[index][0][0][1] + corners[index][0][1][1] + corners[index][0][2][1] + corners[index][0][3][1])*.25)
            cX = mtx[0][2]
            cY = mtx[1][2]
           
            #angle = np.arctan2((cX-centerX),(cY-centerY)) * 180 / np.pi
            #angle = (CAMERA_HFOV)/2 * (cX-centerX)/cX
            angle =  (CAMERA_HFOV/w) * (cX-centerX)
            
            newTime = time.time()
            if prevAngle == round(angle):
                if newTime - prevTime > 1:
                    prevTime = newTime
                    mainLCDConn.send(f"{angle:.2f}")
            prevAngle = round(angle)
            centXR = round(centerX)
            centYR = round(centerY)
            camXR = round(cX)
            camYR = round(cY)
            image = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)
            image = cv2.line(image,pt1=(centXR,centYR), pt2=(centXR,centYR), color = (0,0,255), thickness=10)
            image = cv2.line(image,pt1=(camXR,camYR), pt2=(camXR,camYR), color = (0,255,0), thickness=10)
            
            #mainLCDConn.send("ab")
            
            #Labels marker with numeric id
            #for(outline, id) in zip(corners, ids):
            #    markerCorners = outline.reshape((4,2))
            #    overlay = cv2.putText(overlay,str(id),(int(markerCorners[0,0]),int(markerCorners[0,1]) -15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)
            #except:
            #    print()
            #    cv2.imshow("undistort", image)
             #   continue
        
        
        #cv2.imshow("overlay",overlay)
        cv2.imshow("undistort", image)
        #Exits loop after pressign 'q'
        if cv2.waitKey(10) == ord('q'):
            break
    
    #Releases camera and kills all windows
    camera.release()
    cv2.destroyAllWindows()
    
    
    #Closes pipes and joins sub-processes back to main process
    mainLCDConn.close()
    lcd_process.join() 
    lcd.clear()
    
