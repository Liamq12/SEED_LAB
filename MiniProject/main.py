from time import sleep
import numpy as np
import cv2
from cv2 import aruco
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import busio
from smbus2 import SMBus
import serial
from multiprocessing import Process, Pipe

#LCD I2C Connection Handler
def lcdHandler(lcdConn):
    lcd_columns=16
    lcd_rows = 2
    i2c = board.I2C()
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    lcd.clear()
    lcd.color = [5,5,5]
    lcd.message="On"
    previous = None
    while True:
        #Recieves move commands from main process
        toPrint = lcdConn.recv()
        if previous != toPrint:
            previous = toPrint
            lcd.message = f"[{toPrint[0]},{toPrint[1]}]"
        
#Arduino Serial Connection Handler
#Responsible for sending data to Arduino
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
    
    # Sets up pipes for interprocess communication
    serialConn,mainSerialConn = Pipe()
    lcdConn,mainLCDConn = Pipe()
    
    #Defines Arduino Serial Process
    serial_process = Process(target = serialHandler, args=(serialConn,))
    #Defines LCD iC2 Process
    lcd_process = Process(target = lcdHandler, args=(lcdConn,))
    #Starts Processes
    serial_process.start()
    lcd_process.start()
    
    
    
    # initialize the camera
    camera = cv2.VideoCapture(0)
    #Arcuo setup

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    #Let camera warmup
    sleep(1)

    #Main control loop
    #Takes image frame from video capture and processes it
    while camera.isOpened():
        ret, frame = camera.read()
        heightCam, widthCam, _ = frame.shape
        grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)
        overlay = cv2.cvtColor(grey,cv2.COLOR_GRAY2RGB)
        overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor=4)
        
        #If Marker is detected, perform advanced image processing
        if ids is not None:
            ids = ids.flatten()
            centerX = round((corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0])*.25)
            centerY = round((corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1])*.25)
            #Debug code below, uncomment to see where cv sees the center
            #cv2.line(overlay,pt1=(centerX,centerY), pt2=(centerX,centerY), color = (0,0,255), thickness=10)
            leftW = -1
            rightW = -1
            #if true, centerX is in West
            if centerX <= round(widthCam/2): 
                #true if CenterY is in North
                if centerY <=round(heightCam/2):
                    #Arduino(0,1)
                    leftW = 0
                    rightW = 1
                #CenterY in South
                else:
                    #sendToArduino(1,1)
                    leftW = 1
                    rightW = 1
            #centerX is in East, if true centerY is in North        
            elif centerY <=round(heightCam/2):
                #sendToArduino(0,0)
                leftW = 0
                rightW = 0
            #CenterX East, Center Y South
            else:
                #sendToArduino(1,0)
                leftW = 1
                rightW = 0
            
            
            #Sending Data to Sub-Processes
            messageToSend = [0,0]
            messageToSend[0] = leftW
            messageToSend[1] = rightW 
            mainSerialConn.send(messageToSend)
            mainLCDConn.send(messageToSend)
            
            #Labels marker with numeric id
            for(outline, id) in zip(corners, ids):
                markerCorners = outline.reshape((4,2))
                overlay = cv2.putText(overlay,str(id),(int(markerCorners[0,0]),int(markerCorners[0,1]) -15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)
        
        #Defines horizontal and vertical line thickness and color
        thicknessLine = 2
        #color=(blue,green,red)
        colorLine = (0,0,255)
        #Horizontal Line
        cv2.line(overlay,pt1=(0,round(heightCam/2)), pt2=(widthCam,round(heightCam/2)), color = colorLine, thickness=thicknessLine)
        
        #Vertical Line
        cv2.line(overlay,pt1=(round(widthCam/2),0), pt2=(round(widthCam/2),heightCam), color = colorLine, thickness=thicknessLine)
        
        cv2.imshow("overlay",overlay)
        
        #Exits loop after pressign 'q'
        if cv2.waitKey(10) == ord('q'):
            break
    
    #Releases camera and kills all windows
    camera.release()
    cv2.destroyAllWindows()
    
    
    #Closes pipes and joins sub-processes back to main process
    mainSerialConn.close()
    mainLCDConn.close()
    serial_process.join()
    lcd_process.join() 
    lcd.clear()
    
