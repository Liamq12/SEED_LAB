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

def lcdHandler(lcdConn):
    lcd_columns=16
    lcd_rows = 2
    i2c = board.I2C()
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    lcd.clear()
    lcd.color = [5,5,5]
    lcd.message="Online"
    previous = None
    while True:
        toPrint = lcdConn.recv()
        if previous != toPrint:
            previous = toPrint
            lcd.message = f"[{toPrint[0]},{toPrint[1]}]"
        

def serialHandler(serialConn):
    # I2C address of Arduino, set in Arduino sketch
    #ARD_ADDR=8
    #Offset register
    #offset=2
    #Initialize SMbus library with I2C bus 1
    #i2c = SMBus(1)
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout =1)
    prePosition = None
    
    while True:
        messageObj = serialConn.recv()
        leftWheel = messageObj[0]
        rightWheel = messageObj[1]
        toSend = f"[{leftWheel},{rightWheel}]"
        #command = [ord(character) for character in toSend]
        if prePosition != messageObj:
            ser.write(toSend.encode(encoding='ascii'))
            prePosition = messageObj
            #try:
                #i2c.write_i2c_block_data(ARD_ADDR,offset,command)
                
           # except IOError:
           #     print(f"Could not write data to Arduino, offset register: {offset}")
           # except ValueError:
           #     print("Message must be 32 bytes or less")
        
        
if __name__ == '__main__':
    
    # Set up pipe
    serialConn,mainSerialConn = Pipe()
    lcdConn,mainLCDConn = Pipe()
    
    
    serial_process = Process(target = serialHandler, args=(serialConn,))
    lcd_process = Process(target = lcdHandler, args=(lcdConn,))
    serial_process.start()
    lcd_process.start()
    
    
    
    # initialize the camera
    camera = cv2.VideoCapture(0)
    #Arcuo setup

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

    ##lcd.message = "Online"

    #id to use for movement
    #idSelected =
    #Let camera warmup
    sleep(1)
    while camera.isOpened():
        ret, frame = camera.read()
        heightCam, widthCam, _ = frame.shape
        grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)
        overlay = cv2.cvtColor(grey,cv2.COLOR_GRAY2RGB)
        overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor=4)
        if ids is not None:
            ids = ids.flatten()
            centerX = round((corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0])*.25)
            centerY = round((corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1])*.25)
            cv2.line(overlay,pt1=(centerX,centerY), pt2=(centerX,centerY), color = (0,0,255), thickness=10)
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
            
            
            #Sending Process
            messageToSend = [0,0]
            messageToSend[0] = leftW
            messageToSend[1] = rightW 
            mainSerialConn.send(messageToSend)
            mainLCDConn.send(messageToSend)
            
            for(outline, id) in zip(corners, ids):
                markerCorners = outline.reshape((4,2))
                overlay = cv2.putText(overlay,str(id),(int(markerCorners[0,0]),int(markerCorners[0,1]) -15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)
        
        thicknessLine = 2
        colorLine = (0,0,255)
        #Horizontal
        cv2.line(overlay,pt1=(0,round(heightCam/2)), pt2=(widthCam,round(heightCam/2)), color = colorLine, thickness=thicknessLine)
        
        #Vertical
        cv2.line(overlay,pt1=(round(widthCam/2),0), pt2=(round(widthCam/2),heightCam), color = colorLine, thickness=thicknessLine)
        #color=(blue,green,red)
        cv2.imshow("overlay",overlay)
        
        if cv2.waitKey(10) == ord('q'):
            break
            
    camera.release()
    cv2.destroyAllWindows()
    
    
    #End multiprocessing
    mainSerialConn.close()
    serial_process.join()
    lcd.clear()
    
