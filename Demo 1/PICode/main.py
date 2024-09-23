from time import sleep
import numpy as np
import cv2
from cv2 import aruco
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import busio
from smbus2 import SMBus
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
        
if __name__ == '__main__':
    
    # Sets up pipes for interprocess communication
    lcdConn,mainLCDConn = Pipe()
    #Defines LCD iC2 Process
    lcd_process = Process(target = lcdHandler, args=(lcdConn,))
    #Starts Processes
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
            
            mainLCDConn.send("ab")
            
            #Labels marker with numeric id
            for(outline, id) in zip(corners, ids):
                markerCorners = outline.reshape((4,2))
                overlay = cv2.putText(overlay,str(id),(int(markerCorners[0,0]),int(markerCorners[0,1]) -15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)
        
        
        
        
        cv2.imshow("overlay",overlay)
        
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
    
