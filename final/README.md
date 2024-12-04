## Demo 2



### Raspberry Pi
This PI code sends the Distance and Angle between the center of a camera and an ArUco marker, and a turn indicator (Left or right)to the Arduino when within around 1 foot of the marker. We use piping to speed up our serial communication process and have a separate function for sending information to the Arduino, and essentially manually control the speed of sending data using time information. From demo 1, we have added a distance calculation, a turning indicator, and a process of sending this data to the Arduino.

### Arduino
This Arduino code uses a Serial connection to read inline error and angle error from the raspberry pi. It then uses the same control system from Demo 2, however feeds angle error and inline error into their respective control systems, instead of using the encoders. Because of this, encoders are no longer required to run our robot. We also added some PWM correction to ensure the robot corrects course as it drifts. this enables it to adjust course as it is moving, saving us valuable time. 
We then repeat this loop as many times as needed until we reach the final marker.

### Interface
The commands the arduino will be able to operate on
    f"TR,{turnError}": Turn Right with turnError in radians 
    f"TL,{turnError}": Turn Left with turnError in radians
    f"GF,{distanceError}": Go Forward with distanceError in meters

### Organization


