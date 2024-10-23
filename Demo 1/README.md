## Demo 1

This project allows for muliple capabilities. First, the ability for the Raspberry Pi to obtain the angle an Aruco Marker is away from the camera center. Second, the ability for the robot to go foward in a straight line. Third and finally, the ability for the robot to turn based on an angle and a position.

### Raspberry Pi

The Pi computer vision system undistorts the camera image and detects the center of the detected ArUco marker using `cv2`. Then the Pi calculates the angle the marker is from the center of the camera with a precalculated HFOV along with the X coordinate of the camera center. The Pi then displays this calculated angle on the LCD on a seperate process through the use of the `multiprocessing` library. The LCD is updated at a rate of 1 frame per second. The library used to update the LCD takes approximately 0.8 second to update the screen. This system ensures the computer vision system can run at full performance while still informing the user through visuals.

### Arduino

The Arduino Motor Controller implements two PI loops. One that tracks inline error, and a second loop that tracks "normal" error away from the heartline. It uses this data to align itself correctly, and apprach its target without overshoot. 

### Organization

The `Demo 1/` directory can be found within the root folder of our repository. Within `Demo 1/` includes the following:
1. `ArduinoCode/`
    Contains the rotation system and straight line controller.
2. `Main.m`
    Contains the simulink models that can be used to simulate the control system.
3. `PICode`
    Contains the computer vision system, and calibration file.
4. `TestModels/`
    Contains python scripts to test control system calculations.
