## Mini Project

This project allows for a Rasberry Pi to send positional data of an ArUco marker to an Arduino and LCD.

### Raspberry Pi

The Pi computer vision system detects the quadrant an ArUco marker occupies utilizing `cv2` and a camera. After determining the quadrant that the ArUco marker is in, the Pi uses `multiprocessing` to send positional data to the Arduino over Serial and to the LCD using I2C.  The Serial and I2C connections are handled on seperate processes that receive their information via Pipes from the main process. Multiprocessing better utilizes the multicore hardware of the Raspberry Pi hardware than the `threading` module.

### Arduino

Insert text here.

### Organization

The Mini project can be found in the MiniProject folder inside the SEED_LAB repository. Inside of this folder there is a `main.py` file that contains all python code for the computer vision section of this project, as well as a folder containing the Arduino code for the motor functionality.
