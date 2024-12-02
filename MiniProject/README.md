## Mini Project

This project allows for a Raspberry Pi to send positional data of an ArUco marker to an Arduino and LCD.

### Raspberry Pi

The Pi computer vision system detects the quadrant an ArUco marker occupies utilizing `cv2` and a camera. After determining the quadrant that the ArUco marker is in, the Pi uses `multiprocessing` to send positional data to the Arduino over Serial and to the LCD using I2C.  The Serial and I2C connections are handled on seperate processes that receive their information via Pipes from the main process. Multiprocessing better utilizes the multicore hardware of the Raspberry Pi hardware than the `threading` module.

### Arduino

The Arduino Motor Controller reads two ascii values from the raspberry pi depending on the desired state of the motors. The motor controller then calculates the desired target position given the inputs from the Pi. This is either 0 deg or 180 deg. The Arduino then uses an PI Controller to move the motors to the given position. The error in position is calculated, multiplied by an proportional constant, and combined with an integral term (grows by error*KI every ms). This value is then normalized to the battery voltage, and then used to calculate an PWM value for the motors. Each motor has an independent PI controller. 

There is a small amount of hysteresis which allows the motors to fully shut off when the target position has been achieved. In addition, the integral term provides strong feedback if the motor is manually moved away from its target position.

### Organization

The Mini project can be found in the MiniProject folder inside the SEED_LAB repository. Inside of this folder there is a `main.py` file that contains all python code for the computer vision section of this project, as well as a folder containing the Arduino code for the motor functionality.
