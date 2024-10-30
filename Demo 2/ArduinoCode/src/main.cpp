// Lib
#include <Arduino.h>
#include <robot.h>

// Timing vars
double loop_delay = 1500;
long lastTime = 0;

Robot* Robot::instance = nullptr;

void serialInterface(Robot* robot){
    while (loop_delay + lastTime > micros()){
        if (Serial.available()){ // wait for data available
            String inputString = Serial.readStringUntil('\n'); // Read the input until a newline
            inputString.trim(); // Remove any leading or trailing whitespace

            if(inputString.indexOf('T') != -1){ // Turn mode
                robot->mode = RobotMode::TURN;
                String angle = inputString.substring(1);
                robot->setTargetAngle(angle.toDouble()); // Set target angle
                Serial.println("Target angle: " + angle);
            } else if(inputString.indexOf('P') != -1){ // Go To
                robot->mode = RobotMode::GO_TO;

                int commaIndex = inputString.indexOf(','); // Find the index of the comma
                if (commaIndex != -1) { // Check if a comma was found
                    String firstValueStr = inputString.substring(1, commaIndex); // Extract the first value
                    String secondValueStr = inputString.substring(commaIndex + 1); // Extract the second value
                    
                    firstValueStr.trim(); // Trim whitespace
                    secondValueStr.trim(); // Trim whitespace

                    double firstValue = firstValueStr.toDouble(); // Convert to double
                    double secondValue = secondValueStr.toDouble(); // Convert to double

                    // Store new targets
                    Position targetPosition(0, 0, 0);
                    targetPosition.x = firstValue;
                    targetPosition.y = secondValue;

                    robot->setTargetPosition(targetPosition);

                    Serial.println("Target x: " + firstValueStr + ", Target y: " + secondValueStr);
                }
            } else {
                Serial.println("Error: Check Input String");
            }
        }
    }
}

// Setup
void setup(){
    Robot::getInstance();
    // Serial
    Serial.begin(9600);
    Serial.println("Program Start!");

    // Set target position
    Robot::getInstance()->mode = RobotMode::TURN;
    // Store new targets
    Position targetPosition(0, 0, 0);
    // Test 1
    targetPosition.x = 0.3048*2;
    targetPosition.y = 0.3048*-2;

    // Test 1
    // targetPosition.x = 0.3048*-2;
    // targetPosition.y = 0.3048*2;

    Robot::getInstance()->setTargetPosition(targetPosition);
    delay(2000);
    
    // Set Start Time
    lastTime = micros();

    
}

void loop(){
    // Serial Interface
    // serialInterface(Robot::getInstance());

    // Robot Control
    Robot::getInstance()->positionController();
    
    lastTime = micros();
}
