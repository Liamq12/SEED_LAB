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

            int commaIndex = inputString.indexOf(','); // Find the index of the comma
            if (commaIndex != -1) { // Check if a comma was found
                String firstValueStr = inputString.substring(0, commaIndex); // Extract the first value
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

                // Now you can use firstValue and secondValue as needed
                Serial.print("Target x: ");
                Serial.println(firstValue);
                Serial.print("Target y: ");
                Serial.println(secondValue);
            } else {
                Serial.println("Error: No comma found in input.");
            }
        }
    }
}

// Setup
void setup(){
    Robot::getInstance();
    // Serial
    Serial.begin(115200);
    Serial.println("Program Start!");
    
    // Set Start Time
    lastTime = micros();
}

void loop(){
    // Serial Interface
    serialInterface(Robot::getInstance());

    // Robot Control
    Robot::getInstance()->positionController();
    
    lastTime = micros();
}