// Lib
#include <Arduino.h>
#include <robot.h>

#define COUNTS_PER_METER 6808.5 

// Timing vars
unsigned long lastTime = 0;

Robot* Robot::instance = nullptr;

/* Interface
    F:{forward_error},T:{turn_error} with distanceError in meters with turnError in radians 


*/


void serialInterface(Robot* robot){
    while (LOOP_DELAY + lastTime > micros()){
        if (Serial.available()){ // wait for data available
            String inputString = Serial.readStringUntil('\n'); // Read the input until a newline
            inputString.trim(); // Remove any leading or trailing whitespace


            if(inputString.indexOf('L') != -1){

            } else if(inputString.indexOf('R') != -1){

            } else if(inputString.indexOf(',') != -1) {
                int commaIndex = inputString.indexOf(',');
                String part1 = inputString.substring(0, commaIndex);
                String part2 = inputString.substring(commaIndex + 1);

                // Convert the strings to floats
                Robot::getInstance()->distanceError = part1.toDouble() * COUNTS_PER_METER;
                Robot::getInstance()->angleError = part2.toDouble();

                Serial.println("Target x: " + String(Robot::getInstance()->distanceError) + ", Target y: " + String(Robot::getInstance()->angleError));

            } else {
                Serial.println("Invalid input format. Please use 'value1,value2'.");
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

    delay(2000);
    
    // Set Start Time
    lastTime = micros();
}

void loop(){
    serialInterface(Robot::getInstance());
    Robot::getInstance()->positionController();
    
    lastTime = micros();
}
