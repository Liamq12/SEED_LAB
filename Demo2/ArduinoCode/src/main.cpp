// Lib
#include <Arduino.h>

// #include <Wire.h>
 
// // LED on pin 13
// const int ledPin = 13; 
 
// void setup() {
//     // Join I2C bus as slave with address 8
//     Wire.begin(0x8);

//     // Call receiveEvent when data received                
//     Wire.onReceive(receiveEvent);

//     Serial.begin(9600);
//     Serial.println("Program Start!");
// }
 
// // Function that executes whenever data is received from master
// void receiveEvent(int howMany) {
//   while (Wire.available()) { // loop through all but the last
//     char c = Wire.read(); // receive byte as a character
//     Serial.println(c);
//   }
// }
// void loop() {
//   delay(100);
// }

#include <robot.h>

#define COUNTS_PER_METER 6808.5 

// Timing vars
unsigned long lastTime = 0;

Robot* Robot::instance = nullptr;

/* Interface
//     F:{forward_error},T:{turn_error} with distanceError in meters with turnError in radians 


// */


void serialInterface(Robot* robot){
    while (LOOP_DELAY + lastTime > micros()){
        if (Serial.available()){ // wait for data available
            String inputString = Serial.readStringUntil('\n'); // Read the input until a newline
            inputString.trim(); // Remove any leading or trailing whitespace

            Serial.println("Received String: " + inputString);

            if(inputString.indexOf('L') != -1){
                Robot::getInstance()->mode = RobotMode::LEFT90;
            } else if(inputString.indexOf('R') != -1){
                Robot::getInstance()->mode = RobotMode::RIGHT90;
            } else if(inputString.indexOf(',') != -1) {
                int commaIndex = inputString.indexOf(',');
                String part1 = inputString.substring(0, commaIndex);
                String part2 = inputString.substring(commaIndex + 1);

                // Convert the strings to floats
                Robot::getInstance()->distanceError = part1.toDouble() * COUNTS_PER_METER;
                Robot::getInstance()->angleError = part2.toDouble();

                Serial.println("Target x: " + String(part1.toDouble()) + ", Target y: " + String(Robot::getInstance()->angleError));

            } else {
                Serial.println("Invalid input format. Please use 'value1,value2'.");
            }
        }
    }
}

// // Setup
void setup(){
    Robot::getInstance();
    // Serial
    Serial.begin(115200);
    Serial.setTimeout(100);
    Serial.println("Program Start!");

    delay(2000);
    
    // Set Start Time
    lastTime = micros();
}

void loop(){
    serialInterface(Robot::getInstance());
    // Robot::getInstance()->positionController();
    
    lastTime = micros();
}
