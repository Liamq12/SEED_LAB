#include "motor.h"

Motor::Motor(uint8_t l_encoderPinA, uint8_t l_encoderPinB, uint8_t l_directionPin, uint8_t l_speedPin){
    encoderPinA = l_encoderPinA;
    encoderPinB = l_encoderPinB;

    directionPin = l_directionPin;
    speedPin = l_speedPin;

    // Setup Pins
    pinMode(encoderPinA, INPUT); // Encoder A
    pinMode(encoderPinB, INPUT); // Encoder B

    // Motor Setup
    pinMode(directionPin, OUTPUT);        // Motor Direction
    pinMode(speedPin, OUTPUT);        // Motor 1 Speed Output
}

void Motor::encoderUpdate(){
    if ((micros() - 300) > lastMeasurement) {
    int stateA = digitalRead(encoderPinA);
    int stateB = digitalRead(encoderPinB);

    if ((stateA == stateB))  // Backwards
      position -= 2;
    else   // Forward
      position += 2;
  }
  lastMeasurement = micros();
}

void Motor::setVoltage(double voltage){
    voltage = max(-BATT_VOLTAGE, voltage);

    if(voltage < 0)
        digitalWrite(directionPin, HIGH);
    else
        digitalWrite(directionPin, LOW);

    // Calculate PWM values based
    double pwm = min((abs((double)voltage / BATT_VOLTAGE) * 255), PWM_MAX);

    // Write to pins
    analogWrite(speedPin, pwm);
}


// Motor Shield
Motor MotorShield::leftMotor(2, 5, 7, 9);
Motor MotorShield::rightMotor(3, 6, 8, 10);

MotorShield::MotorShield(){
    // Motor Shield Setup
    pinMode(4, OUTPUT);        // Enable pin
    digitalWrite(4, HIGH);     // Set Enable High
    pinMode(12, INPUT_PULLUP); // Fault pin

    attachInterrupt(digitalPinToInterrupt(3), leftEncoderUpdate, CHANGE);  // Interrupt for covered motor
    attachInterrupt(digitalPinToInterrupt(2), rightEncoderUpdate, CHANGE);  // Interrupt for uncovered motor
}