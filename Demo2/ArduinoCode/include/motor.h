#include <Arduino.h>

#ifndef MOTOR_H
#define MOTOR_H

#define BATT_VOLTAGE 7.8
#define PWM_MAX 200

class Motor {
public:
    // Constructor
    Motor(uint8_t encoderPinA, uint8_t encoderPinB, uint8_t directionPin, uint8_t speedPin, bool flip = false);

    uint8_t getEncoderPinA(){   return encoderPinA; }

    void encoderUpdate();

    // Sign represents direction
    void setPWM(int16_t pwm);

    long getPosition(){ return position; }

    void clearPosition();

private:
    uint8_t encoderPinA;
    uint8_t encoderPinB;

    uint8_t directionPin;
    uint8_t speedPin;

    long position = 0;
    uint32_t lastMeasurement = 0;

    bool directionFlip = false;
};

class MotorShield {
private:
    static Motor leftMotor;
    static Motor rightMotor;

public:
    MotorShield();

    static void leftEncoderUpdate(){    leftMotor.encoderUpdate();  };
    static void rightEncoderUpdate(){   rightMotor.encoderUpdate(); };

    static Motor& getLeftMotor(){ return leftMotor; }
    static Motor& getRightMotor(){ return rightMotor; }
}; 

#endif // MOTOR_H